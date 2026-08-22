# 10. Send absolute joint targets over the local relay channel, not relative direction deltas

## Status

Accepted.

## Context

`Teleop.RobotHost`'s `JetRoverPlant` talks to the JetRover's ROS 2 node
(`docs/adr/0007-jetrover-plant-and-robot-host.md`) over a local Unix domain socket, via
`Teleop.RobotHost/Relay/RelayProtocol.cs` (`.NET`) and `jetrover-teleop-ros`'s
`teleop_relay/relay_protocol.py` (Python) — two independently-maintained implementations of the
same fixed-size wire format, kept in sync by hand.

That wire format currently carries a *relative direction delta* per joint, not an absolute
target: `JetRoverPlant` computes each joint's IK target pulse, diffs it against its own
optimistic belief of the joint's last-commanded pulse, clamps that diff to
`JetRoverPlantConfig.MaxDirectionMagnitude`, and sends the resulting delta. On the ROS side,
`ServoController.setPos` (`jetrover_arm_control`) reconstructs an absolute pulse from that delta
against its *own*, separately-maintained optimistic belief (`_beliefPos`), seeded from a real
board read on first use — falling back to a hardcoded 500 if that seed read fails.

This session's real-hardware work replacing the middle-arm servo (`robot/README.md`'s Status
section) kept running into exactly this kind of redundant, independently-tracked state: two
belief-tracking systems computing what should be the same fact (where is this joint right now),
liable to drift apart, and each carrying its own historically-real bug (`JetRoverPlant`'s own doc
covers a real overshoot bug from an unseeded belief after a mid-session restart;
`ServoController`'s `getRawPos`-fails-fall-back-to-500 path is the same failure shape on the ROS
side). The operator asked directly for the simpler alternative: send an absolute target per
joint, and have the robot side just move there — "an id and an angle position," not a delta two
different processes each have to reconstruct into the same absolute fact independently.

**This turns out to be a small, safe change, not a rewrite.** `JetRoverPlant.ApplyJointTargets`
already computes the exact, clamped, absolute pulse target for each joint
(`_targetPulseBase`/`_targetPulseLower`/`_targetPulseMiddle`/`_targetPulseUpper`) *before*
deriving the `direction` value it currently sends — the direction is a derived intermediate on
the .NET side already, not the source of truth. All of the actual safety logic
(`MaxDirectionMagnitude`'s per-cycle clamp, `LowerArmMinPulse`'s collision floor, the one-time
seed-from-sensed belief, gap-clamping when a target's required delta exceeds the per-call
maximum) lives entirely in how `_targetPulseX` gets computed, not in what gets serialized onto
the wire afterward. Changing what crosses the wire from "the delta used to get there" to "the
resulting absolute target" does not touch any of that.

## Decision

**The local relay channel's four arm-joint fields become absolute pulse targets (0-1000, hardware
units), not direction deltas.** Pulse, not radians: `JetRoverPlant` already computes pulse
internally as the last step before sending, so sending pulse means the ROS/Python side needs zero
duplicated `PulsePerRadian`/`ZeroPulse` conversion logic — one less constant pair to keep in sync
by hand across the two languages. `GripperDegrees` is unchanged: it was already an absolute value
with no belief tracking on either side.

**`RelayProtocol.Version` (and `relay_protocol.py`'s `VERSION`) bumps 2 → 3.** The byte layout is
identical (still four floats + one float) but the *meaning* of the four arm fields inverts (delta
→ absolute target) — a version mismatch between an upgraded and a stale side must fail closed via
the existing `TryDecodeCommand`/`decode_arm_command` reject-on-mismatch contract, not silently
reinterpret one shape as the other. Silently misinterpreting an absolute target as a delta (or
vice versa) could snap a joint to a wildly wrong position with no other symptom.

**`JetRoverPlant.ApplyJointTargets` sends the post-clamp `_targetPulseX` fields instead of the
`xDirection` locals used to reach them.** Nothing upstream of that final line changes: IK, the
per-cycle magnitude clamp, the lower-arm floor, and the one-time seed-from-sensed belief are all
untouched, and remain covered by their existing unit tests unmodified in behavior — only the
assertions checking *which field* was sent need updating.

**`ServoController.setPos` loses its belief-tracking entirely.** No more `_beliefPos` dict, no
`getRawPos` seed-on-first-use read, no fallback-to-500, no relative-delta arithmetic. It becomes:
clamp the given absolute pulse to the servo's valid range, optionally skip the physical write if
it's within `MIN_MEANINGFUL_PULSE_DELTA` of the last value *sent* (now a plain scalar cache, not a
belief combining physical assumptions about where the servo actually is), then call
`bus_servo_set_position` directly. The existing per-servo write cooldown
(`_nextAllowedWriteAt`) is unrelated to belief tracking — it is a real hardware pacing rule (don't
queue a new move before the previous one's commanded `duration` has elapsed) — and is unchanged.

## Consequences

- **The relative-delta-accumulation drift bug class is structurally eliminated, not just
  papered over.** `MIN_MEANINGFUL_PULSE_DELTA` originally existed because repeatedly summing
  small deltas let floating-point truncation noise *accumulate* into real drift over many calls
  (a real bug found and fixed during Phase 2 hardware testing). Sending a fresh absolute target
  every cycle, recomputed from the current intended target rather than accumulated onto a running
  belief, means truncation noise can only ever jitter the sent value by about a pulse around the
  true target — it has nothing to accumulate onto. The write-gate becomes a bus-traffic
  optimization from here on, not a correctness requirement.
- **One of the two independently-maintained belief-tracking systems is gone.** `JetRoverPlant`'s
  own optimistic-belief/seed-from-sensed system is unchanged and remains the one place "what pulse
  did we last ask for" is tracked; `ServoController` no longer needs its own separate copy of that
  same fact.
- **Wire format is a breaking change (v2 → v3) across two independently-built, independently
  deployed processes** (`Teleop.RobotHost` on `.NET`, `jetrover_arm_control`/`teleop_relay` on
  ROS 2/Python) — unlike `docs/adr/0008`'s `RobotStateFrame` change, these are *not* built from
  shared source, so both sides must be redeployed together; a stale side rejects the new version
  rather than misinterpreting it, per the Decision above.
- `JetRoverPlant.State`'s existing sensed-else-last-commanded-target fallback is unaffected:
  `_targetPulseX` still exists and still means the same thing (the plant's own belief of the last
  pulse it asked for); only the wire representation of that value downstream changes.
- Deliberately unchanged: `Teleop.Core`; `JetRoverPlant.Command`/`CommandJointAngles`'s public
  signatures and IK/clamping behavior; the Unity `JetRoverOperatorBridge`/`Teleop.JetRover` wire
  path (`JointCommandFrame`, a separate, already-absolute-angle protocol between Unity and
  `Teleop.RobotHost` — this ADR only touches the *second* hop, `Teleop.RobotHost` → ROS relay);
  gripper handling on either side.
