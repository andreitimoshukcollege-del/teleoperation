# Autonomy

Implementations of `Contracts/IAutonomyArbiter.cs`. An arbiter decides **how much direct
authority** the operator's command carries, versus how much the robot resolves locally.

The framing is Sheridan's supervisory-control ladder, from most to least direct:

1. **direct** — pose is a setpoint, one-to-one
2. **rate-limited / scaled** — authority attenuated, velocity clamped
3. **waypoint** — commands become goals the robot plans to
4. **intent primitive** — "grasp that", resolved entirely on the robot

Making the rung a *function of measured latency* is one of the more publishable axes in this
project, because it trades a quantity operators feel (directness) against one they feel
differently (instability).

## Implemented

| Name | File | Notes |
|---|---|---|
| `direct` | `DirectArbiter.cs` | pass-through. The baseline |
| `scaled` | `LatencyScaledArbiter.cs` | authority and velocity limit as functions of measured RTT |
| `waypoint` | `WaypointArbiter.cs` | commands become goals; robot plans locally |
| `primitive` | `IntentPrimitiveArbiter.cs` | classified intent -> robot-side primitive |
| `ladder` | `SupervisoryLadderArbiter.cs` | selects a rung from measured latency and jitter |

## Tried and rejected

- *(none yet)*

## Requirements

1. **Monotonic in latency.** More measured delay must never yield *more* direct authority.
   Assert this in a test across a swept latency range; a non-monotonic arbiter is a bug.
2. **Hysteresis at rung boundaries.** Latency estimates are noisy, and an arbiter that flips
   between rungs near a threshold produces behavior operators describe as the robot fighting
   them. Test with a latency signal dithering across a boundary and assert a bounded switch
   rate.
3. **Legible to the operator.** `Diagnostics` must expose the current rung and the reason for
   it, so the HUD can display it. An operator who cannot tell which mode they are in will
   misattribute the robot's behavior — this is a usability requirement, not a nicety.
4. **Bounded authority transitions.** Changing rung must not produce a discontinuity in
   commanded pose. The same C1-continuity discipline as `Reconciliation/`.
5. Deterministic, allocation-free, `Reset()` complete.

## Evaluation note — read this before designing an experiment

Autonomy arbitration is a **closed-loop** question. The arbiter changes what the robot does,
which changes the states the operator sees, which changes their next command. A recorded
`.tlog` therefore cannot serve as ground truth the way it does for prediction: replaying old
operator input against a different arbiter tells you nothing about what the operator *would*
have done.

Consequence: arbiter studies require a plant in the loop (`IRobotPlant`) and, for any usability
claim, a human. Use the Core rigid-body plant for cheap sweeps and headless Unity for
high-fidelity confirmation. Never report an arbiter result scored open-loop from a recording.
