# 8. `ClockSync` must normalize for mismatched `TicksPerSecond` across machines

## Status

Accepted.

## Context

Phase 3 of the JetRover integration (`docs/adr/0007-jetrover-plant-and-robot-host.md`) ran the
first genuinely cross-machine exercise of `Time/ClockSync.cs`: a real `OperatorEndpoint` on this
project's Windows dev machine talking over real UDP to the Jetson's `Teleop.RobotHost`
(`RobotEndpoint`/`JetRoverPlant`) over Tailscale.

`ClockSync.AddRoundTrip` combines four raw tick values — two from the operator's
`ITimeAuthority`, two from the robot's — with direct addition and subtraction
(`operatorSendTicks - robotRecvTicks`, etc.). This is only numerically valid if both domains'
`ITimeAuthority.TicksPerSecond` agree. Every prior exercise of this code — every sweep trial, the
whole Phase-4 loopback baseline, Unity's `TeleopOperatorBridge`/`TeleopRobotBridge` pair — runs
operator and robot logic against the same `ITimeAuthority` instance or two instances of the same
implementation in one process, so the rates trivially matched and this assumption was never
false, and never tested.

Across two real, different machines it is false: this project's Windows dev machine's `Stopwatch`
reports `TicksPerSecond = 10,000,000`; the Jetson's Linux ARM64 `.NET` runtime reports
`1,000,000,000` — a 100x mismatch. `ClockSync`'s raw arithmetic, fed these two domains directly,
produces offset and RTT figures off by that same 100x factor (confirmed on real hardware: RTT
came out on the order of 10,000-40,000ms while the arm still moved correctly and the actual
Tailscale RTT was ~63-110ms). `ITimeAuthority`'s own doc already states the general rule this
violates: "every long tick value elsewhere in Core... is on the timebase of the authority injected
into that component. Mixing timebases is a bug." `ClockSync` is the one component in Core whose
entire job is to reconcile two different timebases, so it cannot avoid mixing them — it must do so
explicitly and correctly instead of implicitly assuming they match.

## Decision

**`ClockSync` takes both domains' `TicksPerSecond` explicitly, on every call, and rescales the
robot-domain tick values into operator-tick-equivalent units (via a `double` ratio) before any
cross-domain arithmetic.** Not a constructor-time setting: `robotTicksPerSecond` is genuinely
per-message information (it arrives over the wire, see below), and passing both rates explicitly
on every call matches this project's established "time is a parameter, not hidden state"
discipline (`ITransport`, `IRobotPlant`, `OperatorEndpoint`'s own doc all state this same rule for
`nowTicks`).

```
AddRoundTrip(operatorSendTicks, operatorTicksPerSecond,
             robotRecvTicks, robotSendTicks, robotTicksPerSecond,
             operatorRecvTicks)

ToOperatorTicks(robotTicks, robotTicksPerSecond, operatorTicksPerSecond)
```

**`RobotStateFrame` gains a `TicksPerSecond` field** (the robot's own `ITimeAuthority.TicksPerSecond`,
populated by `RobotEndpoint`, which already receives that authority in its constructor and
previously discarded it). This is the only way the operator can ever learn the robot's rate — it
is a fact about a remote machine, not something the operator can determine from its own clock.
Sent on every reply rather than negotiated once: `TicksPerSecond` is cheap (8 bytes), "fixed for
the lifetime of the instance" per `ITimeAuthority`'s own doc so resending it is free of any
staleness concern, and avoids adding connection-lifecycle/handshake state this project has nowhere
else. `RobotStateFrameCodec`'s version byte moves 1 → 2 and its fixed size grows 49 → 57 bytes.
This wire frame is Core code compiled into both `Teleop.Eval` and `Teleop.RobotHost` from the same
source — unlike the JetRover-specific relay protocol (`Teleop.RobotHost/Relay/RelayProtocol.cs` /
`jetrover-teleop-ros`'s `relay_protocol.py`), there is no second, independently-maintained
implementation to keep in sync by hand.

**The operator's own `CommandFrame` does not gain a `TicksPerSecond` field.** Per
`docs/adr/0002-latency-trace.md`, clock-domain conversion happens exactly once, operator-side —
`RobotEndpoint` never converts a timestamp into another domain, so it never needs the operator's
rate.

**No change to `unity/`.** Unity's bridges construct `RobotEndpoint`/`OperatorEndpoint`/
`ClockSync`/`RobotStateFrameCodec` generically and never call `AddRoundTrip`/`ToOperatorTicks` or
construct a `RobotStateFrame` directly — those calls are internal to `OperatorEndpoint`/
`RobotEndpoint`, which absorb this entire change. Confirmed by inspection before starting this
work, not assumed.

## Consequences

- `RobotStateFrameCodec`'s wire format is a breaking change (v1 → v2). Safe here specifically
  because both ends of this wire hop are built from the same `Teleop.Core` source in the same
  repo, redeployed together — there is no independent third-party decoder of this specific frame
  to break.
- `ClockSync`'s public API changes (`AddRoundTrip`, `ToOperatorTicks` gain parameters). Every
  existing call site — all same-domain (loopback/sweep/tests) — passes matching
  `TicksPerSecond` values for both parameters, which is a mechanical, behavior-preserving update:
  the rescale ratio is exactly 1.0 whenever both rates are equal, so this is a strict
  generalization, not a change in behavior for any existing same-rate caller.
- Verified against the real JetRover after the fix lands: re-running `clocksync-check`
  (`core/Teleop.Eval/ClockSyncCheck/`) should now report an RTT consistent with the real Tailscale
  RTT observed independently (`~63-110ms`), not the ~100x-inflated figures Phase 3 first reported.

## Amendment (2026-10-31): the smoothing's precision

**The bug.** Each accepted round trip is folded into the running offset by an EWMA, and that EWMA
blended the two `long` values in single precision:
`(long)(alpha * offsetSample + (1f - alpha) * smoothed)` with a `float` alpha. The offset is the gap
between two machines' boot times, which is hours to days of ticks. Single precision has 24 bits, so
from the second accepted sample on, the offset snapped onto a float32 grid and stayed there.
- **Grid size.** 2^22 ticks (4.2 ms) for a 10 h offset at the Galaxy XR's 1 GHz; 2^16 ticks (6.5 ms)
  for 25 h at the editor's 10 MHz.
- **The error.** Up to half a grid step, roughly constant through a session. Clock drift was
  followed only in grid-sized jumps.
- **History.** It was found on 2026-10-20, while writing the trainer's golden files, and left until
  now.

**The fix.** The EWMA now runs in double on the difference between sample and estimate:
`offset += round(alpha * (sample - offset))`. The same applies to the uncertainty. Its precision no
longer depends on the offset's size: it settles within `0.5 / alpha` ticks of a steady sample
(2.5 ticks at 0.2).
- **Tests.** `ClockSyncTests` now recovers a 10 h and a 25 h offset to the tick, and follows a
  50 µs/s drift on every sample. The old code missed by 0.64 ms and 1.02 ms, and lagged the drift by
  1.39 ms instead of 0.1 ms.
- **The trainer's golden exchanges** (`trainer/contract/golden/clocksync-v1.json`). Offsets moved by
  up to 2.3 ms in the headset case and 6.0 ms in the editor case. No round trip changed between
  accepted and rejected.

**What carried the error.** Everything `ToOperatorTicks` converts while two machines' clocks are
far apart:
- the joint twin's sensed times and the predictors' inputs built from them;
- `owd_uplink_ms`, `owd_downlink_ms` and the other cross-clock metrics in `docs/metrics.md`;
- the camera latency figures;
- the trainer's views (`trainer/armpred/net/clocksync.py` copied the float32 arithmetic on purpose,
  to match the headsets of the time).

Absolute cross-clock figures from real cross-machine sessions before this change can be a few
milliseconds off. They are off by a roughly constant amount within one session. `results/` is
append-only, so those runs stay as they are; their `manifest.json` SHAs tell them apart.

**What did not.** Sweeps and loopback tests run both ends on one clock. There the offset is near
zero and single precision was exact to a tick, so they can differ by at most a tick (rounding, where
the old cast truncated).

**Models trained before the fix** saw an error of up to a few milliseconds in their time inputs,
different in each session. The headset now gives them the error-free value, which is inside the
spread they were trained on. They are not retrained for it.
