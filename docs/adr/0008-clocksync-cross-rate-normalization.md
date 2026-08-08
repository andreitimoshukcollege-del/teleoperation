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
