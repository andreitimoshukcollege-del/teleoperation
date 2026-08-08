# Time

`ITimeAuthority` and its Core-side implementation. Nothing else in Core reads a clock.

## Implemented

| Name | File | Notes |
|---|---|---|
| — | `ManualClock.cs` | stepped entirely by hand; what replay and `Teleop.Core.Tests` drive |
| — | `ClockSync.cs` | Cristian's-algorithm offset/uncertainty estimator between two `ITimeAuthority` domains (operator/robot), normalizing for mismatched tick rates; not itself an `ITimeAuthority` |

**`MonotonicClock` (Stopwatch-based) deliberately does not live here.** Constructing a
`Stopwatch` is banned in Core (root `CLAUDE.md` invariant 2). The wall-clock-backed
`ITimeAuthority` implementation lives in `core/Teleop.Eval/Time/MonotonicClock.cs` instead —
that project is a host and is allowed to touch a real clock.

## `ClockSync`

Estimates the offset between the operator's and the robot's `ITimeAuthority` domains from a
round trip's four timestamps (operator-send, robot-recv, robot-send, operator-recv), via
Cristian's algorithm: `offset = ((t0-t1')+(t3-t2'))/2`, `uncertainty ≈ rtt/2`. Smoothed across
samples with an EWMA (`ClockSyncConfig.SmoothingAlpha`, same shape as `PredictorConfig`'s) plus
an NTP-style min-RTT outlier filter — a sample far slower than the best RTT seen so far is
rejected rather than blended in, since a slow round trip bounds the offset less tightly.

The `'` marks above are the part that is easy to miss: `t1`/`t2` are robot-domain ticks, and the
two domains need not agree on `TicksPerSecond` (a Windows `Stopwatch` reports 10,000,000, a Jetson
1,000,000,000 — a 100x mismatch that inflated every RTT this class produced on the first real
cross-machine run). So `AddRoundTrip` and `ToOperatorTicks` both take **both** rates explicitly on
every call and rescale the robot's stamps into operator-tick-equivalent units — `t1' = round(t1 *
operatorRate / robotRate)` — before any cross-domain arithmetic. Equal rates make the ratio exactly
1.0, so same-domain callers (loopback, sweep, Unity's bridges) are unaffected. The robot's rate
reaches the operator on `Pipeline/RobotStateFrame.TicksPerSecond`; see
`docs/adr/0008-clocksync-cross-rate-normalization.md`. Every tick value `ClockSyncDiagnostics`
reports is in operator ticks.

This is what `docs/metrics.md`'s "on the synced clock (`Time/ClockSync.cs`)" line already
refers to, and what ADR-0002 (`docs/adr/0002-latency-trace.md`) requires before `LatencyTrace`'s
robot-domain fields (`WithRobotRecvTicks`, `WithDownlinkSendTicks`, `WithClockSync`) are ever
populated for real rather than just exercised by a test.

Not behind a `Contracts/` interface: there is exactly one clock-sync algorithm in this project,
not a family of competing implementations to select between, so it is a plain `sealed class`
like `Stamped<T>` rather than an interface+registry axis.
