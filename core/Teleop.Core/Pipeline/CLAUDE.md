# Pipeline

Composition — the wiring diagram, expressed in code. Ties `ICommandCodec`, `ITransport`,
`IRobotPlant`, `ClockSync`, and `IMetricSink` together into the two halves of a round trip.
Holds no algorithm of its own; every research question (prediction, reconciliation, buffering,
autonomy) is a `Contracts/` interface this layer will eventually wire in, not something
implemented here.

## Implemented

| Name | File | Notes |
|---|---|---|
| — | `OperatorEndpoint.cs` | capture pose → command → in-flight `LatencyTrace`; matches downlink replies by `Sequence`, drives `ClockSync` |
| — | `RobotEndpoint.cs` | drains commands → `IRobotPlant`; replies with plant state once per received datagram |
| — | `RobotStateFrame.cs` / `RobotStateFrameCodec.cs` | the downlink wire message and its 49-byte fixed binary codec (plain class, not a `Contracts/` interface — exactly one downlink shape exists) |

`OperatorEndpoint` now hosts a live, injected `IPredictor<Pose>`/`IReconciler<Pose>` pair
(Phase 5): `TryReceiveState` folds each robot-state sample into both, and
`EstimateRobotState(nowTicks)` (`Reconcile(Predict(nowTicks), nowTicks)`) is the live estimate a
host calls from `Application.onBeforeRender` per docs/setup.md's callback table. The pair is a
**required** constructor dependency, never defaulted internally — Pipeline "holds no algorithm
of its own" is a real constraint, so the zero-mitigation configuration
(`PassthroughPredictor` + `SnapReconciler`) must be visible at the call site, not hidden in this
class. Only operator-side prediction is wired (estimating the robot's state from stale downlink
samples); `RobotEndpoint` is untouched, since robot-side prediction of operator intent is a
different problem per `IPredictor<TState>`'s own doc and nothing here needs it yet.

No autonomy arbiter, no `IPlayoutPolicy` are wired in yet — `Autonomy/`/`Buffering/` still have
no implementation. `OperatorEndpoint` hardcodes `t_playout = t_operatorRecv` inline as the
explicit, temporary stand-in for the not-yet-built `ImmediatePlayout` — replace that line, not
add around it, once `IPlayoutPolicy` has a real implementation.

## Requirements

1. **`ITimeAuthority` is read for `TicksPerSecond` only, never `NowTicks`.** Every "when is it
   now" arrives as an explicit `nowTicks` parameter — the same discipline `ITransport` and
   `IRobotPlant` already enforce in their own doc comments.
2. **No third "driver" class.** Cadence (advance clock → submit → step → receive) is host
   orchestration, per docs/setup.md's callback-placement table (network thread / `FixedUpdate`
   / `Update` / `onBeforeRender`) — Core exposes step-oriented methods and never a driving loop.
3. **`RobotEndpoint` replies once per received datagram, not once per accepted command** —
   `IRobotPlant.Command` returns `void`, so replying with the plant's current state for every
   receipt (after `Step`, not before) is what avoids needing to duplicate the plant's own
   staleness policy in the caller.
4. **No `Registry/Registries.cs` entry for `OperatorEndpoint`/`RobotEndpoint` themselves.**
   Exactly one pairing exists; nothing here is a family of competing implementations. The
   predictor/reconciler `OperatorEndpoint` is constructed with, by contrast, are resolved by the
   caller via `Registries.Predictors`/`Registries.Reconcilers`.
5. Allocation-free per call: every buffer and the in-flight-trace ring are preallocated in each
   endpoint's constructor.

## Metric names

`owd_uplink_ms` and `owd_downlink_ms`, defined in docs/metrics.md §2, emitted by
`OperatorEndpoint` on every completed round trip.
