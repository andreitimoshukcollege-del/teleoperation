# Buffering

Implementations of `Contracts/IPlayoutPolicy.cs`. A playout policy decides **when** a received
sample becomes usable — the moment stamped `t_playout`.

This is the axis that trades latency against loss. Every policy sits somewhere on that curve;
the research question is where to sit and whether to move adaptively. A policy that does not
report its own operating point is not evaluable.

## Implemented

| Name | File | Notes |
|---|---|---|
| `immediate` | `ImmediatePlayout.cs` | zero buffer. The baseline — maximum jitter, minimum latency |
| `fixed` | `FixedDelayPlayout.cs` | constant delay budget; the other baseline |
| `percentile` | `PercentileTrackingPlayout.cs` | tracks a target percentile of observed one-way delay |
| `kalman-jitter` | `KalmanJitterPlayout.cs` | Kalman estimate of delay mean and variance |
| `adaptive` | `NetEqAdaptivePlayout.cs` | NetEQ-style: expands/contracts against buffer occupancy |
| `pareto` | `LatencyLossOptimizingPlayout.cs` | explicit operating point on the latency/loss curve |

## Tried and rejected

Record failures here with a link to the `results/` directory.

- *(none yet)*

## Requirements

1. **Report the operating point.** `Diagnostics` must expose current delay budget, buffer
   occupancy, and the induced late-arrival (effective loss) rate. Two policies with the same
   mean latency and different loss rates are not comparable without this.
2. **Never starve silently.** Buffer underrun is an event to emit through `IMetricSink`, not a
   condition to paper over. Underruns are the failure mode operators actually feel.
3. **Handle burst loss, not just a loss rate.** Real links drop runs of packets. A policy tuned
   on Bernoulli loss will collapse on a 20-packet burst; test against the bursty profiles in
   `core/testdata/traces/`.
4. **Handle reordering and duplicates** without corrupting playout order.
5. Deterministic, allocation-free, `Reset()` fully restores as-constructed state.
6. Adaptive policies must not oscillate. Test with a step change in delay and assert the
   adaptation settles within a stated bound rather than ringing.

## Interaction note

Playout and prediction are coupled: a larger buffer means less prediction horizon needed, and a
better predictor means a smaller buffer is tolerable. Never tune both in one sweep — you will
not be able to attribute the result. Fix one, vary the other, then swap.
