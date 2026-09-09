# Buffering

Implementations of `Contracts/IPlayoutPolicy.cs`. A playout policy decides **when** a received
sample becomes usable — the moment stamped `t_playout`.

This is the axis that trades latency against loss. Every policy sits somewhere on that curve;
the research question is where to sit and whether to move adaptively. A policy that does not
report its own operating point is not evaluable.

## Implemented

| Name | File | Notes |
|---|---|---|
| `immediate` | `ImmediatePlayout.cs` | zero buffer. The baseline — maximum jitter, minimum latency. Still enforces capture order, so its late rate *is* the stream's out-of-order rate |
| `fixed` | `FixedDelayPlayout.cs` | constant delay budget; the other baseline, and the denominator every adaptive claim is stated against |

Both are thin policies over `PlayoutSampleBuffer.cs`, an `internal` shared implementation of
ordering, duplicate rejection and late/underrun counting — the same reasoning
`Reconciliation/DisplayedJerkEstimator.cs` records for itself, since these are the terms policies
are *compared on*. `PlayoutMetrics.cs` holds the five metric names and the per-release emission.

`Pipeline/OperatorEndpoint.cs` no longer hardcodes `t_playout`: receive is two-phase, and
`TryPlayoutState` is what stamps it and feeds the predictor. See
`docs/adr/0012-playout-policy-wiring.md`, which also records why `immediate` is **not** behaviour-
identical to the inline stand-in it replaced, and what that costs in comparability with results
already recorded.

Keep this table current — it once overclaimed all six rows as implemented.

## Planned, not yet implemented

| Name | File | Notes |
|---|---|---|
| `percentile` | `PercentileTrackingPlayout.cs` | tracks a target percentile of observed one-way delay |
| `kalman-jitter` | `KalmanJitterPlayout.cs` | Kalman estimate of delay mean and variance |
| `adaptive` | `NetEqAdaptivePlayout.cs` | NetEQ-style: expands/contracts against buffer occupancy |
| `pareto` | `LatencyLossOptimizingPlayout.cs` | explicit operating point on the latency/loss curve |

**There is measured reason to build the rest of this table.** `analysis/playout_bounds.py` scores
`oracle`/`fixed`/`adaptive` offline against `core/testdata/traces/synthetic-burst.trace`: at
matched late-loss, a policy no cleverer than "budget = max of the last w delays" buffers 47-61%
less than the best possible constant budget. That is a lower bound -- `percentile` and `adaptive`
have strictly more to work with -- and it is *not* currently reproducible through a sweep, because
a 500-step trial contains only ~2 burst episodes. Read
`docs/research-log/2026-09-09-playout-bounds-decisions.md` before starting here; it also records
why the earlier transport survey concluded the opposite.

Move a row up to "Implemented" only once its file, tests, and `Registry/Registries.cs` entry
all actually exist — `Teleop.Eval -- audit`'s registry-completeness check will catch a row that
claims otherwise. The wiring the first one needed is done; the next one is a new file, a registry
entry and tests, with no `Pipeline/` change.

## Tried and rejected

Record failures here with a link to the `results/` directory.

- *(none yet)*

## Requirements

1. **Report the operating point.** `Diagnostics` must expose current delay budget, buffer
   occupancy, and the induced late-arrival (effective loss) rate. Two policies with the same
   mean latency and different loss rates are not comparable without this. As of
   `docs/adr/0012-playout-policy-wiring.md` this is also *emitted*, not merely exposed:
   `playout_budget_ms`, `playout_occupancy` and `playout_late` reach `IMetricSink`, alongside
   `playout_delay_ms` — the buffer's own latency cost, which no metric could see before.
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
