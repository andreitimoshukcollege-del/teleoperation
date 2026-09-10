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
| `percentile` | `PercentileTrackingPlayout.cs` | budget = nearest-rank quantile of the last `w` observed one-way delays. At `p = 1.0` it *is* `analysis/playout_bounds.py`'s "max of the last w", exactly |

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
| `kalman-jitter` | `KalmanJitterPlayout.cs` | Kalman estimate of delay mean and variance. Note `percentile` deliberately makes no distribution assumption; this row is where that assumption gets tested, on a delay distribution known to be bimodal |
| `adaptive` | `NetEqAdaptivePlayout.cs` | NetEQ-style: expands/contracts against buffer occupancy |
| `pareto` | `LatencyLossOptimizingPlayout.cs` | explicit operating point on the latency/loss curve |

**The offline 47-61% is now confirmed in the pipeline, and it is confirmed only where it should
be.** `exp-009-percentile-tracking` puts `percentile` at `p = 1.0, w = 64` against a 15-point
`fixed` budget curve at matched loss
(`docs/research-log/2026-09-09-percentile-tracking-decisions.md`):

| profile family | result |
|---|---|
| bursty / bimodal delay (`synthetic-burst`, `50ms-5j`) | `percentile` buffers **56-59% less** at matched loss |
| bounded-uniform delay (`150ms-20j-0.5loss`, `300ms-60j-2loss-bursty`) | a **per-profile-tuned** `fixed` budget is slightly better on both axes |

Both halves were predicted. The transport survey said a bounded-uniform profile has an optimal
constant `Base + J` and nothing to estimate; the bounds analysis said a trace with persistent burst
structure is trackable. Each was right about its own half, and neither is right about the other.

The reordering that `immediate`'s late-arrival rate measures is largely a harness artifact on the
parametric profiles -- at the 10 ms step, `50ms-5j`'s 13.0% is *entirely* `RobotEndpoint` batching
its replies, with neither transit leg inverting anything. `synthetic-burst`, which the headline
rests on, is driven by a ~230 ms delay step instead and is unaffected. See the addenda in both
Buffering decision records.

Re-verified after `docs/adr/0013-composable-network-impairments.md` rebuilt the impairment
pipeline: `synthetic-burst` and `lan` are identical to the digit (neither consumes a parametric
draw), and the three parametric profiles moved by under one percentage point of loss. The
conclusions are unchanged — see the addenda in both Buffering decision records for the numbers.

**The case for `percentile` is transferability, not dominance.** The `fixed` budgets that beat it
were each chosen by sweeping 15 budgets against that one profile, and the winner on
`150ms-20j-0.5loss` (160 ms) is catastrophic on `300ms-60j-2loss-bursty`, which needs 350 ms.
`percentile` needs no per-profile tuning and is the only policy here that behaves sensibly on all
five profiles at once.

Move a row up to "Implemented" only once its file, tests, and `Registry/Registries.cs` entry
all actually exist — `Teleop.Eval -- audit`'s registry-completeness check will catch a row that
claims otherwise. The wiring the first one needed is done; the next one is a new file, a registry
entry and tests, with no `Pipeline/` change.

## Tried and rejected

Record failures here with a link to the `results/` directory.

- **A fixed budget as a transferable operating point.** Not rejected as an implementation --
  `fixed` is the baseline and stays -- but rejected as an answer. A budget is measured from
  capture, so one value cannot serve profiles with different base delays: at 40 ms it is
  byte-identical to `immediate` on every profile whose one-way delay exceeds it
  (`exp-008-playout-baselines`), and the per-profile optima found by sweeping 15 budgets range from
  60 ms to 350 ms across four profiles (`exp-009-percentile-tracking`).

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
