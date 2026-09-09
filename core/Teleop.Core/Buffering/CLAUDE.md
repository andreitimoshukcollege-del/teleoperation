# Buffering

Implementations of `Contracts/IPlayoutPolicy.cs`. A playout policy decides **when** a received
sample becomes usable — the moment stamped `t_playout`.

This is the axis that trades latency against loss. Every policy sits somewhere on that curve;
the research question is where to sit and whether to move adaptively. A policy that does not
report its own operating point is not evaluable.

## Implemented

*(none)* — this folder contains no `.cs` files at all. `Registry/Registries.cs`'s
`PlayoutPolicies` table is declared, correctly typed, and empty (`RegistriesTests`
asserts that emptiness rather than pretending otherwise), and `Pipeline/OperatorEndpoint.cs`
hardcodes `t_playout = t_operatorRecv` inline as the explicit, temporary stand-in for the
not-yet-built `immediate`. `Types/PlayoutPolicyConfig.cs` and
`Types/PlayoutPolicyDiagnostics.cs` are written ahead of any implementation, so this axis is
contracts-and-types only.

Keep this table current — it previously overclaimed `immediate`/`fixed`/`percentile`/
`kalman-jitter`/`adaptive`/`pareto` as implemented; they are **planned, not built** — see below.

## Planned, not yet implemented

| Name | File | Notes |
|---|---|---|
| `immediate` | `ImmediatePlayout.cs` | zero buffer. The baseline — maximum jitter, minimum latency |
| `fixed` | `FixedDelayPlayout.cs` | constant delay budget; the other baseline |
| `percentile` | `PercentileTrackingPlayout.cs` | tracks a target percentile of observed one-way delay |
| `kalman-jitter` | `KalmanJitterPlayout.cs` | Kalman estimate of delay mean and variance |
| `adaptive` | `NetEqAdaptivePlayout.cs` | NetEQ-style: expands/contracts against buffer occupancy |
| `pareto` | `LatencyLossOptimizingPlayout.cs` | explicit operating point on the latency/loss curve |

**There is measured reason to build the top of this table.** `analysis/playout_bounds.py` scores
`oracle`/`fixed`/`adaptive` offline against `core/testdata/traces/synthetic-burst.trace`: at
matched late-loss, a policy no cleverer than "budget = max of the last w delays" buffers 47-61%
less than the best possible constant budget. That is a lower bound -- `percentile` and `adaptive`
have strictly more to work with -- and it is *not* currently reproducible through a sweep, because
a 500-step trial contains only ~2 burst episodes. Read
`docs/research-log/2026-09-09-playout-bounds-decisions.md` before starting here; it also records
why the earlier transport survey concluded the opposite.

Move a row up to "Implemented" only once its file, tests, and `Registry/Registries.cs` entry
all actually exist — `Teleop.Eval -- audit`'s registry-completeness check will catch a row that
claims otherwise. Building the first one also means **replacing** `OperatorEndpoint`'s hardcoded
`t_playout` line, not adding around it — see `Pipeline/CLAUDE.md`.

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
