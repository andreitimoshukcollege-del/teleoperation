# Reconciliation

Implementations of `Contracts/IReconciler.cs`. When an authoritative sample arrives and
disagrees with what was predicted, the reconciler decides **how the visible state gets from
the prediction to the truth.**

This is the most underestimated axis in the project. In VR it decides whether the system is
usable at all: a hard snap on correction is nausea, regardless of how good the predictor is.

## Implemented

| Name | File | Notes |
|---|---|---|
| `snap` | `SnapReconciler.cs` | jump to truth. The baseline — measure how bad it is, don't skip it |
| `spring` | `SpringReconciler.cs` | critically damped decay of a residual offset; no overshoot, C1, converges within `MaxTimeToConvergenceTicks` to a stated 1% envelope |
| `budget-blend` | `TimeBudgetedBlendReconciler.cs` | quintic-Hermite blend; residual **bit-exactly zero** at a deadline fixed at seed time, for any correction magnitude. The hard-deadline counterpart to `spring`'s asymptotic envelope. Rate caps sized into the deadline a priori rather than clamped |
| `velocity-match` | `VelocityMatchedReconciler.cs` | `spring`'s dynamics on a warped time axis, slowing the correction in proportion to apparent operator motion. Lowest jerk on the axis, at the highest convergence cost. Bound: 1% envelope within 4x the budget, unconditionally |

`DisplayedJerkEstimator.cs` is in this folder but is **not** a reconciler and has no registry
entry: it is the shared `jerk_mm_s3` cascade (docs/metrics.md §5) that every reconciler here
delegates to, so the axis is compared on one jerk definition rather than one per implementation.

Keep this table current — it previously overclaimed `exp-smooth`/`spring`/`budget-blend`/
`velocity-match`/`rollback` as implemented. `spring` is now real; the rest remain **planned, not
built** — see below.

_Results: `results/exp-004-reconciler-head-to-head/` (exp-003 for snap-vs-spring alone). Logs
with full reasoning in `docs/research-log/2026-09-08-*.md` — read them before citing any number
from those runs; both contain caveats that change how the percentiles should be read._

## Planned, not yet implemented

| Name | File | Notes |
|---|---|---|
| `exp-smooth` | `ExponentialSmoothingReconciler.cs` | one time constant; simple, biased. **Note:** a single-time-constant lag is C0 but *not* C1 — its offset velocity steps from 0 at correction onset — so building it needs requirement 2 amended to cover it as a second quantified exception, the way `snap` is. Unresolved; that is why `spring` was built first |

Move a row up to "Implemented" only once its file, tests, and `Registry/Registries.cs` entry
all actually exist — `Teleop.Eval -- audit`'s registry-completeness check will catch a row that
claims otherwise.

## Tried and rejected

- **`rollback` (GGPO-style rewind-and-replay) — infeasible within `IReconciler<TState>`.**
  Investigated 2026-09-08; full argument in `docs/research-log/2026-09-08-rollback.md`.
  Rollback needs an input history, an anchor and a simulation function; the contract supplies
  **only the anchor**. There is no command stream on the interface and `OperatorEndpoint` never
  receives an `IRobotPlant`. The only reading that fits inside the contract telescopes to
  `truth + (p_now - p_capture)`, so every interior term cancels and `RollbackHistoryCapacity`
  **provably cannot affect the output** (verified numerically: bit-identical across capacities
  200-512). What survives is a full-magnitude step, i.e. the discontinuity only `snap` is granted,
  and hiding it collapses into seed-an-offset-and-decay-it, which is `spring`.
  The structural reason is the useful part: **the predictor already performs the rollback** --
  `Predict` anchors on the newest authoritative sample and re-integrates forward every frame, and
  `OperatorEndpoint` calls `predictor.Observe` before the reconciler runs. The reconciler's job
  here is to *hide* a rollback that happened upstream; a second one from strictly less information
  cannot win. Rollback is therefore not a reconciler at all -- it is operator-side client
  prediction, and belongs **composed with** a reconciler rather than as a row on this axis.
  Reviving it needs an ADR (input-history path, an operator-side shadow plant, and a widened
  registry factory signature). Note `ReconcilerConfig.RollbackHistoryCapacity` is a documented
  intention this boundary does not support: `SweepCommand` hardcodes it to 16 and
  `ExperimentConfig` never exposed it, so nothing could ever have swept it.

- **Removing the one-frame display hold by applying the measured error — rejected on measurement.**
  Every offset-carrying reconciler here seeds its offset from the *previous frame's* displayed
  pose, which cancels the predictor's correction jump exactly by construction and, as a side
  effect, **holds the display for exactly one frame at every onset and retarget**. That is an O(1)
  velocity dropout and a genuine C1 violation, not a sampling artifact.

  The natural fix — advance the offset, then apply the disagreement `Observe` already measures
  (`offset -= error`) — was implemented across `spring`, `budget-blend` and `velocity-match`
  together and swept on 2026-09-08 (`exp-004`, runs `20260908-044443Z` before and
  `20260908-222240Z` after). **It is worse.** The error is measured at *capture* time against
  `predictedAtCapture`, while the predictor's actual jump lands at *now* after it re-anchors on the
  new sample and extrapolates forward; the mismatch leaks a residual step onto every correction
  frame, and that residual scales with correction frequency:

  | profile | corrections | `spring` | `budget-blend` | `velocity-match` | spread across the three |
  |---|---|---|---|---|---|
  | `lan` | ~75 | **-97.4%** | 0% | **-73.1%** | 112.7x -> 2.88x |
  | `150ms-20j-0.5loss` | ~526 | +80.8% | +383% | +424% | 2.84x -> **1.02x** |
  | `300ms-60j-2loss-bursty` | ~625 | +163% | +502% | +865% | 3.65x -> **1.01x** |

  (jerk p99, after vs before.) The spread collapse is the real damage: at 1.01x the three
  reconcilers are indistinguishable on impaired links, so the metric had stopped measuring the
  convergence law and started measuring a shared artifact. Exact jump cancellation is worth more
  than one frame of motion continuity whenever corrections are frequent — which was not the
  expected result.

  **Reverted.** The hold is kept as a deliberate, measured tradeoff and is pinned by the
  `..._ADeliberateTradeoff` tests in `SpringReconcilerTests` and by
  `TheOneFrameHoldAtARetargetIsSharedWithSpring` in `TimeBudgetedBlendReconcilerTests`, which
  guards against a one-sided fix silently skewing a head-to-head. Doing this properly needs exact
  cancellation *and* motion continuity — advancing the displayed pose by the predictor's estimated
  rate before seeding — which is an open question rather than a patch.

## Requirements

1. **Convergence is provable.** A test must show the error reaches zero (or a stated bound)
   within a bounded time under a constant correction. A reconciler that can lag indefinitely
   is a bug, not a tradeoff.
2. **C1 continuity of visible output.** No position or velocity discontinuities. Test with a
   step correction and assert the jerk bound. **`snap` is the one deliberate exception**: a
   snap *is* a position/velocity discontinuity by definition, so its test suite proves and
   quantifies one (a witness test asserting the jerk exceeds a large, stated threshold) rather
   than asserting continuity it cannot have. If `snap` ever became smooth, that would itself be
   a regression -- it exists specifically to measure the cost every other reconciler here is
   trying to avoid. Every other reconciler still owes the full requirement.
3. **Emit correction cost every step** via `IMetricSink`: correction magnitude, corrections
   per second, peak jerk. This is the metric that trades off against prediction accuracy and
   the whole point of studying this axis separately.
4. Deterministic and allocation-free, as everywhere in Core.
5. If it consumes predictor uncertainty (e.g. covariance from `ekf`), it must degrade
   gracefully when the predictor supplies none.

## Experiment design note

Reconciler studies vary **only** the reconciler — same predictor, same trace, same seed.
Cheap to run and historically where the surprising results are. Resist the urge to tune the
predictor in the same sweep; you will not be able to attribute the result.
