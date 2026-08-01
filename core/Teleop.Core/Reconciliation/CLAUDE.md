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

Keep this table current — it previously overclaimed `exp-smooth`/`spring`/`budget-blend`/
`velocity-match`/`rollback` as implemented; they are **planned, not built** — see below.

## Planned, not yet implemented

| Name | File | Notes |
|---|---|---|
| `exp-smooth` | `ExponentialSmoothingReconciler.cs` | one time constant; simple, biased |
| `spring` | `SpringReconciler.cs` | critically damped; no overshoot |
| `budget-blend` | `TimeBudgetedBlendReconciler.cs` | guarantees convergence within N ms |
| `velocity-match` | `VelocityMatchedReconciler.cs` | corrects position while preserving apparent motion |
| `rollback` | `RollbackReconciler.cs` | rewind to authoritative state, re-apply buffered inputs (GGPO-style) |

Move a row up to "Implemented" only once its file, tests, and `Registry/Registries.cs` entry
all actually exist — `Teleop.Eval -- audit`'s registry-completeness check will catch a row that
claims otherwise.

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
