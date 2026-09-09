# 2026-09-09 — `ekf` (`ExtendedKalmanPredictor`) feasibility assessment

Agent: feasibility-assessment run, one of two parallel researchers on this question. **This is
an assessment, not an implementation.** No `.cs` file, test, registry entry, experiment YAML or
`results/` output was produced or is claimed here. The only file changed by this run is this log.

## Seed self-check

| Check | Result |
|---|---|
| Worktree | `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-a1a80669f93f41b47` |
| Branch | `worktree-agent-a1a80669f93f41b47` |
| `git log --oneline -1` | `ce39dfb Merge pull request #22 from andreitimoshukcollege-del/research-documentation-docx` |
| `git rev-parse HEAD` | `ce39dfb3925ffa4fbc6d147bcffb2824589fe0d8` — **matches** the `main` sha (`ce39dfb`) given in the task |
| `git status` | clean, nothing to commit |
| `dotnet test` (core) | green — 458 Teleop.Core.Tests + 28 Teleop.RobotArm.Tests + 3 Teleop.Eval.Tests + 36 Teleop.RobotHost.Tests, 0 failed |
| `Teleop.Eval -- verify` | `PASS` — golden log replays byte-identical across two passes |
| `Teleop.Eval -- audit` | `PASS` — no invariant violations |
| Root `CLAUDE.md`, `core/Teleop.Core/CLAUDE.md`, `core/Teleop.Core/Prediction/CLAUDE.md`, `docs/research-log/CLAUDE.md` | all read in full before starting |
| `core/Teleop.Core/Prediction/DoubleExponentialPredictor.cs` | read in full for house style |

## Feasibility verdict (written before the rest of the analysis)

**Yes-with-caveats.** `ekf` fits inside `IPredictor<TState>` exactly as written — nothing about
the interface's shape (`Observe`/`Predict`/`Reset`/`Diagnostics`, no allocation, no I/O, no wall
clock) blocks a Kalman-style filter. The caveats are not about the interface; they are about
everything *around* it:

1. `PredictorDiagnostics`'s uncertainty fields are two scalars (`PositionSigmaMeters`,
   `OrientationSigmaRadians`), which is a real, lossy projection of a multi-dimensional
   covariance — acceptable as a summary, but see §2.
2. **Nothing downstream reads those fields today.** All four existing reconcilers explicitly
   discard `diagnostics` (`_ = diagnostics;`), no metric in `docs/metrics.md` captures
   uncertainty, and the sweep harness never touches `Diagnostics` at all. Building `ekf` today
   produces a predictor whose one distinguishing output is provably unobservable by every
   existing consumer. This is not an ADR-level problem — no `Contracts/` change is needed, the
   plumbing (the `in PredictorDiagnostics diagnostics` parameter) already exists end-to-end — but
   it means `ekf` alone is not a complete, scorable piece of work. A consumer has to exist first,
   or the filter's only real deliverable (uncertainty) can never appear in a `results/` table.
3. Core has **no linear-algebra facility** beyond `Vector3`/`Quaternion`/`Matrix4x4`
   (`System.Numerics`, fixed 4x4). An EKF over pose needs at least a 6x6 (position+velocity) or
   9-12 dimensional (adding orientation and its rate) covariance, which does not exist as a type
   anywhere in this codebase and cannot come from NuGet. It would have to be hand-rolled.
4. `PredictorConfig.ProcessNoise`/`MeasurementNoise` are one scalar each; an EKF over both
   position and orientation plausibly needs at least four (translational/rotational process
   noise, translational/rotational measurement noise) to be sweepable in a way that reflects
   real motion-noise asymmetry. Two scalars under-specify it.

None of this is a "no" — it argues for build order (consumer before producer) and for treating
the config/diagnostics fields as adequate-but-lossy rather than wrong. See each section below for
the concrete trace.

## §1 — What `ekf` needs from `IPredictor<TState>`, and does it fit

`IPredictor<TState>` (`Contracts/IPredictor.cs`) requires: deterministic `Observe`/`Predict`,
robustness to out-of-order/duplicate/gapped observations, allocation-free `Predict`, parameters
from `PredictorConfig`, and a `Diagnostics` struct. None of this is EKF-hostile:

- **Determinism**: an EKF is a deterministic recursion (predict/update) given fixed inputs and a
  fixed RNG-free implementation — no different in kind from `double-exp`'s recursion, just with
  matrices instead of scalars. Satisfiable.
- **Out-of-order/duplicate/gap handling**: `DoubleExponentialPredictor` already sets the pattern
  (reject non-monotonic stamps outright, re-seed across `MaxObservationGapTicks`). An EKF can
  follow the identical policy — reject-outright for ordering (a filter update is not
  commutative/associative in a way that tolerates being replayed out of sequence any better than
  double-exp's recursion does), re-seed (reset `x` to the observation, reset `P` to a
  configured-but-currently-nonexistent `P0`) across a gap. The *gap* failure mode for an EKF is
  covariance divergence or loss of positive-definiteness rather than double-exp's NaN blowup, but
  the shape of the fix (detect via `MaxObservationGapTicks`, re-seed rather than propagate across
  it) is the same.
- **Allocation-free `Predict`**: achievable if every matrix buffer is preallocated in the
  constructor and reused in place — see §3.
- **Parameters from `PredictorConfig`**: partially — see §4.

So the interface itself is not the obstacle. Nothing in `IPredictor<TState>` needs to change.

## §2 — THE PLUMBING QUESTION

### 2a. What the diagnostics fields actually hold

`Types/PredictorDiagnostics.cs`:

- `HasUncertainty` — `bool`.
- `PositionSigmaMeters` — **one** `float`. One-sigma, isotropic (no axis breakdown, no
  cross-correlation with orientation or velocity).
- `OrientationSigmaRadians` — **one** `float`. Geodesic-angle one-sigma, also isotropic.

That is two numbers total. A real EKF over a 6-12 dimensional pose+derivative state produces a
6x6 or larger covariance matrix — 21+ independent numbers for a 6x6 symmetric matrix, or more for
a larger state including velocity/angular-rate. Collapsing that to two scalars is necessarily
lossy: it throws away the covariance's anisotropy (e.g. an EKF tracking a fast-moving hand is
usually far more uncertain along the direction of motion than across it) and every off-diagonal
term (position-orientation correlation, position-velocity correlation). This matters concretely
for the one use case the interface's own doc anticipates — "a reconciler that scales its
correction by predictor uncertainty" — because an isotropic sigma cannot express "trust the
lateral estimate but not the along-track one," which is exactly the information a matrix filter
uniquely has to offer over `double-exp`. The two scalars are a defensible *summary* (e.g.
`sqrt(trace(P_position)/3)` and the geodesic analogue for orientation) but they are a genuine
information loss, not merely a reporting convenience — and it is the loss of exactly the
information an EKF is supposed to contribute over the cheaper alternatives already in
`Prediction/`.

### 2b. Does anything downstream *read* the fields today?

Traced by direct grep across `Reconciliation/`, `Pipeline/`, `Metrics/`, `core/Teleop.Eval/`:

```
grep -rn "HasUncertainty\|PositionSigmaMeters\|OrientationSigmaRadians" (whole repo)
```

Every non-doc-comment, non-test hit is one of:

- `Types/PredictorDiagnostics.cs` — the definition itself.
- `Contracts/IReconciler.cs` line 59 — the doc comment instructing implementations to degrade
  gracefully.
- `Reconciliation/{SnapReconciler,SpringReconciler,VelocityMatchedReconciler,TimeBudgetedBlendReconciler}.cs`
  — doc comments, and each one's `Observe` body contains literally `_ = diagnostics;` right after
  the parameter is bound. Verified by reading all four bodies directly (not just grep): each XML
  doc says, near-verbatim across the four, "this reconciler behaves identically whether
  `HasUncertainty` is true or false ... reached by not depending on the field at all." This is
  not an oversight in one implementation, it is the uniform, deliberate design of **every**
  reconciler that exists.
- Two test files (`SnapReconcilerTests.cs`, `PassthroughPredictorTests.cs`,
  `DoubleExponentialPredictorTests.cs`, `ConstantVelocityPredictorTests.cs`) assert
  `HasUncertainty` is `false` for the existing (uncertainty-free) predictors, or that a
  hand-constructed diagnostics struct round-trips. None of these tests exercise a reconciler
  actually *changing* behaviour based on the flag.

The one real call site in the running pipeline is `Pipeline/OperatorEndpoint.cs:220`:
```csharp
_robotStateReconciler.Observe(sample, predictedAtCapture, _robotStatePredictor.Diagnostics);
```
So the value *is* threaded from predictor to reconciler in the actual pipeline (the plumbing is
wired, not merely declared) — it just terminates at `_ = diagnostics;` on the other end, in all
four current reconcilers. `Teleop.Eval/Sweep/SweepCommand.cs`, separately, calls
`predictor.Predict(now)` for its own online prediction-error metric but never reads
`predictor.Diagnostics` at all.

**Conclusion for 2b: the plumbing is wired end-to-end but the pipe is capped at the far end.**
Every consumer that could act on uncertainty explicitly, deliberately, and (per the doc comments)
correctly does not. This is not a bug — `IReconciler.Observe`'s own contract *requires* graceful
degradation when `HasUncertainty` is false, and these four reconcilers satisfy that by simply
never depending on the field, which is a valid way to satisfy the contract. But it means the
contract's "must degrade gracefully" clause has, in practice, been implemented as "always
degrade," because no reconciler yet has an uncertainty-aware branch to degrade *from*.

### 2c. Is uncertainty recorded anywhere a sweep could score it?

`docs/metrics.md` §4 ("Prediction quality") defines position error, orientation error, velocity
error, and failure rate — all accuracy metrics against ground truth, none of them uncertainty.
`grep -in "sigma\|uncertain\|covarian\|ekf\|kalman" docs/metrics.md` returns **zero** hits. §5
("Correction cost") defines `correction_magnitude_mm/deg`, `time_to_convergence_ms`, `jerk_mm_s3`
— all downstream of the reconciler's actual correction, and per §2b none of the four reconcilers
let uncertainty influence that correction, so even these would not carry an uncertainty signal
today.

There is no metric definition to add uncertainty to without inventing one, and this task
explicitly does not permit inventing one (nor would I propose it here without a reconciler that
consumes it, since a metric with no producer-to-consumer path is exactly the kind of thing that
gets recorded and never moves).

### 2d. Is the value of `ekf` currently unobservable?

**Yes.** Concretely: build `ExtendedKalmanPredictor.cs` today, register it, benchmark it against
the four existing reconcilers, and its one architecturally distinguishing output — a covariance
estimate — cannot appear in any `results/` table, because:

1. No reconciler reads it (confirmed by reading all four, not just grepping).
2. No metric name exists to carry it even if a reconciler did.
3. The only accuracy/correction-cost metrics that do exist (§4, §5 of `docs/metrics.md`) are
   observable *regardless* of whether the predictor reports uncertainty — an EKF would be scored
   on exactly the same axes `const-vel`/`double-exp` already are, and any win there would have to
   come from the filter's *point estimate* (the mean) being better than double-exp's smoothed
   level, not from anything related to its covariance. That is a fair fight to have, but it is not
   the fight this row in `Prediction/CLAUDE.md` says `ekf` exists for — the table's own annotation
   is "reports covariance; the reconciler can use it," and today no reconciler can, or does.

**What would have to be built first:** at least one reconciler that has an uncertainty-scaled
branch — e.g. a fifth `IReconciler<Pose>` implementation that scales correction rate/aggressiveness
by `PositionSigmaMeters`/`OrientationSigmaRadians` when `HasUncertainty` is true, and falls back to
one of the existing four's behaviour when it is false. **This is not an ADR-level change** —
`IReconciler.Observe` already accepts `in PredictorDiagnostics diagnostics` for exactly this
purpose, and a new reconciler is ordinary `Reconciliation/` + registry + test work, the same kind
of change as `budget-blend` or `spring` before it. The ADR-level question, if there is one, is
narrower and optional: whether two isotropic scalars are a sufficient interface for uncertainty
long-term (§2a), which only becomes worth raising once a consumer exists and is found to need more
than an isotropic summary.

## §3 — Determinism and allocation for a matrix filter

**Where would the matrices live?** Per `Prediction/CLAUDE.md` requirement 3 ("no allocation in
`Predict`; preallocate in the constructor") and the house style's "fixed-size buffers over
`List<T>`," the natural approach is a small number of `private readonly float[]` buffers sized
once in the constructor (e.g. a flat row-major buffer for an N x N covariance, N in {6, 9, 12}
depending on whether velocity and angular rate are tracked) and mutated in place by hand-written
predict/update code. A `float[]` allocated once at construction and never reallocated satisfies
invariant 8 (the ban is on allocation in the *hot path*, i.e. `Observe`/`Predict`, not on
construction). `readonly` on the array reference (not `readonly` contents, C# has no
fixed-size-buffer-of-float in safe code outside `unsafe`) is the idiom already used elsewhere in
Core for preallocated buffers.

**Is there a linear-algebra facility in Core today?** No. `grep`-ing for `Matrix`, `float[]`, and
`double[]` across `core/Teleop.Core` turns up nothing beyond `System.Numerics`'s `Vector3`,
`Quaternion`, and the unrelated 4x4 `Matrix4x4` (which is exactly 4x4, fixed, and intended for
homogeneous transforms — not a general NxN facility and not resizable to a 6x6 or 9x9 covariance).
Building `ekf` therefore means bringing along hand-rolled fixed-size matrix math: multiply, add,
transpose, and — the hard part — an invertible solve for the Kalman gain (`K = P Hᵀ (H P Hᵀ + R)⁻¹`).
For a 3-dimensional measurement (position-only observations) that inner term is 3x3 and invertible
in closed form; adding an orientation measurement makes it 6x6 and closed-form inversion stops
being practical, pushing toward a hand-written Cholesky or Gauss-Jordan solve. Zero-NuGet means
none of this can come from `MathNet.Numerics` or similar — it is new code, and it is exactly the
kind of numerical code this repo has avoided needing so far (`double-exp`'s docstring calls itself
"Kalman-free... no matrix" as a selling point).

**Float precision.** Core's value types (`Vector3`, `Quaternion`, `Pose`) are `float`
(single-precision), and `PredictorConfig`'s noise fields are `float`. Covariance recursions
(`P = (I - KH) P` or the numerically-preferred Joseph form) are known in the numerical-methods
literature to lose symmetry and positive-definiteness under repeated float32 updates, especially
under a stiff process/measurement noise ratio — an EKF-specific failure mode with no analogue in
`double-exp`'s four-scalar state. The analogous guard to `double-exp`'s "clamp the trend in
`Observe`, not only in `Predict`" (this file's documented divergence-to-NaN trap) would be:
enforce symmetry explicitly after every update (`P = (P + Pᵀ)/2`) and detect/reject a covariance
that has lost positive-definiteness (e.g. a failed Cholesky factorization) by re-seeding, the same
policy double-exp uses for a `dt` that produces an implausible trend. This has to be designed in,
not discovered later — an EKF that silently returns a rotation built from a NaN-corrupted quaternion
is a worse failure than double-exp's, because it is one matrix operation removed from where the
corruption started, harder to trace back.

**Cross-compiler determinism.** `verify`'s bit-identical-replay check runs within one process on
one compiler (`dotnet`); it does not cross-check `dotnet`'s CoreCLR JIT against Unity's IL2CPP
AOT compiler for the same golden log, and nothing in this repo currently does that cross-check
(confirmed: `audit`'s own description in root `CLAUDE.md` calls itself "the headless
approximation, not a guarantee" for exactly this reason). A two-line extrapolator like
`const-vel`/`double-exp` has very few floating-point operations per call, so any JIT-vs-AOT
difference in operation fusion (e.g. whether a multiply-add gets contracted to an FMA instruction)
is a single-ULP-scale discrepancy that does not compound across a run. An EKF's predict/update
recursion is applied every `Observe` call and *feeds its own output back into the next call's
input* (`P` and `x` are both stateful across time), the same way `double-exp`'s trend/level
feedback loop is — except with many more chained floating-point operations per step (matrix
multiplies, a matrix solve) than double-exp's four scalar updates. Whether this produces a
practically observable divergence between the `dotnet` `Teleop.Eval` replay and the same trace
run through Unity/IL2CPP is a real, EKF-specific open question that this assessment cannot resolve
without an actual IL2CPP build (which is out of scope here and belongs on the Windows box, per
root `CLAUDE.md`'s box-ownership rules) — flagged as a risk, not dismissed and not confirmed.

## §4 — `PredictorConfig`, field by field, for an EKF

| Field | Read by `ekf`? | Sufficient? |
|---|---|---|
| `MaxHorizonTicks` | yes — same clamp-the-extrapolation-horizon role as every other predictor | yes |
| `MaxObservationGapTicks` | yes — same re-seed-across-a-gap role | yes |
| `HistoryCapacity` | probably ignored — an EKF's "history" is its state vector + covariance, not a retained window; `double-exp` ignores this field for the identical reason | n/a |
| `SmoothingAlpha` / `SmoothingBeta` | ignored — these are exponential-smoothing-family parameters with no EKF analogue | n/a |
| `ProcessNoise` | yes, but **one scalar for the whole state** | **no** — see below |
| `MeasurementNoise` | yes, but **one scalar for the whole state** | **no** — see below |
| `MaxLinearSpeed` / `MaxAngularSpeed` | yes — same output-clamp role as `double-exp`'s clamp | yes |

**`ProcessNoise`/`MeasurementNoise` are under-specified for a pose EKF.** The doc comments give
them concrete units (`ProcessNoise`: m²/s³, continuous white-noise acceleration; `MeasurementNoise`:
m²) — both explicitly *positional*. There is no rotational counterpart field. A pose EKF's process
model plausibly needs a different noise intensity for translational acceleration than for angular
acceleration (a hand can accelerate rotationally far faster, relative to its dynamic range, than it
translates), and its measurement model needs a separate noise variance for the orientation
observation (radians², not the existing field's metres²). With only two scalars, a sweep varying
`ProcessNoise`/`MeasurementNoise` can only explore the case where translational and rotational
noise are forced into a fixed, arbitrary ratio (whatever the implementation hard-codes to convert
one scalar into two block sizes) — it cannot independently explore "confident in position, unsure
about orientation" or vice versa, which is a real, physically distinct regime (e.g. a controller
held rigidly but waved through the air). This is the same kind of under-specification the task
description flags for `double-exp`, generalized: `double-exp` explicitly ignores both fields
because it has no noise model at all; `ekf` would need to read them but the two-scalar shape
cannot express the axis-conditional noise a real pose EKF's tuning space has. Concretely this means
`PredictorConfig` would need at least two more fields (`RotationalProcessNoise`,
`RotationalMeasurementNoise`) for a sweep to characterize `ekf` properly — a small, additive change
to a shared struct (not a `Contracts/` change), but a real one, and one every other predictor's
registry-factory call site would have to keep compiling around (the struct is already the "one
shared block, each implementation reads its subset" design, so this is consistent with existing
practice, not a violation of it).

## §5 — The rotational problem

Quaternions live on a 3-sphere double-covering `SO(3)`, not a vector space: naive per-component
Kalman arithmetic on quaternion coefficients (treating `(x,y,z,w)` as an unconstrained ℝ⁴ Kalman
state) breaks the unit-norm constraint on every update and has no principled process-noise model
in that parametrization. The standard fix or is one of:

- **Error-state / multiplicative EKF (MEKF/ESKF)**: keep the nominal orientation as a quaternion,
  represent the *error* between true and estimated orientation as a small rotation vector in the
  tangent space at the current estimate (the log map), run the linear Kalman recursion on that
  3-dimensional error state (plus whatever other error states — position, velocity — are in ℝ³ᵏ
  already), then re-inject the corrected error back into the nominal quaternion via the exp map and
  reset the error state to zero. This is the standard approach in the aerospace/robotics
  attitude-estimation literature (e.g. Markley's multiplicative EKF, widely used for spacecraft
  attitude).
- **Manifold-aware full nonlinear filter** (e.g. an invariant EKF on `SE(3)`), which is more
  principled but a materially larger implementation lift and arguably out of proportion to what
  this repo needs next.

**Which one, and why:** the error-state formulation is the right fit here, because Core already
has almost exactly the primitives an error-state filter needs: `Types/MotionMath.cs` has
`ToRotationVector`/`FromRotationVector` (the log/exp maps), `IntegrateWorld` (compose a quaternion
with a world-frame rotation-vector increment — the "inject the corrected error state back" step),
and `RelativeRotationVector` (the "compute the tangent-space error between two orientations" step
— exactly what an innovation/residual computation needs). `DoubleExponentialPredictor` already
uses all four for its own smoothing recursion, so the pattern of composing pose updates through
this log/exp toolkit instead of raw quaternion arithmetic is established house style, not a new
convention `ekf` would have to invent.

**What is missing from `MotionMath` that an error-state EKF would need and does not have today:**
the *Jacobians* of these maps. An EKF's covariance propagation needs the derivative of the
process/measurement model with respect to the state, and for the rotational block that means the
Jacobian of the exp map (commonly the "right Jacobian of `SO(3)`," `J_r`, and its inverse) — a
closed-form expression involving the rotation angle and a `sinc`-like term, used to correctly
propagate covariance through the nonlinear log/exp composition rather than through a linear
approximation that is only valid for very small rotations. `MotionMath` has the maps themselves
but none of their derivatives. This is a genuine, nontrivial addition — not large in line count,
but exactly the kind of numerical code (small-angle series expansions, guarded against the same
near-identity singularity `MotionMath.RotationEpsilon` already guards in the maps it does have) that
is easy to get subtly wrong in a way unit tests miss unless they specifically probe near-zero and
near-π rotations.

## §6 — What would falsify "worth building" cheaply, before writing the filter

The most useful falsifier does not require writing the EKF at all, and much of it has effectively
already been run in §2 by reading the source rather than executing anything: **there is currently
no consumer**, confirmed by reading all four `Observe` bodies, not just grepping a symbol name.

The forward-looking, cheap-to-run version, for whoever picks this up next: **build the consumer
before the producer.** Take an existing reconciler (or a small new one) and give it a genuine
uncertainty-scaled branch — e.g. scale `TimeBudgetedBlendReconciler`'s convergence budget or
`SpringReconciler`'s stiffness by `PositionSigmaMeters` when `HasUncertainty` is true — fed by a
*hand-constructed* `PredictorDiagnostics` in a unit test (no filter needed yet, just a literal
struct with `HasUncertainty: true` and a chosen sigma). Then check, in that unit test and in a
minimal sweep, whether varying the injected sigma measurably moves `correction_magnitude_mm` or
`jerk_mm_s3` in the expected direction.

- If it does not move them — if, as the four existing reconcilers' docstrings already argue,
  correction cost is set entirely by the convergence budget/rate caps and not by anything
  uncertainty could scale — that **kills** the case for `ekf` outright, cheaply, without ever
  writing a matrix inversion: the filter's only distinguishing output would still be unobservable
  even after adding a consumer, because the consumer's own architecture (budget-driven, not
  uncertainty-driven) has no dial for it to turn.
- If it does move them, that is the falsifiable, buildable prerequisite this section's §2d called
  for, and *then* `ekf`'s covariance has something to be measured against.

This is a cheap (one afternoon, one new reconciler + one test) experiment that should be run before
committing to the matrix math in §3, because it tests the part of the idea (does uncertainty-aware
correction help at all) that is orthogonal to how the uncertainty is produced (a real EKF vs. a
hand-set constant).

## §7 — Estimated cost to build (if pursued as originally scoped, `ekf` alone)

**Files touched**, following `/new-impl`'s discipline once a consumer exists (per §2d) or if built
speculatively without one:

- `core/Teleop.Core/Prediction/ExtendedKalmanPredictor.cs` — the filter itself. Likely the largest
  new file in `Prediction/` to date: state vector + covariance buffers, predict step, update step,
  a hand-rolled small-matrix solve, symmetry/PD enforcement, gap re-seed policy, output clamps
  matching `MaxLinearSpeed`/`MaxAngularSpeed`.
- `core/Teleop.Core/Types/MotionMath.cs` — additive change, the `SO(3)` Jacobian(s) needed for
  covariance propagation through the log/exp maps (§5). Shared file — touching it affects every
  other consumer of `MotionMath`, so it should be pure addition, not a signature change.
- `core/Teleop.Core/Types/PredictorConfig.cs` — additive change, at minimum two more fields for
  rotational process/measurement noise (§4). Shared struct — every existing registry factory call
  site keeps compiling, but every predictor's constructor call in `Registries.cs` and every test
  that constructs a `PredictorConfig` literal would need the two new arguments.
- `core/Teleop.Core/Registry/Registries.cs` — one hand-written entry.
- `core/Teleop.Core.Tests/Prediction/ExtendedKalmanPredictorTests.cs` — new test file.
- If §6's consumer-first path is taken: a new or modified file in `Reconciliation/` plus its own
  test — but that is explicitly a *different*, separable piece of work with its own hypothesis and
  falsifier, not a sub-task of `ekf`.

**Roughly what the tests would need to prove**, beyond the standard `Reset()`-returns-to-
as-constructed and allocation-assertion tests every predictor gets:

1. Determinism: identical observation sequence + target times => bit-identical output, run twice.
2. Ordering/duplicate/gap robustness, mirroring `double-exp`'s test suite structure — including a
   test that a gap re-seeds `P` rather than propagating a stale covariance across it.
3. Convergence: `P`'s diagonal shrinks under repeated informative observations of a static or
   constant-velocity target (the filter should become *more* confident, not diverge) — this is the
   EKF-specific analogue of `double-exp`'s "constant velocity is a fixed point" property, and it is
   the test most likely to catch a Jacobian sign error or an unnormalized log/exp round trip.
4. Positive-definiteness / symmetry preserved after N updates for N large enough to be meaningful
   (the float32 numerical-stability concern in §3) — a test that would have caught the exact class
   of divergence `double-exp`'s own docstring describes for its own state, but for covariance
   instead of trend.
5. Near-singular rotation guard: behaviour at a near-identity and a near-π relative rotation does
   not NaN, mirroring `MotionMath.RotationEpsilon`'s existing guard but exercised through the new
   Jacobian code specifically.
6. `HasUncertainty` is true once the filter has processed enough observations to have a meaningful
   covariance, and the reported sigma is a documented, principled function of `P` (e.g.
   `sqrt(trace(P_position)/3)`), not an arbitrary placeholder.

**What is genuinely uncertain**, and would need to be found out by building rather than argued in
advance:

- Whether a 3x3-closed-form position-only measurement update is enough for the intended use case,
  or whether orientation must be a joint measurement (6x6 solve) — this changes the numerical
  method (closed-form inverse vs. hand-written Cholesky) and therefore the implementation cost
  materially.
- Whether float32 is numerically adequate for the covariance recursion at the noise/horizon ranges
  this repo's profiles actually exercise, or whether it diverges often enough in practice to need
  the symmetry/PD guard to trigger routinely (in which case the "re-seed on failure" policy is not
  a rare-edge-case fallback but a load-bearing, frequently-hit part of the filter's normal
  operation) — only measurable by actually running it against `core/testdata/traces/`.
- The cross-compiler float-determinism question in §3, which this assessment explicitly could not
  resolve without an IL2CPP build.

## Left undone / for a human

- No code was written, per the hard scope of this assessment.
- The consumer-first experiment in §6 is the single highest-value next step and is *not* an
  ADR-level change — it is ordinary `Reconciliation/` work. Recommend running it before `ekf` is
  picked up as an implementation task, so that whoever builds `ekf` is building toward a metric
  that can actually move.
- The narrower, optional question in §2a/§2d — whether two isotropic scalars are the right
  long-term shape for `PredictorDiagnostics`'s uncertainty fields, once a real anisotropic
  covariance exists to summarize — is left open. It only becomes worth raising once a consumer
  exists and is found to need more than an isotropic summary; raising it now would be redefining
  a `Types/` struct ahead of any evidence it is wrong, which this assessment's scope does not
  permit and which the task's own hard stops caution against doing without cause.
- The cross-compiler (dotnet vs IL2CPP) float-determinism risk for an iterative matrix filter
  (§3) is flagged, not resolved — resolving it needs a Unity build, which belongs on the Windows
  box per root `CLAUDE.md`'s box-ownership rules, not this assessment.
