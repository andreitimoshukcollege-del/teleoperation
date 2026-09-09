# 2026-09-08 — `exp-smooth-track` / `TrackingLagReconciler`

**Agent:** candidate C of a three-candidate panel on the Reconciliation axis. **No comparison
sweep was run and no ranking against any other reconciler is claimed anywhere in this file.** The
head-to-head is the organizer's step; a candidate that scores itself is not evidence.
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-a03ebfc90f49aa972`
**Branch:** `main` (worktree, left uncommitted by instruction)
**HEAD at start:** `76878bf87b06896d35e026e336b78ccfc2f8c06e`

## Seed self-check

| Check | Result |
|---|---|
| `git rev-parse HEAD` == `76878bf87b06896d35e026e336b78ccfc2f8c06e` | **PASS** |
| Root `CLAUDE.md`, `core/Teleop.Core/CLAUDE.md`, `Reconciliation/CLAUDE.md` read in full | **PASS** |
| `Reconciliation/CLAUDE.md` "Tried and rejected" read in full (both entries) | **PASS** |
| `docs/metrics.md` §5 read (metric names, units, cadence) | **PASS** |
| `docs/research-log/CLAUDE.md` read (log structure) | **PASS** |
| `docs/research-log/2026-09-08-budget-blend.md`, `2026-09-08-velocity-match.md` read for `exp-004` caveats | **PASS** |
| `dotnet test` green on clean tree before any edit | **PASS** — 458 Core + 28 RobotArm + 3 Eval + 36 RobotHost |
| `Teleop.Eval -- verify` on clean tree | **PASS** |
| `Teleop.Eval -- audit` on clean tree | **PASS** |
| Files I may touch confirmed against the brief before editing | **PASS** |

Environment prefix used for every command in this run (the sandbox refuses `export VAR=...`):
`DOTNET_ROOT=/home/andrei/.dotnet PATH=/home/andrei/.dotnet:/home/andrei/.local/bin:/usr/local/bin:/usr/bin:/bin /home/andrei/.dotnet/dotnet ...`

## Prior work consulted (candidates and vocabulary only — never evidence about this system)

Searched for first-order-lag / exponential-smoothing latency compensation in VR. Two useful items,
neither of which enters `results/` and neither of which settles anything here:

- LaViola, *An Alternative to Kalman Filter-Based Predictive Tracking*
  (https://cs.brown.edu/people/jlaviola/pubs/kfvsexp_final_laviola.pdf) — proposes **double**
  exponential smoothing as a predictive tracker, claims parity with an EKF at roughly 135x lower
  cost. **Conditions it was measured under:** head-orientation traces, prediction horizons of tens
  of ms, desktop-era VR; no packet loss, no network, no separate reconciliation stage. Relevant to
  this repo as the already-implemented `double-exp` *predictor*, not to this axis. Worth stating
  plainly: the literature's "exponential smoothing" is almost always a **predictor**; using the
  same law as a **reconciler** (what this candidate does) is a different placement in the pipeline
  with a different failure mode.
- The general result that a first-order (overdamped, no velocity state) tracker has a **nonzero
  steady-state error under constant-velocity input**, and that the residual is the
  velocity-times-latency term `Δp = v·t_l` (recurring across the predictive-tracking literature,
  e.g. the survey at https://vrarwiki.com/wiki/Predictive_tracking). This is exactly the bias this
  candidate is built to expose and quantify, arrived at from the opposite direction: prior work
  adds `v·t_l` to *cancel* a lag; this law *creates* one of the same form.

Nothing found that measures a first-order lag as a *reconciler* (a post-prediction display filter)
against jerk and time-to-convergence. Recording that as a genuine gap rather than as a claim.

## Hypothesis (written before any code)

The axis's planned row calls `exp-smooth` "one time constant; simple, **biased**". Candidates A
and B read "biased" as a property of a decaying *residual offset*, which in fact has zero
steady-state bias. This candidate tests the other reading — the one the word names — **no offset
state at all**: the displayed pose is a first-order lag chasing the predictor's output directly,

```
displayed <- displayed + (predicted - displayed) * (1 - exp(-dt / tau))
```

with the geodesic equivalent for orientation. One state vector. No seeding, no correction-onset
bookkeeping for the trajectory, no residual.

**Claim.** This is the smallest thing that can be called a reconciler, and its cost is a
**permanent, velocity-proportional tracking bias**: against truth moving at speed `v` the display
settles `v·tau` behind and never catches up while motion continues. Its benefit is that it has no
position step, no seeding hold, and therefore an unusually smooth displayed trajectory.

**What I expect to be true, stated as falsifiable predictions:**

1. **Static-truth convergence is exponential and bounded.** With truth held still, displayed error
   decays as `|e0|·exp(-t/tau)` and reaches 1% of `|e0|` at exactly `MaxTimeToConvergenceTicks`,
   with `tau = budget / ln(100)`. *Falsified if* the measured decay is not exponential or misses
   the 1% envelope at the budget.
2. **Moving-truth steady-state lag has an exact closed form.** With fixed frame interval `dt` and
   truth advancing `v·dt` per frame, the displayed error converges to the fixed point
   `e* = v·dt·r/(1-r)` with `r = exp(-dt/tau)`, whose `dt -> 0` limit is `v·tau` (more precisely
   `v·(tau - dt/2) + O(dt^2)`). *Falsified if* the measured steady-state error does not match that
   closed form to float tolerance, or if it decays toward zero over long runs.
3. **The law is C0 but not C1.** Displayed position is continuous always. Displayed *velocity*
   steps whenever the predictor's output steps: a predictor jump of `Δ` injects a velocity step of
   `(1-exp(-dt/tau))·Δ/dt`, which tends to `Δ/tau` as `dt -> 0` — bounded, unlike `snap`'s
   `~Δ/dt`, but nonzero. *Falsified if* the measured velocity step is zero (I would then actually
   have C1 and the premise is wrong) or if it grows without bound as `dt -> 0` (it would then be
   no better than `snap` and the candidate is pointless).
4. **Requirement 1 is satisfiable only under a reading I must state, not assume.** Requirement 1
   says "under a **constant** correction the visible error reaches zero, or a stated bound, within
   a bounded time" and then "a reconciler that can lag indefinitely is a bug, not a tradeoff".
   Under static truth I satisfy it cleanly. Under moving truth I do not converge at all, and the
   second sentence points directly at this law. My prediction, written before the work: **I will
   be able to prove a stated bound (`v·tau`), and I will not be able to argue that the bound
   satisfies the sentence about lagging indefinitely.** That is a verdict for the admissibility
   judge, and if the answer is "inadmissible on the axis as written", that is a complete outcome.

**What would make me abandon the candidate outright:** if the law cannot be made allocation-free
or deterministic within `IReconciler<Pose>`; if `time_to_convergence_ms` cannot be given a
defensible definition (it is not obvious that a law with no correction episode has one); or if the
metric cadence cannot be made identical to the rest of the axis for `jerk_mm_s3` and
`correction_magnitude_*`.

**Known artifacts I am not rediscovering** (from the brief and from `Reconciliation/CLAUDE.md`):
`correction_magnitude_*` is a control identical across the axis by construction;
`prediction_*_error_*` is emitted by `SweepCommand` from the *predictor* against ground truth, not
from the displayed pose; `jerk_mm_s3` must come from the shared `DisplayedJerkEstimator` on every
advancing frame; every trial opens with a ~1.3 m correction because the plant starts at
`Pose.Identity`; the one-frame display hold in the offset family is a deliberate, measured
tradeoff that I must not touch and must not present my lack of it as a fix.

## Design decisions

**`tau` from the budget, against a stated envelope.** `SpringReconciler` reads
`MaxTimeToConvergenceTicks` as the time by which its critically damped envelope falls to 1% of the
initial error, and pins the resulting constant with a test against its defining equation. This does
the same thing with the first-order envelope: `exp(-T/tau) = 0.01`, so `tau = T / ln(100)` and
`LagSettleConstant = 4.6051702`. Same 1% fraction deliberately, so that "converged within the
budget" denotes the same envelope on both laws. `LagSettleConstantMatchesItsDefiningEquation`
recomputes `exp(-x)` from the *implementation's* `TimeConstantSeconds` and asserts it lands on 0.01,
so the literal and the fraction cannot drift apart.

Rejected alternative: `tau = T` (one time constant per budget, i.e. a 37% envelope). It is simpler
to explain but it means "converged within the budget" would denote something different here than on
the rest of the axis, which is exactly the kind of quiet metric drift the axis's own experiment note
warns about.

**No config field was added.** The reconciler reads both tolerances, `MaxTimeToConvergenceTicks`
and both rate caps, and ignores `RollbackHistoryCapacity`. Same subset as `spring`.

**The rate caps necessarily mean something different here, and I could not avoid it.** In the offset
family the caps bound the *correction* speed, because the correction is a separable offset. This law
has no such decomposition — one chase step carries both the correction and the operator's own motion
— so the caps bound *total displayed speed*. That is strictly tighter. I chose to apply them anyway
rather than ignore them, because ignoring a configured rate cap is worse than honouring a slightly
different quantity, and the type doc states which quantity. At the sweep's operating point
(`maxCorrectionLinearSpeedMetersPerSecond: 5`, synthetic operator peak speed 0.54 m/s) the cap is
about 9x from binding, so it does not bite there — but a future experiment that tightens it would
slow this reconciler where it would only slow the others' corrections. Flagged, not hidden.

**Bit-exact pass-through short circuit.** If the displayed pose already equals the prediction
exactly, the pose is returned untouched rather than run through the arithmetic, so an already-
agreeing predictor reduces to exact pass-through instead of drifting by renormalization noise. Note
the scope: this holds only while the prediction is *constant*. A moving prediction never passes
through — that is the bias, not an exception.

**Frame-rate-independent blend.** The blend fraction is `1 - exp(-dt/tau)` recomputed per frame, not
a fixed per-frame alpha. A fixed alpha would silently retune the time constant whenever the frame
interval moved, which on the impaired profiles this project studies is constantly.

### A bug the tests caught, worth recording because the wrong version looked right

The first version defined `IsConverged` the way the offset family does — "nothing pending, nothing
in flight". `AgainstMovingTruthTheDisplayedPoseSettlesAtTheClosedFormBiasAndStaysThere` failed on
it: with no `Observe` call at all (a steadily moving operator whose predictions never disagree
beyond tolerance), no episode ever opens, so the flag read **`true` while the display sat a steady
8.5 mm behind truth**.

For an offset-carrying law that definition is sound — no correction in flight implies zero residual
implies the display *is* the prediction. For this law the implication is false. `IsConverged` is now
a statement about the display (`... && _displayHasCaughtUp`, cached each advancing frame), which is
what `IReconciler`'s own wording asks for ("the visible state is within the implementation's stated
tolerance"). Pinned by `IsConvergedIsAboutTheDisplayNotAboutACorrectionEpisode`.

This is the single most important implementation detail in the candidate: the natural, copy-the-
neighbour definition of the convergence flag **silently conceals exactly the property the candidate
exists to expose.**

### An instrument limitation found in passing — reported, not changed

`PoseMath.OrientationErrorRadians` is `2*acos(|dot|)`, which loses roughly half the float mantissa
as the angle approaches zero. Measuring a 4.0 mrad residual, it reads 3.9 mrad — a 2.3% error, and
the error grows as the angle shrinks. This is a property of the shared error function, not of any
reconciler, and it is far below the 17 mrad orientation tolerance the sweep uses, so it changes
nothing about any recorded result.

It does mean an orientation-convergence assertion cannot be written against that function below
about 10 mrad. `OrientationFollowsTheSameExponentialAlongTheGeodesic` therefore measures the
residual with `MotionMath.RelativeRotationVector(...).Length()` (an `atan2` path, well conditioned
near zero) for the closed-form comparison, and still uses `PoseMath` for the tolerance-facing
assertion, because that is the instrument the reconciler itself uses. **`PoseMath` was not touched**
— it is out of my file scope and changing a shared error function mid-panel would invalidate every
comparison in the repo. Left for a human in the section below.

## Results

### What was built

| Artifact | Path |
|---|---|
| Implementation | `core/Teleop.Core/Reconciliation/TrackingLagReconciler.cs` |
| Unity meta (fresh guid `e9695a87a6a44263a59a99031fa8e218`, checked for collision) | `core/Teleop.Core/Reconciliation/TrackingLagReconciler.cs.meta` |
| Tests, 31 of them | `core/Teleop.Core.Tests/Reconciliation/TrackingLagReconcilerTests.cs` |
| Registry, exactly one line, key `exp-smooth-track` | `core/Teleop.Core/Registry/Registries.cs` |

No other file was created or modified. `Reconciliation/CLAUDE.md`, `docs/metrics.md`,
`Types/ReconcilerConfig.cs`, `Contracts/IReconciler.cs`, the other four reconcilers,
`DisplayedJerkEstimator.cs`, `Teleop.Eval/`, `experiments/`, `analysis/` and `results/` are all
untouched — verified by `git status` (see Verification).

### Prediction 1 — static-truth convergence: **CONFIRMED**

`UnderStaticTruthTheErrorDecaysExponentiallyToTheStatedEnvelope` asserts the displayed error against
the closed form `|e0| exp(-t/tau)` **frame by frame** over 20 frames (not just at the endpoint,
which any law landing in the right place would pass), to 6 decimal places, and confirms it is at
exactly 1% of the initial error one budget after onset.
`UnderStaticTruthTheDisplayLandsInsideToleranceAndReportsItOnce` confirms it lands inside the
configured 1 mm tolerance, that `IsConverged` goes true, and that exactly one
`time_to_convergence_ms` sample is emitted.

### Prediction 2 — the closed-form tracking bias: **CONFIRMED, and this is the candidate's core result**

The bias bound, in the form the admissibility judge should read it:

> Against a prediction advancing at constant speed `v` with a fixed frame interval `dt`, the
> displayed pose settles at
>
> **`e* = v · dt · r / (1 − r)`,  `r = exp(−dt/tau)`,  `tau = MaxTimeToConvergenceTicks / ln(100)`**
>
> and stays there for as long as motion continues. Its small-`dt` limit is `v · tau`; more precisely
> `v · (tau − dt/2) + O(dt²)`, the `−dt/2` being the sampling offset from stepping toward the
> current frame's prediction rather than the next one's.

Published as `TrackingLagReconciler.SteadyStateLagMeters(v, dt)` so it is asserted against a
simulation rather than trusted as prose. Four tests:

| Test | What it establishes |
|---|---|
| `AgainstMovingTruthTheDisplayedPoseSettlesAtTheClosedFormBiasAndStaysThere` | measured lag matches the closed form to 5 dp; lag at 20 budgets equals lag at 10 budgets, i.e. it is a bias and not a slow transient; it exceeds the convergence tolerance, i.e. it is a visible failure and not a technicality below the noise floor |
| `TheTrackingBiasIsProportionalToOperatorSpeed` | doubling the speed doubles the lag — a property of the law, not of the trace, so there is no operating point at which it becomes negligible other than standing still |
| `TheClosedFormBiasApproachesSpeedTimesTauAsTheFrameIntervalShrinks` | the discrete fixed point rises monotonically to `v·tau` as `dt` falls, with the whole deficit accounted for by `v·dt/2` to within 2% |
| `TheBiasDecaysOnceTheOperatorStops` | the bias is permanent *while motion continues*, which is weaker than "permanent" — once truth stops the lag decays away on the ordinary exponential |

**Analytic prediction for the sweep's operating point** — derived from `SweepCommand`'s synthetic
operator trajectory (`x = 0.5 sin t`, `z = 1 + 0.3 cos 0.7t`) and the closed form above, at
`convergenceBudgetMs: 100` (so `tau = 21.7 ms`) and the sweep's 5 mm position tolerance.
**These are calculations, not measurements. No sweep was run. They are here so a judge can check the
measurement against them, not so they can be quoted as results:**

| quantity | value |
|---|---|
| operator speed, p50 / p95 / max | 0.383 / 0.528 / 0.542 m/s |
| displayed tracking bias, p50 / p95 / max | **8.3 / 11.5 / 11.8 mm** |
| fraction of the cycle the bias is inside the sweep's 5 mm tolerance | **22%** |

### Prediction 3 — C0 yes, C1 no: **CONFIRMED, violation quantified**

> A predictor jump of `d` across a frame of `dt` steps the displayed velocity by
> **`(1 − exp(−dt/tau)) · d / dt`**, bounded above by `d / tau` at every frame interval.

Published as `VelocityStepAtPredictorJump(d, dt)`. `APredictorJumpStepsTheDisplayedVelocity`
asserts the measured step equals it, asserts it is strictly positive (**so C1 does not hold, stated
rather than glossed**), and asserts it stays under `d/tau`.

`TheVelocityStepDoesNotScaleWithTheInverseFrameInterval` is the load-bearing one: halving the frame
interval grows the step by ~12%, not by 2x. That is the structural difference from a
position-discontinuous law, whose velocity step scales as `1/dt` without bound.

`TheDisplayedPositionNeverStepsAndItsDisplacementShrinksWithTheFrameInterval` establishes the half
of requirement 2 that *is* satisfied: displayed position is continuous, and the per-frame
displacement halves when the frame interval halves.

**Three distinct violations of requirement 2 now exist on this axis and they are not
interchangeable.** `snap`: position step `d`, velocity step `d/dt`, unbounded as `dt → 0`. The
offset family: no position step, an O(1) velocity *dropout* of one frame at onset and at every
retarget. This law: no position step, a magnitude-proportional velocity *step* of at most `d/tau`.
Summarising all three as "violates C1" would lose the only information that matters about them.

### Prediction 4 — is requirement 1 satisfiable by this law? **My honest answer: not as written.**

Both halves are proved. The judgement is a human's, and here is mine with the argument, not a
verdict dressed as a fact.

The clause has two sentences and this law splits them:

- *"Under a **constant** correction the visible error reaches zero, or a stated bound, within a
  bounded time — provable by test."* On the literal reading — a constant correction is a fixed
  disagreement against static truth — **this law satisfies it cleanly**, with an exponential to a
  1% envelope at the configured budget, proved by test.
- *"A reconciler that can lag indefinitely is a bug, not a tradeoff."* **This law lags
  indefinitely, in time, whenever the operator keeps moving.** The lag's *magnitude* is bounded
  (`v·tau`, provable, tight), but its *duration* is unbounded, and the sentence is about duration.

So the honest position is: **`exp-smooth-track` satisfies requirement 1 only if the axis is willing
to read "a stated bound" as admitting a bound proportional to operator speed rather than to
configuration.** I do not think the requirement, as written, means that — the second sentence reads
like it was written to exclude precisely this. My recommendation is therefore that **this candidate
is inadmissible on the axis as currently written**, and that the interesting question for a human is
not "does it pass" but "is requirement 1 the right requirement". Two ways forward, both a human's
call and neither of them mine to take:

1. Rule it inadmissible. Then the axis has established, with a closed form and a test, *why*
   the no-offset reading of `exp-smooth` cannot live on this axis — which is a finished result and
   closes the question rather than leaving it to be re-proposed.
2. Amend requirement 1 to distinguish "converges to truth" from "tracks truth with a bounded
   offset", and admit the second class with a mandatory published bias bound. That is a real
   research position — a bounded, smooth, permanently lagging display may well be preferable to a
   converging but jerkier one for nausea — but it is an axis-definition change and it needs an
   owner, an ADR, and a metric that can see the bias (see below).

I did not amend requirement 1 or 2, and did not touch `Reconciliation/CLAUDE.md`, per the brief.

### Requirements 3, 4, 5

- **3 (correction cost every step).** `correction_magnitude_mm` / `_deg` emitted from `Observe` on
  every qualifying sample, stamped at the sample's own capture tick, with the same staleness and
  duplicate rejection `SpringReconciler` documents — pinned by
  `ObserveEmitsCorrectionMagnitudeOnceAtTheSamplesCaptureTick`, `ObserveIgnoresDuplicateAndStaleSamples`
  and `AnAgreeingSampleIsNotACorrection`. `jerk_mm_s3` from the shared `DisplayedJerkEstimator` on
  **every advancing frame**, pinned by `JerkIsEmittedOnEveryAdvancingFrameOnceFourPositionsExist`
  (23 frames driven, 20 emissions). `time_to_convergence_ms` once per episode — **with a cadence
  caveat that is itself a finding**, below.
- **4 (deterministic, allocation-free).** `TwoInstancesGivenIdenticalInputAgreeBitForBit`,
  `ReconcileAllocatesNothing`, `ObserveAllocatesNothing` (via `AllocationAssert.Zero`, 10 000
  warmup + 10 000 measured iterations, exactly zero bytes), `ReconcileIsIdempotentInNowTicks`,
  `ResetReturnsTheReconcilerToItsAsConstructedState` (compares both trajectory and the full metric
  stream against a fresh instance).
- **5 (graceful degradation without predictor uncertainty).** Reached by not reading the field at
  all; `PredictorUncertaintyIsIgnoredEntirely` drives two instances with `PredictorDiagnostics.None`
  and with a populated covariance and asserts bit-identical trajectories.

### The `time_to_convergence_ms` cadence problem — a finding, not a caveat

The metric definition is unchanged (docs/metrics.md §5: onset frame to the frame the displayed state
is within tolerance). What differs is **which episodes ever reach the terminating condition.** An
episode opened while the operator is moving cannot close until the motion slows enough that `v·tau`
falls inside tolerance.

`NoConvergenceIsReportedWhileTheOperatorKeepsMoving` pins this: one qualifying correction is
reported, 297 frames of continuous motion follow, and **zero** `time_to_convergence_ms` samples are
emitted. The two metric families disagree about how many events happened, by construction.

Against the sweep's actual synthetic operator this is worse than "never emitted", because it is
*selectively* emitted: by the calculation above, the bias is inside tolerance for 22% of the cycle,
clustered at the sinusoid's velocity minima. **The episodes that survive to be measured are exactly
the slow-motion ones, which are the ones with the shortest lag and hence the shortest convergence
time.** That is textbook survivorship, and it points in this candidate's favour.

Whoever pools `time_to_convergence_ms` percentiles across this axis must exclude this reconciler
from that pool, or report its sample count next to its percentiles. I could not fix this without
either redefining the metric (forbidden, and correctly so) or adding a new one (out of my file
scope). It is written up under "Left undone" as the concrete thing a human should decide.

## How this result could flatter itself

Read this section before reading any sweep number for `exp-smooth-track`. It is the most important
part of this log.

### 1. The sweep cannot see this candidate's defining cost. Its defining benefit is fully visible.

This is not a caveat. It is a structural asymmetry that will hand this candidate an unearned win if
anyone reads the head-to-head naively.

**Nothing in the sweep measures displayed-pose accuracy.** Per known artifact 2,
`prediction_position_error_mm` and `prediction_orientation_error_deg` are emitted by
`Teleop.Eval/Sweep/SweepCommand.cs` from the **predictor's** estimate against plant ground truth —
*before the reconciler runs, and identically for every reconciler*. Per known artifact 1,
`correction_magnitude_mm` / `_deg` are measured from the predictor's disagreement in `Observe`, also
before any reconciler acts, also identical across the axis by construction. Both are controls.

So the **8.3 mm p50 / 11.5 mm p95 displayed tracking bias** calculated above — the entire cost of
this candidate, the thing that makes it "biased", the reason it may be inadmissible on requirement 1
— **does not appear in any metric the sweep emits.** Not in one column. Not partially. At all.

Meanwhile `jerk_mm_s3` *is* fully visible, and this law should score very well on it: it never steps
position, it has no seeding hold, its velocity step at a correction is bounded by `d/tau` instead of
scaling as `d/dt`, and it absorbs the ~1.3 m opening transient (known artifact 4) as a smooth
exponential ramp rather than as a hold-then-decay.

**Stated plainly: a naive reading of the head-to-head will show this candidate as the smoothest
reconciler on the axis while being blind to the only thing it does worse. That apparent win would be
an artifact of the instrument, not a property of the algorithm.** If this candidate comes out ahead
on jerk, the correct conclusion is *"the sweep does not measure what this candidate costs"*, not
*"this candidate is better"*.

### 2. Its jerk numbers are not mechanism-comparable with the offset family's

Per known artifact 5, every offset-carrying reconciler holds the display for exactly one frame at
correction onset and at every retarget. That hold is deliberate and measured: the obvious fix was
implemented, swept and **reverted** because jerk p99 regressed 81–865% on impaired profiles and the
spread between reconcilers collapsed to 1.01x.

This law has no such hold — **because it seeds no offset**. That is a structural consequence of
having no offset state, **not a fix, not a discovery, and not a solution to the problem the reverted
attempt failed on** (which required exact jump cancellation *and* motion continuity together). I did
not touch the other reconcilers.

The consequence for a judge: a jerk difference between this candidate and the offset family is
substantially a difference in *whether a one-frame hold exists*, not a difference in convergence
law. Those distributions are shaped by different mechanisms and comparing them measures the
mechanism difference at least as much as the law.

### 3. Survivorship in `time_to_convergence_ms`

Covered in full above. Short version: episodes that never converge are never emitted, and the ones
that do converge are exactly the slow-motion ones with the shortest lag. A small, fast-looking
`time_to_convergence_ms` sample here is survivorship, not speed. Sample counts must be reported next
to percentiles or this reconciler must be excluded from that pool.

### 4. The opening transient is absorbed differently

Known artifact 4: every trial opens with a ~1.3 m correction because the plant starts at
`Pose.Identity` while the synthetic operator starts at z≈1.3, and on `lan` (~75 corrections in 2500
frames) that startup transient dominates p95/p99. This law's first frame adopts the prediction
exactly and then ramps, so its opening frames look nothing like the offset family's. On a profile
where corrections are rare, the opening transient is a large share of the distribution and this
difference alone could move a percentile. **Flagged, not exploited** — I did not special-case the
first frame to improve any number; adopting the prediction on frame one is the only behaviour that
does not invent an opening correction against a pose that was never displayed.

### 5. Things I checked because a clean result is suspicious

- **The static-truth exponential looked too clean.** It is clean because it genuinely is exactly
  geometric — I asserted it frame by frame against the closed form rather than at the endpoint, so
  the test would fail for any law that merely lands in the right place.
- **The first `IsConverged` definition made the candidate look better than it is** and I caught it
  only because a test asserted the *negative*. See the design-decisions section. If I had written
  only positive-convergence tests, that flattering bug would have shipped.
- **No sweep was run**, so nothing in this log is a measurement of this system under a network
  profile. Every number here is either a unit-test assertion or an analytic calculation, and both
  are labelled as such.

## Verification

### Final gate run — `just core-check`, green

Run after the session interruption described below, on the restored tree:

```
Passed!  - Failed: 0, Passed: 489, Total: 489 - Teleop.Core.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 28,  Total: 28  - Teleop.RobotArm.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 3,   Total: 3   - Teleop.Eval.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 36,  Total: 36  - Teleop.RobotHost.Tests.dll (net8.0)
verify: PASS -- .../basic-session.tlog replays byte-identical across two independent passes,
                and matches the original file exactly.
audit: PASS -- no invariant violations found in .../core/Teleop.Core or
               .../build/Teleop.Core/bin/Debug/netstandard2.1/Teleop.Core.dll.
```

### An incident worth recording: the test file was briefly off disk

While running the flake diagnostic below I moved `TrackingLagReconcilerTests.cs` to `/tmp` to get an
A/B measurement with and without it, and the session hit its rate limit mid-diagnostic — leaving the
worktree with an implementation, a log describing 31 passing tests, and **no test file**. It was
restored byte-identically (44 080 bytes, 31 `[Fact]`s, all passing) and every test the Results
section names is present and green.

The lesson is procedural and worth more than the incident: **a diagnostic that removes a file from
the tree leaves the tree in a state that looks like the work was never done.** If the restore had
failed, the log would have been claiming verification that did not exist on disk — the exact failure
mode the research-log discipline exists to prevent. A copy rather than a move would have cost
nothing and carried no such risk.

### The pre-existing allocation flake — investigated, not mine, and not fully fixed either

During the final gates a test failed that I did not write:

```
Teleop.Core.Tests.Types.MotionMathTests.ClampMagnitude_Allocates_Zero_Bytes [FAIL]
  Expected zero allocation over 10000 iterations, but 2872 bytes were allocated (0.287 bytes/call).
```

**0.287 bytes/call is not a real allocation** — the smallest managed object is 24 bytes, so a
fractional per-call figure can only be the JIT-tiering artifact `AllocationAssert`'s own doc comment
describes ("failures of 0.165 to 0.678 bytes/call"). The test covers `MotionMath.ClampMagnitude`,
a method I did not modify, in a file I did not touch.

Measurements taken before drawing a conclusion:

| condition | runs | Core-test failures |
|---|---|---|
| `dotnet test Teleop.Core.Tests` alone, my tests present | 8 | 0 |
| `dotnet test Teleop.sln` (4 projects in parallel), my tests present | 6 | 1 |
| `dotnet test Teleop.sln`, my test file moved out of the build | 6 | 0 |

So it reproduces only under solution-wide parallel load, i.e. under CPU contention, which is exactly
the condition that moves the tiering transition into the measured window.

**The honest read on attribution: inconclusive, and I am not claiming innocence on one event.**
1-of-6 versus 0-of-6 is a single failure and nowhere near enough to attribute. My two new allocation
tests do add 40 000 measured iterations of parallel load to that assembly, so it is entirely
plausible they raise the *rate* of a pre-existing flake without being its cause. What can be said
firmly is that the failing assertion is in someone else's test, about someone else's method, with a
failure magnitude that cannot be a real allocation.

**The part a human should know:** the allocation harness was reportedly fixed on `main` at
`76878bf`, which is the commit this worktree is branched from — and this fractional-byte failure
still occurred at that commit, roughly 1 run in 6, under full-solution parallel load. **The fix
reduced the flake but has not eliminated it under contention.** That is a live gate-reliability
issue, it belongs to `TestSupport/AllocationAssert.cs` rather than to this candidate, and I did not
touch it: weakening or loosening an allocation gate to make a run look clean is exactly the thing
that manufactures confidence (invariant 10). Carried to "Left undone".

All three gates, on a clean tree **before** any edit:

```
Passed!  - Failed: 0, Passed: 28,  Total: 28  - Teleop.RobotArm.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 458, Total: 458 - Teleop.Core.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 3,   Total: 3   - Teleop.Eval.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 36,  Total: 36  - Teleop.RobotHost.Tests.dll (net8.0)
verify: PASS -- .../testdata/golden/basic-session.tlog replays byte-identical across two
                independent passes, and matches the original file exactly.
audit: PASS -- no invariant violations found in .../core/Teleop.Core or
               .../build/Teleop.Core/bin/Debug/netstandard2.1/Teleop.Core.dll.
```

All three gates, **after** the change (`dotnet test Teleop.sln`, then `verify`, then `audit`):

```
Passed!  - Failed: 0, Passed: 28,  Total: 28  - Teleop.RobotArm.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 489, Total: 489 - Teleop.Core.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 3,   Total: 3   - Teleop.Eval.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 36,  Total: 36  - Teleop.RobotHost.Tests.dll (net8.0)

verify: PASS -- /home/andrei/Projects/teleoperation/.claude/worktrees/agent-a03ebfc90f49aa972/build/Teleop.Eval/bin/Debug/net8.0/testdata/golden/basic-session.tlog replays byte-identical across two independent passes, and matches the original file exactly.

audit: PASS -- no invariant violations found in /home/andrei/Projects/teleoperation/.claude/worktrees/agent-a03ebfc90f49aa972/core/Teleop.Core or /home/andrei/Projects/teleoperation/.claude/worktrees/agent-a03ebfc90f49aa972/build/Teleop.Core/bin/Debug/netstandard2.1/Teleop.Core.dll.
```

458 → 489 Core tests: +31, all in `TrackingLagReconcilerTests`. No pre-existing test changed,
weakened or was deleted. `audit` passing is the load-bearing one here: it includes the
registry-completeness check, which reflects for public non-abstract `IReconciler<>` implementers and
fails if one is not textually referenced in `Registries.cs`, so it confirms the new entry is real.

Two intermediate failures occurred and both were fixed by changing **my** code, never a threshold:

1. `AgainstMovingTruthTheDisplayedPoseSettlesAtTheClosedFormBiasAndStaysThere` failed on
   `IsConverged` reading true while the display lagged 8.5 mm. **Implementation was wrong**; the
   convergence flag is now display-based. Written up in Design decisions.
2. `OrientationFollowsTheSameExponentialAlongTheGeodesic` failed at 0.0039 vs 0.0040 rad. **The
   reconciler was right and the measuring function was imprecise** —
   `PoseMath.OrientationErrorRadians`'s `2*acos` loses half the mantissa near zero. The test now
   measures with the better-conditioned `atan2` path for the closed-form comparison and keeps
   `PoseMath` for the tolerance assertion. `PoseMath` itself was **not** modified.

Exact working-tree state (`git status --short`), confirming file scope was respected:

```
 M core/Teleop.Core/Registry/Registries.cs
?? core/Teleop.Core.Tests/Reconciliation/TrackingLagReconcilerTests.cs
?? core/Teleop.Core/Reconciliation/TrackingLagReconciler.cs
?? core/Teleop.Core/Reconciliation/TrackingLagReconciler.cs.meta
?? docs/research-log/2026-09-08-exp-smooth-track.md
```

and the entire diff to the one shared file is a single added line:

```
+                ["exp-smooth-track"] = (config, metrics, clock) => new TrackingLagReconciler(config, metrics, clock),
```

Nothing is committed. No `git commit`, `push`, `tag`, `amend` or rebase was run.

### The smoke sweep I did not run, and why

The brief permits a single-candidate smoke sweep to confirm metric cadence. I did not run one.
`SweepCommand` requires an `experiments/*.yaml` path and writes a run directory under `results/`,
and **both** of those are on my explicitly-forbidden file list ("no exceptions"). The narrower
prohibition wins over the narrower permission, so I stopped rather than guess.

The cadence claims the sweep would have confirmed are instead pinned by unit tests, which is the
stronger check anyway because it asserts an exact count rather than eyeballing a CSV:
`JerkIsEmittedOnEveryAdvancingFrameOnceFourPositionsExist` (23 frames driven, exactly 20
emissions — same cadence and same shared estimator as the rest of the axis),
`ObserveEmitsCorrectionMagnitudeOnceAtTheSamplesCaptureTick` (one sample per qualifying observation,
at the sample's own capture tick), `ObserveIgnoresDuplicateAndStaleSamples`, and
`NoConvergenceIsReportedWhileTheOperatorKeepsMoving`. The one thing a smoke sweep would have added
is the *observed* `time_to_convergence_ms` sample count on a real trace, which is the number the
survivorship warning is about. **That is the first thing the organizer should measure.**

## Left undone / for a human

1. **The admissibility decision on requirement 1.** My argued position is *inadmissible on the axis
   as written* — see Prediction 4 above. I did not amend requirement 1 or 2 and did not touch
   `Reconciliation/CLAUDE.md`, per the brief. Whichever way it goes, the row belongs in that file:
   in "Implemented" if the velocity-proportional bound is accepted, in "Tried and rejected" with a
   pointer to this log if it is not. **Every "Tried and rejected" entry the axis has is worth more
   than a marginal win, and this candidate is a good one either way — it closes the no-offset
   reading of `exp-smooth` with a closed form and a test rather than leaving it to be re-proposed.**
2. **The metric gap is the real finding, and it is bigger than this candidate.** The Reconciliation
   axis has **no metric that measures the reconciled/displayed pose against ground truth.** Every
   accuracy column in a sweep is measured upstream of the reconciler. That means the axis currently
   cannot detect *any* reconciler that trades displayed accuracy for smoothness — this candidate is
   simply the first one to make the gap unmissable. Closing it needs a new metric defined in
   `docs/metrics.md` (something like a displayed-pose position/orientation error against plant
   truth, emitted on the same every-advancing-frame cadence as `jerk_mm_s3`) and emitted from
   `SweepCommand`. Both files are outside my scope. **Until that exists, no head-to-head on this
   axis can distinguish "smooth" from "smooth and wrong".**
3. **`time_to_convergence_ms` pooling.** Either exclude this reconciler from pooled percentiles or
   report sample counts alongside them. A human should decide which, and record it in
   `docs/metrics.md` — I did not change the definition and should not.
4. **`PoseMath.OrientationErrorRadians` conditioning.** `2*acos(|dot|)` loses roughly half the float
   mantissa near zero (reads 3.9 mrad for a true 4.0 mrad). Harmless at the 17 mrad tolerance the
   sweep uses and it changes no recorded result, but it caps how precisely any orientation
   convergence claim on this axis can be asserted. A `2*atan2(|xyz|, |w|)` formulation would be
   well conditioned everywhere. **Not changed**: it is a shared error function, out of my scope, and
   changing it mid-panel would perturb every candidate's tests at once.
5. **The rate-cap semantics difference.** Here the caps bound total displayed speed rather than
   correction speed, because this law has no separable correction. Not binding at the current
   operating point (5 m/s cap vs 0.54 m/s peak operator speed), but a future experiment that
   tightens the cap would be comparing different quantities across the axis without saying so.
6. **`AllocationAssert` still flakes under solution-wide parallel load**, at `76878bf`, roughly 1
   run in 6, with a fractional bytes-per-call figure that cannot be a real allocation
   (`MotionMathTests.ClampMagnitude_Allocates_Zero_Bytes`, 0.287 bytes/call). Measurements and the
   honest attribution caveat are in Verification. This is a gate-reliability issue in
   `core/Teleop.Core.Tests/TestSupport/AllocationAssert.cs`, not in any reconciler. **Not touched**
   — loosening an allocation gate to make a run look clean is invariant 10's exact failure mode.
   Worth a human deciding whether the warmup needs to be contention-aware or the measurement
   pinned to a single thread.
7. **Nothing is blocked on hardware, Unity, or a human review gate.** No `unity/` or `robot/` file
   was read or written, and no hardware command was run.
