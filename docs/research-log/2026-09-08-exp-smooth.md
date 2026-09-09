# 2026-09-08 — `exp-smooth` / `ExponentialSmoothingReconciler`

**Agent:** candidate A of a three-candidate panel on the Reconciliation axis. **No comparison
sweep was run and no ranking against any other reconciler is claimed here.** The head-to-head is
the organizer's step. This log records the hypothesis, the design, what each test proves, the
closed-form size of the C1 violation, gate output, and an honest account of how the result could
flatter itself.

**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-a9299a0a7fbef4237`
**Branch:** `worktree-agent-a9299a0a7fbef4237`
**HEAD at start:** `76878bf87b06896d35e026e336b78ccfc2f8c06e`

## Seed self-check

| Check | Result |
|---|---|
| `git rev-parse HEAD` matches the expected sha | **PASS** — `76878bf87b06896d35e026e336b78ccfc2f8c06e` |
| Root `CLAUDE.md` read in full | **PASS** |
| `core/Teleop.Core/Reconciliation/CLAUDE.md` read in full, incl. "Tried and rejected" | **PASS** |
| `docs/research-log/CLAUDE.md` read | **PASS** |
| `SnapReconcilerTests.cs` witness-test pattern read | **PASS** — `Reconcile_OnASnap_ProducesAJerkFarBeyondAnythingC1Continuous` |
| `docs/research-log/2026-09-08-budget-blend.md`, `-velocity-match.md` caveats read | **PASS** — both warn the ~1.3 m opening transient dominates `lan` p95/p99, and `velocity-match` warns p95 of `jerk_mm_s3` ranks by correction *duration*, not severity |
| `dotnet test` green before any edit | **PASS** — 458 Core + 28 RobotArm + 3 Eval + 36 RobotHost |
| `Teleop.Eval -- verify` green before any edit | **PASS** |
| `Teleop.Eval -- audit` green before any edit | **PASS** |
| File scope understood: only `ExponentialSmoothingReconciler.cs` (+`.meta`), its test file, one line of `Registries.cs`, this log | **PASS** |

## Hypothesis (written before any code)

`exp-smooth` is the literal single-pole lag: one time constant, no second-order state. It carries
the visible disagreement as a residual offset seeded at correction onset from
`lastDisplayed - predicted` (as `spring` does, which is what makes *position* continuous), and
decays that offset by `offset *= exp(-dt/tau)`.

`tau` is derived from `ReconcilerConfig.MaxTimeToConvergenceTicks` against the **same 1% envelope
fraction `spring` uses**, so that "converged within the budget" means the same thing for both:

```
|o(t)| = |o0| e^(-t/tau) ;  |o(T)| = 0.01 |o0|  =>  tau = T / ln(100) = T / 4.6051702
```

Unlike `spring`'s `CriticalSettleConstant` (a Lambert-W root with no closed form, hence a
precomputed literal), mine *has* a closed form, so the defining-equation test can be exact rather
than a 5-decimal round trip.

### What I claim, before measuring

**The claim is not that this is smoother.** It is that the exception it needs is *finite, analytic,
and scales in a stateable way* — which is the quantity a human needs to decide whether requirement
2 should be amended. Specifically, derived on paper before implementation:

A single-pole lag's offset velocity is `o'(t) = -o(t)/tau`. Before onset the offset is identically
zero, so `o'(t0-) = 0`; immediately after seeding, `o'(t0+) = -o0/tau`. Hence

| quantity | closed form | in terms of the config |
|---|---|---|
| **onset offset-velocity step** | `|Δv| = |o0| / tau` | `= |o0| ln(100) / T = 4.6052 |o0| / T` |
| — scaling with correction magnitude | **linear** in `|o0|` | doubling the correction doubles the step |
| — scaling with the convergence budget | **inverse** in `T` | halving the budget doubles the step |
| — scaling with frame interval `dt` | **O(1)** — independent of `dt` | this is what makes it *finite*, unlike a snap |
| **retarget velocity step** (second sample mid-decay) | `|Δv| = |o_new - o_old| / tau` | first-order state cannot carry a velocity across a re-seed |
| **steady-state (post-onset) jerk** | `|o0| / tau^3 = 97.66 |o0| / T^3` | the smooth part of the response |
| **onset jerk spike, sampled** | `≈ 2|o0| / (tau dt^2)` | **O(1/dt²)** — diverges under frame-rate refinement |

The two numbers that matter to the panel are the first row and the last row, and they say different
things. The velocity step is **O(1) in `dt`** — a snap's is `O(1/dt)`. But `jerk_mm_s3`, the metric
this axis is actually compared on, still **diverges as `O(1/dt²)`** — a snap's diverges as
`O(1/dt³)`. So the exception is one order milder than the one already granted, in both instruments,
and it is not "small".

### Falsifiers — stated up front, any one kills the characterisation

1. **The measured onset offset-velocity step does not converge to `|o0|/tau` as `dt` is refined.**
   If it shrinks (like a C1 response) my whole framing is wrong and the row should just be built;
   if it diverges, the exception is worse than snap's and the row should be closed.
2. **The step is not linear in `|o0|` or not inverse in `T`.** Then the closed-form scaling I am
   handing the panel is fiction and the evidence is worthless.
3. **Bounded convergence (requirement 1) fails**: the envelope is not at 1% of `|o0|` at the
   budget, or a rate cap makes it never converge rather than converge later.
4. **Emission cadence is not sample-for-sample the established cadence** (`jerk_mm_s3` on every
   advancing frame via the shared `DisplayedJerkEstimator`; `correction_magnitude_*` from `Observe`
   stamped at capture; `time_to_convergence_ms` once per episode). Then the candidate is
   inadmissible for a head-to-head no matter how it scores, because pooled percentiles would be
   comparing populations rather than algorithms.
5. **It allocates on the per-frame path, or two instances given identical input diverge.**

### What I deliberately do *not* claim

I am not predicting whether `exp-smooth` wins or loses a sweep, and this log contains no
cross-reconciler numbers. The one comparison I do make is against `snap`, and only in closed form
and only in orders of `dt` — because `snap` is the axis's *baseline* and the holder of the one
already-granted exception to requirement 2, so the shape of its violation is the reference a human
weighing a second exception has to have. No `results/` numbers from any other reconciler appear
here.

---

## Design decisions and why

### One pole, no velocity state — kept literal on purpose

The row exists to answer "is the simple thing good enough". Adding any second-order state to smooth
the onset would make it a different algorithm and would answer a different question, so:

- offset state is **two `Vector3`s** (position offset, rotation-vector offset) and nothing else;
- there is no offset-velocity field, therefore nothing to carry across a re-seed, therefore the
  retarget step in the table above is inherent and is measured rather than hidden.

### Seeding: identical to `spring`, including the one-frame hold

`offset = lastDisplayed - predicted` in position, and
`MotionMath.RelativeRotationVector(predicted.Rotation, lastDisplayed.Rotation)` in orientation.
This inherits the axis-wide **one-frame display hold** at onset and at every retarget, which is a
known, deliberate, measured tradeoff (`Reconciliation/CLAUDE.md`, "Tried and rejected"; the
`offset -= error` fix was swept and was worse, collapsing the spread between reconcilers to 1.01x
on impaired profiles). I inherit it deliberately and pin it with a `..._ADeliberateTradeoff` test,
so this candidate sits on the same footing as the rest of the axis. **It is not a discovery and I
am not fixing it.**

### Rate caps without a velocity state

`spring` clamps its offset *velocity* after each step. With no velocity state the equivalent is to
clamp the per-step offset *displacement*:

```
delta        = offset * (1 - exp(-dt/tau))        // what the pole wants to remove this step
clampedDelta = MotionMath.ClampMagnitude(delta, maxSpeed * dt)
offset       = offset - clampedDelta
```

Reusing the shared `ClampMagnitude` rather than reimplementing it. Three properties this buys:

- **Direction is preserved**, so the rotation-vector offset shrinks along a fixed axis — the
  geodesic path to zero rotation, same as `spring`.
- **The clamp is continuous in the rate**: at the magnitude where the cap stops binding, capped and
  uncapped speeds are equal, so switching regimes injects no velocity step of its own. The cap can
  only *stretch* convergence, never break C0.
- **Bounded convergence survives the cap** with a stateable bound: while capped the offset shrinks
  strictly at `maxSpeed`, which lasts at most `|o0|/maxSpeed`; from whatever remains, the uncapped
  exponential reaches 1% within `T`. So `t_converge <= |o0|/v_max + T` unconditionally.

### Residual is never truncated to zero

Same reasoning `spring` documents: zeroing the residual once it falls inside tolerance is a
position discontinuity of up to one tolerance, i.e. a velocity step of `tolerance/dt` that is
*unbounded* as `dt` shrinks. The residual keeps decaying geometrically instead. `IsConverged`
therefore ends a *reporting* episode, not the motion.

### Config fields read

All but `RollbackHistoryCapacity`: both tolerances (to decide whether an arriving sample is worth
correcting, and when the correction has landed), `MaxTimeToConvergenceTicks` (sets `tau`), and both
rate caps. No new config fields, as required.

### Requirement 5 (predictor uncertainty)

Reached by not depending on it, exactly as `spring` does: `Observe` discards `diagnostics`, so
behaviour is identical with `PredictorDiagnostics.None` and with a covariance-bearing one. A test
pins that.

---

### Prior work consulted (sources, not evidence)

Searched before finalising the design, to avoid reinventing and to check whether anyone states the
C1 cost of a single-pole lag explicitly.

- **Unreal Engine ships exactly this algorithm as a named mode.**
  `ENetworkSmoothingMode` has `Disabled`, `Linear` and `Exponential`; the documented behaviour of
  `Exponential` is "moves faster as you are further from target", which is the single-pole law on a
  residual offset — the same construction as this class, and `Linear` is the constant-rate
  alternative (which is what my rate cap degenerates to while it binds).
  <https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Engine/ENetworkSmoothingMode>
  **What it claims:** nothing quantitative. **Under what conditions measured:** none stated — it is
  an API reference, not a measurement, and there is no operating point, no frame rate, no correction
  magnitude and no jerk figure attached. So it establishes that the technique is standard and
  shipping, and it establishes nothing about this system. No number from it enters `results/`.
- **The game-networking discussion of correction smoothing** consistently frames the choice as
  snap-then-visually-smooth versus interpolate-the-error, with an epsilon below which no correction
  is applied — which is the same structure as `Observe`'s tolerance gate here.
  <https://www.gamedev.net/forums/topic/658931-smoothing-corrections-to-client-side-prediction/>
  Again: no operating point, no continuity analysis.
- **Null result worth recording so the next run does not repeat it:** I found *no* source that
  states the C1 cost of the single-pole correction law, gives its onset velocity step in closed
  form, or reports its scaling with correction magnitude or convergence budget. The technique is
  ubiquitous and the discontinuity appears to be simply tolerated and undiscussed. That is the gap
  this candidate's test suite fills, and it is a reason to derive it here rather than look harder.

---

## Results

### What was built

| Artifact | Path |
|---|---|
| Implementation | `core/Teleop.Core/Reconciliation/ExponentialSmoothingReconciler.cs` |
| Unity meta (fresh guid `716003ac036740388377745cf80ef5f7`, checked for collision) | `...ExponentialSmoothingReconciler.cs.meta` |
| Tests, 35 of them | `core/Teleop.Core.Tests/Reconciliation/ExponentialSmoothingReconcilerTests.cs` |
| Registry, exactly one line | `core/Teleop.Core/Registry/Registries.cs` — `["exp-smooth"]` |

At the axis's default operating point (100 ms budget, 100 Hz, 1% envelope): **`tau` = 21.71 ms**.

### The C1 violation, measured (this is the deliverable)

Every row below is asserted by a named test, against a closed form rather than a magic number.

**Provenance of the decimal values:** the *forms* and the *bounds* are asserted by the named tests
(which check the implementation against them to 0.1% relative, or within stated ranges for the
refinement ratios). The decimals shown are those closed forms evaluated at the stated operating
point with the same `float` time constant the implementation stores. **They are not from a
`results/` manifest and must not be treated as citable numbers** — nothing in this table came from
a sweep.

| quantity | closed form | value at the default operating point | test |
|---|---|---|---|
| onset offset-velocity step, sampled | `\|o0\|(1 - e^(-dt/tau))/dt` | **1.845 m/s** for a 5 cm correction | `AtCorrectionOnset_TheOffsetVelocityStepsByItsClosedForm_AWitnessNotAClaim` |
| onset offset-velocity step, continuous | `\|o0\| / tau = \|o0\| ln(100)/T` | **2.303 m/s** for a 5 cm correction | same |
| order in `dt` | **`O(1)`** — converges upward to `\|o0\|/tau`, does not vanish | at a slow 1 s budget and 5 cm (the regime where `dt << tau` is reachable at 1 ms tick resolution): 0.2261 -> 0.2282 -> 0.2292 m/s under two halvings of `dt`, ratios **1.0092, 1.0046** against 0.5 for a C1 law, limit 0.2303 | `TheOnsetVelocityStepDoesNotShrinkAsTheFrameIntervalIsRefined` |
| scaling with correction magnitude | **linear**, `d(step)/d\|o0\| = 1/tau` | 2.000x at 2x, 10.000x at 10x | `TheOnsetVelocityStepScalesLinearlyWithCorrectionMagnitude` |
| scaling with convergence budget | **inverse**, `step ∝ ln(100)/T` | halving `T` doubles the step (measured 1.991x, within 0.5% of 2) | `TheOnsetVelocityStepScalesInverselyWithTheConvergenceBudget` |
| **saturation** | `min(\|o0\|/tau, v_max)` | clipped to **5 m/s** above `\|o0\| = v_max·tau = 10.9 cm` | `TheRateCapBoundsTheOnsetVelocityStep` |
| retarget velocity step | `\|o_new - o_old\| / tau`, reducing to `\|Δpredicted\|/tau` | matches to within 5% | `ARetargetMidDecayStepsTheOffsetVelocityByItsClosedForm` |
| onset jerk, frame after seed | `c(1-d)/dt³` | **1.845e7 mm/s³** | `TheOnsetJerkSpikeMatchesItsClosedFormAndDwarfsTheSmoothPartOfTheSameResponse` |
| onset jerk, peak (second frame) | `c(1-d)(2-d)/dt³` | **2.526e7 mm/s³** | same |
| smooth part of the same response | `c(1-d)³d^k/dt³` | **2.513e6 mm/s³** | same |
| **peak / smooth ratio** — the stated threshold | `(2-d)/(1-d)²` | **10.05x** | same |
| order of the jerk spike in `dt` | **`O(1/dt²)`** | 3.97x and 3.98x per halving of `dt` | `TheOnsetJerkSpikeDivergesQuadraticallyAsTheFrameIntervalIsRefined` |

with `d = e^(-dt/tau) = 0.01^(dt/T)`, `c = |o0|`, `T` the convergence budget.

**The one-sentence answer to the panel's question.** The exception `exp-smooth` needs is *finite in
velocity* (`O(1)` in `dt`, unlike a snap's `O(1/dt)`) but still *unbounded in jerk* (`O(1/dt²)`,
one order milder than a snap's `O(1/dt³)`), and its coefficient is
`min(|o0| ln(100)/T, v_max)` — **linear in correction magnitude, inverse in the convergence budget,
and clipped by the rate cap.**

### The result that changed the shape of the tradeoff, and how it was found

I derived the linear-in-magnitude law on paper and would have reported it unqualified. Running the
permitted single-candidate smoke sweep showed the largest observed jerk was ~1.3e8 mm/s³, not the
~1e9 the uncapped law predicts for the known ~1.3 m opening transient. Chasing that gap found the
qualification: **the rate cap bounds the onset step at `min(|o0|/tau, v_max)`**, because the first
advancing frame removes `offset·(1 - e^(-dt/tau))` clamped to `v_max·dt`.

That is not a detail. It changes the answer:

- Above `|o0| = v_max·tau` — about **10.9 cm** at the numbered experiments' operating point
  (`v_max = 5 m/s`, `T = 100 ms`) — the violation **stops scaling with correction magnitude** and
  becomes a configured constant.
- Every correction larger than 10.9 cm therefore has an identical onset step of 5 m/s. The scary
  60 m/s figure for the 1.3 m startup transient **never occurs at any operating point this repo
  actually runs**.
- The *order* is unchanged: a 5 m/s step across one frame is still `O(1/dt²)` in jerk. Only the
  coefficient is bounded.
- The cap buys this by taking longer to converge, and `time_to_convergence_ms` is a reported
  metric. So the bound is not free; it is the same trade the axis exists to measure, relocated.

I added `TheRateCapBoundsTheOnsetVelocityStep` and corrected the implementation's own type doc,
which had stated the linear law without the qualification.

### Requirement-by-requirement standing

| Requirement | Standing | Evidence |
|---|---|---|
| 1. Bounded convergence, provable, stated bound | **Met.** `\|o(t)\| = \|o0\|e^(-t/tau)`, at 1% of `\|o0\|` when the budget elapses. With a binding rate cap the bound is `\|o0\|/v_max + T`, proved and asserted | `AConstantCorrectionConvergesWithinTheBudget`, `TheResidualFollowsTheSinglePoleEnvelopeExactly`, `ARateCapStretchesConvergenceWithinAStatedBound` |
| 2. C1 continuity | **NOT met, deliberately, and quantified above.** C0 is met (position continuous across onset) | the whole of section 5 of the suite |
| 3. Correction cost every step via `IMetricSink`, same names, same cadence | **Met.** Emission is sample-for-sample identical to the baseline on a driven scenario | `CorrectionMagnitudeAndEmissionCadenceMatchTheBaselineExactly`, `JerkIsEmittedOnEveryAdvancingFrameOnceTheHistoryIsFull` |
| 4. Deterministic, allocation-free | **Met** | `TwoInstancesGivenTheSameInputProduceIdenticalOutput`, three `Allocates_Zero_Bytes` tests |
| 5. Degrade gracefully with no predictor uncertainty | **Met**, by not depending on it at all | `BehavesIdenticallyWithAndWithoutPredictorUncertainty` |

### Falsifier status

| Falsifier | Outcome |
|---|---|
| 1. Onset step does not converge to `\|o0\|/tau` under refinement | **Not triggered.** It converges upward to within 2% of the limit |
| 2. Step not linear in `\|o0\|` / not inverse in `T` | **Partially triggered — and this is the finding.** Linear and inverse both confirmed *uncapped*; the rate cap saturates the magnitude law above `v_max·tau`. The characterisation handed to the panel is the corrected one |
| 3. Bounded convergence fails | **Not triggered** |
| 4. Emission cadence differs | **Not triggered.** Identical counts of all four metrics against the baseline |
| 5. Allocates or non-deterministic | **Not triggered** |

### Smoke sweep — not a result, and deliberately single-candidate

`smoke-exp-smooth`, 2 seeds x `double-exp` x {`lan`, `300ms-60j-2loss-bursty`}, 300 steps, written to
`results/smoke-exp-smooth/20260909-005026Z/` **in this worktree**, which is gitignored and will be
deleted with it. It exists only to confirm the candidate resolves in the registry, runs end-to-end
and emits at the right cadence. **It is not evidence and no comparison was run.**

Cadence confirmed exactly as predicted: 594 `jerk_mm_s3` samples per profile = 600 frames minus 3
per trial x 2 trials. 151 corrections on the bursty profile against 30 on `lan`, and 51 convergence
episodes against 2 — a 5x difference in how often the retarget discontinuity fires, which is the
covariate any later head-to-head has to hold.

No percentiles from this run should be quoted. `lan`'s 30 corrections include the ~1.3 m opening
transient, which prior logs (`2026-09-08-budget-blend.md`, `-velocity-match.md`) establish dominates
p95/p99 there; and `-velocity-match.md` establishes that jerk p95 below p99 ranks by correction
*duration* rather than severity.

---

## Verification

All three gates, run from the worktree after the final edit. Baseline was green before any edit
(458 Core tests, `verify: PASS`, `audit: PASS`), so the delta is attributable.

```
$ dotnet test core/Teleop.sln
Passed!  - Failed: 0, Passed:  28, Skipped: 0, Total:  28 - Teleop.RobotArm.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed: 493, Skipped: 0, Total: 493 - Teleop.Core.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed:   3, Skipped: 0, Total:   3 - Teleop.Eval.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed:  36, Skipped: 0, Total:  36 - Teleop.RobotHost.Tests.dll (net8.0)

$ dotnet run --project core/Teleop.Eval -- verify
verify: PASS -- .../testdata/golden/basic-session.tlog replays byte-identical across two
independent passes, and matches the original file exactly.

$ dotnet run --project core/Teleop.Eval -- audit
audit: PASS -- no invariant violations found in .../core/Teleop.Core or
.../build/Teleop.Core/bin/Debug/netstandard2.1/Teleop.Core.dll.
EXIT=0
```

458 -> 493 Core tests is the 35 added here; no existing test changed.

### A pre-existing flake observed while re-running the gates — not mine, but recorded

On one of five solution-wide `dotnet test` runs, **`Teleop.Core.Tests.Transport.RawPoseCodecTests.
TryDecode_Allocates_Zero_Bytes`** failed. Characterised rather than dismissed, because "it was
flaky" is how a real regression gets waved through later:

- It is in `Transport/`, over `RawPoseCodec` — a namespace and a type **this change does not touch
  and does not sit downstream of**. My diff is `Reconciliation/`, one line of `Registries.cs` (a
  dictionary entry, which cannot alter codec behaviour), and docs.
- **6/6 passes** when the `RawPoseCodecTests` class is run alone.
- **4/5 passes** solution-wide, where four test host processes run concurrently.

That signature — an allocation assertion failing only under concurrent test hosts — points at GC
pressure from the other hosts perturbing `GC.GetAllocatedBytesForCurrentThread()`, i.e. the harness,
not the codec. **I did not touch it, and I did not weaken it.** Flagged for a human below; the
correct fix is in the allocation harness or the test's isolation, not in the tolerance.

### One test failed on the way, and it was mine being wrong

`SettleConstantMatchesItsDefiningEquation` originally asserted the round trip `e^(-T/tau) = 0.01` to
9 decimal places. It produced `0.010000001071306865`. `tau` is a `float` field, so 1.07e-9 absolute
is ~1.1e-7 relative — **one float ulp, i.e. the type's precision, not a defect.** I replaced the
absolute assertion with a relative bound of 2e-7 and documented why. Recording it because the
tempting move here is to loosen the number until it passes without working out what the number
means, and that is how a gate stops being a gate.

---

## How this result could flatter itself

Read this section before quoting anything above.

1. **The closed forms describe the offset in isolation, on a stationary predictor.** All of section
   5 drives a synthetic step against a predictor that is at rest before the correction and constant
   after it. In a real trace the offset is superposed on genuine operator motion, and the emitted
   `jerk_mm_s3` contains the predictor's own jerk as well. **My onset-jerk closed forms are a lower
   bound on what a trace will produce, not a prediction of it.**
2. **The scenario silently suppresses the inherited one-frame hold.** Holding a *stationary* display
   for one frame is a no-op, so the hold contributes nothing to my measured jerk figures. On a real
   trace the hold and the pole's onset step **superpose on the same frame** — both fire at onset and
   at every retarget, both are `O(1/dt²)` in jerk. This is the single largest way the numbers above
   under-report the violation, and I have not measured the superposition.
3. **The 1% envelope choice sets the exception's size.** `tau = T/ln(1/f)`, so the discontinuity is
   proportional to `ln(1/f)`. I chose 1% to match `spring`, which is right for comparability, but a
   5% envelope would make `tau` 1.6x larger and every number above 1.6x smaller, for no change in
   the algorithm. Anyone reading "2.3 m/s" as a property of the design rather than of the definition
   is being misled.
4. **The rate-cap finding flatters the candidate and must not be quoted alone.** `min(|o0|/tau,
   v_max)` makes the violation look bounded and tame. It is bounded *because convergence is
   stretched*, and `time_to_convergence_ms` moves the wrong way when the cap binds. Reporting the
   bounded step without the convergence cost is precisely the "prediction error without correction
   cost" error the operating contract warns about, in a different coordinate.
5. **The smoke sweep is the wrong instrument for anything and I have used it only as a smoke test.**
   Two seeds, two profiles, 300 steps, one candidate. Its `lan` percentiles are dominated by the
   startup transient and its sub-p99 jerk percentiles rank by correction duration.
6. **`prediction_position_error_mm` and `correction_magnitude_mm` cannot differentiate this
   candidate.** Both are emitted before any reconciler acts and are identical across the axis by
   construction. Nothing in the sweep measures *displayed-pose* accuracy. If a later comparison
   appears to show `exp-smooth` winning or losing on either, that is a bug in the comparison.

---

## My argument on requirement 2 — this is an argument, not a decision

I am not permitted to amend requirement 2 or to edit the axis `CLAUDE.md`, and I have not. The
following is the case as I see it, for a human to weigh.

**The framing in the planned-table row is, I think, slightly wrong.** It says building `exp-smooth`
"needs requirement 2 amended to cover it as a second quantified exception, the way `snap` is". Two
things complicate that:

**First, requirement 2 as literally written is already violated by every reconciler on this axis**,
not just `snap`. The one-frame display hold at onset and retarget is an `O(1)` velocity dropout,
documented as such, kept deliberately on measurement, and pinned by `..._ADeliberateTradeoff` tests.
So the axis does not currently enforce "no velocity discontinuities"; it enforces "no velocity
discontinuity *other than* the shared seeding artifact and `snap`". The real question is therefore
not "C1 or not" but **which violations, at what order, with what coefficient, on what events.**

**Second, `exp-smooth` is not a `snap`-like exception.** On the three axes that distinguish them:

| | order in `dt` (velocity / jerk) | coefficient | events it fires on |
|---|---|---|---|
| the shared one-frame hold | `O(1)` / `O(1/dt²)` | the operator's own speed — bounded, small, independent of the link | every onset and retarget |
| **`exp-smooth`'s onset step** | **`O(1)` / `O(1/dt²)`** | `min(\|o0\| ln(100)/T, v_max)` | **every onset and retarget — the same events** |
| `snap` | `O(1/dt)` / `O(1/dt³)` | the entire correction | every correction |

`exp-smooth` sits in the **same class as the violation the axis already accepts**, at the same
order, on the same events. It differs in one thing: the coefficient is set by correction magnitude,
convergence budget and rate cap rather than by operator speed. Granting it "a second exception the
way `snap` is" therefore overstates it by a full order in both instruments.

**The case for amending.** If requirement 2 were restated as what the axis actually enforces — *no
`O(1/dt)` velocity discontinuity; `O(1)` steps admitted with their coefficient stated and tested* —
then `spring`, `budget-blend` and `velocity-match` pass with coefficient = operator speed,
`exp-smooth` passes with coefficient `min(|o0|/tau, v_max)`, and `snap` remains the sole excluded
case. That restatement describes the codebase as it is, makes the existing `..._ADeliberateTradeoff`
tests part of the requirement instead of exceptions to it, and needs no per-implementation carve-out.

**The case against.** The coefficient scaling is genuinely bad in the regime that matters. `|o0|/tau`
is largest exactly on the largest corrections — the nausea-relevant ones — and the retarget step
fires ~5x more often on impaired profiles than on `lan` (151 vs 30 corrections in my smoke run), so
the violation's *rate* is highest exactly where the link is worst. A requirement that admits a
violation whose size grows with the disturbance it exists to protect against is a weak requirement.
The rate cap blunts this, but by spending convergence time — and the cap is config, so a future
experiment that raises `v_max` silently un-blunts it with no test failing anywhere.

**What I would want measured before deciding**, and cannot measure myself without running the
comparison this candidate is barred from: `jerk_mm_s3` p99 together with `time_to_convergence_ms`,
at equal convergence budget, with correction *rate* reported as a covariate — because if the
coefficient difference does not show up above the shared hold's contribution, then the whole
distinction is analytic rather than operational and requirement 2 should simply be restated as
above. If it does show up, `exp-smooth`'s simplicity is not worth it and the row should be closed.
**That is the organizer's step, not mine.**

---

## Left undone / for a human

1. **The axis `CLAUDE.md` still lists `exp-smooth` under "Planned, not yet implemented".** I am
   forbidden from editing it, so a human must move the row to "Implemented" (or into "Tried and
   rejected" if the head-to-head closes it). Note that **`audit` does not catch this direction of
   drift** — its registry-completeness check catches a row claiming an implementation that does not
   exist, but not an implementation with no row, so `audit` passes today with the table understating
   reality. That asymmetry may itself be worth a look.
2. **Requirement 2's wording** — argued above, decision deliberately left to a human.
3. **The superposition of the inherited hold with the pole's onset step is unmeasured** (self-
   flattery item 2). It needs a trace-driven measurement with a moving predictor, not a synthetic
   step. This is the most important gap in my evidence.
4. **The 1% envelope fraction is now duplicated** as a private const in `SpringReconciler`,
   `ExponentialSmoothingReconciler` and (I believe, unread) the other two. If it is a cross-axis
   definition — and the comparability argument says it is — it belongs in one place. I could not
   change it: `ReconcilerConfig.cs` and the other reconcilers are outside my file scope.
5. **`RawPoseCodecTests.TryDecode_Allocates_Zero_Bytes` is flaky under solution-wide runs**
   (1 failure in 5; 0 in 6 isolated runs) — see Verification. Pre-existing, unrelated to this
   change, and outside my file scope. Worth fixing in the allocation harness's isolation, since an
   allocation gate that fails 20% of the time under CI's own parallelism will eventually be
   loosened by someone in a hurry, which is invariant 10 in slow motion.
6. **`results/smoke-exp-smooth/` lives in a disposable worktree** and will vanish with it. Nothing
   citable depends on it; it was a smoke test.
7. **Nothing is committed.** The branch is left dirty, as instructed. Diff:
   `worktree-agent-a9299a0a7fbef4237` against `76878bf`.

