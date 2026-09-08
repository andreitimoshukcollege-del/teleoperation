# 2026-09-08 — `budget-blend` / `TimeBudgetedBlendReconciler`

Agent: parallel candidate builder (one of three). **No comparison sweep was run and no ranking
is claimed here** — a later head-to-head decides. This log records design, rationale, tests and
gate output only, plus the analytic predictions a judge should check against the measurement.

## Seed self-check

| Check | Result |
|---|---|
| `git log --oneline -1` | `16b45dc Add the deep-researcher agent for unattended runs on Core's open axes` — matches the expected sha |
| `core/Teleop.Core/Reconciliation/SpringReconciler.cs` exists | yes |
| `.claude/agents/deep-researcher.md` readable | yes, read in full |
| `just core-check` green before any edit | yes — 384 Core + 28 RobotArm + 3 Eval + 36 RobotHost tests passed, `verify: PASS`, `audit: PASS` |

Worktree: `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-acd879fefa6de0690`
Branch: `reconciliation-spring-and-sweep-stacks`

## Hypothesis (written before any code)

`spring`'s convergence bound is **relative and asymptotic**: it reaches 1% of the initial error at
`MaxTimeToConvergenceTicks` and never reaches zero. `budget-blend` replaces the exponential decay
with a **finite-duration quintic blend** whose residual offset is *identically* zero at a deadline
computed at correction-seed time, for any correction magnitude.

**What I expect to improve, and on which metric.** `jerk_mm_s3` (docs/metrics.md §5), at equal
configured `ConvergenceBudgetMs`. The analytic reason, derived before implementation:

| for a step offset `o0` absorbed over budget `T` from rest | `spring` (critically damped, `w = 6.6384/T`) | `budget-blend` (quintic, exact at `T`) | ratio |
|---|---|---|---|
| peak offset speed | `o0 w / e = 2.44 o0/T` | `(15/8) o0/T = 1.875 o0/T` | 0.77x |
| peak offset acceleration | `o0 w^2 = 44.1 o0/T^2` | `5.77 o0/T^2` | 0.13x |
| peak offset jerk | `2 o0 w^3 = 585 o0/T^3` | `60 o0/T^3` | 0.10x |

So on paper the quintic should be roughly an order of magnitude smoother at the same nominal
budget. **This is a prediction, not a measurement, and it is exactly the shape of "surprising win"
the operating contract warns about** — see "How this implementation could flatter itself" below
before believing it.

**Network profiles.** Expected to hold on all of them, because the effect is a property of the
convergence law, not of the arrival process. If the effect turns out to be profile-dependent, the
cause is the rate cap binding (see the cap analysis below), not the blend.

**Falsifiers — any one of these kills the idea:**

1. Measured `jerk_mm_s3` p95/p99 for `budget-blend` is **not** below `spring`'s at equal
   `ConvergenceBudgetMs` on the same trace and seed. That would mean the analytic advantage is
   swamped by the onset/retarget discontinuity that both share, i.e. the convergence law is not
   what sets measured jerk — itself a finding worth keeping.
2. `time_to_convergence_ms` is not bounded by the effective deadline in a test. A finite blend that
   misses its own deadline has no reason to exist next to `spring`.
3. Convergence is not exact: a test at several correction magnitudes fails to show the residual
   offset reaching *identically* zero at the deadline. Relative-and-asymptotic is `spring`'s job;
   if I can only match that, there is no second reconciler here.
4. The retarget path introduces a velocity step: the refined-`dt` C1 test (max per-frame velocity
   step must shrink as `dt` shrinks) fails when a retarget happens mid-blend.

## Design decisions and why

### The blend function: quintic Hermite with a structural triple root at the deadline

Offset as a function of normalised blend time `u = (t - t_seed)/T_eff`:

```
offset(u) = (1 - u)^3 * (c0 + c1 u + c2 u^2)
c0 = o0
c1 = 3 o0 + v0 T
c2 = 6 o0 + 3 v0 T
```

with `o0` the seed offset and `v0` the offset velocity carried in from an in-flight blend.

Three reasons for this exact form:

- **A linear ramp is not admissible.** `Reconciliation/CLAUDE.md` requirement 2 asks for C1, and a
  finite blend needs *zero derivative at both ends* or it puts a velocity step either at onset or
  at completion. A cubic smoothstep gives C1; the quintic gives C1 *and* C2 (zero acceleration at
  both ends), which is what buys the 0.10x peak-jerk figure above. That is why quintic, not cubic.
- **`v0 = 0` reduces it exactly to `o0 * (1 - smootherstep(u))`** — verified algebraically:
  `(1-u)^3 (1 + 3u + 6u^2) = 1 - 10u^3 + 15u^4 - 6u^5`. The retarget case is therefore the *same*
  function family as the isolated case, not a special case bolted on. A test pins the identity.
- **The `(1-u)^3` factorisation is load-bearing, not cosmetic.** The whole claim is exactness at
  the deadline. Evaluating the expanded quintic `b0 + b1 u + a3 u^3 + a4 u^4 + a5 u^5` at `u = 1`
  gives zero only to float rounding. Factored, `u == 1` makes `1 - u` exactly `0`, so the offset
  and its derivative are *bitwise* zero at the deadline and stay zero afterwards — which lets
  `Compose` short-circuit to returning `predicted` bit-identically, the same degenerate
  pass-through property `spring` documents.

### Retarget mid-blend: fresh full budget, carried velocity, reset acceleration

**Blend clock:** restarts, with a fresh *full* `T_eff` recomputed for the new offset.
**In-flight velocity:** carried into `c1`/`c2`, so the trajectory bends rather than restarting from
rest. The initial *acceleration* term is reset to zero (`b2 = 0`).

Why a fresh full budget rather than keeping the original deadline: keeping it would make the
guarantee "converged by the *first* correction's deadline", which sounds stronger and is actually
worse — a correction arriving 1 ms before that deadline would have to be absorbed in 1 ms, i.e. a
snap at unbounded speed, precisely on the frames a lossy link produces most often. Restarting makes
the guarantee **per correction**: the offset is identically zero `T_eff` after the most recent
correction. That is the same class of statement `spring` makes (its envelope restarts too), so the
two stay comparable, and it is the only version of the guarantee that does not degenerate into a
snap.

Why carry velocity but not acceleration: carrying velocity is what preserves C1 across the
retarget, and it is exactly what `SpringReconciler.SeedOffsetFromDisplayedPose` does ("the offset
velocity is deliberately left alone"). Carrying acceleration too would preserve C2 across the
retarget and is cleanly available here (the `b2` term exists in the derivation) — I deliberately
did **not** take it, because it would make `budget-blend` differ from `spring` in *two* ways
(convergence law **and** how much derivative state survives a retarget) and the head-to-head could
then not attribute the result to the convergence law. Carrying acceleration is a clean follow-up
experiment on top of this file, holding everything else fixed.

### Rate caps versus the hard deadline — resolution

They genuinely conflict: absorbing `o0` in `T` needs a peak speed of `1.875 o0 / T`, which a cap
can forbid.

**`spring` resolves it by clamping the offset velocity every step**, which stretches convergence.
That option is **not available to a finite blend**: clamping the derivative of a *scheduled*
trajectory desynchronises it from its own schedule, and the offset would then not be zero at the
deadline — destroying the one property this reconciler exists to provide.

**Resolution: the cap is enforced a priori, by sizing the deadline, not a posteriori by clamping.**
At seed time,

```
T_eff = max(configured budget,
            ceil(15/8 * |o0_pos| / maxLinearSpeed),
            ceil(15/8 * |o0_rot| / maxAngularSpeed))
```

one deadline shared by both channels, so `time_to_convergence_ms` refers to one instant, not two.

Which wins, stated plainly: **the rate cap wins over the configured budget** — the same precedence
`Teleop.Eval/Sweep/ExperimentConfig.cs` already documents for that field, and the same observable
behaviour `spring` has (cap binds ⇒ convergence takes longer than `ConvergenceBudgetMs`). What is
preserved unconditionally is not the *configured* budget but the **hard-deadline property itself**:
whatever `T_eff` turns out to be, it is finite, computed once at seed time, known in advance, and
the offset is identically zero at it. That, not "always 100 ms", is the contrast with `spring`'s
asymptotic 1% envelope.

Honest caveat, quantified: the sizing uses the offset magnitude only and ignores the carried
velocity, so peak correction speed is `<= cap` for an isolated correction from rest and
`<= cap + |v_carried|` across a retarget. Subtracting the carried speed from the cap instead would
make `T_eff` diverge as `|v_carried| -> cap`, trading a bounded cap excursion for an unbounded
convergence time — the wrong way round for a contract whose first clause is bounded convergence.

**Where this binds in practice** (from `ExperimentConfig` defaults: budget 100 ms, linear cap
5 m/s, angular cap 10 rad/s): the cap binds for positional corrections above
`5 * 0.1 / 1.875 = 26.7 cm` and angular corrections above `0.53 rad ~ 30 deg`. A typical
few-millimetre correction is nowhere near it. **The known ~1.3 m startup transient is far past it**:
`T_eff` becomes `1.875 * 1.3 / 5 = 487 ms`, not 100 ms. Anyone reading a `lan`-profile p95
dominated by that transient is reading the cap, not the blend.

### Config fields read and ignored

Reads: `ConvergencePositionToleranceMeters`, `ConvergenceOrientationToleranceRadians` (the gate in
`Observe`, and the frame the reporting episode closes), `MaxTimeToConvergenceTicks` (the budget
floor), `MaxCorrectionLinearSpeedMetersPerSecond`, `MaxCorrectionAngularSpeedRadPerSecond` (both
size the deadline). Ignores: `RollbackHistoryCapacity` — for `rollback`, and nothing here rewinds.
**No config field was added**, as instructed.

### Metric contract — deliberately identical to `spring`

Same four names, same definitions (`PoseMath` for the magnitudes, the shared
`DisplayedJerkEstimator` for jerk), and critically the **same cadence**: `jerk_mm_s3` on *every*
advancing frame once four displayed positions exist; `time_to_convergence_ms` once per episode, on
the frame the offset first falls inside tolerance, measured from onset. No second jerk cascade was
written. Unequal cadence would make the head-to-head compare populations rather than algorithms.

## What was built

| File | Change |
|---|---|
| `core/Teleop.Core/Reconciliation/TimeBudgetedBlendReconciler.cs` | new, 766 lines incl. docs |
| `core/Teleop.Core/Reconciliation/TimeBudgetedBlendReconciler.cs.meta` | new, fresh guid `f31a5c2eaef541ed897d02a93d2b7d5c` |
| `core/Teleop.Core.Tests/Reconciliation/TimeBudgetedBlendReconcilerTests.cs` | new, 38 tests |
| `core/Teleop.Core/Registry/Registries.cs` | **exactly one line** added to `Reconcilers`: `["budget-blend"]` |

Nothing else was touched. `git diff --stat` on tracked files is `1 file changed, 1 insertion(+)`;
everything else is untracked additions. `Reconciliation/CLAUDE.md`, `docs/metrics.md`,
`SnapReconciler.cs`, `SpringReconciler.cs`, `DisplayedJerkEstimator.cs`, `experiments/*.yaml` and
`core/Teleop.Eval/` were deliberately left alone as shared-with-other-agents surface — see
"Left for a human / for the merge" below for the two rows that *should* change and why I did not
change them.

## What each test proves

38 tests. The ones that carry weight, and what would break them:

| Test | Proves | Fails if |
|---|---|---|
| `TheResidualIsExactlyZeroAtTheDeadline_ForAnyCorrectionMagnitude` (7 rows: 1 µm to 10 m) | the distinguishing claim. Asserts **bit-exact pose equality** with the corrected prediction at the deadline — reachable only because the offset is bitwise `Vector3.Zero` and `Compose` short-circuits. Also asserts the display is *not* yet on truth one frame earlier, so the test cannot be satisfied by an early snap | the convergence is relative rather than absolute. `spring` cannot pass this at any magnitude |
| `ADeadlineFallingBetweenTwoFramesRetiresExactlyOnTheNextFrame` | the corner a finite blend has and an exponential does not — the polynomial is never evaluated past `u = 1`, where `(1-u)^3` changes sign and the offset would grow *away* from truth | the retire branch is removed or made `>` |
| `AnIsolatedBlendFollowsSmootherstepExactly` | the factored form equals `1 - smootherstep(u)` to 1e-7, i.e. the algebra `(1-u)^3(1+3u+6u^2) = 1 - 10u^3 + 15u^4 - 6u^5` holds in the implementation, against an independently written polynomial | a coefficient drifts |
| `MaxPerFrameVelocityStepShrinksAsTheFrameIntervalIsRefined` | C1 across **onset, mid-blend and completion** (the window is four budgets wide). Refines `dt` 8→4→2 ticks and requires the max per-frame velocity step to shrink by >25% each halving. Same method and same slow-budget rationale as the `spring` version, for the reason that file documents: a single absolute bound at one frame rate passes for a discontinuous signal too | any of the three boundaries has an O(1) velocity step |
| `ARetargetMidBlendCarriesTheOffsetVelocityIntoTheNewBlend` | the retarget policy. Measures offset speed after / before the retarget and requires the ratio → 1 as `dt` refines | the velocity is dropped at re-seed |
| `ARetargetMidBlendRestartsTheDeadlineAndStillLandsExactlyOnIt` | fresh full deadline, not the inherited remainder; still bit-exact at the new deadline; two corrections, one convergence episode | the clock is inherited or the episode accounting drifts from `spring`'s |
| `TheOneFrameHoldAtARetargetIsSharedWithSpring` | runs the *same* scenario through `SpringReconciler` and asserts both hold for exactly one frame. A guard on the head-to-head, not on this reconciler — see the finding below | either implementation stops sharing the artifact |
| `EmissionCadenceIsSampleForSampleIdenticalToSpring` | equal sample counts of all four metrics from the same driven scenario. This is what makes pooled percentiles legitimate | a metric is emitted on a different schedule |
| `CorrectionMagnitudeMatchesSnapAndSpringExactly` | all three reconcilers emit byte-identical correction magnitudes | a second error definition creeps in |
| `PeakSpeedFactorMatchesTheImplementedBlendPolynomial` | the `15/8` constant that converts a rate cap into a duration is the true peak of the implemented polynomial, measured from 1000 samples of a real run, not a remembered value | the polynomial changes without the constant |
| `ARateCapLengthensTheDeadlineRatherThanBeingExceeded` | 0.7 m at a 5 m/s cap ⇒ deadline 263 ticks (2.6× the budget); measured peak displayed speed ≤ cap; still bit-exact at the deadline; `time_to_convergence_ms` reports the lengthened deadline, not the configured budget | the cap is ignored, or clamped a posteriori and the schedule desynchronises |
| `TheAngularCapAlsoLengthensTheSharedDeadline` | 1.1 rad at 5 rad/s ⇒ 413 ticks, one shared deadline for both channels | the channels get separate clocks |
| `AGapInsideTheBlendIsFrameRateIndependent` | a run that drops 45 ms of frames displays *the same pose at the same tick* as one that does not — the blend is a function of absolute time, not an integrated state | the schedule becomes step-dependent |
| `AGapOfSeveralHundredMillisecondsLandsExactlyOnTruth`, `Reconcile_AtOrBeforeTheLastTick_IsANoOp`, `Observe_IgnoresStaleAndDuplicateSamplesWhole`, `Observe_DuplicateMidBlend_DoesNotRestartTheDeadline`, `TwoInstancesGivenTheSameInputProduceIdenticalOutput`, `Reset_RestoresTheAsConstructedState` | gaps, idempotence, out-of-order and duplicate `Observe` (including the mid-blend duplicate, which must not push the deadline back), determinism, and `Reset` returning a reused instance to exactly as-constructed | — |
| three `AllocationAssert.Zero` tests | `Observe` + `Reconcile` mid-correction, `Reconcile` while converged, `Observe` rejected as stale | any allocation on the hot path |

### Mutation testing — the tests are not vacuous

I broke the implementation three ways and confirmed the intended test caught each. This matters
because a test that asserts a property the code cannot violate is worse than no test.

| Mutation | Caught by |
|---|---|
| `c1 = c2 = 0` (pure `(1-u)^3` decay, nonzero velocity at onset) | `MaxPerFrameVelocityStepShrinksAsTheFrameIntervalIsRefined`, `ARetargetMidBlendCarriesTheOffsetVelocityIntoTheNewBlend`, `AnIsolatedBlendFollowsSmootherstepExactly`, `PeakSpeedFactorMatchesTheImplementedBlendPolynomial`, `ARateCapLengthensTheDeadlineRatherThanBeingExceeded` |
| carried velocity dropped at re-seed, quintic shape kept | **only** `ARetargetMidBlendCarriesTheOffsetVelocityIntoTheNewBlend`, ratio 0.0007 instead of ~1.0 — a clean, isolated falsifier |
| blend evaluated past its own deadline (retire at 4× duration) | all 7 exactness rows, plus `ADeadlineFallingBetweenTwoFramesRetiresExactlyOnTheNextFrame`, `AnOrientationOnlyCorrectionConvergesExactlyOnTheGeodesic`, `TheDisplayedTrajectoryNeverOvershootsTheTarget`, `ARateCapLengthensTheDeadlineRatherThanBeingExceeded` |

Implementation restored bit-identically after each (`diff` clean) before the final gate run.

## Three test expectations I had to correct (recording these so nobody re-derives them)

None of these were the implementation being wrong; all three were my test arithmetic. Written down
rather than silently fixed, because two of them are traps the next person will hit.

1. **The retarget hold is exact only to float rounding.** The held output is
   `predicted + (lastDisplayed - predicted)`, which loses about an ulp of the predicted magnitude —
   4e-9 m on a 3 mm displacement. Bit-equality is the wrong assertion there. It *is* the right
   assertion at the deadline, where the offset is bitwise zero and `Compose` adds nothing at all.
   The two cases genuinely differ and the tests now say so.
2. **`BlendDurationTicks` reports zero once the blend retires**, by design — it is a statement about
   the correction currently in flight. My first cap test read it after driving 40 frames past the
   deadline and got 0. Read it mid-blend.
3. **Do not pick cap-test numbers whose exact requirement lands on a whole tick.** 1.2 rad at
   5 rad/s is exactly 450 ticks, and the quaternion round trip through `RelativeRotationVector`
   returns 1.2000001 rad, so `Math.Ceiling` gives 451 and the test asserts the rounding rather than
   the sizing. Both cap tests now use values (0.7 m, 1.1 rad) whose requirement lands mid-tick.

## Gate output — all three green

Run with `just core-check` from the worktree root, after restoring the implementation from the
mutation runs:

```
Passed!  - Failed: 0, Passed: 422, Skipped: 0, Total: 422 - Teleop.Core.Tests.dll (net8.0)
Passed!  - Failed: 0, Passed:  28, Skipped: 0, Total:  28 - Teleop.RobotArm.Tests.dll
Passed!  - Failed: 0, Passed:   3, Skipped: 0, Total:   3 - Teleop.Eval.Tests.dll
Passed!  - Failed: 0, Passed:  36, Skipped: 0, Total:  36 - Teleop.RobotHost.Tests.dll
verify: PASS -- basic-session.tlog replays byte-identical across two independent passes
audit:  PASS -- no invariant violations found
```

422 Core tests, up from the 384 at seed; the 38 new ones are all this file's. `audit` passing also
confirms the registry entry is real — its registry-completeness check reflects for implementors of
the contract interfaces and would flag a `Reconciliation/` type that no table names.

Invariant 6 re-checked by hand on the new Core file: block-scoped namespace, no `global using`, no
collection expressions, no `required` members, no primary constructor, `System`/`System.Numerics`
only, no NuGet, no clock read, no I/O, no reflection.

## How this implementation could flatter itself in the head-to-head

The judge should read this section before believing any number.

1. **At equal `ConvergenceBudgetMs`, `spring` and `budget-blend` are not equal-effort operating
   points, and the gap is set by a constant inside `spring`.** `spring` derives its natural
   frequency from `SettledFractionOfInitialError = 0.01`, which forces `w = 6.638/T` — an
   aggressive spring, because an exponential is slow in the tail and reaching 1% costs early speed.
   If that constant were 0.10 instead, `w` would be `3.89/T` and `spring`'s peak jerk would be
   `118 o0/T^3` against the quintic's `60` — a 2× gap, not the ~10× the current definition implies.
   **The head-to-head at a single budget therefore partly measures `spring`'s choice of settle
   fraction.** The right defence is to sweep `ConvergenceBudgetMs` over at least a decade and
   compare the *curves*, not one point; if `budget-blend`'s advantage is a fixed multiple across
   the sweep, it is structural, and if it moves with the budget, it is the settle fraction.
2. **`time_to_convergence_ms` will look better for reasons that are partly definitional.** Both use
   the identical docs/metrics.md §5 instrument (first frame the residual is inside tolerance), but
   the quintic is inside a 1%-of-`o0` tolerance at about `u = 0.90` while `spring` is defined to
   reach 1% exactly at `u = 1`. Reporting a shorter convergence time is a genuine consequence of
   the shape, not an artifact — but it is not evidence of "converging faster than the budget
   allows", and it should not be added to a jerk win as if it were an independent one.
3. **The known ~1.3 m startup transient is a cap test, not a blend test.** With the default 5 m/s
   cap this reconciler stretches its deadline to 487 ms while `spring` clamps velocity and takes
   roughly 260 ms plus tail — so on that one transient `spring` may well converge *sooner* and
   `budget-blend` may show *lower* jerk, and neither number says anything about the convergence
   law. On the `lan` profile that single transient dominates p95/p99. **Any comparison that does
   not exclude or separately report the opening transient is measuring the two rate-cap policies.**
4. **Peak speed is `<= cap` only for an isolated correction.** Across a retarget it can reach
   `cap + |v_carried|`. On a lossy profile with frequent retargets, `budget-blend` can exceed the
   nominal cap where `spring` (which clamps every frame) cannot. If a result turns on apparent
   motion rather than jerk, this asymmetry is the first thing to check.
5. **`correction_magnitude_mm`/`_deg` are identical across all three reconcilers by construction** —
   they measure the predictor's disagreement before any reconciler acts. Never a differentiator,
   and a "win" there is a bug in the harness.

## A finding that is not about this reconciler: the one-frame hold at a retarget

Worth recording as an axis-level observation, because I found it while reasoning about C1 and it is
**pre-existing in `spring`**, not introduced here.

Both reconcilers seed the residual offset from the *last displayed* pose:
`offset := lastDisplayed - predicted_now`. Because `lastDisplayed` is the previous frame's output
while `predicted_now` is this frame's prediction, the composed output on a retarget frame is
exactly the previous frame's output. **The display is held for exactly one frame at every
retarget**, which is a velocity dropout of the full in-flight offset speed and is O(1) in the frame
interval — it does not shrink as the frame rate rises. On a lossy profile, where retargets are
frequent, this is plausibly a larger contributor to measured `jerk_mm_s3` than either convergence
law, which would make the whole head-to-head less sensitive than the analytic table above suggests.

The fix is to advance the in-flight blend to `nowTicks` *before* seeding, so the offset is taken at
the current instant rather than one frame stale. It is a change to a rule both files share, so it
belongs in one change touching `SpringReconciler.cs` and this file together — I did not make it,
because `SpringReconciler.cs` is out of my file scope and changing one reconciler's seeding but not
the other's would silently confound the very comparison this work exists to enable.
`TimeBudgetedBlendReconcilerTests.TheOneFrameHoldAtARetargetIsSharedWithSpring` pins the parity so
that if someone does fix one of them, the test fails and the comparison gets re-examined rather
than quietly shifting.

## Left for a human / for the merge

- **`Reconciliation/CLAUDE.md`** needs `budget-blend` moved from "Planned, not yet implemented" to
  "Implemented". I did not edit it — it is shared with two other agents building the other
  candidates, and three agents rewriting the same table produces a conflict rather than a record.
  Suggested Implemented row:
  `| budget-blend | TimeBudgetedBlendReconciler.cs | finite quintic blend; residual identically zero at a deadline fixed at seed time; C1 at onset/completion and across a retarget; rate caps lengthen the deadline rather than clamping the schedule |`
- **No experiment YAML and no sweep were produced**, per instruction. When the head-to-head is run,
  points 1–3 of "How this implementation could flatter itself" are the design constraints on it:
  sweep `ConvergenceBudgetMs` over a decade rather than fixing it, hold the predictor fixed, and
  report the opening transient separately from steady state.
- **Unresolved, deliberately:** carrying offset *acceleration* across a retarget (would give C2
  there, and is one line — the `b2` term is already in the derivation in `SetBlendCoefficients`'s
  doc). Not done because it would make this differ from `spring` in two ways at once. It is the
  obvious next experiment once the two-way comparison is attributed.
- **Nothing is blocked on hardware or on `unity/`.** No hardware was commanded, nothing under
  `results/` was touched, and no commit was made.

## Verdict

**Implementation complete and verified; no performance claim made.** The reconciler does what it
was specified to do — exact, finite, magnitude-independent convergence at a deadline fixed at seed
time, C1 at onset, mid-blend, completion and across a retarget, allocation-free, deterministic,
metric-compatible with `spring` sample for sample. Whether the hard deadline beats the asymptotic
decay on `jerk_mm_s3` is **not answered here** and must not be inferred from the analytic table in
the hypothesis: that table is arithmetic on the two convergence laws in isolation, and the
one-frame retarget hold both reconcilers share is a plausible mechanism for it to be substantially
wrong in practice. Falsifier 1 remains open and is the head-to-head's job.
