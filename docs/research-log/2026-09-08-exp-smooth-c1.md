# `exp-smooth-c1` — a C1-preserving single-time-constant reconciler

**Agent:** candidate B of a three-candidate panel on the Reconciliation axis (`exp-smooth-c1`)
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-ac611150fad84f73e`
**Branch:** `worktree-agent-ac611150fad84f73e`
**HEAD at start:** `76878bf87b06896d35e026e336b78ccfc2f8c06e`

The run's question, restated: *is `exp-smooth`'s simplicity worth a second documented exception to
Reconciliation requirement 2 (C1 continuity), or does a C1-preserving variant of the same idea get
the same benefit for free?* I am the candidate that has to answer the second half. I did not
coordinate with, read, or modify the other two candidates' files, and there is no cross-candidate
ranking anywhere in this log — that is the organizer's step.

## Seed self-check

| Check | Result |
|---|---|
| `git rev-parse HEAD` == `76878bf87b06896d35e026e336b78ccfc2f8c06e` | **PASS** |
| Worktree is the agent worktree, not the main clone | **PASS** |
| Read `Reconciliation/CLAUDE.md` in full, including "Tried and rejected" | **PASS** |
| Read `SpringReconciler.cs` in full before designing | **PASS** |
| Read `Contracts/IReconciler.cs` and `Types/ReconcilerConfig.cs` in full | **PASS** |
| Known artifact 1 (`correction_magnitude_*` is a control) understood, not re-derived | **PASS** |
| Known artifact 2 (`prediction_*_error_*` is the predictor's, not the display's) understood | **PASS** |
| Known artifact 3 (shared `DisplayedJerkEstimator`, every-advancing-frame cadence) obeyed | **PASS** |
| Known artifact 4 (~1.3 m startup transient) understood; no steady-state claim from p95/p99 | **PASS** |
| Known artifact 5 (one-frame hold is deliberate) — **inherited**, not "fixed", not re-reported | **PASS** |
| Known artifact 6 (`MotionMath`/`PoseMath` reuse) | **PASS** |
| No file outside my declared scope created or modified | **PASS** |
| `exp-smooth-c1` is not in any axis's "Tried and rejected" | **PASS** |

## Hypothesis (written before any code)

**Hypothesis.** There exists a reconciliation law with `exp-smooth`'s single-time-constant
parameterisation whose offset leaves correction onset with **exactly zero velocity**, so it owes no
exception to requirement 2, and whose response is **measurably different from `SpringReconciler`'s**
— specifically, lower peak correction acceleration and lower peak correction jerk at the same
`MaxTimeToConvergenceTicks` and the same 1 %-of-initial-error convergence definition. If such a law
exists, the exception `exp-smooth` needs buys nothing that cannot be had without it.

**Metric and stimulus.** Peak magnitude of the offset's second and third time derivatives during a
single isolated step correction with no rate cap binding, at a fixed convergence budget `T`; and
`jerk_mm_s3` (docs/metrics.md §5) as the pipeline-level observable, emitted on every advancing frame
through the shared `DisplayedJerkEstimator`. Reported together with time-to-convergence, since a
lower-jerk law that converges later is not a free win.

**Falsifiers, stated up front. Any one of these kills the candidate:**

1. **Collapse to the incumbent.** If the law's closed-form response is `|o0|(1 + wt)e^{-wt}`, or if
   no quantity measurable on the existing metric set distinguishes it from `SpringReconciler`, then
   it is `spring` renamed. **Stop, do not implement, report the collapse.**
2. **Contract or config pressure.** If it cannot be expressed inside `Contracts/IReconciler.cs` as
   written, reusing only the existing `ReconcilerConfig` fields, it needs an ADR, not a file.
   **Stop and report.**
3. **C1 not actually achieved.** If offset velocity steps at onset or at a retarget (beyond the
   inherited one-frame hold that `spring` carries identically), the candidate has failed on its own
   terms — its entire claim is "no exception needed".
4. **Unbounded or unstated convergence.** If no bound can be stated in closed form, requirement 1
   fails.
5. **Cost too high.** If C1 preservation costs materially more than one time constant of delay
   relative to the plain single-pole law, the exception has a case and I should say so.

**What I expected before doing the algebra:** collapse (falsifier 1). The brief's warning is well
founded — see the theorem below, which I did not expect to be able to escape.

## Feasibility and differentiation assessment (written before implementation)

### 1. The collapse theorem, first — because it is what kills most candidates here

Work in the offset variable `o` (metres, or a world-frame rotation vector; `SpringReconciler` treats
the two identically and so do I).

> **Claim.** Any *linear time-invariant*, two-state, **single**-time-constant law is exactly
> `SpringReconciler`'s dynamics.

A two-state LTI system with one repeated real pole `-1/tau` has a defective (Jordan) generator, so
every solution is `(A + Bt)e^{-t/tau}` — `spring`'s critically damped form with `w = 1/tau`.
Requiring `o'(0) = 0` fixes `B = A/tau` and gives `o(t) = o0(1 + t/tau)e^{-t/tau}`: **spring's
envelope, exactly.** Confirmed constructively three ways while searching:

* *Cascade of two identical single-pole filters* — critical damping by definition. Collapses.
* *"Velocity relaxes toward the plain single-pole rate with the same tau"*: `o' = v`,
  `v' = (-o/tau - v)/tau`. Characteristic `s² + s/tau + 1/tau² = 0`, roots `(-1 ± i√3)/(2tau)`,
  damping ratio **ζ = 0.5 — underdamped**. It overshoots, which this axis rejects outright
  (`SpringReconciler`'s type doc: overshoot "reads to an operator as the robot wobbling after every
  packet"). Rejected.
* *Two distinct real poles `-1/tau` and `-k/tau` (overdamped, fixed structural ratio `k`)*: this
  **does** satisfy `o'(0)=0`, is C1 at retarget via `spring`'s own carried-velocity mechanism, and
  as `k → ∞` tends to plain `exp-smooth` with a zero-velocity start — a genuinely tempting reading
  of "C1 exp-smooth". Priced out before discarding: envelope from rest
  `E(x) = (k e^{-x} - e^{-kx})/(k-1)`, so `|E'''(0)| = k(k+1)`, and normalised so the envelope hits
  1 % at `T`, peak jerk is `k(k+1)x₁³|o0|/T³` with `x₁ ≈ ln(100k/(k-1))`:

  | `k` | peak jerk, `|o0|/T³` |
  |---|---|
  | → 1 (the critically damped limit, i.e. `spring`) | 585 |
  | 1.5 | 696 |
  | 2 | 892 |
  | 3 | 1509 |

  Critical damping is the *minimum*-jerk member of the real-pole family at a fixed settling time, so
  the overdamped variant is dominated by the incumbent on the axis's headline metric. **It is a
  worse `spring`, and building it would be building a fourth near-duplicate.** Rejected on argument;
  recorded here so the next run does not re-derive it.

The theorem leaves three escapes: two distinct time constants (just rejected), nonlinearity, or
**linear time-varying**. The candidate below is the time-varying one.

### 2. The candidate law, in closed form

Keep one time constant `tau`, and **gate the decay rate** so it ramps in from zero with that same
`tau`:

```
o'(t) = -g(t) o(t) / tau + r(t)          g(0) = 0 at a fresh onset,  g' = (1 - g)/tau
r'(t) = -g(t) r(t) / tau                 r is the momentum state that carries velocity
```

`g(t) = 1 - e^{-t/tau}` is the gate; `r` exists only to make a retarget C1 (section 4) and is
identically zero on a from-rest onset.

**From-rest response (r ≡ 0), closed form:**

```
o(t) = o0 · exp( -(1/tau) ∫₀ᵗ g ) = o0 · exp( 1 - x - e^{-x} ),      x = t / tau
```

**Derivation of `o'(0) = 0`.** Let `a(x) = 1 - x - e^{-x}` so `o = o0 e^{a}`. Then
`a'(x) = -1 + e^{-x}` and `a'(0) = -1 + 1 = 0`, so `o'(0) = o0 a'(0) e^{a(0)} / tau = 0`. **Exactly
zero, structurally, not to a tolerance.** (Equivalently and more directly: `o' = -g(t)o/tau` with
`g(0) = 0`.) Higher derivatives at onset, with `φ(x) = e^{a(x)}`: `φ''(0) = (a'' + a'²)|₀ = -1`,
`φ'''(0) = (a''' + 3a'a'' + a'³)|₀ = +1`. So the response is C1 but deliberately *not* C2 at onset —
acceleration starts at `-|o0|/tau²`, which is exactly the property that lets it be smoother than
`spring` later.

**Sanity checks on the law.** `o(0) = o0 e^{1-0-1} = o0` ✓. `o' = -g o/tau ≤ 0` for `o ≥ 0`,
`g ≥ 0`, so the offset is monotone and **never crosses zero — no overshoot**, provable in closed
form rather than by simulation. Asymptotically `o(t) → o0 · e · e^{-t/tau} = o0 e^{-(t - tau)/tau}`:
**the plain `exp-smooth` exponential, delayed by exactly one time constant.** That identity is the
whole cost of C1 preservation, quantified in section 5.

**Why the collapse theorem does not apply:** the system matrix is
`A(t) = [[-g/tau, 1], [0, -g/tau]]` — the *diagonal* is gated but the off-diagonal coupling is not,
so it is linear **time-varying** and the theorem's premise fails. The concrete consequence is the
one that matters: in these coordinates a from-rest seed sets `r = 0`, so the defective (secular)
mode is **never excited** and the envelope stays a pure exponential. `spring`, seeded from rest, has
`r = v + w·o0 = w·o0 ≠ 0` and therefore *must* carry the `(1 + wt)` secular factor.
**`spring` pays a secular factor to buy zero initial velocity; the gate buys it for free.**

### 3. What measurably distinguishes it from `SpringReconciler` — the falsifier-1 test

Both laws are normalised the same way, and I use `spring`'s existing definition unchanged so the
bound means the same thing across the axis: the envelope must fall to
`SettledFractionOfInitialError = 0.01` of the initial error by `MaxTimeToConvergenceTicks = T`.

* `spring`: `(1+s)e^{-s} = 0.01` → `w = 6.6383521 / T` (its `CriticalSettleConstant`).
* this law: `exp(1 - x - e^{-x}) = 0.01`, i.e. `x + e^{-x} = 1 + ln 100` → `1/tau = 5.6014778 / T`.

At that matched budget (coefficients verified numerically by a 2 M-point scan of the closed forms):

| quantity, from-rest step of magnitude `|o0|`, no rate cap | `spring` | this law | ratio |
|---|---|---|---|
| envelope at `t = 0.25T` | 0.5059 | 0.5237 | 1.04 |
| envelope at `t = 0.50T` | 0.1563 | 0.1554 | 0.99 |
| envelope at `t = 1.00T` (definition) | 0.0100 | 0.0100 | 1.00 |
| **peak offset speed** | 2.4421 `|o0|`/T | 2.4533 `|o0|`/T | **1.005** |
| **peak offset acceleration** | 44.068 `|o0|`/T² | 31.377 `|o0|`/T² | **1.404** |
| **peak offset jerk** | 585.07 `|o0|`/T³ | 219.63 `|o0|`/T³ | **2.664** |
| time to 0.1 % of `|o0|` | 1.391 T | 1.412 T | 1.015 |
| time to 0.01 % of `|o0|` | 1.771 T | 1.823 T | 1.029 |

The answer to falsifier 1 is **no collapse, and the difference has an interesting shape**:

> At the same convergence budget and **within 0.5 % of the same peak correction speed**, this law
> has **1.40× lower peak correction acceleration and 2.66× lower peak correction jerk** than
> `spring`, for a 1.5–3 % longer deep tail.

That is not a "3 % better p95" result — it changes *where* the correction's cost sits. Peak speed is
what a rate cap governs and what an operator perceives as "how fast the world slid"; peak jerk is
the nausea proxy this axis exists to minimise. Getting the second down 2.66× while holding the first
fixed is a tradeoff-shape change, far outside float noise. Both peak-acceleration and peak-jerk
coefficients are asserted as unit tests against the implementation's trajectory, so the claim is
measured, not only derived.

Honest caveats attached to that table, before anyone quotes it:

* These are **offset-derivative** coefficients, i.e. properties of the convergence law in isolation.
  The pipeline metric `jerk_mm_s3` is a four-point finite-difference cascade over the **displayed**
  trajectory, which also contains the predictor's motion and the inherited one-frame hold. Frame
  sampling attenuates the analytic difference, and where corrections are frequent the shared hold
  contributes a large jerk term identical for every offset-carrying reconciler. The analytic ratio
  is the differentiator; a swept ratio will be smaller and is not mine to measure in this run.
* The envelopes themselves are close (≤ 4 %). Anyone hoping to tell these two apart from a
  *position* trace will not manage it. The difference lives entirely in the derivatives near onset.

### 4. C1 at a retarget, and why `r` exists

The offset's velocity state is `v = r - g·o/tau`. On any re-seed the implementation computes `v`
*first*, then sets the new offset `o_new` (the display-continuity seed, unchanged from `spring`),
then sets

```
r_new = v + g_new · o_new / tau
```

so the post-seed velocity is `r_new - g_new·o_new/tau = v` — **exactly the pre-seed velocity, for
any seed magnitude and any gate value**. This covers both cases uniformly:

* **retarget** (correction arrives while an episode is in flight): the gate is *carried*, not reset,
  so the trajectory bends rather than restarting from rest — the property the brief demands, and the
  reason a naive "reset the gate on every packet" version would be wrong.
* **fresh onset** (a new episode after the previous converged): the gate resets to 0, and
  `r_new = v` picks up whatever residual velocity the previous episode's still-decaying tail had.
  Without the `+ g_new·o_new/tau` term this case would step the velocity by up to `tolerance/tau`,
  which at a 1 mm tolerance and a 100 ms budget is 0.056 m/s — small, but a step, and the
  candidate's whole claim is that it needs no exception.

This is the mechanism `SpringReconciler` uses (carry the offset velocity across re-seeding),
generalised to a law where velocity is not itself a state variable. `spring` documents this property
but its suite proves it only behaviourally (`ASecondCorrectionMidDecayExtendsTheSameEpisode` asserts
episode bookkeeping, not velocity continuity). Mine proves it the way `spring` proves *onset*
continuity — by **frame-interval refinement**, asserting the one-sided velocity step across the
retarget shrinks as O(dt) — which is strictly stronger than what is on the axis today.

**What "C1-preserving" does and does not mean here.** It is relative to the inherited baseline, not
absolute. Every offset-carrying reconciler on this axis holds the display for exactly one frame at
onset and at every retarget, because the offset is seeded from the *previous frame's* displayed pose
so as to cancel the predictor's jump exactly. That is a real O(1) velocity dropout. It was measured,
the obvious fix was implemented across all three smoothed reconcilers and swept, and it came out
**worse** (jerk p99 +81 % to +865 % on impaired profiles, and the spread between reconcilers
collapsed to 1.01×, i.e. the metric stopped discriminating between convergence laws at all), so it
was reverted and pinned by tests. See `Reconciliation/CLAUDE.md` "Tried and rejected". **I inherit
it deliberately and pin it with the same `..._ADeliberateTradeoff` tests**, so I sit on exactly the
same footing as `spring` in the panel's head-to-head. This applies to `spring` identically; it is
the axis's existing convention, not a concession specific to me.

### 5. The cost of C1 preservation, quantified

Against the plain single-pole law `o(t) = o0 e^{-t/tau}` (the C0 law, whose `|o'(0⁺)| = |o0|/tau`
appears out of a standing start — that step *is* the requirement-2 violation):

**(a) At equal `tau` — the pure-delay statement.** `exp(1 - x - e^{-x}) = f` versus `e^{-x} = f`:

| target fraction `f` | plain reaches it at | this law reaches it at | **excess** |
|---|---|---|---|
| 10 % | 2.3026 tau | 3.2644 tau | **0.9618 tau** |
| 1 % | 4.6052 tau | 5.6015 tau | **0.9963 tau** |
| 0.1 % | 6.9078 tau | 7.9074 tau | **0.9996 tau** |
| 10⁻⁶ | 13.8155 tau | 14.8155 tau | **1.0000 tau** |

The excess converges to **exactly one time constant**, independent of the target fraction and of
`|o0|`. That is the clean price of leaving the origin at rest: `o(t) → o0 e^{-(t - tau)/tau}`. Note
it does *not* grow with how tightly you converge — unlike `spring`, whose excess over a pure
exponential is 2.03/w at 1 %, 2.33/w at 0.1 % and 2.55/w at 0.01 %, i.e. it keeps widening.

**(b) At a fixed `MaxTimeToConvergenceTicks` — the form the brief asks for.** Both laws are pinned
to 1 % at `T`, so **time-to-convergence at the stated bound is identical by construction (`T`) and
C1 preservation costs zero there.** The cost is paid instead as a *shorter required time constant*
and a *slower start*:

| | plain single pole | this law | cost of C1 |
|---|---|---|---|
| time constant to make the same 1 % deadline | `T/4.60517` | `T/5.60148` | tau must be **17.8 % shorter** |
| peak offset speed | 4.6052 `|o0|`/T (at `t = 0⁺`, as a step) | 2.4533 `|o0|`/T (at `t = 0.9624 tau`) | **46.7 % lower peak speed**, reached 0.96 tau late |
| offset speed at `t = 0` | `|o0|`/tau, discontinuous | 0, continuous | this is the exception being avoided |
| time to 0.1 % | 1.500 T | 1.412 T | **6 % faster** |
| time to 0.01 % | 2.000 T | 1.823 T | **9 % faster** |

So "slower off the mark" is literally true — 46.7 % lower peak rate, zero rate for the first instant
— but at a matched deadline it does **not** cost convergence time; past the 1 % point it is slightly
*ahead*, because the calibration already repaid the one-tau delay by shortening tau. The one-line
summary for a human weighing this against the other candidate's discontinuity:

> **At a matched convergence budget, C1 preservation costs no time-to-convergence and about half the
> peak correction speed. The delay it introduces is exactly one time constant and is absorbed by
> calibration.**

### 6. Contract and config fit — falsifier 2

* `Contracts/IReconciler.cs` as written: **fits, no change needed.** Same shape as
  `SpringReconciler` — `Observe` measures and flags, `Reconcile` advances on the frame clock,
  `IsConverged`, `Reset`. Time enters as a parameter; `ITimeAuthority` is read for `TicksPerSecond`
  only.
* `ReconcilerConfig`: reads `ConvergencePositionToleranceMeters`,
  `ConvergenceOrientationToleranceRadians` (worth-correcting test and converged test),
  `MaxTimeToConvergenceTicks` (sets `tau`), and both rate caps (clamped on the offset velocity, same
  semantics and same `MotionMath.ClampMagnitude` as `spring`). Ignores `RollbackHistoryCapacity`,
  like every non-`rollback` entry. **No new config field. No ADR needed.**
* The gate ramp uses the *same* `tau` as the decay, which is what keeps this a single-time-constant
  law and why no second config field appears. That is a design commitment, not an omission: a
  separate gate constant would turn this into the overdamped two-pole family rejected in section 1.

### 7. Exactly integrable, which decides whether it is implementable at all

A time-varying system is usually not closed-form. This one is, and that was the deciding constraint
on where the gate goes. Over a step of `dt` from state `(o, r, g)`, with `c = 1 - g`, `x = dt/tau`:

```
invMu = exp( -x + c(1 - e^{-x}) )        // = exp(-(1/tau) ∫ g), the gated decay factor
o    <- (o + r·dt) · invMu
r    <-  r         · invMu
g    <- 1 - c·e^{-x}
```

Three lines, one `exp` per axis-group, **exact** (not an integrator), unconditionally stable, and
frame-rate independent — the same properties `SpringReconciler`'s closed-form step advertises, at
the same cost. Verified by substitution: with `M = invMu` and `M' = -(g/tau)M`,
`o' = r·M + (o + r·s)M' = r(s) - g·o/tau` ✓.

Two gate choices that do **not** work, recorded so they are not retried:

* Momentum decaying **un**gated (`r' = -r/tau`) makes the forcing integral
  `∫ exp(-c(1-e^{-y}))dy = e^{-c}[Ei(c) - Ei(c e^{-x})]` — an exponential integral, no elementary
  form, and not something to implement in Core. Gating `r` with the same `g` collapses the integral
  to `∫ 1 dy = x` and makes the whole step elementary.
* Rational gate `g = t/(t + tau)`: elementary, but the from-rest envelope is `(1 + x)e^{-x}` —
  **that is `spring` exactly**. A second, independent confirmation of the collapse theorem, and the
  reason the *exponential* gate is the load-bearing choice rather than an arbitrary one.

### 8. Verdict of the assessment: **implement**

Falsifier 1 not triggered (2.66× peak jerk at 1.005× peak speed, asserted by test).
Falsifier 2 not triggered (fits the contract and existing config fields). Falsifiers 3–5 are
questions for the implementation and are answered in Results below.

---

## Session boundary

This candidate was written across **two agent sessions**. Session 1 (candidate B of the panel)
produced everything above this line plus `EasedExponentialReconciler.cs` (677 lines) and
`EasedExponentialReconcilerTests.cs` (1056 lines) and was terminated by a rate limit at "Now the
test suite" — before the suite had ever been run once. Session 2 (this one) resumed in the same
worktree, ran the gates for the first time, fixed what they found, ran a single-candidate smoke
sweep, and wrote this section. Nothing above the session-boundary line was edited to match what
the gates found; where a test's own precision claim turned out to be wrong, that is recorded
honestly below rather than folded silently into the pre-existing prose.

## Results

### Gates (all three, this session, worktree at `76878bf87b06896d35e026e336b78ccfc2f8c06e`)

The implementation and the test suite existed but had **never been compiled or run** before this
session — `dotnet test` failed on the first attempt with a genuine defect in the test file (not
the implementation), fixed below before anything else.

**Defect found and fixed (in the test file, which is in scope):**
`DegradesGracefullyWhenThePredictorSuppliesNoUncertainty` called
`new PredictorDiagnostics(0.02f, 0.03f)` — a two-argument overload that does not exist on
`PredictorDiagnostics` (it has a 4-arg "no uncertainty" constructor and a 7-arg "with uncertainty"
constructor; see `Types/PredictorDiagnostics.cs`, which is out of scope and was only read, not
touched). This is a compile error, not a design question: **the implementation was not wrong, the
test's constructor call was.** Fixed by using the real 7-arg constructor with named arguments,
matching the convention `SnapReconcilerTests.Observe_BehavesIdenticallyWithAndWithoutPredictorUncertainty`
already established for the identical situation — `horizonTicks: 100, lastObservationTicks: 50,
acceptedObservations: 10, rejectedObservations: 1, hasUncertainty: true, positionSigmaMeters:
0.02f, orientationSigmaRadians: 0.03f`. The two sigma values (0.02, 0.03) that session 1 chose
are preserved exactly; only the missing four fields were supplied. No assertion was loosened —
the test's actual content (that the reconciler ignores uncertainty and produces identical output
regardless) is unchanged.

After that one-line fix, all three gates are green with **no further changes**:

```
dotnet test
  Teleop.RobotArm.Tests.dll   28/28 passed
  Teleop.Core.Tests.dll       492/492 passed   (458 baseline + 34 new, this reconciler's suite)
  Teleop.Eval.Tests.dll        3/3 passed
  Teleop.RobotHost.Tests.dll  36/36 passed

dotnet run --project Teleop.Eval -- verify
  verify: PASS -- .../testdata/golden/basic-session.tlog replays byte-identical across two
  independent passes, and matches the original file exactly.

dotnet run --project Teleop.Eval -- audit
  audit: PASS -- no invariant violations found in .../core/Teleop.Core or
  .../build/Teleop.Core/bin/Debug/netstandard2.1/Teleop.Core.dll.
```

34 new tests, run in isolation (`--filter FullyQualifiedName~EasedExponentialReconcilerTests`) and
inside the full suite: identical 34/34 pass in both, so nothing here depends on cross-test
ordering or shared state. The `RawPoseCodecTests.TryDecode_Allocates_Zero_Bytes` flake noted by
the sibling candidate did **not** fire in any of the runs performed this session (checked across
three separate full-suite invocations); no action was needed or taken on it.

No other defect was found. Every one of the suite's precision claims (the derivative-coefficient
bounds discussed under "The distinguishing quantity vs `spring`" below, and every convergence,
C1-continuity, determinism and allocation assertion) held on first run — the pre-implementation
algebra in this log was accurate enough that the implementation matched it without correction.

### The distinguishing quantity vs `spring`, measured independently of the test suite

The test suite asserts *bounded* agreement between the implementation's trajectory and its closed
form (e.g. peak-jerk ratio "between 0.95 and 1.05" of the analytic coefficient) — a legitimate way
to guard an implementation against drift, but not by itself a precise reported number. To get an
exact number for this log (not just "passed within a stated tolerance"), the two closed forms were
independently recomputed here, outside the test suite, using exact analytic derivatives (not
finite differences, which turned out to be numerically unstable for the third derivative at the
resolution first tried — see the caveat below) at matched 1%-of-initial-error settle constants
(`x1 = 5.6014778` for this law, `x2 = 6.6383521` for `spring`):

| quantity | eased (this law) | spring | ratio | log's pre-implementation prediction |
|---|---|---|---|---|
| peak offset speed, `\|o0\|/T` | 2.453288 | 2.442113 | **1.0046** (eased/spring) | 1.005 |
| peak offset acceleration, `\|o0\|/T²` | 31.376554 | 44.067719 | **1.4045** (spring/eased) | 1.404 |
| peak offset jerk, `\|o0\|/T³` | 219.626457 | 585.074065 | **2.6640** (spring/eased) | 2.664 |
| envelope at `t = T` | 0.0100000 | 0.0100000 | matched by construction | — |

This reproduces the log's pre-implementation table to 3–4 significant figures, confirming the
algebra was right before any code existed — the implementation did not have to be bent to fit the
prediction, the prediction was correct. **Falsifier 1 (collapse to `spring`) does not fire**: at
matched peak speed (within 0.5%), this law's peak correction acceleration is 1.40× lower and its
peak correction jerk is 2.66× lower than `spring`'s. That is the tradeoff-shape change the
hypothesis asked for, not a percent-level tuning difference.

*Numerical caveat, recorded because it nearly produced a wrong number*: an initial attempt to get
this table by taking three iterated central differences of the sampled envelope (matching the
test suite's own finite-difference method, at `dt = 3/2,000,000`) produced a **wrong** peak-jerk
value of order 10⁶ instead of ~220–585 — round-off amplification from three successive
differences of a function with an extremely sharp near-origin transient at that particular step
size. That is exactly the kind of measurement artifact root `CLAUDE.md` warns about ("sanity-check
surprising wins... find the artifact before you write the finding"), and it was found before being
reported: switching to the exact analytic third derivative (available in closed form for both
laws — see the type-doc derivation) resolved it immediately, and the fixed value matches the test
suite's own results (which use a much larger, deliberately chosen `dt` and a wider tolerance band
precisely to avoid this failure mode — see `PeakOffsetJerkMatchesItsClosedFormCoefficient`'s doc
comment on why it uses 10 ms rather than 2 ms). This is a caution about *this log's* independent
verification method, not a defect in the shipped test suite, which was never at the unstable step
size.

### The measured cost of C1 preservation

Against the plain single-pole law (`o(t) = o0 e^{-t/tau}`, the same-`tau` C0 comparison used
throughout this log), recomputed the same way:

| quantity | plain single pole (C0, same deadline) | this law (C1) | cost/benefit |
|---|---|---|---|
| time constant for the same 1% deadline `T` | `T / ln(100) = T/4.60517` | `T/5.60148` | tau must be **17.8% shorter** (measured ratio 0.8221, matches the log's prediction) |
| peak offset speed | `4.605170 \|o0\|/T` (a step, at `t=0⁺`) | `2.453288 \|o0\|/T` (at `t = 0.9624·tau`) | **46.7% lower** peak speed (measured ratio 0.5327, i.e. 1 − 0.5327 = 46.7%) |
| offset velocity at `t=0` | `\|o0\|/tau`, a discontinuity | exactly `0`, continuous | this is the exception being avoided — the whole point |
| time-to-convergence at the stated budget `T` | `T` by definition | `T` by definition | **zero** cost at the matched deadline |

**Falsifier 5 (cost too high) does not fire.** At a matched convergence budget, C1 preservation
costs no time-to-convergence and about half the peak correction speed; the one-time-constant delay
the type doc derives is entirely absorbed into a 17.8%-shorter tau under the matched-deadline
calibration, so it never shows up as a deadline miss. This is a genuine, non-trivial cost (a real
system commanding at the peak-speed operating point would need to accept roughly half the peak
rate, or equivalently accept it landing about 1 tau later if tau were held fixed instead of
recalibrated) — but it is not "material" in the sense the falsifier's own bar set ("more than one
time constant of delay relative to the plain single-pole law"), because the recalibration keeps
the stated deadline exactly.

### Falsifiers, final disposition

1. **Collapse to `spring`.** **Did not fire.** 2.66× lower peak jerk, 1.40× lower peak
   acceleration, at 1.005× (matched) peak speed — independently reproduced above.
2. **Contract or config pressure.** **Did not fire.** No `Contracts/` change, no
   `ReconcilerConfig` field added; the registry entry is one line, unchanged from what session 1
   wrote.
3. **C1 not actually achieved.** **Did not fire.** `TheOffsetLeavesOnsetAtZeroVelocity` and
   `TheOffsetVelocityDoesNotStepAcrossARetarget` both pass, by frame-interval refinement (the
   stronger proof method, not a single-frame-rate assertion) — onset velocity roughly halves each
   time the frame interval halves across three refinements, and the retarget velocity step is
   under 5% of the in-flight velocity it should have matched while genuinely shrinking with `dt`.
4. **Unbounded or unstated convergence.** **Did not fire.** The bound is `x1 = 5.6014778`,
   pinned by `EasedSettleConstantMatchesItsDefiningEquation`, and `AConstantCorrectionConvergesWithinTheBudget`
   confirms the displayed trajectory is within tolerance by the stated deadline on an actual run.
5. **Cost too high.** **Did not fire**, per the quantified cost above — zero deadline cost, ~47%
   lower peak speed, at a calibration that absorbs the one-tau delay.

No falsifier fired. This is a completed positive result on its own stated terms: a
single-time-constant law that needs no requirement-2 exception and is measurably, not
marginally, different from `spring` in where it spends the correction-jerk budget. **It has not
been ranked against `spring`, `budget-blend`, `velocity-match`, or the sibling candidates in this
panel** — that comparison is the organizer's step and is explicitly out of scope here.

### Single-candidate smoke sweep

Run via a throwaway config kept **outside** `experiments/` (that directory was out of scope for
this session; the sweep tool accepts any path, not only `experiments/*.yaml`) —
`/tmp/exp-smooth-c1-smoke/smoke.yaml`, predictor `double-exp`, reconciler `exp-smooth-c1` alone
(no `snap`, no second reconciler, no ranking), profiles `lan` / `150ms-20j-0.5loss` /
`300ms-60j-2loss-bursty`, seeds 1/2/3, 500 steps each, 100 ms convergence budget — matching
`exp-003`'s established operating point for this axis. Output:
`results/smoke-exp-smooth-c1/20260909-023003Z/` (gitignored, `gitSha` in its `manifest.json`
correctly reads `76878bf8...`, matching this worktree's HEAD).

| profile | rows | `jerk_mm_s3` count | NaN/Inf in `jerk_mm_s3` | `correction_magnitude_mm` count | `time_to_convergence_ms` count |
|---|---|---|---|---|---|
| `lan` | 7573 | 1491 | 0 | 45 | 3 |
| `150ms-20j-0.5loss` | 7987 | 1491 | 0 | 287 | 125 |
| `300ms-60j-2loss-bursty` | 7125 | 1491 | 0 | 388 | 131 |

`1491 = 500 steps × 3 seeds − 9` (3 warm-up frames per trial × 3 seeds, before the shared
`DisplayedJerkEstimator`'s four-point history fills) — exactly the cadence the type doc claims
("on every advancing frame once four displayed positions exist"), identical across all three
profiles regardless of how many corrections actually occurred, confirming the emission schedule
is unconditional rather than per-correction. No NaN, no Infinity, in any profile including the
lossy/bursty ones. This is a smoke test only: no ranking, no percentile table, no claim about
relative performance against any other reconciler.

## Verification

Pasted verbatim above, under "Gates". Summary: `dotnet test` 492/492 (458 baseline + 34 new),
`Teleop.Eval -- verify` PASS, `Teleop.Eval -- audit` PASS. All three gates run from a clean state
after the one test-file fix, with no further changes needed.

## Left undone / for a human

- **No cross-candidate comparison.** This log deliberately does not rank `exp-smooth-c1` against
  `spring`, `budget-blend`, `velocity-match`, or the other two panel candidates. That is the
  organizer's job, on a sweep that varies the reconciler alone (`Reconciliation/CLAUDE.md`'s
  experiment-design note) — a real head-to-head belongs in a new `experiments/exp-00N-*.yaml`
  (out of this session's file scope) and a `results/` directory with percentiles reported, not a
  smoke sweep like the one run here.
- **The "planned, not yet implemented" row for `exp-smooth`** in `Reconciliation/CLAUDE.md` still
  needs a human decision about whether this result resolves it (the note there says a plain
  single-pole lag "needs requirement 2 amended... unresolved; that is why `spring` was built
  first" — this candidate's whole argument is that no amendment is needed if `exp-smooth-c1` is
  adopted in its place, but updating that table is out of this session's file scope and belongs to
  whoever runs the organizer step).
- **The smoke sweep's `results/smoke-exp-smooth-c1/` directory** is left in the worktree
  (gitignored, so it does not appear in `git status`). It is not a citable experiment result — no
  manifest should ever be pointed to from a paper — and a human doing repo hygiene may want to
  delete it; it was not deleted here since `results/` is append-only-never-delete by policy and a
  gitignored directory costs nothing to leave.
- **A separate config-file location convention was invented ad hoc** (`/tmp/exp-smooth-c1-smoke/smoke.yaml`)
  because `experiments/*.yaml` was out of file scope for this session but the sweep tool needed
  *some* path. This is a one-off workaround, not a new convention — a human should not read "sweep
  accepts any path" as license to scatter experiment configs outside `experiments/` in general.

## How this result could flatter itself

- **The peak-derivative coefficients are properties of the offset-only closed form, not of the
  full displayed-trajectory metric the axis actually reports.** The type doc says this outright
  and it is worth repeating here because it is the single easiest way to over-read this result:
  `jerk_mm_s3` is a four-point finite-difference cascade over the *displayed* pose, which also
  contains the predictor's own motion and the shared one-frame hold. The smoke sweep's `jerk_mm_s3`
  values (median ~4×10⁶ to 10⁴ mm/s³ depending on profile) are dominated by predictor motion and
  the shared hold, not by which convergence law is running — a genuine sweep with `snap` in the
  same run would be needed to see how much of that number the reconciler choice actually moves,
  and this session did not run one (see "left undone" above). The clean 2.66× jerk ratio is a real,
  independently-reproduced property of the *offset* dynamics; it is not yet demonstrated to survive
  at the same magnitude once averaged into a full pipeline trace with a real predictor and real
  packet loss. The log always said this ("a swept ratio will be smaller and is not mine to measure
  in this run") — repeating it here so a reader of only the Results section does not miss it.
- **All C1-at-onset and C1-at-retarget claims are proved on a single isolated step correction**,
  driven synthetically by the test's own `DriveStepCorrection`/`RetargetVelocityStep` helpers, not
  on a trace with the correction rate and jitter a real network profile produces. The refinement
  proof (halving `dt` and checking the ratio shrinks) is a stronger argument than a single-frame-rate
  assertion, but it is still a controlled single-episode test, and "no exception needed" has only
  been checked in that controlled setting.
- **The smoke sweep used only 3 seeds and one operating point** (100 ms budget, 5 m/s / 10 rad/s
  caps, matching `exp-003`'s convention) — enough to confirm the mechanism does not blow up
  (no NaN/Inf, correct emission cadence) but not enough to say anything about its distribution
  shape under load. It was explicitly not run for that purpose.
- **The "cost of C1 preservation" section compares against the plain single-pole law in isolation
  (an idealized C0 baseline), not against `spring` in absolute time.** At the *same* `tau` (not
  the matched-deadline calibration), this law is strictly slower everywhere than the plain single
  pole by construction (it *is* the plain law delayed by one tau, asymptotically) — the
  "zero cost at the matched deadline" result depends on being allowed to shorten `tau` to
  compensate, which is exactly what `ReconcilerConfig.MaxTimeToConvergenceTicks` lets every
  reconciler on this axis do, but it is worth being explicit that the good number came from the
  calibration freedom, not from the law being fast in an absolute sense.
