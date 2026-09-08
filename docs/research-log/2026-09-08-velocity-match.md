# 2026-09-08 — `velocity-match` (`VelocityMatchedReconciler`)

Agent: parallel candidate build. One of three reconcilers being built independently for a later
head-to-head. **No comparison sweep was run here and no ranking is claimed** — that is the judge's
job, deliberately not mine.

## Seed self-check

| Check | Result |
|---|---|
| `git log --oneline -1` | `16b45dc Add the deep-researcher agent for unattended runs on Core's open axes` — matches the expected sha |
| `Reconciliation/SpringReconciler.cs` exists | yes (30 611 bytes) |
| `.claude/agents/deep-researcher.md` readable | yes, read in full |
| `just core-check` green on a clean tree | yes — 384 + 28 + 3 + 36 tests passed, `verify: PASS`, `audit: PASS` |

Environment note: the sandbox refuses `export VAR=... ; cmd` and refuses `PATH=...:$PATH`, so every
command in this run used a fully literal environment prefix:
`DOTNET_ROOT=/home/andrei/.dotnet PATH=/home/andrei/.dotnet:/home/andrei/.local/bin:/usr/local/bin:/usr/bin:/bin /home/andrei/.local/bin/just core-check`

## Hypothesis (written before any code)

**Claim.** A reconciler that bounds the *velocity it injects into the displayed trajectory* as a
fraction of the operator's own apparent speed, rather than bounding the *time* it takes to remove
position error, will produce a materially lower injected-velocity peak (and therefore lower
displayed jerk) at the cost of a longer, but still bounded, convergence time.

**What should improve.** `jerk_mm_s3` percentiles, and the peak injected speed (offset velocity)
under a step correction while the operator is stationary.

**What must get worse if the design is honest.** `time_to_convergence_ms`. The stated worst-case
bound is 4x the configured convergence budget. If it did *not* get worse, the correction is not
actually being slowed and the mechanism is not doing what it claims.

**Falsifiers** — any of these kills the idea:

1. The apparent-motion estimate cannot be sourced from the `IReconciler<Pose>` contract without
   reading a clock, allocating, or breaking on duplicate/out-of-order/gapped `Observe`. Then the
   contract genuinely cannot support the idea and that is the finding.
2. The stated convergence bound cannot be proved by test — i.e. a stationary-operator step
   correction does not reach the 1%-of-initial envelope within `budget / MinimumTimeWarp`.
3. C1 fails: the max per-frame velocity step does **not** shrink as the frame interval is refined.
4. The central claim fails: peak injected speed under a step correction is **not** measurably lower
   than a position-first reconciler's (`spring`, same budget, same trace, same seed).
5. The adaptive term is inert — if the time warp sits at its floor for every realistic correction,
   then `velocity-match` is just "`spring` with a 4x budget" and the idea adds nothing that a config
   change could not. (This one is partially true; see "Honest characterisation" below.)

## Where apparent motion comes from, and why

`IReconciler<Pose>` hands out poses, never velocities, so this had to be decided explicitly.

**Chosen: a finite difference of the predictor's per-frame output** (`Reconcile`'s `predicted`
argument), exponentially smoothed with a time constant equal to the convergence budget, and
**held — not updated — on any frame where a correction is seeded.**

Why the predictor's output and not the authoritative stream:

- *It is the right signal by definition.* "Apparent motion" is the motion of the thing on screen.
  The displayed pose is `predicted + offset`; the operator's genuine apparent motion is the
  `predicted` term. The authoritative stream describes the **robot's** motion one transport delay
  ago, which is not what is being looked at.
- *Attribution.* Authoritative samples arrive at a cadence set by the network profile. A velocity
  estimate finite-differenced from them would have profile-dependent noise, which would make this
  reconciler's behaviour a function of the transport axis. A reconciler-only sweep could then not
  attribute its own result — the exact trap root `CLAUDE.md` and `Reconciliation/CLAUDE.md` warn
  about.
- *Availability.* The warp is needed every frame. `Observe` may not be called for hundreds of ms.
- *Robustness comes for free.* `Observe` contributes **nothing** to the velocity estimate except the
  pending-seed flag, and stale/duplicate samples are rejected (strictly-increasing capture stamp,
  the same rule `spring` uses) before that flag is set. So duplicate, out-of-order and gapped
  observations are structurally incapable of corrupting the estimate.

Why **not** the displayed output, which was the other tempting choice: it is circular. The
displayed motion contains the injected correction, so modulating the correction by it is positive
feedback — a correction that moves makes the display look fast, which licenses a faster correction.

Why the **hold on a seeding frame**: on the frame a correction is seeded, the predictor's own output
has just absorbed truth and therefore contains a step. `predicted_k - predicted_{k-1}` on that frame
is (operator motion + the predictor's jump), and the two are *not* separable from poses alone. They
are separable statistically — the jump is a one-frame impulse, operator motion is persistent — so the
estimate is held for that one frame. Without the hold, the estimator would report a large apparent
speed at exactly the instant the correction starts, which would license the most aggressive possible
correction at exactly the moment the premise says to be gentlest. That is the sharpest failure mode
in this design and the hold is the fix.

No clock is read: the estimate is driven entirely by `Reconcile`'s `nowTicks` parameter, and
`ITimeAuthority` is consulted for `TicksPerSecond` only, exactly as `snap` and `spring` do.

## Stated convergence bound

The offset follows `spring`'s **exact** critically damped closed form, but integrated on a
**warped** time axis: `dtau = w * dt`, with `w` in `[MinimumTimeWarp, 1]` and
`MinimumTimeWarp = 0.25`.

- The critically damped envelope `|o(tau)| = |o0| (1 + omega*tau) e^(-omega*tau)` is strictly
  decreasing in `tau`, and `tau(t) >= MinimumTimeWarp * t`. Therefore
  **`|o(t)| <= |o0| (1 + omega*w_min*t) e^(-omega*w_min*t)`** unconditionally, whatever the operator
  does.
- `omega` is fixed from `MaxTimeToConvergenceTicks` by exactly `spring`'s rule (the same
  `CriticalSettleConstant`, the same 1% settle fraction), so at `w = 1` the dynamics are *identical
  to `spring`*.
- **Stated bound: the 1%-of-initial-error envelope is reached within
  `MaxTimeToConvergenceTicks / MinimumTimeWarp` = 4x the configured budget**, worst case, and within
  exactly the budget when the operator's apparent speed is high enough to license `w = 1`.
- `w <= 1` means this reconciler is **never more aggressive than `spring` at the same budget**. It
  either matches it or is gentler. That is a deliberate structural property: it makes the
  head-to-head a one-sided question.
- Same caveats as `spring`, inherited verbatim: the bound is *relative* (1% of a large correction can
  still exceed the absolute position tolerance), and the rate caps take precedence — if
  `MaxCorrectionLinearSpeedMetersPerSecond` binds, convergence stretches further.

## The apparent-motion guarantee (the central claim, as a bound)

Per frame, the warp is chosen as
`w = clamp(VelocityMaskingFraction * sHat / |v_trial|, MinimumTimeWarp, 1)`
where `v_trial` is the offset velocity the unwarped (`w = 1`, i.e. `spring`) step would have
produced this frame and `sHat` is the smoothed apparent speed. Hence the injected speed satisfies

> **`|injected velocity| <= max(MinimumTimeWarp * |spring's injected velocity|,
> VelocityMaskingFraction * apparent speed)`**

`spring` has no such bound at all — its injected speed is whatever the budget demands, regardless of
what the operator is doing. This inequality, not a percentage, is what the candidate actually offers.

`VelocityMaskingFraction = 0.25`. Chosen as a **definition, not a tuning knob** (same status as
`spring`'s `SettledFractionOfInitialError`): psychophysical velocity-discrimination Weber fractions
are roughly 0.05-0.10 for deliberate discrimination of an isolated moving stimulus, and larger for a
perturbation superimposed on self-generated motion. 0.25 is the permissive end of that range. It is
not swept because `ReconcilerConfig` may not gain fields (task constraint), and I did not want a
silently-tuned constant.

## `ReconcilerConfig` fields read and ignored

| Field | Use |
|---|---|
| `ConvergencePositionToleranceMeters` | read — is an arriving sample worth correcting; has the correction landed |
| `ConvergenceOrientationToleranceRadians` | read — same, angular |
| `MaxTimeToConvergenceTicks` | read — sets `omega` (identically to `spring`) **and** the apparent-speed smoother's time constant |
| `MaxCorrectionLinearSpeedMetersPerSecond` | read — hard cap on offset linear speed, same semantics as `spring` |
| `MaxCorrectionAngularSpeedRadPerSecond` | read — hard cap on offset angular speed, same semantics as `spring` |
| `RollbackHistoryCapacity` | **ignored** — for `rollback`; nothing here rewinds |

Deliberately **no** new `ReconcilerConfig` fields, per the task constraint. Both new constants
(`MinimumTimeWarp`, `VelocityMaskingFraction`) are compile-time definitions with the same status as
`spring`'s settle fraction.

## Results

All numbers below are unit-scale measurements from the test suite, not a sweep. **No comparison
sweep was run and no ranking is claimed.** They are here because the falsifiers were stated in terms
of them, and because a judge should not have to rediscover the artifact in the last subsection.

Conditions, held fixed across both reconcilers: 100 Hz frames (10 ms), 100 ms convergence budget,
5 cm step correction, stationary operator, rate caps set far above the response so they never bind,
position tolerance 1 mm. Only the reconciler differs.

### The central claim — injected velocity (falsifier 4)

| | velocity-match | spring | ratio |
|---|---|---|---|
| peak injected speed | 0.3041 m/s | 1.1981 m/s | **0.254** |
| `time_to_convergence_ms` | 360 ms | 90 ms | 4.0x |

Injected speed is `d/dt(displayed - predicted)` — everything the operator sees that is not their own
motion. 0.254 against a predicted 0.25 (peak speed of a critically damped response scales linearly
with the natural frequency, and warping time by `w` scales the effective frequency by `w`). The cost
side is exactly as predicted and is not hidden: convergence is 4x slower. **Falsifier 4 not
triggered; falsifier 2 not triggered** — 360 ms is inside the stated 400 ms window, and, importantly,
*outside* the plain 100 ms budget, so the bound is not vacuous. Both halves are asserted.

### The masking rule is live, not inert (falsifier 5)

Convergence time as a multiple of `spring`'s on identical input, 5 mm correction:

| apparent speed | velocity-match / spring |
|---|---|
| 0 m/s | 3.80x |
| 0.3 m/s | 1.40x |
| 2 m/s | 1.00x |

At 2 m/s the displayed trajectory is `spring`'s to six decimal places — the warp saturates at 1 and
the two reconcilers coincide. **Falsifier 5 partially triggered, and this is the main caveat:** the
speed needed to reach `w = 1` scales with the correction's own demanded speed, so it is ~2 m/s for a
5 mm correction and utterly unreachable for a 5 cm one. Large corrections therefore start at the warp
floor, where `velocity-match` *is* `spring` with a 4x budget.

Two things stop that from collapsing the idea entirely, and both were discovered by tests failing:

1. **The warp rises as the correction shrinks.** It is a ratio against the correction's *current*
   demanded speed, so the tail of a large correction is absorbed at close to full rate even when the
   head was not. `velocity-match` is not a uniformly slower `spring`; it is slow where the correction
   is conspicuous and fast where it is not. Found when `Reset_RestoresTheAsConstructedState` failed
   because a 5 cm correction at 1.5 m/s had already converged in 20 frames.
2. **The masking rule is a function of live signal, not of config.** "`spring` with a 4x budget" is a
   fixed operating point; this is not.

Still, a judge should weigh this seriously: on a trace dominated by large corrections, this candidate
will be hard to distinguish from `spring` at a longer budget, and *that comparison should be in the
head-to-head* — `spring` at `ConvergenceBudgetMs: 400` is the control this candidate actually needs,
and it is the one I would add if I were designing the sweep.

### C1 continuity (falsifier 3)

Max per-frame velocity step under frame-interval refinement, 1 s budget:

| frame interval | max velocity step | ratio to previous |
|---|---|---|
| 8 ms | 1.073e-3 | — |
| 4 ms | 5.430e-4 | 0.506 |
| 2 ms | 2.738e-4 | 0.504 |

Clean `O(dt)`. **Falsifier 3 not triggered.** This carries more weight here than for `spring`: the
warp multiplies the visible velocity, so a warp that jumped between frames would be a velocity
discontinuity. It does not, because the apparent-speed estimate is exponentially smoothed with an
`O(dt)` per-frame change. The smoothing is load-bearing for continuity, not cosmetic.

### The artifact a judge must not step on — `jerk_mm_s3` percentiles

This is the most useful thing in this log and it is a *warning*, not a win. Same 600-frame window,
one correction, stationary operator, and **both reconcilers emit exactly one jerk sample per
advancing frame** (600 each — asserted, so this is not the unequal-sample-set failure):

| statistic | velocity-match | spring | direction |
|---|---|---|---|
| max | 6.2e5 | 7.2e6 | velocity-match ~12x **lower** |
| p99 | 1.0e5 | 4.6e5 | velocity-match ~4.5x **lower** |
| p95 | 4.0e3 | 3.7 | velocity-match ~1000x **higher** |
| p50 | 0 | 0 | tie |
| frames with jerk > 1 mm/s³ | 111 | 32 | — |

**p95 ranks these two by how long the correction lasted, not by how violent it was.** The correction
occupies 4x as many frames, so ~111 frames carry non-trivial jerk against ~32; at p95 of 600 samples
the cut falls at rank 570, inside `spring`'s tail of exact zeros and inside `velocity-match`'s
still-decaying tail. Every individual jerk value is smaller and the pooled p95 is 1000x larger.

Consequences I would want a judge to carry:

- Read **peak/p99 jerk together with `time_to_convergence_ms`**. A jerk percentile computed over a
  trace where corrections are sparse is partly a convergence-time metric wearing a jerk metric's
  name.
- The equal-cadence rule (`jerk_mm_s3` every advancing frame) is necessary but **not sufficient** for
  comparability. Equal sample-set *size* does not imply equal sample-set *composition*: here the
  proportion of post-convergence zeros is itself set by the thing under test.
- I did **not** change the metric and am not proposing to. `jerk_mm_s3`'s definition and cadence are
  shared across the axis (docs/metrics.md §5) and a candidate author is exactly the wrong person to
  redefine the ruler mid-investigation. Recording it and leaving it is the correct move.

This is pinned as a test,
`JerkPercentilesBelowP99RankByCorrectionDurationRatherThanSeverity`, which asserts *both*
directions — so if either flips, someone is told rather than left to rediscover it.

Secondary note: at 100 Hz with a 100 ms budget, `omega*dt` is about 0.66 for `spring`, so the
four-point difference cascade is sampling `spring`'s response near the edge of its resolution. That
is a property of the shared metric at this operating point, not of either implementation, but it is
another reason to treat these absolute jerk magnitudes as ordinal rather than physical.

### Where the design could flatter itself, stated plainly

- **A lingering offset buys low jerk without merit.** A reconciler that corrects more slowly will
  post lower peak jerk almost tautologically. The counterweight is `time_to_convergence_ms`, which
  this candidate makes 4x worse in the stationary case and reports honestly. Never read one without
  the other. Guarded by `TheResidualOffsetDecaysToZeroRatherThanParkingInsideTolerance`, which
  asserts the residual reaches < 1e-7 m rather than parking inside tolerance — so the "low jerk" is
  not being bought with a permanent offset.
- **The sweep's ~1.3 m opening transient will be maximally favourable to this candidate**, because a
  1.3 m correction sits deep in the warp-floor regime and gets the full 4x gentling. On the `lan`
  profile that single transient is known to dominate p95/p99. Any dramatic-looking `lan` result here
  should be assumed to be that transient until shown otherwise.
- **`correction_magnitude_mm`/`_deg` cannot differentiate this candidate**, by construction — they
  measure the predictor's disagreement before any reconciler acts. Asserted identical to `snap`'s.

## Verification

`just core-check` — all three gates green after the change:

- `dotnet test`: **418 passed**, 0 failed in `Teleop.Core.Tests` (384 before; +34 from this file),
  plus 28 / 3 / 36 in `RobotArm` / `Eval` / `RobotHost`.
- `verify: PASS` — golden log replays byte-identical.
- `audit: PASS` — no invariant violations, and the registry-completeness check accepts the new entry.

Test coverage, matching the required list: the stated convergence bound (both halves — inside the
window, outside the plain budget); C1 via frame-interval refinement; the central claim as a
comparison against `spring` under identical driving; velocity estimation under duplicate,
out-of-order and gapped observations; the predictor's own jump not being mistaken for apparent
motion; determinism; `Reconcile` idempotence including the speed estimate; `Reset()` including the
speed estimate; four `AllocationAssert.Zero` cases including the moving-operator path; metric
definition and cadence equality with `snap` and `spring`.

## Two test failures during development, and what they taught

Both were my assumptions being wrong, not the implementation. Recording them because they are the
kind of thing that silently becomes a tuned-away result:

1. `Reset_RestoresTheAsConstructedState` asserted a correction was still in flight after 20 frames at
   1.5 m/s. It had already converged — because the warp rises as the offset shrinks (see above). The
   test was narrowed to 5 frames and the behaviour documented, not the reconciler changed.
2. `ApparentMotionSpeedsTheCorrectionUp` asserted absolute convergence times fell monotonically with
   apparent speed. They did not: 70 ms at 0.3 m/s against 80 ms at 2 m/s. Cause is a **driver
   confound that affects every offset-carrying reconciler**: the offset is seeded from
   `lastDisplayed - predicted`, so one frame of unhidden operator motion lands in the initial offset
   — at 2 m/s and a 10 ms frame that is 20 mm, four times the 5 mm correction under test. A faster
   operator hands *both* reconcilers a larger correction. Fixed by measuring the ratio to `spring`
   under identical driving, which cancels it. **This confound is worth knowing for the head-to-head
   too**: any experiment that varies operator speed varies the effective correction size along with
   it.

## Left undone / for a human

- **`Reconciliation/CLAUDE.md` was deliberately not edited** (shared file, three agents merging).
  Whoever merges must move the `velocity-match` row from "Planned, not yet implemented" to
  "Implemented" with a note along the lines of: *critically damped offset on a time axis warped by
  apparent speed; converges within `MaxTimeToConvergenceTicks / 0.25`; injected speed bounded by
  `max(0.25 x spring's, 0.25 x apparent speed)`.* Until then the table is stale in the other
  direction from before — it under-claims rather than over-claims.
- **No `experiments/*.yaml` row was added** (shared, out of scope for this agent). The head-to-head
  YAML should include `none`/`snap` baselines, `spring`, and — my one design request — **`spring` at
  a 400 ms budget as the control**, because that is the null hypothesis this candidate has to beat.
- `VelocityMaskingFraction` (0.25) and `MinimumTimeWarp` (0.25) cannot be swept, because
  `ReconcilerConfig` may not gain fields under this task's constraints. If the axis owner wants them
  swept, that is a `ReconcilerConfig` change and a config-hash change, and it is a human call.
- **The directional variant was not built.** Perceptual direction discrimination (~1-2°) is sharper
  than speed discrimination (~5-10%), which argues for correcting *along* the operator's current
  motion axis and deferring the orthogonal component. That does not degenerate at large corrections
  the way the magnitude formulation does, so it is the obvious next candidate on this axis. Noted
  here rather than half-built.

## Progress

- [x] Seed self-check
- [x] Read `SpringReconciler.cs`, its tests, `DisplayedJerkEstimator`, `MotionMath`, `PoseMath`,
      `ReconcilerConfig`, `IReconciler`, `docs/metrics.md` §5
- [x] Hypothesis + falsifiers written before code
- [x] Implementation — `core/Teleop.Core/Reconciliation/VelocityMatchedReconciler.cs` (+ `.cs.meta`)
- [x] One-line registry entry in `Registries.Reconcilers`
- [x] Tests — 34 in `core/Teleop.Core.Tests/Reconciliation/VelocityMatchedReconcilerTests.cs`
- [x] `just core-check` green (418 Core tests, `verify: PASS`, `audit: PASS`)
- [ ] Head-to-head sweep — **deliberately not run**, per task: a later judge decides.
