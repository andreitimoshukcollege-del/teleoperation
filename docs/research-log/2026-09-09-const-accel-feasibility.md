# 2026-09-09 — feasibility assessment: `const-accel` / `ConstantAccelPredictor`

**This is an assessment, not an implementation.** Per task scope, no `.cs`, no test, no registry
entry, no experiment YAML, and no sweep were produced or run. The only file touched is this log.

## Seed self-check

| Check | Result |
|---|---|
| Worktree | `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-a1a39e4385bd9a647` |
| Branch | `worktree-agent-a1a39e4385bd9a647` |
| `git log --oneline -1` | `ce39dfb Merge pull request #22 from andreitimoshukcollege-del/research-documentation-docx` |
| `git rev-parse HEAD` | `ce39dfb3925ffa4fbc6d147bcffb2824589fe0d8` |
| Matches expected `main` sha (`ce39dfb`, per task) | yes, exactly |
| `git status` | clean |
| Gates run | none — task explicitly does not require it for an assessment; tree was already clean at `main`'s tip, so a stale-tree risk did not apply |

Read in full before writing anything below: root `CLAUDE.md`, `core/Teleop.Core/CLAUDE.md`,
`core/Teleop.Core/Prediction/CLAUDE.md`, `docs/research-log/CLAUDE.md`,
`Contracts/IPredictor.cs`, `Prediction/DoubleExponentialPredictor.cs` (house style reference),
`Prediction/ConstantVelocityPredictor.cs`, `Types/PredictorConfig.cs`,
`Types/PredictorDiagnostics.cs`, `Types/MotionMath.cs`, `Registry/Registries.cs`'s `Predictors`
table, `docs/metrics.md` §4–5, and `docs/research-log/2026-09-08-budget-blend.md` for log
structure and the "how this could flatter itself" convention.

## Feasibility verdict (written before the rest of the analysis)

**Yes, with caveats.** `const-accel` is expressible within `IPredictor<TState>` exactly as
written — the interface is state-shape-agnostic (`Observe`/`Predict`/`Reset`/`Diagnostics`, no
assumption about model order baked in), and `const-vel` and `double-exp` already demonstrate two
different internal state shapes (a fixed window vs. four recursive scalars) living behind the
same four methods. Second-order dead reckoning needs a third retained sample and a second finite
difference, both of which are ordinary object state, not a contract extension. No new
`Contracts/` file, no ADR, no `Pipeline/` change.

The caveats are all about `PredictorConfig`/robustness-policy *choices*, not about the interface:
one new config field is unavoidable if the estimator is to have an acceleration-specific clamp
distinct from `MaxLinearSpeed`/`MaxAngularSpeed` (see §2), and the ordering/gap/duplicate policy
needs a three-sample-aware design that neither existing predictor's policy transfers to
unmodified (see §3). Those are real design work, not blockers.

## 1. Verdict, restated for the record

**Yes-with-caveats.** Falls short of a clean "yes" only because a faithful implementation
plausibly wants a new `PredictorConfig` field (a shared-type cost, flagged per the task's
instruction rather than added), and falls well short of "needs an ADR" because nothing here
requires a new `Contracts/` interface, a `Pipeline/` change, or a wire-format change.

## 2. What it needs that does not exist yet

### `PredictorConfig` — field by field

| Field | Needed by const-accel? | Notes |
|---|---|---|
| `MaxHorizonTicks` | yes, reads it | same semantics as `const-vel`/`double-exp`: clamp the horizon on the future side only. No change needed. |
| `MaxObservationGapTicks` | yes, reads it | governs the gap policy (§3), but a three-sample estimator arguably needs it applied to *two* consecutive gaps (oldest-to-middle and middle-to-newest), not one — see below. |
| `HistoryCapacity` | yes, reads it, with a **stricter minimum** | `const-vel` enforces `HistoryCapacity >= 2` in its constructor because a first difference needs two samples. A second difference needs three: `MinimumHistoryCapacity` for `const-accel` must be `3`, enforced the same way (`ArgumentOutOfRangeException` at construction, not a silent no-op at `Predict` time — this repo's own precedent, see `DoubleExponentialPredictor`'s constructor guards). Existing field, no shape change, just a different constant inside the new file. |
| `SmoothingAlpha` / `SmoothingBeta` | no | ignored, same as `const-vel` — nothing here smooths; the rate/acceleration are raw finite differences. Document as ignored, per the struct's own convention. |
| `ProcessNoise` / `MeasurementNoise` | no | ignored — no noise model, so `PredictorDiagnostics.HasUncertainty` stays false. Same as both existing dead-reckoning predictors. |
| `MaxLinearSpeed` / `MaxAngularSpeed` | yes, reads it | clamps the estimated **velocity** term before extrapolation, same purpose as in `const-vel`. **Does not by itself bound the acceleration term** — see next row. |
| *(missing)* an acceleration-magnitude bound | **not present in `PredictorConfig` today** | This is the one plausible new field: `MaxLinearAcceleration` (m/s²) and, for the rotational side, `MaxAngularAcceleration` (rad/s²). Without it, a two-sample-derived *velocity* is bounded but the *second* difference (acceleration, derived from three samples two `dt`s apart) is unbounded, and it enters `Predict` multiplied by `dt²` — exactly the term the failure mode in §4 is about. Reusing `MaxLinearSpeed`/`MaxAngularSpeed` as an ad hoc acceleration bound would silently redefine what those fields mean (a speed bound repurposed as an accel bound), which the struct's own doc comment ("Upper bound on extrapolated linear speed") forecloses — so the honest options are (a) add two new fields, a real cost to a shared type used by every other predictor and every experiment YAML that constructs `PredictorConfig` positionally, or (b) derive an implicit accel bound from the existing speed bound divided by `MaxObservationGapTicks` (no new field, but a physically arbitrary derivation that ties acceleration limiting to an unrelated field's value). Neither is free; this is the real design cost of the row. |

`PredictorConfig`'s constructor takes nine positional arguments with no named-parameter
convention enforced, so adding two fields is a breaking change to every call site that constructs
it positionally (tests, `Teleop.Eval`'s experiment-YAML deserializer, and any other predictor
test file) — small mechanically, but real, and exactly why the row is "yes-with-caveats" rather
than an unqualified "yes."

### `PredictorDiagnostics` — field by field

Every field `const-accel` needs already exists and needs no change:

- `HorizonTicks` — same "post-clamp, from the newest observation" semantics as the other two.
- `LastObservationTicks`, `AcceptedObservations`, `RejectedObservations` — direct reuse.
- `HasUncertainty` / `PositionSigmaMeters` / `OrientationSigmaRadians` — `false`/`0`/`0`, same as
  `const-vel` and `double-exp`: a raw second difference is not a noise model. The existing
  four-argument constructor (`PredictorDiagnostics(long, long, int, int)`) is exactly the one
  `const-accel` would call. **No new diagnostics field is needed** — worth stating plainly since
  it is the one place I expected a gap and did not find one. An argument could be made for
  surfacing the estimated acceleration magnitude itself (useful for understanding *why* a
  prediction overshot), but that is a "nice to have," not something the contract demands, and
  `PredictorDiagnostics`'s own doc scopes it to "horizon actually extrapolated... and, where
  meaningful, an uncertainty estimate" — an acceleration readout is neither.

### `MotionMath` — rotational side

This is where the real new math is, and it is not present today. `MotionMath` currently offers
exactly four operations: `ClampMagnitude`, `ToRotationVector` (log map), `FromRotationVector`
(exp map), `IntegrateWorld`, and `RelativeRotationVector`. Every one of these treats a rotation
rate as a single axis-angle vector taken between two orientations over one `dt` — there is no
concept of an angular *acceleration* anywhere in the file, and no operation that would produce
one.

A `const-accel` rotational term needs the rotational analogue of "second derivative of position,"
which is qualitatively harder than the translational case for a reason worth stating precisely:
position lives in a vector space, so a second finite difference of positions is just another
vector operation (subtract two velocity vectors, divide by `dt`). Orientation does not live in a
vector space — it lives on SO(3) — so "the rate of change of the angular rate vector" is only
well-defined once you have fixed a frame to differentiate the rate *in*. `MotionMath`'s own doc
is explicit that `IntegrateWorld`/`RelativeRotationVector` chose the **world frame** deliberately,
to match `Plant/RigidBodyPlant.Step`. The natural extension — three consecutive angular-rate
vectors (each itself already a world-frame rate between a pair of samples), finite-differenced
again — is consistent with that same choice only if the difference is taken as an ordinary vector
subtraction of two world-frame rate vectors, which is legitimate (both rates already live in the
same fixed frame, unlike orientations themselves) but is a new claim nobody has written down or
tested here, and it degrades exactly where `RotationEpsilon` already warns near-identity
rotations degrade: two rate vectors that are each individually well-conditioned can still have a
poorly conditioned *difference* if the underlying rotation is nearly constant, though that is a
much milder condition than the near-zero-rotation singularity `ToRotationVector` guards.

Concretely, this needs one new static helper — call it `AngularAccelerationVector` or similar,
taking two already-computed angular-rate vectors and a `dt`, dividing their (ordinary Euclidean)
difference by `dt` — which is a small, honest addition to `MotionMath` (a shared file, so also a
cost, but a much smaller one than touching `PredictorConfig`). It needs its own unit tests
(constant angular velocity ⇒ zero acceleration; constant angular acceleration ⇒ recovers it
exactly; near-identity/near-zero-rate degenerate cases guarded the same way `RotationEpsilon`
guards the log map) before any predictor is allowed to depend on it, per the repo's own
"two predictors that each rolled their own log map would usually agree and differ in the
corners" rationale for why this math lives in one shared place at all.

## 3. The three robustness clauses, for a three-sample estimator

`Prediction/CLAUDE.md` requirement 2 and `IPredictor.Observe`'s own doc both demand: robust to
out-of-order arrival, robust to duplicate stamps, robust to gaps of several hundred ms. `const-vel`
answers all three with one mechanism (a stamp-ordered window, reinsertion up to
`HistoryCapacity` deep, rate always recomputed from the two *newest* retained samples only).
`double-exp` answers all three by rejecting reordering outright (no window, no reinsertion — a
late sample is just discarded) and re-seeding across a gap.

**Estimating acceleration needs three samples, not two — this is the load-bearing fact for every
one of the three clauses, not a footnote.** A second finite difference is `(v1 - v0) / dt`, where
`v1` itself is `(p2 - p1) / dt1` and `v0` is `(p1 - p0) / dt0`. So acceleration is a function of
three consecutive stamps, not two — one more piece of state than `const-vel` carries, and the
window/reinsertion machinery inherits an extra layer of consequence:

- **Out-of-order stamps.** `const-vel`'s policy — keep a window, reinsert a late sample in stamp
  order, recompute the rate from whichever two entries end up newest after the insert — does
  *not* transfer unmodified. A late sample reinserted into the *middle* of a three-sample window
  changes which triple of consecutive samples produces the acceleration estimate, and unlike
  `const-vel`'s documented "restrict to the newest pair so a mid-window insert cannot retroactively
  change the current estimate," a three-sample derivative computed from the newest **three**
  retained entries is not similarly insulated: inserting one sample between the current two newest
  changes the middle-triple entirely, not just extends it. The honest policy is therefore one of:
  (a) restrict acceleration, like `const-vel` restricts velocity, to the newest three *retained*
  entries recomputed after every accepted insert (accepting that a mid-window insert *does* change
  the current acceleration estimate, which is a real behavioral difference from `const-vel` worth
  documenting explicitly rather than silently inheriting the comment that says it doesn't), or
  (b) reject reordering outright like `double-exp` does, which is simpler to reason about but
  gives up the reordering tolerance the axis's own "const-vel keeps a window and can splice"
  framing treats as a virtue. Given the task's framing explicitly contrasts "const-vel keeps a
  window and can splice" against "double-exp rejects outright" and asks which `const-accel`
  needs: **it needs a documented middle ground, not a straight copy of either** — a window (so a
  moderately late sample is not simply discarded, since a second derivative is already noise-
  amplifying and throwing away data would only worsen its variance), but with an explicit note that
  the *currently active triple* changes on any mid-window insert, which `const-vel`'s
  two-sample case is specifically documented as being immune to and a three-sample case is not.

- **Duplicate stamps.** Mechanically identical to both existing predictors: a stamp equal to one
  already retained is rejected whole, counted in `RejectedObservations`, state unchanged. Nothing
  new here — `Observe` called twice with an identical sample must leave state unchanged per the
  interface's own contract, and duplicate rejection is the same one-line check either predictor
  already uses (`captureTicks <= _history[0].CaptureTicks` guard, or an equality scan).

- **Gaps of several hundred ms.** This is where the extra sample matters most. With two
  consecutive gaps to check (`t1 - t0` and `t2 - t1`) rather than one, the policy has to decide
  what "a gap" means for a three-point estimator: is it a gap in *either* of the two intervals, or
  only the newest one? A gap in the *older* interval only (`t1 - t0` large, `t2 - t1` normal) still
  poisons the acceleration estimate, because `(v1 - v0)/dt` differences a `v0` that was itself
  computed across the stale interval — so the correct policy is **collapse both velocity and
  acceleration to zero (degrade to constant-position, then to constant-velocity as data arrives) if
  *either* interval exceeds `MaxObservationGapTicks`**, not just the newest one. This is stricter
  than `const-vel`'s single-interval check and is a real, non-obvious design decision that a naive
  port of `const-vel`'s gap check (which only ever looks at the newest pair) would get wrong
  silently — precisely the "produces garbage rather than fail" failure mode requirement 2 warns
  about.

**What this implies for state, warm-up, and `Reset()`:**

- **State.** Three retained `Stamped<Pose>` entries minimum (so `HistoryCapacity`'s effective
  floor is 3, not 2), plus derived velocity and acceleration terms (both linear and, per §2's
  `MotionMath` gap, angular) recomputed after every accepted `Observe`. This is one array slot and
  one extra derived-quantity pair beyond `const-vel`'s state — mechanically small, but the
  recomputation logic is genuinely more involved than `const-vel`'s (which recomputes one rate from
  one pair; `const-accel` recomputes two rates from two pairs, then differences them).
- **Warm-up.** With fewer than three samples there is no second difference to take. The honest
  degradation ladder, matching this repo's existing convention of degrading gracefully rather than
  throwing (`const-vel` returns `Pose.Identity` with zero observations, and zero rate with fewer
  than two): zero observations ⇒ `Pose.Identity`; exactly one ⇒ passthrough of that sample
  (zero velocity, zero acceleration); exactly two ⇒ first-order only (behaves like `const-vel` — a
  velocity but no acceleration term, since accel needs the third sample); three or more ⇒ full
  second-order behavior. That is a three-rung ladder where `const-vel` has a two-rung one, and
  each rung needs its own test, which is real additional cost (see §6) but not a design risk —
  it is a direct, mechanical consequence of "three samples, not two," it does not need new contract
  surface, and it is exactly the kind of thing a predictor is expected to degrade through per
  requirement 2 rather than throw on.
- **`Reset()`.** No new concern beyond what `const-vel`'s `Reset()` already establishes: clear the
  count/array-validity marker, zero both derived rates (velocity and, newly, acceleration, both
  channels), zero the horizon and counters. The array itself survives (capacity is configuration),
  matching `const-vel`'s stated rationale exactly. A dedicated `Reset()` test proving the
  as-constructed state is reached (this repo's own stated requirement for every stateful
  component) is unchanged in kind from the existing two predictors' equivalent test, just checking
  one more zeroed field.

## 4. The known failure mode, quantified

The axis table's one-line claim — "overshoots on direction reversal" — has a clean analytic shape.
For a second-order (constant-acceleration) extrapolation, the predicted displacement at horizon
`Δ` from the newest sample is

```
Δp(Δ) = v·Δ + ½·a·Δ²
```

against `const-vel`'s `v·Δ` and `double-exp`'s filtered-trend equivalent. The **excess** term over
first-order dead reckoning is exactly `½·a·Δ²` — quadratic in the horizon, and its *sign* is set
by whatever acceleration the last two `dt` intervals happened to measure. On a direction reversal
(operator decelerating and reversing — the human-motion case this axis exists to serve, per
`Prediction/CLAUDE.md`'s "operator-side" framing), the two most recent samples measure a strongly
negative acceleration relative to the direction of travel *before* the reversal actually happens or
just as it starts, so `a` is large and pointed the "correct" way for a beat that has not fully
arrived — and the quadratic term extrapolates that deceleration two, three, four multiples of `Δ`
past where the reversal will actually settle, precisely because nothing in the model knows the
deceleration itself is not constant (real human deceleration into a reversal is not constant — it
eases in and out, so a constant-acceleration model fit to the steepest part of it overshoots by
construction, not as an edge case).

**Where this dominates, at the four benchmark horizons (50/100/200/400 ms):** the excess term
scales as `Δ²`, so relative to `const-vel`'s linear term (`v·Δ`) the ratio of excess-to-linear
displacement is `(a·Δ)/(2v)` — it grows *linearly* in `Δ`, not quadratically, once normalized
against the first-order term it is competing with. Concretely: doubling the horizon from 100 ms to
200 ms doubles the ratio of the acceleration-driven excess to the velocity-driven baseline term,
and doubling again from 200 ms to 400 ms doubles it again — so **whatever headroom const-accel has
over const-vel at 50 ms (the shortest benchmark horizon, where `Δ` is small and the quadratic term
is a small correction) is the same headroom that becomes the dominant *liability* by 400 ms (the
longest benchmark horizon, where the same `Δ²` growth that helped at 50 ms turns the deceleration
term into a lurch several multiples the size of the velocity term alone), with 100/200 ms as the
transition band** — the exact crossover point is a function of the ratio `a/v` for the trace in
question (bigger for fast, sharply-decelerating hand motion; smaller for slow, gentle robot
dynamics), which is precisely why this needs a measurement (§5) rather than a closed-form horizon
number: the analytic argument only establishes the *shape* (linear growth in the relative excess,
monotonically worsening with horizon), not the crossover's absolute location, and the two research
directions this contract explicitly separates — operator-side human motion vs. robot-side command
prediction — plausibly land that crossover at different horizons because their `a/v` statistics
differ.

## 5. What would falsify "worth building," cheaply, before writing the file

The cheapest kill test does not require implementing `const-accel` at all: it requires **no new
code**, only a `MotionMath`-level or even spreadsheet-level analysis against a `.tlog` already
committed in `core/testdata/traces/`. Compute, per profile:

1. From each committed trace, take every consecutive triple of ground-truth samples and compute
   the second finite difference (the exact `a` term `const-accel` would compute).
2. For each of the four benchmark horizons Δ, compute the candidate excess term `½·a·Δ²` at every
   trace point and compare its magnitude to the *actual* ground-truth position change over that
   same Δ (available from the trace itself, since these are recorded, not live).
3. **Falsifier:** if the sign of `a` (estimated from the two most recent samples, exactly as
   `const-accel` would see it in real time) agrees with the sign of the true displacement's
   curvature *less than half the time* — i.e. no better than chance — at horizons ≥200 ms, across
   the operator-motion traces, then the quadratic term is adding noise, not signal, before a single
   line of `ConstantAccelPredictor.cs` is written, and the idea is dead on arrival for those
   horizons without needing a benchmark run, a registry entry, or a test suite.

This is cheap specifically because it reuses ground truth already committed to the repository and
needs no new predictor, no `PredictorConfig` change, and no gate — it is pure post-hoc arithmetic
over an existing `.tlog`, exactly the kind of "run the experiment that could kill the idea first"
the operating contract asks for. It was **not run** in this assessment (out of scope — no code, no
computation beyond reading files, per the hard scope), but it is the concrete first move for
whoever picks this row up next, and it should be run *before* the `MotionMath` helper or the new
config field are written, since a negative result there kills both of those costs along with the
predictor itself.

## 6. Estimated cost to build

**Files touched**, using `budget-blend`'s actual file list as the size precedent for a
comparable-complexity new predictor in this axis:

| File | Change | Rough size, by analogy |
|---|---|---|
| `core/Teleop.Core/Prediction/ConstantAccelPredictor.cs` | new | `const-vel` is 387 lines including a dense doc comment; a second-order estimator with an extra retained sample, two extra derived-rate channels (linear + angular acceleration), a three-rung warm-up ladder, and the dual-gap check in §3 is plausibly larger, not smaller — estimate 450–550 lines in this repo's documentation density. |
| `core/Teleop.Core/Prediction/ConstantAccelPredictor.cs.meta` | new | Unity package metadata; mechanical, a fresh GUID. |
| `core/Teleop.Core/Types/MotionMath.cs` | modified | one new static method (`AngularAccelerationVector` or similar, §2) plus doc — a shared file, so a small but real cost with a wider blast radius than a new file (every predictor and reconciler that imports `MotionMath` recompiles). |
| `core/Teleop.Core/Types/PredictorConfig.cs` | modified, **if** the acceleration-bound field is added rather than derived (§2) | two new `readonly float` fields plus a constructor-signature change — the more expensive shared-type cost, and the one genuinely uncertain item on this list (see below). |
| `core/Teleop.Core/Registry/Registries.cs` | modified | exactly one line in the `Predictors` table, per `/new-impl` discipline and the `budget-blend` precedent (`git diff --stat` was "1 file changed, 1 insertion(+)" for that entry). |
| `core/Teleop.Core.Tests/Prediction/ConstantAccelPredictorTests.cs` | new | `const-vel`'s test file is 573 lines; `double-exp`'s is 697. A three-sample estimator with a three-rung warm-up ladder and a dual-interval gap check needs strictly more test rows than either, not fewer — estimate 600–750 lines. |
| `core/Teleop.Core/Types/MotionMathTests.cs` (or wherever `MotionMath` is tested today) | modified | new tests for the new helper, in isolation from the predictor — constant angular velocity ⇒ zero accel; constant angular accel ⇒ recovered exactly; near-identity degenerate case. |

**Roughly what tests would be needed, and what each proves** (extending, not duplicating,
`const-vel`'s existing 573-line suite's structure):

- Warm-up ladder (3 tests): 0/1/2 samples produce identity / passthrough / first-order-only
  behavior, proving the degradation ladder in §3 rather than a throw.
- Exact recovery (2 tests): fed synthetic constant-acceleration position/rotation data, `Predict`
  at any horizon matches the closed-form `p0 + v0·Δ + ½a·Δ²` (and its rotational analogue) to
  floating-point tolerance — the equivalent of `const-vel`'s implicit "constant velocity is a fixed
  point" property, made explicit here because it is the entire justification for the model.
- Direction-reversal overshoot (1–2 tests, **descriptive, not a pass/fail gate on "less accurate
  than const-vel"** per the repo's own rule against a gate that manufactures a predetermined
  verdict): quantify the overshoot magnitude on a synthetic reversal at each of the four benchmark
  horizons, matching the analytic shape in §4 — this is the test that would surface whether the
  crossover predicted in §4 actually lands where the arithmetic suggests, on synthetic data, before
  touching a real trace.
- Out-of-order / reinsertion (3–4 tests): late sample inside/outside the window, effect on the
  *current triple* (proving the §3 middle-ground policy, and specifically that it is documented as
  behaving differently from `const-vel`'s two-sample insulation, not silently inheriting that
  claim).
- Duplicate rejection (1 test): mechanically identical to `const-vel`'s.
- Gap policy, both intervals (2–3 tests): gap in the newer interval only, gap in the older interval
  only, gap in both — proving the "either interval" rule in §3, which is the one place a naive
  const-vel port would silently get wrong.
- Clamping (2 tests, mirroring `DoubleExponentialPredictor`'s documented trap): a very small `dt`
  between two accepted observations must not let an ordinary position delta become a huge
  instantaneous rate that compounds through the *second* difference — this predictor divides by
  `dt` twice (once for velocity, once for acceleration), so it is **strictly more exposed** to the
  bug `double-exp`'s doc comment documents diverging to NaN/infinity under a bursty trace-driven
  profile. The clamp needs to be applied at both derivative stages, in `Observe`, not only in
  `Predict` — matching `double-exp`'s precedent exactly, doubled.
- Determinism / idempotence / `Reset()` (3 tests): identical in kind to both existing predictors'
  equivalents.
- Allocation assertions (2–3 `AllocationAssert.Zero` tests): `Observe` and `Predict` on the hot
  path, matching the existing convention.

Total estimate: roughly **20–26 tests**, comparable in count to `const-vel`'s and `double-exp`'s
existing suites, but with a higher proportion of genuinely new cases (the three-rung warm-up
ladder and the dual-gap check have no direct analogue in either existing file) rather than
mechanical repeats.

**What is genuinely uncertain**, stated plainly rather than papered over:

1. **Whether the acceleration bound needs a new `PredictorConfig` field at all**, or whether an
   implicit derivation (accel bound = speed bound / gap-ticks, or similar) is acceptable. This is
   the single largest cost-driver on the list and I do not think it can be resolved by analysis
   alone — it is a design call about how much a shared, widely-used type should grow for one
   predictor, and reasonable people could land either way. Flagging it rather than deciding it is
   itself the point of an assessment.
2. **Whether the §3 "either interval" gap policy is even the right one**, versus a simpler (and
   possibly equally defensible) policy that only checks the newest interval and accepts that a
   stale older sample silently degrades the acceleration estimate rather than collapsing it to
   zero. I believe the stricter policy is correct on the "produce garbage silently" argument the
   contract itself makes, but it is a judgment call, not a derivation, and it changes the size of
   the test suite (§6's gap-policy tests assume the stricter reading).
3. **Whether the crossover horizon in §4 is 100 ms, 200 ms, or somewhere else** for this project's
   actual traces — the analytic argument establishes only the *shape*, and §5's falsifier is
   exactly the cheap way to find out before paying any of the costs above. This is the most
   important uncertainty of the six, because it bears directly on whether the row is worth its
   cost at all, and it is answerable without writing the predictor.

## Left undone / for a human

- The §5 falsifier measurement itself — cheap, concrete, and deliberately not run here (out of
  scope for an assessment that writes no code and runs no computation beyond reading committed
  files). This is the recommended first action for whoever next picks up this row.
- The `PredictorConfig` field-vs.-implicit-derivation decision (uncertainty 1 above) is a genuine
  fork that this assessment does not resolve, on purpose — it is a design call, not a feasibility
  question, and belongs to whoever actually implements the row.
- Nothing here is blocked on `unity/`, on hardware, or on a human for *feasibility* reasons — the
  verdict is unconditional (yes-with-caveats). What is deferred to a human/implementer is the two
  design forks above, and the measurement in §5.

## Verdict, restated

**Yes, with caveats.** `const-accel` fits `IPredictor<TState>` as written, needs no contract or
architecture change, and needs no ADR. It plausibly needs one new pair of fields on
`PredictorConfig` (a real but small cost to a shared type) and definitely needs one new
`MotionMath` helper for angular acceleration (small, shared, needs its own tests before any
predictor depends on it). Its ordering/gap/duplicate policy cannot be a straight copy of either
existing predictor's — it needs a documented middle ground driven by the fact that acceleration is
a three-sample, not two-sample, quantity. Its known failure mode (direction-reversal overshoot) has
a clean analytic shape (a `Δ²`-growing excess term whose *relative* contribution grows linearly
with horizon), and the crossover horizon where it starts to dominate is an empirical question this
assessment does not answer, deliberately, because §5 gives a cheap way to answer it first — before
paying for the `MotionMath` change, the `PredictorConfig` change, or the ~20–26 tests estimated in
§6.
