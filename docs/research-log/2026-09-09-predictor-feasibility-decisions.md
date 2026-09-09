# 2026-09-09 — decision record: feasibility of `const-accel` and `ekf`

**Organizer:** research-organizer. **Run type:** assessment only — deliberately scoped as a
pipeline smoke test, not a research run. No implementation, no sweep, no judge panel.
**HEAD for the whole run:** `ce39dfb` (`main`). Both candidate worktrees branched from it and
both reported that sha back independently.

## The question

Are `const-accel` and `ekf` — two rows in `Prediction/CLAUDE.md`'s "Planned, not yet
implemented" table — expressible within `Contracts/IPredictor.cs` as it stands, and what would
each cost to build?

## How it was decomposed, and why

One researcher per predictor, in parallel, isolated worktrees. The two are genuinely different
approaches rather than variants: `const-accel` is a deterministic finite-difference extrapolator
whose entire output is a pose, while `ekf` is a stochastic filter whose distinguishing output is
a *distribution*. The measurement that distinguishes them was named in advance and it is not a
metric: whether the existing shared types (`PredictorConfig`, `PredictorDiagnostics`) and the
existing consumers suffice unchanged. `const-accel` was expected to strain `PredictorConfig`;
`ekf` was expected to strain everything downstream of `Diagnostics`. Both expectations were
borne out, which is why the split was the right one.

The `ekf` brief deliberately front-loaded the plumbing question over the filter mathematics. An
EKF's algorithmic feasibility was never seriously in doubt; whether anything in this repo can
*observe* its advantage was the open question, and it is the one that decided the verdict.

## Approaches considered and not pursued

- **A third candidate, `seq-model`.** The remaining planned row. Not pursued: it depends on
  `IInferenceBackend` and an ONNX export path that is a Phase-7 concern, so its feasibility is
  gated on a decision that has not been made yet. Assessing it now would produce an answer that
  expires.
- **Splitting `ekf` into two candidates — filter feasibility and plumbing feasibility.** Rejected
  during decomposition: as assessments the two would read the same files and reach the same
  desk, and no measurement could distinguish their outputs. Collapsing them into one brief with
  the plumbing question weighted heaviest was the cheaper equivalent.
- **Decomposing by the axis's own operator-side / robot-side split** rather than by predictor.
  That split is real and `Prediction/CLAUDE.md` insists on it for *benchmarking*, but it
  partitions the evaluation, not the feasibility question. Both predictors face the same
  interface either way.
- **Building a throwaway prototype of each to measure allocation empirically.** Rejected: it
  violates the run's assessment-only scope, and the allocation question is answerable by reading
  the contract and `DoubleExponentialPredictor` — which is what both researchers did.
- **Running the judge panel.** Rejected, and worth recording because it is the pipeline's normal
  next step. Admissibility judges an implementation against its contract, methodology judges a
  sweep, and verdict ranks candidates. There is no implementation and no sweep here, so all three
  would have been asked to score nothing. A judge with nothing to judge produces noise that later
  readers mistake for evidence.
- **Letting the researchers file the `Prediction/CLAUDE.md` row updates themselves.** Rejected:
  the axis `CLAUDE.md` is a shared file and two agents editing it in parallel is the exact
  conflict this pipeline's file-scoping exists to prevent. Neither row moves anyway — nothing was
  built.

## Verdicts

Neither candidate was built, by design; both were asked for an argument and both delivered one.

- **`const-accel` — feasible, yes-with-caveats.** Fits the interface unchanged; no ADR, no new
  `Contracts/` file, no `Pipeline/` change. The cost is concentrated in one place nobody would
  guess from the row title: there is no acceleration bound in `PredictorConfig`, and the two
  honest ways to get one (add fields to a shared positional struct, or derive a bound from an
  unrelated field) are both real costs. Detail, including the three-sample ordering and gap
  policy that neither existing predictor's policy transfers to:
  `docs/research-log/2026-09-09-const-accel-feasibility.md`.
- **`ekf` — feasible as a filter, but not a self-contained unit of work.** The finding that
  matters is negative and I verified it myself rather than take it on report: all four
  reconcilers discard the diagnostics they are handed, nothing outside `Types/` reads the
  uncertainty fields at all, and `docs/metrics.md` defines no metric that would capture
  uncertainty. The covariance an EKF exists to produce is currently unobservable. Building it
  first would produce a predictor whose one distinguishing output nothing consumes and no sweep
  can score. Detail: `docs/research-log/2026-09-09-ekf-feasibility.md`.

The two verdicts are not a ranking — nothing was measured, and no ranking is claimed. They do
imply an ordering of work, recorded below as an open question rather than a decision, because
choosing it is a human's call.

## Course changes

None. The run executed as decomposed.

## Left open

- **The consumer-before-filter question.** `ekf`'s researcher proposes building the consumer
  first: give one reconciler a genuine uncertainty-scaled branch, feed it a hand-constructed
  `PredictorDiagnostics` in a unit test, and see whether varying the injected sigma moves
  correction cost at all — no filter required. If it does not move, `ekf` is dead regardless of
  filter quality. Whether that consumer is an ordinary implementation or an ADR-level change to
  `Contracts/` is unresolved and is the first thing a human should decide.
- **Whether `PredictorDiagnostics`' two isotropic scalars are an acceptable summary of a
  covariance.** Lossy by construction. Currently moot, since there is no consumer; it stops being
  moot the moment one exists.
- **Cross-compiler float determinism for an iterative matrix filter.** Flagged as materially
  higher risk for an EKF than for a four-scalar recursion. Unresolvable here — it needs an IL2CPP
  build on the Windows box.
- **Two `const-accel` design forks**: the acceleration-bound question above, and how strict the
  dual-interval gap policy should be. Both are implementer's choices, neither blocks.
