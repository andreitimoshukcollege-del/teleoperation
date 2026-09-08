# 2026-09-08 — Reconciliation axis: decision record

**Question.** `Reconciliation/` had one implementation — `snap`, the deliberate worst case — so the
axis its own `CLAUDE.md` calls "the most underestimated in the project" had nothing to compare
against. What should be built, and what does each approach cost?

**Candidate logs:** `2026-09-08-{budget-blend,velocity-match,rollback}.md`.
**Runs:** `results/exp-003-reconciler-comparison/`, `results/exp-004-reconciler-head-to-head/`.
Nothing is tagged, so nothing from this run is citable.

## How it was decomposed, and why

Started as one unattended agent working the axis sequentially. Re-scoped to **three agents in
parallel, one candidate each, followed by a judging step** — sequential meant a single window spent
across three ideas, and no head-to-head was possible because one agent scoring its own work is not
evidence. Each researcher was told explicitly not to run a comparison sweep and not to self-rank.

Every researcher got an identical, narrow file scope (its own implementation, its own test, one line
in `Registries.cs`, its own log) and was forbidden the shared files. That kept a three-way merge down
to a single dictionary literal.

## Approaches considered and not pursued

- **`exp-smooth`** — deliberately not built, and the one genuinely open decision this run hands
  back. A single-time-constant lag is C0 but not C1: its offset velocity steps from zero at
  correction onset. It cannot satisfy `Reconciliation/CLAUDE.md` requirement 2 as written, so
  building it first needs that requirement amended to cover it as a second quantified exception the
  way `snap` is. That is a human's call, not an agent's.
- **A second variant of `spring`** (different damping, different envelope) — collapsed into one
  candidate. Variants of one approach are not competing approaches; if no measurement can
  distinguish two candidates, running both spends two agents to learn nothing.
- **Working `Buffering/` in parallel** — rejected. It has zero implementations *and* `IPlayoutPolicy`
  is not wired into `Pipeline/` at all; `OperatorEndpoint` hardcodes `t_playout = t_operatorRecv`.
  Teaching `Pipeline/` a new contract is an architecture change that needs an ADR first, which is
  not unattended work.
- **Working `Autonomy/` in parallel** — rejected on a stronger ground: arbiter studies are
  closed-loop, so a recorded `.tlog` cannot serve as ground truth and results scored open-loop are
  invalid by that folder's own rule. An agent could write arbiters but could not produce a result.
- **A warm-up discard in the sweep**, to remove the ~1.3 m opening transient every trial carries —
  considered and deliberately not done. It would shift `exp-001`'s recorded numbers as well, so it is
  flagged in the affected experiment rather than silently applied.

## Verdicts

- **`spring`** — built earlier the same day; critically damped decay of a residual offset. Became the
  reference implementation the three parallel candidates were written against.
- **`budget-blend`** — built. Quintic-Hermite blend, residual bit-exactly zero at a deadline fixed at
  seed time. The hard-deadline counterpart to `spring`'s asymptotic envelope.
- **`velocity-match`** — built. `spring`'s dynamics on a warped time axis, slowing the correction in
  proportion to apparent operator motion. Lowest jerk, highest convergence cost.
- **`rollback`** — **rejected on argument**, without being built. `IReconciler<Pose>` supplies only
  an anchor: no command stream, no plant. The only reading that fits the contract telescopes, so the
  history depth provably cannot affect the output. The useful part is structural — the predictor
  already performs the rollback, so a reconciler doing a second one from less information cannot
  win. It is operator-side client prediction, not a reconciler. Reviving it needs an ADR.
- **Removing the one-frame display hold** — **rejected on measurement.** See below.

## Course changes mid-run

1. **The first unattended run was seeded from the wrong commit** and killed. Worktree agents default
   to branching from `origin/<default>`, so a local-only branch is invisible to them; committing
   locally does not fix it. Set `worktree.baseRef: "head"` and added a seed self-check as every
   agent's first action, so this fails in seconds rather than silently researching the wrong tree.
2. **The axis was not measurable at all when the run began.** `SweepCommand` never called
   `EstimateRobotState`, the only caller of `Reconcile`, so no sweep had ever emitted `jerk_mm_s3`
   or `time_to_convergence_ms` — and the one reconciler metric that *was* emitted is identical
   across reconcilers by construction. Fixed before any comparison could mean anything.
3. **Jerk emission cadence changed** from once-per-correction to every-advancing-frame, a baseline
   change, with `docs/metrics.md` updated in the same commit. Done deliberately at a moment when
   `results/` held no run output, so no committed figure's comparability was at stake. It is the
   reason numbers recorded before this run are not comparable to numbers after it.
4. **The one-frame hold was fixed, measured, and reverted.** It is a real C1 violation, and the
   natural fix made things worse: the disagreement is measured at capture time while the predictor's
   jump lands at now, so the mismatch leaks a residual step onto every correction frame. It improved
   jerk where corrections are rare and regressed it badly where they are frequent, collapsing the
   spread between candidates to ~1.01x — the metric stopped measuring the convergence law. Kept as a
   documented tradeoff, pinned by tests so it cannot be silently "fixed" again.

## Left open

- **`exp-smooth`'s requirement-2 amendment** — the one decision blocked on a human.
- **Rate-compensated seeding** — the fix that would remove the one-frame hold properly, by advancing
  the displayed pose by the predictor's estimated rate before seeding, keeping exact cancellation
  *and* motion continuity. `velocity-match` already builds such an estimator, so it is tractable.
- **A warm-up discard**, if the opening transient is to stop dominating low-correction profiles.
- **An ADR for rollback**, if operator-side client prediction is wanted.
- **Nothing here is citable.** No tag was made and `results/` is gitignored, so the numbers exist
  only on the machine that produced them.
