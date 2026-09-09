# Decision record — is `exp-smooth` worth a second C1 exception?

**Run:** 2026-09-08 to 2026-09-09. **Organizer:** research-organizer.
**Base:** `76878bf`, left uncommitted.
**Results:** `results/exp-005-exp-smooth-c1-tradeoff/20260909-025400Z/`
**Config:** `experiments/exp-005-exp-smooth-c1-tradeoff.yaml`

## The question

`Reconciliation/CLAUDE.md` has carried `exp-smooth` in "Planned, not yet implemented" with a
blocking note: a single-time-constant lag is C0 but not C1, so building it appeared to require
requirement 2 to gain a second deliberate exception alongside `snap`. The axis had never resolved
that, and it is why `spring` was built first.

The question was deliberately *not* "does exp-smooth work". It was: **is exp-smooth's simplicity
worth a second documented exception, or does a C1-preserving variant of the same idea get the same
benefit for free?** That framing decided the decomposition, because it makes the run a comparison
between an exception and its avoidance rather than a search for the lowest jerk.

## How it was decomposed, and why

Three candidates, one researcher each, in separate worktrees, chosen so that a *measurement* could
distinguish every pair:

1. **`exp-smooth`** — the literal single-pole offset lag from the planned row. Its job was to make
   the violation measured rather than asserted, in the style `snap`'s suite already uses.
2. **`exp-smooth-c1`** — a C1-preserving law keeping the one-time-constant shape but leaving onset
   at zero offset velocity. This is the candidate that answers the question: if it matches
   `exp-smooth`, the exception buys nothing.
3. **`exp-smooth-track`** — the no-offset tracking lag, i.e. the "simple, **biased**" reading the
   planned row's own wording suggests. Included as the strongest form of "simple", to find out what
   the minimum costs.

`spring` was the required incumbent reference and `snap` the baseline; neither was rebuilt.

Candidate 2 was briefed to write its differentiation argument **before** implementing and to stop if
its law collapsed to `spring`, since the critically damped envelope is itself "an exponential shaped
to leave the origin at zero velocity". It did not collapse, and it said why in advance.

## Approaches considered and not pursued

- **Amending requirement 2 to admit `exp-smooth`, as the planned row proposed.** Out of scope by
  construction: changing a documented requirement to admit a candidate is weakening a gate. The run
  produces evidence; the amendment is a human's decision. Every brief forbade it explicitly.
- **Adding a displayed-pose-accuracy metric.** The run discovered that nothing on this axis measures
  the reconciled pose against ground truth (see "Left open"). Adding one mid-run would have redefined
  the instrument and destroyed comparability with `exp-004`, which this run otherwise preserves
  byte-for-byte. Recorded as a finding instead.
- **Fixing the shared one-frame display hold.** Closed by the axis's "Tried and rejected" section on
  measurement. Every brief named it and told the researcher to inherit it, specifically so no
  candidate got a one-sided advantage in the head-to-head.
- **A fourth candidate: exponential decay restarted from rest at each retarget.** Assessed during
  decomposition and dropped. Restarting from rest while an in-flight offset has non-zero velocity is
  itself a velocity step, so it would have been a *worse* C1 violation than `exp-smooth` while
  looking like a fix. Folded into candidate 2's brief as a requirement to prove continuity at
  retarget, not just at onset.
- **Sweeping `budget-blend` and `velocity-match` alongside.** Dropped to keep the head-to-head
  focused on the exception question. They are already characterised in `exp-004`.
- **Varying the convergence budget or rate caps.** Rejected: the axis's own experiment-design note
  warns that varying anything besides the reconciler makes the result unattributable. Every
  non-reconciler field was pinned to `exp-004`'s values, which turned out to be worth more than
  expected — two stacks reproduced `exp-004` byte-for-byte and pinned the whole harness by
  replication.

## Verdicts

- **`exp-smooth` — built, admitted.** Gates green; C1 violation quantified in closed form rather
  than asserted. Detail: `docs/research-log/2026-09-08-exp-smooth.md`.
- **`exp-smooth-c1` — built, admitted.** Did not collapse to `spring`; its advantage was confirmed
  structural, not an artifact of a different calibration constant. Detail:
  `docs/research-log/2026-09-08-exp-smooth-c1.md`.
- **`exp-smooth-track` — excluded by admissibility**, narrowly, for a missing non-vacuous test (the
  several-hundred-millisecond gap test each of its three siblings carries). Notably it was *not*
  excluded on bounded convergence: the judge ruled that requirement 1's clause is scoped "under a
  constant correction" throughout, and every existing reconciler's requirement-1 proof is a
  step-then-hold-static test. It did not enter the sweep. Detail:
  `docs/research-log/2026-09-08-exp-smooth-track.md`.

## Course changes mid-run

- **Candidate 2 was interrupted by a session rate limit** after writing its implementation, tests and
  pre-implementation assessment but before running a single gate. It was resumed in the same
  worktree with instructions not to revise the hypothesis it had already committed to. Its test
  suite failed to compile on first execution; that defect and its fix are in its log. Recording this
  because "the assessment was written before the code ran" is load-bearing for how its results read.
- **The panel's admissibility step was told to treat `exp-smooth`'s C1 failure as the subject of the
  investigation rather than a disqualifier**, and to report it quantitatively. Any other contract
  failure remained a genuine exclusion — which is exactly what happened to candidate 3.
- **The verdict step was scoped down mid-panel.** Methodology struck two of five profiles and one
  entire metric column, so the verdict judge was given three profiles and barred from reading
  convergence time across stacks.

## What is left open

- **The requirement-2 decision itself.** The sweep separates the two candidates but cannot close the
  question, for a structural reason: the only metric that would price what `exp-smooth` pays for its
  truncated jerk tail is convergence time, and that column proved unreadable across stacks. A
  tradeoff study with an unreadable cost column cannot conclude "worth it". The organizer's
  recommendation is in the final report; the amendment is a human's.
- **The axis has a smoothness metric with no functioning cost metric.** This is the run's most
  important finding and it outlives the run. `correction_magnitude_*` and `prediction_*_error_*` are
  stamped upstream of the reconciler and are bit-identical across all stacks by construction — they
  are controls and can never be differentiators. `time_to_convergence_ms` counts only
  survivorship-selected completed episodes, so its per-stack populations differ over an identical
  correction set. Nothing compares the displayed pose to ground truth. The axis therefore rewards
  any law that moves less, with no working counterweight. Fixing this is a `docs/metrics.md` change
  and needs a human; it must not be done inside a comparison run.
- **Provenance.** The run's manifest records `76878bf` but the tree was dirty in exactly the
  load-bearing place: at that SHA the manifest's own command cannot run, because neither the YAML nor
  the two registry keys exist. The numbers are real but not attached to a commit. Anything citable
  needs the implementations, the registry line and the YAML committed and tagged, then re-run — with
  the `snap`/`spring` byte-identity against `exp-004` as a free check that the re-run reproduced.
- **Seed count.** Five seeds make 5/5 the *floor* of a sign test (p=0.0625). The direction is
  consistent on every valid profile, but twenty seeds on the two stochastic, transient-free profiles
  would settle it and would cost less than this run did.
- **`synthetic-burst` is deterministic across seeds** — five identical trials, no error bar. Whether
  a seeded burst profile is wanted is a `NetworkProfileCatalog` question, not this run's.
- **No pre-registered decision rule for the sweep.** Both candidate logs stated falsifiers in
  advance; `exp-005` did not state what jerk result would have argued *against* the exception. Close
  that before the next reconciler head-to-head rather than retrofitting it onto this one.
- The axis `CLAUDE.md` tables are stale by design — no candidate was permitted to edit them, so
  `exp-smooth` still reads as planned. Reconciling them is a merge step for a human, and should
  follow the requirement-2 decision rather than precede it.
