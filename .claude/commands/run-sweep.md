---
description: Run an experiment sweep through Teleop.Eval and record a manifested result
argument-hint: <experiment-yaml-or-id>
allowed-tools: Read, Write, Glob, Grep, Bash(dotnet:*), Bash(git status:*), Bash(git rev-parse:*), Bash(git log:*), Bash(git tag:*), Bash(ls:*), Bash(mkdir:*)
model: sonnet
---

## Repo state

!`git status --short`
!`git rev-parse HEAD`

## Task

Run the experiment described by `$ARGUMENTS` and record it reproducibly.

1. **Gate on cleanliness.** If the working tree above is dirty, stop. A result produced from
   uncommitted code is not reproducible, and recording it anyway is worse than not running.
2. **Gate on reachability.** Confirm the SHA above is on `main` or on a tag. If it is only on a
   branch that may be squash-merged or deleted, the SHA in the manifest will become
   unresolvable — tell the user to tag first, then stop.
3. Resolve `$ARGUMENTS` to a file under `experiments/`. Read it. Verify every algorithm name it
   references resolves in `Registry/Registries.cs` *before* launching anything.
4. If the config specifies a single seed, flag it — one seed cannot distinguish an effect from
   run-to-run variance.
5. Run: `dotnet run --project core/Teleop.Eval -- sweep <resolved-yaml>`
6. Write `results/<exp-id>/<ISO-8601-timestamp>/manifest.json` with the resolved config, the
   git SHA, every seed, the machine identifier, and the exact command line.
7. Report a configuration × metric table using p50/p95/p99. Include prediction error **and**
   correction cost together, and include the baseline row (`none` / `snap`) even though it will
   lose — it is what makes the other numbers interpretable.

Do not tune anything mid-run to improve a result. Surface anomalies — divergence, NaN, buffer
starvation, allocation spikes — prominently rather than in a footnote; they are usually more
informative than a clean win.
