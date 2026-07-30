---
description: Compare two recorded experiment runs and report what actually differs
argument-hint: <results-dir-a> <results-dir-b>
allowed-tools: Read, Glob, Grep, Bash(ls:*), Bash(find:*), Bash(git log:*), Bash(git diff:*), Bash(python:*), Bash(python3:*)
model: sonnet
---

Compare the runs at `$1` and `$2`.

1. Read both `manifest.json` files first. Report every difference in config, git SHA, and
   seeds **before** looking at any metric.
2. **Check comparability.** If the runs used different network profiles, different traces,
   different task sets, or different seeds, say clearly what can and cannot be concluded. Two
   runs that differ in more than one dimension do not support a causal claim about either. Do
   not proceed to a "winner" in that case.
3. If the SHAs differ, summarize what changed in Core between them (`git log --oneline A..B`
   restricted to `core/`). A metric difference attributed to an algorithm change is wrong if
   an unrelated commit also landed.
4. Compare metrics at p50/p95/p99. Report absolute values, not just deltas — a 20% improvement
   on an already-unusable number is not progress.
5. Report the tradeoff explicitly: which run wins on prediction error, which on correction
   cost, which on task performance. These frequently disagree, and that disagreement is the
   interesting finding.
6. State whether the difference exceeds the run-to-run variance visible in the seed spread. If
   it doesn't, the answer is "no detectable difference" — say that rather than reporting the
   nominally larger number as a win.

Do not recompute metrics from raw recordings. Read what was emitted. If a metric you need is
absent, report that Core needs to emit it.
