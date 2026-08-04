# results/

**Append-only.** Write new directories, never edit an old one (root `CLAUDE.md`). A number that
appears in a paper came from here and has a manifest — if it doesn't have both, it isn't citable.

## Layout

```
results/<experiment-id>/<UTC-timestamp>/
    manifest.json
    metrics.csv
```

Written by `Teleop.Eval -- sweep <experiments/*.yaml>` (`core/Teleop.Eval/Sweep/SweepCommand.cs`),
normally invoked via `/run-sweep`, never by hand.

## `manifest.json`

No example existed anywhere in the repo before this was written (`Sweep/ManifestWriter.cs`).
Fields:

| Field | Meaning |
|---|---|
| `experimentId`, `predictors`, `reconciler`, `networkProfiles`, `seeds`, `trialSteps`, `stepIntervalTicks` | The experiment config, as resolved — copied from the YAML, not just a path reference, so the manifest is self-contained even if the YAML changes later |
| `gitSha` | `git rev-parse HEAD` at run time. **Only trust a result whose SHA is reachable from `main` or a tag** — `/run-sweep` gates on a clean working tree specifically so this SHA actually corresponds to the code that produced the result. A dirty-tree run's SHA is real but the code isn't fully described by it; do not cite one. |
| `configPath` | Path to the experiment YAML as invoked |
| `machine` | `Environment.MachineName` — result variance across machines is a real thing to be able to check for |
| `command` | The exact command line, for literal reproduction |
| `generatedAtUtc` | ISO-8601 timestamp |

## `metrics.csv`

Raw `name,value,ticks` rows (`Metrics/CsvMetricSink.cs`'s existing format) — long/tidy, one row
per `IMetricSink.Record` call across every trial in the sweep. No percentiles, no aggregation:
per `.claude/commands/run-sweep.md`'s own step split, computing the p50/p95/p99 table happens
*after* `sweep` runs, from this file, not inside `Teleop.Eval` itself.

## Squash-merging warning

Repeated from root `CLAUDE.md` because it matters most here: squash-merging a branch that
produced a result still sitting in an unreached commit makes its `gitSha` unreachable, and an
unreachable SHA is an uncitable result. Tag before running anything you intend to keep.
