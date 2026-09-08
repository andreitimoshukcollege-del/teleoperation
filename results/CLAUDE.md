# results/

**Append-only.** Write new directories, never edit an old one (root `CLAUDE.md`). A number that
appears in a paper came from here and has a manifest — if it doesn't have both, it isn't citable.

## Layout

```
results/<experiment-id>/<UTC-timestamp>/
    manifest.json
    <stack>/<network-profile>/metrics.csv
```

One `metrics.csv` per (stack, network profile) combination, pooling every seed of that
configuration into a single file — seeds are repeated trials of the *same* configuration, meant
to be pooled into one percentile distribution, not scattered across configurations with no way
to tell them apart afterward.

**`<stack>` is named for the axes the experiment actually varies**, and every name appears in
`manifest.json`'s `stacks` array:

- one reconciler held fixed (a predictor study, e.g. `exp-001`) → the bare predictor,
  `double-exp/lan/metrics.csv`. Byte-identical to the layout every run used before stacks
  existed, so re-running an old experiment puts its output exactly where it always went.
- the reconciler swept too (e.g. `exp-003`) → `predictor__reconciler`,
  `double-exp__spring/lan/metrics.csv`. Written by `Teleop.Eval -- sweep <experiments/*.yaml>`
(`core/Teleop.Eval/Sweep/SweepCommand.cs`), normally invoked via `/run-sweep`, never by hand.

## `manifest.json`

No example existed anywhere in the repo before this was written (`Sweep/ManifestWriter.cs`).
Fields:

| Field | Meaning |
|---|---|
| `experimentId`, `predictors`, `reconcilers`, `networkProfiles`, `seeds`, `trialSteps`, `stepIntervalTicks` | The experiment config, as resolved — copied from the YAML, not just a path reference, so the manifest is self-contained even if the YAML changes later. `reconcilers` is a list whichever spelling the YAML used; the older singular `reconciler` key is no longer written, because a reconciler sweep has no single honest value for it |
| `convergenceBudgetMs`, `maxCorrectionLinearSpeedMetersPerSecond`, `maxCorrectionAngularSpeedRadiansPerSecond` | The reconciler operating point the run used. A jerk-vs-convergence-time figure is uninterpretable without it. `snap` ignores all three |
| `stacks` | One entry per fully-resolved mitigation stack: `name`, `predictor`, `reconciler`, `playoutPolicy`, `arbiter`. **`name` is the results subdirectory** (see the layout above). `playoutPolicy`/`arbiter` record the pipeline's current stand-ins (`immediate`/`direct`) rather than registry keys — `Buffering/` and `Autonomy/` have no implementations yet. This is the shape `analysis/teleop_analysis/manifest.py` prefers |
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
