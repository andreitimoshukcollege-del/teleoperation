# experiments/

One YAML file per experiment (root `CLAUDE.md`). Loaded by `Teleop.Eval -- sweep <path>`
(`core/Teleop.Eval/Sweep/ExperimentConfig.cs`) via `/run-sweep`.

## Schema

```yaml
id: exp-001-predictor-baseline      # also the results/ subdirectory name
seeds: [1, 2, 3, 4, 5]               # every (predictor, network profile) pair runs under each
predictors:                          # Registry/Registries.cs Predictors keys
  - none
  - const-vel
  - double-exp
reconciler: snap                     # a single Reconcilers key, held fixed
networkProfiles:                     # Sweep/NetworkProfileCatalog names
  - lan
  - 50ms-5j
  - 150ms-20j-0.5loss
  - 300ms-60j-2loss-bursty
  - synthetic-burst
trialSteps: 500                      # command-submission steps per trial
stepIntervalTicks: 100000            # ticks between steps (100,000 @ 10,000,000 ticks/sec = 10ms)
```

Deliberately minimal — just what `exp-001-predictor-baseline.yaml` needs. Extend it (a delay
distribution shape, a task script, a codec axis) only when an actual experiment needs the field,
per the same "no invented knobs" reasoning `Types/PredictorConfig.cs` gives for its own fields.

## Requirements

1. **Vary one axis at a time.** `Reconciliation/CLAUDE.md`'s experiment-design note ("Reconciler
   studies vary only the reconciler — same predictor, same trace, same seed") generalizes: a
   predictor study holds the reconciler fixed (as `exp-001` does), a reconciler study holds the
   predictor fixed. Mixing both in one sweep produces a result nobody can attribute correctly.
2. **Every `predictors`/`reconciler` entry must resolve in `Registry/Registries.cs`**, and every
   `networkProfiles` entry in `Sweep/NetworkProfileCatalog.cs` — `sweep` validates both before
   running anything and fails loudly (exit 1) rather than skipping an unresolvable entry.
3. **Multiple seeds, always.** `docs/metrics.md` §8: "Never declare a winner from a single seed."
   `sweep` runs every seed listed, not just the first.
4. Numbered sequentially (`exp-001-`, `exp-002-`, ...), named for what's being compared, not for
   the date or the person running it.
