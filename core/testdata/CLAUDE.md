# testdata

Committed fixtures the build depends on.

## `golden/`

`basic-session.tlog` is the golden log `Teleop.Eval -- verify` replays (Gate 3: replay twice,
byte-identical). Frozen once committed — do not hand-edit it. Regenerate only via
`dotnet run --project Teleop.Eval -- gen-golden` (run from `core/`), inspect the diff, and
commit the result. A legitimate reason to regenerate is an intentional `RecordFormat` version
bump; that is ordinary maintenance, not something that needs an ADR.

`.gitattributes` already declares `*.tlog text eol=lf` — this is a plain, diffable text file,
not a binary blob, and is not LFS-tracked.

## `traces/`

`synthetic-burst.trace` is the one committed trace fixture, authorized by
`docs/adr/0004-network-profile-suite.md`. Same discipline as `golden/`: never hand-edit it.
Regenerate only via `dotnet run --project Teleop.Eval -- gen-trace` (run from `core/`), inspect
the diff, and commit the result.

`cellular-congested`, `leo-satellite`, and `long-haul` — the three frozen-set names implying a
real network capture — are still intentionally absent; adding a real capture here does not need
a new ADR (0004 already reserves the names), but does need an actual measurement, which no agent
can produce.

## `learned/`

The learned joint predictor's and the network forecasts' fixtures (docs/adr/0025, docs/adr/0028,
docs/adr/0035). Written by the trainer, never hand-edited.

- `golden-v3.json` pins `LearnedJointWindow` (layout v3, the JetRover) to the trainer: the window,
  the model's outputs and the predictions. Written by `armpred golden-learned`.
- `golden-v4.json` does the same for `LearnedJointWindowV4` (layout v4, the FR3 Duo, a simulated
  view). Written by `armpred golden-learned-v4`.
- `forecast-golden-v1.json` pins `Forecast/`'s C# copies of the trainer's network forecasts.
  Written by `armpred golden-forecast` (`just gen-forecast-golden`).
- `finetune-pilot2-early7.weights` is the JetRover model the headset ships
  (`unity/Teleop.Bridge/Runtime/Resources/models/finetune-pilot2-early7.bytes`), and the default
  model of `joint-predict-bench` (`--learned-model`) and `headset-replay` (`--model`).
- `fr3-sim-1.weights` is the FR3 Duo's layout-v4 model, trained in simulation. Its headset copy,
  `Resources/models/fr3-sim-1.bytes`, must be the same bytes (`BridgeResourcesTests`).
