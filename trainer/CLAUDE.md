# trainer — the arm simulator and model training (docs/adr/0021)

Everything that learns lives here: the JetRover simulator, the network synthesis, the features, the
models, training, ONNX export and the trainer's own evaluation. It runs on a GPU machine (an RTX
5090 box, or an H200 Slurm cluster) from **one packed file**, `just pack-trainer`'s
`dist/armpred-<sha>.tar.gz`. The repo's build never runs it; the repo only records data for it and,
after the gate, runs what it exports.

## The isolation rule

- **Nothing here imports anything from the rest of the repo**, and nothing in the repo imports this.
  The packed file must work on a machine that has never seen the repo: no .NET, no Unity, no LFS.
- **The only shared surface is `contract/`**: the three file-format specs (`SESSION_FORMAT.md`,
  `VIEW_FORMAT.md`, `MODEL_FORMAT.md`) and `golden/`. A format change bumps its version, changes the
  spec and both sides in the same commit, and keeps the old version readable until no recording
  uses it.
- **`golden/` is written by the repo, never by hand**: `cd core && dotnet run --project Teleop.Eval
  -- gen-trainer-golden ../trainer/contract/golden`. It pins this package's copies of Core
  (`SeededRng`, the impairments, the profile catalog, `ClockSync`, `relay-replay`) to the C#
  originals. If a golden test fails, the copy is wrong or Core changed; fix whichever moved, and never
  loosen a tolerance to make it pass.

## Copies of Core are exact, or they are not copies

`armpred/net/` and `armpred/baselines/relay_replay.py` reproduce Core's arithmetic, including where
Core works in `float` (single precision) and where C# truncates toward zero. Integer division here
uses `trunc_div`, never `//`, wherever C# divides a `long` that can be negative. A copy that is
"close enough" silently trains the model on a network or a clock the headset does not have.

## What a model may see

At prediction time the model gets only what `ICommandAwarePredictor` gets on the headset: the
commands sent (with their effective times), the measurements received (with their sensed times),
and the target time. It is **never** given the requested speed (except the speed-aware ablation),
the network profile or schedule, true delays, `AddedDelayMs`, the payload, or anything in a
simulated session's `latents`. Latents exist only to stratify results and for oracles, which are
diagnostics and never results.

## Results

- **A trained model is not a result.** Held-out loss is not a result, and neither is this package's
  own evaluation: it decides whether the repo's C# consumer gets built (ADR 0021 decision 8). A number
  becomes citable only when `Teleop.Eval` has scored the exported `.onnx` and written `results/`.
- **Simulated numbers are never the robot's.** A report on simulated sessions says so in its title.
- Metric names are the ones in `docs/metrics.md`; a new one is defined there before it is reported.
- Percentiles (p50/p95/p99/max), never means. Never a winner from one seed.

## Determinism

Every random draw goes through a seeded generator: `numpy.random.Generator(PCG64(seed))` for the
simulator and sampling, and `armpred.net.rng.SeededRng` wherever the draw must match Core's. The
simulator and the view synthesis are deterministic for a given seed and tested to be. GPU training
is not bit-reproducible, which is why the scored artifact is the exported `.onnx`, named by its
sha256 in the model card.

## Outputs

- `runs/<name>/` on the GPU machine: `model.onnx`, `model-card.json`, `report.md`, `views/`,
  `checkpoints/`. Nothing under `runs/` is committed; checkpoints never leave the GPU machine.
- A model is committed only when a published result depends on it, under `core/testdata/models/`,
  by the repo's C# phase.

## Layout

| Path | Holds |
|---|---|
| `armpred/contract/` | session, view and bundle readers and writers, the session checker. numpy only, so `just recording-check` and `just pack-dataset` run without torch |
| `armpred/net/` | the copies of Core: `SeededRng`, impairments, `EmulatedTransport`'s ordering, the profile catalog, `ClockSync`, the headset's feed; and schedules |
| `armpred/baselines/` | `hold-last`, `relay-replay` (copies of Core), `relay-replay-adaptive` (a dev baseline: online lag and droop, speed-blind) |
| `armpred/sim/` | the simulator: operator motion, RobotHost, the relay gate, the servos, the reads |
| `armpred/data/` | turning a session plus a schedule and a seed into what the headset would have seen |
| `armpred/features/` | the model's input window (layout v1, `contract/MODEL_FORMAT.md`) |
| `armpred/models/`, `train.py`, `export.py`, `evaluate.py`, `report.py` | torch models, training, ONNX export, the evaluation and the trainer gate |
| `armpred/sysid.py` | what the recordings say about the robot: the simulator's ranges and the prior's fitted reading ages |
| `armpred/pipeline.py`, `armpred/cli.py` | the session matrix, `prepare`, `train-run`, `sim-pretrain`, `grid`, `finetune`, `sysid` |
| `configs/` | TOML run configs, and the pre-registered `split-v1.toml` |
| `slurm/` | `apptainer.def`, `train-array.sbatch` |
| `tests/` | pytest; `tests/golden/` checks the copies of Core |
