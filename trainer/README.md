# armpred — the JetRover arm-state predictor's trainer

The simulator, the network synthesis, training and ONNX export for the learned joint predictor
(docs/adr/0021), as one folder that runs on a GPU machine from **one packed file**. It imports
nothing from the teleoperation repo; the only things it shares with it are in `contract/`.

**What it learns:** where the arm really is now, from the joint targets the headset sent and the
joint readings that have arrived, adapting without being told to the arm's speed, the network, and
resistance (a payload, a hand). The prediction is `relay-replay` (run speed-blind) plus a learned
correction; `contract/MODEL_FORMAT.md` has the exact input window (layout v2: the commanded path on
a fixed grid, and a network-forecast block).

**Network forecasts** (ADR 0021 decisions 11-14): a separate network model, built later by someone
else, will tell this one how late the network is and is about to be (`contract/FORECAST_FORMAT.md`).
Until it exists, every view is built with a forecast at one of the fixed quality levels
(`oracle`/`good`/`fair`/`poor`, and `persistence`/`rtt-half`/`absent` from what the headset sees
today), and every evaluation sweeps them all. The report's "By network forecast quality" table is the
curve that tells the network model's builders how good a forecast has to be, and gate F1 says whether a
forecast is worth having at all.

## 1. Pack it (on the machine with the repo)

```bash
just pack-trainer        # -> dist/armpred-<sha>.tar.gz
just pack-dataset ...    # later, real recordings -> dist/jetrover-data-<UTC>.tar
```

Copy the `.tar.gz` (and later the data `.tar`) to the GPU machine, then `tar xzf armpred-<sha>.tar.gz`.

## 2a. On the RTX 5090 box

```bash
cd armpred
./setup.sh                               # venv, the torch build this driver needs, selftest
source .venv/bin/activate
armpred sim-pretrain --config configs/smoke.toml --out runs/smoke          # minutes: proves it runs
armpred sim-pretrain --config configs/sim-pretrain.toml --out runs/sim1    # the real pretraining
```

`setup.sh` reads the driver's CUDA version from `nvidia-smi`: CUDA 13.0 or later gets
torch 2.14.1+cu130, CUDA 12.8-12.9 gets torch 2.11.0+cu128 (a 5090 is Blackwell, sm_120, and needs
12.8 at least), no GPU gets the CPU build. `armpred selftest` fails if the installed torch has no
kernels for the GPU it finds.

## 2b. On the H200 Slurm cluster

```bash
cd armpred
apptainer build --build-arg TORCH_INDEX=cu130 --build-arg TORCH_VERSION=2.14.1 armpred.sif slurm/apptainer.def
apptainer exec --nv armpred.sif armpred selftest                    # on a GPU node

# Prepare once (CPU: simulation and shards), then the grid, one run per GPU:
jid=$(sbatch --parsable slurm/prepare.sbatch configs/sim-pretrain.toml $SCRATCH/armpred/data-sim1)
N=$(apptainer exec armpred.sif armpred grid --config configs/sim-pretrain.toml --grid configs/grid.toml --data x --out x --list | wc -l)
sbatch --dependency=afterok:$jid --array=0-$((N - 1))%6 slurm/train-array.sbatch \
  configs/sim-pretrain.toml configs/grid.toml $SCRATCH/armpred/data-sim1 $SCRATCH/armpred/runs-sim1
```

If the cluster's driver is older than CUDA 13.0, build with `TORCH_INDEX=cu128 TORCH_VERSION=2.11.0`.
Edit the `#SBATCH` lines (partition, limits) for your cluster.

## 2c. Real recordings

```bash
# On the machine with the repo, after recording (robot/README.md, ADR 0021 step D1-D2):
just fetch-recordings                         # copies new sessions and checks them
just pack-dataset recordings/jetrover/<id> ...   # -> dist/jetrover-data-<UTC>.tar
# Commit configs/split-v1.toml first: every session in the bundle, in train, val or test.

# On the GPU machine:
armpred finetune --data jetrover-data-<UTC>.tar --split configs/split-v1.toml \
  --config configs/real-finetune.toml --init runs/sim1 --out runs/real1
```

`finetune` refuses a bundle whose files do not match its index, and a split that leaves any session
out. Test sessions are recorded after the model is frozen and evaluated with
`armpred evaluate --run runs/real1 --sessions <dir> --real --profile <bundle>/robot-profile.json`.

**After the pilot hour (ADR 0021 step D1)**, fit the simulator to the robot before pretraining for
real: `armpred sysid --data jetrover-data-<UTC>.tar --out runs/sysid1`, then paste
`runs/sysid1/sysid.toml`'s `[ranges]` into the `sim-pretrain` config and add `[prior] file =
"runs/sysid1/prior.json"`, so the model's prior uses the robot's measured reading ages. With a
fitted prior, every evaluation also scores `relay-replay-fitted`. The servo's speed limit shows only
when a move asks for more than the servo can give: record a few sessions at the fastest slider with
large, quick moves, or `sysid` leaves the speed range wide and says so.

## 3. What to bring back

From each `runs/<name>/`:

| File | What it is |
|---|---|
| `model.onnx` | the model (model format v1, layout v2) |
| `model-card.json` | what made it: trainer SHA, data hashes, config, parity, validation numbers |
| `eval/report.md`, `eval/report.json` | the trainer's own evaluation and the trainer gate |
| `log.txt`, `train-summary.json` | the run's log and learning curve |

`checkpoints/` stays on the GPU machine. **Nothing here is a result.** A simulated run's report
says so in its title. A model becomes a citable number only when the repo's `Teleop.Eval` has scored
it (ADR 0021 decision 8), and that C# side is built only once the trainer gate passes on recordings.

## Commands

| Command | Does |
|---|---|
| `armpred selftest` | environment (torch, the GPU's architecture) and the test suite |
| `armpred check-session <dirs>` | is a recorded session fit to use (`just recording-check` runs this) |
| `armpred simulate --out d --sessions n` | simulated sessions, in the same format the robot writes |
| `armpred prepare --config c --out data` | simulate the session matrix, build the training shards |
| `armpred train-run --config c --data data --out run` | train, export, evaluate on prepared data |
| `armpred sim-pretrain --config c --out run` | prepare and train-run in one directory |
| `armpred grid ... --index i` | one grid combination (a Slurm array task) |
| `armpred evaluate --run run --sessions dir` | the trainer gate's numbers on any directory of sessions |
| `armpred make-views <sessions> --schedule s --out d [--forecast-level l]` | named-profile views (version 2) for the repo's official scoring, optionally carrying a forecast stream |
| `armpred pack-dataset <sessions> --out f --profile p` | checked sessions into one data bundle (`just pack-dataset` runs it) |
| `armpred finetune --data f --split s --config c [--init run] --out run` | train on a real bundle, from a pretrained run |
| `armpred sysid <sessions> \| --data f --out d` | what the recordings say: bus timing, reading ages, servo lag and speed; writes `sysid.toml` (a `[ranges]` table for the next `sim-pretrain`) and `prior.json` (the fitted reading ages) |

`--set key=value` overrides any config value, e.g. `--set train.arch=conv --set train.steps=50000`.

## Layout

`CLAUDE.md` has the rules (isolation, exact copies of Core, what a model may see) and the layout of
the package.
