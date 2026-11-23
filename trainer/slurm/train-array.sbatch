#!/usr/bin/env bash
# One grid task per GPU: the runs are independent, so this is how six H200s shorten the grid.
#   N=$(apptainer exec armpred.sif armpred grid --config configs/sim-pretrain.toml --grid configs/grid.toml --data x --out x --list | wc -l)
#   sbatch --array=0-$((N - 1))%6 slurm/train-array.sbatch configs/sim-pretrain.toml configs/grid.toml /scratch/$USER/armpred/data-sim1 /scratch/$USER/armpred/runs-sim1
# (%6: at most six at once, one per GPU.) Run it after slurm/prepare.sbatch has finished, e.g.
#   sbatch --dependency=afterok:<prepare job id> --array=... slurm/train-array.sbatch ...
#SBATCH --job-name=armpred-train
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=16
#SBATCH --mem=64G
#SBATCH --time=06:00:00
#SBATCH --output=armpred-train-%A_%a.out
set -euo pipefail

CONFIG="${1:?config}"
GRID="${2:?grid}"
DATA="${3:?prepared data directory}"
OUT="${4:?runs directory}"
SIF="${ARMPRED_SIF:-armpred.sif}"

# Apptainer mounts only $HOME, /tmp and the working directory into the container: the folders the outputs
# go to (a /scratch or /projects path) are bound explicitly, at the same path, made first if missing.
binds=()
bind_parent() { local d; d=$(dirname "$(realpath -m "$1")"); mkdir -p "$d"; binds+=(--bind "$d"); }
bind_parent "$DATA"; bind_parent "$OUT"

apptainer exec --nv "${binds[@]}" "$SIF" armpred grid --config "$CONFIG" --grid "$GRID" --data "$DATA" --out "$OUT" \
  --index "${SLURM_ARRAY_TASK_ID:?run as an array job}" --workers "${SLURM_CPUS_PER_TASK:-8}"
