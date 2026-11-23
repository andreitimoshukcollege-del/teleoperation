#!/usr/bin/env bash
# Simulate the session matrix and build the shards once, on CPUs; the training array then reads them.
#   sbatch slurm/prepare.sbatch configs/sim-pretrain.toml /scratch/$USER/armpred/data-sim1
# Adjust the partition and limits to your cluster.
#SBATCH --job-name=armpred-prepare
#SBATCH --cpus-per-task=64
#SBATCH --mem=128G
#SBATCH --time=08:00:00
#SBATCH --output=armpred-prepare-%j.out
set -euo pipefail

CONFIG="${1:?config}"
DATA="${2:?data directory}"
SIF="${ARMPRED_SIF:-armpred.sif}"

# Apptainer mounts only $HOME, /tmp and the working directory into the container: the folders the outputs
# go to (a /scratch or /projects path) are bound explicitly, at the same path, made first if missing.
binds=()
bind_parent() { local d; d=$(dirname "$(realpath -m "$1")"); mkdir -p "$d"; binds+=(--bind "$d"); }
bind_parent "$DATA"

apptainer exec "${binds[@]}" "$SIF" armpred prepare --config "$CONFIG" --out "$DATA" --workers "${SLURM_CPUS_PER_TASK:-8}"
