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

apptainer exec "$SIF" armpred prepare --config "$CONFIG" --out "$DATA" --workers "${SLURM_CPUS_PER_TASK:-8}"
