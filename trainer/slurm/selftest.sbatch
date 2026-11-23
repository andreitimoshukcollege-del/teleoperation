#!/usr/bin/env bash
# Check a built image on one GPU: `armpred selftest` (torch sees the GPU and has kernels for it, a tiny
# training step, the test suite), then, unless told not to, the minutes-long smoke pretraining
# (configs/smoke.toml), which proves the whole pipeline runs there, not that anything was learned.
#   sbatch slurm/selftest.sbatch /scratch/$USER/armpred/runs/smoke-<sha>      # selftest + smoke
#   sbatch slurm/selftest.sbatch -                                             # selftest only
# Run from the unpacked pack-trainer folder that holds armpred.sif (slurm/build.sbatch).
# Adjust the partition and limits to your cluster.
#SBATCH --job-name=armpred-selftest
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=01:00:00
#SBATCH --output=armpred-selftest-%j.out
set -euo pipefail

OUT="${1:?smoke run directory, or - for the selftest only}"
SIF="${ARMPRED_SIF:-armpred.sif}"
[ -f "$SIF" ] || { echo "error: no $SIF here (sbatch slurm/build.sbatch first)" >&2; exit 2; }

echo "selftest on $(hostname), $(tr -d '\n' < VERSION)"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
apptainer exec --nv "$SIF" armpred selftest
if [ "$OUT" != "-" ]; then
  mkdir -p "$(dirname "$OUT")"
  apptainer exec --nv "$SIF" armpred sim-pretrain --config configs/smoke.toml --out "$OUT"
  echo "smoke run: $OUT"
fi
