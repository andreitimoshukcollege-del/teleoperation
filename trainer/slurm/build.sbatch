#!/usr/bin/env bash
# Build the trainer's Apptainer image (slurm/apptainer.def) on a CPU node, from an unpacked
# `just pack-trainer` folder (cd armpred first):
#   sbatch slurm/build.sbatch cu128      # the driver supports CUDA 12.8-12.9 (Explorer's H200s: 570.x = 12.8)
#   sbatch slurm/build.sbatch cu130      # the driver supports CUDA 13.0 or later
# The torch build per CUDA level is setup.sh's (requirements.txt says why). Building needs network
# access (Docker Hub, PyPI, download.pytorch.org) and --fakeroot. It runs as a job because a login
# node is for editing and submitting, not for a multi-GB pip install. The image appears as
# armpred.sif in this folder only once it is whole (built to a temporary name, then renamed).
# Adjust the partition and limits to your cluster (the `just explorer-*` recipes pass Explorer's on
# the sbatch command line, which overrides these).
#SBATCH --job-name=armpred-build
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=01:30:00
#SBATCH --output=armpred-build-%j.out
set -euo pipefail

case "${1:-}" in
  cu128) index=cu128 version=2.11.0 ;;
  cu130) index=cu130 version=2.14.1 ;;
  *) echo "usage: sbatch slurm/build.sbatch cu128|cu130 (which CUDA the GPU nodes' driver supports: nvidia-smi)" >&2; exit 2 ;;
esac
[ -f slurm/apptainer.def ] && [ -f VERSION ] || { echo "error: run from an unpacked pack-trainer folder (armpred/)" >&2; exit 2; }

# The layer cache and the build's scratch space: not the home directory (a quota) and not /tmp on a
# shared login node. A job-local directory if the cluster gives one.
export APPTAINER_CACHEDIR="${APPTAINER_CACHEDIR:-${SCRATCH:-$HOME}/.apptainer-cache}"
export APPTAINER_TMPDIR="${APPTAINER_TMPDIR:-${SLURM_TMPDIR:-${TMPDIR:-/tmp}}}"
mkdir -p "$APPTAINER_CACHEDIR"

echo "building armpred.sif: torch $version+$index, $(tr -d '\n' < VERSION), on $(hostname)"
rm -f armpred.sif.partial
apptainer build --fakeroot --build-arg TORCH_INDEX="$index" --build-arg TORCH_VERSION="$version" \
  armpred.sif.partial slurm/apptainer.def
mv -f armpred.sif.partial armpred.sif
echo "built $(pwd)/armpred.sif ($(du -h armpred.sif | cut -f1))"
