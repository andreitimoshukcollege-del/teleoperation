#!/usr/bin/env bash
# One-time setup on a GPU machine (or any Linux box): a venv with pinned libraries and the torch
# build that matches this machine's NVIDIA driver, then `armpred selftest`.
#
#   ./setup.sh                 # into ./.venv
#   ARMPRED_VENV=/scratch/me/armpred ./setup.sh
#   ARMPRED_TORCH=cpu ./setup.sh   # force the CPU build
#
# Needs Python >= 3.11 with venv (Ubuntu: `sudo apt install python3.12-venv`) and network access to
# pypi.org and download.pytorch.org. On a Slurm cluster without either, use slurm/apptainer.def.
set -euo pipefail

# Per this package's isolation rule (trainer/CLAUDE.md): nothing here may depend on what else is
# on this machine. A leaked PYTHONPATH (e.g. a sourced ROS setup.bash) survives venv creation and
# is on sys.path ahead of the venv's own site-packages, so pytest's plugin autoloader can pick up
# an unrelated package's pytest11 entry point (ROS's launch_testing) and fail importing pytest
# itself with an unrelated ModuleNotFoundError.
unset PYTHONPATH

cd "$(dirname "$0")"
VENV="${ARMPRED_VENV:-.venv}"
PYTHON="${PYTHON:-python3}"

"$PYTHON" -c 'import sys; assert sys.version_info >= (3, 11), "armpred needs Python 3.11+"'
"$PYTHON" -m venv "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -m pip install --quiet --upgrade pip
python -m pip install --quiet -r requirements.txt

torch_build="${ARMPRED_TORCH:-auto}"
if [ "$torch_build" = auto ]; then
  torch_build=cpu
  if command -v nvidia-smi >/dev/null 2>&1; then
    # The driver's CUDA version, e.g. "CUDA Version: 13.0" -> 13.0
    cuda="$(nvidia-smi | sed -n 's/.*CUDA Version: *\([0-9][0-9]*\.[0-9]*\).*/\1/p' | head -1)"
    major="${cuda%%.*}"; minor="${cuda#*.}"
    if [ -n "$cuda" ] && { [ "$major" -gt 13 ] || { [ "$major" -eq 13 ] && [ "$minor" -ge 0 ]; }; }; then
      torch_build=cu130
    elif [ -n "$cuda" ] && [ "$major" -eq 12 ] && [ "$minor" -ge 8 ]; then
      torch_build=cu128
    else
      echo "This driver reports CUDA ${cuda:-?}: a 5090 needs 12.8 or later. Update the driver, or ARMPRED_TORCH=cpu." >&2
      exit 1
    fi
  fi
fi
case "$torch_build" in
  cu130) python -m pip install --quiet torch==2.14.1 --index-url https://download.pytorch.org/whl/cu130 ;;
  cu128) python -m pip install --quiet torch==2.11.0 --index-url https://download.pytorch.org/whl/cu128 ;;
  cpu)   python -m pip install --quiet torch==2.14.1 --index-url https://download.pytorch.org/whl/cpu ;;
  *) echo "unknown ARMPRED_TORCH=$torch_build (cu130, cu128, cpu)" >&2; exit 1 ;;
esac
python -m pip install --quiet --no-deps -e .

echo "torch build: $torch_build"
armpred selftest
echo "Ready: source $VENV/bin/activate, then see README.md."
