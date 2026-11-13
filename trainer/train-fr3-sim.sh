#!/usr/bin/env bash
# The FR3 Duo's simulated pretraining straight from the repo (docs/adr/0028): `trainer/train-fr3-sim.sh <config>
# [name] [args...]`, like train-sim.sh: setup.sh builds trainer/.venv the first time, then armpred
# fr3-sim-pretrain into trainer/runs/<name> (default <config>-<commit>). SIMULATED: never the robot's numbers.
set -euo pipefail
config="${1:-fr3-sim-pretrain}"
name="${2:-}"
shift $(( $# > 2 ? 2 : $# ))
unset PYTHONPATH
cd "$(dirname "$0")"
[ -f "configs/$config.toml" ] || { echo "error: no trainer/configs/$config.toml" >&2; exit 1; }
if [ ! -x .venv/bin/armpred ]; then ./setup.sh; fi
if [ -z "$name" ]; then name="$config-$(git rev-parse --short HEAD)"; fi
echo "run: trainer/runs/$name (log: trainer/runs/$name/log.txt)"
exec .venv/bin/armpred fr3-sim-pretrain --config "configs/$config.toml" --out "runs/$name" "$@"
