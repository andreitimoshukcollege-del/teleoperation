#!/usr/bin/env bash
# Simulated pretraining straight from the repo (docs/adr/0021): `trainer/train-sim.sh <config> [name] [args...]`.
# The first time, setup.sh builds trainer/.venv with the torch build this NVIDIA driver needs and runs the
# selftest; then armpred sim-pretrain into trainer/runs/<name> (default <config>-<commit>). `just train-sim`
# and `just gpu-train` both run this, so a GPU machine needs only bash and git, not a recent `just`.
set -euo pipefail
config="${1:-sim-pretrain}"
name="${2:-}"
shift $(( $# > 2 ? 2 : $# ))
unset PYTHONPATH  # trainer/.venv is isolated from the rest of this machine; see setup.sh
cd "$(dirname "$0")"
[ -f "configs/$config.toml" ] || { echo "error: no trainer/configs/$config.toml" >&2; exit 1; }
if [ ! -x .venv/bin/armpred ]; then ./setup.sh; fi
if [ -z "$name" ]; then name="$config-$(git rev-parse --short HEAD)"; fi
echo "run: trainer/runs/$name (log: trainer/runs/$name/log.txt)"
exec .venv/bin/armpred sim-pretrain --config "configs/$config.toml" --out "runs/$name" "$@"
