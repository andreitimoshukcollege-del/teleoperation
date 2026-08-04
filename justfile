# Teleop Research Platform -- convenience commands.
# Install `just`: https://github.com/casey/just
# `just` with no arguments lists everything below.

default:
    @just --list

# ---- core/ (dotnet: algorithms, unit tests, sweep) ----

# Unit + allocation tests for Teleop.Core
core-test:
    cd core && dotnet test

# Replay the golden .tlog twice, assert identical (determinism gate)
core-verify:
    cd core && dotnet run --project Teleop.Eval -- verify

# Invariant + registry-completeness check over the built assembly
core-audit:
    cd core && dotnet run --project Teleop.Eval -- audit

# Run all three core/ gates -- "Verify your work" in root CLAUDE.md
core-check: core-test core-verify core-audit

# Run an experiment sweep, e.g. `just sweep experiments/exp-001-predictor-baseline.yaml`
sweep config:
    cd core && dotnet run --project Teleop.Eval -- sweep ../{{config}}

# ---- analysis/ (python: figures, percentile tables) ----

# One-time analysis/ venv setup (Windows Python via WSL interop -- see analysis/CLAUDE.md)
analysis-setup:
    cd analysis
    /mnt/c/Users/andre/AppData/Local/Microsoft/WindowsApps/python.exe -m venv .venv
    ./.venv/Scripts/python.exe -m pip install -r requirements-dev.txt
    ./.venv/Scripts/python.exe -m pip install -e . --no-build-isolation

# Run the analysis/ pytest suite -- scriptable, use this for CI/agent verification
test:
    cd analysis && ./.venv/Scripts/python.exe -m pytest -v

# Interactively pick which analysis/ tests to run (needs a real terminal, not piped output)
test-pick:
    cd analysis && ./.venv/Scripts/python.exe run_tests.py

# Generate figures + summary table for a run, e.g. `just report results/exp-001-predictor-baseline/20260804-020431Z`
report run_dir:
    cd analysis && ./.venv/Scripts/python.exe -m teleop_analysis.cli ../{{run_dir}}

# ---- everything ----

# Every verification gate in the repo: core/ dotnet tests + analysis/ python tests
check: core-check test
