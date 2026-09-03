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

# Move the real JetRover arm to a Cartesian target (wrist frame, meters) and hold, e.g. `just move-arm 0.15 0 0.08` -- requires Teleop.RobotHost running on the Jetson and a human watching the hardware
move-arm x y z gripper="0" remote_host="100.112.90.72" remote_port="6000" local_port="6001":
    cd core && dotnet run --project Teleop.Eval -- move-arm \
        --x {{x}} --y {{y}} --z {{z}} --gripper {{gripper}} \
        --remote-host {{remote_host}} --remote-port {{remote_port}} --local-port {{local_port}} \
        --confirm-hardware-motion

# Interactively author a new RobotArmProfile JSON (docs/adr/0011); answers prompts from the terminal, e.g. `just build-profile`
build-profile output="" force="false":
    cd core && dotnet run --project Teleop.Eval -- build-profile \
        {{ if output != "" { "--output " + output } else { "" } }} \
        {{ if force == "true" { "--force" } else { "" } }}

# Phase-3 cross-machine ClockSync diagnostic against an already-running Teleop.RobotHost -- moves the real arm once and holds; a human must be watching the hardware
clocksync-check remote_host="100.112.90.72" remote_port="6000" local_port="6001" rate_hz="20" duration_seconds="20":
    cd core && dotnet run --project Teleop.Eval -- clocksync-check \
        --remote-host {{remote_host}} --remote-port {{remote_port}} --local-port {{local_port}} \
        --rate-hz {{rate_hz}} --duration-seconds {{duration_seconds}} \
        --confirm-hardware-motion

# Publish Teleop.RobotHost for the Jetson (linux-arm64), copy it over, and restart its systemd service -- needs passwordless ssh/scp; does NOT touch the Jetson's ROS 2 nodes, see robot/systemd/README.md
deploy-robothost remote_host="100.112.90.72" remote_user="jetson" operator_host="100.82.140.80" max_direction_magnitude="" profile_path="":
    #!/usr/bin/env bash
    set -euo pipefail
    cd core
    stamp=$(date +%Y%m%d-%H%M%S)
    remote_dir="robothost_deploy_${stamp}"
    # A relative path, not mktemp -d's /tmp/... -- dotnet here is the Windows SDK (see root
    # CLAUDE.md's Environment section), which doesn't resolve a WSL-native absolute path passed as
    # an argument (only the CWD gets translated), so -o would silently publish somewhere under
    # C:\tmp instead and leave this script tarring up an empty directory.
    publish_dir="_scratch_publish"
    archive=$(mktemp -u --suffix=.tar.gz)
    trap 'rm -f "$archive"; rm -rf "$publish_dir"' EXIT
    rm -rf "$publish_dir"
    echo "Publishing Teleop.RobotHost for linux-arm64..." >&2
    dotnet publish Teleop.RobotHost -c Release -r linux-arm64 --self-contained false -f net8.0 -o "$publish_dir"
    tar czf "$archive" -C "$publish_dir" .
    echo "Copying to {{remote_user}}@{{remote_host}}:~/${remote_dir}..." >&2
    scp -o StrictHostKeyChecking=accept-new "$archive" "{{remote_user}}@{{remote_host}}:/tmp/${remote_dir}.tar.gz"
    extra_args=""
    if [ -n "{{max_direction_magnitude}}" ]; then extra_args="$extra_args --max-direction-magnitude {{max_direction_magnitude}}"; fi
    if [ -n "{{profile_path}}" ]; then extra_args="$extra_args --profile-path {{profile_path}}"; fi
    # robothost-run.sh is the stable script teleop-robothost.service's ExecStart points at
    # (robot/systemd/teleop-robothost.service) -- regenerated on every deploy with this run's exact
    # args (including any optional extra_args), so redeploying never needs `systemctl
    # daemon-reload` or touching the unit file itself, only `systemctl restart`. This also sidesteps
    # the earlier pkill+nohup approach's self-inflicted-signal bug entirely (systemd owns the
    # process lifecycle instead of a hand-rolled pkill matching its own ancestor shell).
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" "cat > /home/jetson/robothost-run.sh" <<EOF
    #!/bin/bash
    exec /home/jetson/.dotnet/dotnet /home/jetson/robothost_current/Teleop.RobotHost.dll \
        --local-port 6000 --remote-host {{operator_host}} --remote-port 6001 \
        --relay-socket /tmp/jetrover_relay.sock --local-relay-socket /tmp/teleop_robot_host.sock \
        --joint-local-port 6002 ${extra_args}
    EOF
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" "
        set -e
        mkdir -p ~/${remote_dir}
        tar xzf /tmp/${remote_dir}.tar.gz -C ~/${remote_dir}
        chmod +x ~/${remote_dir}/Teleop.RobotHost
        ln -sfn ~/${remote_dir} ~/robothost_current
        chmod +x ~/robothost-run.sh
        sudo systemctl restart teleop-robothost
        sleep 2
        echo '--- teleop-robothost.service status ---'
        systemctl status teleop-robothost --no-pager -n 10
    "

# SSH in and print systemctl status for all three Jetson-side services (teleop-robothost, jetrover-relay, jetrover-arm-control) in one shot -- see robot/systemd/README.md
robot-status remote_host="100.112.90.72" remote_user="jetson":
    ssh -o StrictHostKeyChecking=accept-new "{{remote_user}}@{{remote_host}}" \
        "systemctl status teleop-robothost jetrover-relay jetrover-arm-control --no-pager -n 5"

# ---- analysis/ (python: figures, percentile tables) ----

# Internal: create analysis/.venv if it doesn't exist yet (fast no-op otherwise). `.venv/` is
# gitignored, so a fresh clone has none -- this makes `test`/`experiment-gui`/`report` below work
# on the first try instead of failing with "python.exe: not found" and telling you to run setup
# yourself. Safe to depend on from every recipe below; it won't touch an existing venv.
_analysis-venv:
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    if [ -f .venv/Scripts/python.exe ] || [ -x .venv/bin/python ]; then
        exit 0
    fi
    echo "analysis/.venv not found -- running one-time setup..." >&2
    just analysis-setup

# One-time analysis/ venv setup (Windows Python via WSL interop -- see analysis/CLAUDE.md).
# Also called automatically by test/experiment-gui/report if analysis/.venv doesn't exist yet.
analysis-setup:
    #!/usr/bin/env bash
    set -euo pipefail
    cd analysis
    /mnt/c/Users/andre/AppData/Local/Microsoft/WindowsApps/python.exe -m venv .venv
    ./.venv/Scripts/python.exe -m pip install -r requirements-dev.txt
    ./.venv/Scripts/python.exe -m pip install -e . --no-build-isolation

# Run the analysis/ pytest suite -- scriptable, use this for CI/agent verification
test: _analysis-venv
    cd analysis && ./.venv/Scripts/python.exe -m pytest -v

# Opens a GUI window to configure and run a sweep, then view its figures (needs a real display)
experiment-gui: _analysis-venv
    cd analysis && ./.venv/Scripts/python.exe run_tests.py

# Generate figures + summary table for a run, e.g. `just report results/exp-001-predictor-baseline/20260804-020431Z`
report run_dir: _analysis-venv
    cd analysis && ./.venv/Scripts/python.exe -m teleop_analysis.cli ../{{run_dir}}

# ---- everything ----

# Every verification gate in the repo: core/ dotnet tests + analysis/ python tests
check: core-check test
