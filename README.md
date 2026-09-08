# Teleop Research Platform

VR teleoperation of a remote robot (Meta Quest + Unity), built to produce measured, reproducible
results about latency mitigation: prediction, reconciliation, jitter buffering, autonomy
arbitration, view synthesis. This is a research platform, not a product — see root `CLAUDE.md`
for the architecture, invariants, and rules this repo is built around (required reading before
changing `core/`). `docs/adr/` has the design history behind every non-obvious decision.

This file is the human "how do I actually run this" quickstart. It doesn't repeat `CLAUDE.md`'s
rules or `robot/README.md`'s hardware incident log — it points at both.

## Prerequisites

- **.NET SDK 8** — the Linux SDK on the Linux box, the Windows SDK on the Windows box
  (see "Environment notes" below — the two are not interchangeable).
- **Python 3** — for `analysis/`, via a venv `just` sets up automatically.
- **[`just`](https://github.com/casey/just)** — optional but recommended; every command below has
  a `just` recipe. Run `just --list` any time for the full, current list — this README doesn't
  duplicate it.
- **Unity 2022.3.46f1** — only needed for the VR side (`unity/TeleopVR/`).

## Build & test

```bash
cd core && dotnet test          # unit + allocation tests
just core-check                 # the same, plus `verify` (determinism) and `audit` (invariants)
just check                      # core-check + the analysis/ python test suite
```

Never trust a green build alone — `core-check`'s `verify`/`audit` steps catch things unit tests
miss (see root `CLAUDE.md`'s "Verify your work"). `docs/metrics.md` defines every metric these
tools report.

## Running a sweep

```bash
just sweep experiments/exp-001-predictor-baseline.yaml
just report results/exp-001-predictor-baseline/<timestamp>     # figures + summary table
just experiment-gui                                            # GUI to configure/run/view instead
```

See `experiments/CLAUDE.md` for writing a new experiment config and `results/CLAUDE.md` for the
manifest convention every run produces.

## Controlling the real JetRover arm

This platform's one physical robot today is a Hiwonder JetRover, reachable over Tailscale, whose
ROS 2 side lives in a **separate** repo
([`jetrover-teleop-ros`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros)) on its
Jetson Nano — see `robot/README.md` for the full architecture diagram and hardware incident
history. The steps below are everything needed to go from a clean checkout to actually moving the
arm.

### 1. The robot side auto-starts — nothing to do after a Jetson reboot

`Teleop.RobotHost`, `teleop_relay`'s `relay_node`, and `jetrover_arm_control`'s
`robot_controller_manager` all run as systemd services (`robot/systemd/`), enabled on boot and
auto-restarted on crash. As long as the Jetson is powered on, they're already up — including right
after a reboot, with zero manual SSH step — so Unity's UDP sends (which start unconditionally the
moment Play begins) actually land the instant the Jetson is reachable. `just robot-status` gives a
one-line health check of all three without a manual SSH session. See `robot/systemd/README.md` if
you need to (re)install these on a fresh Jetson image.

### 2. Redeploy `Teleop.RobotHost` after a code change

```bash
just deploy-robothost                          # defaults: Jetson at 100.112.90.72, user jetson
just deploy-robothost 100.112.90.72 jetson      # explicit
```

This publishes for `linux-arm64`, copies the build over, and restarts `teleop-robothost.service` —
it prints the service's own startup banner (profile name, joint count, `MaxDirectionMagnitude`) so
you can confirm it landed correctly. It does **not** touch the two ROS services from step 1. If
you don't have passwordless SSH to the Jetson set up, run the `dotnet publish` line from the
recipe by hand and copy the output over yourself.

### 3. Drive the arm

Headless, from a dev machine:

```bash
just move-arm 0.15 0 0.08                       # Cartesian target (wrist frame, meters), holds
just clocksync-check                            # Phase-3 cross-machine ClockSync diagnostic
just build-profile                              # interactively author a new RobotArmProfile JSON
```

**A human must be watching the physical hardware and have confirmed clearance before running any
of these** — each one commands real motion (`--confirm-hardware-motion` is baked into the recipe).
See `robot/README.md`'s supervised-hardware-test discipline.

From Unity (VR drag-target path): open `unity/TeleopVR/`, set `RemoteHost`/ports in
`Assets/Teleop/Runtime/Bridge/Resources/jetrover_connection.json` to match your `Teleop.RobotHost`
instance, set `ConfirmHardwareMotion: true` only once clearance is confirmed, and press Play.
`JetRoverArmConfig.CommandRateHz` there controls how often real hardware commands are sent — see
that field's own doc comment before changing it, it's tuned against a real, documented servo
cooldown constraint, not an arbitrary number. `JetRoverOperatorBridge` starts sending the moment
Play begins, with no separate "connect" step (UDP has no handshake) — if a
`JetRoverConnectionHud` is wired into the scene (`Bridge/JetRoverConnectionHud.cs`, reads
`JetRoverOperatorBridge.Status`), it shows "connected"/"no connection yet"/"connection lost" so
you don't have to watch the arm move or SSH into Jetson logs to tell whether it actually reached
the robot.

### Reference

- `core/RobotProfiles/*.json` — robot topology/geometry profiles (docs/adr/0011).
- `core/Teleop.RobotHost/RobotHostArgs.cs` — the process's full CLI flag reference.
- `robot/systemd/README.md` — the three auto-start services and how to (re)install them.
- `robot/README.md` — architecture diagram, hardware status, and the incident log (servo faults,
  calibration findings, rate-dependent command loss) worth reading before debugging anything that
  looks like a repeat of a solved problem.

## Environment notes

This project is developed across **two machines with opposite rules**. Root `CLAUDE.md`'s
"Environment" section is the full reference; the short version:

**Linux box** — `core/` and `analysis/` work. Native Ubuntu on ext4, its own clone, the Linux
.NET SDK, absolute paths fine, no WSL interop. Unity cannot open this clone. The
Unix-domain-socket tests in `core/Teleop.RobotHost.Tests` only run here.

**Windows box** — Unity, Quest builds, and JetRover hardware testing.

- Repo lives on NTFS (`C:\Users\...`), reached from WSL — Unity requires this, it can't open a
  project over `\\wsl$\`.
- `dotnet` is the **Windows** SDK even when invoked from a WSL shell. It does not resolve
  WSL-native absolute paths (e.g. from `mktemp -d`) passed as arguments — only the current
  directory gets translated. Use relative paths for anything `dotnet` needs to read or write.
- Do not add the Linux SDK *on this box* — two SDKs sharing one working tree's `build/`/`obj/`
  churn each other. That constraint is specific to this box's shared tree.

**Both** — `git-lfs` must be installed before any working-tree-modifying git command, or
LFS-tracked binaries get written as pointer text files. And Core's C# 9 / `netstandard2.1`
constraint comes from Unity on the Windows box but binds everywhere: code can test green under
the Linux SDK and still break the Quest build.

## Contributing

Read root `CLAUDE.md` first — in particular the "Boundaries for agents" section (free rein in
`core/`/`analysis/`/`experiments/`/`docs/`; anything under `unity/` needs human review; `results/`
is append-only). `docs/adr/` explains why the architecture looks the way it does before you change
it.
