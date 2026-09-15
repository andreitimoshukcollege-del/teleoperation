# Teleop Research Platform

Controlling a robot arm from a VR headset, over the internet, where the round trip takes 50–300 ms
and varies unpredictably. You move your hand; the robot moves a third of a second later. That is
hard to work with and makes some people motion-sick.

You can't make the internet faster. You *can* hide the delay — predict where the robot will be and
draw that, then correct smoothly when the truth arrives. Every technique for doing so trades one
bad thing for another: predict harder and you're wrong more often, correct faster and the picture
jumps.

**This repo measures those trades instead of arguing about them.** It is a research platform, not a
product. A result here looks like:

> On a link with bursty delay, a playout buffer that adapts to recent conditions holds the same
> loss rate as the best possible fixed buffer while adding **56–59% less delay**. On a steady link,
> a hand-tuned fixed buffer beats it. The crossover is the finding.

**New here? → [`ONBOARDING.md`](ONBOARDING.md)** — setup, in order, with what to expect at each
step. This file is the day-to-day command reference.

---

## Quick start

```bash
git lfs install                 # BEFORE cloning -- see ONBOARDING.md
git clone https://github.com/andreitimoshukcollege-del/teleoperation.git
cd teleoperation

cd core && dotnet test          # ~800 tests, a few seconds
just check                      # every gate in the repo
just sweep experiments/exp-001-predictor-baseline.yaml
```

Needs .NET SDK 8, Python 3 (with `python3-venv` on Debian/Ubuntu), `git-lfs`, and optionally
[`just`](https://github.com/casey/just). Unity 2022.3.46f1 only for the VR side; `python3-tk` only
if you want `analysis/`'s GUI.

## Where things are

| Path | What it holds |
|---|---|
| `core/Teleop.Core/` | every algorithm — one copy, compiled by both `dotnet` and Unity |
| `core/Teleop.Eval/` | headless CLI: `sweep`, `verify`, `audit` |
| `unity/TeleopVR/` | scenes, XR, rendering, real I/O |
| `analysis/` | Python — reads `results/`, produces figures |
| `experiments/` | one YAML per experiment |
| `results/` | **append-only.** New directories only; never edit an old one |
| `docs/metrics.md` | every metric is defined here |
| `docs/research-log/` | what was tried, what happened, what was rejected |
| `docs/adr/` | why the architecture looks the way it does |

Most directories have a `CLAUDE.md` with the working rules for that directory. They record
decisions that were expensive to learn — read the one for anything you're about to change.

## Everyday commands

```bash
just --list          # the full, current list -- this file doesn't duplicate it
```

**Verifying a change**

```bash
just core-check      # dotnet test + verify (determinism) + audit (invariants)
just bridge-check    # does a Core API change break the Unity side?
just check           # all of the above plus the analysis/ test suite
```

A green `dotnet test` alone is **not** enough. `verify` replays a recorded session twice and
requires byte-identical output; `audit` inspects the built assembly for invariant violations. Both
exist because unit tests here have repeatedly missed the kind of bug that invalidates a
measurement.

**Running experiments**

```bash
just sweep experiments/<config>.yaml          # run it
just report results/<experiment>/<timestamp>  # figures + summary table
just experiment-gui                           # configure, run and plot in one window
```

Every run writes a `manifest.json` with the git SHA that produced it. See `experiments/CLAUDE.md`
to write a new config and `results/CLAUDE.md` for the manifest convention.

## Two machines, opposite rules

Development is split across a **Linux box** (`core/`, `analysis/` — algorithms and experiments) and
a **Windows box** (`unity/`, Quest builds, hardware). This isn't a preference: Unity can't open a
project stored inside WSL, and the Windows box's `dotnet` is the Windows SDK even from a WSL shell,
so one shared tree would have two SDKs fighting over `build/`.

Practical consequences:

- **One branch per machine.** Never check out a branch the other box is working on — reach `main`
  through a PR.
- **Pull `main` before opening Unity.** Core is linked by relative path, so a Core change on `main`
  changes the Unity build immediately.
- Core's C# 9 / `netstandard2.1` limit comes from Unity but **binds everywhere** — code can pass
  `dotnet test` on Linux and still break the Quest build.

Full reference: root `CLAUDE.md`'s "Environment" section.

## The real robot

A Hiwonder JetRover with a Jetson Nano, over Tailscale. Its ROS 2 side is a separate repo
([`jetrover-teleop-ros`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros)).

> **A human must be watching the arm with clearance confirmed before any of these run.** Each one
> commands real motion.

```bash
just robot-status                 # health of all three Jetson services
just deploy-robothost             # redeploy Teleop.RobotHost after a Core change
just move-arm 0.15 0 0.08         # Cartesian target, metres, wrist frame -- MOVES THE ARM
just clocksync-check              # cross-machine clock-sync diagnostic
```

The Jetson's services auto-start on boot, so there's nothing to do after a reboot. Drive hardware
through a `just` recipe, never by hand-typing the underlying command — if what you need has no
recipe and is reusable, add one.

From Unity: open `Assets/Scenes/JetRoverControl.unity`, set the host and ports in
`Assets/Teleop/Runtime/Bridge/Resources/jetrover_connection.json`, set
`ConfirmHardwareMotion: true` only once clearance is confirmed, and press Play. The connection HUD
reports connected / no connection yet / connection lost, so you don't have to watch the arm or read
Jetson logs to tell whether packets are landing.

**Read `robot/README.md` before any hardware work.** Its incident log covers real servo faults,
calibration findings and a rate-dependent command-loss bug — most "new" hardware problems turn out
to be repeats.

## Contributing

Read root `CLAUDE.md` first, especially "Boundaries for agents": free rein in `core/`, `analysis/`,
`experiments/` and `docs/`; anything under `unity/` needs human review; `results/` is append-only.

Write up what you find in `docs/research-log/` — **including the things that didn't work.** A
finished negative result beats three unfinished positive ones, and a rejected idea that nobody
recorded gets proposed again next quarter.
