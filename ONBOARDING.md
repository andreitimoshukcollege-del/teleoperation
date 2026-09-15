# Onboarding

Everything you need to go from "I have never seen this repo" to "I ran an experiment and
understand what the numbers mean." Budget about 30 minutes for the algorithm setup; the Unity and
robot setups are longer and most people don't need them on day one.

`README.md` is the day-to-day command reference. This file is the one-time setup and the mental
model behind it.

---

## 1. What you're joining

We teleoperate a robot arm from a VR headset over the internet. The headset is here, the robot is
somewhere else, and the network between them adds 50–300 ms of delay that varies unpredictably.
Move your hand and the robot moves a third of a second later — which makes precise work hard and
makes some people motion-sick.

You cannot make the internet faster. What you *can* do is hide the delay: guess where the robot
will be and draw that instead of where you last heard it was, then correct smoothly when the truth
arrives. Every such technique trades one bad thing for another — guess harder and you're wrong more
often; correct faster and the picture jumps.

**This project exists to measure those trades rather than argue about them.** That is the single
most important thing to understand about it. It is a research platform, not a product. The
deliverable is a number with a method behind it.

A real result from this repo looks like:

> On a link with bursty delay, a playout buffer that adapts to recent conditions holds the same
> loss rate as the best possible fixed buffer while adding **56–59% less delay**. On a link whose
> delay is steady, a hand-tuned fixed buffer beats it. The crossover is the finding.

Not "adaptive buffering is better." A number, a condition, and the case where it fails.

---

## 2. Which machine do you need?

The project spans two development machines with **deliberately opposite rules**, plus the robot.
Most work needs only the first.

| You want to work on | You need | Setup time |
|---|---|---|
| Algorithms, experiments, analysis (`core/`, `analysis/`) | **Linux box** — track A | ~30 min |
| VR scenes, headset builds, rendering (`unity/`) | **Windows box** — track B | ~2 h incl. Unity install |
| The physical robot arm | Windows box + supervised access — track C | read first, act later |

If you're unsure, do **track A**. It covers every research axis, runs the whole test suite, and
produces real results without any hardware.

> **Why two machines and not one?** Unity is a Windows application that cannot open a project
> stored inside WSL, and the `dotnet` SDK on the Windows box is the *Windows* SDK even when you
> invoke it from a Linux-looking shell. Trying to do both jobs on one box means two SDKs fighting
> over one `build/` directory. Root `CLAUDE.md`'s "Environment" section is the full reference.

---

## Track A — the algorithm box (Linux)

### A1. Install the toolchain

```bash
# .NET SDK 8 -- the version CI pins. Follow Microsoft's instructions for your distro:
#   https://learn.microsoft.com/dotnet/core/install/linux
dotnet --version        # expect 8.0.x

# git-lfs. Install this BEFORE you clone -- see the warning below.
sudo apt install git-lfs && git lfs install
git lfs version

# Python 3 with venv support. On Debian/Ubuntu the venv module is a SEPARATE package and
# leaving it out produces a confusing half-broken virtualenv later, not a clear error.
sudo apt install python3 python3-venv
python3 --version

# `just`, the command runner. Optional, but every command in the docs has a recipe.
#   https://github.com/casey/just#installation
just --version

# Optional: only if you want analysis/'s GUI. Nothing headless needs it.
# sudo apt install python3-tk
```

> ⚠️ **Install `git-lfs` before your first clone or any `git checkout`.** Some Unity binaries are
> stored in LFS. Without the filter installed, git writes small text pointer files where the real
> binaries should be, and nothing warns you — you find out when Unity fails to open an asset.
> If you cloned without it: `git lfs install && git lfs pull`.

### A2. Clone and build

```bash
git clone https://github.com/andreitimoshukcollege-del/teleoperation.git
cd teleoperation

cd core && dotnet test
```

**Expected:** four test projects pass, roughly 800 tests total, in a few seconds.

If `dotnet test` fails on a *fresh* clone, that is a real problem worth reporting — `main` is kept
green.

### A3. Set up the Python analysis environment

```bash
just analysis-setup
just test
```

**Expected:** 90 passed, 3 skipped.

The skips are the GUI tests. They need `tkinter`, which is a separate system package and is absent
on a headless box — skipping is correct, not a gap. The GUI is a human-facing convenience on the
Windows box; the scriptable path that CI uses has never needed a display. If you do want it:
`sudo apt install python3-tk`.

`just analysis-setup` auto-detects the right interpreter for whichever box you're on, so you do not
need to configure a path.

<details>
<summary><b>If it says "analysis/.venv exists but is unusable (no pip)"</b></summary>

You hit the `python3-venv` trap: on Debian/Ubuntu, `python3 -m venv` half-succeeds without that
package, leaving an interpreter with no `pip`. Fix:

```bash
sudo apt install python3-venv
rm -rf analysis/.venv
just analysis-setup
```
</details>

### A4. Run every gate

```bash
just check
```

This is the command to run before you push anything. It runs, in order:

| Gate | What it catches |
|---|---|
| `dotnet test` | ordinary correctness, plus zero-allocation assertions on the per-frame path |
| `verify` | **non-determinism** — replays a recorded session twice and requires byte-identical output |
| `audit` | **invariant violations** in the built assembly (no Unity references, no clock reads, no reflection) |
| `bridge-check` | a Core API change that silently breaks the Unity side |
| `pytest` | the Python analysis layer |

A green `dotnet test` alone does **not** mean your change is safe. `verify` and `audit` exist
because unit tests here have repeatedly missed exactly the kind of bug that invalidates a
measurement. Root `CLAUDE.md`: *"Never report success on the basis of a successful build alone."*

### A5. Run your first experiment

```bash
just sweep experiments/exp-001-predictor-baseline.yaml
```

This runs every prediction algorithm against five simulated network conditions, five random seeds
each, and writes to `results/exp-001-predictor-baseline/<timestamp>/`. Look at what it produced:

```
results/<experiment>/<timestamp>/
├── manifest.json                 # git SHA, seeds, config -- what makes the run reproducible
└── <stack>/<network-profile>/
    └── metrics.csv               # name,value,ticks -- one row per measurement
```

Then turn it into figures:

```bash
just report results/exp-001-predictor-baseline/<timestamp>
```

Or use the GUI, which configures, runs and plots in one window (needs a display):

```bash
just experiment-gui
```

**You are now set up.** Skip to section 5 for how the project actually works.

---

## Track B — the Unity box (Windows)

Only needed for VR scenes, rendering, or headset builds.

### B1. What's different about this box

- The repo must live on **NTFS** (`C:\Users\<you>\Projects\teleoperation`), reached from WSL as
  `/mnt/c/...`. Unity cannot open a project over `\\wsl$\`.
- `dotnet` is the **Windows** SDK even from a WSL shell. It does not resolve WSL-native absolute
  paths passed as arguments — only the working directory is translated. **Pass relative paths**, or
  wrap with `$(wslpath -w <path>)`.
- **Do not install the Linux .NET SDK on this box.** Two SDKs sharing one `build/`/`obj/` tree
  churn each other and produce restore errors that look like corruption.

### B2. Install

1. **Unity 2022.3.46f1** — this exact version. Not 2022.3.x-something-else.
2. Add the **Android** build support module (for the Quest) during installation.
3. `git` and `git-lfs` inside WSL. Same warning as track A: LFS first, then clone.

### B3. Open it

Open `unity/TeleopVR/` in Unity Hub. Core is linked by **relative path** —
`Packages/manifest.json` has `"com.teleop.core": "file:../../../core/Teleop.Core"` — so the C#
algorithms are the *same files* the Linux box tests. There is no copy and no sync step.

The consequence: **pull `main` before opening the editor.** A Core change that landed since your
last pull changes the Unity build immediately.

Two scenes matter:

| Scene | What it is |
|---|---|
| `Assets/Scenes/SampleScene.unity` | the loopback baseline — everything in-process, no network, no hardware |
| `Assets/Scenes/JetRoverControl.unity` | the real-robot path (track C) |

Start with `SampleScene`. Press Play; you should see a "ghost" robot tracking a target, with a
latency figure on the HUD.

### B4. Verify before you commit

```bash
just bridge-check        # compiles Bridge/ against Core headlessly
```

This catches signature breaks fast, but it is **not a substitute for opening the editor** — it
compiles against UnityEngine stubs, so it cannot catch anything about scenes, prefabs or rendering.
An IL2CPP build for the Quest is the only real check that Core's C# 9 constraint holds.

---

## Track C — the physical robot (read this before touching anything)

The robot is a Hiwonder JetRover with a Jetson Nano, reachable over Tailscale. Its ROS 2 side lives
in a **separate repository**
([`jetrover-teleop-ros`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros)).

**Rules, not suggestions:**

1. **A human must be physically watching the arm, with clearance confirmed, before any command is
   sent.** Every hardware recipe bakes in `--confirm-hardware-motion` precisely so this cannot be
   an accident.
2. **Drive hardware through a `just` recipe, never by hand-typing the underlying command.** If the
   operation you need has no recipe and is reusable, add one as part of your change.
3. **Read `robot/README.md`'s incident log first.** It records real servo faults, calibration
   findings, and a rate-dependent command-loss bug. Most "new" hardware problems are repeats.

The robot-side services auto-start on boot, so after a Jetson reboot there is nothing to do:

```bash
just robot-status              # health of all three Jetson services, no manual SSH
just deploy-robothost          # redeploy Teleop.RobotHost after a Core change
just move-arm 0.15 0 0.08      # Cartesian target in metres -- MOVES REAL HARDWARE
```

---

## 5. How the project actually works

Setup is the easy part. This is the part that makes the repo make sense.

### The one law

**Dependencies point one direction.** `Teleop.Core` sits at the bottom and depends on nothing —
not Unity, not the network, not the filesystem, not even a clock.

```
analysis/  ──reads──>  results/  <──writes──  Teleop.Eval ──┐
                                                            ├──> Teleop.Core
                                            unity/TeleopVR ─┘     (depends on NOTHING)
```

Unity gives Core capabilities by **implementing interfaces Core declares**, never by Core importing
Unity. That is what makes every algorithm testable headlessly, which is what makes results
reproducible.

### Why Core is so restricted

`core/Teleop.Core/CLAUDE.md` lists rules that look arbitrary until you see the reason. No clock
reads, no I/O, no allocation on the per-frame path, no reflection, C# 9 only. Each one exists
because breaking it destroys something specific:

- **No clock reads** → time arrives as a parameter → the same input replays to the same output →
  `verify` can prove determinism, and a latency measurement means something.
- **No reflection** → the Quest's ahead-of-time compiler strips code nothing references directly,
  so reflection compiles fine on your desktop and crashes on the headset.
- **C# 9 / netstandard2.1** → Unity 2022.3's compiler. Newer syntax passes `dotnet test` on Linux
  and breaks the headset build silently.

### The research loop

1. **A question.** "Does adapting the buffer beat a fixed one?"
2. **An implementation** — one new file in the matching axis folder (`Prediction/`,
   `Reconciliation/`, `Buffering/`, `Transport/`, `Autonomy/`, `Plant/`) plus a hand-written entry
   in `Registry/Registries.cs`. Use `/new-impl` to scaffold all the pieces at once.
3. **An experiment** — a YAML in `experiments/` naming the algorithms, network profiles and seeds.
4. **A sweep** → `results/<id>/<timestamp>/` with a `manifest.json` recording the git SHA.
5. **A written verdict** in `docs/research-log/` — including the negatives. **A finished negative
   result beats three unfinished positive ones.**

### Where things live

| Path | What |
|---|---|
| `core/Teleop.Core/` | every algorithm; one copy compiled by both `dotnet` and Unity |
| `core/Teleop.Eval/` | the headless CLI: `sweep`, `verify`, `audit` |
| `analysis/` | Python: reads `results/`, makes figures |
| `experiments/` | one YAML per experiment |
| `results/` | **append-only** — write new directories, never edit old ones |
| `docs/adr/` | architecture decisions, with the reasoning |
| `docs/research-log/` | what was tried, what happened, and what was rejected |
| `docs/metrics.md` | **every metric is defined here.** Never invent one without adding it |

### The CLAUDE.md files

Most directories have one. They are the working rules for that directory and they are **not**
generic style guides — they record decisions that were expensive to learn. Read the one for any
folder you are about to change. Root `CLAUDE.md` is required reading before touching `core/`.

---

## 6. Things that will bite you

Every one of these has actually happened here.

| Symptom | Cause | Fix |
|---|---|---|
| Unity can't read an asset; file looks like 3 lines of text | cloned without `git-lfs` | `git lfs install && git lfs pull` |
| `No module named pip` from anything Python | `python3-venv` missing when the venv was made | `sudo apt install python3-venv`, delete `analysis/.venv`, re-run |
| Builds on your desktop, breaks on the Quest | newer C# than 9, or reflection | `just bridge-check`, then an actual IL2CPP build |
| Duplicate type definitions in Unity | a DLL got written inside `core/Teleop.Core/` | delete it; don't add output paths to the csproj |
| Works locally, fails in CI | filename case — Linux and CI are case-sensitive, NTFS isn't | match on-disk casing exactly |
| Prediction looks subtly wrong | coordinate conversion done twice | conversion happens **only** in `Bridge/CoordConversion.cs` |
| A result you can't reproduce | the run's SHA isn't reachable | tag before running anything citable; squash-merging orphans the SHA |

---

## 7. Your first contribution

A good first change is a new algorithm on an existing axis — it touches one folder and has a
well-worn path:

```bash
git switch -c my-change            # never commit to main; both machines share it
/new-impl reconciler my-idea       # scaffolds file + test + registry entry + benchmark row
just check                         # all gates
```

Then write up what you found in `docs/research-log/`, **including if it didn't work.**

Two rules worth stating explicitly because they surprise people:

- **One branch per machine.** Never check out a branch the other box is working on. Reach `main`
  through a PR instead.
- **Never rebase, amend, or rewrite history.** Merge instead.

## 8. Where to read next

| Read | When |
|---|---|
| Root `CLAUDE.md` | before touching `core/` — architecture, invariants, boundaries |
| `docs/metrics.md` | before reading or emitting any number |
| `docs/research-log/*-decisions.md` | to find out whether your idea was already tried |
| `docs/adr/` | when something looks oddly designed — the reason is here |
| `robot/README.md` | before any hardware work |

If something in this guide was wrong or missing, fix it — you are the last person who will ever see
it with fresh eyes.
