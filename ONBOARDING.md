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

### Where the project is heading

Worth knowing on day one, because it reframes what "prediction" means here.

Every predictor built so far extrapolates from the robot's **own recent motion and nothing else**.
`IPredictor.Observe` takes a timestamped pose and has no second input channel — so the predictor is
a dead-reckoner. It has no idea there is a wall in front of the arm, and no idea the gripper just
closed on something with weight. Ask it where the arm will be in 300 ms and it will cheerfully
answer with a position inside the wall, or with the light-and-empty trajectory for an arm that is
now carrying a load.

The next direction is to give the prediction a model of the **physical situation** rather than just
the trajectory — world models and other physical-AI models that understand the environment well
enough to simulate what is about to happen:

- the arm is approaching a wall, so predict it **stopping there**, and show that before the real
  arm has arrived;
- the gripper has picked up a ball with mass, so the dynamics have changed — predict accordingly.

**It combines with the network work rather than replacing it,** and that is the point of doing both.
The existing axes decide *when* a sample is shown and how its correction is absorbed. A model of the
world improves *what* gets shown. The two multiply: a prediction that is right more often leaves
less correction to hide and needs less buffer to cover.

**Now the honest part, because otherwise this reads like a description of something that exists.**
It does not. Nothing in this system observes the environment — at any layer:

- What the robot reports is its **own body**, never its surroundings:
  - `RobotStateFrame`, 57 fixed bytes: the wrist pose plus clock bookkeeping.
  - The joint-state stream (docs/adr/0018): each servo's angle, and whether it was measured or only
    commanded.
- There is a **camera feed** (docs/adr/0014): the arm's Orbbec DaBai streams colour video, shown on
  the display in the headset. It goes to the operator's eyes only. Nothing interprets the image,
  the depth sensor is not streamed, and there is no contact sensing.
- The plant is kinematic on purpose — `Plant/CLAUDE.md`: *"no mass, no forces, no inertia, no
  contact, no joint limits, no actuator dynamics."*
- The one piece of environment-awareness in the whole project is a single hard-coded number that
  stops the arm driving into its own base plate, calibrated by a human watching it strain.

So: a direction, not a design. Nobody has decided how the environment gets sensed, how it reaches
Core without breaking the no-I/O invariant, or what the model actually is. If you are looking for
the perception layer, there isn't one yet — and finding that out from this paragraph is cheaper than
finding it out by grepping.

---

## 2. What do you need installed?

Depends on what you want to work on. Most work needs only the first row.

| You want to work on | Set up | Time |
|---|---|---|
| Algorithms, experiments, analysis (`core/`, `analysis/`) | track A | ~30 min |
| VR scenes, headset builds, rendering (`unity/`) | track A **+** B | ~2 h incl. Unity install |
| The physical robot arm | A + B + supervised access — track C | read first, act later |

If you're unsure, do **track A**. It covers every research axis, runs the whole test suite, and
produces real results without Unity and without any hardware.

> **Why is Unity separate?** It's a Windows application and cannot open a project stored inside
> WSL, so the Unity checkout lives on NTFS. On that machine `dotnet` is the *Windows* SDK even from
> a WSL shell. That's a fact about what runs where, not a rule about who may edit what — every
> directory in this repo is editable from anywhere. Root `CLAUDE.md`'s "Environment" section is the
> full reference.

---

## Track A — algorithms and experiments (Linux or WSL)

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
Windows side; the scriptable path that CI uses has never needed a display. If you do want it:
`sudo apt install python3-tk`.

`just analysis-setup` auto-detects the right interpreter for whichever machine you're on, so you do not
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

## Track B — Unity and the headset (Windows)

Only needed for VR scenes, rendering, or headset builds.

**The headset is the Samsung Galaxy XR** (`unity/TeleopXR`, Unity 6000.6.4f1, hand tracking,
docs/adr/0016). The Meta Quest project (`unity/TeleopVR`, Unity 2022.3.46f1) is parked: kept in the
repo in case it is wanted again, but no longer developed or tested. Its shared code still has to
compile under both editors (`unity/CLAUDE.md`).

### B1. What's different here

- The repo must live on **NTFS** (`C:\Users\<you>\Projects\teleoperation`), reached from WSL as
  `/mnt/c/...`. Unity cannot open a project over `\\wsl$\`.
- `dotnet` is the **Windows** SDK even from a WSL shell. It does not resolve WSL-native absolute
  paths passed as arguments — only the working directory is translated. **Pass relative paths**, or
  wrap with `$(wslpath -w <path>)`.
- **Do not install the Linux .NET SDK on this machine.** Two SDKs sharing one `build/`/`obj/` tree
  churn each other and produce restore errors that look like corruption.

### B2. Install

1. **Unity 6000.6.4f1**, this exact version, with the **Android** build support module (OpenJDK,
   SDK and NDK).
2. `git` and `git-lfs` inside WSL. Same warning as track A: LFS first, then clone. The lab's
   lighting, its mesh library and the robot's CAD model are all LFS.
3. For Direct Preview (Play mode streamed to the headset), Google's **Android XR Engine Hub**, once
   per PC. `unity/TeleopXR/CLAUDE.md` has the steps.

### B3. Open it

Run `just unity-setup-xr` **twice** before the first open (it imports XR samples that are not
committed, and they compile only after the first run ends). Then open `unity/TeleopXR/` in Unity
Hub. Core, RobotArm and the shared Bridge are linked by **relative path** from
`Packages/manifest.json`, so the C# algorithms are the *same files* `dotnet test` runs against.
There is no copy and no sync step.

The consequence: **pull `main` before opening the editor.** A Core change that landed since your
last pull changes the Unity build immediately.

The scene is `Assets/Scenes/JetRoverLabXR.unity`: a generated lab with the robot's digital twin, the
camera display, and hand-tracked clutch control of the arm. `unity/TeleopXR/CLAUDE.md` describes
every part of it, its safety settings, and how to put it on the headset:

```bash
just build-galaxy          # the APK, after the scene's safety check
just install-galaxy        # over USB
just push-galaxy-config    # point it at the robot's current address; restart the app
just mirror-galaxy         # watch the headset's view on the PC (scrcpy)
```

### B4. Verify before you commit

```bash
just bridge-check        # compiles Bridge/ against Core headlessly
```

```bash
just unity-check-xr      # opens the scene headless: missing scripts, broken wiring, safety settings
just unity-lab-xr        # rebuilds the generated lab and renders preview images to %TEMP%
```

`bridge-check` catches signature breaks fast, but it is **not a substitute for opening the
editor**: it compiles against UnityEngine stubs, so it cannot catch anything about scenes, prefabs
or rendering. An IL2CPP build of the headset app (`just build-galaxy`) is the only real check that
Core's C# 9 constraint holds.

---

## Track C — the physical robot (read this before touching anything)

The robot is a Hiwonder JetRover with a Jetson Nano. On the lab network it is found by its mDNS
name, `jetrover-sinrg.local`, with Tailscale as the fallback; its address changes (DHCP), so never
hard-code it (`just robot-ip`). Everything that runs on it is in this repository:
- `core/Teleop.RobotHost` and `core/Teleop.CameraHost` (.NET);
- the ROS 2 workspace in `robot/ros/` that drives the servo board (docs/adr/0019);
- the systemd units in `robot/systemd/`.

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
just robot-status              # the Jetson services, no manual SSH
just joint-probe               # read-only: is joint state arriving, and which motors are measured
just robot-battery             # read-only: battery voltage
just deploy-robot              # redeploy everything on the robot and check it (someone at the robot)
just deploy-robothost          # ...or one piece: Teleop.RobotHost after a Core change
just deploy-ros                # robot/ros/ only (restarts the servo driver)
just move-arm 0.15 0 0.08      # Cartesian target in metres -- MOVES REAL HARDWARE
```

**Low battery looks like a software fault.** When the 3-cell pack runs low (it was at 9.25 V the
first time, 2026-10-14; full is 12.6 V), the servo board stops answering. The arm ignores commands, the joint values freeze, the camera drops off USB, and the
board beeps. Charge it with the robot switched off: running, it draws more than the charger
supplies.

---

## 5. How the project actually works

Setup is the easy part. This is the part that makes the repo make sense.

### The one law

**Dependencies point one direction.** `Teleop.Core` sits at the bottom and depends on nothing —
not Unity, not the network, not the filesystem, not even a clock.

```
analysis/  ──reads──>  results/  <──writes──  Teleop.Eval ───────────────┐
                                                                         ├──> Teleop.Core
                      unity/TeleopXR ─┬──> unity/Teleop.Bridge ──────────┘     (depends on NOTHING)
                      unity/TeleopVR ─┘
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
- **No reflection** → the headset build's ahead-of-time compiler (IL2CPP) strips code nothing
  references directly, so reflection compiles fine on your desktop and crashes on the headset.
- **C# 9 / netstandard2.1** → both Unity editors' compiler (Unity 6.6 for the Galaxy XR, 2022.3 for
  the parked Quest project). Newer syntax passes `dotnet test` on Linux and breaks the headset
  build silently.

### The research loop

1. **A question.** "Does adapting the buffer beat a fixed one?"
2. **An implementation** — one new file in the matching axis folder (`Prediction/`,
   `Reconciliation/`, `Buffering/`, `Transport/`, `Autonomy/`, `Plant/`) plus a hand-written entry
   in `Registry/Registries.cs`. Use `/new-impl` to scaffold all the pieces at once.
3. **An experiment** — a YAML in `experiments/` naming the algorithms, network profiles and seeds.
4. **A sweep** → `results/<id>/<timestamp>/` with a `manifest.json` recording the git SHA.
5. **A written verdict** in `docs/research-log/` — including the negatives. **A finished negative
   result beats three unfinished positive ones.**

Two things named in §1 have no folder in that list and so no home in this loop yet: the
world-model direction, and the *view synthesis* the root `CLAUDE.md` scope sentence has always
claimed. Both would be a new `Contracts/` interface plus a new folder plus `Pipeline/` learning to
wire it — which root `CLAUDE.md` calls an architecture change and requires an ADR for, **before**
the code. Nobody has written that ADR.

### Where things live

| Path | What |
|---|---|
| `core/Teleop.Core/` | every algorithm; one copy compiled by both `dotnet` and Unity |
| `core/Teleop.Eval/` | the headless CLI: `sweep`, `verify`, `audit` |
| `core/Teleop.RobotArm/` | arm kinematics, the clutch and gripper mappings, joint wire formats |
| `core/Teleop.RobotHost/`, `core/Teleop.CameraHost/` | what runs on the robot's Jetson (.NET) |
| `unity/TeleopXR/` | the Galaxy XR operator app (the headset in use) |
| `unity/Teleop.Bridge/` | the only Unity code that touches Core; shared by both Unity projects |
| `unity/TeleopVR/` | the Meta Quest operator app (parked) |
| `robot/ros/` | the robot's ROS 2 workspace: servo board driver and relay |
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
| Builds on your desktop, breaks on the headset | newer C# than 9, or reflection | `just bridge-check`, then an actual IL2CPP build (`just build-galaxy`) |
| Robot arm ignores commands, camera keeps dropping, robot beeps | battery low | charge it with the robot switched off; `just robot-battery` |
| Duplicate type definitions in Unity | a DLL got written inside `core/Teleop.Core/` | delete it; don't add output paths to the csproj |
| Works locally, fails in CI | filename case — Linux and CI are case-sensitive, NTFS isn't | match on-disk casing exactly |
| Prediction looks subtly wrong | coordinate conversion done twice | conversion happens **only** in `Bridge/CoordConversion.cs` |
| A result you can't reproduce | the run's SHA isn't reachable | tag before running anything citable; squash-merging orphans the SHA |

---

## 7. Your first contribution

A good first change is a new algorithm on an existing axis — it touches one folder and has a
well-worn path:

```bash
git switch -c my-change            # work on a branch; reach main through a PR
/new-impl reconciler my-idea       # scaffolds file + test + registry entry + benchmark row
just check                         # all gates
```

Then write up what you found in `docs/research-log/`, **including if it didn't work.**

Two rules worth stating explicitly because they surprise people:

- **`results/` is append-only.** Write new directories; never edit or delete an old one. A recorded
  run is evidence, even a bad one.
- **Never rebase, amend, or rewrite history.** Merge instead.

## 8. Where to read next

| Read | When |
|---|---|
| Root `CLAUDE.md` | before touching `core/` — architecture, invariants, boundaries |
| `docs/metrics.md` | before reading or emitting any number |
| `docs/research-log/*-decisions.md` | to find out whether your idea was already tried |
| `docs/adr/` | when something looks oddly designed — the reason is here |
| `robot/README.md` | before any hardware work |
| `unity/TeleopXR/CLAUDE.md` | before working on the headset app |

If something in this guide was wrong or missing, fix it — you are the last person who will ever see
it with fresh eyes.
