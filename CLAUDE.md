# Teleop Research Platform

VR teleoperation of a remote robot (Meta Quest + Unity). This is a **research platform, not
a product.** The deliverable is measured, reproducible results about latency mitigation:
prediction, reconciliation, jitter buffering, autonomy arbitration, view synthesis.

The consequence that shapes every decision: **an algorithm that cannot be evaluated
headlessly does not count.** If a change can only be verified by putting on a headset, it is
either in the wrong folder or built the wrong way.

**Direction, not yet architecture.** The intended next step is prediction that understands the
*physical situation* rather than only the trajectory — world models and other physical-AI models
that can say the arm is about to reach a wall and will stop there, or that a mass now in the
gripper has changed how it accelerates. It is meant to combine with the latency work above, not
replace it: those axes decide when a sample is shown and how its correction is absorbed, while a
model of the world improves what is shown at all.

Treat this paragraph as intent and nothing more. **Nothing in this system observes the environment
today** — the robot reports a pose and nothing else (57 fixed bytes, no field for anything sensed),
there is no camera, depth or contact signal anywhere, and `Plant/` is kinematic by deliberate
choice. Like `view synthesis` in the sentence above, this has no `Contracts/` interface, no folder
and no ADR. Giving it any of those is an architecture change under "Where new code goes", so the
ADR comes first. Do not write code against this paragraph.

## The one law

Dependencies point one direction. `Teleop.Core` sits at the bottom and depends on nothing.

```
analysis/  ──reads──>  results/  <──writes──  Teleop.Eval ──┐
                                                            ├──> Teleop.Core
                                            unity/TeleopVR ─┘     (depends on NOTHING)
```

Core must never reference `UnityEngine` and must never know Unity exists. Unity supplies
capabilities to Core by **implementing interfaces that Core declares** (`IRobotPlant`,
`ITransport`, `IInferenceBackend`, `IMetricSink`) — never by Core importing Unity.

## Directories

| Path | Built by | Notes |
|---|---|---|
| `core/Teleop.Core/` | **both** `dotnet` and Unity | all algorithms; one copy, two compilers |
| `core/Teleop.Core.Tests/` | `dotnet` | xUnit; must stay green |
| `core/Teleop.Eval/` | `dotnet` | headless CLI: replay, sweep, compare |
| `unity/TeleopVR/` | Unity | scenes, XR, rendering, real I/O |
| `robot/` | colcon (ROS 2) | independent; does not interact with the above builds |
| `analysis/` | nothing | Python; reads `results/`, exports `.onnx` |
| `experiments/` | — | one YAML per experiment |
| `results/` | — | append-only; every run has a `manifest.json` |

`core/Teleop.Core/` is a local UPM package (`package.json` + `.asmdef`) *and* a .NET project
(`.csproj`) in the same folder. Unity resolves it via a relative `file:` path in
`unity/TeleopVR/Packages/manifest.json`. Do not duplicate, copy, or vendor it.

## Invariants — do not violate, do not "improve"

1. **No `UnityEngine` in Core.** Enforced by `noEngineReferences: true` and by CI.
2. **No wall-clock reads in Core.** Time arrives via injected `ITimeAuthority`. No
   `DateTime.Now`, no `Stopwatch` construction, no `Time.time`. This is what makes replay
   deterministic and latency figures trustworthy.
3. **No I/O in Core.** Core is a synchronous function of (observations, time). No sockets, no
   files, no threads. I/O belongs to the host.
4. **All randomness through an injected seeded RNG.** Never `new Random()`.
5. **Static registration only — no reflection.** IL2CPP is AOT: the stripper removes types
   nothing references directly, and there is no runtime codegen. Add entries to the tables in
   `Registry/Registries.cs` by hand. No `Activator.CreateInstance`, no `Expression.Compile`,
   no `Reflection.Emit`.
6. **Core targets `netstandard2.1` with `LangVersion 9.0`.** This editor is Unity 2022.3,
   which is C# 9. Do not retarget to `net8.0` and do not raise `LangVersion`. Banned because
   they compile under `dotnet` and break the Quest build while `dotnet test` stays green:
   file-scoped namespaces (`namespace X;` — use block-scoped), `global using`, collection
   expressions (`[1, 2]`), `required` members, primary constructors on classes.
7. **Zero NuGet dependencies in Core.** Need a library? Declare an interface in Core and
   implement it in `Teleop.Eval` (headless) and `Bridge/` (Unity).
8. **No allocations in the per-frame hot path.** There are allocation-assertion tests; keep
   them passing.
9. **New algorithm = new file + `Registries.cs` entry + unit test + benchmark row.** No
   exceptions. Use `/new-impl`.
10. **An unimplemented check must exit non-zero.** Never a stub that returns success. An
    always-passing gate manufactures confidence and is worse than no gate.

## Where new code goes

- New approach to an existing question → new file in the matching Core folder
  (`Prediction/`, `Reconciliation/`, `Buffering/`, `Transport/`, `Autonomy/`, `Plant/`) +
  registry entry + test. Nothing else changes.
- New question entirely → new interface in `Contracts/`, new folder, `Pipeline/` learns to
  wire it. This is an architecture change: write an ADR in `docs/adr/` first.
- Needs a `Transform`, `GameObject`, GPU, XR device, socket, or file → `unity/TeleopVR/
  Assets/Teleop/Runtime/Bridge/`. **Requires human review.**
- Reads a `metrics.csv` → `analysis/`, in Python.
- Is a number that will appear in a paper → it came from `results/` and it has a manifest.

## Verify your work

```bash
cd core
dotnet test                                        # unit + allocation tests
dotnet run --project Teleop.Eval -- verify         # replay a golden log twice; assert identical
dotnet run --project Teleop.Eval -- audit          # invariant check over the built assembly
```

`verify` and `audit` are the two that catch the failures unit tests miss. Run all three
before claiming a task is done. Never report success on the basis of a successful build alone.

**If you changed a Core signature, also run `just bridge-check`** (or `cd unity/BridgeCheck &&
dotnet build`). It compiles `unity/`'s `Bridge/` against Core headlessly, at Unity's own
`netstandard2.1`/C# 9 settings, and needs no Unity — so it runs on Linux too. This exists
because a Core interface change breaks `unity/` *silently*: nothing in `core/` references Bridge,
so all three gates above stay green while the editor no longer compiles. That happened twice on
2026-09-09 (ADR 0012 added a required `IPlayoutPolicy<Pose>` to `OperatorEndpoint`; both bridges
kept calling the old signature). It is **not** a substitute for opening the editor — it cannot see
serialization, scene wiring, or IL2CPP. See `unity/BridgeCheck/README.md`.

If [`just`](https://github.com/casey/just) is installed, the repo-root `justfile` wraps the
above plus `analysis/`'s test suite: `just core-check` runs all three `core/` gates,
`just bridge-check` compiles `Bridge/` against Core, `just test` runs `analysis/`'s pytest suite,
`just check` runs everything. `just --list` shows every
recipe (`sweep`, `report`, `analysis-setup`, `experiment-gui`, ...). This is a convenience wrapper,
not a new source of truth — the raw commands above and in `analysis/CLAUDE.md` still work
unchanged and are what CI/agents without `just` should fall back to.

## Testing the real robot

Always drive real hardware through a `just` recipe (`just move-arm`, `just clocksync-check`,
`just build-profile`, `just deploy-robothost`, ...), never by hand-typing the underlying
`dotnet run --project Teleop.Eval -- ...` invocation or raw SSH/scp/deploy commands — see root
`README.md`'s JetRover section for the current end-to-end procedure. If the operation you need
doesn't have a recipe yet and it's a real, reusable step (not a one-off diagnostic), add it to
the `justfile` as part of the same change instead of running it ad hoc — the next session (agent
or human) should not have to reinvent or reverse-engineer a deploy/test step that's already been
worked out once. `robot/README.md`'s incident log exists precisely because ad hoc hardware
commands got lost otherwise.

## Boundaries for agents

- **Free rein:** `core/`, `analysis/`, `experiments/`, `docs/`, `unity/`.
- **Never touch:** `results/` (append-only — write new directories, never edit old ones),
  `unity/TeleopVR/Library/`, `build/`, anything gitignored.
- `robot/` is documentation-only: ask first, and never command real hardware without a human
  watching (see "Testing the real robot").
- **Changes under `unity/TeleopVR/Assets/Teleop/Runtime/Bridge/` want human review** — not because
  of where they are edited, but because that is where real I/O, XR devices and hardware live, and
  `just bridge-check` compiles against stubs so it cannot catch anything about scenes, prefabs or
  rendering. An IL2CPP build is the only real check.
- **Pull before starting, and before opening Unity.** `unity/TeleopVR/Packages/manifest.json` links
  Core by relative path, so a Core change on `main` changes the Unity build the moment it is on
  disk — open the editor against a stale tree and you will debug a mismatch git already resolved.

  ```bash
  git fetch && git log --oneline HEAD..origin/main   # empty == nothing incoming
  ```
- Work on a branch and reach `main` through a PR. Never `git commit --amend`, rebase, or rewrite
  history. Do not `git push`, open a PR, or tag unless asked — when asked, that request is
  sufficient authority and no further confirmation is needed.

## Environment

**Two environments, and a path or a `dotnet` invocation that works in one may not work in the
other.** `core/` and `analysis/` can be developed and verified anywhere; Unity, the Quest and the
JetRover need the Windows side. This section is about what each environment can *do* — it is not an
ownership rule, and there is no longer a restriction on where any directory may be edited.

### Linux — `core/`, `analysis/`

- Native Ubuntu on ext4, its own clone at `~/Projects/teleoperation`. Shell is zsh. There is no
  `/mnt/c`, no WSL interop, and no Unity — so `unity/` can be *edited* here but not opened, and a
  change to it is unverified until an editor sees it.
- `dotnet` here is the **Linux** SDK, installed normally. The Windows side's "never install the
  Linux SDK" rule does not apply: that rule exists because two SDKs sharing one working tree's
  `build/`/`obj/` churn each other, and this clone is never touched by a Windows `dotnet`.
  Absolute paths work; none of the `wslpath` handling below applies.
- `git-lfs` must be installed before any working-tree-modifying git command, or LFS-tracked
  binaries get written as pointer text files. The LFS payloads are Unity assets only, so leaving
  them unfetched here is fine — but the filter has to exist.
- `core/Teleop.RobotHost.Tests`' Unix-domain-socket tests are gated by `LinuxOnlyFactAttribute`,
  so they **run here and skip on Windows**. This is the only box that exercises them.

### Windows — Unity, Quest, hardware testing

- Repo lives on NTFS at `C:\Users\andre\Projects\teleoperation` (required — Unity is a
  Windows app and cannot open a project over `\\wsl$\`). Reached from WSL as
  `/mnt/c/Users/andre/Projects/teleoperation`.
- Shell is zsh under WSL, but `dotnet` is the **Windows** SDK, reached via a wrapper at
  `~/.local/bin/dotnet`. Never install the Linux SDK *on this machine* — two SDKs sharing
  `build/` and `obj/` cause rebuild churn and restore errors. That is a constraint on this
  machine's shared tree, not a rule about who may change what. Pass relative paths only; WSL
  translates the CWD for Windows processes but not arguments. Use
  `$(wslpath -w <path>)` if an absolute path is unavoidable.
- Unity, `adb`, and Unity CLI builds run on the Windows side. `git` and `git-lfs` are
  installed in WSL; never run a working-tree-modifying git command from a shell
  without `git-lfs`, or LFS-tracked binaries get written as pointer text files.
- **Unity 2022.3.46f1** — C# 9, API Compatibility Level `.NET Standard 2.1`. Sentis
  requires 2023.2+, so on-device ML inference goes through `IInferenceBackend` with a
  backend chosen at Phase 7; do not write `using Unity.Sentis` anywhere.

### Both

- CI runs on Linux and paths are case-sensitive there, as they are on native Linux. NTFS is
  not, so a casing mismatch may work in Unity and fail everywhere else. Match on-disk casing
  exactly.
- Core's `netstandard2.1`/C# 9 constraint (invariant 6) comes from the Unity editor and binds
  **everywhere**. Code can compile and test green under the Linux SDK and still break the Quest build; `audit` plus invariant 6's
  banned-feature list is the headless approximation, not a guarantee. An IL2CPP build is the
  only real check.

## Traps that have bitten this repo before

- **`bin/`/`obj/` inside the package folder.** `Directory.Build.props` redirects MSBuild
  output to `core/../build/`. If you see duplicate type definitions in Unity, something wrote
  a DLL into `core/Teleop.Core/`. Don't add output paths to the csproj.
- **`.meta` files inside `core/Teleop.Core/`.** Unity writes these into local packages. They
  are tracked on purpose. Do not delete or gitignore them.
- **Coordinate handedness.** Core is ROS convention: right-handed, Z-up, X-forward.
  Conversion happens *only* in `Bridge/CoordConversion.cs`. A second conversion site produces
  bugs that look exactly like prediction error.
- **Squash-merging a branch that produced results** makes the SHA in `manifest.json`
  unreachable. Tag before running anything citable.

## Vocabulary

- **horizon** — how far ahead a predictor is asked to extrapolate, in ms.
- **playout** — the moment a received sample is consumed for rendering or actuation.
- **correction cost** — magnitude/rate of visual correction after truth arrives. The
  nausea proxy, and the counterweight to prediction aggressiveness.
- **plant** — the robot being commanded (`IRobotPlant`): Unity physics, a Core rigid-body
  approximation, or real hardware.
- **golden log** — a committed `.tlog` in `core/testdata/` that tests replay against.

Metric definitions are in `docs/metrics.md`. Never invent a metric; if it isn't defined
there, add the definition in the same PR.
