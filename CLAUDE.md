# Teleop Research Platform

VR teleoperation of a remote robot (Meta Quest + Unity). This is a **research platform, not
a product.** The deliverable is measured, reproducible results about latency mitigation:
prediction, reconciliation, jitter buffering, autonomy arbitration, view synthesis.

The consequence that shapes every decision: **an algorithm that cannot be evaluated
headlessly does not count.** If a change can only be verified by putting on a headset, it is
either in the wrong folder or built the wrong way.

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

If [`just`](https://github.com/casey/just) is installed, the repo-root `justfile` wraps the
above plus `analysis/`'s test suite: `just core-check` runs all three `core/` gates, `just test`
runs `analysis/`'s pytest suite, `just check` runs everything. `just --list` shows every
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

**Scope follows the machine, and the two are worked on at the same time.** Ownership is by
directory and is not advisory: an edit made on the wrong box is a merge conflict waiting to
happen, not a style violation. "Environment" below says what each box can *build*; this says what
each box may *change*.

### On the Linux box — `core/` and backend

- Free rein: `core/`, `analysis/`, `experiments/`, `docs/`.
- **Never edit `unity/`. Propose instead** — describe the change and let it be applied on Windows,
  where an editor can actually verify it. This is deliberately stricter than "ask first": Unity
  rewrites scenes, prefabs, `.asset` and `.meta` files by itself while it is open, so an edit made
  here can collide with a live editor session there. That is true even for plain C# under
  `Bridge/`, because saving it makes Unity recompile and touch adjacent files.
- `robot/` is documentation-only: ask first, and never command real hardware from this box (see
  "Testing the real robot").

### On the Windows box — Unity, Quest, hardware

- Free rein: `unity/`.
- Core changes belong on the Linux box, including Core problems *discovered* here. The standing
  exception is the `.meta` files Unity writes into `core/Teleop.Core/` — Unity is their author,
  they are tracked on purpose, and committing them from Windows is correct.
- `unity/TeleopVR/Packages/manifest.json` links Core by relative path, so Core edits landing on
  `main` change the Unity build immediately. Pull before opening the editor, or you will debug a
  mismatch that git already resolved.

### Both

- Never touch: `results/` (append-only — write new directories, never edit old ones),
  `unity/TeleopVR/Library/`, `build/`, anything gitignored.
- **One branch per machine.** Never check out or commit to a branch the other box is working on;
  reach `main` through a PR instead. Two machines committing to one branch diverge, and
  untangling that costs more than the PR ever does.
- **Check for incoming work before starting anything, on either box.** Not "pull if you happen to
  think of it" — actually look, every time, before the first edit:

  ```bash
  git fetch && git log --oneline HEAD..origin/main   # empty == nothing incoming; otherwise pull
  ```

  The two boxes commit independently and neither sees the other's work until it reaches `main`, so
  a stale tree is the normal state here, not the exception. Starting on one means either rebuilding
  something that already landed, or writing a change against code that has since moved — and the
  Windows box will not notice the second case until Unity recompiles, because
  `unity/TeleopVR/Packages/manifest.json` links Core by relative path and picks up whatever is on
  disk. If something *is* incoming, pull it and skim what changed before starting; a merge conflict
  found now costs a minute, and the same one found at PR time costs an afternoon.
- The real conflict surface is the files neither box exclusively owns: root `CLAUDE.md`,
  `justfile`, `.claude/`, `docs/metrics.md`, `Registry/Registries.cs`. Editing one is fine;
  editing one while the other box is mid-change is what hurts. Keep those changes small and merge
  them promptly rather than parking them on a long-lived branch.
- Never `git commit --amend`, rebase, or rewrite history. Do not `git push`, open a PR, or tag
  unless asked — when asked, that request is sufficient authority and no further confirmation is
  needed.

## Environment

**Two machines, two jobs.** A change to `core/` or `analysis/` can be developed and verified
entirely on the Linux box. Anything involving Unity, the Quest, or the JetRover needs the
Windows one. Check which box you are on before trusting a path or a `dotnet` invocation — the
two have opposite rules.

### Linux box — `core/`, `analysis/` (primary for algorithm work)

- Native Ubuntu on ext4, its own clone at `~/Projects/teleoperation`. Shell is zsh. There is no
  `/mnt/c`, no WSL interop, and no Unity.
- `dotnet` here is the **Linux** SDK, installed normally. The Windows box's "never install the
  Linux SDK" rule does not apply: that rule exists because two SDKs sharing one working tree's
  `build/`/`obj/` churn each other, and this clone is never touched by a Windows `dotnet`.
  Absolute paths work; none of the `wslpath` handling below applies.
- `git-lfs` must be installed before any working-tree-modifying git command, or LFS-tracked
  binaries get written as pointer text files. The LFS payloads are Unity assets only, so leaving
  them unfetched here is fine — but the filter has to exist.
- `unity/` changes are proposed here and built and reviewed on Windows; Unity cannot open this
  clone and is not expected to.
- `core/Teleop.RobotHost.Tests`' Unix-domain-socket tests are gated by `LinuxOnlyFactAttribute`,
  so they **run here and skip on Windows**. This is the only box that exercises them.

### Windows box — Unity, Quest, hardware testing

- Repo lives on NTFS at `C:\Users\andre\Projects\teleoperation` (required — Unity is a
  Windows app and cannot open a project over `\\wsl$\`). Reached from WSL as
  `/mnt/c/Users/andre/Projects/teleoperation`.
- Shell is zsh under WSL, but `dotnet` is the **Windows** SDK, reached via a wrapper at
  `~/.local/bin/dotnet`. Never install the Linux SDK *on this machine* — two SDKs sharing
  `build/` and `obj/` cause rebuild churn and restore errors. Pass relative paths only; WSL
  translates the CWD for Windows processes but not arguments. Use
  `$(wslpath -w <path>)` if an absolute path is unavoidable.
- Unity, `adb`, and Unity CLI builds run on the Windows side. `git` and `git-lfs` are
  installed in WSL; never run a working-tree-modifying git command from a shell
  without `git-lfs`, or LFS-tracked binaries get written as pointer text files.
- **Unity 2022.3.46f1** — C# 9, API Compatibility Level `.NET Standard 2.1`. Sentis
  requires 2023.2+, so on-device ML inference goes through `IInferenceBackend` with a
  backend chosen at Phase 7; do not write `using Unity.Sentis` anywhere.

### Both

- CI runs on Linux and paths are case-sensitive there, as they are on the Linux box. NTFS is
  not, so a casing mismatch may work in Unity and fail everywhere else. Match on-disk casing
  exactly.
- Core's `netstandard2.1`/C# 9 constraint (invariant 6) comes from the Windows box's Unity
  editor and binds **everywhere**. Code written on the Linux box can compile and test green
  under the Linux SDK and still break the Quest build; `audit` plus invariant 6's
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
