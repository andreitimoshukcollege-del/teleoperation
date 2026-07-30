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

## Where new code goes

- New approach to an existing question → new file in the matching Core folder
  (`Prediction/`, `Reconciliation/`, `Buffering/`, `Transport/`, `Autonomy/`) + registry entry
  + test. Nothing else changes.
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

## Boundaries for agents

- Free rein: `core/`, `analysis/`, `experiments/`, `docs/`.
- **Ask first:** anything under `unity/`. Scene wiring and XR rig behavior cannot be verified
  headlessly, so a human checks it.
- Never touch: `results/` (append-only — write new directories, never edit old ones),
  `unity/TeleopVR/Library/`, `build/`, anything gitignored.
- Never run `git push`, `git commit --amend`, or rewrite history.

## Environment

- Windows dev box; repo root is `C:\Users\andre\Projects\teleoperation`.
- **Unity 2022.3.46f1** — C# 9, API Compatibility Level `.NET Standard 2.1`. Sentis requires
  2023.2+, so on-device ML inference goes through `IInferenceBackend` with a backend chosen at
  Phase 7; do not write `using Unity.Sentis` anywhere.
- CI runs on Linux. **Paths are case-sensitive there even though Windows tolerates
  mismatches** — match the on-disk casing of files, folders, and namespaces exactly. A CI
  failure on a path that obviously exists is almost always a casing mismatch.
- PowerShell for `dotnet`, `git`, `adb`, Unity CLI. Git Bash for `scripts/*.sh`.

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
