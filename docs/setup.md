# Setup

Repo root: `C:\Users\andre\Projects\teleoperation` · Windows · Unity 2022.3.46f1

Each phase ends in a **gate**. Don't pass a gate you haven't checked — every one catches a
class of problem that gets exponentially more confusing later. PowerShell unless noted.

- [x] **Phase 0 — repo restructure.** Done. Unity project at `unity\TeleopVR`, skeleton dirs
      created, root has `.vscode analysis core docs experiments results robot scripts unity`.

---

## Phase 1 — Core, wired to both compilers

The fiddly part. **Do this by hand.** Mistakes here produce cascading errors that look like a
hundred unrelated problems, and an agent will confidently "fix" them in ways that break the
other compiler.

### 1.1 Drop in the bundle

Copy the contents of `teleop-setup\` over the repo root, then:

```powershell
cd C:\Users\andre\Projects\teleoperation
.\scripts\setup-dev.ps1
```

That checks your toolchain, sets `core.longpaths` / `core.autocrlf`, installs LFS, wires the
UnityYAMLMerge driver, and fixes the hook's line endings if needed.

### 1.2 Create the .NET projects

Do **not** run `dotnet new classlib` for Teleop.Core — the bundle already ships a correct
`Teleop.Core.csproj`, and the template would overwrite it with a `net8.0` default. Only
scaffold the two projects that don't exist yet:

```powershell
cd core
dotnet new sln -n Teleop
dotnet new xunit   -n Teleop.Core.Tests -o Teleop.Core.Tests
dotnet new console -n Teleop.Eval       -o Teleop.Eval
dotnet sln add Teleop.Core\Teleop.Core.csproj Teleop.Core.Tests Teleop.Eval
dotnet add Teleop.Core.Tests reference Teleop.Core\Teleop.Core.csproj
dotnet add Teleop.Eval       reference Teleop.Core\Teleop.Core.csproj
cd ..
```

Tests and eval stay on `net8.0` — only Core is constrained by Unity.

### 1.3 Point Unity at Core

Add one line to `unity\TeleopVR\Packages\manifest.json`, leaving every existing entry alone:

```jsonc
"dependencies": {
  "com.teleop.core": "file:../../../core/Teleop.Core",
  "com.unity.xr.openxr": "...",
  ...
}
```

### 1.4 Attach the smoke test

The bundle already placed `Pose.cs`, `CoordConversion.cs`, `SmokeTest.cs`, and both `.asmdef`
files. In Unity: add a GameObject to any scene, attach `SmokeTest`, press Play.

### Gate 1 — all five

- [ ] `cd core; dotnet build` succeeds
- [ ] `core\Teleop.Core\` contains **no** `bin\` or `obj\` (they belong in `build\`)
- [ ] Unity's Project window shows *Teleop Core* under **Packages**, no console errors
- [ ] Press Play → four `[SmokeTest]` lines, both round-trips **PASS**
- [ ] **Build an APK, sideload it, `adb logcat -s Unity` shows the same four lines**

That last one is the highest-value step in the whole setup. You are validating
asmdef → managed DLL → IL2CPP → C++ → NDK → APK with a fifteen-line payload. Every AOT
problem — stripping, reflection, language level — surfaces here or not at all. You already
shipped an APK from this project, so the toolchain is proven; this confirms the *package*
survives it.

If Unity reports `namespace Teleop not found`: check Package Manager first. It's one bad
relative path or a `"unity"` field mismatch in `package.json`, not a hundred real errors.

Delete `SmokeTest.cs` once green.

---

## Phase 2 — Agent config and CI

```powershell
git add -A
git commit -m "Phase 1: Core dual-build wiring, smoke test green"
```

Add the three fast-tier workflows (`dotnet test`, `Teleop.Eval verify`, `Teleop.Eval audit`),
branch-protect `main` on them, then **start a fresh Claude Code session** — subagents load at
session start, so edits to `.claude\agents\` won't take effect until you restart.

### Gate 2

- [ ] `/check-core` runs and reports
- [ ] Editing a Core file triggers the guard hook (test it: temporarily add
      `var x = DateTime.UtcNow;` to `Pose.cs`, save, confirm the hook fires, revert)
- [ ] CI green on a trivial PR
- [ ] `/permissions` shows the allow/ask/deny rules resolved as expected

---

## Phase 3 — Time, clock sync, recording

First phase worth delegating heavily to `algorithm-implementer`.

Build: `ITimeAuthority`, `MonotonicClock` (Stopwatch-based), `ManualClock` (stepped by hand),
`ClockSync`, `Stamped<T>`, `SessionWriter`/`SessionReader` with a versioned `RecordFormat`,
`IMetricSink` + `CsvMetricSink`, `LoopbackTransport`, `EmulatedTransport`, and the `verify` /
`audit` subcommands in `Teleop.Eval` that everything else references.

### Gate 3

- [ ] Inject a synthetic **137 ms** through `EmulatedTransport` over `LoopbackTransport`; the
      measurement pipeline reports 137 ± 1 ms
- [ ] Replay a golden `.tlog` twice → byte-identical output
- [ ] `audit` passes over the built assembly

Nothing after this point is trustworthy if this gate is soft. A latency research platform that
measures latency wrong produces confident, publishable, wrong results. Budget a few days; it's
less fun than predictors and it is the foundation.

---

## Phase 4 — Loopback baseline

VR pose → sim robot → back → render, zero mitigation, in-VR latency HUD. Ugly on purpose.
You'll cite this number in every comparison you ever make.

Your existing XR rig is the operator side — `TeleopOperatorBridge` attaches to it. Don't
rebuild it.

Callback placement (this is a latency decision, not a style choice):

| Callback | What goes there |
|---|---|
| network thread | `TryReceive`, stamp arrival, lock-free queue |
| `FixedUpdate` | digital-twin physics only |
| `Update` | drain queue → `Observe`; capture poses → `SubmitCommand` |
| `Application.onBeforeRender` | `EstimateRobotState` → write Transforms |

### Gate 4

- [ ] Recorded session with a measured end-to-end motion-to-photon figure
- [ ] **Physical validation** — LED + photodiode, or high-speed camera on a spinning marker —
      agreeing with the software estimate
- [ ] `DisplayOffset` calibrated for this headset and refresh rate

---

## Phase 5 — First sweep

Real network profiles in `core\testdata\traces\`, trace replay, and three predictors: `none`,
`const-vel`, `double-exp`.

### Gate 5

- [ ] `/run-sweep experiments/exp-001-predictor-baseline.yaml` produces a manifested directory
      under `results\` with a reachable git SHA
- [ ] p50/p95/p99 table across five network profiles
- [ ] Prediction error **and** correction cost reported together
- [ ] Baseline (`none` / `snap`) row present

Reaching Gate 5 is the real milestone. After it, trying a new idea costs one `/new-impl` and
one `/run-sweep` — which is the entire reason for this architecture. Everything before it is
scaffolding; everything after is research.

---

## Summary

| Phase | Effort | Delegate? |
|---|---|---|
| 0 · restructure | done | — |
| 1 · Core dual-build | hours | **no** — build plumbing |
| 2 · agents + CI | ~1 hr | **no** |
| 3 · time + recording | days | yes |
| 4 · loopback baseline | ~1 day | Core yes, Unity review |
| 5 · first sweep | days | yes |

## Open decision

**Stay on Unity 2022.3, or upgrade?** 2022.3 LTS is past its support window, and Sentis needs
2023.2+. If you're going to upgrade, do it **before Phase 4** — after that you have a
physically-validated motion-to-photon baseline and a frozen benchmark suite, and an editor
change alters the render path and XR plugin stack, invalidating comparability with everything
recorded before it. Right now the project is nearly empty and the upgrade costs an afternoon.

Staying is defensible: it works, the APK chain is proven, and toolchain stability has real
value in a platform whose job is trustworthy measurement. If you stay, write
`docs\adr\0001-unity-2022-3.md` so the reason is recorded rather than reconstructed.

## Which shell for what

- **PowerShell** — `dotnet`, `git`, `adb`, Unity CLI, the migration and setup scripts
- **Git Bash** — `scripts\hooks\*.sh`
- **Claude Code** — launch from either; uses Git Bash internally for its Bash tool
- **Rider / VS** — open `core\Teleop.sln`, never Unity's generated `.csproj`
