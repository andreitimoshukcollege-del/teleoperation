# Unity — shared rules for every project under `unity/`

There are two Unity projects and one shared Unity package (docs/adr/0016):

| Path | Editor | Headset | Pipeline | Notes |
|---|---|---|---|---|
| `unity/TeleopVR/` | 2022.3.46f1 | Meta Quest (Link from the editor, or APK) | built-in | the measured baseline; ADR 0001 pins its editor. Rules in `TeleopVR/Assets/Teleop/CLAUDE.md` |
| `unity/TeleopXR/` | Unity 6.3 LTS (≥ 6000.3.5f2) | Samsung Galaxy XR (Direct Preview over USB-C, or APK) | URP | hand tracking, no Meta SDK. Rules in `TeleopXR/CLAUDE.md` |
| `unity/Teleop.Bridge/` | both | — | either | the shared UPM package (`com.teleop.bridge`); rules in its `CLAUDE.md` |

Both projects reference `core/Teleop.Core`, `core/Teleop.RobotArm` and `unity/Teleop.Bridge` by
relative `file:` paths in `Packages/manifest.json`. **Never copy any of them into a project.** Two
copies drift and nothing catches it.

**Everything under `unity/` requires human review before merge.** Scene wiring, XR rig
behavior, and rendering cannot be verified headlessly, so CI cannot catch mistakes here.
Propose changes and explain them; do not merge them.

Unity's job is to **host and drive** Core. It is never a participant. Every MonoBehaviour is a
thin adapter. If a file here contains a filter coefficient, a blend curve, or a buffering
decision, that logic belongs in Core and this is a bug.

## Assembly boundaries — these are enforced by the compiler

```
Teleop.Core      (noEngineReferences: true)   <-- structurally blind to Unity
     ^
Teleop.Bridge    references Teleop.Core       <-- the ONLY place both worlds appear
     ^
Operator / RobotSim / Diagnostics             <-- reference Bridge ONLY, never Core
```

Do not add `Teleop.Core` to the references of `Operator.asmdef` or `RobotSim.asmdef`. The
omission is deliberate: it makes "XR code reaches into a predictor" a compile error.

## Shared code compiles under both editors

`Teleop.Bridge` is compiled by Unity 2022.3 *and* Unity 6.3. Both are C# 9 against .NET Standard
2.1, so root CLAUDE.md invariant 6 is the same rule twice. Beyond the language level:

- **The Unity API used must exist, and not be obsolete, in both.** For example, use
  `FindAnyObjectByType` (2021.3.18+), never `FindObjectOfType`, which Unity 6 marks obsolete.
- **Assume neither render pipeline.** Shader property names differ: built-in reads `_Color` and
  `_MainTex`, URP reads `_BaseColor` and `_BaseMap`. Set both names, or go through
  `Material.mainTexture`, which maps to the shader's `[MainTexture]` property.
- **TextMeshPro comes from the project:** `com.unity.textmeshpro` on 2022.3, `com.unity.ugui` 2.x
  on Unity 6. Both expose the `Unity.TextMeshPro` assembly the Bridge asmdef references, so the
  package does not declare it as a dependency.
- `just bridge-check` compiles Bridge against hand-written stubs, which is necessary but not
  sufficient. A change that touches the Unity API surface is also compiled against each editor's
  real assemblies (`<Editor>/Data/Managed/UnityEngine/*.dll`) before it is merged.

## Callback placement is a latency decision

| Callback | What belongs there |
|---|---|
| network thread | `TryReceive`, stamp arrival, push to a lock-free queue |
| `FixedUpdate` | digital-twin physics only |
| `Update` | capture controller poses -> `SubmitCommand`; drain `TryReceiveState`, **then** drain `TryPlayoutState` |
| `Application.onBeforeRender` | `EstimateRobotState` -> write Transforms |

State estimation goes in `onBeforeRender`, not `Update`: it is the last hook before rendering,
so the prediction target sits as close as possible to photon emission. Moving it to `Update`
adds a frame of avoidable staleness to the one number this project exists to measure. Do not
"simplify" it into `Update`.

**The two drains are one step, not two.** `TryReceiveState` does the arrival work -- `ClockSync`,
`owd_uplink_ms`/`owd_downlink_ms` -- and hands the sample to the injected `IPlayoutPolicy`;
`TryPlayoutState` is what stamps `t_playout`, folds the sample into the predictor and reconciler,
and returns the completed `LatencyTrace`
(`docs/adr/0012-playout-policy-wiring.md`). A bridge that drains only the first compiles, runs, and
is wrong in two ways at once: the ghost robot freezes, because nothing reaches the predictor any
more, and every recorded `.tlog` silently loses `t_playout`. Neither failure raises anything.

## IL2CPP / Android (both headsets)

- ARM64 + IL2CPP + Vulkan in both projects. API Compatibility Level stays `.NET Standard 2.1`.
  The scripting backend stays IL2CPP (Mono has no ARM64 Android backend).
- No reflection-based construction anywhere in the runtime path — the stripper removes what
  nothing references and AOT has no runtime codegen. Failures appear on device only.
- Managed Stripping Level stays `Low` while baselines are being established.
- `Internet Access` must be `Require`; auto-detection is unreliable with custom socket code.
- No arbitrary filesystem paths. Defaults load from `Resources` as a `TextAsset`; overrides
  from `Application.persistentDataPath` (pushed with `adb push`, no rebuild).
- No `Debug.Log` in the hot path — it allocates and is slow. Route diagnostics through the
  preallocated ring buffer that the recorder drains.

## Time

`Stopwatch.GetTimestamp()` via `MonotonicClock`, never `Time.time` (frame-quantized, resets on
scene load, stops in a paused editor). `clock.DisplayOffset` is time-until-photons; it comes
from OpenXR `predictedDisplayTime` where available, otherwise a per-headset calibrated
constant measured with the photodiode rig.
