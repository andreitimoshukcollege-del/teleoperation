# Unity — TeleopVR

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

## Bridge/ contains exactly two kinds of file

**Adapters** — drive Core from Unity callbacks: `TeleopOperatorBridge`, `TeleopRobotBridge`,
`CoordConversion`, `ConfigLoader`, `XrDisplayTimeProvider`.

**Implementations of Core interfaces** — the direction inverts here. Core declares, Unity
provides: `UnityRobotPlant : IRobotPlant`, `UdpTransport : ITransport`,
`UnityMetricSink : IMetricSink`, and eventually an `IInferenceBackend` (see Sentis note below).

Bridge should stay small: roughly a dozen files, mostly under 100 lines. Growth means logic is
leaking out of Core, and every leaked line is a line the headless sweeps can no longer test.

## Callback placement is a latency decision

| Callback | What belongs there |
|---|---|
| network thread | `TryReceive`, stamp arrival, push to a lock-free queue |
| `FixedUpdate` | digital-twin physics only |
| `Update` | drain inbound queue -> `Observe`; capture controller poses -> `SubmitCommand` |
| `Application.onBeforeRender` | `EstimateRobotState` -> write Transforms |

State estimation goes in `onBeforeRender`, not `Update`: it is the last hook before rendering,
so the prediction target sits as close as possible to photon emission. Moving it to `Update`
adds a frame of avoidable staleness to the one number this project exists to measure. Do not
"simplify" it into `Update`.

## Quest / IL2CPP constraints

- Editor is **Unity 2022.3.46f1**. ARM64 + IL2CPP + Vulkan; OpenXR with the Meta feature group.
- API Compatibility Level stays `.NET Standard 2.1`. Scripting backend stays IL2CPP (Mono has
  no ARM64 Android backend).
- **Sentis is unavailable on 2022.3** (it needs 2023.2+), and Barracuda is deprecated. So
  `IInferenceBackend` has no Unity implementation yet — that is deliberate and blocks nothing
  until Phase 7. Do not add `using Unity.Sentis`.
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

## Scenes

`Operator.unity` and `RobotTwin.unity` are two scenes and two build profiles in **one** Unity
project. Keep scenes thin — a handful of GameObjects whose behavior lives in scripts — because
Unity YAML merges badly. Prefabs over deep hierarchy, config over inspector values.

Asset Serialization is `Force Text`. Do not change it; binary scenes are unmergeable.
