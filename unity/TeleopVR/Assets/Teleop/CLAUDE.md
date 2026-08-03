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
`CoordConversion`, `ConfigLoader`, `XrDisplayTimeProvider`, `LatencyHud` (display only, reads the
metric sink and writes nothing back). `DisplayCalibrationConfig` is the plain data type
`ConfigLoader` loads, not an adapter itself.

**Implementations of Core interfaces** — the direction inverts here. Core declares, Unity
provides: `UnityRobotPlant : IRobotPlant` (not yet built — Phase 4 reuses Core's own
`RigidBodyPlant` directly instead, see `Plant/CLAUDE.md`), `UdpTransport : ITransport` (not yet
built — Phase 4 is in-process only), `UnityMetricSink : IMetricSink` (built),
`UnityMonotonicClock : ITimeAuthority` (built; every Core component Bridge constructs needs one,
the same reasoning `Time/CLAUDE.md` gives for why `Teleop.Eval` has its own `MonotonicClock`),
and eventually an `IInferenceBackend` (see Sentis note below).

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

## Phase 4 scene wiring (`SampleScene.unity`)

The two-scene split above is the longer-term target; Phase 4's loopback baseline deliberately
runs both sides in-process in the one existing `SampleScene.unity` instead (no real transport
yet, so there is nothing to put in a second scene/process — see
`Bridge/TeleopOperatorBridge.cs`'s type doc). `SampleScene.unity` already has a real XRI rig
(`XR Origin`, `Left`/`Right Controller`, `XR Interaction Manager`) — do not rebuild it. This
wiring is hand-done in the Editor, not scripted or hand-edited into the `.unity` YAML: scene
wiring is exactly the kind of change this file's own first line says needs a human, and Unity's
own serializer should be the only thing writing scene YAML.

1. Add an empty GameObject `TeleopOperator` (anywhere in the hierarchy) with a
   `TeleopOperatorBridge` component. Set **Pose Source** to `Right Controller`'s `Transform`
   (the commanded end-effector pose should be a hand, not the head — `Left Controller` or
   `Main Camera` are equally valid if you want to teleoperate a different motion). Create a
   `GhostRobot` GameObject (a primitive is enough) and set it as **Ghost Robot Target**.
2. Add an empty GameObject `TeleopRobot` with a `TeleopRobotBridge` component. Set
   **Operator Bridge** to the `TeleopOperator` GameObject. Create a `GroundTruthRobot` GameObject
   (visually distinct from `GhostRobot` — e.g. a different color — so a correction's divergence
   is visible) and set it as **Ground Truth Target**.
3. Add a **World Space** Canvas positioned in front of the rig (parent it under `Main Camera`
   with a forward local offset, e.g. `(0, 0, 1.5)` — parenting under `XR Origin`'s root instead
   puts it at the tracking-space floor origin, not in front of your face). Add a child
   TextMeshPro `Text` element, and a `LatencyHud` component (on the Canvas or a dedicated
   GameObject) with **Operator Bridge** set to `TeleopOperator` and **Label** set to that
   TextMeshPro object.
4. Press Play. Console should be clean; moving the right controller should move both robot
   GameObjects; the HUD should show live `M2P`/`uplink OWD`/`downlink OWD` numbers. On stop, a
   `phase4-session-<timestamp>.tlog` should exist under `Application.persistentDataPath` (in the
   Editor on Windows: `%userprofile%\AppData\LocalLow\<CompanyName>\<ProductName>\`).

## Known-broken: `XRI Default Input Actions.inputactions`

`Assets/Samples/XR Interaction Toolkit/2.6.5/Starter Assets/XRI Default Input Actions.inputactions`
fails to import under the installed Input System version (1.19.0) — Console shows "Could not
parse input actions in JSON format... Failed to add object of type `InputActionReference`",
and the asset shows as an unusable "Default Asset" rather than a real Input Action Asset. This
is a genuine incompatibility between that file (vintage XRI 2.6.5, itself only a transitive
dependency at Input System ~1.7.0 originally) and 1.19.0's importer — not something introduced
by this project. **Do not spend time trying to fix the asset itself** (already tried; the
importer bug is upstream). Consequences:

- `XR Controller (Action-based)`'s Position/Rotation Action fields have nothing to bind to.
  `Bridge/../Scripts/XRNodePoseDriver.cs` (`unity/TeleopVR/Assets/Scripts/`, not `Bridge/` — it's
  a generic rig workaround, not a Core adapter) works around this by creating Input System
  actions **inline in code** with explicit binding paths (`<XRController>{RightHand}/pointerPosition`),
  which never touches the broken asset file at all. It's attached to both `Left Controller` and
  `Right Controller`, each with **Hand** set accordingly and **Origin** set to `XR Origin (XR Rig)`.
- Input System 1.19.0 is required regardless: XRI 2.6.5's own `Editor/Scripts/StarterAssetsSampleProjectValidation.cs`
  needed the newer `InputSystem.actions` API to compile at all (guarded by
  `UNITY_INPUT_SYSTEM_PROJECT_WIDE_ACTIONS`, which the Editor sets based on package version, not
  actual per-platform feature availability). That validation script did nothing but register
  Project Validation nag messages — zero runtime logic — so it was deleted rather than chased
  further.
- `Main Camera`'s `Tracked Pose Driver` can serialize into a corrupted state (renders zero
  Inspector fields, no console error) after Input System version churn. Fix is Remove Component
  → Add Component "Tracked Pose Driver" fresh, then set **Pose Source** to **Center Eye - HMD
  Reference**. Note this resolves to the **legacy** `UnityEngine.SpatialTracking.TrackedPoseDriver`
  (Device/Pose Source dropdown) when added this way, not the Input-System-actions one — that's
  fine for the camera (head-only poses), but that legacy component's Pose Source dropdown has
  **no hand/controller option at all**, which is why controllers need `XRNodePoseDriver` instead.
- **Pointer pose vs. device (grip) pose**: Touch controllers report both, tilted relative to each
  other by design (`pointerPosition`/`pointerRotation` vs `devicePosition`/`deviceRotation`). Ray
  interactors expect pointer pose. Driving a controller's Transform from the grip pose instead
  produces a ray that visibly points higher than where you're actually aiming — an easy mistake
  since `devicePosition`/`deviceRotation` are the more obvious/commonly-referenced names.
- A driven Transform must validate `TryGetFeatureValue`'s output isn't NaN/degenerate before
  assigning it, not just check the bool return — a garbage frame during device connect/reconnect
  can otherwise get written once and permanently break the ray interactor's UI raycasting
  (`Screen position out of view frustum (-nan(ind))`), surviving even a full Editor restart until
  the bad Transform value is overwritten by a valid frame.
