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

## Network impairment: the checkbox panel

Lets an operator switch lag, jitter, loss and reordering on and off during Play mode, so "what does
200ms feel like?" can be answered in a headset instead of argued about.

- `NetworkImpairment` + `Impairments/*.cs` — the axis contract and one file per axis. See the file
  layout section below.
- `NetworkImpairmentSettings` — the aggregate: one axis per field, and a
  `ToProfile(ticksPerSecond)` that folds the enabled ones into Core's `NetworkProfile`. Same
  category as `RobotArmProfileData` — a Unity-serializable surface feeding a constructor-only Core
  struct.
- `NetworkProfilePresets` — loads a frozen profile's values into the axes (see below).
- `DelayTraceLoader` — reads a recorded `.trace`, since Core cannot do I/O.
- `SwappableTransport : ITransport` — installs and removes an `EmulatedTransport` over its inner
  transport while running. Needed because the endpoints take their transports once, in their
  constructors, so without it every settings change would rebuild the endpoint stack and throw away
  `ClockSync`'s convergence and the recording with it.
- `NetworkImpairmentController` — the MonoBehaviour holding the checkboxes, wired to
  `TeleopOperatorBridge`'s two transports.

**That is a lot of files in a folder whose rule above is that growth is a warning sign, so the
justification matters.** None of them impair anything. Every delay, drop and reorder decision is
made by Core's `EmulatedTransport` from a Core `NetworkProfile`; what these add is a serializable
surface for it and one level of indirection so it can be swapped at runtime. The count is spread
across small single-purpose files precisely so each stays a description rather than a computation.
If a coefficient, a distribution, or a drop decision ever appears in any of them, that is the leak
this section's rule is about and it belongs back in Core.

### File layout: one file per impairment

`NetworkImpairmentSettings` is the aggregate; each axis is its own file under `Impairments/`
(`DelayImpairment`, `JitterImpairment`, `LossImpairment`, `ReorderImpairment`,
`DelayTraceImpairment`), all deriving from `NetworkImpairment`.

**Adding an axis** is a new file plus a field and an array entry in the aggregate — `ToProfile`,
`Describe`, `ValueEquals` and `CopyTo` are flat loops over `AllAxes`, so none of them need editing.
Existing axes are untouched.

`NetworkProfileDraft` is gone: it existed only because `NetworkProfile` was a flat six-argument
struct that five objects could not each build part of. Axes now construct Core impairments directly,
and Core owns the clamping.

A delay trace no longer supersedes delay and jitter — with separate impairments they compose, so
enabling all three means a recorded link plus an extra fixed hop, and the delays sum. That is a
legal configuration, not a contradiction to validate away.

`AllAxes` is an explicitly-built array, never reflection: IL2CPP strips what nothing references and
has no runtime codegen (invariant 5), so a reflective scan would work in the Editor and fail on
device — the same reasoning behind `Registry/Registries.cs`.

**The impairments themselves now live in Core** (`docs/adr/0013`). `Impairments/*.cs` here are
serializable *authoring shells*: Inspector fields plus one `ToCoreImpairment` that constructs the
real `Teleop.Core.Transport.Impairments` object. Every parameter clamp and every model decision is
Core's.

This layer survives rather than being deleted because Unity serialization needs parameterless
constructors and mutable public fields, while Core validates at construction and holds its
parameters readonly; `[Range]`/`[Min]`/`[Tooltip]` are `UnityEngine` and can never appear in Core;
and `[SerializeReference]` loses managed references on a type rename, so binding scene data to Core
type identities would make a later Core refactor a silent data-loss event here. `RobotArmProfileData`
exists for the same reasons.

Adding a new *kind* of disturbance is now a new Core file plus one shell here — no longer a
widening of a fixed six-field struct and an edit at every construction site.

### Presets, and how this relates to the sweeps

The impairment runs through the identical `EmulatedTransport` the sweeps use — same Gilbert-Elliott
loss chain, same uniform jitter draw, both directions, decorrelated seeds. `SweepCommand` and
`TeleopOperatorBridge` build structurally identical stacks. **The mechanism was never the
difference; only how the numbers were authored was.**

`NetworkProfilePresets` closes that in the safe direction. A preset dropdown loads a frozen
profile's exact values — read from `NetworkProfileCatalog`, never duplicated here — into the
per-axis fields. So "feel what `300ms-60j-2loss-bursty` is like" uses the numbers the sweep citing
that name used. Every axis then stays independently editable, because "start from that profile and
halve the jitter" is a real question a preset list can't enumerate.

The moment any axis differs from what the preset loaded, the HUD/log says `<name> (modified)`. The
settings genuinely are no longer that profile, and a summary that kept the bare name would make a
recording look comparable to a sweep it isn't comparable to.

**The frozen suite is never authored from here.** ADR 0004 records its numbers so a manifest and
the documentation can't drift; a preset reads that suite, it does not write it.

`synthetic-burst` is reachable too, via the `DelayTrace` axis — its bursts live in recorded samples and
are not expressible as base+jitter, which is exactly why it matters (the Buffering result rests on
it). `DelayTraceLoader` reads the file, since Core cannot do I/O. Two things about it:

- **No copy of the trace is committed under `unity/`.** `core/testdata/traces/` is the one source;
  `just install-traces` copies it to `Application.persistentDataPath` for the Editor, and `adb
  push` does the same on device. A tracked duplicate would drift silently.
- **The loader rescales samples between tick rates, and that is not cosmetic.** The header records
  the writing machine's rate (10,000,000 on Windows, 1,000,000,000 on Linux ARM64). Replaying a
  Windows-written trace on a Quest unrescaled would inflate every delay 100× while each individual
  number still looked plausible — the exact failure already recorded in `robot/README.md`'s
  ClockSync finding.

In trace mode the delay and jitter axes are switched off rather than left ticked and ignored:
`EmulatedTransport`'s trace constructor rejects a profile carrying either, on the grounds that
synthetic jitter on an already-recorded delay double-models the same variance.

Two things that are deliberate rather than incidental:

- **Only the loopback path is wired.** `TeleopOperatorBridge` owns both directions in-process, so
  impairing them is honest. `JetRoverOperatorBridge` is deliberately left out: `EmulatedTransport`
  impairs on the *receiving* side, so impairing what the real robot receives means wrapping
  `Teleop.RobotHost`'s transport on the Jetson. Wiring it there anyway would give a checkbox that
  moves the HUD's numbers while the physical arm behaves identically — worse than no checkbox.
- **Changing conditions mid-session invalidates the recording.** The impairment state is not
  recoverable from a `.tlog`, so every change is logged with its tick. This control is for feeling
  out the parameter space and for demos. Citable numbers come from `Teleop.Eval` sweeps against the
  frozen, *named* profile suite (`docs/adr/0004`–`0006`) — which is also why this composes an
  ad-hoc profile instead of making that suite mutable.

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
4. *(Optional — network impairment.)* Add a `NetworkImpairmentController` component to the
   `TeleopOperator` GameObject. It finds `TeleopOperatorBridge` on its own when they share a
   GameObject; otherwise set **Operator Bridge** explicitly. Leave every checkbox unchecked for a
   clean link — with nothing checked, no emulator is installed at all and the path is what it was
   before this component existed.
5. Press Play. Console should be clean; moving the right controller should move both robot
   GameObjects; the HUD should show live `M2P`/`uplink OWD`/`downlink OWD` numbers. On stop, a
   `phase4-session-<timestamp>.tlog` should exist under `Application.persistentDataPath` (in the
   Editor on Windows: `%userprofile%\AppData\LocalLow\<CompanyName>\<ProductName>\`).
6. *(If you added step 4.)* Still in Play mode, tick **Enable Delay** in the Inspector. The ghost
   robot should visibly lag the controller, the HUD's OWD numbers should rise by roughly the
   configured amount, and the Console should log one `NetworkImpairment @ tick ...` line per
   change. Unticking it should restore the previous feel immediately. If ticking a box changes the
   HUD but not the motion, or vice versa, something is wired wrong — both come off the same
   transport.

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
