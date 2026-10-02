# Unity — TeleopVR (Meta Quest, Unity 2022.3.46f1, built-in pipeline)

The Quest project, and the one the recorded baseline was measured on (ADR 0001 pins its editor).
Rules shared by every Unity project are in `unity/CLAUDE.md`. Bridge (`com.teleop.bridge`, the shared
package at `unity/Teleop.Bridge/`) has its own `CLAUDE.md`. The Galaxy XR project is
`unity/TeleopXR/`.

## Quest specifics

- OpenXR with the Meta feature group; Meta XR SDK (`com.meta.xr.sdk.core`).
- **Sentis is unavailable on 2022.3** (it needs 2023.2+), and Barracuda is deprecated. So
  `IInferenceBackend` has no Unity implementation yet — that is deliberate and blocks nothing
  until Phase 7. Do not add `using Unity.Sentis`.
  ADR 0001's ban on `using Unity.Sentis` holds repo-wide, Unity 6 project included, until a Phase 7
  ADR decides the inference backend.

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

## Camera feed panel (scene wiring in this project)

Shows the JetRover's camera beside the arm proxy. Hand-wired in the Editor, like everything above.
The component's rules (material asset, never under the camera, firewall, decode cost) are in
`unity/Teleop.Bridge/CLAUDE.md`.

1. Create a **Quad** named `CameraPanel` and give it a **material asset** using `Unlit/Texture`.
2. Add a `CameraFeedBridge` component, for example on the GameObject that holds
   `JetRoverOperatorBridge`. Set **Panel** to the quad's MeshRenderer. Set **Arm Rig** to the
   `JetRoverArmRig` to have the panel placed at **Offset From Arm Base** at start; leave it empty to
   keep the quad where you put it.
3. Press Play. Within a second or two the panel shows the feed.

## Lab scene (`JetRoverLab.unity`, built by **Teleop → Build Lab Scene**)

A lab room with the JetRover's virtual twin on a bench and the camera feed on a monitor to its
right. `Assets/Teleop/Editor/LabSceneBuilder.cs` made the first version through the Editor API.

**Since then, the scene itself is the source of truth.** Change the lab by editing `JetRoverLab.unity`
in the Unity editor and saving it. The builder does not read those edits back, so it refuses to run
while the scene exists, and it never changes a material that is already in `Assets/Teleop/Lab/`.
Rerun it only to start a fresh lab, after renaming or deleting the scene. Two things still hold:

- Edit the scene in the editor, never as text. The scene is a few hundred objects and Unity YAML
  merges badly, so avoid two people (or two branches) editing it at once.
- The rules the builder followed are the scene's rules too. Keep colliders off set dressing, keep
  the monitor world-locked (never under `Main Camera`), and keep **Arm Rig** empty on
  `CameraFeedBridge`, or it moves the screen out of the monitor at start.

What the builder did, in order:

1. Copies `JetRoverControl.unity`, so the XR rig, arm rig, drag target, `JetRoverOperatorBridge`
   and HUD arrive with their working wiring. **None of them is moved or edited**; the room is
   placed around the arm's base where that scene puts it.
2. Adds set dressing under a static `Lab` root: an 8 x 8 x 3 m room, a bench with a taped workcell,
   a chassis under the arm (visual only, never driven, not under any pivot), shelving, a desk, a
   whiteboard. Primitives only; colliders removed so nothing catches the XR ray meant for the drag
   target; the room shell casts no shadows (the ceiling would otherwise shadow the whole room).
3. Builds `CameraMonitor` on the bench, turned to face the operator, with a `CameraScreen` quad
   using the `CameraScreen` material asset (`Unlit/Texture`, see step 1 of the camera panel above).
   It shows colour bars until the first frame decodes, so "no feed yet" differs from "black feed".
4. Adds `CameraFeedBridge` to the `JetRoverOperatorBridge` object with **Panel** set to the screen
   and **Arm Rig** deliberately empty: with an arm rig it would move the screen out of its monitor.
5. Sets Trilight ambient, no skybox, and retunes the one directional light. No baked lighting.

Materials and the colour-bar texture are in `Assets/Teleop/Lab/`. Commit them with their `.meta`
files whenever they change. To run it on the Quest, add the scene in **Build
Settings** (above `JetRoverControl` to make it the one that loads).

The builder is compiled by Unity's `Teleop.Editor` assembly (Editor-only, references `Teleop.Bridge`
and TextMeshPro, never Core), so it is not in any player build. `just bridge-check` does not cover
it: it needs the real `UnityEditor` API, which the stubs do not provide.

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
