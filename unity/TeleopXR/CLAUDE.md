# Unity — TeleopXR (Samsung Galaxy XR, Unity 6000.6.4f1, URP, hand tracking)

The Galaxy XR operator project (docs/adr/0016). It shares Core, RobotArm and Bridge with
`unity/TeleopVR` (Quest) by `file:` path and adds only what Android XR and hand tracking need. Rules
shared by every Unity project are in `unity/CLAUDE.md`, and Bridge's are in
`unity/Teleop.Bridge/CLAUDE.md`.

## Pinned stack

The versions are the ones Unity 6000.6.4f1 recommends, pinned in `Packages/manifest.json`:

| Package | Version | Why |
|---|---|---|
| URP | 17.6.0 | Google's recommendation for Android XR; HDR off, no post-processing |
| OpenXR | 1.18.0 | |
| Unity OpenXR: Android XR (`com.unity.xr.androidxr-openxr`) | 1.4.0 | Android XR Support; adds the `HAND_TRACKING` permission to the manifest |
| Android XR Extensions for Unity (`com.google.xr.extensions`) | git tag v1.4.0 | Direct Preview (the Android XR Streaming feature). Apache-2.0 |
| XR Hands | 1.9.0 | the Hand Tracking Subsystem |
| XR Interaction Toolkit | 3.6.1 | the hands rig and pinch interactions |
| AR Foundation / XR Management / Input System | 6.6.2 / 4.7.0 / 1.20.0 | dependencies |

There is no Meta SDK and no Quest feature group. Quest builds come from TeleopVR.

## Project configuration is code

`Assets/Teleop/Editor/XrProjectSetup.cs` (**Teleop → XR → Configure Project**, or
`just unity-setup-xr`) sets everything: the samples, URP, the Android Player settings, and OpenXR
for Android and for Windows (Direct Preview). Its doc comment gives the reason for each value.

**To change a setting, change it there and rerun.** A value clicked into an Inspector is undone
by the next run, and nobody can review it.

## Fresh clone

1. Install Unity **6000.6.4f1** with Android Build Support (OpenJDK, Android SDK & NDK).
2. Run `just unity-setup-xr` **twice**. The first run imports XRI's Starter Assets and Hands
   Interaction Demo samples and XR Hands' HandVisualizer, which compile only after it ends. The
   second run confirms the rest.
3. `Assets/Samples/` is **gitignored**: the samples are regenerated from the pinned packages, and
   their GUIDs ship inside the packages, so the committed scene's references resolve. Never edit
   anything under `Assets/Samples`; the next import overwrites it.
4. `just unity-check-xr`. The scene, `Assets/Scenes/JetRoverLabXR.unity`, is committed.

## The scene (`JetRoverLabXR`) and its safety settings

`XrLabSceneBuilder` (**Teleop → XR → Build Lab Scene**, or `just unity-scene-xr`) made it, and it is
create-only, like TeleopVR's lab. It dresses the lab as it builds (see *The lab* below). From now on
the scene is edited by hand in the editor. `rebuild=true` replaces it and discards those edits.
Contents:

- **XRI's `XR Origin Hands (XR Rig)`.**
  - Its modality manager has **no controllers**, so the scene is hands only.
  - Tracking origin is **Floor**.
  - Its **One Euro hand-smoothing filter (`HandsOneEuroFilterPostProcessor`) is disabled.** It
    filters every hand joint, including the pinch pose that becomes the robot's target.
- **The arm rig, `JetRoverOperatorBridge` and `CameraFeedBridge`.** Values are identical to
  TeleopVR's `JetRoverControl`, so the IK sees the same arm in both projects.
- **`HandTrackingSession`.** It asks for the `HAND_TRACKING` runtime permission and logs the session's
  input modality once.
- **Clutch control and the gripper (docs/adr/0017), from `XrControlSetup`** (**Teleop → XR → Set Up
  Clutch Control**; a lab rebuild reruns it):
  - **`HandClutchDriver`** on `JetRoverOperator`.
    - The **right hand's pinch is the clutch**: pinch anywhere and the arm's wrist follows your
      hand's motion from there, scaled by `ClutchScale` (0.5). Release and the arm holds, so you
      can re-pinch somewhere comfortable.
    - The **left hand's pinch is the gripper** (motor 10): thumb and index 8 cm apart or more
      is open, 1.5 cm or less is closed, linear in between.
    - The **left hand's twist is the wrist roll** (motor 5): palm down is 0, clamped to ±90°.
      `GripperRollSign` flips the direction.
    - It reads hand joints from XR Hands directly. The mapping is Core's (`Teleop.RobotArm`'s
      `ClutchMapping` and `GripperMapping`), reached through Bridge's `JetRoverClutch`.
    - Feedback: the wrist LED ring turns cyan while clutched, and red still means out of reach.
      The display's caption line reads `CLUTCH ON · gripper N%`.
  - **`DragTarget`** is no longer grabbable. It is a 2 cm marker showing the commanded wrist
    position, and the bridge still reads it.
  - **`TwinGripper`** on the wrist slides the twin's fingers by the gripper command and rolls the
    gripper by the wrist roll. It shows what is sent; the real gripper has no feedback.
  - **`MotionToggleButton`, the robot's start/stop switch, on the bench's front-left corner.** It
    is red when off (`ROBOT OFF`) and green when on, and **off at every launch**. Press it down
    with a fingertip. Only a fingertip counts; no ray or far pinch can press it. Off means nothing
    is sent and the robot holds still, while the virtual arm still follows your hands. It shows
    LOCKED when `ConfirmHardwareMotion` is off in the config.
  - Tune these with `just push-galaxy-config`, no rebuild: `ClutchScale`, `ClutchMinHeightMeters`
    (the wrist never goes lower, default −0.05 m below the base pivot), `GripperPinchOpenMeters` and
    `GripperPinchClosedMeters`. Edit them in `unity/Teleop.Bridge/Runtime/Resources/jetrover_connection.json`
    first; the recipe pushes that file.

- **The twin's three arms (docs/adr/0018), built by the same `XrControlSetup`:**
  - **Predicted, solid** (`PredictedArm`): where the robot most likely is right now. It is the
    newest joint-state frame from the robot, rolled forward by the joint predictor through the
    commands sent since. `JointPredictorName` is `servo-replay` (default) or `hold-last` (no
    prediction). `JointModelSpeedRadPerSec` is the servo speed the model assumes, a guess until
    it is fitted from recordings.
  - **Commanded, ghost**: the clutch's arm, where you told it to go.
  - **Measured, faint amber** (`MeasuredArm`): the newest frame exactly as received, about one
    network trip old.
  - **Switches.** Three fingertip switches next to the motion button (PREDICTED, COMMANDED,
    MEASURED) show or hide each arm, and are lit when it is shown. The line above them says
    whether the robot's state is arriving: `NO ROBOT STATE` means RobotHost is too old for the
    stream; otherwise it shows `live · rtt N ms`.
  - **Mechanics.** `TwinArmsDriver` (on `JetRoverOperator`) drives the predicted and measured
    arms in `onBeforeRender` from Bridge's `JetRoverJointStateLink`. That link sends the
    keepalive on the joint socket, receives the frames, runs its own `ClockSync`, and feeds the
    Core predictor. The arms are hidden until the first frame arrives, and the predicted one
    until the clocks are synced.
  - **See-through only on device.** `ArmAppearance` makes an arm see-through at runtime only, so
    editor previews show all three solid. The predicted and measured arms start inactive, so
    previews show only the commanded arm.
  - **Finding the right arm.** Editor tools find the commanded arm through the bridge's `armRig`
    (`XrControlSetup.CommandedRig`), never "any rig": the scene holds three.

**These are safety settings.** `just unity-check-xr` fails, and `just build-galaxy` refuses to
build, if any of them changes:

| Setting | Value | Because |
|---|---|---|
| `DragTarget` interactable | none | a pinch could grab it and move the robot outside the clutch's scale and workspace limits |
| `HandClutchDriver` | present, wired to the bridge, rig and XR Origin, enabled | nothing else drives the arm |
| Clutch engage / release distance | engage below release (1.5 / 3 cm) | otherwise the clutch chatters on and off |
| `ClutchScale` | in (0, 1] | above 1 the arm moves further than the hand |
| Gripper pinch range | open distance > closed distance | otherwise the mapping is inverted |
| `JetRoverOperatorBridge.motionEnabledAtStart` | off | the robot must not move at launch, before anyone presses the button |
| `MotionToggleButton` | present, wired, enabled | the only way to start, and stop, the robot from the headset |
| `TwinArmsDriver` | present, wired, not driving the commanded arm | the robot's state must never overwrite the display of the command |
| Hand smoothing filter | inactive | it filters the hand joints the clutch reads; intent is sent raw (ADR 0016 §5) |
| Modality manager controllers | none | hands only |
| Active Input Handling (Player Settings) | Both | XRI's hand interactions read the Input System |
| Android XR Support, Hand Tracking (OpenXR, Android) | on | without them the APK launches as a 2D panel and crashes |

The clutch is safe by construction, not just by settings:
- **Releasing, or losing tracking of the clutch hand,** disengages, and the arm holds.
- **The target is projected onto the reachable workspace every frame,** above
  `ClutchMinHeightMeters`. Pushing past the edge leaves the arm at the edge, and moving back moves
  it immediately.
- **Losing tracking of the gripper hand holds the gripper.** It never opens on its own.

What the check cannot see:

- **The gripper's pulse range is uncalibrated** on the robot (`GenericArmPlantConfig.GripperOpenPulse`
  and `GripperClosedPulse`). Measure it with `just move-arm 0.15 0 0.08 gripper=…` while you
  watch the robot, before the headset drives the gripper.
- **Whether the palm's pitch feels right for the wrist** is a headset question. `followPitch` on
  `HandClutchDriver` turns it off.
- **Hand jitter and dropouts reach the robot**, scaled by `ClutchScale`, through the same IK and
  48 Hz command stream. Filtering them would be a Core research axis with its own ADR, never a
  smoothing checkbox here.
- The base rig also carries locomotion providers (move, turn, teleport, climb, grab-move). All of
  them are driven by controller input actions, so with no controllers they do nothing.

## Testing on the Galaxy XR

**Once per headset: developer mode and USB debugging.** The menu names below are from ManageXR's
Galaxy XR guide and may differ slightly between headset software versions.

1. **Settings → About device**: tap **Build number** seven times and enter the PIN.
2. **Settings → System → Developer options**: turn on **USB debugging**.
3. Connect it with a data-capable USB-C cable. In the headset, allow USB debugging for this
   computer, and tick *always allow*.
4. `just _adb devices -l` should list the headset as `device`. If it shows `unauthorized`, the
   prompt in step 3 is still waiting.

**Direct Preview** (Play mode streamed to the headset, the Link equivalent). It needs Windows 11
and a GPU with Vulkan video encoding (this PC: RTX 4060 Laptop).

1. **Once per PC:** install Google's **Android XR Engine Hub**. Close the Unity editor and open the
   Engine Hub. Pick the headset in the device drop-down, click **Install Stream Client**, then
   click **Set** next to **Active OpenXR Runtime**. That runtime is a setting of the machine, not
   the project, which is why the setup code doesn't make it.
2. **Once per clone:** `just unity-setup-xr` (or **Teleop → XR → Configure Project**) sets the
   editor up the way Android XR Streaming requires: Vulkan only on Windows, multi-pass, 24-bit
   depth, legacy foveation. The editor changes graphics API only when it restarts, so reopen it;
   its title bar then ends in `<Vulkan>`. **Project Settings → XR Plug-in Management → Project
   Validation**, Standalone tab, should list no `[OpenXR]` or `[Android XR Streaming]` errors.
3. `just unity-robot-host` points Play mode at the robot's current address.
4. Connect the headset over USB-C and press Play. The stream starts by itself.

**APK** (the headset alone, over Wi-Fi):

1. `just build-galaxy` produces `Builds/TeleopXR.apk`. It runs the scene check first and refuses
   to build if the check fails.
2. `just install-galaxy` installs it over USB.
3. `just push-galaxy-config` points the app at the robot's current LAN address without a
   rebuild. Use `just push-galaxy-config auto false` the first time you try the controls: the
   virtual arm moves and the real one doesn't. It finds the robot itself (`just robot-ip`); pass an address to override. It pushes a
   full `jetrover_connection.json` override to the app's files directory; restart the app
   afterwards.
4. The headset must be on the robot's network. Accept the hand-tracking permission on the first
   launch.

**The robot must reply to the headset.** This applies to the APK; in Direct Preview the PC is the
sender. Make sure the Jetson runs the reply-to-sender RobotHost: `just deploy-robothost`. The
camera sender already answers whoever sends the keepalive.

**M2P on Galaxy XR is relative only** until its `DisplayOffset` has been measured with the photodiode
rig and pushed as a per-device `display_calibration.json` (ADR 0016 §6).

## The lab: room, twin and display are generated

The night-shift lab around the arm is built by code in `Assets/Teleop/Editor/Lab/`:

- the room: sign wall, window skyline, desks, racks, shelving, a taped workcell with a status ring,
  a mobile-robot test area, motion-capture cameras;
- the JetRover twin: chassis, mecanum wheels, lidar, 7-inch screen, and arm visuals on the rig's
  pivots;
- the 1.2 × 0.9 m camera display on a stand, with a live status strip.

It goes into the scene under one `LabDressing` root. The meshes, textures, materials and lighting
settings it generates go to `Assets/Teleop/Lab/Generated/`.

- **Review it headless:** `just unity-lab-xr`, or the **Teleop → XR** menu: *Rebuild Lab Dressing*,
  *Bake Lab Lighting*, *Capture Lab Previews*. It rebuilds the dressing and bakes the lighting (the
  bake takes under a minute on the RTX 4060; the whole run takes a few minutes). It then renders nine
  fixed views and a contact sheet to `%TEMP%\teleop-lab\<time>\`, with a `report.txt` of triangles,
  renderers, materials and lightmaps. Look there before putting on a headset. `bake=false` skips
  the bake. Close the editor first.
- **Regenerated, never hand-edited.**
  - A rebuild deletes and recreates `LabDressing` and the twin's `TwinVisual` children. It also
    deletes the builder's original placeholder roots, `Room`, `CameraPanel` and `ConnectionStatus`.
  - It rewrites the generated assets in place, so their GUIDs stay stable.
  - It resets the directional light, the scene's lighting settings and the wiring below.
  - Hand edits anywhere else in the scene survive. To change the look, change the code and rerun.
- **It rewires three things, and `just unity-check-xr` guards both safety-relevant ones:**
  - `CameraFeedBridge.panel` becomes the display's screen, with `armRig` left empty. With an arm
    rig set, the bridge would move the screen out of the display at start.
  - `JetRoverArmRig.reachWarningRenderer` becomes the LED ring on the twin's wrist. The old segment
    cubes are hidden, not deleted, and the check fails if the warning renderer isn't drawn.
  - `CameraStatusBar` drives the display's strip, and `LinkStatusLamp` the workcell ring. These
    are display-only and reference Bridge only.
- **Lighting is baked.**
  - Subtractive mode: area lights under and over the pendant fixtures, cove lines, light probes
    where the arm moves, and one box-projected reflection probe.
  - The single Mixed directional light gives the moving arm a real-time shadow on the bench, which
    tells the operator how high the gripper is.
  - The cove lines and fixture diffusers have HDR colours. The screen clamps them, but the
    reflection probe keeps the full value, so the glossy floor reflects them.
  - Bake output (`Assets/Scenes/JetRoverLabXR/`) and the mesh library are LFS-tracked.
- **The twin is our own geometry**, from Hiwonder's published dimensions and the repo's measured
  arm. No vendor files (the repo is public).
- **TextMesh Pro's essential resources are committed** in `Assets/TextMesh Pro/`; the lab's labels
  need them. Batch mode cannot import them, because the import is asynchronous and `-quit` ends the
  run first. `just unity-setup-xr` only checks that they are there.

## Not here yet

- Controllers, passthrough and eye tracking (ADR 0016 *Out of scope*).
- Robot discovery by name (ADR 0015).
