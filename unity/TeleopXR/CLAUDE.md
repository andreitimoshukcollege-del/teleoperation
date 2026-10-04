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
create-only, like TeleopVR's lab. From now on it is edited by hand in the editor.
`rebuild=true` replaces it and discards those edits. Contents:

- **XRI's `XR Origin Hands (XR Rig)`.**
  - Its modality manager has **no controllers**, so the scene is hands only.
  - Tracking origin is **Floor**.
  - Its **One Euro hand-smoothing filter (`HandsOneEuroFilterPostProcessor`) is disabled.** It
    filters every hand joint, including the pinch pose that becomes the robot's target.
- **The arm rig, `DragTarget` start pose, `JetRoverOperatorBridge` and `CameraFeedBridge`.** Values
  are identical to TeleopVR's `JetRoverControl`, so the IK sees the same arm in both projects.
- **`HandTrackingSession`.** It asks for the `HAND_TRACKING` runtime permission and logs the session's
  input modality once.
- **`GrabHighlight` on the target:** cyan when idle, yellow when a pinch now would grab it, green when
  grabbed. With hands there is no trigger to feel.

**The target's grab settings are safety settings.** `just unity-check-xr` fails, and `just
build-galaxy` refuses to build, if any of them changes:

| Setting | Value | Because |
|---|---|---|
| Far attach mode | Far | a far pinch must not pull the target, and with it the robot, to the hand |
| Dynamic attach | on | grabbing must not snap the target's centre to the fingers |
| Movement type | Instantaneous | no physics-step delay between pinch and command |
| Smooth position / rotation | off | the operator's intent is sent raw (ADR 0016 §5) |
| Throw on detach | off | releasing leaves the target, and the robot, where it was let go |
| Rigidbody | kinematic, no gravity | the target moves only when grabbed |
| Hand smoothing filter | inactive | as above, at the joint level |
| Modality manager controllers | none | hands only |

What the check cannot see:

- **Releasing on tracking loss** is XRI's behaviour when a hand's interactor goes away. Verify it on
  the headset before relying on it.
- **Far grabs follow XRI's aim ray.** The rig includes `XRTransformStabilizer` components for ray
  aiming, and whether they smooth a hand's far grab is not verified. Near grabs (pinching on the
  sphere) follow the pinch pose directly, so prefer them for anything citable.
- **Hand jitter and dropouts reach the robot** through the same IK and 48 Hz command stream as
  controller motion does. Filtering them would be a Core research axis with its own ADR, never a
  smoothing checkbox here.
- The base rig also carries locomotion providers (move, turn, teleport, climb, grab-move). All of
  them are driven by controller input actions, so with no controllers they do nothing.

## Testing on the Galaxy XR

**Direct Preview** (Play mode streamed to the headset, the Link equivalent):

1. Windows 11 and a GPU with Vulkan video encoding (this PC: RTX 4060 Laptop).
2. Install Google's **Android XR Engine Hub** and its Android XR Streaming runtime.
3. Turn on developer mode on the headset and connect it with a good USB-C cable.
4. In the editor, open **Project Settings → XR Plug-in Management → Project Validation** and apply
   *Android XR Streaming runtime selected*. This sets the machine's active OpenXR runtime, which is
   why it isn't in the setup code.
5. Press Play.

**APK:**

1. `just build-galaxy` produces `Builds/TeleopXR.apk`. It runs the scene check first and refuses
   to build if the check fails.
2. `just install-galaxy` installs it over USB.
3. `just push-galaxy-config` points the app at the robot's current LAN address without a
   rebuild. It finds the robot itself (`just robot-ip`); pass an address to override. It pushes a full `jetrover_connection.json` override to the app's files directory;
   restart the app afterwards.

**The robot must reply to the headset.** Make sure the Jetson runs the reply-to-sender RobotHost:
`just deploy-robothost`. The camera sender already answers whoever sends
the keepalive.

**M2P on Galaxy XR is relative only** until its `DisplayOffset` has been measured with the photodiode
rig and pushed as a per-device `display_calibration.json` (ADR 0016 §6).

## Not here yet

- The lab visuals: the night-shift room, detailed JetRover twin and big display, planned for URP in
  this project.
- Controllers, passthrough and eye tracking (ADR 0016 *Out of scope*).
- Robot discovery by name (ADR 0015).
