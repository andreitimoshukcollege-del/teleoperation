# 16. Galaxy XR runs from a second Unity project on Unity 6, sharing Core and Bridge

## Status

Accepted (2026-10-02). The user chose a separate project over upgrading `unity/TeleopVR`,
pinch-grab as the hand interaction, and two separate builds (Quest and Galaxy XR).

## Context

The operator scene is to run on the **Samsung Galaxy XR**, an Android XR headset, with **hand
tracking instead of controllers**. Quest keeps working.

What Android XR requires of a Unity project, from Google's and Unity's setup pages (2026-10):

- **Unity 6.** Unity's `com.unity.xr.androidxr-openxr` package needs Unity 6000.0.23f1 or newer.
  Google's project-setup page asks for 6000.0.58f2 or newer, and Unity 2022.3 is not supported.
- **Direct Preview**, the Android XR equivalent of Quest Link (Play mode streamed to the headset
  over USB-C), needs **Unity 6000.3.5f2 or newer** and **Android XR Extensions for Unity ≥ 1.2.0**.
  It also needs Windows 11 and a GPU with Vulkan video encoding. This PC has Windows 11 Home and an
  RTX 4060 Laptop GPU.
- Google recommends **URP and Vulkan**. Unity's foveated rendering for Android XR is URP-only.
- Required player settings: **GameActivity** as the application entry point (a Unity 2023.1+
  feature), a resizable activity, and minimum API level 24 for the OpenXR loader.
- OpenXR ≥ 1.15 and XR Hands ≥ 1.6.

That collides with **ADR 0001, "Stay on Unity 2022.3 LTS"**. ADR 0001 says reopening the editor
version costs comparability with every result recorded on the Quest baseline, and that it must be
done with a tagged repo and a re-run of the frozen benchmark suite on both editors.

What the existing code makes easy:

- **`Bridge/` is headset-agnostic.** No file in it touches OVR, Meta or XR device APIs.
  `XrDisplayTimeProvider` mentions OpenXR only in a comment.
- **The operator's intent is a scene object, not a controller.** `JetRoverOperatorBridge` reads
  the pose of a `DragTarget` that the operator grabs with an `XRGrabInteractable`. Hardware motion
  is gated by the `ConfirmHardwareMotion` config flag, not by any button. Grabbing that same target
  with a pinch instead of a controller grip is the whole interaction change.
- **The robot side already answers whoever asks.** `Teleop.CameraHost` streams to the source of
  the latest keepalive (ADR 0014). `Teleop.RobotHost` replies to the source of the latest command
  (#57), once that build is deployed.

## Decision

### 1. A second Unity project, `unity/TeleopXR`, on Unity 6

`unity/TeleopXR` needs Unity 6000.3.6f1 or newer, which covers Direct Preview (6000.3.5f2+) and
Google's Extensions v1.4.0 (6000.3.6f1+). It was created on **6000.6.4f1** (Unity 6.6, a Supported
release), which the user had installed; 6.3 LTS would also have qualified.
The exact version is recorded in its `ProjectSettings/ProjectVersion.txt`.

`unity/TeleopVR` stays on 2022.3.46f1, so ADR 0001 still governs it and the Quest baseline stays
comparable. The two projects produce **two separate builds**: the Quest APK, or Link, from
TeleopVR, and the Galaxy XR APK, or Direct Preview, from TeleopXR.

### 2. Core and Bridge are shared, never copied

- **Core and RobotArm:** both projects reference `core/Teleop.Core` and `core/Teleop.RobotArm`
  through relative `file:` package paths, as TeleopVR does today.
- **Root CLAUDE.md invariant 6 is unchanged.** Unity 6 compiles C# 9 against .NET Standard 2.1,
  the same level as 2022.3, so the C# 9 constraint now comes from two editors rather than one.
- **Bridge becomes a local package, `unity/Teleop.Bridge/`.** It moves out of
  `unity/TeleopVR/Assets/Teleop/Runtime/Bridge/` with its `.meta` files, so script and asmdef GUIDs
  are preserved and TeleopVR's scenes keep their references. Both projects reference it by
  `file:`, and its `Resources/` defaults (`jetrover_connection.json`, `display_calibration.json`)
  move with it.
- **Copying Bridge into the new project is ruled out** for the same reason Core is never copied:
  two copies drift, and nothing would catch it.
- **Bridge code must compile under both editors.**
  - No API that is obsolete in Unity 6 (e.g. `FindObjectOfType`) and none that is newer than 2022.3.
  - `just bridge-check` keeps compiling it against hand-written stubs.
  - Changes that touch the Unity API surface are also compiled against each editor's real
    assemblies before merging.

### 3. URP in TeleopXR, built-in stays in TeleopVR, and shared code assumes neither

TeleopXR uses URP with Vulkan only, HDR off and no post-processing, as Google recommends for
Android XR. Bridge code must work with both pipelines:

- **`JetRoverArmRig` sets both `_Color` and `_BaseColor`** in its reach-warning property block.
  URP's Lit and Unlit shaders read `_BaseColor` and ignore `_Color`; the built-in pipeline does the
  opposite. Setting both is harmless on either.
- **`CameraFeedBridge` already works.** It assigns `material.mainTexture`, which maps to whatever
  property the shader marks `[MainTexture]`: `_MainTex` in built-in `Unlit/Texture`, `_BaseMap` in
  URP Unlit.

### 4. The XR stack in TeleopXR: OpenXR and Android XR, no Meta SDK

The project uses:

- OpenXR ≥ 1.15;
- Unity OpenXR: Android XR (`com.unity.xr.androidxr-openxr`);
- **Android XR Extensions for Unity** (`https://github.com/android/android-xr-unity-package.git`,
  Apache-2.0, ≥ 1.2.0, pinned to a release tag);
- XR Hands ≥ 1.6;
- XR Interaction Toolkit 3.x.

The exact versions Package Manager resolves are pinned in `Packages/manifest.json`.

Project configuration is code: `Assets/Teleop/Editor/XrProjectSetup.cs` applies the player, XR
and URP settings idempotently, so a second run changes nothing. A setting that only exists as a
click in an Inspector cannot be reviewed, reproduced or rebuilt.

### 5. Input is hands only: pinch to grab the same target

- **Rig:** the XR rig carries hand interactors only. A pinch is select, near (on the sphere) and far
  (aim ray). Poke is used for UI, and a hand visualizer shows the tracked hands.
- **Target:** the `DragTarget` keeps its role, start pose and `XRGrabInteractable`.
  - It gets a kinematic Rigidbody and **no smoothing**: the operator's raw intent is what is sent,
    as with controllers, and smoothing would add latency.
  - When a hand stops being tracked, its interactor deselects. The target, and so the robot, stays
    where it was released. This is verified on the device before relying on it.
- **Hand tracking is a different input modality.** Its jitter and dropouts reach the robot exactly
  as controller motion does, through the same IK and the same 48 Hz command stream.
  - The modality is logged once at session start.
  - A result recorded with hands must say so, and must not be compared with a controller session
    without saying so.
  - Filtering hand jitter would be a research axis in Core with its own ADR, not a Unity-side
    smoothing setting.

### 6. Per-headset calibration

Galaxy XR has its own optics and display pipeline. Its `DisplayOffset`, the constant in
`display_calibration.json`, has to be measured with the photodiode rig before any absolute M2P
figure from it is cited. Until then, Galaxy M2P numbers are relative only. The per-device override
in `Application.persistentDataPath` already supports a value per headset.

## Consequences

- **Two Unity editors on the Windows machine.** Each recipe that runs Unity names its editor
  version and refuses to run while that project's editor is open.
- **A Bridge change has two compile targets.** A green `bridge-check` is necessary but not
  sufficient: it uses stubs. The Unity CLAUDE.md files say how to compile Bridge against both
  editors' real assemblies.
- **Galaxy XR results are a new series.** They differ in editor, render pipeline, headset and input
  modality all at once, so they say nothing about the Quest baseline and are labelled as such.
- **TeleopXR starts deliberately plain:** a floor, a bench, the arm, the target and the camera
  panel. The lab visual upgrade (night-shift room, detailed twin, big display) is planned to land
  there next, built once for URP.

## Alternatives considered

- **Upgrade TeleopVR itself to Unity 6.** One project instead of two, but it reopens ADR 0001 for
  the Quest baseline: a tag, a frozen-suite re-run on both editors, and a render-pipeline move, all
  before Galaxy XR runs at all. Rejected for now. It remains open if keeping two projects in step
  turns out to cost more than that.
- **Run the 2022.3 project on Android XR as generic OpenXR.** Unsupported: the Android XR package
  needs Unity 6, and GameActivity does not exist in 2022.3.
- **Copy Bridge into TeleopXR.** Rejected; see §2.
- **A clutch ("pinch anywhere, the target follows your hand's motion").** Lower fatigue and no jump
  on engage. Not chosen as the default; it can be added later as a second interaction behind a
  config switch.

## Out of scope

Passthrough, eye tracking, controllers on Galaxy XR, hand tracking on Quest (possible on 2022.3
with XR Hands), robot discovery over mDNS (ADR 0015) and the lab visual upgrade.
