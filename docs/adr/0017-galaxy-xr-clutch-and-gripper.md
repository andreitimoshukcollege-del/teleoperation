# 17. Galaxy XR input: clutch control and an off-hand gripper

## Status

Accepted (2026-10-08). Supersedes the *target* half of ADR 0016 §5 ("pinch to grab the same
target"). The rest of §5 still holds: hands only, no smoothing, and the modality is logged.

The user's choices:
- clutch control, over grabbing the virtual gripper, point-and-send, or puppeting the joints;
- motion scaled at 0.5×, adjustable without a rebuild;
- a continuous gripper driven by the other hand's pinch;
- Galaxy XR only. Quest keeps its controllers and its target sphere.

## Context

On Galaxy XR the operator pinch-grabbed a sphere, and the arm's IK followed it. On the headset that
felt unintuitive, for three reasons:
- **The sphere drifted away from the arm.** It could be released beyond the arm's 26 cm reach, and
  it stayed there while the arm stopped at its limit.
- **The scale was 1:1 on a tiny arm.** An ordinary hand movement was a large arm movement.
- **You had to reach for the sphere.**

The gripper was never driven from the operator side. Every hop already carries it:
- `CommandFrame.Gripper`;
- `JointCommandCodec`'s `GripperMain` entry (motor 5, a 0–1 fraction);
- `GenericArmPlant`, which maps the fraction to a pulse.

Only `JetRoverOperatorBridge` hardcoded `0`.

## Decision

### 1. Clutch: relative, scaled, and always reachable

- **Engage and release.** A pinch of the clutch hand (the right hand by default) engages. It
  engages under 1.5 cm thumb–index distance and releases over 3 cm. The gap is hysteresis, so a
  borderline pinch does not chatter. Releasing, or losing tracking of that hand, disengages, and
  the arm holds where it is.
- **Position**, while engaged, in the arm-base frame:
  `target = target_at_engage + scale × (hand − hand_at_engage)`.
- **Pitch** follows the hand's pitch change since engage, 1:1. Roll and yaw are still dropped, as
  `ArmKinematics` has always done.
- **The target never leaves the workspace.** Every update projects it onto the reachable set:
  - the same radial reach clamp `TryInverse` applies;
  - plus a minimum height, `ClutchMinHeightMeters` (default −0.05 m relative to the base).

  The projected point is what is stored. The next engage therefore starts from where the arm really
  is, and pushing past the edge does nothing until the hand comes back.
- **The `DragTarget` Transform stays as the bridge's input.** The clutch writes to it, and the
  bridge's IK, rig, network and recording paths are unchanged. It is a small marker now, not a grab
  handle.
- **The mapping lives in Core, not Unity.** `Teleop.RobotArm` gains `ClutchMapping`,
  `ArmKinematics.ProjectToWorkspace` and `GripperMapping`, all unit-tested. The Unity component
  only reads hand joints and writes Transforms (`unity/CLAUDE.md`, thin adapters).

### 2. Gripper: the other hand's pinch, continuous

- **The mapping.** The off hand's thumb–index distance maps linearly to a closing fraction:
  `GripperPinchOpenMeters` (0.08) or wider is 0, open, and `GripperPinchClosedMeters` (0.015) or
  tighter is 1, closed. The fraction goes to `JetRoverOperatorBridge.GripperFraction` and from
  there into both command streams.
- **Losing tracking of the off hand holds the last value.** It never opens on its own, so a held
  object is not dropped because a hand left the cameras' view.
- **The gripper stays open-loop**, as ADR 0011 left it: there is no feedback.

### 3. Configuration

`ClutchScale` (0.5), `ClutchMinHeightMeters`, `GripperPinchOpenMeters` and
`GripperPinchClosedMeters` live in `jetrover_connection.json`. They can be changed per device with
`just push-galaxy-config`, with no rebuild.

### 4. Amendment (2026-10-12): the wrist roll, the motor ids, and a start/stop button

The user tried the gripper on the robot, and pinching **rotated the gripper instead of closing it**.

**Cause.** The JetRover profile called motor 5 the gripper. Hiwonder documents motor 5 as the
**wrist roll** and motor 10 as the **gripper**.

**The profile now maps them that way everywhere:**
- `core/RobotProfiles/jetrover.json`;
- `RobotArmProfile.JetRoverMeasuredDefault`;
- Bridge's `RobotArmProfileData`.

That is motor 10 `GripperMain`, motor 5 `GripperRotate`, and `gripperCanRotate: true`.
`GripperRotate` is now an **angle in radians**, converted and stepped like an arm joint. It was a
0–1 fraction before, which no caller ever used.

**The left hand's twist is the wrist roll.**
- **The mapping.** The hand's roll about its pointing direction, in the arm-base frame, palm down = 0.
  It is multiplied by `GripperRollSign` and clamped to ±`GripperRollLimitDegrees` (90°).
- **Where it goes.** The roll is part of the target's orientation, so both command paths extract the
  same value with `ArmKinematics.ExtractRollRadians`:
  - the joint stream, from the bridge;
  - the Cartesian path, from `GenericArmPlant.Command`.
- **The twin's gripper rolls with it.** The palm, the gripper servo and the fingers turn about the
  link axis.
- **Losing the hand holds the roll.**
- **Wrist pitch still follows the right hand.**

**A start/stop button on the bench gates the robot.**
- **What it gates.** `MotionToggleButton` is pressed down with an index fingertip. It toggles
  `JetRoverOperatorBridge.MotionEnabled`, which sits on top of `ConfirmHardwareMotion`.
- **Off at every launch.** The scene sets `motionEnabledAtStart` to off, and the scene check fails if
  it is on. Nothing reaches the robot until someone presses the button, and pressing it again stops
  sending; the robot holds its last command. The virtual arm follows the hands either way.
- **Only a fingertip presses it.** It reads fingertips from XR Hands, not XRI, so no ray or far
  pinch can press it. It must be released and re-approached between presses.
- **When the config forbids motion, the button shows LOCKED and does nothing.**

**Consequences of the amendment:**
- **RobotHost must be redeployed before the new app is allowed to move the robot**
  (`just deploy-robothost`). The deployed RobotHost still believes motor 5 is the gripper, so it
  would read the new roll angle as a gripper fraction.
- **Motor 10 has never been commanded by this codebase.** The Jetson relay is id-keyed (ADR 0011),
  but that it accepts id 10 is unverified.
- **The roll's direction against the real servo is unverified.** `GripperRollSign` flips it, with
  no rebuild.
- **`IsFullySensed` still ignores both gripper motors.**

## Consequences

- **A Galaxy XR command is now a scaled, clutched intent.** The `.tlog` records the target that
  was sent, which is no longer the hand's pose.
  - The scale is logged once at session start, with the modality.
  - Results recorded under different scales, or against controller sessions, must say so.
- **The pinch-grab safety settings no longer apply**, because nothing is grabbed. `XrSceneCheck`
  now checks something else:
  - the target has no interactable;
  - the clutch driver is wired;
  - the scale is in (0, 1].
- **The gripper's pulse range is not yet calibrated.** `GenericArmPlantConfig.GripperOpenPulse`
  and `GripperClosedPulse` are marked uncalibrated. Measure them with `just move-arm … gripper=`
  while a person watches the robot, before the headset drives the gripper.

## Alternatives considered

- **Grab the virtual gripper itself.** It cannot drift from the arm, but the hand covers the robot
  and the display, and it cannot scale motion.
- **Point and send** (goal-based, with a ghost preview). It tolerates latency best, but it is not
  continuous. It remains a candidate for a latency-mode experiment later.
- **Puppet the joints.** It never asks for an unreachable pose, but it is slow for reaching. It
  could become a fine-adjust mode later.
- **Mirror the wrist continuously.** With no clutch, every stray movement moves the robot.
