# 29. The FR3 Duo's two-hand operator on the Galaxy XR

## Status

Proposed (2026-10-29). Part of the FR3 Duo work (ADRs 0026–0028); phase P7 of the plan. It adds a
second operator beside ADR 0017's JetRover clutch, which it does not change. The user chose the gesture:
**grab clutches, pinch grips**, one hand per arm.

## Context

ADR 0017's operator drives one five-joint arm. The right hand's pinch is the clutch, and the left
hand's pinch and twist are the gripper and the wrist roll. `ClutchMapping` keeps the wrist's position
and pitch and drops yaw and roll, because a five-joint arm cannot follow them.

The FR3 Duo has two seven-joint arms, each with its own Franka Hand (ADR 0026), and the user asked for
both arms, one per hand. Three things change:

- **Each hand needs a clutch and a gripper command at once.** The pinch can no longer be both.
- **A seven-joint arm can follow a whole pose.** That makes it a six-degree-of-freedom clutch, and the
  wrist roll is no longer a separate gesture.
- **The arms can hit each other.** ADR 0026's amendment gives a capsule model of both arms and the
  mount, and RobotHost already refuses colliding commands. The headset must not ask for them.

The robot is still simulated (`just fr3-sim-host`, ADR 0027), so nothing here moves hardware yet.

## Decisions

1. **One hand per arm, by side.** The left hand drives the profile's `left` arm and the right hand the
   `right` arm. The twin faces away from the operator, so each hand is on its own arm's side.

2. **Grab is the clutch, and the thumb-index gap is the gripper** (`Teleop.RobotArm.Operator.GrabGesture`).
   - **The grab:** the middle, ring and little fingers curled into the palm hold that hand's arm. It
     is the deadman: open the hand, or lose tracking of it, and the arm holds.
   - **The curl ratio:** each finger's tip-to-wrist distance over its knuckle-to-wrist distance,
     averaged over the three fingers. It does not depend on hand size: about 2 for a flat hand and
     about 1 for a fist.
   - **Hysteresis:** the grab engages below 1.25 and releases above 1.55, so a borderline fist does
     not chatter.
   - **The gripper:** while the hand is held, the thumb-index gap sets that arm's opening fraction,
     with the same 8 cm (open) and 1.5 cm (closed) as ADR 0017. Letting go holds the gripper; it never
     opens on its own.
   - **Every threshold is a first guess,** tuned on the headset and pushed with
     `just push-galaxy-fr3-config`.

3. **A six-degree-of-freedom relative clutch** (`PoseClutch`). Everything is in the robot's root
   frame. At engage it anchors the hand's pose and the tool's. After that:
   - the target's position is the tool's anchor plus the hand's displacement, times `PositionScale`
     (1.0: a seven-joint arm reaches about as far as a human arm);
   - its rotation is the hand's rotation since engage, applied in the root frame on the left of the
     tool's anchor. Turning the hand about the vertical turns the gripper about the vertical.

   Re-engaging anywhere never jumps the arm. Unlike `ClutchMapping`, the target is not projected onto
   a workspace: the solver reaches as close as it can, and the status says so.

4. **IK on the headset, checked as a whole robot before anything is sent** (`SerialChainTeleop`).
   ADR 0026's plan put solving on the headset and checking on both sides. Each frame, every held arm:
   - **Solves** with `SerialChainSolver` from its current command, so the solution stays on the same
     branch.
   - **Clamps the step.** No joint's command moves more than `MaxStepRadians` (0.1) per step. A
     larger solution is shortened along its direction, so every committed pose is close to the last
     checked one. The robot's shaper then bounds velocity, acceleration and jerk (ADR 0027).
   - **Checks for collision.** The candidate pose, this arm's new joints with the other arm's command,
     goes through `SerialChainCollision.Check`. A hit is not committed, and the arm holds where it was
     (`Blocked`). A target the solver cannot reach commits the closest pose inside the limits
     (`StoppedShort`).
   - **Waits for the robot's pose.** Nothing is commanded before the robot's measured pose has been
     taken (`TryTakePose`), so the first command is where the robot already is.

   The command is every arm joint and every gripper, in profile order. Arm joints are in radians and
   grippers are opening fractions (ADR 0027's wire). Speed is always 0: the robot's own limits shape
   the motion.

5. **A serial-chain joint link** (`SerialChainJointLink`). It speaks to the joint port the same way as
   ADR 0018's JetRover link: a keepalive every half second, ClockSync from the frames' echo, and a
   `JointPredictorBank` fed with frames (at their reading's time) and with commands (shifted by the
   clock's uncertainty). It drops the JetRover's arm power, training course and headset reports,
   which a serial-chain host does not take (ADR 0027 decision 5).
   - **The operator heartbeat** (ADR 0018 §9) goes once a second and at every motion change. It says
     `Headset` and the motion mode, and never makes a power request. RobotHost records sessions
     automatically only while a headset's heartbeat arrives, and those sessions are the FR3's training
     data, so without the heartbeat nothing the headset did would be recorded.
   - **The presets** are `franka-replay` (drawing at launch), `hold-last`, and `learned (sim)`
     (`fr3-sim-1`, layout v4, labelled simulated).
   - **The predictor config** is `SerialChainJointLink.PredictorConfig`: the shaper, the grippers
     moved at once, and the v4 layout. It is the same as `joint-predict-bench --robot fr3`'s.

6. **The twin is built from the URDF, in the editor** (`Fr3DuoModel`, `SerialChainArmRig`). There is
   one GameObject per URDF link, placed at its joint's origin through `CoordConversion`, the one
   conversion site. Each moving joint is posed as its origin followed by its motion, which is the
   profile's FK. `just unity-check-fr3-xr` checks this: at 25 seeded poses per arm, the twin's
   `*_hand_tcp` must be within 1 mm and 0.1° of `SerialChainKinematics.Tool`.
   - **Collision meshes, not visual ones.** The twin uses the URDF's collision STLs (a few hundred
     triangles each) and boxes. The visual COLLADA files are 59 MB of CAD, which is too dense for a
     mobile headset whose frame time is part of what this project measures. They are also exactly
     the shapes the collision model was fitted to.
   - **The twin shows the active predictor's estimate.** Before the clocks sync it shows the newest
     frame. The two markers show where each clutch asks its tool to be: blue while following, orange
     when out of reach, red when blocked.

7. **Motion is off at launch, and switching it on resyncs.**
   - `Fr3MotionButton` is a fingertip press, as in ADR 0023, so no grab, ray or pinch can press it.
     It toggles `MotionEnabled`.
   - Switching on first takes the robot's measured pose as the command and lets go of both clutches.
     While motion is off, the markers still follow your hands, so the arms would otherwise jump to
     where they were.
   - Android pausing the app, or the app losing focus (in a build), turns motion off.
   - The serial-chain host has no arm power to park or relax. Off only stops commanding, and the
     robot's shaper brings each arm to rest at its last target.
   - `AllowMotion: false` in the config locks motion off.

8. **A second scene, not a profile switch inside the JetRover's.** `Fr3DuoXR.unity` is built by
   `XrFr3SceneBuilder` (`just unity-scene-fr3-xr`), checked by `XrFr3SceneCheck`
   (`just unity-check-fr3-xr`), and built as `TeleopXR-FR3.apk` (`just build-galaxy-fr3`). The
   JetRover's scene, check and APK are untouched. The two scenes share the hands rig and the lab's
   materials, and nothing robot-specific. The FR3's generated files are copied into Bridge's
   Resources, because Unity loads only what is under the project. `BridgeResourcesTests` holds the
   copies to the files they came from, and `just import-fr3-profile` refreshes them.

## Consequences

- **All of the logic is headless and tested:** `Teleop.RobotArm.Tests/Operator/`, 32 tests,
  allocation-free on the frame path.
- **The whole headset path runs headless against the real host.** `just fr3-teleop-probe` uses the
  same link and teleop over a socket to `just fr3-sim-host`, with scripted hands. It checks that the
  measured tools reach their targets, and scores the twin: each reading against what the twin showed
  for the instant it was taken. On the loopback this gave p95 0.028–0.056° for `franka-replay`
  against 0.138–0.159° for `hold-last`, which is a diagnostic, not a result.
- **Bridge grows by three files:** the rig, the operator bridge and its config. Both MonoBehaviours
  only convert frames, own the socket and write Transforms. `just bridge-check` compiles them.
- **What the checks cannot see:**
  - whether the grab thresholds suit real hands on the Galaxy XR's tracking;
  - whether the clutch's rotation feels right;
  - frame time on the device with the twin's meshes.

  Those need the headset.
- **No camera, and no recording from the headset.** Sessions are recorded on the host side, as
  before (`--auto-record`, teleop-session v2). There is no `.tlog` of the Cartesian path, because the
  FR3 has none.
- **The real FR3 (P8) changes only the host's backend.** The headset's wire, gestures and checks stay
  as they are.
