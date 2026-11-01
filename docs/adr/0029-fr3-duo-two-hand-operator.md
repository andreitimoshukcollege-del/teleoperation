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

## Amendment (2026-10-30): after the first headset session

The user drove the simulated FR3 Duo on the Galaxy XR and asked for six changes. Each is recorded
here with what it replaced.

1. **Absolute hands: the robot anchored to the operator's body** (`BodyAnchoredMapping`). This
   replaces decision 3's relative clutch, which stays as `HandMapping: "relative"`.
   - **The body frame** is taken from the headset at RECENTRE (a blue button on the bench's
     front-right) and whenever motion is switched on: its position, and its heading about the
     vertical only.
   - **Position, per arm:** a neutral hand maps onto the arm's neutral tool, and every centimetre
     from there is a centimetre of tool. The neutral hand is 30 cm ahead of the head, 40 cm below it
     and 20 cm to that hand's side. It is per arm because the FR3 Duo's shoulders are 0.6 m apart,
     not a person's 0.4 m.
   - **Orientation, absolute:** a flat hand pointing forward points the gripper forward, with its
     fingers closing sideways.
   - **The neutral tool** is 55 cm ahead of and 30 cm above each shoulder. A neutral at the ready
     pose's tool reached only 6 of 54 nearby targets with the gripper pointing forward; this one
     reaches 53 of 54 (`BodyAnchoredMappingTests`).
   - **A grab slides the arm to the hand** over `ApproachSteps` (about a second) through checked
     poses, so the arm never lunges. An open hand moves nothing, and its marker stays on the arm (the
     user's call: a preview of where a grab would go read as the arm moving uncommanded).

2. **A double curl locks an arm on, and another unlocks it** (`GrabToggle`, `GrabMode: "toggle"`).
   This replaces holding the curl, which stays as `"hold"`, because tracking does not read a curled
   hand as curled at every angle.
   - **A click** is a curl held 40 ms to 0.6 s, and the second must start within 0.6 s.
   - **Untracked frames say nothing,** so a dropout cannot fake a double click.
   - **Losing the hand while locked holds the arm.** On return the arm slides back to the hand.

3. **Collision pairs left to the joint limits** (`fit-capsules`, reason `joint-limited`). The
   capsules could not keep three pairs per arm apart across the two joints between them:
   `link2`/`link4`, `link3`/`link5`, and `link5` against both fingers. Checking them blocked the poses
   the absolute mapping needs. The FR3's own limits are assumed to keep those links apart, as
   Franka's controller does. **This must be checked on the robot before P8.**

4. **The solid twin draws Franka's visual meshes** (about 510k triangles; the mount and head stay
   low-poly), replacing decision 6's collision meshes. `Fr3DuoModel` imports the COLLADA files and
   finds the importer's turn by laying every link over its collision shape: the worst link is 1.1 mm
   off. The commanded and measured ghosts stay low-poly and draw no mount.

5. **The same lab and the same controls as the JetRover's scene.** These replace the bare floor and
   table.
   - **One base class for both robots.** `OperatorBridgeBase` is now under both
     `JetRoverOperatorBridge`, with no change in behaviour, and `SerialChainOperatorBridge`. The
     console, network sliders, added-delay slider, power and pause, twin switches, headset guard,
     status strip and status lamp are wired to it.
   - **The FR3 scene is built by the JetRover's own code.** It reuses `LabRoom`, `LabDisplay`,
     `LabLighting` and `XrControlSetup`'s builders.
     - **Bench and probes:** a deeper bench, with the FR3 at its back so its arms clear the console
       panel, and light probes over the FR3's workspace.
     - **Mesh library:** its own generated-mesh library. The JetRover's would delete its robot's
       meshes, because a build removes what it did not write.
   - **Three twins and the status lines:** the twin has predicted, commanded and measured arms, as
     the JetRover's does. The display's caption line carries the arms' state, and the switches carry
     the link's.
   - **The FR3 link grew what the console needs:**
     - live per-predictor scoring (`LiveJointScorer`), with the gripper in percent of its opening;
     - the added delay;
     - reports of the disturbance and the delay to the recording (`HeadsetConditions`), so the
       trainer refuses sessions that were not on the real network, as for the JetRover.
   - **Off and paused both stop commanding.** The serial-chain host has no torque to keep or drop,
     so the only difference is the label.
   - **Not carried over:** the arm-speed slider (the FR3's shaper sets its own speed) and the
     training course (built around the JetRover's single arm).

6. **The checks.**
   - **The FR3 scene check** also requires power and pause and the headset guard wired to this
     bridge, the switches, the console and its sliders wired, and no arm-speed slider.
   - **The JetRover scene check still passes,** which shows its saved references survived the move to
     the base class.
   - **The headless parts are tested:** `BodyAnchoredMappingTests`, `GrabToggleTests`, and the link's
     delay, scoring and condition reports in `SerialChainJointLinkTests`.
   - **The end-to-end probe** (`just fr3-teleop-probe`) passes on the new link.

## Amendment 2 (2026-10-31): the shortest-path step

The user found the IK "glitchy" on the headset. It said it could not reach some poses, and it made
huge rotations to reach others. They suggested moving the gripper along the shortest path. That is
now the default, `ChainStepMode.CartesianPath`. The first way stays as `FullSolve`, kept so the two
can be compared.

**The first way.** Each frame solved all the way to the hand's target. The joints then moved
straight toward that solution, cut to 0.1 rad per frame. A far target can solve on another branch,
and the straight joint-space line to it swings the arm.

1. **The step.** Each frame the gripper moves toward its target along the straight line and the
   smaller rotation. It moves at most `MaxLinearSpeed` (1 m/s) and `MaxAngularSpeed` (3 rad/s), and
   only that small step is solved, from where the arm is. The speed caps also handle the approach
   to a newly grabbed hand, which `ApproachSteps` did before.

2. **The spare joint, used every frame before the step is solved.** The FR3 has seven joints for a
   six-number pose, which leaves one free motion (the elbow's swing). It is steered:
   - **Off the joint limits.** A joint inside `LimitZone` (0.6 rad) of a limit is pushed back at up
     to `LimitReliefSpeed` (2 rad/s). Without this, the forearm joint (5) drifted onto its limit
     as the hand wandered. That joint was at its limit in 1,335 of 1,839 out-of-reach frames, and
     the other joints swung to make up for it.
   - **Away from what the arm is closing on.** The arm follows the gradient of its clearance from
     `AvoidDistance` (4 cm) before the margin.

   The step is then solved from the moved pose, so the gripper stays on its path. A step that would
   collide is cut to its largest clear part (a half, then a quarter), or not taken.

3. **Joint speed.** No joint moves faster than its velocity limit in the profile (`JointSpeedShare`
   1). The whole arm's step is cut together, so the gripper keeps its path and only slows. A step
   longer than 50 ms (`LongestStep`, a stall in the host) counts as 50 ms. The bridge and the probe
   step by their real frame length, because Direct Preview runs below 90 Hz.

4. **OUT OF REACH is reported only when it is true.** The solver's tolerance is 0.1 mm and 0.06°,
   and missing it was enough to report `StoppedShort`. 30% of those frames were within 5 mm and 3°
   of the target. Now an arm reports out of reach only when two things hold:
   - it is stuck: the solve did not reach half of its small step;
   - it is more than `ReachTolerance` (1 cm) or `ReachAngleTolerance` (5°) from its target.

**Measured** by `SerialChainTeleopTrackingTests`, on the same synthetic hands for both. Each arm's
hand wanders about ±15 cm with its palm ±35° in yaw, ±30° in pitch and ±70° in roll. Each run is
30 s at 90 Hz, over five seeds:

| | blocked | out of reach | off target¹ | false alarm² | error p95 | angle p95 | swing p99 / max | frames > 0.05 rad |
|---|---|---|---|---|---|---|---|---|
| FullSolve (before) | 5.3% | 7.4% | 8.2% | 5.0% | 36.6 mm | 16.7° | 0.066 / 0.100 rad | 415 |
| CartesianPath (now) | 3.2% | 2.7% | 7.2% | 0.2% | 6.7 mm | 8.4° | 0.038 / 0.058 rad | 92 |

¹ More than 1 cm or 5° from the target, whatever the arm reports. ² The arm reports a refusal
within both.

**Reading the table:**
- **Swings.** Every remaining frame above 0.05 rad is joint 7 or joint 5 turning at its own
  velocity limit.
- **Refusals left.** The blocks are mostly the right upper arm (`link3`) touching the head capsule.
  The out-of-reach frames are mostly wrist joints 6 and 5 at a limit, which are real.

**Tried and rejected:**
- **The gripper as the same grasp turned half a turn.** The arm aimed for whichever of the target
  and its twin was nearer, switching after a 0.35 rad margin.
  - Mid-motion switches never happened in the benchmark.
  - The choice made at lock-on hurt. Aiming for the twin left the right arm blocked 1.6–13.9% of the
    time, against 0–3.4% aiming for the target.
  - A version that chose the twin by the wrist's remaining twist was worse still: 11.9% off target,
    against 7.7% for the plain target.
  - The gripper now follows the hand's roll exactly, and a wrist turned to its limit stops there.
- **`AvoidDistance` of 6 or 8 cm.** The arm wandered (20% and 87% off target), so 4 cm stays.

**Not done or not verified:**
- **Label flicker.** The headset's label flickers on blocked spells of one to three frames (15 of
  30 in the benchmark). This is not addressed.
- **The headset.** How the change feels there is not yet tested.
- **The real FR3.** Not yet tested (P8).

## Amendment 3 (2026-11-01): the arm slides along what it touches

The user drove the arms in Isaac (ADR 0031) with amendment 2's IK. It "feels a lot better", but in
certain positions it felt stuck: once the arm got there, it was hard to move it on.

**The cause, from the session's recording.** The right arm stalled seven times. Every stall from
47 s on was at the collision margin (10–11 mm). In five of them the right upper arm (`link3`)
touched the head capsule, and in one `link2` touched `link5`.

From those poses, a headless replay asked the IK for a 6 cm move in each of the six directions.
Two or three of the six went nowhere (0–14 mm in 2 s), each one a direction that would have
brought the arm a hair closer to the obstacle. A step that collided was cut to a half or a quarter,
or else not taken. So an arm at the margin was refused every step with any component toward the
obstacle, and it stuck there.

**The fix.** A step that would collide is replaced by the solver's `ConstrainedStep`.
- **What it computes:** the damped least-squares step toward the same pose that keeps the arm
  exactly as far from the obstacle, to first order, and backs out to 2 mm above the margin when
  inside it.
- **How it spends the constraint:** in the step's own metric (`JᵀJ + λ²I`), so the elbow's spare
  motion takes it first, and then the smallest error in the gripper's pose.
- **When a slide is taken:** only if it is clear, moves the gripper toward the hand (sideways along
  the obstacle too, never away), and does not turn it further.
- **Otherwise** the old cut to a half or a quarter applies.
- **The label:** an arm sliding within the reach tolerances of its target reports following, not
  blocked.

From the seven stuck poses, the moves that went nowhere now reach the full 6 cm in a few frames. The
exceptions are into the head, and at the two poses with joints 1 and 2 on their limits, which are
real limits. `SerialChainTeleopTests` replays the 59.8 s pose.

**Measured** by `SerialChainTeleopTrackingTests` (the same hands as amendment 2):

| | blocked | out of reach | off target | false alarm | error p95 | angle p95 | frames > 0.05 rad | changes/s |
|---|---|---|---|---|---|---|---|---|
| amendment 2 | 3.2% | 2.7% | 7.2% | 0.2% | 6.7 mm | 8.4° | 92 | 0.32 |
| sliding | 2.9% | 2.6% | 6.1% | 0.4% | 4.0 mm | 6.5° | 120 | 0.30 |

More frames go over 0.05 rad, and all of them are wrist joints at their own velocity limit.

**Also found:**
- **Two collision models.** The headset (this branch) leaves the joint-limited pairs to the joint
  limits (amendment 1, item 3). The Isaac session's RobotHost ran from main, which still checks them,
  so it refused 19 of the headset's commands. They come into line when this branch reaches main.
- **The refusal log.** RobotHost's periodic refusal line never printed: its first-time check
  overflowed. That fix is on the Isaac branch.
