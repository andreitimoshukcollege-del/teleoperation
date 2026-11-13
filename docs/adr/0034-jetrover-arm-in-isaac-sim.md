# ADR 0034: The JetRover's arm in Isaac Sim, behind its own RobotHost

## Status

Accepted, 2026-11-08 (direction). Each phase's details are settled by the change that builds it, as amendments here.

## Context

The FR3 Duo runs in Isaac Sim behind RobotHost (ADR 0031), with objects that can be knocked over, four rendered cameras,
random backdrops, recorded video (ADR 0032) and the headset's RECORD button (ADR 0033). The user: "we want to adapt this
setup for the jetrover, is there a way we can simulate that arm with isaac sim or any other sim method that would account
for physics?" Of the two ways offered they chose the first: Isaac stands in for the robot's servo driver, so the JetRover's
own RobotHost and headset scene run unchanged.

What exists for the JetRover:

- **The JetRover's RobotHost** (`Program.cs`, `GenericArmPlant`) talks to the robot through `IRelayClient`. That is a Unix
  datagram socket to the ROS relay node (`robot/ros/src/teleop_relay`), relay protocol v5 (`RelayProtocol.cs`,
  `relay_protocol.py`):
  - commands: per motor, pulse and pulses per second;
  - feedback: per motor, valid and pulse;
  - torque on or off.

  Everything above that seam is the real robot's code path: the staleness gate, the step clamp, pulse conversion
  (`GenericArmPlantConfig`), arm power and park (ADR 0023), recording.
- **The headset's JetRover scene** (`JetRoverLabXR`): the clutch, the collision-aware IK (ADR 0020), power and pause, the
  twin's three arms, the console. It knows nothing of a world, Isaac cameras or RECORD.
- **No URDF in the repo.**
  - The arm is described by `core/RobotProfiles/jetrover.json`: base yaw, a 2-link planar arm of 0.13 m links, one wrist
    pitch, gripper roll, gripper.
  - The headset twin uses the robot's CAD (`JetRoverCad`, links measured 13.04 and 12.91 cm).
  - Hiwonder's ROS packages on the Jetson probably carry a URDF; not yet checked.
- **The trainer's JetRover simulator** (`trainer/armpred/sim/robot.py`, ADR 0021) models the pipeline's timing and the
  servos as a fitted lag, speed limit and droop. It has no rigid bodies, no contacts and no rendering.

## Decision

1. **Isaac replaces the ROS driver, behind the relay protocol.** A new bridge, `robot/isaac/teleop_isaac/jetrover_bridge.py`:
   - It binds the socket RobotHost's `--relay-socket` names and speaks relay protocol v5 exactly.
   - The JetRover's RobotHost runs unchanged on the GPU machine with `--relay-socket` pointing at it. Sim sessions then
     pass through the same code as real ones, and `RelayProtocol.cs`, `relay_protocol.py` and the bridge's copy change
     together.
   - A golden file of the protocol, written by the C# tests as the arm-server wire's is, pins the Python copy.
2. **The arm's model is generated from the JetRover's own descriptions, not drawn by hand:**
   - the profile's kinematics: link lengths and joint roles, so the simulated arm is the arm the IK reasons about;
   - `GenericArmPlantConfig`'s pulse conversion and joint ranges;
   - masses and collision shapes from the CAD.

   A `Teleop.Eval` verb writes `robot/jetrover/urdf/jetrover.urdf`, and a test requires the committed bytes. If
   Hiwonder's URDF turns out to be usable, it is checked against ours, masses first, and the differences are noted here.
3. **The servos are position servos, not the FR3's torque-controlled joints:**
   - Each joint is a PhysX position drive whose velocity limit is the command's `pulsesPerSecond`, converted.
   - Its effort limit is the servo's stall torque. The first guesses come from Hiwonder's specifications, then are tuned
     by decision 6.
   - **Gravity acts on the arm.** The real servos sag and slow under load; the FR3 cancels its own weight.
   - **Torque off makes the drives limp,** so the arm falls under gravity, as the real one does when it relaxes.
4. **It reports what the real robot can sense, no better:**
   - motors 1 to 5, about every 50 ms each, in whole pulses;
   - motor 10, the gripper, never valid;
   - and later, optionally, the trainer's fitted read misses and board silences.

   Otherwise the headset and the predictors would see a better robot than the real one, and sim data would teach the
   wrong lesson.
5. **The base is fixed** (the JetRover's chassis on the bench). Driving the base is out of scope.
6. **Sim against real is measured before sim data counts.**
   - Real recorded sessions' relay targets (`RO` records) are replayed into the bridge, headless, and its feedback is
     compared with the relay feedback they recorded (`RI`). The metrics are `sim_tracking_error_deg` and
     `sim_settle_error_deg` (`docs/metrics.md` §10).
   - The drive gains and effort limits are fitted to minimise it, reusing the trainer's sysid sessions.
   - Until that number exists and is written here, sim sessions are labelled `source: isaac` and kept out of real-data
     splits.
7. **Then the FR3 setup's features, ported to the JetRover's host and scene, in this order:**
   - **the world:** a scene file for the JetRover's smaller workspace, movable objects, NEW LAYOUT and EDIT OBJECTS, the
     world twin. RobotHost's `WorldHost` needs a channel to Isaac beside the relay: the arm-server wire's world kinds
     (7–11) on a second socket;
   - **cameras and recording:** the renderer posed from the relay's feedback, a wrist camera on the gripper, two
     third-person cameras, backdrops, the `isaac` writer;
   - **the RECORD button and scenarios** (`ScenarioRecorder`), and **pointing** (`Fr3HandPointer` generalised).

## Phases (one PR each; `unity/` changes want human review)

| Phase | What | Needs |
|---|---|---|
| J0 | This ADR; the URDF generator (decision 2) and its test; `docs/metrics.md`'s sim-vs-real metrics (§10) | — |
| J1 | `jetrover_bridge.py`: relay v5 server, drives, gravity, torque, feedback as the robot's; the relay golden; `just isaac-jetrover-check` on the 5090 (a move reaches its target at the commanded speed, torque off falls, the gripper reads invalid) | the GPU machine |
| J2 | `just isaac-jetrover-host`: RobotHost (JetRover) plus the bridge on the GPU machine; the headset's JetRover scene pointed at it (`just push-galaxy-config <GPU machine>`); `just joint-probe` / `move-arm` against it | the headset |
| J3 | Sim against real: `just jetrover-sim-vs-real` replays recorded sessions, reports the metric, fits the drives | recorded JetRover sessions |
| J4 | The world, ported (decision 7) | — |
| J5 | Cameras, backdrops and recording, ported | — |
| J6 | RECORD, scenarios and pointing, ported | — |

## Consequences

- **Data from two robots, one pipeline.** The FR3 and the JetRover share the renderer, recording, backdrops and RECORD.
  Their hosts differ where the robots differ.
- **The real robot's code is exercised without the robot.** RobotHost, park and relax, and the headset's clutch can all
  be tested against physics when the robot is off the network, which it often is.
- **Relay protocol v5 now has three implementations.** The golden file is what keeps them together.
- **Physics is not truth.** Bus servos have backlash, deadband and a temperature-dependent speed that a PhysX drive does
  not. Decision 6 says how far off the sim is; nothing claims more.

## Alternatives considered

- **The JetRover as a serial-chain robot in the FR3's pipeline** (option 2): every FR3 feature at once, but the
  operator would become the FR3's (absolute hands, two arms), the 5-joint arm needs an IK that can give up orientation,
  and sim sessions would not pass through the JetRover's RobotHost.
- **MuJoCo:** lighter, deterministic and CPU-only, better for headless sweeps and decision 6's fits. It has no rendered
  cameras or backdrops. A candidate for a later headless twin of the same URDF.
- **Gazebo with Hiwonder's ROS stack:** simulates the driver too, but brings the whole ROS stack into the loop. Decision 1
  keeps the real stack's behaviour where it matters, above the relay seam, and models the driver's reporting in the
  bridge.
- **The trainer's simulator alone:** it has the pipeline's timing, but no contacts, objects or images.

## Not decided here

- Driving the base.
- A real JetRover camera's pose in the sim (ADR 0014's camera is on the robot; its mount is measured in J5).
- Whether the trainer's read misses and board silences are switched on by default (J1 makes them optional).

## Amendment 1 (2026-11-08): J0, the model

- **`JetRoverSimModel`** (`Teleop.RobotHost/Sim`) writes `robot/jetrover/urdf/jetrover.urdf` and its sidecar
  `jetrover.sim.json`. `JetRoverSimModelTests` requires the committed bytes and rewrites them with
  `TELEOP_WRITE_GOLDEN=1`.
- **The chain is the IK's.** At 50 seeded poses, walking the URDF's joints puts the wrist within 0.01 mm of
  `ArmKinematics.Forward`.
- **Collision shapes:** every box of `JetRoverCollisionModel` on its link, and the chassis's 18 boxes on the fixed base.
  The two jaw bodies are fingers on prismatic joints, 11 mm of travel each.
- **Mass** is given per link from what it carries (a bus servo is about 60 g, plus its bracket): 0.73 kg for the arm.
  Volume times a density gave 2.2 kg, because the collision boxes overlap and are padded. The inertia spreads each link's
  mass over its boxes by volume.
- **Effort and speed:** 3.4 N·m on the yaw, shoulder, elbow and wrist pitch, 1.5 N·m on the roll; 9.2 rad/s, the plant's
  fastest pulse rate (2200 pulses/s). Both are first guesses for J3 to fit.
- **The sidecar** carries `GenericArmPlant`'s pulse conversion: zero pulse 500, 238.73 pulses/rad, the shoulder's
  0.122 rad mounting offset, the gripper's open and closed pulses. The gripper (motor 10) is marked as never reporting.
- **Hiwonder's URDF** has not been compared yet: the robot was off the network.

## Amendment 2 (2026-11-08): J1 written, not yet run on the 5090

- **`teleop_isaac/jetrover_bridge.py`** binds the relay socket. It takes each drain of datagrams through the ROS relay's own
  `sort_datagrams`: every torque switch in order, and the newest command only. Relay protocol v5 therefore keeps two
  implementations, `RelayProtocol.cs` and `relay_protocol.py`, not three: decision 1's planned third copy, and its golden,
  are not needed.
- **`teleop_isaac/servo.py`** holds the rest, tested without Isaac (`tests/test_servo.py`):
  - `TimedSetpoint`: a move ramps the servo's setpoint linearly over `|delta| / pulses_per_second` (the driver's
    `durationMs`), and a new move starts from where the ramp is.
  - `FeedbackEmulator`: motors 1–5 read in turn every 10 ms in whole pulses, every reading sent every 50 ms, valid while
    under 0.2 s old, the gripper never.
  - `SimModel`: `GenericArmPlant`'s pulse conversion from the sidecar.
- **Drives.** Stiffness 60 N·m/rad and damping 1.5, within each servo's stall torque. With torque off, stiffness is 0 and
  damping 0.02, so the arm falls. When torque comes back on, the servos hold where the arm lies.
- **The arm starts at the collision model's home pose,** torque on (`--torque-off-at-start` starts it relaxed).
- **`just isaac-jetrover-check`** (`robot/isaac/jetrover_check.py`) is J1's gate: feedback cadence and validity, home, a move
  at its commanded pace, falling with torque off and holding with it on, and late steps.
- **`just isaac-jetrover-check` on the 5090,** 2026-11-08 (644433c): all 9 checks pass, on the first run.

  | Check | Measured |
  |---|---|
  | Feedback | 20.0 Hz; motors 1–5 valid, the gripper never |
  | At home, under gravity | shoulder 3.2 pulses (0.8°) low, elbow 2, wrist 1 |
  | Elbow +100 pulses at 400 pulses/s | past halfway at 0.19 s; within 3 pulses at 0.34 s (the ramp alone takes 0.25 s); settles 2 pulses short |
  | Torque off | the shoulder fell 372 pulses (89°) in 2 s |
  | Torque back on | held, 0 pulses of drift over 1 s |
  | Late steps | 0 in 8,479 |

  These say the bridge behaves like a servo arm. How close it is to the JetRover is J3's measurement.

## Amendment 3 (2026-11-08): J2, the JetRover's RobotHost against Isaac

- **`just isaac-jetrover-host`** runs the bridge and then RobotHost with the Jetson's flags (`deploy-robothost`'s), the relay
  socket aside:
  - joint port 6002 and discovery 6005;
  - the JetRover profile;
  - the Jetson's rest pose, so ROBOT OFF parks and relaxes;
  - sessions in `~/isaacsim/jetrover-recordings` marked `source: isaac` by the new `--auto-record-source`.

  `isaac-stop`, `isaac-status` and `isaac-host` now see the JetRover bridge too. The FR3 and JetRover hosts share their
  ports, so one runs at a time.
- **Feedback before any command.** The bridge first sent feedback only to whoever had commanded it, so an idle RobotHost
  saw no servo at all. The real relay sends to RobotHost's fixed socket every 50 ms, so now the bridge does too
  (`--client-socket`, RobotHost's `--local-relay-socket`).
- **Measured on the 5090,** 2026-11-08, from the PC:
  - `just joint-probe 3 <GPU machine>`: 49.5 Hz joint frames; servo feedback at 20.3 Hz with motors 1–5 sensed and the
    gripper not, as on the robot. At home the shoulder reads 0.76° below its target under gravity.
  - `just move-arm 0.15 0 0.08 0 <GPU machine>`: converged in 4 round trips, the wrist at (0.150, 0, 0.078). That is
    2.2 mm low, the sag of a servo arm holding itself up.
- **Not done yet:** the headset's JetRover scene against it. Installing that APK replaces the FR3's (one app id), and the
  scene's camera has no Isaac feed until J5.

## Amendment 4 (2026-11-08): the feedback timing is the real robot's since 2026-10-25

The user, in the headset: "the measured values seem low hz".

- **What was wrong.** Amendments 2 and 3 timed the feedback by an outdated description: one motor read every 10 ms in turn,
  and feedback sent every 50 ms, about 20 Hz a motor. The robot has been faster since 2026-10-25:
  - the driver (`robot_controller_node.py`, `pollRate` 0.02) reads motors 1–5 back to back every 20 ms, 2.9 ms a read
    (the trainer's fit);
  - the relay (`relay_node.py`, `_FEEDBACK_SEND_PERIOD_SECONDS` 0.02) sends every 20 ms.
- **Now.** `servo.FeedbackEmulator` runs that cycle, and a cycle's readings are published when it ends. The relay's sends
  come every 20 ms, so each motor gets a fresh reading about every 20 ms.
  - The sidecar carries `feedbackPeriodSeconds` 0.02, `driverCycleSeconds` 0.02 and `readSeconds` 0.0029.
  - `just isaac-jetrover-check` expects feedback at 45–55 Hz.

## Amendment 5 (2026-11-08): J4–J6, the FR3 setup ported

The user: "make the scene like the fr3 duo scene". Done on one branch rather than three PRs, since the headset half needs all
three to be useful.

- **J4, the world.**
  - `core/Scenes/jetrover-bench.json` (version 2) is the JetRover's bench, from the CAD: the bench top 0.1962 m below the
    yaw pivot, a table ahead of it flush with the top, four small movable objects (cube, post, can, wall) placed 0.12–0.36 m
    ahead and ±0.18 m aside, inside the arm's reach. Bench and table are `builtin`: the headset's lab has them already.
  - The bridge builds the scene (`--scene`) and serves the world on a second socket (`--world-socket`, kinds 7–11).
    RobotHost (JetRover) gains `--scene`, `--scene-seed`, `--world-socket`; its `ArmServerClient` connects world-only
    (no motors, no state). `WorldHost` is the FR3's.
  - Measured on the 5090 with `world-probe`: 49.2 Hz world frames, seed 7's poses matched `Draw(7)` to 0.0002 mm, NEW
    LAYOUT gave seed 8's.
- **J5, cameras and recording.** The renderer takes `--sim robot/jetrover/urdf/jetrover.sim.json` and poses the JetRover
  from the relay's feedback. Cameras: `third-left`/`third-right` (feeds 6008/6009, 0.78 m ahead, ±0.48 m aside, 0.30 m up)
  and `wrist` on `gripper_link` (6003), each recording `rgb`, `depth`, `mask`. Masks add `arm` (7) and `hand` (8).
  JetRover sessions have no `QS`, so each recorded frame carries its joint values in a `VQ` record. RobotHost waits for the
  writer (`--auto-record-writer isaac:60`). Measured: feeds at 29.5–29.9 fps; a manual session had 274 `VF`, 274 `VQ`, 455
  `RI`, an `END` trailer and every `FS` line. The robot renders as its collision boxes: the URDF has no CAD visuals yet.
- **J6, the headset.**
  - **Shared code.** The joint port's world and record side channels moved out of `SerialChainJointLink` into Core's
    `JointPortSideChannels`; the JetRover's link uses the same class. In Bridge, `OperatorSideControls` holds NEW LAYOUT,
    queued moves, pointer drags and RECORD over it, and `OperatorBridgeBase.SideControls` hands it to the scene, so
    `SerialChainWorldTwin`, `Fr3HandPointer` and `Fr3MotionButton`'s RECORD and NEW LAYOUT roles take either robot's bridge.
    The FR3's EDIT OBJECTS, power and RECENTRE stay FR3-only.
  - **The JetRover scene** gets them from `XrJetRoverWorldSetup` (`just unity-world-jetrover-xr`), which replaces only its
    own roots in the hand-edited scene: the world twin at the rig's `BaseAnchor`; RECORD, DISCARD LAST, < PREV, NEXT > and
    NEW LAYOUT on the bench's right wing behind the training course (the left wing behind power is the console's); the hand
    pointer; the third-person displays (front-left on the operator's left, front-right above the wrist display), all three
    floating and movable.
  - **The clutch and the pointer share the right hand's pinch.** With the robot off, a pinch the pointer owns (its ray on a
    button, an object or a display) never engages the clutch (`HandClutchDriver.handPointer`, `Fr3HandPointer.Owns`), and
    the pointer, not the clutch, moves the displays. With the robot on the pointer is off, so nothing changes for driving.
  - `just unity-check-xr` checks the twin's ids against the file, the buttons, the pointer and its three displays, the
    clutch's pointer reference and the feeds' ports.
- **Not measured yet:** any of J6 in the headset, against `just isaac-jetrover-host`. J3 (sim against real) is still open,
  so every session from it stays `source: isaac`.

## Amendment 6 (2026-11-08): resizing objects from the headset, and the JetRover's own look in Isaac

The user, in the headset: "the objects are showing but they are pretty small, can you add a way to resize, also the model that
is showing in the isaac sim stream is very different to the actual robot model". Asked how, they chose resizing each object
in the headset over a scale for the whole scene.

- **Resizing.** An object's size is a factor on the scene file's size, the same in every direction, in steps: 1.25 to a
  whole power from −3 to 5, so 0.512 to 3.05 (`WorldScene.ResizeStep`, `ScaleOfStep`; to 8 since amendment 7). Steps because of Isaac, below.
  - **Headset.** With the robot off, the last object a pointer (or the FR3's EDIT OBJECTS) grabbed is selected; SIZE − and
    SIZE + (beside NEW LAYOUT in both scenes) move it one step. A new world edit, `WorldEditOp.Resize` (5), carries the factor
    in the edit's x. The twin draws each object at its size.
  - **RobotHost** sends it to the backend and settles the edit by its answer. The backend's sizes are the truth: a new
    joint-port datagram, `0x8B` world sizes, carries every object's factor to the headset with the status (once a second and
    on change); `WZ` records them in the session.
  - **The arm-server wire** gains kind 12 RESIZE (answered by a PLACED of the same request) and kind 13 SIZES (at start, on
    change, once a second). The golden file has both.
  - **Isaac** checks a resize by the placement rules at the new size and the object's current place (an arm touching it, off
    the table, too close to another object, an arm in the way), then stands it upright there at its new size.
  - **PhysX cannot change a collider while it simulates.** Measured on the 5090, 2026-11-08: a cylinder's new radius and
    height were ignored (the can, 1.5×, settled at its old height, 16 mm low), and a box's new scale invalidated every
    simulation view, so the next articulation call failed and the bridge exited. So the bridge builds one body per step for
    each object the headset may resize (9 a object, mass × factor³), parks those not in use on an invisible shelf 5 m below
    and 20 m aside, and swaps two by teleport, as a placement moves one. The renderer, which simulates nothing, changes its
    copy's shape. Every placement rule uses each
    object's current size. The renderer applies SIZES to its copy and records them as `SZ`.
  - **Not done:** layouts drawn by RobotHost (`SceneLayout`, the redraws) still space objects by their file sizes; Isaac
    refuses a spot that overlaps at the real size, and the redraw takes over. The FR3's EDIT OBJECTS ghost is drawn at the
    file's height for a resized object.
- **The JetRover's look.** The URDF drew each link as its collision boxes.
  - `JetRoverVisualExport` (`just jetrover-visuals`, TeleopXR) writes the lab twin's CAD as OBJ meshes, one per URDF link and
    colour, in the link frames the collision extractor uses (the commanded arm at every joint 0, gripper open). They are
    decimated by vertex clustering on a 1.5 mm grid, a cell's vertices kept apart by facing: 447,441 triangles became 130,421
    in 11 files, 7.7 MB, kept as plain text, not LFS, since the GPU machine clones from GitHub. `visuals.json` names each
    file's link and colour (the lab palette's) and the FBX hashes.
  - `JetRoverSimModel` writes a `<visual>` per mesh, with its colour, and drops a link's box visuals when it has meshes. The
    collision boxes are unchanged, so physics is unchanged. The jaws go to the finger links, which slide where the real jaws
    turn.
  - The renderer labels `<link>_<k>_visual` for the masks (the arm 7, the hand 8, the chassis `mount`) and hides the
    collision boxes should the importer draw them.

## Amendment 7 (2026-11-08): resizing that reaches an edge, the jaws' sides, the hand camera

The user, in the headset: "the resize works, but it maxes out at a certain size, also the hand camera is being partially
blocked. also the gripper close and open seems to be inverted".

- **Resizing.** The bridge's log showed most refusals were OUTSIDE_REGION at small sizes (the post at 1.95×, the can at
  1.25×): an object near the placement region's edge could not grow past it. A resize now moves the object in until its
  new footprint fits (`WorldScene.TryClampOnTable`, `Scene.clamp_on_table`), and is refused only when it fits nowhere. The
  steps go to 1.25⁸ (5.96×; 12 bodies per resizable object in Isaac). Measured after that, objects still stopped growing where
  a neighbour stood (OVERLAPS_OBJECT), so a resize now goes to the nearest spot where the new size fits, on a 1 cm grid over
  the region (`WorldScene.TryFindSpot`, `Scene.spots_near`; Isaac asks PhysX about the arm at up to 24 such spots), and is
  refused only when there is none. `jetrover-bench`'s placement region is widened to
  x 0.12–0.44, y ±0.24.
- **The jaws.** The twin's "left" jaw lies at −y in the URDF's frame, where the left finger's box is at +y and closes toward
  −y, so closing opened them. `JetRoverVisualExport` now gives each jaw to the finger on its side, by where its vertices
  are, and fails when they are not one each side.
- **The hand camera** sat 1.2 cm above the servo horn's plate (the CAD's grey part, z 0.038–0.063 in the gripper frame),
  which filled its view. It is now at (0.10, 0, 0.05), ahead of the plate, looking at (0.26, 0, −0.01): the jaw tips at the
  bottom of the frame, the work ahead.
- **And Isaac kept the old jaws.** Its URDF conversion was cached by the URDF's text alone, and the re-exported meshes keep
  their names, so the bridge and the renderer went on using the meshes converted before the fix (the stream still opened
  the gripper on close). The cache is now named by the URDF and every mesh file it names (`jetrover_bridge.urdf_digest`).
  The FR3's bridge keys its cache as before: its meshes are Franka's, which do not change.

## Amendment 8 (2026-11-08): grasping, and the real network on the headset

The user, through `just isaac-relay` from home: "the gripper still doesn't seem to open all the way and the collision boxes
don't allow me to grab objects ... can you display the ping, lag, loss somewhere, since we are actually communicating
remotely".

- **The fingers collide as their pads.** The URDF's finger collisions were the arm's self-collision boxes, padded round each
  whole jaw linkage: their inner faces stood 21 mm apart at "open", narrower than the 25 mm cube. `JetRoverVisualExport`
  now writes each jaw's last 3 cm as a box (`visuals.json`'s `fingerPads`), and `JetRoverSimModel` uses those as the
  fingers' collisions: 33.5 mm apart at the CAD's pose.
- **The fingers' travel.** Closed, each finger now moves 16.5 mm (the twin's own `TwinGripper.travelMeters`), so the pads
  meet; it was 11 mm, which left 11 mm. Open, each moves 10 mm past the CAD's pose (`FingerOpenMeters`), for 53.5 mm between
  the pads, so the 35 mm can fits. **That 10 mm is a guess:** the real gripper's opening was never measured
  (`GripperOpenPulse` is uncalibrated). The sidecar carries both (`fingerTravelMeters`, `fingerOpenMeters`); the bridge,
  the renderer and the reported opening use them (`SimModel.finger_value`).
- **The real network, live.** `LiveLinkMeter` (Core) takes every joint-state frame and gives, over the last 5 s, the median
  one-way delay to the headset (lag), RFC 3550 jitter, and downlink loss by sequence number; the round trip is
  `ClockSync`'s. The JetRover scene's twin status line reads `live · ping N ms · lag N ms · jitter N ms · loss N%`
  (docs/metrics.md §3, "Live link health"). Diagnostics only; nothing is recorded.
- **`just isaac-relay`** builds into its own folder, so a running relay no longer locks `core-check`'s build of
  `Teleop.Eval`.

## Amendment 9 (2026-11-08): the gripper reports, opens to 70 mm, and the third-person cameras see the whole arm

The user, grasping in Isaac through the relay: "when I grab an object, the measured values seem to phase into the object,
also the gripper doesn't seem to open as far as it does in real life, also the 3rd person camera angles cut off the arm when
it is pointed straight up".

- **The gripper reports its position.** Decision 4 had the sim report what the robot senses, and the gripper never
  answered reads when it was written. It has since 2026-10-25: the real driver reads motors 1–5 and 10
  (`FEEDBACK_MOTOR_IDS`, PR #70, with `ReadBackoff` for one that stops answering). The sidecar now marks the gripper
  `feedback: true`, so the sim reads it in each 20 ms cycle and RobotHost passes the fingers' real place to the headset:
  the measured twin stops at the object instead of showing the command. `just isaac-jetrover-check` expects it valid.
  The J3 fit started before this (e18c645) replays with the old five-read cycle; its scored motors are 1–5 either way.
- **Open, the pads are about 70 mm apart** (`FingerOpenMeters` 0.018), the user's estimate of the real gripper, up from my
  10 mm guess (53.5 mm). Still not measured.
- **The third-person cameras** stood at (0.78, ±0.48, 0.30) looking at (0.12, 0, −0.08), and the arm pointed straight up
  (its tip about 0.46 m over the yaw pivot) left the frame by 180 px. They now stand at (1.15, ±0.68, 0.40) looking at
  (0.08, 0, 0.07): the arm straight up, stretched forward, the chassis and the whole placement region all fall at least
  20 px inside the 640 × 360 frame. The far backdrop screen moves from x 1.0 to 1.6 (the cameras would have stood behind
  it), the side screens stretch to x 1.6 and the floor mat to 1.6.

## Amendment 10 (2026-11-08): one gripper geometry on both sides, and the wrist camera off the roll

The user: "it looks like the gripper expanding more on the video but not on the measured values, why is that and why are
we not just using the measured isaac sim values. Also the camera rotates with the gripper which should not happen".

- **Why they differed.** The headset is given Isaac's measured gripper, but as one number, how closed it is (0–1), and
  each side drew that number its own way: Isaac's fingers slid from 70 mm open (amendment 9), the twin turned its CAD jaws
  from their CAD pose (33.4 mm). Same reading, two widths.
- **The CAD four-bar cannot open 70 mm.** Turning the twin's jaws back past the CAD pose, measured on the twin: the pads'
  gap is 40.6 mm at 10°, 46.4 at 20°, 53.3 at 40–60°, then falls as the jaws swing past. The twin is the robot's own CAD,
  so about 53 mm is most likely the real gripper's most too; amendment 9's 70 mm was an unmeasured estimate.
- **Now both draw the twin's four-bar.** `JetRoverVisualExport` finds the angle for a 53 mm opening (38.25°), which
  `TwinGripper.jawOpenDegrees` takes (set by `XrJetRoverWorldSetup`, checked by `unity-check-xr`), and tabulates the gap
  over the closing, 0 to 1 in 0.05 steps (`fingerTravelByFraction`). `JetRoverSimModel` takes the table for the fingers'
  limits (−9.8 to +16.2 mm) and the sidecar; the bridge, the renderer and the readings interpolate it
  (`SimModel.finger_value`, `finger_fraction`). So a reading draws the same gap in the twin and in Isaac at every value,
  not only at its ends; a linear slide would differ by about 7 mm halfway.
- **The wrist camera rides `wrist_link`**, before the roll joint, not `gripper_link`, so the roll servo no longer turns it.
  The two frames coincide at roll 0, so its mount pose is unchanged.

## Amendment 11 (2026-11-08): J3, the simulated arm fitted to the real one

Decision 6, done (`just jetrover-sim-fit`; `teleop_isaac/simfit.py`, `jetrover_bridge --replay`, run as
`teleop_isaac.simfit_run`). Real sessions' relay targets (`RO`) and torque switches (`TQ`) are replayed at their recorded
times into the bridge's own Isaac arm, headless and unpaced (about 2.4× real time), from each session's first valid reading,
with the driver's feedback cycles; the simulated readings are scored against the recorded ones (`RI`) as docs/metrics.md
section 10 defines. A coordinate search (2 rounds, 22 evaluations of the first 90 s of each fit session) minimised the RMS
tracking error of motors 1–4 (a pooled p95 is blind when most readings are of joints standing still).

- **Split.** Fit: the four scripted drives of 2026-10-26 (`drive-steps-2p5`, `drive-steps-6`, `drive-wp-2p5`,
  `drive-wp-5`). Held out, scored only: `drive-wp-sched`, the two `teleop-2p5` sessions of 2026-10-25 (20:09, 20:20) and
  the two course sessions of 2026-10-27 (22:46, 22:55). Sessions before 2026-10-25 are left out (the old feedback timing).
- **Fitted** (against the previous defaults): drive stiffness 21.2 Nm/rad (60), damping 4.24 Nm·s/rad (1.5), force limit
  1.5 × stall torque (1.0), command delay 0 ms (0), and **servo speed 1.5625 × the commanded pulses per second** (1.0): the
  real servos ramp about half again faster than RobotHost's `pulsesPerSecond` says, which was most of the old sim's lag.
- **Result** (`sim_tracking_error_deg`, motors 1–4 pooled, p50 / p95):

  | | default | fitted |
  |---|---|---|
  | fit sessions | 0.72 / 11.28 | 0.72 / 4.32 |
  | held-out sessions | 1.44 / 12.72 | 0.96 / 5.28 |

  Per held-out session p95: `drive-wp-sched` 10.8 → 3.4, teleop 20:09 10.1 → 3.8, teleop 20:20 12.7 → 5.3, course 22:55
  6.5 → 2.9; course 22:46 19.7 → 18.5. Simple steps lose a little (p95 1.7 → 2.6–2.9) for the much closer motion.
  `sim_settle_error_deg` held-out p95 rose 4.8 → 6.5, all of it course 22:46.
- **Course 22:46, and a relaxed arm.** Its shoulder and wrist were off by 109–119° (p95) under both settings: the session
  parks the arm and switches torque off, and the real readings do not move afterwards (identical 0, 1.2, 5.4 and 21 s
  after `TQ 0`), the servos' gearboxes holding it, while decision 3's limp drives let the simulated arm fall. So torque off
  now holds the arm where it is and the servos ignore commands until torque returns (`--relaxed hold`, the default;
  `--relaxed fall` is the old limp arm). `just isaac-jetrover-check` expects it held.
- **Now the bridge's defaults.** The live bridge and the replay drive one `IsaacServoSim`, so what was fitted is what runs.
- **Not fitted:** the gripper (scored nowhere; its geometry is the CAD's, amendment 10), contact, and payloads. The fit
  replays the robot side only: the headset's network is not in it.
- **Confirmed with the new defaults** (738b1a7, held relaxed arm, score only): held-out p50 / p95 **0.72 / 4.32** (fit
  sessions the same), `sim_settle_error_deg` held-out p95 1.68 (4.8 before J3). Course 22:46 went 18.5 → 5.3 (its
  shoulder 119 → 5.8, wrist 109 → 4.3): the gearbox hold, not the fit, was what it lacked.
- **How the hold holds.** First built as re-targeting each servo at its measured place, which drops the drives' preload
  against gravity: the J1 check saw the shoulder sag 10 more pulses after torque off. Torque off now freezes each setpoint
  where its ramp stands (2c54cf1): 1 pulse. The confirmation above ran with the first form; the course session's figure
  can only improve. With the softer fitted drive the elbow settles 7 pulses short in the J1 check, as the real one does
  (recorded drives: median 4.5–4.7 pulses short, a tenth of settled readings 7 or more; the shoulder 5.8, 8–9), so the
  check's arrival tolerance is 10 pulses, not 3. `just isaac-jetrover-check` passes all nine checks.

## Amendment 12 (2026-11-09): the simulated arm follows its commands as the real one does; the gap was the driving

Scoring the network forecast on an Isaac session (ADR 0035 amendment 4, `just headset-replay`) found every predictor that
replays commands losing to `hold-last` (error p95 7.3° for `learned` against 1.7°), because the arm's readings sat far from
the commands: on the worst joint, 55 % of readings were more than 5° from the command sent 150 ms before. Why, from
`20261109T221225Z-auto`'s `robothost.jlog` (JC, RO, RI, FR) against three real headset sessions (`20261025T200935Z`,
`20261025T202026Z`, `20261027T224623Z`):

| | Isaac | real |
|---|---|---|
| The same relay commands replayed into the sim against what the real arm did (amendment 11's held-out teleop sessions) | p50 0.5–1.0°, p95 2.9–5.3° (one session's tail p95 18.5°) | — |
| Command → servo report, the best-fitting delay (motors 1–3) | 130–170 ms | 190–220 ms |
| A target held 0.5 s or more | within about a pulse; joints 2 and 3 about 5 pulses (1.2°) low, gravity sag | similar |
| Overshoot at a stop, p95; time to within 1° | 1.4°; 330 ms; no ringing | 2.1–3.1°; 230–380 ms |
| Physics | 1000 Hz in real time (39 late steps in 1.1 million), the fitted drives loaded | — |
| Commands clamped on the way to the servo while driving | none (the large wrist mismatch is the park at the end) | none |
| **Command reversals of 3° or more, joints 1–4** | **452 a minute** | 117–267 |
| **Commanded joint travel, summed** | **185°/s** | 51–110°/s |
| **Base joint's command speed p95 / p99** | **315 / 610°/s** | 92–121 / 178–373°/s |

- **The sim is not at fault.** Given the real arm's commands it moves as the real arm did, it holds its targets, it settles
  without ringing, and it keeps real time. It responds somewhat sooner than the real arm (130–170 ms against 190–220 ms).
- **The gap is the driving.** The Isaac session asked for about twice the motion and twice the reversals of any real
  session, and often faster than the relay is told to move (3.8 rad/s, 218°/s). Any arm here, real or simulated, trails
  its commands by 150–200 ms, so with commands reversing every fraction of a second it is rarely where the newest one
  says, and a predictor that replays commands through that lag overshoots and is corrected. The learned model was trained
  on real sessions with half the reversals. This is what the operator felt as jitter in Isaac.
- **One session.** It predicts that driving Isaac as the real arm is driven brings the command–reading gap, and `learned`'s
  error, back to the real sessions' level; a session driven that way and scored with `just headset-replay` tests it.
- **Not decided here.** Whether the headset should cap or smooth the hand's commanded speed, since commands faster than
  the servos are asked to move can never be followed.

## Amendment 13 (2026-11-11): the driver's write gate and its single thread, in the bridge

Decision 1 says Isaac replaces the ROS driver. Until now the bridge replaced it with less than it does: every relay target
started a new servo ramp the moment it arrived. The real driver (`robot_controller_node.py`, `ServoController`) does two
more things, and both shape when a servo moves:

- **The write gate.** A servo takes a new move only after its last one's duration (`|target − last sent| / pulses per
  second`, at least 50 ms) has passed. A target that arrives during that cooldown waits, and the newest one wins; it is
  written between position reads (`flushPending`). A target under 5 pulses from the last one written is dropped. The board
  packet carries the move's duration in whole milliseconds, and servos with the same duration share a packet. A torque
  switch drops the waiting targets and times the next move from the last reading.
- **One thread.** The node runs on rclpy's single-threaded executor. Its 20 ms feedback timer reads every servo back to
  back, and a command that arrives during a read cycle waits until the cycle ends (the command subscription keeps the
  newest 3, the torque one the newest 1). Six reads take about 21 ms, longer than the timer's period, so the timer is
  already due when a cycle ends: the executor runs one waiting command, then the next cycle (see "One command between two
  cycles" below).

The bridge was 40–60 ms quicker to respond than the real arm (amendment 12: 130–170 ms against 190–220 ms) and overshot
less at a stop (1.4° against 2.1–3.1°), and J3's fit (amendment 11) chose servos 1.56 times faster than commanded. Some of
that may have been the fit making up for what is missing here.

- **One implementation of the gate.** It moves out of `ServoController` into `servo_controller/write_gate.py`, plain Python
  with no serial port, and `ServoController` uses it unchanged in behaviour (the driver's tests stay as they are). The
  bridge imports that module from `robot/ros`, as it imports the relay protocol, so the two can never drift.
- **The executor is emulated** (`servo.DriverEmulator`): commands and torque switches that arrive during a read cycle wait
  for its end, the gate's waiting targets are flushed before each read, and each board packet starts the servos' ramps over
  its duration. The live bridge and the J3 replay drive the same emulator.
- **`driver.jlog` from the bridge now has the real driver's gate records:** `CI` as the gate saw it, `GP`, `GD`, and `WR` per
  board packet with its commanded duration (no longer one per servo at the simulated speed).
- **What the real logs say** (the ten recorded sessions of 2026-10-25 and 26, `driver.jlog`): not one of about 85,000 command
  arrays (`CI`) reached the gate during a read cycle (`CY`); the driver was reading about 90 % of the time, in cycles of
  17–20 ms (p50) every 21–23 ms. Replaying each session's `CI` through `write_gate.py`, flushing where the node flushes (at
  a cycle's start and after each read but its last), gives the gate decisions the driver logged: 99.3–99.97 % of `GP` and
  `GD`, 96.5–98.5 % of `WR` packets. The rest are flushes the log does not time.
- **The J3 replay reads at each session's own driver cycles** (`simfit.extract_cycles` from `driver.jlog`,
  `servo.RecordedFeedback`; `--replay-cycles emulated` runs cycles of its own, as before). On a synthetic arm behind the gate,
  the replay on its recorded cycles reproduces it exactly. On cycles of its own, 7 ms out of phase, it misses by 17° (p95;
  p50 1.4°), and without the gate by 14.4° (p50 3.8°). The sweep sends a command every 20 ms and the driver takes one per
  ~27 ms cycle, so its queue is full and the phase decides which targets survive it. The replay's clock now counts physics steps instead of summing them: the sum drifted off the recorded times
  by enough to move a command past a cycle boundary, and that alone had cost 14° on the same arm.
- **First look, without Isaac:** the held-out sessions `drive-wp-sched` and the two `teleop-2p5` of 2026-10-25 (120 s
  each), replayed into an ideal servo that is exactly its timed ramp: tracking p95 14.9° without the gate, 11.0° with it on
  emulated cycles, 10.1° on recorded ones (p50 2.2° → 1.7°). With that servo, J3's 1.56 × speed makes it worse (10.8°), as
  it would if the speed-up had been standing in for the gate.
- **`--driver-gate off`** keeps the old behaviour, for comparing the two and reproducing amendment 11's fit.
- **Then J3 is fitted again** with the gate, on the same split, and the result is written here. The fit's parameters stay
  what they were: what changes is what they have to explain.
- **Not modelled:** the ROS transport between the relay and the driver (J3's `delay_ms` is where it goes; the fit below
  puts 5 ms there), the 3 ms settle wait before a read after a packet, and the board's own pacing of a packet. The relay
  node was listed here as about a millisecond. It is not: see the next section.

### What the refits found (2026-11-12)

J3 was fitted again on amendment 11's split: four sessions for the fit, five held out, 90 s of each fit session per
evaluation. Three runs:

| Run | Driver in the replay | Fitted (stiffness / damping / effort / delay / speed) | Fit objective | Held-out p50 / p95 |
|---|---|---|---|---|
| Amendment 11 | none (`--driver-gate off`) | 21.2 / 4.24 / 1.5 / 0 ms / 1.56 | 2.335 | 0.72° / 4.32° |
| `3548c46` | gate, queue emptied at once | 50.5 / 4.24 / 0.49 / 50 ms / 1.32 | 1.348 | 0.72° / 2.88° |
| `a87cea9` | gate, one command per cycle, relay poll | 50.5 / 3.27 / 0.54 / 5 ms / 0.97 | 1.283 | 0.72° / 3.12° |

Held-out p95 by session (wp-sched, teleop 20:09, teleop 20:20, course 22:46, course 22:55):

- amendment 11: 3.36, 3.84, 5.28, 5.28, 2.88;
- `3548c46`: 2.64, 2.64, 3.12, 3.12, 2.16;
- `a87cea9`: 2.40, 3.12, 3.60, 3.12, 2.40.

Readings are whole pulses, 0.24° each, so these differences are one or two pulses.

- **The gate is most of the gain.** Held-out p95 goes from 4.32° to about 3°, and the fit objective from 2.3 to 1.3.
- **The `3548c46` fit's 50 ms delay was standing in for two stages the replay left out.** The real `driver.jlog` shows
  both (below). With them modelled, the fit wants 5 ms of delay, about a ROS hop. It also wants the servos at 0.97 × the
  commanded speed. Amendment 11's 1.56 × and the 50 ms were both making up for the missing driver. The last fit's search
  had converged: its final steps changed the objective by under 0.002.
- **Held-out p95 is one pulse worse than `3548c46`'s, on the two 2026-10-25 teleop sessions.** In those sessions a
  command takes 54 ms from the relay to the driver (cross-correlation of their `RO` and `CI` streams). The replay explains
  26 ms of that, with the same number of commands reaching the driver (12,674 against 12,666). The other 28 ms is
  unexplained. The 50 ms stand-in happened to fit those two sessions; on the waypoint session the modelled stages already
  account for the lag (below). The `a87cea9` parameters stand, because each one now means what it says.
- **The bridge's defaults are now the `a87cea9` fit:** stiffness 50.45, damping 3.27, effort scale 0.544, speed scale
  0.972. The live bridge has no command delay, so its 5 ms is not applied live: RobotHost's socket to the bridge is a hop
  of its own.

### One command between two cycles, and the relay's 20 ms poll

Two stages between RobotHost and the servos shape when a command takes effect. The Isaac driver emulator now models both
(`servo.DriverEmulator`, `servo.RelayEmulator`; `--relay-poll on|off`, on by default).

- **The executor takes one command array per turn.** In the real `driver.jlog`, 12,740 of the 13,135 gaps between cycles
  in `drive-wp-2p5` ran one command array, 59 ran two, and the rest none. Gaps last 2.5 ms (median). A cycle's first
  command starts 0.95 ms after the cycle ends, and a second command in the same gap 1.8 ms after the first. rclpy takes
  one message per subscription per turn, and the overdue read timer ends the turn.
- **The queue stays full.** The relay sends at 48 Hz and the driver takes one command per cycle, about every 23 ms. So
  the three-deep queue stays full, its oldest entry is dropped, and each command waits about two cycles. The emulator used
  to empty the queue in every gap.
- **How the emulator does it.** Each callback holds the thread for 2.75 ms, and an emulated cycle waits for a running
  callback. With a session's recorded cycles, the turn ends when another callback would still be running at the next
  recorded cycle's start.
- **The ROS relay** (`relay_node.py`) drains its socket every 20 ms. It publishes every torque switch and only the newest
  command, using the relay's own `relay_protocol.sort_actions`, which the Isaac side calls too.
- **Emulated reads take 3.5 ms each** (`JetRoverSimModel.ReadSeconds`): a real read takes 3.2 ms, plus 0.3 ms before the
  next. Reads are now timed from the cycle's start, so a 1 ms physics step no longer rounds each one up to 4 ms. The
  trainer keeps its own 2.9 ms.

Checked against `drive-wp-2p5`, where the relay sent 14,743 commands:

| | Real | Replay now | Replay before |
|---|---|---|---|
| Relay-to-driver lag (cross-correlated, every motor) | 66 ms | 62 ms | 10 ms |
| Commands that reached the driver | 12,858 | 13,037 | 14,743 |

### The other 28 ms: the executor's order (2026-11-12)

The 28 ms the replay left unexplained in the 2026-10-25 teleop sessions comes from rclpy's executor. Of the 10,980
commands in those sessions that could be told apart, 10,977 reached the driver one full cycle after the cycle they
arrived in, though the queue was mostly empty.

- **Why.** Each wait, the executor runs the callbacks it found ready: timers first, then subscriptions. A message that
  arrives during a read cycle is first seen by the next wait. That wait also finds the read timer overdue, so a whole
  read cycle runs before the message does.
- **Why the waypoint drives already matched.** Their queue is always backlogged, so the subscription is always ready.
- **What the emulator does now** (`a7d1a1b`). `DriverEmulator` notes which subscriptions had a message waiting when each
  read cycle starts (`cycles_started` on both feedback classes). After the cycle, while the timer is due, only those run.
  In an idle gap with the timer not due, messages run as they arrive, as before.

Relay-to-driver lag, cross-correlated over every motor:

| Session | Real | Replay now | Replay before |
|---|---|---|---|
| teleop 20:09 / 20:20 | 54 / 52 ms | 48 ms | 26 ms |
| `drive-wp-2p5` | 66 ms | 62 ms | 62 ms |

**J3 fitted again at `a7d1a1b`**, with the same split and two rounds:

- **The search stayed where it started,** the `a87cea9` parameters with no command delay. All 20 probes scored worse than
  the start's 1.273. A delay of 10 ms scored 1.312 and 20 ms scored 1.411, so the fit no longer wants the 5 ms either.
- **The parameters do not change,** and neither do the bridge's defaults.
- **Held-out p50 / p95 is 0.72° / 2.64°,** from 3.12° at the same parameters before the executor rule. This is now the
  best held-out p95 of any fit, `3548c46`'s 50 ms stand-in included.
- **By session** (wp-sched, teleop 20:09, teleop 20:20, course 22:46, course 22:55): 2.40, 2.64, 2.88, 2.64, 1.92.
- **The fit sessions** score p95 0.96, 0.96, 2.16 and 3.12.
