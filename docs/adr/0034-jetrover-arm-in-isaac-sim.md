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
