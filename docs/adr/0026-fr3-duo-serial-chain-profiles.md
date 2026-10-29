# 26. The FR3 Duo: serial-chain, multi-arm robot profiles and a 256-byte joint datagram

## Status

Proposed (2026-10-28). It opens the FR3 Duo work, with ADR 0027 (the arm backend and the
trajectory shaper) and ADR 0028 (the learned predictor for the FR3).

The user asked for the platform to move to a better arm, Franka's **FR3 Duo**, keeping the base and
the research idea and retraining the model. They chose:
- **the hardware:** it arrives later. Build simulation and training now, and design for a real
  backend without building it;
- **the JetRover:** it stays alongside as a supported robot. Its bytes, recordings, golden files
  and layout v3 model stay valid;
- **the operator:** drives both arms, the Galaxy XR left hand the left arm and the right hand the
  right arm;
- **delivery:** ADRs first, then one PR per phase.

## Context

**The FR3 Duo** is two Franka Research 3 (v2) arms on a fixed mount, each with a Franka Hand, plus
a head with a camera mounting point. Franka publishes its description, which is vendored in
`robot/franka/fr3_duo/` with its provenance:
- each arm has 7 revolute joints. Each joint has a position limit, a velocity limit (2.62 rad/s on
  joints 1–4; 5.26, 4.18 and 5.26 rad/s on joints 5–7), `safety_controller` soft limits and
  `position_based_velocity_limits`;
- each hand has one prismatic finger (0–0.04 m, 0.2 m/s), and the second finger mimics it;
- the arms' bases are **10 cm apart** (y = ±0.05018 m), tilted outward and down. Their workspaces
  overlap in front of the robot.

**What this codebase can describe today.** `RobotArmProfile` (ADR 0011) is generic only within one
planar topology:
- an optional base yaw;
- exactly one proximal joint and one distal joint in a vertical plane;
- N wrist pitches;
- an optional gripper with an optional roll.

`Validate()` requires the proximal and the distal joint (`RobotArmProfile.cs:267`). ADR 0011 ruled
out redundant arms, arbitrary axes and numerical IK on purpose. Neither an FR3 arm (7 joints,
alternating axes, offsets a4 and a5) nor two arms on one robot can be written down in it. Everything
downstream assumes the same thing:
- the collision model (`ArmPose`, five fields);
- the learned model's layout (six fixed motor ids);
- the Unity rig (four pivots);
- the right-hand-only clutch.

**The joint wire is too small for two arms.** `JointCommandCodec` and `JointStateCodec` size
themselves from a literal 128-byte datagram. That allows 12 commanded and 13 reported joints. The
FR3 Duo has 16 (2 × 7 + 2 fingers): a command is 14 + 16·9 = 158 bytes and a state frame is
46 + 16·6 = 142 bytes. The literal appears in eight places:
- `JointCommandCodec.cs:59`;
- `JointStateCodec.cs:31`;
- RobotHost `Program.cs:40`;
- the Bridge's `JetRoverJointStateLink.cs:114` and `JetRoverOperatorBridge.cs:74`;
- Eval's `JointDriveCommand.cs:51`, `ClockSyncCheckCommand.cs:72` and `MoveArmCommand.cs:40`.

## Decision

1. **A new profile kind, `SerialChainProfile`, beside `RobotArmProfile`.**
   - **Where it lives:** `Teleop.RobotArm/Types/SerialChainProfile.cs`, in the same leaf package as
     ADR 0011's profile.
   - **`RobotArmProfile` is not touched.** Its closed-form IK, `ArmSolver`, the JetRover collision
     model and every JetRover test stay as they are.
   - **One file describes one robot:** one RobotHost, one wire, one `ClockSync`, one recorded
     session. A profile has a name, a `kind` (`"serial-chain"`; a file without a `kind` is ADR 0011's
     planar profile, so `jetrover.json` is unchanged) and an ordered `arms[]`. Each arm has:
     - **a name** (`left`, `right`);
     - **a mount**, the fixed transform from the robot's root frame to the arm's base;
     - **`joints[]`**, in chain order. Each joint carries:
       - the motor id;
       - the URDF joint name;
       - the type (`revolute` or `prismatic`);
       - the origin (from the parent link's frame);
       - the unit axis;
       - the position limits and the soft limits;
       - the velocity, acceleration and jerk limits;
       - the position-based velocity limit (deceleration limit, velocity offset);
     - **a tool frame:** the fixed transform from the last joint to the TCP (flange + hand + TCP);
     - **an optional gripper:** its motor id, its travel (0–0.04 m per finger for the Franka Hand),
       its speed, and whether a second finger mimics the first.

     **In the file, every frame is the URDF's own numbers.** The mount, each origin and the tool are
     each a list of `{joint, xyz, rpy}`, one entry per URDF joint folded into it. The profile type
     composes them at load. So the importer does no trigonometry, a re-import is the same bytes on
     every platform, and each number can be checked against the URDF by eye.

     **One document type is shared by every reader.** The file's form
     (`Teleop.RobotArm/Types/SerialChainProfileDocument.cs`) has plain public fields named as the
     JSON names them. RobotHost and Eval read it with `System.Text.Json` and Unity with `JsonUtility`.
     Only the file handling is per host, unlike ADR 0011's per-host mirrors.
   - **`Validate()` fails loudly on:**
     - non-unit axes or quaternions;
     - duplicate ids across arms;
     - a lower limit above its upper limit;
     - non-positive velocity, acceleration or jerk limits;
     - an arm with no joints.
2. **Motor ids are one byte, unique across the robot, and arranged per arm.** For the FR3 Duo:
   - left arm joints 1–7, left hand 8;
   - right arm joints 11–17, right hand 18.

   Every predictor is already keyed by motor id and sized by `JointPredictorConfig.MaxJoints`, so 16
   ids need no new predictor plumbing. On the wire the gripper keeps ADR 0011's convention: a 0..1
   opening fraction. For the Franka Hand, 0 is closed and 1 is 0.04 m per finger.
3. **The FR3 Duo's profile is generated, never hand-edited.** A new verb, `Teleop.Eval import-urdf
   --urdf <file> --out <file> --name <profile> --arm <name>:<tipLink>:<firstMotorId>...
   --gripper <arm>:<fingerJoint>:<motorId>... --acceleration <a> --jerk <j> --upstream <repo@sha>`:
   - reads the URDF (`System.Xml.Linq`, a host, so file I/O is fine);
   - takes each arm as the URDF path from the root link to its tip link, and folds the fixed joints
     before the first moving joint into the mount, those between moving joints into the next joint's
     origin, and those after the last into the tool;
   - refuses a `continuous`, `floating` or `planar` joint, or a mimic joint, on an arm's path;
   - writes `core/RobotProfiles/fr3-duo.json`.

   It records the URDF's sha256 and the upstream commit in the profile. A test re-imports the
   vendored URDF and requires the committed file byte for byte. A new `just import-fr3-profile`
   recipe runs it. Jerk and acceleration limits are not in the URDF, so the verb takes them from
   Franka's published FR3 table (10 rad/s², 5000 rad/s³), passed as named arguments that the recipe
   fixes and the profile records.
4. **Forward kinematics is generic and allocation-free.**
   `Teleop.RobotArm/Kinematics/SerialChainKinematics.cs` computes every link frame and the TCP of one
   arm in the robot's root frame from its joint values:
   - it composes, per joint, origin · rotation (or translation) about the axis;
   - it works in double precision and writes into a caller-supplied span.

   It is tested two ways, with two oracles that do not share its code:
   - each arm's base-to-flange transform matches Franka's published Craig DH table at seeded random
     joint vectors;
   - each mount matches upstream's `fr3_duo_mount_v0_3/kinematics.yaml` (translation plus roll,
     pitch and yaw).
5. **Inverse kinematics and collision are later decisions, not this one.**
   - **Not needed for the research slice.** Prediction runs in joint space, and the simulator and the
     scripted operators generate joint-space commands, as `joint-drive` does today.
   - **Needed before Unity drives the FR3 and before any real arm moves:** a redundant-arm solver (7
     joints, a null space) and a self- and arm-to-arm collision model fitted to the URDF's collision
     meshes. Each gets its own amendment here.
6. **One joint datagram budget, 256 bytes, in one place.** `JointDatagram.MaxBytes = 256` goes in
   `Teleop.RobotArm/Wire/`. Both codecs' `MaxJointsPerMessage` and every receive buffer above derive
   from it: 26 commanded joints, 35 reported. The Cartesian path's own `MaxDatagramBytes` is a
   different budget and stays 128.
   - **The layouts and versions do not change.** A JetRover datagram is byte for byte what it was,
     and recorded sessions and golden files stay valid.
   - **Old receivers fail closed.** `UdpTransport` refuses a datagram larger than its buffer, and the
     decoders refuse a count above their limit, so a stale headset build cannot misread an FR3 frame.
   - **`sensedTicks` stays one value per frame.** It is the older of the arms' latest state times.
     The FR3 controller reports at 1 kHz, so this costs under a millisecond.
7. **Hosts pick the robot from the profile.** RobotHost and the Eval verbs that load a profile take
   `--profile <file>` as today and dispatch on `kind`:
   - a planar profile builds today's JetRover path, unchanged;
   - a serial-chain profile builds ADR 0027's path.

   The default with no `--profile` stays the JetRover. Unity's profile switch belongs to the Unity
   phase and to human review.

## Consequences

- **Two profile types live side by side for good.** A future robot that fits ADR 0011's topology
  can still use it. A robot that does not is a serial chain.
- **The JetRover path is a regression baseline, not a dependency.** Every existing JetRover test,
  golden file and recording must pass unchanged through each FR3 phase. Any phase that would have to
  change one has gone wrong.
- **A Unity build that predates this change rejects the FR3's frames.** That is the intended
  failure, not a crash. Bridge's buffers grow in the same change, so `just bridge-check` sees it.
- **Mesh payloads.** The visual `.dae` meshes (58 MB) are in LFS. The collision `.stl` meshes
  (180 KB) are plain git so that capsule fitting runs on Linux and in CI.
- **Nothing here moves a robot.** There is no FR3 to move, and the JetRover's bytes are unchanged.

## Alternatives considered

- **Extending `RobotArmProfile` with optional chain fields.** Rejected. Every consumer of the planar
  roles (closed-form IK, `ArmSolver`, `ArmPose`, the rig) would have to branch on which half of the
  type is filled in. Two types with one `kind` switch at the host keeps each simple.
- **Two single-arm profiles plus an assembly file.** Rejected. Three loaders (RobotHost, Eval,
  Unity) would each have to stitch three files together. The robot is also one unit: one host, one
  clock, one session.
- **One joint stream per arm.** Rejected. It doubles the headset's keepalive, `ClockSync` and
  predictor bank, and RobotHost's subscriber, session and power port, for two arms that share one
  robot clock.
- **Denavit–Hartenberg parameters in the profile instead of URDF origins.** Rejected. DH cannot
  express the mount's tilted fixed frames without extra conventions, while the URDF is what Franka
  maintains. The DH table remains a test oracle.
