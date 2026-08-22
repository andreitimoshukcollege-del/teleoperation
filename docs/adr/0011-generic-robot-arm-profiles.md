# 11. Generic robot-arm profiles: configurable topology, id+angle+speed wire protocol

## Status

Accepted.

## Context

Every layer of this platform's robot-arm support was hardcoded to one specific robot — the
JetRover 4-DOF arm — with its geometry and joint topology independently duplicated by hand across
at least eight places, with no shared schema: `FourDofArmKinematics`/`ArmLinkLengths` (closed-form
IK, three hardcoded link-length floats), `JointCommandFrame`/`JointCommandCodec` (a fixed 5-float
Unity→`Teleop.RobotHost` wire struct), `JetRoverPlant`/`JetRoverPlantConfig` (per-joint *named*
fields — `_targetPulseBase`/`_targetPulseLower`/`_targetPulseMiddle`/`_targetPulseUpper` — not
arrays), `RelayProtocol`'s `LocalArmCommand`/`LocalFeedback` (a fixed 5-float
`Teleop.RobotHost`→ROS struct, `docs/adr/0010`), and on the Jetson side `servo_id_enum.py`
(a hardcoded `Enum`), `servo_sub_callback.py` (one hand-written callback per joint),
`robot_controller_node.py` (one hand-written topic/publisher per joint, ×2 for command and
feedback), and `relay_protocol.py`'s own fixed struct format. Only the literal segment-length
numbers were ever genuinely shared (`Teleop.JetRover`, one package compiled by both Unity and
`Teleop.RobotHost`, per `docs/adr/0009`) — the *topology itself* (joint count, names, motor-id
identity) was duplicated by hand in every one of those eight places, with nothing to keep them in
sync but discipline.

The operator asked for two things: the ability to define different **robot profiles** — rotating
base yes/no, joint count, segment lengths, gripper yes/no, gripper-rotates yes/no — so this
codebase can eventually drive a different physical robot, not just this one JetRover; and a wire
protocol, all the way to the real robot, that carries only **motor id, angle, and speed** per
joint, decoded generically by id with no per-joint hardcoded knowledge anywhere.

Three scope decisions were made directly, not assumed:

1. **Parametrized topology, not an arbitrary numerical IK solver.** Closed-form law-of-cosines IK
   is well-posed for exactly two position-solving joints (two unknowns, two position constraints)
   — one joint only sweeps a circle, and three or more in one plane is redundant without an extra
   constraint this platform doesn't compute. Joint-count flexibility instead comes from an
   optional rotating base, a configurable number of trailing orientation-only wrist joints, and an
   optional gripper (with optional independent rotation). This is an explicit, honestly-scoped
   non-goal, not an oversight: SCARA, prismatic, non-coplanar-elbow, and parallel-linkage arms are
   not representable. It covers the realistic range of small serial teleop-friendly arms this
   platform targets.
2. **Unity gets only the minimal mechanical edit needed to keep working**, not a runtime
   profile-picker UI — deferred to a separate, human-reviewed follow-up (`unity/` requires human
   review per this repo's own rule, and can't be verified headlessly).
3. **Both wire hops are generalized in this same change** — Unity→`Teleop.RobotHost` *and*
   `Teleop.RobotHost`→ROS/Jetson — including collapsing the Jetson side's hardcoded per-joint
   topics/callbacks into one generic, motor-id-keyed decode loop. This is real, hardware-verified
   pipeline surface being rewritten in one pass, accepted because it directly delivers the
   operator's actual ask (id+angle+speed, decoded generically) rather than leaving half the
   pipeline still joint-name-hardcoded. Verified first against the real servo SDK
   (`board_manager/robot_controller_sdk.py`'s `bus_servo_set_position(duration, positions)`) that
   one physical packet carries one shared duration for its whole batch — but the existing code
   already calls it once per individual servo and never batches, so per-joint independent speed
   was already the natural pattern; generalizing it introduced no new complexity there.

A side effect of generalizing both hops: the gripper stops being a special degrees-based case. It
becomes just another joint with its own motor id, flowing through the exact same generic path as
every other joint — removing `setGripperPos`/`degToPulse`/`pulseToDeg` from the Jetson side
entirely (both command and feedback move to pulse units uniformly, since
`bus_servo_read_position` already returns raw pulse and the old degree round-trip existed only
because the previous fixed protocol happened to choose degrees for that one field).

## Decision

**`core/Teleop.JetRover` is renamed to `core/Teleop.RobotArm`.** Mechanical, no behavior change:
once the kinematics/wire types are generic, the old name would mislead the next implementer about
what the package holds (`docs/adr/0009`'s own reason this package isn't in `Teleop.Core` — "one
specific robot's ruler-measured hardware geometry" — now applies only to the *default profile
instance*, not the code). Touches the directory, `.csproj`/`.asmdef`/`package.json`
(`com.teleop.jetrover` → `com.teleop.robotarm`), `unity/TeleopVR/Packages/manifest.json`, and every
consuming `using` site. Stays a sibling leaf package, not part of `Teleop.Core` — no second
competing IK technique is anticipated, so no `Registries.cs` entry either (the same reasoning
`Plant/CLAUDE.md` already gives for why there is no `Plants` table).

**A new `RobotArmProfile` type (`Teleop.RobotArm/Types/RobotArmProfile.cs`) replaces
`ArmLinkLengths`.** Carries `HasRotatingBase`, `BaseHeight`, `ProximalLinkLength`,
`DistalLinkLength`, `WristJointCount`, `HasGripper`, `GripperCanRotate`, and an ordered
`JointHardwareSpec[]` (motor id, `JointRole`, optional per-joint `MinAngleRadians`/
`MaxAngleRadians`). `RobotArmProfile.Validate()` checks internal consistency (joint count matches
the topology flags, motor ids unique, wrist indices well-formed, link lengths positive) and fails
loud rather than letting a malformed profile silently misroute angles to the wrong servos.
`RobotArmProfile.JetRoverMeasuredDefault` reproduces every number this codebase has always used —
including the old `LowerArmMinPulse` safety floor, re-derived exactly into angle space
(`(50 - 500) / PulsePerRadian`) as the proximal joint's `MinAngleRadians` — so default behavior is
unchanged. Profiles are JSON files (`core/RobotProfiles/*.json` convention, sibling to
`experiments/`), loaded by `Teleop.RobotHost` directly (a host, file I/O is fine) via a small,
deliberately duplicated loader in each of `Teleop.RobotHost` and `Teleop.Eval` (same reasoning
this codebase already applies to `MonotonicClock`: two sibling host processes each owning a
~40-line utility isn't worth a shared dependency edge between them).

**`ArmKinematics` (`Teleop.RobotArm/Kinematics/ArmKinematics.cs`) replaces
`FourDofArmKinematics`.** Same closed-form math, parametrized by `RobotArmProfile` instead of
hardcoded fields: `Forward`/`TryInverse` use `ProximalLinkLength`/`DistalLinkLength` in place of
`Links.Lower`/`Links.Middle`, force `baseYaw` to 0 when `!HasRotatingBase`, and write into a
caller-supplied `Span<float> wristPitchesOut` sized to `WristJointCount` — when more than one
wrist joint exists, only the first absorbs the commanded pitch and the rest are held at exactly 0
(a documented, deterministic redundancy-resolution choice, the same spirit as the existing
elbow-up-only choice). A new `MapAnglesToJointTargets` helper walks `RobotArmProfile.Joints` and
emits one `JointTarget` per entry, turning IK output into the wire-ready shape for either hop.

**Both wire hops become count-prefixed arrays of `JointTarget{MotorId, Angle, Speed}`.**
- Unity→`Teleop.RobotHost` (`JointCommandCodec`, version 1→2): angle stays in radians (Core's
  convention for this hop); a new `MaxJointsPerMessage` (derived from a joint-listener-only
  `MaxJointDatagramBytes` budget, kept separate from the unrelated Cartesian path's own
  `MaxDatagramBytes`) bounds one record.
- `Teleop.RobotHost`→ROS (`RelayProtocol`, version 3→4): both angle and speed move to pulse units
  (pulse, and pulses/second), continuing `docs/adr/0010`'s reasoning for choosing pulse over
  radians on this hop. Speed becomes an explicit, wire-transmitted value
  (`GenericArmPlantConfig.PulsesPerSecond`) instead of a hardcoded Python module constant, so the
  two independently-deployed sides no longer have to keep that number in sync by hand. The fixed
  `LocalArmCommand`/`LocalFeedback` structs are deleted outright.

**`JetRoverPlant`/`JetRoverPlantConfig` are renamed and generalized to `GenericArmPlant`/
`GenericArmPlantConfig`, one clean cutover** — not a parallel implementation, since duplicating the
intricate belief-tracking/staleness logic a second time is exactly what `docs/adr/0010` fought to
stop doing. Per-joint named fields become arrays sized to `RobotArmProfile.JointCount`.
`CommandJointAngles` necessarily changes signature, from four fixed named floats to
`ReadOnlySpan<JointTarget>` — an honest, unavoidable breaking change matching the new wire shape.
`Command` and `CommandJointAngles` now converge on one shared `ApplyJointTargets(ReadOnlySpan<JointTarget>)`
tail, matched by motor id (a target for a motor id the profile doesn't have is ignored, not
thrown, so a stale or foreign sender can't crash the plant) — a structural simplification over the
pre-existing design, where the two entry points each built their own named-parameter call into a
shared method. `LowerArmMinPulse` (hardcoded to one joint by name) generalizes to an optional
per-joint `MinAngleRadians`/`MaxAngleRadians` any joint can carry. The gripper stays open-loop
(no per-cycle stepping, sent as an absolute target every call) exactly as before, now expressed as
"any joint whose `JointRole` is `GripperMain`/`GripperRotate`" instead of a separately named field.

**On the Jetson (`jetrover-teleop-ros`), the per-joint hardcoding collapses into a
motor-id-keyed decode loop.** `relay_protocol.py`'s fixed struct format becomes a header + repeated
tuple encode/decode mirroring `RelayProtocol`'s byte layout exactly. The internal hop between
`relay_node.py` and `robot_controller_node.py` uses a plain `std_msgs/Float32MultiArray` (not a
new custom ROS message type) — a deliberate implementation choice to avoid introducing `rosidl`
message-generation build machinery for a purely internal, single-hop array, packed as
`[count, motor_id, value, ...]` tuples mirroring `relay_protocol.py`'s own shapes.
`servo_sub_callback.py`'s six per-joint callbacks collapse into one `armCommandArrayCbk` looping
over the array and calling `setPos` per entry (the gripper included, no longer a separate
`setGripperPos` path). `robot_controller_node.py`'s six per-joint feedback-polling methods
collapse into one loop over a small, explicit, Jetson-local `FEEDBACK_MOTOR_IDS` list — deliberately
separate from the operator-side `RobotArmProfile`, since this node no longer needs to know joint
*roles*, only which raw motor ids exist on its bus to read back. `servo_controller.py`'s `setPos`
gains an explicit `pulses_per_second` parameter (the old hardcoded `PULSES_PER_SECOND` module
constant is deleted); `setGripperPos`/`degToPulse`/`pulseToDeg` are deleted entirely. `SERVO_ENUM`
(`servo_id_enum.py`) is deleted — motor ids now arrive directly on the wire, so nothing constructs
a command by looking up a name. Safety clamping stays entirely host-side
(`GenericArmPlant`'s per-joint pulse floor/ceiling) — the Jetson side is now a genuinely "dumb"
id-keyed read/write layer with no clamping logic of its own.

**A new `Teleop.Eval build-profile` verb** (`core/Teleop.Eval/BuildProfile/`) interactively prompts
for a robot's topology (rotating base, link lengths, wrist joint count, gripper, per-joint motor
ids and optional angle limits) and writes a validated `RobotArmProfile` JSON file — explicitly not
a hardware scan, since physical dimensions can't be auto-detected. Reads from an injected
`TextReader`/writes to an injected `TextWriter` rather than `Console` directly, so a test can pipe
a canned answer transcript and assert on the result, per this repo's "an algorithm that cannot be
evaluated headlessly does not count" ethos extended to this interactive CLI tool.

**Unity gets the smallest edit that keeps it working.** `JetRoverArmConfig.cs` and
`JetRoverArmRig.cs` are unchanged — the rig's one entry point, `ApplyAngles(baseYaw, proximalPitch,
distalPitch, wristPitch, wasClamped)`, keeps the same 4-float meaning regardless of how many
joints the underlying profile actually has; building a truly generic pivot array is exactly the
deferred picker-UI-adjacent work. A new `RobotArmProfileData.cs` (a plain `[Serializable]` POCO
mirror of `RobotArmProfile`, since `JsonUtility` can't deserialize a constructor-only readonly
struct or `Nullable<float>`) loads via the existing `ConfigLoader.Load<T>` pattern from a new
sibling config file (`jetrover_arm_profile.json`), defaulting in-code to
`RobotArmProfile.JetRoverMeasuredDefault`'s exact numbers. `JetRoverOperatorBridge.cs` is the one
file that can't avoid an edit, since it's the only place that calls the kinematics API and builds
the wire message directly — the edit is mechanical: load the profile, call the renamed/generalized
`ArmKinematics`/`JointCommandCodec` APIs, keep the same call into `armRig.ApplyAngles`.

## Consequences

- **Two coordinated breaking wire-version bumps** (`JointCommandCodec` 1→2, `RelayProtocol` 3→4)
  requiring Unity, `Teleop.RobotHost`, and the Jetson (`jetrover-teleop-ros`, including a
  `colcon build` to pick up the rewritten Python) to redeploy together — no dual-support window,
  consistent with this repo's established clean-cutover style (`docs/adr/0009`, `docs/adr/0010`).
  A stale side on either hop rejects on the version byte rather than misinterpreting a
  differently-shaped record.
- `FourDofArmKinematics`/`ArmLinkLengths`/the old fixed-shape `JointCommandFrame`/`JointCommandCodec`/
  `JetRoverPlant`/`JetRoverPlantConfig`/`LocalArmCommand`/`LocalFeedback`/`servo_id_enum.py`/
  `setGripperPos`/`degToPulse`/`pulseToDeg` are deleted outright, not deprecated.
- Full existing test suites (`FourDofArmKinematicsTests` → `ArmKinematicsTests`,
  `JointCommandCodecTests`, `JetRoverPlantTests` → `GenericArmPlantTests`, `RelayProtocolTests`,
  `UdsRelayClientTests`, `test_servo_controller.py`, `test_relay_protocol.py`) are ported with
  `RobotArmProfile.JetRoverMeasuredDefault` as an explicit regression baseline on both the .NET and
  Python sides, plus new tests against a second, structurally different profile (no rotating base,
  different link lengths, no wrist joint, no gripper) proving the generalization actually
  generalizes.
- Real end-to-end hardware re-verification (`move-arm` against the JetRover, per this project's
  established supervised-hardware-test discipline) is required before calling JetRover
  teleoperation "working" again on the new wire.
- A second physical robot is not yet supported end-to-end by this ADR alone — `build-profile` can
  describe one, and `GenericArmPlant`/the wire protocol can carry its commands, but nothing in this
  change adds a UI to *select* a profile at runtime; that remains the deferred, human-reviewed
  Unity follow-up.
