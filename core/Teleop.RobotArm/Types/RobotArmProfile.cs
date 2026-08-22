using System;

namespace Teleop.RobotArm.Types
{
    /// <summary>
    /// What a joint's angle actually does in the arm's closed-form kinematics
    /// (<c>Kinematics/ArmKinematics.cs</c>) -- see that class's own doc for the full topology this
    /// enum describes (rotating base yaw + exactly two position-solving planar joints + zero or
    /// more orientation-only wrist joints + an optional gripper).
    /// </summary>
    public enum JointRole : byte
    {
        /// <summary>Rotates the whole arm's reach direction. Present iff <see cref="RobotArmProfile.HasRotatingBase"/>.</summary>
        BaseYaw = 0,

        /// <summary>The shoulder-to-elbow link -- one of exactly two position-solving joints. Always present.</summary>
        Proximal = 1,

        /// <summary>The elbow-to-wrist link -- the other position-solving joint. Always present.</summary>
        Distal = 2,

        /// <summary>
        /// An orientation-only trailing joint that does not move the target position -- see
        /// <see cref="JointHardwareSpec.WristIndex"/> for which one when
        /// <see cref="RobotArmProfile.WristJointCount"/> is more than 1.
        /// </summary>
        Wrist = 3,

        /// <summary>The gripper's open/close actuator. Present iff <see cref="RobotArmProfile.HasGripper"/>.</summary>
        GripperMain = 4,

        /// <summary>The gripper's independent rotation actuator. Present iff <see cref="RobotArmProfile.GripperCanRotate"/>.</summary>
        GripperRotate = 5,
    }

    /// <summary>
    /// One joint's physical identity on the real robot: which servo answers to it, and any
    /// per-joint safety limit -- the direct generalization of <c>JetRoverPlantConfig</c>'s old
    /// <c>LowerArmMinPulse</c> (a real mechanical-collision floor hardcoded to one joint by name)
    /// into "an optional per-joint override any joint can carry."
    /// </summary>
    public readonly struct JointHardwareSpec
    {
        public readonly byte MotorId;
        public readonly JointRole Role;

        /// <summary>Meaningful only when <see cref="Role"/> is <see cref="JointRole.Wrist"/> -- which of the <see cref="RobotArmProfile.WristJointCount"/> wrist joints this is, 0-indexed. 0 for every other role.</summary>
        public readonly int WristIndex;

        /// <summary>Optional per-joint floor, radians. Falls back to the plant's global minimum when null.</summary>
        public readonly float? MinAngleRadians;

        /// <summary>Optional per-joint ceiling, radians. Falls back to the plant's global maximum when null.</summary>
        public readonly float? MaxAngleRadians;

        /// <summary>
        /// Added to every commanded kinematic angle for this joint before converting to pulse --
        /// compensates for a real mechanical mounting offset (the servo horn only mounts at
        /// discrete spline positions, so it may not land exactly on the kinematic model's assumed
        /// zero). Found and calibrated 2026-08-17 for the JetRover's proximal joint: commanding
        /// the kinematically-correct "straight up" angle (90 degrees) produced a real pose
        /// noticeably short of vertical, and 97 degrees was needed to reach true vertical for that
        /// joint's actual mounting -- see docs/adr/0011-generic-robot-arm-profiles.md's own note on
        /// this. Deliberately does <b>not</b> affect <see cref="MinAngleRadians"/>/
        /// <see cref="MaxAngleRadians"/>: those express a real, pulse-space physical safety limit
        /// (e.g. a collision floor) that does not shift just because the kinematic-to-pulse mapping
        /// gets a mounting correction -- see <c>GenericArmPlant</c>'s own doc for where each is
        /// applied. Zero (no correction) for a joint that mounts exactly on the kinematic zero.
        /// </summary>
        public readonly float ZeroOffsetRadians;

        public JointHardwareSpec(
            byte motorId, JointRole role, int wristIndex = 0,
            float? minAngleRadians = null, float? maxAngleRadians = null, float zeroOffsetRadians = 0f)
        {
            MotorId = motorId;
            Role = role;
            WristIndex = wristIndex;
            MinAngleRadians = minAngleRadians;
            MaxAngleRadians = maxAngleRadians;
            ZeroOffsetRadians = zeroOffsetRadians;
        }
    }

    /// <summary>
    /// A robot arm's topology and geometry: whether it has a rotating base, its two position-
    /// solving link lengths, how many orientation-only wrist joints trail them, whether it has a
    /// gripper (and whether that gripper independently rotates), and which physical motor id plays
    /// each role. This is the data <c>Kinematics/ArmKinematics.cs</c> is parametrized by, replacing
    /// the old hardcoded <c>ArmLinkLengths</c>/JetRover-only assumption throughout this codebase
    /// (docs/adr/0011-generic-robot-arm-profiles.md).
    ///
    /// <b>Deliberately not arbitrary-N position-solving joints</b> -- closed-form law-of-cosines
    /// IK is well-posed for exactly 2 position constraints; 1 joint only sweeps a circle, and 3+
    /// in one plane is redundant without an extra constraint this platform doesn't compute
    /// (an explicit non-goal, not an oversight -- see ArmKinematics's own doc). Joint-count
    /// flexibility instead comes from <see cref="HasRotatingBase"/>, <see cref="WristJointCount"/>,
    /// and the gripper flags -- this covers the realistic range of small serial teleop-friendly
    /// arms this platform targets, not SCARA/prismatic/non-coplanar-elbow/parallel-linkage arms.
    /// </summary>
    public readonly struct RobotArmProfile
    {
        public readonly string Name;
        public readonly bool HasRotatingBase;

        /// <summary>Height of the shoulder (proximal-joint) pivot above the base-yaw axis, metres.</summary>
        public readonly float BaseHeight;

        /// <summary>Shoulder-to-elbow link length, metres (was <c>ArmLinkLengths.Lower</c>).</summary>
        public readonly float ProximalLinkLength;

        /// <summary>Elbow-to-wrist link length, metres (was <c>ArmLinkLengths.Middle</c>).</summary>
        public readonly float DistalLinkLength;

        /// <summary>
        /// Number of trailing orientation-only wrist joints, &gt;= 0. When more than 1, only the
        /// first absorbs the commanded pitch and the rest are held at exactly 0 -- a documented,
        /// deterministic redundancy-resolution choice (see <c>ArmKinematics.TryInverse</c>'s own
        /// doc), the same spirit as this model's existing elbow-up-only choice.
        /// </summary>
        public readonly int WristJointCount;

        public readonly bool HasGripper;

        /// <summary>Only meaningful when <see cref="HasGripper"/> is true.</summary>
        public readonly bool GripperCanRotate;

        /// <summary>Every joint this profile has, ordered base-to-tip. Length must equal <see cref="ExpectedJointCount"/> -- see <see cref="Validate"/>.</summary>
        public readonly JointHardwareSpec[] Joints;

        public RobotArmProfile(
            string name, bool hasRotatingBase, float baseHeight,
            float proximalLinkLength, float distalLinkLength, int wristJointCount,
            bool hasGripper, bool gripperCanRotate, JointHardwareSpec[] joints)
        {
            Name = name;
            HasRotatingBase = hasRotatingBase;
            BaseHeight = baseHeight;
            ProximalLinkLength = proximalLinkLength;
            DistalLinkLength = distalLinkLength;
            WristJointCount = wristJointCount;
            HasGripper = hasGripper;
            GripperCanRotate = gripperCanRotate;
            Joints = joints;
        }

        /// <summary>Actual joint count -- <see cref="Joints"/>'s own length is authoritative; use <see cref="Validate"/> to catch it drifting from <see cref="ExpectedJointCount"/>.</summary>
        public int JointCount => Joints.Length;

        /// <summary>What <see cref="JointCount"/> should be, derived from the topology flags alone -- the check <see cref="Validate"/> runs <see cref="Joints"/> against.</summary>
        public int ExpectedJointCount =>
            (HasRotatingBase ? 1 : 0) + 2 + WristJointCount +
            (HasGripper ? (GripperCanRotate ? 2 : 1) : 0);

        /// <summary>
        /// Index into <see cref="Joints"/> for the given role (and, for <see cref="JointRole.Wrist"/>, wrist
        /// index), or -1 if this profile has no such joint. Linear scan over at most a handful of
        /// joints -- not a hot-path concern, called a few times per command, never per-frame in a loop.
        /// </summary>
        public int TryGetJointIndex(JointRole role, int wristIndex = 0)
        {
            for (int i = 0; i < Joints.Length; i++)
            {
                JointHardwareSpec joint = Joints[i];
                if (joint.Role == role && (role != JointRole.Wrist || joint.WristIndex == wristIndex))
                {
                    return i;
                }
            }

            return -1;
        }

        /// <summary>
        /// Checks internal consistency: <see cref="Joints"/>'s length and role composition match
        /// the topology flags, motor ids are unique, wrist indices are exactly 0..WristJointCount-1
        /// with no gaps or repeats, link lengths are positive, and any given per-joint angle limits
        /// are ordered. Returns null when valid, otherwise a human-readable reason -- callers
        /// (<c>GenericArmPlant</c>'s constructor, the <c>build-profile</c> CLI verb) fail loud on a
        /// non-null result rather than silently misrouting angles to the wrong servos.
        /// </summary>
        public string? Validate()
        {
            if (string.IsNullOrWhiteSpace(Name))
            {
                return "Profile name must not be empty.";
            }

            if (ProximalLinkLength <= 0f || DistalLinkLength <= 0f)
            {
                return "ProximalLinkLength and DistalLinkLength must both be positive.";
            }

            if (WristJointCount < 0)
            {
                return "WristJointCount must not be negative.";
            }

            if (Joints.Length != ExpectedJointCount)
            {
                return $"Expected {ExpectedJointCount} joints for this topology, but Joints has {Joints.Length}.";
            }

            var seenMotorIds = new System.Collections.Generic.HashSet<byte>();
            var seenWristIndices = new System.Collections.Generic.HashSet<int>();
            int baseYawCount = 0, proximalCount = 0, distalCount = 0, gripperMainCount = 0, gripperRotateCount = 0;

            foreach (JointHardwareSpec joint in Joints)
            {
                if (!seenMotorIds.Add(joint.MotorId))
                {
                    return $"Motor id {joint.MotorId} is used by more than one joint.";
                }

                if (joint.MinAngleRadians.HasValue && joint.MaxAngleRadians.HasValue &&
                    joint.MinAngleRadians.Value >= joint.MaxAngleRadians.Value)
                {
                    return $"Joint with motor id {joint.MotorId} has MinAngleRadians >= MaxAngleRadians.";
                }

                switch (joint.Role)
                {
                    case JointRole.BaseYaw: baseYawCount++; break;
                    case JointRole.Proximal: proximalCount++; break;
                    case JointRole.Distal: distalCount++; break;
                    case JointRole.GripperMain: gripperMainCount++; break;
                    case JointRole.GripperRotate: gripperRotateCount++; break;
                    case JointRole.Wrist:
                        if (joint.WristIndex < 0 || joint.WristIndex >= WristJointCount || !seenWristIndices.Add(joint.WristIndex))
                        {
                            return $"Wrist joint index {joint.WristIndex} is out of range or duplicated (WristJointCount={WristJointCount}).";
                        }
                        break;
                }
            }

            if (baseYawCount != (HasRotatingBase ? 1 : 0))
            {
                return $"Expected {(HasRotatingBase ? 1 : 0)} BaseYaw joint(s) for HasRotatingBase={HasRotatingBase}, found {baseYawCount}.";
            }

            if (proximalCount != 1 || distalCount != 1)
            {
                return "Exactly one Proximal and one Distal joint are required.";
            }

            if (gripperMainCount != (HasGripper ? 1 : 0))
            {
                return $"Expected {(HasGripper ? 1 : 0)} GripperMain joint(s) for HasGripper={HasGripper}, found {gripperMainCount}.";
            }

            if (gripperRotateCount != (HasGripper && GripperCanRotate ? 1 : 0))
            {
                return $"Expected {(HasGripper && GripperCanRotate ? 1 : 0)} GripperRotate joint(s), found {gripperRotateCount}.";
            }

            return null;
        }

        /// <summary>
        /// The exact JetRover arm this codebase has always driven, expressed as a profile instead
        /// of hardcoded fields -- preserves every number <c>ArmLinkLengths.Measured</c> and the old
        /// <c>SERVO_ENUM</c> (jetrover-teleop-ros) had, so default behavior is unchanged by this
        /// generalization. <see cref="JointHardwareSpec.MinAngleRadians"/> on the proximal joint is
        /// the old <c>LowerArmMinPulse</c> (50) re-derived exactly in angle space --
        /// (50 - ZeroPulse=500) / PulsePerRadian -- a real collision-safety number
        /// (robot/README.md), not eyeballed; <c>RobotArmProfileTests</c> checks this conversion
        /// against the original pulse-space arithmetic. The proximal joint's
        /// <see cref="JointHardwareSpec.ZeroOffsetRadians"/> (+7 degrees) is a real, empirically
        /// calibrated mounting correction (2026-08-17, live-hardware testing): commanding the
        /// kinematically "correct" 90 degrees produced a real pose noticeably short of true
        /// vertical, and +7 degrees was needed to reach it for this joint's actual servo-horn
        /// mounting -- see docs/adr/0011-generic-robot-arm-profiles.md.
        /// </summary>
        public static RobotArmProfile JetRoverMeasuredDefault
        {
            get
            {
                const float pulsePerRadian = 1000f / (240f * MathF.PI / 180f);
                const int zeroPulse = 500;
                const int lowerArmMinPulse = 50;
                float proximalMinAngleRadians = (lowerArmMinPulse - zeroPulse) / pulsePerRadian;
                const float proximalZeroOffsetRadians = 7f * MathF.PI / 180f;

                return new RobotArmProfile(
                    name: "jetrover",
                    hasRotatingBase: true,
                    baseHeight: 0.035f,
                    proximalLinkLength: 0.13f,
                    distalLinkLength: 0.13f,
                    wristJointCount: 1,
                    hasGripper: true,
                    gripperCanRotate: false,
                    joints: new[]
                    {
                        new JointHardwareSpec(motorId: 1, role: JointRole.BaseYaw),
                        new JointHardwareSpec(
                            motorId: 2, role: JointRole.Proximal, minAngleRadians: proximalMinAngleRadians,
                            zeroOffsetRadians: proximalZeroOffsetRadians),
                        new JointHardwareSpec(motorId: 3, role: JointRole.Distal),
                        new JointHardwareSpec(motorId: 4, role: JointRole.Wrist, wristIndex: 0),
                        new JointHardwareSpec(motorId: 5, role: JointRole.GripperMain),
                    });
            }
        }
    }
}
