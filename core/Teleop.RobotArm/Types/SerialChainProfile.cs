using System;
using System.Collections.Generic;

namespace Teleop.RobotArm.Types
{
    /// <summary>How a chain joint moves its child frame: a rotation about its axis, or a slide along it.</summary>
    public enum ChainJointType : byte
    {
        Revolute = 0,
        Prismatic = 1,
    }

    /// <summary>
    /// One chain joint's limits, in its own units (radians or metres, per second and so on). These are
    /// the robot's own limits as its description gives them. A host that commands the robot keeps its
    /// own margin below them (docs/adr/0027 decision 1).
    /// </summary>
    public readonly struct ChainJointLimits
    {
        public readonly double Lower;
        public readonly double Upper;

        /// <summary>The URDF's <c>safety_controller</c> soft limits; equal to the hard limits when it has none.</summary>
        public readonly double SoftLower;
        public readonly double SoftUpper;

        public readonly double Velocity;
        public readonly double Acceleration;
        public readonly double Jerk;
        public readonly double Effort;

        /// <summary>
        /// Franka's position-based velocity limit (URDF <c>position_based_velocity_limits</c>): near a
        /// limit, the velocity toward it may be at most <c>offset + sqrt(2 · deceleration · distance)</c>.
        /// Both 0 when the joint has none.
        /// </summary>
        public readonly double PositionBasedDeceleration;
        public readonly double PositionBasedVelocityOffset;

        public ChainJointLimits(
            double lower, double upper, double softLower, double softUpper,
            double velocity, double acceleration, double jerk, double effort,
            double positionBasedDeceleration = 0, double positionBasedVelocityOffset = 0)
        {
            Lower = lower;
            Upper = upper;
            SoftLower = softLower;
            SoftUpper = softUpper;
            Velocity = velocity;
            Acceleration = acceleration;
            Jerk = jerk;
            Effort = effort;
            PositionBasedDeceleration = positionBasedDeceleration;
            PositionBasedVelocityOffset = positionBasedVelocityOffset;
        }

        public bool HasPositionBasedVelocityLimit => PositionBasedDeceleration > 0;
    }

    /// <summary>
    /// One moving joint of an arm: which motor id it answers to, where it sits on its parent link,
    /// and how it moves. Its frame is <c>parent · Origin · motion(Axis, value)</c>.
    /// </summary>
    public readonly struct ChainJoint
    {
        public readonly byte MotorId;

        /// <summary>The robot description's own name for it (the URDF joint name).</summary>
        public readonly string Name;

        public readonly ChainJointType Type;

        /// <summary>From the parent link's frame to this joint's frame at zero.</summary>
        public readonly RigidTransform Origin;

        /// <summary>The unit axis, in the joint's own frame.</summary>
        public readonly double AxisX;
        public readonly double AxisY;
        public readonly double AxisZ;

        public readonly ChainJointLimits Limits;

        public ChainJoint(
            byte motorId, string name, ChainJointType type, RigidTransform origin,
            double axisX, double axisY, double axisZ, ChainJointLimits limits)
        {
            MotorId = motorId;
            Name = name;
            Type = type;
            Origin = origin;
            AxisX = axisX;
            AxisY = axisY;
            AxisZ = axisZ;
            Limits = limits;
        }
    }

    /// <summary>
    /// An arm's gripper. On the wire it is a 0..1 opening fraction, as every gripper is
    /// (docs/adr/0011): 0 closed, 1 at <see cref="FingerTravel"/> per finger.
    /// </summary>
    public readonly struct ChainGripper
    {
        public readonly byte MotorId;

        /// <summary>The robot description's name for the driven finger joint.</summary>
        public readonly string Name;

        /// <summary>Each finger's travel from closed to fully open, metres (0.04 on the Franka Hand).</summary>
        public readonly double FingerTravel;

        /// <summary>Each finger's top speed, metres per second.</summary>
        public readonly double Speed;

        /// <summary>Whether a second finger mirrors the first (the URDF's <c>mimic</c>), so the opening is twice the travel.</summary>
        public readonly bool SecondFingerMimics;

        public ChainGripper(byte motorId, string name, double fingerTravel, double speed, bool secondFingerMimics)
        {
            MotorId = motorId;
            Name = name;
            FingerTravel = fingerTravel;
            Speed = speed;
            SecondFingerMimics = secondFingerMimics;
        }
    }

    /// <summary>
    /// One arm of a <see cref="SerialChainProfile"/>: where its base sits on the robot, its moving
    /// joints from base to tip, its tool frame and its gripper.
    /// </summary>
    public sealed class ChainArm
    {
        public string Name { get; }

        /// <summary>From the robot's root frame to the arm's base.</summary>
        public RigidTransform Mount { get; }

        /// <summary>The moving joints, base to tip.</summary>
        public ChainJoint[] Joints { get; }

        /// <summary>From the last joint's frame to the tool centre point.</summary>
        public RigidTransform Tool { get; }

        public bool HasGripper { get; }

        /// <summary>Meaningful only when <see cref="HasGripper"/> is true.</summary>
        public ChainGripper Gripper { get; }

        public ChainArm(string name, RigidTransform mount, ChainJoint[] joints, RigidTransform tool, ChainGripper? gripper)
        {
            Name = name;
            Mount = mount;
            Joints = joints ?? throw new ArgumentNullException(nameof(joints));
            Tool = tool;
            HasGripper = gripper.HasValue;
            Gripper = gripper ?? default;
        }

        /// <summary>The arm's joints plus its gripper: how many motor ids it answers to.</summary>
        public int MotorCount => Joints.Length + (HasGripper ? 1 : 0);
    }

    /// <summary>
    /// A robot made of one or more serial-chain arms (docs/adr/0026): any number of revolute or
    /// prismatic joints with arbitrary axes and origins, each arm mounted anywhere on the robot.
    /// <see cref="RobotArmProfile"/>, the planar profile of ADR 0011, describes the JetRover and
    /// stays as it is; this describes what that one cannot, such as the FR3 Duo's two seven-joint
    /// arms.
    ///
    /// <para>It is built from <see cref="SerialChainProfileDocument"/>, the file form, which is
    /// generated from the robot's URDF (<c>Teleop.Eval import-urdf</c>) and never edited by hand.</para>
    /// </summary>
    public sealed class SerialChainProfile
    {
        /// <summary>The <c>kind</c> a profile file carries for this type. A file without one is a planar <see cref="RobotArmProfile"/>.</summary>
        public const string Kind = "serial-chain";

        public string Name { get; }
        public ChainArm[] Arms { get; }

        /// <summary>The robot description this was generated from: its path when imported, and its sha256.</summary>
        public string SourceUrdf { get; }
        public string SourceSha256 { get; }

        /// <summary>Where the description came from upstream (repository and commit), as the importer was told.</summary>
        public string Upstream { get; }

        public SerialChainProfile(string name, ChainArm[] arms, string sourceUrdf = "", string sourceSha256 = "", string upstream = "")
        {
            Name = name;
            Arms = arms ?? throw new ArgumentNullException(nameof(arms));
            SourceUrdf = sourceUrdf;
            SourceSha256 = sourceSha256;
            Upstream = upstream;
        }

        /// <summary>Every motor id across every arm, grippers included.</summary>
        public int MotorCount
        {
            get
            {
                int n = 0;
                foreach (ChainArm arm in Arms) n += arm.MotorCount;
                return n;
            }
        }

        /// <summary>
        /// Which arm, and which joint of it, answers to <paramref name="motorId"/>. <paramref name="jointIndex"/>
        /// is -1 for the arm's gripper. False when no arm has the id.
        /// </summary>
        public bool TryFindMotor(byte motorId, out int armIndex, out int jointIndex)
        {
            for (int a = 0; a < Arms.Length; a++)
            {
                ChainArm arm = Arms[a];
                for (int j = 0; j < arm.Joints.Length; j++)
                {
                    if (arm.Joints[j].MotorId == motorId)
                    {
                        armIndex = a;
                        jointIndex = j;
                        return true;
                    }
                }

                if (arm.HasGripper && arm.Gripper.MotorId == motorId)
                {
                    armIndex = a;
                    jointIndex = -1;
                    return true;
                }
            }

            armIndex = -1;
            jointIndex = -1;
            return false;
        }

        /// <summary>
        /// Checks internal consistency and returns null when valid, otherwise a reason. Loaders fail loudly
        /// on a non-null result, because a malformed profile routes angles to the wrong motors.
        /// </summary>
        public string? Validate()
        {
            if (string.IsNullOrWhiteSpace(Name)) return "Profile name must not be empty.";
            if (Arms.Length == 0) return "A profile needs at least one arm.";

            var armNames = new HashSet<string>(StringComparer.Ordinal);
            var motorIds = new HashSet<byte>();

            foreach (ChainArm arm in Arms)
            {
                if (string.IsNullOrWhiteSpace(arm.Name)) return "Every arm needs a name.";
                if (!armNames.Add(arm.Name)) return $"Arm name '{arm.Name}' is used twice.";
                if (arm.Joints.Length == 0) return $"Arm '{arm.Name}' has no joints.";

                string? frame = CheckFrame(arm.Mount, $"arm '{arm.Name}' mount") ?? CheckFrame(arm.Tool, $"arm '{arm.Name}' tool");
                if (frame != null) return frame;

                foreach (ChainJoint joint in arm.Joints)
                {
                    string where = $"arm '{arm.Name}' joint '{joint.Name}' (motor {joint.MotorId})";
                    if (!motorIds.Add(joint.MotorId)) return $"Motor id {joint.MotorId} is used more than once ({where}).";

                    string? origin = CheckFrame(joint.Origin, where + " origin");
                    if (origin != null) return origin;

                    double axisNorm = Math.Sqrt(joint.AxisX * joint.AxisX + joint.AxisY * joint.AxisY + joint.AxisZ * joint.AxisZ);
                    if (!(Math.Abs(axisNorm - 1.0) < UnitTolerance)) return $"{where}: the axis is not a unit vector (norm {axisNorm}).";

                    ChainJointLimits l = joint.Limits;
                    if (!(l.Lower < l.Upper)) return $"{where}: the lower limit is not below the upper.";
                    if (!(l.Lower <= l.SoftLower && l.SoftLower < l.SoftUpper && l.SoftUpper <= l.Upper)) return $"{where}: the soft limits are not inside the hard limits.";
                    if (!(l.Velocity > 0 && l.Acceleration > 0 && l.Jerk > 0)) return $"{where}: velocity, acceleration and jerk limits must be positive.";
                    if (!(l.PositionBasedDeceleration >= 0 && l.PositionBasedVelocityOffset >= 0)) return $"{where}: the position-based velocity limit must not be negative.";
                }

                if (arm.HasGripper)
                {
                    ChainGripper g = arm.Gripper;
                    if (!motorIds.Add(g.MotorId)) return $"Motor id {g.MotorId} is used more than once (arm '{arm.Name}' gripper).";
                    if (!(g.FingerTravel > 0 && g.Speed > 0)) return $"Arm '{arm.Name}' gripper: travel and speed must be positive.";
                }
            }

            return null;
        }

        private const double UnitTolerance = 1e-9;

        private static string? CheckFrame(in RigidTransform t, string where)
        {
            double n = Math.Sqrt(t.Qw * t.Qw + t.Qx * t.Qx + t.Qy * t.Qy + t.Qz * t.Qz);
            if (!(Math.Abs(n - 1.0) < UnitTolerance)) return $"{where}: the rotation is not a unit quaternion (norm {n}).";
            if (!(IsFinite(t.X) && IsFinite(t.Y) && IsFinite(t.Z))) return $"{where}: the translation is not finite.";
            return null;
        }

        private static bool IsFinite(double v) => !double.IsNaN(v) && !double.IsInfinity(v);
    }
}
