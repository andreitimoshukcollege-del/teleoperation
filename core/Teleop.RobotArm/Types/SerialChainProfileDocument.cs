using System;

namespace Teleop.RobotArm.Types
{
    /// <summary>
    /// The file form of a <see cref="SerialChainProfile"/> (<c>core/RobotProfiles/*.json</c>,
    /// docs/adr/0026). Plain public fields, named exactly as the JSON names them, so the same file
    /// loads through <c>System.Text.Json</c> in the hosts (with fields included) and through Unity's
    /// <c>JsonUtility</c>. ADR 0011's planar profile needed a separate mirror per host; this one
    /// is shared, and only the reading and writing of the file stays in each host.
    ///
    /// <para><b>Frames are kept as the URDF writes them.</b> Every frame is a list of
    /// <see cref="UrdfOriginDocument"/>s, one per URDF joint it was folded from, with the URDF's own
    /// <c>xyz</c>/<c>rpy</c> numbers. The importer does no trigonometry, so a re-import is the same
    /// bytes on every platform, and each number can be checked against the URDF by eye.
    /// <see cref="ToProfile"/> composes them.</para>
    /// </summary>
    [Serializable]
    public sealed class SerialChainProfileDocument
    {
        public string name = "";
        public string kind = SerialChainProfile.Kind;
        public ProfileSourceDocument source = new ProfileSourceDocument();
        public ChainArmDocument[] arms = Array.Empty<ChainArmDocument>();

        /// <summary>
        /// Builds and validates the profile.
        /// </summary>
        /// <exception cref="FormatException">The document is not a valid serial-chain profile.</exception>
        public SerialChainProfile ToProfile()
        {
            if (kind != SerialChainProfile.Kind)
            {
                throw new FormatException($"profile '{name}' has kind '{kind}', not '{SerialChainProfile.Kind}'");
            }

            var built = new ChainArm[arms.Length];
            for (int a = 0; a < arms.Length; a++)
            {
                ChainArmDocument arm = arms[a] ?? throw new FormatException($"profile '{name}': arm {a} is missing");
                var joints = new ChainJoint[arm.joints.Length];
                for (int j = 0; j < joints.Length; j++)
                {
                    ChainJointDocument d = arm.joints[j];
                    string where = $"arm '{arm.name}' joint '{d.name}'";
                    joints[j] = new ChainJoint(
                        d.motorId, d.name, ParseType(d.type, where), Compose(d.origin, where + " origin"),
                        Component(d.axis, 0, where + " axis"), Component(d.axis, 1, where + " axis"), Component(d.axis, 2, where + " axis"),
                        new ChainJointLimits(
                            d.lower, d.upper, d.softLower, d.softUpper,
                            d.velocity, d.acceleration, d.jerk, d.effort,
                            d.positionBasedDeceleration, d.positionBasedVelocityOffset));
                }

                ChainGripper? gripper = arm.hasGripper
                    ? new ChainGripper(arm.gripper.motorId, arm.gripper.name, arm.gripper.fingerTravel, arm.gripper.speed, arm.gripper.secondFingerMimics)
                    : (ChainGripper?)null;

                built[a] = new ChainArm(
                    arm.name, Compose(arm.mount, $"arm '{arm.name}' mount"), joints,
                    Compose(arm.tool, $"arm '{arm.name}' tool"), gripper);
            }

            var profile = new SerialChainProfile(name, built, source.urdf, source.sha256, source.upstream);
            string? error = profile.Validate();
            if (error != null)
            {
                throw new FormatException($"profile '{name}' is invalid: {error}");
            }

            return profile;
        }

        /// <summary>The composition of a frame's URDF origins, in order: <c>origins[0] · origins[1] · ...</c>.</summary>
        public static RigidTransform Compose(UrdfOriginDocument[] origins, string where)
        {
            RigidTransform result = RigidTransform.Identity;
            if (origins == null) return result;

            foreach (UrdfOriginDocument o in origins)
            {
                result = result.Compose(RigidTransform.FromXyzRpy(
                    Component(o.xyz, 0, where), Component(o.xyz, 1, where), Component(o.xyz, 2, where),
                    Component(o.rpy, 0, where), Component(o.rpy, 1, where), Component(o.rpy, 2, where)));
            }

            return result;
        }

        private static double Component(double[] v, int i, string where)
        {
            if (v == null || v.Length != 3) throw new FormatException($"{where}: needs exactly three numbers");
            return v[i];
        }

        private static ChainJointType ParseType(string type, string where)
        {
            switch (type)
            {
                case "revolute": return ChainJointType.Revolute;
                case "prismatic": return ChainJointType.Prismatic;
                default: throw new FormatException($"{where}: type '{type}' is not 'revolute' or 'prismatic'");
            }
        }
    }

    /// <summary>Where a generated profile came from.</summary>
    [Serializable]
    public sealed class ProfileSourceDocument
    {
        /// <summary>The URDF's path, as given to the importer (repository-relative by convention).</summary>
        public string urdf = "";
        public string sha256 = "";

        /// <summary>The upstream repository and commit the URDF was expanded from.</summary>
        public string upstream = "";
    }

    /// <summary>One URDF <c>&lt;origin&gt;</c>, with the name of the URDF joint it belongs to.</summary>
    [Serializable]
    public sealed class UrdfOriginDocument
    {
        public string joint = "";
        public double[] xyz = { 0, 0, 0 };
        public double[] rpy = { 0, 0, 0 };
    }

    [Serializable]
    public sealed class ChainArmDocument
    {
        public string name = "";

        /// <summary>The fixed URDF joints from the robot's root link to the arm's first moving joint.</summary>
        public UrdfOriginDocument[] mount = Array.Empty<UrdfOriginDocument>();

        public ChainJointDocument[] joints = Array.Empty<ChainJointDocument>();

        /// <summary>The fixed URDF joints from the last moving joint to the tool centre point.</summary>
        public UrdfOriginDocument[] tool = Array.Empty<UrdfOriginDocument>();

        public bool hasGripper;
        public ChainGripperDocument gripper = new ChainGripperDocument();
    }

    [Serializable]
    public sealed class ChainJointDocument
    {
        public byte motorId;
        public string name = "";
        public string type = "revolute";

        /// <summary>Any fixed URDF joints between the previous moving joint and this one, then this joint's own origin.</summary>
        public UrdfOriginDocument[] origin = Array.Empty<UrdfOriginDocument>();

        public double[] axis = { 0, 0, 1 };
        public double lower;
        public double upper;
        public double softLower;
        public double softUpper;
        public double velocity;
        public double acceleration;
        public double jerk;
        public double effort;
        public double positionBasedDeceleration;
        public double positionBasedVelocityOffset;
    }

    [Serializable]
    public sealed class ChainGripperDocument
    {
        public byte motorId;
        public string name = "";
        public double fingerTravel;
        public double speed;
        public bool secondFingerMimics;
    }
}
