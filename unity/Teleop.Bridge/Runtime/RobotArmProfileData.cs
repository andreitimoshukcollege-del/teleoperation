using System;
using Teleop.RobotArm.Types;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Plain <see cref="Serializable"/> mirror of one <see cref="JointHardwareSpec"/> --
    /// <c>JsonUtility</c> can't deserialize <see cref="Nullable{Single}"/> directly, so an
    /// optional angle limit is represented as an explicit has-flag plus a value, rather than
    /// <c>float?</c>.
    /// </summary>
    [Serializable]
    public sealed class JointHardwareSpecData
    {
        public byte MotorId;
        public JointRole Role;
        public int WristIndex;
        public bool HasMinAngleRadians;
        public float MinAngleRadians;
        public bool HasMaxAngleRadians;
        public float MaxAngleRadians;

        /// <summary>See <see cref="Teleop.RobotArm.Types.JointHardwareSpec.ZeroOffsetRadians"/>'s own doc -- 0 (no correction) unless this joint has been calibrated against real hardware.</summary>
        public float ZeroOffsetRadians;
    }

    /// <summary>
    /// Plain <see cref="Serializable"/> mirror of <see cref="RobotArmProfile"/>
    /// (docs/adr/0011-generic-robot-arm-profiles.md) -- <c>JsonUtility</c> can't deserialize that
    /// type directly (a constructor-only readonly struct, and no support for
    /// <see cref="Nullable{Single}"/>), so this exists purely to move the same data through
    /// Unity's JSON path, the same reason <see cref="JetRoverArmConfig"/> itself is a plain
    /// mutable class rather than a reused shared type.
    ///
    /// Loaded via the existing <see cref="ConfigLoader.Load{T}"/> pattern
    /// (<c>ConfigLoader.Load("jetrover_arm_profile", "jetrover_arm_profile.json", new
    /// RobotArmProfileData())</c>) -- a new sibling config file, not a change to
    /// <see cref="JetRoverArmConfig"/> itself, which never held geometry. The field defaults
    /// below are <see cref="RobotArmProfile.JetRoverMeasuredDefault"/>'s exact numbers, so
    /// <see cref="JetRoverOperatorBridge"/>'s behavior is unchanged when no override file exists.
    /// </summary>
    [Serializable]
    public sealed class RobotArmProfileData
    {
        public string Name = "jetrover";
        public bool HasRotatingBase = true;
        public float BaseHeight = 0.035f;
        public float ProximalLinkLength = 0.13f;
        public float DistalLinkLength = 0.13f;
        public int WristJointCount = 1;
        public bool HasGripper = true;
        public bool GripperCanRotate = false;

        public JointHardwareSpecData[] Joints =
        {
            new JointHardwareSpecData { MotorId = 1, Role = JointRole.BaseYaw },
            new JointHardwareSpecData
            {
                MotorId = 2, Role = JointRole.Proximal,
                HasMinAngleRadians = true, MinAngleRadians = -1.8851179f,
                ZeroOffsetRadians = 0.12217305f, // +7 degrees, calibrated 2026-08-17 against real hardware
            },
            new JointHardwareSpecData { MotorId = 3, Role = JointRole.Distal },
            new JointHardwareSpecData { MotorId = 4, Role = JointRole.Wrist, WristIndex = 0 },
            new JointHardwareSpecData { MotorId = 5, Role = JointRole.GripperMain },
        };

        public RobotArmProfile ToProfile()
        {
            var joints = new JointHardwareSpec[Joints.Length];
            for (int i = 0; i < Joints.Length; i++)
            {
                JointHardwareSpecData j = Joints[i];
                joints[i] = new JointHardwareSpec(
                    j.MotorId, j.Role, j.WristIndex,
                    j.HasMinAngleRadians ? j.MinAngleRadians : (float?)null,
                    j.HasMaxAngleRadians ? j.MaxAngleRadians : (float?)null,
                    j.ZeroOffsetRadians);
            }

            return new RobotArmProfile(
                Name, HasRotatingBase, BaseHeight, ProximalLinkLength, DistalLinkLength,
                WristJointCount, HasGripper, GripperCanRotate, joints);
        }
    }
}
