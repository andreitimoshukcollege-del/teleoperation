using System;
using System.Numerics;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Gate
{
    /// <summary>
    /// The command gate's example check (docs/adr/0038 N, <c>--command-gate table:&lt;clearanceMm&gt;:&lt;tableTopZ&gt;</c>): holds a
    /// command whose wrist (<see cref="ArmKinematics.Forward"/>, the arm's local frame: Z up from the base-yaw axis) would be
    /// within <c>clearanceMm</c> of a table top at <c>tableTopZ</c> metres, or below it. <b>An example and a test fixture, not a
    /// safety check:</b> it looks at the wrist point alone, not the links, the gripper or the objects. A command that does not
    /// name the base yaw, shoulder and elbow is let through (there is nothing to place).
    /// </summary>
    public sealed class TableClearanceCheck : ICommandCheck
    {
        private readonly RobotArmProfile _profile;
        private readonly byte _yawMotor, _proximalMotor, _distalMotor;
        private readonly double _limitZ;

        public TableClearanceCheck(RobotArmProfile profile, double clearanceMetres, double tableTopZ)
        {
            _profile = profile;
            _yawMotor = Motor(profile, JointRole.BaseYaw);
            _proximalMotor = Motor(profile, JointRole.Proximal);
            _distalMotor = Motor(profile, JointRole.Distal);
            _limitZ = tableTopZ + clearanceMetres;
            Reason = FormattableStringFactory(clearanceMetres);
        }

        /// <summary>What a hold says: <c>table:&lt;clearanceMm&gt;</c>.</summary>
        public string Reason { get; }

        public CommandVerdict Judge(uint sequence, long captureTicks, long nowTicks, ReadOnlySpan<JointTarget> targets, out string reason)
        {
            reason = "";
            float yaw = float.NaN, proximal = float.NaN, distal = float.NaN;
            foreach (JointTarget t in targets)
            {
                if (t.MotorId == _yawMotor) yaw = t.Angle;
                else if (t.MotorId == _proximalMotor) proximal = t.Angle;
                else if (t.MotorId == _distalMotor) distal = t.Angle;
            }

            if (float.IsNaN(yaw) || float.IsNaN(proximal) || float.IsNaN(distal)) return CommandVerdict.Accept;
            Vector3 wrist = ArmKinematics.Forward(_profile, yaw, proximal, distal);
            if (wrist.Z > _limitZ) return CommandVerdict.Accept;
            reason = Reason;
            return CommandVerdict.Hold;
        }

        private static byte Motor(RobotArmProfile profile, JointRole role)
        {
            int i = profile.TryGetJointIndex(role);
            if (i < 0) throw new ArgumentException($"the robot profile has no {role} joint for the table check");
            return profile.Joints[i].MotorId;
        }

        private static string FormattableStringFactory(double clearanceMetres) =>
            "table:" + Math.Round(clearanceMetres * 1000.0, 1).ToString("R", System.Globalization.CultureInfo.InvariantCulture);
    }
}
