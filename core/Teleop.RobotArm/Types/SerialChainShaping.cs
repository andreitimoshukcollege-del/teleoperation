using System.Collections.Generic;
using Teleop.Core.Types;

namespace Teleop.RobotArm.Types
{
    /// <summary>
    /// Builds the trajectory shaper's limits (Core's <see cref="TrajectoryShaperConfig"/>, docs/adr/0027
    /// decision 1) from a <see cref="SerialChainProfile"/>. Core cannot reference the profile, so this is where
    /// the two meet. Every host that shapes or predicts a serial-chain arm (the RobotHost backend, Eval's
    /// bench, the headset) builds its shaper here, so they all shape alike.
    /// </summary>
    public static class SerialChainShaping
    {
        /// <summary>A finger target nearer than this to the last move's (a 0..1 fraction, about 1.6 mm of a Franka Hand's 80 mm opening) starts no move.</summary>
        public const double DefaultFingerDeadband = 0.02;

        /// <summary>
        /// Every arm joint and every gripper of <paramref name="profile"/>, in profile order. A finger's speed in
        /// fractions per second is its travel speed over its travel. The scales and the step take
        /// <see cref="TrajectoryShaperConfig"/>'s defaults unless given.
        /// </summary>
        public static TrajectoryShaperConfig ForProfile(
            SerialChainProfile profile,
            double fingerDeadband = DefaultFingerDeadband,
            double stepSeconds = TrajectoryShaperConfig.DefaultStepSeconds,
            double velocityScale = TrajectoryShaperConfig.DefaultVelocityScale,
            double accelerationScale = TrajectoryShaperConfig.DefaultAccelerationScale,
            double jerkScale = TrajectoryShaperConfig.DefaultJerkScale,
            double brakingScale = TrajectoryShaperConfig.DefaultBrakingScale,
            double settleSeconds = TrajectoryShaperConfig.DefaultSettleSeconds,
            double positionMargin = TrajectoryShaperConfig.DefaultPositionMargin)
        {
            var limits = new List<ShapedJointLimits>(profile.MotorCount);
            foreach (ChainArm arm in profile.Arms)
            {
                foreach (ChainJoint joint in arm.Joints)
                {
                    ChainJointLimits l = joint.Limits;
                    limits.Add(ShapedJointLimits.Joint(
                        joint.MotorId, l.Lower, l.Upper, l.SoftLower, l.SoftUpper,
                        l.Velocity, l.Acceleration, l.Jerk,
                        l.PositionBasedDeceleration, l.PositionBasedVelocityOffset));
                }

                if (arm.HasGripper)
                {
                    limits.Add(ShapedJointLimits.Finger(arm.Gripper.MotorId, arm.Gripper.Speed / arm.Gripper.FingerTravel, fingerDeadband));
                }
            }

            return new TrajectoryShaperConfig(
                limits.ToArray(), stepSeconds, velocityScale, accelerationScale, jerkScale, brakingScale, settleSeconds, positionMargin);
        }

        /// <summary>The motor ids of every gripper in <paramref name="profile"/>: what a predictor without a finger model treats as instant.</summary>
        public static byte[] GripperMotorIds(SerialChainProfile profile)
        {
            var ids = new List<byte>();
            foreach (ChainArm arm in profile.Arms)
            {
                if (arm.HasGripper) ids.Add(arm.Gripper.MotorId);
            }

            return ids.ToArray();
        }
    }
}
