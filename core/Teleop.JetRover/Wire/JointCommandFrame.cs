namespace Teleop.JetRover.Wire
{
    /// <summary>
    /// One operator command carrying already-computed joint angles, as it crosses the wire to
    /// <c>Teleop.RobotHost</c>'s JetRover-specific joint listener
    /// (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md). Deliberately not
    /// <c>Teleop.Core.Types.CommandFrame</c>: that type carries a Cartesian <c>Pose</c> and is
    /// consumed generically by any <c>IRobotPlant</c> via <c>ICommandCodec</c>/<c>RobotEndpoint</c>;
    /// this type exists specifically so the Jetson never has to run
    /// <c>FourDofArmKinematics.TryInverse</c> for this command path -- the angles here are
    /// already the answer.
    ///
    /// Uplink-only: there is no matching downlink frame. A caller sending these still gets robot
    /// state feedback through its own separate, unmodified Cartesian
    /// <c>OperatorEndpoint</c>/<c>CommandFrame</c> connection (used for
    /// prediction/reconciliation/<c>ClockSync</c> regardless of how the actual command was
    /// shaped) -- see the ADR for why a second reply channel isn't needed.
    /// </summary>
    public readonly struct JointCommandFrame
    {
        /// <summary>Monotonically increasing per sender, wrapping at <c>uint.MaxValue</c> -- same convention as <c>CommandFrame.Sequence</c>, though this channel has no downlink to acknowledge against.</summary>
        public readonly uint Sequence;

        /// <summary><c>t_capture</c> on the sender's <c>ITimeAuthority</c> timebase -- same role as <c>CommandFrame.CaptureTicks</c>, used by <c>JetRoverPlant</c>'s stale/duplicate rejection.</summary>
        public readonly long CaptureTicks;

        /// <summary>Base-yaw joint angle, radians -- already-computed output of <c>FourDofArmKinematics.TryInverse</c>.</summary>
        public readonly float BaseYaw;

        /// <summary>Lower-arm (shoulder) pitch, radians.</summary>
        public readonly float LowerPitch;

        /// <summary>Middle-arm (elbow) pitch, radians.</summary>
        public readonly float MiddlePitch;

        /// <summary>Upper-arm (wrist) pitch, radians -- from <c>FourDofArmKinematics.InverseUpperPitch</c>, not part of the position solve.</summary>
        public readonly float UpperPitch;

        /// <summary>Gripper command, 0 = fully open, 1 = fully closed -- same normalization as <c>CommandFrame.Gripper</c>.</summary>
        public readonly float Gripper;

        public JointCommandFrame(
            uint sequence,
            long captureTicks,
            float baseYaw,
            float lowerPitch,
            float middlePitch,
            float upperPitch,
            float gripper)
        {
            Sequence = sequence;
            CaptureTicks = captureTicks;
            BaseYaw = baseYaw;
            LowerPitch = lowerPitch;
            MiddlePitch = middlePitch;
            UpperPitch = upperPitch;
            Gripper = gripper;
        }

        public override string ToString() =>
            $"JointCommandFrame(seq={Sequence}, t={CaptureTicks}, " +
            $"base={BaseYaw:F3}, lower={LowerPitch:F3}, middle={MiddlePitch:F3}, upper={UpperPitch:F3}, " +
            $"grip={Gripper:F3})";
    }
}
