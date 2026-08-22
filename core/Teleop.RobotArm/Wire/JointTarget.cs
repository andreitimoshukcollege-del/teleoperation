namespace Teleop.RobotArm.Wire
{
    /// <summary>
    /// One joint's absolute command: which motor, what angle, how fast to move there. This is the
    /// generic unit both wire hops carry (docs/adr/0011-generic-robot-arm-profiles.md) -- Unity's
    /// <see cref="JointCommandCodec"/> (this project) and <c>Teleop.RobotHost.Relay.RelayProtocol</c>
    /// (a separate hop) both encode arrays of this struct, so a robot with any profile-described
    /// joint count moves through one shared shape instead of a fixed named-field struct sized for
    /// one specific robot.
    ///
    /// <b><see cref="Angle"/>'s unit depends on which hop is carrying it</b>, documented at each
    /// codec: radians on the Unity-&gt;<c>Teleop.RobotHost</c> hop (Core's convention), pulse
    /// (0-1000, hardware units) on the <c>Teleop.RobotHost</c>-&gt;ROS hop (continuing
    /// docs/adr/0010's reasoning for avoiding a duplicated <c>PulsePerRadian</c>/<c>ZeroPulse</c>
    /// conversion on the Python side). Same for <see cref="Speed"/> (radians/second vs.
    /// pulses/second).
    /// </summary>
    public readonly struct JointTarget
    {
        public readonly byte MotorId;
        public readonly float Angle;
        public readonly float Speed;

        public JointTarget(byte motorId, float angle, float speed)
        {
            MotorId = motorId;
            Angle = angle;
            Speed = speed;
        }
    }
}
