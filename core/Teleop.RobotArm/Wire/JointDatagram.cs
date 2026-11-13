namespace Teleop.RobotArm.Wire
{
    /// <summary>
    /// The joint port's datagram budget (docs/adr/0026 decision 6): every joint command, joint-state frame and
    /// message on the joint port fits in <see cref="MaxBytes"/>, and every receive buffer for it is that size.
    /// One constant, so the codecs' joint limits and the buffers that carry them cannot drift apart.
    ///
    /// <para>256 bytes carries 26 commanded joints (<see cref="JointCommandCodec.MaxJointsPerMessage"/>) and 35
    /// reported ones (<see cref="JointStateCodec.MaxJointsPerMessage"/>): the FR3 Duo's 16 with room to spare,
    /// in one datagram, under any network's MTU. It was 128 until the FR3 Duo: a JetRover datagram is the same
    /// bytes as before, and a receiver built with the old budget refuses a larger datagram rather than
    /// misreading it. The Cartesian path's budget is separate and stays 128.</para>
    /// </summary>
    public static class JointDatagram
    {
        public const int MaxBytes = 256;
    }
}
