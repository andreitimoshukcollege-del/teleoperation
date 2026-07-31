using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Pipeline
{
    /// <summary>
    /// The robot's downlink reply to one uplink command: "here is my state, as of when I
    /// received command <see cref="Sequence"/> and replied." Distinct from
    /// <see cref="Stamped{Pose}"/>, which carries only a capture stamp and a value — this needs
    /// two more fields <c>docs/adr/0002-latency-trace.md</c>'s round-trip correlation requires:
    /// the echoed <see cref="Sequence"/> (so <see cref="OperatorEndpoint"/> can find the
    /// in-flight <c>LatencyTrace</c> it belongs to) and both raw robot-domain timestamps a
    /// <c>ClockSync</c> four-timestamp estimate needs (<see cref="RobotRecvTicks"/>,
    /// <see cref="DownlinkSendTicks"/> — uncorrected; domain conversion happens exactly once,
    /// operator-side, per that ADR).
    /// </summary>
    public readonly struct RobotStateFrame
    {
        /// <summary>The uplink <c>CommandFrame.Sequence</c> this reply is for.</summary>
        public readonly uint Sequence;

        /// <summary>t1: when the uplink command arrived at the robot, robot-domain, uncorrected.</summary>
        public readonly long RobotRecvTicks;

        /// <summary>t2: when this reply left the robot, robot-domain, uncorrected.</summary>
        public readonly long DownlinkSendTicks;

        /// <summary>The plant's state at the moment this reply was sent.</summary>
        public readonly Pose Pose;

        public RobotStateFrame(uint sequence, long robotRecvTicks, long downlinkSendTicks, Pose pose)
        {
            Sequence = sequence;
            RobotRecvTicks = robotRecvTicks;
            DownlinkSendTicks = downlinkSendTicks;
            Pose = pose;
        }

        public override string ToString() =>
            $"RobotStateFrame(seq={Sequence}, robotRecv={RobotRecvTicks}, " +
            $"downlinkSend={DownlinkSendTicks}, {Pose})";
    }
}
