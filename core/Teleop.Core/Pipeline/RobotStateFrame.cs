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
    ///
    /// It also carries <see cref="TicksPerSecond"/>, the robot <c>ITimeAuthority</c>'s own tick
    /// rate, which is what makes those two robot-domain stamps interpretable at all: the two
    /// stamps are meaningless numbers without the rate they are counted in, and that rate is a
    /// fact about a remote machine the operator cannot determine from its own clock. Sent on
    /// every reply rather than negotiated once — it is 8 bytes and "fixed for the lifetime of the
    /// instance" per <c>ITimeAuthority</c>, so resending it is free of any staleness concern and
    /// costs no connection-lifecycle state. See
    /// docs/adr/0008-clocksync-cross-rate-normalization.md, which introduced this field after a
    /// real Windows-operator/Jetson-robot pairing (10 MHz vs 1 GHz) inflated every measured RTT
    /// 100x. The uplink <c>CommandFrame</c> deliberately has no counterpart field: conversion is
    /// operator-side only, so the robot never needs the operator's rate.
    /// </summary>
    public readonly struct RobotStateFrame
    {
        /// <summary>The uplink <c>CommandFrame.Sequence</c> this reply is for.</summary>
        public readonly uint Sequence;

        /// <summary>t1: when the uplink command arrived at the robot, robot-domain, uncorrected.</summary>
        public readonly long RobotRecvTicks;

        /// <summary>t2: when this reply left the robot, robot-domain, uncorrected.</summary>
        public readonly long DownlinkSendTicks;

        /// <summary>
        /// The robot <c>ITimeAuthority</c>'s tick rate — the unit <see cref="RobotRecvTicks"/> and
        /// <see cref="DownlinkSendTicks"/> are counted in, without which the operator cannot
        /// compare them against its own.
        /// </summary>
        public readonly long TicksPerSecond;

        /// <summary>The plant's state at the moment this reply was sent.</summary>
        public readonly Pose Pose;

        public RobotStateFrame(
            uint sequence, long robotRecvTicks, long downlinkSendTicks, long ticksPerSecond, Pose pose)
        {
            Sequence = sequence;
            RobotRecvTicks = robotRecvTicks;
            DownlinkSendTicks = downlinkSendTicks;
            TicksPerSecond = ticksPerSecond;
            Pose = pose;
        }

        public override string ToString() =>
            $"RobotStateFrame(seq={Sequence}, robotRecv={RobotRecvTicks}, " +
            $"downlinkSend={DownlinkSendTicks}, ticksPerSecond={TicksPerSecond}, {Pose})";
    }
}
