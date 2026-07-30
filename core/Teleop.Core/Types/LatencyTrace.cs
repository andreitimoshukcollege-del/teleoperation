// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// The docs/metrics.md §1 timestamp table for one command's round trip: operator capture
    /// through robot receipt (uplink) and robot's resulting state through operator playout and
    /// render (downlink). See docs/adr/0002-latency-trace.md for the design rationale.
    ///
    /// Correlated by <c>Sequence</c>, matching <see cref="CommandFrame.Sequence"/> — not by any
    /// capture stamp, which is per-sender-clock and does not survive duplication or reordering.
    /// The downlink half of a round trip is expected to echo the uplink command's
    /// <c>Sequence</c> (the same pattern <see cref="CommandFrame.AckSequence"/> already uses),
    /// which is how a trace opened on the uplink is found again when the downlink arrives.
    ///
    /// Every tick field is on the **operator's** <c>ITimeAuthority</c> timebase, because
    /// motion-to-photon is defined at the operator's display. Stamps produced on the robot
    /// (<see cref="TryGetRobotRecvTicks"/>, <see cref="TryGetDownlinkSendTicks"/>) must already
    /// be offset-corrected into operator time by <c>ClockSync</c> before being stored here —
    /// this type does not perform that conversion. <see cref="TryGetClockOffsetTicks"/> and
    /// <see cref="TryGetClockOffsetUncertaintyTicks"/> carry the offset that correction used and
    /// its uncertainty, because one-way-delay precision is floored by sync uncertainty and
    /// reporting tighter than that is false precision.
    ///
    /// Most fields are unknown for most of a trace's life — some (render, photon) are unknown
    /// for the entire life of a headless run, because only a host with a compositor can produce
    /// them. There is deliberately no plain getter: every field is read through a
    /// <c>TryGetXTicks</c> that reports whether it was ever set, so an unset stamp cannot be
    /// silently used as if it were zero.
    ///
    /// Struct, allocation-free. Built by chaining <c>WithXTicks</c> calls from
    /// <see cref="ForSequence"/>; there is no other way to construct a valid instance.
    /// </summary>
    public readonly struct LatencyTrace
    {
        /// <summary>
        /// Sentinel for a field that has never been set. Never returned to a caller as a tick
        /// value — <c>TryGetXTicks</c> methods return <c>false</c> instead. Not zero, because
        /// zero is a legitimate tick value early in a replay.
        /// </summary>
        public const long Unset = long.MinValue;

        /// <summary>
        /// Correlation key, matching <see cref="CommandFrame.Sequence"/> of the uplink command
        /// this trace is for.
        /// </summary>
        public readonly uint Sequence;

        private readonly long _captureTicks;
        private readonly long _uplinkSendTicks;
        private readonly long _robotRecvTicks;
        private readonly long _downlinkSendTicks;
        private readonly long _operatorRecvTicks;
        private readonly long _playoutTicks;
        private readonly long _renderTicks;
        private readonly long _photonTicks;
        private readonly long _clockOffsetTicks;
        private readonly long _clockOffsetUncertaintyTicks;

        private LatencyTrace(
            uint sequence,
            long captureTicks,
            long uplinkSendTicks,
            long robotRecvTicks,
            long downlinkSendTicks,
            long operatorRecvTicks,
            long playoutTicks,
            long renderTicks,
            long photonTicks,
            long clockOffsetTicks,
            long clockOffsetUncertaintyTicks)
        {
            Sequence = sequence;
            _captureTicks = captureTicks;
            _uplinkSendTicks = uplinkSendTicks;
            _robotRecvTicks = robotRecvTicks;
            _downlinkSendTicks = downlinkSendTicks;
            _operatorRecvTicks = operatorRecvTicks;
            _playoutTicks = playoutTicks;
            _renderTicks = renderTicks;
            _photonTicks = photonTicks;
            _clockOffsetTicks = clockOffsetTicks;
            _clockOffsetUncertaintyTicks = clockOffsetUncertaintyTicks;
        }

        /// <summary>
        /// The all-unset starting value for the round trip identified by
        /// <paramref name="sequence"/>. The only way to obtain a valid <see cref="LatencyTrace"/>
        /// — a default-constructed one carries no <see cref="Sequence"/> and cannot be
        /// correlated to anything.
        /// </summary>
        public static LatencyTrace ForSequence(uint sequence) =>
            new LatencyTrace(
                sequence,
                Unset, Unset, Unset, Unset, Unset, Unset, Unset, Unset, Unset, Unset);

        /// <summary><c>t_capture</c>: operator input device sampled, operator domain natively.</summary>
        public LatencyTrace WithCaptureTicks(long ticks) =>
            new LatencyTrace(
                Sequence, ticks, _uplinkSendTicks, _robotRecvTicks, _downlinkSendTicks,
                _operatorRecvTicks, _playoutTicks, _renderTicks, _photonTicks,
                _clockOffsetTicks, _clockOffsetUncertaintyTicks);

        /// <summary><c>t_send</c>, uplink: command handed to the transport, operator domain natively.</summary>
        public LatencyTrace WithUplinkSendTicks(long ticks) =>
            new LatencyTrace(
                Sequence, _captureTicks, ticks, _robotRecvTicks, _downlinkSendTicks,
                _operatorRecvTicks, _playoutTicks, _renderTicks, _photonTicks,
                _clockOffsetTicks, _clockOffsetUncertaintyTicks);

        /// <summary>
        /// <c>t_recv</c>, uplink: command arrived at the robot. Caller must offset-correct this
        /// from the robot's timebase into operator time before calling — see
        /// <see cref="WithClockSync"/>.
        /// </summary>
        public LatencyTrace WithRobotRecvTicks(long ticks) =>
            new LatencyTrace(
                Sequence, _captureTicks, _uplinkSendTicks, ticks, _downlinkSendTicks,
                _operatorRecvTicks, _playoutTicks, _renderTicks, _photonTicks,
                _clockOffsetTicks, _clockOffsetUncertaintyTicks);

        /// <summary>
        /// <c>t_send</c>, downlink: robot's resulting state handed to the transport. Caller must
        /// offset-correct this from the robot's timebase into operator time before calling —
        /// see <see cref="WithClockSync"/>.
        /// </summary>
        public LatencyTrace WithDownlinkSendTicks(long ticks) =>
            new LatencyTrace(
                Sequence, _captureTicks, _uplinkSendTicks, _robotRecvTicks, ticks,
                _operatorRecvTicks, _playoutTicks, _renderTicks, _photonTicks,
                _clockOffsetTicks, _clockOffsetUncertaintyTicks);

        /// <summary><c>t_recv</c>, downlink: robot's state arrived at the operator, operator domain natively.</summary>
        public LatencyTrace WithOperatorRecvTicks(long ticks) =>
            new LatencyTrace(
                Sequence, _captureTicks, _uplinkSendTicks, _robotRecvTicks, _downlinkSendTicks,
                ticks, _playoutTicks, _renderTicks, _photonTicks,
                _clockOffsetTicks, _clockOffsetUncertaintyTicks);

        /// <summary><c>t_playout</c>: consumed by the playout policy, operator domain natively.</summary>
        public LatencyTrace WithPlayoutTicks(long ticks) =>
            new LatencyTrace(
                Sequence, _captureTicks, _uplinkSendTicks, _robotRecvTicks, _downlinkSendTicks,
                _operatorRecvTicks, ticks, _renderTicks, _photonTicks,
                _clockOffsetTicks, _clockOffsetUncertaintyTicks);

        /// <summary>
        /// <c>t_render</c>: frame submitted to the compositor. Only the host can produce this —
        /// Core has no compositor and no frame loop — so this field is legitimately never set on
        /// a headless run.
        /// </summary>
        public LatencyTrace WithRenderTicks(long ticks) =>
            new LatencyTrace(
                Sequence, _captureTicks, _uplinkSendTicks, _robotRecvTicks, _downlinkSendTicks,
                _operatorRecvTicks, _playoutTicks, ticks, _photonTicks,
                _clockOffsetTicks, _clockOffsetUncertaintyTicks);

        /// <summary>
        /// <c>t_photon</c>: <c>t_render + DisplayOffset</c>, estimated light emission. Host-only,
        /// for the same reason as <see cref="WithRenderTicks"/>.
        /// </summary>
        public LatencyTrace WithPhotonTicks(long ticks) =>
            new LatencyTrace(
                Sequence, _captureTicks, _uplinkSendTicks, _robotRecvTicks, _downlinkSendTicks,
                _operatorRecvTicks, _playoutTicks, _renderTicks, ticks,
                _clockOffsetTicks, _clockOffsetUncertaintyTicks);

        /// <summary>
        /// The <c>ClockSync</c> offset (operator domain minus robot domain, in ticks) used to
        /// convert this trace's robot-origin stamps, and its one-sigma uncertainty. Set both
        /// together: an offset without its uncertainty invites reporting precision the sync
        /// cannot actually support.
        /// </summary>
        public LatencyTrace WithClockSync(long offsetTicks, long offsetUncertaintyTicks) =>
            new LatencyTrace(
                Sequence, _captureTicks, _uplinkSendTicks, _robotRecvTicks, _downlinkSendTicks,
                _operatorRecvTicks, _playoutTicks, _renderTicks, _photonTicks,
                offsetTicks, offsetUncertaintyTicks);

        /// <summary>Operator input device sampled, in operator-domain ticks.</summary>
        public bool TryGetCaptureTicks(out long ticks) => TryGet(_captureTicks, out ticks);

        /// <summary>Uplink command handed to the transport, in operator-domain ticks.</summary>
        public bool TryGetUplinkSendTicks(out long ticks) => TryGet(_uplinkSendTicks, out ticks);

        /// <summary>Uplink command arrived at the robot, already converted to operator-domain ticks.</summary>
        public bool TryGetRobotRecvTicks(out long ticks) => TryGet(_robotRecvTicks, out ticks);

        /// <summary>Robot's resulting state handed to the transport, already converted to operator-domain ticks.</summary>
        public bool TryGetDownlinkSendTicks(out long ticks) => TryGet(_downlinkSendTicks, out ticks);

        /// <summary>Robot's state arrived at the operator, in operator-domain ticks.</summary>
        public bool TryGetOperatorRecvTicks(out long ticks) => TryGet(_operatorRecvTicks, out ticks);

        /// <summary>Consumed by the playout policy, in operator-domain ticks.</summary>
        public bool TryGetPlayoutTicks(out long ticks) => TryGet(_playoutTicks, out ticks);

        /// <summary>Frame submitted to the compositor, in operator-domain ticks. Host-only.</summary>
        public bool TryGetRenderTicks(out long ticks) => TryGet(_renderTicks, out ticks);

        /// <summary>Estimated light emission, in operator-domain ticks. Host-only.</summary>
        public bool TryGetPhotonTicks(out long ticks) => TryGet(_photonTicks, out ticks);

        /// <summary>The <c>ClockSync</c> offset applied to this trace's robot-origin stamps, in ticks.</summary>
        public bool TryGetClockOffsetTicks(out long ticks) => TryGet(_clockOffsetTicks, out ticks);

        /// <summary>One-sigma uncertainty of <see cref="TryGetClockOffsetTicks"/>, in ticks.</summary>
        public bool TryGetClockOffsetUncertaintyTicks(out long ticks) =>
            TryGet(_clockOffsetUncertaintyTicks, out ticks);

        private static bool TryGet(long stored, out long ticks)
        {
            ticks = stored;
            return stored != Unset;
        }

        public override string ToString() =>
            $"LatencyTrace(seq={Sequence}, " +
            $"capture={FormatField(_captureTicks)}, uplinkSend={FormatField(_uplinkSendTicks)}, " +
            $"robotRecv={FormatField(_robotRecvTicks)}, downlinkSend={FormatField(_downlinkSendTicks)}, " +
            $"operatorRecv={FormatField(_operatorRecvTicks)}, playout={FormatField(_playoutTicks)}, " +
            $"render={FormatField(_renderTicks)}, photon={FormatField(_photonTicks)}, " +
            $"clockOffset={FormatField(_clockOffsetTicks)}, " +
            $"clockOffsetSigma={FormatField(_clockOffsetUncertaintyTicks)})";

        private static string FormatField(long stored) => stored == Unset ? "unset" : stored.ToString();
    }
}
