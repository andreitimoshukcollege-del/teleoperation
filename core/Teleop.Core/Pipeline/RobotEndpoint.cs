using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Pipeline
{
    /// <summary>
    /// The robot side of a zero-mitigation loopback: apply received commands to the plant,
    /// step it, and echo the plant's current state back for every uplink datagram received.
    /// Composition only, per the same "wiring diagram, expressed in code" reasoning as
    /// <see cref="OperatorEndpoint"/>.
    ///
    /// No <c>ClockSync</c> or <c>IMetricSink</c> here: per docs/adr/0002-latency-trace.md,
    /// clock-domain conversion and latency reporting happen exactly once, operator-side. A
    /// second <c>ClockSync</c> instance robot-side would contradict "conversion happens once,
    /// at the point <c>ClockSync</c> has the sample."
    ///
    /// <see cref="ITimeAuthority"/> is used only for construction-time context (this endpoint
    /// does not currently read <c>TicksPerSecond</c>, but takes the same authority the plant and
    /// caller share, for symmetry with <see cref="OperatorEndpoint"/> and in case a future
    /// change needs it) -- time otherwise arrives as an explicit <c>nowTicks</c> parameter.
    ///
    /// Allocation-free per call: the send/receive buffers and the per-step scratch buffer of
    /// received sequences are all preallocated in the constructor.
    /// </summary>
    public sealed class RobotEndpoint
    {
        private readonly IRobotPlant _plant;
        private readonly ICommandCodec _commandCodec;
        private readonly RobotStateFrameCodec _stateCodec;
        private readonly ITransport _uplinkTransport;
        private readonly ITransport _downlinkTransport;

        private readonly byte[] _sendBuffer;
        private readonly byte[] _recvBuffer;

        // Scratch space for datagrams received this Step call, so replies can be sent after
        // plant.Step(nowTicks) has run -- using the freshly-stepped state, not the pre-step one.
        private readonly uint[] _pendingSequences;
        private readonly long[] _pendingArrivalTicks;
        private int _pendingCount;

        public RobotEndpoint(
            IRobotPlant plant,
            ICommandCodec commandCodec,
            RobotStateFrameCodec stateCodec,
            ITransport uplinkTransport,
            ITransport downlinkTransport,
            ITimeAuthority robotClock,
            int maxDatagramsPerStep = 64)
        {
            if (stateCodec.MaxEncodedBytes > downlinkTransport.MaxPayloadBytes)
            {
                throw new ArgumentException(
                    "stateCodec.MaxEncodedBytes exceeds downlinkTransport.MaxPayloadBytes -- wiring error.");
            }

            _plant = plant;
            _commandCodec = commandCodec;
            _stateCodec = stateCodec;
            _uplinkTransport = uplinkTransport;
            _downlinkTransport = downlinkTransport;
            _ = robotClock;

            _sendBuffer = new byte[stateCodec.MaxEncodedBytes];
            _recvBuffer = new byte[uplinkTransport.MaxPayloadBytes];

            _pendingSequences = new uint[maxDatagramsPerStep];
            _pendingArrivalTicks = new long[maxDatagramsPerStep];
        }

        /// <summary>
        /// Drains the uplink transport, applies every decoded command to the plant (a corrupt
        /// or undecodable datagram is silently dropped, per <see cref="ICommandCodec.TryDecode"/>'s
        /// "reject rather than throw" contract), then steps the plant to
        /// <paramref name="nowTicks"/> <b>unconditionally</b> -- whether or not anything arrived
        /// this call, per <c>IRobotPlant.Step</c>'s own doc ("Called every step whether or not a
        /// command arrived"), which is what keeps the plant coasting through a gap. Only then are
        /// replies sent, so each one reports state as of the just-completed step, not the
        /// pre-step state.
        ///
        /// Replies once per <b>received</b> datagram, not once per <b>accepted</b> command:
        /// <see cref="IRobotPlant.Command"/> returns <c>void</c>, so the caller has no way to
        /// know whether a given frame was accepted or silently dropped as stale without
        /// re-implementing the plant's own dedup policy -- which the interface's own doc
        /// explicitly locates inside the plant, not its caller. Replying with the plant's honest
        /// current state for every receipt sidesteps needing that boolean, matches how a real
        /// robot would behave (it reports what it actually is, not a filtered acknowledgment of
        /// which packets it liked), and preserves the 1-reply-per-uplink-datagram correspondence
        /// <see cref="LatencyTrace"/>'s per-sequence correlation model assumes. A datagram beyond
        /// <c>maxDatagramsPerStep</c> in a single call is silently not replied to -- back
        /// pressure, not a crash; size the constructor's capacity generously for the traffic
        /// under test. Allocation-free.
        /// </summary>
        public void Step(long nowTicks)
        {
            _pendingCount = 0;

            while (_uplinkTransport.TryReceive(nowTicks, _recvBuffer, out int byteCount, out long arrivalTicks))
            {
                if (!_commandCodec.TryDecode(_recvBuffer.AsSpan(0, byteCount), out CommandFrame frame))
                {
                    continue;
                }

                _plant.Command(frame);

                if (_pendingCount < _pendingSequences.Length)
                {
                    _pendingSequences[_pendingCount] = frame.Sequence;
                    _pendingArrivalTicks[_pendingCount] = arrivalTicks;
                    _pendingCount++;
                }
            }

            _plant.Step(nowTicks);

            for (int i = 0; i < _pendingCount; i++)
            {
                SendStateReply(_pendingSequences[i], _pendingArrivalTicks[i], nowTicks);
            }
        }

        private void SendStateReply(uint sequence, long robotRecvTicks, long downlinkSendTicks)
        {
            var stateFrame = new RobotStateFrame(sequence, robotRecvTicks, downlinkSendTicks, _plant.State.Value);

            if (_stateCodec.TryEncode(stateFrame, _sendBuffer, out int bytesWritten))
            {
                _downlinkTransport.Send(_sendBuffer.AsSpan(0, bytesWritten), downlinkSendTicks);
            }
        }

        /// <summary>
        /// Returns the endpoint to its as-constructed state. Present for consistency with every
        /// other stateful component in Core, though it is provably a no-op today: the only
        /// mutable state this class carries, the per-step pending-reply scratch buffer, is
        /// already unconditionally cleared at the top of every <see cref="Step"/> call, so no
        /// value survives between calls for a trial boundary to catch. Does not reset the
        /// injected plant or transports -- those are separate dependencies with their own
        /// <c>Reset()</c>, called separately by whatever owns them.
        /// </summary>
        public void Reset()
        {
            _pendingCount = 0;
        }
    }
}
