using System.Net;
using Teleop.Core.Camera;

namespace Teleop.CameraHost.Streaming
{
    /// <summary>What <see cref="CameraStreamer.OfferFrame"/> did with one captured frame.</summary>
    internal enum FrameOutcome
    {
        Sent,
        NoSubscriber,
        Paced,
        SendFailed,
        Unsendable,
    }

    /// <summary>
    /// The sender side of docs/adr/0014-camera-frame-downlink.md, without the device or the socket:
    /// keeps track of the subscriber from keepalive datagrams, paces frames to the rate it asked for,
    /// and splits each frame it sends into chunks with Core's <see cref="CameraChunkCodec"/>.
    ///
    /// <para>Two rules the viewer's reassembler depends on (core/Teleop.Core/Camera/CLAUDE.md):</para>
    /// <list type="bullet">
    /// <item>Every chunk of a frame carries the same stamp, <c>sendTicks</c> included: read once,
    /// just before the first chunk is handed over.</item>
    /// <item>Frame ids never restart below where they were: the first id is supplied by the host
    /// (milliseconds since boot, so a restarted sender starts ahead of every id it sent before), and an
    /// id is consumed by every frame that starts sending, even one that fails partway.</item>
    /// </list>
    /// </summary>
    internal sealed class CameraStreamer
    {
        private readonly IDatagramSender _sender;
        private readonly Func<long> _clock;
        private readonly long _ticksPerSecond;
        private readonly SubscriberTracker _subscribers;
        private readonly FramePacer _pacer;
        private readonly CameraChunkCodec _chunkCodec = new CameraChunkCodec();
        private readonly CameraSubscribeCodec _subscribeCodec = new CameraSubscribeCodec();
        private readonly byte[] _datagram = new byte[CameraChunkCodec.MaxDatagramBytes];
        private uint _nextFrameId;

        public CameraStreamer(
            IDatagramSender sender, Func<long> clock, long ticksPerSecond, long subscriberTimeoutTicks, uint firstFrameId)
        {
            _sender = sender ?? throw new ArgumentNullException(nameof(sender));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            _ticksPerSecond = ticksPerSecond;
            _subscribers = new SubscriberTracker(subscriberTimeoutTicks);
            _pacer = new FramePacer(ticksPerSecond);
            _nextFrameId = firstFrameId;
        }

        public long FramesSent { get; private set; }
        public long FramesWithoutSubscriber { get; private set; }
        public long FramesPaced { get; private set; }
        public long FramesFailed { get; private set; }
        public long ChunksSent { get; private set; }
        public long BytesSent { get; private set; }
        public long SubscribesAccepted { get; private set; }
        public long DatagramsIgnored { get; private set; }
        public uint NextFrameId => _nextFrameId;
        public EndPoint? LastSubscriber => _subscribers.LastTarget;

        /// <summary>
        /// Handles a datagram received on the sender's port. Keepalives update the subscriber;
        /// anything else is counted and ignored. Returns true when the stream's target changed.
        /// </summary>
        public bool HandleDatagram(ReadOnlySpan<byte> datagram, EndPoint from, long nowTicks)
        {
            if (!_subscribeCodec.TryDecode(datagram, out ushort requestedMaxFps, out _))
            {
                DatagramsIgnored++;
                return false;
            }

            SubscribesAccepted++;
            bool changed = _subscribers.Observe(from, requestedMaxFps, nowTicks);
            if (changed)
            {
                _pacer.Reset();
            }

            return changed;
        }

        /// <param name="captureTicks">The driver's stamp in this host's clock, or <c>LatencyTrace.Unset</c> (ADR 0014 §4).</param>
        /// <param name="nowTicks">Used for pacing when the capture stamp is unset, and for the subscriber timeout.</param>
        public FrameOutcome OfferFrame(ReadOnlySpan<byte> jpeg, long captureTicks, ushort width, ushort height, long nowTicks)
        {
            if (!_subscribers.TryGetActive(nowTicks, out EndPoint target, out ushort requestedMaxFps))
            {
                FramesWithoutSubscriber++;
                return FrameOutcome.NoSubscriber;
            }

            long paceTicks = captureTicks == Teleop.Core.Types.LatencyTrace.Unset ? nowTicks : captureTicks;
            if (!_pacer.ShouldSend(paceTicks, requestedMaxFps))
            {
                FramesPaced++;
                return FrameOutcome.Paced;
            }

            int chunkCount = CameraChunkCodec.ChunkCountFor(jpeg.Length);
            if (chunkCount == 0 || jpeg.Length > CameraChunkCodec.MaxFrameBytes || width == 0 || height == 0)
            {
                FramesFailed++;
                return FrameOutcome.Unsendable;
            }

            var stamp = new CameraFrameStamp(_nextFrameId, captureTicks, _clock(), _ticksPerSecond, width, height);
            _nextFrameId++;
            for (int i = 0; i < chunkCount; i++)
            {
                if (!_chunkCodec.TryEncodeChunk(stamp, jpeg, i, _datagram, out int written) ||
                    !_sender.TrySend(new ReadOnlySpan<byte>(_datagram, 0, written), target))
                {
                    // The rest of this frame would be useless to the viewer, so stop here; the
                    // reassembler counts the partial frame when a newer one completes.
                    FramesFailed++;
                    return FrameOutcome.SendFailed;
                }

                ChunksSent++;
                BytesSent += written;
            }

            _pacer.MarkSent(paceTicks);
            FramesSent++;
            return FrameOutcome.Sent;
        }
    }
}
