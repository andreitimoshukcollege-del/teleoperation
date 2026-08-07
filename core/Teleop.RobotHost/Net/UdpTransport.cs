using System;
using System.Net;
using System.Net.Sockets;
using Teleop.Core.Contracts;

namespace Teleop.RobotHost.Net
{
    /// <summary>
    /// The first real, socket-backed <c>ITransport</c> in this project -- everything before this
    /// (Phase 4's whole loopback baseline) ran operator and robot logic in one process over
    /// <c>LoopbackTransport</c>. This is a fixed point-to-point link to exactly one peer (the
    /// operator's Tailscale address, given at construction), not a listening server that learns
    /// its peer from traffic -- matching how a plant is constructed directly by the host that
    /// knows which one it wants, rather than discovered.
    ///
    /// One instance is meant to be passed as <b>both</b> the uplink and downlink transport to
    /// <c>RobotEndpoint</c>'s constructor. The loopback baseline uses two separate instances
    /// specifically so each direction can be wrapped by its own independently-configured
    /// <c>EmulatedTransport</c> (asymmetric synthetic impairment); a real link has no such
    /// per-direction knob to inject, since the impairment here is whatever Tailscale actually
    /// does, not something this project is simulating.
    ///
    /// <b>Time is a parameter everywhere else in this project; here it necessarily is not, for
    /// <see cref="TryReceive"/>'s <c>arrivalTicks</c> output.</b> A real datagram's arrival time
    /// cannot be synthesized from the caller's <c>nowTicks</c> the way <c>LoopbackTransport</c>
    /// and <c>EmulatedTransport</c> do -- it must come from an actual clock read at the moment
    /// the datagram is pulled off the socket. The injected <see cref="ITimeAuthority"/> is read
    /// at that moment (not the poll-time <c>nowTicks</c> parameter, and not a raw
    /// <c>Stopwatch</c>/<c>DateTime</c> call), so every tick value this process produces stays on
    /// one consistent timebase. This read happens as close to the actual socket dequeue as
    /// plain <see cref="Socket"/> APIs allow; it does not use OS-level packet timestamping
    /// (e.g. Linux <c>SO_TIMESTAMP</c>), so a small amount of scheduling jitter between a
    /// datagram's real arrival and this process's next poll is folded into the measurement --
    /// an acknowledged, documented approximation, not a claim of kernel-level precision.
    ///
    /// <see cref="Send"/> ignores its own <c>nowTicks</c> parameter entirely: there is no virtual
    /// schedule to honor, delivery timing is whatever the real network does.
    ///
    /// Not thread-safe, by contract, same as every other <c>ITransport</c> implementation.
    /// </summary>
    public sealed class UdpTransport : ITransport, IDisposable
    {
        private readonly Socket _socket;
        private readonly EndPoint _remoteEndPoint;
        private readonly ITimeAuthority _clock;
        private readonly int _maxPayloadBytes;
        private readonly byte[] _receiveScratch;

        /// <param name="localPort">Port this host listens on.</param>
        /// <param name="remoteEndPoint">The one peer this transport ever sends to.</param>
        /// <param name="maxPayloadBytes">
        /// Largest datagram this transport carries. A received datagram larger than this is a
        /// contract violation by the sender (every codec here must not produce more than this)
        /// -- unlike <c>LoopbackTransport</c>, which can leave an oversized datagram queued for a
        /// retry with a bigger buffer, a real socket has already dequeued it by the time the
        /// size mismatch is discovered, so it is simply dropped. See <see cref="TryReceive"/>.
        /// </param>
        /// <param name="clock">
        /// Read once per received datagram, at the moment it is dequeued, to produce
        /// <see cref="TryReceive"/>'s <c>arrivalTicks</c> -- must be the same
        /// <see cref="ITimeAuthority"/> instance driving this host's own step loop, so every
        /// tick value stays on one timebase.
        /// </param>
        public UdpTransport(int localPort, IPEndPoint remoteEndPoint, int maxPayloadBytes, ITimeAuthority clock)
        {
            if (maxPayloadBytes <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(maxPayloadBytes), maxPayloadBytes, "Max payload bytes must be positive.");
            }

            _remoteEndPoint = remoteEndPoint ?? throw new ArgumentNullException(nameof(remoteEndPoint));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            _maxPayloadBytes = maxPayloadBytes;
            _receiveScratch = new byte[maxPayloadBytes];

            _socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            _socket.Bind(new IPEndPoint(IPAddress.Any, localPort));
            _socket.Blocking = false;
        }

        /// <inheritdoc/>
        public int MaxPayloadBytes => _maxPayloadBytes;

        /// <summary>
        /// Sends to the single peer given at construction. Returns false on any socket-level
        /// failure (e.g. the peer's address is momentarily unreachable) rather than throwing --
        /// same "false is an ordinary outcome, not an error" contract <see cref="ITransport"/>
        /// documents for emulated loss, now backed by a real, occasionally-actually-lossy link.
        /// </summary>
        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            _ = nowTicks; // no virtual schedule here -- see class doc.

            if (payload.Length > _maxPayloadBytes)
            {
                return false;
            }

            try
            {
                _socket.SendTo(payload, SocketFlags.None, _remoteEndPoint);
                return true;
            }
            catch (SocketException)
            {
                return false;
            }
        }

        /// <summary>
        /// Dequeues at most one real datagram already sitting in the socket's receive buffer.
        /// <paramref name="nowTicks"/> is accepted for interface compatibility but not used to
        /// gate delivery (there is no future to wait for -- if it has arrived, the OS already
        /// has it); see the class doc for why <paramref name="arrivalTicks"/> comes from the
        /// injected clock instead.
        /// </summary>
        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            _ = nowTicks;
            byteCount = 0;
            arrivalTicks = 0;

            if (_socket.Available <= 0)
            {
                return false;
            }

            int received;
            try
            {
                received = _socket.Receive(_receiveScratch, SocketFlags.None);
            }
            catch (SocketException)
            {
                return false;
            }

            // Read now, not at TryReceive's entry -- this is the closest this transport gets to
            // a real arrival timestamp. See the class doc's precision caveat.
            arrivalTicks = _clock.NowTicks;

            if (received > destination.Length)
            {
                // Unlike LoopbackTransport, the datagram is already off the wire and gone --
                // there is nothing left to re-queue for a retry with a bigger buffer. This is a
                // sender contract violation (every codec here must fit MaxPayloadBytes), not an
                // expected runtime condition.
                byteCount = received;
                return false;
            }

            _receiveScratch.AsSpan(0, received).CopyTo(destination);
            byteCount = received;
            return true;
        }

        /// <summary>
        /// Drains any datagrams already buffered in the socket's receive queue -- the closest a
        /// real socket can get to <c>LoopbackTransport</c>'s "nothing queued" guarantee. There is
        /// no equivalent for "nothing in flight": unlike a virtual queue, a real packet already
        /// sent onto the wire cannot be recalled.
        /// </summary>
        public void Reset()
        {
            while (_socket.Available > 0)
            {
                try
                {
                    _socket.Receive(_receiveScratch, SocketFlags.None);
                }
                catch (SocketException)
                {
                    break;
                }
            }
        }

        public void Dispose() => _socket.Dispose();
    }
}
