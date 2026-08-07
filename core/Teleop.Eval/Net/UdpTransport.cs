using System;
using System.Net;
using System.Net.Sockets;
using Teleop.Core.Contracts;

namespace Teleop.Eval.Net
{
    /// <summary>
    /// A byte-for-byte copy of <c>Teleop.RobotHost.Net.UdpTransport</c>, for the operator/dev-
    /// machine side of a real cross-machine link (<c>clocksync-check</c>). Each host gets its own
    /// copy rather than a shared project reference between sibling host projects -- the same
    /// "every host gets its own copy" precedent <c>Teleop.Eval.Time.MonotonicClock</c>'s own doc
    /// comment already establishes for <c>ITimeAuthority</c>, extended here to the first real
    /// <c>ITransport</c> a second host needs. See that class for the full rationale on every
    /// design choice below (one fixed peer, arrival time read from the injected clock at dequeue
    /// rather than synthesized, <c>Send</c> ignoring its own <c>nowTicks</c>).
    /// </summary>
    public sealed class UdpTransport : ITransport, IDisposable
    {
        private readonly Socket _socket;
        private readonly EndPoint _remoteEndPoint;
        private readonly ITimeAuthority _clock;
        private readonly int _maxPayloadBytes;
        private readonly byte[] _receiveScratch;

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

        public int MaxPayloadBytes => _maxPayloadBytes;

        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            _ = nowTicks;

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

            arrivalTicks = _clock.NowTicks;

            if (received > destination.Length)
            {
                byteCount = received;
                return false;
            }

            _receiveScratch.AsSpan(0, received).CopyTo(destination);
            byteCount = received;
            return true;
        }

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
