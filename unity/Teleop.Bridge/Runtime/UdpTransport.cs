using System;
using System.Net;
using System.Net.Sockets;
using Teleop.Core.Contracts;

namespace Teleop.Bridge
{
    /// <summary>
    /// A near-byte-for-byte copy of <c>Teleop.RobotHost.Net.UdpTransport</c> (itself already a
    /// byte-for-byte copy of <c>Teleop.Eval.Net.UdpTransport</c>) -- the third copy of this class,
    /// for Unity's side of a real cross-machine link to the JetRover
    /// (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md). Each host gets its own copy
    /// rather than a shared project reference between sibling host projects -- the same "every
    /// host gets its own copy" precedent <c>UnityMonotonicClock</c>'s own doc comment already
    /// establishes for <c>ITimeAuthority</c>, extended here to the first real <c>ITransport</c>
    /// Unity needs. See <c>Teleop.RobotHost.Net.UdpTransport</c> for the full rationale on every
    /// design choice below (one fixed peer, arrival time read from the injected clock at dequeue
    /// rather than synthesized, <see cref="Send"/> ignoring its own <c>nowTicks</c>). Pure BCL
    /// sockets, no <c>UnityEngine</c> dependency -- <c>System.Net.Sockets</c> is available under
    /// IL2CPP (Teleop/CLAUDE.md's own Quest constraints require <c>Internet Access = Require</c>
    /// for exactly this reason).
    ///
    /// <b>One real divergence from the other two copies, found by the Unity compiler, not
    /// guessed:</b> <see cref="Send"/> cannot call <c>Socket.SendTo(ReadOnlySpan&lt;byte&gt;, ...)</c>
    /// the way the net8.0 copies do -- that overload doesn't exist in Unity's Mono/IL2CPP corlib
    /// for this API compatibility level. Copies the span into a preallocated <c>byte[]</c> scratch
    /// buffer (mirroring <see cref="TryReceive"/>'s existing <c>_receiveScratch</c> pattern, which
    /// already used a <c>byte[]</c>-based <c>Receive</c> overload for the same underlying reason)
    /// and calls the offset/count <c>SendTo</c> overload instead, which does exist.
    /// </summary>
    public sealed class UdpTransport : ITransport, IDisposable
    {
        private readonly Socket _socket;
        private readonly EndPoint _remoteEndPoint;
        private readonly ITimeAuthority _clock;
        private readonly int _maxPayloadBytes;
        private readonly byte[] _receiveScratch;
        private readonly byte[] _sendScratch;

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
            _sendScratch = new byte[maxPayloadBytes];

            _socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            _socket.Bind(new IPEndPoint(IPAddress.Any, localPort));
            _socket.Blocking = false;
        }

        public int MaxPayloadBytes => _maxPayloadBytes;

        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            _ = nowTicks;
            if (payload.Length > _maxPayloadBytes) { return false; }
            payload.CopyTo(_sendScratch);
            try { _socket.SendTo(_sendScratch, 0, payload.Length, SocketFlags.None, _remoteEndPoint); return true; }
            catch (SocketException) { return false; }
        }

        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            _ = nowTicks;
            byteCount = 0;
            arrivalTicks = 0;
            if (_socket.Available <= 0) { return false; }

            int received;
            try { received = _socket.Receive(_receiveScratch, SocketFlags.None); }
            catch (SocketException) { return false; }

            arrivalTicks = _clock.NowTicks;

            if (received > destination.Length) { byteCount = received; return false; }

            _receiveScratch.AsSpan(0, received).CopyTo(destination);
            byteCount = received;
            return true;
        }

        public void Reset()
        {
            while (_socket.Available > 0)
            {
                try { _socket.Receive(_receiveScratch, SocketFlags.None); }
                catch (SocketException) { break; }
            }
        }

        public void Dispose() => _socket.Dispose();
    }
}
