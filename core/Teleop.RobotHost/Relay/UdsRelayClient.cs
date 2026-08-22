using System;
using System.IO;
using System.Net.Sockets;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Relay
{
    /// <summary>
    /// The real <see cref="IRelayClient"/>: a Unix domain datagram socket to the ROS relay node
    /// running on the same machine. Deliberately a Unix domain socket, not UDP-loopback --
    /// a UDS path is categorically unreachable from Tailscale/LAN, which matters here because
    /// this hop drives real motors with zero authentication of its own (all of that lives one
    /// layer up, in whatever authenticates the real <c>UdpTransport</c> link to the operator). A
    /// UDP-loopback socket is one accidental non-loopback bind -- now or in a future refactor --
    /// away from being reachable off-host; a filesystem-path-bound Unix socket categorically
    /// cannot be.
    ///
    /// Fire-and-forget in both directions, matching <see cref="IRelayClient"/>'s "here are the
    /// current numbers" contract: <see cref="Send"/> does not confirm the relay node received
    /// anything, and <see cref="TryReceiveFeedback"/> only ever returns the most recent
    /// datagram sitting in the socket's receive buffer. Fixed-size buffers sized to
    /// <see cref="RelayProtocol.MaxJointsPerMessage"/> -- allocation-free regardless of how many
    /// joints the profile in use actually has.
    /// </summary>
    public sealed class UdsRelayClient : IRelayClient, IDisposable
    {
        private readonly Socket _socket;
        private readonly UnixDomainSocketEndPoint _peerEndPoint;
        private readonly string _localSocketPath;
        private readonly byte[] _sendBuffer = new byte[RelayProtocol.CommandEncodedSize(RelayProtocol.MaxJointsPerMessage)];
        private readonly byte[] _receiveBuffer = new byte[RelayProtocol.FeedbackEncodedSize(RelayProtocol.MaxJointsPerMessage)];

        /// <param name="localSocketPath">
        /// Filesystem path this process binds to receive feedback on. Deleted and recreated at
        /// construction (a stale socket file left over from a crashed previous run must not
        /// block binding), and deleted again on <see cref="Dispose"/>.
        /// </param>
        /// <param name="relaySocketPath">Filesystem path the ROS relay node listens on.</param>
        public UdsRelayClient(string localSocketPath, string relaySocketPath)
        {
            _localSocketPath = localSocketPath;
            _peerEndPoint = new UnixDomainSocketEndPoint(relaySocketPath);

            if (File.Exists(localSocketPath))
            {
                File.Delete(localSocketPath);
            }

            _socket = new Socket(AddressFamily.Unix, SocketType.Dgram, ProtocolType.Unspecified);
            _socket.Bind(new UnixDomainSocketEndPoint(localSocketPath));
            _socket.Blocking = false;
        }

        /// <summary>
        /// Fire-and-forget. Swallows the case where the relay node isn't listening yet (or has
        /// restarted) rather than throwing -- the next <see cref="Send"/> after it comes back up
        /// simply succeeds, same as the robot's own bus servos holding position through a gap.
        /// </summary>
        public void Send(ReadOnlySpan<JointTarget> targets)
        {
            int n = RelayProtocol.EncodeCommand(targets, _sendBuffer);
            try
            {
                _socket.SendTo(_sendBuffer.AsSpan(0, n), SocketFlags.None, _peerEndPoint);
            }
            catch (SocketException)
            {
            }
        }

        public bool TryReceiveFeedback(Span<JointFeedbackEntry> entriesBuffer, out int entryCount)
        {
            entryCount = 0;

            if (_socket.Available <= 0)
            {
                return false;
            }

            int received;
            try
            {
                received = _socket.Receive(_receiveBuffer, SocketFlags.None);
            }
            catch (SocketException)
            {
                return false;
            }

            return RelayProtocol.TryDecodeFeedback(_receiveBuffer.AsSpan(0, received), entriesBuffer, out entryCount);
        }

        public void Dispose()
        {
            _socket.Dispose();
            try
            {
                File.Delete(_localSocketPath);
            }
            catch (IOException)
            {
            }
        }
    }
}
