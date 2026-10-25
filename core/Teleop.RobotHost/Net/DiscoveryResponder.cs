using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Net
{
    /// <summary>
    /// Answers the headset's discovery query (docs/adr/0022-robot-discovery.md) on its own UDP port:
    /// the hostname and every IPv4 address a device on the robot's network could reach it at, read
    /// from the interfaces at each query so a new DHCP lease is reported at once. Never moves anything
    /// and never touches the joint port, where any sender takes the joint-state stream. Polled from
    /// RobotHost's loop; non-blocking.
    /// </summary>
    public sealed class DiscoveryResponder : IDisposable
    {
        private readonly Socket _socket;
        private readonly byte[] _receive = new byte[64];
        private readonly byte[] _reply = new byte[RobotDiscoveryCodec.MaxReplySize];
        private readonly Func<IReadOnlyList<IPAddress>> _addresses;
        private readonly string _name;

        public DiscoveryResponder(int port, Func<IReadOnlyList<IPAddress>>? addresses = null, string? name = null)
        {
            _socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp) { Blocking = false };
            _socket.Bind(new IPEndPoint(IPAddress.Any, port));
            _addresses = addresses ?? LanAddresses;
            _name = name ?? Environment.MachineName;
        }

        public int Port => ((IPEndPoint)_socket.LocalEndPoint!).Port;

        /// <summary>Queries answered so far.</summary>
        public int Answered { get; private set; }

        /// <summary>Answers every query waiting. Returns how many it answered.</summary>
        public int Poll()
        {
            int answered = 0;
            while (_socket.Available > 0)
            {
                EndPoint from = new IPEndPoint(IPAddress.Any, 0);
                int n;
                try
                {
                    n = _socket.ReceiveFrom(_receive, ref from);
                }
                catch (SocketException)
                {
                    break;
                }

                if (!RobotDiscoveryCodec.TryDecodeQuery(_receive.AsSpan(0, n), out uint nonce)) continue;
                IPAddress[] addresses = _addresses().ToArray();
                if (RobotDiscoveryCodec.TryEncodeReply(nonce, addresses, _name, _reply, out int written))
                {
                    try
                    {
                        _socket.SendTo(_reply, 0, written, SocketFlags.None, from);
                        answered++;
                    }
                    catch (SocketException)
                    {
                    }
                }
            }

            Answered += answered;
            return answered;
        }

        public void Dispose() => _socket.Dispose();

        /// <summary>The IPv4 addresses of the interfaces that are up, less those no other device on the LAN reaches the robot at.</summary>
        public static IReadOnlyList<IPAddress> LanAddresses()
        {
            var found = new List<(string, IPAddress)>();
            foreach (NetworkInterface nic in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (nic.OperationalStatus != OperationalStatus.Up) continue;
                foreach (UnicastIPAddressInformation a in nic.GetIPProperties().UnicastAddresses)
                {
                    found.Add((nic.Name, a.Address));
                }
            }

            return Filter(found);
        }

        /// <summary>
        /// Keeps IPv4 addresses on real interfaces: no loopback, link-local (169.254/16), Tailscale
        /// (100.64/10, <c>tailscale*</c>), or Docker (172.17/16, <c>docker*</c>, <c>br-*</c>,
        /// <c>veth*</c>). The headset already knows the Tailscale address it asked through.
        /// </summary>
        public static IReadOnlyList<IPAddress> Filter(IEnumerable<(string Interface, IPAddress Address)> addresses)
        {
            var kept = new List<IPAddress>();
            foreach ((string name, IPAddress address) in addresses)
            {
                if (address.AddressFamily != AddressFamily.InterNetwork || IPAddress.IsLoopback(address)) continue;
                if (name.StartsWith("tailscale", StringComparison.Ordinal) || name.StartsWith("docker", StringComparison.Ordinal) ||
                    name.StartsWith("br-", StringComparison.Ordinal) || name.StartsWith("veth", StringComparison.Ordinal) || name == "lo")
                {
                    continue;
                }

                byte[] b = address.GetAddressBytes();
                if ((b[0] == 169 && b[1] == 254) || (b[0] == 172 && b[1] == 17) || RobotAddressResolver.IsTailscaleRange(address)) continue;
                if (!kept.Contains(address)) kept.Add(address);
            }

            return kept;
        }
    }
}
