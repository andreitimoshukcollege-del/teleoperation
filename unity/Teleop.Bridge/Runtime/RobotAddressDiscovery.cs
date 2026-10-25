using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;
using Teleop.RobotArm.Wire;
using Debug = UnityEngine.Debug;

namespace Teleop.Bridge
{
    /// <summary>
    /// Finds the robot's current address once, at start (docs/adr/0022-robot-discovery.md), so a new
    /// DHCP lease needs no config push: asks through Tailscale for its LAN addresses and uses the LAN
    /// when the robot answers there, Tailscale when it does not, and, when Tailscale is not running
    /// here, the robot's mDNS name (docs/adr/0015's legacy-unicast query). Every decision is
    /// <see cref="RobotAddressResolver"/>'s (Teleop.RobotArm, tested headlessly); this only owns the
    /// sockets, as <c>Teleop.Eval discover-robot</c> does on the PC. Blocks for up to about 2.5 s,
    /// once: the first caller pays it, and every later one (the operator bridge, the camera feed) gets
    /// the same answer.
    /// </summary>
    public static class RobotAddressDiscovery
    {
        private static readonly Dictionary<string, IPAddress> Resolved = new Dictionary<string, IPAddress>();

        /// <summary>
        /// The robot's address for <paramref name="config"/>: discovered when
        /// <see cref="JetRoverArmConfig.DiscoverRobot"/> is on, otherwise <see cref="JetRoverArmConfig.RemoteHost"/>
        /// as before. Falls back to <see cref="JetRoverArmConfig.RemoteHost"/> when nothing answers.
        /// </summary>
        public static IPAddress Resolve(JetRoverArmConfig config)
        {
            IPAddress configured = IPAddress.Parse(config.RemoteHost);
            if (!config.DiscoverRobot)
            {
                return configured;
            }

            string key = config.RemoteHost + "|" + config.RobotTailscaleHost + "|" + config.RemoteHostName + "|" + config.DiscoveryPort;
            if (Resolved.TryGetValue(key, out IPAddress cached))
            {
                return cached;
            }

            IPAddress tailscale = null;
            if (!string.IsNullOrEmpty(config.RobotTailscaleHost) && !IPAddress.TryParse(config.RobotTailscaleHost, out tailscale))
            {
                Debug.LogWarning($"[discovery] RobotTailscaleHost '{config.RobotTailscaleHost}' is not an IP address; skipping Tailscale");
                tailscale = null;
            }

            IPAddress found = Discover(tailscale, configured, config.RemoteHostName, config.DiscoveryPort);
            Resolved[key] = found;
            return found;
        }

        private static IPAddress Discover(IPAddress tailscale, IPAddress configured, string mdnsName, int port)
        {
            var resolver = new RobotAddressResolver(tailscale, configured, mdnsName, (uint)Environment.TickCount ^ (uint)Stopwatch.GetTimestamp());
            var all = new List<Socket>();
            try
            {
                // Discovery queries go from 0.0.0.0, so the one through Tailscale takes Tailscale's route.
                var main = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp) { Blocking = false };
                main.Bind(new IPEndPoint(IPAddress.Any, 0));
                List<Socket> mdnsSockets = MdnsSockets(main);
                all.AddRange(mdnsSockets);
                if (!all.Contains(main)) all.Add(main);

                var query = new byte[RobotDiscoveryCodec.QuerySize];
                RobotDiscoveryCodec.TryEncodeQuery(resolver.Nonce, query, out _);
                var mdnsQuery = new byte[MdnsQueryCodec.MaxMessageBytes];
                int mdnsLength = 0;
                if (resolver.MdnsName != null) MdnsQueryCodec.TryEncodeQuery(resolver.MdnsName, resolver.MdnsId, mdnsQuery, out mdnsLength);
                var buffer = new byte[MdnsQueryCodec.MaxMessageBytes];
                Stopwatch watch = Stopwatch.StartNew();
                while (resolver.Source == RobotAddressSource.Pending)
                {
                    while (resolver.NextQuery(watch.Elapsed.TotalSeconds, out IPAddress to, out RobotQueryKind kind))
                    {
                        if (kind == RobotQueryKind.Discovery)
                        {
                            TrySend(main, query, query.Length, new IPEndPoint(to, port));
                        }
                        else
                        {
                            foreach (Socket s in mdnsSockets) TrySend(s, mdnsQuery, mdnsLength, new IPEndPoint(to, MdnsQueryCodec.Port));
                        }
                    }

                    foreach (Socket s in all)
                    {
                        while (s.Available > 0)
                        {
                            EndPoint from = new IPEndPoint(IPAddress.Any, 0);
                            int n;
                            try
                            {
                                n = s.ReceiveFrom(buffer, ref from);
                            }
                            catch (SocketException)
                            {
                                break;
                            }

                            resolver.OnDatagram(watch.Elapsed.TotalSeconds, ((IPEndPoint)from).Address, new ReadOnlySpan<byte>(buffer, 0, n));
                        }
                    }

                    Thread.Sleep(5);
                }

                foreach (string line in resolver.Log)
                {
                    Debug.Log($"[discovery] {line}");
                }

                Debug.Log($"[discovery] settled in {watch.Elapsed.TotalSeconds:0.00} s");
            }
            catch (SocketException e)
            {
                Debug.LogWarning($"[discovery] could not open a socket ({e.SocketErrorCode}); using the configured {configured}");
                return configured;
            }
            finally
            {
                foreach (Socket s in all) s.Dispose();
            }

            if (resolver.Source == RobotAddressSource.Configured || resolver.Source == RobotAddressSource.NotFound)
            {
                Debug.LogWarning($"[discovery] the robot did not answer; using the configured {configured}. " +
                                 "Is it on this network, and is RobotHost running with --discovery-port (just deploy-robothost)?");
            }

            return resolver.Address ?? configured;
        }

        /// <summary>
        /// One socket bound to each up IPv4 interface that could reach the robot's LAN (docs/adr/0015 §3);
        /// the main socket when none can be listed, which IL2CPP on Android may not allow.
        /// </summary>
        private static List<Socket> MdnsSockets(Socket fallback)
        {
            var sockets = new List<Socket>();
            try
            {
                foreach (NetworkInterface nic in NetworkInterface.GetAllNetworkInterfaces())
                {
                    if (nic.OperationalStatus != OperationalStatus.Up) continue;
                    foreach (UnicastIPAddressInformation a in nic.GetIPProperties().UnicastAddresses)
                    {
                        if (a.Address.AddressFamily != AddressFamily.InterNetwork || !RobotAddressResolver.IsLanCandidate(a.Address)) continue;
                        try
                        {
                            var s = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp) { Blocking = false };
                            s.Bind(new IPEndPoint(a.Address, 0));
                            s.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastInterface, a.Address.GetAddressBytes());
                            sockets.Add(s);
                        }
                        catch (SocketException)
                        {
                        }
                    }
                }
            }
            catch (Exception e) when (e is NetworkInformationException || e is NotSupportedException || e is PlatformNotSupportedException)
            {
                Debug.Log($"[discovery] cannot list network interfaces here ({e.GetType().Name}); mDNS goes out on the default one");
            }

            if (sockets.Count == 0) sockets.Add(fallback);
            return sockets;
        }

        private static void TrySend(Socket socket, byte[] payload, int length, IPEndPoint to)
        {
            try
            {
                socket.SendTo(payload, 0, length, SocketFlags.None, to);
            }
            catch (SocketException)
            {
                // No route (Tailscale absent, say): that step times out like silence.
            }
        }
    }
}
