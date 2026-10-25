using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Teleop.RobotArm.Wire;
using Debug = UnityEngine.Debug;

namespace Teleop.Bridge
{
    /// <summary>
    /// Finds the robot's current address once, at start (docs/adr/0022-robot-discovery.md), so a new
    /// DHCP lease needs no config push: asks through Tailscale for its LAN addresses and uses the LAN
    /// when the robot answers there, Tailscale when it does not, and a LAN broadcast when Tailscale is
    /// not running here. Every decision is <see cref="RobotAddressResolver"/>'s (Teleop.RobotArm, tested
    /// headlessly); this only owns the socket. Blocks for up to about 2 s, once: the first caller pays
    /// it, and every later one (the operator bridge, the camera feed) gets the same answer.
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

            string key = config.RemoteHost + "|" + config.RobotTailscaleHost + "|" + config.DiscoveryPort;
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

            IPAddress found = Discover(tailscale, configured, config.DiscoveryPort);
            Resolved[key] = found;
            return found;
        }

        private static IPAddress Discover(IPAddress tailscale, IPAddress configured, int port)
        {
            var resolver = new RobotAddressResolver(tailscale, configured, (uint)Environment.TickCount ^ (uint)Stopwatch.GetTimestamp());
            try
            {
                using (var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
                {
                    socket.EnableBroadcast = true;
                    socket.Blocking = false;
                    socket.Bind(new IPEndPoint(IPAddress.Any, 0));
                    var query = new byte[RobotDiscoveryCodec.QuerySize];
                    RobotDiscoveryCodec.TryEncodeQuery(resolver.Nonce, query, out _);
                    var buffer = new byte[RobotDiscoveryCodec.MaxReplySize];
                    Stopwatch watch = Stopwatch.StartNew();
                    while (resolver.Source == RobotAddressSource.Pending)
                    {
                        while (resolver.NextQuery(watch.Elapsed.TotalSeconds, out IPAddress to))
                        {
                            try
                            {
                                socket.SendTo(query, new IPEndPoint(to, port));
                            }
                            catch (SocketException)
                            {
                                // No route (Tailscale absent, say): that step times out like silence.
                            }
                        }

                        while (socket.Available > 0)
                        {
                            EndPoint from = new IPEndPoint(IPAddress.Any, 0);
                            int n;
                            try
                            {
                                n = socket.ReceiveFrom(buffer, ref from);
                            }
                            catch (SocketException)
                            {
                                break;
                            }

                            resolver.OnDatagram(watch.Elapsed.TotalSeconds, ((IPEndPoint)from).Address, new ReadOnlySpan<byte>(buffer, 0, n));
                        }

                        Thread.Sleep(5);
                    }

                    foreach (string line in resolver.Log)
                    {
                        Debug.Log($"[discovery] {line}");
                    }

                    Debug.Log($"[discovery] settled in {watch.Elapsed.TotalSeconds:0.00} s");
                }
            }
            catch (SocketException e)
            {
                Debug.LogWarning($"[discovery] could not open a socket ({e.SocketErrorCode}); using the configured {configured}");
                return configured;
            }

            if (resolver.Source == RobotAddressSource.Configured || resolver.Source == RobotAddressSource.NotFound)
            {
                Debug.LogWarning($"[discovery] the robot did not answer; using the configured {configured}. " +
                                 "Is RobotHost running with --discovery-port (just deploy-robothost)?");
            }

            return resolver.Address ?? configured;
        }
    }
}
