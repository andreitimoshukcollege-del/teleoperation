using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.MoveArm
{
    /// <summary>
    /// <c>discover-robot</c> (<c>just robot-discover</c>): finds the robot exactly as the headset does at
    /// start (docs/adr/0022-robot-discovery.md), with the same <see cref="RobotAddressResolver"/>, so both
    /// halves can be checked from the laptop. Sends only discovery and mDNS queries; moves nothing. Exits
    /// 0 when the robot answered (on the LAN or through Tailscale), 1 when nothing did.
    ///
    /// <para>The socket handling mirrors <c>unity/Teleop.Bridge/Runtime/RobotAddressDiscovery.cs</c>: one
    /// socket on 0.0.0.0 for discovery queries, which must route through Tailscale too, and one per IPv4
    /// interface for mDNS (docs/adr/0015 §3: from 0.0.0.0, Windows sent it out of a virtual adapter).</para>
    /// </summary>
    public static class DiscoverRobotCommand
    {
        public static int Run(string[] args)
        {
            IPAddress? tailscale = IPAddress.Parse("100.112.90.72");
            IPAddress? configured = null;
            string? mdnsName = "jetrover-sinrg.local";
            int port = RobotDiscoveryCodec.DefaultPort;
            for (int i = 1; i < args.Length; i++)
            {
                string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                try
                {
                    switch (args[i])
                    {
                        case "--tailscale": string t = Next(); tailscale = t == "none" ? null : IPAddress.Parse(t); break;
                        case "--configured": configured = IPAddress.Parse(Next()); break;
                        case "--mdns-name": string m = Next(); mdnsName = m == "none" ? null : m; break;
                        case "--port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        default: throw new ArgumentException($"unknown argument '{args[i]}'");
                    }
                }
                catch (Exception e) when (e is ArgumentException || e is FormatException)
                {
                    Console.Error.WriteLine($"discover-robot: {e.Message}");
                    Console.Error.WriteLine("usage: discover-robot [--tailscale <ip>|none] [--mdns-name <name>|none] [--configured <ip>] [--port 6005]");
                    return 64;
                }
            }

            var resolver = new RobotAddressResolver(tailscale, configured, mdnsName, (uint)Environment.TickCount ^ (uint)Stopwatch.GetTimestamp());
            var main = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp) { Blocking = false };
            main.Bind(new IPEndPoint(IPAddress.Any, 0));
            List<Socket> mdnsSockets = MdnsSockets(main);
            var all = new List<Socket>(mdnsSockets);
            if (!all.Contains(main)) all.Add(main);
            try
            {
                var query = new byte[RobotDiscoveryCodec.QuerySize];
                RobotDiscoveryCodec.TryEncodeQuery(resolver.Nonce, query, out _);
                var mdnsQuery = new byte[MdnsQueryCodec.MaxMessageBytes];
                int mdnsLength = 0;
                if (mdnsName != null) MdnsQueryCodec.TryEncodeQuery(mdnsName, resolver.MdnsId, mdnsQuery, out mdnsLength);
                var buffer = new byte[MdnsQueryCodec.MaxMessageBytes];
                var watch = Stopwatch.StartNew();
                while (resolver.Source == RobotAddressSource.Pending)
                {
                    while (resolver.NextQuery(watch.Elapsed.TotalSeconds, out IPAddress to, out RobotQueryKind kind))
                    {
                        if (kind == RobotQueryKind.Discovery) TrySend(main, query, query.Length, new IPEndPoint(to, port));
                        else foreach (Socket s in mdnsSockets) TrySend(s, mdnsQuery, mdnsLength, new IPEndPoint(to, MdnsQueryCodec.Port));
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

                            resolver.OnDatagram(watch.Elapsed.TotalSeconds, ((IPEndPoint)from).Address, buffer.AsSpan(0, n));
                        }
                    }

                    Thread.Sleep(5);
                }

                foreach (string line in resolver.Log) Console.WriteLine($"discover-robot: {line}");
                Console.WriteLine($"discover-robot: settled in {watch.Elapsed.TotalSeconds:0.00} s");
            }
            finally
            {
                foreach (Socket s in all) s.Dispose();
            }

            return resolver.Source == RobotAddressSource.Lan || resolver.Source == RobotAddressSource.Tailscale ? 0 : 1;
        }

        /// <summary>One socket bound to each up IPv4 interface that could reach the robot's LAN; the main socket when none can be listed.</summary>
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
            catch (NetworkInformationException)
            {
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
