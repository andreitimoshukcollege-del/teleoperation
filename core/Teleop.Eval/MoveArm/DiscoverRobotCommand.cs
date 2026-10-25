using System;
using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.MoveArm
{
    /// <summary>
    /// <c>discover-robot</c> (<c>just robot-discover</c>): finds the robot exactly as the headset does at
    /// start (docs/adr/0022-robot-discovery.md), with the same <see cref="RobotAddressResolver"/>, so the
    /// robot's discovery responder can be checked from the laptop. Sends only discovery queries; moves
    /// nothing. Exits 0 when the robot answered (on the LAN or through Tailscale), 1 when nothing did.
    /// </summary>
    public static class DiscoverRobotCommand
    {
        public static int Run(string[] args)
        {
            IPAddress? tailscale = IPAddress.Parse("100.112.90.72");
            IPAddress? configured = null;
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
                        case "--port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        default: throw new ArgumentException($"unknown argument '{args[i]}'");
                    }
                }
                catch (Exception e) when (e is ArgumentException || e is FormatException)
                {
                    Console.Error.WriteLine($"discover-robot: {e.Message}");
                    Console.Error.WriteLine("usage: discover-robot [--tailscale <ip>|none] [--configured <ip>] [--port 6005]");
                    return 64;
                }
            }

            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp) { EnableBroadcast = true, Blocking = false };
            socket.Bind(new IPEndPoint(IPAddress.Any, 0));
            var resolver = new RobotAddressResolver(tailscale, configured, (uint)Environment.TickCount ^ (uint)Stopwatch.GetTimestamp());
            var query = new byte[RobotDiscoveryCodec.QuerySize];
            RobotDiscoveryCodec.TryEncodeQuery(resolver.Nonce, query, out _);
            var buffer = new byte[RobotDiscoveryCodec.MaxReplySize];
            var watch = Stopwatch.StartNew();
            while (resolver.Source == RobotAddressSource.Pending)
            {
                double now = watch.Elapsed.TotalSeconds;
                while (resolver.NextQuery(now, out IPAddress to))
                {
                    try
                    {
                        socket.SendTo(query, new IPEndPoint(to, port));
                    }
                    catch (SocketException e)
                    {
                        Console.WriteLine($"  (could not send to {to}: {e.SocketErrorCode})");
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

                    resolver.OnDatagram(watch.Elapsed.TotalSeconds, ((IPEndPoint)from).Address, buffer.AsSpan(0, n));
                }

                Thread.Sleep(5);
            }

            foreach (string line in resolver.Log) Console.WriteLine($"discover-robot: {line}");
            Console.WriteLine($"discover-robot: settled in {watch.Elapsed.TotalSeconds:0.00} s");
            return resolver.Source == RobotAddressSource.Lan || resolver.Source == RobotAddressSource.Tailscale ? 0 : 1;
        }
    }
}
