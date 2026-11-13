using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Teleop.Eval.Net
{
    /// <summary>
    /// <c>udp-relay</c>: forwards a headset's UDP to a host it cannot reach, from this machine, which can (<c>just isaac-relay</c>).
    /// The case it exists for: Isaac's RobotHost on the GPU machine is on the lab's network, this PC reaches it over the lab VPN,
    /// and the headset is on this PC's Wi-Fi without the VPN.
    /// <list type="bullet">
    /// <item>For each port P it listens on this machine's P, and sends what arrives to the upstream host's P from a socket of its
    /// own; what comes back on that socket goes to whoever last sent on P. RobotHost and the camera senders answer whoever sent
    /// to them, so the headset needs only its <c>RemoteHost</c> set to this machine.</item>
    /// <item>One headset at a time: a new sender on a port takes it over, as RobotHost's reply-to-sender does.</item>
    /// <item>It reads nothing and changes nothing: the bytes pass as they are. It adds this machine's hop to the delay, so
    /// anything measured through it is not a lab-network measurement.</item>
    /// </list>
    /// Stops on Enter (a WSL Ctrl+C does not reach a Windows dotnet), or after <c>--seconds</c>.
    /// </summary>
    internal static class UdpRelayCommand
    {
        private sealed class Leg
        {
            public int Port;
            public Socket Front = null!;     // this machine's P: the headset sends here
            public Socket Back = null!;      // an ephemeral socket: to and from the upstream host's P
            public IPEndPoint Upstream = null!;
            public EndPoint? Client;
            public long Up, Down, UpBytes, DownBytes;
        }

        public static int Run(string[] args)
        {
            string? host = null;
            string ports = "6000,6002,6003,6008,6009";
            double seconds = 0;
            for (int i = 1; i < args.Length; i++)
            {
                string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                switch (args[i])
                {
                    case "--upstream": host = Next(); break;
                    case "--ports": ports = Next(); break;
                    case "--seconds": seconds = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                    default:
                        Console.Error.WriteLine("usage: udp-relay --upstream <host> [--ports 6000,6002,6003,6008,6009] [--seconds 0]");
                        return 2;
                }
            }

            if (host == null)
            {
                Console.Error.WriteLine("udp-relay: --upstream <host> is required");
                return 2;
            }

            IPAddress upstream = IPAddress.TryParse(host, out IPAddress? a) ? a : Dns.GetHostAddresses(host).First(x => x.AddressFamily == AddressFamily.InterNetwork);
            var legs = new List<Leg>();
            try
            {
                foreach (string p in ports.Split(',', StringSplitOptions.RemoveEmptyEntries))
                {
                    int port = int.Parse(p, CultureInfo.InvariantCulture);
                    var leg = new Leg { Port = port, Upstream = new IPEndPoint(upstream, port) };
                    leg.Front = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                    leg.Front.Bind(new IPEndPoint(IPAddress.Any, port));
                    leg.Back = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                    leg.Back.Bind(new IPEndPoint(IPAddress.Any, 0));
                    foreach (Socket s in new[] { leg.Front, leg.Back })
                    {
                        s.ReceiveBufferSize = 1 << 21;
                        s.SendBufferSize = 1 << 21;
                        IgnoreConnectionResets(s);
                    }

                    legs.Add(leg);
                }
            }
            catch (SocketException e)
            {
                Console.Error.WriteLine($"udp-relay: cannot listen ({e.SocketErrorCode}): is another relay, RobotHost or the editor using one of {ports}?");
                foreach (Leg l in legs)
                {
                    l.Front.Dispose();
                    l.Back.Dispose();
                }

                return 1;
            }

            Console.WriteLine($"[relay] {string.Join(", ", legs.Select(l => l.Port))} on this machine <-> {upstream}; point the headset here " +
                              "(just push-galaxy-config <this machine's address> true false). Enter stops it.");
            var stop = new ManualResetEventSlim();
            var input = new Thread(() =>
            {
                try
                {
                    if (Console.In.ReadLine() != null) stop.Set();
                }
                catch (Exception)
                {
                    // no console: --seconds or the process ending stops it
                }
            }) { IsBackground = true };
            input.Start();

            var buffer = new byte[65536];
            var readable = new List<Socket>();
            long freq = Stopwatch.Frequency, start = Stopwatch.GetTimestamp(), nextReport = start + 5 * freq;
            long end = seconds > 0 ? start + (long)(seconds * freq) : long.MaxValue;
            while (!stop.IsSet && Stopwatch.GetTimestamp() < end)
            {
                readable.Clear();
                foreach (Leg l in legs)
                {
                    readable.Add(l.Front);
                    readable.Add(l.Back);
                }

                Socket.Select(readable, null, null, 100_000);
                foreach (Socket s in readable)
                {
                    Leg l = legs.First(x => x.Front == s || x.Back == s);
                    while (s.Poll(0, SelectMode.SelectRead)) // Poll, not Available: an empty datagram reads as Available 0 and would block the queue (B-01)
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

                        if (s == l.Front)
                        {
                            if (l.Client == null || !l.Client.Equals(from)) Console.WriteLine($"[relay] {l.Port}: headset at {from}");
                            l.Client = from;
                            Send(l.Back, buffer, n, l.Upstream);
                            l.Up++;
                            l.UpBytes += n;
                        }
                        else if (l.Client != null)
                        {
                            Send(l.Front, buffer, n, l.Client);
                            l.Down++;
                            l.DownBytes += n;
                        }
                    }
                }

                long now = Stopwatch.GetTimestamp();
                if (now >= nextReport)
                {
                    double span = (now - nextReport + 5 * freq) / (double)freq;
                    Console.WriteLine("[relay] " + string.Join("; ", legs.Select(x =>
                        $"{x.Port}: {x.Up} up, {x.Down} down ({x.DownBytes * 8 / span / 1e6:0.0} Mbit/s)")));
                    foreach (Leg x in legs) x.Up = x.Down = x.UpBytes = x.DownBytes = 0;
                    nextReport = now + 5 * freq;
                }
            }

            foreach (Leg l in legs)
            {
                l.Front.Dispose();
                l.Back.Dispose();
            }

            Console.WriteLine("[relay] stopped");
            return 0;
        }

        private static void Send(Socket s, byte[] buffer, int n, EndPoint to)
        {
            try
            {
                s.SendTo(buffer, 0, n, SocketFlags.None, to);
            }
            catch (SocketException)
            {
                // the other side is gone for now; the next datagram tries again
            }
        }

        /// <summary>Windows reports an ICMP port-unreachable as a reset on the next receive; a relay must not stop for it.</summary>
        private static void IgnoreConnectionResets(Socket s)
        {
            if (!OperatingSystem.IsWindows()) return;
            const int SioUdpConnReset = -1744830452;
            s.IOControl(SioUdpConnReset, new byte[] { 0 }, null);
        }
    }
}
