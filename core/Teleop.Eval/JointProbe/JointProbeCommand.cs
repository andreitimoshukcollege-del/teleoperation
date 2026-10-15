using System;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using Teleop.Core.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointProbe
{
    /// <summary>
    /// <c>joint-probe</c>: subscribes to a RobotHost's joint-state stream (docs/adr/0018) for a few
    /// seconds and reports what it carries: the frame rate, the round trip, and per joint whether it
    /// is <i>sensed</i> (read from the servo) or a commanded belief, with its value range. Sends only
    /// the keepalive, never a command, so nothing moves.
    /// </summary>
    public static class JointProbeCommand
    {
        public static int Run(string[] args)
        {
            string? host = null;
            int port = 6002;
            double seconds = 3;
            for (int i = 1; i < args.Length; i++)
            {
                string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                switch (args[i])
                {
                    case "--remote-host": host = Next(); break;
                    case "--remote-port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--seconds": seconds = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                    default:
                        Console.Error.WriteLine("usage: joint-probe --remote-host <ip> [--remote-port 6002] [--seconds 3]");
                        return 64;
                }
            }

            if (host == null || !IPAddress.TryParse(host, out IPAddress? address))
            {
                Console.Error.WriteLine("joint-probe: --remote-host <ip> is required");
                return 64;
            }

            var robot = new IPEndPoint(address, port);
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.Bind(new IPEndPoint(IPAddress.Any, 0));
            socket.ReceiveTimeout = 100;

            var buffer = new byte[256];
            var joints = new JointSample[JointStateCodec.MaxJointsPerMessage];
            var seen = new System.Collections.Generic.SortedDictionary<byte, (int frames, int sensed, float min, float max, float last)>();
            long frequency = Stopwatch.Frequency, start = Stopwatch.GetTimestamp(), nextKeepalive = 0;
            long end = start + (long)(seconds * frequency);
            int frames = 0;
            double minRttMs = double.MaxValue;
            long lastEcho = long.MinValue;

            while (Stopwatch.GetTimestamp() < end)
            {
                long now = Stopwatch.GetTimestamp();
                if (now >= nextKeepalive)
                {
                    JointStateCodec.TryEncodeSubscribe(now, frequency, buffer, out int n);
                    socket.SendTo(buffer, 0, n, SocketFlags.None, robot);
                    nextKeepalive = now + frequency / 2;
                }

                int received;
                try
                {
                    received = socket.Receive(buffer);
                }
                catch (SocketException)
                {
                    continue;
                }

                long arrival = Stopwatch.GetTimestamp();
                if (!JointStateCodec.TryDecode(buffer.AsSpan(0, received), out _, out long robotSend, out _, out long echoOp,
                        out long echoRecv, out long robotTps, joints, out int count))
                {
                    continue;
                }

                frames++;
                if (echoOp != lastEcho)
                {
                    lastEcho = echoOp;
                    double held = (robotSend - echoRecv) / (double)robotTps;
                    double rtt = (arrival - echoOp) / (double)frequency - held;
                    minRttMs = Math.Min(minRttMs, rtt * 1000);
                }

                for (int i = 0; i < count; i++)
                {
                    JointSample j = joints[i];
                    seen.TryGetValue(j.MotorId, out var s);
                    seen[j.MotorId] = s.frames == 0
                        ? (1, j.Sensed ? 1 : 0, j.Value, j.Value, j.Value)
                        : (s.frames + 1, s.sensed + (j.Sensed ? 1 : 0), Math.Min(s.min, j.Value), Math.Max(s.max, j.Value), j.Value);
                }
            }

            double elapsed = (Stopwatch.GetTimestamp() - start) / (double)frequency;
            if (frames == 0)
            {
                Console.WriteLine($"joint-probe: no joint-state frames from {robot} in {elapsed:0.0} s. Is RobotHost new enough (ADR 0018) and started with --joint-local-port?");
                return 1;
            }

            Console.WriteLine($"joint-probe: {frames} frames from {robot} in {elapsed:0.0} s ({frames / elapsed:0.0} Hz), min round trip {minRttMs:0.0} ms");
            Console.WriteLine($"{"motor",6} {"sensed",8} {"last",9} {"min",9} {"max",9}");
            foreach (var (id, s) in seen)
            {
                Console.WriteLine($"{id,6} {(s.sensed == s.frames ? "yes" : s.sensed == 0 ? "NO" : $"{s.sensed}/{s.frames}"),8} {s.last,9:0.####} {s.min,9:0.####} {s.max,9:0.####}");
            }

            Console.WriteLine("sensed NO = no servo feedback for that motor: its value is what RobotHost last commanded, not a measurement.");
            return 0;
        }
    }
}
