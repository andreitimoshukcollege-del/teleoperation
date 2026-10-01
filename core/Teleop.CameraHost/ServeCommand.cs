using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using Teleop.CameraHost.Streaming;
using Teleop.CameraHost.V4l2;
using Teleop.Core.Types;

namespace Teleop.CameraHost
{
    /// <summary>
    /// The streaming sender of docs/adr/0014-camera-frame-downlink.md: captures the camera's own
    /// MJPEG frames, stamps each with the driver's capture time, and streams them in chunks to
    /// whoever last sent a keepalive to the listening port. Captures continuously so the camera stays
    /// warm (its first frame after opening takes about half a second), and sends only while
    /// subscribed. Runs until stopped (Ctrl+C or SIGTERM), or for <c>--seconds</c>.
    /// </summary>
    internal static class ServeCommand
    {
        private const int StatsIntervalSeconds = 10;

        public static int Run(string device, uint width, uint height, uint fps, int port, double subscriberTimeoutSeconds, double seconds)
        {
            V4l2Capture capture;
            try
            {
                capture = new V4l2Capture(device, width, height, fps);
            }
            catch (Exception e)
            {
                Console.Error.WriteLine($"error: could not start capture on {device}: {e.Message}");
                return 2;
            }

            using (capture)
            using (var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
            using (var stop = new ManualResetEventSlim(false))
            {
                socket.Bind(new IPEndPoint(IPAddress.Any, port));
                socket.Blocking = false;
                // One 640x480 frame is about 37 chunks sent back to back; give the kernel room for a few.
                socket.SendBufferSize = 1 << 20;

                Console.CancelKeyPress += (_, e) => { e.Cancel = true; stop.Set(); };
                using PosixSignalRegistration term = PosixSignalRegistration.Create(PosixSignal.SIGTERM, ctx => { ctx.Cancel = true; stop.Set(); });

                long ticksPerSecond = Stopwatch.Frequency;
                // Milliseconds since boot: a restarted sender starts ahead of every id it sent before,
                // so a viewer's newest-wins reassembler never sees it as old (Teleop.Core/Camera/CLAUDE.md).
                uint firstFrameId = unchecked((uint)(Stopwatch.GetTimestamp() / (ticksPerSecond / 1000)));
                var streamer = new CameraStreamer(
                    new SocketSender(socket), Stopwatch.GetTimestamp, ticksPerSecond,
                    (long)(subscriberTimeoutSeconds * ticksPerSecond), firstFrameId);

                Console.WriteLine(
                    $"Teleop.CameraHost serving {device} ({capture.Card}) {capture.Width}x{capture.Height} MJPEG at " +
                    $"{capture.FramesPerSecondNumerator}/{capture.FramesPerSecondDenominator} fps on UDP :{port}; " +
                    $"subscribers time out after {subscriberTimeoutSeconds:0.#} s; first frame id {firstFrameId}.");

                var receive = new byte[64];
                EndPoint from = new IPEndPoint(IPAddress.Any, 0);
                long started = Stopwatch.GetTimestamp();
                long nextStats = started + (StatsIntervalSeconds * ticksPerSecond);
                long deadline = seconds > 0 ? started + (long)(seconds * ticksPerSecond) : long.MaxValue;
                long captured = 0, unstamped = 0;

                while (!stop.IsSet && Stopwatch.GetTimestamp() < deadline)
                {
                    while (socket.Available > 0)
                    {
                        int n;
                        try
                        {
                            n = socket.ReceiveFrom(receive, ref from);
                        }
                        catch (SocketException)
                        {
                            break;
                        }

                        if (streamer.HandleDatagram(receive.AsSpan(0, n), from, Stopwatch.GetTimestamp()))
                        {
                            Console.WriteLine($"[net] streaming to {from}");
                        }
                    }

                    if (!capture.TryDequeue(100, out CapturedFrame frame))
                    {
                        continue;
                    }

                    captured++;
                    long captureTicks = CaptureTicks(frame, ticksPerSecond);
                    if (captureTicks == LatencyTrace.Unset)
                    {
                        unstamped++;
                    }

                    streamer.OfferFrame(
                        capture.FrameBytes(frame), captureTicks, (ushort)capture.Width, (ushort)capture.Height,
                        Stopwatch.GetTimestamp());
                    capture.Requeue(frame);

                    long now = Stopwatch.GetTimestamp();
                    if (now >= nextStats)
                    {
                        PrintStats(streamer, captured, unstamped);
                        nextStats = now + (StatsIntervalSeconds * ticksPerSecond);
                    }
                }

                PrintStats(streamer, captured, unstamped);
                return 0;
            }
        }

        /// <summary>
        /// The driver stamp, in Stopwatch ticks, if and only if the driver says it is
        /// CLOCK_MONOTONIC (the clock Stopwatch reads on Linux, per the timestamp spike); otherwise
        /// unset, never a substitute (ADR 0014 §4).
        /// </summary>
        private static long CaptureTicks(in CapturedFrame frame, long ticksPerSecond)
        {
            if ((frame.Flags & V4l2Native.BufFlagTimestampMask) != V4l2Native.BufFlagTimestampMonotonic)
            {
                return LatencyTrace.Unset;
            }

            return ticksPerSecond == 1_000_000_000L
                ? frame.TimestampNanoseconds
                : (long)(frame.TimestampNanoseconds * (ticksPerSecond / 1e9));
        }

        private static void PrintStats(CameraStreamer s, long captured, long unstamped) =>
            Console.WriteLine(
                $"[stats] captured {captured} (unstamped {unstamped}); sent {s.FramesSent} frames, {s.ChunksSent} chunks, " +
                $"{s.BytesSent / 1024} KiB; no subscriber {s.FramesWithoutSubscriber}, paced {s.FramesPaced}, " +
                $"failed {s.FramesFailed}; keepalives {s.SubscribesAccepted}, ignored {s.DatagramsIgnored}; " +
                $"subscriber {s.LastSubscriber?.ToString() ?? "none yet"}");

        private sealed class SocketSender : IDatagramSender
        {
            private readonly Socket _socket;

            public SocketSender(Socket socket) => _socket = socket;

            public bool TrySend(ReadOnlySpan<byte> datagram, EndPoint target)
            {
                try
                {
                    return _socket.SendTo(datagram, SocketFlags.None, target) == datagram.Length;
                }
                catch (SocketException)
                {
                    return false;
                }
            }
        }
    }
}
