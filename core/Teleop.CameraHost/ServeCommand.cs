using System.Linq;
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

        /// <summary>How long a camera that delivers nothing is trusted before it is reopened.</summary>
        private const double StallSeconds = 3;

        /// <summary>How often a missing camera is looked for again.</summary>
        private const int RetryMilliseconds = 1000;

        public static int Run(string device, uint width, uint height, uint fps, int port, double subscriberTimeoutSeconds, double seconds)
        {
            // The camera is opened, and reopened, inside the loop: the DaBai drops off the USB bus now
            // and then and comes back, sometimes under another /dev/videoN (2026-10-12). Exiting on that,
            // and leaving it to systemd, ended with systemd giving up for good, so the sender waits for
            // the camera instead and keeps answering keepalives meanwhile.
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
                    $"Teleop.CameraHost on UDP :{port}, camera {device}; subscribers time out after " +
                    $"{subscriberTimeoutSeconds:0.#} s; first frame id {firstFrameId}.");

                var receive = new byte[64];
                EndPoint from = new IPEndPoint(IPAddress.Any, 0);
                long started = Stopwatch.GetTimestamp();
                long nextStats = started + (StatsIntervalSeconds * ticksPerSecond);
                long deadline = seconds > 0 ? started + (long)(seconds * ticksPerSecond) : long.MaxValue;
                long stallTicks = (long)(StallSeconds * ticksPerSecond);
                long captured = 0, unstamped = 0, reopened = 0;
                V4l2Capture? capture = null;
                long lastFrameTicks = 0, nextOpenTicks = 0;
                string? lastOpenError = null;

                try
                {
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

                        long now = Stopwatch.GetTimestamp();
                        if (now >= nextStats)
                        {
                            PrintStats(streamer, captured, unstamped, reopened, capture != null);
                            nextStats = now + (StatsIntervalSeconds * ticksPerSecond);
                        }

                        if (capture == null)
                        {
                            if (now < nextOpenTicks)
                            {
                                stop.Wait(50);
                                continue;
                            }

                            capture = TryOpen(device, width, height, fps, ref lastOpenError);
                            if (capture == null)
                            {
                                nextOpenTicks = now + (RetryMilliseconds * ticksPerSecond / 1000);
                                continue;
                            }

                            reopened++;
                            lastFrameTicks = now;
                        }

                        try
                        {
                            if (!capture.TryDequeue(100, out CapturedFrame frame))
                            {
                                if (Stopwatch.GetTimestamp() - lastFrameTicks > stallTicks)
                                {
                                    Console.WriteLine($"[camera] no frame for {StallSeconds:0} s; reopening");
                                    capture.Dispose();
                                    capture = null;
                                }

                                continue;
                            }

                            captured++;
                            lastFrameTicks = Stopwatch.GetTimestamp();
                            long captureTicks = CaptureTicks(frame, ticksPerSecond);
                            if (captureTicks == LatencyTrace.Unset)
                            {
                                unstamped++;
                            }

                            streamer.OfferFrame(
                                capture.FrameBytes(frame), captureTicks, (ushort)capture.Width, (ushort)capture.Height,
                                Stopwatch.GetTimestamp());
                            capture.Requeue(frame);
                        }
                        catch (Exception e)
                        {
                            // Typically ENODEV: the camera left the USB bus. Wait for it to come back.
                            Console.WriteLine($"[camera] lost the camera ({e.Message}); waiting for it to come back");
                            try { capture?.Dispose(); } catch (Exception) { }
                            capture = null;
                            nextOpenTicks = Stopwatch.GetTimestamp() + (RetryMilliseconds * ticksPerSecond / 1000);
                        }
                    }
                }
                finally
                {
                    capture?.Dispose();
                }

                PrintStats(streamer, captured, unstamped, reopened, capture != null);
                return 0;
            }
        }

        /// <summary>
        /// Opens the camera, resolving <c>auto</c> to the DaBai's RGB camera by its stable
        /// <c>/dev/v4l/by-id</c> name, which follows it to whatever /dev/videoN it gets. A failure
        /// is logged once per distinct message, not once a second.
        /// </summary>
        private static V4l2Capture? TryOpen(string device, uint width, uint height, uint fps, ref string? lastError)
        {
            string? path = device == "auto" ? FindCamera() : device;
            string error;
            if (path == null)
            {
                error = "no Orbbec DaBai RGB camera under /dev/v4l/by-id (unplugged, or still re-enumerating)";
            }
            else
            {
                try
                {
                    var capture = new V4l2Capture(path, width, height, fps);
                    Console.WriteLine(
                        $"[camera] capturing {path} ({capture.Card}) {capture.Width}x{capture.Height} MJPEG at " +
                        $"{capture.FramesPerSecondNumerator}/{capture.FramesPerSecondDenominator} fps");
                    lastError = null;
                    return capture;
                }
                catch (Exception e)
                {
                    error = $"could not start capture on {path}: {e.Message}";
                }
            }

            if (error != lastError)
            {
                Console.WriteLine($"[camera] {error}; retrying every {RetryMilliseconds / 1000.0:0.#} s");
                lastError = error;
            }

            return null;
        }

        private static string? FindCamera()
        {
            const string byId = "/dev/v4l/by-id";
            if (!Directory.Exists(byId))
            {
                return null;
            }

            foreach (string link in Directory.GetFiles(byId).OrderBy(p => p, StringComparer.Ordinal))
            {
                string name = Path.GetFileName(link);
                if (name.Contains("DaBai", StringComparison.Ordinal) && name.Contains("RGB", StringComparison.Ordinal)
                    && name.EndsWith("-video-index0", StringComparison.Ordinal))
                {
                    return link;
                }
            }

            return null;
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

        private static void PrintStats(CameraStreamer s, long captured, long unstamped, long opened, bool cameraOpen) =>
            Console.WriteLine(
                $"[stats] camera {(cameraOpen ? "open" : "MISSING")}, opened {opened}x; captured {captured} (unstamped {unstamped}); sent {s.FramesSent} frames, {s.ChunksSent} chunks, " +
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
