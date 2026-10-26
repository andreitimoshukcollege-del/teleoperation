using System.ComponentModel;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using Teleop.CameraHost.Viewing;
using Teleop.Core.Camera;

namespace Teleop.CameraHost
{
    /// <summary>
    /// A live window on the robot's camera, on the operator's machine: subscribes as <see cref="ProbeCommand"/>
    /// does, reassembles with Core's <see cref="CameraFrameReassembler"/>, and hands each completed frame
    /// to <see cref="MjpegServer"/>, which a browser window shows. The camera's JPEGs go through untouched.
    ///
    /// The sender streams to its newest subscriber only (docs/adr/0014 §2), so while this runs it takes
    /// the feed from the headset, and the two trade it every second if both are open.
    ///
    /// Stops on Enter (Ctrl+C from WSL does not reach a Windows dotnet), Ctrl+C, or after --seconds.
    /// </summary>
    internal static class ViewCommand
    {
        public static int Run(string host, int port, int localPort, double seconds, ushort maxFps, int httpPort, bool open)
        {
            if (!IPAddress.TryParse(host, out IPAddress? address))
            {
                Console.Error.WriteLine($"error: --host must be an IP address, not '{host}'");
                return 2;
            }

            var remote = new IPEndPoint(address, port);
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            MjpegServer server;
            try
            {
                socket.Bind(new IPEndPoint(IPAddress.Any, localPort));
                server = new MjpegServer(httpPort, $"JetRover camera ({host})");
            }
            catch (SocketException e)
            {
                Console.Error.WriteLine($"error: could not open UDP :{localPort} or http :{httpPort} ({e.SocketErrorCode}); is another viewer or camera-probe running?");
                return 2;
            }

            using (server)
            {
                socket.ReceiveBufferSize = 4 << 20;

                using var stop = new ManualResetEventSlim(false);
                Console.CancelKeyPress += (_, e) =>
                {
                    e.Cancel = true;
                    stop.Set();
                };
                new Thread(() =>
                {
                    if (Console.In.ReadLine() != null) stop.Set();
                }) { IsBackground = true }.Start();

                Console.WriteLine($"Subscribing to {remote} from UDP :{localPort}; the window is {server.Url}");
                Console.WriteLine(seconds > 0 ? $"Runs for {seconds:0.#} s, or press Enter to stop." : "Press Enter to stop.");
                if (open)
                {
                    OpenWindow(server.Url);
                }

                var subscribe = new byte[CameraSubscribeCodec.EncodedSize];
                var subscribeCodec = new CameraSubscribeCodec();
                var reassembler = new CameraFrameReassembler(slotCount: 3, maxFrameBytes: 256 * 1024);
                var receive = new byte[CameraChunkCodec.MaxDatagramBytes];
                long tps = Stopwatch.Frequency;
                long start = Stopwatch.GetTimestamp();
                long end = seconds > 0 ? start + (long)(seconds * tps) : long.MaxValue;
                long nextSubscribe = start, nextStats = start + tps, lastFrameAt = 0;
                int frames = 0, framesThisSecond = 0;
                long bytesThisSecond = 0;
                ushort width = 0, height = 0;

                while (!stop.IsSet && Stopwatch.GetTimestamp() < end)
                {
                    long now = Stopwatch.GetTimestamp();
                    if (now >= nextSubscribe)
                    {
                        subscribeCodec.TryEncode(maxFps, now, subscribe, out int written);
                        try
                        {
                            socket.SendTo(subscribe.AsSpan(0, written), SocketFlags.None, remote);
                        }
                        catch (SocketException)
                        {
                            // No route right now (Wi-Fi roaming): the next keepalive tries again.
                        }

                        nextSubscribe = now + tps;
                    }

                    if (now >= nextStats)
                    {
                        server.SetStats(Stats(host, frames, framesThisSecond, bytesThisSecond, width, height,
                            lastFrameAt == 0 ? -1 : (now - lastFrameAt) / (double)tps, reassembler.Diagnostics));
                        framesThisSecond = 0;
                        bytesThisSecond = 0;
                        nextStats = now + tps;
                    }

                    if (!socket.Poll(10_000, SelectMode.SelectRead))
                    {
                        continue;
                    }

                    while (socket.Available > 0)
                    {
                        int n;
                        try
                        {
                            n = socket.Receive(receive);
                        }
                        catch (SocketException)
                        {
                            break; // Windows reports an ICMP port-unreachable this way: the sender is not up yet.
                        }

                        long arrival = Stopwatch.GetTimestamp();
                        if (reassembler.Accept(receive.AsSpan(0, n), arrival) != CameraChunkOutcome.CompletedFrame ||
                            !reassembler.TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> jpeg))
                        {
                            continue;
                        }

                        server.Publish(jpeg.ToArray());
                        frames++;
                        framesThisSecond++;
                        bytesThisSecond += info.ByteCount;
                        width = info.Frame.Width;
                        height = info.Frame.Height;
                        lastFrameAt = arrival;
                    }
                }

                CameraReassemblerDiagnostics d = reassembler.Diagnostics;
                Console.WriteLine($"Stopped: {frames} frames shown; dropped {d.DroppedIncomplete} incomplete, {d.DroppedSuperseded} superseded.");
                if (frames == 0)
                {
                    Console.Error.WriteLine("No frame arrived. Is the camera sender running (just camera-logs), and does this machine's firewall allow inbound UDP on " +
                                            $"{localPort}?");
                    return 1;
                }

                return 0;
            }
        }

        private static string Stats(string host, int frames, int fps, long bytes, ushort width, ushort height, double sinceFrame,
            CameraReassemblerDiagnostics d)
        {
            if (frames == 0)
            {
                return $"waiting for the camera at {host}...";
            }

            string stale = sinceFrame > 2 ? $" | no frame for {sinceFrame:0} s" : string.Empty;
            string size = fps > 0 ? $" | {bytes / fps / 1024.0:0} KB/frame" : string.Empty;
            return $"{host} | {width}x{height} | {fps} fps{size} | dropped {d.DroppedIncomplete}{stale}";
        }

        /// <summary>
        /// A window without browser chrome where Edge or Chrome can make one (--app), otherwise the
        /// default browser. A failure only means opening the URL by hand.
        /// </summary>
        private static void OpenWindow(string url)
        {
            if (OperatingSystem.IsWindows())
            {
                foreach (string browser in new[] { "msedge", "chrome" })
                {
                    try
                    {
                        Process.Start(new ProcessStartInfo(browser, $"--app={url} --window-size=960,580") { UseShellExecute = true });
                        return;
                    }
                    catch (Win32Exception)
                    {
                    }
                }
            }

            try
            {
                Process.Start(OperatingSystem.IsLinux()
                    ? new ProcessStartInfo("xdg-open", url)
                    : new ProcessStartInfo(url) { UseShellExecute = true });
            }
            catch (Exception e) when (e is Win32Exception || e is InvalidOperationException)
            {
                Console.WriteLine($"Could not open a window ({e.Message}); open {url} in a browser.");
            }
        }
    }
}
