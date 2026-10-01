using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using Teleop.Core.Camera;
using Teleop.Core.Types;

namespace Teleop.CameraHost
{
    /// <summary>
    /// A headless viewer for checking the sender over a real network: subscribes, keeps the
    /// subscription alive, reassembles frames with Core's <see cref="CameraFrameReassembler"/>, and
    /// reports what arrived. Runs anywhere .NET does, including the operator's Windows machine.
    ///
    /// It reports arrival-side figures only. Capture-to-arrival latency crosses two clocks and needs
    /// the pose path's ClockSync (ADR 0014 §5), which this tool does not run, so it does not print a
    /// number that would look like one.
    ///
    /// Exit 0 when at least one complete frame arrived, 1 when none did.
    /// </summary>
    internal static class ProbeCommand
    {
        public static int Run(string host, int port, int localPort, double seconds, ushort maxFps, string? savePath)
        {
            if (!IPAddress.TryParse(host, out IPAddress? address))
            {
                Console.Error.WriteLine($"error: --host must be an IP address, not '{host}'");
                return 2;
            }

            var remote = new IPEndPoint(address, port);
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.Bind(new IPEndPoint(IPAddress.Any, localPort));
            socket.ReceiveBufferSize = 4 << 20;

            var subscribe = new byte[CameraSubscribeCodec.EncodedSize];
            var subscribeCodec = new CameraSubscribeCodec();
            var reassembler = new CameraFrameReassembler(slotCount: 3, maxFrameBytes: 256 * 1024);
            var receive = new byte[CameraChunkCodec.MaxDatagramBytes];
            long tps = Stopwatch.Frequency;

            var arrivalIntervalsMs = new List<double>();
            var sizes = new List<double>();
            long previousArrival = 0, unstamped = 0, datagrams = 0;
            uint firstId = 0, lastId = 0;
            byte[]? lastJpeg = null;
            int frames = 0;
            CameraFrameStamp lastStamp = default;

            Console.WriteLine($"Subscribing to {remote} from UDP :{localPort} for {seconds:0.#} s" +
                (maxFps > 0 ? $" at up to {maxFps} fps" : string.Empty) + "...");

            long start = Stopwatch.GetTimestamp();
            long end = start + (long)(seconds * tps);
            long nextSubscribe = start;
            while (Stopwatch.GetTimestamp() < end)
            {
                long now = Stopwatch.GetTimestamp();
                if (now >= nextSubscribe)
                {
                    subscribeCodec.TryEncode(maxFps, now, subscribe, out int written);
                    socket.SendTo(subscribe.AsSpan(0, written), SocketFlags.None, remote);
                    nextSubscribe = now + tps;
                }

                if (!socket.Poll(10_000, SelectMode.SelectRead))
                {
                    continue;
                }

                while (socket.Available > 0)
                {
                    int n = socket.Receive(receive);
                    long arrival = Stopwatch.GetTimestamp(); // at dequeue, docs/metrics.md §1
                    datagrams++;
                    if (reassembler.Accept(receive.AsSpan(0, n), arrival) != CameraChunkOutcome.CompletedFrame)
                    {
                        continue;
                    }

                    if (!reassembler.TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> jpeg))
                    {
                        continue;
                    }

                    if (frames == 0)
                    {
                        firstId = info.Frame.FrameId;
                    }
                    else
                    {
                        arrivalIntervalsMs.Add((info.LastChunkArrivalTicks - previousArrival) * 1000.0 / tps);
                    }

                    frames++;
                    lastId = info.Frame.FrameId;
                    lastStamp = info.Frame;
                    previousArrival = info.LastChunkArrivalTicks;
                    sizes.Add(info.ByteCount);
                    if (info.Frame.CaptureTicks == LatencyTrace.Unset)
                    {
                        unstamped++;
                    }

                    lastJpeg = jpeg.ToArray();
                }
            }

            CameraReassemblerDiagnostics d = reassembler.Diagnostics;
            double elapsed = (Stopwatch.GetTimestamp() - start) / (double)tps;
            Console.WriteLine($"== received {datagrams} datagrams, {frames} complete frames in {elapsed:0.0} s ({frames / elapsed:0.0} fps)");
            if (frames > 0)
            {
                Console.WriteLine($"   frame ids {firstId}..{lastId}; {lastStamp.Width}x{lastStamp.Height}; sender clock {lastStamp.TicksPerSecond} ticks/s");
                Console.WriteLine($"   capture stamp present on {frames - unstamped}/{frames} frames");
                Console.WriteLine($"   interval between frame arrivals, ms: {Percentiles(arrivalIntervalsMs)}");
                Console.WriteLine($"   bytes per frame:                     {Percentiles(sizes)}");
            }

            Console.WriteLine(
                $"   reassembler: completed {d.FramesCompleted}, superseded {d.DroppedSuperseded}, incomplete {d.DroppedIncomplete}, " +
                $"late {d.LateChunks}, duplicate {d.DuplicateChunks}, malformed {d.MalformedDatagrams}, inconsistent {d.InconsistentChunks}");
            Console.WriteLine("   (no capture-to-arrival latency here: that crosses clocks and needs the pose path's ClockSync, ADR 0014 §5)");

            if (lastJpeg != null && !string.IsNullOrEmpty(savePath))
            {
                File.WriteAllBytes(savePath, lastJpeg);
                Console.WriteLine($"   last frame saved to {Path.GetFullPath(savePath)}");
            }

            if (frames == 0)
            {
                Console.Error.WriteLine(datagrams == 0
                    ? "FAIL: nothing arrived. Is the sender running, and does this machine's firewall allow inbound UDP on this port?"
                    : "FAIL: datagrams arrived but no frame completed.");
                return 1;
            }

            return 0;
        }

        private static string Percentiles(List<double> values)
        {
            if (values.Count == 0)
            {
                return "n/a";
            }

            var sorted = values.OrderBy(v => v).ToArray();
            double At(double p) => sorted[Math.Min(sorted.Length - 1, (int)Math.Ceiling(p * sorted.Length) - 1)];
            return $"p50 {At(0.50):0.0}  p95 {At(0.95):0.0}  max {sorted[^1]:0.0}";
        }
    }
}
