using System.Diagnostics;
using Teleop.CameraHost.V4l2;

namespace Teleop.CameraHost
{
    /// <summary>
    /// The first implementation step of docs/adr/0014-camera-frame-downlink.md (resolved question 1):
    /// prove that the driver's per-frame capture timestamp is reachable from .NET and lives in the same
    /// clock domain as <see cref="Stopwatch"/>, which is what <c>Teleop.RobotHost</c> stamps with.
    ///
    /// Exit codes: 0 when every check passes, 1 when a check fails, 2 when capture could not start.
    /// A check that cannot be evaluated counts as failed (root CLAUDE.md invariant 10).
    /// </summary>
    internal static class TimestampSpike
    {
        private const int WarmUpFrames = 5;
        private const long MaxPlausibleAgeNanoseconds = 1_000_000_000L;

        public static int Run(string device, uint width, uint height, uint fps, int frames)
        {
            Console.WriteLine($"== clock: Stopwatch.Frequency={Stopwatch.Frequency}, IsHighResolution={Stopwatch.IsHighResolution}");
            bool clockOk = CheckStopwatchIsClockMonotonic(out string clockDetail);
            Console.WriteLine($"   {clockDetail}");

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
            {
                Console.WriteLine($"== device: {device} driver={capture.Driver} card=\"{capture.Card}\" " +
                    $"{capture.Width}x{capture.Height} MJPEG at {capture.FramesPerSecondNumerator}/{capture.FramesPerSecondDenominator} fps");

                var ages = new List<double>();
                var intervals = new List<double>();
                var sizes = new List<double>();
                var timestampTypes = new Dictionary<string, int>();
                var timestampSources = new Dictionary<string, int>();
                int received = 0, sequenceGaps = 0, implausibleAges = 0, missingDht = 0, notJpeg = 0;
                long previousTimestamp = 0;
                uint previousSequence = 0;

                Console.WriteLine("== frames (first 10): seq  bytes  flags       age_ms  interval_ms  dht");
                for (int i = 0; i < frames; i++)
                {
                    if (!capture.TryDequeue(2000, out CapturedFrame frame))
                    {
                        Console.Error.WriteLine($"error: no frame within 2 s after {received} frames");
                        break;
                    }

                    long nowNanoseconds = StopwatchNanoseconds(Stopwatch.GetTimestamp());
                    ReadOnlySpan<byte> jpeg = capture.FrameBytes(frame);
                    bool isJpeg = jpeg.Length >= 4 && jpeg[0] == 0xFF && jpeg[1] == 0xD8;
                    bool hasDht = isJpeg && HasHuffmanTables(jpeg);
                    capture.Requeue(frame);

                    received++;
                    Count(timestampTypes, TimestampType(frame.Flags));
                    Count(timestampSources, TimestampSource(frame.Flags));
                    double ageMs = (nowNanoseconds - frame.TimestampNanoseconds) / 1e6;
                    double intervalMs = previousTimestamp == 0 ? double.NaN : (frame.TimestampNanoseconds - previousTimestamp) / 1e6;
                    if (previousTimestamp != 0 && frame.Sequence != previousSequence + 1)
                    {
                        sequenceGaps++;
                    }

                    if (i < 10)
                    {
                        Console.WriteLine($"   {frame.Sequence,5} {frame.BytesUsed,6}  0x{frame.Flags:x8} {ageMs,8:0.00} {intervalMs,12:0.00}  {(hasDht ? "yes" : "no")}");
                    }

                    if (i >= WarmUpFrames)
                    {
                        ages.Add(ageMs);
                        if (!double.IsNaN(intervalMs))
                        {
                            intervals.Add(intervalMs);
                        }

                        sizes.Add(frame.BytesUsed);
                        if (ageMs <= 0 || ageMs * 1e6 > MaxPlausibleAgeNanoseconds)
                        {
                            implausibleAges++;
                        }
                    }

                    if (!isJpeg)
                    {
                        notJpeg++;
                    }
                    else if (!hasDht)
                    {
                        missingDht++;
                    }

                    previousTimestamp = frame.TimestampNanoseconds;
                    previousSequence = frame.Sequence;
                }

                Console.WriteLine($"== summary over {ages.Count} frames after {WarmUpFrames} warm-up ({received}/{frames} received)");
                Console.WriteLine($"   timestamp type:   {Describe(timestampTypes)}");
                Console.WriteLine($"   timestamp source: {Describe(timestampSources)}");
                Console.WriteLine($"   age at dequeue (Stopwatch now - driver stamp), ms: {Percentiles(ages)}");
                Console.WriteLine($"   interval between driver stamps, ms:               {Percentiles(intervals)}  (expect {1000.0 / fps:0.0})");
                Console.WriteLine($"   bytes per frame:                                  {Percentiles(sizes)}");
                Console.WriteLine($"   sequence gaps (driver-dropped frames): {sequenceGaps}");
                Console.WriteLine($"   frames without Huffman tables (DHT): {missingDht}, not JPEG at all: {notJpeg}");

                var failures = new List<string>();
                if (!clockOk) failures.Add("Stopwatch does not read CLOCK_MONOTONIC on this machine");
                if (received < frames * 9 / 10) failures.Add($"only {received}/{frames} frames arrived");
                if (timestampTypes.Count != 1 || !timestampTypes.ContainsKey("monotonic")) failures.Add("not every frame carried a CLOCK_MONOTONIC timestamp");
                if (ages.Count == 0) failures.Add("no frames left after warm-up to evaluate");
                if (implausibleAges > 0) failures.Add($"{implausibleAges} frames had an age <= 0 or > 1 s, so the stamp is not on Stopwatch's clock");
                if (notJpeg > 0) failures.Add($"{notJpeg} frames were not JPEG");

                if (failures.Count == 0)
                {
                    Console.WriteLine("PASS: driver capture timestamps are CLOCK_MONOTONIC and agree with Stopwatch (docs/adr/0014 §4).");
                    return 0;
                }

                foreach (string failure in failures)
                {
                    Console.Error.WriteLine($"FAIL: {failure}");
                }

                return 1;
            }
        }

        /// <summary>
        /// Brackets a direct <c>clock_gettime(CLOCK_MONOTONIC)</c> between two <see cref="Stopwatch"/>
        /// reads. If Stopwatch is that clock, the direct read falls between them every time.
        /// </summary>
        private static unsafe bool CheckStopwatchIsClockMonotonic(out string detail)
        {
            int outside = 0;
            long worstNanoseconds = 0;
            const int samples = 1000;
            for (int i = 0; i < samples; i++)
            {
                long before = StopwatchNanoseconds(Stopwatch.GetTimestamp());
                Timespec now;
                if (V4l2Native.clock_gettime(V4l2Native.ClockMonotonic, &now) != 0)
                {
                    detail = "clock_gettime(CLOCK_MONOTONIC) failed";
                    return false;
                }

                long after = StopwatchNanoseconds(Stopwatch.GetTimestamp());
                long monotonic = (now.Seconds * 1_000_000_000L) + now.Nanoseconds;
                if (monotonic < before || monotonic > after)
                {
                    outside++;
                    worstNanoseconds = Math.Max(worstNanoseconds, Math.Max(before - monotonic, monotonic - after));
                }
            }

            detail = outside == 0
                ? $"CLOCK_MONOTONIC fell between two Stopwatch reads in all {samples} samples"
                : $"CLOCK_MONOTONIC fell outside the Stopwatch bracket in {outside}/{samples} samples, worst by {worstNanoseconds} ns";
            return outside == 0;
        }

        private static long StopwatchNanoseconds(long ticks) =>
            Stopwatch.Frequency == 1_000_000_000L ? ticks : (long)(ticks * (1e9 / Stopwatch.Frequency));

        /// <summary>Walks JPEG marker segments up to start-of-scan, looking for a DHT segment.</summary>
        internal static bool HasHuffmanTables(ReadOnlySpan<byte> jpeg)
        {
            int i = 2;
            while (i + 4 <= jpeg.Length && jpeg[i] == 0xFF)
            {
                byte marker = jpeg[i + 1];
                if (marker == 0xDA)
                {
                    return false;
                }

                if (marker == 0xC4)
                {
                    return true;
                }

                int segmentLength = (jpeg[i + 2] << 8) | jpeg[i + 3];
                i += 2 + segmentLength;
            }

            return false;
        }

        private static string TimestampType(uint flags) => (flags & V4l2Native.BufFlagTimestampMask) switch
        {
            V4l2Native.BufFlagTimestampMonotonic => "monotonic",
            V4l2Native.BufFlagTimestampCopy => "copy",
            V4l2Native.BufFlagTimestampUnknown => "unknown",
            _ => "other",
        };

        private static string TimestampSource(uint flags) => (flags & V4l2Native.BufFlagTstampSrcMask) switch
        {
            V4l2Native.BufFlagTstampSrcEof => "end-of-frame",
            V4l2Native.BufFlagTstampSrcSoe => "start-of-exposure",
            _ => "other",
        };

        private static void Count(Dictionary<string, int> counts, string key) =>
            counts[key] = counts.TryGetValue(key, out int n) ? n + 1 : 1;

        private static string Describe(Dictionary<string, int> counts) =>
            string.Join(", ", counts.Select(kv => $"{kv.Key} x{kv.Value}"));

        private static string Percentiles(List<double> values)
        {
            if (values.Count == 0)
            {
                return "n/a";
            }

            var sorted = values.OrderBy(v => v).ToArray();
            double At(double p) => sorted[Math.Min(sorted.Length - 1, (int)Math.Ceiling(p * sorted.Length) - 1)];
            return $"p50 {At(0.50):0.00}  p95 {At(0.95):0.00}  max {sorted[^1]:0.00}";
        }
    }
}
