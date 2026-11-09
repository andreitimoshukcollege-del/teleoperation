using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Text.Json;
using Teleop.RobotArm.Tests.Kinematics;
using Teleop.RobotArm.Wire;
using Xunit;

namespace Teleop.RobotArm.Tests.Wire
{
    /// <summary>
    /// The headset's link trace (docs/adr/0035 N1), and the golden file that pins the trainer's reader to it
    /// (trainer/contract/golden/linktrace-v1.json, read by trainer/tests/test_linktrace.py). Rewrite the golden after a deliberate
    /// change with <c>TELEOP_WRITE_GOLDEN=1</c>, and change trainer/armpred/net/linktrace.py with it.
    /// </summary>
    public sealed class LinkTraceCodecTests
    {
        [Fact]
        public void EveryKind_RoundTrips_AndOthersAreRefused()
        {
            var b = new byte[LinkTraceCodec.RecordBytes];
            foreach (LinkTraceKind kind in new[] { LinkTraceKind.Frame, LinkTraceKind.RoundTrip, LinkTraceKind.Conditions })
            {
                Assert.True(LinkTraceCodec.TryEncode(kind, 77, -1, long.MaxValue, long.MinValue, 123456789012, 0, b));
                Assert.True(LinkTraceCodec.TryDecode(b, out LinkTraceKind k, out uint seq, out long a, out long bb, out long c, out long d, out long e));
                Assert.Equal((kind, 77u, -1L, long.MaxValue, long.MinValue, 123456789012L, 0L), (k, seq, a, bb, c, d, e));
            }

            Assert.False(LinkTraceCodec.TryEncode((LinkTraceKind)9, 0, 0, 0, 0, 0, 0, b));
            b[0] = 9;
            Assert.False(LinkTraceCodec.TryDecode(b, out _, out _, out _, out _, out _, out _, out _));
            Assert.False(LinkTraceCodec.TryDecode(b.AsSpan(0, 47), out _, out _, out _, out _, out _, out _, out _));
        }

        [Fact]
        public void Encoding_AllocatesNothing()
        {
            var b = new byte[LinkTraceCodec.RecordBytes];
            LinkTraceCodec.TryEncode(LinkTraceKind.Frame, 1, 2, 3, 4, 5, 6, b);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (uint i = 0; i < 1000; i++) LinkTraceCodec.TryEncode(LinkTraceKind.Frame, i, i, 1_000_000_000, i * 20_000L, i * 20_000L - 9, 0, b);
            Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
        }

        [Fact]
        public void TheTrainersGoldenFile_IsWhatThisCodecWrites()
        {
            string expected = Golden();
            string path = Path.Combine(SerialChainKinematicsTests.RepoRoot(), "trainer", "contract", "golden", "linktrace-v1.json");
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1") File.WriteAllText(path, expected, new UTF8Encoding(false));
            Assert.True(File.Exists(path), $"{path} is missing: TELEOP_WRITE_GOLDEN=1 dotnet test Teleop.RobotArm.Tests --filter LinkTraceCodecTests");
            Assert.Equal(expected, File.ReadAllText(path).Replace("\r\n", "\n"));
        }

        private static string Golden()
        {
            var records = new List<Dictionary<string, object>>();
            var b = new byte[LinkTraceCodec.RecordBytes];
            void Add(LinkTraceKind kind, uint seq, long a, long bb, long c, long d, long e)
            {
                Assert.True(LinkTraceCodec.TryEncode(kind, seq, a, bb, c, d, e, b));
                records.Add(new Dictionary<string, object>
                {
                    ["kind"] = (int)kind, ["sequence"] = seq, ["a"] = a, ["b"] = bb, ["c"] = c, ["d"] = d, ["e"] = e,
                    ["hex"] = Convert.ToHexString(b).ToLowerInvariant(),
                });
            }

            // A frame before the clocks synced, a round trip, a frame after, a disturbance switched on.
            Add(LinkTraceKind.Frame, 1001, 2_056_853_965_299, 1_000_000_000, 229_471_511_411, long.MinValue, 2_056_853_945_000);
            Add(LinkTraceKind.RoundTrip, 0, 229_471_400_000, 2_056_853_900_000, 2_056_853_965_299, 1_000_000_000, 229_471_511_411);
            Add(LinkTraceKind.Frame, 1002, 2_056_873_965_299, 1_000_000_000, 229_471_713_000, 229_471_610_000, 0);
            Add(LinkTraceKind.Conditions, 0, 150_000, 20_000, 5_000, 0, 229_472_000_000);
            var doc = new Dictionary<string, object>
            {
                ["about"] = "Link trace v1 (docs/adr/0035 N1): 48-byte records and their fields, written by LinkTraceCodecTests.",
                ["recordBytes"] = LinkTraceCodec.RecordBytes,
                ["records"] = records,
            };
            return JsonSerializer.Serialize(doc, new JsonSerializerOptions { WriteIndented = true }).Replace("\r\n", "\n") + "\n";
        }
    }
}
