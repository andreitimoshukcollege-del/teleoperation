using System.Buffers.Binary;
using System.Security.Cryptography;
using System.Text;
using Teleop.CameraHost.Recording;
using Teleop.Core.Types;

namespace Teleop.CameraHost.Tests.Recording
{
    /// <summary>
    /// <see cref="CameraSessionWriter"/>: the camera recorded into the session the control file names (docs/adr/0014
    /// amendment 3), in the format <c>trainer/contract/SESSION_FORMAT.md</c> gives ("The robot's camera") and
    /// <c>robot/lerobot</c> reads.
    /// </summary>
    public sealed class CameraSessionWriterTests : IDisposable
    {
        private const long Tps = 1_000_000_000;
        private readonly string _root = Path.Combine(Path.GetTempPath(), "camera-session-" + Guid.NewGuid().ToString("N"));
        private readonly string _control;
        private readonly string _session;
        private long _now = 1_000 * Tps;
        private long _free = long.MaxValue;
        private readonly List<string> _log = new();

        public CameraSessionWriterTests()
        {
            Directory.CreateDirectory(_root);
            _control = Path.Combine(_root, ".active");
            _session = Session("scenarios/reach/20261112T120000Z-take001", take: true);
        }

        /// <summary>A session directory as RobotHost opens it: session.json first, with a scenario for a take.</summary>
        private string Session(string relative, bool take)
        {
            string directory = Path.Combine(_root, relative);
            Directory.CreateDirectory(directory);
            File.WriteAllText(Path.Combine(directory, "session.json"), take
                ? "{\"format\":\"jetrover-session\",\"version\":1,\"scenario\":{\"name\":\"reach\",\"take\":1,\"description\":\"\"}}"
                : "{\"format\":\"jetrover-session\",\"version\":1,\"label\":\"auto\"}");
            return directory;
        }

        public void Dispose()
        {
            try { Directory.Delete(_root, true); } catch (IOException) { }
        }

        private CameraSessionWriter NewWriter(int capacity = 90) =>
            new(_control, () => _now, Tps, minFreeBytes: 1L << 30, freeBytes: _ => _free, log: _log.Add, capacity: capacity,
                pollSeconds: 0.5, startThread: false);

        private void Name(string? directory) => File.WriteAllText(_control, directory == null ? string.Empty : directory + "\n");

        private void Advance(double seconds) => _now += (long)(seconds * Tps);

        private static byte[] Jpeg(int n) => Encoding.ASCII.GetBytes($"\xFF\xD8frame-{n}-" + new string('x', n % 7) + "\xFF\xD9");

        /// <summary>Reads a jlog as the converter does: the trailer's sha256 and count checked, records by line.</summary>
        private static (string[] Header, List<string[]> Records, string[] Trailer) Read(string path)
        {
            byte[] data = File.ReadAllBytes(path);
            Assert.Equal((byte)'\n', data[^1]);
            int last = Array.LastIndexOf(data, (byte)'\n', data.Length - 2) + 1;
            string[] trailer = Encoding.ASCII.GetString(data, last, data.Length - last - 1).Split('|');
            Assert.Equal("END", trailer[0]);
            Assert.Equal(Convert.ToHexString(SHA256.HashData(data.AsSpan(0, last))).ToLowerInvariant(), trailer[3]);
            string[] lines = Encoding.ASCII.GetString(data, 0, last).TrimEnd('\n').Split('\n');
            Assert.Equal(lines.Length - 1, int.Parse(trailer[1]));
            return (lines[0].Split('|'), lines.Skip(1).Select(l => l.Split('|')).ToList(), trailer);
        }

        private static byte[] Payload(string sidecar, long offset, int length)
        {
            byte[] data = File.ReadAllBytes(sidecar);
            Assert.Equal((uint)length, BinaryPrimitives.ReadUInt32LittleEndian(data.AsSpan((int)offset - 4)));
            return data.AsSpan((int)offset, length).ToArray();
        }

        [Fact]
        public void WhileTheControlFileNamesASession_EveryFrameIsRecordedWithItsCaptureStamp()
        {
            using CameraSessionWriter w = NewWriter();
            w.Step();
            w.OfferFrame(Jpeg(0), 0, 999 * Tps, 640, 360); // no session yet: not recorded
            Assert.False(w.Active);
            Name(_session);
            Advance(1);
            w.Step();
            Assert.True(w.Active);
            for (int n = 1; n <= 3; n++)
            {
                w.OfferFrame(Jpeg(n), (uint)n, _now - 5_000_000, 640, 360);
                Advance(1.0 / 30);
            }

            w.Step();
            Name(null);
            Advance(1);
            w.Step();
            Assert.False(w.Active);

            var (header, records, trailer) = Read(Path.Combine(_session, "camera.jlog"));
            Assert.Equal(new[] { "JSES", "1", "camera", "1000000000", "CLOCK_MONOTONIC", "20261112T120000Z-take001" }, header);
            Assert.Equal(new[] { "CM", "front", "640", "360" }, records[0]);
            List<string[]> vf = records.Where(r => r[0] == "VF").ToList();
            Assert.Equal(new[] { "1", "2", "3" }, vf.Select(r => r[2]));
            string sidecar = Path.Combine(_session, "camera", "front.rgb.frames");
            Assert.Equal("TFRM"u8.ToArray(), File.ReadAllBytes(sidecar).Take(4));
            for (int i = 0; i < 3; i++)
            {
                string[] parts = vf[i][5].Split(':');
                Assert.Equal("front.rgb", parts[0]);
                Assert.Equal(Jpeg(i + 1), Payload(sidecar, long.Parse(parts[1]), int.Parse(parts[2])));
                Assert.Equal(string.Empty, vf[i][4]); // no flags
            }

            Assert.Equal(1001 * Tps - 5_000_000, long.Parse(vf[0][3])); // stateSensed is the capture stamp
            string[] fs = records.Single(r => r[0] == "FS");
            Assert.Equal("camera/front.rgb.frames", fs[1]);
            Assert.Equal(new FileInfo(sidecar).Length, long.Parse(fs[2]));
            Assert.Equal(Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(sidecar))).ToLowerInvariant(), fs[3]);
            Assert.Equal("0", trailer[2]);
        }

        [Fact]
        public void ARestartInTheSameSession_NeverTruncatesItsLog()
        {
            Name(_session);
            using (CameraSessionWriter first = NewWriter())
            {
                first.Step();
                first.OfferFrame(Jpeg(1), 1, _now, 640, 360);
                first.Step();
            }

            string firstLog = File.ReadAllText(Path.Combine(_session, "camera.jlog"));
            using (CameraSessionWriter second = NewWriter())
            {
                second.Step();
                second.OfferFrame(Jpeg(2), 2, _now, 640, 360);
                second.Step();
            }

            Assert.Equal(firstLog, File.ReadAllText(Path.Combine(_session, "camera.jlog")));
            var (_, records, _) = Read(Path.Combine(_session, "camera.2.jlog"));
            Assert.Equal("camera/front.rgb.2.frames", records.Single(r => r[0] == "FS")[1]);
            Assert.True(File.Exists(Path.Combine(_session, "camera", "front.rgb.2.frames")));
        }

        [Fact]
        public void AFullQueue_DropsTheFrameFromTheRecording_WithAnFd()
        {
            Name(_session);
            using CameraSessionWriter w = NewWriter(capacity: 2);
            w.Step();
            for (int n = 1; n <= 4; n++)
            {
                w.OfferFrame(Jpeg(n), (uint)n, _now, 640, 360);
            }

            w.Step();
            Name(null);
            Advance(1);
            w.Step();
            var (_, records, trailer) = Read(Path.Combine(_session, "camera.jlog"));
            Assert.Equal(new[] { "1", "2" }, records.Where(r => r[0] == "VF").Select(r => r[2]));
            Assert.Equal(new[] { "3", "4" }, records.Where(r => r[0] == "FD" && r[3] == "writer-behind").Select(r => r[2]));
            Assert.Equal("2", trailer[2]);
            Assert.Equal(2, w.DroppedFrames);
        }

        [Fact]
        public void AFrameWithoutACaptureStamp_IsNotRecorded()
        {
            Name(_session);
            using CameraSessionWriter w = NewWriter();
            w.Step();
            w.OfferFrame(Jpeg(1), 1, LatencyTrace.Unset, 640, 360);
            w.Step();
            Name(null);
            Advance(1);
            w.Step();
            var (_, records, _) = Read(Path.Combine(_session, "camera.jlog"));
            Assert.DoesNotContain(records, r => r[0] == "VF");
            Assert.Contains(records, r => r[0] == "FD" && r[3] == "unstamped");
        }

        [Fact]
        public void BelowTheDiskFloor_TheSessionStopsWithAnEr_AndIsNotReopened()
        {
            Name(_session);
            using CameraSessionWriter w = NewWriter();
            w.Step();
            w.OfferFrame(Jpeg(1), 1, _now, 640, 360);
            _free = 100;
            Advance(1);
            w.Step();
            Assert.False(w.Active);
            var (_, records, _) = Read(Path.Combine(_session, "camera.jlog"));
            Assert.Contains(records, r => r[0] == "ER" && r[2].StartsWith("disk below", StringComparison.Ordinal));
            Assert.Single(records, r => r[0] == "VF");
            _free = long.MaxValue;
            Advance(1);
            w.Step();
            Assert.False(w.Active); // still the same session named: not reopened
            Assert.False(File.Exists(Path.Combine(_session, "camera.2.jlog")));
        }

        [Fact]
        public void ASessionThatIsNotATake_GetsALogWithNoFrames()
        {
            string auto = Session("jetrover/20261112T130000Z-auto", take: false);
            Name(auto);
            using CameraSessionWriter w = NewWriter();
            w.Step();
            Assert.True(w.Active);
            w.OfferFrame(Jpeg(1), 1, _now, 640, 360);
            w.Step();
            Name(null);
            Advance(1);
            w.Step();
            var (header, records, trailer) = Read(Path.Combine(auto, "camera.jlog"));
            Assert.Equal("20261112T130000Z-auto", header[5]);
            Assert.Empty(records);
            Assert.Equal("0", trailer[2]);
            Assert.False(Directory.Exists(Path.Combine(auto, "camera")));
        }

        [Fact]
        public void WithTakesOnlyOff_EverySessionGetsFrames()
        {
            string auto = Session("jetrover/20261112T130000Z-auto", take: false);
            Name(auto);
            using var w = new CameraSessionWriter(_control, () => _now, Tps, 1L << 30, _ => _free, _log.Add, takesOnly: false, startThread: false);
            w.Step();
            w.OfferFrame(Jpeg(1), 1, _now, 640, 360);
            w.Step();
            Name(null);
            Advance(1);
            w.Step();
            Assert.Single(Read(Path.Combine(auto, "camera.jlog")).Records, r => r[0] == "VF");
        }

        [Fact]
        public void ANewSizeWritesANewCm()
        {
            Name(_session);
            using CameraSessionWriter w = NewWriter();
            w.Step();
            w.OfferFrame(Jpeg(1), 1, _now, 640, 360);
            w.OfferFrame(Jpeg(2), 2, _now, 640, 360);
            w.OfferFrame(Jpeg(3), 3, _now, 1280, 720);
            w.Step();
            Name(null);
            Advance(1);
            w.Step();
            var (_, records, _) = Read(Path.Combine(_session, "camera.jlog"));
            Assert.Equal(new[] { "640x360", "1280x720" }, records.Where(r => r[0] == "CM").Select(r => r[2] + "x" + r[3]));
        }
    }
}
