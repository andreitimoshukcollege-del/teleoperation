using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json.Nodes;
using System.Threading;
using Teleop.RobotHost.Recording;

namespace Teleop.RobotHost.Tests.Recording
{
    /// <summary>
    /// What the writer does with an automatic session (docs/adr/0023 decision 4): the files it opens
    /// with, the split into two sessions, completing one when it closes (as <c>just record-stop</c>
    /// does), giving way to a manual session, and the two fixes both writers got: a log that exists is
    /// never truncated, and an I/O error closes the session instead of the process.
    /// </summary>
    public class SessionRecorderAutoTests
    {
        private const long Tps = 1_000_000_000;
        private static readonly DateTime Utc = new DateTime(2026, 10, 26, 12, 0, 0, DateTimeKind.Utc);

        private static bool WaitFor(Func<bool> condition, double seconds = 10.0)
        {
            var sw = Stopwatch.StartNew();
            while (sw.Elapsed.TotalSeconds < seconds)
            {
                if (condition()) return true;
                Thread.Sleep(10);
            }

            return condition();
        }

        private static string ReadShared(string path)
        {
            if (!File.Exists(path)) return "";
            try
            {
                using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete);
                using var reader = new StreamReader(stream, Encoding.ASCII);
                return reader.ReadToEnd();
            }
            catch (IOException)
            {
                return "";
            }
        }

        private static string Sha(byte[] data) => Convert.ToHexString(SHA256.HashData(data)).ToLowerInvariant();

        private sealed class Rig : IDisposable
        {
            public readonly string Root;
            public readonly string Control;
            public readonly string Marker;
            public readonly string Profile;
            public readonly ConcurrentQueue<string> Log = new ConcurrentQueue<string>();
            public readonly AutoRecordOptions Options;
            public SessionRecorder Recorder = null!;

            public Rig(double driverWaitSeconds = 5, bool start = true, Func<string, Stream>? openFile = null, bool auto = true)
            {
                Root = Path.Combine(Path.GetTempPath(), "teleop-auto-" + Guid.NewGuid().ToString("N"));
                Directory.CreateDirectory(Root);
                Control = Path.Combine(Root, ".active");
                Marker = Path.Combine(Root, SessionRecorder.AutoMarkerName);
                Profile = Path.Combine(Root, "jetrover.json");
                File.WriteAllText(Profile, "{\"Name\": \"JetRover\"}\n");
                File.WriteAllText(Path.Combine(Root, "rh.BUILD_INFO"), "{\"sha\": \"abc123\", \"dirty\": false}\n");
                File.WriteAllText(Path.Combine(Root, "drv.BUILD_INFO"), "{\"sha\": \"def456\", \"dirty\": false}\n");
                Options = new AutoRecordOptions
                {
                    ProfilePath = Profile,
                    RobotHostBuildInfoPath = Path.Combine(Root, "rh.BUILD_INFO"),
                    DriverBuildInfoPath = Path.Combine(Root, "drv.BUILD_INFO"),
                    DriverWaitSeconds = driverWaitSeconds,
                };
                _auto = auto;
                _openFile = openFile;
                if (start) Start();
            }

            private readonly bool _auto;
            private readonly Func<string, Stream>? _openFile;

            public void Start() =>
                Recorder = new SessionRecorder(Control, Tps, SessionRecorder.DefaultCapacity, 0.02, _auto ? Options : null, () => Utc, Log.Enqueue, _openFile);

            public string AutoDir(string id = "20261026T120000Z-auto") => Path.Combine(Root, "jetrover", id);

            public int Begin(string sender = "10.0.0.5:6002", UplinkCondition? uplink = null) =>
                Recorder.BeginAuto(new AutoSessionSpec(Utc, sender, uplink ?? UplinkCondition.None, "first joint command after 30 s without one"));

            /// <summary>A driver.jlog the way the driver leaves it: header and trailer.</summary>
            public static void WriteDriverLog(string directory)
            {
                string body = "JSES|1|driver|1000000000|CLOCK_MONOTONIC|" + Path.GetFileName(directory) + "\nRD|1|2|1|500\n";
                File.WriteAllText(Path.Combine(directory, "driver.jlog"), body + "END|1|0|" + Sha(Encoding.ASCII.GetBytes(body)) + "\n");
            }

            public void Dispose()
            {
                Recorder?.Dispose();
                try
                {
                    Directory.Delete(Root, recursive: true);
                }
                catch (IOException)
                {
                }
                catch (UnauthorizedAccessException)
                {
                }
            }
        }

        private static (string[] Lines, string[] Trailer) ReadLog(string path)
        {
            string text = File.ReadAllText(path);
            Assert.EndsWith("\n", text);
            string[] lines = text.Split('\n');
            string[] trailer = lines[lines.Length - 2].Split('|');
            Assert.Equal("END", trailer[0]);
            byte[] body = Encoding.ASCII.GetBytes(string.Join("\n", lines.Take(lines.Length - 2)) + "\n");
            Assert.Equal(Sha(body), trailer[3]);
            return (lines.Take(lines.Length - 2).ToArray(), trailer);
        }

        [Fact]
        public void AnAutomaticSession_OpensWithItsFiles_AndIsCompletedWhenItCloses()
        {
            using var rig = new Rig();
            int generation = rig.Begin(uplink: new UplinkCondition("50ms-5j", 3));
            Assert.NotEqual(0, generation);
            Assert.True(rig.Recorder.IsActive);
            Assert.True(rig.Recorder.TakePreambleOwed());
            Assert.False(rig.Recorder.TakePreambleOwed());
            rig.Recorder.Record("SS|1|10.0.0.5:6002");
            rig.Recorder.Record("JS|1|10.0.0.5:6002");
            rig.Recorder.Record("UP|1|50ms-5j|3");
            rig.Recorder.Record("JC|2|2|0|0|A|1:0.1:2.5;10:1:0");
            rig.Recorder.Record("JC|3|3|1|1|A|1:0.2:2.5;2:0.1:2.5;10:1:3");
            string dir = rig.AutoDir();
            Assert.True(WaitFor(() => ReadShared(rig.Control).Trim() == dir));
            Assert.Equal(dir, ReadShared(rig.Marker).Trim());

            JsonNode meta = JsonNode.Parse(File.ReadAllText(Path.Combine(dir, "session.json")))!;
            Assert.Equal("jetrover-session", (string)meta["format"]!);
            Assert.Equal(1, (int)meta["version"]!);
            Assert.Equal("20261026T120000Z-auto", (string)meta["id"]!);
            Assert.Equal("robot", (string)meta["source"]!);
            Assert.Equal("auto", (string)meta["label"]!);
            Assert.Equal("2026-10-26T12:00:00Z", (string)meta["startedUtc"]!);
            JsonNode c = meta["conditions"]!;
            Assert.Equal("unknown", (string)c["speedSliderRadPerSec"]!);
            Assert.Equal("unknown", (string)c["load"]!["kind"]!);
            Assert.Null(c["load"]!["payloadGrams"]);
            Assert.Equal("50ms-5j", (string)c["network"]!["uplinkProfile"]!);
            Assert.Equal(3UL, (ulong)c["network"]!["uplinkSeed"]!);
            Assert.Equal("auto", (string)c["motion"]!);
            Assert.Equal("headset 10.0.0.5:6002", (string)c["operator"]!);
            Assert.Equal("abc123", (string)meta["builds"]!["robothost"]!["sha"]!);
            Assert.False((bool)meta["builds"]!["robothost"]!["dirty"]!);
            Assert.Equal("def456", (string)meta["builds"]!["driver"]!["sha"]!);
            byte[] profile = File.ReadAllBytes(rig.Profile);
            Assert.Equal(profile, File.ReadAllBytes(Path.Combine(dir, "robot-profile.json")));
            Assert.Equal(Sha(profile), (string)meta["robotProfileSha256"]!);

            Rig.WriteDriverLog(dir);
            rig.Recorder.EndAuto(generation, "test");
            Assert.False(rig.Recorder.IsSessionActive(generation));
            Assert.True(WaitFor(() => File.Exists(Path.Combine(dir, "files.sha256"))));
            Assert.False(File.Exists(rig.Control));
            Assert.False(File.Exists(rig.Marker));

            var (lines, trailer) = ReadLog(Path.Combine(dir, "robothost.jlog"));
            Assert.Equal("JSES|1|robothost|1000000000|CLOCK_MONOTONIC|20261026T120000Z-auto", lines[0]);
            Assert.Equal(new[] { "SS|1|10.0.0.5:6002", "JS|1|10.0.0.5:6002", "UP|1|50ms-5j|3" }, lines.Skip(1).Take(3).ToArray());
            Assert.Equal(new[] { "END", "5", "0" }, trailer.Take(3).ToArray());

            meta = JsonNode.Parse(File.ReadAllText(Path.Combine(dir, "session.json")))!;
            Assert.Equal("2026-10-26T12:00:00Z", (string)meta["stoppedUtc"]!);
            Assert.Equal(2.5, (double)meta["conditions"]!["speedSliderRadPerSec"]!);
            Assert.Equal("50ms-5j", (string)meta["conditions"]!["network"]!["uplinkProfile"]!);

            string[] sums = File.ReadAllLines(Path.Combine(dir, "files.sha256"));
            Assert.Equal(new[] { "session.json", "robot-profile.json", "driver.jlog", "robothost.jlog" }, sums.Select(l => l.Split("  ")[1]).ToArray());
            foreach (string line in sums)
            {
                string[] parts = line.Split("  ");
                Assert.Equal(Sha(File.ReadAllBytes(Path.Combine(dir, parts[1]))), parts[0]);
            }
        }

        [Fact]
        public void ASplit_GivesTwoCompleteSessions_EachWithItsOwnRecords()
        {
            using var rig = new Rig(driverWaitSeconds: 0.2);
            int first = rig.Begin();
            rig.Recorder.Record("JC|1|1|0|0|A|1:0.1:2.5");
            rig.Recorder.Record("JC|2|2|1|1|A|1:0.1:3.8");
            rig.Recorder.Record("UP|3|50ms-5j|1");
            rig.Recorder.EndAuto(first, "the joint-command sender changed");
            int second = rig.Begin("10.0.0.5:40000");
            Assert.NotEqual(first, second);
            rig.Recorder.Record("KA|4|4|1000000000");
            rig.Recorder.EndAuto(second, "no joint command for 30 s");

            string a = rig.AutoDir(), b = rig.AutoDir("20261026T120000Z-auto-2");
            Assert.True(WaitFor(() => File.Exists(Path.Combine(a, "files.sha256")) && File.Exists(Path.Combine(b, "files.sha256"))), string.Join("\n", rig.Log));
            Assert.Equal(new[] { "JC|1|1|0|0|A|1:0.1:2.5", "JC|2|2|1|1|A|1:0.1:3.8", "UP|3|50ms-5j|1" },
                ReadLog(Path.Combine(a, "robothost.jlog")).Lines.Skip(1).ToArray());
            Assert.Equal(new[] { "KA|4|4|1000000000" }, ReadLog(Path.Combine(b, "robothost.jlog")).Lines.Skip(1).ToArray());

            JsonNode ma = JsonNode.Parse(File.ReadAllText(Path.Combine(a, "session.json")))!;
            Assert.Equal("varied", (string)ma["conditions"]!["speedSliderRadPerSec"]!);
            Assert.Equal("varied", (string)ma["conditions"]!["network"]!["uplinkProfile"]!);
            Assert.Contains("no END trailer", (string)ma["notes"]!); // no driver in this test
            JsonNode mb = JsonNode.Parse(File.ReadAllText(Path.Combine(b, "session.json")))!;
            Assert.Equal("unknown", (string)mb["conditions"]!["speedSliderRadPerSec"]!); // no command carried a speed
            Assert.Equal("headset 10.0.0.5:40000", (string)mb["conditions"]!["operator"]!);
            Assert.Equal("lan", (string)mb["conditions"]!["network"]!["uplinkProfile"]!);
        }

        [Fact]
        public void AManualSession_TakesOver_AndTheAutomaticRecorderStaysOutWhileItRuns()
        {
            using var rig = new Rig(driverWaitSeconds: 0.2);
            int generation = rig.Begin();
            string dir = rig.AutoDir();
            Assert.True(WaitFor(() => ReadShared(rig.Control).Trim() == dir));
            Assert.False(rig.Recorder.ControlFileBusy);

            // just record-start: clear the control file, wait for the trailers and files.sha256.
            File.Delete(rig.Control);
            Assert.True(WaitFor(() => !rig.Recorder.IsSessionActive(generation)));
            Assert.True(WaitFor(() => File.Exists(Path.Combine(dir, "files.sha256"))));
            Assert.False(File.Exists(rig.Marker));
            Assert.EndsWith("\n", ReadShared(Path.Combine(dir, "robothost.jlog")));
            Assert.StartsWith("END|", SessionFinalizer.LastLine(Path.Combine(dir, "robothost.jlog")));

            // ... then open its own.
            string manual = Path.Combine(rig.Root, "jetrover", "20261026T120100Z-teleop");
            File.WriteAllText(rig.Control, manual + "\n");
            Assert.True(WaitFor(() => rig.Recorder.IsActive && rig.Recorder.ControlFileBusy));
            Assert.True(rig.Recorder.TakePreambleOwed());
            Assert.Equal(0, rig.Begin());
            rig.Recorder.Record("KA|1|1|1000000000");
            File.Delete(rig.Control);
            Assert.True(WaitFor(() => !rig.Recorder.IsActive && SessionFinalizer.LastLine(Path.Combine(manual, "robothost.jlog")).StartsWith("END|1|")));
            Assert.False(File.Exists(Path.Combine(manual, "session.json"))); // record-start's to write, not RobotHost's
            Assert.False(File.Exists(Path.Combine(manual, "files.sha256")));
            Assert.False(rig.Recorder.ControlFileBusy);
        }

        [Fact]
        public void AManualSessionThatStartsBeforeTheAutomaticOneIsOpened_Wins()
        {
            using var rig = new Rig(start: false);
            string manual = Path.Combine(rig.Root, "m");
            rig.Start();
            // The automatic open is queued in the same instant record-start writes the control file.
            File.WriteAllText(rig.Control, manual + "\n");
            int generation = rig.Begin();
            Assert.True(WaitFor(() => rig.Recorder.ControlFileBusy && rig.Recorder.Directory == manual));
            Assert.False(rig.Recorder.IsSessionActive(generation));
            Assert.Equal(manual, ReadShared(rig.Control).Trim());
            Assert.False(Directory.Exists(rig.AutoDir()));
            Assert.False(File.Exists(rig.Marker));
        }

        [Fact]
        public void AnAutomaticSessionAnEarlierProcessLeftOpen_IsClosedAndCompleted_NotContinued()
        {
            using var rig = new Rig(driverWaitSeconds: 1, start: false);
            string dir = rig.AutoDir("20261026T110000Z-auto");
            Directory.CreateDirectory(dir);
            File.WriteAllText(Path.Combine(dir, "session.json"),
                "{\"format\": \"jetrover-session\", \"version\": 1, \"id\": \"20261026T110000Z-auto\", \"label\": \"auto\", \"notes\": \"\", " +
                "\"conditions\": {\"speedSliderRadPerSec\": \"unknown\", \"network\": {\"uplinkProfile\": \"lan\"}}}");
            string cutOff = "JSES|1|robothost|1000000000|CLOCK_MONOTONIC|20261026T110000Z-auto\nJC|1|1|0|0|A|1:0.1:1.5\n";
            File.WriteAllText(Path.Combine(dir, "robothost.jlog"), cutOff);
            Rig.WriteDriverLog(dir);
            File.WriteAllText(rig.Control, dir + "\n");
            File.WriteAllText(rig.Marker, dir + "\n");
            rig.Start();

            Assert.True(WaitFor(() => File.Exists(Path.Combine(dir, "files.sha256"))));
            Assert.False(File.Exists(rig.Control));
            Assert.False(File.Exists(rig.Marker));
            Assert.Equal(cutOff, File.ReadAllText(Path.Combine(dir, "robothost.jlog"))); // kept as it was: it fails the check, honestly
            Assert.False(File.Exists(Path.Combine(dir, "robothost.2.jlog")));
            Assert.False(rig.Recorder.IsActive);
            JsonNode meta = JsonNode.Parse(File.ReadAllText(Path.Combine(dir, "session.json")))!;
            Assert.Equal(1.5, (double)meta["conditions"]!["speedSliderRadPerSec"]!);
        }

        [Fact]
        public void ALogThatExists_IsNeverTruncated_TheWriterContinuesInASegment()
        {
            using var rig = new Rig(auto: false);
            string dir = Path.Combine(rig.Root, "jetrover", "20261026T100000Z-teleop");
            Directory.CreateDirectory(dir);
            const string before = "JSES|1|robothost|1000000000|CLOCK_MONOTONIC|20261026T100000Z-teleop\nKA|1|1|1\n";
            File.WriteAllText(Path.Combine(dir, "robothost.jlog"), before);
            File.WriteAllText(rig.Control, dir + "\n"); // RobotHost restarted while record-start's session ran
            Assert.True(WaitFor(() => rig.Recorder.IsActive));
            rig.Recorder.Record("KA|2|2|1000000000");
            File.Delete(rig.Control);
            Assert.True(WaitFor(() => !rig.Recorder.IsActive && SessionFinalizer.LastLine(Path.Combine(dir, "robothost.2.jlog")).StartsWith("END|")));
            Assert.Equal(before, File.ReadAllText(Path.Combine(dir, "robothost.jlog")));
            var (lines, trailer) = ReadLog(Path.Combine(dir, "robothost.2.jlog"));
            Assert.Equal("JSES|1|robothost|1000000000|CLOCK_MONOTONIC|20261026T100000Z-teleop", lines[0]);
            Assert.Equal("KA|2|2|1000000000", lines[1]);
            Assert.Equal("1", trailer[1]);
            Assert.Contains(rig.Log, l => l.Contains("continuing in robothost.2.jlog"));
        }

        /// <summary>A log file whose writes fail as a full disk's do: once past a budget, or for good.</summary>
        private sealed class FailingStream : Stream
        {
            private readonly Stream _inner;
            private readonly long _budget;
            private readonly bool _once;
            private long _written;
            private bool _failed;

            public FailingStream(Stream inner, long budget, bool once)
            {
                _inner = inner;
                _budget = budget;
                _once = once;
            }

            public override void Write(byte[] buffer, int offset, int count)
            {
                if (_written + count > _budget && !(_once && _failed))
                {
                    _failed = true;
                    throw new IOException("No space left on device");
                }

                _inner.Write(buffer, offset, count);
                _written += count;
            }

            public override void Flush()
            {
                if (_failed && !_once) throw new IOException("No space left on device");
                _inner.Flush();
            }

            protected override void Dispose(bool disposing)
            {
                if (disposing) _inner.Dispose();
                base.Dispose(disposing);
            }

            public override bool CanRead => false;
            public override bool CanSeek => false;
            public override bool CanWrite => true;
            public override long Length => _inner.Length;
            public override long Position { get => _inner.Position; set => throw new NotSupportedException(); }
            public override int Read(byte[] buffer, int offset, int count) => throw new NotSupportedException();
            public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
            public override void SetLength(long value) => throw new NotSupportedException();
        }

        private static Func<string, Stream> FailIn(string directoryName, bool once) => path =>
        {
            Stream file = new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read);
            return Path.GetFileName(Path.GetDirectoryName(path)) == directoryName ? new FailingStream(file, 200, once) : file;
        };

        [Fact]
        public void AnIOError_ClosesTheManualSessionWithItsTrailer_AndTheRecorderKeepsWorking()
        {
            using var rig = new Rig(auto: false, openFile: FailIn("bad", once: true));
            string bad = Path.Combine(rig.Root, "bad"), good = Path.Combine(rig.Root, "good");
            File.WriteAllText(rig.Control, bad + "\n");
            Assert.True(WaitFor(() => rig.Recorder.IsActive));
            for (int i = 0; i < 50; i++) rig.Recorder.Record($"KA|{i}|{i}|1000000000");
            Assert.True(WaitFor(() => !rig.Recorder.IsActive && rig.Log.Any(l => l.Contains("closed after the error"))));
            Assert.Contains(rig.Log, l => l.Contains("write failed") && l.Contains("No space left on device"));
            string text = File.ReadAllText(Path.Combine(bad, "robothost.jlog"));
            string[] trailer = text.TrimEnd('\n').Split('\n').Last().Split('|');
            Assert.Equal("END", trailer[0]);
            // The hash covers the line the disk refused: the file fails the trainer's check, as it should.
            byte[] body = Encoding.ASCII.GetBytes(text.Substring(0, text.TrimEnd('\n').LastIndexOf('\n') + 1));
            Assert.NotEqual(Sha(body), trailer[3]);
            Thread.Sleep(200);
            Assert.False(rig.Recorder.IsActive); // not reopened while the control file still names it
            Assert.False(File.Exists(Path.Combine(bad, "robothost.2.jlog")));

            File.WriteAllText(rig.Control, good + "\n");
            Assert.True(WaitFor(() => rig.Recorder.IsActive));
            rig.Recorder.Record("KA|1|1|1000000000");
            File.Delete(rig.Control);
            Assert.True(WaitFor(() => SessionFinalizer.LastLine(Path.Combine(good, "robothost.jlog")).StartsWith("END|1|0|")));
            ReadLog(Path.Combine(good, "robothost.jlog"));
        }

        [Fact]
        public void AnIOError_InAnAutomaticSession_ClosesAndCompletesIt_EvenWithoutATrailer()
        {
            using var rig = new Rig(driverWaitSeconds: 0.2, openFile: FailIn("20261026T120000Z-auto", once: false));
            int generation = rig.Begin();
            string dir = rig.AutoDir();
            for (int i = 0; i < 50; i++) rig.Recorder.Record($"JC|{i}|{i}|{i}|{i}|A|1:0.1:2.5");
            Assert.True(WaitFor(() => !rig.Recorder.IsSessionActive(generation)));
            Assert.True(WaitFor(() => File.Exists(Path.Combine(dir, "files.sha256"))));
            Assert.False(File.Exists(rig.Control)); // the driver closes its log too
            Assert.False(File.Exists(rig.Marker));
            Assert.DoesNotContain("END|", File.ReadAllText(Path.Combine(dir, "robothost.jlog")));
            Assert.Contains(rig.Log, l => l.Contains("without a trailer"));

            // The next session is written normally.
            int next = rig.Begin();
            Assert.NotEqual(0, next);
            rig.Recorder.Record("KA|1|1|1000000000");
            rig.Recorder.EndAuto(next, "test");
            string second = rig.AutoDir("20261026T120000Z-auto-2");
            Assert.True(WaitFor(() => File.Exists(Path.Combine(second, "files.sha256"))));
            ReadLog(Path.Combine(second, "robothost.jlog"));
        }

        [Fact]
        public void Disposing_ClosesAndCompletesAnOpenAutomaticSession()
        {
            using var rig = new Rig(driverWaitSeconds: 0.2);
            rig.Begin();
            rig.Recorder.Record("JC|1|1|0|0|A|1:0.1:2.5");
            string dir = rig.AutoDir();
            Assert.True(WaitFor(() => File.Exists(rig.Control)));
            rig.Recorder.Dispose(); // as RobotHost does when it stops
            Assert.True(File.Exists(Path.Combine(dir, "files.sha256")));
            Assert.False(File.Exists(rig.Control));
            Assert.Equal("1", ReadLog(Path.Combine(dir, "robothost.jlog")).Trailer[1]);
        }
    }
}
