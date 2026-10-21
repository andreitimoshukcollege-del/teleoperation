using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using Teleop.Core.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Recording;
using Teleop.RobotHost.Relay;
using Teleop.RobotHost.Tests.Plant;

namespace Teleop.RobotHost.Tests.Recording
{
    /// <summary>
    /// RobotHost's half of a recorded session (docs/adr/0021): written only while the control file
    /// names a session, closed with a trailer whose count and SHA-256 are right, never blocking the
    /// main loop. The exact line formats below are also what trainer/tests/test_robot_lines.py parses,
    /// so a format change has to change both.
    /// </summary>
    public class SessionRecorderTests
    {
        private static bool WaitFor(Func<bool> condition, double seconds = 5.0)
        {
            var sw = Stopwatch.StartNew();
            while (sw.Elapsed.TotalSeconds < seconds)
            {
                if (condition()) return true;
                Thread.Sleep(10);
            }

            return false;
        }

        /// <summary>Reads a file the recorder may still have open (Windows refuses a plain read then).</summary>
        private static string ReadShared(string path)
        {
            if (!File.Exists(path)) return "";
            using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete);
            using var reader = new StreamReader(stream, Encoding.ASCII);
            return reader.ReadToEnd();
        }

        private static string TempDir()
        {
            string dir = Path.Combine(Path.GetTempPath(), "teleop-recorder-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            return dir;
        }

        [Fact]
        public void ItRecordsOnlyWhileTheControlFileNamesASession_AndTheTrailerIsRight()
        {
            string root = TempDir();
            string control = Path.Combine(root, ".active");
            string session = Path.Combine(root, "jetrover", "20261021T150000Z-test");
            string path = Path.Combine(session, "robothost.jlog");
            using (var recorder = new SessionRecorder(control, 1_000_000_000, pollSeconds: 0.02))
            {
                recorder.Record("KA|1|2|3"); // nothing open yet: kept nowhere
                Assert.False(recorder.IsActive);
                File.WriteAllText(control, session + "\n");
                Assert.True(WaitFor(() => recorder.IsActive));
                recorder.Record("KA|10|20|1000000000");
                recorder.Record("FR|30|0|30|25|20|10|1:1:0.5");
                File.Delete(control);
                Assert.True(WaitFor(() => !recorder.IsActive && ReadShared(path).Contains("\nEND|")));
            }

            string text = File.ReadAllText(path);
            string[] lines = text.Split('\n');
            Assert.Equal("JSES|1|robothost|1000000000|CLOCK_MONOTONIC|20261021T150000Z-test", lines[0]);
            Assert.Equal("KA|10|20|1000000000", lines[1]);
            Assert.Equal("FR|30|0|30|25|20|10|1:1:0.5", lines[2]);
            string[] trailer = lines[3].Split('|');
            Assert.Equal(new[] { "END", "2", "0" }, trailer.Take(3).ToArray());
            byte[] body = Encoding.ASCII.GetBytes(string.Join("\n", lines.Take(3)) + "\n");
            Assert.Equal(Convert.ToHexString(SHA256.HashData(body)).ToLowerInvariant(), trailer[3]);
            Assert.Equal("", lines[4]);
        }

        [Fact]
        public void AFullQueueDropsAndCounts_InsteadOfBlocking()
        {
            string root = TempDir();
            string control = Path.Combine(root, ".active");
            string session = Path.Combine(root, "s");
            long dropped;
            using (var recorder = new SessionRecorder(control, 1_000_000_000, capacity: 1, pollSeconds: 0.02))
            {
                File.WriteAllText(control, session);
                Assert.True(WaitFor(() => recorder.IsActive));
                var sw = Stopwatch.StartNew();
                for (int i = 0; i < 20000; i++) recorder.Record($"KA|{i}|0|1");
                Assert.True(sw.Elapsed.TotalSeconds < 2.0);
                dropped = recorder.Dropped;
                File.Delete(control);
                Assert.True(WaitFor(() => !recorder.IsActive));
            }

            Assert.True(dropped > 0);
            string[] trailer = File.ReadAllLines(Path.Combine(session, "robothost.jlog")).Last().Split('|');
            Assert.True(long.Parse(trailer[2]) >= dropped);
        }

        [Fact]
        public void TheLineFormats_AreTheSessionFormats()
        {
            var targets = new[] { new JointTarget(1, 0.25f, 2.5f), new JointTarget(10, 1f, 0f) };
            Assert.Equal("JC|100|105|7|99|A|1:0.25:2.5;10:1:0",
                SessionLines.JointCommand(100, 105, 7, 99, JointCommandOutcome.Accepted, targets));
            Assert.Equal("JC|1|1|0|0|S|", SessionLines.JointCommand(1, 1, 0, 0, JointCommandOutcome.Stale, ReadOnlySpan<JointTarget>.Empty));
            Assert.Equal("RO|5|1:560.5:597.2", SessionLines.RelayOut(5, new[] { new JointTarget(1, 560.5f, 597.2f) }));
            Assert.Equal("RI|6|1:1:512;2:0:0",
                SessionLines.RelayIn(6, new[] { new JointFeedbackEntry(1, true, 512f), new JointFeedbackEntry(2, false, 0f) }));
            Assert.Equal("FR|7|3|7|6|5|4|1:1:-0.1;10:0:0.5",
                SessionLines.Frame(7, 3, 7, 6, 5, 4, new[] { new JointSample(1, -0.1f, true), new JointSample(10, 0.5f, false) }));
            Assert.Equal("KA|8|9|1000000000", SessionLines.Keepalive(8, 9, 1_000_000_000));
            Assert.Equal("SS|9|10.0.0.5:6002", SessionLines.Subscriber(9, "10.0.0.5:6002"));
            Assert.Equal("JS|9|a/b", SessionLines.Sender(9, "a|b"));
            Assert.Equal("UP|9|150ms-20j-0.5loss|3", SessionLines.Uplink(9, "150ms-20j-0.5loss", 3));
        }

        [Fact]
        public void TheRecordingRelayClient_RecordsWhatPassesAndChangesNothing()
        {
            string root = TempDir();
            string control = Path.Combine(root, ".active");
            string session = Path.Combine(root, "r");
            var fake = new FakeRelayClient();
            fake.EnqueueFeedback(new JointFeedbackEntry(1, true, 400f));
            using (var recorder = new SessionRecorder(control, 1_000_000_000, pollSeconds: 0.02))
            {
                var clock = new ManualClock();
                var client = new RecordingRelayClient(fake, recorder, clock);
                File.WriteAllText(control, session);
                Assert.True(WaitFor(() => recorder.IsActive));
                clock.Now = 42;
                client.Send(new[] { new JointTarget(1, 500f, 2200f) });
                Span<JointFeedbackEntry> buffer = new JointFeedbackEntry[4];
                Assert.True(client.TryReceiveFeedback(buffer, out int n));
                Assert.Equal(1, n);
                Assert.Equal(400f, buffer[0].Pulse);
                File.Delete(control);
                Assert.True(WaitFor(() => !recorder.IsActive));
            }

            Assert.Single(fake.SentCommands);
            string[] lines = File.ReadAllLines(Path.Combine(session, "robothost.jlog"));
            Assert.Equal("RO|42|1:500:2200", lines[1]);
            Assert.Equal("RI|42|1:1:400", lines[2]);
        }

        [Fact]
        public void TheRecordControlArgument_IsParsed()
        {
            RobotHostArgs? parsed = RobotHostArgs.TryParse(new[]
            {
                "--local-port", "6000", "--relay-socket", "/tmp/a", "--local-relay-socket", "/tmp/b",
                "--record-control", "/home/jetson/recordings/.active",
            }, out _);
            Assert.NotNull(parsed);
            Assert.Equal("/home/jetson/recordings/.active", parsed!.Value.RecordControlPath);
            RobotHostArgs? without = RobotHostArgs.TryParse(new[] { "--local-port", "6000", "--relay-socket", "/tmp/a", "--local-relay-socket", "/tmp/b" }, out _);
            Assert.Null(without!.Value.RecordControlPath);
        }

        private sealed class ManualClock : Teleop.Core.Contracts.ITimeAuthority
        {
            public long Now;
            public long TicksPerSecond => 1_000_000_000;
            public long NowTicks => Now;
        }
    }
}
