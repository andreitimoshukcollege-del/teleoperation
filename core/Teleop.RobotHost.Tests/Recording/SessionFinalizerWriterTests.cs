using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json.Nodes;
using Teleop.RobotHost.Recording;
using Xunit;

namespace Teleop.RobotHost.Tests.Recording
{
    /// <summary>
    /// Another writer in a session (docs/adr/0032 decision 6): the finalizer waits for its trailer, checks the files its FS lines
    /// name, and hashes its folder into files.sha256, so <c>sha256sum -c</c> covers the frames.
    /// </summary>
    public sealed class SessionFinalizerWriterTests : IDisposable
    {
        private readonly string _dir = Path.Combine(Path.GetTempPath(), "teleop-writer-" + Guid.NewGuid().ToString("N"));
        private static readonly WriterWait[] Isaac = { new WriterWait("isaac", 0.2) };

        public SessionFinalizerWriterTests()
        {
            Directory.CreateDirectory(Path.Combine(_dir, "isaac"));
            File.WriteAllText(Path.Combine(_dir, "session.json"), "{\"format\": \"teleop-session\", \"version\": 2, \"notes\": \"\"}\n");
            File.WriteAllText(Path.Combine(_dir, "robothost.jlog"), "JSES|2|robothost|1000000000|CLOCK_MONOTONIC|s\nEND|0|0|x\n");
        }

        public void Dispose()
        {
            try { Directory.Delete(_dir, true); } catch (IOException) { }
        }

        private static string Sha(byte[] data) => Convert.ToHexString(SHA256.HashData(data)).ToLowerInvariant();

        private void WriteIsaac(bool trailer, Func<byte[], string>? shaOf = null)
        {
            byte[] frames = Encoding.ASCII.GetBytes("TFRM\x01\x00\x00\x00 frames");
            File.WriteAllBytes(Path.Combine(_dir, "isaac", "head.rgb.frames"), frames);
            var log = new StringBuilder("JSES|1|isaac|1000000000|CLOCK_MONOTONIC|s\n");
            log.Append($"FS|isaac/head.rgb.frames|{frames.Length}|{(shaOf ?? Sha)(frames)}\n");
            if (trailer) log.Append("END|1|0|abc\n");
            File.WriteAllText(Path.Combine(_dir, "isaac.jlog"), log.ToString());
        }

        private string Notes() => (string?)JsonNode.Parse(File.ReadAllText(Path.Combine(_dir, "session.json")))!["notes"] ?? "";

        [Fact]
        public void AWritersFolder_IsHashed_AndAGoodWriterLeavesNoNote()
        {
            WriteIsaac(trailer: true);
            SessionFinalizer.Complete(_dir, 0, () => DateTime.UnixEpoch, _ => { }, Isaac);
            string[] sums = File.ReadAllLines(Path.Combine(_dir, "files.sha256"));
            Assert.Contains(sums, l => l.EndsWith("  isaac/head.rgb.frames", StringComparison.Ordinal));
            Assert.Contains(sums, l => l.EndsWith("  isaac.jlog", StringComparison.Ordinal));
            Assert.DoesNotContain("isaac", Notes());
        }

        [Fact]
        public void ASidecarThatIsNotWhatItsLogSays_IsNoted()
        {
            WriteIsaac(trailer: true, shaOf: _ => new string('0', 64));
            SessionFinalizer.Complete(_dir, 0, () => DateTime.UnixEpoch, _ => { }, Isaac);
            Assert.Contains("isaac/head.rgb.frames is not what isaac.jlog says", Notes());
        }

        [Fact]
        public void AWriterWithoutItsTrailer_OrWithoutALog_IsNoted()
        {
            WriteIsaac(trailer: false);
            SessionFinalizer.Complete(_dir, 0, () => DateTime.UnixEpoch, _ => { }, Isaac);
            Assert.Contains("isaac.jlog had no END trailer 0.2 s", Notes());

            File.Delete(Path.Combine(_dir, "isaac.jlog"));
            File.WriteAllText(Path.Combine(_dir, "session.json"), "{\"notes\": \"\"}\n");
            SessionFinalizer.Complete(_dir, 0, () => DateTime.UnixEpoch, _ => { }, Isaac);
            Assert.Contains("isaac.jlog is missing", Notes());
        }

        [Fact]
        public void AnFsLineOutsideTheSession_IsRefused()
        {
            File.WriteAllText(Path.Combine(_dir, "isaac.jlog"), "JSES|1|isaac|1|C|s\nFS|../../etc/passwd|1|00\nEND|1|0|x\n");
            Assert.Single(SessionFinalizer.CheckFileLines(_dir, new[] { Path.Combine(_dir, "isaac.jlog") }), p => p.Contains("outside the session"));
        }

        [Theory]
        [InlineData("isaac:60", true)]
        [InlineData("isaac:0", true)]
        [InlineData("isaac", false)]
        [InlineData("Isaac:60", false)]
        [InlineData("../x:1", false)]
        [InlineData("driver:5", false)]
        [InlineData("isaac:-1", false)]
        public void AWriterIsNameColonSeconds(string spec, bool ok)
        {
            Assert.Equal(ok, WriterWait.TryParse(spec, out WriterWait w, out string error));
            if (ok) Assert.Equal("isaac", w.Name);
            else Assert.NotEmpty(error);
        }

        [Fact]
        public void TheSerialChainHost_TakesWriters()
        {
            var args = SerialChainHostArgs.TryParse(new[] { "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "sim", "--record-control", "c",
                "--auto-record", "--auto-record-writer", "isaac:60" }, out string error);
            Assert.True(args != null, error);
            Assert.Equal(60, args!.AutoRecord!.Writers.Single().Seconds);
        }
    }
}
