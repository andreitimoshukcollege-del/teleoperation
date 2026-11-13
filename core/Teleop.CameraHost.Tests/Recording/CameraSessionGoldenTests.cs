using System.Text;
using Teleop.CameraHost.Recording;
using Teleop.Core.Types;

namespace Teleop.CameraHost.Tests.Recording
{
    /// <summary>
    /// The golden take that pins the LeRobot converter's reading of CameraHost's recording (docs/adr/0036,
    /// <c>robot/lerobot/tests/test_camera_golden.py</c>) to what <see cref="CameraSessionWriter"/> writes:
    /// <c>robot/lerobot/tests/golden/camera/</c> holds a take's <c>camera.jlog</c> and <c>camera/front.rgb.frames</c>.
    /// <c>just gen-camera-session-golden</c> rewrites them after a deliberate change.
    /// </summary>
    public sealed class CameraSessionGoldenTests : IDisposable
    {
        private const long Tps = 1_000_000_000;
        public const string SessionId = "20261112T100000Z-take001";
        private readonly string _root = Path.Combine(Path.GetTempPath(), "camera-golden-" + Guid.NewGuid().ToString("N"));

        public void Dispose()
        {
            try { Directory.Delete(_root, true); } catch (IOException) { }
        }

        /// <summary>
        /// Frame n is captured at 0.5 s + n/30 s (the converter test's own frame times, <c>conftest.jetrover_take</c>),
        /// queued 5 ms later, and is the ASCII bytes <c>front-n</c>. Frame 45 has no capture stamp.
        /// </summary>
        private void WriteTake(string session)
        {
            Directory.CreateDirectory(session);
            File.WriteAllText(Path.Combine(session, "session.json"), "{\"scenario\":{\"name\":\"reach\"}}");
            string control = Path.Combine(_root, ".active");
            File.WriteAllText(control, session + "\n");
            long now = 0;
            using var w = new CameraSessionWriter(control, () => now, Tps, minFreeBytes: 0, freeBytes: _ => long.MaxValue,
                startThread: false);
            w.Step();
            for (int n = 0; n <= 90; n++)
            {
                long captured = 500_000_000 + (long)Math.Round(n * 1e9 / 30);
                now = captured + 5_000_000;
                w.OfferFrame(Encoding.ASCII.GetBytes($"front-{n}"), (uint)n, n == 45 ? LatencyTrace.Unset : captured, 640, 360);
                w.Step();
            }

            File.WriteAllText(control, string.Empty);
            now += Tps;
            w.Step();
        }

        [Fact]
        public void TheConvertersGoldenTake_IsWhatThisWriterWrites()
        {
            string session = Path.Combine(_root, SessionId);
            WriteTake(session);
            string golden = GoldenDirectory();
            foreach (string file in new[] { "camera.jlog", Path.Combine("camera", "front.rgb.frames") })
            {
                byte[] written = File.ReadAllBytes(Path.Combine(session, file));
                string path = Path.Combine(golden, file);
                if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(path)!);
                    File.WriteAllBytes(path, written);
                }

                Assert.True(File.Exists(path), $"{path} is missing: run `just gen-camera-session-golden`");
                Assert.Equal(written, File.ReadAllBytes(path));
            }
        }

        private static string GoldenDirectory()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                if (Directory.Exists(Path.Combine(dir.FullName, "core", "RobotProfiles")) && Directory.Exists(Path.Combine(dir.FullName, "robot")))
                {
                    return Path.Combine(dir.FullName, "robot", "lerobot", "tests", "golden", "camera");
                }
            }

            throw new InvalidOperationException($"no repository root above {AppContext.BaseDirectory}");
        }
    }
}
