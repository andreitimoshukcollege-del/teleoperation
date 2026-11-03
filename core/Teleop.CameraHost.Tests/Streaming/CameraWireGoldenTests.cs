using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using Teleop.CameraHost.Streaming;
using Teleop.Core.Camera;
using Xunit;

namespace Teleop.CameraHost.Tests.Streaming
{
    /// <summary>
    /// The golden file that pins the Isaac renderer's Python copy of ADR 0014's camera wire (docs/adr/0032 decision 3,
    /// robot/isaac/teleop_isaac/camwire.py and camfeed.py): a frame's chunks, the viewer's keepalive, and the pacer's
    /// decisions, as this code writes them. `just gen-camera-wire-golden` rewrites it after a deliberate change.
    /// </summary>
    public sealed class CameraWireGoldenTests
    {
        [Fact]
        public void TheIsaacRenderersGoldenFile_IsWhatThisCodeWrites()
        {
            string expected = GoldenJson();
            string path = GoldenPath();
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, expected, new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-camera-wire-golden`");
            Assert.Equal(expected, File.ReadAllText(path).Replace("\r\n", "\n"));
        }

        /// <summary>The frame's bytes: byte i is (7 i + 3) mod 256.</summary>
        internal static byte[] Pattern(int length) => Enumerable.Range(0, length).Select(i => (byte)((7 * i + 3) & 0xFF)).ToArray();

        internal static string GoldenJson()
        {
            var codec = new CameraChunkCodec();
            var buffer = new byte[CameraChunkCodec.MaxDatagramBytes];
            var frames = new List<Dictionary<string, object>>();
            foreach ((uint id, long capture, long send, int length, ushort w, ushort h) in new[]
            {
                (0x01020304u, 123_456_789_012L, 123_456_790_000L, 2500, (ushort)640, (ushort)360),
                (7u, -1L, 5_000_000_000L, CameraChunkCodec.MaxPayloadBytes, (ushort)320, (ushort)180),
            })
            {
                var stamp = new CameraFrameStamp(id, capture, send, 1_000_000_000, w, h);
                byte[] jpeg = Pattern(length);
                var chunks = new List<string>();
                for (int i = 0; i < CameraChunkCodec.ChunkCountFor(length); i++)
                {
                    Assert.True(codec.TryEncodeChunk(stamp, jpeg, i, buffer, out int n));
                    chunks.Add(Convert.ToHexString(buffer, 0, n).ToLowerInvariant());
                }

                frames.Add(new Dictionary<string, object>
                {
                    ["frameId"] = id, ["captureTicks"] = capture, ["sendTicks"] = send, ["ticksPerSecond"] = 1_000_000_000L,
                    ["width"] = (int)w, ["height"] = (int)h, ["length"] = length, ["chunks"] = chunks,
                });
            }

            var subscribe = new CameraSubscribeCodec();
            var keepalives = new List<Dictionary<string, object>>();
            foreach ((ushort fps, long ticks) in new[] { ((ushort)0, 987_654_321L), ((ushort)15, -42L) })
            {
                Assert.True(subscribe.TryEncode(fps, ticks, buffer, out int n));
                keepalives.Add(new Dictionary<string, object> { ["maxFramesPerSecond"] = (int)fps, ["sendTicks"] = ticks,
                    ["hex"] = Convert.ToHexString(buffer, 0, n).ToLowerInvariant() });
            }

            // Frames every 33.3 ms with jitter, asked for at 30, 15 and no limit, then a reset (a new subscriber).
            var pacer = new FramePacer(1_000_000_000);
            var steps = new List<object[]>();
            long t = 0;
            for (int k = 0; k < 24; k++)
            {
                t += 33_333_333 + (k % 3 - 1) * 1_500_000;
                ushort fps = k < 8 ? (ushort)30 : k < 16 ? (ushort)15 : (ushort)0;
                if (k == 20) pacer.Reset();
                bool send = pacer.ShouldSend(t, fps);
                if (send) pacer.MarkSent(t);
                steps.Add(new object[] { t, (int)fps, k == 20, send });
            }

            var golden = new Dictionary<string, object>
            {
                ["format"] = "camera-wire-golden", ["version"] = 1, ["maxDatagramBytes"] = CameraChunkCodec.MaxDatagramBytes,
                ["headerBytes"] = CameraChunkCodec.HeaderSize, ["framePattern"] = "byte i is (7 i + 3) mod 256",
                ["frames"] = frames, ["keepalives"] = keepalives,
                ["pacer"] = new Dictionary<string, object> { ["ticksPerSecond"] = 1_000_000_000L, ["steps"] = steps, ["stepFields"] = "frameTicks, requestedMaxFps, resetFirst, send" },
            };
            return JsonSerializer.Serialize(golden, new JsonSerializerOptions { WriteIndented = true }).Replace("\r\n", "\n") + "\n";
        }

        private static string GoldenPath()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                if (Directory.Exists(Path.Combine(dir.FullName, "core", "RobotProfiles")) && Directory.Exists(Path.Combine(dir.FullName, "robot")))
                {
                    string golden = Path.Combine(dir.FullName, "robot", "isaac", "golden");
                    Directory.CreateDirectory(golden);
                    return Path.Combine(golden, "camera-wire-v1.json");
                }
            }

            throw new InvalidOperationException($"no repository root above {AppContext.BaseDirectory}");
        }
    }
}
