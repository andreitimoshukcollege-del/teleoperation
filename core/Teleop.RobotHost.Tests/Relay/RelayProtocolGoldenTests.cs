using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Relay
{
    /// <summary>
    /// The relay protocol's golden file, robot/ros/src/teleop_relay/test/golden/relay-protocol-v5.json: fixed messages of
    /// every kind with their exact bytes, and messages this codec rejects. relay_protocol.py's test_relay_protocol_golden.py
    /// decodes and encodes the same file, so the two ends cannot drift apart while their tests stay green (the C# and the
    /// Python copy change together, and nothing else pinned them).
    /// </summary>
    public sealed class RelayProtocolGoldenTests
    {
        [Fact]
        public void TheRelaysGoldenFile_IsWhatThisCodecWrites()
        {
            string expected = GoldenJson();
            string path = GoldenPath();
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, expected, new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-relay-golden`");
            Assert.Equal(expected, File.ReadAllText(path).Replace("\r\n", "\n"));
        }

        /// <summary>Every message, its fields and bytes (values exact in float32), and rejected bytes with the decoder that rejects them.</summary>
        internal static string GoldenJson()
        {
            var buffer = new byte[RelayProtocol.CommandEncodedSize(RelayProtocol.MaxJointsPerMessage + 1)];
            var messages = new List<Dictionary<string, object>>();
            string Hex(int n) => Convert.ToHexString(buffer, 0, n).ToLowerInvariant();

            foreach (JointTarget[] targets in new[]
            {
                new[] { new JointTarget(1, 500f, 300f), new JointTarget(2, -3.25f, 300f), new JointTarget(3, 1.5f, 250f), new JointTarget(10, 812.5f, 119f) },
                Array.Empty<JointTarget>(),
            })
            {
                int n = RelayProtocol.EncodeCommand(targets, buffer);
                messages.Add(new Dictionary<string, object>
                {
                    ["kind"] = "command",
                    ["targets"] = targets.Select(t => new object[] { (int)t.MotorId, t.Angle, t.Speed }).ToArray(),
                    ["hex"] = Hex(n),
                });
            }

            foreach (JointFeedbackEntry[] entries in new[]
            {
                new[] { new JointFeedbackEntry(1, true, 500.5f), new JointFeedbackEntry(5, false, 0f), new JointFeedbackEntry(10, true, 123.25f) },
                Array.Empty<JointFeedbackEntry>(),
            })
            {
                int n = RelayProtocol.EncodeFeedback(entries, buffer);
                messages.Add(new Dictionary<string, object>
                {
                    ["kind"] = "feedback",
                    ["entries"] = entries.Select(e => new object[] { (int)e.MotorId, e.Valid, e.Pulse }).ToArray(),
                    ["hex"] = Hex(n),
                });
            }

            foreach (bool on in new[] { true, false })
            {
                int n = RelayProtocol.EncodeTorque(on, buffer);
                messages.Add(new Dictionary<string, object> { ["kind"] = "torque", ["on"] = on, ["hex"] = Hex(n) });
            }

            // Bytes each end must refuse, and which decoder refuses them. Asserted here against this codec, and in Python
            // against relay_protocol.py.
            int good = RelayProtocol.EncodeCommand(new[] { new JointTarget(7, 500f, 119f) }, buffer);
            byte[] command = buffer.Take(good).ToArray();
            byte[] tooMany = new byte[RelayProtocol.CommandEncodedSize(RelayProtocol.MaxJointsPerMessage + 1)];
            tooMany[0] = RelayProtocol.Version;
            tooMany[1] = RelayProtocol.KindCommand;
            tooMany[2] = RelayProtocol.MaxJointsPerMessage + 1;
            var rejects = new List<(string Decoder, string Why, byte[] Bytes)>
            {
                ("command", "an older version", With(command, 0, 4)),
                ("command", "the feedback kind", With(command, 1, RelayProtocol.KindFeedback)),
                ("command", "an unknown kind", With(command, 1, 9)),
                ("command", "count says 2, one tuple follows", With(command, 2, 2)),
                ("command", "more joints than MaxJointsPerMessage", tooMany),
                ("command", "shorter than a header", command.Take(2).ToArray()),
                ("torque", "a value other than 0 and 1", new byte[] { RelayProtocol.Version, RelayProtocol.KindTorque, 2 }),
                ("torque", "an older version", new byte[] { 4, RelayProtocol.KindTorque, 1 }),
                ("torque", "the command kind", new byte[] { RelayProtocol.Version, RelayProtocol.KindCommand, 1 }),
                ("torque", "shorter than a torque message", new byte[] { RelayProtocol.Version, RelayProtocol.KindTorque }),
            };

            var targetsBuffer = new JointTarget[RelayProtocol.MaxJointsPerMessage + 1];
            foreach (var (decoder, why, bytes) in rejects)
            {
                bool accepted = decoder == "command"
                    ? RelayProtocol.TryDecodeCommand(bytes, targetsBuffer, out _)
                    : RelayProtocol.TryDecodeTorque(bytes, out _);
                Assert.False(accepted, $"{decoder} accepted {why}");
            }

            var golden = new Dictionary<string, object>
            {
                ["version"] = (int)RelayProtocol.Version,
                ["about"] = "The protocol between RobotHost and the ROS relay (core/Teleop.RobotHost/Relay/RelayProtocol.cs and robot/ros/src/teleop_relay/teleop_relay/relay_protocol.py): " +
                    "fixed messages and their bytes, and bytes both ends reject. Written by RelayProtocolGoldenTests (just gen-relay-golden).",
                ["messages"] = messages,
                ["rejects"] = rejects.Select(r => new Dictionary<string, object>
                {
                    ["decoder"] = r.Decoder, ["why"] = r.Why, ["hex"] = Convert.ToHexString(r.Bytes).ToLowerInvariant(),
                }).ToArray(),
            };
            // LF on every machine: the file is the same bytes whichever one wrote it.
            return JsonSerializer.Serialize(golden, new JsonSerializerOptions { WriteIndented = true }).Replace("\r\n", "\n") + "\n";
        }

        private static byte[] With(byte[] source, int index, byte value)
        {
            byte[] copy = (byte[])source.Clone();
            copy[index] = value;
            return copy;
        }

        private static string GoldenPath()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                string candidate = Path.Combine(dir.FullName, "robot", "ros", "src", "teleop_relay", "test");
                if (Directory.Exists(candidate) && Directory.Exists(Path.Combine(dir.FullName, "core")))
                {
                    Directory.CreateDirectory(Path.Combine(candidate, "golden"));
                    return Path.Combine(candidate, "golden", "relay-protocol-v5.json");
                }
            }

            throw new DirectoryNotFoundException("could not find robot/ros/src/teleop_relay/test from the test's output folder");
        }
    }
}
