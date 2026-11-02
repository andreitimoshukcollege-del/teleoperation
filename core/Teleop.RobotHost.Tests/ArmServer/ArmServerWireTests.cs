using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using Teleop.Core.Types;
using Teleop.RobotHost.ArmServer;

namespace Teleop.RobotHost.Tests.ArmServer
{
    /// <summary>
    /// The arm-server wire (docs/adr/0030 decision 5), and the golden file that pins the Isaac bridge's copy of it
    /// (robot/isaac/golden/arm-server-wire-v1.json, read by robot/isaac/tests/test_wire.py). Regenerate the golden after a
    /// deliberate change with <c>just gen-arm-server-golden</c>, and change robot/isaac/teleop_isaac/wire.py with it.
    /// </summary>
    public class ArmServerWireTests
    {
        private static readonly byte[] Motors = { 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18 };

        [Fact]
        public void EveryMessage_RoundTrips()
        {
            var buffer = new byte[ArmServerWire.MaxBytes];

            Assert.True(ArmServerWire.TryEncodeHello(7, 123_456_789_012, 1_000_000_000, "fr3-duo", buffer, out int n));
            Assert.Equal(ArmServerWire.KindHello, ArmServerWire.KindOf(buffer.AsSpan(0, n)));
            Assert.Equal(7u, ArmServerWire.SequenceOf(buffer.AsSpan(0, n)));
            Assert.True(ArmServerWire.TryDecodeHello(buffer.AsSpan(0, n), out long ticks, out long tps, out ReadOnlySpan<byte> name));
            Assert.Equal((123_456_789_012L, 1_000_000_000L, "fr3-duo"), (ticks, tps, Encoding.UTF8.GetString(name)));

            Assert.True(ArmServerWire.TryEncodeWelcome(8, ArmServerKind.Isaac, 99, Motors, "Isaac Sim 6.2.0.0", buffer, out n));
            Assert.True(ArmServerWire.TryDecodeWelcome(buffer.AsSpan(0, n), out ArmServerKind kind, out long serverNow, out ReadOnlySpan<byte> ids, out ReadOnlySpan<byte> text));
            Assert.Equal(ArmServerKind.Isaac, kind);
            Assert.Equal(99, serverNow);
            Assert.Equal(Motors, ids.ToArray());
            Assert.Equal("Isaac Sim 6.2.0.0", Encoding.UTF8.GetString(text));

            var targets = new[] { new JointSample(1, -0.75f, false), new JointSample(8, 1f, false) };
            Assert.True(ArmServerWire.TryEncodeTargets(9, 5, targets, buffer, out n));
            var decoded = new JointSample[ArmServerWire.MaxMotors];
            Assert.True(ArmServerWire.TryDecodeTargets(buffer.AsSpan(0, n), out long sent, decoded, out int count));
            Assert.Equal(5, sent);
            Assert.Equal(new[] { (1, -0.75f), (8, 1f) }, decoded.Take(count).Select(t => ((int)t.MotorId, t.Value)));

            Assert.True(ArmServerWire.TryEncodeEnable(10, 6, false, buffer, out n));
            Assert.True(ArmServerWire.TryDecodeEnable(buffer.AsSpan(0, n), out long at, out bool enabled));
            Assert.Equal((6L, false), (at, enabled));

            float[] q = Motors.Select(m => m * 0.125f).ToArray(), v = Motors.Select(m => -m * 0.25f).ToArray(), c = Motors.Select(m => m * 0.5f).ToArray();
            var header = new ArmServerStateHeader(1000, 2000, ArmServerWire.FlagEnabled | ArmServerWire.FlagBehind);
            Assert.True(ArmServerWire.TryEncodeState(11, header, Motors, q, v, c, buffer, out n));
            var sid = new byte[ArmServerWire.MaxMotors];
            float[] sq = new float[ArmServerWire.MaxMotors], sv = new float[ArmServerWire.MaxMotors], sc = new float[ArmServerWire.MaxMotors];
            Assert.True(ArmServerWire.TryDecodeState(buffer.AsSpan(0, n), out ArmServerStateHeader got, sid, sq, sv, sc, out count));
            Assert.Equal((1000L, 2000L, true, true, false), (got.SensedTicks, got.ServerTicks, got.Enabled, got.Behind, got.Error));
            Assert.Equal(Motors, sid.Take(count));
            Assert.Equal(q, sq.Take(count));
            Assert.Equal(v, sv.Take(count));
            Assert.Equal(c, sc.Take(count));

            Assert.True(ArmServerWire.TryEncodeHealth(12, new ArmServerHealth(2.5f, 0.25f, 3, 1000), buffer, out n));
            Assert.True(ArmServerWire.TryDecodeHealth(buffer.AsSpan(0, n), out ArmServerHealth health));
            Assert.Equal((2.5f, 0.25f, 3u, 1000u), (health.RealTimeFactor, health.LagMilliseconds, health.LateSteps, health.Steps));
        }

        [Fact]
        public void ForeignOrMalformedDatagrams_AreRefused()
        {
            var buffer = new byte[ArmServerWire.MaxBytes];
            Assert.True(ArmServerWire.TryEncodeEnable(1, 2, true, buffer, out int n));
            byte[] good = buffer.AsSpan(0, n).ToArray();

            byte[] otherVersion = (byte[])good.Clone();
            otherVersion[2] = 2;
            Assert.Equal(0, ArmServerWire.KindOf(otherVersion));
            Assert.False(ArmServerWire.TryDecodeEnable(otherVersion, out _, out _));
            Assert.False(ArmServerWire.TryDecodeEnable(good.AsSpan(0, n - 1), out _, out _));
            byte[] notBool = (byte[])good.Clone();
            notBool[16] = 2;
            Assert.False(ArmServerWire.TryDecodeEnable(notBool, out _, out _));
            Assert.Equal(0, ArmServerWire.KindOf(new byte[] { (byte)'X', (byte)'S', 1, 4, 0, 0, 0, 0 }));

            // A state whose count says more motors than the datagram holds, or than the caller has room for.
            float[] f = new float[ArmServerWire.MaxMotors];
            Assert.True(ArmServerWire.TryEncodeState(1, new ArmServerStateHeader(1, 2, 0), Motors, f, f, f, buffer, out n));
            Assert.False(ArmServerWire.TryDecodeState(buffer.AsSpan(0, n - 1), out _, new byte[36], f, f, f, out _));
            Assert.False(ArmServerWire.TryDecodeState(buffer.AsSpan(0, n), out _, new byte[4], f, f, f, out _));
            Assert.False(ArmServerWire.TryEncodeState(1, new ArmServerStateHeader(1, 2, 0), new byte[ArmServerWire.MaxMotors + 1], f, f, f, buffer, out _));
            Assert.False(ArmServerWire.TryEncodeHello(1, 1, 1, new string('x', ArmServerWire.MaxNameBytes + 1), buffer, out _));
        }

        [Fact]
        public void TheIsaacBridgesGoldenFile_IsWhatThisCodecWrites()
        {
            string expected = GoldenJson();
            string path = GoldenPath();
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, expected, new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-arm-server-golden`");
            Assert.Equal(expected, File.ReadAllText(path).Replace("\r\n", "\n"));
        }

        /// <summary>Fixed messages of every kind, their fields and their exact bytes; values exact in float32.</summary>
        internal static string GoldenJson()
        {
            var buffer = new byte[ArmServerWire.MaxBytes];
            var messages = new List<Dictionary<string, object>>();
            void Add(Dictionary<string, object> fields, int n)
            {
                fields["hex"] = Convert.ToHexString(buffer, 0, n).ToLowerInvariant();
                messages.Add(fields);
            }

            ArmServerWire.TryEncodeHello(1, 123_456_789_012, 1_000_000_000, "fr3-duo", buffer, out int n);
            Add(new Dictionary<string, object> { ["kind"] = "hello", ["sequence"] = 1, ["hostTicks"] = 123_456_789_012L, ["ticksPerSecond"] = 1_000_000_000L, ["profile"] = "fr3-duo" }, n);

            ArmServerWire.TryEncodeWelcome(2, ArmServerKind.Isaac, 123_456_790_000, Motors, "Isaac Sim 6.2.0.0, PhysX 1 kHz", buffer, out n);
            Add(new Dictionary<string, object> { ["kind"] = "welcome", ["sequence"] = 2, ["server"] = (int)ArmServerKind.Isaac, ["serverNowTicks"] = 123_456_790_000L,
                ["motors"] = Motors.Select(m => (int)m).ToArray(), ["description"] = "Isaac Sim 6.2.0.0, PhysX 1 kHz" }, n);

            var targets = new[] { new JointSample(1, 0.5f, false), new JointSample(4, -2.25f, false), new JointSample(8, 0.75f, false) };
            ArmServerWire.TryEncodeTargets(3, 123_456_800_000, targets, buffer, out n);
            Add(new Dictionary<string, object> { ["kind"] = "targets", ["sequence"] = 3, ["hostTicks"] = 123_456_800_000L,
                ["targets"] = targets.Select(t => new object[] { (int)t.MotorId, t.Value }).ToArray() }, n);

            ArmServerWire.TryEncodeEnable(4, 123_456_900_000, false, buffer, out n);
            Add(new Dictionary<string, object> { ["kind"] = "enable", ["sequence"] = 4, ["hostTicks"] = 123_456_900_000L, ["enabled"] = false }, n);

            byte[] ids = { 1, 7, 8 };
            float[] q = { 0.25f, -1.5f, 0.5f }, v = { 0.125f, 0f, -0.25f }, c = { 0.375f, -1.25f, 1f };
            ArmServerWire.TryEncodeState(5, new ArmServerStateHeader(123_457_000_000, 4_000_000, ArmServerWire.FlagEnabled), ids, q, v, c, buffer, out n);
            Add(new Dictionary<string, object> { ["kind"] = "state", ["sequence"] = 5, ["sensedTicks"] = 123_457_000_000L, ["serverTicks"] = 4_000_000L,
                ["flags"] = (int)ArmServerWire.FlagEnabled,
                ["motors"] = ids.Select((m, i) => new object[] { (int)m, q[i], v[i], c[i] }).ToArray() }, n);

            ArmServerWire.TryEncodeHealth(6, new ArmServerHealth(2.5f, 0.75f, 3, 60_000), buffer, out n);
            Add(new Dictionary<string, object> { ["kind"] = "health", ["sequence"] = 6, ["realTimeFactor"] = 2.5f, ["lagMilliseconds"] = 0.75f,
                ["lateSteps"] = 3u, ["steps"] = 60_000u }, n);

            var golden = new Dictionary<string, object>
            {
                ["version"] = (int)ArmServerWire.Version,
                ["about"] = "Arm-server wire v1 (docs/adr/0030 decision 5): fixed messages, their fields and their bytes, written by ArmServerWireTests.",
                ["messages"] = messages,
            };
            // LF on every machine: the file is the same bytes whichever one wrote it.
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
                    return Path.Combine(golden, "arm-server-wire-v1.json");
                }
            }

            throw new InvalidOperationException($"no repository root above {AppContext.BaseDirectory}");
        }
    }
}
