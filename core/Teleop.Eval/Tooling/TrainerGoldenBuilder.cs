using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Text.Json;
using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Transport.Impairments;
using Teleop.Core.Types;
using Teleop.Eval.Sweep;

namespace Teleop.Eval.Tooling
{
    /// <summary>
    /// <c>gen-trainer-golden</c> (docs/adr/0021 decision 5): writes the golden files that pin the
    /// trainer's Python copies of Core to the C# originals, into <c>trainer/contract/golden/</c>.
    ///
    /// <para>The trainer (<c>trainer/</c>) runs on a GPU machine with no .NET, so it carries its own
    /// copies of <see cref="SeededRng"/>, the impairments and <see cref="EmulatedTransport"/>'s
    /// ordering, the profile catalog, <see cref="ClockSync"/> and <see cref="RelayReplayPredictor"/>.
    /// These files are what Core actually produced for fixed inputs; the trainer's tests replay the
    /// same inputs and must match them exactly, and <c>TrainerGoldenFilesTests</c> fails here if Core
    /// stops reproducing them. Regenerate after any change to those classes, inspect the diff, and
    /// commit both sides in the same change.</para>
    ///
    /// <para>Everything is built from fixed seeds, so two runs write identical bytes. Unsigned
    /// 64-bit values are written as decimal strings (JSON readers disagree about integers past
    /// 2^53), and single-precision values as their IEEE bit pattern next to the readable value,
    /// because "close" is not what these files test.</para>
    /// </summary>
    public static class TrainerGoldenBuilder
    {
        /// <summary>Each golden file's name and its content; the test compares these to the committed copies.</summary>
        public static IReadOnlyList<(string Name, string Content)> BuildAll(string syntheticBurstTracePath)
        {
            return new[]
            {
                ("rng-v1.json", Rng()),
                ("impairments-v1.json", Impairments(syntheticBurstTracePath)),
                ("clocksync-v1.json", ClockSyncCases()),
                ("relay-replay-v1.json", RelayReplay()),
            };
        }

        public static void Write(string outputDirectory, string syntheticBurstTracePath)
        {
            Directory.CreateDirectory(outputDirectory);
            foreach ((string name, string content) in BuildAll(syntheticBurstTracePath))
            {
                File.WriteAllText(Path.Combine(outputDirectory, name), content, new UTF8Encoding(false));
            }

            // The trainer cannot read core/testdata, so it carries a byte-identical copy of the trace.
            string traces = Path.Combine(Path.GetDirectoryName(Path.GetFullPath(outputDirectory))!, "traces");
            Directory.CreateDirectory(traces);
            File.Copy(syntheticBurstTracePath, Path.Combine(traces, "synthetic-burst.trace"), overwrite: true);
        }

        // ---------------------------------------------------------------- SeededRng, ImpairmentStreams

        private static readonly ulong[] RngSeeds = { 0UL, 1UL, 2UL, 42UL, 20260804UL, 0x8000000000000000UL, ulong.MaxValue };
        private static readonly string[] AxisNames = { "delay", "jitter", "loss", "reorder", "delay-trace" };

        private static string Rng()
        {
            return Json(w =>
            {
                w.WriteString("about", "SeededRng and ImpairmentStreams outputs for fixed seeds (TrainerGoldenBuilder).");
                w.WriteStartArray("streams");
                foreach (ulong seed in RngSeeds)
                {
                    var rng = new SeededRng(seed);
                    w.WriteStartObject();
                    w.WriteString("seed", seed.ToString());
                    w.WriteStartArray("uint64");
                    for (int i = 0; i < 16; i++) w.WriteStringValue(rng.NextUInt64().ToString());
                    w.WriteEndArray();
                    w.WriteStartArray("double");
                    for (int i = 0; i < 16; i++) w.WriteNumberValue(rng.NextDouble());
                    w.WriteEndArray();
                    w.WriteEndObject();
                }

                w.WriteEndArray();
                w.WriteStartObject("streamIds");
                foreach (string axis in AxisNames) w.WriteString(axis, ImpairmentStreams.StreamIdFromName(axis).ToString());
                w.WriteEndObject();
                w.WriteStartArray("derive");
                foreach (ulong seed in RngSeeds)
                {
                    foreach (string axis in AxisNames)
                    {
                        w.WriteStartObject();
                        w.WriteString("seed", seed.ToString());
                        w.WriteString("axis", axis);
                        w.WriteString("value", ImpairmentStreams.DeriveForAxis(seed, axis).ToString());
                        w.WriteEndObject();
                    }
                }

                w.WriteEndArray();
            });
        }

        // ---------------------------------------------------------------- impairments through EmulatedTransport

        private const int DatagramCount = 400;
        private const int Capacity = 1024; // never binds: the trainer models an unbounded link

        private static readonly string[] ProfileNames =
        {
            "lan", "50ms-5j", "150ms-20j-0.5loss", "300ms-60j-2loss-bursty",
            "delay-250ms", "jitter-30ms", "loss-5pct", "combo__delay-80ms__jitter-15ms__loss-3pct",
        };

        private static readonly ulong[] LinkSeeds = { 1UL, 2UL, 42UL };

        /// <summary>Send times: a 20 ms period with a small repeating wobble, so ties and near-ties occur.</summary>
        private static long[] SendTicks(long ticksPerSecond)
        {
            var t = new long[DatagramCount];
            long period = ticksPerSecond / 50;
            long wobble = ticksPerSecond * 3 / 10_000; // 0.3 ms
            for (int i = 0; i < DatagramCount; i++) t[i] = 1_000 * ticksPerSecond + i * period + (i % 7) * wobble;
            return t;
        }

        private static string Impairments(string syntheticBurstTracePath)
        {
            return Json(w =>
            {
                w.WriteString("about", "Per-datagram fates through Core's EmulatedTransport over a LoopbackTransport (TrainerGoldenBuilder).");
                w.WriteNumber("datagrams", DatagramCount);
                w.WriteStartArray("cases");
                foreach (long tps in new[] { 10_000_000L, 1_000_000_000L })
                {
                    foreach (string name in ProfileNames)
                    {
                        if (!Teleop.Core.Transport.NetworkProfileCatalog.TryResolveParametric(name, tps, out NetworkProfile profile, out string? error))
                        {
                            throw new InvalidOperationException(error);
                        }

                        foreach (ulong seed in LinkSeeds)
                        {
                            WriteCase(w, name, tps, seed, () => Teleop.Core.Transport.NetworkProfileCatalog.CreateImpairments(profile));
                        }
                    }

                    // No catalog profile reorders, so one hand-built set exercises that axis.
                    var reorder = new NetworkProfile(
                        baseDelayTicks: tps / 25, jitterTicks: tps / 200, lossProbabilityAfterDelivered: 0.02,
                        lossProbabilityAfterLost: 0.3, reorderProbability: 0.1, reorderDelayTicks: tps * 3 / 100);
                    foreach (ulong seed in LinkSeeds)
                    {
                        WriteCase(w, "custom:delay40-jitter5-ge0.02/0.3-reorder0.1@30ms", tps, seed,
                            () => Teleop.Core.Transport.NetworkProfileCatalog.CreateImpairments(reorder));
                    }
                }

                // synthetic-burst at its own rate (Eval refuses another; the trainer rescales, separately tested).
                (long traceTps, long[] samples) = TraceFile.Read(syntheticBurstTracePath);
                foreach (ulong seed in LinkSeeds)
                {
                    WriteCase(w, "synthetic-burst", traceTps, seed,
                        () => new NamedProfile("synthetic-burst", default, samples).CreateImpairments());
                }

                w.WriteEndArray();
            });
        }

        private static void WriteCase(Utf8JsonWriter w, string name, long tps, ulong seed, Func<INetworkImpairment[]> impairments)
        {
            long[] send = SendTicks(tps);
            var emulated = new EmulatedTransport(new LoopbackTransport(8, Capacity), impairments(), seed, Capacity);
            var delivered = new bool[DatagramCount];
            var arrival = new long[DatagramCount];
            var order = new List<int>(DatagramCount);
            byte[] payload = new byte[4];
            for (int i = 0; i < DatagramCount; i++)
            {
                BitConverter.TryWriteBytes(payload, i);
                delivered[i] = emulated.Send(payload, send[i]);
            }

            byte[] buffer = new byte[8];
            while (emulated.TryReceive(long.MaxValue / 4, buffer, out int count, out long at))
            {
                int index = BitConverter.ToInt32(buffer, 0);
                arrival[index] = at;
                order.Add(index);
            }

            w.WriteStartObject();
            w.WriteString("profile", name);
            w.WriteNumber("ticksPerSecond", tps);
            w.WriteString("seed", seed.ToString());
            w.WriteStartArray("send");
            foreach (long t in send) w.WriteNumberValue(t);
            w.WriteEndArray();
            w.WriteStartArray("delivered");
            foreach (bool d in delivered) w.WriteNumberValue(d ? 1 : 0);
            w.WriteEndArray();
            w.WriteStartArray("arrival");
            foreach (long a in arrival) w.WriteNumberValue(a);
            w.WriteEndArray();
            w.WriteStartArray("order");
            foreach (int i in order) w.WriteNumberValue(i);
            w.WriteEndArray();
            w.WriteEndObject();
        }

        // ---------------------------------------------------------------- ClockSync

        private static string ClockSyncCases()
        {
            return Json(w =>
            {
                w.WriteString("about", "ClockSync state after each AddRoundTrip on fixed exchanges (TrainerGoldenBuilder).");
                w.WriteStartArray("cases");
                // The headset's config (JetRoverArmConfig) at the Galaxy XR's rate and at the editor's,
                // and a small history so the ring wraps and the outlier gate bites.
                WriteClockCase(w, "headset-android", new ClockSyncConfig(32, 0.2f, 2_000_000_000L, 3.0, 3),
                    operatorTps: 1_000_000_000L, offsetTicks: 37_123_456_789_012L, seed: 11);
                WriteClockCase(w, "headset-editor", new ClockSyncConfig(32, 0.2f, 20_000_000L, 3.0, 3),
                    operatorTps: 10_000_000L, offsetTicks: -912_345_678_901L, seed: 12);
                WriteClockCase(w, "small-history", new ClockSyncConfig(8, 0.5f, 5_000_000L, 2.0, 2),
                    operatorTps: 10_000_000L, offsetTicks: 4_321L, seed: 13);
                w.WriteEndArray();
            });
        }

        private static void WriteClockCase(Utf8JsonWriter w, string name, ClockSyncConfig config, long operatorTps,
            long offsetTicks, ulong seed)
        {
            const long robotTps = 1_000_000_000L;
            var sync = new ClockSync(config);
            var rng = new SeededRng(seed);
            w.WriteStartObject();
            w.WriteString("name", name);
            w.WriteStartObject("config");
            w.WriteNumber("historyCapacity", config.HistoryCapacity);
            w.WriteNumber("smoothingAlpha", config.SmoothingAlpha);
            w.WriteNumber("maxAcceptableRttTicks", config.MaxAcceptableRttTicks);
            w.WriteNumber("outlierRttMultiple", config.OutlierRttMultiple);
            w.WriteNumber("minSamplesBeforeTrusted", config.MinSamplesBeforeTrusted);
            w.WriteEndObject();
            w.WriteNumber("operatorTicksPerSecond", operatorTps);
            w.WriteNumber("robotTicksPerSecond", robotTps);
            w.WriteStartArray("steps");
            long robotNow = 3_600L * robotTps; // an hour since the robot booted
            double ratio = (double)operatorTps / robotTps;
            for (int i = 0; i < 120; i++)
            {
                robotNow += robotTps / 2; // one keepalive echo every half second
                // One-way delays in ms: mostly a few, sometimes long, now and then an outlier, and a
                // few exchanges whose robot stamps are inconsistent so the RTT comes out negative.
                double upMs = 1.0 + 8.0 * rng.NextDouble();
                double downMs = 1.0 + 8.0 * rng.NextDouble();
                double kind = rng.NextDouble();
                if (kind < 0.08) upMs += 400.0 * rng.NextDouble();
                else if (kind < 0.12) downMs += 2_500.0 * rng.NextDouble();
                long robotRecv = robotNow;
                long robotSend = robotRecv + (long)(robotTps * (0.2 + 2.0 * rng.NextDouble()) / 1000.0);
                long operatorSend = (long)Math.Round((robotRecv - (long)(upMs / 1000.0 * robotTps)) * ratio) + offsetTicks;
                long operatorRecv = (long)Math.Round((robotSend + (long)(downMs / 1000.0 * robotTps)) * ratio) + offsetTicks;
                // Robot turnaround twice the operator's round trip: the RTT comes out negative.
                if (kind > 0.97) robotSend = robotRecv + (long)((operatorRecv - operatorSend) / ratio * 2.0);
                bool accepted = sync.AddRoundTrip(operatorSend, operatorTps, robotRecv, robotSend, robotTps, operatorRecv);
                ClockSyncDiagnostics d = sync.Diagnostics;
                long probe = robotNow + robotTps / 7;
                w.WriteStartObject();
                w.WriteNumber("operatorSend", operatorSend);
                w.WriteNumber("robotRecv", robotRecv);
                w.WriteNumber("robotSend", robotSend);
                w.WriteNumber("operatorRecv", operatorRecv);
                w.WriteBoolean("accepted", accepted);
                w.WriteNumber("offset", d.OffsetTicks);
                w.WriteNumber("uncertainty", d.OffsetUncertaintyTicks);
                w.WriteNumber("lastRtt", d.LastRttTicks);
                w.WriteNumber("minRtt", d.MinRttTicks);
                w.WriteNumber("acceptedCount", d.AcceptedSampleCount);
                w.WriteNumber("rejectedCount", d.RejectedSampleCount);
                w.WriteBoolean("synced", d.IsSynced);
                w.WriteNumber("probeRobot", probe);
                w.WriteNumber("probeOperator", sync.ToOperatorTicks(probe, robotTps, operatorTps));
                w.WriteEndObject();
            }

            w.WriteEndArray();
            w.WriteEndObject();
        }

        // ---------------------------------------------------------------- relay-replay

        private static readonly byte[] ArmMotors = { 1, 2, 3, 4, 5 };
        private const byte Gripper = 10;

        private static string RelayReplay()
        {
            return Json(w =>
            {
                w.WriteString("about", "RelayReplayPredictor outputs on fixed call sequences (TrainerGoldenBuilder).");
                w.WriteStartArray("cases");
                const long tps = 1_000_000_000L;
                // The headset's config (JetRoverJointStateLink with JetRoverArmConfig's defaults).
                var headset = new JointPredictorConfig(
                    maxJoints: 13, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: 6f,
                    instantMotorIds: new byte[0], maxHorizonTicks: 500L * tps / 1000L, ticksPerSecond: tps,
                    scaledMotorIds: new byte[] { Gripper }, speedScales: new[] { 0.358f },
                    sensedAgeMotorIds: ArmMotors, sensedAgeSeconds: new[] { 0.045f, 0.035f, 0.025f, 0.015f, 0.005f });
                // A small ring, an instant gripper and no reading ages: the ring wraps, the instant path runs.
                var small = new JointPredictorConfig(
                    maxJoints: 6, commandHistoryCapacity: 8, maxJointSpeedRadPerSec: 5f,
                    instantMotorIds: new[] { Gripper }, maxHorizonTicks: 300L * tps / 1000L, ticksPerSecond: tps,
                    relayMinMoveSeconds: 0.04f, relayDeadbandRad: 0.01f, relayMoveSpeedRadPerSec: 7f);
                WriteRelayCase(w, "headset-blind", headset, tps, seed: 21, speedMode: 0);
                WriteRelayCase(w, "headset-speeds", headset, tps, seed: 22, speedMode: 1);
                WriteRelayCase(w, "small-ring-instant-gripper", small, tps, seed: 23, speedMode: 1);
                w.WriteEndArray();
            });
        }

        private static void WriteRelayCase(Utf8JsonWriter w, string name, JointPredictorConfig config, long tps,
            ulong seed, int speedMode)
        {
            var predictor = new RelayReplayPredictor(config);
            var rng = new SeededRng(seed);
            var output = new JointSample[config.MaxJoints];
            var targets = new JointSample[ArmMotors.Length + 1];
            var joints = new JointSample[ArmMotors.Length + 1];
            var position = new float[ArmMotors.Length + 1];
            var goal = new float[ArmMotors.Length + 1];

            w.WriteStartObject();
            w.WriteString("name", name);
            WriteConfig(w, config);
            w.WriteStartArray("calls");

            long start = 5_000L * tps;
            long ms = tps / 1000;
            long nextCommand = start, nextState = start + 3 * ms, nextPredict = start + 1 * ms, nextGoal = start;
            long lastSensed = 0;
            float speed = 0f;
            for (long now = start; now < start + 12_000 * ms; now += ms)
            {
                if (now == start + 6_000 * ms)
                {
                    predictor.Reset();
                    w.WriteStartObject();
                    w.WriteString("op", "reset");
                    w.WriteEndObject();
                }

                if (now >= nextGoal)
                {
                    for (int j = 0; j < goal.Length; j++) goal[j] = (float)(rng.NextDouble() * 2.0 - 1.0);
                    goal[goal.Length - 1] = rng.NextDouble() < 0.5 ? 0f : 1f;
                    double s = rng.NextDouble();
                    speed = speedMode == 0 ? 0f : s < 0.3 ? 0f : s < 0.7 ? 2.5f : s < 0.9 ? 1.2f : 50f;
                    nextGoal = now + (long)((400 + 1_100 * rng.NextDouble()) * ms);
                }

                if (now >= nextCommand)
                {
                    // 48 Hz, the target easing toward the goal so consecutive commands are close.
                    for (int j = 0; j < goal.Length; j++) position[j] += 0.15f * (goal[j] - position[j]);
                    for (int j = 0; j < ArmMotors.Length; j++) targets[j] = new JointSample(ArmMotors[j], position[j], false);
                    targets[ArmMotors.Length] = new JointSample(Gripper, position[ArmMotors.Length], false);
                    long effective = now + 4 * ms;
                    // Now and then an older command, which must be rejected.
                    if (rng.NextDouble() < 0.02) effective -= 60 * ms;
                    predictor.ObserveCommand(effective, targets, speed);
                    WriteCommand(w, effective, speed, targets);
                    nextCommand = now + 21 * ms;
                }

                bool gap = now > start + 3_000 * ms && now < start + 3_700 * ms; // 700 ms with no state
                if (now >= nextState && !gap)
                {
                    // Fresh readings at 20 Hz, each sent twice or three times (the 50 Hz repeats).
                    long sensed = rng.NextDouble() < 0.6 ? now - 7 * ms : lastSensed;
                    if (sensed == 0) sensed = now - 7 * ms;
                    if (rng.NextDouble() < 0.03) sensed -= 120 * ms; // an out-of-order frame
                    for (int j = 0; j < ArmMotors.Length; j++)
                    {
                        float lagged = position[j] - (float)(0.05 * (rng.NextDouble() - 0.5));
                        joints[j] = new JointSample(ArmMotors[j], lagged, true);
                    }

                    joints[ArmMotors.Length] = new JointSample(Gripper, position[ArmMotors.Length], false);
                    predictor.ObserveState(sensed, joints);
                    WriteState(w, sensed, joints);
                    lastSensed = sensed;
                    nextState = now + 20 * ms;
                }

                if (now >= nextPredict)
                {
                    long target = now + (long)((rng.NextDouble() * 80.0 - 20.0) * ms);
                    bool ok = predictor.TryPredict(target, output, out int count);
                    PredictorDiagnostics d = predictor.Diagnostics;
                    w.WriteStartObject();
                    w.WriteString("op", "predict");
                    w.WriteNumber("t", target);
                    w.WriteBoolean("ok", ok);
                    WriteSamples(w, "out", output, ok ? count : 0);
                    w.WriteNumber("horizon", d.HorizonTicks);
                    w.WriteNumber("accepted", d.AcceptedObservations);
                    w.WriteNumber("rejected", d.RejectedObservations);
                    w.WriteEndObject();
                    nextPredict = now + 11 * ms;
                }
            }

            w.WriteEndArray();
            RelayReplayPredictor.GateCounts gate = predictor.Gate;
            w.WriteStartObject("gate");
            w.WriteNumber("written", gate.Written);
            w.WriteNumber("deadbandDrops", gate.DeadbandDrops);
            w.WriteNumber("coalesced", gate.Coalesced);
            w.WriteEndObject();
            w.WriteEndObject();
        }

        private static void WriteConfig(Utf8JsonWriter w, JointPredictorConfig c)
        {
            w.WriteStartObject("config");
            w.WriteNumber("maxJoints", c.MaxJoints);
            w.WriteNumber("commandHistoryCapacity", c.CommandHistoryCapacity);
            WriteFloat(w, "maxJointSpeedRadPerSec", c.MaxJointSpeedRadPerSec);
            WriteBytes(w, "instantMotorIds", c.InstantMotorIds);
            w.WriteNumber("maxHorizonTicks", c.MaxHorizonTicks);
            w.WriteNumber("ticksPerSecond", c.TicksPerSecond);
            WriteBytes(w, "scaledMotorIds", c.ScaledMotorIds);
            WriteFloats(w, "speedScales", c.SpeedScales);
            WriteFloat(w, "relayMinMoveSeconds", c.RelayMinMoveSeconds);
            WriteFloat(w, "relayDeadbandRad", c.RelayDeadbandRad);
            WriteFloat(w, "relayMoveSpeedRadPerSec", c.RelayMoveSpeedRadPerSec);
            WriteBytes(w, "sensedAgeMotorIds", c.SensedAgeMotorIds);
            WriteFloats(w, "sensedAgeSeconds", c.SensedAgeSeconds);
            w.WriteEndObject();
        }

        private static void WriteCommand(Utf8JsonWriter w, long effective, float speed, JointSample[] targets)
        {
            w.WriteStartObject();
            w.WriteString("op", "command");
            w.WriteNumber("t", effective);
            WriteFloat(w, "speed", speed);
            WriteSamples(w, "targets", targets, targets.Length);
            w.WriteEndObject();
        }

        private static void WriteState(Utf8JsonWriter w, long sensed, JointSample[] joints)
        {
            w.WriteStartObject();
            w.WriteString("op", "state");
            w.WriteNumber("t", sensed);
            WriteSamples(w, "joints", joints, joints.Length);
            w.WriteEndObject();
        }

        /// <summary>A sample is [motorId, float bits, sensed (0/1)].</summary>
        private static void WriteSamples(Utf8JsonWriter w, string name, JointSample[] samples, int count)
        {
            w.WriteStartArray(name);
            for (int i = 0; i < count; i++)
            {
                w.WriteStartArray();
                w.WriteNumberValue(samples[i].MotorId);
                w.WriteNumberValue(BitConverter.SingleToInt32Bits(samples[i].Value));
                w.WriteNumberValue(samples[i].Sensed ? 1 : 0);
                w.WriteEndArray();
            }

            w.WriteEndArray();
        }

        private static void WriteFloat(Utf8JsonWriter w, string name, float value)
        {
            w.WriteStartArray(name);
            w.WriteNumberValue(value);
            w.WriteNumberValue(BitConverter.SingleToInt32Bits(value));
            w.WriteEndArray();
        }

        private static void WriteFloats(Utf8JsonWriter w, string name, float[] values)
        {
            w.WriteStartArray(name);
            foreach (float v in values) w.WriteNumberValue(BitConverter.SingleToInt32Bits(v));
            w.WriteEndArray();
        }

        private static void WriteBytes(Utf8JsonWriter w, string name, byte[] values)
        {
            w.WriteStartArray(name);
            foreach (byte b in values) w.WriteNumberValue(b);
            w.WriteEndArray();
        }

        /// <summary>One compact JSON document and a final LF: these files are regenerated, never hand-edited, so size wins over diffability.</summary>
        private static string Json(Action<Utf8JsonWriter> body)
        {
            using var stream = new MemoryStream();
            using (var w = new Utf8JsonWriter(stream, new JsonWriterOptions { Indented = false }))
            {
                w.WriteStartObject();
                w.WriteNumber("version", 1);
                body(w);
                w.WriteEndObject();
            }

            return Encoding.UTF8.GetString(stream.ToArray()).Replace("\r\n", "\n") + "\n";
        }
    }
}
