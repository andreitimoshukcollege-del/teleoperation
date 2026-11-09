using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.Json;
using Teleop.Core.Contracts;
using Teleop.Core.Forecast;
using Teleop.Core.Inference;
using Teleop.Core.Metrics;
using Teleop.Core.Prediction;
using Teleop.Core.Registry;
using Teleop.Core.Time;
using Teleop.Core.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointBench
{
    /// <summary>
    /// <c>headset-replay</c> (docs/adr/0035 N3): a real JetRover headset session, replayed offline through the headset's own
    /// joint predictors, with and without the network forecast, so the two are scored on exactly the same stream.
    ///
    /// <para><b>What the headset saw</b> is rebuilt from two recordings of the same run: RobotHost's <c>robothost.jlog</c>
    /// (<c>FR</c>, every joint-state frame sent, with its joints and the keepalive echo it carried; <c>JC</c>, every command
    /// that arrived, with the headset's send stamp, trainer/contract/SESSION_FORMAT.md) and the headset's <c>.ltrace</c>
    /// (when each frame arrived on the headset's clock, by sequence; trainer/contract/LINK_TRACE_FORMAT.md). Frames are
    /// observed at their arrival, as <c>JetRoverJointStateLink</c> does with no added delay (it releases them at the next
    /// frame update, up to one display frame later; not modelled). ClockSync runs with the headset's settings, the
    /// forecaster is fed and issued every 100 ms as the link does, and each command is observed at its send stamp.</para>
    ///
    /// <para><b>Scored</b> by <c>LiveJointScorer</c> over the whole session (docs/metrics.md section 4's live variants:
    /// error at each fresh reading, correction at each, display jitter at <c>--display-hz</c>), arm joints only. Two banks:
    /// the headset's default presets with the forecast, and <c>learned</c> alone without it.</para>
    ///
    /// <para><b>Not modelled:</b> commands lost on the uplink (only arrived ones are in <c>JC</c>; the headset also observed
    /// the lost ones), the display's real frame times, and the frame-update release. One session is an anecdote, not a
    /// result: it is printed, not written to results/.</para>
    /// </summary>
    public static class HeadsetReplayCommand
    {
        private const int MaxJoints = JointStateCodec.MaxJointsPerMessage;
        private static readonly byte[] GripperIds = { 10 };
        private static readonly byte[] AgeIds = { 1, 2, 3, 4, 5 };

        private sealed class Frame
        {
            public long RobotSend, Sensed, EchoOperator, EchoRobotRecv;
            public JointSample[] Joints = Array.Empty<JointSample>();
        }

        public static int Run(string[] args)
        {
            string? session = null, trace = null, model = "testdata/learned/finetune-pilot2-early7.weights", forecast = "persistence";
            double displayHz = 72.0;
            for (int i = 1; i < args.Length; i++)
            {
                switch (args[i])
                {
                    case "--session": session = args[++i]; break;
                    case "--trace": trace = args[++i]; break;
                    case "--model": model = args[++i]; break;
                    case "--forecast": forecast = args[++i]; break;
                    case "--display-hz": displayHz = double.Parse(args[++i], CultureInfo.InvariantCulture); break;
                    default:
                        Console.Error.WriteLine($"headset-replay: unknown argument {args[i]}");
                        return 2;
                }
            }

            if (session == null || trace == null)
            {
                Console.Error.WriteLine("usage: headset-replay --session <dir with robothost.jlog> --trace <file.ltrace> [--forecast persistence|rtt-half] [--model <.weights>] [--display-hz 72]");
                return 2;
            }

            if (!Registries.NetworkForecasters.TryGetValue(forecast!, out Func<long, INetworkForecaster>? makeForecaster))
            {
                Console.Error.WriteLine($"headset-replay: no forecast '{forecast}' ({string.Join(", ", Registries.NetworkForecasters.Keys)})");
                return 2;
            }

            // RobotHost's frames and commands.
            var frames = new Dictionary<uint, Frame>();
            var commands = new List<(long Send, JointSample[] Targets, float Speed)>();
            var seenCommands = new HashSet<long>();
            string jlog = Path.Combine(session, "robothost.jlog");
            foreach (string line in File.ReadLines(jlog))
            {
                string[] f = line.Split('|');
                if (f[0] == "FR" && f.Length >= 8)
                {
                    frames[uint.Parse(f[2], CultureInfo.InvariantCulture)] = new Frame
                    {
                        RobotSend = long.Parse(f[3], CultureInfo.InvariantCulture), Sensed = long.Parse(f[4], CultureInfo.InvariantCulture),
                        EchoOperator = long.Parse(f[5], CultureInfo.InvariantCulture), EchoRobotRecv = long.Parse(f[6], CultureInfo.InvariantCulture),
                        Joints = f[7].Split(';', StringSplitOptions.RemoveEmptyEntries).Select(j =>
                        {
                            string[] p = j.Split(':');
                            return new JointSample(byte.Parse(p[0], CultureInfo.InvariantCulture), float.Parse(p[2], CultureInfo.InvariantCulture), p[1] == "1");
                        }).ToArray(),
                    };
                }
                else if (f[0] == "JC" && f.Length >= 7 && seenCommands.Add(long.Parse(f[3], CultureInfo.InvariantCulture)))
                {
                    float speed = 0f;
                    JointSample[] targets = f[6].Split(';', StringSplitOptions.RemoveEmptyEntries).Select(j =>
                    {
                        string[] p = j.Split(':');
                        float s = float.Parse(p[2], CultureInfo.InvariantCulture);
                        if (s > speed) speed = s;
                        return new JointSample(byte.Parse(p[0], CultureInfo.InvariantCulture), float.Parse(p[1], CultureInfo.InvariantCulture), false);
                    }).ToArray();
                    commands.Add((long.Parse(f[4], CultureInfo.InvariantCulture), targets, speed));
                }
            }

            commands.Sort((a, b) => a.Send.CompareTo(b.Send));

            // The headset's arrivals.
            byte[] blob = File.ReadAllBytes(trace);
            int newline = Array.IndexOf(blob, (byte)'\n');
            using JsonDocument header = JsonDocument.Parse(blob.AsMemory(0, newline));
            long tps = header.RootElement.GetProperty("headsetTps").GetInt64();
            var arrivals = new List<(long Arrival, uint Seq)>();
            for (int at = newline + 1; at + LinkTraceCodec.RecordBytes <= blob.Length; at += LinkTraceCodec.RecordBytes)
            {
                if (!LinkTraceCodec.TryDecode(blob.AsSpan(at, LinkTraceCodec.RecordBytes), out LinkTraceKind kind, out uint seq, out _, out _, out long c, out _, out _)) continue;
                if (kind == LinkTraceKind.Frame && frames.ContainsKey(seq)) arrivals.Add((c, seq));
            }

            if (arrivals.Count < 100)
            {
                Console.Error.WriteLine($"headset-replay: only {arrivals.Count} of the trace's frames are in {jlog}: not the same run?");
                return 1;
            }

            long robotTps = long.Parse(File.ReadLines(jlog).First().Split('|')[3], CultureInfo.InvariantCulture);
            if (!MlpWeights.TryParse(File.ReadAllBytes(model!), out MlpWeights? weights, out string error))
            {
                Console.Error.WriteLine($"headset-replay: {model}: {error}");
                return 1;
            }

            // Two banks: the headset's presets with the forecast; learned alone without it.
            JointPredictorBank withForecast = JointPredictorBank.Create(Presets(tps, new MlpBackend(weights!), all: true), 0);
            JointPredictorBank without = JointPredictorBank.Create(Presets(tps, new MlpBackend(weights!), all: false), 0);
            long window = long.MaxValue / 4;
            var scoreWith = new LiveJointScorer(withForecast, window, MaxJoints, GripperIds, 0.4513f, readingCapacity: 16384, tickCapacity: 16384);
            var scoreWithout = new LiveJointScorer(without, window, MaxJoints, GripperIds, 0.4513f, readingCapacity: 16384, tickCapacity: 16384);
            var sync = new ClockSync(new ClockSyncConfig(historyCapacity: 32, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 2 * tps,
                outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 3));
            INetworkForecaster forecaster = makeForecaster(tps);
            var forecastNow = new NetworkForecast();
            var effective = new ForecastEffectiveTimes(tps);
            bool hasForecast = false;
            long lastIssue = long.MinValue, interval = (long)(0.1 * tps), tick = (long)(tps / displayHz);
            long lastEcho = long.MinValue, seqUnwrapped = 0;
            uint lastSeq = 0;
            bool hasSeq = false;
            int validUp = 0, validDown = 0, issues = 0;
            double forecastUpSum = 0, rttHalfSum = 0;
            int upCompared = 0;

            long t = arrivals[0].Arrival;
            long end = arrivals[^1].Arrival;
            int fi = 0, ci = 0;
            while (t <= end)
            {
                // Everything up to this display tick, in time order: frames by arrival, commands by send.
                while (true)
                {
                    long nextFrame = fi < arrivals.Count ? arrivals[fi].Arrival : long.MaxValue;
                    long nextCommand = ci < commands.Count ? commands[ci].Send : long.MaxValue;
                    if (Math.Min(nextFrame, nextCommand) > t) break;
                    if (nextFrame <= nextCommand)
                    {
                        (long arrival, uint seq) = arrivals[fi++];
                        Frame f = frames[seq];
                        if (f.EchoOperator != lastEcho)
                        {
                            lastEcho = f.EchoOperator;
                            long rtt = ClockSync.RoundTripTicks(f.EchoOperator, tps, f.EchoRobotRecv, f.RobotSend, robotTps, arrival);
                            sync.AddRoundTrip(f.EchoOperator, tps, f.EchoRobotRecv, f.RobotSend, robotTps, arrival);
                            ClockSyncDiagnostics d = sync.Diagnostics;
                            long down = d.IsSynced ? arrival - sync.ToOperatorTicks(f.RobotSend, robotTps, tps) : -1;
                            forecaster.ObserveRoundTrip(arrival, rtt, down, d.OffsetUncertaintyTicks, d.MinRttTicks, d.IsSynced);
                        }

                        bool synced = sync.Diagnostics.IsSynced;
                        if (hasSeq) seqUnwrapped += unchecked((int)(seq - lastSeq));
                        else seqUnwrapped = seq;
                        hasSeq = true;
                        lastSeq = seq;
                        forecaster.ObserveFrame(arrival, synced ? sync.ToOperatorTicks(f.RobotSend, robotTps, tps) : -1, seqUnwrapped);
                        if (synced)
                        {
                            long sensedOp = sync.ToOperatorTicks(f.Sensed != 0 ? f.Sensed : f.RobotSend, robotTps, tps);
                            scoreWith.ObserveState(sensedOp, arrival, f.Joints);
                            scoreWithout.ObserveState(sensedOp, arrival, f.Joints);
                        }
                    }
                    else
                    {
                        var cmd = commands[ci++];
                        ClockSyncDiagnostics d = sync.Diagnostics;
                        long eff = cmd.Send + (d.IsSynced ? d.OffsetUncertaintyTicks : 0);
                        long effF = effective.Next(cmd.Send, eff, hasForecast ? forecastNow : null);
                        withForecast.ObserveCommand(eff, effF, cmd.Targets, cmd.Speed);
                        without.ObserveCommand(eff, cmd.Targets, cmd.Speed);
                        if (hasForecast && forecastNow.ValidQ[NetworkForecast.Up] && d.IsSynced)
                        {
                            forecastUpSum += forecastNow.QMs[NetworkForecast.QIndex(NetworkForecast.Up, NetworkForecast.H0, NetworkForecast.P50)];
                            rttHalfSum += d.OffsetUncertaintyTicks * 1000.0 / tps;
                            upCompared++;
                        }
                    }
                }

                if (lastIssue == long.MinValue || t - lastIssue >= interval)
                {
                    forecaster.Issue(t, forecastNow);
                    withForecast.ObserveForecast(forecastNow);
                    hasForecast = true;
                    lastIssue = t;
                    issues++;
                    if (forecastNow.ValidQ[NetworkForecast.Up]) validUp++;
                    if (forecastNow.ValidQ[NetworkForecast.Down]) validDown++;
                }

                scoreWith.OnDisplayTick(t, t);
                scoreWithout.OnDisplayTick(t, t);
                t += tick;
            }

            Console.WriteLine($"headset-replay: {Path.GetFileName(Path.GetFullPath(session))} with {Path.GetFileName(trace)}: " +
                              $"{arrivals.Count} frames, {commands.Count} commands, {(end - arrivals[0].Arrival) / (double)tps:F1} s; " +
                              $"forecast '{forecaster.Level}': {issues} issued, uplink valid {validUp}, downlink valid {validDown}" +
                              (upCompared > 0 ? $"; uplink median {forecastUpSum / upCompared:F1} ms against RTT/2 {rttHalfSum / upCompared:F1} ms (mean at the commands)" : ""));
            Console.WriteLine("Arm joints, degrees (LiveJointScorer over the whole session; nearest-rank percentiles):");
            Console.WriteLine($"{"predictor",-28} {"readings",8} {"err p50",8} {"err p95",8} {"corr p95",9} {"jitter p95",11}");
            void Row(string label, LiveJointScorer s, int index)
            {
                if (!s.TryGetStats(index, out LiveJointStats st) || !st.HasData) return;
                Console.WriteLine($"{label,-28} {st.Readings,8} {st.ErrorP50Deg,8:F2} {st.ErrorP95Deg,8:F2} {st.CorrectionP95Deg,9:F2} {st.JitterP95Deg,11:F3}");
            }

            Row($"learned + {forecaster.Level}", scoreWith, 0);
            Row("learned, no forecast", scoreWithout, 0);
            for (int i = 1; i < withForecast.Count; i++) Row(withForecast.LabelAt(i), scoreWith, i);
            return 0;
        }

        /// <summary>The headset's default presets (JetRoverArmConfig's JointPredictorPresetData.Defaults), as its link builds them.</summary>
        private static JointPredictorPreset[] Presets(long tps, IInferenceBackend model, bool all)
        {
            JointPredictorConfig Config(float[] ages, float speed = 6f, float servoMax = 5f, float lagMs = 50f, float pipelineMs = 16.8f) =>
                new JointPredictorConfig(maxJoints: MaxJoints, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: speed, instantMotorIds: new byte[0],
                    maxHorizonTicks: (long)(0.5 * tps), ticksPerSecond: tps, scaledMotorIds: GripperIds, speedScales: new[] { 0.4513f },
                    sensedAgeMotorIds: ages.Length > 0 ? AgeIds : new byte[0], sensedAgeSeconds: ages.Select(a => a / 1000f).ToArray(),
                    servoMaxSpeedRadPerSec: servoMax, servoLagSeconds: lagMs / 1000f, servoPipelineDelaySeconds: pipelineMs / 1000f,
                    servoFirstMoveFromMeasured: true);
            float[] ages1025 = { 30.2f, 25.3f, 22.6f, 20.1f, 17.1f };
            float[] ages1026 = { 29.5f, 25.1f, 22.2f, 19.7f, 17.7f };
            float[] agesLearned = { 29.49f, 25.09f, 22.23f, 19.73f, 17.66f };
            var learned = new JointPredictorPreset("learned", "learned", Config(agesLearned).WithModel(model));
            if (!all) return new[] { learned };
            return new[]
            {
                learned,
                new JointPredictorPreset("hold-last", "hold-last", Config(new float[0])),
                new JointPredictorPreset("relay-replay", "relay-replay", Config(ages1025)),
                new JointPredictorPreset("servo 10-25", "relay-replay-servo", Config(ages1025, servoMax: 5f, lagMs: 50f, pipelineMs: 16.8f)),
                new JointPredictorPreset("servo 10-26", "relay-replay-servo", Config(ages1026, servoMax: 3.25f, lagMs: 40f, pipelineMs: 13.8f)),
            };
        }
    }
}
