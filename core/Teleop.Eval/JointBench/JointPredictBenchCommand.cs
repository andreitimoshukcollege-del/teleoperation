using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Teleop.Core.Contracts;
using Teleop.Core.Registry;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Eval.JointBench
{
    /// <summary>
    /// <c>joint-predict-bench</c>: the benchmark row for the joint-space predictors (docs/adr/0018),
    /// headless and deterministic. It simulates a servo arm and the network between it and the
    /// operator, and scores every registered <see cref="ICommandAwarePredictor"/> against the
    /// simulation's ground truth on the same seeds and the same frozen network profiles. Two
    /// metrics, side by side: <c>joint_prediction_error_deg</c> and <c>joint_correction_deg</c>
    /// (docs/metrics.md §4).
    ///
    /// <para><b>The simulation, and its assumptions, stated so a number is never read without
    /// them.</b> Common to both robots:</para>
    /// <list type="bullet">
    /// <item>Five arm joints and an instant gripper. The gripper is excluded from both metrics.</item>
    /// <item>The operator sends at 48 Hz. Targets follow a seeded random path, with a new waypoint
    /// every 0.5–1.5 s and linear motion between them, within ±1 rad. The gripper steps between
    /// open and closed. No command asks for a speed.</item>
    /// <item>Each direction gets the profile's base delay plus uniform jitter, and independent
    /// Bernoulli loss at the profile's after-delivered rate. Burst structure is ignored, and so is
    /// clock-sync error: the operator knows the robot's clock exactly.</item>
    /// <item>The operator gives every command an effective time of its send time plus the profile's
    /// base delay, the best a symmetric RTT/2 estimate can do.</item>
    /// <item>The operator predicts for every 90 Hz display frame, for that same instant. The error is
    /// the absolute joint difference to the truth then, in degrees, pooled over the arm joints, after
    /// a 1 s warm-up.</item>
    /// <item>The correction is, at each state frame whose sensed stamp is newer than every frame
    /// observed before it, the absolute difference per arm joint between the prediction for the
    /// arrival instant just before and just after the frame is observed, pooled, after the same
    /// warm-up. Both are reported as percentiles (Prediction/CLAUDE.md).</item>
    /// </list>
    /// <para><c>--robot ideal</c> (the default, unchanged since 2026-10-12):</para>
    /// <list type="bullet">
    /// <item>Each arm joint drives toward its latest <i>arrived</i> target at <c>--true-speed</c>
    /// rad/s (default 4.0). <c>servo-replay</c>'s model speed is <c>--model-speed</c> (default 5.0):
    /// deliberately not equal, because the real servo's speed is a guess too.</item>
    /// <item>Arrived commands apply in send order, so a late, older command can overwrite a newer one.</item>
    /// <item>The robot reads every servo 5 ms before each 50 Hz frame and stamps the frame with that
    /// read time, so every frame is fresh and its stamp is exact.</item>
    /// </list>
    /// <para><c>--robot relay</c>: the JetRover's paced relay, as robot/ros servo_controller.py and
    /// robot_controller_node.py implement it, with newest-wins pending targets:</para>
    /// <list type="bullet">
    /// <item>The robot drops a command older than the newest it has taken (RobotHost's staleness
    /// check), in arrival order. Targets become integer pulses: 1000 pulses per 240°, 500 at 0 rad,
    /// truncated as <c>int()</c> does.</item>
    /// <item>Per servo, a command arriving inside the write cooldown becomes the pending target,
    /// newest wins, and is offered to the gate exactly when the cooldown ends. The real relay
    /// flushes pending targets between feedback reads, so up to one read (about 10 ms) later; that
    /// and the executor's command bursts are not modelled.</item>
    /// <item>The gate drops a target within <c>--relay-deadband-pulses</c> (5) of the last one
    /// written. Otherwise it writes a move lasting max(<c>--relay-min-move-ms</c> (50),
    /// |target − last written| / <c>--relay-pps</c> (2200)), which is also the cooldown. A servo's
    /// first write lasts the minimum.</item>
    /// <item>The servo follows each move exactly: linear, from where it is to the target, over the
    /// move's duration. No dynamics, overshoot, load or stall.</item>
    /// <item>Feedback reads motors 1..5 one after another, <c>--read-ms</c> (10) each, back to back,
    /// so a cycle is 50 ms (20 Hz fresh readings), at a seeded phase. Each reading is the servo's
    /// position then, rounded to a whole pulse. The frame's sensed stamp is <c>--stamp-ms</c> (5)
    /// after the last read, so motor 1 is about 45 ms older than its stamp and motor 5 about 5 ms.</item>
    /// <item>The robot streams 50 Hz frames, each carrying the newest feedback and its stamp, so most
    /// fresh readings arrive two or three times.</item>
    /// <item><c>servo-replay</c>'s model speed is <c>--model-speed</c>, default 6.0 here, the
    /// headset's configured value. <c>relay-replay</c> is given the relay's own gate parameters
    /// (speed <c>--model-relay-pps</c>, default the true one) and, unless
    /// <c>--model-sensed-age off</c>, each motor's true reading age.</item>
    /// </list>
    /// </summary>
    public static class JointPredictBenchCommand
    {
        private const long Tps = 1_000_000; // microseconds
        private const int ArmJoints = 5;
        private const byte GripperId = 10;

        // The JetRover's pulse space (Teleop.RobotHost GenericArmPlantConfig.Default).
        private const double PulsesPerRadian = JointPredictorConfig.JetRoverPulsesPerRadian;
        private const int ZeroPulse = 500, MinPulse = 0, MaxPulse = 1000;

        /// <summary>The relay robot's parameters, and what relay-replay is told about it.</summary>
        public sealed class RelayOptions
        {
            public double PulsesPerSecond = 2200;
            public double MinMoveMs = 50;
            public int DeadbandPulses = 5;
            public double ReadMs = 10;
            public double StampMs = 5;
            public double? ModelPulsesPerSecond;
            public bool ModelSensedAge = true;
        }

        /// <summary>One seeded run's pooled samples, in degrees.</summary>
        public sealed class Samples
        {
            public readonly List<double> Errors = new List<double>();
            public readonly List<double> Corrections = new List<double>();
        }

        public static int Run(string[] args)
        {
            int seeds = 5;
            double seconds = 20, trueSpeed = 4.0;
            double? modelSpeed = null;
            string robot = "ideal";
            var relay = new RelayOptions();
            for (int i = 1; i < args.Length; i++)
            {
                string a = args[i];
                string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{a} needs a value");
                double D() => double.Parse(Next(), CultureInfo.InvariantCulture);
                switch (a)
                {
                    case "--seeds": seeds = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--seconds": seconds = D(); break;
                    case "--true-speed": trueSpeed = D(); break;
                    case "--model-speed": modelSpeed = D(); break;
                    case "--robot": robot = Next(); break;
                    case "--relay-pps": relay.PulsesPerSecond = D(); break;
                    case "--relay-min-move-ms": relay.MinMoveMs = D(); break;
                    case "--relay-deadband-pulses": relay.DeadbandPulses = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--read-ms": relay.ReadMs = D(); break;
                    case "--stamp-ms": relay.StampMs = D(); break;
                    case "--model-relay-pps": relay.ModelPulsesPerSecond = D(); break;
                    case "--model-sensed-age":
                        string age = Next();
                        if (age != "on" && age != "off") return Usage();
                        relay.ModelSensedAge = age == "on";
                        break;
                    default:
                        return Usage();
                }
            }

            if (robot != "ideal" && robot != "relay")
            {
                return Usage();
            }

            bool isRelay = robot == "relay";
            double servoModelSpeed = modelSpeed ?? (isRelay ? 6.0 : 5.0);
            if (isRelay)
            {
                Console.WriteLine(
                    $"joint-predict-bench: robot relay, {seeds} seeds x {seconds:0} s, relay {relay.PulsesPerSecond:0} pulses/s ({relay.PulsesPerSecond / PulsesPerRadian:0.##} rad/s), " +
                    $"min move {relay.MinMoveMs:0} ms, deadband {relay.DeadbandPulses} pulses, reads {relay.ReadMs:0} ms each, stamp +{relay.StampMs:0} ms; " +
                    $"servo-replay model speed {servoModelSpeed:0.##} rad/s, relay-replay {relay.ModelPulsesPerSecond ?? relay.PulsesPerSecond:0} pulses/s, sensed age {(relay.ModelSensedAge ? "on" : "off")} (docs/adr/0018)");
            }
            else
            {
                Console.WriteLine($"joint-predict-bench: robot ideal, {seeds} seeds x {seconds:0} s, true speed {trueSpeed:0.##} rad/s, model speed {servoModelSpeed:0.##} rad/s (docs/adr/0018)");
            }

            Console.WriteLine("joint_prediction_error_deg (error) and joint_correction_deg (correction), docs/metrics.md §4");
            Console.WriteLine(
                $"{"profile",-24} {"predictor",-14} {"err p50",8} {"err p95",8} {"err p99",8} {"err max",8} {"cor p50",8} {"cor p95",8} {"cor p99",8} {"cor max",8}");
            foreach (string profileName in NetworkProfileCatalog.NamedProfiles)
            {
                if (!NetworkProfileCatalog.TryResolveParametric(profileName, Tps, out NetworkProfile profile, out _))
                {
                    Console.Error.WriteLine($"joint-predict-bench: cannot resolve profile {profileName}");
                    return 1;
                }

                foreach (string predictorName in Registries.JointPredictors.Keys.OrderBy(k => k, StringComparer.Ordinal))
                {
                    var errors = new List<double>();
                    var corrections = new List<double>();
                    for (int seed = 1; seed <= seeds; seed++)
                    {
                        Samples s = isRelay
                            ? SimulateRelay(profile, predictorName, (ulong)seed, seconds, servoModelSpeed, relay)
                            : SimulateIdeal(profile, predictorName, (ulong)seed, seconds, trueSpeed, servoModelSpeed);
                        errors.AddRange(s.Errors);
                        corrections.AddRange(s.Corrections);
                    }

                    errors.Sort();
                    corrections.Sort();
                    Console.WriteLine(
                        $"{profileName,-24} {predictorName,-14} {Pct(errors, 0.50),8:0.00} {Pct(errors, 0.95),8:0.00} {Pct(errors, 0.99),8:0.00} {Max(errors),8:0.00} " +
                        $"{Pct(corrections, 0.50),8:0.00} {Pct(corrections, 0.95),8:0.00} {Pct(corrections, 0.99),8:0.00} {Max(corrections),8:0.00}");
                }
            }

            return 0;
        }

        private static int Usage()
        {
            Console.Error.WriteLine(
                "usage: joint-predict-bench [--robot ideal|relay] [--seeds N] [--seconds S] [--model-speed rad/s]\n" +
                "  ideal: [--true-speed rad/s]\n" +
                "  relay: [--relay-pps P] [--relay-min-move-ms MS] [--relay-deadband-pulses N] [--read-ms MS] [--stamp-ms MS]\n" +
                "         [--model-relay-pps P] [--model-sensed-age on|off]");
            return 64;
        }

        /// <summary>One seeded run of the ideal robot; returns every pooled per-joint error in degrees.</summary>
        public static List<double> Simulate(NetworkProfile profile, string predictorName, ulong seed, double seconds, double trueSpeed, double modelSpeed) =>
            SimulateIdeal(profile, predictorName, seed, seconds, trueSpeed, modelSpeed).Errors;

        /// <summary>One seeded run of the ideal robot (see the class doc).</summary>
        public static Samples SimulateIdeal(NetworkProfile profile, string predictorName, ulong seed, double seconds, double trueSpeed, double modelSpeed)
        {
            var rng = new SeededRng(seed);
            var config = new JointPredictorConfig(
                maxJoints: ArmJoints + 1, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: (float)modelSpeed,
                instantMotorIds: new[] { GripperId }, maxHorizonTicks: Tps, ticksPerSecond: Tps);
            ICommandAwarePredictor predictor = Registries.JointPredictors[predictorName](config);

            long end = (long)(seconds * Tps), step = Tps / 1000;
            long commandInterval = Tps / 48, stateInterval = Tps / 50, displayInterval = Tps / 90, warmup = Tps;

            // The operator's intent: waypoints per joint.
            var from = new double[ArmJoints];
            var to = new double[ArmJoints];
            var legStart = new long[ArmJoints];
            var legEnd = new long[ArmJoints];
            for (int j = 0; j < ArmJoints; j++) NewLeg(ref rng, j, 0, from, to, legStart, legEnd);
            double gripper = 0;

            var truth = new double[ArmJoints + 1];
            var robotTarget = new double[ArmJoints + 1];
            var uplink = new List<(long arrive, double[] targets)>();
            var downlink = new List<(long arrive, long sensed, double[] joints)>();
            var samples = new Samples();
            var scorer = new CorrectionScorer(ArmJoints + 1, warmup);
            var command = new JointSample[ArmJoints + 1];
            var state = new JointSample[ArmJoints + 1];
            var predicted = new JointSample[ArmJoints + 1];

            long nextCommand = 0, nextState = Tps / 200, nextDisplay = warmup;
            for (long t = 0; t <= end; t += step)
            {
                // Operator: send a command.
                if (t >= nextCommand)
                {
                    nextCommand += commandInterval;
                    double[] targets = NextTargets(ref rng, t, from, to, legStart, legEnd, ref gripper);
                    for (int j = 0; j <= ArmJoints; j++) command[j] = new JointSample(Id(j), (float)targets[j], false);
                    predictor.ObserveCommand(t + profile.BaseDelayTicks, command, 0f);
                    if (!Lost(ref rng, profile)) uplink.Add((t + Delay(ref rng, profile), targets));
                }

                // Robot: take arrived commands, move the servos.
                for (int k = uplink.Count - 1; k >= 0; k--)
                {
                    if (uplink[k].arrive <= t)
                    {
                        // Apply every arrived command in send order, so the newest wins.
                        for (int m = 0; m <= k; m++)
                        {
                            if (uplink[m].arrive <= t) Array.Copy(uplink[m].targets, robotTarget, ArmJoints + 1);
                        }

                        uplink.RemoveAll(c => c.arrive <= t);
                        break;
                    }
                }

                double move = trueSpeed * step / Tps;
                for (int j = 0; j < ArmJoints; j++)
                {
                    double d = robotTarget[j] - truth[j];
                    truth[j] = Math.Abs(d) <= move ? robotTarget[j] : truth[j] + Math.Sign(d) * move;
                }

                truth[ArmJoints] = robotTarget[ArmJoints];

                // Robot: a state frame, sensed 5 ms earlier.
                if (t >= nextState)
                {
                    nextState += stateInterval;
                    if (!Lost(ref rng, profile)) downlink.Add((t + Delay(ref rng, profile), t - Tps / 200, (double[])truth.Clone()));
                }

                // Operator: arrived state frames.
                for (int k = 0; k < downlink.Count; k++)
                {
                    if (downlink[k].arrive <= t)
                    {
                        for (int j = 0; j <= ArmJoints; j++) state[j] = new JointSample(Id(j), (float)downlink[k].joints[j], j < ArmJoints);
                        scorer.Observe(predictor, t, downlink[k].sensed, state, samples.Corrections);
                    }
                }

                downlink.RemoveAll(f => f.arrive <= t);

                // Operator: a display frame.
                if (t >= nextDisplay && predictor.TryPredict(t, predicted, out int count))
                {
                    nextDisplay += displayInterval;
                    for (int j = 0; j < Math.Min(count, ArmJoints); j++)
                    {
                        samples.Errors.Add(Math.Abs(predicted[j].Value - truth[j]) * 180.0 / Math.PI);
                    }
                }
            }

            return samples;
        }

        /// <summary>One seeded run of the paced-relay robot (see the class doc).</summary>
        public static Samples SimulateRelay(NetworkProfile profile, string predictorName, ulong seed, double seconds, double modelSpeed, RelayOptions relay)
        {
            var rng = new SeededRng(seed);
            long readTicks = (long)(relay.ReadMs / 1000.0 * Tps);
            long stampTicks = (long)(relay.StampMs / 1000.0 * Tps);
            long minMoveTicks = (long)(relay.MinMoveMs / 1000.0 * Tps);

            // What relay-replay is told: the gate, and each motor's reading age (motor k is read
            // (5 - k) reads before the last one, which is stampTicks before the stamp).
            var ageIds = new byte[relay.ModelSensedAge ? ArmJoints : 0];
            var ageSeconds = new float[ageIds.Length];
            for (int j = 0; j < ageIds.Length; j++)
            {
                ageIds[j] = Id(j);
                ageSeconds[j] = (float)((stampTicks + (ArmJoints - 1 - j) * readTicks) / (double)Tps);
            }

            var config = new JointPredictorConfig(
                maxJoints: ArmJoints + 1, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: (float)modelSpeed,
                instantMotorIds: new[] { GripperId }, maxHorizonTicks: Tps, ticksPerSecond: Tps,
                relayMinMoveSeconds: (float)(relay.MinMoveMs / 1000.0),
                relayDeadbandRad: (float)(relay.DeadbandPulses / PulsesPerRadian),
                relayMoveSpeedRadPerSec: (float)((relay.ModelPulsesPerSecond ?? relay.PulsesPerSecond) / PulsesPerRadian),
                sensedAgeMotorIds: ageIds, sensedAgeSeconds: ageSeconds);
            ICommandAwarePredictor predictor = Registries.JointPredictors[predictorName](config);

            long end = (long)(seconds * Tps), step = Tps / 1000;
            long commandInterval = Tps / 48, frameInterval = Tps / 50, displayInterval = Tps / 90, warmup = Tps;

            var from = new double[ArmJoints];
            var to = new double[ArmJoints];
            var legStart = new long[ArmJoints];
            var legEnd = new long[ArmJoints];
            for (int j = 0; j < ArmJoints; j++) NewLeg(ref rng, j, 0, from, to, legStart, legEnd);
            double gripper = 0;

            var servos = new RelayServo[ArmJoints];
            for (int j = 0; j < ArmJoints; j++) servos[j] = new RelayServo(ZeroPulse);
            double gripperTarget = 0;
            long newestAcceptedSend = long.MinValue;

            // Feedback: back-to-back cycles of ArmJoints sequential reads, at a seeded phase.
            long cycleTicks = ArmJoints * readTicks;
            long cycleStart = (long)(rng.NextDouble() * cycleTicks);
            int nextRead = 0;
            var reading = new double[ArmJoints];
            double[]? published = null;          // the newest feedback RobotHost has, in radians
            long publishedStamp = 0;
            double[]? inFlight = null;           // a finished cycle not yet stamped by RobotHost
            long inFlightStamp = 0;

            var uplink = new List<(long arrive, long send, double[] targets)>();
            var downlink = new List<(long arrive, long sensed, double[] joints)>();
            var samples = new Samples();
            var scorer = new CorrectionScorer(ArmJoints + 1, warmup);
            var command = new JointSample[ArmJoints + 1];
            var state = new JointSample[ArmJoints + 1];
            var predicted = new JointSample[ArmJoints + 1];

            long nextCommand = 0, nextFrame = Tps / 200, nextDisplay = warmup;
            for (long t = 0; t <= end; t += step)
            {
                // Operator: send a command.
                if (t >= nextCommand)
                {
                    nextCommand += commandInterval;
                    double[] targets = NextTargets(ref rng, t, from, to, legStart, legEnd, ref gripper);
                    for (int j = 0; j <= ArmJoints; j++) command[j] = new JointSample(Id(j), (float)targets[j], false);
                    predictor.ObserveCommand(t + profile.BaseDelayTicks, command, 0f);
                    if (!Lost(ref rng, profile)) uplink.Add((t + Delay(ref rng, profile), t, targets));
                }

                // Robot: commands that have arrived, in arrival order; older than the newest taken are dropped.
                if (uplink.Count > 0)
                {
                    var arrived = uplink.Where(c => c.arrive <= t).OrderBy(c => c.arrive).ThenBy(c => c.send).ToList();
                    foreach (var c in arrived)
                    {
                        if (c.send <= newestAcceptedSend) continue;
                        newestAcceptedSend = c.send;
                        for (int j = 0; j < ArmJoints; j++)
                        {
                            servos[j].Command(c.arrive, ToPulse(c.targets[j]), relay, minMoveTicks);
                        }

                        gripperTarget = c.targets[ArmJoints];
                    }

                    uplink.RemoveAll(c => c.arrive <= t);
                }

                // Robot: pending targets whose cooldown has ended are written then.
                for (int j = 0; j < ArmJoints; j++) servos[j].Flush(t, relay, minMoveTicks);

                // Robot: sequential feedback reads; a cycle's last read finishes it.
                while (cycleStart + (nextRead + 1) * readTicks <= t)
                {
                    long readAt = cycleStart + (nextRead + 1) * readTicks;
                    reading[nextRead] = (Math.Round(servos[nextRead].PositionAt(readAt)) - ZeroPulse) / PulsesPerRadian;
                    nextRead++;
                    if (nextRead == ArmJoints)
                    {
                        inFlight = (double[])reading.Clone();
                        inFlightStamp = readAt + stampTicks;
                        cycleStart = readAt;
                        nextRead = 0;
                    }
                }

                if (inFlight != null && inFlightStamp <= t)
                {
                    published = inFlight;
                    publishedStamp = inFlightStamp;
                    inFlight = null;
                }

                // Robot: a 50 Hz frame carrying the newest feedback, fresh or not.
                if (t >= nextFrame)
                {
                    nextFrame += frameInterval;
                    if (published != null && !Lost(ref rng, profile))
                    {
                        var joints = new double[ArmJoints + 1];
                        Array.Copy(published, joints, ArmJoints);
                        joints[ArmJoints] = gripperTarget;
                        downlink.Add((t + Delay(ref rng, profile), publishedStamp, joints));
                    }
                }

                // Operator: arrived state frames, in arrival order.
                if (downlink.Count > 0)
                {
                    foreach (var f in downlink.Where(f => f.arrive <= t).OrderBy(f => f.arrive).ToList())
                    {
                        for (int j = 0; j <= ArmJoints; j++) state[j] = new JointSample(Id(j), (float)f.joints[j], j < ArmJoints);
                        scorer.Observe(predictor, t, f.sensed, state, samples.Corrections);
                    }

                    downlink.RemoveAll(f => f.arrive <= t);
                }

                // Operator: a display frame.
                if (t >= nextDisplay && predictor.TryPredict(t, predicted, out int count))
                {
                    nextDisplay += displayInterval;
                    for (int j = 0; j < Math.Min(count, ArmJoints); j++)
                    {
                        double truth = (servos[j].PositionAt(t) - ZeroPulse) / PulsesPerRadian;
                        samples.Errors.Add(Math.Abs(predicted[j].Value - truth) * 180.0 / Math.PI);
                    }
                }
            }

            return samples;
        }

        /// <summary>One servo behind the paced relay: its gate, and the linear move it is in.</summary>
        private sealed class RelayServo
        {
            private int _lastSent;
            private bool _hasLastSent;
            private long _cooldownEnd = long.MinValue;
            private int _pending;
            private bool _hasPending;
            private long _moveStart, _moveEnd;
            private double _moveFrom, _moveTo;

            // The move before the current one: a read can fall in the same 1 ms step as, but
            // before, a write, and must see the servo where it was then.
            private long _prevStart, _prevEnd;
            private double _prevFrom, _prevTo;

            public RelayServo(double restPulse)
            {
                _moveFrom = _moveTo = _prevFrom = _prevTo = restPulse;
            }

            public double PositionAt(long t) =>
                t < _moveStart
                    ? Lerp(t, _prevStart, _prevEnd, _prevFrom, _prevTo)
                    : Lerp(t, _moveStart, _moveEnd, _moveFrom, _moveTo);

            private static double Lerp(long t, long start, long end, double from, double to)
            {
                if (t >= end) return to;
                if (t <= start) return from;
                return from + (to - from) * ((double)(t - start) / (end - start));
            }

            /// <summary>setPos: write now if the cooldown is over, otherwise keep as the pending target.</summary>
            public void Command(long at, int target, RelayOptions relay, long minMoveTicks)
            {
                Flush(at, relay, minMoveTicks);
                if (at >= _cooldownEnd)
                {
                    _hasPending = false;
                    Write(at, target, relay, minMoveTicks);
                }
                else
                {
                    _pending = target;
                    _hasPending = true;
                }
            }

            /// <summary>flushPending, at the instant the cooldown ended, if that is by <paramref name="now"/>.</summary>
            public void Flush(long now, RelayOptions relay, long minMoveTicks)
            {
                if (_hasPending && _cooldownEnd <= now)
                {
                    _hasPending = false;
                    Write(_cooldownEnd, _pending, relay, minMoveTicks);
                }
            }

            private void Write(long at, int target, RelayOptions relay, long minMoveTicks)
            {
                if (_hasLastSent && Math.Abs(target - _lastSent) < relay.DeadbandPulses)
                {
                    return;
                }

                int distance = _hasLastSent ? Math.Abs(target - _lastSent) : 0;
                long duration = Math.Max(minMoveTicks, (long)(distance / relay.PulsesPerSecond * Tps));
                double here = PositionAt(at);
                _prevStart = _moveStart;
                _prevEnd = _moveEnd;
                _prevFrom = _moveFrom;
                _prevTo = _moveTo;
                _moveFrom = here;
                _moveTo = target;
                _moveStart = at;
                _moveEnd = at + duration;
                _lastSent = target;
                _hasLastSent = true;
                _cooldownEnd = _moveEnd;
            }
        }

        /// <summary>
        /// <c>joint_correction_deg</c>: at each frame newer than every one observed before, the jump
        /// in the prediction for that instant across the observation, per arm joint.
        /// </summary>
        private sealed class CorrectionScorer
        {
            private readonly JointSample[] _before;
            private readonly JointSample[] _after;
            private readonly long _warmup;
            private long _newestSensed = long.MinValue;

            public CorrectionScorer(int joints, long warmup)
            {
                _before = new JointSample[joints];
                _after = new JointSample[joints];
                _warmup = warmup;
            }

            public void Observe(ICommandAwarePredictor predictor, long now, long sensed, JointSample[] state, List<double> corrections)
            {
                bool fresh = sensed > _newestSensed;
                if (fresh) _newestSensed = sensed;
                int beforeCount = 0;
                bool score = fresh && now >= _warmup && predictor.TryPredict(now, _before, out beforeCount);
                predictor.ObserveState(sensed, state);
                if (!score || !predictor.TryPredict(now, _after, out int afterCount))
                {
                    return;
                }

                for (int j = 0; j < afterCount; j++)
                {
                    if (_after[j].MotorId == GripperId) continue;
                    for (int i = 0; i < beforeCount; i++)
                    {
                        if (_before[i].MotorId == _after[j].MotorId)
                        {
                            corrections.Add(Math.Abs(_after[j].Value - _before[i].Value) * 180.0 / Math.PI);
                            break;
                        }
                    }
                }
            }
        }

        private static double[] NextTargets(ref SeededRng rng, long t, double[] from, double[] to, long[] legStart, long[] legEnd, ref double gripper)
        {
            var targets = new double[ArmJoints + 1];
            for (int j = 0; j < ArmJoints; j++)
            {
                if (t >= legEnd[j]) NewLeg(ref rng, j, t, from, to, legStart, legEnd);
                double u = (double)(t - legStart[j]) / (legEnd[j] - legStart[j]);
                targets[j] = from[j] + (to[j] - from[j]) * u;
            }

            if (rng.NextDouble() < 0.01) gripper = gripper > 0.5 ? 0 : 1;
            targets[ArmJoints] = gripper;
            return targets;
        }

        private static int ToPulse(double radians)
        {
            int pulse = (int)(ZeroPulse + radians * PulsesPerRadian);
            return pulse < MinPulse ? MinPulse : pulse > MaxPulse ? MaxPulse : pulse;
        }

        private static byte Id(int j) => j < ArmJoints ? (byte)(j + 1) : GripperId;

        private static void NewLeg(ref SeededRng rng, int j, long t, double[] from, double[] to, long[] start, long[] end)
        {
            from[j] = to[j];
            to[j] = (rng.NextDouble() * 2 - 1);
            start[j] = t;
            end[j] = t + (long)((0.5 + rng.NextDouble()) * Tps);
        }

        private static long Delay(ref SeededRng rng, NetworkProfile p) =>
            p.BaseDelayTicks + (long)((rng.NextDouble() * 2 - 1) * p.JitterTicks);

        private static bool Lost(ref SeededRng rng, NetworkProfile p) => rng.NextDouble() < p.LossProbabilityAfterDelivered;

        private static double Pct(List<double> sorted, double q) =>
            sorted.Count == 0 ? double.NaN : sorted[Math.Min(sorted.Count - 1, (int)(q * (sorted.Count - 1)))];

        private static double Max(List<double> sorted) => sorted.Count == 0 ? double.NaN : sorted[sorted.Count - 1];
    }
}
