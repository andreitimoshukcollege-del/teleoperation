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
    /// simulation's ground truth on the same seeds and the same frozen network profiles.
    ///
    /// <para><b>The simulation, and its assumptions, stated so a number is never read without
    /// them:</b></para>
    /// <list type="bullet">
    /// <item>Five arm joints and an instant gripper. Each arm joint drives toward its latest
    /// <i>arrived</i> target at <c>--true-speed</c> rad/s (default 4.0). The predictor's model
    /// speed is <c>--model-speed</c> (default 5.0): deliberately not equal, because the real servo's
    /// speed is a guess too.</item>
    /// <item>The operator sends at 48 Hz. Targets follow a seeded random path, with a new waypoint
    /// every 0.5–1.5 s and linear motion between them, within ±1 rad. The gripper steps between
    /// open and closed.</item>
    /// <item>Each direction gets the profile's base delay plus uniform jitter, and independent
    /// Bernoulli loss at the profile's after-delivered rate. Reordering and burst structure are
    /// ignored, and so is clock-sync error: the operator knows the robot's clock exactly.</item>
    /// <item>The robot reads the servos 5 ms before each 50 Hz frame. The operator estimates the
    /// uplink delay as the profile's base delay, the best a symmetric RTT/2 estimate can do.</item>
    /// <item>The operator predicts for every 90 Hz display frame. The error is the absolute joint
    /// difference to the truth at that instant, in degrees, pooled over the arm joints, after a 1 s
    /// warm-up. It is reported as percentiles (Prediction/CLAUDE.md).</item>
    /// </list>
    /// </summary>
    public static class JointPredictBenchCommand
    {
        private const long Tps = 1_000_000; // microseconds
        private const int ArmJoints = 5;
        private const byte GripperId = 10;

        public static int Run(string[] args)
        {
            int seeds = 5;
            double seconds = 20, trueSpeed = 4.0, modelSpeed = 5.0;
            for (int i = 1; i < args.Length; i++)
            {
                string a = args[i];
                string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{a} needs a value");
                switch (a)
                {
                    case "--seeds": seeds = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--seconds": seconds = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--true-speed": trueSpeed = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--model-speed": modelSpeed = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                    default:
                        Console.Error.WriteLine("usage: joint-predict-bench [--seeds N] [--seconds S] [--true-speed rad/s] [--model-speed rad/s]");
                        return 64;
                }
            }

            Console.WriteLine($"joint-predict-bench: {seeds} seeds x {seconds:0} s, true speed {trueSpeed:0.##} rad/s, model speed {modelSpeed:0.##} rad/s (docs/adr/0018)");
            Console.WriteLine($"{"profile",-24} {"predictor",-14} {"p50 deg",9} {"p95 deg",9} {"p99 deg",9} {"max deg",9}");
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
                    for (int seed = 1; seed <= seeds; seed++)
                    {
                        errors.AddRange(Simulate(profile, predictorName, (ulong)seed, seconds, trueSpeed, modelSpeed));
                    }

                    errors.Sort();
                    Console.WriteLine(
                        $"{profileName,-24} {predictorName,-14} {Pct(errors, 0.50),9:0.00} {Pct(errors, 0.95),9:0.00} {Pct(errors, 0.99),9:0.00} {errors[errors.Count - 1],9:0.00}");
                }
            }

            return 0;
        }

        /// <summary>One seeded run; returns every pooled per-joint error in degrees.</summary>
        public static List<double> Simulate(NetworkProfile profile, string predictorName, ulong seed, double seconds, double trueSpeed, double modelSpeed)
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
            var errors = new List<double>();
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
                    var targets = new double[ArmJoints + 1];
                    for (int j = 0; j < ArmJoints; j++)
                    {
                        if (t >= legEnd[j]) NewLeg(ref rng, j, t, from, to, legStart, legEnd);
                        double u = (double)(t - legStart[j]) / (legEnd[j] - legStart[j]);
                        targets[j] = from[j] + (to[j] - from[j]) * u;
                    }

                    if (rng.NextDouble() < 0.01) gripper = gripper > 0.5 ? 0 : 1;
                    targets[ArmJoints] = gripper;
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
                        predictor.ObserveState(downlink[k].sensed, state);
                    }
                }

                downlink.RemoveAll(f => f.arrive <= t);

                // Operator: a display frame.
                if (t >= nextDisplay && predictor.TryPredict(t, predicted, out int count))
                {
                    nextDisplay += displayInterval;
                    for (int j = 0; j < Math.Min(count, ArmJoints); j++)
                    {
                        errors.Add(Math.Abs(predicted[j].Value - truth[j]) * 180.0 / Math.PI);
                    }
                }
            }

            return errors;
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
    }
}
