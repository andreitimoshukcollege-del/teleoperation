using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.Core.Contracts;
using Teleop.Core.Registry;
using Teleop.Core.Transport;
using Teleop.Core.Types;
using Teleop.Eval.JointBench;
using Teleop.Eval.Sweep;

namespace Teleop.Eval.Verification
{
    /// <summary>
    /// The half of <c>verify</c> that replays algorithms. The golden .tlog round trip only proves the codec is
    /// deterministic: no predictor, reconciler or playout policy ran in it, so an order-dependent or seed-leaking
    /// algorithm still passed (the 2026-11 bug hunt, A-03). Here every registry entry runs twice through the same
    /// pipeline the sweep and the joint bench use, on an impaired network with a fixed seed, and the two metric streams
    /// must match bit for bit.
    /// <list type="bullet">
    /// <item>Pose stacks (<see cref="SweepCommand.RunTrial"/>, the loopback pipeline with impairments and ClockSync): each
    /// pose predictor, reconciler and playout policy, against fixed defaults for the other two.</item>
    /// <item>Joint predictors (<see cref="JointPredictBenchCommand.SimulateIdeal"/>): each one the JetRover bench runs.</item>
    /// </list>
    /// </summary>
    internal static class DeterminismCheck
    {
        /// <summary>The impaired profile every stack runs on: delay, jitter and bursty loss, so reordering and gaps happen.</summary>
        internal const string Profile = "300ms-60j-2loss-bursty";

        internal const ulong Seed = 7;
        private const string DefaultPredictor = "const-vel", DefaultReconciler = "spring", DefaultPlayout = "percentile";

        private sealed class RecordingSink : IMetricSink
        {
            public readonly List<(string Name, long Bits, long Ticks)> Samples = new List<(string, long, long)>();

            public void Record(string name, double value, long ticks) => Samples.Add((name, BitConverter.DoubleToInt64Bits(value), ticks));
        }

        /// <summary>Runs every stack twice; returns one line per stack that differed or threw (empty: all deterministic).</summary>
        internal static List<string> Run(out int stacks)
        {
            var failures = new List<string>();
            var poseStacks = new List<(string Predictor, string Reconciler, string Playout)>();
            poseStacks.AddRange(Registries.Predictors.Keys.OrderBy(k => k, StringComparer.Ordinal).Select(p => (p, DefaultReconciler, DefaultPlayout)));
            poseStacks.AddRange(Registries.Reconcilers.Keys.OrderBy(k => k, StringComparer.Ordinal).Where(r => r != DefaultReconciler).Select(r => (DefaultPredictor, r, DefaultPlayout)));
            poseStacks.AddRange(Registries.PlayoutPolicies.Keys.OrderBy(k => k, StringComparer.Ordinal).Where(p => p != DefaultPlayout).Select(p => (DefaultPredictor, DefaultReconciler, p)));
            var config = new ExperimentConfig
            {
                TrialSteps = 900,
                StepIntervalTicks = 111_111, // 90 Hz on the sweep's 10 MHz clock, as the experiments use
                PlayoutBudgetMs = 50,
            };

            foreach (var (predictor, reconciler, playout) in poseStacks)
            {
                string name = $"{predictor} + {reconciler} + {playout}";
                Compare(name, failures, () =>
                {
                    var sink = new RecordingSink();
                    SweepCommand.RunTrial(predictor, reconciler, playout, Profile, Seed, config, tracesDirectory: string.Empty, sink);
                    return sink.Samples;
                });
            }

            if (!Teleop.Core.Transport.NetworkProfileCatalog.TryResolveParametric(Profile, 1_000_000, out NetworkProfile joint, out string? error))
            {
                failures.Add($"cannot resolve {Profile} for the joint bench: {error}");
                stacks = poseStacks.Count;
                return failures;
            }

            // The learned row runs the shipped model, as joint-predict-bench does; not finding it is a failure, not a skip.
            string? coreDir = FindCoreDirectory();
            if (coreDir == null || !JointPredictBenchCommand.LoadLearnedModel(System.IO.Path.Combine(coreDir, JointPredictBenchCommand.DefaultLearnedModel)))
            {
                failures.Add($"cannot load the learned predictor's model ({JointPredictBenchCommand.DefaultLearnedModel} under core/)");
            }

            // franka-replay needs the FR3's shaper, which the JetRover bench does not run (as joint-predict-bench).
            string[] jointPredictors = Registries.JointPredictors.Keys.Where(k => k != "franka-replay").OrderBy(k => k, StringComparer.Ordinal).ToArray();
            foreach (string predictor in jointPredictors)
            {
                Compare($"joint {predictor}", failures, () =>
                {
                    JointPredictBenchCommand.Samples s = JointPredictBenchCommand.SimulateIdeal(joint, predictor, Seed, seconds: 20, trueSpeed: 1.5, modelSpeed: 2.5);
                    return s.Errors.Select(v => ("error", BitConverter.DoubleToInt64Bits(v), 0L))
                        .Concat(s.Corrections.Select(v => ("correction", BitConverter.DoubleToInt64Bits(v), 0L))).ToList();
                });
            }

            stacks = poseStacks.Count + jointPredictors.Length;
            return failures;
        }

        /// <summary>The repository's core/ folder, found from the build output (as audit finds it), or null.</summary>
        private static string? FindCoreDirectory()
        {
            string? dir = AppContext.BaseDirectory;
            for (int i = 0; i < 15 && dir != null; i++)
            {
                if (System.IO.File.Exists(System.IO.Path.Combine(dir, "core", "Teleop.sln"))) return System.IO.Path.Combine(dir, "core");
                dir = System.IO.Path.GetDirectoryName(dir);
            }

            return null;
        }

        internal static void Compare(string name, List<string> failures, Func<List<(string Name, long Bits, long Ticks)>> run)
        {
            List<(string Name, long Bits, long Ticks)> first, second;
            try
            {
                first = run();
                second = run();
            }
            catch (Exception e)
            {
                failures.Add($"{name}: threw {e.GetType().Name}: {e.Message}");
                return;
            }

            if (first.Count == 0)
            {
                failures.Add($"{name}: recorded nothing, so nothing was compared");
                return;
            }

            int n = Math.Min(first.Count, second.Count);
            for (int i = 0; i < n; i++)
            {
                if (first[i] != second[i])
                {
                    failures.Add($"{name}: sample {i} differs: {first[i].Name}={BitConverter.Int64BitsToDouble(first[i].Bits)} at {first[i].Ticks} " +
                        $"against {second[i].Name}={BitConverter.Int64BitsToDouble(second[i].Bits)} at {second[i].Ticks}");
                    return;
                }
            }

            if (first.Count != second.Count) failures.Add($"{name}: {first.Count} samples against {second.Count}");
        }
    }
}
