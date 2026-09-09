using System.Numerics;
using System.Threading.Tasks;
using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Pipeline;
using Teleop.Core.Plant;
using Teleop.Core.Registry;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;
using Teleop.Eval.Metrics;

namespace Teleop.Eval.Sweep
{
    /// <summary>
    /// Implements <c>sweep</c>: runs the (predictor × network profile × seed) matrix an
    /// experiment YAML describes, through the same loopback pipeline
    /// <c>LoopbackPipelineIntegrationTests</c> already proves correct, and writes raw metric rows
    /// plus a <c>manifest.json</c> under <c>results/&lt;id&gt;/&lt;timestamp&gt;/</c>. Emits no
    /// percentile table itself -- per <c>.claude/commands/run-sweep.md</c>'s own step split, that
    /// aggregation happens after this tool runs, from the raw CSV it writes.
    /// </summary>
    public static class SweepCommand
    {
        private const long TicksPerSecond = 10_000_000;
        private const int InFlightCapacity = 64;
        private const int MaxDatagramsPerStep = 64;
        private const int TransportCapacity = 64;

        public static int Run(string[] args)
        {
            if (args.Length < 2)
            {
                Console.Error.WriteLine("usage: sweep <experiment.yaml>");
                return 64; // EX_USAGE
            }

            string yamlPath = args[1];
            if (!File.Exists(yamlPath))
            {
                Console.Error.WriteLine($"sweep: experiment file not found: {yamlPath}");
                return 66; // EX_NOINPUT
            }

            ExperimentConfig config;
            try
            {
                config = ExperimentConfig.Load(yamlPath);
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine($"sweep: failed to parse {yamlPath}: {ex.Message}");
                return 1;
            }

            string? repoRoot = FindRepoRoot();
            if (repoRoot == null)
            {
                Console.Error.WriteLine("sweep: could not locate repo root (core/Teleop.sln) from the build output directory.");
                return 66;
            }

            string tracesDirectory = Path.Combine(repoRoot, "core", "testdata", "traces");

            if (!TryValidate(config, tracesDirectory, out string? validationError))
            {
                Console.Error.WriteLine($"sweep: {validationError}");
                return 1;
            }

            string timestamp = DateTime.UtcNow.ToString("yyyyMMdd-HHmmssZ");
            string outputDir = Path.Combine(repoRoot, "results", config.Id, timestamp);
            Directory.CreateDirectory(outputDir);

            // One metrics.csv per (predictor, network profile), pooling every seed of that
            // configuration together -- multiple seeds are multiple trials of the SAME
            // configuration, meant to be pooled into one percentile distribution, not
            // interleaved with other configurations in a single undifferentiated file with no
            // way to tell them apart afterward.
            //
            // Parallelized across (predictor, profile) pairs, not across seeds within a pair:
            // each pair already gets its own CsvMetricSink/output file, so pairs share no mutable
            // state and running them concurrently is safe. Seeds within one pair stay sequential
            // on purpose -- they'd otherwise be multiple threads calling the same CsvMetricSink's
            // non-thread-safe StreamWriter.WriteLine concurrently, corrupting metrics.csv.
            List<ResolvedStack> stacks =
                ResolvedStack.Enumerate(
                    config.Predictors, config.ResolveReconcilers(), config.ResolvePlayoutPolicies());

            var configPairs = new List<(ResolvedStack Stack, string Profile)>();
            foreach (ResolvedStack stack in stacks)
            {
                foreach (string profileName in config.NetworkProfiles)
                {
                    configPairs.Add((stack, profileName));
                }
            }

            Parallel.ForEach(configPairs, pair =>
            {
                string configDir = Path.Combine(outputDir, pair.Stack.Name, pair.Profile);
                Directory.CreateDirectory(configDir);
                string csvPath = Path.Combine(configDir, "metrics.csv");

                using var sink = new CsvMetricSink(csvPath);
                foreach (ulong seed in config.Seeds)
                {
                    RunTrial(
                        pair.Stack.Predictor, pair.Stack.Reconciler, pair.Stack.PlayoutPolicy,
                        pair.Profile, seed, config, tracesDirectory, sink);
                }
            });

            string commandLine = "dotnet run --project core/Teleop.Eval -- sweep " + yamlPath;
            ManifestWriter.Write(
                Path.Combine(outputDir, "manifest.json"), config, stacks, yamlPath, commandLine);

            Console.WriteLine($"sweep: wrote {outputDir}");
            return 0;
        }

        private static bool TryValidate(ExperimentConfig config, string tracesDirectory, out string? error)
        {
            if (string.IsNullOrWhiteSpace(config.Id))
            {
                error = "experiment config has no 'id'";
                return false;
            }

            if (config.Seeds.Count == 0)
            {
                error = "experiment config has no 'seeds'";
                return false;
            }

            if (config.Predictors.Count == 0)
            {
                error = "experiment config has no 'predictors'";
                return false;
            }

            foreach (string predictorName in config.Predictors)
            {
                if (!Registries.Predictors.ContainsKey(predictorName))
                {
                    error = $"unknown predictor '{predictorName}' -- not in Registry/Registries.cs";
                    return false;
                }
            }

            if (config.HasConflictingReconcilerKeys)
            {
                error = "experiment config sets both 'reconciler' and 'reconcilers' -- pick one; " +
                    "which would win is not something a reader of the YAML could predict";
                return false;
            }

            List<string> reconcilers = config.ResolveReconcilers();
            if (reconcilers.Count == 0)
            {
                error = "experiment config has no 'reconciler' or 'reconcilers'";
                return false;
            }

            foreach (string reconcilerName in reconcilers)
            {
                if (!Registries.Reconcilers.ContainsKey(reconcilerName))
                {
                    error = $"unknown reconciler '{reconcilerName}' -- not in Registry/Registries.cs";
                    return false;
                }
            }

            if (config.ConvergenceBudgetMs <= 0.0)
            {
                error = "'convergenceBudgetMs' must be positive -- it is a smoothed reconciler's " +
                    "natural frequency, so there is no defined response without it";
                return false;
            }

            if (config.MaxCorrectionLinearSpeedMetersPerSecond <= 0f ||
                config.MaxCorrectionAngularSpeedRadiansPerSecond <= 0f)
            {
                error = "'maxCorrectionLinearSpeedMetersPerSecond' and " +
                    "'maxCorrectionAngularSpeedRadiansPerSecond' must be positive -- a zero cap " +
                    "can never converge";
                return false;
            }

            if (config.HasConflictingPlayoutKeys)
            {
                error = "experiment config sets both 'playoutPolicy' and 'playoutPolicies' -- pick " +
                    "one; which would win is not something a reader of the YAML could predict";
                return false;
            }

            foreach (string playoutName in config.ResolvePlayoutPolicies())
            {
                if (!Registries.PlayoutPolicies.ContainsKey(playoutName))
                {
                    error = $"unknown playout policy '{playoutName}' -- not in Registry/Registries.cs";
                    return false;
                }
            }

            if (config.PlayoutBudgetMs < 0.0)
            {
                error = "'playoutBudgetMs' must not be negative -- a playout policy cannot schedule " +
                    "a sample before it was captured";
                return false;
            }

            if (config.PlayoutHistoryCapacity <= 0)
            {
                error = "'playoutHistoryCapacity' must be positive";
                return false;
            }

            if (config.PlayoutTargetPercentile <= 0.0 || config.PlayoutTargetPercentile > 1.0)
            {
                error = "'playoutTargetPercentile' must be in (0, 1] -- 1.0 is the window maximum " +
                    "and a legitimate operating point, 0.0 corresponds to no order statistic";
                return false;
            }

            if (config.PlayoutDelayWindowSamples <= 0)
            {
                error = "'playoutDelayWindowSamples' must be positive -- a quantile over an empty " +
                    "window is undefined";
                return false;
            }

            if (config.PlayoutMinBudgetMs < 0.0 || config.PlayoutMaxBudgetMs < config.PlayoutMinBudgetMs)
            {
                error = "'playoutMinBudgetMs' and 'playoutMaxBudgetMs' must satisfy " +
                    "0 <= min <= max -- they are the clamp a deriving policy settles inside";
                return false;
            }

            // A budget the buffer has no room to hold turns into discarded samples, and the late
            // rate then measures the capacity rather than the policy -- the one way this axis can
            // report a confidently wrong number. Caught here rather than left to the reader.
            //
            // Checked against the largest budget any policy in this config could reach, not just
            // the fixed one: `percentile` settles anywhere up to 'playoutMaxBudgetMs', so a
            // capacity sized only for 'playoutBudgetMs' would silently cap it.
            double largestBudgetMs = config.PlayoutBudgetMs;
            foreach (string playoutName in config.ResolvePlayoutPolicies())
            {
                if (playoutName != "immediate" && playoutName != "fixed")
                {
                    largestBudgetMs = Math.Max(largestBudgetMs, config.PlayoutMaxBudgetMs);
                }
            }

            long budgetSteps = config.StepIntervalTicks <= 0
                ? 0
                : MillisecondsToTicks(largestBudgetMs) / config.StepIntervalTicks;
            if (budgetSteps >= config.PlayoutHistoryCapacity)
            {
                error = $"a playout budget of {largestBudgetMs}ms spans {budgetSteps} steps at the " +
                    $"configured step interval, which does not fit in a 'playoutHistoryCapacity' " +
                    $"of {config.PlayoutHistoryCapacity} -- the buffer would discard samples it had " +
                    "room to hold, and the measured late-arrival rate would describe the capacity " +
                    "rather than the policy";
                return false;
            }

            if (config.NetworkProfiles.Count == 0)
            {
                error = "experiment config has no 'networkProfiles'";
                return false;
            }

            foreach (string profileName in config.NetworkProfiles)
            {
                if (!NetworkProfileCatalog.TryResolve(profileName, TicksPerSecond, tracesDirectory, out _, out string? profileError))
                {
                    error = profileError;
                    return false;
                }
            }

            if (config.TrialSteps <= 0)
            {
                error = "experiment config's 'trialSteps' must be positive";
                return false;
            }

            if (config.StepIntervalTicks <= 0)
            {
                error = "experiment config's 'stepIntervalTicks' must be positive";
                return false;
            }

            error = null;
            return true;
        }

        private static void RunTrial(
            string predictorName, string reconcilerName, string playoutPolicyName, string profileName,
            ulong seed, ExperimentConfig config, string tracesDirectory, IMetricSink sink)
        {
            var clock = new ManualClock(TicksPerSecond);

            var predictorConfig = new PredictorConfig(
                maxHorizonTicks: TicksPerSecond / 2, maxObservationGapTicks: TicksPerSecond,
                historyCapacity: 16, smoothingAlpha: 0.3f, smoothingBeta: 0.1f,
                processNoise: 0.01f, measurementNoise: 0.001f, maxLinearSpeed: 10f, maxAngularSpeed: 10f);
            // Tolerances stay hardcoded: they decide what counts as a correction at all, so
            // varying them would change which events are counted rather than how they are spent,
            // and no experiment needs that yet (experiments/CLAUDE.md: no invented knobs). The
            // budget and the two rate caps come from the config -- they are the operating point.
            var reconcilerConfig = new ReconcilerConfig(
                convergencePositionToleranceMeters: 0.005f, convergenceOrientationToleranceRadians: 0.017f,
                maxTimeToConvergenceTicks: MillisecondsToTicks(config.ConvergenceBudgetMs),
                maxCorrectionLinearSpeedMetersPerSecond: config.MaxCorrectionLinearSpeedMetersPerSecond,
                maxCorrectionAngularSpeedRadPerSecond: config.MaxCorrectionAngularSpeedRadiansPerSecond,
                rollbackHistoryCapacity: 16);
            var clockSyncConfig = new ClockSyncConfig(
                historyCapacity: 32, smoothingAlpha: 0.2f, maxAcceptableRttTicks: TicksPerSecond * 2,
                outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 3);

            // The two filter-noise fields and MaxAdaptationRatePerSecond stay at placeholders: no
            // policy registered here reads them, and `kalman-jitter`/`adaptive` will promote them
            // when they land, the same way ConvergenceBudgetMs was promoted for the reconcilers
            // (experiments/CLAUDE.md: no invented knobs). Everything `percentile` reads is now a
            // real config field, because a tracking policy with a hardcoded window and quantile
            // would be one fixed operating point wearing an adaptive policy's name.
            var playoutConfig = new PlayoutPolicyConfig(
                historyCapacity: config.PlayoutHistoryCapacity,
                initialDelayBudgetTicks: MillisecondsToTicks(config.PlayoutBudgetMs),
                minDelayBudgetTicks: MillisecondsToTicks(config.PlayoutMinBudgetMs),
                maxDelayBudgetTicks: MillisecondsToTicks(config.PlayoutMaxBudgetMs),
                targetPercentile: config.PlayoutTargetPercentile,
                delayWindowSamples: config.PlayoutDelayWindowSamples,
                delayProcessNoise: 0.01f, delayMeasurementNoise: 0.001f,
                maxAdaptationRatePerSecond: 0.0, lossWeight: 0.5);

            IPredictor<Pose> predictor = Registries.Predictors[predictorName](predictorConfig, clock);
            IReconciler<Pose> reconciler = Registries.Reconcilers[reconcilerName](reconcilerConfig, sink, clock);
            IPlayoutPolicy<Pose> playoutPolicy = Registries.PlayoutPolicies[playoutPolicyName](playoutConfig, sink, clock);
            var clockSync = new ClockSync(clockSyncConfig);

            NetworkProfileCatalog.TryResolve(profileName, TicksPerSecond, tracesDirectory, out NamedProfile namedProfile, out _);

            var uplinkInner = new LoopbackTransport(RawPoseCodec.EncodedSize, TransportCapacity);
            var downlinkInner = new LoopbackTransport(RobotStateFrameCodec.EncodedSize, TransportCapacity);
            ITransport uplink = MakeTransport(uplinkInner, namedProfile, new SeededRng(seed));
            ITransport downlink = MakeTransport(downlinkInner, namedProfile, new SeededRng(unchecked(seed + 1)));

            var plant = new RigidBodyPlant(Pose.Identity, TicksPerSecond);

            var operatorEndpoint = new OperatorEndpoint(
                new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink,
                clock, sink, clockSync, predictor, reconciler, playoutPolicy, InFlightCapacity);
            var robotEndpoint = new RobotEndpoint(
                plant, new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink, clock, MaxDatagramsPerStep);

            for (int step = 0; step < config.TrialSteps; step++)
            {
                clock.AdvanceTicks(config.StepIntervalTicks);
                long now = clock.NowTicks;

                Pose operatorPose = SyntheticOperatorMotion(step, config.StepIntervalTicks, TicksPerSecond);
                operatorEndpoint.SubmitCommand(operatorPose, Vector3.Zero, Vector3.Zero, gripper: 0f, now);

                robotEndpoint.Step(now);

                while (operatorEndpoint.TryReceiveState(now, out _))
                {
                    // owd_uplink_ms and owd_downlink_ms are recorded internally as a side effect of
                    // this call. Nothing reaches the predictor or reconciler here any more -- that
                    // moved to the playout drain below (docs/adr/0012-playout-policy-wiring.md).
                }

                // The playout drain. This is what feeds the predictor and reconciler, so
                // correction_magnitude_mm/deg come from here, along with the buffering axis's own
                // playout_delay_ms/playout_budget_ms/playout_occupancy. Omitting it does not fail
                // loudly -- it silently produces a trial in which the estimator never observed
                // anything, which is precisely the shape of the EstimateRobotState defect below.
                while (operatorEndpoint.TryPlayoutState(now, out _))
                {
                }

                // The frame tick. docs/setup.md's callback-placement table puts EstimateRobotState
                // in Application.onBeforeRender -- once per frame, after the inbound queue is
                // drained -- and one sweep step is one frame, so it belongs here.
                //
                // It is not optional bookkeeping: EstimateRobotState is the only thing that calls
                // IReconciler.Reconcile, and Reconcile is what emits jerk_mm_s3 and
                // time_to_convergence_ms (docs/metrics.md §5). Without this call a sweep records
                // correction *magnitude* and nothing else -- and magnitude is identical across
                // reconcilers by construction, since it is measured from the predictor's
                // disagreement before any reconciler has acted. That made the whole
                // Reconciliation/ axis unmeasurable by sweep: every reconciler produced byte-
                // identical output. The returned pose is the displayed state a host would render;
                // nothing here renders, so it is discarded, but the call must still happen.
                _ = operatorEndpoint.EstimateRobotState(now);

                // Online prediction error: compares the predictor's live estimate against the
                // plant's simultaneous ground truth. This is a simplified proxy, not
                // docs/metrics.md §4's full counterfactual, horizon-binned methodology (which
                // needs an offline .tlog replay scorer that does not exist yet) -- see
                // docs/metrics.md's definition of these two metric names for the distinction.
                Pose predicted = predictor.Predict(now);
                Pose truth = plant.State.Value;
                double positionErrorMm = PoseMath.PositionErrorMeters(predicted, truth) * 1000.0;
                double orientationErrorDeg = PoseMath.OrientationErrorRadians(predicted, truth) * (180.0 / Math.PI);
                sink.Record("prediction_position_error_mm", positionErrorMm, now);
                sink.Record("prediction_orientation_error_deg", orientationErrorDeg, now);
            }
        }

        /// <summary>
        /// A duration in milliseconds as ticks on this command's internal timebase. Rounded, not
        /// truncated, so a budget that does not divide evenly into ticks lands on the nearest tick
        /// rather than systematically short.
        /// </summary>
        private static long MillisecondsToTicks(double milliseconds) =>
            (long)Math.Round(milliseconds / 1000.0 * TicksPerSecond);

        private static ITransport MakeTransport(ITransport inner, NamedProfile namedProfile, SeededRng rng) =>
            namedProfile.TraceTicks != null
                ? new EmulatedTransport(inner, namedProfile.TraceTicks, namedProfile.Profile, rng, TransportCapacity)
                : new EmulatedTransport(inner, namedProfile.Profile, rng, TransportCapacity);

        /// <summary>
        /// A fixed, deterministic operator trajectory -- a slow sinusoidal sweep -- shared by
        /// every trial regardless of seed. The seed instead varies the network realization
        /// (<see cref="MakeTransport"/>), matching this sweep's actual research question ("how do
        /// algorithms perform under different network realizations"), not a study of varied
        /// operator motion.
        /// </summary>
        private static Pose SyntheticOperatorMotion(int step, long stepIntervalTicks, long ticksPerSecond)
        {
            double t = step * stepIntervalTicks / (double)ticksPerSecond;
            float x = (float)(Math.Sin(t) * 0.5);
            float z = (float)(1.0 + Math.Cos(t * 0.7) * 0.3);
            return new Pose(new Vector3(x, 0f, z), Quaternion.Identity);
        }

        private static string? FindRepoRoot()
        {
            // Mirrors AuditCommand.FindCoreDirectory: Directory.Build.props redirects build
            // output to <repoRoot>/build/..., a sibling of core/, so this walks up from the build
            // output directory looking for an ancestor whose "core" subdirectory has Teleop.sln.
            string? dir = AppContext.BaseDirectory;
            for (int i = 0; i < 15 && dir != null; i++)
            {
                if (File.Exists(Path.Combine(dir, "core", "Teleop.sln")))
                {
                    return dir;
                }

                dir = Path.GetDirectoryName(dir);
            }

            return null;
        }
    }
}
