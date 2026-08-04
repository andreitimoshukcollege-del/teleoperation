using System.Numerics;
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

            string csvPath = Path.Combine(outputDir, "metrics.csv");
            using (var sink = new CsvMetricSink(csvPath))
            {
                foreach (string predictorName in config.Predictors)
                {
                    foreach (string profileName in config.NetworkProfiles)
                    {
                        foreach (ulong seed in config.Seeds)
                        {
                            RunTrial(predictorName, config.Reconciler, profileName, seed, config, tracesDirectory, sink);
                        }
                    }
                }
            }

            string commandLine = "dotnet run --project core/Teleop.Eval -- sweep " + yamlPath;
            ManifestWriter.Write(Path.Combine(outputDir, "manifest.json"), config, yamlPath, commandLine);

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

            if (!Registries.Reconcilers.ContainsKey(config.Reconciler))
            {
                error = $"unknown reconciler '{config.Reconciler}' -- not in Registry/Registries.cs";
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
            string predictorName, string reconcilerName, string profileName, ulong seed,
            ExperimentConfig config, string tracesDirectory, IMetricSink sink)
        {
            var clock = new ManualClock(TicksPerSecond);

            var predictorConfig = new PredictorConfig(
                maxHorizonTicks: TicksPerSecond / 2, maxObservationGapTicks: TicksPerSecond,
                historyCapacity: 16, smoothingAlpha: 0.3f, smoothingBeta: 0.1f,
                processNoise: 0.01f, measurementNoise: 0.001f, maxLinearSpeed: 10f, maxAngularSpeed: 10f);
            var reconcilerConfig = new ReconcilerConfig(
                convergencePositionToleranceMeters: 0.005f, convergenceOrientationToleranceRadians: 0.017f,
                maxTimeToConvergenceTicks: TicksPerSecond, maxCorrectionLinearSpeedMetersPerSecond: 5f,
                maxCorrectionAngularSpeedRadPerSecond: 10f, rollbackHistoryCapacity: 16);
            var clockSyncConfig = new ClockSyncConfig(
                historyCapacity: 32, smoothingAlpha: 0.2f, maxAcceptableRttTicks: TicksPerSecond * 2,
                outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 3);

            IPredictor<Pose> predictor = Registries.Predictors[predictorName](predictorConfig, clock);
            IReconciler<Pose> reconciler = Registries.Reconcilers[reconcilerName](reconcilerConfig, sink, clock);
            var clockSync = new ClockSync(clockSyncConfig);

            NetworkProfileCatalog.TryResolve(profileName, TicksPerSecond, tracesDirectory, out NamedProfile namedProfile, out _);

            var uplinkInner = new LoopbackTransport(RawPoseCodec.EncodedSize, TransportCapacity);
            var downlinkInner = new LoopbackTransport(RobotStateFrameCodec.EncodedSize, TransportCapacity);
            ITransport uplink = MakeTransport(uplinkInner, namedProfile, new SeededRng(seed));
            ITransport downlink = MakeTransport(downlinkInner, namedProfile, new SeededRng(unchecked(seed + 1)));

            var plant = new RigidBodyPlant(Pose.Identity, TicksPerSecond);

            var operatorEndpoint = new OperatorEndpoint(
                new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink,
                clock, sink, clockSync, predictor, reconciler, InFlightCapacity);
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
                    // Metrics (owd_uplink_ms, owd_downlink_ms, correction_magnitude_mm/deg,
                    // jerk_mm_s3, time_to_convergence_ms) are recorded internally by
                    // OperatorEndpoint/SnapReconciler as a side effect of this call.
                }

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
