using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Numerics;
using System.Threading;
using Teleop.Core.Contracts;
using Teleop.Core.Pipeline;
using Teleop.Core.Registry;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;
using Teleop.Eval.Net;

namespace Teleop.Eval.ClockSyncCheck
{
    /// <summary>
    /// Phase 3 of the JetRover integration (docs/adr/0007-jetrover-plant-and-robot-host.md): the
    /// first genuinely cross-machine exercise of <c>Time/ClockSync.cs</c>. Every earlier use --
    /// every sweep trial, the whole loopback baseline -- runs operator and robot logic in one
    /// process on one <c>ITimeAuthority</c>, which proves the arithmetic but not the assumption
    /// underneath it: <c>ClockSync.AddRoundTrip</c> used to add and subtract operator-domain and
    /// robot-domain ticks directly (<c>operatorSendTicks - robotRecvTicks</c>), which is only
    /// numerically meaningful if both sides' <see cref="ITimeAuthority.TicksPerSecond"/> agree.
    /// On one process they trivially do. Across two real machines -- this dev host's Windows
    /// <c>Stopwatch</c> and the Jetson's Linux ARM one -- they are not, and this tool's first run
    /// proved it (Windows reported 10,000,000; .NET on Linux reported 1,000,000,000, and every
    /// RTT/offset below came out 100x inflated).
    ///
    /// That is fixed at the source now, not worked around here:
    /// docs/adr/0008-clocksync-cross-rate-normalization.md made <c>ClockSync</c> take both rates
    /// on every call and rescale the robot's stamps into operator-tick units first, and gave
    /// <c>RobotStateFrame</c> a <c>TicksPerSecond</c> field (wire version 2) so the operator
    /// learns the robot's rate from the reply itself. The figures below are therefore
    /// rate-corrected without a human doing anything. This tool still prints its own rate, and
    /// <c>Teleop.RobotHost</c> still prints its own at startup (see its <c>Program.cs</c>),
    /// because seeing the two rates remains the fastest way to recognize this specific failure if
    /// it ever reappears -- but comparing them by hand is no longer a precondition for trusting
    /// the numbers. The robot's rate is not printed here: it arrives inside a decoded
    /// <c>RobotStateFrame</c> that <see cref="OperatorEndpoint.TryReceiveState"/> consumes
    /// internally and reports only as a <c>LatencyTrace</c>, and widening that API purely for a
    /// diagnostic print is not worth the coupling.
    ///
    /// Talks to an already-running <c>Teleop.RobotHost</c> (unmodified) over real UDP, exactly
    /// the way a real operator eventually will: builds a real <see cref="OperatorEndpoint"/>
    /// (the "none"/"snap" predictor+reconciler pair -- the simplest pairing that lets
    /// <see cref="OperatorEndpoint.TryReceiveState"/> run for real, since this tool cares about
    /// clock/OWD numbers, not prediction quality), submits a single fixed <see cref="Pose"/>
    /// repeatedly for the configured duration, and reports <see cref="Time.ClockSync"/>'s final
    /// diagnostics plus the distribution of <c>owd_uplink_ms</c>/<c>owd_downlink_ms</c> (both
    /// already-defined metrics, docs/metrics.md §2 -- this tool records them, it does not invent
    /// them).
    ///
    /// <b>This drives a real <c>IRobotPlant</c> when pointed at the real Jetson.</b> A fixed
    /// target, repeated, is deliberately the safest possible real command (converges once, then
    /// holds -- the same shape <c>JetRoverPlantTests.Command_RepeatingTheSameTarget_...</c>
    /// exercises against a fake relay) -- see <see cref="ClockSyncCheckArgs"/>'s
    /// <c>--confirm-hardware-motion</c> requirement.
    /// </summary>
    internal static class ClockSyncCheckCommand
    {
        private const int MaxDatagramBytes = 128;
        private const int InFlightCapacity = 32;

        // A fixed, reachable, gentle target -- same magnitude as JetRoverPlantTests'
        // ReachableTarget, chosen for the same reason: well inside the arm's working envelope,
        // so a repeated identical command converges quickly and then sends a steady zero delta.
        private static readonly Vector3 FixedTargetPosition = new Vector3(0.15f, 0f, 0.08f);

        public static int Run(string[] args)
        {
            ClockSyncCheckArgs? parsed = ClockSyncCheckArgs.TryParse(args, out string? error);
            if (parsed is null)
            {
                Console.Error.WriteLine(error);
                Console.Error.WriteLine(ClockSyncCheckArgs.Usage);
                return 64; // EX_USAGE
            }

            ClockSyncCheckArgs a = parsed.Value;

            var clock = new Teleop.Eval.Time.MonotonicClock();
            Console.WriteLine(
                $"[clocksync-check] operator TicksPerSecond={clock.TicksPerSecond}. The robot-side " +
                "Teleop.RobotHost process prints its own rate at startup; the two need not match -- " +
                "ClockSync now normalizes across them per ADR 0008, using the rate carried on every " +
                "RobotStateFrame (wire v2). Both ends must be built from the same Teleop.Core revision.");

            var remoteEndPoint = new IPEndPoint(a.RemoteHost, a.RemotePort);
            using var transport = new UdpTransport(a.LocalPort, remoteEndPoint, MaxDatagramBytes, clock);

            var predictorConfig = new PredictorConfig(
                maxHorizonTicks: clock.TicksPerSecond / 2, maxObservationGapTicks: clock.TicksPerSecond,
                historyCapacity: 16, smoothingAlpha: 0.3f, smoothingBeta: 0.1f,
                processNoise: 0.01f, measurementNoise: 0.001f, maxLinearSpeed: 10f, maxAngularSpeed: 10f);
            var reconcilerConfig = new ReconcilerConfig(
                convergencePositionToleranceMeters: 0.005f, convergenceOrientationToleranceRadians: 0.017f,
                maxTimeToConvergenceTicks: clock.TicksPerSecond, maxCorrectionLinearSpeedMetersPerSecond: 5f,
                maxCorrectionAngularSpeedRadPerSecond: 10f, rollbackHistoryCapacity: 16);
            var clockSyncConfig = new ClockSyncConfig(
                historyCapacity: 32, smoothingAlpha: 0.2f, maxAcceptableRttTicks: clock.TicksPerSecond * 2,
                outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 3);

            var sink = new RecordingMetricSink();
            IPredictor<Pose> predictor = Registries.Predictors["none"](predictorConfig, clock);
            IReconciler<Pose> reconciler = Registries.Reconcilers["snap"](reconcilerConfig, sink, clock);
            var clockSync = new Teleop.Core.Time.ClockSync(clockSyncConfig);

            var operatorEndpoint = new OperatorEndpoint(
                new RawPoseCodec(), new RobotStateFrameCodec(), transport, transport,
                clock, sink, clockSync, predictor, reconciler, InFlightCapacity);

            Console.WriteLine(
                $"[clocksync-check] sending a fixed CommandFrame to {remoteEndPoint} at {a.RateHz:0.#} Hz " +
                $"for {a.DurationSeconds:0.#}s. Target position {FixedTargetPosition} -- on real hardware " +
                "the arm will move once to this pose and hold. Ctrl+C to stop early.");

            long stepIntervalMs = (long)(1000.0 / a.RateHz);
            long endAtMs = Environment.TickCount64 + (long)(a.DurationSeconds * 1000.0);
            uint sequence = 0;

            using var stop = new ManualResetEventSlim(initialState: false);
            Console.CancelKeyPress += (_, cancelArgs) =>
            {
                cancelArgs.Cancel = true;
                stop.Set();
            };

            while (!stop.IsSet && Environment.TickCount64 < endAtMs)
            {
                long now = clock.NowTicks;
                sequence++;
                operatorEndpoint.SubmitCommand(
                    new Pose(FixedTargetPosition, Quaternion.Identity), Vector3.Zero, Vector3.Zero,
                    gripper: 0f, now);

                while (operatorEndpoint.TryReceiveState(clock.NowTicks, out _))
                {
                    // owd_uplink_ms/owd_downlink_ms recorded internally as a side effect.
                }

                Thread.Sleep((int)stepIntervalMs);
            }

            Report(clockSync, sink, clock);
            return 0;
        }

        private static void Report(
            Teleop.Core.Time.ClockSync clockSync, RecordingMetricSink sink, ITimeAuthority clock)
        {
            ClockSyncDiagnostics diag = clockSync.Diagnostics;
            double ticksToMs(long ticks) => ticks * 1000.0 / clock.TicksPerSecond;

            Console.WriteLine();
            Console.WriteLine("=== ClockSync diagnostics ===");
            Console.WriteLine($"IsSynced:                {diag.IsSynced}");
            Console.WriteLine($"AcceptedSampleCount:     {diag.AcceptedSampleCount}");
            Console.WriteLine($"RejectedSampleCount:     {diag.RejectedSampleCount}");
            Console.WriteLine($"OffsetMs (smoothed):     {ticksToMs(diag.OffsetTicks):0.###}");
            Console.WriteLine($"OffsetUncertaintyMs:     {ticksToMs(diag.OffsetUncertaintyTicks):0.###}");
            Console.WriteLine($"LastRttMs:               {ticksToMs(diag.LastRttTicks):0.###}");
            Console.WriteLine($"MinRttMs:                {ticksToMs(diag.MinRttTicks):0.###}");

            Console.WriteLine();
            Console.WriteLine("=== One-way delay (docs/metrics.md §2) ===");
            ReportPercentiles(sink, "owd_uplink_ms");
            ReportPercentiles(sink, "owd_downlink_ms");

            Console.WriteLine();
            Console.WriteLine(
                "Sanity check: owd_uplink_ms + owd_downlink_ms should track close to MinRttMs above -- " +
                "both derive from the same round trips, corrected by the same smoothed offset.");
        }

        private static void ReportPercentiles(RecordingMetricSink sink, string metricName)
        {
            List<double> values = sink.Values(metricName);
            if (values.Count == 0)
            {
                Console.WriteLine($"{metricName}: no samples recorded.");
                return;
            }

            values.Sort();
            Console.WriteLine(
                $"{metricName}: n={values.Count} p50={Percentile(values, 0.50):0.###} " +
                $"p95={Percentile(values, 0.95):0.###} p99={Percentile(values, 0.99):0.###}");
        }

        // Nearest-rank percentile over an already-sorted list -- matches the p50/p95/p99
        // convention docs/metrics.md mandates for every reported distribution.
        private static double Percentile(IReadOnlyList<double> sortedValues, double fraction)
        {
            int index = (int)Math.Ceiling(fraction * sortedValues.Count) - 1;
            index = Math.Max(0, Math.Min(sortedValues.Count - 1, index));
            return sortedValues[index];
        }

        /// <summary>
        /// The simplest possible in-memory <see cref="IMetricSink"/>: every named sample kept for
        /// later percentile computation. No file I/O (unlike <c>CsvMetricSink</c>) -- this tool is
        /// a diagnostic check, not a manifested experiment result, so nothing here belongs under
        /// <c>results/</c>.
        /// </summary>
        private sealed class RecordingMetricSink : IMetricSink
        {
            private readonly Dictionary<string, List<double>> _samples = new Dictionary<string, List<double>>();

            public void Record(string name, double value, long ticks)
            {
                if (!_samples.TryGetValue(name, out List<double>? list))
                {
                    list = new List<double>();
                    _samples[name] = list;
                }

                list.Add(value);
            }

            public List<double> Values(string name) =>
                _samples.TryGetValue(name, out List<double>? list) ? list.ToList() : new List<double>();
        }
    }
}
