using System;
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

namespace Teleop.Eval.MoveArm
{
    /// <summary>
    /// A minimal, general-purpose "move the real JetRover arm to a given Cartesian target"
    /// operator tool -- everything <see cref="ClockSyncCheck.ClockSyncCheckCommand"/> needs to
    /// send a real <see cref="CommandFrame"/> to an already-running <c>Teleop.RobotHost</c>, minus
    /// that tool's <c>ClockSync</c> diagnostics reporting (this one only cares whether the arm
    /// gets there, not the link's latency numbers). Talks to an unmodified <see cref="RobotEndpoint"/>
    /// exactly the way a real operator eventually will: builds a real <see cref="OperatorEndpoint"/>
    /// (the "none"/"snap" predictor+reconciler pair -- the simplest pairing that lets commands flow),
    /// submits the target repeatedly for the configured duration (repetition is deliberate, not
    /// redundant: any single UDP datagram can be lost, and the plant's own "hold" gap policy means
    /// this is safe to repeat), then exits.
    ///
    /// <b>This drives a real <c>IRobotPlant</c> when pointed at the real Jetson.</b> See
    /// <see cref="MoveArmArgs"/>'s <c>--confirm-hardware-motion</c> requirement.
    /// </summary>
    internal static class MoveArmCommand
    {
        private const int MaxDatagramBytes = 128;
        private const int InFlightCapacity = 32;

        public static int Run(string[] args)
        {
            MoveArmArgs? parsed = MoveArmArgs.TryParse(args, out string? error);
            if (parsed is null)
            {
                Console.Error.WriteLine(error);
                Console.Error.WriteLine(MoveArmArgs.Usage);
                return 64; // EX_USAGE
            }

            MoveArmArgs a = parsed.Value;
            var targetPosition = new Vector3(a.X, a.Y, a.Z);

            var clock = new Teleop.Eval.Time.MonotonicClock();
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

            var sink = new NullMetricSink();
            IPredictor<Pose> predictor = Registries.Predictors["none"](predictorConfig, clock);
            IReconciler<Pose> reconciler = Registries.Reconcilers["snap"](reconcilerConfig, sink, clock);
            var clockSync = new Teleop.Core.Time.ClockSync(clockSyncConfig);

            var operatorEndpoint = new OperatorEndpoint(
                new RawPoseCodec(), new RobotStateFrameCodec(), transport, transport,
                clock, sink, clockSync, predictor, reconciler, InFlightCapacity);

            Console.WriteLine(
                $"[move-arm] sending {targetPosition} (gripper={a.Gripper:0.##}) to {remoteEndPoint} at " +
                $"{a.RateHz:0.#} Hz for {a.DurationSeconds:0.#}s -- the arm will move once to this pose and " +
                "hold. Ctrl+C to stop early.");

            long stepIntervalMs = (long)(1000.0 / a.RateHz);
            long endAtMs = Environment.TickCount64 + (long)(a.DurationSeconds * 1000.0);
            int acceptedRoundTrips = 0;

            using var stop = new ManualResetEventSlim(initialState: false);
            Console.CancelKeyPress += (_, cancelArgs) =>
            {
                cancelArgs.Cancel = true;
                stop.Set();
            };

            while (!stop.IsSet && Environment.TickCount64 < endAtMs)
            {
                long now = clock.NowTicks;
                operatorEndpoint.SubmitCommand(
                    new Pose(targetPosition, Quaternion.Identity), Vector3.Zero, Vector3.Zero, a.Gripper, now);

                while (operatorEndpoint.TryReceiveState(clock.NowTicks, out _))
                {
                    acceptedRoundTrips++;
                }

                Thread.Sleep((int)stepIntervalMs);
            }

            Console.WriteLine(
                $"[move-arm] done -- {acceptedRoundTrips} round trip(s) acknowledged by the robot. " +
                "The arm should now be holding at the target.");
            return 0;
        }

        /// <summary>Discards every metric -- this tool reports pass/fail by whether the robot
        /// acknowledged the command, not by any latency figure.</summary>
        private sealed class NullMetricSink : IMetricSink
        {
            public void Record(string name, double value, long ticks)
            {
            }
        }
    }
}
