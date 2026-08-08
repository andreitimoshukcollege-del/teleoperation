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
    /// submits the target repeatedly, and watches <see cref="OperatorEndpoint.EstimateRobotState"/>
    /// (the robot's own reported position, snapped immediately by the "snap" reconciler) until it
    /// actually converges within <see cref="MoveArmArgs.PositionToleranceMeters"/> for several
    /// consecutive samples, rather than firing for a fixed wall-clock duration and hoping. A fixed
    /// duration was tried first and was not reliable: convergence needs enough successfully
    /// delivered UDP round trips over a real, lossy Tailscale link, and <c>JetRoverPlant</c> only
    /// closes a fraction of the remaining distance per round trip (its own
    /// <c>MaxDirectionMagnitude</c> clamp) -- a fixed short window could exit before the arm was
    /// actually done moving, silently reporting success on a partially-converged position. Exits
    /// non-zero if convergence is not reached before <see cref="MoveArmArgs.TimeoutSeconds"/>, per
    /// root CLAUDE.md invariant 10 (never fake a pass).
    ///
    /// <b>This drives a real <c>IRobotPlant</c> when pointed at the real Jetson.</b> See
    /// <see cref="MoveArmArgs"/>'s <c>--confirm-hardware-motion</c> requirement.
    /// </summary>
    internal static class MoveArmCommand
    {
        private const int MaxDatagramBytes = 128;
        private const int InFlightCapacity = 32;

        // How many consecutive in-tolerance samples before declaring convergence, rather than
        // acting on a single sample that might be a fluke (e.g. a stale reply still in flight).
        private const int ConsecutiveInToleranceSamplesRequired = 3;

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
                $"{a.RateHz:0.#} Hz, waiting up to {a.TimeoutSeconds:0.#}s for the robot's reported " +
                $"position to converge within {a.PositionToleranceMeters * 1000:0.#}mm. Ctrl+C to stop early.");

            long stepIntervalMs = (long)(1000.0 / a.RateHz);
            long timeoutAtMs = Environment.TickCount64 + (long)(a.TimeoutSeconds * 1000.0);
            int acceptedRoundTrips = 0;
            int consecutiveInTolerance = 0;
            bool everObservedAnyState = false;
            Vector3 lastObservedPosition = default;

            using var stop = new ManualResetEventSlim(initialState: false);
            Console.CancelKeyPress += (_, cancelArgs) =>
            {
                cancelArgs.Cancel = true;
                stop.Set();
            };

            while (!stop.IsSet && Environment.TickCount64 < timeoutAtMs)
            {
                long now = clock.NowTicks;
                operatorEndpoint.SubmitCommand(
                    new Pose(targetPosition, Quaternion.Identity), Vector3.Zero, Vector3.Zero, a.Gripper, now);

                while (operatorEndpoint.TryReceiveState(clock.NowTicks, out _))
                {
                    acceptedRoundTrips++;
                    everObservedAnyState = true;
                    lastObservedPosition = operatorEndpoint.EstimateRobotState(clock.NowTicks).Position;

                    float distanceToTarget = Vector3.Distance(lastObservedPosition, targetPosition);
                    consecutiveInTolerance = distanceToTarget <= a.PositionToleranceMeters
                        ? consecutiveInTolerance + 1
                        : 0;
                }

                if (consecutiveInTolerance >= ConsecutiveInToleranceSamplesRequired)
                {
                    Console.WriteLine(
                        $"[move-arm] converged -- robot reports {lastObservedPosition}, within " +
                        $"{a.PositionToleranceMeters * 1000:0.#}mm of the target, for " +
                        $"{consecutiveInTolerance} consecutive samples ({acceptedRoundTrips} round " +
                        "trip(s) total). The arm should now be holding there.");
                    return 0;
                }

                Thread.Sleep((int)stepIntervalMs);
            }

            if (!everObservedAnyState)
            {
                Console.Error.WriteLine(
                    "[move-arm] FAILED -- no reply ever received from the robot. Is Teleop.RobotHost " +
                    "actually running and reachable at the given --remote-host/--remote-port?");
                return 1;
            }

            Console.Error.WriteLine(
                $"[move-arm] FAILED -- timed out after {a.TimeoutSeconds:0.#}s without converging. " +
                $"Last reported position was {lastObservedPosition}, " +
                $"{Vector3.Distance(lastObservedPosition, targetPosition) * 1000:0.#}mm from the target " +
                $"(tolerance {a.PositionToleranceMeters * 1000:0.#}mm). The arm may be partway there, " +
                "the target may be outside the arm's reach (FourDofArmKinematics clamps rather than " +
                "failing, so an unreachable target silently becomes the nearest reachable point), or " +
                "the run just needs more time -- try again with a larger --timeout-seconds.");
            return 1;
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
