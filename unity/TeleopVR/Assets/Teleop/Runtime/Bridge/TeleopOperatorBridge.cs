using System;
using Teleop.Core.Contracts;
using Teleop.Core.Pipeline;
using Teleop.Core.Registry;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;
using UnityEngine;
using CoreVec = System.Numerics.Vector3;
using CorePose = Teleop.Core.Types.Pose;

namespace Teleop.Bridge
{
    /// <summary>
    /// The operator side of the Phase 4 loopback baseline (docs/setup.md), attached to
    /// <c>XR Origin</c>. Owns the whole operator-side Core stack -- codec, both transports,
    /// <c>ClockSync</c>, the zero-mitigation predictor/reconciler pair, and
    /// <see cref="OperatorEndpoint"/> itself -- and drives it per Teleop/CLAUDE.md's
    /// callback-placement table:
    ///
    /// <list type="bullet">
    /// <item><c>Update</c>: capture <see cref="poseSource"/>'s pose, <c>SubmitCommand</c>; drain
    /// the downlink via <c>TryReceiveState</c>. The "network thread: TryReceive, stamp arrival"
    /// row collapses into here for this phase -- both transports are in-process
    /// (<see cref="LoopbackTransport"/>), so there is no real socket to hand a thread for. A real
    /// <c>UdpTransport</c> would reintroduce that row; this is a deliberate simplification, not an
    /// oversight (docs/adr/0003-display-offset-calibration.md's sibling plan explains why).</item>
    /// <item><c>Application.onBeforeRender</c>: <c>EstimateRobotState</c>, write the ghost robot's
    /// Transform, and -- if a round trip completed since the last render -- stamp
    /// <c>t_render</c>/<c>t_photon</c> and record the finished trace. This is the last hook before
    /// rendering, so the estimate and the M2P figure it produces are as fresh as possible.</item>
    /// </list>
    ///
    /// Zero mitigation, by construction: <c>none</c> + <c>snap</c> resolved through
    /// <see cref="Registries"/>, matching Gate 5's own baseline row and docs/setup.md's "zero
    /// mitigation" description of this phase.
    /// </summary>
    public sealed class TeleopOperatorBridge : MonoBehaviour
    {
        [SerializeField] private Transform poseSource;
        [SerializeField] private Transform ghostRobotTarget;

        private UnityMonotonicClock _clock;
        private DisplayCalibrationConfig _displayConfig;
        private XrDisplayTimeProvider _displayTimeProvider;

        private ITransport _uplinkTransport;
        private ITransport _downlinkTransport;
        private ClockSync _clockSync;
        private IPredictor<CorePose> _predictor;
        private IReconciler<CorePose> _reconciler;
        private OperatorEndpoint _operatorEndpoint;
        private UnityMetricSink _metricSink;

        private LatencyTrace _pendingTraceForRender;
        private bool _hasPendingTraceForRender;

        /// <summary>The uplink channel, shared with <see cref="TeleopRobotBridge"/> so the two sides of the loopback talk to each other.</summary>
        public ITransport UplinkTransport => _uplinkTransport;

        /// <summary>The downlink channel, shared with <see cref="TeleopRobotBridge"/>.</summary>
        public ITransport DownlinkTransport => _downlinkTransport;

        /// <summary>The live metric sink, for <see cref="LatencyHud"/> to read the latest samples from.</summary>
        public UnityMetricSink MetricSink => _metricSink;

        private void Awake()
        {
            _clock = new UnityMonotonicClock();
            _displayConfig = ConfigLoader.Load();
            _displayTimeProvider = new XrDisplayTimeProvider();

            string tlogPath = System.IO.Path.Combine(
                Application.persistentDataPath,
                $"phase4-session-{DateTime.UtcNow:yyyyMMdd-HHmmss}.tlog");
            _metricSink = new UnityMetricSink(
                hudCapacity: 256,
                tlogPath: tlogPath,
                ticksPerSecond: _clock.TicksPerSecond,
                sessionId: unchecked((ulong)DateTime.UtcNow.Ticks));

            _uplinkTransport = new LoopbackTransport(RawPoseCodec.EncodedSize, capacity: 64);
            _downlinkTransport = new LoopbackTransport(RobotStateFrameCodec.EncodedSize, capacity: 64);

            var clockSyncConfig = new ClockSyncConfig(
                historyCapacity: 32,
                smoothingAlpha: 0.2f,
                maxAcceptableRttTicks: _clock.TicksPerSecond * 2,
                outlierRttMultiple: 3.0,
                minSamplesBeforeTrusted: 3);
            _clockSync = new ClockSync(clockSyncConfig);

            // "none" + "snap": zero mitigation, per docs/setup.md's description of this phase and
            // Gate 5's required baseline row. Values below are unused by either -- see
            // PassthroughPredictor's and SnapReconciler's own docs for exactly which fields they
            // read -- and exist only for registry-factory signature uniformity.
            var predictorConfig = new PredictorConfig(
                maxHorizonTicks: MillisecondsToTicks(500),
                maxObservationGapTicks: MillisecondsToTicks(1000),
                historyCapacity: 2,
                smoothingAlpha: 0f,
                smoothingBeta: 0f,
                processNoise: 0f,
                measurementNoise: 0f,
                maxLinearSpeed: 10f,
                maxAngularSpeed: 10f);
            var reconcilerConfig = new ReconcilerConfig(
                convergencePositionToleranceMeters: 0.005f,
                convergenceOrientationToleranceRadians: 0.017f,
                maxTimeToConvergenceTicks: 0,
                maxCorrectionLinearSpeedMetersPerSecond: 0f,
                maxCorrectionAngularSpeedRadPerSecond: 0f,
                rollbackHistoryCapacity: 0);

            _predictor = Registries.Predictors["none"](predictorConfig, _clock);
            _reconciler = Registries.Reconcilers["snap"](reconcilerConfig, _metricSink, _clock);

            _operatorEndpoint = new OperatorEndpoint(
                new RawPoseCodec(),
                new RobotStateFrameCodec(),
                _uplinkTransport,
                _downlinkTransport,
                _clock,
                _metricSink,
                _clockSync,
                _predictor,
                _reconciler,
                inFlightCapacity: 64);
        }

        private void OnEnable() => Application.onBeforeRender += HandleBeforeRender;

        private void OnDisable() => Application.onBeforeRender -= HandleBeforeRender;

        private void Update()
        {
            long now = _clock.NowTicks;

            CorePose capturedPose = poseSource.ToCorePose();
            // No velocity sensor: this is fine specifically because the in-process loopback never
            // gaps (every Update tick delivers), so RigidBodyPlant's coast-on-velocity gap policy
            // never actually engages. A real transport reintroduces gaps and would need a real
            // velocity estimate here.
            _operatorEndpoint.SubmitCommand(capturedPose, CoreVec.Zero, CoreVec.Zero, gripper: 0f, now);

            while (_operatorEndpoint.TryReceiveState(now, out LatencyTrace completedTrace))
            {
                _pendingTraceForRender = completedTrace;
                _hasPendingTraceForRender = true;
            }
        }

        private void HandleBeforeRender()
        {
            long now = _clock.NowTicks;

            CorePose estimate = _operatorEndpoint.EstimateRobotState(now);
            if (ghostRobotTarget != null)
            {
                estimate.ApplyTo(ghostRobotTarget);
            }

            if (!_hasPendingTraceForRender)
            {
                return;
            }

            _hasPendingTraceForRender = false;

            long displayOffsetTicks = _displayTimeProvider.GetDisplayOffsetTicks(_clock, _displayConfig);
            LatencyTrace finalTrace = _pendingTraceForRender
                .WithRenderTicks(now)
                .WithPhotonTicks(now + displayOffsetTicks);

            if (finalTrace.TryGetCaptureTicks(out long captureTicks) &&
                finalTrace.TryGetPhotonTicks(out long photonTicks))
            {
                double m2pMs = (photonTicks - captureTicks) * 1000.0 / _clock.TicksPerSecond;
                _metricSink.Record("m2p_ms", m2pMs, now);
            }

            _metricSink.WriteLatencyTrace(finalTrace);
        }

        private void OnDestroy() => _metricSink?.Dispose();

        private long MillisecondsToTicks(double ms) => (long)(ms / 1000.0 * _clock.TicksPerSecond);
    }
}
