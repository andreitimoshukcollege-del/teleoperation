using System;
using System.Net;
using Teleop.Core.Contracts;
using Teleop.Core.Pipeline;
using Teleop.Core.Registry;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;
using Teleop.JetRover.Kinematics;
using Teleop.JetRover.Wire;
using UnityEngine;
using CoreVec = System.Numerics.Vector3;
using CorePose = Teleop.Core.Types.Pose;

namespace Teleop.Bridge
{
    /// <summary>
    /// Drives the real JetRover arm from a draggable Unity <see cref="Transform"/>
    /// (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md). Modeled on
    /// <c>core/Teleop.Eval/MoveArm/MoveArmCommand.cs</c>'s real-hardware wiring, not
    /// <see cref="TeleopOperatorBridge"/>'s in-process loopback -- this talks over a real
    /// <see cref="UdpTransport"/> to an already-running <c>Teleop.RobotHost</c>.
    ///
    /// <b>Two separate command paths, on purpose:</b>
    /// <list type="bullet">
    /// <item>A Cartesian <see cref="OperatorEndpoint"/> connection (same shape as
    /// <see cref="TeleopOperatorBridge"/>'s) whose only job is prediction/reconciliation/
    /// <c>ClockSync</c> bookkeeping and latency recording -- this is where a chosen
    /// predictor/reconciler/network profile becomes visible in recorded metrics, exactly like
    /// <see cref="TeleopOperatorBridge"/>'s ghost robot shows reconciliation against its own
    /// simulated robot. <b>This connection's command is never what drives the real arm, and its
    /// <c>EstimateRobotState</c> is never what drives <see cref="armRig"/></b> -- see below.</item>
    /// <item>A separate, plain <c>UdpTransport</c>-backed channel (<c>_jointTransport</c>) that
    /// sends already-computed joint angles (<see cref="JointCommandFrame"/>) built
    /// from the drag target's <i>raw, unreconciled</i> pose, rate-limited to
    /// <see cref="JetRoverArmConfig.CommandRateHz"/>. This is the one that actually moves the real
    /// robot, mirroring how <see cref="TeleopOperatorBridge.Update"/> also always sends the raw
    /// captured pose, never a reconciled one -- reconciliation smooths the operator's *estimate of
    /// the robot*, it was never meant to smooth the operator's own outgoing intent.</item>
    /// </list>
    ///
    /// <b><see cref="armRig"/> is driven directly from the same raw drag-target pose every
    /// <c>Update()</c>, unconditionally -- not from <c>EstimateRobotState</c>, and not gated on
    /// <see cref="JetRoverArmConfig.ConfirmHardwareMotion"/>.</b> A real, found-in-practice bug
    /// (2026-08-13): an earlier version drove the rig from <c>_operatorEndpoint.EstimateRobotState</c>
    /// in <see cref="HandleBeforeRender"/>, which requires a live, replying <c>Teleop.RobotHost</c>
    /// to produce anything meaningful -- contradicting <see cref="JetRoverArmRig"/>'s own doc
    /// ("not re-derived from a round-tripped pose estimate") and making the rig, and therefore the
    /// kinematics/axis-sign calibration work it exists for, impossible to exercise without a
    /// connected robot. Fixed by computing angles from the drag target directly in
    /// <c>Update()</c> and applying them to the rig immediately, every frame -- this is a
    /// deliberate exception to Teleop/CLAUDE.md's "state estimation belongs in
    /// <c>onBeforeRender</c>" rule: that rule is about minimizing staleness when estimating the
    /// *remote robot's* state, which does not apply here, since this rig displays the operator's
    /// own already-known local intent, not a remote estimate.
    ///
    /// <b>Known limitation, not engineered around this pass:</b> only the Cartesian connection is
    /// wrapped in <see cref="EmulatedTransport"/> when <see cref="JetRoverArmConfig.NetworkProfileName"/>
    /// is set. <see cref="EmulatedTransport"/> applies delay/jitter/loss on the *receiving* side
    /// (Transport/CLAUDE.md), so impairing the joint channel for real would mean wrapping
    /// <c>Teleop.RobotHost</c>'s own receiving transport -- a robot-side change, out of scope
    /// here. The configured network profile currently affects only recorded latency metrics on
    /// the Cartesian connection, not what conditions the real joint commands actually travel
    /// under, and (per the fix above) never affects the rig's visualization either.
    /// </summary>
    public sealed class JetRoverOperatorBridge : MonoBehaviour
    {
        private const int MaxDatagramBytes = 128;

        [SerializeField] private Transform dragTarget;
        [SerializeField] private JetRoverArmRig armRig;

        private JetRoverArmConfig _config;
        private UnityMonotonicClock _clock;

        private UdpTransport _cartesianUdp;
        private UdpTransport _jointTransport;
        private ITransport _cartesianTransport;

        private ClockSync _clockSync;
        private IPredictor<CorePose> _predictor;
        private IReconciler<CorePose> _reconciler;
        private OperatorEndpoint _operatorEndpoint;
        private UnityMetricSink _metricSink;

        private uint _jointSequence;
        private long _lastJointSendTicks;
        private bool _hasSentJointCommand;
        private long _sendIntervalTicks;

        private LatencyTrace _pendingTrace;
        private bool _hasPendingTrace;

        /// <summary>True once at least one downlink reply has ever been received -- a simple, cheap signal that Teleop.RobotHost is actually reachable, mirroring MoveArmCommand's own everObservedAnyState diagnostic.</summary>
        public bool HasReceivedAnyState { get; private set; }

        private void Awake()
        {
            _config = ConfigLoader.Load("jetrover_connection", "jetrover_connection.json", new JetRoverArmConfig());
            _clock = new UnityMonotonicClock();
            _sendIntervalTicks = (long)(_clock.TicksPerSecond / _config.CommandRateHz);

            IPAddress remoteHost = IPAddress.Parse(_config.RemoteHost);

            _cartesianUdp = new UdpTransport(
                _config.LocalPort, new IPEndPoint(remoteHost, _config.RemotePort), MaxDatagramBytes, _clock);
            _cartesianTransport = _cartesianUdp;

            if (!string.IsNullOrEmpty(_config.NetworkProfileName))
            {
                if (Teleop.Core.Transport.NetworkProfileCatalog.TryResolveParametric(
                    _config.NetworkProfileName, _clock.TicksPerSecond, out NetworkProfile profile, out string? profileError))
                {
                    _cartesianTransport = new EmulatedTransport(
                        _cartesianUdp, profile, new SeededRng(unchecked((ulong)DateTime.UtcNow.Ticks)), maxInFlight: 64);
                }
                else
                {
                    throw new InvalidOperationException(
                        $"JetRoverOperatorBridge: NetworkProfileName '{_config.NetworkProfileName}' did not resolve: {profileError}");
                }
            }

            _jointTransport = new UdpTransport(
                _config.JointLocalPort, new IPEndPoint(remoteHost, _config.JointRemotePort), MaxDatagramBytes, _clock);

            if (!Registries.Predictors.TryGetValue(_config.PredictorName, out var predictorFactory))
            {
                throw new InvalidOperationException($"JetRoverOperatorBridge: unknown PredictorName '{_config.PredictorName}'.");
            }

            if (!Registries.Reconcilers.TryGetValue(_config.ReconcilerName, out var reconcilerFactory))
            {
                throw new InvalidOperationException($"JetRoverOperatorBridge: unknown ReconcilerName '{_config.ReconcilerName}'.");
            }

            string tlogPath = System.IO.Path.Combine(
                Application.persistentDataPath, $"jetrover-session-{DateTime.UtcNow:yyyyMMdd-HHmmss}.tlog");
            _metricSink = new UnityMetricSink(
                hudCapacity: 256, tlogPath: tlogPath, ticksPerSecond: _clock.TicksPerSecond,
                sessionId: unchecked((ulong)DateTime.UtcNow.Ticks));

            var predictorConfig = new PredictorConfig(
                maxHorizonTicks: MillisecondsToTicks(_config.MaxHorizonMs),
                maxObservationGapTicks: MillisecondsToTicks(_config.MaxObservationGapMs),
                historyCapacity: _config.PredictorHistoryCapacity,
                smoothingAlpha: _config.SmoothingAlpha,
                smoothingBeta: _config.SmoothingBeta,
                processNoise: _config.ProcessNoise,
                measurementNoise: _config.MeasurementNoise,
                maxLinearSpeed: _config.MaxLinearSpeed,
                maxAngularSpeed: _config.MaxAngularSpeed);
            var reconcilerConfig = new ReconcilerConfig(
                convergencePositionToleranceMeters: _config.ConvergencePositionToleranceMeters,
                convergenceOrientationToleranceRadians: _config.ConvergenceOrientationToleranceRadians,
                maxTimeToConvergenceTicks: MillisecondsToTicks(_config.MaxTimeToConvergenceMs),
                maxCorrectionLinearSpeedMetersPerSecond: _config.MaxCorrectionLinearSpeedMetersPerSecond,
                maxCorrectionAngularSpeedRadPerSecond: _config.MaxCorrectionAngularSpeedRadPerSecond,
                rollbackHistoryCapacity: _config.RollbackHistoryCapacity);
            var clockSyncConfig = new ClockSyncConfig(
                historyCapacity: _config.ClockSyncHistoryCapacity,
                smoothingAlpha: _config.ClockSyncSmoothingAlpha,
                maxAcceptableRttTicks: MillisecondsToTicks(_config.MaxAcceptableRttMs),
                outlierRttMultiple: _config.OutlierRttMultiple,
                minSamplesBeforeTrusted: _config.MinSamplesBeforeTrusted);

            _clockSync = new ClockSync(clockSyncConfig);
            _predictor = predictorFactory(predictorConfig, _clock);
            _reconciler = reconcilerFactory(reconcilerConfig, _metricSink, _clock);

            _operatorEndpoint = new OperatorEndpoint(
                new RawPoseCodec(), new RobotStateFrameCodec(), _cartesianTransport, _cartesianTransport,
                _clock, _metricSink, _clockSync, _predictor, _reconciler, inFlightCapacity: 32);
        }

        private void OnEnable() => Application.onBeforeRender += HandleBeforeRender;

        private void OnDisable() => Application.onBeforeRender -= HandleBeforeRender;

        private void Update()
        {
            long now = _clock.NowTicks;

            while (_operatorEndpoint.TryReceiveState(now, out LatencyTrace completedTrace))
            {
                HasReceivedAnyState = true;
                _pendingTrace = completedTrace;
                _hasPendingTrace = true;
            }

            if (dragTarget == null)
            {
                return;
            }

            // Expressed relative to the arm's own base (armRig's transform), NOT world space:
            // FourDofArmKinematics -- and the real robot -- assume the origin is the arm's own
            // base-yaw axis. dragTarget.ToCorePose() would hand over dragTarget's raw world
            // position, which is only correct if BaseYawPivot happens to sit at the world origin.
            // It generally won't (you position it wherever is convenient to reach in the scene),
            // so skipping this conversion would aim the arm at a position offset by wherever
            // BaseYawPivot actually is -- silently wrong, not obviously broken, since
            // MaxDirectionMagnitude's per-step clamp would still make it move slowly *toward*
            // the wrong target rather than jumping there.
            Transform armBase = armRig != null ? armRig.transform : dragTarget;
            Vector3 localPosition = armBase.InverseTransformPoint(dragTarget.position);
            Quaternion localRotation = Quaternion.Inverse(armBase.rotation) * dragTarget.rotation;
            CorePose capturedPose = new CorePose(localPosition.ToCore(), localRotation.ToCore());

            FourDofArmKinematics.TryInverse(
                ArmLinkLengths.Measured, capturedPose.Position,
                out float baseYaw, out float lowerPitch, out float middlePitch, out bool wasClamped);
            float desiredPitch = FourDofArmKinematics.ExtractPitchRadians(capturedPose.Rotation);
            float upperPitch = FourDofArmKinematics.InverseUpperPitch(lowerPitch, middlePitch, desiredPitch);

            // Applied every frame, unconditionally -- see this class's own doc for why this is
            // deliberately not gated on ConfirmHardwareMotion or rate-limited: the rig needs to
            // be exercisable (and its axis signs calibratable) with no robot connected at all.
            if (armRig != null)
            {
                armRig.ApplyAngles(baseYaw, lowerPitch, middlePitch, upperPitch, wasClamped);
            }

            if (!_config.ConfirmHardwareMotion)
            {
                return;
            }

            if (_hasSentJointCommand && now - _lastJointSendTicks < _sendIntervalTicks)
            {
                return;
            }

            _hasSentJointCommand = true;
            _lastJointSendTicks = now;

            // Raw, unreconciled target -- SubmitCommand feeds ClockSync/OWD accounting on the
            // Cartesian connection (recorded metrics only -- see this class's own doc), matching
            // TeleopOperatorBridge.Update's own "always send the raw captured pose" precedent.
            _operatorEndpoint.SubmitCommand(capturedPose, CoreVec.Zero, CoreVec.Zero, gripper: 0f, now);

            var jointFrame = new JointCommandFrame(
                _jointSequence++, now, baseYaw, lowerPitch, middlePitch, upperPitch, gripper: 0f);
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize];
            if (JointCommandCodec.TryEncode(jointFrame, buffer, out int bytesWritten))
            {
                _jointTransport.Send(buffer.Slice(0, bytesWritten), now);
            }
        }

        private void HandleBeforeRender()
        {
            if (!_hasPendingTrace)
            {
                return;
            }

            _hasPendingTrace = false;
            // No display-offset/photon stamping here, unlike TeleopOperatorBridge -- this tool
            // isn't measuring M2P against a headset's photons, just recording OWD/reconciliation
            // samples for analysis/. WithRenderTicks alone is an honest simplification, not a
            // claim of full M2P fidelity.
            _metricSink.WriteLatencyTrace(_pendingTrace.WithRenderTicks(_clock.NowTicks));
        }

        private void OnDestroy()
        {
            _cartesianUdp?.Dispose();
            _jointTransport?.Dispose();
            _metricSink?.Dispose();
        }

        private long MillisecondsToTicks(double ms) => (long)(ms / 1000.0 * _clock.TicksPerSecond);
    }
}
