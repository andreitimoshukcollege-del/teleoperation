using System;
using System.Net;
using Teleop.Core.Contracts;
using Teleop.Core.Pipeline;
using Teleop.Core.Registry;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
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
    /// simulated robot. <b>Its <c>EstimateRobotState</c> is never what drives <see cref="armRig"/></b>
    /// -- see below. Its command does reach the robot: <c>Teleop.RobotHost</c> runs its own IK on
    /// every Cartesian frame and drives the same plant, which tracks the two paths' staleness
    /// separately (docs/adr/0009). Both carry the same raw pose and <see cref="GripperFraction"/>,
    /// so they agree.</item>
    /// <item>A separate, plain <c>UdpTransport</c>-backed channel (<c>_jointTransport</c>) that
    /// sends already-computed joint targets (<see cref="JointCommandCodec"/>, one
    /// <see cref="JointTarget"/> per <see cref="RobotArmProfile"/> joint) built
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

        [Tooltip("When the IK cannot reach the target exactly (blocked by the robot's body, out of reach, or the pitch tilted to keep clear), move the drag target to what the arm does reach. TeleopXR turns it on, so the marker stays on the arm's wrist and the clutch re-engages from there (docs/adr/0020). Off by default, so a grabbed target (TeleopVR) is never pulled out of the hand.")]
        [SerializeField] private bool snapTargetToReachable;
        [Tooltip("Whether commands reach the robot from the start. Off means nothing is sent until something sets MotionEnabled (TeleopXR's motion button, docs/adr/0017). TeleopVR leaves it on, as it always behaved.")]
        [SerializeField] private bool motionEnabledAtStart = true;

        private JetRoverArmConfig _config;
        private RobotArmProfile _profile;
        private UnityMonotonicClock _clock;

        private UdpTransport _cartesianUdp;
        private UdpTransport _jointUdp;
        private JointLinkDisturbance _jointTransport;
        private ulong _disturbanceSeed;
        private ITransport _cartesianTransport;

        private ClockSync _clockSync;
        private IPredictor<CorePose> _predictor;
        private IReconciler<CorePose> _reconciler;
        private IPlayoutPolicy<CorePose> _playoutPolicy;
        private OperatorEndpoint _operatorEndpoint;
        private UnityMetricSink _metricSink;

        private uint _jointSequence;
        private JetRoverJointStateLink _jointStateLink;
        private long _lastJointSendTicks;
        private bool _hasSentJointCommand;
        private ArmSolver _solver;
        private bool _solverSeededFromRobot;
        private long _sendIntervalTicks;

        private LatencyTrace _pendingTrace;
        private bool _hasPendingTrace;

        /// <summary>
        /// How long since the last received downlink reply before <see cref="Status"/> reports
        /// <see cref="ConnectionStatus.Stale"/> instead of <see cref="ConnectionStatus.Connected"/>
        /// -- generous headroom above the current ~20-48Hz <see cref="JetRoverArmConfig.CommandRateHz"/>
        /// range while still surfacing a real drop (a Teleop.RobotHost restart, a Jetson reboot,
        /// a network partition) within about a second, not many seconds later.
        /// </summary>
        private const float StaleAfterSeconds = 1f;

        private long _lastStateReceivedTicks;

        private float _gripperFraction;

        /// <summary>
        /// The gripper command, 0 = open to 1 = closed, sent with every command on both paths.
        /// Set by an input component (TeleopXR's off-hand pinch, docs/adr/0017); nothing sets it
        /// in TeleopVR, so it stays 0 there, which is what this bridge always sent. Clamped; NaN is
        /// ignored, so an untracked hand can never turn into "open".
        /// </summary>
        public float GripperFraction
        {
            get => _gripperFraction;
            set
            {
                if (!float.IsNaN(value))
                {
                    _gripperFraction = Math.Clamp(value, 0f, 1f);
                }
            }
        }

        private JetRoverMotionMode _motionMode = JetRoverMotionMode.Off;
        private bool _motionModeSet;

        /// <summary>
        /// The operator's motion mode (docs/adr/0023 decision 5), on top of the config's
        /// <c>ConfirmHardwareMotion</c>. The virtual arm keeps following in every mode.
        /// <list type="bullet">
        /// <item><see cref="JetRoverMotionMode.On"/>: joint commands go while the robot holds. If it
        /// reports parking or relaxed, a hold request goes first (repeated, a few a second) and no joint
        /// command until a status says holding.</item>
        /// <item><see cref="JetRoverMotionMode.Paused"/>: no commands; the robot holds its last one with
        /// torque on, or stays relaxed if it was: pausing never powers the servos.</item>
        /// <item><see cref="JetRoverMotionMode.Off"/>: no commands, and a relax request (park slowly at
        /// the rest pose, then torque off), repeated until the status says parking or relaxed, or refused
        /// (<see cref="JetRoverJointStateLink.ArmPowerReasonText"/>). Setting Off again retries.</item>
        /// </list>
        /// At launch the mode is Off with nothing requested (the arm may already be resting), or On when
        /// the scene's <c>motionEnabledAtStart</c> is set (TeleopVR).
        /// </summary>
        public JetRoverMotionMode MotionMode
        {
            get => _motionMode;
            set
            {
                _motionMode = value;
                _motionModeSet = true;
                _jointStateLink?.SetMotionMode(value, NowTicks);
            }
        }

        /// <summary>
        /// The two-way switch the motion button has always used: true is <see cref="JetRoverMotionMode.On"/>,
        /// false is <see cref="JetRoverMotionMode.Paused"/> (commands stop, the torque stays on). Reads true
        /// only in On.
        /// </summary>
        public bool MotionEnabled
        {
            get => _motionMode == JetRoverMotionMode.On;
            set => MotionMode = value ? JetRoverMotionMode.On : JetRoverMotionMode.Paused;
        }

        /// <summary>
        /// The mode is On but joint commands are held back: a hold request is waiting for the robot to say
        /// it holds again (docs/adr/0023 decision 5).
        /// </summary>
        public bool WaitingForHold => _motionMode == JetRoverMotionMode.On && _jointStateLink != null && !_jointStateLink.MayCommand(NowTicks);

        /// <summary>True when the config lets this bridge command the robot at all (<c>ConfirmHardwareMotion</c>).</summary>
        public bool HardwareMotionAllowed => _config != null && _config.ConfirmHardwareMotion;

        /// <summary>
        /// The joint-state downlink (docs/adr/0018): the robot's measured joints and the predictor's
        /// estimate of them now. Null until <c>Awake</c>.
        /// </summary>
        public JetRoverJointStateLink JointState => _jointStateLink;

        /// <summary>
        /// The network disturbance on the joint socket (docs/adr/0023 decision 3): whether one is on,
        /// what it is, and its seed. Null until <c>Awake</c>. Set it with <see cref="TryApplyDisturbance"/>
        /// or <see cref="ApplyCustomDisturbance"/>.
        /// </summary>
        public JointLinkDisturbance JointDisturbance => _jointTransport;

        /// <summary>
        /// Disturbs the joint socket in both directions with a named profile from Core's
        /// <c>NetworkProfileCatalog</c> (<c>lan</c> or empty for none, <c>50ms-5j</c>,
        /// <c>150ms-20j-0.5loss</c>, <c>300ms-60j-2loss-bursty</c>, <c>delay-500ms</c>, <c>loss-10pct</c>,
        /// ...). Each direction gets the profile's impairments on its own substream. False, with the
        /// reason, when the name does not resolve or exceeds the config's caps.
        /// </summary>
        public bool TryApplyDisturbance(string profileName, out string error)
        {
            error = null;
            if (_jointTransport == null) { error = "not started"; return false; }
            if (string.IsNullOrEmpty(profileName) || profileName == "lan" || profileName == "none")
            {
                _jointTransport.Clear();
                Debug.Log("[network] disturbance off");
                return true;
            }

            if (!Teleop.Core.Transport.NetworkProfileCatalog.TryResolveImpairments(profileName, _clock.TicksPerSecond,
                    out INetworkImpairment[] up, out error)
                || !Teleop.Core.Transport.NetworkProfileCatalog.TryResolveImpairments(profileName, _clock.TicksPerSecond,
                    out INetworkImpairment[] down, out error))
            {
                return false;
            }

            if (Teleop.Core.Transport.NetworkProfileCatalog.TryResolveParametric(profileName, _clock.TicksPerSecond, out NetworkProfile profile, out _)
                && ((profile.BaseDelayTicks + profile.JitterTicks) * 1000.0 / _clock.TicksPerSecond > _config.DisturbanceMaxDelayMs
                    || profile.LossProbabilityAfterDelivered * 100.0 > _config.DisturbanceMaxLossPercent))
            {
                error = $"'{profileName}' exceeds the caps ({_config.DisturbanceMaxDelayMs} ms, {_config.DisturbanceMaxLossPercent}% loss)";
                return false;
            }

            ApplyDisturbance(up, down, profileName + " both ways");
            return true;
        }

        /// <summary>
        /// Disturbs the joint socket in both directions with a one-way delay, a uniform jitter and a
        /// random loss, clamped to the config's caps; all zero switches it off.
        /// </summary>
        public void ApplyCustomDisturbance(float delayMs, float jitterMs, float lossPercent)
        {
            if (_jointTransport == null) return;
            delayMs = Math.Min(Math.Max(delayMs, 0f), _config.DisturbanceMaxDelayMs);
            jitterMs = Math.Min(Math.Max(jitterMs, 0f), Math.Max(0f, _config.DisturbanceMaxDelayMs - delayMs));
            lossPercent = Math.Min(Math.Max(lossPercent, 0f), _config.DisturbanceMaxLossPercent);
            if (delayMs <= 0f && jitterMs <= 0f && lossPercent <= 0f)
            {
                _jointTransport.Clear();
                Debug.Log("[network] disturbance off");
                return;
            }

            var settings = new NetworkImpairmentSettings
            {
                EnableDelay = delayMs > 0f, BaseDelayMs = delayMs,
                EnableJitter = jitterMs > 0f, JitterMs = jitterMs,
                EnableLoss = lossPercent > 0f, LossPercent = lossPercent,
            };
            ApplyDisturbance(settings.CreateImpairments(_clock.TicksPerSecond, null), settings.CreateImpairments(_clock.TicksPerSecond, null),
                $"{delayMs:0} ms ± {jitterMs:0} ms, {lossPercent:0.#}% loss both ways");
        }

        private void ApplyDisturbance(INetworkImpairment[] up, INetworkImpairment[] down, string description)
        {
            _disturbanceSeed += 2;
            _jointTransport.Set(up, down, _disturbanceSeed, description);
            // Every change is logged with its seed and time: a run with a disturbance on is not a
            // real-network measurement, and its draws can be reproduced from this line.
            Debug.Log($"[network] disturbance: {description}, seed {_disturbanceSeed}, at {_clock.NowTicks} ticks");
        }

        /// <summary>How the last IK solve related to the target: exact, pitch tilted, or blocked by the robot's body (docs/adr/0020).</summary>
        public ArmSolveStatus LastSolveStatus { get; private set; }

        private float _speedRadPerSec;

        /// <summary>
        /// The joint speed (rad/s) asked of the robot with every joint command, or 0 for the robot's
        /// own default, which is what this bridge always sent. RobotHost caps it at its deployed
        /// <c>--pulses-per-second</c>; the joint predictor replays commands at the same speed. Set by
        /// TeleopXR's speed slider; TeleopVR leaves it at 0.
        /// </summary>
        public float SpeedRadPerSec
        {
            get => _speedRadPerSec;
            set => _speedRadPerSec = float.IsNaN(value) ? 0f : Math.Max(0f, value);
        }

        private JetRoverTrainingCourse _training;

        /// <summary>
        /// The training course over the robot's workspace (docs/adr/0024), or null without an arm rig or
        /// a collision model. While it runs it sets the speed every joint command carries, in place of
        /// <see cref="SpeedRadPerSec"/>.
        /// </summary>
        public JetRoverTrainingCourse Training => _training;

        /// <summary>This bridge's clock, for asking <see cref="JointState"/> for a prediction "now".</summary>
        public long NowTicks => _clock != null ? _clock.NowTicks : 0;

        /// <summary>The wrist-roll angle (radians) in the current target, as sent to the gripper's rotation motor.</summary>
        public float WristRollRadians { get; private set; }

        /// <summary>The loaded connection config, for input components that read its operator-input fields.</summary>
        public JetRoverArmConfig Config => _config;

        private JetRoverClutch _clutch;

        /// <summary>
        /// Clutch control over this bridge's drag target (docs/adr/0017), created on first use after
        /// <c>Awake</c> from the config's <c>ClutchScale</c> and <c>ClutchMinHeightMeters</c>. Null
        /// until the bridge is awake, or when the drag target or arm rig is not wired. An input
        /// component drives it; TeleopVR's controllers never touch it.
        /// </summary>
        public JetRoverClutch Clutch
        {
            get
            {
                if (_clutch == null && _config != null && dragTarget != null && armRig != null)
                {
                    _clutch = new JetRoverClutch(armRig.BaseAnchor, dragTarget, _profile, _config.ClutchScale, _config.ClutchMinHeightMeters);
                }

                return _clutch;
            }
        }

        /// <summary>True once at least one downlink reply has ever been received -- a simple, cheap signal that Teleop.RobotHost is actually reachable, mirroring MoveArmCommand's own everObservedAnyState diagnostic.</summary>
        public bool HasReceivedAnyState { get; private set; }

        /// <summary>
        /// Whether this bridge has ever heard back from the real robot, and if so, whether that
        /// contact is still fresh. There is no handshake or persistent "connection" to query here
        /// (UDP has none, and this class sends every tick unconditionally regardless of replies --
        /// see this class's own doc) -- this is a read of recency, not a live socket state.
        /// Recovery after a drop needs no code of its own: sending never stops, so once
        /// Teleop.RobotHost/the Jetson-side nodes come back, this flips back to
        /// <see cref="ConnectionStatus.Connected"/> on its own the next time a reply arrives.
        /// </summary>
        public enum ConnectionStatus
        {
            /// <summary>No downlink reply has ever been received since this bridge started.</summary>
            NeverConnected,

            /// <summary>A reply arrived within the last <see cref="StaleAfterSeconds"/>.</summary>
            Connected,

            /// <summary>At least one reply arrived once, but not within the last <see cref="StaleAfterSeconds"/>.</summary>
            Stale,
        }

        /// <summary>See <see cref="ConnectionStatus"/>'s own doc.</summary>
        public ConnectionStatus Status
        {
            get
            {
                if (!HasReceivedAnyState)
                {
                    return ConnectionStatus.NeverConnected;
                }

                float secondsSinceLastReply = (_clock.NowTicks - _lastStateReceivedTicks) / (float)_clock.TicksPerSecond;
                return secondsSinceLastReply < StaleAfterSeconds ? ConnectionStatus.Connected : ConnectionStatus.Stale;
            }
        }

        private void Awake()
        {
            _config = ConfigLoader.Load("jetrover_connection", "jetrover_connection.json", new JetRoverArmConfig());
            _profile = ConfigLoader.Load(
                "jetrover_arm_profile", "jetrover_arm_profile.json", new RobotArmProfileData()).ToProfile();
            _clock = new UnityMonotonicClock();
            _sendIntervalTicks = (long)(_clock.TicksPerSecond / _config.CommandRateHz);

            // Discovered at start (docs/adr/0022), so a new DHCP lease needs no config push.
            IPAddress remoteHost = RobotAddressDiscovery.Resolve(_config);

            _cartesianUdp = new UdpTransport(
                _config.LocalPort, new IPEndPoint(remoteHost, _config.RemotePort), MaxDatagramBytes, _clock);
            _cartesianTransport = _cartesianUdp;

            if (!string.IsNullOrEmpty(_config.NetworkProfileName))
            {
                // Resolved to an impairment set rather than a profile struct (docs/adr/0013); the
                // frozen names and their numbers are unchanged.
                if (Teleop.Core.Transport.NetworkProfileCatalog.TryResolveImpairments(
                    _config.NetworkProfileName, _clock.TicksPerSecond,
                    out Teleop.Core.Contracts.INetworkImpairment[] impairments, out string? profileError))
                {
                    _cartesianTransport = new EmulatedTransport(
                        _cartesianUdp, impairments,
                        unchecked((ulong)DateTime.UtcNow.Ticks), maxInFlight: 64);
                }
                else
                {
                    throw new InvalidOperationException(
                        $"JetRoverOperatorBridge: NetworkProfileName '{_config.NetworkProfileName}' did not resolve: {profileError}");
                }
            }

            _jointUdp = new UdpTransport(
                _config.JointLocalPort, new IPEndPoint(remoteHost, _config.JointRemotePort), MaxDatagramBytes, _clock);
            // Off until the console switches one on; commands, keepalives and frames all pass through.
            _jointTransport = new JointLinkDisturbance(_jointUdp);
            _disturbanceSeed = unchecked((ulong)DateTime.UtcNow.Ticks) & ~1UL;

            if (!Registries.Predictors.TryGetValue(_config.PredictorName, out var predictorFactory))
            {
                throw new InvalidOperationException($"JetRoverOperatorBridge: unknown PredictorName '{_config.PredictorName}'.");
            }

            if (!Registries.Reconcilers.TryGetValue(_config.ReconcilerName, out var reconcilerFactory))
            {
                throw new InvalidOperationException($"JetRoverOperatorBridge: unknown ReconcilerName '{_config.ReconcilerName}'.");
            }

            if (!Registries.PlayoutPolicies.TryGetValue(_config.PlayoutPolicyName, out var playoutFactory))
            {
                throw new InvalidOperationException($"JetRoverOperatorBridge: unknown PlayoutPolicyName '{_config.PlayoutPolicyName}'.");
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

            var playoutConfig = new PlayoutPolicyConfig(
                historyCapacity: _config.PlayoutHistoryCapacity,
                initialDelayBudgetTicks: MillisecondsToTicks(_config.PlayoutBudgetMs),
                minDelayBudgetTicks: MillisecondsToTicks(_config.PlayoutMinBudgetMs),
                maxDelayBudgetTicks: MillisecondsToTicks(_config.PlayoutMaxBudgetMs),
                targetPercentile: _config.PlayoutTargetPercentile,
                delayWindowSamples: _config.PlayoutDelayWindowSamples,
                delayProcessNoise: 0.01f,
                delayMeasurementNoise: 0.001f,
                maxAdaptationRatePerSecond: 0.0,
                lossWeight: 0.5);

            _clockSync = new ClockSync(clockSyncConfig);
            // The joint-state downlink rides the joint socket: the robot streams to whoever last sent
            // on that port, so the keepalive must come from the same socket as the commands.
            JointPredictorPresetData[] presets = JointPresets(_config, out byte[] readingAgeIds);
            try
            {
                _jointStateLink = new JetRoverJointStateLink(_jointTransport, _clock, _profile, clockSyncConfig, presets,
                    _config.JointPredictorActive, _config.MaxHorizonMs, _config.GripperFractionPerRadian, readingAgeIds,
                    LearnedModels.TryLoad);
            }
            catch (ArgumentException e)
            {
                // A mistyped predictor or a bad preset must not take the command path down with it.
                Debug.LogError($"[twin] {e.Message}; falling back to hold-last (no prediction)");
                _jointStateLink = new JetRoverJointStateLink(_jointTransport, _clock, _profile, clockSyncConfig,
                    new[] { new JointPredictorPresetData { Label = "hold-last", Predictor = "hold-last" } }, "hold-last",
                    _config.MaxHorizonMs, _config.GripperFractionPerRadian, new byte[0]);
            }

            if (armRig != null && _profile.Collision != null)
            {
                _training = new JetRoverTrainingCourse(_profile, _config.ClutchMinHeightMeters, armRig.BaseAnchor, _clock);
            }

            // Park, then relax (docs/adr/0023 decision 5). Requests only when the robot may be commanded at
            // all; the heartbeat goes either way. At launch: Off with nothing requested, unless the scene
            // starts with motion on (TeleopVR) or something set a mode before Awake.
            _jointStateLink.PowerRequestsAllowed = _config.ConfirmHardwareMotion;
            if (_motionModeSet) _jointStateLink.SetMotionMode(_motionMode, _clock.NowTicks);
            else if (motionEnabledAtStart) MotionMode = JetRoverMotionMode.On;
            _predictor = predictorFactory(predictorConfig, _clock);
            _reconciler = reconcilerFactory(reconcilerConfig, _metricSink, _clock);
            _playoutPolicy = playoutFactory(playoutConfig, _metricSink, _clock);

            _operatorEndpoint = new OperatorEndpoint(
                new RawPoseCodec(), new RobotStateFrameCodec(), _cartesianTransport, _cartesianTransport,
                _clock, _metricSink, _clockSync, _predictor, _reconciler, _playoutPolicy,
                inFlightCapacity: 32,
                // Five seconds against a measured 61-110ms Tailscale round trip
                // (robot/README.md), so this only reclaims slots whose replies the link lost.
                inFlightMaxAgeTicks: _clock.TicksPerSecond * 5);
        }

        private void OnEnable() => Application.onBeforeRender += HandleBeforeRender;

        private void OnDisable() => Application.onBeforeRender -= HandleBeforeRender;

        private void Update()
        {
            long now = _clock.NowTicks;
            _jointStateLink?.Tick(now);

            while (_operatorEndpoint.TryReceiveState(now, out _))
            {
                // Arrival only: ClockSync and owd_uplink_ms/owd_downlink_ms are recorded as a side
                // effect here. The trace this yields has no t_playout yet.
            }

            // The playout drain, and it is not optional (docs/adr/0012-playout-policy-wiring.md).
            // Nothing reaches the predictor or reconciler at arrival any more, so skipping this
            // freezes EstimateRobotState -- and the trace handed to WriteLatencyTrace below would
            // be missing t_playout, silently, in every recorded .tlog.
            while (_operatorEndpoint.TryPlayoutState(now, out LatencyTrace completedTrace))
            {
                HasReceivedAnyState = true;
                _lastStateReceivedTicks = now;
                _pendingTrace = completedTrace;
                _hasPendingTrace = true;
            }

            if (dragTarget == null)
            {
                return;
            }

            // Expressed relative to the arm's own stable base anchor, NOT world space:
            // ArmKinematics -- and the real robot -- assume the origin is the arm's own
            // base-yaw axis. dragTarget.ToCorePose() would hand over dragTarget's raw world
            // position, which is only correct if the base sits at the world origin. It generally
            // won't (you position it wherever is convenient to reach in the scene), so skipping
            // this conversion would aim the arm at a position offset by wherever the base
            // actually is -- silently wrong, not obviously broken, since MaxDirectionMagnitude's
            // per-step clamp would still make it move slowly *toward* the wrong target rather
            // than jumping there.
            //
            // Deliberately armRig.BaseAnchor, NOT armRig.transform: a real, found-in-practice bug
            // (2026-08-17) used armRig.transform directly here, which in the scene at the time was
            // the same Transform ApplyAngles rotates as the base-yaw pivot -- a self-referential
            // feedback loop where each frame's computed yaw altered the very frame the next
            // frame's yaw was computed relative to, oscillating the arm between two poses forever
            // even for a perfectly stationary target. BaseAnchor's own doc covers this in full;
            // it must be a Transform this class never rotates.
            Transform armBase = armRig != null ? armRig.BaseAnchor : dragTarget;
            Vector3 localPosition = armBase.InverseTransformPoint(dragTarget.position);
            Quaternion localRotation = Quaternion.Inverse(armBase.rotation) * dragTarget.rotation;
            CorePose capturedPose = new CorePose(localPosition.ToCore(), localRotation.ToCore());

            // Read near last frame's values, so a target past straight down keeps its pitch instead
            // of reading as the same orientation with the roll half a turn round (docs/adr/0020).
            float desiredPitch, desiredRoll;
            if (_hasTargetOrientation)
            {
                ArmKinematics.ExtractPitchRollNear(capturedPose.Rotation, _targetPitch, _targetRoll, out desiredPitch, out desiredRoll);
            }
            else
            {
                desiredPitch = ArmKinematics.ExtractPitchRadians(capturedPose.Rotation);
                desiredRoll = ArmKinematics.ExtractRollRadians(capturedPose.Rotation);
            }

            _targetPitch = desiredPitch;
            _targetRoll = desiredRoll;
            _hasTargetOrientation = true;

            // The collision-aware IK (docs/adr/0020): never a pose that hits the robot, the same
            // model RobotHost enforces. It starts from the robot's own measured pose when one has
            // arrived before anything was sent, and otherwise from the model's home pose.
            if (!_solverSeededFromRobot && !_hasSentJointCommand && _jointStateLink != null &&
                _jointStateLink.TryGetMeasured(out JetRoverJointPose measured) && measured.ArmSensed)
            {
                _solver.Reset(_profile, new ArmPose(measured.BaseYaw, measured.Proximal, measured.Distal, measured.WristPitch,
                    measured.RollSensed ? measured.WristRoll : 0f));
                _solverSeededFromRobot = true;
            }

            // Holding again after a park and relax (docs/adr/0023 decision 5): the arm is at the rest
            // pose, or wherever it was moved while limp, not where the ghost was. The solver starts over
            // from the measured pose, so the first commands approach the target from there along checked
            // joint paths instead of jumping straight to it.
            if (_jointStateLink != null && _jointStateLink.TakeResumedFromRelaxed() &&
                _jointStateLink.TryGetMeasured(out JetRoverJointPose resumed) && resumed.ArmSensed)
            {
                _solver.Reset(_profile, new ArmPose(resumed.BaseYaw, resumed.Proximal, resumed.Distal, resumed.WristPitch,
                    resumed.RollSensed ? resumed.WristRoll : 0f));
            }

            ArmSolution solution = _solver.Solve(_profile, capturedPose.Position, desiredPitch, desiredRoll);
            Span<float> wristPitches = stackalloc float[Math.Max(1, _profile.WristJointCount)];
            float baseYaw = solution.Pose.BaseYaw, proximalPitch = solution.Pose.Proximal, distalPitch = solution.Pose.Distal;
            wristPitches[0] = solution.Pose.Wrist;
            desiredRoll = solution.Pose.Roll;
            WristRollRadians = desiredRoll;
            bool wasClamped = solution.IsCompromised;
            bool pitchAdjusted = solution.Status == ArmSolveStatus.PitchAdjusted;
            float upperPitch = wristPitches[0];
            LastSolveStatus = solution.Status;

            // Not while approaching: the arm is still on its way to the target, and moving the target
            // to the arm would stop it there.
            if (snapTargetToReachable && solution.Status != ArmSolveStatus.Approaching &&
                (solution.Status != ArmSolveStatus.Exact || solution.ReachClamped))
            {
                // The target follows what the arm can reach, so the marker sits on the ghost's wrist
                // and a clutch re-engaged from here starts where the arm really is.
                dragTarget.position = armBase.TransformPoint(solution.Position.ToUnity());
                dragTarget.rotation = armBase.rotation
                    * Quaternion.AngleAxis(-solution.Pitch * Mathf.Rad2Deg, Vector3.right)
                    * Quaternion.AngleAxis(-desiredRoll * Mathf.Rad2Deg, Vector3.forward);
            }

            // Applied every frame, unconditionally -- see this class's own doc for why this is
            // deliberately not gated on ConfirmHardwareMotion or rate-limited: the rig needs to
            // be exercisable (and its axis signs calibratable) with no robot connected at all.
            // armRig's own ApplyAngles signature stays the 4-float shape it always had
            // (docs/adr/0011-generic-robot-arm-profiles.md deliberately keeps JetRoverArmRig
            // untouched -- a generic pivot-array rig is the deferred profile-picker-UI-adjacent
            // work, not part of this pass).
            if (armRig != null)
            {
                armRig.ApplyAngles(baseYaw, proximalPitch, distalPitch, upperPitch, wasClamped, pitchAdjusted);
            }

            // The course scores the commanded wrist, and its clock runs only while commands reach a
            // holding robot (docs/adr/0024 decisions 1 and 5).
            if (_training != null)
            {
                bool robotReady = _motionMode == JetRoverMotionMode.On && _config.ConfirmHardwareMotion && _jointStateLink.MayCommand(now);
                bool disturbed = (_jointTransport != null && _jointTransport.IsActive) || _jointStateLink.AddedDelayMs > 0;
                _training.Update(now, solution.Position, _gripperFraction, robotReady, disturbed);
                // Only once the course has run (its id is set by Start): a headset that never uses it sends nothing new.
                if (_training.CourseId != 0) _jointStateLink.SetTraining(_training.State);
            }

            // On, and the robot holds (or an older RobotHost that does not say): docs/adr/0023 decision 5.
            if (!_config.ConfirmHardwareMotion || !_jointStateLink.MayCommand(now))
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
            _operatorEndpoint.SubmitCommand(capturedPose, CoreVec.Zero, CoreVec.Zero, gripper: _gripperFraction, now);

            Span<JointTarget> jointTargets = stackalloc JointTarget[_profile.JointCount];
            int targetCount = ArmKinematics.MapAnglesToJointTargets(
                _profile, baseYaw, proximalPitch, distalPitch, wristPitches, gripperFraction: _gripperFraction, gripperRollRadians: desiredRoll, jointTargets);
            float speed = _training != null && _training.Running ? _training.SpeedRadPerSec : _speedRadPerSec;
            if (speed > 0f)
            {
                // The requested joint speed rides in every arm joint's Speed field (docs/adr/0018);
                // RobotHost caps it at what it was deployed to allow. The course's, while it runs.
                for (int i = 0; i < targetCount; i++)
                {
                    jointTargets[i] = new JointTarget(jointTargets[i].MotorId, jointTargets[i].Angle, speed);
                }
            }

            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize(targetCount)];
            if (JointCommandCodec.TryEncode(_jointSequence++, now, jointTargets.Slice(0, targetCount), buffer, out int bytesWritten))
            {
                _jointTransport.Send(buffer.Slice(0, bytesWritten), now);
                _jointStateLink.OnJointCommandSent(now, jointTargets.Slice(0, targetCount));
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
            _jointUdp?.Dispose();
            _metricSink?.Dispose();
        }

        // The drag target's pitch and roll as last read (Core convention, radians).
        private bool _hasTargetOrientation;
        private float _targetPitch, _targetRoll;

        private long MillisecondsToTicks(double ms) => (long)(ms / 1000.0 * _clock.TicksPerSecond);

        /// <summary>
        /// The config's predictor presets and the motors their reading ages are for. An empty preset
        /// list falls back to the older single-predictor fields (JointPredictorName, JointServo*,
        /// JointReadingAge*), as one preset that is also the active one.
        /// </summary>
        private static JointPredictorPresetData[] JointPresets(JetRoverArmConfig config, out byte[] readingAgeIds)
        {
            int[] configIds = config.JointReadingAgeMotorIds ?? new int[0];
            readingAgeIds = new byte[configIds.Length];
            for (int i = 0; i < configIds.Length; i++) readingAgeIds[i] = (byte)configIds[i];
            if (config.JointPredictorPresets != null && config.JointPredictorPresets.Length > 0)
            {
                return config.JointPredictorPresets;
            }

            config.JointPredictorActive = config.JointPredictorName;
            return new[]
            {
                new JointPredictorPresetData
                {
                    Label = config.JointPredictorName, Predictor = config.JointPredictorName, ModelSpeedRadPerSec = config.JointModelSpeedRadPerSec,
                    ServoMaxSpeedRadPerSec = config.JointServoMaxSpeedRadPerSec, ServoLagMs = config.JointServoLagMs,
                    ServoPipelineDelayMs = config.JointServoPipelineDelayMs, ServoFirstMoveFromMeasured = config.JointServoFirstMoveFromMeasured,
                    ReadingAgeMs = config.JointReadingAgeMs ?? new float[0],
                },
            };
        }
    }
}
