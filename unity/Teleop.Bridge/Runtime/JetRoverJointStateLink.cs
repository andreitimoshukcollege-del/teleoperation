using System;
using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Prediction;
using Teleop.Core.Time;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>
    /// One arm's joints as the scene shows them: the angles in radians (Core convention), and the
    /// gripper as a 0..1 fraction. A plain Unity-free view of a Core joint state, so display code
    /// outside Bridge never touches Core types (unity/CLAUDE.md).
    /// </summary>
    public readonly struct JetRoverJointPose
    {
        public readonly float BaseYaw, Proximal, Distal, WristPitch, WristRoll, Gripper;

        /// <summary>True when every arm joint in it came from servo feedback, not a commanded belief.</summary>
        public readonly bool ArmSensed;

        /// <summary>Whether the wrist roll and the gripper were measured. Without servo feedback they are only the last command.</summary>
        public readonly bool RollSensed, GripperSensed;

        public JetRoverJointPose(float baseYaw, float proximal, float distal, float wristPitch, float wristRoll, float gripper,
            bool armSensed, bool rollSensed = false, bool gripperSensed = false)
        {
            BaseYaw = baseYaw; Proximal = proximal; Distal = distal; WristPitch = wristPitch;
            WristRoll = wristRoll; Gripper = gripper; ArmSensed = armSensed; RollSensed = rollSensed; GripperSensed = gripperSensed;
        }
    }

    /// <summary>
    /// One predictor's live numbers (docs/adr/0023 decision 2), as plain floats so display code
    /// outside Bridge never touches Core types: degrees over the arm joints, the gripper in servo
    /// degrees of its travel, NaN where there is nothing yet. Diagnostics, never results.
    /// </summary>
    public readonly struct JetRoverPredictorStats
    {
        public readonly bool HasData;
        public readonly int Readings;
        public readonly float ErrorP50Deg, ErrorP95Deg, CorrectionP95Deg, JitterP95Deg, GripperErrorP95ServoDeg;

        public JetRoverPredictorStats(bool hasData, int readings, float errorP50Deg, float errorP95Deg, float correctionP95Deg,
            float jitterP95Deg, float gripperErrorP95ServoDeg)
        {
            HasData = hasData; Readings = readings; ErrorP50Deg = errorP50Deg; ErrorP95Deg = errorP95Deg;
            CorrectionP95Deg = correctionP95Deg; JitterP95Deg = jitterP95Deg; GripperErrorP95ServoDeg = gripperErrorP95ServoDeg;
        }
    }

    /// <summary>
    /// The operator end of the joint-state downlink (docs/adr/0018), riding on
    /// <see cref="JetRoverOperatorBridge"/>'s joint socket. The stream goes to whoever last sent on
    /// that port, so it must be the same socket the joint commands use.
    /// <list type="bullet">
    /// <item>Sends the keepalive every half second, so the robot streams even with motion off.</item>
    /// <item>Runs its own <see cref="ClockSync"/> from the frames' keepalive echo.</item>
    /// <item>Feeds a <see cref="JointPredictorBank"/> (docs/adr/0023 decision 1) with every measurement
    /// (once synced) and every joint command sent, the latter shifted by the estimated uplink delay
    /// (half the smoothed round trip). The bank's active preset draws the twin; the others run
    /// beside it, and the console switches and toggles them.</item>
    /// <item>Scores every enabled preset live (<see cref="LiveJointScorer"/>, a rolling 30 s window).</item>
    /// </list>
    /// Plain C#, allocation-free after construction. Every decision about the robot's motion is
    /// the predictor's (Core).
    /// </summary>
    public sealed class JetRoverJointStateLink
    {
        private const double KeepaliveSeconds = 0.5;
        private const double ScoreWindowSeconds = 30.0;
        private const int MaxJoints = JointStateCodec.MaxJointsPerMessage;

        private readonly ITransport _transport;
        private readonly ITimeAuthority _clock;
        private readonly RobotArmProfile _profile;
        private readonly ClockSync _clockSync;
        private readonly JointPredictorBank _bank;
        private readonly LiveJointScorer _scorer;
        private readonly long _keepaliveIntervalTicks;
        private readonly byte[] _buffer = new byte[128];
        private readonly JointSample[] _received = new JointSample[MaxJoints];
        private readonly JointSample[] _measured = new JointSample[MaxJoints];
        private readonly JointSample[] _command = new JointSample[MaxJoints];
        private readonly JointSample[] _predicted = new JointSample[MaxJoints];
        private int _measuredCount;
        private long _lastKeepaliveTicks = long.MinValue;
        private long _lastEchoOperatorTicks = long.MinValue;
        private long _lastFrameArrivalTicks;

        /// <summary>The label of the preset drawing the twin.</summary>
        public string PredictorName => _bank.ActiveLabel;
        public bool HasState => _measuredCount > 0;
        public bool IsSynced => _clockSync.Diagnostics.IsSynced;
        public long FramesReceived { get; private set; }
        public float RoundTripMilliseconds => _clockSync.Diagnostics.LastRttTicks * 1000f / _clock.TicksPerSecond;

        /// <summary>Seconds since the last frame arrived; infinity before the first.</summary>
        public float SecondsSinceLastFrame =>
            FramesReceived == 0 ? float.PositiveInfinity : (_clock.NowTicks - _lastFrameArrivalTicks) / (float)_clock.TicksPerSecond;

        /// <summary>
        /// The link, with one bank member per entry of <paramref name="presets"/> and
        /// <paramref name="activeLabel"/> drawing (the first enabled preset when no label matches).
        /// <paramref name="readingAgeMotorIds"/> are the motors each preset's <c>ReadingAgeMs</c> are for.
        /// Throws <see cref="ArgumentException"/> for an unknown predictor key or mismatched reading ages.
        /// </summary>
        public JetRoverJointStateLink(ITransport transport, ITimeAuthority clock, in RobotArmProfile profile,
            ClockSyncConfig clockSyncConfig, JointPredictorPresetData[] presets, string activeLabel, double maxHorizonMs,
            float gripperFractionPerRadian, byte[] readingAgeMotorIds)
        {
            _transport = transport ?? throw new ArgumentNullException(nameof(transport));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            if (presets == null || presets.Length == 0) throw new ArgumentException("no joint predictor presets");
            _profile = profile;
            _clockSync = new ClockSync(clockSyncConfig);
            _keepaliveIntervalTicks = (long)(KeepaliveSeconds * clock.TicksPerSecond);

            byte[] gripperIds = GripperIds(profile);
            var corePresets = new JointPredictorPreset[presets.Length];
            int active = -1, firstEnabled = -1;
            for (int i = 0; i < presets.Length; i++)
            {
                JointPredictorPresetData p = presets[i] ?? throw new ArgumentException($"preset {i} is null");
                corePresets[i] = new JointPredictorPreset(p.Label, p.Predictor, new JointPredictorConfig(
                    maxJoints: MaxJoints, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: p.ModelSpeedRadPerSec,
                    instantMotorIds: new byte[0], maxHorizonTicks: (long)(maxHorizonMs / 1000.0 * clock.TicksPerSecond),
                    ticksPerSecond: clock.TicksPerSecond,
                    // The gripper moves like the other joints, at the servo's speed, in its own units
                    // (a 0..1 fraction): fraction per radian of servo travel.
                    scaledMotorIds: gripperIds, speedScales: Repeat(gripperFractionPerRadian, gripperIds.Length),
                    // How much older than the frame's stamp each motor's reading is (relay-replay*).
                    sensedAgeMotorIds: ReadingAgeIds(p, readingAgeMotorIds), sensedAgeSeconds: ReadingAgeSeconds(p, readingAgeMotorIds),
                    // relay-replay-servo's servo (docs/adr/0021 decision 15); the others ignore it.
                    servoMaxSpeedRadPerSec: p.ServoMaxSpeedRadPerSec, servoLagSeconds: p.ServoLagMs / 1000f,
                    servoPipelineDelaySeconds: p.ServoPipelineDelayMs / 1000f, servoFirstMoveFromMeasured: p.ServoFirstMoveFromMeasured));
                if (p.Label == activeLabel) active = i;
                if (p.Enabled && firstEnabled < 0) firstEnabled = i;
            }

            if (active < 0) active = firstEnabled >= 0 ? firstEnabled : 0;
            _bank = JointPredictorBank.Create(corePresets, active);
            for (int i = 0; i < presets.Length; i++)
            {
                if (!presets[i].Enabled && i != active) _bank.TrySetEnabled(i, false);
            }

            _scorer = new LiveJointScorer(_bank, (long)(ScoreWindowSeconds * clock.TicksPerSecond), MaxJoints, gripperIds,
                gripperFractionPerRadian);
        }

        // ---- The console's view of the bank (docs/adr/0023): indexes are the presets' order. ----

        public int PredictorCount => _bank.Count;

        public int ActivePredictorIndex => _bank.ActiveIndex;

        public string PredictorLabel(int index) => _bank.LabelAt(index);

        public bool IsPredictorEnabled(int index) => _bank.IsEnabled(index);

        /// <summary>Enabled, but has not seen a reading since it was turned on.</summary>
        public bool IsPredictorWarmingUp(int index) => _bank.IsEnabled(index) && !_bank.HasState(index);

        /// <summary>Draws the twin with preset <paramref name="index"/>; false when it is off.</summary>
        public bool TrySelectPredictor(int index) => _bank.TrySetActive(index);

        /// <summary>Turns a preset on (it starts from nothing) or off; false for the drawing one being turned off.</summary>
        public bool TrySetPredictorEnabled(int index, bool enabled) => _bank.TrySetEnabled(index, enabled);

        public bool TryGetPredictorStats(int index, out JetRoverPredictorStats stats)
        {
            stats = default;
            if (!_scorer.TryGetStats(index, out LiveJointStats s)) return false;
            stats = new JetRoverPredictorStats(s.HasData, s.Readings, s.ErrorP50Deg, s.ErrorP95Deg, s.CorrectionP95Deg,
                s.JitterP95Deg, s.GripperErrorP95ServoDeg);
            return true;
        }

        private static byte[] ReadingAgeIds(JointPredictorPresetData p, byte[] ids) =>
            p.ReadingAgeMs == null || p.ReadingAgeMs.Length == 0 ? new byte[0] : CheckedIds(p, ids);

        private static byte[] CheckedIds(JointPredictorPresetData p, byte[] ids)
        {
            if (ids == null || ids.Length != p.ReadingAgeMs.Length)
            {
                throw new ArgumentException($"preset '{p.Label}': {p.ReadingAgeMs.Length} reading ages for {(ids == null ? 0 : ids.Length)} motor ids");
            }

            return ids;
        }

        private static float[] ReadingAgeSeconds(JointPredictorPresetData p, byte[] ids)
        {
            if (p.ReadingAgeMs == null || p.ReadingAgeMs.Length == 0) return new float[0];
            CheckedIds(p, ids);
            var seconds = new float[p.ReadingAgeMs.Length];
            for (int i = 0; i < seconds.Length; i++) seconds[i] = p.ReadingAgeMs[i] / 1000f;
            return seconds;
        }

        /// <summary>Sends a keepalive when one is due, and takes in every frame that has arrived.</summary>
        public void Tick(long now)
        {
            if (_lastKeepaliveTicks == long.MinValue || now - _lastKeepaliveTicks >= _keepaliveIntervalTicks)
            {
                if (JointStateCodec.TryEncodeSubscribe(now, _clock.TicksPerSecond, _buffer, out int written))
                {
                    _transport.Send(_buffer.AsSpan(0, written), now);
                }

                _lastKeepaliveTicks = now;
            }

            while (_transport.TryReceive(now, _buffer, out int count, out long arrival))
            {
                if (!JointStateCodec.TryDecode(_buffer.AsSpan(0, count), out _, out long robotSend, out long sensed,
                        out long echoOperator, out long echoRobotRecv, out long robotTps, _received, out int jointCount))
                {
                    continue;
                }

                FramesReceived++;
                _lastFrameArrivalTicks = arrival;

                // ClockSync sees the real arrival: an added delay must not bias the clock offset.
                if (echoOperator != _lastEchoOperatorTicks)
                {
                    _lastEchoOperatorTicks = echoOperator;
                    _clockSync.AddRoundTrip(echoOperator, _clock.TicksPerSecond, echoRobotRecv, robotSend, robotTps, arrival);
                }

                // No feedback yet means the values are beliefs from the moment of sending.
                long robotTime = sensed != 0 ? sensed : robotSend;
                long sensedOperator = IsSynced ? _clockSync.ToOperatorTicks(robotTime, robotTps, _clock.TicksPerSecond) : long.MinValue;
                Enqueue(arrival + AddedDelayTicks, sensedOperator, jointCount);
            }

            // Frames become the measurement, and reach the predictor, once their added delay is over.
            while (_queueSize > 0 && _queueRelease[_queueHead] <= now)
            {
                int slot = _queueHead;
                int count = _queueCounts[slot];
                Array.Copy(_queueJoints, slot * MaxJoints, _measured, 0, count);
                _measuredCount = count;
                if (_queueSensed[slot] != long.MinValue)
                {
                    // Scored first (each preset's prediction for this reading's time), then observed.
                    _scorer.ObserveState(_queueSensed[slot], now, _measured.AsSpan(0, _measuredCount));
                }

                _queueHead = (_queueHead + 1) % QueueCapacity;
                _queueSize--;
            }

            // Once per frame: the jitter of each preset's prediction for now.
            _scorer.OnDisplayTick(now, now);
        }

        private const int QueueCapacity = 128;
        private readonly long[] _queueRelease = new long[QueueCapacity];
        private readonly long[] _queueSensed = new long[QueueCapacity];
        private readonly int[] _queueCounts = new int[QueueCapacity];
        private readonly JointSample[] _queueJoints = new JointSample[QueueCapacity * MaxJoints];
        private int _queueHead, _queueSize;

        private void Enqueue(long releaseTicks, long sensedOperatorTicks, int jointCount)
        {
            if (_queueSize == QueueCapacity)
            {
                // Full (an added delay longer than 2.5 s at 50 Hz): drop the oldest.
                _queueHead = (_queueHead + 1) % QueueCapacity;
                _queueSize--;
            }

            int slot = (_queueHead + _queueSize) % QueueCapacity;
            _queueRelease[slot] = releaseTicks;
            _queueSensed[slot] = sensedOperatorTicks;
            _queueCounts[slot] = jointCount;
            Array.Copy(_received, 0, _queueJoints, slot * MaxJoints, jointCount);
            _queueSize++;
        }

        private double _addedDelayMs;

        /// <summary>
        /// An artificial delay on the robot's state reports (ms), for seeing the prediction work on a
        /// fast LAN: frames are held back this long before they become the measured arm and reach the
        /// predictor, as if the downlink were that slow. Commands are not delayed, and clock sync
        /// sees the real arrival. 0 (the default) adds nothing. Anything recorded with a delay set is
        /// not a real-network measurement.
        /// </summary>
        public double AddedDelayMs
        {
            get => _addedDelayMs;
            set => _addedDelayMs = double.IsNaN(value) ? 0 : Math.Max(0, Math.Min(2000, value));
        }

        private long AddedDelayTicks => (long)(_addedDelayMs / 1000.0 * _clock.TicksPerSecond);

        /// <summary>A joint command was just sent; the predictor replays it from when it reaches the robot.</summary>
        public void OnJointCommandSent(long sendTicks, ReadOnlySpan<JointTarget> targets)
        {
            int n = Math.Min(targets.Length, MaxJoints);
            float speed = 0f;
            for (int i = 0; i < n; i++)
            {
                _command[i] = new JointSample(targets[i].MotorId, targets[i].Angle, sensed: false);
                if (targets[i].Speed > speed) speed = targets[i].Speed;
            }

            long uplink = IsSynced ? _clockSync.Diagnostics.OffsetUncertaintyTicks : 0;
            _bank.ObserveCommand(sendTicks + uplink, _command.AsSpan(0, n), speed);
        }

        /// <summary>The newest measured joints, as received.</summary>
        public bool TryGetMeasured(out JetRoverJointPose pose)
        {
            pose = default;
            if (_measuredCount == 0) return false;
            pose = ToPose(_measured, _measuredCount);
            return true;
        }

        /// <summary>The drawing preset's estimate for <paramref name="targetTicks"/> (operator clock).</summary>
        public bool TryGetPredicted(long targetTicks, out JetRoverJointPose pose)
        {
            pose = default;
            if (!_bank.TryPredict(targetTicks, _predicted, out int count) || count == 0) return false;
            pose = ToPose(_predicted, count);
            return true;
        }

        private JetRoverJointPose ToPose(JointSample[] joints, int count)
        {
            float yaw = 0f, proximal = 0f, distal = 0f, pitch = 0f, roll = 0f, gripper = 0f;
            bool sensed = true, rollSensed = false, gripperSensed = false;
            for (int i = 0; i < count; i++)
            {
                int index = IndexOf(joints[i].MotorId);
                if (index < 0) continue;
                JointHardwareSpec spec = _profile.Joints[index];
                float v = joints[i].Value;
                switch (spec.Role)
                {
                    case JointRole.BaseYaw: yaw = v; sensed &= joints[i].Sensed; break;
                    case JointRole.Proximal: proximal = v; sensed &= joints[i].Sensed; break;
                    case JointRole.Distal: distal = v; sensed &= joints[i].Sensed; break;
                    case JointRole.Wrist: if (spec.WristIndex == 0) { pitch = v; sensed &= joints[i].Sensed; } break;
                    case JointRole.GripperRotate: roll = v; rollSensed = joints[i].Sensed; break;
                    case JointRole.GripperMain: gripper = v; gripperSensed = joints[i].Sensed; break;
                }
            }

            return new JetRoverJointPose(yaw, proximal, distal, pitch, roll, gripper, sensed, rollSensed, gripperSensed);
        }

        private int IndexOf(byte motorId)
        {
            for (int i = 0; i < _profile.Joints.Length; i++)
            {
                if (_profile.Joints[i].MotorId == motorId) return i;
            }

            return -1;
        }

        private static float[] Repeat(float value, int count)
        {
            var values = new float[count];
            for (int i = 0; i < count; i++) values[i] = value;
            return values;
        }

        private static byte[] GripperIds(in RobotArmProfile profile)
        {
            int n = 0;
            foreach (JointHardwareSpec j in profile.Joints) if (j.Role == JointRole.GripperMain) n++;
            var ids = new byte[n];
            n = 0;
            foreach (JointHardwareSpec j in profile.Joints) if (j.Role == JointRole.GripperMain) ids[n++] = j.MotorId;
            return ids;
        }
    }
}
