using System;
using System.Collections.Generic;
using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Prediction;
using Teleop.Core.Time;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>
    /// The operator end of a serial-chain RobotHost's joint port (docs/adr/0027 decision 5, docs/adr/0029 decision 5):
    /// the same wire as the JetRover's joint link (docs/adr/0018), without the JetRover's arm power, training course
    /// and headset reports, which a serial-chain host does not take.
    /// <list type="bullet">
    /// <item>Sends the keepalive every half second, so the host streams joint-state frames even before a command.</item>
    /// <item>Runs its own <see cref="ClockSync"/> from the frames' keepalive echo.</item>
    /// <item>Sends the operator heartbeat (docs/adr/0018 §9) once a second and at every change of <see cref="MotionOn"/>:
    /// who is sending and in which mode, never a power request (a serial-chain host has no arm power). RobotHost
    /// records sessions automatically only while a headset's heartbeat arrives, so the headset's sessions are the
    /// ones recorded for training, and a tool's are not.</item>
    /// <item>Feeds a <see cref="JointPredictorBank"/> with every frame (once synced, at its reading's time on the
    /// operator clock) and every command sent (shifted by the clock's uncertainty, the estimated uplink delay, as the
    /// JetRover link does). The bank's active preset is the twin's estimate.</item>
    /// <item>Scores every enabled preset live (<see cref="LiveJointScorer"/>, a rolling 30 s window), for the operator
    /// console (docs/adr/0029 amendment): the grippers in percent of their opening.</item>
    /// <item>Holds frames back by <see cref="AddedDelayMs"/> before they become the measured arm and reach the
    /// predictors, as if the downlink were that slow; ClockSync sees the real arrival.</item>
    /// <item>Reports what the headset does to its link and view (<see cref="SetHeadsetConditions"/>: a disturbance,
    /// an added delay) at every change and once a second, as the JetRover link does (docs/adr/0023 amendment), so
    /// RobotHost writes them into the recording and the trainer refuses sessions that were not on the real network.</item>
    /// </list>
    /// Plain C# behind <see cref="ITransport"/> and <see cref="ITimeAuthority"/>: no sockets, no clock reads of its
    /// own, nothing allocated after construction. A frame from another profile (a motor id this one does not have) is
    /// still taken; its unknown motors are ignored where the profile's order is asked for.
    /// </summary>
    public sealed class SerialChainJointLink
    {
        private const double KeepaliveSeconds = 0.5;
        private const double ScoreWindowSeconds = 30.0;
        private const int MaxJoints = JointStateCodec.MaxJointsPerMessage;
        private const int QueueCapacity = 128;

        /// <summary>The scorer's gripper unit: an opening fraction per "radian" of 0.573 reports its error in percent of the opening.</summary>
        public const float GripperFractionPerRadian = 0.5729578f;

        private readonly ITransport _transport;
        private readonly ITimeAuthority _clock;
        private readonly SerialChainProfile _profile;
        private readonly ClockSync _clockSync;
        private readonly JointPredictorBank _bank;
        private readonly LiveJointScorer _scorer;
        private readonly long _keepaliveIntervalTicks;
        private readonly OperatorClientKind _clientKind;
        private readonly byte[] _buffer = new byte[JointDatagram.MaxBytes];
        private readonly byte[] _heartbeat = new byte[ArmPowerCodec.RequestSize];
        private long _lastHeartbeatTicks = long.MinValue;
        private uint _heartbeatSequence;
        private bool _motionOn, _motionChanged = true;
        private readonly byte[] _headsetBuffer = new byte[HeadsetConditionsCodec.Size];
        private HeadsetConditions _headset;
        private bool _headsetChanged = true;
        private long _lastHeadsetSentTicks = long.MinValue;
        private uint _headsetSequence;
        private readonly JointSample[] _received = new JointSample[MaxJoints];
        private readonly JointSample[] _measured = new JointSample[MaxJoints];
        private readonly JointSample[] _predicted = new JointSample[MaxJoints];
        private readonly JointSample[] _command = new JointSample[MaxJoints];
        private int _measuredCount;
        private long _lastKeepaliveTicks = long.MinValue;
        private long _lastEchoOperatorTicks = long.MinValue;
        private long _lastFrameArrivalTicks = long.MinValue;
        private long _lastSensedRobotTicks = long.MinValue;
        private long _lastFreshArrivalTicks = long.MinValue;
        private double _addedDelayMs;

        // Frames waiting out the added delay: when each is released, its reading's time (operator clock), and its joints.
        private readonly long[] _queueRelease = new long[QueueCapacity];
        private readonly long[] _queueSensed = new long[QueueCapacity];
        private readonly int[] _queueCounts = new int[QueueCapacity];
        private readonly JointSample[] _queueJoints = new JointSample[QueueCapacity * MaxJoints];
        private int _queueHead, _queueSize;

        public SerialChainJointLink(ITransport transport, ITimeAuthority clock, SerialChainProfile profile, ClockSyncConfig clockSyncConfig,
            IReadOnlyList<JointPredictorPreset> presets, int activePreset, OperatorClientKind clientKind = OperatorClientKind.Headset)
        {
            _transport = transport ?? throw new ArgumentNullException(nameof(transport));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            _profile = profile ?? throw new ArgumentNullException(nameof(profile));
            if (presets == null || presets.Count == 0) throw new ArgumentException("no joint predictor presets", nameof(presets));
            _clockSync = new ClockSync(clockSyncConfig);
            _bank = JointPredictorBank.Create(presets, activePreset);
            _scorer = new LiveJointScorer(_bank, (long)(ScoreWindowSeconds * clock.TicksPerSecond), MaxJoints,
                SerialChainShaping.GripperMotorIds(profile), GripperFractionPerRadian);
            _keepaliveIntervalTicks = (long)(KeepaliveSeconds * clock.TicksPerSecond);
            _clientKind = clientKind;
        }

        /// <summary>The motion mode the heartbeat reports: on while commands are going, off otherwise.</summary>
        public bool MotionOn
        {
            get => _motionOn;
            set
            {
                if (value == _motionOn) return;
                _motionOn = value;
                _motionChanged = true;
            }
        }

        /// <summary>
        /// What the headset does to its link and view now (the console's disturbance, the added delay): sent at the next
        /// <see cref="Tick"/> when it changed, and once a second whatever it is, off included.
        /// </summary>
        public void SetHeadsetConditions(in HeadsetConditions conditions)
        {
            if (!_headset.Equals(conditions)) _headsetChanged = true;
            _headset = conditions;
        }

        /// <summary>
        /// The predictor config for <paramref name="profile"/>: the robot's trajectory shaper (<c>franka-replay</c>'s
        /// replay), the grippers moved at once on command, and the learned layout (<c>learned</c>, layout v4).
        /// </summary>
        public static JointPredictorConfig PredictorConfig(SerialChainProfile profile, long ticksPerSecond, double maxHorizonMs, float modelSpeedRadPerSec)
        {
            TrajectoryShaperConfig shaper = SerialChainShaping.ForProfile(profile);
            return new JointPredictorConfig(
                    maxJoints: MaxJoints, commandHistoryCapacity: 96, maxJointSpeedRadPerSec: modelSpeedRadPerSec,
                    instantMotorIds: SerialChainShaping.GripperMotorIds(profile), maxHorizonTicks: (long)(maxHorizonMs / 1000.0 * ticksPerSecond),
                    ticksPerSecond: ticksPerSecond, shaper: shaper)
                .WithChainLayout(SerialChainShaping.LearnedLayout(profile));
        }

        public JointPredictorBank Bank => _bank;

        /// <summary>Every enabled preset's live numbers over the last 30 s (diagnostics, never results).</summary>
        public LiveJointScorer Scorer => _scorer;

        /// <summary>
        /// An artificial delay on the robot's reports (ms, 0..2000), for seeing prediction work on a fast LAN: frames are
        /// held back this long before they become the measured arm and reach the predictors. Commands are not delayed,
        /// and ClockSync sees the real arrival. Anything recorded with a delay set is not a real-network measurement.
        /// </summary>
        public double AddedDelayMs
        {
            get => _addedDelayMs;
            set => _addedDelayMs = double.IsNaN(value) ? 0 : Math.Max(0, Math.Min(2000, value));
        }

        /// <summary>Seconds since a frame last brought a fresh reading (a sensed stamp later than every earlier one); infinity before one.</summary>
        public double SecondsSinceFreshReading(long now) =>
            _lastFreshArrivalTicks == long.MinValue ? double.PositiveInfinity : (now - _lastFreshArrivalTicks) / (double)_clock.TicksPerSecond;

        public bool IsSynced => _clockSync.Diagnostics.IsSynced;

        public long FramesReceived { get; private set; }

        /// <summary>The newest keepalive round trip, ms.</summary>
        public float RoundTripMilliseconds => _clockSync.Diagnostics.LastRttTicks * 1000f / _clock.TicksPerSecond;

        /// <summary>Seconds since the newest frame arrived; infinity before one.</summary>
        public double SecondsSinceLastFrame(long now) =>
            _lastFrameArrivalTicks == long.MinValue ? double.PositiveInfinity : (now - _lastFrameArrivalTicks) / (double)_clock.TicksPerSecond;

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

            if ((_motionChanged || _lastHeartbeatTicks == long.MinValue || now - _lastHeartbeatTicks >= _clock.TicksPerSecond) &&
                ArmPowerCodec.TryEncodeRequest(_clientKind, _motionOn ? OperatorMotionMode.On : OperatorMotionMode.Off, ArmPowerRequest.None,
                    ++_heartbeatSequence, now, _heartbeat, out int heartbeatBytes))
            {
                _transport.Send(_heartbeat.AsSpan(0, heartbeatBytes), now);
                _lastHeartbeatTicks = now;
                _motionChanged = false;
            }

            if ((_headsetChanged || _lastHeadsetSentTicks == long.MinValue || now - _lastHeadsetSentTicks >= _clock.TicksPerSecond) &&
                HeadsetConditionsCodec.TryEncode(_headset, ++_headsetSequence, now, _headsetBuffer, out int headsetBytes))
            {
                _transport.Send(_headsetBuffer.AsSpan(0, headsetBytes), now);
                _headsetChanged = false;
                _lastHeadsetSentTicks = now;
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
                if (echoOperator != _lastEchoOperatorTicks)
                {
                    _lastEchoOperatorTicks = echoOperator;
                    _clockSync.AddRoundTrip(echoOperator, _clock.TicksPerSecond, echoRobotRecv, robotSend, robotTps, arrival);
                }

                // A reading newer than any before is fresh. A stamp more than 10 s older is a restarted robot clock.
                if (sensed != 0 && (_lastSensedRobotTicks == long.MinValue || sensed > _lastSensedRobotTicks || sensed < _lastSensedRobotTicks - 10 * robotTps))
                {
                    _lastSensedRobotTicks = sensed;
                    _lastFreshArrivalTicks = arrival;
                }

                // No reading yet means the values are the host's beliefs from the moment of sending.
                long robotTime = sensed != 0 ? sensed : robotSend;
                long sensedOperator = IsSynced ? _clockSync.ToOperatorTicks(robotTime, robotTps, _clock.TicksPerSecond) : long.MinValue;
                Enqueue(arrival + (long)(_addedDelayMs / 1000.0 * _clock.TicksPerSecond), sensedOperator, jointCount);
            }

            // Frames become the measurement, and reach the predictors (scored first), once their added delay is over.
            while (_queueSize > 0 && _queueRelease[_queueHead] <= now)
            {
                int slot = _queueHead;
                int jointCount = _queueCounts[slot];
                Array.Copy(_queueJoints, slot * MaxJoints, _measured, 0, jointCount);
                _measuredCount = jointCount;
                MeasuredOperatorTicks = _queueSensed[slot];
                if (MeasuredOperatorTicks != long.MinValue) _scorer.ObserveState(MeasuredOperatorTicks, now, _measured.AsSpan(0, jointCount));
                _queueHead = (_queueHead + 1) % QueueCapacity;
                _queueSize--;
            }

            // Once per tick: the jitter of each preset's estimate for now.
            _scorer.OnDisplayTick(now, now);
        }

        private void Enqueue(long releaseTicks, long sensedOperatorTicks, int jointCount)
        {
            if (_queueSize == QueueCapacity)
            {
                // Full (a delay longer than 2.5 s at 50 Hz): the oldest goes.
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

        /// <summary>A joint command was just sent; the predictors replay it from when it reaches the robot.</summary>
        public void OnCommandSent(long sendTicks, ReadOnlySpan<JointTarget> targets)
        {
            int n = Math.Min(targets.Length, MaxJoints);
            for (int i = 0; i < n; i++)
            {
                _command[i] = new JointSample(targets[i].MotorId, targets[i].Angle, sensed: false);
            }

            long uplink = IsSynced ? _clockSync.Diagnostics.OffsetUncertaintyTicks : 0;
            _bank.ObserveCommand(sendTicks + uplink, _command.AsSpan(0, n), 0f);
        }

        /// <summary>When the newest frame's joints were read, on the operator's clock; <c>long.MinValue</c> before the clocks are synced.</summary>
        public long MeasuredOperatorTicks { get; private set; } = long.MinValue;

        /// <summary>The newest frame's joints, exactly as received.</summary>
        public ReadOnlySpan<JointSample> Measured => _measured.AsSpan(0, _measuredCount);

        /// <summary>The active preset's estimate for <paramref name="targetTicks"/> (operator clock); empty when it has none.</summary>
        public ReadOnlySpan<JointSample> Predicted(long targetTicks) =>
            _bank.TryPredict(targetTicks, _predicted, out int count) ? _predicted.AsSpan(0, count) : ReadOnlySpan<JointSample>.Empty;

        /// <summary>
        /// Writes <paramref name="samples"/>' values in the profile's motor order (each arm's joints, then its gripper)
        /// into <paramref name="values"/>; a motor missing from them keeps its value there. Returns how many were found.
        /// </summary>
        public int ToProfileOrder(ReadOnlySpan<JointSample> samples, Span<float> values)
        {
            int found = 0;
            int k = 0;
            for (int a = 0; a < _profile.Arms.Length; a++)
            {
                ChainArm arm = _profile.Arms[a];
                for (int j = 0; j < arm.Joints.Length; j++, k++)
                {
                    if (k < values.Length && TryFind(samples, arm.Joints[j].MotorId, out float v))
                    {
                        values[k] = v;
                        found++;
                    }
                }

                if (arm.HasGripper)
                {
                    if (k < values.Length && TryFind(samples, arm.Gripper.MotorId, out float g))
                    {
                        values[k] = g;
                        found++;
                    }

                    k++;
                }
            }

            return found;
        }

        private static bool TryFind(ReadOnlySpan<JointSample> samples, byte motorId, out float value)
        {
            for (int i = 0; i < samples.Length; i++)
            {
                if (samples[i].MotorId == motorId)
                {
                    value = samples[i].Value;
                    return true;
                }
            }

            value = 0f;
            return false;
        }
    }
}
