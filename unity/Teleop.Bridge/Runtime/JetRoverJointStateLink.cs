using System;
using Teleop.Core.Contracts;
using Teleop.Core.Registry;
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
    /// The operator end of the joint-state downlink (docs/adr/0018), riding on
    /// <see cref="JetRoverOperatorBridge"/>'s joint socket. The stream goes to whoever last sent on
    /// that port, so it must be the same socket the joint commands use.
    /// <list type="bullet">
    /// <item>Sends the keepalive every half second, so the robot streams even with motion off.</item>
    /// <item>Runs its own <see cref="ClockSync"/> from the frames' keepalive echo.</item>
    /// <item>Feeds the configured <see cref="ICommandAwarePredictor"/> with every measurement (once
    /// synced) and every joint command sent, the latter shifted by the estimated uplink delay
    /// (half the smoothed round trip).</item>
    /// </list>
    /// Plain C#, allocation-free after construction. Every decision about the robot's motion is
    /// the predictor's (Core).
    /// </summary>
    public sealed class JetRoverJointStateLink
    {
        private const double KeepaliveSeconds = 0.5;
        private const int MaxJoints = JointStateCodec.MaxJointsPerMessage;

        private readonly UdpTransport _transport;
        private readonly ITimeAuthority _clock;
        private readonly RobotArmProfile _profile;
        private readonly ClockSync _clockSync;
        private readonly ICommandAwarePredictor _predictor;
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

        public string PredictorName { get; }
        public bool HasState => _measuredCount > 0;
        public bool IsSynced => _clockSync.Diagnostics.IsSynced;
        public long FramesReceived { get; private set; }
        public float RoundTripMilliseconds => _clockSync.Diagnostics.LastRttTicks * 1000f / _clock.TicksPerSecond;

        /// <summary>Seconds since the last frame arrived; infinity before the first.</summary>
        public float SecondsSinceLastFrame =>
            FramesReceived == 0 ? float.PositiveInfinity : (_clock.NowTicks - _lastFrameArrivalTicks) / (float)_clock.TicksPerSecond;

        public JetRoverJointStateLink(UdpTransport transport, ITimeAuthority clock, in RobotArmProfile profile,
            ClockSyncConfig clockSyncConfig, string predictorName, float maxJointSpeedRadPerSec, double maxHorizonMs)
        {
            _transport = transport ?? throw new ArgumentNullException(nameof(transport));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            _profile = profile;
            _clockSync = new ClockSync(clockSyncConfig);
            _keepaliveIntervalTicks = (long)(KeepaliveSeconds * clock.TicksPerSecond);

            if (!Registries.JointPredictors.TryGetValue(predictorName, out Func<JointPredictorConfig, ICommandAwarePredictor> factory))
            {
                throw new ArgumentException($"unknown joint predictor '{predictorName}'; registered: {string.Join(", ", Registries.JointPredictors.Keys)}");
            }

            PredictorName = predictorName;
            _predictor = factory(new JointPredictorConfig(
                maxJoints: MaxJoints, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: maxJointSpeedRadPerSec,
                instantMotorIds: GripperIds(profile), maxHorizonTicks: (long)(maxHorizonMs / 1000.0 * clock.TicksPerSecond),
                ticksPerSecond: clock.TicksPerSecond));
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
                    _predictor.ObserveState(_queueSensed[slot], _measured.AsSpan(0, _measuredCount));
                }

                _queueHead = (_queueHead + 1) % QueueCapacity;
                _queueSize--;
            }
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
            _predictor.ObserveCommand(sendTicks + uplink, _command.AsSpan(0, n), speed);
        }

        /// <summary>The newest measured joints, as received.</summary>
        public bool TryGetMeasured(out JetRoverJointPose pose)
        {
            pose = default;
            if (_measuredCount == 0) return false;
            pose = ToPose(_measured, _measuredCount);
            return true;
        }

        /// <summary>The predictor's estimate for <paramref name="targetTicks"/> (operator clock).</summary>
        public bool TryGetPredicted(long targetTicks, out JetRoverJointPose pose)
        {
            pose = default;
            if (!_predictor.TryPredict(targetTicks, _predicted, out int count) || count == 0) return false;
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
