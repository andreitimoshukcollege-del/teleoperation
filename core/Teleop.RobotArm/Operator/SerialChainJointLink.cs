using System;
using System.Collections.Generic;
using Teleop.Core.Contracts;
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
    /// <item>Feeds a <see cref="JointPredictorBank"/> with every frame (once synced, at its reading's time on the
    /// operator clock) and every command sent (shifted by the clock's uncertainty, the estimated uplink delay, as the
    /// JetRover link does). The bank's active preset is the twin's estimate.</item>
    /// </list>
    /// Plain C# behind <see cref="ITransport"/> and <see cref="ITimeAuthority"/>: no sockets, no clock reads of its
    /// own, nothing allocated after construction. A frame from another profile (a motor id this one does not have) is
    /// still taken; its unknown motors are ignored where the profile's order is asked for.
    /// </summary>
    public sealed class SerialChainJointLink
    {
        private const double KeepaliveSeconds = 0.5;
        private const int MaxJoints = JointStateCodec.MaxJointsPerMessage;

        private readonly ITransport _transport;
        private readonly ITimeAuthority _clock;
        private readonly SerialChainProfile _profile;
        private readonly ClockSync _clockSync;
        private readonly JointPredictorBank _bank;
        private readonly long _keepaliveIntervalTicks;
        private readonly byte[] _buffer = new byte[JointDatagram.MaxBytes];
        private readonly JointSample[] _received = new JointSample[MaxJoints];
        private readonly JointSample[] _measured = new JointSample[MaxJoints];
        private readonly JointSample[] _predicted = new JointSample[MaxJoints];
        private readonly JointSample[] _command = new JointSample[MaxJoints];
        private int _measuredCount;
        private long _lastKeepaliveTicks = long.MinValue;
        private long _lastEchoOperatorTicks = long.MinValue;
        private long _lastFrameArrivalTicks = long.MinValue;

        public SerialChainJointLink(ITransport transport, ITimeAuthority clock, SerialChainProfile profile, ClockSyncConfig clockSyncConfig,
            IReadOnlyList<JointPredictorPreset> presets, int activePreset)
        {
            _transport = transport ?? throw new ArgumentNullException(nameof(transport));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            _profile = profile ?? throw new ArgumentNullException(nameof(profile));
            if (presets == null || presets.Count == 0) throw new ArgumentException("no joint predictor presets", nameof(presets));
            _clockSync = new ClockSync(clockSyncConfig);
            _bank = JointPredictorBank.Create(presets, activePreset);
            _keepaliveIntervalTicks = (long)(KeepaliveSeconds * clock.TicksPerSecond);
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

                Array.Copy(_received, _measured, jointCount);
                _measuredCount = jointCount;
                if (IsSynced)
                {
                    // No reading yet means the values are the host's beliefs from the moment of sending.
                    long robotTime = sensed != 0 ? sensed : robotSend;
                    _bank.ObserveState(_clockSync.ToOperatorTicks(robotTime, robotTps, _clock.TicksPerSecond), _measured.AsSpan(0, jointCount));
                }
            }
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
