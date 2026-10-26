using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Recording;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Power
{
    /// <summary>
    /// <see cref="ArmPowerController"/>'s port on the real robot: readings and waypoints through the
    /// <see cref="GenericArmPlant"/>, torque through the relay, the status to the joint socket's latest
    /// sender, and every change to the console and the session log (docs/adr/0023 decision 5).
    /// </summary>
    public sealed class PlantArmPowerPort : IArmPowerPort
    {
        private readonly GenericArmPlant _plant;
        private readonly IRelayClient _relay;
        private readonly ITransport _joint;
        private readonly SessionRecorder? _recorder;
        private readonly Action<string> _log;
        private readonly int[] _armIndexes;
        private readonly JointSample[] _samples;
        private readonly long[] _ticks;
        private readonly byte[] _statusBuffer = new byte[ArmPowerCodec.StatusSize];

        public PlantArmPowerPort(GenericArmPlant plant, in RobotArmProfile profile, IRelayClient relay, ITransport jointTransport,
            SessionRecorder? recorder, Action<string> log)
        {
            _plant = plant ?? throw new ArgumentNullException(nameof(plant));
            _relay = relay ?? throw new ArgumentNullException(nameof(relay));
            _joint = jointTransport ?? throw new ArgumentNullException(nameof(jointTransport));
            _recorder = recorder;
            _log = log ?? throw new ArgumentNullException(nameof(log));
            int n = 0;
            foreach (JointHardwareSpec j in profile.Joints) if (j.Role != JointRole.GripperMain) n++;
            _armIndexes = new int[n];
            n = 0;
            for (int i = 0; i < profile.Joints.Length; i++) if (profile.Joints[i].Role != JointRole.GripperMain) _armIndexes[n++] = i;
            _samples = new JointSample[profile.JointCount];
            _ticks = new long[profile.JointCount];
        }

        public void ReadArm(Span<float> angles, Span<long> sensedTicks)
        {
            _plant.ReadJointState(_samples);
            _plant.ReadSensedTicks(_ticks);
            for (int i = 0; i < _armIndexes.Length; i++)
            {
                angles[i] = _samples[_armIndexes[i]].Value;
                sensedTicks[i] = _ticks[_armIndexes[i]];
            }
        }

        public JointCommandOutcome CommandArm(ReadOnlySpan<JointTarget> targets, long now)
        {
            if (_recorder != null && _recorder.IsActive) _recorder.Record(SessionLines.Park(now, targets));
            // Stamped on the robot's clock; the ordering restarts on every change of state, so neither
            // the park nor the operator's commands after it are judged against the other's clock.
            return _plant.CommandJointAngles(targets, now);
        }

        public void SetTorque(bool on, long now)
        {
            // Recorded (TQ) by the recording relay client, when there is one.
            _relay.SendTorque(on);
            _log($"[power] torque {(on ? "on" : "off")} for every servo, sent to the relay");
        }

        public void SendStatus(ArmPowerState state, ArmPowerReason reason, uint echoSequence, long echoOperatorTicks, float remainingDeg, long now)
        {
            if (ArmPowerCodec.TryEncodeStatus(state, reason, echoSequence, echoOperatorTicks, now, remainingDeg, _statusBuffer, out int n))
            {
                _joint.Send(_statusBuffer.AsSpan(0, n), now);
            }
        }

        public void StateChanged(long now, ArmPowerState from, ArmPowerState to, ArmPowerReason reason, string detail)
        {
            // A new clock on the joint path either way: the park's own stamps, or the operator's again.
            _plant.RestartJointCommandOrdering();
            if (_recorder != null && _recorder.IsActive) _recorder.Record(SessionLines.PowerState(now, to, reason));
            string what = from == to
                ? $"relax refused: {ArmPowerCodec.Describe(reason)}"
                : $"{SessionLines.StateName(from)} -> {SessionLines.StateName(to)}" +
                  (reason != ArmPowerReason.None ? $" ({ArmPowerCodec.Describe(reason)})" : "");
            _log($"[power] {what}{(detail.Length > 0 ? ": " + detail : "")}");
        }
    }
}
