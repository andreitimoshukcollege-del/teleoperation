using System;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Backend;

namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// The robot side of a serial-chain arm, such as the FR3 Duo (docs/adr/0027 decision 3): joint commands in,
    /// through staleness and limits, to an <see cref="IArmBackend"/>; the backend's measured state out, for the
    /// joint-state downlink. In radians throughout. The JetRover's pulse-based <see cref="GenericArmPlant"/>
    /// is not generalized.
    ///
    /// <para><b>Staleness:</b> a command whose capture stamp is not newer than the last one taken is dropped
    /// (<see cref="JointCommandOutcome.Stale"/>), as <see cref="GenericArmPlant"/> does. A new sender's stamps
    /// are another clock's, so the host restarts the check (<see cref="RestartJointCommandOrdering"/>).</para>
    /// <para><b>Limits:</b> an arm joint's target is clamped into its soft limits, a hand's into 0..1. The
    /// backend's shaper keeps its own margin inside them. A motor the profile does not have is ignored and
    /// counted, not thrown, so a stale or foreign sender cannot crash the host.</para>
    /// <para><b>Gap policy</b> (decision 2): every accepted command's targets stay in force until the next one,
    /// so with no new command the arm finishes its move under the shaper's limits and stays there.</para>
    /// </summary>
    public sealed class SerialChainPlant
    {
        private readonly SerialChainProfile _profile;
        private readonly IArmBackend _backend;
        private readonly JointSample[] _targets;
        private readonly JointSample[] _scratch;
        private readonly double[] _lower;
        private readonly double[] _upper;
        private readonly byte[] _motorIds;
        private long _newestCaptureTicks = long.MinValue;

        public SerialChainPlant(SerialChainProfile profile, IArmBackend backend)
        {
            _profile = profile ?? throw new ArgumentNullException(nameof(profile));
            _backend = backend ?? throw new ArgumentNullException(nameof(backend));
            string? invalid = profile.Validate();
            if (invalid != null) throw new ArgumentException(invalid, nameof(profile));

            int n = profile.MotorCount;
            if (n > JointStateCodec.MaxJointsPerMessage) throw new ArgumentException($"{n} motors do not fit one joint-state frame ({JointStateCodec.MaxJointsPerMessage})", nameof(profile));
            _targets = new JointSample[n];
            _scratch = new JointSample[n];
            _lower = new double[n];
            _upper = new double[n];
            _motorIds = new byte[n];
            int k = 0;
            foreach (ChainArm arm in profile.Arms)
            {
                foreach (ChainJoint joint in arm.Joints)
                {
                    _motorIds[k] = joint.MotorId;
                    _lower[k] = joint.Limits.SoftLower;
                    _upper[k] = joint.Limits.SoftUpper;
                    k++;
                }

                if (arm.HasGripper)
                {
                    _motorIds[k] = arm.Gripper.MotorId;
                    _lower[k] = 0;
                    _upper[k] = 1;
                    k++;
                }
            }
        }

        /// <summary>Joint commands taken, dropped as stale, and targets for motors the profile does not have.</summary>
        public int AcceptedCommands { get; private set; }
        public int StaleCommands { get; private set; }
        public int UnknownTargets { get; private set; }

        /// <summary>Targets clamped into their limits.</summary>
        public int ClampedTargets { get; private set; }

        /// <summary>When the newest state the backend reported was measured, on the host's clock.</summary>
        public long LastFeedbackTicks { get; private set; }

        public int MotorCount => _motorIds.Length;

        public void RestartJointCommandOrdering() => _newestCaptureTicks = long.MinValue;

        /// <summary>
        /// Takes a joint command captured at <paramref name="captureTicks"/> (the sender's clock). The targets it
        /// carries replace those in force for their motors and go to the backend now; motors it leaves out keep
        /// theirs. The targets as sent are left in <see cref="LastSent"/>.
        /// </summary>
        public JointCommandOutcome CommandJointTargets(ReadOnlySpan<JointTarget> targets, long captureTicks, long nowTicks)
        {
            if (captureTicks <= _newestCaptureTicks)
            {
                StaleCommands++;
                return JointCommandOutcome.Stale;
            }

            _newestCaptureTicks = captureTicks;
            int count = 0;
            for (int i = 0; i < targets.Length; i++)
            {
                int m = Array.IndexOf(_motorIds, targets[i].MotorId);
                if (m < 0)
                {
                    UnknownTargets++;
                    continue;
                }

                double value = targets[i].Angle;
                if (double.IsNaN(value)) continue;
                double clamped = value < _lower[m] ? _lower[m] : value > _upper[m] ? _upper[m] : value;
                if (clamped != value) ClampedTargets++;
                _scratch[count++] = new JointSample(_motorIds[m], (float)clamped, false);
            }

            _backend.Send(_scratch.AsSpan(0, count), nowTicks);
            LastSentCount = count;
            Array.Copy(_scratch, _targets, count);
            AcceptedCommands++;
            return JointCommandOutcome.Accepted;
        }

        /// <summary>The targets the last accepted command sent to the backend.</summary>
        public ReadOnlySpan<JointSample> LastSent => _targets.AsSpan(0, LastSentCount);

        public int LastSentCount { get; private set; }

        /// <summary>The backend's newest measured state into <paramref name="buffer"/>, for the joint-state frame.</summary>
        public int ReadJointState(long nowTicks, JointSample[] buffer)
        {
            int n = _backend.ReadState(nowTicks, buffer, out long sensed);
            if (n > 0) LastFeedbackTicks = sensed;
            return n;
        }

        public SerialChainProfile Profile => _profile;
    }
}
