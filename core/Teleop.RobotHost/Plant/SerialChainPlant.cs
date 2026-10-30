using System;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
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
    /// <para><b>Collision</b> (docs/adr/0026 amendment): with a collision model, a command whose arm targets would put the
    /// robot within the model's margin of itself (self, arm to arm, arm to body) is refused
    /// (<see cref="JointCommandOutcome.Refused"/>): its targets never reach the backend, and the arm finishes its last
    /// accepted move instead. Without a model nothing is checked, which only a simulated backend accepts.</para>
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
        private readonly SerialChainCollisionModel? _collision;
        private readonly double[] _armPose;
        private readonly double[] _candidate;
        private readonly int[] _poseIndex;

        /// <param name="initialTargets">Each motor's target before the first command, in profile order (each arm's joints, then its
        /// hand): where the backend starts. The collision check judges a command's targets with the others in force.</param>
        public SerialChainPlant(SerialChainProfile profile, IArmBackend backend, SerialChainCollisionModel? collision = null, double[]? initialTargets = null)
        {
            _profile = profile ?? throw new ArgumentNullException(nameof(profile));
            _backend = backend ?? throw new ArgumentNullException(nameof(backend));
            _collision = collision;
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

            // The arm joints' targets as one robot pose (arm after arm), for the collision check.
            int joints = 0;
            foreach (ChainArm arm in profile.Arms) joints += arm.Joints.Length;
            _armPose = new double[joints];
            _candidate = new double[joints];
            _poseIndex = new int[n];
            int p = 0;
            k = 0;
            foreach (ChainArm arm in profile.Arms)
            {
                for (int j = 0; j < arm.Joints.Length; j++) _poseIndex[k++] = p++;
                if (arm.HasGripper) _poseIndex[k++] = -1;
            }

            if (initialTargets != null)
            {
                if (initialTargets.Length != n) throw new ArgumentException($"one initial target per motor ({n})", nameof(initialTargets));
                for (int m = 0; m < n; m++) if (_poseIndex[m] >= 0) _armPose[_poseIndex[m]] = initialTargets[m];
            }
            else if (collision != null)
            {
                throw new ArgumentException("a collision check needs the initial targets", nameof(initialTargets));
            }
        }

        /// <summary>Commands refused because their pose would collide, and the last such hit.</summary>
        public int RefusedCommands { get; private set; }

        public ChainCollisionHit LastRefusal { get; private set; } = ChainCollisionHit.None;

        /// <summary>The last refusal in words: the two capsules and their distance, or the joint out of range.</summary>
        public string DescribeLastRefusal()
        {
            ChainCollisionHit h = LastRefusal;
            if (!h.IsHit || _collision == null) return "none";
            return h.CapsuleA >= 0
                ? $"{_collision.Capsules[h.CapsuleA].Name} / {_collision.Capsules[h.CapsuleB].Name}, {h.Distance * 1000:0} mm apart (margin {_collision.Margin * 1000:0} mm)"
                : $"arm {h.Arm} joint {h.Joint + 1} outside its soft range";
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

            int count = 0;
            if (_collision != null)
            {
                Array.Copy(_armPose, _candidate, _armPose.Length);
                for (int i = 0; i < targets.Length; i++)
                {
                    int m = Array.IndexOf(_motorIds, targets[i].MotorId);
                    if (m < 0 || _poseIndex[m] < 0 || double.IsNaN(targets[i].Angle)) continue;
                    double v = targets[i].Angle;
                    _candidate[_poseIndex[m]] = v < _lower[m] ? _lower[m] : v > _upper[m] ? _upper[m] : v;
                }

                ChainCollisionHit hit = SerialChainCollision.Check(_profile, _collision, _candidate);
                if (hit.IsHit)
                {
                    // Not taken, so its capture stamp is not either: a later, clear command still counts as newer.
                    RefusedCommands++;
                    LastRefusal = hit;
                    return JointCommandOutcome.Refused;
                }

                Array.Copy(_candidate, _armPose, _armPose.Length);
            }

            _newestCaptureTicks = captureTicks;
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
