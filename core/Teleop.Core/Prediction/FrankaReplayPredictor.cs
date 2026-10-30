using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// <c>franka-replay</c> (docs/adr/0027 decision 4): predicts a Franka arm's joints by replaying the commands
    /// sent through the same <see cref="JointTrajectoryShaper"/> the robot's backend drives the arm with,
    /// re-anchored on each measurement. On the JetRover the robot's behaviour between commands was someone
    /// else's relay, which <c>relay-replay</c> models. On the FR3 it is this repo's shaper, so the prior can be
    /// exact up to the arm's own tracking and the network.
    ///
    /// <para><b>The replay.</b> Each shaped motor (one listed in <see cref="JointPredictorConfig.Shaper"/>)
    /// keeps a shaped state at its anchor (position, velocity, acceleration, and a finger's move) and a ring of
    /// the targets commanded, with their effective times. A prediction copies the anchor state and steps the
    /// shaper from the anchor to the target time, at the shaper's step. The step from t to t + step steers
    /// toward the target in force at t: the newest command whose effective time is not later than t. A last
    /// partial step is interpolated linearly between two whole steps, so a prediction moves smoothly with the
    /// target time.</para>
    ///
    /// <para><b>Re-anchoring.</b> A measurement at <c>s</c> advances the motor's anchor state from the
    /// previous anchor to <c>s</c> the same way, then replaces the position with the measured one, keeping the
    /// replayed velocity, acceleration and move: a measurement says where the arm is, not how fast it is going.
    /// The shaper steers to the commanded target from any state, so a position disagreement is absorbed by the
    /// replay rather than carried, and the prediction always ends at the commanded target. An anchor more than
    /// <see cref="JointPredictorConfig.MaxHorizonTicks"/> old is taken as settled: at rest where measured.
    /// Before a motor's first command, its target is its first measured value (hold where it is).</para>
    ///
    /// <para><b>Other motors.</b> A motor the shaper does not list keeps its last measured value, as
    /// <c>hold-last</c> does. The output carries every measured joint, in the order last observed, with its
    /// <see cref="JointSample.Sensed"/> flag.</para>
    ///
    /// <para><b>Robustness.</b> A measurement older than the current one, and a command older than the newest,
    /// are rejected and counted in <see cref="Diagnostics"/>. A prediction is never earlier than the
    /// measurement and never more than <see cref="JointPredictorConfig.MaxHorizonTicks"/> past it. Each motor
    /// remembers its last <see cref="JointPredictorConfig.CommandHistoryCapacity"/> targets; the newest one
    /// pushed out stays in force before the oldest kept. All storage is allocated once, in the constructor.</para>
    /// </summary>
    public sealed class FrankaReplayPredictor : ICommandAwarePredictor
    {
        private readonly JointPredictorConfig _config;
        private readonly TrajectoryShaperConfig _shaper;
        private readonly int _maxJoints;
        private readonly int _capacity;
        private readonly long _stepTicks;

        // Per motor, keyed by id in the order first seen (commanded or measured).
        private readonly byte[] _slotIds;
        private readonly int[] _limitIndex;
        private readonly long[] _commandTicks;
        private readonly double[] _commandValue;
        private readonly int[] _commandHead;
        private readonly int[] _commandSize;
        private readonly bool[] _hasBaseTarget;
        private readonly double[] _baseTarget;
        private readonly ShapedJointState[] _anchorState;
        private readonly long[] _anchorTicks;
        private readonly bool[] _hasAnchor;
        private int _slotCount;

        // The last measurement, in the order observed.
        private readonly byte[] _ids;
        private readonly int[] _slotOf;
        private readonly float[] _measured;
        private readonly bool[] _sensed;
        private int _jointCount;
        private bool _hasState;
        private long _stateTicks;
        private long _lastCommandTicks;

        private int _accepted;
        private int _rejected;
        private long _lastHorizon;

        public FrankaReplayPredictor(JointPredictorConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _shaper = config.Shaper ?? throw new ArgumentException(
                "franka-replay needs the robot's trajectory shaper (JointPredictorConfig.Shaper, built from its profile)", nameof(config));
            _maxJoints = config.MaxJoints;
            _capacity = config.CommandHistoryCapacity;
            _stepTicks = Math.Max(1L, (long)Math.Round(_shaper.StepSeconds * config.TicksPerSecond));

            _slotIds = new byte[_maxJoints];
            _limitIndex = new int[_maxJoints];
            _commandTicks = new long[_maxJoints * _capacity];
            _commandValue = new double[_maxJoints * _capacity];
            _commandHead = new int[_maxJoints];
            _commandSize = new int[_maxJoints];
            _hasBaseTarget = new bool[_maxJoints];
            _baseTarget = new double[_maxJoints];
            _anchorState = new ShapedJointState[_maxJoints];
            _anchorTicks = new long[_maxJoints];
            _hasAnchor = new bool[_maxJoints];

            _ids = new byte[_maxJoints];
            _slotOf = new int[_maxJoints];
            _measured = new float[_maxJoints];
            _sensed = new bool[_maxJoints];
            Reset();
        }

        public PredictorDiagnostics Diagnostics =>
            new PredictorDiagnostics(_lastHorizon, _hasState ? _stateTicks : 0, _accepted, _rejected);

        /// <summary>Whether a state has been observed, and when it was sensed: layout v4's window measures this prior's error against each new reading from it.</summary>
        public bool HasState => _hasState;

        public long StateTicks => _stateTicks;

        public void Reset()
        {
            _slotCount = 0;
            _jointCount = 0;
            _hasState = false;
            _stateTicks = 0;
            _lastCommandTicks = long.MinValue;
            _accepted = 0;
            _rejected = 0;
            _lastHorizon = 0;
        }

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets, float speedRadPerSec)
        {
            if (effectiveTicks < _lastCommandTicks)
            {
                _rejected++;
                return;
            }

            _lastCommandTicks = effectiveTicks;
            for (int i = 0; i < targets.Length; i++)
            {
                int s = Slot(targets[i].MotorId);
                if (s < 0 || _limitIndex[s] < 0) continue;

                int head = s * _capacity + _commandHead[s];
                if (_commandSize[s] == _capacity)
                {
                    // The oldest kept is pushed out; it stays in force before the new oldest.
                    _hasBaseTarget[s] = true;
                    _baseTarget[s] = _commandValue[head];
                }
                else
                {
                    _commandSize[s]++;
                }

                _commandTicks[head] = effectiveTicks;
                _commandValue[head] = targets[i].Value;
                _commandHead[s] = (_commandHead[s] + 1) % _capacity;
            }

            _accepted++;
        }

        public void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints)
        {
            if (_hasState && sensedTicks < _stateTicks)
            {
                _rejected++;
                return;
            }

            int n = Math.Min(joints.Length, _maxJoints);
            int kept = 0;
            for (int i = 0; i < n; i++)
            {
                JointSample joint = joints[i];
                int s = Slot(joint.MotorId);
                if (s < 0) continue;

                if (_limitIndex[s] >= 0)
                {
                    ShapedJointLimits limits = _shaper[_limitIndex[s]];
                    ShapedJointState state;
                    if (!_hasAnchor[s])
                    {
                        state = ShapedJointState.AtRest(joint.Value);
                        if (!_hasBaseTarget[s])
                        {
                            // Until a command is in force, the motor holds where it was first measured.
                            _hasBaseTarget[s] = true;
                            _baseTarget[s] = joint.Value;
                        }
                    }
                    else if (sensedTicks - _anchorTicks[s] > _config.MaxHorizonTicks)
                    {
                        state = ShapedJointState.AtRest(joint.Sensed ? joint.Value : _anchorState[s].Q);
                        state.LastTo = TargetAt(s, sensedTicks, out _);
                    }
                    else
                    {
                        state = _anchorState[s];
                        Advance(s, limits, ref state, _anchorTicks[s], sensedTicks);
                    }

                    if (joint.Sensed) state.Q = joint.Value;
                    _anchorState[s] = state;
                    _anchorTicks[s] = sensedTicks;
                    _hasAnchor[s] = true;
                }

                _ids[kept] = joint.MotorId;
                _slotOf[kept] = s;
                _measured[kept] = joint.Value;
                _sensed[kept] = joint.Sensed;
                kept++;
            }

            _jointCount = kept;
            _stateTicks = sensedTicks;
            _hasState = true;
            _accepted++;
        }

        public bool TryPredict(long targetTicks, Span<JointSample> output, out int count)
        {
            count = 0;
            if (!_hasState)
            {
                return false;
            }

            long end = targetTicks;
            if (end < _stateTicks) end = _stateTicks;
            if (end > _stateTicks + _config.MaxHorizonTicks) end = _stateTicks + _config.MaxHorizonTicks;
            _lastHorizon = end - _stateTicks;

            count = Math.Min(_jointCount, output.Length);
            for (int j = 0; j < count; j++)
            {
                int s = _slotOf[j];
                float value = _measured[j];
                if (_limitIndex[s] >= 0 && _hasAnchor[s])
                {
                    ShapedJointState state = _anchorState[s];
                    Advance(s, _shaper[_limitIndex[s]], ref state, _anchorTicks[s], end);
                    value = (float)state.Q;
                }

                output[j] = new JointSample(_ids[j], value, _sensed[j]);
            }

            return true;
        }

        /// <summary>
        /// Steps <paramref name="state"/> from <paramref name="from"/> to <paramref name="to"/>: whole shaper steps,
        /// then a last partial one interpolated between two whole steps. The interpolation covers position,
        /// velocity and acceleration; a finger's move flags stay those of the last whole step.
        /// </summary>
        private void Advance(int s, in ShapedJointLimits limits, ref ShapedJointState state, long from, long to)
        {
            if (to <= from) return;

            long span = to - from;
            long whole = span / _stepTicks;
            long remainder = span - whole * _stepTicks;
            int cursor;
            double target = TargetAt(s, from, out cursor);
            for (long k = 0; k < whole; k++)
            {
                target = TargetFrom(s, from + k * _stepTicks, ref cursor, target);
                JointTrajectoryShaper.Step(ref state, target, limits, _shaper);
            }

            if (remainder > 0)
            {
                target = TargetFrom(s, from + whole * _stepTicks, ref cursor, target);
                ShapedJointState next = state;
                JointTrajectoryShaper.Step(ref next, target, limits, _shaper);
                double u = (double)remainder / _stepTicks;
                state.Q += (next.Q - state.Q) * u;
                state.V += (next.V - state.V) * u;
                state.A += (next.A - state.A) * u;
            }
        }

        /// <summary>The target in force at <paramref name="t"/>, and the ring position just past the command that set it (0 when none did).</summary>
        private double TargetAt(int s, long t, out int cursor)
        {
            cursor = 0;
            double target = _hasBaseTarget[s] ? _baseTarget[s] : _anchorState[s].Q;
            return TargetFrom(s, t, ref cursor, target);
        }

        /// <summary>Moves <paramref name="cursor"/> (an offset from the oldest command kept) past every command in force by <paramref name="t"/>.</summary>
        private double TargetFrom(int s, long t, ref int cursor, double target)
        {
            int size = _commandSize[s];
            int oldest = (_commandHead[s] - size + _capacity) % _capacity;
            int baseIndex = s * _capacity;
            while (cursor < size)
            {
                int slot = baseIndex + (oldest + cursor) % _capacity;
                if (_commandTicks[slot] > t) break;
                target = _commandValue[slot];
                cursor++;
            }

            return target;
        }

        /// <summary>The motor's slot, created on first sight; -1 once every slot is taken.</summary>
        private int Slot(byte motorId)
        {
            for (int s = 0; s < _slotCount; s++)
            {
                if (_slotIds[s] == motorId) return s;
            }

            if (_slotCount == _maxJoints) return -1;

            int created = _slotCount++;
            _slotIds[created] = motorId;
            _limitIndex[created] = _shaper.IndexOf(motorId);
            _commandHead[created] = 0;
            _commandSize[created] = 0;
            _hasBaseTarget[created] = false;
            _baseTarget[created] = 0;
            _hasAnchor[created] = false;
            _anchorTicks[created] = 0;
            _anchorState[created] = default;
            return created;
        }
    }
}
