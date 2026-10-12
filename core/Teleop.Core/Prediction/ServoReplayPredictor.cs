using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// <c>servo-replay</c> (docs/adr/0018): predicts a servo arm's joints by starting from the last
    /// measurement and replaying every command that has taken effect since. Each arm joint moves
    /// toward its latest effective target at the speed that command asked for, capped at
    /// <see cref="JointPredictorConfig.MaxJointSpeedRadPerSec"/> (the servo's own limit; a command
    /// that asked for no speed moves at that limit). An instant joint (a gripper) takes its target as
    /// soon as the command takes effect.
    ///
    /// <para><b>Model.</b> A bus servo given a position target drives to it at a bounded speed and
    /// stops. That one speed is the whole model, and it is a parameter to be fitted from recordings.
    /// The target a joint was moving toward when it was measured is the newest command that took
    /// effect at or before the measurement; if that command has already left the history ring, the
    /// joint holds until the next one.</para>
    ///
    /// <para><b>Robustness.</b> A measurement older than the current one is rejected; a command older
    /// than the newest one is rejected (both counted in <see cref="Diagnostics"/>). A prediction is
    /// never earlier than the measurement, and never more than
    /// <see cref="JointPredictorConfig.MaxHorizonTicks"/> past it. All storage is allocated once, in
    /// the constructor.</para>
    /// </summary>
    public sealed class ServoReplayPredictor : ICommandAwarePredictor
    {
        private readonly JointPredictorConfig _config;
        private readonly int _maxJoints;
        private readonly int _capacity;

        // The last measurement.
        private readonly byte[] _ids;
        private readonly float[] _measured;
        private readonly bool[] _sensed;
        private readonly bool[] _instant;
        private int _jointCount;
        private bool _hasState;
        private long _stateTicks;

        // The command history: a ring of (effective ticks, targets).
        private readonly long[] _commandTicks;
        private readonly float[] _commandSpeed;
        private readonly int[] _commandCounts;
        private readonly byte[] _commandIds;
        private readonly float[] _commandValues;
        private int _commandHead;   // next slot to write
        private int _commandSize;
        private long _lastCommandTicks;

        // Replay scratch.
        private readonly float[] _work;
        private readonly float[] _target;
        private readonly bool[] _hasTarget;
        private float _speed;

        private int _accepted;
        private int _rejected;
        private long _lastHorizon;

        public ServoReplayPredictor(JointPredictorConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _maxJoints = config.MaxJoints;
            _capacity = config.CommandHistoryCapacity;
            _ids = new byte[_maxJoints];
            _measured = new float[_maxJoints];
            _sensed = new bool[_maxJoints];
            _instant = new bool[_maxJoints];
            _commandTicks = new long[_capacity];
            _commandSpeed = new float[_capacity];
            _commandCounts = new int[_capacity];
            _commandIds = new byte[_capacity * _maxJoints];
            _commandValues = new float[_capacity * _maxJoints];
            _work = new float[_maxJoints];
            _target = new float[_maxJoints];
            _hasTarget = new bool[_maxJoints];
            Reset();
        }

        public PredictorDiagnostics Diagnostics =>
            new PredictorDiagnostics(_lastHorizon, _hasState ? _stateTicks : 0, _accepted, _rejected);

        public void Reset()
        {
            _jointCount = 0;
            _hasState = false;
            _stateTicks = 0;
            _commandHead = 0;
            _commandSize = 0;
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
            int slot = _commandHead;
            int n = Math.Min(targets.Length, _maxJoints);
            int baseIndex = slot * _maxJoints;
            for (int i = 0; i < n; i++)
            {
                _commandIds[baseIndex + i] = targets[i].MotorId;
                _commandValues[baseIndex + i] = targets[i].Value;
            }

            _commandTicks[slot] = effectiveTicks;
            _commandSpeed[slot] = speedRadPerSec > 0f && speedRadPerSec < _config.MaxJointSpeedRadPerSec
                ? speedRadPerSec
                : _config.MaxJointSpeedRadPerSec;
            _commandCounts[slot] = n;
            _commandHead = (_commandHead + 1) % _capacity;
            if (_commandSize < _capacity) _commandSize++;
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
            for (int i = 0; i < n; i++)
            {
                _ids[i] = joints[i].MotorId;
                _measured[i] = joints[i].Value;
                _sensed[i] = joints[i].Sensed;
                _instant[i] = IsInstant(joints[i].MotorId);
            }

            _jointCount = n;
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

            for (int j = 0; j < _jointCount; j++)
            {
                _work[j] = _measured[j];
                _hasTarget[j] = false;
            }

            _speed = _config.MaxJointSpeedRadPerSec;

            long cursor = _stateTicks;
            int oldest = (_commandHead - _commandSize + _capacity) % _capacity;
            for (int k = 0; k < _commandSize; k++)
            {
                int slot = (oldest + k) % _capacity;
                long at = _commandTicks[slot];
                if (at > end)
                {
                    break;
                }

                if (at > cursor)
                {
                    Advance(cursor, at);
                    cursor = at;
                }

                ApplyTargets(slot);
            }

            Advance(cursor, end);

            count = Math.Min(_jointCount, output.Length);
            for (int j = 0; j < count; j++)
            {
                output[j] = new JointSample(_ids[j], _work[j], _sensed[j]);
            }

            return true;
        }

        /// <summary>Takes a command's targets as the joints' current targets. Instant joints jump to theirs.</summary>
        private void ApplyTargets(int slot)
        {
            int baseIndex = slot * _maxJoints;
            _speed = _commandSpeed[slot];
            for (int i = 0; i < _commandCounts[slot]; i++)
            {
                int j = IndexOf(_commandIds[baseIndex + i]);
                if (j < 0)
                {
                    continue;
                }

                _target[j] = _commandValues[baseIndex + i];
                _hasTarget[j] = true;
                if (_instant[j])
                {
                    _work[j] = _target[j];
                }
            }
        }

        /// <summary>Moves every joint toward its target for the interval [from, to).</summary>
        private void Advance(long from, long to)
        {
            if (to <= from)
            {
                return;
            }

            float step = _speed * (float)((double)(to - from) / _config.TicksPerSecond);
            for (int j = 0; j < _jointCount; j++)
            {
                if (!_hasTarget[j] || _instant[j])
                {
                    continue;
                }

                float delta = _target[j] - _work[j];
                _work[j] = Math.Abs(delta) <= step ? _target[j] : _work[j] + Math.Sign(delta) * step;
            }
        }

        private int IndexOf(byte motorId)
        {
            for (int j = 0; j < _jointCount; j++)
            {
                if (_ids[j] == motorId) return j;
            }

            return -1;
        }

        private bool IsInstant(byte motorId)
        {
            byte[] instant = _config.InstantMotorIds;
            for (int i = 0; i < instant.Length; i++)
            {
                if (instant[i] == motorId) return true;
            }

            return false;
        }
    }
}
