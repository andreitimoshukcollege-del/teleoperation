using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// <c>relay-replay</c> (docs/adr/0018): predicts a servo arm's joints by replaying the commands
    /// sent through a model of the robot's relay gate, then re-anchoring that replay on each
    /// measurement. It is <c>servo-replay</c> with a different servo model: instead of "every command
    /// takes effect, and the servo chases the latest target at a constant speed", each servo takes
    /// the commands the relay would actually write, as timed linear moves.
    ///
    /// <para><b>The gate</b>, per servo, the JetRover relay's <c>setPos</c> (robot/ros
    /// servo_controller.py) with newest-wins pending targets:</para>
    /// <list type="bullet">
    /// <item>A command that arrives while the servo is in its write cooldown becomes that servo's
    /// pending target, replacing any older one. When the cooldown ends, the pending target is
    /// offered to the gate at that instant.</item>
    /// <item>A target nearer than <see cref="JointPredictorConfig.RelayDeadbandRad"/> to the last one
    /// written is dropped, and starts no cooldown.</item>
    /// <item>Otherwise the relay writes a linear move from wherever the servo is to the target, lasting
    /// max(<see cref="JointPredictorConfig.RelayMinMoveSeconds"/>, |target − last written| / speed),
    /// and the servo's cooldown lasts as long. The speed is the one the command asked for, capped at
    /// <see cref="JointPredictorConfig.RelayMoveSpeedRadPerSec"/>, which is also the speed when none
    /// was asked for. A servo's first write has no last target and lasts the minimum.</item>
    /// </list>
    /// <para>The gate depends only on the commands and their effective times, so it is run once, as
    /// each command is observed, into a ring of written moves per servo. A pending target whose
    /// cooldown ends before the prediction time is treated as written then, without being stored.</para>
    ///
    /// <para><b>Re-anchoring.</b> A measurement says where the servo was, not which move it was in.
    /// If it lands inside a move, the joint continues from the measured value to that move's target,
    /// arriving when the move ends; if it lands between moves, the joint holds the measured value.
    /// Each later move starts from wherever the replay has the joint at its write time. So a
    /// disagreement between model and measurement is absorbed within the current move, never carried.</para>
    ///
    /// <para><b>Reading age.</b> A robot that reads its servos one after another and stamps the
    /// frame after the last read reports each joint older than the stamp. A sensed joint listed in
    /// <see cref="JointPredictorConfig.SensedAgeMotorIds"/> is anchored that much before the
    /// stamp. By default no age is subtracted.</para>
    ///
    /// <para><b>Other joints.</b> Instant joints (grippers in <see cref="JointPredictorConfig.InstantMotorIds"/>)
    /// take each command's target as soon as it takes effect, with no gate. A scaled joint
    /// (<see cref="JointPredictorConfig.ScaledMotorIds"/>, a value not in radians) has its deadband
    /// and speed multiplied by its scale. A joint the robot does not measure keeps this model's own
    /// estimate across measurements, as in <c>servo-replay</c>.</para>
    ///
    /// <para><b>Robustness.</b> A measurement older than the current one, and a command older than
    /// the newest one, are rejected and counted in <see cref="Diagnostics"/>. A prediction is never
    /// earlier than the measurement and never more than <see cref="JointPredictorConfig.MaxHorizonTicks"/>
    /// past it. Each servo remembers its last <see cref="JointPredictorConfig.CommandHistoryCapacity"/>
    /// moves; a measurement older than all of them holds until the oldest. All storage is allocated
    /// once, in the constructor.</para>
    /// </summary>
    public sealed class RelayReplayPredictor : ICommandAwarePredictor
    {
        /// <summary>What the gate has done with the commands since construction or the last <see cref="Reset"/>.</summary>
        public readonly struct GateCounts
        {
            /// <summary>Moves written (gated joints only).</summary>
            public readonly int Written;

            /// <summary>Targets dropped for being within the deadband of the last one written.</summary>
            public readonly int DeadbandDrops;

            /// <summary>Pending targets replaced by a newer one before their cooldown ended, so never written.</summary>
            public readonly int Coalesced;

            public GateCounts(int written, int deadbandDrops, int coalesced)
            {
                Written = written;
                DeadbandDrops = deadbandDrops;
                Coalesced = coalesced;
            }
        }

        private readonly JointPredictorConfig _config;
        private readonly int _maxJoints;
        private readonly int _capacity;
        private readonly long _minMoveTicks;
        private readonly float _maxMoveSpeed;

        // The last measurement, per joint in the order observed.
        private readonly byte[] _ids;
        private readonly float[] _measured;
        private readonly long[] _anchor;
        private readonly bool[] _sensed;
        private int _jointCount;
        private bool _hasState;
        private long _stateTicks;

        // The gate, per servo, keyed by motor id in the order first commanded.
        private readonly byte[] _gateIds;
        private readonly bool[] _gateInstant;
        private readonly float[] _gateScale;
        private readonly float[] _lastSent;
        private readonly bool[] _hasLastSent;
        private readonly long[] _cooldownEnd;
        private readonly bool[] _hasPending;
        private readonly float[] _pendingTarget;
        private readonly float[] _pendingSpeed;
        private int _gateCount;
        private long _lastCommandTicks;

        // The moves written, a ring per servo: [servo * capacity + slot].
        private readonly long[] _moveStart;
        private readonly long[] _moveEnd;
        private readonly float[] _moveTarget;
        private readonly int[] _moveHead;
        private readonly int[] _moveSize;

        // ObserveState scratch.
        private readonly JointSample[] _carried;
        private readonly byte[] _scratchIds;
        private readonly float[] _scratchValues;
        private readonly long[] _scratchAnchor;
        private readonly bool[] _scratchSensed;

        private int _accepted;
        private int _rejected;
        private long _lastHorizon;
        private int _written;
        private int _deadbandDrops;
        private int _coalesced;

        public RelayReplayPredictor(JointPredictorConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _maxJoints = config.MaxJoints;
            _capacity = config.CommandHistoryCapacity;
            _minMoveTicks = (long)Math.Round(config.RelayMinMoveSeconds * (double)config.TicksPerSecond);
            _maxMoveSpeed = config.RelayMoveSpeedRadPerSec;

            _ids = new byte[_maxJoints];
            _measured = new float[_maxJoints];
            _anchor = new long[_maxJoints];
            _sensed = new bool[_maxJoints];

            _gateIds = new byte[_maxJoints];
            _gateInstant = new bool[_maxJoints];
            _gateScale = new float[_maxJoints];
            _lastSent = new float[_maxJoints];
            _hasLastSent = new bool[_maxJoints];
            _cooldownEnd = new long[_maxJoints];
            _hasPending = new bool[_maxJoints];
            _pendingTarget = new float[_maxJoints];
            _pendingSpeed = new float[_maxJoints];

            _moveStart = new long[_maxJoints * _capacity];
            _moveEnd = new long[_maxJoints * _capacity];
            _moveTarget = new float[_maxJoints * _capacity];
            _moveHead = new int[_maxJoints];
            _moveSize = new int[_maxJoints];

            _carried = new JointSample[_maxJoints];
            _scratchIds = new byte[_maxJoints];
            _scratchValues = new float[_maxJoints];
            _scratchAnchor = new long[_maxJoints];
            _scratchSensed = new bool[_maxJoints];
            Reset();
        }

        public PredictorDiagnostics Diagnostics =>
            new PredictorDiagnostics(_lastHorizon, _hasState ? _stateTicks : 0, _accepted, _rejected);

        /// <summary>Whether a state has been observed, and when it was sensed: the learned predictor's window
        /// measures this model's error against each new reading from it (docs/adr/0025).</summary>
        public bool HasState => _hasState;

        public long StateTicks => _stateTicks;

        /// <summary>The gate's counts: moves written, deadband drops, coalesced targets.</summary>
        public GateCounts Gate => new GateCounts(_written, _deadbandDrops, _coalesced);

        public void Reset()
        {
            _jointCount = 0;
            _hasState = false;
            _stateTicks = 0;
            _gateCount = 0;
            _lastCommandTicks = long.MinValue;
            _accepted = 0;
            _rejected = 0;
            _lastHorizon = 0;
            _written = 0;
            _deadbandDrops = 0;
            _coalesced = 0;
        }

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets, float speedRadPerSec)
        {
            if (effectiveTicks < _lastCommandTicks)
            {
                _rejected++;
                return;
            }

            _lastCommandTicks = effectiveTicks;
            float speed = speedRadPerSec > 0f && speedRadPerSec < _maxMoveSpeed ? speedRadPerSec : _maxMoveSpeed;
            for (int i = 0; i < targets.Length; i++)
            {
                int g = GateSlot(targets[i].MotorId);
                if (g < 0)
                {
                    continue;
                }

                float target = targets[i].Value;
                if (_gateInstant[g])
                {
                    AppendMove(g, effectiveTicks, effectiveTicks, target);
                    continue;
                }

                // The pending target is offered to the gate when its cooldown ends, before this one.
                if (_hasPending[g] && _cooldownEnd[g] <= effectiveTicks)
                {
                    _hasPending[g] = false;
                    TryWrite(g, _cooldownEnd[g], _pendingTarget[g], _pendingSpeed[g]);
                }

                if (effectiveTicks >= _cooldownEnd[g])
                {
                    TryWrite(g, effectiveTicks, target, speed);
                }
                else
                {
                    if (_hasPending[g]) _coalesced++;
                    _hasPending[g] = true;
                    _pendingTarget[g] = target;
                    _pendingSpeed[g] = speed;
                }
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

            // A joint the robot does not measure reports only what it was last told. Keep this
            // model's own estimate for it, replayed up to this measurement's time, as servo-replay does.
            bool carry = _hasState;
            if (carry)
            {
                long horizon = _lastHorizon;
                TryPredict(sensedTicks, _carried, out _);
                _lastHorizon = horizon;
            }

            int n = Math.Min(joints.Length, _maxJoints);
            for (int i = 0; i < n; i++)
            {
                byte id = joints[i].MotorId;
                bool instant = IsInstant(id);
                bool sensed = joints[i].Sensed;
                float value = joints[i].Value;
                long anchor = sensedTicks;
                if (sensed && !instant)
                {
                    anchor -= SensedAgeTicks(id);
                }
                else if (carry && !instant)
                {
                    int previous = IndexOf(id);
                    if (previous >= 0)
                    {
                        value = _carried[previous].Value;
                    }
                }

                _scratchIds[i] = id;
                _scratchValues[i] = value;
                _scratchAnchor[i] = anchor;
                _scratchSensed[i] = sensed;
            }

            for (int i = 0; i < n; i++)
            {
                _ids[i] = _scratchIds[i];
                _measured[i] = _scratchValues[i];
                _anchor[i] = _scratchAnchor[i];
                _sensed[i] = _scratchSensed[i];
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

            count = Math.Min(_jointCount, output.Length);
            for (int j = 0; j < count; j++)
            {
                output[j] = new JointSample(_ids[j], PredictJoint(j, end), _sensed[j]);
            }

            return true;
        }

        private float PredictJoint(int j, long end)
        {
            int g = GateIndex(_ids[j]);
            if (g < 0)
            {
                return _measured[j];
            }

            int size = _moveSize[g];
            int oldest = (_moveHead[g] - size + _capacity) % _capacity;
            int baseIndex = g * _capacity;

            if (_gateInstant[g])
            {
                float value = _measured[j];
                for (int k = 0; k < size; k++)
                {
                    int slot = baseIndex + (oldest + k) % _capacity;
                    if (_moveStart[slot] > end) break;
                    value = _moveTarget[slot];
                }

                return value;
            }

            var replay = new JointReplay(_anchor[j], _measured[j], end);
            for (int k = 0; k < size; k++)
            {
                int slot = baseIndex + (oldest + k) % _capacity;
                if (!replay.Visit(_moveStart[slot], _moveEnd[slot], _moveTarget[slot]))
                {
                    return replay.Finish();
                }
            }

            // A pending target whose cooldown ends in time is written then (unless the deadband drops it).
            if (_hasPending[g] && _cooldownEnd[g] <= end)
            {
                long at = _cooldownEnd[g];
                if (Passes(g, _pendingTarget[g], out float distance))
                {
                    replay.Visit(at, at + MoveTicks(g, distance, _pendingSpeed[g]), _pendingTarget[g]);
                }
            }

            return replay.Finish();
        }

        /// <summary>Offers a target to servo <paramref name="g"/>'s gate at <paramref name="at"/>.</summary>
        private void TryWrite(int g, long at, float target, float speed)
        {
            if (!Passes(g, target, out float distance))
            {
                _deadbandDrops++;
                return;
            }

            long stop = at + MoveTicks(g, distance, speed);
            AppendMove(g, at, stop, target);
            _lastSent[g] = target;
            _hasLastSent[g] = true;
            _cooldownEnd[g] = stop;
            _written++;
        }

        /// <summary>False if the deadband drops the target. The distance is from the last target written; 0 if none.</summary>
        private bool Passes(int g, float target, out float distance)
        {
            if (!_hasLastSent[g])
            {
                distance = 0f;
                return true;
            }

            distance = Math.Abs(target - _lastSent[g]);
            return !(distance < _config.RelayDeadbandRad * _gateScale[g]);
        }

        private long MoveTicks(int g, float distance, float speed)
        {
            long ticks = (long)Math.Round(distance / ((double)speed * _gateScale[g]) * _config.TicksPerSecond);
            return ticks > _minMoveTicks ? ticks : _minMoveTicks;
        }

        private void AppendMove(int g, long start, long stop, float target)
        {
            int slot = g * _capacity + _moveHead[g];
            _moveStart[slot] = start;
            _moveEnd[slot] = stop;
            _moveTarget[slot] = target;
            _moveHead[g] = (_moveHead[g] + 1) % _capacity;
            if (_moveSize[g] < _capacity) _moveSize[g]++;
        }

        /// <summary>The servo's gate slot, created on first sight; -1 once every slot is taken.</summary>
        private int GateSlot(byte motorId)
        {
            int g = GateIndex(motorId);
            if (g >= 0 || _gateCount == _maxJoints)
            {
                return g;
            }

            g = _gateCount++;
            _gateIds[g] = motorId;
            _gateInstant[g] = IsInstant(motorId);
            _gateScale[g] = SpeedScale(motorId);
            _hasLastSent[g] = false;
            _lastSent[g] = 0f;
            _cooldownEnd[g] = long.MinValue;
            _hasPending[g] = false;
            _moveHead[g] = 0;
            _moveSize[g] = 0;
            return g;
        }

        private int GateIndex(byte motorId)
        {
            for (int g = 0; g < _gateCount; g++)
            {
                if (_gateIds[g] == motorId) return g;
            }

            return -1;
        }

        private int IndexOf(byte motorId)
        {
            for (int j = 0; j < _jointCount; j++)
            {
                if (_ids[j] == motorId) return j;
            }

            return -1;
        }

        private float SpeedScale(byte motorId)
        {
            byte[] ids = _config.ScaledMotorIds;
            for (int i = 0; i < ids.Length; i++)
            {
                if (ids[i] == motorId) return _config.SpeedScales[i];
            }

            return 1f;
        }

        private long SensedAgeTicks(byte motorId)
        {
            byte[] ids = _config.SensedAgeMotorIds;
            for (int i = 0; i < ids.Length; i++)
            {
                if (ids[i] == motorId) return (long)Math.Round(_config.SensedAgeSeconds[i] * (double)_config.TicksPerSecond);
            }

            return 0;
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

        /// <summary>
        /// One joint's replay from its anchor: the moves are visited in write order. Moves written
        /// at or before the anchor only decide whether the anchor is mid-move; each later one starts
        /// from wherever the replay has the joint when it is written.
        /// </summary>
        private struct JointReplay
        {
            private readonly long _anchor;
            private readonly float _measured;
            private readonly long _end;
            private bool _anchored;
            private bool _hasActive;
            private long _activeEnd;
            private float _activeTarget;
            private long _segStart;
            private long _segEnd;
            private float _segFrom;
            private float _segTo;

            public JointReplay(long anchor, float measured, long end)
            {
                _anchor = anchor;
                _measured = measured;
                _end = end;
                _anchored = false;
                _hasActive = false;
                _activeEnd = 0;
                _activeTarget = 0f;
                _segStart = anchor;
                _segEnd = anchor;
                _segFrom = measured;
                _segTo = measured;
            }

            /// <summary>Takes in one move; false once moves start after the prediction time.</summary>
            public bool Visit(long start, long stop, float target)
            {
                if (start <= _anchor)
                {
                    _hasActive = true;
                    _activeEnd = stop;
                    _activeTarget = target;
                    return true;
                }

                if (!_anchored) AnchorNow();
                if (start > _end) return false;

                float from = At(start);
                _segStart = start;
                _segEnd = stop;
                _segFrom = from;
                _segTo = target;
                return true;
            }

            public float Finish()
            {
                if (!_anchored) AnchorNow();
                return At(_end);
            }

            private void AnchorNow()
            {
                _anchored = true;
                if (_hasActive && _activeEnd > _anchor)
                {
                    // Mid-move: from the measurement to the move's target, over what is left of it.
                    _segStart = _anchor;
                    _segEnd = _activeEnd;
                    _segFrom = _measured;
                    _segTo = _activeTarget;
                }
                else
                {
                    _segStart = _anchor;
                    _segEnd = _anchor;
                    _segFrom = _measured;
                    _segTo = _measured;
                }
            }

            private float At(long t)
            {
                if (t >= _segEnd) return _segTo;
                if (t <= _segStart) return _segFrom;
                double u = (double)(t - _segStart) / (_segEnd - _segStart);
                return _segFrom + (float)((_segTo - _segFrom) * u);
            }
        }
    }
}
