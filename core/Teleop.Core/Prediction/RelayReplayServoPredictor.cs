using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// <c>relay-replay-servo</c>: <c>relay-replay</c> (<see cref="RelayReplayPredictor"/>, docs/adr/0018)
    /// with a physical servo behind the relay's gate. relay-replay has each servo follow the relay's
    /// linear moves exactly, so it closes any gap between model and measurement within one move at any
    /// speed, has no lag, and assumes a command reaches the gate at its effective time. On the JetRover
    /// that runs ahead of the arm. This predictor keeps relay-replay's gate and replay as the servo's
    /// setpoint, and adds the servo's response to it.
    ///
    /// <para><b>The setpoint c(t)</b> is exactly what relay-replay predicts for the joint at t, with the
    /// same re-anchoring on the measurement, except for two of its inputs:</para>
    /// <list type="bullet">
    /// <item><b>Pipeline delay.</b> Every command is offered to the gate (and an instant joint's target
    /// takes effect) at its effective time plus round(<see cref="JointPredictorConfig.ServoPipelineDelaySeconds"/>
    /// × TicksPerSecond) ticks: RobotHost's loop, the relay's poll and the driver's cycle, which the
    /// host's effective time (send + RTT/2) does not see. Out-of-order commands are judged by the
    /// effective time as given. Every command is shifted by the same amount, so judging the shifted
    /// time would decide the same.</item>
    /// <item><b>First move from the measurement.</b> With <see cref="JointPredictorConfig.ServoFirstMoveFromMeasured"/>,
    /// a servo with no target written yet takes the measurement held when the command is observed, if
    /// that motor's is sensed (not carried), as its last target written: the deadband applies against
    /// it, and the move lasts max(minimum, |target − measured| / speed). The measurement is not latched;
    /// a dropped first target leaves the servo with no target written. Without a sensed measurement the
    /// first move lasts the minimum, as in relay-replay.</item>
    /// </list>
    ///
    /// <para><b>The servo.</b> A gated, non-instant joint's prediction at T is a filter run forward from
    /// the joint's anchor a (the measurement's stamp less that motor's reading age, relay-replay's anchor),
    /// starting at the measured (or carried) value. It takes whole steps of h = round(<see cref="JointPredictorConfig.ServoStepSeconds"/>
    /// × TicksPerSecond) ticks (at least one) on the grid a + k·h, then one partial step from the last
    /// grid point to T. Each step of dt = ticks / TicksPerSecond seconds ending at t is, in double
    /// precision:</para>
    /// <code>
    /// c      = (double)c(t)
    /// lagged = ServoLagSeconds &gt; 0 ? c + (p - c) * exp(-dt / ServoLagSeconds) : c
    /// limit  = ServoMaxSpeedRadPerSec * scale * dt      (scale: the joint's SpeedScales entry, else 1)
    /// p      = lagged - p &gt; limit ? p + limit : lagged - p &lt; -limit ? p - limit : lagged
    /// </code>
    /// <para>That is p + clamp(lagged − p, ±limit), except that a step inside the cap lands exactly on
    /// <c>lagged</c>, which is what makes the identity below exact. An infinite speed is no cap. The
    /// whole steps' exp(−dt / lag) is computed once. The prediction is (float)p, and T is clamped as
    /// relay-replay clamps it: never before the measurement, at most <see cref="JointPredictorConfig.MaxHorizonTicks"/>
    /// past it.</para>
    /// <para><b>For a copy that must agree</b>: every round is <see cref="Math.Round(double)"/> (halves to
    /// even, as relay-replay's own conversions) of the float parameter widened to double times
    /// TicksPerSecond; dt is the tick count divided by TicksPerSecond in double; lag, speed and scale
    /// are the float parameters widened to double, and the limit is (speed × scale) × dt in that
    /// order. The setpoint is sampled at each step's end, so the response to a step in c(t) starts up to
    /// one step early.</para>
    /// <para><b>The identity.</b> With an infinite speed, no lag, no pipeline delay and the first move
    /// off, every output, diagnostic and gate count is bit-identical to relay-replay's.</para>
    ///
    /// <para><b>Other joints</b> are relay-replay's: an instant joint takes its targets (delayed) with no
    /// filter, a joint that was never commanded holds its measurement, and a joint the robot does not
    /// measure keeps this predictor's own estimate across measurements (its prediction at the new
    /// stamp, filter included).</para>
    ///
    /// <para><b>Assumptions</b>, each a guess until fitted from recordings (trainer <c>armpred sysid</c>,
    /// docs/adr/0021): the servo follows its setpoint through a first-order lag and a symmetric speed
    /// cap, the same for every joint (a scaled joint's cap times its scale); no load, droop, gravity,
    /// stall or backlash; the pipeline delay is constant. The filter's only state is position: it
    /// restarts at each anchor from the measured value at rest relative to c(t), so a measurement taken
    /// while the servo is moving starts the prediction with no velocity. And relay-replay's re-anchoring
    /// is kept as it is: a measurement after the relay's move ended is read as the servo having stopped
    /// there, so c(t), and with it the prediction, holds the measured value until the next write, even
    /// where a lagging or capped servo would still be moving toward the target.</para>
    ///
    /// <para><b>Cost.</b> For each gated joint and prediction, c(t)'s piecewise-linear segments are built
    /// once into preallocated arrays (the segments relay-replay's replay visits, in the same float
    /// arithmetic), then the grid is walked with a moving segment index: O(moves + (T − a) / h). Nothing
    /// is allocated after construction.</para>
    ///
    /// <para>The gate and the replay are a copy of <see cref="RelayReplayPredictor"/>'s with the two input
    /// hooks, so relay-replay itself is untouched; <c>RelayReplayServoPredictorTests</c> pins the copy to
    /// the original on randomized call sequences.</para>
    /// </summary>
    public sealed class RelayReplayServoPredictor : ICommandAwarePredictor
    {
        private readonly JointPredictorConfig _config;
        private readonly int _maxJoints;
        private readonly int _capacity;
        private readonly long _minMoveTicks;
        private readonly float _maxMoveSpeed;

        // The two changes to relay-replay's inputs.
        private readonly long _pipelineTicks;
        private readonly bool _firstMoveFromMeasured;

        // The servo filter.
        private readonly double _ticksPerSecond;
        private readonly long _stepTicks;
        private readonly double _stepSeconds;
        private readonly bool _hasLag;
        private readonly double _lagSeconds;
        private readonly double _stepDecay;
        private readonly bool _capped;
        private readonly double _servoSpeed;

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

        // One joint's setpoint segments for one prediction: the anchor's, then one per move after the
        // anchor, then a pending target's.
        private readonly long[] _segStart;
        private readonly long[] _segEnd;
        private readonly float[] _segFrom;
        private readonly float[] _segTo;

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

        public RelayReplayServoPredictor(JointPredictorConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _maxJoints = config.MaxJoints;
            _capacity = config.CommandHistoryCapacity;
            _minMoveTicks = (long)Math.Round(config.RelayMinMoveSeconds * (double)config.TicksPerSecond);
            _maxMoveSpeed = config.RelayMoveSpeedRadPerSec;

            _pipelineTicks = (long)Math.Round(config.ServoPipelineDelaySeconds * (double)config.TicksPerSecond);
            _firstMoveFromMeasured = config.ServoFirstMoveFromMeasured;

            // A step shorter than the clock resolves is one tick.
            _ticksPerSecond = config.TicksPerSecond;
            _stepTicks = Math.Max(1L, (long)Math.Round(config.ServoStepSeconds * (double)config.TicksPerSecond));
            _stepSeconds = _stepTicks / _ticksPerSecond;
            _lagSeconds = config.ServoLagSeconds;
            _hasLag = _lagSeconds > 0.0;
            _stepDecay = _hasLag ? Math.Exp(-_stepSeconds / _lagSeconds) : 0.0;
            _servoSpeed = config.ServoMaxSpeedRadPerSec;
            _capped = !double.IsPositiveInfinity(_servoSpeed);

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

            // The anchor's segment, at most one per move in the ring, and a pending target's.
            _segStart = new long[_capacity + 2];
            _segEnd = new long[_capacity + 2];
            _segFrom = new float[_capacity + 2];
            _segTo = new float[_capacity + 2];

            _carried = new JointSample[_maxJoints];
            _scratchIds = new byte[_maxJoints];
            _scratchValues = new float[_maxJoints];
            _scratchAnchor = new long[_maxJoints];
            _scratchSensed = new bool[_maxJoints];
            Reset();
        }

        public PredictorDiagnostics Diagnostics =>
            new PredictorDiagnostics(_lastHorizon, _hasState ? _stateTicks : 0, _accepted, _rejected);

        /// <summary>The gate's counts, as relay-replay reports them: moves written, deadband drops, coalesced targets.</summary>
        public RelayReplayPredictor.GateCounts Gate => new RelayReplayPredictor.GateCounts(_written, _deadbandDrops, _coalesced);

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
            // Judged by the effective time as given; every command is shifted by the same delay below.
            if (effectiveTicks < _lastCommandTicks)
            {
                _rejected++;
                return;
            }

            _lastCommandTicks = effectiveTicks;
            long at = effectiveTicks + _pipelineTicks;
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
                    AppendMove(g, at, at, target);
                    continue;
                }

                // The pending target is offered to the gate when its cooldown ends, before this one.
                if (_hasPending[g] && _cooldownEnd[g] <= at)
                {
                    _hasPending[g] = false;
                    TryWrite(g, _cooldownEnd[g], _pendingTarget[g], _pendingSpeed[g]);
                }

                if (at >= _cooldownEnd[g])
                {
                    TryWrite(g, at, target, speed);
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
            // model's own estimate for it, filter included, at this measurement's time.
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

            if (_gateInstant[g])
            {
                int size = _moveSize[g];
                int oldest = (_moveHead[g] - size + _capacity) % _capacity;
                int baseIndex = g * _capacity;
                float value = _measured[j];
                for (int k = 0; k < size; k++)
                {
                    int slot = baseIndex + (oldest + k) % _capacity;
                    if (_moveStart[slot] > end) break;
                    value = _moveTarget[slot];
                }

                return value;
            }

            int segments = BuildSetpoint(j, g, end);
            return Respond(j, g, segments, end);
        }

        /// <summary>
        /// Builds joint <paramref name="j"/>'s setpoint c(t) up to <paramref name="end"/> as relay-replay's
        /// replay visits the moves, into the segment arrays; returns how many there are (at least one).
        /// Moves written at or before the anchor only decide whether the anchor is mid-move; each later
        /// one starts from wherever the setpoint is when it is written.
        /// </summary>
        private int BuildSetpoint(int j, int g, long end)
        {
            long anchor = _anchor[j];
            float measured = _measured[j];
            bool hasActive = false;
            long activeEnd = 0;
            float activeTarget = 0f;
            int n = 0;
            bool past = false;

            int size = _moveSize[g];
            int oldest = (_moveHead[g] - size + _capacity) % _capacity;
            int baseIndex = g * _capacity;
            for (int k = 0; k < size; k++)
            {
                int slot = baseIndex + (oldest + k) % _capacity;
                long start = _moveStart[slot];
                if (start <= anchor)
                {
                    hasActive = true;
                    activeEnd = _moveEnd[slot];
                    activeTarget = _moveTarget[slot];
                    continue;
                }

                if (n == 0) n = AnchorSegment(anchor, measured, hasActive, activeEnd, activeTarget);
                if (start > end)
                {
                    past = true;
                    break;
                }

                n = AppendSegment(n, start, _moveEnd[slot], _moveTarget[slot]);
            }

            // A pending target whose cooldown ends in time is written then (unless the deadband drops it).
            if (!past && _hasPending[g] && _cooldownEnd[g] <= end)
            {
                long at = _cooldownEnd[g];
                if (Passes(g, _pendingTarget[g], out float distance))
                {
                    long stop = at + MoveTicks(g, distance, _pendingSpeed[g]);
                    if (at <= anchor)
                    {
                        hasActive = true;
                        activeEnd = stop;
                        activeTarget = _pendingTarget[g];
                    }
                    else
                    {
                        if (n == 0) n = AnchorSegment(anchor, measured, hasActive, activeEnd, activeTarget);
                        n = AppendSegment(n, at, stop, _pendingTarget[g]);
                    }
                }
            }

            if (n == 0) n = AnchorSegment(anchor, measured, hasActive, activeEnd, activeTarget);
            return n;
        }

        /// <summary>The first segment: mid-move, from the measurement to that move's target over what is left of it; otherwise a hold.</summary>
        private int AnchorSegment(long anchor, float measured, bool hasActive, long activeEnd, float activeTarget)
        {
            _segStart[0] = anchor;
            _segFrom[0] = measured;
            if (hasActive && activeEnd > anchor)
            {
                _segEnd[0] = activeEnd;
                _segTo[0] = activeTarget;
            }
            else
            {
                _segEnd[0] = anchor;
                _segTo[0] = measured;
            }

            return 1;
        }

        /// <summary>A move written at <paramref name="start"/>, from where the last segment has the setpoint then.</summary>
        private int AppendSegment(int n, long start, long stop, float target)
        {
            float from = SetpointAt(n - 1, start);
            _segStart[n] = start;
            _segEnd[n] = stop;
            _segFrom[n] = from;
            _segTo[n] = target;
            return n + 1;
        }

        /// <summary>Segment <paramref name="s"/> at <paramref name="t"/>, in relay-replay's float arithmetic.</summary>
        private float SetpointAt(int s, long t)
        {
            if (t >= _segEnd[s]) return _segTo[s];
            if (t <= _segStart[s]) return _segFrom[s];
            double u = (double)(t - _segStart[s]) / (_segEnd[s] - _segStart[s]);
            return _segFrom[s] + (float)((_segTo[s] - _segFrom[s]) * u);
        }

        /// <summary>The servo filter from joint <paramref name="j"/>'s anchor to <paramref name="end"/> (see the class doc).</summary>
        private float Respond(int j, int g, int segments, long end)
        {
            long t = _anchor[j];
            double p = _measured[j];
            double speed = _servoSpeed * _gateScale[g];
            double stepLimit = speed * _stepSeconds;
            int s = 0;
            long steps = (end - t) / _stepTicks;
            for (long k = 0; k < steps; k++)
            {
                t += _stepTicks;
                while (s + 1 < segments && _segStart[s + 1] <= t) s++;
                p = Step(p, SetpointAt(s, t), _stepDecay, stepLimit);
            }

            if (end > t)
            {
                double dt = (end - t) / _ticksPerSecond;
                while (s + 1 < segments && _segStart[s + 1] <= end) s++;
                double decay = _hasLag ? Math.Exp(-dt / _lagSeconds) : 0.0;
                p = Step(p, SetpointAt(s, end), decay, speed * dt);
            }

            return (float)p;
        }

        /// <summary>One step toward the setpoint <paramref name="c"/>: the lag, then the speed cap.</summary>
        private double Step(double p, double c, double decay, double limit)
        {
            double lagged = _hasLag ? c + (p - c) * decay : c;
            if (_capped)
            {
                double step = lagged - p;
                if (step > limit) return p + limit;
                if (step < -limit) return p - limit;
            }

            return lagged;
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

        /// <summary>
        /// False if the deadband drops the target. The distance is from the last target written; with
        /// none, from the servo's sensed measurement when the first move is sized from it, else 0.
        /// </summary>
        private bool Passes(int g, float target, out float distance)
        {
            float last;
            if (_hasLastSent[g])
            {
                last = _lastSent[g];
            }
            else
            {
                int j = _firstMoveFromMeasured ? SensedIndexOf(_gateIds[g]) : -1;
                if (j < 0)
                {
                    distance = 0f;
                    return true;
                }

                last = _measured[j];
            }

            distance = Math.Abs(target - last);
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

        /// <summary>The joint holding a sensed (not carried) measurement of the motor; -1 if none.</summary>
        private int SensedIndexOf(byte motorId)
        {
            for (int j = 0; j < _jointCount; j++)
            {
                if (_ids[j] == motorId && _sensed[j]) return j;
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
    }
}
