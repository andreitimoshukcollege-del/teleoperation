using System;
using Teleop.Core.Prediction;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Metrics
{
    /// <summary>One bank member's live numbers over the scorer's window (docs/adr/0023 decision 2).</summary>
    public readonly struct LiveJointStats
    {
        /// <summary>The window held at least one scored reading.</summary>
        public readonly bool HasData;

        /// <summary>Fresh readings scored in the window (each pools its arm joints).</summary>
        public readonly int Readings;

        public readonly float ErrorP50Deg;
        public readonly float ErrorP95Deg;
        public readonly float CorrectionP95Deg;
        public readonly float JitterP95Deg;

        /// <summary>The gripper's error p95 in servo degrees of its travel; NaN without a sensed gripper.</summary>
        public readonly float GripperErrorP95ServoDeg;

        public LiveJointStats(bool hasData, int readings, float errorP50Deg, float errorP95Deg, float correctionP95Deg,
            float jitterP95Deg, float gripperErrorP95ServoDeg)
        {
            HasData = hasData;
            Readings = readings;
            ErrorP50Deg = errorP50Deg;
            ErrorP95Deg = errorP95Deg;
            CorrectionP95Deg = correctionP95Deg;
            JitterP95Deg = jitterP95Deg;
            GripperErrorP95ServoDeg = gripperErrorP95ServoDeg;
        }
    }

    /// <summary>
    /// Scores every enabled member of a <see cref="JointPredictorBank"/> live, over a rolling window
    /// (docs/adr/0023 decision 2), with docs/metrics.md §4's definitions computed at the headset:
    /// <list type="bullet">
    /// <item><c>joint_prediction_error_deg</c>: when a frame brings a new measurement (a sensed stamp
    /// later than every earlier one), each member's prediction for that sensed time, asked
    /// <i>before</i> the frame is observed, against the measured value; sensed arm joints only.</item>
    /// <item><c>joint_correction_deg</c>: at the same arrival, the prediction for <c>now</c> just
    /// before and just after the frame is observed.</item>
    /// <item><c>joint_display_jitter_deg</c>: |p_k − 2 p_(k−1) + p_(k−2)| of each enabled member's
    /// prediction, at every <see cref="OnDisplayTick"/> (the host calls it once per display frame).</item>
    /// </list>
    /// The gripper's error is kept apart, in servo degrees of travel (fraction ÷ fraction per radian),
    /// as the trainer reports it; it is in no other number.
    ///
    /// <para><b>A display aid, never a result</b> (Prediction/CLAUDE.md's evaluation note still
    /// governs results: those are scored offline). The host feeds frames through
    /// <see cref="ObserveState"/> instead of the bank's own; commands go to the bank directly.
    /// Percentiles are nearest-rank over the samples younger than the window. A member's numbers are
    /// cleared whenever the bank resets it (its <see cref="JointPredictorBank.Generation"/> moves).
    /// Each member keeps fixed-capacity rings, so a long, fast session overwrites its oldest samples
    /// rather than growing; nothing allocates after construction (invariant 8).</para>
    /// </summary>
    public sealed class LiveJointScorer
    {
        private const float DegPerRad = 57.29578f;

        private readonly JointPredictorBank _bank;
        private readonly long _windowTicks;
        private readonly byte[] _gripperMotorIds;
        private readonly float _gripperFractionPerRadian;
        private readonly int _maxJoints;

        private readonly Ring[] _error;
        private readonly Ring[] _correction;
        private readonly Ring[] _jitter;
        private readonly Ring[] _gripper;
        private readonly Ring[] _readings;
        private readonly int[] _generation;

        // Jitter history per member: the two previous predictions, by motor.
        private readonly JointSample[][] _prev1;
        private readonly JointSample[][] _prev2;
        private readonly int[] _prevCount;
        private readonly int[] _prevDepth;

        private readonly JointSample[] _before;
        private readonly JointSample[][] _beforeStash;
        private readonly int[] _beforeCounts;
        private readonly JointSample[] _after;
        private readonly JointSample[] _predicted;
        private readonly float[] _scratch;

        private bool _hasSensed;
        private long _lastSensedTicks;
        private long _nowTicks;

        public LiveJointScorer(JointPredictorBank bank, long windowTicks, int maxJoints, byte[] gripperMotorIds,
            float gripperFractionPerRadian, int readingCapacity = 4096, int tickCapacity = 4096)
        {
            _bank = bank ?? throw new ArgumentNullException(nameof(bank));
            if (windowTicks <= 0) throw new ArgumentOutOfRangeException(nameof(windowTicks), "the window must be positive");
            if (maxJoints <= 0) throw new ArgumentOutOfRangeException(nameof(maxJoints));
            if (!(gripperFractionPerRadian > 0f) || float.IsInfinity(gripperFractionPerRadian))
            {
                throw new ArgumentOutOfRangeException(nameof(gripperFractionPerRadian), "a positive, finite fraction per radian");
            }

            if (readingCapacity <= 0) throw new ArgumentOutOfRangeException(nameof(readingCapacity));
            if (tickCapacity <= 0) throw new ArgumentOutOfRangeException(nameof(tickCapacity));

            _windowTicks = windowTicks;
            _maxJoints = maxJoints;
            _gripperMotorIds = (byte[])(gripperMotorIds ?? new byte[0]).Clone();
            _gripperFractionPerRadian = gripperFractionPerRadian;

            int n = bank.Count;
            int perReading = readingCapacity * maxJoints;
            int perTicks = tickCapacity * maxJoints;
            _error = new Ring[n];
            _correction = new Ring[n];
            _jitter = new Ring[n];
            _gripper = new Ring[n];
            _readings = new Ring[n];
            _generation = new int[n];
            _prev1 = new JointSample[n][];
            _prev2 = new JointSample[n][];
            _prevCount = new int[n];
            _prevDepth = new int[n];
            for (int i = 0; i < n; i++)
            {
                _error[i] = new Ring(perReading);
                _correction[i] = new Ring(perReading);
                _jitter[i] = new Ring(perTicks);
                _gripper[i] = new Ring(readingCapacity);
                _readings[i] = new Ring(readingCapacity);
                _prev1[i] = new JointSample[maxJoints];
                _prev2[i] = new JointSample[maxJoints];
                _generation[i] = bank.Generation(i);
            }

            _before = new JointSample[maxJoints];
            _beforeStash = new JointSample[n][];
            for (int i = 0; i < n; i++) _beforeStash[i] = new JointSample[maxJoints];
            _beforeCounts = new int[n];
            _after = new JointSample[maxJoints];
            _predicted = new JointSample[maxJoints];
            _scratch = new float[Math.Max(perReading, perTicks)];
        }

        public JointPredictorBank Bank => _bank;

        /// <summary>
        /// Passes a frame to the bank, scoring each enabled member first when it brings a new
        /// measurement. <paramref name="nowTicks"/> is the headset's time of arrival, which dates the
        /// samples for the window and is when the correction is taken.
        /// </summary>
        public void ObserveState(long sensedTicks, long nowTicks, ReadOnlySpan<JointSample> joints)
        {
            _nowTicks = nowTicks;
            bool fresh = !_hasSensed || sensedTicks > _lastSensedTicks;
            if (!fresh)
            {
                _bank.ObserveState(sensedTicks, joints);
                return;
            }

            int n = _bank.Count;
            for (int i = 0; i < n; i++)
            {
                SyncGeneration(i);
                if (!_bank.IsEnabled(i) || !_bank.HasState(i)) continue;

                // The error: the prediction for the reading's own time, made before the reading is seen.
                if (_bank.TryPredictMember(i, sensedTicks, _predicted, out int predictedCount))
                {
                    bool any = false;
                    for (int j = 0; j < joints.Length; j++)
                    {
                        JointSample m = joints[j];
                        if (!m.Sensed) continue;
                        if (!TryFind(_predicted, predictedCount, m.MotorId, out float p)) continue;
                        float diff = Math.Abs(p - m.Value);
                        if (IsGripper(m.MotorId))
                        {
                            _gripper[i].Push(nowTicks, diff / _gripperFractionPerRadian * DegPerRad);
                        }
                        else
                        {
                            _error[i].Push(nowTicks, diff * DegPerRad);
                            any = true;
                        }
                    }

                    if (any) _readings[i].Push(nowTicks, 1f);
                }
            }

            // The correction: the prediction for now, just before and just after the frame.
            for (int i = 0; i < n; i++)
            {
                _beforeCounts[i] = -1;
                if (!_bank.IsEnabled(i) || !_bank.HasState(i)) continue;
                if (!_bank.TryPredictMember(i, nowTicks, _before, out int beforeCount))
                {
                    beforeCount = -1;
                }

                StashBefore(i, beforeCount);
            }

            _bank.ObserveState(sensedTicks, joints);
            _hasSensed = true;
            _lastSensedTicks = sensedTicks;

            for (int i = 0; i < n; i++)
            {
                if (!_bank.IsEnabled(i) || _beforeCounts[i] < 0) continue;
                if (!_bank.TryPredictMember(i, nowTicks, _after, out int afterCount)) continue;
                JointSample[] before = _beforeStash[i];
                for (int j = 0; j < afterCount; j++)
                {
                    JointSample a = _after[j];
                    if (IsGripper(a.MotorId)) continue;
                    if (!TryFind(before, _beforeCounts[i], a.MotorId, out float b)) continue;
                    _correction[i].Push(nowTicks, Math.Abs(a.Value - b) * DegPerRad);
                }
            }
        }

        /// <summary>
        /// Call once per display frame with the time the twin is drawn for: adds each enabled member's
        /// second difference of its prediction to its jitter window.
        /// </summary>
        public void OnDisplayTick(long targetTicks, long nowTicks)
        {
            _nowTicks = nowTicks;
            int n = _bank.Count;
            for (int i = 0; i < n; i++)
            {
                SyncGeneration(i);
                if (!_bank.IsEnabled(i) || !_bank.TryPredictMember(i, targetTicks, _predicted, out int count))
                {
                    _prevDepth[i] = 0;
                    continue;
                }

                JointSample[] p1 = _prev1[i];
                JointSample[] p2 = _prev2[i];
                if (_prevDepth[i] >= 2)
                {
                    for (int j = 0; j < count; j++)
                    {
                        JointSample s = _predicted[j];
                        if (IsGripper(s.MotorId)) continue;
                        if (!TryFind(p1, _prevCount[i], s.MotorId, out float v1)) continue;
                        if (!TryFind(p2, _prevCount[i], s.MotorId, out float v2)) continue;
                        _jitter[i].Push(nowTicks, Math.Abs(s.Value - 2f * v1 + v2) * DegPerRad);
                    }
                }

                // Shift: p2 <- p1, p1 <- now. Same motor set assumed; a changed set restarts the history.
                if (_prevDepth[i] > 0 && _prevCount[i] != count) _prevDepth[i] = 0;
                Array.Copy(p1, p2, _maxJoints);
                Array.Copy(_predicted, p1, Math.Min(count, _maxJoints));
                _prevCount[i] = Math.Min(count, _maxJoints);
                if (_prevDepth[i] < 2) _prevDepth[i]++;
            }
        }

        /// <summary>
        /// Member <paramref name="index"/>'s numbers over the window ending at the last time the scorer
        /// was given. False for a disabled member.
        /// </summary>
        public bool TryGetStats(int index, out LiveJointStats stats)
        {
            stats = default;
            if (index < 0 || index >= _bank.Count || !_bank.IsEnabled(index)) return false;
            SyncGeneration(index);
            long since = _nowTicks - _windowTicks;
            int readings = _readings[index].CountSince(since);
            float e50 = Percentile(_error[index], since, 0.50f);
            float e95 = Percentile(_error[index], since, 0.95f);
            float c95 = Percentile(_correction[index], since, 0.95f);
            float j95 = Percentile(_jitter[index], since, 0.95f);
            float g95 = Percentile(_gripper[index], since, 0.95f);
            stats = new LiveJointStats(readings > 0, readings, e50, e95, c95, j95, g95);
            return true;
        }

        /// <summary>Clears every member's numbers and the freshness state; the bank is left as it is.</summary>
        public void Reset()
        {
            for (int i = 0; i < _bank.Count; i++)
            {
                ClearMember(i);
                _generation[i] = _bank.Generation(i);
            }

            _hasSensed = false;
            _lastSensedTicks = 0;
            _nowTicks = 0;
        }

        // ------------------------------------------------------------------------------------------

        private void StashBefore(int index, int count)
        {
            _beforeCounts[index] = count;
            if (count > 0) Array.Copy(_before, _beforeStash[index], Math.Min(count, _maxJoints));
        }

        private void SyncGeneration(int index)
        {
            int g = _bank.Generation(index);
            if (g == _generation[index]) return;
            ClearMember(index);
            _generation[index] = g;
        }

        private void ClearMember(int index)
        {
            _error[index].Clear();
            _correction[index].Clear();
            _jitter[index].Clear();
            _gripper[index].Clear();
            _readings[index].Clear();
            _prevDepth[index] = 0;
            _prevCount[index] = 0;
        }

        private bool IsGripper(byte motorId)
        {
            for (int i = 0; i < _gripperMotorIds.Length; i++)
            {
                if (_gripperMotorIds[i] == motorId) return true;
            }

            return false;
        }

        private static bool TryFind(JointSample[] samples, int count, byte motorId, out float value)
        {
            for (int i = 0; i < count; i++)
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

        /// <summary>Nearest-rank percentile of the ring's samples at or after <paramref name="since"/>; NaN when none.</summary>
        private float Percentile(Ring ring, long since, float q)
        {
            int n = ring.CopySince(since, _scratch);
            if (n == 0) return float.NaN;
            int rank = (int)Math.Ceiling(q * n) - 1;
            if (rank < 0) rank = 0;
            return Select(_scratch, n, rank);
        }

        /// <summary>The <paramref name="k"/>-th smallest of the first <paramref name="n"/> values (Hoare's selection, in place).</summary>
        private static float Select(float[] a, int n, int k)
        {
            int lo = 0, hi = n - 1;
            while (lo < hi)
            {
                float pivot = a[(lo + hi) >> 1];
                int i = lo, j = hi;
                while (i <= j)
                {
                    while (a[i] < pivot) i++;
                    while (a[j] > pivot) j--;
                    if (i <= j)
                    {
                        float t = a[i];
                        a[i] = a[j];
                        a[j] = t;
                        i++;
                        j--;
                    }
                }

                if (k <= j) hi = j;
                else if (k >= i) lo = i;
                else break;
            }

            return a[k];
        }

        /// <summary>A fixed-capacity ring of (time, value), oldest overwritten first.</summary>
        private sealed class Ring
        {
            private readonly long[] _t;
            private readonly float[] _v;
            private int _start;
            private int _count;

            public Ring(int capacity)
            {
                _t = new long[capacity];
                _v = new float[capacity];
            }

            public void Push(long t, float v)
            {
                int index = (_start + _count) % _t.Length;
                if (_count == _t.Length)
                {
                    _start = (_start + 1) % _t.Length;
                }
                else
                {
                    _count++;
                }

                _t[index] = t;
                _v[index] = v;
            }

            public void Clear()
            {
                _start = 0;
                _count = 0;
            }

            public int CountSince(long since)
            {
                int n = 0;
                for (int k = 0; k < _count; k++)
                {
                    if (_t[(_start + k) % _t.Length] >= since) n++;
                }

                return n;
            }

            public int CopySince(long since, float[] into)
            {
                int n = 0;
                for (int k = 0; k < _count; k++)
                {
                    int index = (_start + k) % _t.Length;
                    if (_t[index] >= since) into[n++] = _v[index];
                }

                return n;
            }
        }
    }
}
