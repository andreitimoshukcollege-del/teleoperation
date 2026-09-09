using System;
using Teleop.Core.Contracts;
using Teleop.Core.Time;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Buffering
{
    /// <summary>
    /// Registry key <c>percentile</c>: hold the delay budget at a target quantile of the last
    /// <see cref="PlayoutPolicyConfig.DelayWindowSamples"/> observed one-way delays. The first
    /// policy on this axis that <b>derives</b> its budget instead of being told one.
    ///
    /// <b>This is the Core implementation of the policy that reopened the axis.</b>
    /// <c>analysis/playout_bounds.py</c> measured "budget = max of the last w delays" at 47-61%
    /// less buffering than a matched-loss fixed budget on <c>synthetic-burst</c>. That policy is
    /// exactly this one at <see cref="PlayoutPolicyConfig.TargetPercentile"/> = 1.0, by
    /// construction and not by approximation — the quantile is the empirical nearest-rank order
    /// statistic over the same window, so p = 1.0 selects the window maximum. The offline result is
    /// therefore reproducible in the pipeline rather than merely cited at it, and
    /// <c>PercentileTrackingPlayoutTests</c> pins that correspondence directly.
    ///
    /// <b>Why an empirical window and not a filter.</b> A mean-plus-k-sigma estimator, or a Kalman
    /// filter over delay mean and variance, assumes a distribution shape. The delay distribution
    /// this project actually owns is sharply bimodal — a baseline cluster and a burst cluster with a
    /// 128 ms empty band between them
    /// (<c>docs/research-log/2026-09-09-playout-bounds.md</c>) — and on a bimodal sample a mean sits
    /// in the empty band, describing nothing. An order statistic makes no shape assumption and lands
    /// in whichever mode holds the quantile. <c>kalman-jitter</c> remains a separate planned row
    /// precisely so that assumption is tested rather than smuggled in here.
    ///
    /// <b>Requirement 6 (no oscillation) holds structurally, not by rate-limiting.</b> The estimate
    /// is an order statistic over a sliding window, so after a step change in delay it moves
    /// <i>monotonically</i> to the new level as stale samples leave the window, and reaches it in at
    /// most <c>DelayWindowSamples</c> observations. There is no feedback path — the budget does not
    /// influence the delays it is estimated from — so there is nothing to ring. This is why
    /// <see cref="PlayoutPolicyConfig.MaxAdaptationRatePerSecond"/> is ignored here: capping the
    /// rate could only add lag to a response that already cannot overshoot, and
    /// <c>MaxAdaptationRatePerSecond</c>'s own doc scopes it to <c>adaptive</c>, whose
    /// occupancy-driven feedback loop genuinely can.
    ///
    /// <b>A budget change applies to samples enqueued after it, never to samples already
    /// buffered.</b> Each sample's playout instant is fixed at <see cref="Enqueue"/>, so a budget
    /// raised at burst onset does not retroactively rescue the samples already scheduled under the
    /// old one. That is a real limit and it is deliberate: rescheduling live samples is
    /// buffer expansion, which is <c>adaptive</c>'s mechanism (NetEQ-style, against occupancy) and
    /// belongs in that row where it can be measured on its own.
    ///
    /// Reads <see cref="PlayoutPolicyConfig.HistoryCapacity"/>,
    /// <see cref="PlayoutPolicyConfig.DelayWindowSamples"/>,
    /// <see cref="PlayoutPolicyConfig.TargetPercentile"/>,
    /// <see cref="PlayoutPolicyConfig.InitialDelayBudgetTicks"/> (the warm-up budget, used until the
    /// window is full — <c>InitialDelayBudgetTicks</c>' own doc calls this "where every adaptive
    /// policy starts before its own estimator takes over"), and the
    /// <see cref="PlayoutPolicyConfig.MinDelayBudgetTicks"/>/<see cref="PlayoutPolicyConfig.MaxDelayBudgetTicks"/>
    /// clamp. Ignores the two filter-noise fields, <c>MaxAdaptationRatePerSecond</c>, and
    /// <c>LossWeight</c>.
    ///
    /// Deterministic and allocation-free: both windows are sized once in the constructor, and each
    /// observation costs one binary search plus one <see cref="Array.Copy"/> shift, never a sort.
    /// <see cref="ITimeAuthority"/> is read for <c>TicksPerSecond</c> only, never <c>NowTicks</c>.
    /// </summary>
    public sealed class PercentileTrackingPlayout : IPlayoutPolicy<Pose>
    {
        private readonly PlayoutSampleBuffer _buffer;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        private readonly double _targetPercentile;
        private readonly long _initialDelayBudgetTicks;
        private readonly long _minDelayBudgetTicks;
        private readonly long _maxDelayBudgetTicks;

        /// <summary>
        /// The window in arrival order, so the oldest observation is known and can be evicted.
        /// </summary>
        private readonly long[] _delaysByArrival;

        /// <summary>
        /// The same observations kept sorted ascending, so the quantile is an index rather than a
        /// sort. Maintained by insertion and deletion at a binary-searched position: O(w) per
        /// sample against O(w log w) for re-sorting, and — the part that actually matters here —
        /// allocation-free, since <see cref="Array.Sort(System.Array)"/> on a subrange would need a
        /// scratch array or a comparer.
        /// </summary>
        private readonly long[] _delaysSorted;

        private int _windowNextIndex;
        private int _windowCount;
        private long _delayBudgetTicks;

        public PercentileTrackingPlayout(PlayoutPolicyConfig config, IMetricSink metrics, ITimeAuthority clock)
        {
            if (config.HistoryCapacity <= 0)
            {
                throw new ArgumentException(
                    "PlayoutPolicyConfig.HistoryCapacity must be positive.", nameof(config));
            }

            if (config.DelayWindowSamples <= 0)
            {
                throw new ArgumentException(
                    "PlayoutPolicyConfig.DelayWindowSamples must be positive -- a quantile over an " +
                    "empty window is undefined.", nameof(config));
            }

            if (config.TargetPercentile <= 0.0 || config.TargetPercentile > 1.0)
            {
                throw new ArgumentException(
                    "PlayoutPolicyConfig.TargetPercentile must be in (0, 1]. 1.0 is the window " +
                    "maximum and is a legitimate operating point; 0.0 is not, because no order " +
                    "statistic corresponds to it.", nameof(config));
            }

            if (config.MinDelayBudgetTicks < 0 || config.MaxDelayBudgetTicks < config.MinDelayBudgetTicks)
            {
                throw new ArgumentException(
                    "PlayoutPolicyConfig requires 0 <= MinDelayBudgetTicks <= MaxDelayBudgetTicks.",
                    nameof(config));
            }

            _buffer = new PlayoutSampleBuffer(config.HistoryCapacity);
            _metrics = metrics;
            _ticksPerSecond = clock.TicksPerSecond;

            _targetPercentile = config.TargetPercentile;
            _initialDelayBudgetTicks = Clamp(
                config.InitialDelayBudgetTicks, config.MinDelayBudgetTicks, config.MaxDelayBudgetTicks);
            _minDelayBudgetTicks = config.MinDelayBudgetTicks;
            _maxDelayBudgetTicks = config.MaxDelayBudgetTicks;

            _delaysByArrival = new long[config.DelayWindowSamples];
            _delaysSorted = new long[config.DelayWindowSamples];
            _delayBudgetTicks = _initialDelayBudgetTicks;
        }

        public void Enqueue(uint sequence, Stamped<Pose> sample, long arrivalTicks)
        {
            // The arriving sample's own delay is folded in *before* its due instant is computed.
            // That is still strictly causal -- the sample is in hand, nothing about the future is
            // read -- and it is what lets the first sample of a burst be caught by the budget rise
            // it itself triggers. It is one step stronger than the offline analysis's `max of the
            // previous w`, which excluded the current sample; the difference is noted in the
            // analysis's own caveats as a lower bound.
            //
            // A negative delay is not possible in a single clock domain, but ClockSync converts the
            // capture stamp from the robot's domain and its offset estimate moves, so a small
            // negative is reachable early in a trial. Clamped rather than rejected: dropping the
            // observation would bias the window toward whatever the estimate happened to be.
            long observedDelay = arrivalTicks - sample.CaptureTicks;
            ObserveDelay(observedDelay < 0 ? 0 : observedDelay);

            PlayoutAdmission admission = _buffer.Enqueue(
                sequence, sample, arrivalTicks, dueTicks: sample.CaptureTicks + _delayBudgetTicks);

            if (admission == PlayoutAdmission.Late)
            {
                _metrics.Record(PlayoutMetrics.Late, 1.0, arrivalTicks);
            }
        }

        public bool TryDequeue(long nowTicks, out uint sequence, out Pose value, out long playoutTicks)
        {
            int underrunsBefore = _buffer.UnderrunCount;

            if (!_buffer.TryRelease(nowTicks, out sequence, out Stamped<Pose> sample, out playoutTicks, out long arrivalTicks))
            {
                if (_buffer.UnderrunCount != underrunsBefore)
                {
                    _metrics.Record(PlayoutMetrics.Underrun, 1.0, nowTicks);
                }

                value = default;
                return false;
            }

            value = sample.Value;
            PlayoutMetrics.RecordRelease(
                _metrics, _ticksPerSecond, playoutTicks, arrivalTicks, _delayBudgetTicks, _buffer.OccupancyFraction);
            return true;
        }

        public void Reset()
        {
            _buffer.Reset();
            _windowNextIndex = 0;
            _windowCount = 0;
            _delayBudgetTicks = _initialDelayBudgetTicks;
        }

        public PlayoutPolicyDiagnostics Diagnostics => new PlayoutPolicyDiagnostics(
            _delayBudgetTicks,
            _buffer.BufferedCount,
            _buffer.OccupancyFraction,
            _buffer.LateArrivalRate,
            _buffer.UnderrunCount,
            _buffer.DuplicatesRejected);

        /// <summary>
        /// Folds one delay into the window and recomputes the budget. The budget only starts
        /// tracking once the window is <b>full</b>: a quantile over three samples is not the
        /// quantile the config asked for, and letting it drive the budget would make the first
        /// steps of every trial a different policy from the rest of it.
        /// </summary>
        private void ObserveDelay(long delayTicks)
        {
            if (_windowCount == _delaysByArrival.Length)
            {
                RemoveFromSorted(_delaysByArrival[_windowNextIndex]);
            }
            else
            {
                _windowCount++;
            }

            _delaysByArrival[_windowNextIndex] = delayTicks;
            _windowNextIndex = (_windowNextIndex + 1) % _delaysByArrival.Length;
            InsertIntoSorted(delayTicks);

            if (_windowCount == _delaysByArrival.Length)
            {
                _delayBudgetTicks = Clamp(
                    _delaysSorted[NearestRankIndex(_targetPercentile, _windowCount)],
                    _minDelayBudgetTicks,
                    _maxDelayBudgetTicks);
            }
        }

        /// <summary>
        /// Nearest-rank: the smallest observation at or above the target fraction of the window.
        /// The same rule <c>analysis/playout_bounds.py</c> uses, deliberately — an interpolating
        /// definition would put the budget at a delay no sample ever had, and would break the exact
        /// correspondence at p = 1.0, which must select the maximum.
        /// </summary>
        private static int NearestRankIndex(double percentile, int count)
        {
            int index = (int)(percentile * count);
            return index >= count ? count - 1 : index;
        }

        private void InsertIntoSorted(long value)
        {
            int position = LowerBound(value, _windowCount - 1);
            Array.Copy(_delaysSorted, position, _delaysSorted, position + 1, _windowCount - 1 - position);
            _delaysSorted[position] = value;
        }

        private void RemoveFromSorted(long value)
        {
            int position = LowerBound(value, _windowCount);
            Array.Copy(_delaysSorted, position + 1, _delaysSorted, position, _windowCount - 1 - position);
        }

        /// <summary>
        /// First index in <c>_delaysSorted[0..length)</c> whose value is not less than
        /// <paramref name="value"/>. Used both to place an insertion and to locate an exact value
        /// for removal — duplicates are common (a quiet link produces long runs of identical
        /// delays), and removing the first match rather than searching for a specific one is
        /// correct precisely because the entries are indistinguishable.
        /// </summary>
        private int LowerBound(long value, int length)
        {
            int low = 0;
            int high = length;

            while (low < high)
            {
                int mid = low + ((high - low) / 2);
                if (_delaysSorted[mid] < value)
                {
                    low = mid + 1;
                }
                else
                {
                    high = mid;
                }
            }

            return low;
        }

        private static long Clamp(long value, long min, long max)
        {
            if (value < min)
            {
                return min;
            }

            return value > max ? max : value;
        }
    }
}
