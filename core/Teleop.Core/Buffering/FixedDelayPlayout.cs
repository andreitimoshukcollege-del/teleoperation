using System;
using Teleop.Core.Contracts;
using Teleop.Core.Time;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Buffering
{
    /// <summary>
    /// Registry key <c>fixed</c>: one constant delay budget for the whole run. A sample captured
    /// at <c>t_capture</c> is due at <c>t_capture + budget</c> and is discarded if it arrives so
    /// late that something newer has already played. The other end of the curve from
    /// <see cref="ImmediatePlayout"/>, and — more importantly — <b>the denominator every adaptive
    /// policy's claim is stated against.</b>
    ///
    /// <b>Why this is not scaffolding.</b> "Adaptive beats fixed" is only a result at <i>matched
    /// loss</i>: a policy can always look better on delay by discarding more, and better on loss by
    /// buffering longer, so a single number from a single policy says nothing. The measurement in
    /// <c>analysis/playout_bounds.py</c> that reopened this axis is the gap between this policy's
    /// delay and an adaptive one's <i>at the same late-arrival rate</i> — 47-61% on
    /// <c>synthetic-burst</c>. Sweeping this row at several budgets is what produces the curve that
    /// gap is measured against, which is why it lands before <c>percentile</c> and <c>adaptive</c>
    /// rather than alongside them.
    ///
    /// <b>The budget is not a tuning knob to be optimised in place.</b> It comes from
    /// <see cref="PlayoutPolicyConfig.InitialDelayBudgetTicks"/> and never moves — not toward
    /// <see cref="PlayoutPolicyConfig.MinDelayBudgetTicks"/>, not toward
    /// <see cref="PlayoutPolicyConfig.MaxDelayBudgetTicks"/>, both of which this policy ignores
    /// along with every estimator field. A <c>fixed</c> policy that crept toward a better operating
    /// point would be an adaptive policy with an undocumented estimator, and the comparison it
    /// anchors would be meaningless.
    ///
    /// The honest caveat this row carries into any writeup: swept across budgets, the best
    /// <c>fixed</c> row is chosen with full knowledge of the trace, which a real deployment cannot
    /// do. It therefore <i>understates</i> the advantage of adapting, and overstates it against
    /// nothing at all.
    ///
    /// Reads <see cref="PlayoutPolicyConfig.HistoryCapacity"/> and
    /// <see cref="PlayoutPolicyConfig.InitialDelayBudgetTicks"/>; ignores every other field.
    /// Deterministic and allocation-free. <see cref="ITimeAuthority"/> is read for
    /// <c>TicksPerSecond</c> only, never <c>NowTicks</c>.
    /// </summary>
    public sealed class FixedDelayPlayout : IPlayoutPolicy<Pose>
    {
        private readonly PlayoutSampleBuffer _buffer;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;
        private readonly long _delayBudgetTicks;

        public FixedDelayPlayout(PlayoutPolicyConfig config, IMetricSink metrics, ITimeAuthority clock)
        {
            if (config.HistoryCapacity <= 0)
            {
                throw new ArgumentException(
                    "PlayoutPolicyConfig.HistoryCapacity must be positive.", nameof(config));
            }

            if (config.InitialDelayBudgetTicks < 0)
            {
                throw new ArgumentException(
                    "PlayoutPolicyConfig.InitialDelayBudgetTicks must not be negative -- a playout " +
                    "policy cannot schedule a sample before it was captured.", nameof(config));
            }

            _buffer = new PlayoutSampleBuffer(config.HistoryCapacity);
            _metrics = metrics;
            _ticksPerSecond = clock.TicksPerSecond;
            _delayBudgetTicks = config.InitialDelayBudgetTicks;
        }

        public void Enqueue(uint sequence, Stamped<Pose> sample, long arrivalTicks)
        {
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

        public void Reset() => _buffer.Reset();

        public PlayoutPolicyDiagnostics Diagnostics => new PlayoutPolicyDiagnostics(
            _delayBudgetTicks,
            _buffer.BufferedCount,
            _buffer.OccupancyFraction,
            _buffer.LateArrivalRate,
            _buffer.UnderrunCount,
            _buffer.DuplicatesRejected);
    }
}
