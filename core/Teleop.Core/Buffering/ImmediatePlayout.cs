using System;
using Teleop.Core.Contracts;
using Teleop.Core.Time;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Buffering
{
    /// <summary>
    /// Registry key <c>immediate</c>: zero buffer. Every sample is due the instant it was
    /// captured, so it plays out on the first drain after it arrives and the delay budget is
    /// always zero. Maximum jitter, minimum latency — one end of the curve this axis exists to
    /// explore, and the baseline docs/metrics.md §8 rule 1 requires in every comparison.
    ///
    /// <b>This is not a null object.</b> It is the policy that does the least, which is a
    /// different thing: it still enforces capture-time ordering and still rejects duplicates,
    /// because <c>IPlayoutPolicy</c> clause 2 requires both of every implementation and because a
    /// baseline that quietly tolerated reordering would make every policy compared against it look
    /// like it had introduced loss it did not introduce. With a zero budget the only way to
    /// preserve order is to discard a sample that arrives after something newer has played, so
    /// <c>immediate</c>'s late-arrival rate is exactly the stream's out-of-order rate. That is the
    /// honest zero-latency operating point, and it is the number a buffered policy has to beat.
    ///
    /// <b>It replaces, and does not reproduce, the inline stand-in it supersedes.</b>
    /// <c>Pipeline/OperatorEndpoint</c> previously assigned <c>t_playout = t_operatorRecv</c> and
    /// fed every decoded frame to the predictor in <i>arrival</i> order. On a link that reorders —
    /// which is every impaired profile, since the emulator delivers by earliest synthetic arrival
    /// even at <c>reorderProbability = 0</c> — this class drops what that line spliced in.
    /// docs/adr/0012-playout-policy-wiring.md records why that is a fix rather than a regression,
    /// and what it costs in comparability with results already recorded.
    ///
    /// Reads <see cref="PlayoutPolicyConfig.HistoryCapacity"/> and nothing else. Every other field
    /// belongs to a policy that adapts; the budget fields are meaningless here because the budget
    /// is structurally zero, not configured to zero.
    ///
    /// Deterministic and allocation-free. <see cref="ITimeAuthority"/> is read for
    /// <c>TicksPerSecond</c> only, never <c>NowTicks</c> — the same "time is always a parameter"
    /// discipline <c>Pipeline/CLAUDE.md</c> requirement 1 states for the endpoints.
    /// </summary>
    public sealed class ImmediatePlayout : IPlayoutPolicy<Pose>
    {
        /// <summary>
        /// The delay budget, structurally. Not a configured default and not a knob: "zero buffer"
        /// is the definition of this policy, so reading a budget from config would let an
        /// experiment silently turn it into <see cref="FixedDelayPlayout"/> under the wrong name.
        /// </summary>
        private const long DelayBudgetTicks = 0;

        private readonly PlayoutSampleBuffer _buffer;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        public ImmediatePlayout(PlayoutPolicyConfig config, IMetricSink metrics, ITimeAuthority clock)
        {
            if (config.HistoryCapacity <= 0)
            {
                throw new ArgumentException(
                    "PlayoutPolicyConfig.HistoryCapacity must be positive.", nameof(config));
            }

            _buffer = new PlayoutSampleBuffer(config.HistoryCapacity);
            _metrics = metrics;
            _ticksPerSecond = clock.TicksPerSecond;
        }

        public void Enqueue(uint sequence, Stamped<Pose> sample, long arrivalTicks)
        {
            PlayoutAdmission admission = _buffer.Enqueue(
                sequence, sample, arrivalTicks, dueTicks: sample.CaptureTicks + DelayBudgetTicks);

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
                _metrics, _ticksPerSecond, playoutTicks, arrivalTicks, DelayBudgetTicks, _buffer.OccupancyFraction);
            return true;
        }

        public void Reset() => _buffer.Reset();

        public PlayoutPolicyDiagnostics Diagnostics => new PlayoutPolicyDiagnostics(
            DelayBudgetTicks,
            _buffer.BufferedCount,
            _buffer.OccupancyFraction,
            _buffer.LateArrivalRate,
            _buffer.UnderrunCount,
            _buffer.DuplicatesRejected);
    }
}
