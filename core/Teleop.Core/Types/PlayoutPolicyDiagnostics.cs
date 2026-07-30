// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// What a playout policy reports about its own operating point. Returned from a property as
    /// a struct — Core never logs. Per Buffering/CLAUDE.md, a policy that does not expose this is
    /// not evaluable: two policies with the same mean latency and different loss rates are not
    /// comparable without it.
    /// </summary>
    public readonly struct PlayoutPolicyDiagnostics
    {
        /// <summary>
        /// Current target gap between a sample's capture time and its playout instant, in ticks.
        /// Constant for <c>fixed</c> and <c>immediate</c> (always zero for the latter); moves for
        /// every adaptive policy, which is the number a latency/loss plot puts on its x-axis.
        /// </summary>
        public readonly long DelayBudgetTicks;

        /// <summary>Samples currently buffered, awaiting playout.</summary>
        public readonly int BufferedCount;

        /// <summary>
        /// <see cref="BufferedCount"/> as a fraction of
        /// <see cref="PlayoutPolicyConfig.HistoryCapacity"/>, in [0, 1]. Reported directly rather
        /// than left for a caller to divide, so occupancy is comparable across policies
        /// configured with different capacities.
        /// </summary>
        public readonly float OccupancyFraction;

        /// <summary>
        /// Fraction of enqueued samples that missed their own playout instant — arrived after
        /// <see cref="DelayBudgetTicks"/> had already elapsed for them — since construction or
        /// the last <c>Reset</c>. This is the induced, effective loss docs/metrics.md
        /// distinguishes from network loss: a sample the network delivered but the buffer
        /// discarded as too late to use.
        /// </summary>
        public readonly double LateArrivalRate;

        /// <summary>
        /// Buffer underruns since construction or the last <c>Reset</c>: a <c>TryDequeue</c>
        /// that had nothing to give when the pipeline expected a sample. Per Buffering/CLAUDE.md
        /// this must never be silently zero on a lossy trace just because an implementation
        /// degrades gracefully — grace is what the caller does with an underrun, not a reason to
        /// stop counting it.
        /// </summary>
        public readonly int UnderrunCount;

        /// <summary>
        /// Enqueued samples discarded as duplicates of an already-buffered or already-released
        /// sequence, since construction or the last <c>Reset</c>.
        /// </summary>
        public readonly int DuplicatesRejected;

        public PlayoutPolicyDiagnostics(
            long delayBudgetTicks,
            int bufferedCount,
            float occupancyFraction,
            double lateArrivalRate,
            int underrunCount,
            int duplicatesRejected)
        {
            DelayBudgetTicks = delayBudgetTicks;
            BufferedCount = bufferedCount;
            OccupancyFraction = occupancyFraction;
            LateArrivalRate = lateArrivalRate;
            UnderrunCount = underrunCount;
            DuplicatesRejected = duplicatesRejected;
        }

        /// <summary>
        /// "Nothing has happened yet": empty buffer, zero budget, zero rates. The value a policy
        /// reports before its first <c>Enqueue</c> call.
        /// </summary>
        public static PlayoutPolicyDiagnostics Empty => default;

        public override string ToString() =>
            $"PlayoutPolicyDiagnostics(budget={DelayBudgetTicks}, buffered={BufferedCount}, " +
            $"occupancy={OccupancyFraction:P1}, lateArrival={LateArrivalRate:P2}, " +
            $"underruns={UnderrunCount}, duplicates={DuplicatesRejected})";
    }
}
