using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Buffering
{
    /// <summary>
    /// The five metric names every playout policy emits, and the one call that emits the four
    /// per-release ones. Defined in docs/metrics.md §2 and §3, per <c>IMetricSink</c>'s rule that
    /// a name must be defined there in the same change that first records it.
    ///
    /// Shared rather than duplicated per policy for the reason the names exist at all: these are
    /// the operating point two policies are compared on, and a policy that emitted
    /// <c>playout_delay_ms</c> measured from capture instead of from arrival would produce a
    /// latency/loss plot that silently disagreed with every other row on it.
    ///
    /// <c>const</c> rather than <c>static readonly</c> so the literals are interned and passing
    /// one costs nothing — <c>IMetricSink.Record</c>'s doc requires a constant or interned literal
    /// on the hot path.
    /// </summary>
    internal static class PlayoutMetrics
    {
        /// <summary>
        /// <c>t_playout - t_recv</c> in ms: what the buffer itself cost this sample. The stage
        /// docs/metrics.md §2's breakdown has always named and never had a number for. Emitted per
        /// released sample, so it has a distribution and §8 rule 3's percentile discipline applies.
        /// </summary>
        internal const string DelayMs = "playout_delay_ms";

        /// <summary>
        /// The policy's current delay budget in ms — the x-axis of a latency/loss plot. Constant
        /// for <c>immediate</c> and <c>fixed</c>; the whole point of an adaptive policy is that it
        /// moves. Emitted per released sample rather than on change, so a run has it at every
        /// instant a delay sample exists to pair it with.
        /// </summary>
        internal const string BudgetMs = "playout_budget_ms";

        /// <summary>
        /// Buffer occupancy in [0, 1]. Already a fraction of capacity, so it is comparable across
        /// policies configured with different <see cref="Types.PlayoutPolicyConfig.HistoryCapacity"/>.
        /// </summary>
        internal const string Occupancy = "playout_occupancy";

        /// <summary>
        /// One sample of value 1 per sample discarded as too late to play in capture order. A
        /// count, not a rate: the rate is
        /// <c>count(playout_late) / (count(playout_late) + count(playout_delay_ms))</c>, computed
        /// by the analyst, so that a policy which discards nothing emits nothing rather than a
        /// stream of zeroes.
        /// </summary>
        internal const string Late = "playout_late";

        /// <summary>
        /// One sample of value 1 per drain that released nothing — see
        /// <see cref="PlayoutSampleBuffer"/>'s remarks for why that, and not a false return, is
        /// the definition.
        /// </summary>
        internal const string Underrun = "playout_underrun";

        /// <summary>
        /// Emits the three per-release samples that describe the operating point at the instant a
        /// sample played out. Stamped at <paramref name="playoutTicks"/> rather than at the
        /// caller's poll time, matching <c>IMetricSink.Record</c>'s "the event's own time, not the
        /// time it happened to be recorded".
        /// </summary>
        internal static void RecordRelease(
            IMetricSink metrics,
            long ticksPerSecond,
            long playoutTicks,
            long arrivalTicks,
            long delayBudgetTicks,
            float occupancyFraction)
        {
            metrics.Record(DelayMs, TicksToMilliseconds(playoutTicks - arrivalTicks, ticksPerSecond), playoutTicks);
            metrics.Record(BudgetMs, TicksToMilliseconds(delayBudgetTicks, ticksPerSecond), playoutTicks);
            metrics.Record(Occupancy, occupancyFraction, playoutTicks);
        }

        private static double TicksToMilliseconds(long ticks, long ticksPerSecond) =>
            ticks * 1000.0 / ticksPerSecond;
    }
}
