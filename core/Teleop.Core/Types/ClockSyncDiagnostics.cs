// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// What a <c>ClockSync</c> reports about its current estimate. Returned from a property as a
    /// struct — Core never logs. Carried alongside every conversion this estimate produces
    /// (see <see cref="LatencyTrace.WithClockSync"/>) because, per docs/adr/0002-latency-trace.md,
    /// one-way-delay precision is floored by sync uncertainty, and reporting tighter than that
    /// floor is false precision.
    /// </summary>
    public readonly struct ClockSyncDiagnostics
    {
        /// <summary>
        /// Current smoothed offset, operator domain minus robot domain, in ticks. Zero before
        /// the first accepted round trip — matching <see cref="PredictorDiagnostics"/>'s
        /// "zero before the first accepted observation" convention.
        /// </summary>
        public readonly long OffsetTicks;

        /// <summary>
        /// Current smoothed one-sigma-equivalent uncertainty of <see cref="OffsetTicks"/>, in
        /// ticks, derived from accepted round trips' RTT/2. Zero before the first accepted round
        /// trip.
        /// </summary>
        public readonly long OffsetUncertaintyTicks;

        /// <summary>Round-trip time of the most recently accepted round trip, in ticks.</summary>
        public readonly long LastRttTicks;

        /// <summary>
        /// Best (lowest) RTT within the current history window, in ticks. Zero before any round
        /// trip has been recorded.
        /// </summary>
        public readonly long MinRttTicks;

        /// <summary>Round trips accepted since construction or the last <c>Reset</c>.</summary>
        public readonly int AcceptedSampleCount;

        /// <summary>
        /// Round trips rejected since construction or the last <c>Reset</c>: negative-RTT
        /// (invalid), over the hard ceiling, or over the relative min-RTT outlier multiple. Must
        /// never be silently zero on a congested trace just because the estimator degrades
        /// gracefully.
        /// </summary>
        public readonly int RejectedSampleCount;

        /// <summary>
        /// True once <see cref="AcceptedSampleCount"/> has reached
        /// <see cref="ClockSyncConfig.MinSamplesBeforeTrusted"/>. Callers should not treat
        /// <see cref="OffsetTicks"/> as trustworthy while this is false.
        /// </summary>
        public readonly bool IsSynced;

        public ClockSyncDiagnostics(
            long offsetTicks,
            long offsetUncertaintyTicks,
            long lastRttTicks,
            long minRttTicks,
            int acceptedSampleCount,
            int rejectedSampleCount,
            bool isSynced)
        {
            OffsetTicks = offsetTicks;
            OffsetUncertaintyTicks = offsetUncertaintyTicks;
            LastRttTicks = lastRttTicks;
            MinRttTicks = minRttTicks;
            AcceptedSampleCount = acceptedSampleCount;
            RejectedSampleCount = rejectedSampleCount;
            IsSynced = isSynced;
        }

        /// <summary>"Nothing has happened yet": no round trips, no offset, not synced.</summary>
        public static ClockSyncDiagnostics Empty => default;

        public override string ToString() =>
            $"ClockSyncDiagnostics(offset={OffsetTicks}, uncertainty={OffsetUncertaintyTicks}, " +
            $"lastRtt={LastRttTicks}, minRtt={MinRttTicks}, accepted={AcceptedSampleCount}, " +
            $"rejected={RejectedSampleCount}, synced={IsSynced})";
    }
}
