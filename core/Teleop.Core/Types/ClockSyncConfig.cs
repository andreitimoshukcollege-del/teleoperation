// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// The complete parameter set of a <c>ClockSync</c> instance, matching the
    /// <see cref="PredictorConfig"/> rationale: every number the estimator uses comes from here,
    /// so a sweep can vary it purely by varying this struct. There is deliberately no
    /// <c>Default</c> — see <see cref="PredictorConfig"/> for why.
    /// </summary>
    public readonly struct ClockSyncConfig
    {
        /// <summary>
        /// Round trips retained in the sliding window used to track the best (lowest) RTT
        /// recently observed. Allocated once, as a fixed-size buffer, in the constructor.
        /// A window rather than an all-time minimum so the estimator can adapt if the network
        /// path genuinely improves or degrades over a long-running session, instead of being
        /// permanently anchored to whatever the very first few round trips happened to measure.
        /// </summary>
        public readonly int HistoryCapacity;

        /// <summary>
        /// EWMA weight applied to a newly accepted offset (and uncertainty) sample, in [0, 1].
        /// Same name and shape as <see cref="PredictorConfig.SmoothingAlpha"/>: higher tracks the
        /// newest sample more aggressively. A single round trip's offset estimate is noisy —
        /// this is what turns a stream of samples into a usable estimate.
        /// </summary>
        public readonly float SmoothingAlpha;

        /// <summary>
        /// Hard ceiling on round-trip time, in ticks. A round trip slower than this is rejected
        /// outright as unusable for synchronization, regardless of how it compares to recently
        /// observed RTTs — a congestion event this severe is not "a bit worse than usual," it is
        /// not informative about clock offset at all.
        /// </summary>
        public readonly long MaxAcceptableRttTicks;

        /// <summary>
        /// A round trip is rejected if its RTT exceeds this multiple of the best RTT currently
        /// tracked in the <see cref="HistoryCapacity"/> window — the NTP-style min-RTT outlier
        /// filter. Independent of <see cref="MaxAcceptableRttTicks"/>: that field is an absolute
        /// ceiling, this one is relative to current network conditions, and a round trip only
        /// needs to fail one of the two to be rejected.
        /// </summary>
        public readonly double OutlierRttMultiple;

        /// <summary>
        /// Accepted round trips required before <c>ClockSyncDiagnostics.IsSynced</c> becomes
        /// true, so a caller does not act on a single noisy early sample.
        /// </summary>
        public readonly int MinSamplesBeforeTrusted;

        public ClockSyncConfig(
            int historyCapacity,
            float smoothingAlpha,
            long maxAcceptableRttTicks,
            double outlierRttMultiple,
            int minSamplesBeforeTrusted)
        {
            HistoryCapacity = historyCapacity;
            SmoothingAlpha = smoothingAlpha;
            MaxAcceptableRttTicks = maxAcceptableRttTicks;
            OutlierRttMultiple = outlierRttMultiple;
            MinSamplesBeforeTrusted = minSamplesBeforeTrusted;
        }
    }
}
