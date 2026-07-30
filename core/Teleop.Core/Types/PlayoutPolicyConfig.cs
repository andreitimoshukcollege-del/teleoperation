// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// The complete parameter set of a playout policy. Every number a policy uses comes from
    /// here — no magic numbers in the body — so a sweep can vary a policy purely by varying this
    /// struct, and a run manifest can record exactly what was run.
    ///
    /// One shared block across all playout policies rather than one config type per
    /// implementation, matching <see cref="PredictorConfig"/>: each implementation reads the
    /// subset that applies to it and documents which fields it ignores.
    ///
    /// Units: ticks on the injected <c>ITimeAuthority</c> timebase for delay quantities;
    /// dimensionless for rates and weights. There is deliberately no <c>Default</c> — see
    /// <see cref="PredictorConfig"/> for why.
    /// </summary>
    public readonly struct PlayoutPolicyConfig
    {
        /// <summary>
        /// Samples the buffer retains at once. Allocated once in the constructor; <c>Enqueue</c>
        /// and <c>TryDequeue</c> must not allocate. Also bounds how far out-of-order a sample can
        /// arrive before it is rejected as too late to reinsert.
        /// </summary>
        public readonly int HistoryCapacity;

        /// <summary>
        /// Starting delay budget, in ticks: target gap between a sample's capture time and its
        /// playout instant. The only budget <c>fixed</c> ever uses; every adaptive policy starts
        /// here before its own estimator takes over.
        /// </summary>
        public readonly long InitialDelayBudgetTicks;

        /// <summary>
        /// Hard floor on the delay budget an adaptive policy may settle on, in ticks. Prevents a
        /// quiet, low-jitter stretch of trace from tuning the buffer down to nothing right before
        /// a burst. Ignored by <c>fixed</c> and <c>immediate</c>.
        /// </summary>
        public readonly long MinDelayBudgetTicks;

        /// <summary>
        /// Hard ceiling on the delay budget an adaptive policy may settle on, in ticks. Bounds
        /// how much latency a bad run of jitter can extract before the policy is judged to have
        /// failed rather than adapted. Ignored by <c>fixed</c> and <c>immediate</c>.
        /// </summary>
        public readonly long MaxDelayBudgetTicks;

        /// <summary>
        /// Target quantile of observed one-way delay to hold the delay budget at, in (0, 1).
        /// Used by <c>percentile</c>; ignored by policies that do not track a delay
        /// distribution.
        /// </summary>
        public readonly double TargetPercentile;

        /// <summary>
        /// Process-noise intensity for the delay-mean/variance filter. Used by
        /// <c>kalman-jitter</c>; ignored by non-filter policies. Larger means the filter trusts
        /// its own delay-drift model less.
        /// </summary>
        public readonly float DelayProcessNoise;

        /// <summary>
        /// Measurement-noise variance for the delay-mean/variance filter. Used by
        /// <c>kalman-jitter</c>; ignored by non-filter policies. Larger means the filter trusts
        /// each observed one-way delay less.
        /// </summary>
        public readonly float DelayMeasurementNoise;

        /// <summary>
        /// Upper bound on how fast the delay budget may change, in ticks of budget per second of
        /// wall time on the policy's own clock. Used by <c>adaptive</c> to satisfy the
        /// no-oscillation requirement in Buffering/CLAUDE.md — rate-limiting the response is what
        /// turns a step change in delay into a settle rather than a ring. Ignored by policies
        /// that do not adapt continuously.
        /// </summary>
        public readonly double MaxAdaptationRatePerSecond;

        /// <summary>
        /// Relative weight of loss versus latency when selecting an operating point on the
        /// latency/loss curve, in [0, 1]: 0 minimizes latency ignoring loss, 1 minimizes loss
        /// ignoring latency. Used by <c>pareto</c>; ignored by every other policy.
        /// </summary>
        public readonly double LossWeight;

        public PlayoutPolicyConfig(
            int historyCapacity,
            long initialDelayBudgetTicks,
            long minDelayBudgetTicks,
            long maxDelayBudgetTicks,
            double targetPercentile,
            float delayProcessNoise,
            float delayMeasurementNoise,
            double maxAdaptationRatePerSecond,
            double lossWeight)
        {
            HistoryCapacity = historyCapacity;
            InitialDelayBudgetTicks = initialDelayBudgetTicks;
            MinDelayBudgetTicks = minDelayBudgetTicks;
            MaxDelayBudgetTicks = maxDelayBudgetTicks;
            TargetPercentile = targetPercentile;
            DelayProcessNoise = delayProcessNoise;
            DelayMeasurementNoise = delayMeasurementNoise;
            MaxAdaptationRatePerSecond = maxAdaptationRatePerSecond;
            LossWeight = lossWeight;
        }
    }
}
