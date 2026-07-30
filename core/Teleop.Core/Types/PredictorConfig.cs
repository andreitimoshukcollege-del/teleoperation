// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// The complete parameter set of a predictor. Every number a predictor uses comes from
    /// here — no magic numbers in the body — so that a sweep can vary a predictor purely by
    /// varying this struct, and so a run manifest can record exactly what was run.
    ///
    /// This is one shared block across all predictors rather than one config type per
    /// implementation: each implementation reads the subset that applies to it and documents
    /// which fields it ignores. That keeps the registry factory signature uniform.
    ///
    /// There is deliberately no <c>Default</c>: default parameters would be invented numbers
    /// with no result behind them. Values come from an experiment YAML or a test.
    ///
    /// Units: ticks on the injected <c>ITimeAuthority</c> timebase; metres, radians, seconds
    /// for physical quantities (ROS convention, matching <see cref="Pose"/>). Conversion to the
    /// millimetres and degrees used in docs/metrics.md happens at report time, not here.
    /// </summary>
    public readonly struct PredictorConfig
    {
        /// <summary>
        /// Hard cap on how far ahead <c>Predict</c> will extrapolate. A request beyond this is
        /// clamped to it, and the clamped value is what appears in
        /// <see cref="PredictorDiagnostics.HorizonTicks"/>. Bounds gross error when a target
        /// time is far in the future because the far end stalled.
        /// </summary>
        public readonly long MaxHorizonTicks;

        /// <summary>
        /// Gap between consecutive accepted observations beyond which derived state (velocity,
        /// acceleration, filter covariance) is treated as stale rather than differenced across
        /// the gap. Real traces contain gaps of several hundred milliseconds; differencing
        /// across one produces a large false velocity, which is the classic silent-garbage
        /// failure mode this field exists to prevent.
        /// </summary>
        public readonly long MaxObservationGapTicks;

        /// <summary>
        /// Number of observations retained. Buffers of this size are allocated once in the
        /// constructor; <c>Observe</c> and <c>Predict</c> must not allocate. Also sets how far
        /// back an out-of-order sample can be reinserted before it is rejected.
        /// </summary>
        public readonly int HistoryCapacity;

        /// <summary>
        /// Level smoothing factor, dimensionless, in [0, 1]. Higher tracks the newest
        /// observation more aggressively. Used by the exponential-smoothing family; ignored by
        /// predictors that do not smooth.
        /// </summary>
        public readonly float SmoothingAlpha;

        /// <summary>
        /// Trend smoothing factor, dimensionless, in [0, 1]. Second parameter of
        /// double-exponential smoothing; ignored by predictors without a trend term.
        /// </summary>
        public readonly float SmoothingBeta;

        /// <summary>
        /// Process-noise intensity for filter-based predictors, in metres²/second³
        /// (continuous white-noise acceleration). Larger means the filter trusts the motion
        /// model less. Ignored by predictors without a noise model.
        /// </summary>
        public readonly float ProcessNoise;

        /// <summary>
        /// Measurement-noise variance for filter-based predictors, in metres². Larger means
        /// the filter trusts each observation less. Ignored by predictors without a noise
        /// model.
        /// </summary>
        public readonly float MeasurementNoise;

        /// <summary>
        /// Upper bound on extrapolated linear speed, metres/second. Applied to the estimated
        /// velocity before extrapolation so that a corrupt or mis-stamped sample cannot throw
        /// the prediction an implausible distance.
        /// </summary>
        public readonly float MaxLinearSpeed;

        /// <summary>
        /// Upper bound on extrapolated angular speed, radians/second. Same purpose as
        /// <see cref="MaxLinearSpeed"/>, for rotation.
        /// </summary>
        public readonly float MaxAngularSpeed;

        public PredictorConfig(
            long maxHorizonTicks,
            long maxObservationGapTicks,
            int historyCapacity,
            float smoothingAlpha,
            float smoothingBeta,
            float processNoise,
            float measurementNoise,
            float maxLinearSpeed,
            float maxAngularSpeed)
        {
            MaxHorizonTicks = maxHorizonTicks;
            MaxObservationGapTicks = maxObservationGapTicks;
            HistoryCapacity = historyCapacity;
            SmoothingAlpha = smoothingAlpha;
            SmoothingBeta = smoothingBeta;
            ProcessNoise = processNoise;
            MeasurementNoise = measurementNoise;
            MaxLinearSpeed = maxLinearSpeed;
            MaxAngularSpeed = maxAngularSpeed;
        }
    }
}
