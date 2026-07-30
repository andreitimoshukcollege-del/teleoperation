// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// What a predictor reports about its own last <c>Predict</c> call. Returned from a
    /// property as a struct — Core never logs. A reconciler reads the uncertainty fields to
    /// scale its correction, and the eval harness reads the rest to attribute prediction error
    /// to the horizon that actually produced it.
    ///
    /// Units are metres and radians (ROS convention, matching <see cref="Pose"/>); the
    /// millimetres and degrees of docs/metrics.md are a reporting-time conversion.
    /// </summary>
    public readonly struct PredictorDiagnostics
    {
        /// <summary>
        /// Horizon actually extrapolated: target time minus the capture stamp of the newest
        /// observation the predictor used, in ticks. This is the horizon that error should be
        /// binned by, and it is not the horizon that was requested — it includes observation
        /// staleness and any clamp to <see cref="PredictorConfig.MaxHorizonTicks"/>. Negative
        /// means the target time was in the past relative to the newest observation, i.e. the
        /// predictor interpolated rather than extrapolated.
        /// </summary>
        public readonly long HorizonTicks;

        /// <summary>
        /// Capture stamp of the newest observation folded into the estimate, in ticks. Zero
        /// before the first accepted observation.
        /// </summary>
        public readonly long LastObservationTicks;

        /// <summary>
        /// Observations folded into the estimate since construction or the last <c>Reset</c>.
        /// </summary>
        public readonly int AcceptedObservations;

        /// <summary>
        /// Observations discarded since construction or the last <c>Reset</c>: duplicates,
        /// samples older than the retained history, and samples the implementation rejected as
        /// implausible. A rising count here is the signal that a trace is disagreeing with the
        /// predictor's assumptions; it must never be silently zero just because the predictor
        /// ignores ordering.
        /// </summary>
        public readonly int RejectedObservations;

        /// <summary>
        /// True when <see cref="PositionSigmaMeters"/> and
        /// <see cref="OrientationSigmaRadians"/> are meaningful. Predictors without a noise
        /// model (dead reckoning, smoothing) report false, and consumers must degrade
        /// gracefully rather than reading the sigmas as zero-uncertainty.
        /// </summary>
        public readonly bool HasUncertainty;

        /// <summary>
        /// One-sigma positional uncertainty of the estimate, metres. Meaningful only when
        /// <see cref="HasUncertainty"/> is true.
        /// </summary>
        public readonly float PositionSigmaMeters;

        /// <summary>
        /// One-sigma orientation uncertainty of the estimate as a geodesic angle, radians.
        /// Meaningful only when <see cref="HasUncertainty"/> is true.
        /// </summary>
        public readonly float OrientationSigmaRadians;

        /// <summary>
        /// Diagnostics for a predictor that supplies no uncertainty estimate.
        /// </summary>
        public PredictorDiagnostics(
            long horizonTicks,
            long lastObservationTicks,
            int acceptedObservations,
            int rejectedObservations)
        {
            HorizonTicks = horizonTicks;
            LastObservationTicks = lastObservationTicks;
            AcceptedObservations = acceptedObservations;
            RejectedObservations = rejectedObservations;
            HasUncertainty = false;
            PositionSigmaMeters = 0f;
            OrientationSigmaRadians = 0f;
        }

        /// <summary>
        /// Diagnostics including an uncertainty estimate. <paramref name="hasUncertainty"/> is
        /// explicit so a filter that has not yet converged can report its sigmas as not yet
        /// meaningful.
        /// </summary>
        public PredictorDiagnostics(
            long horizonTicks,
            long lastObservationTicks,
            int acceptedObservations,
            int rejectedObservations,
            bool hasUncertainty,
            float positionSigmaMeters,
            float orientationSigmaRadians)
        {
            HorizonTicks = horizonTicks;
            LastObservationTicks = lastObservationTicks;
            AcceptedObservations = acceptedObservations;
            RejectedObservations = rejectedObservations;
            HasUncertainty = hasUncertainty;
            PositionSigmaMeters = positionSigmaMeters;
            OrientationSigmaRadians = orientationSigmaRadians;
        }

        /// <summary>
        /// "Nothing is known": no observations, no horizon, no uncertainty. The value to pass
        /// where a predictor's diagnostics are unavailable — for example a reconciler driven
        /// directly from a recording rather than from a predictor.
        /// </summary>
        public static PredictorDiagnostics None => default;

        public override string ToString() =>
            HasUncertainty
                ? $"PredictorDiagnostics(horizon={HorizonTicks}, lastObs={LastObservationTicks}, " +
                  $"accepted={AcceptedObservations}, rejected={RejectedObservations}, " +
                  $"sigma=({PositionSigmaMeters:F4} m, {OrientationSigmaRadians:F4} rad))"
                : $"PredictorDiagnostics(horizon={HorizonTicks}, lastObs={LastObservationTicks}, " +
                  $"accepted={AcceptedObservations}, rejected={RejectedObservations}, sigma=n/a)";
    }
}
