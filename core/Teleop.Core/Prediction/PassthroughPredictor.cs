using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// Registry key <c>none</c>: the zero-prediction baseline. It returns the newest observation
    /// it has seen, unchanged, whatever target time it is asked for. Per <c>Prediction/CLAUDE.md</c>
    /// this is "the baseline everything is measured against", and docs/metrics.md §8 requires it in
    /// every comparison even when it obviously loses -- it is what makes the other numbers
    /// interpretable, since the difference between <c>none</c> and a real predictor at a given
    /// horizon <i>is</i> the mitigation being measured.
    ///
    /// <b>It ignores every field of <see cref="PredictorConfig"/>.</b> Not
    /// <see cref="PredictorConfig.MaxHorizonTicks"/>, not
    /// <see cref="PredictorConfig.MaxObservationGapTicks"/>, not
    /// <see cref="PredictorConfig.HistoryCapacity"/>, not the smoothing factors, not the noise
    /// terms, not the speed caps. The config is still a constructor parameter so the registry
    /// factory signature stays uniform across the predictor family (the same reason
    /// <see cref="PredictorConfig"/> is one shared block rather than one type per implementation),
    /// and it is deliberately not validated either: rejecting a value this class never reads would
    /// make the baseline fail on configurations every other predictor accepts.
    ///
    /// <b>It deliberately does not clamp the reported horizon.</b> Every other predictor clamps to
    /// <see cref="PredictorConfig.MaxHorizonTicks"/> and reports the clamped value, because that is
    /// the horizon it actually extrapolated. This one extrapolates nothing, so there is no clamp to
    /// report and clamping would be a lie: <see cref="PredictorDiagnostics.HorizonTicks"/> here is
    /// the raw <c>targetTicks - lastObservationCaptureTicks</c>, i.e. the <i>true staleness</i> of
    /// the pose being displayed. That is exactly what a scorer binning error by horizon needs from
    /// the baseline -- a 900 ms-stale passthrough must be visible as 900 ms-stale, not reported as
    /// though it were a 400 ms prediction.
    ///
    /// <b>Ordering policy: max-by-capture-stamp.</b> <see cref="Observe"/> keeps the single sample
    /// with the largest <see cref="Stamped{T}.CaptureTicks"/> seen so far. That reduction is
    /// commutative and idempotent, which is what makes out-of-order and duplicate delivery correct
    /// here with no special casing at all -- any permutation of the same set of samples, with any
    /// multiplicity, leaves identical state. Rejected samples are still counted, because
    /// <see cref="PredictorDiagnostics.RejectedObservations"/> "must never be silently zero just
    /// because the predictor ignores ordering".
    ///
    /// Deterministic, allocation-free, and holds no buffers to preallocate. Not thread-safe, by
    /// contract. Time is a parameter, never a clock read -- this class does not even take an
    /// <see cref="ITimeAuthority"/>, since with no rate estimate anywhere in it there is no
    /// tick-to-second conversion to perform.
    /// </summary>
    public sealed class PassthroughPredictor : IPredictor<Pose>
    {
        private Pose _lastObservation;
        private long _lastObservationTicks;
        private bool _hasObservation;

        private long _horizonTicks;
        private int _acceptedObservations;
        private int _rejectedObservations;

        /// <param name="config">
        /// Accepted for signature uniformity with the rest of the predictor family and then
        /// ignored in its entirety -- see the type doc. It is intentionally not stored: a retained
        /// field nothing reads would imply this class has behaviour to configure.
        /// </param>
        public PassthroughPredictor(PredictorConfig config)
        {
            _ = config;
            _lastObservation = Pose.Identity;
            _lastObservationTicks = 0;
            _hasObservation = false;
            _horizonTicks = 0;
            _acceptedObservations = 0;
            _rejectedObservations = 0;
        }

        /// <summary>
        /// Retains <paramref name="obs"/> only if its capture stamp is strictly newer than the
        /// retained one; anything at or below it is counted in
        /// <see cref="PredictorDiagnostics.RejectedObservations"/> and otherwise ignored whole.
        ///
        /// Equal stamps are rejected rather than overwritten. A re-delivered datagram carries no
        /// new information, and <see cref="IPredictor{TState}.Observe"/> requires that observing
        /// the same sample twice leave the estimator exactly as observing it once did -- accepting
        /// the duplicate would also inflate
        /// <see cref="PredictorDiagnostics.AcceptedObservations"/>, which the eval harness reads as
        /// a sample count. Allocation-free.
        /// </summary>
        public void Observe(Stamped<Pose> obs)
        {
            if (_hasObservation && obs.CaptureTicks <= _lastObservationTicks)
            {
                _rejectedObservations++;
                return;
            }

            _lastObservation = obs.Value;
            _lastObservationTicks = obs.CaptureTicks;
            _hasObservation = true;
            _acceptedObservations++;
        }

        /// <summary>
        /// Returns the retained observation unchanged. <paramref name="targetTicks"/> affects only
        /// <see cref="Diagnostics"/>, never the returned pose -- that is the whole definition of
        /// this predictor.
        ///
        /// With no observation yet it returns <see cref="Pose.Identity"/>, and specifically
        /// <b>not</b> <c>default(Pose)</c>. They differ in the one place it matters:
        /// <c>default(Quaternion)</c> is the all-zero quaternion, which is not a rotation at all,
        /// and it would produce NaN out of every downstream normalization, geodesic angle, and log
        /// map -- silently, and only for the opening frames of a trial, which is the hardest kind
        /// of corruption to trace back here. Allocation-free.
        /// </summary>
        public Pose Predict(long targetTicks)
        {
            if (!_hasObservation)
            {
                _horizonTicks = 0;
                return Pose.Identity;
            }

            _horizonTicks = targetTicks - _lastObservationTicks;
            return _lastObservation;
        }

        /// <summary>
        /// The raw, unclamped horizon of the last <see cref="Predict"/> call (see the type doc),
        /// the retained observation's stamp, and the running accepted/rejected counts. Never
        /// reports uncertainty: this predictor has no noise model, so
        /// <see cref="PredictorDiagnostics.HasUncertainty"/> is false and the sigma fields are not
        /// meaningful.
        ///
        /// On a freshly constructed or freshly <see cref="Reset"/> instance every field is zero,
        /// i.e. exactly <see cref="PredictorDiagnostics.None"/>, as
        /// <see cref="IPredictor{TState}.Diagnostics"/> requires before the first
        /// <see cref="Predict"/>. The counters are reported as soon as they change rather than
        /// being withheld until the first <see cref="Predict"/>, because a trial that observes and
        /// rejects without ever predicting still needs its rejection count to be visible.
        /// </summary>
        public PredictorDiagnostics Diagnostics => new PredictorDiagnostics(
            _horizonTicks,
            _hasObservation ? _lastObservationTicks : 0,
            _acceptedObservations,
            _rejectedObservations);

        /// <summary>
        /// Returns the predictor to its as-constructed state: no retained observation, zeroed
        /// horizon and counters. Sweeps reuse instances across trials, and the retained-observation
        /// flag going back to false is what lets the next trial's first sample be accepted whatever
        /// its stamp -- the same staleness-baseline reset <c>Plant/RigidBodyPlant.Reset</c> makes,
        /// for the same reason.
        /// </summary>
        public void Reset()
        {
            _lastObservation = Pose.Identity;
            _lastObservationTicks = 0;
            _hasObservation = false;
            _horizonTicks = 0;
            _acceptedObservations = 0;
            _rejectedObservations = 0;
        }
    }
}
