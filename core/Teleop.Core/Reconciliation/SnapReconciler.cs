using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Reconciliation
{
    /// <summary>
    /// Registry key <c>snap</c>: when truth disagrees with the prediction, jump straight to truth on
    /// the next frame. <c>Reconciliation/CLAUDE.md</c> calls this "the baseline -- measure how bad it
    /// is, don't skip it", and docs/metrics.md §8 requires it in every comparison. It is the zero on
    /// the correction-cost axis that every smoothed reconciler is calibrated against.
    ///
    /// <b>It does not satisfy <see cref="IReconciler{TState}"/>'s C1-continuity clause, and it must
    /// not be changed until it does.</b> That contract requires "no position or velocity
    /// discontinuity in the returned state". A snap is a position and velocity discontinuity by
    /// definition -- that is what the word means and what this class exists to produce. The clause is
    /// the requirement every <i>candidate mitigation</i> owes its callers; this implementation is the
    /// deliberate, single, documented violation of it, and its test suite therefore <b>proves and
    /// quantifies</b> the discontinuity (a witness test asserting a jerk far above any smooth
    /// trajectory's) instead of asserting continuity. If a future change ever made this reconciler
    /// smooth, that would be a regression and not an improvement: every published comparison would
    /// silently be against a baseline that was already mitigating, and the measured benefit of every
    /// other reconciler would shrink for reasons that have nothing to do with the reconcilers.
    ///
    /// Bounded convergence <i>is</i> satisfied, at the tightest bound the interface allows: exactly
    /// one <see cref="Reconcile"/> call, independent of correction magnitude and of any
    /// configuration.
    ///
    /// <b>Fields of <see cref="ReconcilerConfig"/> it reads:</b>
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> and
    /// <see cref="ReconcilerConfig.ConvergenceOrientationToleranceRadians"/>, which decide whether an
    /// arriving sample disagrees enough to be worth correcting at all.
    /// <b>Fields it ignores:</b>
    /// <list type="bullet">
    /// <item><see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> -- an upper bound on
    /// convergence time is vacuous for something that converges in one call. Honouring it could only
    /// mean converging <i>slower</i>, which would make this a smoothed reconciler.</item>
    /// <item><see cref="ReconcilerConfig.MaxCorrectionLinearSpeedMetersPerSecond"/> and
    /// <see cref="ReconcilerConfig.MaxCorrectionAngularSpeedRadPerSecond"/> -- a snap has no rate, so
    /// there is no rate to cap. Applying either would turn it into a rate-limited reconciler, which
    /// is a different implementation with a different registry key.</item>
    /// <item><see cref="ReconcilerConfig.RollbackHistoryCapacity"/> -- for <c>rollback</c>, which
    /// re-simulates buffered inputs. Nothing here rewinds.</item>
    /// </list>
    ///
    /// <b>Time is always a parameter.</b> The injected <see cref="ITimeAuthority"/> is read for
    /// <see cref="ITimeAuthority.TicksPerSecond"/> only and never for
    /// <see cref="ITimeAuthority.NowTicks"/> -- the same discipline
    /// <c>Pipeline/OperatorEndpoint</c> documents. It is needed only to turn tick deltas into the
    /// milliseconds and mm/s³ units docs/metrics.md §5 specifies; every "when" arrives explicitly, as
    /// <c>nowTicks</c> on <see cref="Reconcile"/> and as the sample's own
    /// <see cref="Stamped{T}.CaptureTicks"/> on <see cref="Observe"/>.
    ///
    /// Deterministic and allocation-free: the only buffer is the three-sample position history, sized
    /// once in the constructor. Not thread-safe, by contract.
    /// </summary>
    public sealed class SnapReconciler : IReconciler<Pose>
    {
        /// <summary>docs/metrics.md §5, positional correction magnitude, millimetres.</summary>
        private const string CorrectionMagnitudeMmMetric = "correction_magnitude_mm";

        /// <summary>docs/metrics.md §5, angular correction magnitude, degrees.</summary>
        private const string CorrectionMagnitudeDegMetric = "correction_magnitude_deg";

        /// <summary>docs/metrics.md §5, third derivative of displayed position, mm/s³.</summary>
        private const string JerkMetric = "jerk_mm_s3";

        /// <summary>docs/metrics.md §5, correction onset to within tolerance, milliseconds.</summary>
        private const string TimeToConvergenceMsMetric = "time_to_convergence_ms";

        /// <summary>
        /// Core works in metres and radians (ROS convention); docs/metrics.md reports millimetres and
        /// degrees. The conversion happens here, at the reporting boundary, and nowhere else.
        /// </summary>
        private const double MetresToMillimetres = 1000.0;

        private const double RadiansToDegrees = 180.0 / Math.PI;

        private const double MillisecondsPerSecond = 1000.0;

        private readonly float _positionTolerance;
        private readonly float _orientationTolerance;
        private readonly IMetricSink _metrics;

        /// <summary>
        /// Shared with every other reconciler so that <c>jerk_mm_s3</c> means one thing across the
        /// axis being compared -- see <see cref="DisplayedJerkEstimator"/> for why that matters.
        /// </summary>
        private readonly DisplayedJerkEstimator _jerk;

        private bool _hasPendingCorrection;
        private Pose _pendingTarget;

        /// <summary>
        /// Highest authoritative <see cref="Stamped{T}.CaptureTicks"/> accepted so far. Starts at
        /// <see cref="long.MinValue"/> so the very first sample is accepted whatever its stamp,
        /// including zero and negative.
        /// </summary>
        private long _lastAcceptedCaptureTicks;

        private long _lastReconcileTicks;
        private Pose _lastOutput;

        /// <param name="config">
        /// Parameters; see the type doc for which two fields are read and why the rest are ignored.
        /// </param>
        /// <param name="metrics">
        /// Correction-cost sink (docs/metrics.md §5). A constructor dependency rather than a
        /// per-frame parameter so that <see cref="Reconcile"/>'s signature stays allocation-free, per
        /// <see cref="IReconciler{TState}"/>.
        /// </param>
        /// <param name="clock">
        /// Read for <see cref="ITimeAuthority.TicksPerSecond"/> only -- never
        /// <see cref="ITimeAuthority.NowTicks"/>.
        /// </param>
        public SnapReconciler(ReconcilerConfig config, IMetricSink metrics, ITimeAuthority clock)
        {
            if (metrics == null)
            {
                throw new ArgumentNullException(nameof(metrics));
            }

            if (clock == null)
            {
                throw new ArgumentNullException(nameof(clock));
            }

            if (clock.TicksPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(clock), clock.TicksPerSecond, "Ticks per second must be positive.");
            }

            if (config.ConvergencePositionToleranceMeters < 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.ConvergencePositionToleranceMeters,
                    "ConvergencePositionToleranceMeters must be non-negative.");
            }

            if (config.ConvergenceOrientationToleranceRadians < 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.ConvergenceOrientationToleranceRadians,
                    "ConvergenceOrientationToleranceRadians must be non-negative.");
            }

            _positionTolerance = config.ConvergencePositionToleranceMeters;
            _orientationTolerance = config.ConvergenceOrientationToleranceRadians;
            _metrics = metrics;
            _jerk = new DisplayedJerkEstimator(clock.TicksPerSecond);

            ClearCorrectionState();
        }

        /// <summary>
        /// True when no correction is queued. Because <see cref="Reconcile"/> discharges a pending
        /// correction in the very call that first sees it, this is false only between an
        /// <see cref="Observe"/> that found a disagreement and the next <see cref="Reconcile"/> --
        /// i.e. transiently, within a single frame. It is not "always true": a bounded-convergence
        /// test that never observed it going false would be testing nothing, so the window is real
        /// and observable, just short.
        ///
        /// True on a freshly constructed or freshly <see cref="Reset"/> instance, as
        /// <see cref="IReconciler{TState}.IsConverged"/> requires.
        /// </summary>
        public bool IsConverged => !_hasPendingCorrection;

        /// <summary>
        /// Truth arrived. Measures the disagreement against what was displayed for that same instant
        /// and, if it exceeds tolerance, queues a jump to truth for the next
        /// <see cref="Reconcile"/>. Changes nothing visible here, per
        /// <see cref="IReconciler{TState}.Observe"/>.
        ///
        /// <b>Ordering.</b> A sample whose capture stamp is not strictly greater than the last
        /// accepted one is stale or a duplicate and is ignored whole -- no retarget, no metric, no
        /// state change of any kind. Equal stamps are rejected for the reason
        /// <see cref="IReconciler{TState}.Observe"/> states outright: "a duplicate must not be counted
        /// as a second correction; correction rate is a reported metric and double-counting corrupts
        /// it." A re-delivered datagram must not turn one correction into two in the results.
        ///
        /// <b>Within tolerance means no correction at all</b>, not a zero-magnitude one. This is the
        /// load-bearing case for the <c>none</c> + <c>snap</c> pairing that docs/metrics.md §8
        /// requires in every comparison: with a passthrough predictor whose output already equals
        /// truth, the pipeline must degenerate to exact pass-through, emitting no correction-cost
        /// samples and never leaving <see cref="IsConverged"/>. A stream of zero-magnitude
        /// "corrections" would inflate the baseline's correction rate with events that never happened.
        ///
        /// <b>Retarget, do not stack.</b> A second qualifying sample before the next
        /// <see cref="Reconcile"/> replaces the pending target rather than queueing behind it. The
        /// newest authoritative sample is the best available truth and nothing is gained by first
        /// snapping to an older one -- and a queue would break the one-call convergence bound.
        ///
        /// Allocation-free.
        /// </summary>
        /// <param name="diagnostics">
        /// Ignored. This reconciler does not scale its correction by predictor uncertainty -- it has
        /// no scale factor to apply -- so it behaves identically whether
        /// <see cref="PredictorDiagnostics.HasUncertainty"/> is true or false, which is the graceful
        /// degradation <see cref="IReconciler{TState}.Observe"/> asks for, reached by not depending on
        /// the field in the first place. <see cref="PredictorDiagnostics.None"/> is equally fine.
        /// </param>
        public void Observe(
            Stamped<Pose> authoritative,
            Pose predictedAtCapture,
            in PredictorDiagnostics diagnostics)
        {
            _ = diagnostics;

            if (authoritative.CaptureTicks <= _lastAcceptedCaptureTicks)
            {
                return;
            }

            _lastAcceptedCaptureTicks = authoritative.CaptureTicks;

            float positionErrorMeters =
                PoseMath.PositionErrorMeters(predictedAtCapture, authoritative.Value);
            float orientationErrorRadians =
                PoseMath.OrientationErrorRadians(predictedAtCapture, authoritative.Value);

            if (positionErrorMeters <= _positionTolerance &&
                orientationErrorRadians <= _orientationTolerance)
            {
                return;
            }

            _pendingTarget = authoritative.Value;
            _hasPendingCorrection = true;

            // Stamped at the sample's own capture time, not at a clock read and not at the frame it
            // is applied on: IMetricSink.Record documents `ticks` as "the event's own time", and the
            // event here is the disagreement, which existed at capture.
            _metrics.Record(
                CorrectionMagnitudeMmMetric,
                positionErrorMeters * MetresToMillimetres,
                authoritative.CaptureTicks);
            _metrics.Record(
                CorrectionMagnitudeDegMetric,
                orientationErrorRadians * RadiansToDegrees,
                authoritative.CaptureTicks);
        }

        /// <summary>
        /// With a correction pending, returns the authoritative target <b>exactly</b> and clears the
        /// pending flag -- the snap. With none pending, returns <paramref name="predicted"/>
        /// untouched.
        ///
        /// <b>Idempotent in <paramref name="nowTicks"/>.</b> A call at or before the last reconciled
        /// tick returns the cached previous output and does nothing else: it does not discharge a
        /// pending correction, does not push the jerk history, and does not emit a metric.
        /// <see cref="IReconciler{TState}.Reconcile"/> requires that two calls with the same
        /// <c>nowTicks</c> return the same state and not advance the correction twice; the same guard
        /// covers a frame delivered out of order, for the same reason
        /// <c>Plant/RigidBodyPlant.Step</c> makes a non-advancing step a no-op. Note the consequence:
        /// a repeat call at an already-seen tick ignores <paramref name="predicted"/> entirely, even
        /// if a different value is passed.
        ///
        /// Emits:
        /// <list type="bullet">
        /// <item><c>jerk_mm_s3</c> on <b>every</b> advancing frame, correction or pass-through, from
        /// the four most recent displayed positions -- only once four are available (three retained
        /// plus this one), since before that a third derivative would be an invented number and
        /// inventing one at the start of every trial would bias the metric exactly where the trial
        /// is least representative. Every frame rather than every correction because
        /// docs/metrics.md §5 defines jerk as a property of the displayed trajectory; see
        /// <see cref="Reconcile"/>'s body for why the narrower cadence made this reconciler
        /// incomparable with a smoothed one.</item>
        /// <item><c>time_to_convergence_ms</c> = 0, on the frame a correction is applied.
        /// Convergence completes inside the call that begins it, so the elapsed time is genuinely
        /// zero rather than unmeasured -- the tightest bound <see cref="IReconciler{TState}"/>
        /// permits, and the value every smoothed reconciler's figure should be read against.</item>
        /// </list>
        ///
        /// The jerk history is pushed on <b>every</b> advancing call, corrections and pass-throughs
        /// alike, because docs/metrics.md §5 defines jerk as the third derivative of <i>displayed</i>
        /// position -- the trajectory the snap interrupts is as much a part of that derivative as the
        /// snap itself. Allocation-free.
        /// </summary>
        public Pose Reconcile(Pose predicted, long nowTicks)
        {
            if (nowTicks <= _lastReconcileTicks)
            {
                return _lastOutput;
            }

            bool applied = _hasPendingCorrection;
            Pose output = applied ? _pendingTarget : predicted;
            _hasPendingCorrection = false;

            bool hasJerk = _jerk.TryCompute(
                output.Position, nowTicks, out double jerkMillimetresPerSecondCubed);

            _jerk.Push(output.Position, nowTicks);
            _lastReconcileTicks = nowTicks;
            _lastOutput = output;

            // Jerk is emitted on every advancing frame, correction or pass-through, because
            // docs/metrics.md §5 defines it as the third derivative of *displayed* position -- a
            // property of the trajectory, not of a correction event. Emitting it only on
            // correction frames made this reconciler's sample set incomparable with any reconciler
            // that spreads a correction over many frames: `snap` contributed one sample per
            // correction and `spring` contributed ~100, so pooled percentiles were comparing
            // different populations rather than different reconcilers.
            if (hasJerk)
            {
                _metrics.Record(JerkMetric, jerkMillimetresPerSecondCubed, nowTicks);
            }

            if (applied)
            {
                _metrics.Record(TimeToConvergenceMsMetric, 0.0, nowTicks);
            }

            return output;
        }

        /// <summary>
        /// Returns the reconciler to its as-constructed state: no pending correction, empty jerk
        /// history, no cached output, and -- the one that is easy to miss -- the accepted-capture
        /// baseline back to <see cref="long.MinValue"/> rather than zero. Sweeps reuse instances
        /// across trials, and a reconciler that remembered the previous trial's highest capture stamp
        /// would silently ignore the whole opening stretch of the next one, which looks like a
        /// suspiciously well-behaved baseline rather than like a bug. Same reasoning as
        /// <c>Plant/RigidBodyPlant.Reset</c>. Configuration and the metric sink survive; the sink has
        /// its own <c>Reset</c> and is owned by whoever injected it.
        /// </summary>
        public void Reset()
        {
            ClearCorrectionState();
        }

        private void ClearCorrectionState()
        {
            _jerk.Reset();
            _hasPendingCorrection = false;
            _pendingTarget = Pose.Identity;
            _lastAcceptedCaptureTicks = long.MinValue;
            _lastReconcileTicks = long.MinValue;
            _lastOutput = Pose.Identity;
        }
    }
}
