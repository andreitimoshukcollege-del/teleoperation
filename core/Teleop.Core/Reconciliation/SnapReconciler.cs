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

        /// <summary>
        /// Samples of displayed position retained for the jerk estimate. Three, because a third
        /// derivative needs four points: these three plus the one being produced. Not a research
        /// knob -- it is the arity of the finite difference.
        /// </summary>
        private const int JerkHistoryLength = 3;

        private readonly float _positionTolerance;
        private readonly float _orientationTolerance;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        private readonly Vector3[] _historyPositions;
        private readonly long[] _historyTicks;
        private int _historyCount;

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
            _ticksPerSecond = clock.TicksPerSecond;

            _historyPositions = new Vector3[JerkHistoryLength];
            _historyTicks = new long[JerkHistoryLength];

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
        /// Emits, on the frame a correction is applied:
        /// <list type="bullet">
        /// <item><c>time_to_convergence_ms</c> = 0. Convergence completes inside the call that
        /// begins it, so the elapsed time is genuinely zero rather than unmeasured -- the tightest
        /// bound <see cref="IReconciler{TState}"/> permits, and the value every smoothed reconciler's
        /// figure should be read against.</item>
        /// <item><c>jerk_mm_s3</c>, from the four most recent displayed positions. Emitted only once
        /// four are available (three retained plus this one); before that a third derivative would be
        /// an invented number, and inventing one at the start of every trial would bias the metric
        /// exactly where the trial is least representative.</item>
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

            bool hasJerk = TryComputeJerkMillimetresPerSecondCubed(
                output.Position, nowTicks, out double jerkMillimetresPerSecondCubed);

            PushDisplayedPosition(output.Position, nowTicks);
            _lastReconcileTicks = nowTicks;
            _lastOutput = output;

            if (applied)
            {
                _metrics.Record(TimeToConvergenceMsMetric, 0.0, nowTicks);

                if (hasJerk)
                {
                    _metrics.Record(JerkMetric, jerkMillimetresPerSecondCubed, nowTicks);
                }
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
            _historyCount = 0;
            _hasPendingCorrection = false;
            _pendingTarget = Pose.Identity;
            _lastAcceptedCaptureTicks = long.MinValue;
            _lastReconcileTicks = long.MinValue;
            _lastOutput = Pose.Identity;
        }

        /// <summary>
        /// Appends a displayed position to the three-slot history, dropping the oldest when full.
        /// A shift over three elements rather than a ring index: at this length it is cheaper than
        /// the modulo bookkeeping and leaves the array in oldest-first order, which is what the
        /// finite difference wants.
        /// </summary>
        private void PushDisplayedPosition(in Vector3 position, long nowTicks)
        {
            if (_historyCount < JerkHistoryLength)
            {
                _historyPositions[_historyCount] = position;
                _historyTicks[_historyCount] = nowTicks;
                _historyCount++;
                return;
            }

            for (int i = 0; i < JerkHistoryLength - 1; i++)
            {
                _historyPositions[i] = _historyPositions[i + 1];
                _historyTicks[i] = _historyTicks[i + 1];
            }

            _historyPositions[JerkHistoryLength - 1] = position;
            _historyTicks[JerkHistoryLength - 1] = nowTicks;
        }

        /// <summary>
        /// Third derivative of displayed position at <paramref name="nowTicks"/>, in mm/s³
        /// (docs/metrics.md §5), from the three retained samples plus the one about to be displayed.
        /// Returns false when fewer than three are retained.
        ///
        /// Computed as a cascade of central differences on <b>unequally spaced</b> samples, which is
        /// what a frame schedule actually produces: three velocities located at the midpoints of the
        /// three intervals, two accelerations at the midpoints of those, and one jerk from the two
        /// accelerations. Every division is by a strictly positive interval, since
        /// <see cref="Reconcile"/> only pushes on strictly increasing ticks.
        ///
        /// Accumulated in <c>double</c>, per <see cref="IMetricSink.Record"/>'s note that the
        /// parameter is <c>double</c> "so tick differences and jerk values survive without precision
        /// loss": a snap divides a metre-scale step by a millisecond-scale interval three times, and
        /// the intermediate magnitudes leave little of a float's mantissa.
        /// </summary>
        private bool TryComputeJerkMillimetresPerSecondCubed(
            in Vector3 position, long nowTicks, out double jerkMillimetresPerSecondCubed)
        {
            if (_historyCount < JerkHistoryLength)
            {
                jerkMillimetresPerSecondCubed = 0.0;
                return false;
            }

            // Seconds relative to the oldest retained sample. Relative, so the absolute tick
            // magnitude never enters the arithmetic.
            long originTicks = _historyTicks[0];
            double t0 = 0.0;
            double t1 = (_historyTicks[1] - originTicks) / (double)_ticksPerSecond;
            double t2 = (_historyTicks[2] - originTicks) / (double)_ticksPerSecond;
            double t3 = (nowTicks - originTicks) / (double)_ticksPerSecond;

            Vector3 p0 = _historyPositions[0];
            Vector3 p1 = _historyPositions[1];
            Vector3 p2 = _historyPositions[2];

            double jerkX = Jerk1D(p0.X, p1.X, p2.X, position.X, t0, t1, t2, t3);
            double jerkY = Jerk1D(p0.Y, p1.Y, p2.Y, position.Y, t0, t1, t2, t3);
            double jerkZ = Jerk1D(p0.Z, p1.Z, p2.Z, position.Z, t0, t1, t2, t3);

            double magnitudeMetresPerSecondCubed =
                Math.Sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

            jerkMillimetresPerSecondCubed = magnitudeMetresPerSecondCubed * MetresToMillimetres;
            return true;
        }

        /// <summary>
        /// One axis of the unequally-spaced third derivative described on
        /// <see cref="TryComputeJerkMillimetresPerSecondCubed"/>. Positions in metres, times in
        /// seconds, result in metres/second³.
        /// </summary>
        private static double Jerk1D(
            double p0, double p1, double p2, double p3,
            double t0, double t1, double t2, double t3)
        {
            double v01 = (p1 - p0) / (t1 - t0);
            double v12 = (p2 - p1) / (t2 - t1);
            double v23 = (p3 - p2) / (t3 - t2);

            double tv01 = 0.5 * (t0 + t1);
            double tv12 = 0.5 * (t1 + t2);
            double tv23 = 0.5 * (t2 + t3);

            double a0 = (v12 - v01) / (tv12 - tv01);
            double a1 = (v23 - v12) / (tv23 - tv12);

            double ta0 = 0.5 * (tv01 + tv12);
            double ta1 = 0.5 * (tv12 + tv23);

            return (a1 - a0) / (ta1 - ta0);
        }
    }
}
