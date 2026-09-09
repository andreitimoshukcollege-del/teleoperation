using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Reconciliation
{
    /// <summary>
    /// Registry key <c>exp-smooth-c1</c>: a <b>single-time-constant</b> exponential decay of a
    /// residual offset, like the planned <c>exp-smooth</c>, but with the decay <i>rate</i> gated so
    /// that the offset leaves correction onset with exactly zero velocity. It therefore owes no
    /// exception to <c>Reconciliation/CLAUDE.md</c> requirement 2, which a plain single-pole lag
    /// cannot satisfy: that law's offset velocity steps from 0 to <c>|o0|/tau</c> the instant a
    /// correction begins.
    ///
    /// <b>The law.</b> With offset <c>o</c>, a momentum state <c>r</c> (see "Retargets" below), and
    /// a gate <c>g</c> that opens with the same time constant:
    /// <code>
    /// o' = -g(t) o / tau + r        g(0) = 0 at a fresh onset,  g' = (1 - g) / tau
    /// r' = -g(t) r / tau
    /// </code>
    /// so <c>g(t) = 1 - e^(-t/tau)</c> and, from rest (<c>r = 0</c>, which is every onset that is
    /// not a retarget), the offset follows the closed form
    /// <code>
    /// o(t) = o0 * exp(1 - x - e^(-x)),    x = t / tau
    /// </code>
    /// Differentiating, <c>o'(t) = -g(t) o(t) / tau</c> and <c>g(0) = 0</c>, so <b><c>o'(0) = 0</c>
    /// exactly and structurally</b>, not to a tolerance. The envelope is monotone (<c>o' &lt;= 0</c>
    /// whenever <c>o &gt;= 0</c>), so it never crosses zero: <b>no overshoot</b>, provable in closed
    /// form rather than by simulation.
    ///
    /// <b>Why this is not <see cref="SpringReconciler"/>.</b> Any <i>time-invariant</i> two-state
    /// law with a single repeated pole <c>-1/tau</c> has a defective generator, so every solution is
    /// <c>(A + Bt)e^(-t/tau)</c>; imposing <c>o'(0) = 0</c> forces <c>o0 (1 + t/tau) e^(-t/tau)</c>,
    /// which is exactly <c>spring</c>. This law escapes that because it is time-<i>varying</i>: the
    /// system matrix is <c>[[-g/tau, 1], [0, -g/tau]]</c>, gated on the diagonal but not on the
    /// coupling. The consequence is the point of the whole design -- a from-rest seed leaves
    /// <c>r = 0</c>, so the defective (secular) mode is never excited and the envelope stays a
    /// <i>pure</i> exponential. <c>spring</c>, seeded from rest, necessarily excites it and carries
    /// the <c>(1 + wt)</c> factor. Put plainly: <c>spring</c> pays a secular factor to buy zero
    /// initial velocity; the gate buys it for free.
    ///
    /// Measured against <c>spring</c> at the same convergence budget and the same 1% envelope
    /// definition, the closed forms differ by 1.40x in peak offset acceleration and 2.66x in peak
    /// offset jerk while agreeing on peak offset <i>speed</i> to within 0.5%. Those coefficients are
    /// asserted by <c>EasedExponentialReconcilerTests</c> against the trajectory this class actually
    /// produces; the derivation is in <c>docs/research-log/2026-09-08-exp-smooth-c1.md</c>.
    ///
    /// <b>Why an offset rather than a target.</b> Identical to <see cref="SpringReconciler"/>: the
    /// predictor is the thing being corrected, so displaying its output directly <i>is</i> the snap.
    /// At onset the offset is seeded to <c>lastDisplayed - predicted</c>, which makes the displayed
    /// pose continuous across the onset frame by construction, and then decays to zero, so
    /// convergence is toward truth rather than away from it.
    ///
    /// <b>Retargets, and what <c>r</c> is for.</b> A second correction arriving mid-flight must bend
    /// the trajectory, not restart it from rest -- restarting from rest with a non-zero in-flight
    /// offset velocity is itself a velocity step. <c>spring</c> solves this by carrying its offset
    /// <i>velocity</i> across re-seeding. Here velocity is not a state variable (it is
    /// <c>v = r - g o / tau</c>), so <see cref="SeedOffsetFromDisplayedPose"/> computes <c>v</c>
    /// first, re-seeds the offset, and then sets <c>r = v + g o_new / tau</c>, which reproduces the
    /// same <c>v</c> exactly for any seed magnitude and any gate value. That covers both cases:
    /// at a retarget the gate is <i>carried</i> (the trajectory bends), and at a fresh onset the
    /// gate resets to 0 but the residual velocity of the previous episode's still-decaying tail is
    /// still picked up. Without the <c>+ g o_new / tau</c> term the fresh-onset case would step the
    /// velocity by up to <c>tolerance/tau</c> -- small, but a step, and this reconciler's entire
    /// claim is that it needs no exception.
    ///
    /// <b>The one inherited exception, shared with every offset-carrying reconciler here.</b>
    /// Seeding from the <i>previous frame's</i> displayed pose cancels the predictor's jump exactly,
    /// and cancels the predictor's genuine motion over that frame too, so <b>the display is held for
    /// exactly one frame at every onset and retarget</b> -- an O(1) velocity dropout that does not
    /// shrink with the frame interval. It is kept deliberately: the obvious fix (<c>offset -=
    /// error</c>) was implemented across all three smoothed reconcilers and swept, and it was worse
    /// -- jerk p99 regressed 81-865% on impaired profiles and the spread between reconcilers
    /// collapsed to 1.01x, i.e. the metric stopped discriminating between convergence laws at all.
    /// See <c>Reconciliation/CLAUDE.md</c>'s "Tried and rejected" and the <c>..._ADeliberateTradeoff</c>
    /// tests, which this class carries too so that it sits on exactly the same footing as
    /// <c>spring</c> in a head-to-head. <b>"C1-preserving" here is therefore relative to that shared
    /// baseline, not absolute</b>, which is the axis's existing convention and applies to
    /// <c>spring</c> identically.
    ///
    /// <b>Bounded convergence, and its stated bound.</b>
    /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> is read as the time by which the
    /// from-rest envelope must have fallen to <see cref="SettledFractionOfInitialError"/> of the
    /// initial error, which fixes <c>tau</c> -- see <see cref="EasedSettleConstant"/>. The fraction
    /// is deliberately the same constant <see cref="SpringReconciler"/> uses, so that "converged
    /// within the budget" means one thing across the axis. The same two caveats apply verbatim:
    /// <list type="bullet">
    /// <item>The bound is <i>relative</i>. A correction large enough that 1% of it still exceeds
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> needs slightly longer than
    /// the budget to land inside the tolerance. The response shape is independent of correction
    /// magnitude on purpose.</item>
    /// <item>The rate caps take precedence. A cap low enough to clip the response's peak speed makes
    /// the correction converge <i>slower</i> than the budget. That is the documented interaction,
    /// not a bug.</item>
    /// </list>
    ///
    /// <b>Fields of <see cref="ReconcilerConfig"/> it reads:</b> both tolerances,
    /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> (sets <c>tau</c>), and both rate
    /// caps. <b>Field it ignores:</b> <see cref="ReconcilerConfig.RollbackHistoryCapacity"/>, as
    /// every non-<c>rollback</c> entry does. No new configuration field: the gate ramps with the
    /// <i>same</i> <c>tau</c> as the decay, which is what keeps this a single-time-constant law. A
    /// separate gate constant would turn it into a two-pole overdamped system, which is strictly
    /// worse than <c>spring</c> on peak jerk -- see the research log.
    ///
    /// <b>Time is always a parameter.</b> The injected <see cref="ITimeAuthority"/> is read for
    /// <see cref="ITimeAuthority.TicksPerSecond"/> only and never for
    /// <see cref="ITimeAuthority.NowTicks"/>.
    ///
    /// Deterministic and allocation-free: the only buffer is the shared jerk estimator's history,
    /// sized once in the constructor. Not thread-safe, by contract.
    /// </summary>
    public sealed class EasedExponentialReconciler : IReconciler<Pose>
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
        /// Core works in metres and radians (ROS convention); docs/metrics.md reports millimetres
        /// and degrees. The conversion happens here, at the reporting boundary, and nowhere else.
        /// </summary>
        private const double MetresToMillimetres = 1000.0;

        private const double RadiansToDegrees = 180.0 / Math.PI;

        private const double MillisecondsPerSecond = 1000.0;

        /// <summary>
        /// The fraction of the initial error the from-rest envelope must have decayed to by
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>. Deliberately the same value
        /// <see cref="SpringReconciler"/> uses, so that a budget means the same thing for both and a
        /// head-to-head compares convergence laws rather than two readings of one config field. A
        /// definition, not a tuning knob.
        /// </summary>
        private const float SettledFractionOfInitialError = 0.01f;

        /// <summary>
        /// The <c>x</c> solving <c>exp(1 - x - e^(-x)) = </c>
        /// <see cref="SettledFractionOfInitialError"/>, equivalently <c>x + e^(-x) = 1 + ln(100)</c>,
        /// so that <c>tau = budgetSeconds / x</c> puts the envelope exactly at that fraction when the
        /// budget elapses. There is no closed form, so the constant is precomputed rather than
        /// solved at construction -- the same reasoning
        /// <see cref="SpringReconciler"/>'s <c>CriticalSettleConstant</c> gives: a numeric solve per
        /// instance would be startup cost for a value that can never change, and a literal is
        /// auditable where an iteration is not. Verified by
        /// <c>EasedExponentialReconcilerTests.EasedSettleConstantMatchesItsDefiningEquation</c>,
        /// which recomputes the envelope and asserts it lands on the fraction, so the two constants
        /// cannot drift apart silently.
        ///
        /// Note it is smaller than <c>spring</c>'s 6.6383521: leaving the origin at rest costs
        /// exactly one time constant of delay relative to a plain single-pole lag
        /// (<c>ln(100) + 1 = 5.6052</c> before the <c>e^(-x)</c> correction term), and hitting the
        /// same deadline therefore requires a 17.8% shorter <c>tau</c> than the plain law would.
        /// </summary>
        private const float EasedSettleConstant = 5.6014778f;

        private readonly float _positionTolerance;
        private readonly float _orientationTolerance;
        private readonly float _maxLinearSpeed;
        private readonly float _maxAngularSpeed;
        private readonly float _tau;
        private readonly float _inverseTau;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        /// <summary>
        /// Shared with every other reconciler so that <c>jerk_mm_s3</c> means one thing across the
        /// axis being compared -- see <see cref="DisplayedJerkEstimator"/> for why that matters.
        /// </summary>
        private readonly DisplayedJerkEstimator _jerk;

        private Vector3 _offsetPosition;
        private Vector3 _momentumLinear;
        private Vector3 _offsetRotation;
        private Vector3 _momentumAngular;

        /// <summary>
        /// The gate, in [0, 1). Zero at a fresh correction onset -- which is what makes the offset
        /// leave onset at zero velocity -- and opening with time constant <see cref="_tau"/>. Shared
        /// by the positional and rotational offsets so that the two converge on the same schedule,
        /// exactly as <c>spring</c> shares one natural frequency between them.
        /// </summary>
        private float _gate;

        private bool _correctionPending;
        private bool _correctionInFlight;
        private long _correctionOnsetTicks;

        /// <summary>
        /// Highest authoritative <see cref="Stamped{T}.CaptureTicks"/> accepted so far. Starts at
        /// <see cref="long.MinValue"/> so the very first sample is accepted whatever its stamp,
        /// including zero and negative.
        /// </summary>
        private long _lastAcceptedCaptureTicks;

        private long _lastReconcileTicks;
        private Pose _lastOutput;
        private bool _hasLastOutput;

        /// <param name="config">
        /// Parameters; see the type doc for which fields are read. Like <c>spring</c> and unlike
        /// <c>snap</c>, this reconciler <b>requires</b> a positive
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> and positive rate caps: a zero
        /// budget has no defined response and a zero rate cap could never converge, which would
        /// violate the bounded-convergence clause rather than express a tradeoff.
        /// </param>
        /// <param name="metrics">
        /// Correction-cost sink (docs/metrics.md §5). A constructor dependency rather than a
        /// per-frame parameter so that <see cref="Reconcile"/>'s signature stays allocation-free.
        /// </param>
        /// <param name="clock">
        /// Read for <see cref="ITimeAuthority.TicksPerSecond"/> only -- never
        /// <see cref="ITimeAuthority.NowTicks"/>.
        /// </param>
        public EasedExponentialReconciler(
            ReconcilerConfig config, IMetricSink metrics, ITimeAuthority clock)
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

            if (config.MaxTimeToConvergenceTicks <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxTimeToConvergenceTicks,
                    "MaxTimeToConvergenceTicks must be positive: it sets the time constant of both " +
                    "the decay and the gate, so there is no defined response without it.");
            }

            if (config.MaxCorrectionLinearSpeedMetersPerSecond <= 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxCorrectionLinearSpeedMetersPerSecond,
                    "MaxCorrectionLinearSpeedMetersPerSecond must be positive: a zero cap can " +
                    "never converge.");
            }

            if (config.MaxCorrectionAngularSpeedRadPerSecond <= 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxCorrectionAngularSpeedRadPerSecond,
                    "MaxCorrectionAngularSpeedRadPerSecond must be positive: a zero cap can " +
                    "never converge.");
            }

            _positionTolerance = config.ConvergencePositionToleranceMeters;
            _orientationTolerance = config.ConvergenceOrientationToleranceRadians;
            _maxLinearSpeed = config.MaxCorrectionLinearSpeedMetersPerSecond;
            _maxAngularSpeed = config.MaxCorrectionAngularSpeedRadPerSecond;
            _metrics = metrics;
            _ticksPerSecond = clock.TicksPerSecond;

            float budgetSeconds = config.MaxTimeToConvergenceTicks / (float)clock.TicksPerSecond;
            _tau = budgetSeconds / EasedSettleConstant;
            _inverseTau = 1f / _tau;

            _jerk = new DisplayedJerkEstimator(clock.TicksPerSecond);

            ClearCorrectionState();
        }

        /// <summary>
        /// The time constant in seconds derived from
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>. It governs both the decay and
        /// the gate, and it is the single number that characterises this reconciler's response.
        /// Exposed for the same reason <see cref="SpringReconciler.NaturalFrequencyRadiansPerSecond"/>
        /// is: a test that had to re-derive it would be asserting against its own copy of the
        /// formula rather than against the implementation.
        /// </summary>
        public float TimeConstantSeconds => _tau;

        /// <summary>
        /// True when no correction is pending and none is in flight -- the displayed pose is the
        /// predictor's output with a residual offset inside tolerance. True on a freshly constructed
        /// or freshly <see cref="Reset"/> instance, as
        /// <see cref="IReconciler{TState}.IsConverged"/> requires.
        /// </summary>
        public bool IsConverged => !_correctionPending && !_correctionInFlight;

        /// <summary>
        /// Truth arrived. Measures the disagreement against what was displayed for that same instant
        /// and, if it exceeds tolerance, flags that the offset must be re-seeded on the next
        /// <see cref="Reconcile"/>. Changes nothing visible here, per
        /// <see cref="IReconciler{TState}.Observe"/>.
        ///
        /// Byte-for-byte the same acceptance rules, tolerance short circuit, re-seed-don't-stack
        /// behaviour and metric emission as <see cref="SnapReconciler"/> and
        /// <see cref="SpringReconciler"/>: a sample whose capture stamp is not strictly greater than
        /// the last accepted one is stale or a duplicate and is ignored whole (a duplicate counted
        /// as a second correction would corrupt the reported correction rate); a disagreement inside
        /// both tolerances is no correction at all rather than a zero-magnitude one, so a
        /// passthrough predictor reduces the pipeline to exact pass-through. Those definitions are
        /// shared on purpose -- <c>correction_magnitude_mm</c> is a control across this axis, so it
        /// must be identical by construction.
        ///
        /// Allocation-free.
        /// </summary>
        /// <param name="diagnostics">
        /// Ignored. This reconciler's response is set by the convergence budget, not by predictor
        /// uncertainty, so it behaves identically whether
        /// <see cref="PredictorDiagnostics.HasUncertainty"/> is true or false -- the graceful
        /// degradation <see cref="IReconciler{TState}.Observe"/> asks for, reached by not depending
        /// on the field at all. <see cref="PredictorDiagnostics.None"/> is equally fine.
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

            _correctionPending = true;

            // Stamped at the sample's own capture time, matching SnapReconciler and SpringReconciler
            // exactly: the metric definitions must be identical across the axis or the comparison
            // measures the metric rather than the reconciler.
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
        /// The per-frame call: advances the residual offset, its momentum and the gate to
        /// <paramref name="nowTicks"/> and returns <paramref name="predicted"/> displaced by what
        /// remains of the offset.
        ///
        /// <b>Idempotent in <paramref name="nowTicks"/>.</b> A call at or before the last reconciled
        /// tick returns the cached previous output and does nothing else -- no decay step, no jerk
        /// push, no metric. The same guard covers a frame delivered out of order.
        ///
        /// <b>The first advancing call after a pending correction</b> re-seeds the offset from the
        /// live displayed pose and does not decay, because there is no elapsed interval to decay
        /// over; decay begins on the following frame. On the very first call of a trial there is no
        /// previously displayed pose to stay continuous with, so the offset stays zero and
        /// <paramref name="predicted"/> is returned unmodified -- correcting toward a pose that was
        /// never displayed would be inventing a discontinuity rather than hiding one.
        ///
        /// Emits:
        /// <list type="bullet">
        /// <item><c>jerk_mm_s3</c> on <b>every</b> advancing frame once four displayed positions
        /// exist -- the same unconditional cadence <see cref="SnapReconciler"/> and
        /// <see cref="SpringReconciler"/> use, which is what makes the distributions populations of
        /// the same thing (displayed-trajectory jerk) rather than one sample per correction against
        /// a hundred. Emitting on a different schedule would make the percentiles compare
        /// populations rather than algorithms.</item>
        /// <item><c>time_to_convergence_ms</c>, once per convergence episode, on the frame the
        /// offset first falls inside tolerance, measured from correction onset.</item>
        /// </list>
        ///
        /// Allocation-free.
        /// </summary>
        public Pose Reconcile(Pose predicted, long nowTicks)
        {
            if (_hasLastOutput && nowTicks <= _lastReconcileTicks)
            {
                return _lastOutput;
            }

            bool hasElapsed = _hasLastOutput && _lastReconcileTicks != long.MinValue;
            float elapsedSeconds = hasElapsed
                ? (nowTicks - _lastReconcileTicks) / (float)_ticksPerSecond
                : 0f;

            if (_correctionPending)
            {
                SeedOffsetFromDisplayedPose(predicted, nowTicks);
                _correctionPending = false;
            }
            else if (elapsedSeconds > 0f && HasResidualOffset())
            {
                AdvanceOffset(elapsedSeconds);
            }

            if (_correctionInFlight && OffsetIsWithinTolerance())
            {
                // Stop *reporting* the correction, but do not zero the residual. Truncating it to
                // zero once it fell inside tolerance is a position discontinuity of up to one
                // tolerance, which across a frame is a velocity step of tolerance/dt -- invisible in
                // absolute terms but unbounded as the frame interval shrinks, so it would falsify
                // the C1 claim at exactly the frame rates a headset runs at. The residual instead
                // keeps decaying to zero, which keeps the visible output C1 and still makes
                // Compose's pass-through short circuit reachable again. Same reasoning, and the same
                // behaviour, as SpringReconciler.
                _correctionInFlight = false;

                _metrics.Record(
                    TimeToConvergenceMsMetric,
                    (nowTicks - _correctionOnsetTicks) / (double)_ticksPerSecond * MillisecondsPerSecond,
                    nowTicks);
            }

            Pose output = Compose(predicted);

            bool hasJerk = _jerk.TryCompute(
                output.Position, nowTicks, out double jerkMillimetresPerSecondCubed);

            _jerk.Push(output.Position, nowTicks);
            _lastReconcileTicks = nowTicks;
            _lastOutput = output;
            _hasLastOutput = true;

            if (hasJerk)
            {
                _metrics.Record(JerkMetric, jerkMillimetresPerSecondCubed, nowTicks);
            }

            return output;
        }

        /// <summary>
        /// Returns the reconciler to its as-constructed state: no offset, no momentum, a closed
        /// gate, nothing pending or in flight, an empty jerk history, and -- the one that is easy to
        /// miss -- the accepted-capture baseline back to <see cref="long.MinValue"/> rather than
        /// zero. Sweeps reuse instances across trials, and a reconciler that remembered the previous
        /// trial's highest capture stamp would silently ignore the whole opening stretch of the next
        /// one, which looks like a suspiciously well-behaved result rather than like a bug.
        /// Configuration and the metric sink survive; the sink has its own <c>Reset</c> and is owned
        /// by whoever injected it.
        /// </summary>
        public void Reset()
        {
            ClearCorrectionState();
        }

        private void ClearCorrectionState()
        {
            _jerk.Reset();
            _offsetPosition = Vector3.Zero;
            _momentumLinear = Vector3.Zero;
            _offsetRotation = Vector3.Zero;
            _momentumAngular = Vector3.Zero;
            _gate = 0f;
            _correctionPending = false;
            _correctionInFlight = false;
            _correctionOnsetTicks = 0;
            _lastAcceptedCaptureTicks = long.MinValue;
            _lastReconcileTicks = long.MinValue;
            _lastOutput = Pose.Identity;
            _hasLastOutput = false;
        }

        /// <summary>
        /// Sets the offset to whatever keeps the displayed pose continuous across this frame:
        /// <c>lastDisplayed - predicted</c> in position, and the world-frame rotation vector taking
        /// <c>predicted</c> to <c>lastDisplayed</c> in orientation.
        ///
        /// <b>The offset's velocity is preserved across the re-seed, exactly.</b> Velocity here is
        /// <c>v = r - g o / tau</c> rather than a state variable, so it is read <i>before</i> the
        /// offset changes and the momentum is then chosen to reproduce it:
        /// <c>r_new = v + g o_new / tau</c>. This is the C1-at-retarget guarantee, and it is written
        /// once and applied to both cases rather than special-cased:
        /// <list type="bullet">
        /// <item><b>Retarget</b> (a correction while an episode is in flight): the gate is carried,
        /// so the trajectory bends rather than restarting from rest.</item>
        /// <item><b>Fresh onset</b>: the gate resets to zero, which is what gives a from-rest
        /// correction its <c>o'(0) = 0</c>. The velocity term still picks up whatever residual
        /// motion the previous episode's decaying tail had, which would otherwise be a step of up to
        /// <c>tolerance/tau</c>.</item>
        /// </list>
        ///
        /// Onset ticks are recorded only when a correction was not already in flight, so
        /// <c>time_to_convergence_ms</c> measures one convergence episode from its true start rather
        /// than restarting the clock on every re-seed.
        /// </summary>
        private void SeedOffsetFromDisplayedPose(in Pose predicted, long nowTicks)
        {
            if (!_hasLastOutput)
            {
                // Nothing has been displayed yet: there is no discontinuity to hide.
                _offsetPosition = Vector3.Zero;
                _offsetRotation = Vector3.Zero;
                return;
            }

            // Read the in-flight offset velocity before anything changes.
            Vector3 linearVelocity = _momentumLinear - _gate * _inverseTau * _offsetPosition;
            Vector3 angularVelocity = _momentumAngular - _gate * _inverseTau * _offsetRotation;

            if (!_correctionInFlight)
            {
                _correctionOnsetTicks = nowTicks;
                _correctionInFlight = true;

                // A fresh episode closes the gate, which is the whole mechanism: the offset then
                // leaves onset with zero rate of its own and the response is C1 without an
                // exception.
                _gate = 0f;
            }

            _offsetPosition = _lastOutput.Position - predicted.Position;
            _offsetRotation =
                MotionMath.RelativeRotationVector(predicted.Rotation, _lastOutput.Rotation);

            // Choose the momentum that reproduces the velocity just read, under the (possibly
            // reset) gate. v = r - g o / tau, so r = v + g o / tau.
            _momentumLinear = linearVelocity + _gate * _inverseTau * _offsetPosition;
            _momentumAngular = angularVelocity + _gate * _inverseTau * _offsetRotation;
        }

        /// <summary>
        /// One <b>exact</b> step of the gated decay, applied to the position offset and the
        /// rotation-vector offset independently but under one shared gate.
        ///
        /// For <c>o' = -g o / tau + r</c>, <c>r' = -g r / tau</c>, <c>g' = (1 - g)/tau</c> with
        /// <c>c = 1 - g</c> and <c>x = dt/tau</c>, the solution over the step is
        /// <code>
        /// invMu = exp(-x + c (1 - e^(-x)))      // = exp(-(1/tau) * integral of g over the step)
        /// o(dt) = (o + r dt) * invMu
        /// r(dt) =  r        * invMu
        /// g(dt) = 1 - c e^(-x)
        /// </code>
        /// Closed form rather than a numerical integrator, for the reason
        /// <see cref="SpringReconciler"/> gives for its own: exact, unconditionally stable where a
        /// semi-implicit Euler step would diverge for large <c>dt/tau</c> (reachable with a tight
        /// budget and a slow frame, i.e. exactly the impaired conditions this project studies), and
        /// frame-rate independent, so a sweep and a headset produce the same trajectory from the
        /// same samples. Composing the step over unequal intervals telescopes exactly to the
        /// closed-form envelope in the type doc -- the gate's <c>c</c> multiplies geometrically and
        /// the <c>(1 - e^(-x))</c> factors sum -- which is what
        /// <c>TheOffsetFollowsItsClosedFormEnvelope</c> asserts.
        ///
        /// Scaling the rotation vector is exact rather than a small-angle approximation: shrinking a
        /// rotation vector keeps its axis fixed and scales only its angle, which is the geodesic path
        /// to zero rotation.
        ///
        /// The rate caps are applied to the offset <i>velocity</i> after the step, via
        /// <see cref="MotionMath.ClampMagnitude"/>, and folded back into the momentum. Clamping a
        /// magnitude is continuous, so it cannot introduce the velocity discontinuity the C1 clause
        /// forbids; it only stretches convergence.
        /// </summary>
        private void AdvanceOffset(float elapsedSeconds)
        {
            float x = elapsedSeconds * _inverseTau;
            float decay = MathF.Exp(-x);
            float closedFraction = 1f - _gate;
            float gateAfter = 1f - closedFraction * decay;
            float invMu = MathF.Exp(-x + closedFraction * (1f - decay));

            StepGated(
                ref _offsetPosition, ref _momentumLinear,
                elapsedSeconds, invMu, gateAfter, _maxLinearSpeed);
            StepGated(
                ref _offsetRotation, ref _momentumAngular,
                elapsedSeconds, invMu, gateAfter, _maxAngularSpeed);

            _gate = gateAfter;
        }

        private void StepGated(
            ref Vector3 offset,
            ref Vector3 momentum,
            float elapsedSeconds,
            float invMu,
            float gateAfter,
            float maxSpeed)
        {
            if (invMu <= 0f)
            {
                // Underflow: a gap of many time constants. The limit is exactly zero, and taking it
                // directly avoids multiplying a large momentum-times-dt term by a zero factor, which
                // is the one path that could produce a NaN from a long transport stall.
                offset = Vector3.Zero;
                momentum = Vector3.Zero;
                return;
            }

            Vector3 nextOffset = (offset + momentum * elapsedSeconds) * invMu;
            Vector3 nextMomentum = momentum * invMu;

            Vector3 gatedOffsetRate = gateAfter * _inverseTau * nextOffset;
            Vector3 velocity = MotionMath.ClampMagnitude(nextMomentum - gatedOffsetRate, maxSpeed);

            offset = nextOffset;
            momentum = velocity + gatedOffsetRate;
        }

        /// <summary>
        /// True while any residual offset or momentum remains, so the decay keeps running after the
        /// correction stops being reported. Convergence ends a reporting episode, not the motion:
        /// see <see cref="Reconcile"/> for why truncating the residual instead would break C1
        /// continuity.
        /// </summary>
        private bool HasResidualOffset() =>
            _offsetPosition != Vector3.Zero ||
            _momentumLinear != Vector3.Zero ||
            _offsetRotation != Vector3.Zero ||
            _momentumAngular != Vector3.Zero;

        /// <summary>
        /// True when what remains of the offset is inside both configured tolerances. Compared as
        /// magnitudes of the residual itself rather than via <see cref="PoseMath"/> on the composed
        /// poses, matching <see cref="SpringReconciler"/>: the offset <i>is</i> the error, so
        /// measuring it directly cannot disagree with what the decay is acting on.
        /// </summary>
        private bool OffsetIsWithinTolerance() =>
            _offsetPosition.Length() <= _positionTolerance &&
            _offsetRotation.Length() <= _orientationTolerance;

        /// <summary>
        /// The displayed pose: <paramref name="predicted"/> displaced by the residual offset.
        /// Returns <paramref name="predicted"/> bit-identically when the offset is exactly zero, so
        /// that the degenerate pass-through case reproduces its input rather than differing by
        /// quaternion renormalization noise.
        /// </summary>
        private Pose Compose(in Pose predicted)
        {
            if (_offsetPosition == Vector3.Zero && _offsetRotation == Vector3.Zero)
            {
                return predicted;
            }

            return new Pose(
                predicted.Position + _offsetPosition,
                MotionMath.IntegrateWorld(predicted.Rotation, _offsetRotation));
        }
    }
}
