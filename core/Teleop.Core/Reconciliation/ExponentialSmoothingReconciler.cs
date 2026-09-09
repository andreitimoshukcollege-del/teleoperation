using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Reconciliation
{
    /// <summary>
    /// Registry key <c>exp-smooth</c>: the literal single-pole lag. Carry the visible disagreement as a
    /// <b>residual offset</b> on top of the predictor's output, the way <see cref="SpringReconciler"/>
    /// does, and drive that offset to zero with <b>one time constant</b> --
    /// <c>offset *= exp(-dt / tau)</c> -- rather than with a second-order response. There is no
    /// offset-velocity state anywhere in this class, and that is the point: this is the simplest thing
    /// on the axis, and the row exists to find out what the simplicity costs.
    ///
    /// <b>This reconciler is C0 but NOT C1, and the violation is the reason it is interesting.</b> See
    /// "The exception to requirement 2" below, which states its exact size in closed form. It is not
    /// disguised, not smoothed away, and not argued around; it is measured, by
    /// <c>ExponentialSmoothingReconcilerTests</c>, the same way <see cref="SnapReconciler"/>'s suite
    /// measures the one deliberate exception the axis has already granted.
    ///
    /// <b>Why an offset rather than a target.</b> Identical to <see cref="SpringReconciler"/>'s
    /// reasoning, and deliberately so, because the two differ only in the convergence law: once
    /// <c>Observe</c> has reached the predictor its output already incorporates truth, so displaying it
    /// directly <i>is</i> the snap. What this class hides is the jump the predictor's own correction
    /// caused. The offset is therefore seeded at onset to <c>lastDisplayed - predicted</c>, which makes
    /// the displayed <i>position</i> continuous across the onset frame by construction, and decays to
    /// zero, so the display converges onto the corrected prediction rather than onto a stale sample.
    ///
    /// <b>The exception to requirement 2, in closed form.</b> The offset velocity of a single pole is
    /// <c>o'(t) = -o(t)/tau</c>. Before onset the offset is identically zero, so <c>o'(t0-) = 0</c>;
    /// one instant after seeding, <c>o'(t0+) = -o0/tau</c>. That is a velocity <i>step</i> of
    ///
    /// <code>
    ///   |dv| = |o0| / tau = |o0| * ln(1/f) / T   =   4.6052 * |o0| / T     (f = 1%, T = the budget)
    /// </code>
    ///
    /// and it is inherent to the law: a first-order system reaches its terminal rate instantly, which
    /// is exactly what "one time constant, no second-order state" means. Three consequences, each
    /// pinned by a test rather than asserted here:
    /// <list type="bullet">
    /// <item><b>It scales linearly with correction magnitude</b> and <b>inversely with the convergence
    /// budget</b>. Doubling the correction doubles the step; halving the budget doubles it. This is
    /// the scaling a human weighing an amendment to requirement 2 has to weigh, because it means the
    /// violation is <i>largest exactly on the largest corrections</i> -- <b>up to the rate cap</b>.
    /// The true form is <c>min(|o0|/tau, v_max)</c>: the first advancing frame removes
    /// <c>offset * (1 - e^(-dt/tau))</c> clamped to
    /// <see cref="ReconcilerConfig.MaxCorrectionLinearSpeedMetersPerSecond"/> times the frame
    /// interval, so above <c>|o0| = v_max * tau</c> the step stops growing with the correction and
    /// becomes a configured constant. Quoting the linear law without this overstates the exception on
    /// exactly the large corrections it is meant to describe.</item>
    /// <item><b>It does not shrink as the frame interval is refined</b> -- it is <c>O(1)</c> in
    /// <c>dt</c>, converging to <c>|o0|/tau</c> rather than to zero. A C1 response's per-frame velocity
    /// step is <c>O(dt)</c>; a snap's is <c>O(1/dt)</c> and grows. This one sits between them and is
    /// <i>finite</i>, which is the whole of what the single pole buys.</item>
    /// <item><b>In <c>jerk_mm_s3</c>, the metric this axis is compared on, it still diverges</b> --
    /// as <c>O(1/dt^2)</c>, against a snap's <c>O(1/dt^3)</c>. Finite velocity step, unbounded sampled
    /// jerk. Both facts are true and reporting only the first would flatter the design.</item>
    /// </list>
    ///
    /// A <b>retarget</b> -- a second qualifying sample arriving mid-decay -- steps the offset velocity
    /// again, from <c>-o_old/tau</c> to <c>-o_new/tau</c>, i.e. by <c>|o_new - o_old|/tau</c>.
    /// <see cref="SpringReconciler"/> avoids this by carrying its offset velocity across the re-seed;
    /// there is no velocity here to carry, so the retarget step is inherent too, and is measured.
    ///
    /// <b>The one-frame hold is inherited, deliberately.</b> Seeding from the previous frame's
    /// displayed pose cancels the predictor's jump exactly, and as a side effect cancels the
    /// predictor's genuine motion over that frame, so the display is held for exactly one frame at
    /// every onset and retarget. That is an <c>O(1)</c> velocity dropout shared with every
    /// offset-carrying reconciler on this axis. The obvious fix (<c>offset -= error</c>) was
    /// implemented across the smoothed reconcilers, swept, and measured worse -- jerk p99 regressed
    /// 81-865% on impaired profiles and the spread between reconcilers collapsed to 1.01x, i.e. the
    /// metric stopped discriminating between convergence laws. It was reverted. See
    /// <c>Reconciliation/CLAUDE.md</c>'s "Tried and rejected" and the <c>..._ADeliberateTradeoff</c>
    /// tests. Inheriting it unchanged is what keeps this candidate on the same footing as the rest of
    /// the axis.
    ///
    /// <b>Bounded convergence (requirement 1), and its stated bound.</b> The envelope is exactly
    /// <c>|o(t)| = |o0| e^(-t/tau)</c>. <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> is
    /// read as the time by which that envelope must have fallen to
    /// <see cref="SettledFractionOfInitialError"/> of the initial error, which fixes <c>tau</c> -- see
    /// <see cref="SettleConstant"/>. Same 1% fraction <see cref="SpringReconciler"/> uses, so that
    /// "converged within the budget" means one thing across the axis rather than one thing per
    /// implementation. Two honest caveats, both inherited from <c>spring</c> verbatim:
    /// <list type="bullet">
    /// <item>The bound is <i>relative</i>. A correction large enough that 1% of it still exceeds
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> needs longer than the budget
    /// to land inside the tolerance. The response shape is deliberately independent of correction
    /// magnitude.</item>
    /// <item>The rate caps take precedence and stretch convergence rather than breaking it. The bound
    /// with a binding cap is <c>|o0| / v_max + T</c>: while capped the offset shrinks strictly at
    /// <c>v_max</c>, which can last at most <c>|o0|/v_max</c>, and from whatever remains the uncapped
    /// exponential reaches 1% within <c>T</c>.</item>
    /// </list>
    ///
    /// <b>Fields of <see cref="ReconcilerConfig"/> it reads:</b> all but one --
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> and
    /// <see cref="ReconcilerConfig.ConvergenceOrientationToleranceRadians"/>,
    /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> (sets <c>tau</c>), and both rate caps.
    /// <b>Field it ignores:</b> <see cref="ReconcilerConfig.RollbackHistoryCapacity"/>. Nothing here
    /// rewinds.
    ///
    /// <b>Time is always a parameter.</b> The injected <see cref="ITimeAuthority"/> is read for
    /// <see cref="ITimeAuthority.TicksPerSecond"/> only and never for
    /// <see cref="ITimeAuthority.NowTicks"/>, the same discipline <see cref="SnapReconciler"/> and
    /// <see cref="SpringReconciler"/> document.
    ///
    /// Deterministic and allocation-free: the only buffer is the shared jerk estimator's history,
    /// sized once in the constructor. Not thread-safe, by contract.
    /// </summary>
    public sealed class ExponentialSmoothingReconciler : IReconciler<Pose>
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
        /// The fraction of the initial error the exponential envelope must have decayed to by
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>. Deliberately the same 1%
        /// <see cref="SpringReconciler"/> uses: the two reconcilers differ in convergence <i>law</i>,
        /// and if they also differed in what "converged within the budget" means, a comparison between
        /// them would be measuring the definition rather than the law. A definition, not a tuning
        /// knob -- changing it changes what every recorded result means.
        /// </summary>
        private const float SettledFractionOfInitialError = 0.01f;

        /// <summary>
        /// The <c>x</c> solving <c>e^(-x) = </c><see cref="SettledFractionOfInitialError"/>, so that
        /// <c>tau = budgetSeconds / x</c> puts the envelope exactly at that fraction when the budget
        /// elapses. Unlike <see cref="SpringReconciler"/>'s critically damped constant -- a Lambert-W
        /// root with no closed form -- this one is simply <c>ln(100)</c>, so
        /// <c>ExponentialSmoothingReconcilerTests.SettleConstantMatchesItsDefiningEquation</c> can
        /// recompute it exactly rather than to a few decimals, and the two constants cannot drift
        /// apart silently. A literal rather than a <c>Math.Log</c> at construction for the reason
        /// <c>spring</c> gives: startup work for a value that can never change, and a literal is
        /// auditable where an iteration is not.
        /// </summary>
        private const float SettleConstant = 4.6051702f;

        private readonly float _positionTolerance;
        private readonly float _orientationTolerance;
        private readonly float _maxLinearSpeed;
        private readonly float _maxAngularSpeed;
        private readonly float _tauSeconds;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        /// <summary>
        /// Shared with every other reconciler so that <c>jerk_mm_s3</c> means one thing across the
        /// axis being compared -- see <see cref="DisplayedJerkEstimator"/> for why that matters.
        /// </summary>
        private readonly DisplayedJerkEstimator _jerk;

        /// <summary>
        /// The entire dynamic state: a position offset and a rotation-vector offset. No velocities --
        /// that absence <i>is</i> the algorithm, and it is what makes the onset velocity step in the
        /// type doc inherent rather than incidental.
        /// </summary>
        private Vector3 _offsetPosition;
        private Vector3 _offsetRotation;

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
        /// budget has no defined time constant and a zero rate cap could never converge, which would
        /// violate the bounded-convergence clause rather than express a tradeoff.
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
        public ExponentialSmoothingReconciler(
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
                    "MaxTimeToConvergenceTicks must be positive: it sets the time constant, so " +
                    "there is no defined response without it.");
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
            _tauSeconds = budgetSeconds / SettleConstant;

            _jerk = new DisplayedJerkEstimator(clock.TicksPerSecond);

            ClearCorrectionState();
        }

        /// <summary>
        /// The time constant in seconds derived from
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>. Exposed for the same reason
        /// <see cref="SpringReconciler.NaturalFrequencyRadiansPerSecond"/> is: it is the single number
        /// that characterises this reconciler's response, and the closed-form onset velocity step
        /// (<c>|o0| / tau</c>) is stated in terms of it -- a test that had to re-derive it would be
        /// asserting against its own copy of the formula rather than against the implementation.
        /// </summary>
        public float TimeConstantSeconds => _tauSeconds;

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
        /// Byte-for-byte the same policy <see cref="SnapReconciler"/> and
        /// <see cref="SpringReconciler"/> apply, and deliberately so -- <c>correction_magnitude_mm</c>
        /// and <c>correction_magnitude_deg</c> are measured from the predictor's disagreement before
        /// any reconciler acts, so they are identical across the axis by construction and serve as a
        /// control. A different acceptance rule here would break that control silently:
        /// <list type="bullet">
        /// <item><b>Ordering.</b> A sample whose capture stamp is not strictly greater than the last
        /// accepted one is stale or a duplicate and is ignored whole -- no re-seed, no metric, no
        /// state change. Equal stamps are rejected because a duplicate must not be counted as a
        /// second correction; correction rate is a reported metric and double-counting corrupts
        /// it.</item>
        /// <item><b>Within tolerance means no correction at all</b>, not a zero-magnitude one, so that
        /// <c>none</c> + this reconciler degenerates to exact pass-through.</item>
        /// <item><b>Re-seed, do not stack.</b> A second qualifying sample replaces the pending
        /// re-seed rather than queueing behind it.</item>
        /// </list>
        ///
        /// Allocation-free.
        /// </summary>
        /// <param name="diagnostics">
        /// Ignored. This reconciler's response is set by the convergence budget, not by predictor
        /// uncertainty, so it behaves identically whether
        /// <see cref="PredictorDiagnostics.HasUncertainty"/> is true or false -- the graceful
        /// degradation <see cref="IReconciler{TState}.Observe"/> asks for, reached by not depending on
        /// the field at all. <see cref="PredictorDiagnostics.None"/> is equally fine.
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
        /// The per-frame call: decays the residual offset one step toward zero and returns
        /// <paramref name="predicted"/> displaced by what remains of it.
        ///
        /// <b>Idempotent in <paramref name="nowTicks"/>.</b> A call at or before the last reconciled
        /// tick returns the cached previous output and does nothing else -- no decay step, no jerk
        /// push, no metric.
        ///
        /// <b>The first advancing call after a pending correction</b> re-seeds the offset from the
        /// live displayed pose and does not decay, because there is no elapsed interval to decay over;
        /// the decay begins on the following frame. On the very first call of a trial there is no
        /// previously displayed pose to stay continuous with, so the offset stays zero and
        /// <paramref name="predicted"/> is returned unmodified -- correcting toward a pose that was
        /// never displayed would invent a discontinuity rather than hide one.
        ///
        /// Emits, on exactly the cadence the rest of the axis uses:
        /// <list type="bullet">
        /// <item><c>jerk_mm_s3</c> on <b>every</b> advancing frame once four displayed positions
        /// exist, via the shared <see cref="DisplayedJerkEstimator"/>. Unconditional, because jerk is
        /// a property of the displayed trajectory rather than of a correction episode; conditioning it
        /// on a correction being in flight would make the pooled percentiles compare populations
        /// rather than algorithms.</item>
        /// <item><c>time_to_convergence_ms</c>, once per convergence episode, on the frame the offset
        /// first falls inside tolerance, measured from correction onset.</item>
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
                // Stop *reporting* the correction, but do not zero the residual -- see the type doc.
                // Truncating it to zero on crossing the tolerance would be a position discontinuity
                // of up to one tolerance, i.e. a velocity step of tolerance/dt, which is unbounded as
                // the frame interval shrinks. This class already carries one analytic velocity step
                // by design; adding a second, avoidable, unbounded one would be indefensible.
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
        /// Returns the reconciler to its as-constructed state: no offset, nothing pending or in
        /// flight, an empty jerk history, and -- the one that is easy to miss -- the accepted-capture
        /// baseline back to <see cref="long.MinValue"/> rather than zero. Sweeps reuse instances
        /// across trials, and a reconciler that remembered the previous trial's highest capture stamp
        /// would silently ignore the whole opening stretch of the next one, which looks like a
        /// suspiciously well-behaved result rather than like a bug. Same reasoning as
        /// <c>SnapReconciler.Reset</c> and <c>SpringReconciler.Reset</c>. Configuration and the metric
        /// sink survive; the sink has its own <c>Reset</c> and is owned by whoever injected it.
        /// </summary>
        public void Reset()
        {
            ClearCorrectionState();
        }

        private void ClearCorrectionState()
        {
            _jerk.Reset();
            _offsetPosition = Vector3.Zero;
            _offsetRotation = Vector3.Zero;
            _correctionPending = false;
            _correctionInFlight = false;
            _correctionOnsetTicks = 0;
            _lastAcceptedCaptureTicks = long.MinValue;
            _lastReconcileTicks = long.MinValue;
            _lastOutput = Pose.Identity;
            _hasLastOutput = false;
        }

        /// <summary>
        /// Sets the offset to whatever keeps the displayed <i>position</i> continuous across this
        /// frame: <c>lastDisplayed - predicted</c>, and the world-frame rotation vector taking
        /// <c>predicted</c> to <c>lastDisplayed</c>.
        ///
        /// <b>There is deliberately no velocity to preserve here.</b> <see cref="SpringReconciler"/>
        /// carries its offset velocity across a re-seed precisely so that a correction arriving
        /// mid-decay bends the trajectory instead of restarting it. A single pole has no such state:
        /// the offset velocity is a pure function of the offset (<c>-o/tau</c>), so re-seeding steps
        /// it by <c>|o_new - o_old| / tau</c>. That is a second, inherent C1 violation, quantified by
        /// test rather than papered over -- adding state to smooth it would make this a second-order
        /// reconciler and would stop answering the question the row exists to ask.
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

            _offsetPosition = _lastOutput.Position - predicted.Position;
            _offsetRotation =
                MotionMath.RelativeRotationVector(predicted.Rotation, _lastOutput.Rotation);

            if (!_correctionInFlight)
            {
                _correctionOnsetTicks = nowTicks;
                _correctionInFlight = true;
            }
        }

        /// <summary>
        /// One exact step of the single-pole law, applied to the position offset and the
        /// rotation-vector offset independently: <c>o(t+dt) = o(t) e^(-dt/tau)</c>.
        ///
        /// Exact rather than an integrator step, and therefore unconditionally stable and frame-rate
        /// independent: a sweep and a headset produce the same trajectory from the same samples,
        /// whatever <c>dt/tau</c> is. Scaling the rotation vector is exact rather than a small-angle
        /// approximation -- shrinking a rotation vector keeps its axis fixed and scales only its
        /// angle, which is the geodesic path to zero rotation.
        /// </summary>
        private void AdvanceOffset(float elapsedSeconds)
        {
            float decay = MathF.Exp(-elapsedSeconds / _tauSeconds);
            float removedFraction = 1f - decay;

            StepSinglePole(
                ref _offsetPosition, removedFraction, _maxLinearSpeed * elapsedSeconds);
            StepSinglePole(
                ref _offsetRotation, removedFraction, _maxAngularSpeed * elapsedSeconds);
        }

        /// <summary>
        /// Removes <paramref name="removedFraction"/> of the offset, but never more than
        /// <paramref name="maxStep"/> of it -- the rate cap, expressed as a displacement cap over this
        /// frame because there is no velocity state to clamp.
        ///
        /// Clamping the <i>delta</i> rather than the offset keeps the direction fixed, so the rotation
        /// vector still shrinks along a fixed axis. The clamp is continuous in the resulting rate: at
        /// the offset magnitude where the cap stops binding, the capped and uncapped steps are equal,
        /// so leaving the capped regime injects no velocity step of its own. A cap can therefore only
        /// stretch convergence, never break continuity -- the same relationship
        /// <see cref="SpringReconciler"/> documents for its velocity clamp.
        /// </summary>
        private static void StepSinglePole(ref Vector3 offset, float removedFraction, float maxStep)
        {
            Vector3 delta = offset * removedFraction;
            offset -= MotionMath.ClampMagnitude(delta, maxStep);
        }

        /// <summary>
        /// True while any residual offset remains, so the decay keeps running after the correction
        /// stops being reported. Convergence ends a reporting episode, not the motion.
        /// </summary>
        private bool HasResidualOffset() =>
            _offsetPosition != Vector3.Zero || _offsetRotation != Vector3.Zero;

        /// <summary>
        /// True when what remains of the offset is inside both configured tolerances. Compared as
        /// magnitudes of the residual itself rather than via <see cref="PoseMath"/> on the composed
        /// poses: the offset <i>is</i> the error, so measuring it directly avoids re-deriving it from
        /// two poses and cannot disagree with what the decay is acting on. Identical to
        /// <see cref="SpringReconciler"/>'s test, so <c>time_to_convergence_ms</c> means the same
        /// thing for both.
        /// </summary>
        private bool OffsetIsWithinTolerance() =>
            _offsetPosition.Length() <= _positionTolerance &&
            _offsetRotation.Length() <= _orientationTolerance;

        /// <summary>
        /// The displayed pose: <paramref name="predicted"/> displaced by the residual offset. Returns
        /// <paramref name="predicted"/> bit-identically when the offset is exactly zero, so that the
        /// degenerate pass-through case reproduces its input rather than differing by quaternion
        /// renormalization noise.
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
