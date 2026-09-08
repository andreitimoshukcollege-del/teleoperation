using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Reconciliation
{
    /// <summary>
    /// Registry key <c>budget-blend</c>: carry the visible disagreement as a <b>residual offset</b>
    /// on top of the predictor's output -- exactly as <see cref="SpringReconciler"/> does -- and drive
    /// that offset to zero along a <b>finite-duration quintic blend</b> rather than an exponential
    /// decay. The offset is <i>identically</i> zero at a deadline computed once, at correction-seed
    /// time, for any correction magnitude.
    ///
    /// <b>Why this exists next to <c>spring</c>.</b> <c>spring</c>'s bound is relative and asymptotic:
    /// its envelope reaches <c>SpringReconciler.SettledFractionOfInitialError</c> (1%) of the initial
    /// error at <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> and technically never reaches
    /// zero, so a large enough correction is still outside
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> when the budget elapses. This
    /// reconciler's bound is <i>absolute</i> and <i>exact</i>: at the deadline the residual is zero,
    /// not 1% of something. Hard deadline versus asymptotic decay is the research question the two
    /// files exist to answer, and everything else about them is deliberately identical -- the same
    /// residual-offset formulation, the same onset seeding, the same four metrics on the same cadence,
    /// the same shared <see cref="DisplayedJerkEstimator"/> -- so that a head-to-head attributes its
    /// result to the convergence law and to nothing else.
    ///
    /// <b>The blend function, and why a ramp will not do.</b> With <c>u</c> the normalised blend time
    /// and <c>o0</c>/<c>v0</c> the offset and offset velocity at seed time, the offset follows
    /// <c>o(u) = (1-u)^3 (c0 + c1 u + c2 u^2)</c>, a quintic Hermite with <c>o(1) = o'(1) = o''(1) = 0</c>
    /// and <c>o(0) = o0</c>, <c>o'(0) = v0 T</c>. A finite blend needs zero velocity at <b>both</b>
    /// ends or it puts a velocity step at onset or at completion; a linear ramp has neither and is
    /// therefore not C1 at either end. A cubic (smoothstep) would give C1; the quintic
    /// (smootherstep, in the <c>v0 = 0</c> case) additionally gives C2, and since jerk is the metric
    /// this axis is judged on, the extra order is the point rather than a flourish: for a step of
    /// <c>o0</c> over <c>T</c> the quintic's peak jerk is <c>60 o0/T^3</c> against a critically damped
    /// spring's <c>2 o0 w^3 = 585 o0/T^3</c> at the <c>w</c> that 1%-in-budget forces.
    ///
    /// <b>The <c>(1-u)^3</c> factorisation is load-bearing.</b> The claim is exactness at the
    /// deadline. Evaluated in its expanded form the quintic is zero at <c>u = 1</c> only to float
    /// rounding; factored, <c>u == 1</c> makes <c>1 - u</c> exactly zero, so the offset and its
    /// derivative are bitwise zero there and stay zero afterwards. That is what lets
    /// <see cref="Compose"/> return <c>predicted</c> bit-identically once a correction has landed --
    /// the same degenerate pass-through property <see cref="SnapReconciler"/> and
    /// <see cref="SpringReconciler"/> document, reached here without a tolerance test.
    ///
    /// <b>Retarget mid-blend.</b> A correction arriving while a blend is in flight restarts the blend
    /// clock with a fresh <i>full</i> deadline and <b>carries the in-flight offset velocity</b> into
    /// <c>c1</c>/<c>c2</c>, so the trajectory bends instead of restarting from rest and the displayed
    /// output stays C1. The initial acceleration term is reset to zero. Two decisions worth stating:
    /// <list type="bullet">
    /// <item><b>Fresh full deadline, not the old one.</b> Keeping the original deadline would make the
    /// guarantee "converged by the first correction's deadline", which sounds stronger and is worse: a
    /// correction landing 1 ms before it would have to be absorbed in 1 ms -- a snap, at unbounded
    /// speed, on exactly the frames a lossy link produces most often. Restarting makes the guarantee
    /// <i>per correction</i>, which is the same class of statement <c>spring</c> makes, and is the only
    /// version that does not degenerate.</item>
    /// <item><b>Velocity carried, acceleration not.</b> Carrying velocity is what preserves C1 and is
    /// precisely what <c>spring</c> does across its own re-seed. Carrying acceleration as well is
    /// available here (the <c>b2</c> term of the derivation) and would preserve C2 across a retarget,
    /// but it would make this reconciler differ from <c>spring</c> in two ways at once -- convergence
    /// law <i>and</i> surviving derivative state -- and a sweep could then not attribute its result to
    /// the convergence law. It is a follow-up experiment, not a default.</item>
    /// </list>
    ///
    /// <b>Rate caps versus the deadline, resolved.</b> Absorbing <c>o0</c> in <c>T</c> needs a peak
    /// offset speed of <c>(15/8) o0 / T</c>, which a cap can forbid. <c>spring</c> resolves the
    /// conflict by clamping the offset velocity every step; that option does not exist for a
    /// <i>scheduled</i> trajectory, because clamping a schedule's derivative desynchronises it from its
    /// own schedule and the offset would then not be zero at the deadline. So the cap is enforced
    /// <b>a priori, by sizing the deadline</b>:
    /// <c>T_eff = max(configured budget, (15/8)|o0_pos|/linear cap, (15/8)|o0_rot|/angular cap)</c>,
    /// one deadline shared by both channels so that <c>time_to_convergence_ms</c> names one instant.
    /// <b>The cap therefore wins over the configured budget</b>, the precedence
    /// <c>Teleop.Eval/Sweep/ExperimentConfig</c> documents and the behaviour <c>spring</c> already has.
    /// What survives unconditionally is not the configured budget but the hard-deadline property
    /// itself: whatever <c>T_eff</c> is, it is finite, known at seed time, and the residual is exactly
    /// zero at it. The sizing uses the offset magnitude only and ignores the carried velocity, so peak
    /// correction speed is at most the cap for an isolated correction and at most
    /// <c>cap + |v_carried|</c> across a retarget; subtracting the carried speed instead would send
    /// <c>T_eff</c> to infinity as the carried speed approached the cap, trading a bounded speed
    /// excursion for unbounded convergence time -- the wrong way round for a contract whose first
    /// clause is bounded convergence.
    ///
    /// <b>Fields of <see cref="ReconcilerConfig"/> it reads:</b>
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> and
    /// <see cref="ReconcilerConfig.ConvergenceOrientationToleranceRadians"/> (to decide whether an
    /// arriving sample is worth correcting, and which frame closes a reporting episode),
    /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> (the floor on the deadline), and both
    /// rate caps (they size the deadline). <b>Field it ignores:</b>
    /// <see cref="ReconcilerConfig.RollbackHistoryCapacity"/> -- for <c>rollback</c>, which
    /// re-simulates buffered inputs. Nothing here rewinds.
    ///
    /// <b>Time is always a parameter.</b> The injected <see cref="ITimeAuthority"/> is read for
    /// <see cref="ITimeAuthority.TicksPerSecond"/> only and never for
    /// <see cref="ITimeAuthority.NowTicks"/>, the same discipline <see cref="SnapReconciler"/>,
    /// <see cref="SpringReconciler"/> and <c>Pipeline/OperatorEndpoint</c> document.
    ///
    /// Deterministic and allocation-free: the only buffer is the shared jerk estimator's history,
    /// sized once in the constructor. Not thread-safe, by contract.
    /// </summary>
    public sealed class TimeBudgetedBlendReconciler : IReconciler<Pose>
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
        /// The peak of <c>|d/du (1 - smootherstep(u))|</c> over <c>[0, 1]</c>, i.e. <c>15/8</c>, which
        /// is attained at <c>u = 0.5</c>. Multiplied by <c>|o0| / T</c> it is the fastest the offset
        /// ever moves during a blend seeded from rest, so it is exactly the number that converts a
        /// rate cap into a minimum blend duration. A derived constant of the blend polynomial, not a
        /// tuning knob: changing it would not retune the response, it would make
        /// <see cref="SizeBlendTicks"/> wrong. Verified against a sampled maximum of the implemented
        /// polynomial by
        /// <c>TimeBudgetedBlendReconcilerTests.PeakSpeedFactorMatchesTheImplementedBlendPolynomial</c>.
        /// </summary>
        public const float PeakSpeedFactor = 15f / 8f;

        /// <summary>
        /// Ceiling on a rate-cap-extended blend, in seconds, purely so that an absurd or corrupt
        /// offset cannot produce a deadline that overflows <see cref="long"/> arithmetic when added to
        /// a tick origin. Ten minutes is far beyond any duration at which "bounded convergence" still
        /// means anything -- it is an overflow guard, not a research knob, and no configuration
        /// reachable from <c>ExperimentConfig</c> comes within three orders of magnitude of it.
        /// </summary>
        private const double MaxBlendDurationSeconds = 600.0;

        private readonly float _positionTolerance;
        private readonly float _orientationTolerance;
        private readonly float _maxLinearSpeed;
        private readonly float _maxAngularSpeed;
        private readonly long _budgetTicks;
        private readonly long _maxBlendDurationTicks;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        /// <summary>
        /// Shared with every other reconciler so that <c>jerk_mm_s3</c> means one thing across the axis
        /// being compared -- see <see cref="DisplayedJerkEstimator"/> for why that matters. Writing a
        /// second cascade here would make a <c>spring</c>-versus-<c>budget-blend</c> sweep partly a
        /// comparison of jerk estimators.
        /// </summary>
        private readonly DisplayedJerkEstimator _jerk;

        // Blend coefficients, position channel: o(u) = (1-u)^3 (c0 + c1 u + c2 u^2), metres.
        private Vector3 _positionC0;
        private Vector3 _positionC1;
        private Vector3 _positionC2;

        // Blend coefficients, orientation channel, as a world-frame rotation vector, radians.
        private Vector3 _rotationC0;
        private Vector3 _rotationC1;
        private Vector3 _rotationC2;

        // The evaluated blend at the last advancing frame: what Compose displaces by, and the state a
        // retarget seeds the next blend from.
        private Vector3 _offsetPosition;
        private Vector3 _offsetLinearVelocity;
        private Vector3 _offsetRotation;
        private Vector3 _offsetAngularVelocity;

        private bool _blendActive;
        private long _blendStartTicks;
        private long _blendDurationTicks;

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
        /// budget leaves the blend with no duration to schedule against, and a zero rate cap would
        /// demand an infinite deadline, which is the "lags indefinitely" failure
        /// <c>Reconciliation/CLAUDE.md</c> calls a bug rather than a tradeoff.
        /// </param>
        /// <param name="metrics">
        /// Correction-cost sink (docs/metrics.md §5). A constructor dependency rather than a per-frame
        /// parameter so that <see cref="Reconcile"/>'s signature stays allocation-free, per
        /// <see cref="IReconciler{TState}"/>.
        /// </param>
        /// <param name="clock">
        /// Read for <see cref="ITimeAuthority.TicksPerSecond"/> only -- never
        /// <see cref="ITimeAuthority.NowTicks"/>.
        /// </param>
        public TimeBudgetedBlendReconciler(
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
                    "MaxTimeToConvergenceTicks must be positive: it is the blend's duration, so " +
                    "there is nothing to schedule the correction against without it.");
            }

            if (config.MaxCorrectionLinearSpeedMetersPerSecond <= 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxCorrectionLinearSpeedMetersPerSecond,
                    "MaxCorrectionLinearSpeedMetersPerSecond must be positive: a zero cap would " +
                    "demand an infinite blend duration.");
            }

            if (config.MaxCorrectionAngularSpeedRadPerSecond <= 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxCorrectionAngularSpeedRadPerSecond,
                    "MaxCorrectionAngularSpeedRadPerSecond must be positive: a zero cap would " +
                    "demand an infinite blend duration.");
            }

            _positionTolerance = config.ConvergencePositionToleranceMeters;
            _orientationTolerance = config.ConvergenceOrientationToleranceRadians;
            _maxLinearSpeed = config.MaxCorrectionLinearSpeedMetersPerSecond;
            _maxAngularSpeed = config.MaxCorrectionAngularSpeedRadPerSecond;
            _budgetTicks = config.MaxTimeToConvergenceTicks;
            _metrics = metrics;
            _ticksPerSecond = clock.TicksPerSecond;

            double ceilingTicks = Math.Min(
                clock.TicksPerSecond * MaxBlendDurationSeconds, long.MaxValue / 4.0);
            _maxBlendDurationTicks = Math.Max(_budgetTicks, (long)ceilingTicks);

            _jerk = new DisplayedJerkEstimator(clock.TicksPerSecond);

            ClearCorrectionState();
        }

        /// <summary>
        /// The effective deadline of the blend currently in flight, in ticks, or zero when none is.
        /// Equals <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> unless a rate cap forced it
        /// longer. Exposed because it is the single number that characterises this reconciler's
        /// response to a given correction, and a test that re-derived it would be asserting against its
        /// own copy of <see cref="SizeBlendTicks"/> rather than against the implementation.
        /// </summary>
        public long BlendDurationTicks => _blendActive ? _blendDurationTicks : 0L;

        /// <summary>
        /// True when no correction is pending and none is being reported -- the displayed pose is the
        /// predictor's output with a residual offset inside tolerance. True on a freshly constructed or
        /// freshly <see cref="Reset"/> instance, as
        /// <see cref="IReconciler{TState}.IsConverged"/> requires.
        ///
        /// Deliberately the same predicate <c>spring</c> uses (not "the blend has finished"), because
        /// <c>IsConverged</c> and <c>time_to_convergence_ms</c> must mean the same thing across the
        /// axis: the frame the visible error first falls inside tolerance. The blend keeps running
        /// past that frame, all the way to exactly zero -- see <see cref="Reconcile"/>.
        /// </summary>
        public bool IsConverged => !_correctionPending && !_correctionInFlight;

        /// <summary>
        /// Truth arrived. Measures the disagreement against what was displayed for that same instant
        /// and, if it exceeds tolerance, flags that a blend must be seeded on the next
        /// <see cref="Reconcile"/>. Changes nothing visible here, per
        /// <see cref="IReconciler{TState}.Observe"/>.
        ///
        /// Byte-for-byte the ordering, tolerance and metric behaviour of <c>snap</c> and <c>spring</c>,
        /// on purpose:
        /// <list type="bullet">
        /// <item><b>Ordering.</b> A sample whose capture stamp is not strictly greater than the last
        /// accepted one is stale or a duplicate and is ignored whole -- no seed, no metric, no state
        /// change. Equal stamps are rejected for the reason
        /// <see cref="IReconciler{TState}.Observe"/> states: a duplicate must not be counted as a
        /// second correction, because correction rate is a reported metric and double-counting
        /// corrupts it.</item>
        /// <item><b>Within tolerance means no correction at all</b>, not a zero-magnitude one. With a
        /// passthrough predictor whose output already equals truth, the pipeline must reduce to exact
        /// pass-through, emitting no correction-cost samples and never leaving
        /// <see cref="IsConverged"/>.</item>
        /// <item><b>Re-seed, do not stack.</b> A second qualifying sample replaces the pending seed
        /// rather than queueing behind it, and the actual offset is computed from the live displayed
        /// pose at the next <see cref="Reconcile"/>, so several samples arriving between two frames
        /// cost one seed against the newest truth.</item>
        /// </list>
        ///
        /// Allocation-free.
        /// </summary>
        /// <param name="diagnostics">
        /// Ignored. The blend's duration comes from the convergence budget and the rate caps, not from
        /// predictor uncertainty, so this reconciler behaves identically whether
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
        /// The per-frame call: advances the scheduled blend to <paramref name="nowTicks"/> and returns
        /// <paramref name="predicted"/> displaced by what remains of the offset.
        ///
        /// <b>Idempotent in <paramref name="nowTicks"/>.</b> A call at or before the last reconciled
        /// tick returns the cached previous output and does nothing else -- no blend step, no jerk
        /// push, no metric. <see cref="IReconciler{TState}.Reconcile"/> requires that two calls with
        /// the same <c>nowTicks</c> return the same state and not advance the correction twice, and the
        /// same guard covers a frame delivered out of order.
        ///
        /// <b>The first advancing call after a pending correction</b> seeds a blend from the live
        /// displayed pose (see the type doc) and does not step it, because no blend time has elapsed
        /// yet; the blend starts moving on the following frame. On the very first call of a trial there
        /// is no previously displayed pose to stay continuous with, so no blend is seeded and
        /// <paramref name="predicted"/> is returned unmodified -- correcting toward a pose that was
        /// never displayed would be inventing a discontinuity rather than hiding one.
        ///
        /// <b>At and after the deadline</b> the offset is exactly <see cref="Vector3.Zero"/> and the
        /// blend is retired, so the output is <paramref name="predicted"/> bit-identically. A frame
        /// arriving long after the deadline -- the several-hundred-millisecond gap real traces contain
        /// -- lands in that branch and is therefore trivially safe; there is no exponential to
        /// evaluate over a huge interval.
        ///
        /// Emits:
        /// <list type="bullet">
        /// <item><c>jerk_mm_s3</c> on <b>every</b> advancing frame once four displayed positions exist
        /// -- the same unconditional cadence <see cref="SnapReconciler"/> and
        /// <see cref="SpringReconciler"/> use, which is what makes the three distributions populations
        /// of the same thing (displayed-trajectory jerk) rather than one sample per correction against
        /// a hundred.</item>
        /// <item><c>time_to_convergence_ms</c>, once per convergence episode, on the frame the offset
        /// first falls inside tolerance, measured from correction onset -- the docs/metrics.md §5
        /// definition, unchanged. Note this is generally <i>earlier</i> than the deadline: the blend
        /// keeps running to exactly zero after the episode is reported.</item>
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

            if (_correctionPending)
            {
                SeedBlendFromDisplayedPose(predicted, nowTicks);
                _correctionPending = false;
            }
            else if (_blendActive)
            {
                AdvanceBlend(nowTicks);
            }

            if (_correctionInFlight && OffsetIsWithinTolerance())
            {
                // Stop *reporting* the correction; do not retire the blend. The residual is not yet
                // zero, and truncating it here would be a position discontinuity of up to one
                // tolerance -- across a frame, a velocity step of tolerance/dt, invisible in absolute
                // terms but unbounded as the frame interval shrinks, which would falsify the C1 claim
                // at exactly the frame rates a headset runs at. Same reasoning as
                // SpringReconciler.Reconcile; the difference is that here the residual reaches exactly
                // zero at a known deadline instead of decaying geometrically toward it.
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

            // Unconditional, matching SnapReconciler and SpringReconciler: jerk is a property of the
            // displayed trajectory (docs/metrics.md §5), not of a correction episode. Conditioning it
            // on a correction being in flight is what made two reconcilers' sample sets incomparable
            // -- one sample per correction against roughly one per frame of one.
            if (hasJerk)
            {
                _metrics.Record(JerkMetric, jerkMillimetresPerSecondCubed, nowTicks);
            }

            return output;
        }

        /// <summary>
        /// Returns the reconciler to its as-constructed state: no blend, no coefficients, no offset or
        /// offset velocity, nothing pending or in flight, an empty jerk history, and -- the one that is
        /// easy to miss -- the accepted-capture baseline back to <see cref="long.MinValue"/> rather
        /// than zero. Sweeps reuse instances across trials, and a reconciler that remembered the
        /// previous trial's highest capture stamp would silently ignore the whole opening stretch of
        /// the next one, which looks like a suspiciously well-behaved result rather than like a bug.
        /// Same reasoning as <c>SnapReconciler.Reset</c>, <c>SpringReconciler.Reset</c> and
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

            _positionC0 = Vector3.Zero;
            _positionC1 = Vector3.Zero;
            _positionC2 = Vector3.Zero;
            _rotationC0 = Vector3.Zero;
            _rotationC1 = Vector3.Zero;
            _rotationC2 = Vector3.Zero;

            _offsetPosition = Vector3.Zero;
            _offsetLinearVelocity = Vector3.Zero;
            _offsetRotation = Vector3.Zero;
            _offsetAngularVelocity = Vector3.Zero;

            _blendActive = false;
            _blendStartTicks = 0;
            _blendDurationTicks = 0;

            _correctionPending = false;
            _correctionInFlight = false;
            _correctionOnsetTicks = 0;
            _lastAcceptedCaptureTicks = long.MinValue;
            _lastReconcileTicks = long.MinValue;
            _lastOutput = Pose.Identity;
            _hasLastOutput = false;
        }

        /// <summary>
        /// Starts a blend from whatever keeps the displayed pose continuous across this frame:
        /// <c>lastDisplayed - predicted</c> in position, and the world-frame rotation vector taking
        /// <c>predicted</c> to <c>lastDisplayed</c> in orientation -- the identical seeding
        /// <c>spring</c> uses, so the two differ only in what happens next.
        ///
        /// The offset <i>velocities</i> are read, not reset: they are the boundary conditions
        /// <see cref="SetBlendCoefficients"/> feeds into <c>c1</c>/<c>c2</c>, which is what keeps the
        /// displayed trajectory C1 across a correction that arrives while an earlier one is still in
        /// flight.
        ///
        /// Onset ticks are recorded only when a correction was not already being reported, so
        /// <c>time_to_convergence_ms</c> measures one convergence episode from its true start rather
        /// than restarting the clock on every retarget -- again matching <c>spring</c>.
        /// </summary>
        private void SeedBlendFromDisplayedPose(in Pose predicted, long nowTicks)
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

            _blendDurationTicks = SizeBlendTicks(_offsetPosition, _offsetRotation);
            _blendStartTicks = nowTicks;
            _blendActive = true;

            float durationSeconds = _blendDurationTicks / (float)_ticksPerSecond;
            SetBlendCoefficients(
                _offsetPosition, _offsetLinearVelocity, durationSeconds,
                out _positionC0, out _positionC1, out _positionC2);
            SetBlendCoefficients(
                _offsetRotation, _offsetAngularVelocity, durationSeconds,
                out _rotationC0, out _rotationC1, out _rotationC2);

            if (!_correctionInFlight)
            {
                _correctionOnsetTicks = nowTicks;
                _correctionInFlight = true;
            }
        }

        /// <summary>
        /// The blend's effective duration in ticks: the configured budget, lengthened just enough that
        /// neither rate cap is exceeded by a blend seeded from rest. See the type doc for why the cap
        /// is enforced here, once, rather than by clamping the offset velocity each frame the way
        /// <c>spring</c> does.
        ///
        /// Both channels share one duration -- the larger of the two requirements -- so that a
        /// correction is a single scheduled event with a single deadline and
        /// <c>time_to_convergence_ms</c> names one instant rather than the later of two.
        ///
        /// Ceiling-rounded to whole ticks, so the deadline is representable exactly on the tick
        /// timebase and the <c>u == 1</c> comparison that produces a bitwise-zero residual is
        /// reachable. Clamped to <see cref="MaxBlendDurationSeconds"/> as an overflow guard.
        /// </summary>
        private long SizeBlendTicks(in Vector3 offsetPosition, in Vector3 offsetRotation)
        {
            long ticks = _budgetTicks;

            long linearTicks = RequiredTicks(offsetPosition.Length(), _maxLinearSpeed);
            if (linearTicks > ticks)
            {
                ticks = linearTicks;
            }

            long angularTicks = RequiredTicks(offsetRotation.Length(), _maxAngularSpeed);
            if (angularTicks > ticks)
            {
                ticks = angularTicks;
            }

            return ticks > _maxBlendDurationTicks ? _maxBlendDurationTicks : ticks;
        }

        /// <summary>
        /// The whole ticks a blend of the given magnitude needs so its peak rate stays within
        /// <paramref name="maxRate"/>: <c>(15/8) * magnitude / maxRate</c>, rounded up. Returns zero
        /// for a non-finite magnitude so the configured budget is used rather than a NaN deadline;
        /// a NaN offset is already a bug upstream, and this only ensures it does not become an
        /// unbounded one here.
        /// </summary>
        private long RequiredTicks(float magnitude, float maxRate)
        {
            if (!(magnitude > 0f))
            {
                return 0L;
            }

            double seconds = PeakSpeedFactor * (double)magnitude / maxRate;
            double ticks = Math.Ceiling(seconds * _ticksPerSecond);

            if (!(ticks > 0.0))
            {
                return 0L;
            }

            return ticks >= _maxBlendDurationTicks ? _maxBlendDurationTicks : (long)ticks;
        }

        /// <summary>
        /// The quintic Hermite coefficients of <c>o(u) = (1-u)^3 (c0 + c1 u + c2 u^2)</c> for the
        /// boundary conditions <c>o(0) = offset</c>, <c>o'(0) = velocity * T</c>,
        /// <c>o''(0) = 0</c> and <c>o(1) = o'(1) = o''(1) = 0</c>:
        /// <c>c0 = offset</c>, <c>c1 = 3*offset + velocity*T</c>, <c>c2 = 6*offset + 3*velocity*T</c>.
        ///
        /// The triple root at <c>u = 1</c> is written into the <i>form</i> rather than solved for, so
        /// the terminal conditions hold in floating point and not merely in exact arithmetic. With
        /// <c>velocity = 0</c> this is exactly <c>offset * (1 - smootherstep(u))</c>, since
        /// <c>(1-u)^3 (1 + 3u + 6u^2) = 1 - 10u^3 + 15u^4 - 6u^5</c>; the isolated correction is
        /// therefore the same function as the retarget, not a separate case.
        ///
        /// Applied identically to the position offset and to the rotation-vector offset. Scaling a
        /// rotation vector is exact rather than a small-angle approximation: shrinking it keeps its
        /// axis fixed and scales only its angle, which is the geodesic path to zero rotation.
        /// </summary>
        private static void SetBlendCoefficients(
            in Vector3 offset,
            in Vector3 velocity,
            float durationSeconds,
            out Vector3 c0,
            out Vector3 c1,
            out Vector3 c2)
        {
            Vector3 scaledVelocity = velocity * durationSeconds;

            c0 = offset;
            c1 = 3f * offset + scaledVelocity;
            c2 = 6f * offset + 3f * scaledVelocity;
        }

        /// <summary>
        /// Evaluates the blend at <paramref name="nowTicks"/>, storing the offset and its velocity for
        /// both channels. At or past the deadline the blend is retired and everything is set to exactly
        /// <see cref="Vector3.Zero"/> -- the exactness claim, and what makes
        /// <see cref="Compose"/> reduce to bit-identical pass-through afterwards.
        ///
        /// Frame-rate independent by construction: the offset is a function of absolute blend time,
        /// not an integrated state, so a sweep and a headset produce the same trajectory from the same
        /// samples, and a gap of several hundred milliseconds costs nothing more than a larger
        /// <c>u</c>. That is the structural difference from <c>spring</c>, whose exact exponential step
        /// is frame-rate independent only because it was written as a closed form rather than an
        /// integrator.
        /// </summary>
        private void AdvanceBlend(long nowTicks)
        {
            long elapsedTicks = nowTicks - _blendStartTicks;

            if (elapsedTicks >= _blendDurationTicks)
            {
                RetireBlend();
                return;
            }

            float u = (float)(elapsedTicks / (double)_blendDurationTicks);
            float durationSeconds = _blendDurationTicks / (float)_ticksPerSecond;

            EvaluateBlend(
                _positionC0, _positionC1, _positionC2, u, durationSeconds,
                out _offsetPosition, out _offsetLinearVelocity);
            EvaluateBlend(
                _rotationC0, _rotationC1, _rotationC2, u, durationSeconds,
                out _offsetRotation, out _offsetAngularVelocity);
        }

        private void RetireBlend()
        {
            _blendActive = false;
            _offsetPosition = Vector3.Zero;
            _offsetLinearVelocity = Vector3.Zero;
            _offsetRotation = Vector3.Zero;
            _offsetAngularVelocity = Vector3.Zero;
        }

        /// <summary>
        /// <c>o(u) = (1-u)^3 q(u)</c> and <c>o'(u)/T = (-3(1-u)^2 q(u) + (1-u)^3 q'(u)) / T</c> with
        /// <c>q(u) = c0 + c1 u + c2 u^2</c>. Both are exactly zero when <paramref name="u"/> is
        /// exactly one, which is why the caller retires the blend on <c>u &gt;= 1</c> instead of
        /// letting it evaluate there: past one the <c>(1-u)^3</c> factor changes sign and the offset
        /// would grow away from truth.
        /// </summary>
        private static void EvaluateBlend(
            in Vector3 c0,
            in Vector3 c1,
            in Vector3 c2,
            float u,
            float durationSeconds,
            out Vector3 offset,
            out Vector3 velocity)
        {
            float oneMinusU = 1f - u;
            float oneMinusUSquared = oneMinusU * oneMinusU;
            float oneMinusUCubed = oneMinusUSquared * oneMinusU;

            Vector3 q = c0 + c1 * u + c2 * (u * u);
            Vector3 qPrime = c1 + 2f * c2 * u;

            offset = oneMinusUCubed * q;
            velocity = (oneMinusUCubed * qPrime - 3f * oneMinusUSquared * q) * (1f / durationSeconds);
        }

        /// <summary>
        /// True when what remains of the offset is inside both configured tolerances. Compared as
        /// magnitudes of the residual itself rather than via <see cref="PoseMath"/> on the composed
        /// poses: the offset <i>is</i> the error, so measuring it directly avoids re-deriving it from
        /// two poses and cannot disagree with what the blend is acting on. Identical to
        /// <c>SpringReconciler.OffsetIsWithinTolerance</c>, which is the point -- the frame that closes
        /// a reporting episode must be decided the same way on both.
        /// </summary>
        private bool OffsetIsWithinTolerance() =>
            _offsetPosition.Length() <= _positionTolerance &&
            _offsetRotation.Length() <= _orientationTolerance;

        /// <summary>
        /// The displayed pose: <paramref name="predicted"/> displaced by the residual offset. Returns
        /// <paramref name="predicted"/> bit-identically when the offset is exactly zero, so that the
        /// degenerate pass-through case reproduces its input rather than differing by quaternion
        /// renormalization noise -- the same reasoning <see cref="MotionMath.IntegrateWorld"/> gives
        /// for its own zero-rotation short circuit. Unlike <c>spring</c>, which reaches exact zero only
        /// asymptotically, this branch is reached at a known finite tick.
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
