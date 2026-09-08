using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Reconciliation
{
    /// <summary>
    /// Registry key <c>velocity-match</c>: remove positional error while <b>preserving apparent
    /// motion</b>. Where <see cref="SnapReconciler"/> and <see cref="SpringReconciler"/> both put
    /// position error first and let the resulting velocity fall where it may, this one puts the
    /// velocity signal first and lets a bounded position error linger to protect it.
    ///
    /// <b>The premise is perceptual, not control-theoretic.</b> An operator notices a change in an
    /// object's <i>velocity</i> far more readily than a standing positional offset. So it can be a
    /// better trade to accept a residual offset for longer than to disturb the perceived motion while
    /// removing it. This class is that trade, made explicit and bounded.
    ///
    /// <b>What "apparent motion" is, and where it comes from.</b> <see cref="IReconciler{TState}"/>
    /// hands out poses, never velocities, so the source has to be chosen and defended rather than
    /// assumed. The displayed pose is <c>predicted + offset</c>; the operator's genuine apparent
    /// motion is therefore the motion of the <c>predicted</c> term, and everything this reconciler
    /// adds on top of it is a perturbation of that motion. So the estimate is a finite difference of
    /// <see cref="Reconcile"/>'s own <c>predicted</c> argument, exponentially smoothed. Three
    /// alternatives were weighed and rejected:
    /// <list type="bullet">
    /// <item><b>Finite-differencing the authoritative stream.</b> Rejected on three counts. It
    /// describes the <i>robot's</i> motion one transport delay ago, which is not the signal being
    /// looked at. Its samples arrive at a cadence set by the network profile, so the estimate's noise
    /// -- and hence this reconciler's behaviour -- would become a function of the transport axis,
    /// destroying attribution in a reconciler-only sweep (<c>Reconciliation/CLAUDE.md</c>'s
    /// experiment-design note). And <c>Observe</c> may not be called for hundreds of milliseconds,
    /// while the warp below is needed every frame.</item>
    /// <item><b>Finite-differencing the displayed output.</b> Rejected as circular: the displayed
    /// motion already contains the injected correction, so modulating the correction by it is
    /// positive feedback -- a moving correction makes the display look fast, which would license a
    /// faster correction.</item>
    /// <item><b>Carrying a velocity through <see cref="PredictorDiagnostics"/>.</b> Rejected because
    /// it would need a contract change, and because most predictors supply nothing there.</item>
    /// </list>
    /// The chosen source buys robustness for free, which is the point worth noticing:
    /// <see cref="Observe"/> contributes <b>nothing</b> to the velocity estimate except the
    /// pending-seed flag, and stale or duplicate samples are rejected before that flag is set. A
    /// duplicate, an out-of-order sample, or a gap of several hundred milliseconds in the
    /// authoritative stream is therefore <i>structurally incapable</i> of corrupting the estimate,
    /// rather than merely being handled by it.
    ///
    /// <b>The one place the estimate can be fooled, and the fix.</b> On the frame a correction is
    /// seeded, the predictor's output has just absorbed truth and so contains a step of its own.
    /// <c>predicted_k - predicted_(k-1)</c> on that frame is (operator motion + predictor jump), and
    /// the two are not separable from poses alone. They <i>are</i> separable statistically -- the jump
    /// is a one-frame impulse and operator motion is persistent -- so the estimate is <b>held</b> for
    /// that one frame. Without the hold, the estimator would report a large apparent speed at exactly
    /// the instant a correction begins, licensing the most aggressive possible correction at exactly
    /// the moment the premise says to be gentlest.
    ///
    /// <b>The mechanism: <see cref="SpringReconciler"/>'s exact dynamics on a warped time axis.</b>
    /// The residual offset follows the same critically damped closed form, with the same
    /// <c>omega</c> derived the same way from <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>
    /// -- but integrated against <c>dtau = w * dt</c> rather than <c>dt</c>, where
    /// <c>w</c> in <c>[<see cref="MinimumTimeWarp"/>, 1]</c> is chosen each frame so that the speed
    /// this reconciler injects stays under a fixed fraction of the operator's apparent speed:
    /// <code>
    /// w = clamp(VelocityMaskingFraction * apparentSpeed / demandedSpeed, MinimumTimeWarp, 1)
    /// </code>
    /// with <c>demandedSpeed</c> the offset speed an unwarped (<c>w = 1</c>) step would have produced
    /// this frame -- i.e. exactly what <c>spring</c> would have injected. Two consequences follow
    /// directly and are the substance of the candidate:
    /// <list type="number">
    /// <item><b>The apparent-motion guarantee.</b>
    /// <c>|injected velocity| &lt;= max(MinimumTimeWarp * |spring's injected velocity|,
    /// VelocityMaskingFraction * apparent speed)</c>. <c>spring</c> offers no bound of this shape at
    /// all: its injected speed is whatever the budget demands, regardless of what the operator is
    /// doing.</item>
    /// <item><b>It is never more aggressive than <c>spring</c> at the same budget</b>, because
    /// <c>w &lt;= 1</c>. It matches <c>spring</c> exactly when the operator is moving fast enough to
    /// mask a full-rate correction, and is up to <c>1 / MinimumTimeWarp</c> gentler when they are
    /// still.</item>
    /// </list>
    ///
    /// <b>Bounded convergence, and its stated bound.</b> The critically damped envelope
    /// <c>|o(tau)| = |o0| (1 + omega*tau) e^(-omega*tau)</c> is strictly decreasing in <c>tau</c>, and
    /// <c>tau(t) &gt;= MinimumTimeWarp * t</c> because <c>w</c> is clamped below. Therefore
    /// <c>|o(t)| &lt;= |o0| (1 + omega*w_min*t) e^(-omega*w_min*t)</c> unconditionally, whatever the
    /// operator does. <b>Stated bound: the offset falls to
    /// <see cref="SettledFractionOfInitialError"/> of the initial error within
    /// <c>MaxTimeToConvergenceTicks / MinimumTimeWarp</c></b> -- four times the configured budget in
    /// the worst case, and exactly the budget when the apparent speed licenses <c>w = 1</c>. This is
    /// the "stated bound" branch of <see cref="IReconciler{TState}"/>'s convergence clause, and it is
    /// a <i>time</i> bound, not a permanent residual: the offset still reaches zero, it is merely
    /// allowed to take longer. A reconciler that could lag indefinitely would be a bug, and this one
    /// cannot, because <see cref="MinimumTimeWarp"/> is strictly positive by construction.
    ///
    /// Two caveats, inherited verbatim from <see cref="SpringReconciler"/> because the dynamics are
    /// the same:
    /// <list type="bullet">
    /// <item>The bound is <i>relative</i>. One percent of a very large correction can still exceed
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/>, so such a correction needs
    /// slightly longer than the stated window to land inside the absolute tolerance.</item>
    /// <item>The rate caps take precedence. If
    /// <see cref="ReconcilerConfig.MaxCorrectionLinearSpeedMetersPerSecond"/> or
    /// <see cref="ReconcilerConfig.MaxCorrectionAngularSpeedRadPerSecond"/> binds, convergence
    /// stretches further still.</item>
    /// </list>
    ///
    /// <b>C1 continuity.</b> Three separate things could have broken it and each is handled
    /// structurally rather than by tuning:
    /// <list type="bullet">
    /// <item>At correction onset the offset is seeded to <c>lastDisplayed - predicted</c>, so the
    /// displayed position does not move on the onset frame, and the offset velocity there is zero, so
    /// there is no velocity step either. Same construction as <c>spring</c>.</item>
    /// <item>The offset velocity is carried across a re-seed rather than reset, so a correction
    /// arriving mid-flight bends the trajectory instead of restarting it.</item>
    /// <item>The warp <c>w</c> multiplies the visible velocity, so a <i>jump</i> in <c>w</c> would be
    /// a velocity discontinuity. It cannot jump: <c>demandedSpeed</c> is a continuous function of the
    /// offset velocity state, and the apparent-speed estimate is exponentially smoothed with
    /// <c>alpha = 1 - e^(-dt/tau)</c>, whose per-frame change is <c>O(dt)</c> for any bounded input.
    /// So <c>w</c>'s per-frame change is <c>O(dt)</c> and the visible velocity step is too -- which is
    /// what the refinement test measures. The smoothing is load-bearing for continuity, not
    /// cosmetic.</item>
    /// </list>
    ///
    /// <b>Fields of <see cref="ReconcilerConfig"/> it reads:</b> all but one.
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> and
    /// <see cref="ReconcilerConfig.ConvergenceOrientationToleranceRadians"/> decide whether an
    /// arriving sample is worth correcting and whether a correction has landed;
    /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> sets <c>omega</c> (identically to
    /// <c>spring</c>) <i>and</i> the apparent-speed smoother's time constant; both rate caps bound the
    /// offset velocity exactly as in <c>spring</c>. <b>Field it ignores:</b>
    /// <see cref="ReconcilerConfig.RollbackHistoryCapacity"/> -- for <c>rollback</c>. Nothing here
    /// rewinds. No new configuration field was added: <see cref="MinimumTimeWarp"/> and
    /// <see cref="VelocityMaskingFraction"/> are definitions with the same status as
    /// <c>spring</c>'s settle fraction, not knobs.
    ///
    /// <b>Metrics.</b> The same four names, the same definitions and the same cadence as
    /// <c>snap</c> and <c>spring</c> -- <c>correction_magnitude_mm</c> and
    /// <c>correction_magnitude_deg</c> from <see cref="Observe"/> stamped at the sample's capture
    /// time, <c>jerk_mm_s3</c> on <b>every</b> advancing frame via the shared
    /// <see cref="DisplayedJerkEstimator"/>, and <c>time_to_convergence_ms</c> once per convergence
    /// episode. Unequal cadence would make a head-to-head compare populations rather than algorithms,
    /// so none of it is conditioned on this reconciler's own state.
    ///
    /// <b>Time is always a parameter.</b> The injected <see cref="ITimeAuthority"/> is read for
    /// <see cref="ITimeAuthority.TicksPerSecond"/> only, never for
    /// <see cref="ITimeAuthority.NowTicks"/>. The velocity estimator in particular is driven purely by
    /// <see cref="Reconcile"/>'s <c>nowTicks</c>, which is what keeps it replay-deterministic.
    ///
    /// Deterministic and allocation-free: the only buffer is the shared jerk estimator's history,
    /// sized once in the constructor. Not thread-safe, by contract.
    /// </summary>
    public sealed class VelocityMatchedReconciler : IReconciler<Pose>
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
        /// The fraction of the initial error the critically damped envelope must have decayed to by
        /// the end of the stated window. Identical to <c>spring</c>'s value on purpose: the two
        /// reconcilers must mean the same thing by "converged" or a head-to-head on
        /// <c>time_to_convergence_ms</c> would be comparing definitions.
        /// </summary>
        private const float SettledFractionOfInitialError = 0.01f;

        /// <summary>
        /// The <c>x</c> solving <c>(1 + x) e^(-x) = </c>
        /// <see cref="SettledFractionOfInitialError"/>, so that <c>omega = x / budgetSeconds</c> puts
        /// the envelope exactly at that fraction when the budget elapses at <c>w = 1</c>. The same
        /// precomputed Lambert-W root <c>spring</c> uses, for the same reason (no closed form; a
        /// literal is auditable where a per-instance numeric solve is not), and guarded by the same
        /// style of test -- <c>VelocityMatchedReconcilerTests.CriticalSettleConstantMatchesItsDefiningEquation</c>
        /// recomputes <c>(1 + x) e^(-x)</c> so the two constants cannot drift apart silently.
        /// </summary>
        private const float CriticalSettleConstant = 6.6383521f;

        /// <summary>
        /// The floor on the time warp, and therefore the whole of the bounded-convergence guarantee:
        /// the correction always advances on a time axis running at least this fraction of real time,
        /// so the stated window is <c>MaxTimeToConvergenceTicks / MinimumTimeWarp</c> -- four times the
        /// configured budget at this value.
        ///
        /// A <b>definition, not a tuning knob</b>, in the same sense as
        /// <see cref="SettledFractionOfInitialError"/>: it is the exchange rate this reconciler offers
        /// between apparent-motion fidelity and convergence time, so changing it changes what every
        /// recorded <c>velocity-match</c> result means. Public because it is the number a reader needs
        /// in order to state the bound, and because a test that re-derived it would be asserting
        /// against its own copy rather than against the implementation.
        ///
        /// Four was chosen rather than something larger because the residual offset a slower warp
        /// leaves behind is itself a cost this axis is supposed to be honest about -- a reconciler
        /// that lingers indefinitely would post an excellent <c>jerk_mm_s3</c> for reasons that are
        /// not merit.
        /// </summary>
        public const float MinimumTimeWarp = 0.25f;

        /// <summary>
        /// The fraction of the operator's apparent speed that this reconciler is willing to inject as
        /// correction velocity before it slows the correction down. The perceptual content of the
        /// whole candidate lives in this number.
        ///
        /// Also a <b>definition, not a tuning knob</b>. Psychophysical velocity-discrimination Weber
        /// fractions run about 0.05-0.10 for deliberate discrimination of an isolated moving stimulus,
        /// and are larger for an incidental perturbation superimposed on self-generated motion; 0.25
        /// is the permissive end of that range. It is a compile-time constant rather than a
        /// <see cref="ReconcilerConfig"/> field because that struct is shared across the axis and
        /// gaining a field for one implementation would change every recorded config hash.
        /// </summary>
        public const float VelocityMaskingFraction = 0.25f;

        /// <summary>
        /// Speeds at or below this are treated as exactly zero when forming the warp ratio, purely to
        /// keep <c>0/0</c> out of the arithmetic. Not a dead band on operator motion and not a
        /// research knob: when the correction is demanding no speed there is nothing to mask, so the
        /// warp is 1 and the residual decays at full rate. Chosen well below any speed a real
        /// operator, codec or trace produces, matching <see cref="MotionMath.RotationEpsilon"/>'s
        /// rationale.
        /// </summary>
        private const float SpeedEpsilon = 1e-9f;

        private readonly float _positionTolerance;
        private readonly float _orientationTolerance;
        private readonly float _maxLinearSpeed;
        private readonly float _maxAngularSpeed;
        private readonly float _omega;

        /// <summary>
        /// Time constant of the apparent-speed smoother, seconds. Set to the convergence budget
        /// rather than to an independent constant, on the reasoning that a velocity fluctuation
        /// shorter than the correction itself cannot mask that correction -- so the estimate should
        /// average over exactly the correction's own timescale. This also keeps the reconciler
        /// single-timescale: there is one duration in its configuration and everything is derived
        /// from it.
        /// </summary>
        private readonly float _speedSmoothingSeconds;

        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        /// <summary>
        /// Shared with every other reconciler so that <c>jerk_mm_s3</c> means one thing across the
        /// axis being compared -- see <see cref="DisplayedJerkEstimator"/> for why that matters. A
        /// second copy of the cascade would make a head-to-head partly a comparison of jerk
        /// estimators.
        /// </summary>
        private readonly DisplayedJerkEstimator _jerk;

        private Vector3 _offsetPosition;
        private Vector3 _offsetRotation;

        /// <summary>
        /// Offset velocities in <b>warped</b> time, i.e. <c>d(offset)/d(tau)</c>. The visible
        /// perturbation of apparent motion is this multiplied by the frame's warp factor; keeping the
        /// state in tau is what lets the closed form stay exact.
        /// </summary>
        private Vector3 _offsetLinearVelocity;

        private Vector3 _offsetAngularVelocity;

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

        private Vector3 _lastPredictedPosition;
        private Quaternion _lastPredictedRotation;
        private bool _hasLastPredicted;

        private float _apparentLinearSpeed;
        private float _apparentAngularSpeed;

        /// <param name="config">
        /// Parameters; see the type doc for which fields are read. Like <c>spring</c> and unlike
        /// <c>snap</c>, this reconciler <b>requires</b> a positive
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> and positive rate caps: a zero
        /// budget has no defined response and no defined smoothing timescale, and a zero rate cap
        /// could never converge, which would violate the bounded-convergence clause rather than
        /// express a tradeoff.
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
        public VelocityMatchedReconciler(
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
                    "MaxTimeToConvergenceTicks must be positive: it sets both the natural frequency " +
                    "and the apparent-speed smoothing timescale, so there is no defined response " +
                    "without it.");
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
            _omega = CriticalSettleConstant / budgetSeconds;
            _speedSmoothingSeconds = budgetSeconds;

            _jerk = new DisplayedJerkEstimator(clock.TicksPerSecond);

            ClearCorrectionState();
        }

        /// <summary>
        /// The natural frequency in radians/second at <c>w = 1</c>, derived from
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> by exactly <c>spring</c>'s rule.
        /// Exposed for the same reason <c>spring</c> exposes it: a test that re-derived it would be
        /// asserting against its own copy of the formula.
        /// </summary>
        public float NaturalFrequencyRadiansPerSecond => _omega;

        /// <summary>
        /// The smoothed apparent linear speed, metres/second -- the estimate the warp is computed
        /// from. Exposed so the velocity-estimation tests can assert on the estimator itself
        /// (including that duplicate, out-of-order and gapped observations leave it untouched) rather
        /// than inferring it from a trajectory, which would conflate the estimator with the dynamics.
        /// </summary>
        public float ApparentLinearSpeedMetersPerSecond => _apparentLinearSpeed;

        /// <summary>The smoothed apparent angular speed, radians/second. Same rationale.</summary>
        public float ApparentAngularSpeedRadiansPerSecond => _apparentAngularSpeed;

        /// <summary>
        /// True when no correction is pending and none is in flight. True on a freshly constructed or
        /// freshly <see cref="Reset"/> instance, as <see cref="IReconciler{TState}.IsConverged"/>
        /// requires. Stays false for the whole (possibly warp-stretched) decay, which is what makes it
        /// meaningful to assert the stated bound against.
        /// </summary>
        public bool IsConverged => !_correctionPending && !_correctionInFlight;

        /// <summary>
        /// Truth arrived. Measures the disagreement against what was displayed for that same instant
        /// and, if it exceeds tolerance, flags that the offset must be re-seeded on the next
        /// <see cref="Reconcile"/>. Changes nothing visible here, per
        /// <see cref="IReconciler{TState}.Observe"/>.
        ///
        /// Byte-for-byte the same admission rule and the same metric emission as <c>snap</c> and
        /// <c>spring</c>, deliberately: a sample whose capture stamp is not strictly greater than the
        /// last accepted one is stale or a duplicate and is ignored whole, and a disagreement inside
        /// tolerance is no correction at all rather than a zero-magnitude one. Any divergence here
        /// would show up in a head-to-head as a difference in <c>correction_magnitude_mm</c>, which is
        /// a property of the predictor rather than of any reconciler.
        ///
        /// <b>This method does not touch the apparent-motion estimate</b>, which is the reason
        /// duplicate, reordered and gapped authoritative samples cannot corrupt it. All it can do is
        /// cause the estimate to be held for one frame.
        ///
        /// Allocation-free.
        /// </summary>
        /// <param name="diagnostics">
        /// Ignored. The warp is set by observed apparent motion and the convergence budget, not by
        /// predictor uncertainty, so this behaves identically whether
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
        /// The per-frame call: updates the apparent-motion estimate, advances the residual offset to
        /// <paramref name="nowTicks"/> on the warped time axis, and returns
        /// <paramref name="predicted"/> displaced by what remains of the offset.
        ///
        /// <b>Idempotent in <paramref name="nowTicks"/>.</b> A call at or before the last reconciled
        /// tick returns the cached previous output and does nothing else -- no estimate update, no
        /// warp, no decay step, no jerk push, no metric. This covers both the repeated-tick case
        /// <see cref="IReconciler{TState}.Reconcile"/> requires and a frame delivered out of order.
        ///
        /// <b>Order of operations matters and is deliberate.</b> The apparent-motion estimate is
        /// updated <i>before</i> the offset is seeded or advanced, so the warp for this frame is
        /// computed from motion already observed rather than from motion this frame's correction is
        /// about to add. On a seeding frame the update is skipped entirely -- see the type doc for why
        /// that one frame's finite difference is not operator motion.
        ///
        /// Emits the same four metrics on the same cadence as <c>snap</c> and <c>spring</c>:
        /// <list type="bullet">
        /// <item><c>jerk_mm_s3</c> on <b>every</b> advancing frame once four displayed positions
        /// exist, in flight or not, so the three reconcilers' distributions are populations of the
        /// same thing.</item>
        /// <item><c>time_to_convergence_ms</c> once per convergence episode, on the frame the offset
        /// first falls inside tolerance, measured from correction onset. Expected to be
        /// <i>larger</i> here than for <c>spring</c> at the same budget -- that is the cost side of
        /// this candidate's trade, not a defect.</item>
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

            bool seedingThisFrame = _correctionPending;

            UpdateApparentMotion(predicted, elapsedSeconds, seedingThisFrame);

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
                // Stop *reporting* the correction without zeroing the residual, for the reason
                // SpringReconciler spells out: truncating it is a position discontinuity of up to one
                // tolerance, which across a frame is a velocity step of tolerance/dt -- bounded in
                // absolute terms but unbounded as the frame interval shrinks, so it would falsify the
                // C1 claim at exactly the frame rates a headset runs at. The residual keeps decaying
                // to exactly zero instead.
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

            // Retained even on a seeding frame, so that the *next* frame's finite difference is taken
            // between two post-jump predictions and is clean operator motion again.
            _lastPredictedPosition = predicted.Position;
            _lastPredictedRotation = predicted.Rotation;
            _hasLastPredicted = true;

            if (hasJerk)
            {
                _metrics.Record(JerkMetric, jerkMillimetresPerSecondCubed, nowTicks);
            }

            return output;
        }

        /// <summary>
        /// Returns the reconciler to its as-constructed state: no offset, no offset velocity, nothing
        /// pending or in flight, an empty jerk history, a cleared apparent-motion estimate, and the
        /// accepted-capture baseline back to <see cref="long.MinValue"/> rather than zero. Sweeps
        /// reuse instances across trials, and an instance that remembered the previous trial's highest
        /// capture stamp would silently ignore the whole opening stretch of the next one -- which
        /// looks like a suspiciously well-behaved result rather than like a bug.
        ///
        /// The apparent-motion state is the piece specific to this reconciler and the easiest to
        /// forget: carrying a previous trial's speed estimate across the boundary would let one
        /// trial's operator motion set the correction rate at the start of an unrelated trial.
        /// Configuration and the metric sink survive; the sink is owned by whoever injected it.
        /// </summary>
        public void Reset()
        {
            ClearCorrectionState();
        }

        private void ClearCorrectionState()
        {
            _jerk.Reset();
            _offsetPosition = Vector3.Zero;
            _offsetLinearVelocity = Vector3.Zero;
            _offsetRotation = Vector3.Zero;
            _offsetAngularVelocity = Vector3.Zero;
            _correctionPending = false;
            _correctionInFlight = false;
            _correctionOnsetTicks = 0;
            _lastAcceptedCaptureTicks = long.MinValue;
            _lastReconcileTicks = long.MinValue;
            _lastOutput = Pose.Identity;
            _hasLastOutput = false;
            _lastPredictedPosition = Vector3.Zero;
            _lastPredictedRotation = Quaternion.Identity;
            _hasLastPredicted = false;
            _apparentLinearSpeed = 0f;
            _apparentAngularSpeed = 0f;
        }

        /// <summary>
        /// Advances the smoothed estimate of the operator's apparent linear and angular speed from a
        /// finite difference of the predictor's per-frame output.
        ///
        /// Skipped when there is no previous prediction to difference against, when no time has
        /// elapsed, and -- the case that matters -- when a correction is being seeded this frame, at
        /// which point the difference contains the predictor's own step and is not operator motion.
        /// See the type doc.
        ///
        /// The smoothing coefficient is <c>1 - e^(-dt/tau)</c> rather than a fixed per-frame constant,
        /// so the estimate is frame-rate independent: a sweep at one frame interval and a headset at
        /// another see the same time constant. It also makes the per-frame change of the estimate
        /// <c>O(dt)</c>, which is what keeps the warp -- and therefore the visible velocity --
        /// continuous under refinement.
        ///
        /// Allocation-free.
        /// </summary>
        private void UpdateApparentMotion(in Pose predicted, float elapsedSeconds, bool seeding)
        {
            if (seeding || !_hasLastPredicted || elapsedSeconds <= 0f)
            {
                return;
            }

            float rawLinear =
                Vector3.Distance(predicted.Position, _lastPredictedPosition) / elapsedSeconds;
            float rawAngular =
                MotionMath.RelativeRotationVector(_lastPredictedRotation, predicted.Rotation).Length()
                / elapsedSeconds;

            float alpha = 1f - MathF.Exp(-elapsedSeconds / _speedSmoothingSeconds);

            _apparentLinearSpeed += alpha * (rawLinear - _apparentLinearSpeed);
            _apparentAngularSpeed += alpha * (rawAngular - _apparentAngularSpeed);
        }

        /// <summary>
        /// Sets the offset to whatever keeps the displayed pose continuous across this frame:
        /// <c>lastDisplayed - predicted</c> in position, and the world-frame rotation vector taking
        /// <c>predicted</c> to <c>lastDisplayed</c> in orientation.
        ///
        /// The offset velocity is deliberately left alone, exactly as in <c>spring</c>: carrying it is
        /// what keeps the displayed trajectory C1 across a correction arriving while an earlier one is
        /// still decaying. Onset ticks are recorded only when a correction was not already in flight,
        /// so <c>time_to_convergence_ms</c> measures one episode from its true start.
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
        /// One warped step of the critically damped response, applied independently to the linear and
        /// angular channels against their own apparent speeds. Independent because the two are
        /// perceptually independent signals and because <c>spring</c> treats them independently too,
        /// which keeps the two reconcilers differing in exactly one thing.
        /// </summary>
        private void AdvanceOffset(float elapsedSeconds)
        {
            StepWarped(
                ref _offsetPosition, ref _offsetLinearVelocity,
                elapsedSeconds, _maxLinearSpeed, _apparentLinearSpeed);
            StepWarped(
                ref _offsetRotation, ref _offsetAngularVelocity,
                elapsedSeconds, _maxAngularSpeed, _apparentAngularSpeed);
        }

        /// <summary>
        /// The heart of the candidate. Takes a trial step at <c>w = 1</c> to find out how fast
        /// <c>spring</c> would have moved this frame, picks the largest warp whose resulting injected
        /// speed stays under <see cref="VelocityMaskingFraction"/> of the apparent speed, floors it at
        /// <see cref="MinimumTimeWarp"/> so convergence stays bounded, and then takes the real step
        /// from the <i>original</i> state on the warped interval.
        ///
        /// <b>Why a trial step rather than last frame's velocity.</b> At correction onset the offset
        /// velocity is zero, so a warp computed from it would be 1 for the onset frame and the
        /// correction would inject its largest single-frame velocity step before the masking rule ever
        /// engaged -- the peak this candidate exists to suppress, arriving unsuppressed. The trial
        /// step costs one extra closed-form evaluation and no allocation, and makes the rule bind from
        /// the first advancing frame.
        ///
        /// <b>Why the rate cap is applied to the warped-time velocity.</b> The visible speed is
        /// <c>w * |v|</c> and <c>w &lt;= 1</c>, so clamping <c>|v|</c> to the cap keeps the visible
        /// speed under it as well -- conservatively, never permissively. This matches <c>spring</c>'s
        /// semantics, where the cap likewise takes precedence over the convergence budget.
        /// </summary>
        private void StepWarped(
            ref Vector3 offset,
            ref Vector3 velocity,
            float elapsedSeconds,
            float maxSpeed,
            float apparentSpeed)
        {
            Vector3 trialOffset = offset;
            Vector3 trialVelocity = velocity;
            StepCriticallyDamped(ref trialOffset, ref trialVelocity, elapsedSeconds);

            float demandedSpeed = trialVelocity.Length();

            float warp = 1f;
            if (demandedSpeed > SpeedEpsilon)
            {
                warp = VelocityMaskingFraction * apparentSpeed / demandedSpeed;
                if (warp > 1f)
                {
                    warp = 1f;
                }
                else if (warp < MinimumTimeWarp)
                {
                    warp = MinimumTimeWarp;
                }
            }

            StepCriticallyDamped(ref offset, ref velocity, elapsedSeconds * warp);
            velocity = MotionMath.ClampMagnitude(velocity, maxSpeed);
        }

        /// <summary>
        /// One exact step of the critically damped response over an interval of <b>warped</b> time.
        ///
        /// The closed form for <c>x'' = -2w x' - w^2 x</c> with state <c>(x, v)</c>:
        /// <c>x(t+dt) = (x + (v + w x) dt) e^(-w dt)</c> and
        /// <c>v(t+dt) = (v - (v + w x) w dt) e^(-w dt)</c>. Exact and unconditionally stable, where
        /// semi-implicit Euler diverges for <c>w dt</c> above 2 -- reachable with a tight budget and a
        /// slow frame, which is exactly the impaired regime this project studies. Because it is exact
        /// in its interval argument, feeding it a warped interval integrates the same trajectory on a
        /// stretched clock rather than approximating a different one, which is what makes the
        /// monotone-envelope convergence argument in the type doc hold.
        ///
        /// Scaling the rotation vector is exact rather than a small-angle approximation: shrinking a
        /// rotation vector keeps its axis fixed and scales only its angle, the geodesic path to zero.
        /// </summary>
        private void StepCriticallyDamped(
            ref Vector3 offset, ref Vector3 velocity, float warpedSeconds)
        {
            float omegaDt = _omega * warpedSeconds;
            float decay = MathF.Exp(-omegaDt);

            Vector3 combined = velocity + _omega * offset;

            Vector3 nextOffset = (offset + combined * warpedSeconds) * decay;
            Vector3 nextVelocity = (velocity - combined * omegaDt) * decay;

            offset = nextOffset;
            velocity = nextVelocity;
        }

        /// <summary>
        /// True while any residual offset or offset velocity remains, so the decay keeps running after
        /// the correction stops being reported. Convergence ends a reporting episode, not the motion.
        /// </summary>
        private bool HasResidualOffset() =>
            _offsetPosition != Vector3.Zero ||
            _offsetLinearVelocity != Vector3.Zero ||
            _offsetRotation != Vector3.Zero ||
            _offsetAngularVelocity != Vector3.Zero;

        /// <summary>
        /// True when what remains of the offset is inside both configured tolerances. Compared as
        /// magnitudes of the residual itself rather than via <see cref="PoseMath"/> on the composed
        /// poses: the offset <i>is</i> the error, so measuring it directly cannot disagree with what
        /// the decay is acting on. Identical to <c>spring</c>, so that
        /// <c>time_to_convergence_ms</c> means the same thing for both.
        /// </summary>
        private bool OffsetIsWithinTolerance() =>
            _offsetPosition.Length() <= _positionTolerance &&
            _offsetRotation.Length() <= _orientationTolerance;

        /// <summary>
        /// The displayed pose: <paramref name="predicted"/> displaced by the residual offset. Returns
        /// <paramref name="predicted"/> bit-identically when the offset is exactly zero, so the
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
