using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Reconciliation
{
    /// <summary>
    /// Registry key <c>exp-smooth-track</c>: the displayed pose is a <b>first-order lag chasing the
    /// predictor's output directly</b>, with no residual-offset state of any kind --
    /// <c>displayed += (predicted - displayed) * (1 - exp(-dt/tau))</c>, and the geodesic equivalent
    /// for orientation. One state vector, no seeding, no correction-onset bookkeeping for the
    /// trajectory. This is the smallest thing that can be called a reconciler on this axis, and it
    /// exists to measure what that minimum costs.
    ///
    /// <b>This is deliberately not the same law as the offset family.</b> <see cref="SpringReconciler"/>,
    /// <c>budget-blend</c> and <c>velocity-match</c> all carry the visible disagreement as a residual
    /// offset on top of <c>predicted</c> and drive that offset to zero; their steady-state bias is
    /// therefore <b>zero</b>, because once the offset has decayed the displayed pose <i>is</i> the
    /// predictor's output. This class has no offset to decay, so what it converges to is not
    /// <c>predicted</c> but <c>predicted</c> delayed by roughly <c>tau</c>.
    /// <c>Reconciliation/CLAUDE.md</c>'s planned row describes <c>exp-smooth</c> as "one time
    /// constant; simple, <b>biased</b>"; this implementation is the reading under which that word is
    /// literally true.
    ///
    /// <b>The defining property: permanent, velocity-proportional tracking bias.</b> Against a
    /// prediction advancing at constant speed <c>v</c> with a fixed frame interval <c>dt</c>, the
    /// displayed error has an exact discrete fixed point
    /// <c>e* = v * dt * r / (1 - r)</c> with <c>r = exp(-dt/tau)</c>, whose small-<c>dt</c> limit is
    /// <c>v * tau</c> (more precisely <c>v * (tau - dt/2) + O(dt^2)</c>). The display settles that
    /// far behind and <b>stays there for as long as motion continues</b>; it does not catch up. See
    /// <see cref="SteadyStateLagMeters"/>, which is the closed form exposed for test rather than
    /// re-derived by one.
    ///
    /// <b>What that means for the bounded-convergence requirement, stated rather than papered over.</b>
    /// <see cref="IReconciler{TState}"/> requires that "under a <i>constant</i> correction the visible
    /// error reaches zero, or a stated bound, within a bounded time", and adds that "a reconciler that
    /// can lag indefinitely is a bug, not a tradeoff". Both halves are proved by test here, and they
    /// point in opposite directions:
    /// <list type="bullet">
    /// <item><b>Static truth (a genuinely constant correction): satisfied cleanly.</b> The error
    /// decays as <c>|e0| exp(-t/tau)</c> and is at
    /// <see cref="SettledFractionOfInitialError"/> of its initial magnitude exactly at
    /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>, by construction of <c>tau</c>.</item>
    /// <item><b>Moving truth: not satisfied in the "reaches zero" sense at all.</b> The bound is
    /// <c>v * tau</c>, which is stated, provable and tight -- but it is proportional to operator
    /// speed rather than to anything the configuration controls, and it never shrinks. Whether a
    /// velocity-proportional bound counts as "a stated bound" or as "lagging indefinitely" is a
    /// judgement about the axis's requirements, not about this code, and it is deliberately left to
    /// a human. The honest closed form is supplied so that judgement can be made on a number.</item>
    /// </list>
    ///
    /// <b>C1 continuity: C0 always, C1 never at a predictor jump -- quantified, not claimed.</b>
    /// Displayed position is continuous by construction: the output moves by at most a fraction of
    /// the current disagreement each frame and can never step. Displayed <i>velocity</i>, however, is
    /// proportional to <c>predicted - displayed</c>, so when the predictor's output jumps by
    /// <c>d</c> (which is exactly what happens when it re-anchors on a late authoritative sample),
    /// the displayed velocity steps by <c>(1 - exp(-dt/tau)) * d / dt</c>, tending to <c>d / tau</c>
    /// as the frame interval shrinks. That is a real C1 violation. It differs from <c>snap</c>'s in
    /// kind, not merely in size: <c>snap</c>'s velocity step is <c>d / dt</c> and grows without bound
    /// as the frame rate rises, while this one is bounded by <c>d / tau</c> at any frame rate. It
    /// also differs from the offset family's one-frame hold, which is an O(1) velocity <i>dropout</i>
    /// at onset rather than a magnitude-proportional velocity <i>step</i>. All three are violations
    /// of the same clause; they are not interchangeable and should not be summarised as one.
    /// <see cref="VelocityStepAtPredictorJump"/> is the closed form, asserted by test.
    ///
    /// <b>No one-frame hold, and that is a structural consequence rather than a fix.</b> The offset
    /// family holds the display for exactly one frame at every correction onset and retarget, because
    /// it seeds its offset from the previous frame's displayed pose. That hold is a deliberate,
    /// measured tradeoff (see <c>Reconciliation/CLAUDE.md</c>'s "Tried and rejected": removing it was
    /// implemented, swept and reverted after jerk p99 regressed 81-865% on impaired profiles and the
    /// spread between reconcilers collapsed to 1.01x). This class has no hold only because it seeds
    /// no offset -- it is not a solution to that problem and does not address the exact-cancellation
    /// requirement the reverted attempt failed on. The practical consequence for anyone comparing
    /// numbers: <b>this reconciler's <c>jerk_mm_s3</c> distribution is not shaped by the same
    /// mechanism as the offset family's</b>, so a jerk difference between them is not evidence about
    /// convergence laws.
    ///
    /// <b>Fields of <see cref="ReconcilerConfig"/> it reads:</b>
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> and
    /// <see cref="ReconcilerConfig.ConvergenceOrientationToleranceRadians"/> (to decide whether an
    /// arriving sample is worth reporting as a correction, and to decide when the display has caught
    /// up), <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> (sets <c>tau</c>), and both rate
    /// caps. <b>Field it ignores:</b> <see cref="ReconcilerConfig.RollbackHistoryCapacity"/> --
    /// nothing here rewinds. No configuration field was added for this reconciler.
    ///
    /// <b>The rate caps mean something slightly different here, and that is unavoidable.</b> In the
    /// offset family the caps bound the speed of the <i>correction</i> only, because the correction
    /// is a separable offset. This law has no such decomposition -- the single chase step carries
    /// both the correction and the operator's own motion -- so
    /// <see cref="ReconcilerConfig.MaxCorrectionLinearSpeedMetersPerSecond"/> and
    /// <see cref="ReconcilerConfig.MaxCorrectionAngularSpeedRadPerSecond"/> here bound the
    /// <i>total displayed speed</i>. That is a strictly tighter constraint, it is a direct
    /// consequence of having no offset state, and a sweep that sets a low cap will slow this
    /// reconciler where it would only slow the others' corrections. Clamping a magnitude is
    /// continuous, so the caps cannot introduce a discontinuity; they can only deepen the lag.
    ///
    /// <b>Time is always a parameter.</b> The injected <see cref="ITimeAuthority"/> is read for
    /// <see cref="ITimeAuthority.TicksPerSecond"/> only and never for
    /// <see cref="ITimeAuthority.NowTicks"/>, the same discipline <see cref="SnapReconciler"/> and
    /// <c>Pipeline/OperatorEndpoint</c> document.
    ///
    /// Deterministic and allocation-free: the only buffer is the shared jerk estimator's history,
    /// sized once in the constructor. Not thread-safe, by contract.
    /// </summary>
    public sealed class TrackingLagReconciler : IReconciler<Pose>
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
        /// The fraction of the initial error the first-order envelope must have decayed to by
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>, <b>under static truth</b>. This
        /// is what gives that config field a precise meaning for this reconciler. It is deliberately
        /// the same 1% <see cref="SpringReconciler"/> uses, so that "converged within the budget"
        /// denotes the same envelope on both; it is a definition, not a tuning knob, and changing it
        /// changes what every recorded result means.
        /// </summary>
        private const float SettledFractionOfInitialError = 0.01f;

        /// <summary>
        /// The <c>x</c> solving <c>e^(-x) = </c><see cref="SettledFractionOfInitialError"/>, so that
        /// <c>tau = budgetSeconds / x</c> puts the envelope exactly at that fraction when the budget
        /// elapses. Unlike <see cref="SpringReconciler"/>'s critically damped analogue this one does
        /// have a closed form -- it is simply <c>ln(100)</c> -- but it is still written as a checked
        /// literal rather than a <c>MathF.Log</c> call for the same reason: a literal is auditable
        /// where a computation is not, and
        /// <c>TrackingLagReconcilerTests.LagSettleConstantMatchesItsDefiningEquation</c> recomputes
        /// <c>e^(-x)</c> and asserts it lands on the fraction, so the two constants cannot drift
        /// apart silently.
        /// </summary>
        private const float LagSettleConstant = 4.6051702f;

        private readonly float _positionTolerance;
        private readonly float _orientationTolerance;
        private readonly float _maxLinearSpeed;
        private readonly float _maxAngularSpeed;
        private readonly float _tau;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        /// <summary>
        /// Shared with every other reconciler so that <c>jerk_mm_s3</c> means one thing across the
        /// axis being compared -- see <see cref="DisplayedJerkEstimator"/> for why that matters.
        /// </summary>
        private readonly DisplayedJerkEstimator _jerk;

        /// <summary>
        /// The entire trajectory state: the pose currently displayed. There is no offset, no offset
        /// velocity and no target -- that absence is the design.
        /// </summary>
        private Pose _displayed;

        private bool _hasDisplayed;

        /// <summary>
        /// A qualifying authoritative sample has arrived and the next advancing
        /// <see cref="Reconcile"/> should open a convergence episode if one is not already open. It
        /// changes <i>no</i> trajectory state -- see <see cref="Observe"/>.
        /// </summary>
        private bool _correctionPending;

        private bool _correctionInFlight;
        private long _correctionOnsetTicks;

        /// <summary>
        /// Whether the displayed pose was inside both tolerances of the prediction on the most recent
        /// advancing frame. Cached rather than recomputed in <see cref="IsConverged"/> because the
        /// getter has no prediction to compare against -- the disagreement here exists only as the
        /// difference between two poses, one of which arrives as a <see cref="Reconcile"/> argument.
        /// True before any frame has been displayed, as the contract requires of a fresh instance.
        /// </summary>
        private bool _displayHasCaughtUp;

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
        /// budget has no defined time constant and a zero rate cap would freeze the display forever.
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
        public TrackingLagReconciler(ReconcilerConfig config, IMetricSink metrics, ITimeAuthority clock)
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
                    "MaxTimeToConvergenceTicks must be positive: it sets the lag time constant, so " +
                    "there is no defined response without it.");
            }

            if (config.MaxCorrectionLinearSpeedMetersPerSecond <= 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxCorrectionLinearSpeedMetersPerSecond,
                    "MaxCorrectionLinearSpeedMetersPerSecond must be positive: a zero cap would " +
                    "freeze the displayed pose permanently.");
            }

            if (config.MaxCorrectionAngularSpeedRadPerSecond <= 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxCorrectionAngularSpeedRadPerSecond,
                    "MaxCorrectionAngularSpeedRadPerSecond must be positive: a zero cap would " +
                    "freeze the displayed orientation permanently.");
            }

            _positionTolerance = config.ConvergencePositionToleranceMeters;
            _orientationTolerance = config.ConvergenceOrientationToleranceRadians;
            _maxLinearSpeed = config.MaxCorrectionLinearSpeedMetersPerSecond;
            _maxAngularSpeed = config.MaxCorrectionAngularSpeedRadPerSecond;
            _metrics = metrics;
            _ticksPerSecond = clock.TicksPerSecond;

            float budgetSeconds = config.MaxTimeToConvergenceTicks / (float)clock.TicksPerSecond;
            _tau = budgetSeconds / LagSettleConstant;

            _jerk = new DisplayedJerkEstimator(clock.TicksPerSecond);

            ClearState();
        }

        /// <summary>
        /// The lag time constant in seconds, derived from
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> against the
        /// <see cref="SettledFractionOfInitialError"/> envelope. Exposed because it is the single
        /// number that characterises this reconciler; a test that had to re-derive it would be
        /// asserting against its own copy of the formula rather than against the implementation.
        /// </summary>
        public float TimeConstantSeconds => _tau;

        /// <summary>
        /// The exact steady-state positional lag, in metres, that this reconciler settles into when
        /// the predictor's output advances at <paramref name="speedMetersPerSecond"/> with a fixed
        /// frame interval of <paramref name="frameSeconds"/>: <c>v * dt * r / (1 - r)</c> with
        /// <c>r = exp(-dt/tau)</c>.
        ///
        /// This is the <b>bias bound</b> the type doc refers to, published as code rather than as
        /// prose so it can be asserted against a simulation instead of trusted. Its small-<c>dt</c>
        /// limit is <c>v * tau</c>; the <c>-dt/2</c> first-order term is the sampling offset from
        /// stepping toward the <i>current</i> frame's prediction rather than the next one's, and it
        /// is real rather than an approximation error.
        ///
        /// Not part of <see cref="IReconciler{TState}"/> -- it is specific to a law that has a
        /// steady-state bias at all, which the offset-carrying reconcilers do not.
        /// </summary>
        public float SteadyStateLagMeters(float speedMetersPerSecond, float frameSeconds)
        {
            if (frameSeconds <= 0f)
            {
                return 0f;
            }

            float r = MathF.Exp(-frameSeconds / _tau);
            return speedMetersPerSecond * frameSeconds * r / (1f - r);
        }

        /// <summary>
        /// The displayed-velocity discontinuity, in metres/second, injected when the predictor's
        /// output jumps by <paramref name="jumpMeters"/> between two frames spaced
        /// <paramref name="frameSeconds"/> apart: <c>(1 - exp(-dt/tau)) * d / dt</c>.
        ///
        /// This is the quantified C1 violation, published for the same reason as
        /// <see cref="SteadyStateLagMeters"/>. Note it is bounded above by <c>d / tau</c> for every
        /// frame interval -- taking <c>dt -> 0</c> makes it converge to that value rather than
        /// diverge, which is the one structural respect in which it is unlike <c>snap</c>'s.
        /// </summary>
        public float VelocityStepAtPredictorJump(float jumpMeters, float frameSeconds)
        {
            if (frameSeconds <= 0f)
            {
                return 0f;
            }

            return (1f - MathF.Exp(-frameSeconds / _tau)) * jumpMeters / frameSeconds;
        }

        /// <summary>
        /// True when the displayed pose is within both configured tolerances of the last prediction
        /// it was shown, i.e. the display has caught up with the predictor -- and the predictor is
        /// what carries truth once <c>Observe</c> has reached it. True on a freshly constructed or
        /// freshly <see cref="Reset"/> instance, as <see cref="IReconciler{TState}.IsConverged"/>
        /// requires, and true before any frame has been displayed.
        ///
        /// <b>This will read false for the entire duration of continuous operator motion</b>, because
        /// the steady-state lag <c>v * tau</c> generally exceeds
        /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/>. That is not a defect in
        /// the flag: it is the flag correctly reporting that this law does not converge while motion
        /// continues. Anything consuming <c>IsConverged</c> as "idle" will see a materially different
        /// duty cycle here than on the offset-carrying reconcilers, and
        /// <c>time_to_convergence_ms</c> inherits that difference -- see <see cref="Reconcile"/>.
        ///
        /// <b>It is deliberately a statement about the display, not about a correction episode.</b>
        /// The offset-carrying reconcilers can define this as "no correction is in flight", because
        /// for them no correction in flight implies zero residual implies the display <i>is</i> the
        /// prediction. That implication does not hold here: this law lags whether or not anything
        /// was ever reported as a correction, so an episode flag alone would report "converged"
        /// while the display sat a steady <c>v * tau</c> behind truth. The third term below is what
        /// stops this flag from quietly overstating the reconciler.
        /// </summary>
        public bool IsConverged => !_correctionPending && !_correctionInFlight && _displayHasCaughtUp;

        /// <summary>
        /// Truth arrived. Measures the disagreement against what was displayed for that same instant
        /// and, if it exceeds tolerance, reports it and flags that a convergence episode should be
        /// open. Changes nothing visible, and -- unlike every other reconciler on this axis --
        /// changes <b>no trajectory state at all</b>, because there is no offset to seed. The
        /// bookkeeping here exists purely so the correction-cost metrics mean the same thing they
        /// mean elsewhere on the axis.
        ///
        /// <b>Ordering.</b> Identical to <see cref="SpringReconciler"/> deliberately: a sample whose
        /// capture stamp is not strictly greater than the last accepted one is stale or a duplicate
        /// and is ignored whole -- no metric, no state change. Equal stamps are rejected because a
        /// duplicate must not be counted as a second correction; correction rate is a reported metric
        /// (docs/metrics.md §5 derives it by counting <c>correction_magnitude_mm</c> samples) and
        /// double-counting corrupts it.
        ///
        /// <b>Within tolerance means no correction at all</b>, not a zero-magnitude one -- the same
        /// load-bearing degenerate case <c>snap</c> and <c>spring</c> document, and the reason this
        /// reconciler emits <c>correction_magnitude_mm</c> and <c>correction_magnitude_deg</c> on
        /// exactly the same samples, at exactly the same stamps, as the rest of the axis. Those two
        /// metrics are a control across this axis by construction, not a differentiator.
        ///
        /// Allocation-free.
        /// </summary>
        /// <param name="diagnostics">
        /// Ignored. This reconciler's response is set by the convergence budget alone, so it behaves
        /// identically whether <see cref="PredictorDiagnostics.HasUncertainty"/> is true or false --
        /// the graceful degradation <see cref="IReconciler{TState}.Observe"/> asks for, reached by
        /// not depending on the field at all. <see cref="PredictorDiagnostics.None"/> is equally
        /// fine.
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
        /// The per-frame call: moves the displayed pose a fixed fraction of the way toward
        /// <paramref name="predicted"/> and returns it.
        ///
        /// <b>Idempotent in <paramref name="nowTicks"/>.</b> A call at or before the last reconciled
        /// tick returns the cached previous output and does nothing else -- no step, no jerk push, no
        /// metric. <see cref="IReconciler{TState}.Reconcile"/> requires that two calls with the same
        /// <c>nowTicks</c> return the same state and not advance the correction twice, and the same
        /// guard covers a frame delivered out of order.
        ///
        /// <b>The first call of a trial adopts <paramref name="predicted"/> exactly.</b> There is no
        /// previously displayed pose to lag behind, and starting the display at
        /// <see cref="Pose.Identity"/> would invent an opening correction rather than reflect one.
        /// Note the consequence for the ~1.3 m opening transient every sweep trial contains (the
        /// plant starts at <see cref="Pose.Identity"/> while the operator trace starts at z≈1.3):
        /// this reconciler absorbs that transient as a smooth exponential ramp, where the
        /// offset-carrying reconcilers absorb it as a one-frame hold followed by a decay. The opening
        /// frames of a trial are therefore not comparable between the two families, whatever the
        /// percentiles say.
        ///
        /// Emits:
        /// <list type="bullet">
        /// <item><c>jerk_mm_s3</c> on <b>every</b> advancing frame once four displayed positions
        /// exist -- the same unconditional cadence and the same shared
        /// <see cref="DisplayedJerkEstimator"/> the whole axis uses, which is what makes the
        /// distributions populations of the same thing.</item>
        /// <item><c>time_to_convergence_ms</c> once per convergence episode, on the frame the
        /// displayed pose first falls inside both tolerances of <paramref name="predicted"/>,
        /// measured from the first advancing frame after a qualifying sample. <b>This metric has a
        /// materially different emission rate here than on the offset-carrying reconcilers, and
        /// that is a property of the law rather than a bug.</b> An episode opened while the operator
        /// is moving cannot close until the motion slows enough that <c>v * tau</c> falls inside
        /// tolerance, so on a continuously moving trace this reconciler may emit few samples or
        /// none where <c>spring</c> emits one per correction. Pooled <c>time_to_convergence_ms</c>
        /// percentiles are therefore <b>not</b> comparable between this reconciler and the offset
        /// family: a small, favourable-looking sample here is survivorship, not speed. The definition
        /// is unchanged from docs/metrics.md §5 -- what differs is which episodes ever reach the
        /// terminating condition.</item>
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

            if (!_hasDisplayed)
            {
                _displayed = predicted;
                _hasDisplayed = true;
            }
            else
            {
                // _hasDisplayed implies a previous advancing call, so _lastReconcileTicks is real and
                // the idempotence guard above has already established nowTicks > _lastReconcileTicks.
                float elapsedSeconds = (nowTicks - _lastReconcileTicks) / (float)_ticksPerSecond;

                if (elapsedSeconds > 0f)
                {
                    StepTowards(predicted, elapsedSeconds);
                }
            }

            if (_correctionPending)
            {
                if (!_correctionInFlight)
                {
                    _correctionOnsetTicks = nowTicks;
                    _correctionInFlight = true;
                }

                _correctionPending = false;
            }

            _displayHasCaughtUp = DisplayIsWithinToleranceOf(predicted);

            if (_correctionInFlight && _displayHasCaughtUp)
            {
                _correctionInFlight = false;

                _metrics.Record(
                    TimeToConvergenceMsMetric,
                    (nowTicks - _correctionOnsetTicks) / (double)_ticksPerSecond * MillisecondsPerSecond,
                    nowTicks);
            }

            Pose output = _displayed;

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
        /// Returns the reconciler to its as-constructed state: nothing displayed, nothing pending or
        /// in flight, an empty jerk history, and the accepted-capture baseline back to
        /// <see cref="long.MinValue"/> rather than zero. Sweeps reuse instances across trials, and a
        /// reconciler that remembered the previous trial's highest capture stamp would silently
        /// ignore the whole opening stretch of the next one -- which looks like a suspiciously
        /// well-behaved result rather than like a bug. Same reasoning as <c>SnapReconciler.Reset</c>
        /// and <c>SpringReconciler.Reset</c>. Configuration and the metric sink survive; the sink has
        /// its own <c>Reset</c> and is owned by whoever injected it.
        /// </summary>
        public void Reset()
        {
            ClearState();
        }

        private void ClearState()
        {
            _jerk.Reset();
            _displayed = Pose.Identity;
            _hasDisplayed = false;
            _correctionPending = false;
            _correctionInFlight = false;
            _correctionOnsetTicks = 0;
            _displayHasCaughtUp = true;
            _lastAcceptedCaptureTicks = long.MinValue;
            _lastReconcileTicks = long.MinValue;
            _lastOutput = Pose.Identity;
            _hasLastOutput = false;
        }

        /// <summary>
        /// One exact step of the first-order lag, applied to position directly and to orientation
        /// along the geodesic.
        ///
        /// The blend fraction is <c>1 - exp(-dt/tau)</c> rather than a fixed per-frame constant. That
        /// is what makes the response frame-rate independent: the same elapsed time produces the same
        /// trajectory whether it arrived as one long frame or several short ones, so a sweep and a
        /// headset agree, and a jittered frame schedule does not change the effective time constant.
        /// A hard-coded alpha would silently retune the reconciler whenever the frame rate moved,
        /// which on the impaired profiles this project studies is constantly.
        ///
        /// Orientation reuses <see cref="MotionMath.RelativeRotationVector"/> and
        /// <see cref="MotionMath.IntegrateWorld"/>: scaling the relative rotation vector by the same
        /// fraction keeps its axis fixed and scales only its angle, which is the geodesic (slerp)
        /// path and is exact rather than a small-angle approximation.
        ///
        /// Both rate caps are applied to the step itself via
        /// <see cref="MotionMath.ClampMagnitude"/>, as a distance budget of <c>maxSpeed * dt</c>.
        /// Clamping a magnitude is continuous in the disagreement, so it cannot introduce a
        /// discontinuity; it only deepens the lag. See the type doc for why the caps bound total
        /// displayed speed here rather than correction speed.
        ///
        /// <b>Bit-exact pass-through when there is nothing to chase.</b> If the displayed pose
        /// already equals <paramref name="predicted"/> exactly, the pose is returned untouched rather
        /// than run through the arithmetic, so a degenerate configuration reproduces its input
        /// instead of drifting by quaternion renormalization noise -- the same reasoning
        /// <see cref="MotionMath.IntegrateWorld"/> gives for its own zero-rotation short circuit.
        /// </summary>
        private void StepTowards(in Pose predicted, float elapsedSeconds)
        {
            if (_displayed.Position == predicted.Position && _displayed.Rotation == predicted.Rotation)
            {
                return;
            }

            float blend = 1f - MathF.Exp(-elapsedSeconds / _tau);

            Vector3 positionStep = MotionMath.ClampMagnitude(
                (predicted.Position - _displayed.Position) * blend, _maxLinearSpeed * elapsedSeconds);

            Vector3 rotationStep = MotionMath.ClampMagnitude(
                MotionMath.RelativeRotationVector(_displayed.Rotation, predicted.Rotation) * blend,
                _maxAngularSpeed * elapsedSeconds);

            _displayed = new Pose(
                _displayed.Position + positionStep,
                MotionMath.IntegrateWorld(_displayed.Rotation, rotationStep));
        }

        /// <summary>
        /// True when the displayed pose is inside both configured tolerances of the prediction it is
        /// chasing. Measured between the two poses via <see cref="PoseMath"/> rather than from a
        /// residual, because unlike the offset family there is no residual here -- the disagreement
        /// exists only as the difference between two poses.
        /// </summary>
        private bool DisplayIsWithinToleranceOf(in Pose predicted) =>
            PoseMath.PositionErrorMeters(_displayed, predicted) <= _positionTolerance &&
            PoseMath.OrientationErrorRadians(_displayed, predicted) <= _orientationTolerance;
    }
}
