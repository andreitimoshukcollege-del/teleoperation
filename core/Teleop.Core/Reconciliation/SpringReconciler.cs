using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Reconciliation
{
    /// <summary>
    /// Registry key <c>spring</c>: carry the visible disagreement as a <b>residual offset</b> on top
    /// of the predictor's output and drive that offset to zero with a critically damped second-order
    /// response. Where <see cref="SnapReconciler"/> spends the entire correction in one frame, this
    /// spends it over a configured budget, trading correction magnitude for correction <i>rate</i> and
    /// peak jerk -- the tradeoff <c>Reconciliation/CLAUDE.md</c> exists to measure.
    ///
    /// <b>Critically damped specifically, not underdamped.</b> Critical damping is the fastest
    /// second-order approach that never overshoots. Overshoot here would mean the displayed pose
    /// travelling <i>past</i> the authoritative one and coming back, which reads to an operator as the
    /// robot wobbling after every packet -- worse than the error it corrects, and it would invalidate
    /// the "no overshoot" claim the planned-table row makes.
    ///
    /// <b>Why an offset rather than a target.</b> The predictor is the thing being corrected: once
    /// <c>Observe</c> reaches it, its output already incorporates truth, so displaying it directly
    /// <i>is</i> the snap. What this class hides is the jump the predictor's own correction caused. So
    /// at correction onset the offset is seeded to <c>lastDisplayed - predicted</c>, which makes the
    /// displayed pose continuous across the onset frame by construction, and then decays to zero,
    /// which makes the displayed pose converge onto the corrected prediction rather than onto a stale
    /// sample. Decaying toward zero is therefore convergence toward <i>truth</i>, not away from it.
    ///
    /// <b>C1 continuity, and how it survives a retarget.</b> The offset's velocity is carried across
    /// re-seeding rather than reset, so a second correction arriving mid-flight bends the trajectory
    /// instead of restarting it. Combined with a critically damped response whose velocity is zero at
    /// onset (<c>x'(0) = 0</c> when seeded from rest), the displayed position is continuous and so is
    /// its first derivative -- the full <see cref="IReconciler{TState}"/> C1 clause, which every
    /// reconciler except <c>snap</c> owes its callers.
    ///
    /// <b>Bounded convergence, and its stated bound.</b> The closed-form critically damped envelope is
    /// <c>|o(t)| = |o0| (1 + wt) e^(-wt)</c>. <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>
    /// is read as the time by which that envelope must fall to
    /// <see cref="SettledFractionOfInitialError"/> of the initial error, which fixes <c>w</c> --
    /// see <see cref="CriticalSettleConstant"/>. <see cref="IReconciler{TState}"/> permits "zero, or a
    /// stated bound", and this is the stated bound. Two honest caveats:
    /// <list type="bullet">
    /// <item>The bound is <i>relative</i>. A correction large enough that 1% of it still exceeds
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> needs slightly longer than
    /// the budget to land inside the tolerance. The response shape is deliberately independent of
    /// correction magnitude; making <c>w</c> depend on it would give every correction a different
    /// dynamics and make the axis much harder to attribute.</item>
    /// <item>The rate caps take precedence. If
    /// <see cref="ReconcilerConfig.MaxCorrectionLinearSpeedMetersPerSecond"/> or
    /// <see cref="ReconcilerConfig.MaxCorrectionAngularSpeedRadPerSecond"/> is low enough to clip the
    /// response's peak speed, the correction converges <i>slower</i> than the budget. That is the
    /// documented interaction, not a bug: a rate cap exists precisely to bound apparent motion even
    /// at the cost of convergence time.</item>
    /// </list>
    ///
    /// <b>Fields of <see cref="ReconcilerConfig"/> it reads:</b> all but one --
    /// <see cref="ReconcilerConfig.ConvergencePositionToleranceMeters"/> and
    /// <see cref="ReconcilerConfig.ConvergenceOrientationToleranceRadians"/> (both to decide whether
    /// an arriving sample is worth correcting and to decide when a correction has landed),
    /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> (sets <c>w</c>), and both rate caps.
    /// <b>Field it ignores:</b> <see cref="ReconcilerConfig.RollbackHistoryCapacity"/> -- for
    /// <c>rollback</c>, which re-simulates buffered inputs. Nothing here rewinds.
    ///
    /// <b>Time is always a parameter.</b> The injected <see cref="ITimeAuthority"/> is read for
    /// <see cref="ITimeAuthority.TicksPerSecond"/> only and never for
    /// <see cref="ITimeAuthority.NowTicks"/>, the same discipline <see cref="SnapReconciler"/> and
    /// <c>Pipeline/OperatorEndpoint</c> document.
    ///
    /// Deterministic and allocation-free: the only buffer is the shared jerk estimator's history,
    /// sized once in the constructor. Not thread-safe, by contract.
    /// </summary>
    public sealed class SpringReconciler : IReconciler<Pose>
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
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>. This is what gives that config
        /// field a precise meaning for this reconciler; it is a definition, not a tuning knob, and
        /// changing it changes what "converged within the budget" means for every recorded result.
        /// </summary>
        private const float SettledFractionOfInitialError = 0.01f;

        /// <summary>
        /// The <c>x</c> solving <c>(1 + x) e^(-x) = </c>
        /// <see cref="SettledFractionOfInitialError"/>, so that <c>w = x / budgetSeconds</c> puts the
        /// envelope exactly at that fraction when the budget elapses. There is no closed form (it is a
        /// Lambert-W root), so the constant is precomputed rather than solved at construction: a
        /// numeric solve per instance would be startup cost for a value that can never change, and a
        /// literal is auditable where an iteration is not. Verified by
        /// <c>SpringReconcilerTests.CriticalSettleConstantMatchesItsDefiningEquation</c>, which
        /// recomputes <c>(1 + x) e^(-x)</c> and asserts it lands on the fraction -- so the two
        /// constants cannot drift apart silently.
        /// </summary>
        private const float CriticalSettleConstant = 6.6383521f;

        private readonly float _positionTolerance;
        private readonly float _orientationTolerance;
        private readonly float _maxLinearSpeed;
        private readonly float _maxAngularSpeed;
        private readonly float _omega;
        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        /// <summary>
        /// Shared with every other reconciler so that <c>jerk_mm_s3</c> means one thing across the
        /// axis being compared -- see <see cref="DisplayedJerkEstimator"/> for why that matters.
        /// </summary>
        private readonly DisplayedJerkEstimator _jerk;

        private Vector3 _offsetPosition;
        private Vector3 _offsetLinearVelocity;
        private Vector3 _offsetRotation;
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

        /// <param name="config">
        /// Parameters; see the type doc for which fields are read. Unlike <c>snap</c>, this
        /// reconciler <b>requires</b> a positive
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/> and positive rate caps: a zero
        /// budget has no defined response and a zero rate cap could never converge, which would
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
        public SpringReconciler(ReconcilerConfig config, IMetricSink metrics, ITimeAuthority clock)
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
                    "MaxTimeToConvergenceTicks must be positive: it sets the spring's natural " +
                    "frequency, so there is no defined response without it.");
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

            _jerk = new DisplayedJerkEstimator(clock.TicksPerSecond);

            ClearCorrectionState();
        }

        /// <summary>
        /// The natural frequency in radians/second derived from
        /// <see cref="ReconcilerConfig.MaxTimeToConvergenceTicks"/>. Exposed because it is the single
        /// number that characterises this reconciler's response, and a test that had to re-derive it
        /// would be asserting against its own copy of the formula rather than against the
        /// implementation.
        /// </summary>
        public float NaturalFrequencyRadiansPerSecond => _omega;

        /// <summary>
        /// True when no correction is pending and none is in flight -- the displayed pose is the
        /// predictor's output with a residual offset inside tolerance. True on a freshly constructed
        /// or freshly <see cref="Reset"/> instance, as
        /// <see cref="IReconciler{TState}.IsConverged"/> requires.
        ///
        /// Unlike <c>snap</c>, whose non-converged window is one frame wide, this stays false for the
        /// whole decay, which is what makes it meaningful to assert bounded convergence against.
        /// </summary>
        public bool IsConverged => !_correctionPending && !_correctionInFlight;

        /// <summary>
        /// Truth arrived. Measures the disagreement against what was displayed for that same instant
        /// and, if it exceeds tolerance, flags that the offset must be re-seeded on the next
        /// <see cref="Reconcile"/>. Changes nothing visible here, per
        /// <see cref="IReconciler{TState}.Observe"/>.
        ///
        /// <b>Ordering.</b> A sample whose capture stamp is not strictly greater than the last
        /// accepted one is stale or a duplicate and is ignored whole -- no re-seed, no metric, no
        /// state change of any kind. Equal stamps are rejected for the reason
        /// <see cref="IReconciler{TState}.Observe"/> states outright: a duplicate must not be counted
        /// as a second correction, because correction rate is a reported metric and double-counting
        /// corrupts it.
        ///
        /// <b>Within tolerance means no correction at all</b>, not a zero-magnitude one -- the same
        /// load-bearing degenerate case <c>snap</c> documents. With a passthrough predictor whose
        /// output already equals truth, the pipeline must reduce to exact pass-through, emitting no
        /// correction-cost samples and never leaving <see cref="IsConverged"/>.
        ///
        /// <b>Re-seed, do not stack.</b> A second qualifying sample replaces the pending re-seed
        /// rather than queueing behind it, and the actual offset is computed from the live displayed
        /// pose at the next <see cref="Reconcile"/>, so several samples arriving between two frames
        /// cost one re-seed against the newest truth.
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

            // Stamped at the sample's own capture time, matching SnapReconciler exactly: the metric
            // definitions must be identical across the axis or the comparison measures the metric
            // rather than the reconciler.
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
        /// The per-frame call: advances the residual offset to <paramref name="nowTicks"/> and returns
        /// <paramref name="predicted"/> displaced by what remains of it.
        ///
        /// <b>Idempotent in <paramref name="nowTicks"/>.</b> A call at or before the last reconciled
        /// tick returns the cached previous output and does nothing else -- no decay step, no jerk
        /// push, no metric. <see cref="IReconciler{TState}.Reconcile"/> requires that two calls with
        /// the same <c>nowTicks</c> return the same state and not advance the correction twice, and
        /// the same guard covers a frame delivered out of order.
        ///
        /// <b>The first advancing call after a pending correction</b> re-seeds the offset from the
        /// live displayed pose (see the type doc) and does not decay, because there is no elapsed
        /// interval to decay over; decay begins on the following frame. On the very first call of a
        /// trial there is no previously displayed pose to stay continuous with, so the offset stays
        /// zero and <paramref name="predicted"/> is returned unmodified -- correcting toward a pose
        /// that was never displayed would be inventing a discontinuity rather than hiding one.
        ///
        /// Emits:
        /// <list type="bullet">
        /// <item><c>jerk_mm_s3</c> on <b>every</b> advancing frame once four displayed positions
        /// exist -- the same unconditional cadence <see cref="SnapReconciler"/> uses, which is what
        /// makes the two distributions populations of the same thing (displayed-trajectory jerk)
        /// rather than one sample per correction against a hundred.</item>
        /// <item><c>time_to_convergence_ms</c>, once per convergence episode, on the frame the
        /// offset first falls inside tolerance, measured from correction onset. <c>snap</c> reports 0 here; this reports the
        /// real elapsed time, which is the whole quantity this axis trades against jerk.</item>
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
                // zero once it fell inside tolerance is tempting and wrong: it is a position
                // discontinuity of up to one tolerance, which across a frame is a velocity step of
                // tolerance/dt -- invisible in absolute terms but unbounded as the frame interval
                // shrinks, so it would falsify the C1 claim above at exactly the frame rates a
                // headset runs at. The residual instead keeps decaying geometrically to exactly
                // zero (a few dozen frames at any sane budget), which keeps the visible output C1
                // and still makes Compose's pass-through short circuit reachable again.
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

            // Unconditional, matching SnapReconciler: jerk is a property of the displayed
            // trajectory (docs/metrics.md §5), not of a correction episode. Conditioning it on a
            // correction being in flight is what made the two reconcilers' sample sets
            // incomparable -- one sample per correction against roughly one per frame of one.
            if (hasJerk)
            {
                _metrics.Record(JerkMetric, jerkMillimetresPerSecondCubed, nowTicks);
            }

            return output;
        }

        /// <summary>
        /// Returns the reconciler to its as-constructed state: no offset, no offset velocity, nothing
        /// pending or in flight, an empty jerk history, and -- the one that is easy to miss -- the
        /// accepted-capture baseline back to <see cref="long.MinValue"/> rather than zero. Sweeps
        /// reuse instances across trials, and a reconciler that remembered the previous trial's
        /// highest capture stamp would silently ignore the whole opening stretch of the next one,
        /// which looks like a suspiciously well-behaved result rather than like a bug. Same reasoning
        /// as <c>SnapReconciler.Reset</c> and <c>Plant/RigidBodyPlant.Reset</c>. Configuration and the
        /// metric sink survive; the sink has its own <c>Reset</c> and is owned by whoever injected it.
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
        }

        /// <summary>
        /// Sets the offset to whatever keeps the displayed pose continuous across this frame:
        /// <c>lastDisplayed - predicted</c> in position, and the world-frame rotation vector taking
        /// <c>predicted</c> to <c>lastDisplayed</c> in orientation.
        ///
        /// The offset <i>velocity</i> is deliberately left alone. Carrying it is what keeps the
        /// displayed trajectory C1 across a correction that arrives while an earlier one is still
        /// decaying: zeroing it would put a velocity step into the visible output on exactly the
        /// frames a lossy link produces most often.
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
        /// One exact step of the critically damped response, applied to the position offset and the
        /// rotation-vector offset independently.
        ///
        /// Uses the closed form rather than a numerical integrator. For <c>x'' = -2w x' - w^2 x</c>
        /// with state <c>(x, v)</c>:
        /// <c>x(t+dt) = (x + (v + w x) dt) e^(-w dt)</c> and
        /// <c>v(t+dt) = (v - (v + w x) w dt) e^(-w dt)</c>.
        /// Exact and unconditionally stable, where semi-implicit Euler would diverge for
        /// <c>w dt</c> above 2 -- reachable with a tight convergence budget and a slow frame, i.e.
        /// exactly the impaired conditions this project studies. It is also frame-rate independent,
        /// so a sweep and a headset produce the same trajectory from the same samples.
        ///
        /// Scaling the rotation vector is exact rather than a small-angle approximation: shrinking a
        /// rotation vector keeps its axis fixed and scales only its angle, which is the geodesic path
        /// to zero rotation.
        ///
        /// The rate caps are applied to the offset velocities after the step, via
        /// <see cref="MotionMath.ClampMagnitude"/>. Clamping a magnitude is continuous, so it cannot
        /// introduce the velocity discontinuity the C1 clause forbids; it only stretches convergence,
        /// as the type doc states.
        /// </summary>
        private void AdvanceOffset(float elapsedSeconds)
        {
            StepCriticallyDamped(
                ref _offsetPosition, ref _offsetLinearVelocity, elapsedSeconds, _maxLinearSpeed);
            StepCriticallyDamped(
                ref _offsetRotation, ref _offsetAngularVelocity, elapsedSeconds, _maxAngularSpeed);
        }

        private void StepCriticallyDamped(
            ref Vector3 offset, ref Vector3 velocity, float elapsedSeconds, float maxSpeed)
        {
            float omegaDt = _omega * elapsedSeconds;
            float decay = MathF.Exp(-omegaDt);

            Vector3 combined = velocity + _omega * offset;

            Vector3 nextOffset = (offset + combined * elapsedSeconds) * decay;
            Vector3 nextVelocity = (velocity - combined * omegaDt) * decay;

            offset = nextOffset;
            velocity = MotionMath.ClampMagnitude(nextVelocity, maxSpeed);
        }

        /// <summary>
        /// True while any residual offset or offset velocity remains, so the decay keeps running
        /// after the correction stops being reported. Convergence ends a reporting episode, not the
        /// motion: see <see cref="Reconcile"/> for why truncating the residual instead would break
        /// C1 continuity.
        /// </summary>
        private bool HasResidualOffset() =>
            _offsetPosition != Vector3.Zero ||
            _offsetLinearVelocity != Vector3.Zero ||
            _offsetRotation != Vector3.Zero ||
            _offsetAngularVelocity != Vector3.Zero;

        /// <summary>
        /// True when what remains of the offset is inside both configured tolerances. Compared as
        /// magnitudes of the residual itself rather than via <see cref="PoseMath"/> on the composed
        /// poses: the offset <i>is</i> the error, so measuring it directly avoids re-deriving it from
        /// two poses and cannot disagree with what the decay is acting on.
        /// </summary>
        private bool OffsetIsWithinTolerance() =>
            _offsetPosition.Length() <= _positionTolerance &&
            _offsetRotation.Length() <= _orientationTolerance;

        /// <summary>
        /// The displayed pose: <paramref name="predicted"/> displaced by the residual offset.
        /// Returns <paramref name="predicted"/> bit-identically when the offset is exactly zero, so
        /// that the degenerate pass-through case reproduces its input rather than differing by
        /// quaternion renormalization noise -- the same reasoning
        /// <see cref="MotionMath.IntegrateWorld"/> gives for its own zero-rotation short circuit.
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
