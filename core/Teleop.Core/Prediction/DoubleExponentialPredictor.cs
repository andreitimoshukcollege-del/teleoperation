using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// Registry key <c>double-exp</c>: Holt's linear-trend double-exponential smoothing, over both
    /// position and orientation. Two parameters, no covariance, no matrix -- "Kalman-free, two
    /// parameters, strong baseline for head/hand pose", per <c>Prediction/CLAUDE.md</c>. It buys
    /// noise rejection that <c>const-vel</c>'s raw two-sample difference does not have, and pays
    /// for it with lag: the trend estimate is a filtered quantity and needs several samples to
    /// catch a change in motion. That lag is the documented tradeoff being measured, not a defect.
    ///
    /// <b>Fields of <see cref="PredictorConfig"/> it reads:</b>
    /// <see cref="PredictorConfig.SmoothingAlpha"/>, <see cref="PredictorConfig.SmoothingBeta"/>,
    /// <see cref="PredictorConfig.MaxHorizonTicks"/>,
    /// <see cref="PredictorConfig.MaxObservationGapTicks"/>,
    /// <see cref="PredictorConfig.MaxLinearSpeed"/>, <see cref="PredictorConfig.MaxAngularSpeed"/>.
    /// <b>Fields it ignores:</b> <see cref="PredictorConfig.HistoryCapacity"/> -- a purely
    /// recursive filter keeps no window, so there is no buffer to size and nothing to reinsert
    /// into; and <see cref="PredictorConfig.ProcessNoise"/> /
    /// <see cref="PredictorConfig.MeasurementNoise"/> -- there is no noise model, so
    /// <see cref="PredictorDiagnostics.HasUncertainty"/> is always false.
    ///
    /// <b>Generalized to irregular sampling, deliberately.</b> Textbook Holt's assumes a fixed step
    /// and carries the trend as a per-sample delta. Real observations arrive jittered, and with a
    /// per-sample trend <see cref="PredictorConfig.SmoothingBeta"/> would mean something different
    /// on every interval -- the same parameter would describe a different filter depending on how
    /// far apart two packets happened to land, so a swept beta would not be comparable across
    /// network profiles and the same trace replayed with different jitter would not reproduce. The
    /// trend here is therefore kept as a <b>rate per second</b>, and the elapsed time enters the
    /// recursion explicitly:
    /// <code>
    /// L_k = alpha * z_k + (1 - alpha) * (L_(k-1) + T_(k-1) * dt)
    /// T_k = beta * (L_k - L_(k-1)) / dt + (1 - beta) * T_(k-1)
    /// </code>
    /// with <c>dt</c> in seconds since the last accepted observation. A constant-velocity signal is
    /// an exact fixed point of this recursion (<c>L = z</c>, <c>T = v</c>), so the filter has no
    /// steady-state lag on constant velocity -- only a transient one.
    ///
    /// <b>Rotation follows the identical recursion through the log/exp map</b>
    /// (<see cref="MotionMath"/>): the level is a smoothed orientation, the trend is an angular
    /// rate vector in radians/second, and each difference that would be a subtraction in position
    /// is a world-frame relative rotation instead. <see cref="MotionMath.RotationEpsilon"/> is the
    /// small-angle guard -- a near-identity relative rotation has no defined axis, and dividing one
    /// by a small <c>dt</c> without the guard is where a rotation predictor produces NaN.
    ///
    /// <b>Ordering policy: strictly increasing stamps only, no reinsertion.</b> Unlike
    /// <c>const-vel</c>, which keeps a window and can splice a late sample into it, this is a
    /// sequential recursive filter: its entire state is the result of folding observations in
    /// order, and there is no history to re-fold. A late sample is therefore rejected outright
    /// rather than applied out of sequence, which would corrupt the level and trend with a
    /// backwards <c>dt</c>. That is a real behavioural difference between the two predictors on a
    /// reordering trace and it is visible in
    /// <see cref="PredictorDiagnostics.RejectedObservations"/>, which is where it should be visible.
    ///
    /// <b>Gap policy: re-seed, do not smooth across the gap.</b> Past
    /// <see cref="PredictorConfig.MaxObservationGapTicks"/> the filter restarts from the arriving
    /// sample -- level to the observation, trend to zero. Same intent as <c>const-vel</c>'s
    /// rate collapse, and deliberately the same threshold field so the family behaves consistently,
    /// but a full restart rather than a rate reset: this filter has no retained history to fall
    /// back on, and blending a pre-gap level into a post-gap observation would leave the level
    /// stranded between two positions the operator was never at. The cost is that the post-gap
    /// trend has to be re-learned, which is honest -- nothing is known about motion across a stall.
    ///
    /// Deterministic and allocation-free; there is nothing to preallocate, since the whole state is
    /// four value-typed fields. Not thread-safe, by contract.
    /// </summary>
    public sealed class DoubleExponentialPredictor : IPredictor<Pose>
    {
        private readonly float _alpha;
        private readonly float _beta;
        private readonly long _maxHorizonTicks;
        private readonly long _maxObservationGapTicks;
        private readonly float _maxLinearSpeed;
        private readonly float _maxAngularSpeed;
        private readonly long _ticksPerSecond;

        /// <summary>Smoothed position (the level), metres.</summary>
        private Vector3 _level;

        /// <summary>
        /// Smoothed linear trend as a rate, metres/<b>second</b> -- not a per-sample delta. See the
        /// type doc for why the distinction is load-bearing.
        /// </summary>
        private Vector3 _trend;

        /// <summary>Smoothed orientation (the rotational level), unit quaternion.</summary>
        private Quaternion _rotationLevel;

        /// <summary>
        /// Smoothed angular trend as an axis-angle rate vector, radians/second -- the rotational
        /// counterpart of <see cref="_trend"/>, same convention as
        /// <see cref="CommandFrame.AngularVelocity"/>.
        /// </summary>
        private Vector3 _rotationTrend;

        private bool _hasState;
        private long _lastAcceptedTicks;

        private long _horizonTicks;
        private int _acceptedObservations;
        private int _rejectedObservations;

        /// <param name="config">
        /// Parameters; see the type doc for which fields are read and which are ignored.
        /// </param>
        /// <param name="clock">
        /// Read for <see cref="ITimeAuthority.TicksPerSecond"/> only, <b>never</b>
        /// <see cref="ITimeAuthority.NowTicks"/>. Needed because the trend is a per-second rate and
        /// every stamp in Core is in ticks, and nothing in Core may assume a tick rate.
        /// </param>
        /// <exception cref="ArgumentOutOfRangeException">
        /// If either smoothing factor is outside <c>[0, 1]</c> (NaN included, since the comparison
        /// is written to reject it). Failing in the constructor mirrors
        /// <c>Plant/RigidBodyPlant</c>'s precedent: an alpha of 1.5 does not throw at use time, it
        /// quietly produces an overshooting divergent filter for the entire life of the instance,
        /// and a whole sweep would be scored against it before anyone noticed.
        /// </exception>
        public DoubleExponentialPredictor(PredictorConfig config, ITimeAuthority clock)
        {
            if (clock == null)
            {
                throw new ArgumentNullException(nameof(clock));
            }

            if (clock.TicksPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(clock), clock.TicksPerSecond, "Ticks per second must be positive.");
            }

            if (!(config.SmoothingAlpha >= 0f && config.SmoothingAlpha <= 1f))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.SmoothingAlpha,
                    "SmoothingAlpha must be in [0, 1]; outside that range the level recursion diverges.");
            }

            if (!(config.SmoothingBeta >= 0f && config.SmoothingBeta <= 1f))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.SmoothingBeta,
                    "SmoothingBeta must be in [0, 1]; outside that range the trend recursion diverges.");
            }

            if (config.MaxHorizonTicks < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxHorizonTicks,
                    "MaxHorizonTicks must be non-negative; a negative cap would silently turn every " +
                    "forward prediction into a backwards one.");
            }

            if (config.MaxObservationGapTicks <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxObservationGapTicks,
                    "MaxObservationGapTicks must be positive; a non-positive bound would re-seed the " +
                    "filter on every observation and silently reduce it to passthrough.");
            }

            if (config.MaxLinearSpeed < 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxLinearSpeed, "MaxLinearSpeed must be non-negative.");
            }

            if (config.MaxAngularSpeed < 0f)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.MaxAngularSpeed, "MaxAngularSpeed must be non-negative.");
            }

            _alpha = config.SmoothingAlpha;
            _beta = config.SmoothingBeta;
            _maxHorizonTicks = config.MaxHorizonTicks;
            _maxObservationGapTicks = config.MaxObservationGapTicks;
            _maxLinearSpeed = config.MaxLinearSpeed;
            _maxAngularSpeed = config.MaxAngularSpeed;
            _ticksPerSecond = clock.TicksPerSecond;

            ClearFilterState();
        }

        /// <summary>
        /// Folds <paramref name="obs"/> into the level and trend, or re-seeds from it across a gap.
        ///
        /// A stamp that is not strictly greater than the last accepted one is rejected whole and
        /// counted in <see cref="PredictorDiagnostics.RejectedObservations"/> -- duplicates
        /// included, as <see cref="IPredictor{TState}.Observe"/> requires. See the type doc for why
        /// this filter rejects rather than reinserts.
        ///
        /// The first observation after construction, after <see cref="Reset"/>, or after a gap
        /// longer than <see cref="PredictorConfig.MaxObservationGapTicks"/> seeds the filter:
        /// level to the observed pose, trend to zero. Allocation-free.
        /// </summary>
        public void Observe(Stamped<Pose> obs)
        {
            if (_hasState && obs.CaptureTicks <= _lastAcceptedTicks)
            {
                _rejectedObservations++;
                return;
            }

            if (!_hasState || obs.CaptureTicks - _lastAcceptedTicks > _maxObservationGapTicks)
            {
                Seed(obs);
                return;
            }

            // Strictly positive: the stamp is strictly greater than the last accepted one.
            float dt = (float)(obs.CaptureTicks - _lastAcceptedTicks) / _ticksPerSecond;

            Vector3 previousLevel = _level;
            Vector3 projectedLevel = previousLevel + _trend * dt;
            Vector3 newLevel = _alpha * obs.Value.Position + (1f - _alpha) * projectedLevel;
            _trend = _beta * ((newLevel - previousLevel) / dt) + (1f - _beta) * _trend;
            _level = newLevel;

            Quaternion previousRotationLevel = _rotationLevel;
            Quaternion projectedRotation =
                MotionMath.IntegrateWorld(previousRotationLevel, _rotationTrend * dt);

            // The rotational analogue of "alpha of the way from the projection to the observation":
            // step alpha along the geodesic between them, in the world frame.
            Vector3 innovation =
                MotionMath.RelativeRotationVector(projectedRotation, obs.Value.Rotation);
            Quaternion newRotationLevel =
                MotionMath.IntegrateWorld(projectedRotation, innovation * _alpha);

            Vector3 rotationLevelRate =
                MotionMath.RelativeRotationVector(previousRotationLevel, newRotationLevel) / dt;
            _rotationTrend = _beta * rotationLevelRate + (1f - _beta) * _rotationTrend;
            _rotationLevel = newRotationLevel;

            _lastAcceptedTicks = obs.CaptureTicks;
            _acceptedObservations++;
        }

        /// <summary>
        /// Level advanced by the trend over the horizon.
        ///
        /// Horizon and clamping follow the same rules as <c>const-vel</c>, for the same reasons and
        /// so that the two are comparable at a given horizon: <c>targetTicks - lastAcceptedTicks</c>
        /// clamped down to <see cref="PredictorConfig.MaxHorizonTicks"/> on the future side only (a
        /// negative horizon is interpolation and is left alone, per
        /// <see cref="PredictorDiagnostics.HorizonTicks"/>), and both trends clamped to
        /// <see cref="PredictorConfig.MaxLinearSpeed"/> /
        /// <see cref="PredictorConfig.MaxAngularSpeed"/> before they are multiplied by the horizon
        /// rather than after.
        ///
        /// With no observations it returns <see cref="Pose.Identity"/> -- not
        /// <c>default(Pose)</c>, whose all-zero quaternion is not a rotation and turns every
        /// downstream angle into NaN. Allocation-free.
        /// </summary>
        public Pose Predict(long targetTicks)
        {
            if (!_hasState)
            {
                _horizonTicks = 0;
                return Pose.Identity;
            }

            long horizonTicks = targetTicks - _lastAcceptedTicks;
            if (horizonTicks > _maxHorizonTicks)
            {
                horizonTicks = _maxHorizonTicks;
            }

            _horizonTicks = horizonTicks;

            // A tick *delta*, not an absolute tick count, so float carries it exactly at any
            // horizon a sweep realistically asks for.
            float dt = (float)horizonTicks / _ticksPerSecond;

            Vector3 trend = MotionMath.ClampMagnitude(_trend, _maxLinearSpeed);
            Vector3 rotationTrend = MotionMath.ClampMagnitude(_rotationTrend, _maxAngularSpeed);

            Vector3 position = _level + trend * dt;
            Quaternion rotation = MotionMath.IntegrateWorld(_rotationLevel, rotationTrend * dt);

            return new Pose(position, rotation);
        }

        /// <summary>
        /// Horizon actually extrapolated by the last <see cref="Predict"/> (post-clamp), the last
        /// accepted stamp, and the running accepted/rejected counts. No uncertainty: smoothing
        /// factors are not a noise model, so <see cref="PredictorDiagnostics.HasUncertainty"/> is
        /// always false rather than reporting a sigma that would be an invented number.
        ///
        /// Every field is zero on a freshly constructed or freshly <see cref="Reset"/> instance,
        /// i.e. exactly <see cref="PredictorDiagnostics.None"/>.
        /// </summary>
        public PredictorDiagnostics Diagnostics => new PredictorDiagnostics(
            _horizonTicks,
            _hasState ? _lastAcceptedTicks : 0,
            _acceptedObservations,
            _rejectedObservations);

        /// <summary>
        /// Returns the predictor to its as-constructed state: no filter state, zeroed horizon and
        /// counters. Configuration survives. Clearing <see cref="_hasState"/> is what makes the next
        /// trial's first observation re-seed rather than be rejected as stale against the previous
        /// trial's stamps -- the same reset <c>Plant/RigidBodyPlant.Reset</c> makes to its own
        /// staleness baseline, and the same silent cross-contamination bug if it is missed.
        /// </summary>
        public void Reset()
        {
            ClearFilterState();
            _horizonTicks = 0;
            _acceptedObservations = 0;
            _rejectedObservations = 0;
        }

        /// <summary>
        /// Starts the filter over from <paramref name="obs"/>: level at the observation, trend at
        /// zero. Counted as accepted -- the sample was used, and it is the only sample the estimate
        /// now rests on.
        /// </summary>
        private void Seed(Stamped<Pose> obs)
        {
            _level = obs.Value.Position;
            _trend = Vector3.Zero;
            _rotationLevel = obs.Value.Rotation;
            _rotationTrend = Vector3.Zero;
            _hasState = true;
            _lastAcceptedTicks = obs.CaptureTicks;
            _acceptedObservations++;
        }

        /// <summary>
        /// The as-constructed filter state. The rotational level is
        /// <see cref="Quaternion.Identity"/> rather than <c>default</c> for the same reason
        /// <see cref="Predict"/> returns <see cref="Pose.Identity"/>: the all-zero quaternion is not
        /// a rotation.
        /// </summary>
        private void ClearFilterState()
        {
            _level = Vector3.Zero;
            _trend = Vector3.Zero;
            _rotationLevel = Quaternion.Identity;
            _rotationTrend = Vector3.Zero;
            _hasState = false;
            _lastAcceptedTicks = 0;
        }
    }
}
