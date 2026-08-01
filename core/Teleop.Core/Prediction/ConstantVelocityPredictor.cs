using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// Registry key <c>const-vel</c>: first-order dead reckoning. Position advances on a straight
    /// line and orientation on a fixed axis at a fixed rate, both derived from the two newest
    /// retained observations. No acceleration term, so it lags through a direction reversal rather
    /// than overshooting it -- the opposite failure mode to <c>const-accel</c>, which is why both
    /// are worth running.
    ///
    /// <b>Fields of <see cref="PredictorConfig"/> it reads:</b>
    /// <see cref="PredictorConfig.MaxHorizonTicks"/>,
    /// <see cref="PredictorConfig.MaxObservationGapTicks"/>,
    /// <see cref="PredictorConfig.HistoryCapacity"/>,
    /// <see cref="PredictorConfig.MaxLinearSpeed"/>,
    /// <see cref="PredictorConfig.MaxAngularSpeed"/>.
    /// <b>Fields it ignores</b> (that struct's own convention is that each implementation says so):
    /// <see cref="PredictorConfig.SmoothingAlpha"/> and
    /// <see cref="PredictorConfig.SmoothingBeta"/> -- nothing here smooths, the rate is a raw
    /// two-sample difference; and <see cref="PredictorConfig.ProcessNoise"/> and
    /// <see cref="PredictorConfig.MeasurementNoise"/> -- there is no noise model and therefore no
    /// uncertainty to report (<see cref="PredictorDiagnostics.HasUncertainty"/> is always false).
    ///
    /// <b>The rate comes from the two newest retained samples only, never from the whole
    /// history.</b> This is a deliberate choice, not a shortcut. The history buffer exists so that
    /// a late sample can be reinserted in stamp order (see <see cref="Observe"/>), and reinserting
    /// one in the middle of the window must not retroactively change what the predictor is
    /// currently extrapolating -- with a whole-window fit it would, so the estimate driving
    /// <see cref="Predict"/> would depend on the arrival <i>order</i> of samples the pipeline has
    /// already moved past. Restricting the rate to the newest pair makes the current estimate a
    /// function of the newest pair alone: order-independent with respect to everything older, and
    /// therefore reproducible across replays that reorder the same trace differently. The older
    /// entries remain available for diagnostics and for the eviction bound.
    ///
    /// <b>Gap policy: collapse the rate to zero rather than difference across the gap.</b> When the
    /// two newest samples are more than <see cref="PredictorConfig.MaxObservationGapTicks"/> apart,
    /// the derived rate is treated as stale and set to zero, so <see cref="Predict"/> degenerates
    /// to <c>none</c>-style passthrough of the newest pose until a fresh pair arrives. Real traces
    /// contain gaps of several hundred milliseconds, and differencing a normal-sized position
    /// change across a long stall produces a plausible-looking but entirely fictitious velocity --
    /// "the classic silent-garbage failure mode this field exists to prevent", in
    /// <see cref="PredictorConfig.MaxObservationGapTicks"/>'s own words. Holding still through a
    /// gap is visibly wrong; flying off on a fabricated velocity looks like a working predictor.
    /// The collapse is transient: the next pair inside the bound restores a real rate.
    ///
    /// <b>World-frame rotation</b>, via <see cref="MotionMath"/>, matching
    /// <c>Plant/RigidBodyPlant.Step</c> exactly. Angular rate is an axis-angle rate vector
    /// (direction is the axis, magnitude is radians/second), the same convention as
    /// <see cref="CommandFrame.AngularVelocity"/>.
    ///
    /// Deterministic and allocation-free: the history array is sized once in the constructor and
    /// the insertion sort shifts entries within it. Not thread-safe, by contract.
    /// </summary>
    public sealed class ConstantVelocityPredictor : IPredictor<Pose>
    {
        /// <summary>
        /// Two samples are the minimum from which a first difference exists at all, so a capacity
        /// below that is a configuration error rather than a degenerate-but-valid setting.
        /// </summary>
        private const int MinimumHistoryCapacity = 2;

        private readonly long _maxHorizonTicks;
        private readonly long _maxObservationGapTicks;
        private readonly float _maxLinearSpeed;
        private readonly float _maxAngularSpeed;
        private readonly long _ticksPerSecond;

        /// <summary>
        /// Retained observations, ascending by <see cref="Stamped{T}.CaptureTicks"/>, oldest at
        /// index 0. Sized once at construction; <see cref="Observe"/> only shifts within it.
        /// </summary>
        private readonly Stamped<Pose>[] _history;

        private int _count;

        /// <summary>Estimated linear velocity, metres/second, from the two newest samples.</summary>
        private Vector3 _velocity;

        /// <summary>
        /// Estimated angular rate as an axis-angle rate vector, radians/second, from the two newest
        /// samples. Same convention as <see cref="CommandFrame.AngularVelocity"/>.
        /// </summary>
        private Vector3 _angularRate;

        private long _horizonTicks;
        private int _acceptedObservations;
        private int _rejectedObservations;

        /// <param name="config">
        /// Parameters; see the type doc for which fields are read and which are ignored.
        /// </param>
        /// <param name="clock">
        /// Read for <see cref="ITimeAuthority.TicksPerSecond"/> only, <b>never</b>
        /// <see cref="ITimeAuthority.NowTicks"/> -- the same "time is always a parameter"
        /// discipline <c>Pipeline/OperatorEndpoint</c> and <c>IRobotPlant</c> enforce. The
        /// conversion is needed because <see cref="PredictorConfig.MaxLinearSpeed"/> and
        /// <see cref="PredictorConfig.MaxAngularSpeed"/> are per <i>second</i> while every stamp in
        /// Core is in ticks, and nothing in Core may assume a tick rate.
        /// </param>
        public ConstantVelocityPredictor(PredictorConfig config, ITimeAuthority clock)
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

            if (config.HistoryCapacity < MinimumHistoryCapacity)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(config), config.HistoryCapacity,
                    "HistoryCapacity must be at least 2: a first-order rate needs two samples.");
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
                    "MaxObservationGapTicks must be positive; a non-positive bound would treat every " +
                    "sample pair as stale and silently reduce this predictor to passthrough.");
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

            _maxHorizonTicks = config.MaxHorizonTicks;
            _maxObservationGapTicks = config.MaxObservationGapTicks;
            _maxLinearSpeed = config.MaxLinearSpeed;
            _maxAngularSpeed = config.MaxAngularSpeed;
            _ticksPerSecond = clock.TicksPerSecond;

            _history = new Stamped<Pose>[config.HistoryCapacity];
            _count = 0;
            _velocity = Vector3.Zero;
            _angularRate = Vector3.Zero;
            _horizonTicks = 0;
            _acceptedObservations = 0;
            _rejectedObservations = 0;
        }

        /// <summary>
        /// Inserts <paramref name="obs"/> into the history in capture-stamp order, then recomputes
        /// the rate from the two newest entries.
        ///
        /// <b>Acceptance window.</b> A sample is accepted when its stamp is strictly newer than the
        /// <i>oldest</i> retained sample and does not duplicate a retained stamp. This is the
        /// clause <see cref="PredictorConfig.HistoryCapacity"/> documents as "how far back an
        /// out-of-order sample can be reinserted before it is rejected": the buffer depth <i>is</i>
        /// the reordering tolerance, so a deeper buffer accepts later samples. Rejected are
        /// <list type="bullet">
        /// <item>samples at or before the oldest retained stamp -- out of window, and reinserting
        /// one would silently claim a reordering tolerance the buffer does not have; and</item>
        /// <item>samples whose stamp exactly matches a retained one -- duplicates, which
        /// <see cref="IPredictor{TState}.Observe"/> requires to leave state unchanged.</item>
        /// </list>
        /// Both are counted in <see cref="PredictorDiagnostics.RejectedObservations"/> and change
        /// nothing else.
        ///
        /// When the buffer is full the oldest entry is evicted to make room, which narrows the
        /// acceptance window from below -- an accepted mid-window insertion therefore also reduces
        /// how far back the <i>next</i> late sample may reach.
        ///
        /// Allocation-free: the insertion sort shifts entries inside the preallocated array.
        /// </summary>
        public void Observe(Stamped<Pose> obs)
        {
            if (!IsWithinAcceptanceWindow(obs.CaptureTicks))
            {
                _rejectedObservations++;
                return;
            }

            if (_count == _history.Length)
            {
                EvictOldest();
            }

            InsertSorted(obs);
            _acceptedObservations++;
            RecomputeRateFromNewestPair();
        }

        /// <summary>
        /// Newest retained pose advanced by the estimated rate over the horizon.
        ///
        /// The horizon is <c>targetTicks - newestCaptureTicks</c>, clamped down to
        /// <see cref="PredictorConfig.MaxHorizonTicks"/> on the <b>future side only</b>. A target
        /// in the past relative to the newest observation yields a negative horizon and is left
        /// alone: that is interpolation, which
        /// <see cref="PredictorDiagnostics.HorizonTicks"/> explicitly documents as a legal negative
        /// value, and clamping it would be clamping the wrong direction. The post-clamp horizon is
        /// what <see cref="Diagnostics"/> reports, because it is what was actually extrapolated.
        ///
        /// Both rates are clamped to <see cref="PredictorConfig.MaxLinearSpeed"/> and
        /// <see cref="PredictorConfig.MaxAngularSpeed"/> <b>before</b> being multiplied by the
        /// horizon, not after: bounding the resulting displacement instead would still let an
        /// implausible two-sample spike set the direction of a full-magnitude jump, and would make
        /// the bound depend on the horizon.
        ///
        /// With no observations it returns <see cref="Pose.Identity"/> -- not
        /// <c>default(Pose)</c>, whose all-zero quaternion is not a rotation and turns every
        /// downstream angle into NaN. Allocation-free, and pure with respect to the returned value:
        /// only <see cref="Diagnostics"/> changes.
        /// </summary>
        public Pose Predict(long targetTicks)
        {
            if (_count == 0)
            {
                _horizonTicks = 0;
                return Pose.Identity;
            }

            Stamped<Pose> newest = _history[_count - 1];

            long horizonTicks = targetTicks - newest.CaptureTicks;
            if (horizonTicks > _maxHorizonTicks)
            {
                horizonTicks = _maxHorizonTicks;
            }

            _horizonTicks = horizonTicks;

            // A tick *delta*, not an absolute tick count, so float carries it exactly at any
            // horizon a sweep realistically asks for.
            float dt = (float)horizonTicks / _ticksPerSecond;

            Vector3 velocity = MotionMath.ClampMagnitude(_velocity, _maxLinearSpeed);
            Vector3 angularRate = MotionMath.ClampMagnitude(_angularRate, _maxAngularSpeed);

            Vector3 position = newest.Value.Position + velocity * dt;
            Quaternion rotation = MotionMath.IntegrateWorld(newest.Value.Rotation, angularRate * dt);

            return new Pose(position, rotation);
        }

        /// <summary>
        /// Horizon actually extrapolated by the last <see cref="Predict"/> (post-clamp), the newest
        /// retained stamp, and the running accepted/rejected counts. No uncertainty: this predictor
        /// has no noise model, so <see cref="PredictorDiagnostics.HasUncertainty"/> is always
        /// false and consumers must degrade to their uncertainty-free behaviour.
        ///
        /// Every field is zero on a freshly constructed or freshly <see cref="Reset"/> instance,
        /// i.e. exactly <see cref="PredictorDiagnostics.None"/>.
        /// </summary>
        public PredictorDiagnostics Diagnostics => new PredictorDiagnostics(
            _horizonTicks,
            _count > 0 ? _history[_count - 1].CaptureTicks : 0,
            _acceptedObservations,
            _rejectedObservations);

        /// <summary>
        /// Returns the predictor to its as-constructed state: empty history, zero rates, zeroed
        /// horizon and counters. The array itself survives -- capacity is configuration, not state.
        /// Entries above <see cref="_count"/> are never read, so they are left in place rather than
        /// cleared; sweeps reuse instances across trials and clearing the array would be work with
        /// no observable effect.
        /// </summary>
        public void Reset()
        {
            _count = 0;
            _velocity = Vector3.Zero;
            _angularRate = Vector3.Zero;
            _horizonTicks = 0;
            _acceptedObservations = 0;
            _rejectedObservations = 0;
        }

        /// <summary>
        /// True when <paramref name="captureTicks"/> is newer than the oldest retained sample and
        /// duplicates no retained stamp. The duplicate scan is linear over the retained count,
        /// which is bounded by <see cref="PredictorConfig.HistoryCapacity"/> and small; a binary
        /// search would be the same cost at these sizes and harder to read.
        /// </summary>
        private bool IsWithinAcceptanceWindow(long captureTicks)
        {
            if (_count == 0)
            {
                return true;
            }

            if (captureTicks <= _history[0].CaptureTicks)
            {
                return false;
            }

            for (int i = 1; i < _count; i++)
            {
                if (_history[i].CaptureTicks == captureTicks)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>Drops the oldest entry, shifting the rest down one slot.</summary>
        private void EvictOldest()
        {
            for (int i = 0; i < _count - 1; i++)
            {
                _history[i] = _history[i + 1];
            }

            _count--;
        }

        /// <summary>
        /// Places <paramref name="obs"/> at its stamp-ordered position, shifting later entries up.
        /// Caller guarantees there is room and that the stamp is unique and in window.
        /// </summary>
        private void InsertSorted(Stamped<Pose> obs)
        {
            int index = _count;
            while (index > 0 && _history[index - 1].CaptureTicks > obs.CaptureTicks)
            {
                _history[index] = _history[index - 1];
                index--;
            }

            _history[index] = obs;
            _count++;
        }

        /// <summary>
        /// Recomputes <see cref="_velocity"/> and <see cref="_angularRate"/> from the two newest
        /// retained samples, applying the gap policy described on the type. With fewer than two
        /// samples there is no difference to take and both are zero.
        /// </summary>
        private void RecomputeRateFromNewestPair()
        {
            if (_count < 2)
            {
                _velocity = Vector3.Zero;
                _angularRate = Vector3.Zero;
                return;
            }

            Stamped<Pose> previous = _history[_count - 2];
            Stamped<Pose> newest = _history[_count - 1];

            long gapTicks = newest.CaptureTicks - previous.CaptureTicks;
            if (gapTicks > _maxObservationGapTicks)
            {
                _velocity = Vector3.Zero;
                _angularRate = Vector3.Zero;
                return;
            }

            // Strictly positive: the history is stamp-ordered and stamps are unique by construction.
            float dt = (float)gapTicks / _ticksPerSecond;

            _velocity = (newest.Value.Position - previous.Value.Position) / dt;
            _angularRate =
                MotionMath.RelativeRotationVector(previous.Value.Rotation, newest.Value.Rotation) / dt;
        }
    }
}
