using System;
using System.Numerics;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Reconciliation
{
    /// <summary>
    /// Third derivative of <b>displayed</b> position, in mm/s³ (docs/metrics.md §5) — the nausea
    /// proxy every reconciler owes its callers via <c>IReconciler{TState}</c>'s correction-cost
    /// clause.
    ///
    /// It lives in one place, shared by every reconciler, for the same reason
    /// <see cref="Types.PoseMath"/> and <see cref="Types.MotionMath"/> do: <c>jerk_mm_s3</c> is the
    /// metric reconcilers are <i>compared on</i>, so two implementations computing it from two
    /// copies of this cascade would usually agree and would differ in the corners (unequal frame
    /// spacing, the first four samples of a trial), which is precisely where a correction-cost
    /// figure stops being interpretable. A per-reconciler copy would make
    /// <c>exp-smooth vs snap vs spring</c> a comparison of jerk estimators as much as of
    /// reconcilers.
    ///
    /// Stateful, hence a class rather than a static helper: a third derivative needs four points,
    /// so three displayed positions are retained between calls. Does not implement
    /// <c>IReconciler{TState}</c> and therefore is not a <c>Registry/Registries.cs</c> axis —
    /// <c>Teleop.Eval -- audit</c>'s registry-completeness check works by reflecting for
    /// implementors of the contract interfaces, so a helper is correctly invisible to it.
    ///
    /// Deterministic and allocation-free: both buffers are sized once in the constructor. Not
    /// thread-safe, matching every reconciler that owns one.
    /// </summary>
    internal sealed class DisplayedJerkEstimator
    {
        /// <summary>
        /// Core works in metres (ROS convention); docs/metrics.md reports millimetres. The
        /// conversion happens at the reporting boundary, which is here.
        /// </summary>
        private const double MetresToMillimetres = 1000.0;

        /// <summary>
        /// Samples of displayed position retained. Three, because a third derivative needs four
        /// points: these three plus the one being produced. Not a research knob — it is the arity
        /// of the finite difference.
        /// </summary>
        private const int HistoryLength = 3;

        private readonly Vector3[] _positions;
        private readonly long[] _ticks;
        private readonly long _ticksPerSecond;
        private int _count;

        /// <param name="ticksPerSecond">
        /// From the owning reconciler's injected <c>ITimeAuthority.TicksPerSecond</c>, read for
        /// unit conversion only. Must be positive; the caller validates that.
        /// </param>
        public DisplayedJerkEstimator(long ticksPerSecond)
        {
            _positions = new Vector3[HistoryLength];
            _ticks = new long[HistoryLength];
            _ticksPerSecond = ticksPerSecond;
            _count = 0;
        }

        /// <summary>
        /// Jerk at <paramref name="nowTicks"/> from the three retained samples plus the one about
        /// to be displayed. Returns false when fewer than three are retained: before that a third
        /// derivative would be an invented number, and inventing one at the start of every trial
        /// would bias the metric exactly where the trial is least representative.
        ///
        /// Computed as a cascade of central differences on <b>unequally spaced</b> samples, which
        /// is what a frame schedule actually produces: three velocities at the midpoints of the
        /// three intervals, two accelerations at the midpoints of those, and one jerk from the two
        /// accelerations. Every division is by a strictly positive interval, because callers only
        /// <see cref="Push"/> on strictly increasing ticks.
        ///
        /// Accumulated in <c>double</c>, per <c>IMetricSink.Record</c>'s note that the parameter is
        /// <c>double</c> "so tick differences and jerk values survive without precision loss": a
        /// snap divides a metre-scale step by a millisecond-scale interval three times, and the
        /// intermediate magnitudes leave little of a float's mantissa.
        ///
        /// Call this <b>before</b> <see cref="Push"/> for the same frame — it treats
        /// <paramref name="position"/> as the not-yet-retained fourth point.
        /// </summary>
        public bool TryCompute(
            in Vector3 position, long nowTicks, out double jerkMillimetresPerSecondCubed)
        {
            if (_count < HistoryLength)
            {
                jerkMillimetresPerSecondCubed = 0.0;
                return false;
            }

            // Seconds relative to the oldest retained sample. Relative, so the absolute tick
            // magnitude never enters the arithmetic.
            long originTicks = _ticks[0];
            double t0 = 0.0;
            double t1 = (_ticks[1] - originTicks) / (double)_ticksPerSecond;
            double t2 = (_ticks[2] - originTicks) / (double)_ticksPerSecond;
            double t3 = (nowTicks - originTicks) / (double)_ticksPerSecond;

            Vector3 p0 = _positions[0];
            Vector3 p1 = _positions[1];
            Vector3 p2 = _positions[2];

            double jerkX = Jerk1D(p0.X, p1.X, p2.X, position.X, t0, t1, t2, t3);
            double jerkY = Jerk1D(p0.Y, p1.Y, p2.Y, position.Y, t0, t1, t2, t3);
            double jerkZ = Jerk1D(p0.Z, p1.Z, p2.Z, position.Z, t0, t1, t2, t3);

            double magnitudeMetresPerSecondCubed =
                Math.Sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

            jerkMillimetresPerSecondCubed = magnitudeMetresPerSecondCubed * MetresToMillimetres;
            return true;
        }

        /// <summary>
        /// Appends a displayed position, dropping the oldest when full. A shift over three elements
        /// rather than a ring index: at this length it is cheaper than the modulo bookkeeping and
        /// leaves the array in oldest-first order, which is what the finite difference wants.
        /// </summary>
        public void Push(in Vector3 position, long nowTicks)
        {
            if (_count < HistoryLength)
            {
                _positions[_count] = position;
                _ticks[_count] = nowTicks;
                _count++;
                return;
            }

            for (int i = 0; i < HistoryLength - 1; i++)
            {
                _positions[i] = _positions[i + 1];
                _ticks[i] = _ticks[i + 1];
            }

            _positions[HistoryLength - 1] = position;
            _ticks[HistoryLength - 1] = nowTicks;
        }

        /// <summary>
        /// Empties the history, so the next three frames rebuild it and no jerk is reported until
        /// four real samples exist again. Called from the owning reconciler's <c>Reset()</c>;
        /// carrying history across a sweep's trial boundary would compute one jerk from two
        /// unrelated trajectories.
        /// </summary>
        public void Reset()
        {
            _count = 0;
        }

        /// <summary>
        /// One axis of the unequally-spaced third derivative described on
        /// <see cref="TryCompute"/>. Positions in metres, times in seconds, result in
        /// metres/second³.
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
