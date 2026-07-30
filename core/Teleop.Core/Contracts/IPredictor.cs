using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Estimates state at a <b>future</b> target time from stale observations. Implementations
    /// live in <c>Prediction/</c>, one per file, registered by hand in
    /// <c>Registry/Registries.cs</c>.
    ///
    /// The same contract serves both directions of the loop, but they are different problems
    /// and are benchmarked separately: operator-side predicts where the robot is *now* from
    /// stale robot state; robot-side predicts what the operator wants *now* from stale
    /// commands. Human motion and robot dynamics have different statistics.
    ///
    /// Contract every implementation owes its callers:
    /// <list type="number">
    /// <item>Deterministic — the same observations and target times produce bit-identical
    /// output, every run. Randomness only through an injected seeded RNG.</item>
    /// <item>Robust to observations that arrive out of order, that duplicate a stamp already
    /// seen, and to gaps of several hundred milliseconds. Real traces contain all three, and a
    /// predictor that assumes monotonic arrival produces garbage silently instead of
    /// failing.</item>
    /// <item><see cref="Predict"/> allocates nothing. Preallocate in the constructor.</item>
    /// <item>Every parameter comes from <see cref="PredictorConfig"/>, supplied at
    /// construction. No magic numbers in the body.</item>
    /// </list>
    /// </summary>
    /// <typeparam name="TState">
    /// The predicted state, typically <see cref="Pose"/>. Use a value type: <c>Predict</c> is
    /// called every frame and must not allocate.
    /// </typeparam>
    public interface IPredictor<TState>
    {
        /// <summary>
        /// An authoritative sample arrived. May be called with a
        /// <see cref="Stamped{TState}.CaptureTicks"/> older than or equal to one already seen —
        /// the implementation decides whether to reinsert, ignore, or reject it, and reports
        /// what it did through <see cref="PredictorDiagnostics.RejectedObservations"/>.
        /// Calling this twice with an identical sample must leave the estimator in the same
        /// state as calling it once.
        /// </summary>
        void Observe(Stamped<TState> obs);

        /// <summary>
        /// Estimate of the state at <paramref name="targetTicks"/>, on the injected
        /// <c>ITimeAuthority</c> timebase. Pure with respect to internal state in the sense
        /// that two calls with the same target and no intervening <see cref="Observe"/> return
        /// identical values; it may update <see cref="Diagnostics"/>. With no observations yet,
        /// implementations return a documented, deterministic value rather than throwing.
        /// Allocation-free.
        /// </summary>
        TState Predict(long targetTicks);

        /// <summary>
        /// Returns the predictor to its as-constructed state: no observations, cleared
        /// derivatives and filter state, zeroed diagnostics counters. Configuration and
        /// preallocated buffers survive. Sweeps reuse instances across trials, so a trial that
        /// can see the previous trial's history is a silent cross-contamination bug.
        /// </summary>
        void Reset();

        /// <summary>
        /// Describes the most recent <see cref="Predict"/> call — at minimum the horizon
        /// actually extrapolated, plus an uncertainty estimate where the implementation has
        /// one. Read after <see cref="Predict"/>; before the first call it reports
        /// <see cref="PredictorDiagnostics.None"/>.
        /// </summary>
        PredictorDiagnostics Diagnostics { get; }
    }
}
