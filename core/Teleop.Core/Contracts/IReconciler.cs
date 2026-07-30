using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Decides how the visible state gets from the prediction to the truth when an
    /// authoritative sample arrives and disagrees with what was predicted. Implementations live
    /// in <c>Reconciliation/</c>.
    ///
    /// This axis, not the predictor, decides whether the system is usable in VR: a hard snap on
    /// correction is nausea no matter how good the prediction was. It is the counterweight to
    /// prediction aggressiveness and is always reported alongside prediction error.
    ///
    /// Contract every implementation owes its callers:
    /// <list type="number">
    /// <item><b>Bounded convergence.</b> Under a constant correction the visible error reaches
    /// zero, or a stated bound, within a bounded time — provable by test. A reconciler that can
    /// lag indefinitely is a bug, not a tradeoff.</item>
    /// <item><b>C1-continuous output.</b> No position or velocity discontinuity in the returned
    /// state, including on the step where a correction begins and the step where it
    /// completes.</item>
    /// <item><b>Correction cost every step</b>, pushed to the <c>IMetricSink</c> injected at
    /// construction: correction magnitude, corrections per second, peak jerk,
    /// time-to-convergence (docs/metrics.md §5). The sink is a constructor dependency rather
    /// than a parameter here so that the per-frame signature stays allocation-free.</item>
    /// <item>Deterministic and allocation-free, as everywhere in Core.</item>
    /// </list>
    /// </summary>
    /// <typeparam name="TState">
    /// The reconciled state, typically <see cref="Pose"/>. Use a value type; every method here
    /// is on the per-frame path.
    /// </typeparam>
    public interface IReconciler<TState>
    {
        /// <summary>
        /// Truth arrived and it disagrees with what was predicted. The reconciler records the
        /// resulting error and begins, retargets, or extends a correction; it does not change
        /// the visible output here — that happens in <see cref="Reconcile"/>, on the frame
        /// clock, which is what keeps the output C1-continuous regardless of when samples land.
        ///
        /// <paramref name="predictedAtCapture"/> is what the predictor claimed for
        /// <c>authoritative.CaptureTicks</c>, so that the error is measured at a single instant
        /// rather than across the transport delay.
        ///
        /// May be called with out-of-order or duplicate samples, or not at all for many frames.
        /// A duplicate must not be counted as a second correction; correction rate is a
        /// reported metric and double-counting corrupts it.
        /// </summary>
        /// <param name="authoritative">The sample that arrived, with its capture stamp.</param>
        /// <param name="predictedAtCapture">
        /// The prediction for that same capture time, i.e. the state the operator was actually
        /// shown for that instant.
        /// </param>
        /// <param name="diagnostics">
        /// The predictor's diagnostics for that prediction. Implementations that scale the
        /// correction by predictor uncertainty must check
        /// <see cref="PredictorDiagnostics.HasUncertainty"/> and degrade gracefully to their
        /// uncertainty-free behaviour when it is false — most predictors supply none. Pass
        /// <see cref="PredictorDiagnostics.None"/> when there is no predictor in the loop.
        /// </param>
        void Observe(
            Stamped<TState> authoritative,
            TState predictedAtCapture,
            in PredictorDiagnostics diagnostics);

        /// <summary>
        /// The per-frame call: given the predictor's estimate for this frame, return the state
        /// to display or actuate, advancing any in-flight correction to
        /// <paramref name="nowTicks"/>. Called every frame whether or not a correction is
        /// pending; with none pending it returns <paramref name="predicted"/> unmodified.
        ///
        /// Time is a parameter rather than a clock read so that replay is deterministic. Two
        /// calls with the same <paramref name="nowTicks"/> must return the same state and must
        /// not advance the correction twice. Allocation-free.
        /// </summary>
        TState Reconcile(TState predicted, long nowTicks);

        /// <summary>
        /// True when no correction is in flight: the visible state is within the
        /// implementation's stated tolerance of the last authoritative sample. This is what the
        /// bounded-convergence test asserts on and where time-to-convergence stops counting.
        /// True on a freshly constructed or freshly <see cref="Reset"/> instance.
        /// </summary>
        bool IsConverged { get; }

        /// <summary>
        /// Returns the reconciler to its as-constructed state: no correction in flight, no
        /// residual offset, cleared correction-cost accumulators. Configuration and the metric
        /// sink survive. Sweeps reuse instances across trials.
        /// </summary>
        void Reset();
    }
}
