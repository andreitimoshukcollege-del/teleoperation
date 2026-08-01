// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// The complete parameter set of a reconciler, matching <see cref="PredictorConfig"/>'s
    /// rationale: one shared block across all reconcilers, each implementation reads the subset
    /// that applies to it and documents which fields it ignores. There is deliberately no
    /// <c>Default</c> — see <see cref="PredictorConfig"/> for why.
    ///
    /// Units: metres, radians, seconds for physical quantities (ROS convention, matching
    /// <see cref="Pose"/>); ticks on the injected <c>ITimeAuthority</c> timebase for durations.
    /// </summary>
    public readonly struct ReconcilerConfig
    {
        /// <summary>
        /// Positional distance, metres, within which the visible state is considered converged
        /// to authoritative truth. Used by every reconciler — this is what makes
        /// <c>IsConverged</c> well-defined against floating-point truth rather than requiring
        /// exact equality.
        /// </summary>
        public readonly float ConvergencePositionToleranceMeters;

        /// <summary>
        /// Geodesic angle, radians, within which orientation is considered converged. Used by
        /// every reconciler, alongside <see cref="ConvergencePositionToleranceMeters"/>.
        /// </summary>
        public readonly float ConvergenceOrientationToleranceRadians;

        /// <summary>
        /// Upper bound on ticks a correction may take to converge, per <c>IReconciler</c>'s
        /// bounded-convergence contract. Ignored by <c>snap</c>, whose bound is always the
        /// tightest possible — one <c>Reconcile</c> call — regardless of this field. Used by
        /// implementations that spread a correction over multiple frames (<c>spring</c>,
        /// <c>budget-blend</c>, <c>velocity-match</c>).
        /// </summary>
        public readonly long MaxTimeToConvergenceTicks;

        /// <summary>
        /// Rate cap on a smoothed correction's linear speed, metres/second. Ignored by
        /// <c>snap</c> by definition — a snap has no rate limit, which is the entire reason it
        /// is the "ugly" baseline rather than a candidate mitigation.
        /// </summary>
        public readonly float MaxCorrectionLinearSpeedMetersPerSecond;

        /// <summary>
        /// Rate cap on a smoothed correction's angular speed, radians/second. Ignored by
        /// <c>snap</c>, for the same reason as <see cref="MaxCorrectionLinearSpeedMetersPerSecond"/>.
        /// </summary>
        public readonly float MaxCorrectionAngularSpeedRadPerSecond;

        /// <summary>
        /// History depth for a reconciler that can roll back and replay (<c>rollback</c>).
        /// Ignored by every other reconciler, including <c>snap</c>.
        /// </summary>
        public readonly int RollbackHistoryCapacity;

        public ReconcilerConfig(
            float convergencePositionToleranceMeters,
            float convergenceOrientationToleranceRadians,
            long maxTimeToConvergenceTicks,
            float maxCorrectionLinearSpeedMetersPerSecond,
            float maxCorrectionAngularSpeedRadPerSecond,
            int rollbackHistoryCapacity)
        {
            ConvergencePositionToleranceMeters = convergencePositionToleranceMeters;
            ConvergenceOrientationToleranceRadians = convergenceOrientationToleranceRadians;
            MaxTimeToConvergenceTicks = maxTimeToConvergenceTicks;
            MaxCorrectionLinearSpeedMetersPerSecond = maxCorrectionLinearSpeedMetersPerSecond;
            MaxCorrectionAngularSpeedRadPerSecond = maxCorrectionAngularSpeedRadPerSecond;
            RollbackHistoryCapacity = rollbackHistoryCapacity;
        }
    }
}
