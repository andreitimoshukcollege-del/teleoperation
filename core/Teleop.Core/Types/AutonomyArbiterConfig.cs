// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// The complete parameter set of an autonomy arbiter. Every number an arbiter uses comes
    /// from here, matching <see cref="PredictorConfig"/>'s rationale: a sweep varies an arbiter
    /// purely by varying this struct, and a run manifest records exactly what was run.
    ///
    /// One shared block across all arbiters; each implementation reads the subset that applies
    /// to it. <c>direct</c> ignores every field here. <c>waypoint</c> and <c>primitive</c> ignore
    /// the latency thresholds, since they commit to one rung unconditionally; only <c>scaled</c>
    /// and <c>ladder</c> use the full set.
    ///
    /// Threshold fields are ticks of round-trip time on the injected <c>ITimeAuthority</c>
    /// timebase, ascending: crossing <see cref="DirectToRateLimitedTicks"/> upward steps from
    /// <see cref="AutonomyRung.Direct"/> to <see cref="AutonomyRung.RateLimited"/>, and so on up
    /// the ladder. There is deliberately no <c>Default</c> — see <see cref="PredictorConfig"/>
    /// for why.
    /// </summary>
    public readonly struct AutonomyArbiterConfig
    {
        /// <summary>
        /// Round-trip time, in ticks, above which authority steps from
        /// <see cref="AutonomyRung.Direct"/> to <see cref="AutonomyRung.RateLimited"/>.
        /// </summary>
        public readonly long DirectToRateLimitedTicks;

        /// <summary>
        /// Round-trip time, in ticks, above which authority steps from
        /// <see cref="AutonomyRung.RateLimited"/> to <see cref="AutonomyRung.Waypoint"/>.
        /// </summary>
        public readonly long RateLimitedToWaypointTicks;

        /// <summary>
        /// Round-trip time, in ticks, above which authority steps from
        /// <see cref="AutonomyRung.Waypoint"/> to <see cref="AutonomyRung.IntentPrimitive"/>.
        /// </summary>
        public readonly long WaypointToIntentPrimitiveTicks;

        /// <summary>
        /// Band subtracted from a threshold when latency is falling: the smoothed RTT must drop
        /// below <c>threshold - HysteresisMarginTicks</c>, not just below <c>threshold</c>,
        /// before the rung steps back to more direct. This is what Autonomy/CLAUDE.md's
        /// hysteresis requirement is measured against — a noisy signal dithering around a bare
        /// threshold flips every sample; one that must clear a margin on the way back does not.
        /// </summary>
        public readonly long HysteresisMarginTicks;

        /// <summary>
        /// Smoothing factor applied to each incoming <c>Observe</c> sample before it is compared
        /// against a threshold, dimensionless, in [0, 1]. Higher tracks the newest RTT
        /// measurement more aggressively. Rung decisions are made against this smoothed value,
        /// never the raw sample — smoothing the input is the other half of what keeps the rung
        /// from flapping on a single noisy measurement.
        /// </summary>
        public readonly float LatencySmoothingAlpha;

        /// <summary>
        /// Minimum time a rung must be held before another transition is permitted, in ticks. A
        /// second, independent guard against oscillation beyond
        /// <see cref="HysteresisMarginTicks"/>: it bounds the switch rate directly rather than
        /// relying on the margin alone to do it.
        /// </summary>
        public readonly long MinRungDwellTicks;

        /// <summary>
        /// Time over which authority-affecting parameters (e.g. a velocity clamp taking effect)
        /// ramp when the rung changes, in ticks. What makes <c>Arbitrate</c>'s bounded-transition
        /// contract achievable: stepping the clamp instantly is exactly the discontinuity the
        /// contract forbids.
        /// </summary>
        public readonly long AuthorityRampTicks;

        public AutonomyArbiterConfig(
            long directToRateLimitedTicks,
            long rateLimitedToWaypointTicks,
            long waypointToIntentPrimitiveTicks,
            long hysteresisMarginTicks,
            float latencySmoothingAlpha,
            long minRungDwellTicks,
            long authorityRampTicks)
        {
            DirectToRateLimitedTicks = directToRateLimitedTicks;
            RateLimitedToWaypointTicks = rateLimitedToWaypointTicks;
            WaypointToIntentPrimitiveTicks = waypointToIntentPrimitiveTicks;
            HysteresisMarginTicks = hysteresisMarginTicks;
            LatencySmoothingAlpha = latencySmoothingAlpha;
            MinRungDwellTicks = minRungDwellTicks;
            AuthorityRampTicks = authorityRampTicks;
        }
    }
}
