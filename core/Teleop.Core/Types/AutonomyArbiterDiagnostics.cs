// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// What an autonomy arbiter reports about its own current decision. Returned from a property
    /// as a struct — Core never logs. Per Autonomy/CLAUDE.md, exposing the rung and the reason
    /// for it is a usability requirement: an operator who cannot tell which mode they are in will
    /// misattribute the robot's behavior to something else.
    /// </summary>
    public readonly struct AutonomyArbiterDiagnostics
    {
        /// <summary>Current rung.</summary>
        public readonly AutonomyRung Rung;

        /// <summary>Why <see cref="Rung"/> is what it is.</summary>
        public readonly AutonomyRungReason Reason;

        /// <summary>
        /// Most recent raw round-trip time passed to <c>Observe</c>, in ticks. Zero before the
        /// first observation.
        /// </summary>
        public readonly long LastRoundTripTicks;

        /// <summary>
        /// Smoothed round-trip time the rung decision is actually made against, in ticks — see
        /// <see cref="AutonomyArbiterConfig.LatencySmoothingAlpha"/>. Zero before the first
        /// observation.
        /// </summary>
        public readonly long SmoothedRoundTripTicks;

        /// <summary>
        /// How long, in ticks, the current <see cref="Rung"/> has been held. What
        /// <see cref="AutonomyArbiterConfig.MinRungDwellTicks"/> is compared against, and useful
        /// on its own for a HUD showing rung stability.
        /// </summary>
        public readonly long TicksInRung;

        public AutonomyArbiterDiagnostics(
            AutonomyRung rung,
            AutonomyRungReason reason,
            long lastRoundTripTicks,
            long smoothedRoundTripTicks,
            long ticksInRung)
        {
            Rung = rung;
            Reason = reason;
            LastRoundTripTicks = lastRoundTripTicks;
            SmoothedRoundTripTicks = smoothedRoundTripTicks;
            TicksInRung = ticksInRung;
        }

        public override string ToString() =>
            $"AutonomyArbiterDiagnostics(rung={Rung}, reason={Reason}, " +
            $"lastRtt={LastRoundTripTicks}, smoothedRtt={SmoothedRoundTripTicks}, " +
            $"ticksInRung={TicksInRung})";
    }
}
