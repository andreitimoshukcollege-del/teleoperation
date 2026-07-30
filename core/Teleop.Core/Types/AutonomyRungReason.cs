// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// Why an <c>IAutonomyArbiter</c>'s current <see cref="AutonomyRung"/> is what it is,
    /// exposed through <see cref="AutonomyArbiterDiagnostics"/> so a HUD can explain the robot's
    /// behavior to the operator rather than leaving a rung change unexplained.
    /// </summary>
    public enum AutonomyRungReason : byte
    {
        /// <summary>As-constructed or freshly <c>Reset</c>; no latency observation yet.</summary>
        Initial = 0,

        /// <summary>Smoothed latency is within the current rung's band, clear of both edges.</summary>
        WithinBand = 1,

        /// <summary>Smoothed latency crossed the upper threshold; the rung stepped to less direct.</summary>
        LatencyRose = 2,

        /// <summary>Smoothed latency crossed the lower threshold; the rung stepped to more direct.</summary>
        LatencyFell = 3,

        /// <summary>
        /// Smoothed latency re-crossed a threshold within the hysteresis margin of the last
        /// transition; the rung held rather than stepping back immediately.
        /// </summary>
        HysteresisHold = 4,
    }
}
