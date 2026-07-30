// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// A rung on Sheridan's supervisory-control ladder (Autonomy/CLAUDE.md), ordered from most
    /// to least direct. Numeric order is directness order: an <c>IAutonomyArbiter</c>'s
    /// monotonic-in-latency contract is exactly "the rung value never decreases while measured
    /// latency is non-decreasing."
    /// </summary>
    public enum AutonomyRung : byte
    {
        /// <summary>Operator pose is a setpoint, one-to-one.</summary>
        Direct = 0,

        /// <summary>Authority attenuated: velocity and/or displacement clamped.</summary>
        RateLimited = 1,

        /// <summary>Commands become goals the robot plans a path to.</summary>
        Waypoint = 2,

        /// <summary>Classified intent, resolved into a robot-side behavior primitive.</summary>
        IntentPrimitive = 3,
    }
}
