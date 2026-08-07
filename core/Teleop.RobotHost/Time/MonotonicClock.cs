using System.Diagnostics;
using Teleop.Core.Contracts;

namespace Teleop.RobotHost.Time
{
    /// <summary>
    /// The wall-clock-backed <c>ITimeAuthority</c> for this host. Mirrors
    /// <c>Teleop.Eval.Time.MonotonicClock</c> and <c>Teleop.Bridge.UnityMonotonicClock</c>
    /// exactly -- constructing a <see cref="Stopwatch"/> is banned in Core (root CLAUDE.md
    /// invariant 2), but this host, like Teleop.Eval and Unity, is allowed to touch a real
    /// clock. Every host gets its own copy rather than sharing one across assemblies, matching
    /// the existing precedent.
    /// </summary>
    public sealed class MonotonicClock : ITimeAuthority
    {
        public long TicksPerSecond => Stopwatch.Frequency;

        public long NowTicks => Stopwatch.GetTimestamp();
    }
}
