using System.Diagnostics;
using Teleop.Core.Contracts;

namespace Teleop.Eval.Time
{
    /// <summary>
    /// The wall-clock-backed <c>ITimeAuthority</c> for headless runs. Lives here, not in Core,
    /// because constructing a <see cref="Stopwatch"/> is banned in Core (root CLAUDE.md
    /// invariant 2) -- Core's replay-facing clock is <c>Teleop.Core.Time.ManualClock</c> instead.
    /// </summary>
    public sealed class MonotonicClock : ITimeAuthority
    {
        public long TicksPerSecond => Stopwatch.Frequency;

        public long NowTicks => Stopwatch.GetTimestamp();
    }
}
