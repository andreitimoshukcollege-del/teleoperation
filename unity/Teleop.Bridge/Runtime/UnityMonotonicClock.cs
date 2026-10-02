using System.Diagnostics;
using Teleop.Core.Contracts;

namespace Teleop.Bridge
{
    /// <summary>
    /// The wall-clock-backed <see cref="ITimeAuthority"/> for Unity. Mirrors
    /// <c>Teleop.Eval.Time.MonotonicClock</c> exactly -- constructing a <see cref="Stopwatch"/> is
    /// banned in Core (root CLAUDE.md invariant 2), but Unity is a host, same as Teleop.Eval, and
    /// is allowed to touch a real clock. Never use <c>Time.time</c> here: it is frame-quantized,
    /// resets on scene load, and stops in a paused editor (Teleop/CLAUDE.md's "Time" section).
    /// </summary>
    public sealed class UnityMonotonicClock : ITimeAuthority
    {
        public long TicksPerSecond => Stopwatch.Frequency;

        public long NowTicks => Stopwatch.GetTimestamp();
    }
}
