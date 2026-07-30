using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Time
{
    /// <summary>
    /// An <see cref="ITimeAuthority"/> stepped entirely by hand — no wall clock anywhere inside
    /// it. This is the clock replay and <c>Teleop.Core.Tests</c> use: a test or a recording
    /// replay drives time forward one call to <see cref="AdvanceTicks"/> at a time, and every
    /// two runs given the same sequence of calls produce the exact same sequence of
    /// <see cref="NowTicks"/> values. The host-side, wall-clock-backed counterpart
    /// (<c>MonotonicClock</c>, Stopwatch-based) deliberately lives in <c>Teleop.Eval</c> instead
    /// of here — constructing a <c>Stopwatch</c> is banned in Core.
    /// </summary>
    public sealed class ManualClock : ITimeAuthority
    {
        private readonly long _ticksPerSecond;
        private readonly long _startTicks;
        private long _nowTicks;

        /// <summary>
        /// <paramref name="ticksPerSecond"/> defaults to <c>TimeSpan.TicksPerSecond</c>
        /// (10,000,000) purely for test-code convenience — it is never load-bearing for
        /// correctness, since a real recorded session reconstructs the exact rate used at
        /// capture time from its own header (<c>RecordFormat</c>) rather than assuming this
        /// default.
        /// </summary>
        public ManualClock(long ticksPerSecond = 10_000_000, long startTicks = 0)
        {
            _ticksPerSecond = ticksPerSecond;
            _startTicks = startTicks;
            _nowTicks = startTicks;
        }

        public long TicksPerSecond => _ticksPerSecond;

        public long NowTicks => _nowTicks;

        /// <summary>
        /// Steps time forward by <paramref name="deltaTicks"/>. Throws on a negative delta —
        /// <see cref="ITimeAuthority.NowTicks"/> is documented as monotonic and non-decreasing,
        /// and failing loudly here catches a backwards step at its source instead of letting it
        /// silently corrupt a downstream determinism result.
        /// </summary>
        public void AdvanceTicks(long deltaTicks)
        {
            if (deltaTicks < 0)
            {
                throw new System.ArgumentOutOfRangeException(
                    nameof(deltaTicks), deltaTicks, "ManualClock cannot advance backwards.");
            }

            _nowTicks += deltaTicks;
        }

        /// <summary>
        /// Sets time to an absolute tick value. Throws if <paramref name="ticks"/> is before the
        /// current time, for the same monotonicity reason as <see cref="AdvanceTicks"/>.
        /// </summary>
        public void SetTicks(long ticks)
        {
            if (ticks < _nowTicks)
            {
                throw new System.ArgumentOutOfRangeException(
                    nameof(ticks), ticks, "ManualClock cannot move backwards.");
            }

            _nowTicks = ticks;
        }

        /// <summary>Returns time to the value given at construction.</summary>
        public void Reset() => _nowTicks = _startTicks;
    }
}
