using Teleop.RobotHost.Time;

namespace Teleop.RobotHost.Tests.Time
{
    /// <summary>
    /// RobotHost's lines printed at most once per interval (docs/adr/0038 "M, motion", 2026-11-24): the refused-command line had
    /// never printed, because <c>now - long.MinValue</c> overflows to a negative number.
    /// </summary>
    public sealed class LogThrottleTests
    {
        private const long Second = 1_000_000_000;
        private const long Now = 1_860_973_488_056_466; // CLOCK_MONOTONIC after 21 days up, as on the GPU machine's twin

        [Fact]
        public void TheFirstLine_IsDue()
        {
            Assert.True(LogThrottle.Due(Now, LogThrottle.Never, 2 * Second));
            Assert.True(LogThrottle.Due(0, LogThrottle.Never, 2 * Second));
            // What Program asked before: it overflowed, and the line never came.
            long last = LogThrottle.Never;
            Assert.False(unchecked(Now - last) >= 2 * Second);
        }

        [Fact]
        public void ALaterLine_IsDueOnlyOnceTheIntervalHasPassed()
        {
            Assert.False(LogThrottle.Due(Now, Now - Second, 2 * Second));
            Assert.True(LogThrottle.Due(Now, Now - 2 * Second, 2 * Second));
            Assert.False(LogThrottle.Due(Now, Now, 2 * Second));
        }
    }
}
