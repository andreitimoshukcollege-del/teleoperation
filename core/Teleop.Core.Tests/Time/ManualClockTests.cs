using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;

namespace Teleop.Core.Tests.Time;

public class ManualClockTests
{
    [Fact]
    public void Constructor_SetsInitialTicksAndRate()
    {
        var clock = new ManualClock(ticksPerSecond: 1000, startTicks: 500);

        Assert.Equal(1000, clock.TicksPerSecond);
        Assert.Equal(500, clock.NowTicks);
    }

    [Fact]
    public void DefaultConstructor_UsesTimeSpanTicksPerSecond()
    {
        var clock = new ManualClock();

        Assert.Equal(10_000_000, clock.TicksPerSecond);
        Assert.Equal(0, clock.NowTicks);
    }

    [Fact]
    public void AdvanceTicks_MovesTimeForward()
    {
        var clock = new ManualClock(startTicks: 100);

        clock.AdvanceTicks(50);

        Assert.Equal(150, clock.NowTicks);
    }

    [Fact]
    public void AdvanceTicks_Zero_IsANoOp()
    {
        var clock = new ManualClock(startTicks: 100);

        clock.AdvanceTicks(0);

        Assert.Equal(100, clock.NowTicks);
    }

    [Fact]
    public void AdvanceTicks_Negative_Throws()
    {
        var clock = new ManualClock(startTicks: 100);

        Assert.Throws<ArgumentOutOfRangeException>(() => clock.AdvanceTicks(-1));
        Assert.Equal(100, clock.NowTicks);
    }

    [Fact]
    public void SetTicks_MovesToAnAbsoluteTime()
    {
        var clock = new ManualClock(startTicks: 100);

        clock.SetTicks(300);

        Assert.Equal(300, clock.NowTicks);
    }

    [Fact]
    public void SetTicks_Backwards_Throws()
    {
        var clock = new ManualClock(startTicks: 100);
        clock.AdvanceTicks(50);

        Assert.Throws<ArgumentOutOfRangeException>(() => clock.SetTicks(100));
        Assert.Equal(150, clock.NowTicks);
    }

    [Fact]
    public void Reset_RestoresConstructedStartTicks()
    {
        var clock = new ManualClock(startTicks: 42);
        clock.AdvanceTicks(1000);

        clock.Reset();

        Assert.Equal(42, clock.NowTicks);
    }

    [Fact]
    public void AdvanceTicks_Allocates_Zero_Bytes()
    {
        var clock = new ManualClock();
        AllocationAssert.Zero(() => clock.AdvanceTicks(1));
    }
}
