using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Metrics;

namespace Teleop.Core.Tests.Metrics;

public class InMemoryMetricTrackerTests
{
    [Fact]
    public void Record_IsRetrievableByIndex_OldestFirst()
    {
        var tracker = new InMemoryMetricTracker(capacity: 4);

        tracker.Record("a", 1.0, 10);
        tracker.Record("b", 2.0, 20);

        Assert.Equal(2, tracker.Count);
        Assert.Equal(("a", 1.0, 10L), tracker[0]);
        Assert.Equal(("b", 2.0, 20L), tracker[1]);
    }

    [Fact]
    public void Record_BeyondCapacity_OverwritesOldest()
    {
        var tracker = new InMemoryMetricTracker(capacity: 2);

        tracker.Record("a", 1.0, 10);
        tracker.Record("b", 2.0, 20);
        tracker.Record("c", 3.0, 30);

        Assert.Equal(2, tracker.Count);
        Assert.Equal(("b", 2.0, 20L), tracker[0]);
        Assert.Equal(("c", 3.0, 30L), tracker[1]);
    }

    [Fact]
    public void TryGetLatest_ReturnsMostRecentMatchingSample()
    {
        var tracker = new InMemoryMetricTracker(capacity: 8);

        tracker.Record("m2p_ms", 10.0, 100);
        tracker.Record("owd_ms", 5.0, 105);
        tracker.Record("m2p_ms", 12.0, 110);

        Assert.True(tracker.TryGetLatest("m2p_ms", out double value, out long ticks));
        Assert.Equal(12.0, value);
        Assert.Equal(110, ticks);
    }

    [Fact]
    public void TryGetLatest_UnknownName_ReturnsFalse()
    {
        var tracker = new InMemoryMetricTracker(capacity: 4);
        tracker.Record("m2p_ms", 10.0, 100);

        bool found = tracker.TryGetLatest("nonexistent", out double value, out long ticks);

        Assert.False(found);
        Assert.Equal(0.0, value);
        Assert.Equal(0, ticks);
    }

    [Fact]
    public void Indexer_OutOfRange_Throws()
    {
        var tracker = new InMemoryMetricTracker(capacity: 4);
        tracker.Record("a", 1.0, 10);

        Assert.Throws<ArgumentOutOfRangeException>(() => tracker[1]);
        Assert.Throws<ArgumentOutOfRangeException>(() => tracker[-1]);
    }

    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        var tracker = new InMemoryMetricTracker(capacity: 4);
        tracker.Record("a", 1.0, 10);
        tracker.Record("b", 2.0, 20);

        tracker.Reset();

        Assert.Equal(0, tracker.Count);
        Assert.False(tracker.TryGetLatest("a", out _, out _));
    }

    [Fact]
    public void Record_Allocates_Zero_Bytes()
    {
        var tracker = new InMemoryMetricTracker(capacity: 16);
        AllocationAssert.Zero(() => tracker.Record("m2p_ms", 1.0, 1));
    }
}
