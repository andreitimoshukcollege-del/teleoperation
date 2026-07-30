using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Transport;

public class EmulatedTransportTests
{
    private const int MaxPayload = 32;

    /// <summary>A link with every knob off: fixed delay only, nothing random.</summary>
    private static NetworkProfile FixedDelay(long delayTicks) =>
        new NetworkProfile(
            baseDelayTicks: delayTicks,
            jitterTicks: 0,
            lossProbabilityAfterDelivered: 0.0,
            lossProbabilityAfterLost: 0.0,
            reorderProbability: 0.0,
            reorderDelayTicks: 0);

    private static byte[] Tagged(int tag)
    {
        var payload = new byte[4];
        payload[0] = (byte)tag;
        payload[1] = 0xAB;
        return payload;
    }

    /// <summary>
    /// Drives a fixed schedule: <paramref name="count"/> sends spaced <paramref name="stepTicks"/>
    /// apart, draining fully after each send, then <paramref name="tailPolls"/> further drains to
    /// flush everything still under synthetic delay. Returns the exact per-send outcomes and the
    /// exact receive stream, which is what a determinism assertion compares.
    /// </summary>
    private static (List<bool> Sends, List<(int Tag, long Arrival)> Receives) RunSchedule(
        EmulatedTransport transport, int count, long stepTicks, int tailPolls)
    {
        var sends = new List<bool>(count);
        var receives = new List<(int, long)>(count);
        var destination = new byte[transport.MaxPayloadBytes];
        long now = 0;

        for (int i = 0; i < count; i++)
        {
            sends.Add(transport.Send(Tagged(i), now));
            while (transport.TryReceive(now, destination, out _, out long arrival))
            {
                receives.Add((destination[0], arrival));
            }

            now += stepTicks;
        }

        for (int i = 0; i < tailPolls; i++)
        {
            while (transport.TryReceive(now, destination, out _, out long arrival))
            {
                receives.Add((destination[0], arrival));
            }

            now += stepTicks;
        }

        return (sends, receives);
    }

    [Fact]
    public void MaxPayloadBytes_PassesThroughFromTheWrappedTransport()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, FixedDelay(0), new SeededRng(1UL), maxInFlight: 8);

        Assert.Equal(inner.MaxPayloadBytes, transport.MaxPayloadBytes);
        Assert.Equal(8, transport.MaxInFlight);
        Assert.Equal(0, transport.InFlightCount);
    }

    [Fact]
    public void Constructor_RejectsInvalidArguments()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var rng = new SeededRng(1UL);

        Assert.Throws<ArgumentNullException>(
            () => new EmulatedTransport(null!, FixedDelay(0), rng, 8));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EmulatedTransport(inner, FixedDelay(0), rng, 0));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EmulatedTransport(inner, FixedDelay(-1), rng, 8));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EmulatedTransport(
                inner, new NetworkProfile(0, -1, 0, 0, 0, 0), rng, 8));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EmulatedTransport(
                inner, new NetworkProfile(0, 0, 1.5, 0, 0, 0), rng, 8));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EmulatedTransport(
                inner, new NetworkProfile(0, 0, 0, double.NaN, 0, 0), rng, 8));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EmulatedTransport(
                inner, new NetworkProfile(0, 0, 0, 0, 0, -5), rng, 8));
    }

    /// <summary>
    /// Gate 3, docs/setup.md: "Inject a synthetic 137 ms through EmulatedTransport over
    /// LoopbackTransport; the measurement pipeline reports 137 ± 1 ms." The clock runs at
    /// 10,000,000 ticks/second, so 137 ms is 1,370,000 ticks and the tolerance is ±10,000 ticks.
    /// The datagram must also be invisible before that window and visible at it.
    /// </summary>
    [Fact]
    public void Gate3_SyntheticOneWayDelayOf137Ms_IsMeasuredAs137MsWithin1Ms()
    {
        var clock = new ManualClock();
        long ticksPerMs = clock.TicksPerSecond / 1000;
        long expected = 137 * ticksPerMs;      // 1,370,000
        long tolerance = 1 * ticksPerMs;       //    10,000

        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, FixedDelay(expected), new SeededRng(20260730UL), 8);
        var destination = new byte[MaxPayload];

        long sendTicks = clock.NowTicks;
        Assert.Equal(0, sendTicks);
        Assert.True(transport.Send(Tagged(1), sendTicks));

        // Not receivable anywhere before the lower edge of the tolerance window.
        for (long t = 0; t < expected - tolerance; t += ticksPerMs)
        {
            clock.SetTicks(t);
            Assert.False(transport.TryReceive(clock.NowTicks, destination, out _, out _));
        }

        // Poll at 1 ms, exactly as a host frame loop would, and take the first delivery.
        long firstVisibleTicks = -1;
        long measuredDelay = -1;
        for (long t = expected - tolerance; t <= expected + (10 * ticksPerMs); t += ticksPerMs)
        {
            clock.SetTicks(t);
            if (transport.TryReceive(clock.NowTicks, destination, out int byteCount, out long arrivalTicks))
            {
                firstVisibleTicks = t;
                measuredDelay = arrivalTicks - sendTicks;
                Assert.Equal(4, byteCount);
                Assert.Equal(1, destination[0]);
                break;
            }
        }

        Assert.True(firstVisibleTicks >= 0, "datagram never became visible");
        Assert.InRange(firstVisibleTicks, expected - tolerance, expected + tolerance);
        Assert.InRange(measuredDelay, expected - tolerance, expected + tolerance);

        // With jitter off the delay is not merely within tolerance, it is exact.
        Assert.Equal(expected, measuredDelay);
        Assert.False(transport.TryReceive(clock.NowTicks, destination, out _, out _));
    }

    [Fact]
    public void ArrivalTicks_MeasureInnerArrivalPlusDelay_NotPollTime()
    {
        long delay = 1_000_000;
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, FixedDelay(delay), new SeededRng(5UL), 8);
        var destination = new byte[MaxPayload];

        transport.Send(Tagged(1), nowTicks: 250);

        // Host polls very late. Folding poll time into arrival would report 9,000,000 here and
        // silently inflate every one-way delay figure by the host's frame time.
        Assert.True(transport.TryReceive(9_000_000, destination, out _, out long arrivalTicks));
        Assert.Equal(250 + delay, arrivalTicks);
    }

    [Fact]
    public void TotalLoss_DropsEveryDatagramAndNeverTouchesTheWrappedTransport()
    {
        var profile = new NetworkProfile(0, 0, 1.0, 1.0, 0.0, 0);
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, profile, new SeededRng(77UL), 8);
        var destination = new byte[MaxPayload];

        for (int i = 0; i < 50; i++)
        {
            Assert.False(transport.Send(Tagged(i), i));
        }

        // A dropped packet never reaches the wire, so the wrapped transport saw nothing at all.
        Assert.Equal(0, inner.QueuedCount);
        Assert.Equal(0, transport.InFlightCount);
        Assert.False(transport.TryReceive(long.MaxValue, destination, out _, out _));
    }

    [Fact]
    public void ZeroLossProbabilities_DeliverEveryDatagram()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 64);
        var transport = new EmulatedTransport(inner, FixedDelay(1000), new SeededRng(3UL), 64);

        var (sends, receives) = RunSchedule(transport, count: 50, stepTicks: 500, tailPolls: 8);

        Assert.All(sends, Assert.True);
        Assert.Equal(50, receives.Count);
        Assert.Equal(Enumerable.Range(0, 50).ToArray(), receives.Select(r => r.Tag).ToArray());
    }

    [Fact]
    public void LossAfterLostOfOne_IsAnAbsorbingOutage()
    {
        var profile = new NetworkProfile(0, 0, 0.5, 1.0, 0.0, 0);
        var inner = new LoopbackTransport(MaxPayload, capacity: 64);
        var transport = new EmulatedTransport(inner, profile, new SeededRng(11UL), 64);

        Assert.Equal(double.PositiveInfinity, profile.ExpectedBurstLength);

        bool seenLoss = false;
        for (int i = 0; i < 64; i++)
        {
            bool delivered = transport.Send(Tagged(i), i);
            if (seenLoss)
            {
                Assert.False(delivered);
            }

            seenLoss |= !delivered;
        }

        Assert.True(seenLoss, "a 0.5 good-state loss probability should have dropped something in 64 sends");
    }

    /// <summary>
    /// The Gilbert-Elliott requirement from Transport/CLAUDE.md: losses must come in runs, not
    /// independently. With a 0.8 stay-in-bad probability the model's expected run length is 5.
    /// </summary>
    [Fact]
    public void BurstLoss_ProducesRunsRatherThanIndependentDrops()
    {
        var profile = new NetworkProfile(0, 0, 0.02, 0.8, 0.0, 0);
        var inner = new LoopbackTransport(MaxPayload, capacity: 4);
        var transport = new EmulatedTransport(inner, profile, new SeededRng(2024UL), 4);
        var destination = new byte[MaxPayload];

        int bursts = 0;
        int lost = 0;
        int longestRun = 0;
        int currentRun = 0;

        for (int i = 0; i < 4000; i++)
        {
            bool delivered = transport.Send(Tagged(i & 0xFF), i);
            while (transport.TryReceive(i, destination, out _, out _))
            {
            }

            if (delivered)
            {
                currentRun = 0;
                continue;
            }

            lost++;
            if (currentRun == 0)
            {
                bursts++;
            }

            currentRun++;
            longestRun = Math.Max(longestRun, currentRun);
        }

        Assert.Equal(5.0, profile.ExpectedBurstLength, 9);
        Assert.True(bursts > 0, "expected some losses");

        double meanRun = (double)lost / bursts;
        Assert.InRange(meanRun, 3.0, 8.0);
        Assert.True(longestRun >= 3, $"expected a burst of at least 3, longest was {longestRun}");
    }

    [Fact]
    public void Jitter_StaysWithinTheProfileHalfWidthAndUsesBothSides()
    {
        long baseDelay = 1_000_000;
        long jitter = 200_000;
        var profile = new NetworkProfile(baseDelay, jitter, 0.0, 0.0, 0.0, 0);
        var inner = new LoopbackTransport(MaxPayload, capacity: 128);
        var transport = new EmulatedTransport(inner, profile, new SeededRng(4242UL), 128);
        var destination = new byte[MaxPayload];

        long minDelay = long.MaxValue;
        long maxDelay = long.MinValue;
        int received = 0;

        void Collect(long pollTicks)
        {
            while (transport.TryReceive(pollTicks, destination, out _, out long arrivalTicks))
            {
                long sendTicks = destination[0] * 10_000L;
                long delay = arrivalTicks - sendTicks;
                Assert.InRange(delay, baseDelay - jitter, baseDelay + jitter);
                minDelay = Math.Min(minDelay, delay);
                maxDelay = Math.Max(maxDelay, delay);
                received++;
            }
        }

        for (int i = 0; i < 100; i++)
        {
            long sendTicks = i * 10_000L;
            Assert.True(transport.Send(Tagged(i), sendTicks));

            // Poll every send, as a host frame loop would. Sends are 1 ms apart and the delay is
            // ~100 ms, so the earliest datagrams do come due partway through this loop and must be
            // collected here rather than discarded.
            Collect(sendTicks);
        }

        Collect(long.MaxValue / 4);

        Assert.Equal(100, received);
        Assert.True(minDelay < baseDelay, "jitter never went negative");
        Assert.True(maxDelay > baseDelay, "jitter never went positive");
    }

    /// <summary>
    /// Reordering must actually invert delivery order, and delivery must come out in synthetic
    /// arrival order rather than send order. The reorder delay here exceeds the send interval, so
    /// a selected datagram falls behind its successors.
    /// </summary>
    [Fact]
    public void Reordering_DeliversInSyntheticArrivalOrderNotSendOrder()
    {
        long step = 10_000;
        var profile = new NetworkProfile(
            baseDelayTicks: 100_000,
            jitterTicks: 0,
            lossProbabilityAfterDelivered: 0.0,
            lossProbabilityAfterLost: 0.0,
            reorderProbability: 0.5,
            reorderDelayTicks: 50_000);

        var inner = new LoopbackTransport(MaxPayload, capacity: 64);
        var transport = new EmulatedTransport(inner, profile, new SeededRng(31337UL), 64);
        var destination = new byte[MaxPayload];

        for (int i = 0; i < 24; i++)
        {
            Assert.True(transport.Send(Tagged(i), i * step));
        }

        var tags = new List<int>();
        var arrivals = new List<long>();
        while (transport.TryReceive(long.MaxValue / 4, destination, out _, out long arrivalTicks))
        {
            tags.Add(destination[0]);
            arrivals.Add(arrivalTicks);
        }

        Assert.Equal(24, tags.Count);

        // Everything arrives, but not in send order.
        Assert.Equal(Enumerable.Range(0, 24).ToHashSet(), tags.ToHashSet());
        Assert.NotEqual(Enumerable.Range(0, 24).ToArray(), tags.ToArray());

        // And what does come out is ordered by synthetic arrival, which is the contract's
        // "datagrams are returned in arrival order, which is not send order".
        for (int i = 1; i < arrivals.Count; i++)
        {
            Assert.True(arrivals[i] >= arrivals[i - 1], "delivery was not in synthetic arrival order");
        }
    }

    [Fact]
    public void Send_PropagatesFalseWhenTheWrappedTransportRefuses()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 2);
        var transport = new EmulatedTransport(inner, FixedDelay(1_000_000), new SeededRng(9UL), 8);

        Assert.True(transport.Send(Tagged(0), 0));
        Assert.True(transport.Send(Tagged(1), 0));

        // The emulator's own loss model said "deliver", but the wrapped transport's queue is full.
        Assert.False(transport.Send(Tagged(2), 0));
    }

    [Fact]
    public void InFlightCapacityExhausted_LeavesDatagramsInTheWrappedTransport()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, FixedDelay(1_000_000), new SeededRng(13UL), maxInFlight: 2);
        var destination = new byte[MaxPayload];

        for (int i = 0; i < 5; i++)
        {
            Assert.True(transport.Send(Tagged(i), 0));
        }

        Assert.False(transport.TryReceive(0, destination, out _, out _));
        Assert.Equal(2, transport.InFlightCount);

        // Back-pressure, not extra loss: the three that did not fit are still in the inner queue.
        Assert.Equal(3, inner.QueuedCount);

        var tags = new List<int>();
        for (int poll = 0; poll < 10; poll++)
        {
            while (transport.TryReceive(10_000_000, destination, out _, out _))
            {
                tags.Add(destination[0]);
            }
        }

        Assert.Equal(new[] { 0, 1, 2, 3, 4 }, tags.ToArray());
    }

    [Fact]
    public void TryReceive_DestinationTooSmall_ReportsRequiredLengthAndKeepsDatagramQueued()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, FixedDelay(1000), new SeededRng(17UL), 8);

        transport.Send(Tagged(6), nowTicks: 0);

        var tooSmall = new byte[2];
        Assert.False(transport.TryReceive(5000, tooSmall, out int required, out long arrivalTicks));
        Assert.Equal(4, required);
        Assert.Equal(0, arrivalTicks);
        Assert.Equal(1, transport.InFlightCount);

        var destination = new byte[MaxPayload];
        Assert.True(transport.TryReceive(5000, destination, out int byteCount, out arrivalTicks));
        Assert.Equal(4, byteCount);
        Assert.Equal(6, destination[0]);
        Assert.Equal(1000, arrivalTicks);
    }

    /// <summary>
    /// Same seed, same profile, same call sequence => byte-identical output. The impaired profile
    /// is used deliberately: with loss, jitter and reordering all live, every RNG draw is
    /// load-bearing.
    /// </summary>
    [Fact]
    public void Determinism_TwoInstancesWithTheSameSeedProduceIdenticalStreams()
    {
        var profile = new NetworkProfile(200_000, 40_000, 0.1, 0.6, 0.2, 300_000);

        var a = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, 64), profile, new SeededRng(0xC0FFEEUL), 64);
        var b = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, 64), profile, new SeededRng(0xC0FFEEUL), 64);

        var first = RunSchedule(a, count: 120, stepTicks: 10_000, tailPolls: 64);
        var second = RunSchedule(b, count: 120, stepTicks: 10_000, tailPolls: 64);

        Assert.Equal(first.Sends, second.Sends);
        Assert.Equal(first.Receives, second.Receives);

        // Sanity: the schedule really did exercise loss and reordering, so equality above is not
        // the trivial equality of two unimpaired runs.
        Assert.Contains(false, first.Sends);
        Assert.NotEqual(
            first.Receives.Select(r => r.Tag).OrderBy(t => t).ToArray(),
            first.Receives.Select(r => r.Tag).ToArray());
    }

    [Fact]
    public void DifferentSeeds_ProduceDifferentStreams()
    {
        var profile = new NetworkProfile(200_000, 40_000, 0.1, 0.6, 0.2, 300_000);

        var a = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, 64), profile, new SeededRng(1UL), 64);
        var b = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, 64), profile, new SeededRng(2UL), 64);

        var first = RunSchedule(a, count: 120, stepTicks: 10_000, tailPolls: 64);
        var second = RunSchedule(b, count: 120, stepTicks: 10_000, tailPolls: 64);

        Assert.NotEqual(first.Receives, second.Receives);
    }

    [Fact]
    public void Reset_RestoresAsConstructedStateAndReseedsTheRng()
    {
        var profile = new NetworkProfile(200_000, 40_000, 0.1, 0.6, 0.2, 300_000);
        var inner = new LoopbackTransport(MaxPayload, 64);
        var transport = new EmulatedTransport(inner, profile, new SeededRng(555UL), 64);

        var firstTrial = RunSchedule(transport, count: 80, stepTicks: 10_000, tailPolls: 32);

        // Stop mid-flight so Reset has a populated heap, a used free list and a dirty
        // Gilbert-Elliott bit to clear.
        for (int i = 0; i < 5; i++)
        {
            transport.Send(Tagged(i), 5_000_000 + i);
        }

        transport.TryReceive(5_000_000, new byte[MaxPayload], out _, out _);

        transport.Reset();

        Assert.Equal(0, transport.InFlightCount);
        Assert.Equal(0, inner.QueuedCount);   // a decorator resets what it wraps
        Assert.False(transport.TryReceive(long.MaxValue / 4, new byte[MaxPayload], out int byteCount, out long arrivalTicks));
        Assert.Equal(0, byteCount);
        Assert.Equal(0, arrivalTicks);

        // Reseeded: the identical schedule reproduces the identical trial.
        var secondTrial = RunSchedule(transport, count: 80, stepTicks: 10_000, tailPolls: 32);
        Assert.Equal(firstTrial.Sends, secondTrial.Sends);
        Assert.Equal(firstTrial.Receives, secondTrial.Receives);

        // And it matches a freshly constructed instance with the same seed, which is the actual
        // definition of "as-constructed state".
        var fresh = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, 64), profile, new SeededRng(555UL), 64);
        var freshTrial = RunSchedule(fresh, count: 80, stepTicks: 10_000, tailPolls: 32);
        Assert.Equal(freshTrial.Sends, secondTrial.Sends);
        Assert.Equal(freshTrial.Receives, secondTrial.Receives);
    }

    [Fact]
    public void SendAndTryReceive_SteadyState_Allocate_Zero_Bytes()
    {
        // Common path only: no loss, no jitter, no reordering, and the datagram is due
        // immediately, so every iteration exercises send, drain, heap push, heap pop and copy.
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, FixedDelay(0), new SeededRng(21UL), maxInFlight: 8);
        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(0, destination, out _, out _);
        });
    }

    [Fact]
    public void LossBranch_Allocates_Zero_Bytes()
    {
        // Verified rather than assumed: the drop path must not allocate either, or a lossy sweep
        // would GC in proportion to how lossy the profile is.
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(
            inner, new NetworkProfile(0, 0, 1.0, 1.0, 0.0, 0), new SeededRng(22UL), 8);
        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(0, destination, out _, out _);
        });
    }

    [Fact]
    public void JitterAndReorderBranches_Allocate_Zero_Bytes()
    {
        // Both random-draw branches taken every iteration (reorder probability 1.0), again
        // verified rather than assumed.
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(
            inner, new NetworkProfile(1000, 500, 0.0, 0.0, 1.0, 2000), new SeededRng(23UL), 8);
        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(1_000_000, destination, out _, out _);
        });
    }
}
