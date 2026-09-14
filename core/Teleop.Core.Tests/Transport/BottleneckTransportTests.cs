using Teleop.Core.Contracts;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Transport;
using Teleop.Core.Transport.Impairments;

namespace Teleop.Core.Tests.Transport;

/// <summary>
/// <see cref="BottleneckTransport"/>: the finite-rate, finite-queue link model that makes
/// sender-side send-rate behaviour measurable at all on this platform.
///
/// The claims worth pinning are the ones a later reader would otherwise have to re-derive: that the
/// departure recurrence is exactly the textbook FIFO one, that an over-capacity offered load
/// produces a *standing* queue rather than a growing delay, that overflow is tail drop at
/// <c>Send</c> (so no byte the model discards is ever put on the wire), and that none of it depends
/// on when the host happens to poll.
/// </summary>
public sealed class BottleneckTransportTests
{
    private const long TicksPerSecond = 10_000_000;
    private const int PayloadBytes = 73;

    /// <summary>7300 B/s is the sweep's own offered load: 73 bytes every 10 ms.</summary>
    private const long OfferedBytesPerSecond = 7300;

    private static byte[] Payload(byte fill)
    {
        var payload = new byte[PayloadBytes];
        for (int i = 0; i < payload.Length; i++)
        {
            payload[i] = fill;
        }

        return payload;
    }

    private static BottleneckTransport Build(long capacityBytesPerSecond, int bufferDatagrams, int innerCapacity = 256) =>
        new BottleneckTransport(
            new LoopbackTransport(PayloadBytes, innerCapacity),
            capacityBytesPerSecond,
            bufferDatagrams,
            TicksPerSecond);

    [Fact]
    public void SerializationTicks_IsBytesOverRate()
    {
        // 73 bytes at 7300 B/s is 10 ms is 100_000 ticks at this project's 10 MHz timebase.
        var transport = Build(OfferedBytesPerSecond, bufferDatagrams: 8);
        Assert.Equal(100_000, transport.SerializationTicks(PayloadBytes));

        // Half the rate, twice the time.
        var slow = Build(OfferedBytesPerSecond / 2, bufferDatagrams: 8);
        Assert.Equal(200_000, slow.SerializationTicks(PayloadBytes));
    }

    [Fact]
    public void UnderloadedLink_AddsExactlyOneSerializationTime_AndNoQueue()
    {
        // Capacity is 4x the offered load, so nothing ever waits behind anything.
        var transport = Build(OfferedBytesPerSecond * 4, bufferDatagrams: 8);
        long serialization = transport.SerializationTicks(PayloadBytes);
        var destination = new byte[PayloadBytes];

        for (int step = 0; step < 20; step++)
        {
            long now = step * 100_000L;
            Assert.True(transport.Send(Payload((byte)step), now));

            Assert.True(transport.TryReceive(now + serialization, destination, out int byteCount, out long arrivalTicks));
            Assert.Equal(PayloadBytes, byteCount);
            Assert.Equal(now + serialization, arrivalTicks);
            Assert.Equal(0, transport.QueuedCount);
        }
    }

    [Fact]
    public void DepartureRecurrence_IsMaxOfArrivalAndPreviousDeparture_PlusSerialization()
    {
        // Half the offered rate: 20 ms per datagram against a 10 ms offer, so each datagram departs
        // one full service time after the previous one departed, not after it arrived.
        var transport = Build(OfferedBytesPerSecond / 2, bufferDatagrams: 16);
        long serialization = transport.SerializationTicks(PayloadBytes); // 200_000
        var destination = new byte[PayloadBytes];

        for (int i = 0; i < 8; i++)
        {
            Assert.True(transport.Send(Payload((byte)i), i * 100_000L));
        }

        // Poll far enough ahead that everything has departed, and check each arrival tick.
        long farFuture = 100_000_000L;
        for (int i = 0; i < 8; i++)
        {
            Assert.True(transport.TryReceive(farFuture, destination, out _, out long arrivalTicks));
            Assert.Equal((i + 1) * serialization, arrivalTicks);
        }
    }

    [Fact]
    public void OverCapacityOfferedLoad_ProducesAStandingQueue_NotAGrowingOne()
    {
        // The load-bearing claim behind the whole candidate: at capacity below the offered rate the
        // queue saturates and delay stops growing at bufferDatagrams * serialization -- it does not
        // run away, because the excess is tail-dropped rather than queued.
        const int bufferDatagrams = 10;
        var transport = Build(OfferedBytesPerSecond / 2, bufferDatagrams);
        long serialization = transport.SerializationTicks(PayloadBytes); // 200_000 == 20 ms
        long expectedStandingDelay = bufferDatagrams * serialization;    // 200 ms

        var destination = new byte[PayloadBytes];
        int accepted = 0;
        int delivered = 0;

        for (int step = 0; step < 600; step++)
        {
            long now = step * 100_000L;
            if (transport.Send(Payload(0), now))
            {
                accepted++;
            }

            while (transport.TryReceive(now, destination, out _, out _))
            {
                delivered++;
            }
        }

        // Nothing accepted is lost by this transport; only refused datagrams are lost, and they are
        // refused before anything is forwarded.
        Assert.InRange(delivered, accepted - bufferDatagrams, accepted);

        // Acceptance converges to capacity/payload = 50 per second = one every other 10 ms step.
        // 600 steps at half rate is ~300, plus the buffer's worth absorbed during the initial fill.
        Assert.InRange(accepted, 295, 315);

        // The queue is pinned at (or one below) its depth rather than growing: this is a standing
        // queue, which is why the delay it imposes is bounded by bufferDatagrams * serialization.
        Assert.InRange(transport.QueuedCount, bufferDatagrams - 1, bufferDatagrams);
        Assert.Equal(2_000_000, expectedStandingDelay); // 200 ms, stated so the number is visible
    }

    /// <summary>
    /// The previous test measures acceptance, not per-datagram delay; this one measures delay
    /// directly by stamping the send tick into the payload, which is how the experiment harness
    /// does it too.
    /// </summary>
    [Fact]
    public void SaturatedQueue_DelaysEveryDeliveredDatagramByTheWholeBuffer()
    {
        const int bufferDatagrams = 10;
        var transport = Build(OfferedBytesPerSecond / 2, bufferDatagrams);
        long serialization = transport.SerializationTicks(PayloadBytes);
        long expectedStandingDelay = bufferDatagrams * serialization; // 200 ms

        var destination = new byte[PayloadBytes];
        var delays = new List<long>();

        for (int step = 0; step < 600; step++)
        {
            long now = step * 100_000L;
            var payload = Payload(0);
            System.BitConverter.TryWriteBytes(payload.AsSpan(0, 8), now);
            transport.Send(payload, now);

            while (transport.TryReceive(now, destination, out _, out long arrivalTicks))
            {
                long sendTick = System.BitConverter.ToInt64(destination, 0);
                delays.Add(arrivalTicks - sendTick);
            }
        }

        // Discard the fill transient: the first datagrams cross an empty queue.
        var steady = delays.Skip(delays.Count / 2).ToList();
        Assert.All(steady, d => Assert.InRange(d, expectedStandingDelay - serialization, expectedStandingDelay));
    }

    [Fact]
    public void FullQueue_RefusesAtSend_AndForwardsNothing()
    {
        var inner = new LoopbackTransport(PayloadBytes, 256);
        var transport = new BottleneckTransport(inner, OfferedBytesPerSecond / 10, bufferDatagrams: 3, TicksPerSecond);

        // All four sent at the same instant: three fit, the fourth is tail-dropped.
        Assert.True(transport.Send(Payload(1), 0));
        Assert.True(transport.Send(Payload(2), 0));
        Assert.True(transport.Send(Payload(3), 0));
        Assert.False(transport.Send(Payload(4), 0));

        // Nothing has departed yet, so the wrapped transport has seen nothing at all -- the refused
        // datagram never put bytes on the wire, which is the property docs/adr/0013 protects.
        Assert.Equal(0, inner.QueuedCount);
        Assert.Equal(3, transport.QueuedCount);
    }

    [Fact]
    public void OversizedPayload_IsRefusedWithoutConsumingQueue()
    {
        var transport = Build(OfferedBytesPerSecond, bufferDatagrams: 4);
        Assert.False(transport.Send(new byte[PayloadBytes + 1], 0));
        Assert.Equal(0, transport.QueuedCount);
    }

    [Fact]
    public void QueueDrainsOnSend_NotOnlyOnPoll()
    {
        // A host that never polls must still see the link drain, or acceptance would depend on the
        // polling schedule rather than on the link -- the same class of error as folding poll time
        // into measured one-way delay.
        var transport = Build(OfferedBytesPerSecond, bufferDatagrams: 2);
        long serialization = transport.SerializationTicks(PayloadBytes);

        Assert.True(transport.Send(Payload(1), 0));
        Assert.True(transport.Send(Payload(2), 0));
        Assert.False(transport.Send(Payload(3), 0));

        // One service time later the first has departed, freeing a slot -- with no TryReceive call
        // in between.
        Assert.True(transport.Send(Payload(4), serialization));
    }

    [Fact]
    public void ArrivalTickIsTheDepartureTick_NotThePollTick()
    {
        var transport = Build(OfferedBytesPerSecond, bufferDatagrams: 4);
        long serialization = transport.SerializationTicks(PayloadBytes);
        var destination = new byte[PayloadBytes];

        Assert.True(transport.Send(Payload(1), 0));

        // Poll very late. The reported arrival must still be the departure instant.
        Assert.True(transport.TryReceive(50_000_000L, destination, out _, out long arrivalTicks));
        Assert.Equal(serialization, arrivalTicks);
    }

    [Fact]
    public void ComposesUnderEmulatedTransport_DelaysAdd()
    {
        // The intended stack: EmulatedTransport(BottleneckTransport(Loopback)). The bottleneck's
        // serialization plus the path's fixed delay must sum, with no special case in either class.
        const long fixedDelayTicks = 500_000; // 50 ms
        var bottleneck = Build(OfferedBytesPerSecond, bufferDatagrams: 8);
        var impairments = new INetworkImpairment[] { new FixedDelayImpairment(fixedDelayTicks) };
        var link = new EmulatedTransport(bottleneck, impairments, seed: 1, maxInFlight: 64);

        long serialization = bottleneck.SerializationTicks(PayloadBytes);
        var destination = new byte[PayloadBytes];

        Assert.True(link.Send(Payload(1), 0));
        Assert.False(link.TryReceive(serialization + fixedDelayTicks - 1, destination, out _, out _));
        Assert.True(link.TryReceive(serialization + fixedDelayTicks, destination, out _, out long arrivalTicks));
        Assert.Equal(serialization + fixedDelayTicks, arrivalTicks);
    }

    [Fact]
    public void Reset_ClearsBacklogAndResetsInner()
    {
        var inner = new LoopbackTransport(PayloadBytes, 256);
        var transport = new BottleneckTransport(inner, OfferedBytesPerSecond / 10, bufferDatagrams: 4, TicksPerSecond);

        transport.Send(Payload(1), 0);
        transport.Send(Payload(2), 0);
        Assert.Equal(2, transport.QueuedCount);

        transport.Reset();

        Assert.Equal(0, transport.QueuedCount);
        Assert.Equal(0, inner.QueuedCount);

        // And the departure schedule is cleared: the first datagram after a reset is serialized from
        // its own arrival, not from the previous trial's backlog.
        var destination = new byte[PayloadBytes];
        long serialization = transport.SerializationTicks(PayloadBytes);
        Assert.True(transport.Send(Payload(3), 0));
        Assert.True(transport.TryReceive(serialization, destination, out _, out long arrivalTicks));
        Assert.Equal(serialization, arrivalTicks);
    }

    [Fact]
    public void ResetMakesTrialsReproducible()
    {
        var transport = Build(OfferedBytesPerSecond / 2, bufferDatagrams: 6);
        var first = RunTrial(transport);
        transport.Reset();
        var second = RunTrial(transport);
        Assert.Equal(first, second);
    }

    private static List<long> RunTrial(BottleneckTransport transport)
    {
        var destination = new byte[PayloadBytes];
        var arrivals = new List<long>();
        for (int step = 0; step < 100; step++)
        {
            long now = step * 100_000L;
            transport.Send(new byte[PayloadBytes], now);
            while (transport.TryReceive(now, destination, out _, out long arrivalTicks))
            {
                arrivals.Add(arrivalTicks);
            }
        }

        return arrivals;
    }

    [Theory]
    [InlineData(0, 4, TicksPerSecond)]
    [InlineData(-1, 4, TicksPerSecond)]
    [InlineData(7300, 0, TicksPerSecond)]
    [InlineData(7300, -1, TicksPerSecond)]
    [InlineData(7300, 4, 0)]
    public void InvalidParameters_Throw(long capacity, int bufferDatagrams, long ticksPerSecond)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new BottleneckTransport(
            new LoopbackTransport(PayloadBytes, 4), capacity, bufferDatagrams, ticksPerSecond));
    }

    [Fact]
    public void NullInner_Throws()
    {
        Assert.Throws<ArgumentNullException>(() =>
            new BottleneckTransport(null!, 7300, 4, TicksPerSecond));
    }

    [Fact]
    public void Send_AllocatesNothing()
    {
        // The queue is deliberately left full so the measured path is the tail-drop branch as well
        // as the enqueue branch across iterations.
        var transport = Build(OfferedBytesPerSecond, bufferDatagrams: 8);
        var payload = Payload(7);
        AllocationAssert.Zero(() => transport.Send(payload, 0));
    }

    [Fact]
    public void TryReceive_AllocatesNothing()
    {
        var transport = Build(OfferedBytesPerSecond, bufferDatagrams: 8);
        var destination = new byte[PayloadBytes];
        transport.Send(Payload(7), 0);
        AllocationAssert.Zero(() => transport.TryReceive(1_000_000, destination, out _, out _));
    }

}
