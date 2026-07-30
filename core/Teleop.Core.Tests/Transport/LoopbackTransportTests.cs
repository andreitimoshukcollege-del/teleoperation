using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Transport;

namespace Teleop.Core.Tests.Transport;

public class LoopbackTransportTests
{
    private const int MaxPayload = 16;

    private static byte[] Payload(params byte[] bytes) => bytes;

    [Fact]
    public void Constructor_ReportsMaxPayloadAndCapacityAndIsEmpty()
    {
        var transport = new LoopbackTransport(maxPayloadBytes: MaxPayload, capacity: 4);

        Assert.Equal(MaxPayload, transport.MaxPayloadBytes);
        Assert.Equal(4, transport.Capacity);
        Assert.Equal(0, transport.QueuedCount);
    }

    [Fact]
    public void Constructor_RejectsNonPositiveSizes()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new LoopbackTransport(0, 4));
        Assert.Throws<ArgumentOutOfRangeException>(() => new LoopbackTransport(MaxPayload, 0));
    }

    [Fact]
    public void SendThenReceive_PreservesBytesAndReportsSendTickAsArrival()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 4);
        byte[] sent = Payload(1, 2, 3, 250);
        var destination = new byte[MaxPayload];

        Assert.True(transport.Send(sent, nowTicks: 1234));
        Assert.Equal(1, transport.QueuedCount);

        Assert.True(transport.TryReceive(5678, destination, out int byteCount, out long arrivalTicks));

        Assert.Equal(sent.Length, byteCount);
        Assert.Equal(sent, destination.AsSpan(0, byteCount).ToArray());

        // The whole point of the loopback baseline: zero added delay, so arrival == send, and it
        // is NOT the poll tick (5678). A nonzero one-way delay here would be a measurement bug.
        Assert.Equal(1234, arrivalTicks);
        Assert.Equal(0, transport.QueuedCount);
    }

    [Fact]
    public void TryReceive_OnEmptyChannel_ReturnsFalseWithZeroedOutputs()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 4);
        var destination = new byte[MaxPayload];

        Assert.False(transport.TryReceive(0, destination, out int byteCount, out long arrivalTicks));
        Assert.Equal(0, byteCount);
        Assert.Equal(0, arrivalTicks);
    }

    [Fact]
    public void TryReceive_NeverReturnsTheSameDatagramTwice()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 4);
        var destination = new byte[MaxPayload];
        transport.Send(Payload(9), 0);

        Assert.True(transport.TryReceive(0, destination, out _, out _));
        Assert.False(transport.TryReceive(0, destination, out _, out _));
    }

    [Fact]
    public void Delivery_IsFifoAndSurvivesRingWrapAround()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 3);
        var destination = new byte[MaxPayload];

        // Two full laps of a 3-slot ring, interleaved so the head and tail both wrap.
        for (byte i = 0; i < 9; i++)
        {
            Assert.True(transport.Send(Payload(i), i));
            Assert.True(transport.TryReceive(i, destination, out int byteCount, out long arrivalTicks));
            Assert.Equal(1, byteCount);
            Assert.Equal(i, destination[0]);
            Assert.Equal(i, arrivalTicks);
        }
    }

    [Fact]
    public void Send_WhenRingIsFull_ReturnsFalseAndLeavesQueuedDatagramsIntact()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 2);
        var destination = new byte[MaxPayload];

        Assert.True(transport.Send(Payload(1), 10));
        Assert.True(transport.Send(Payload(2), 20));

        // Full send queue: ITransport.Send documents false as an ordinary outcome, not a throw.
        Assert.False(transport.Send(Payload(3), 30));
        Assert.Equal(2, transport.QueuedCount);

        Assert.True(transport.TryReceive(30, destination, out _, out _));
        Assert.Equal(1, destination[0]);
        Assert.True(transport.TryReceive(30, destination, out _, out _));
        Assert.Equal(2, destination[0]);
        Assert.False(transport.TryReceive(30, destination, out _, out _));
    }

    [Fact]
    public void Send_PayloadLargerThanMax_ReturnsFalseRatherThanTruncatingOrThrowing()
    {
        var transport = new LoopbackTransport(maxPayloadBytes: 4, capacity: 2);

        Assert.False(transport.Send(new byte[5], 0));
        Assert.Equal(0, transport.QueuedCount);
    }

    [Fact]
    public void TryReceive_DestinationTooSmall_ReportsRequiredLengthAndKeepsDatagramQueued()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 4);
        byte[] sent = Payload(7, 7, 7, 7, 7, 7);
        transport.Send(sent, nowTicks: 99);

        var tooSmall = new byte[3];
        Assert.False(transport.TryReceive(100, tooSmall, out int required, out long arrivalTicks));
        Assert.Equal(sent.Length, required);
        Assert.Equal(0, arrivalTicks);
        Assert.Equal(1, transport.QueuedCount);

        // Still there: a short buffer must not consume the datagram.
        var destination = new byte[MaxPayload];
        Assert.True(transport.TryReceive(100, destination, out int byteCount, out arrivalTicks));
        Assert.Equal(sent, destination.AsSpan(0, byteCount).ToArray());
        Assert.Equal(99, arrivalTicks);
    }

    [Fact]
    public void TryReceive_DoesNotSurfaceADatagramSentAfterNow()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 4);
        transport.Send(Payload(1), nowTicks: 500);

        Assert.False(transport.TryReceive(499, new byte[MaxPayload], out _, out _));
        Assert.True(transport.TryReceive(500, new byte[MaxPayload], out _, out long arrivalTicks));
        Assert.Equal(500, arrivalTicks);
    }

    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 3);
        var destination = new byte[MaxPayload];

        // Leave the ring mid-lap with a datagram still queued, so Reset has cursors to fix.
        transport.Send(Payload(1), 1);
        transport.Send(Payload(2), 2);
        transport.TryReceive(2, destination, out _, out _);
        transport.Send(Payload(3), 3);
        Assert.Equal(2, transport.QueuedCount);

        transport.Reset();

        Assert.Equal(0, transport.QueuedCount);
        Assert.Equal(MaxPayload, transport.MaxPayloadBytes);
        Assert.Equal(3, transport.Capacity);
        Assert.False(transport.TryReceive(long.MaxValue, destination, out int byteCount, out long arrivalTicks));
        Assert.Equal(0, byteCount);
        Assert.Equal(0, arrivalTicks);

        // And behaves exactly like a freshly constructed instance from here on.
        var fresh = new LoopbackTransport(MaxPayload, capacity: 3);
        for (byte i = 0; i < 5; i++)
        {
            Assert.Equal(fresh.Send(Payload(i), i), transport.Send(Payload(i), i));
        }

        var afterReset = new byte[MaxPayload];
        var freshDestination = new byte[MaxPayload];
        while (fresh.TryReceive(100, freshDestination, out int freshCount, out long freshArrival))
        {
            Assert.True(transport.TryReceive(100, afterReset, out int count, out long arrival));
            Assert.Equal(freshCount, count);
            Assert.Equal(freshArrival, arrival);
            Assert.Equal(freshDestination, afterReset);
        }

        Assert.False(transport.TryReceive(100, afterReset, out _, out _));
    }

    [Fact]
    public void SendAndTryReceive_Allocate_Zero_Bytes()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 8);
        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(0, destination, out _, out _);
        });
    }

    [Fact]
    public void Send_WhenFull_Allocates_Zero_Bytes()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 1);
        var payload = new byte[MaxPayload];
        Assert.True(transport.Send(payload, 0));

        AllocationAssert.Zero(() => transport.Send(payload, 0));
    }

    [Fact]
    public void TryReceive_OnEmptyChannel_Allocates_Zero_Bytes()
    {
        var transport = new LoopbackTransport(MaxPayload, capacity: 8);
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() => transport.TryReceive(0, destination, out _, out _));
    }
}
