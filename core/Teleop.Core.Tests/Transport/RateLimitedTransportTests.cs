using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Transport;

namespace Teleop.Core.Tests.Transport;

/// <summary>
/// <see cref="RateLimitedTransport"/>: open-loop sender-side admission control.
///
/// The claims worth pinning are that it admits at the configured rate and no faster, that it is a
/// true no-op when its rate is above the offered load (so it cannot hurt an uncongested link), that
/// a refusal forwards nothing at all, and that its bucket arithmetic does not drift over a long run
/// — the last being the reason credit is kept in ticks rather than bytes.
/// </summary>
public sealed class RateLimitedTransportTests
{
    private const long TicksPerSecond = 10_000_000;
    private const int PayloadBytes = 73;
    private const long OfferedBytesPerSecond = 7300;
    private const long StepTicks = 100_000; // 10 ms

    private static byte[] Payload() => new byte[PayloadBytes];

    [Fact]
    public void AboveOfferedLoad_IsANoOp()
    {
        // The falsifier for the whole candidate: on an uncongested link this mechanism must not
        // cost a single command.
        var inner = new LoopbackTransport(PayloadBytes, 4096);
        var limiter = new RateLimitedTransport(inner, OfferedBytesPerSecond * 2, PayloadBytes, TicksPerSecond);

        int accepted = 0;
        for (int step = 0; step < 1000; step++)
        {
            if (limiter.Send(Payload(), step * StepTicks))
            {
                accepted++;
            }
        }

        Assert.Equal(1000, accepted);
        Assert.Equal(0, limiter.RefusedCount);
    }

    [Fact]
    public void AtExactlyTheOfferedLoad_AdmitsEverything()
    {
        var inner = new LoopbackTransport(PayloadBytes, 4096);
        var limiter = new RateLimitedTransport(inner, OfferedBytesPerSecond, PayloadBytes, TicksPerSecond);

        int accepted = 0;
        for (int step = 0; step < 1000; step++)
        {
            if (limiter.Send(Payload(), step * StepTicks))
            {
                accepted++;
            }
        }

        Assert.Equal(1000, accepted);
    }

    [Fact]
    public void AtHalfTheOfferedLoad_AdmitsHalf_WithNoDrift()
    {
        // 1000 offers, half admitted. Exactness here is the point: a bytes-denominated bucket
        // refilled by elapsed*rate/ticksPerSecond truncates on every call and would come in short.
        var inner = new LoopbackTransport(PayloadBytes, 4096);
        var limiter = new RateLimitedTransport(inner, OfferedBytesPerSecond / 2, PayloadBytes, TicksPerSecond);

        int accepted = 0;
        for (int step = 0; step < 1000; step++)
        {
            if (limiter.Send(Payload(), step * StepTicks))
            {
                accepted++;
            }
        }

        // One bucket's worth of head start, then exactly one admission per two steps.
        Assert.InRange(accepted, 500, 502);
        Assert.Equal(1000 - accepted, limiter.RefusedCount);
    }

    [Fact]
    public void Refusal_ForwardsNothing()
    {
        var inner = new LoopbackTransport(PayloadBytes, 4096);
        var limiter = new RateLimitedTransport(inner, OfferedBytesPerSecond / 4, PayloadBytes, TicksPerSecond);

        Assert.True(limiter.Send(Payload(), 0));
        Assert.False(limiter.Send(Payload(), 0));

        // Exactly one datagram reached the wrapped transport. The refused one never did, which is
        // the entire mechanism: it cannot contribute to a downstream backlog.
        Assert.Equal(1, inner.QueuedCount);
    }

    [Fact]
    public void BurstDepthCapsAccumulatedCredit()
    {
        var inner = new LoopbackTransport(PayloadBytes, 4096);
        var limiter = new RateLimitedTransport(inner, OfferedBytesPerSecond, PayloadBytes * 2, TicksPerSecond);

        // Long silence: credit accumulates but is capped at two datagrams.
        Assert.True(limiter.Send(Payload(), 100_000_000L));
        Assert.True(limiter.Send(Payload(), 100_000_000L));
        Assert.False(limiter.Send(Payload(), 100_000_000L));
    }

    [Fact]
    public void Reset_RestoresFullBucketAndResetsInner()
    {
        var inner = new LoopbackTransport(PayloadBytes, 4096);
        var limiter = new RateLimitedTransport(inner, OfferedBytesPerSecond / 4, PayloadBytes, TicksPerSecond);

        Assert.True(limiter.Send(Payload(), 0));
        Assert.False(limiter.Send(Payload(), 0));
        Assert.Equal(1, limiter.RefusedCount);

        limiter.Reset();

        Assert.Equal(0, limiter.RefusedCount);
        Assert.Equal(0, inner.QueuedCount);
        Assert.True(limiter.Send(Payload(), 0));
    }

    [Fact]
    public void TryReceive_DelegatesToInner()
    {
        var inner = new LoopbackTransport(PayloadBytes, 4096);
        var limiter = new RateLimitedTransport(inner, OfferedBytesPerSecond, PayloadBytes, TicksPerSecond);
        var destination = new byte[PayloadBytes];

        Assert.True(limiter.Send(Payload(), 42));
        Assert.True(limiter.TryReceive(42, destination, out int byteCount, out long arrivalTicks));
        Assert.Equal(PayloadBytes, byteCount);
        Assert.Equal(42, arrivalTicks);
    }

    [Theory]
    [InlineData(0, 73, TicksPerSecond)]
    [InlineData(-1, 73, TicksPerSecond)]
    [InlineData(7300, 0, TicksPerSecond)]
    [InlineData(7300, 73, 0)]
    public void InvalidParameters_Throw(long admitRate, long burstBytes, long ticksPerSecond)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new RateLimitedTransport(
            new LoopbackTransport(PayloadBytes, 4), admitRate, burstBytes, ticksPerSecond));
    }

    [Fact]
    public void NullInner_Throws()
    {
        Assert.Throws<ArgumentNullException>(() =>
            new RateLimitedTransport(null!, 7300, 73, TicksPerSecond));
    }

    [Fact]
    public void Send_AllocatesNothing()
    {
        var inner = new LoopbackTransport(PayloadBytes, 4096);
        var limiter = new RateLimitedTransport(inner, OfferedBytesPerSecond, PayloadBytes, TicksPerSecond);
        var payload = Payload();
        AllocationAssert.Zero(() => limiter.Send(payload, 0));
    }
}
