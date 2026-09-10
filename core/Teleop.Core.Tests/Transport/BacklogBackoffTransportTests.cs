using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Transport;

namespace Teleop.Core.Tests.Transport;

/// <summary>
/// <see cref="BacklogBackoffTransport"/>: closed-loop admission control driven by the only signal a
/// transport decorator can see, the wrapped transport returning false.
///
/// What is pinned here is the control law itself — starts greedy, multiplicative decrease of the
/// admitted rate on a refusal, additive increase after a run of successes, bounded by
/// <c>maxGap</c> — plus the two properties that decide whether it is safe to deploy: it is a no-op
/// on a link that never refuses, and it cannot ratchet itself to silence.
/// </summary>
public sealed class BacklogBackoffTransportTests
{
    private const int PayloadBytes = 73;

    private static byte[] Payload() => new byte[PayloadBytes];

    /// <summary>A transport that refuses on demand, so the control law can be driven directly.</summary>
    private sealed class ScriptedTransport : Teleop.Core.Contracts.ITransport
    {
        public bool RefuseNext { get; set; }

        public int SendCount { get; private set; }

        public int MaxPayloadBytes => PayloadBytes;

        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            SendCount++;
            return !RefuseNext;
        }

        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            byteCount = 0;
            arrivalTicks = 7;
            return false;
        }

        public void Reset() => SendCount = 0;
    }

    [Fact]
    public void StartsGreedy_AndStaysGreedyOnALinkThatNeverRefuses()
    {
        // The safety property: on an uncongested link this controller must cost nothing.
        var inner = new ScriptedTransport();
        var controller = new BacklogBackoffTransport(inner, maxGap: 32, successesBeforeDecrease: 4);

        for (int i = 0; i < 500; i++)
        {
            Assert.True(controller.Send(Payload(), i * 100_000L));
        }

        Assert.Equal(0, controller.Gap);
        Assert.Equal(0, controller.RefusedCount);
        Assert.Equal(500, inner.SendCount);
    }

    [Fact]
    public void RefusalGrowsTheGapMultiplicatively()
    {
        var inner = new ScriptedTransport();
        var controller = new BacklogBackoffTransport(inner, maxGap: 64, successesBeforeDecrease: 1000);

        inner.RefuseNext = true;

        Assert.False(controller.Send(Payload(), 0));
        Assert.Equal(1, controller.Gap); // 2*0 + 1

        // The next offer is withheld by the controller: it does not reach the inner transport.
        int before = inner.SendCount;
        Assert.False(controller.Send(Payload(), 100_000));
        Assert.Equal(before, inner.SendCount);

        // Then it probes again, is refused again, and the gap grows 2*1 + 1 = 3.
        Assert.False(controller.Send(Payload(), 200_000));
        Assert.Equal(3, controller.Gap);
    }

    [Fact]
    public void SuccessRunShrinksTheGapAdditively()
    {
        var inner = new ScriptedTransport();
        var controller = new BacklogBackoffTransport(inner, maxGap: 64, successesBeforeDecrease: 2);

        inner.RefuseNext = true;
        controller.Send(Payload(), 0);
        controller.Send(Payload(), 100_000); // withheld
        controller.Send(Payload(), 200_000); // probe, refused -> gap 3
        Assert.Equal(3, controller.Gap);

        inner.RefuseNext = false;

        // Drive many offers. Each admitted one counts toward the success run; after two, gap drops
        // by one. Eventually it reaches zero and stays there.
        for (int i = 0; i < 200; i++)
        {
            controller.Send(Payload(), (300_000L + i) * 100_000L);
        }

        Assert.Equal(0, controller.Gap);
    }

    [Fact]
    public void GapIsBounded_SoTheSenderCannotRatchetItselfToSilence()
    {
        var inner = new ScriptedTransport { RefuseNext = true };
        var controller = new BacklogBackoffTransport(inner, maxGap: 7, successesBeforeDecrease: 1000);

        for (int i = 0; i < 1000; i++)
        {
            controller.Send(Payload(), i * 100_000L);
        }

        Assert.Equal(7, controller.Gap);

        // And it still probes: at least one in eight offers reaches the link.
        int probes = inner.SendCount;
        Assert.InRange(probes, 1000 / 8 - 2, 1000 / 8 + 2);
    }

    [Fact]
    public void RefusedCountExcludesTheLinksOwnRefusals()
    {
        // The controller's cost is what *it* withheld. A datagram it forwarded and the link then
        // dropped is the link's loss, and folding the two together would let the mechanism hide its
        // own bill.
        var inner = new ScriptedTransport { RefuseNext = true };
        var controller = new BacklogBackoffTransport(inner, maxGap: 1, successesBeforeDecrease: 1000);

        controller.Send(Payload(), 0);        // forwarded, link refuses -> not counted, gap -> 1
        controller.Send(Payload(), 100_000);  // withheld by controller -> counted
        controller.Send(Payload(), 200_000);  // forwarded, link refuses -> not counted

        Assert.Equal(1, controller.RefusedCount);
    }

    [Fact]
    public void Reset_ReturnsToGreedyAndResetsInner()
    {
        var inner = new ScriptedTransport { RefuseNext = true };
        var controller = new BacklogBackoffTransport(inner, maxGap: 16, successesBeforeDecrease: 4);

        controller.Send(Payload(), 0);
        controller.Send(Payload(), 100_000);
        Assert.True(controller.Gap > 0);

        controller.Reset();

        Assert.Equal(0, controller.Gap);
        Assert.Equal(0, controller.RefusedCount);
        Assert.Equal(0, inner.SendCount);
    }

    [Fact]
    public void TryReceive_DelegatesToInner()
    {
        var inner = new ScriptedTransport();
        var controller = new BacklogBackoffTransport(inner, maxGap: 4, successesBeforeDecrease: 4);
        var destination = new byte[PayloadBytes];

        Assert.False(controller.TryReceive(0, destination, out int byteCount, out long arrivalTicks));
        Assert.Equal(0, byteCount);
        Assert.Equal(7, arrivalTicks); // proves the out parameters came from the inner transport
    }

    [Theory]
    [InlineData(-1, 4)]
    [InlineData(4, 0)]
    [InlineData(4, -1)]
    public void InvalidParameters_Throw(int maxGap, int successesBeforeDecrease)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new BacklogBackoffTransport(
            new ScriptedTransport(), maxGap, successesBeforeDecrease));
    }

    [Fact]
    public void NullInner_Throws()
    {
        Assert.Throws<ArgumentNullException>(() => new BacklogBackoffTransport(null!, 4, 4));
    }

    [Fact]
    public void Send_AllocatesNothing()
    {
        var inner = new ScriptedTransport();
        var controller = new BacklogBackoffTransport(inner, maxGap: 8, successesBeforeDecrease: 4);
        var payload = Payload();
        AllocationAssert.Zero(() => controller.Send(payload, 0));
    }
}
