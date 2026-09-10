using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Transport;

/// <summary>
/// Two things are proved here. First, that the decorator is <b>transparent</b> — the wrapped
/// transport's behaviour, return values and arrival ticks survive it unchanged, because a measuring
/// device that perturbs the thing it measures is worse than none. Second, that the loss figures it
/// produces match a <b>generative model whose parameters are known</b>: the frozen
/// <c>300ms-60j-2loss-bursty</c> profile declares a 2% steady-state loss rate and an expected burst
/// length of 1/(1−0.7) = 3.33, and those are what the measurement is checked against — not against
/// whatever the code happens to produce.
/// </summary>
public class MeasuredTransportTests
{
    private const long TicksPerSecond = 10_000_000;
    private const int MaxPayload = 32;

    private sealed class RecordingSink : IMetricSink
    {
        public readonly List<(string Name, double Value, long Ticks)> Samples = new();

        public void Record(string name, double value, long ticks) => Samples.Add((name, value, ticks));

        public IEnumerable<double> ValuesOf(string name) =>
            Samples.Where(s => s.Name == name).Select(s => s.Value);

        public int CountOf(string name) => Samples.Count(s => s.Name == name);
    }

    private static NetworkObserver UplinkObserver(IMetricSink sink) =>
        NetworkObserver.ForUplink(sink, new ManualClock(TicksPerSecond));

    // ---------------------------------------------------------------------------------------
    // Transparency.
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void ForwardsMaxPayloadBytesFromTheWrappedTransport()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 4);
        var measured = new MeasuredTransport(inner, UplinkObserver(new NullMetricSink()));

        Assert.Equal(inner.MaxPayloadBytes, measured.MaxPayloadBytes);
    }

    [Fact]
    public void APayloadSurvivesTheDecoratorByteForByteWithItsArrivalTick()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 4);
        var measured = new MeasuredTransport(inner, UplinkObserver(new NullMetricSink()));

        byte[] payload = { 1, 2, 3, 250 };
        Assert.True(measured.Send(payload, nowTicks: 4242));

        var destination = new byte[MaxPayload];
        Assert.True(measured.TryReceive(9999, destination, out int byteCount, out long arrivalTicks));
        Assert.Equal(payload.Length, byteCount);
        Assert.Equal(payload, destination.AsSpan(0, byteCount).ToArray());
        Assert.Equal(4242, arrivalTicks);
    }

    [Fact]
    public void ATooShortDestinationIsNotCountedAsADeliveryAndTheDatagramStaysQueued()
    {
        // ITransport's too-short-buffer protocol returns false, reports the required length, and
        // leaves the datagram queued. Counting that as a delivery would count the same datagram
        // twice once the caller retried with a larger buffer.
        var sink = new RecordingSink();
        var inner = new LoopbackTransport(MaxPayload, capacity: 4);
        var measured = new MeasuredTransport(inner, UplinkObserver(sink));

        byte[] payload = { 1, 2, 3, 4, 5 };
        Assert.True(measured.Send(payload, nowTicks: 10));

        Assert.False(measured.TryReceive(20, new byte[2], out int required, out _));
        Assert.Equal(payload.Length, required);
        Assert.Equal(0, sink.CountOf("net_uplink_received"));

        Assert.True(measured.TryReceive(20, new byte[MaxPayload], out _, out _));
        Assert.Equal(1, sink.CountOf("net_uplink_received"));
    }

    [Fact]
    public void AnEmptyChannelIsNotCountedAsADelivery()
    {
        var sink = new RecordingSink();
        var measured = new MeasuredTransport(
            new LoopbackTransport(MaxPayload, capacity: 4), UplinkObserver(sink));

        Assert.False(measured.TryReceive(0, new byte[MaxPayload], out _, out _));
        Assert.Empty(sink.Samples);
    }

    [Fact]
    public void DecoratingChangesNothingAboutTheWrappedLinksRealization()
    {
        // The measurement must not perturb the measured. Same seed, same profile, same call
        // sequence: with and without the decorator the delivered bytes and arrival ticks must be
        // identical, which also means the RNG draw order is untouched.
        NetworkProfileCatalog.TryResolveParametric(
            "300ms-60j-2loss-bursty", TicksPerSecond, out NetworkProfile profile, out _);

        List<(int Length, long Arrival)> Run(bool decorate)
        {
            ITransport transport = new EmulatedTransport(
                new LoopbackTransport(MaxPayload, capacity: 64),
                NetworkProfileCatalog.CreateImpairments(profile), seed: 7, maxInFlight: 64);
            if (decorate)
            {
                transport = new MeasuredTransport(transport, UplinkObserver(new NullMetricSink()));
            }

            var observed = new List<(int, long)>();
            var destination = new byte[MaxPayload];
            var payload = new byte[8];
            for (int step = 0; step < 400; step++)
            {
                long now = step * (TicksPerSecond / 100);
                payload[0] = (byte)step;
                transport.Send(payload, now);
                while (transport.TryReceive(now, destination, out int byteCount, out long arrival))
                {
                    observed.Add((byteCount, arrival));
                }
            }

            return observed;
        }

        Assert.Equal(Run(decorate: false), Run(decorate: true));
    }

    // ---------------------------------------------------------------------------------------
    // Loss, against a known generative model.
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void ALinkThatLosesNothingReportsNoDrops()
    {
        var sink = new RecordingSink();
        var measured = new MeasuredTransport(
            new LoopbackTransport(MaxPayload, capacity: 64), UplinkObserver(sink));

        var payload = new byte[4];
        var destination = new byte[MaxPayload];
        for (int i = 0; i < 100; i++)
        {
            measured.Send(payload, i);
            measured.TryReceive(i, destination, out _, out _);
        }

        Assert.Equal(100, sink.CountOf("net_uplink_sent"));
        Assert.Equal(100, sink.CountOf("net_uplink_received"));
        Assert.Equal(0, sink.CountOf("net_uplink_dropped"));
    }

    [Fact]
    public void ALinkThatLosesEverythingReportsEverySendAsDroppedAndDeliversNothing()
    {
        // Probability 1.0 with SeededRng.NextDouble() uniform on [0,1) always fires, so this is an
        // exact expectation, not a statistical one.
        var sink = new RecordingSink();
        var profile = new NetworkProfile(
            baseDelayTicks: 0, jitterTicks: 0,
            lossProbabilityAfterDelivered: 1.0, lossProbabilityAfterLost: 1.0,
            reorderProbability: 0.0, reorderDelayTicks: 0);
        var measured = new MeasuredTransport(
            new EmulatedTransport(
                new LoopbackTransport(MaxPayload, capacity: 64),
                NetworkProfileCatalog.CreateImpairments(profile), seed: 3, maxInFlight: 64),
            UplinkObserver(sink));

        var payload = new byte[4];
        var destination = new byte[MaxPayload];
        for (int i = 0; i < 50; i++)
        {
            measured.Send(payload, i);
            measured.TryReceive(i, destination, out _, out _);
        }

        Assert.Equal(50, sink.CountOf("net_uplink_dropped"));
        Assert.Equal(0, sink.CountOf("net_uplink_sent"));
        Assert.Equal(0, sink.CountOf("net_uplink_received"));

        // One run of 50, still open at the end, so nothing is emitted -- the documented censoring.
        Assert.Equal(0, sink.CountOf("net_uplink_loss_burst"));
    }

    [Fact]
    public void MeasuredLossOnTheFrozenBurstyProfileMatchesItsDeclaredGenerativeModel()
    {
        // 300ms-60j-2loss-bursty declares p(lost|delivered)=0.00612 and p(lost|lost)=0.7, i.e. a
        // steady-state loss rate of 0.00612/(0.00612+0.3) = 2.0% and a geometric burst length with
        // mean 1/(1-0.7) = 3.33.
        //
        // The bounds below are set from the sampling distribution, not from the observed value.
        // With 20000 sends there are ~400 losses in ~120 bursts.
        //
        //   Burst length: geometric(0.3), mean 3.33, sd 2.79; se of the mean over ~120 bursts is
        //   0.25, so +/-3 se is +/-0.76 -> [2.5, 4.2].
        //
        //   Loss rate: NOT the Bernoulli se, which is the mistake made on the first pass here.
        //   Losses arrive in correlated runs, so var(total) ~ B*var(L) + E[L]^2*var(B) ~
        //   120*7.78 + 11.1*120, giving sd ~48 losses = 0.0024 as a rate; +/-3 se around 0.02 is
        //   [0.013, 0.027]. The first pass wrote [0.015, 0.026] from the uncorrelated variance and
        //   the observed 0.0156 landed just inside it by luck. Widened on the corrected
        //   derivation, not to make anything pass -- and recorded in
        //   docs/research-log/2026-09-09-network-observability.md, because silently widening a
        //   tolerance after seeing a result is exactly what must not happen.
        NetworkProfileCatalog.TryResolveParametric(
            "300ms-60j-2loss-bursty", TicksPerSecond, out NetworkProfile profile, out _);

        // Loss only: the delay and jitter axes would fill the in-flight heap and add refusals from
        // back-pressure, which is a real effect but a different one, and this test is about the
        // loss model. (The characterization sweep measures the two together on purpose.)
        var lossOnly = new INetworkImpairment[]
        {
            new Teleop.Core.Transport.Impairments.GilbertElliottLossImpairment(
                profile.LossProbabilityAfterDelivered, profile.LossProbabilityAfterLost),
        };

        var sink = new RecordingSink();
        var measured = new MeasuredTransport(
            new EmulatedTransport(
                new LoopbackTransport(MaxPayload, capacity: 64), lossOnly, seed: 11, maxInFlight: 64),
            UplinkObserver(sink));

        const int Sends = 20_000;
        var payload = new byte[4];
        var destination = new byte[MaxPayload];
        for (int i = 0; i < Sends; i++)
        {
            measured.Send(payload, i);
            while (measured.TryReceive(i, destination, out _, out _))
            {
            }
        }

        int dropped = sink.CountOf("net_uplink_dropped");
        int sent = sink.CountOf("net_uplink_sent");
        Assert.Equal(Sends, dropped + sent);

        double lossRate = dropped / (double)Sends;
        Assert.InRange(lossRate, 0.013, 0.027);

        double[] bursts = sink.ValuesOf("net_uplink_loss_burst").ToArray();
        Assert.NotEmpty(bursts);
        Assert.All(bursts, b => Assert.True(b >= 1.0));
        Assert.InRange(bursts.Average(), 2.5, 4.2);

        // Bursty, not Bernoulli: a memoryless link at this rate would essentially never produce a
        // run of four, whereas this chain's runs are geometric with mean 3.33.
        Assert.Contains(bursts, b => b >= 4.0);

        // Every drop belongs to exactly one run, except at most one run left open at the end.
        double burstTotal = bursts.Sum();
        Assert.InRange(burstTotal, dropped - 60, dropped);

        // Nothing was destroyed that no impairment asked to destroy.
        Assert.Equal(sent, sink.CountOf("net_uplink_received"));
    }

    [Fact]
    public void AMemorylessLossProfileProducesShorterRunsThanTheBurstyOneAtTheSameRate()
    {
        // The distinction docs/metrics.md §3 says the rate alone cannot make: same 2% loss, two
        // completely different burst structures. This is what makes the burst metric worth having.
        double[] BurstsFor(double afterDelivered, double afterLost, ulong seed)
        {
            var sink = new RecordingSink();
            var impairments = new INetworkImpairment[]
            {
                new Teleop.Core.Transport.Impairments.GilbertElliottLossImpairment(afterDelivered, afterLost),
            };
            var measured = new MeasuredTransport(
                new EmulatedTransport(
                    new LoopbackTransport(MaxPayload, capacity: 64), impairments, seed, maxInFlight: 64),
                UplinkObserver(sink));

            var payload = new byte[4];
            var destination = new byte[MaxPayload];
            for (int i = 0; i < 20_000; i++)
            {
                measured.Send(payload, i);
                while (measured.TryReceive(i, destination, out _, out _))
                {
                }
            }

            return sink.ValuesOf("net_uplink_loss_burst").ToArray();
        }

        double[] bursty = BurstsFor(0.00612, 0.7, seed: 11);
        double[] memoryless = BurstsFor(0.02, 0.02, seed: 11);

        Assert.InRange(memoryless.Average(), 1.0, 1.1);
        Assert.True(
            bursty.Average() > 2.0 * memoryless.Average(),
            $"bursty mean {bursty.Average():F2} should be far above memoryless mean {memoryless.Average():F2}");
    }

    // ---------------------------------------------------------------------------------------
    // Reset and allocation.
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void ResetResetsBothTheObserverAndTheWrappedTransport()
    {
        var sink = new RecordingSink();
        var inner = new LoopbackTransport(MaxPayload, capacity: 4);
        var measured = new MeasuredTransport(inner, UplinkObserver(sink));

        Assert.True(measured.Send(new byte[4], 10));
        Assert.Equal(1, inner.QueuedCount);

        measured.Reset();

        Assert.Equal(0, inner.QueuedCount);
        Assert.False(measured.TryReceive(100, new byte[MaxPayload], out _, out _));
    }

    [Fact]
    public void ConstructionRejectsNullArguments()
    {
        Assert.Throws<ArgumentNullException>(
            () => new MeasuredTransport(null!, UplinkObserver(new NullMetricSink())));
        Assert.Throws<ArgumentNullException>(
            () => new MeasuredTransport(new LoopbackTransport(MaxPayload, 4), null!));
    }

    [Fact]
    public void SendAndTryReceiveAllocateNothing()
    {
        var measured = new MeasuredTransport(
            new LoopbackTransport(MaxPayload, capacity: 4),
            UplinkObserver(new NullMetricSink()));

        var payload = new byte[4];
        var destination = new byte[MaxPayload];
        long tick = 0;

        AllocationAssert.Zero(() =>
        {
            measured.Send(payload, tick);
            measured.TryReceive(tick, destination, out _, out _);
            tick++;
        });
    }
}
