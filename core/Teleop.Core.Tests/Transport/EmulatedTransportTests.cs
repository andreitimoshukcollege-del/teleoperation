using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Transport.Impairments;
using Teleop.Core.Contracts;
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


    /// <summary>
    /// Builds a transport from a <see cref="NetworkProfile"/>, through the same
    /// <c>CreateImpairments</c> path the frozen catalog uses.
    ///
    /// These tests were written against the profile struct, and keeping them expressed that way is
    /// deliberate: every assertion below still describes the same link it always described, so a
    /// failure here means the behaviour moved, not that the test was rewritten to match new
    /// behaviour. What changed underneath is that each axis now draws from its own substream
    /// (docs/adr/0013), so tests asserting a *specific* random realization -- rather than a bound,
    /// a rate, or an exact arithmetic result -- necessarily see different draws than before.
    /// </summary>
    private static EmulatedTransport Emulate(
        ITransport inner, NetworkProfile profile, ulong seed, int maxInFlight) =>
        new EmulatedTransport(
            inner, NetworkProfileCatalog.CreateImpairments(profile), seed, maxInFlight);

    /// <summary>
    /// Trace-driven build: the trace axis plus whatever loss/reorder the profile carries.
    ///
    /// There is no longer a separate trace-mode constructor to reject a profile carrying base delay
    /// or jitter -- with separate impairments there is nothing to contradict. A caller wanting both
    /// simply includes both, and the delays sum.
    /// </summary>
    private static EmulatedTransport EmulateTrace(
        ITransport inner, long[] trace, NetworkProfile profile, ulong seed, int maxInFlight)
    {
        INetworkImpairment[] parametric = NetworkProfileCatalog.CreateImpairments(profile);
        var impairments = new INetworkImpairment[parametric.Length + 1];
        impairments[0] = NetworkProfileCatalog.CreateTraceDelay(trace);
        System.Array.Copy(parametric, 0, impairments, 1, parametric.Length);
        return new EmulatedTransport(inner, impairments, seed, maxInFlight);
    }


    /// <summary>
    /// <b>The headline property of docs/adr/0013, and one the previous design could not have
    /// satisfied.</b>
    ///
    /// Before, every datagram consumed exactly three draws from one shared stream in a fixed order.
    /// Adding a fourth axis, or removing one, shifted every subsequent draw for every other axis, so
    /// two runs differing by one axis were two independent realizations rather than a controlled
    /// comparison. With per-impairment substreams, an axis's realization depends only on the trial
    /// seed and its own name.
    ///
    /// Here: the same loss axis, once alone and once beside jitter, must drop exactly the same
    /// datagrams.
    /// </summary>
    [Fact]
    public void CommonRandomNumbers_AddingAnAxisDoesNotPerturbAnother()
    {
        const ulong seed = 4242UL;

        var lossOnly = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64),
            new INetworkImpairment[] { new GilbertElliottLossImpairment(0.3, 0.6) },
            seed, 64);

        var lossPlusJitter = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64),
            new INetworkImpairment[]
            {
                new GilbertElliottLossImpairment(0.3, 0.6),
                new UniformJitterImpairment(5_000),
            },
            seed, 64);

        var a = RunSchedule(lossOnly, count: 200, stepTicks: 1_000, tailPolls: 8);
        var b = RunSchedule(lossPlusJitter, count: 200, stepTicks: 1_000, tailPolls: 8);

        Assert.Equal(a.Sends, b.Sends);
        Assert.Contains(false, a.Sends);   // the loss axis actually fired; otherwise this proves nothing
    }

    /// <summary>
    /// The same axis at two magnitudes must keep the other axes' realizations identical -- the
    /// property the old design did have, which must not be lost in the change.
    ///
    /// Delay is drawless, so the two runs differ by exactly the delay difference on every datagram,
    /// asserted exactly rather than approximately.
    /// </summary>
    [Fact]
    public void CommonRandomNumbers_ChangingOneAxisMagnitudeDoesNotPerturbAnother()
    {
        const ulong seed = 99UL;
        const long extraDelay = 40_000;

        var near = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64),
            new INetworkImpairment[]
            {
                new FixedDelayImpairment(10_000),
                new GilbertElliottLossImpairment(0.2, 0.2),
            },
            seed, 64);

        var far = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64),
            new INetworkImpairment[]
            {
                new FixedDelayImpairment(10_000 + extraDelay),
                new GilbertElliottLossImpairment(0.2, 0.2),
            },
            seed, 64);

        var a = RunSchedule(near, count: 120, stepTicks: 2_000, tailPolls: 64);
        var b = RunSchedule(far, count: 120, stepTicks: 2_000, tailPolls: 64);

        Assert.Equal(a.Sends, b.Sends);
        Assert.Equal(a.Receives.Count, b.Receives.Count);

        for (int i = 0; i < a.Receives.Count; i++)
        {
            Assert.Equal(a.Receives[i].Tag, b.Receives[i].Tag);
            Assert.Equal(a.Receives[i].Arrival + extraDelay, b.Receives[i].Arrival);
        }
    }

    /// <summary>
    /// An axis at neutral parameters must be observationally identical to its absence.
    ///
    /// This is not a curiosity: it is the property that lets <c>NetworkProfileCatalog</c> emit only
    /// non-zero axes while still reproducing ADR 0004's frozen profiles exactly. If it ever fails,
    /// `lan` resolved through the catalog stops matching `lan` as the ADR defines it.
    /// </summary>
    [Fact]
    public void NeutralAxis_IsObservationallyIdenticalToItsAbsence()
    {
        const ulong seed = 7UL;

        var without = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64),
            new INetworkImpairment[]
            {
                new FixedDelayImpairment(20_000),
                new UniformJitterImpairment(3_000),
            },
            seed, 64);

        var withNeutral = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64),
            new INetworkImpairment[]
            {
                new FixedDelayImpairment(20_000),
                new UniformJitterImpairment(3_000),
                new GilbertElliottLossImpairment(0.0, 0.0),
                new ReorderImpairment(0.0, 50_000),
            },
            seed, 64);

        var a = RunSchedule(without, count: 150, stepTicks: 1_500, tailPolls: 32);
        var b = RunSchedule(withNeutral, count: 150, stepTicks: 1_500, tailPolls: 32);

        Assert.Equal(a.Sends, b.Sends);
        Assert.Equal(a.Receives, b.Receives);
    }

    /// <summary>
    /// Permuting the impairment array changes nothing, because delay is additive and <c>Dropped</c>
    /// is monotone (see <c>DatagramFate</c>). Order is therefore not part of the configuration for
    /// any impairment shipped today, and a manifest listing them in a different order describes the
    /// same link.
    /// </summary>
    [Fact]
    public void Composition_IsOrderIndependent_ForTheShippedSet()
    {
        const ulong seed = 31337UL;

        var forward = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64),
            new INetworkImpairment[]
            {
                new FixedDelayImpairment(15_000),
                new UniformJitterImpairment(4_000),
                new ReorderImpairment(0.25, 60_000),
            },
            seed, 64);

        var reversed = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64),
            new INetworkImpairment[]
            {
                new ReorderImpairment(0.25, 60_000),
                new UniformJitterImpairment(4_000),
                new FixedDelayImpairment(15_000),
            },
            seed, 64);

        var a = RunSchedule(forward, count: 150, stepTicks: 2_000, tailPolls: 64);
        var b = RunSchedule(reversed, count: 150, stepTicks: 2_000, tailPolls: 64);

        Assert.Equal(a.Sends, b.Sends);
        Assert.Equal(a.Receives, b.Receives);
    }

    /// <summary>
    /// Every impairment resets to as-constructed state, so a sweep reusing an instance across trials
    /// reproduces the previous trial exactly -- the <c>ITransport.Reset</c> contract, now spanning
    /// the impairments the transport owns rather than one RNG it used to own itself.
    /// </summary>
    [Fact]
    public void Reset_RestoresAsConstructedStateAcrossEveryImpairment()
    {
        const ulong seed = 5150UL;

        INetworkImpairment[] Build() => new INetworkImpairment[]
        {
            new FixedDelayImpairment(10_000),
            new UniformJitterImpairment(2_500),
            new GilbertElliottLossImpairment(0.15, 0.5),
            new ReorderImpairment(0.2, 30_000),
        };

        var transport = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64), Build(), seed, 64);

        var first = RunSchedule(transport, count: 100, stepTicks: 1_500, tailPolls: 32);
        transport.Reset();
        var second = RunSchedule(transport, count: 100, stepTicks: 1_500, tailPolls: 32);

        Assert.Equal(first.Sends, second.Sends);
        Assert.Equal(first.Receives, second.Receives);

        // ...and matches a freshly constructed instance, which is the actual definition of
        // "as-constructed state" rather than merely "repeatable".
        var fresh = new EmulatedTransport(
            new LoopbackTransport(MaxPayload, capacity: 64), Build(), seed, 64);
        var freshRun = RunSchedule(fresh, count: 100, stepTicks: 1_500, tailPolls: 32);

        Assert.Equal(first.Sends, freshRun.Sends);
        Assert.Equal(first.Receives, freshRun.Receives);
    }

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
        var transport = Emulate(inner, FixedDelay(0), 1UL, 8);

        Assert.Equal(inner.MaxPayloadBytes, transport.MaxPayloadBytes);
        Assert.Equal(8, transport.MaxInFlight);
        Assert.Equal(0, transport.InFlightCount);
    }

    /// <summary>
    /// What only the transport can know: a null inner, a non-positive capacity, a null or duplicate
    /// impairment.
    ///
    /// The parameter-range rejections that used to live here -- negative delay, jitter or reorder
    /// delay, a probability outside [0, 1], NaN -- moved to the impairments themselves, where the
    /// exception can name the parameter that was actually wrong instead of pointing at a six-field
    /// struct. They are covered per impairment in <c>Transport/Impairments/</c>.
    /// </summary>
    [Fact]
    public void Constructor_RejectsInvalidArguments()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);

        Assert.Throws<ArgumentNullException>(
            () => new EmulatedTransport(null!, new INetworkImpairment[0], 1UL, 8));
        Assert.Throws<ArgumentNullException>(
            () => new EmulatedTransport(inner, null!, 1UL, 8));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EmulatedTransport(inner, new INetworkImpairment[0], 1UL, 0));
        Assert.Throws<ArgumentException>(
            () => new EmulatedTransport(inner, new INetworkImpairment[] { null! }, 1UL, 8));
    }

    /// <summary>
    /// Two impairments on the same axis is rejected, and that is what makes name-derived RNG
    /// substream identity safe: a duplicate would give both instances the same substream, so they
    /// would draw identical numbers and correlate two axes that should be independent -- with
    /// nothing in the source looking wrong.
    /// </summary>
    [Fact]
    public void Constructor_RejectsTwoImpairmentsSharingAnAxisName()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var impairments = new INetworkImpairment[]
        {
            new FixedDelayImpairment(100),
            new FixedDelayImpairment(200),
        };

        ArgumentException ex = Assert.Throws<ArgumentException>(
            () => new EmulatedTransport(inner, impairments, 1UL, 8));
        Assert.Contains("delay", ex.Message);
    }

    /// <summary>
    /// An impairment belongs to exactly one transport. Sharing one between a link's uplink and
    /// downlink would make both directions replay the same model state and the same draws, which no
    /// real pair of paths does -- so it fails loudly rather than producing a plausible wrong result.
    /// </summary>
    [Fact]
    public void Constructor_RejectsAnImpairmentAlreadyBoundToAnotherTransport()
    {
        var first = new LoopbackTransport(MaxPayload, capacity: 8);
        var second = new LoopbackTransport(MaxPayload, capacity: 8);
        var shared = new FixedDelayImpairment(100);

        _ = new EmulatedTransport(first, new INetworkImpairment[] { shared }, 1UL, 8);

        Assert.Throws<InvalidOperationException>(
            () => new EmulatedTransport(second, new INetworkImpairment[] { shared }, 1UL, 8));
    }

    /// <summary>An empty set is legal: an unimpaired decorator that still exercises the scheduling path.</summary>
    [Fact]
    public void EmptyImpairmentSet_BehavesAsAPassThroughDecorator()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, new INetworkImpairment[0], 1UL, 8);

        Assert.True(transport.Send(Tagged(1), 1000));
        var destination = new byte[MaxPayload];
        Assert.True(transport.TryReceive(1000, destination, out int byteCount, out long arrivalTicks));
        Assert.Equal(4, byteCount);
        Assert.Equal(1, destination[0]);
        Assert.Equal(1000, arrivalTicks);
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
        var transport = Emulate(inner, FixedDelay(expected), 20260730UL, 8);
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
        var transport = Emulate(inner, FixedDelay(delay), 5UL, 8);
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
        var transport = Emulate(inner, profile, 77UL, 8);
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
        var transport = Emulate(inner, FixedDelay(1000), 3UL, 64);

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
        var transport = Emulate(inner, profile, 11UL, 64);

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
        var transport = Emulate(inner, profile, 2024UL, 4);
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
        var transport = Emulate(inner, profile, 4242UL, 128);
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
        var transport = Emulate(inner, profile, 31337UL, 64);
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
        var transport = Emulate(inner, FixedDelay(1_000_000), 9UL, 8);

        Assert.True(transport.Send(Tagged(0), 0));
        Assert.True(transport.Send(Tagged(1), 0));

        // The emulator's own loss model said "deliver", but the wrapped transport's queue is full.
        Assert.False(transport.Send(Tagged(2), 0));
    }

    [Fact]
    public void InFlightCapacityExhausted_LeavesDatagramsInTheWrappedTransport()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = Emulate(inner, FixedDelay(1_000_000), 13UL, 2);
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
        var transport = Emulate(inner, FixedDelay(1000), 17UL, 8);

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

        var a = Emulate(new LoopbackTransport(MaxPayload, 64), profile, 0xC0FFEEUL, 64);
        var b = Emulate(new LoopbackTransport(MaxPayload, 64), profile, 0xC0FFEEUL, 64);

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

        var a = Emulate(new LoopbackTransport(MaxPayload, 64), profile, 1UL, 64);
        var b = Emulate(new LoopbackTransport(MaxPayload, 64), profile, 2UL, 64);

        var first = RunSchedule(a, count: 120, stepTicks: 10_000, tailPolls: 64);
        var second = RunSchedule(b, count: 120, stepTicks: 10_000, tailPolls: 64);

        Assert.NotEqual(first.Receives, second.Receives);
    }

    [Fact]
    public void Reset_RestoresAsConstructedStateAndReseedsTheRng()
    {
        var profile = new NetworkProfile(200_000, 40_000, 0.1, 0.6, 0.2, 300_000);
        var inner = new LoopbackTransport(MaxPayload, 64);
        var transport = Emulate(inner, profile, 555UL, 64);

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
        var fresh = Emulate(new LoopbackTransport(MaxPayload, 64), profile, 555UL, 64);
        var freshTrial = RunSchedule(fresh, count: 80, stepTicks: 10_000, tailPolls: 32);
        Assert.Equal(freshTrial.Sends, secondTrial.Sends);
        Assert.Equal(freshTrial.Receives, secondTrial.Receives);
    }

    [Fact]
    public void TraceMode_DelaysDatagramsByTheTraceSamplesInOrderAndWrapsAround()
    {
        var trace = new long[] { 100_000, 200_000, 300_000 };
        var profile = new NetworkProfile(0, 0, 0.0, 0.0, 0.0, 0);
        var inner = new LoopbackTransport(MaxPayload, capacity: 16);
        var transport = EmulateTrace(inner, trace, profile, 1UL, 16);
        var destination = new byte[MaxPayload];

        // Sends spaced far enough apart (1,000,000 ticks) that each drains before the next is
        // sent, so send order and arrival order coincide -- isolates "which trace sample did this
        // send get" from the heap's separate earliest-arrival ordering (covered by
        // Reordering_DeliversInSyntheticArrivalOrderNotSendOrder). Four sends against a 3-sample
        // trace: the fourth must wrap back to the first sample (100_000), not throw or draw fresh.
        var arrivals = new List<long>();
        for (int i = 0; i < 4; i++)
        {
            long sendTicks = i * 1_000_000L;
            Assert.True(transport.Send(Tagged(i), sendTicks));
            Assert.True(transport.TryReceive(sendTicks + 900_000, destination, out _, out long arrival));
            arrivals.Add(arrival - sendTicks);
        }

        Assert.Equal(new long[] { 100_000, 200_000, 300_000, 100_000 }, arrivals);
    }

    [Fact]
    public void TraceMode_Determinism_TwoInstancesWithTheSameSeedAndTraceProduceIdenticalStreams()
    {
        var trace = new long[] { 50_000, 10_000, 80_000, 20_000 };
        // Loss and reorder both live, so the RNG draws made in trace mode are load-bearing too.
        var profile = new NetworkProfile(0, 0, 0.1, 0.5, 0.2, 15_000);

        var a = EmulateTrace(new LoopbackTransport(MaxPayload, 64), trace, profile, 0xC0FFEEUL, 64);
        var b = EmulateTrace(new LoopbackTransport(MaxPayload, 64), trace, profile, 0xC0FFEEUL, 64);

        var first = RunSchedule(a, count: 100, stepTicks: 10_000, tailPolls: 64);
        var second = RunSchedule(b, count: 100, stepTicks: 10_000, tailPolls: 64);

        Assert.Equal(first.Sends, second.Sends);
        Assert.Equal(first.Receives, second.Receives);
        Assert.Contains(false, first.Sends);
    }

    /// <summary>Trace validation now lives in the impairment's own constructor.</summary>
    [Fact]
    public void TraceMode_Constructor_RejectsNullEmptyOrNegativeTrace()
    {
        Assert.Throws<ArgumentNullException>(() => new TraceDelayImpairment(null!));
        Assert.Throws<ArgumentException>(() => new TraceDelayImpairment(Array.Empty<long>()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TraceDelayImpairment(new long[] { 100, -1, 200 }));
    }

    /// <summary>
    /// <b>Replaces <c>TraceMode_Constructor_RejectsNonZeroBaseDelayOrJitter</c>, deliberately.</b>
    ///
    /// The old transport rejected a trace-mode profile carrying base delay or jitter, because one
    /// struct could not distinguish a recorded delay from a synthetic one and layering them
    /// double-modelled the same variance. With separate impairments there is nothing to contradict:
    /// including a fixed delay alongside a trace is an explicit, legal configuration meaning "a
    /// recorded link plus an extra fixed hop", and the delays simply sum. The composition is the
    /// configuration (docs/adr/0013).
    ///
    /// Asserted exactly, because both axes here are drawless: this is arithmetic, not a draw.
    /// </summary>
    [Fact]
    public void TraceAndFixedDelayTogether_SumRatherThanBeingRejected()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var impairments = new INetworkImpairment[]
        {
            new TraceDelayImpairment(new long[] { 100_000 }),
            new FixedDelayImpairment(25_000),
        };
        var transport = new EmulatedTransport(inner, impairments, 1UL, 8);

        Assert.True(transport.Send(Tagged(1), 1000));

        var destination = new byte[MaxPayload];
        Assert.False(transport.TryReceive(1000 + 124_999, destination, out _, out _));
        Assert.True(transport.TryReceive(1000 + 125_000, destination, out _, out long arrivalTicks));
        Assert.Equal(1000 + 125_000, arrivalTicks);
    }

    [Fact]
    public void TraceMode_Reset_ReturnsTheTraceCursorToTheStart()
    {
        var trace = new long[] { 111_000, 222_000, 333_000 };
        var profile = new NetworkProfile(0, 0, 0.0, 0.0, 0.0, 0);
        var inner = new LoopbackTransport(MaxPayload, capacity: 16);
        var transport = EmulateTrace(inner, trace, profile, 9UL, 16);
        var destination = new byte[MaxPayload];

        // Consume two samples (advance the cursor into the middle of the trace).
        transport.Send(Tagged(0), 0);
        transport.Send(Tagged(1), 0);
        transport.TryReceive(long.MaxValue / 4, destination, out _, out _);
        transport.TryReceive(long.MaxValue / 4, destination, out _, out _);

        transport.Reset();

        transport.Send(Tagged(2), 0);
        Assert.True(transport.TryReceive(long.MaxValue / 4, destination, out _, out long arrival));
        Assert.Equal(111_000, arrival);
    }

    [Fact]
    public void TraceMode_BurstLossStillApplies()
    {
        var trace = new long[] { 10_000 };
        var profile = new NetworkProfile(0, 0, 0.02, 0.8, 0.0, 0);
        var inner = new LoopbackTransport(MaxPayload, capacity: 4);
        var transport = EmulateTrace(inner, trace, profile, 2024UL, 4);
        var destination = new byte[MaxPayload];

        int lost = 0;
        for (int i = 0; i < 2000; i++)
        {
            if (!transport.Send(Tagged(i & 0xFF), i))
            {
                lost++;
            }

            while (transport.TryReceive(i, destination, out _, out _))
            {
            }
        }

        Assert.True(lost > 0, "expected some losses even in trace mode");
    }

    [Fact]
    public void TraceMode_ReorderStillApplies()
    {
        long step = 10_000;
        var trace = new long[] { 100_000 };
        var profile = new NetworkProfile(0, 0, 0.0, 0.0, 0.5, 50_000);
        var inner = new LoopbackTransport(MaxPayload, capacity: 64);
        var transport = EmulateTrace(inner, trace, profile, 31337UL, 64);
        var destination = new byte[MaxPayload];

        for (int i = 0; i < 24; i++)
        {
            Assert.True(transport.Send(Tagged(i), i * step));
        }

        var tags = new List<int>();
        while (transport.TryReceive(long.MaxValue / 4, destination, out _, out _))
        {
            tags.Add(destination[0]);
        }

        Assert.Equal(24, tags.Count);
        Assert.NotEqual(Enumerable.Range(0, 24).ToArray(), tags.ToArray());
    }

    [Fact]
    public void TraceMode_SendAndTryReceive_Allocate_Zero_Bytes()
    {
        var trace = new long[] { 0 };
        var profile = new NetworkProfile(0, 0, 0.0, 0.0, 0.0, 0);
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = EmulateTrace(inner, trace, profile, 21UL, 8);
        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(0, destination, out _, out _);
        });
    }

    [Fact]
    public void SendAndTryReceive_SteadyState_Allocate_Zero_Bytes()
    {
        // Common path only: no loss, no jitter, no reordering, and the datagram is due
        // immediately, so every iteration exercises send, drain, heap push, heap pop and copy.
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = Emulate(inner, FixedDelay(0), 21UL, 8);
        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(0, destination, out _, out _);
        });
    }

    /// <summary>
    /// The worst case for composition: every axis installed, so every datagram pays the maximum
    /// number of interface dispatches at both stages.
    ///
    /// This is the specific regression composition introduced a way to cause. Iterating an
    /// interface-typed sequence with <c>foreach</c> boxes the enumerator on <b>every datagram</b>,
    /// and storing impairments as structs would box on every array access. Neither shows up as a
    /// wrong number -- it shows up as a sweep that GCs in proportion to how impaired the link is.
    /// The emulator holds a concrete <c>INetworkImpairment[]</c> and iterates it with an indexed
    /// <c>for</c> precisely so this passes.
    /// </summary>
    [Fact]
    public void FullImpairmentSet_Allocates_Zero_Bytes()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(
            inner,
            new INetworkImpairment[]
            {
                new FixedDelayImpairment(1_000),
                new UniformJitterImpairment(500),
                new GilbertElliottLossImpairment(0.25, 0.5),
                new ReorderImpairment(0.5, 2_000),
            },
            77UL, 8);

        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(0, destination, out _, out _);
        });
    }

    /// <summary>The empty set, proving the dispatch loops themselves cost nothing when there is nothing to dispatch to.</summary>
    [Fact]
    public void EmptyImpairmentSet_Allocates_Zero_Bytes()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(inner, new INetworkImpairment[0], 78UL, 8);
        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(0, destination, out _, out _);
        });
    }

    /// <summary>
    /// A trace axis alongside loss and reorder -- the composed equivalent of what used to be
    /// "trace mode", now just another set.
    /// </summary>
    [Fact]
    public void TraceWithLossAndReorder_Allocates_Zero_Bytes()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(
            inner,
            new INetworkImpairment[]
            {
                new TraceDelayImpairment(new long[] { 1_000, 2_000, 1_500 }),
                new GilbertElliottLossImpairment(0.2, 0.4),
                new ReorderImpairment(0.5, 2_000),
            },
            79UL, 8);

        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(0, destination, out _, out _);
        });
    }

    /// <summary>
    /// <c>Reset()</c> now fans out across every impairment, so it must stay allocation-free too --
    /// a sweep calls it once per trial, and an allocation here would scale with trial count.
    /// </summary>
    [Fact]
    public void Reset_Allocates_Zero_Bytes()
    {
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = new EmulatedTransport(
            inner,
            new INetworkImpairment[]
            {
                new FixedDelayImpairment(1_000),
                new UniformJitterImpairment(500),
                new GilbertElliottLossImpairment(0.25, 0.5),
                new ReorderImpairment(0.5, 2_000),
                new TraceDelayImpairment(new long[] { 100, 200 }),
            },
            80UL, 8);

        AllocationAssert.Zero(() => transport.Reset());
    }

    [Fact]
    public void LossBranch_Allocates_Zero_Bytes()
    {
        // Verified rather than assumed: the drop path must not allocate either, or a lossy sweep
        // would GC in proportion to how lossy the profile is.
        var inner = new LoopbackTransport(MaxPayload, capacity: 8);
        var transport = Emulate(inner, new NetworkProfile(0, 0, 1.0, 1.0, 0.0, 0), 22UL, 8);
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
        var transport = Emulate(inner, new NetworkProfile(1000, 500, 0.0, 0.0, 1.0, 2000), 23UL, 8);
        var payload = new byte[MaxPayload];
        var destination = new byte[MaxPayload];

        AllocationAssert.Zero(() =>
        {
            transport.Send(payload, 0);
            transport.TryReceive(1_000_000, destination, out _, out _);
        });
    }
}
