using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Transport;

namespace Teleop.Core.Tests.Transport;

/// <summary>
/// Every assertion here is against a <b>hand-constructed stream with known ground truth</b> — a
/// send-result pattern whose burst lengths are visible in the literal, a sequence stream reordered
/// by construction, an arrival stream whose RFC 3550 <c>J</c> is worked out by hand in the test's
/// own comment. Nothing here asserts "whatever the code currently does".
/// </summary>
public class NetworkObserverTests
{
    private const long ReceiverTicksPerSecond = 10_000_000;

    /// <summary>Records everything, in order, with no capacity limit — a test needs the whole stream.</summary>
    private sealed class RecordingSink : IMetricSink
    {
        public readonly List<(string Name, double Value, long Ticks)> Samples = new();

        public void Record(string name, double value, long ticks) => Samples.Add((name, value, ticks));

        public IEnumerable<double> ValuesOf(string name) =>
            Samples.Where(s => s.Name == name).Select(s => s.Value);

        public int CountOf(string name) => Samples.Count(s => s.Name == name);
    }

    private static long Ms(double milliseconds, long ticksPerSecond = ReceiverTicksPerSecond) =>
        (long)(milliseconds / 1000.0 * ticksPerSecond);

    private static NetworkObserver Downlink(RecordingSink sink) =>
        NetworkObserver.ForDownlink(sink, new ManualClock(ReceiverTicksPerSecond));

    private static NetworkObserver Uplink(RecordingSink sink) =>
        NetworkObserver.ForUplink(sink, new ManualClock(ReceiverTicksPerSecond));

    // ---------------------------------------------------------------------------------------
    // Metric names. Pinned to literals because docs/metrics.md §3 defines these exact strings and
    // IMetricSink stores whatever name it is given -- a rename here would silently produce an
    // undefined metric, and nothing else in the build would notice.
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void UplinkEmitsTheUplinkMetricNames()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Uplink(sink);

        observer.OnSendResult(accepted: false, nowTicks: 10);
        observer.OnSendResult(accepted: true, nowTicks: 20);
        observer.OnDelivery(arrivalTicks: 30);

        Assert.Equal(
            new[] { "net_uplink_dropped", "net_uplink_loss_burst", "net_uplink_sent", "net_uplink_received" },
            sink.Samples.Select(s => s.Name).ToArray());
    }

    [Fact]
    public void DownlinkEmitsTheDownlinkMetricNames()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        observer.OnSendResult(accepted: false, nowTicks: 10);
        observer.OnSendResult(accepted: true, nowTicks: 20);
        observer.OnDelivery(arrivalTicks: 30);
        observer.OnSequencedArrival(5, senderSendTicks: 0, senderTicksPerSecond: ReceiverTicksPerSecond, arrivalTicks: 40);
        observer.OnSequencedArrival(4, senderSendTicks: Ms(1), senderTicksPerSecond: ReceiverTicksPerSecond, arrivalTicks: 50);

        Assert.Equal(
            new[]
            {
                "net_downlink_dropped", "net_downlink_loss_burst", "net_downlink_sent",
                "net_downlink_received", "net_downlink_reorder_displacement", "net_downlink_jitter_ms",
            },
            sink.Samples.Select(s => s.Name).ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Loss rate and burst-length distribution (docs/metrics.md §3).
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void CountsOneSentPerAcceptanceAndOneDroppedPerRefusal()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Uplink(sink);

        // Ground truth written as a literal: 3 accepted, 4 refused.
        bool[] stream = { true, false, false, false, true, false, true };
        for (int i = 0; i < stream.Length; i++)
        {
            observer.OnSendResult(stream[i], nowTicks: i);
        }

        Assert.Equal(3, sink.CountOf("net_uplink_sent"));
        Assert.Equal(4, sink.CountOf("net_uplink_dropped"));
        Assert.All(sink.ValuesOf("net_uplink_sent"), v => Assert.Equal(1.0, v));
        Assert.All(sink.ValuesOf("net_uplink_dropped"), v => Assert.Equal(1.0, v));
    }

    [Fact]
    public void EmitsOneBurstSamplePerCompletedRunOfConsecutiveRefusals()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Uplink(sink);

        // T F F F T F T  ->  runs of length 3 and 1, both closed by a following acceptance.
        bool[] stream = { true, false, false, false, true, false, true };
        for (int i = 0; i < stream.Length; i++)
        {
            observer.OnSendResult(stream[i], nowTicks: i);
        }

        Assert.Equal(new[] { 3.0, 1.0 }, sink.ValuesOf("net_uplink_loss_burst").ToArray());

        // The identity worth relying on in analysis: every drop belongs to exactly one run.
        Assert.Equal(
            sink.CountOf("net_uplink_dropped"),
            (int)sink.ValuesOf("net_uplink_loss_burst").Sum());
    }

    [Fact]
    public void ABurstIsStampedAtTheAcceptanceThatClosesIt()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Uplink(sink);

        observer.OnSendResult(accepted: false, nowTicks: 100);
        observer.OnSendResult(accepted: false, nowTicks: 200);
        observer.OnSendResult(accepted: true, nowTicks: 300);

        (string Name, double Value, long Ticks) burst =
            sink.Samples.Single(s => s.Name == "net_uplink_loss_burst");
        Assert.Equal(2.0, burst.Value);
        Assert.Equal(300, burst.Ticks);
    }

    [Fact]
    public void ARunStillOpenAtResetIsDiscardedNotEmitted()
    {
        // This is the documented censoring in OnSendResult's doc, asserted so it stays documented
        // behaviour rather than becoming an accident. Reset has no tick to stamp a flush at, and
        // Core may not read a clock to invent one.
        var sink = new RecordingSink();
        NetworkObserver observer = Uplink(sink);

        observer.OnSendResult(accepted: true, nowTicks: 0);
        observer.OnSendResult(accepted: false, nowTicks: 1);
        observer.OnSendResult(accepted: false, nowTicks: 2);
        observer.Reset();

        Assert.Equal(0, sink.CountOf("net_uplink_loss_burst"));
        Assert.Equal(2, sink.CountOf("net_uplink_dropped"));

        // And the run does not leak across the boundary: the next acceptance closes nothing.
        observer.OnSendResult(accepted: true, nowTicks: 3);
        Assert.Equal(0, sink.CountOf("net_uplink_loss_burst"));
    }

    [Fact]
    public void ALosslessStreamEmitsNoDroppedAndNoBurstSamplesAtAll()
    {
        // The playout_late precedent: a direction that loses nothing emits nothing, rather than a
        // stream of zeroes that would drag every percentile of the burst distribution to zero.
        var sink = new RecordingSink();
        NetworkObserver observer = Uplink(sink);

        for (int i = 0; i < 50; i++)
        {
            observer.OnSendResult(accepted: true, nowTicks: i);
        }

        Assert.Equal(0, sink.CountOf("net_uplink_dropped"));
        Assert.Equal(0, sink.CountOf("net_uplink_loss_burst"));
        Assert.Equal(50, sink.CountOf("net_uplink_sent"));
    }

    [Fact]
    public void DeliveriesAreStampedAtArrivalNotAtPollTime()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Uplink(sink);

        observer.OnDelivery(arrivalTicks: 12345);

        (string Name, double Value, long Ticks) delivery = sink.Samples.Single();
        Assert.Equal("net_uplink_received", delivery.Name);
        Assert.Equal(1.0, delivery.Value);
        Assert.Equal(12345, delivery.Ticks);
    }

    // ---------------------------------------------------------------------------------------
    // Reordering rate and max displacement (docs/metrics.md §3).
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void AnInOrderStreamProducesNoReorderSamples()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        for (uint seq = 0; seq < 20; seq++)
        {
            observer.OnSequencedArrival(seq, senderSendTicks: Ms(seq * 10), ReceiverTicksPerSecond, arrivalTicks: Ms(100 + seq * 10));
        }

        Assert.Equal(0, sink.CountOf("net_downlink_reorder_displacement"));
    }

    [Fact]
    public void AStreamReorderedByConstructionProducesTheDisplacementsItWasBuiltWith()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        // Arrival order 0 1 3 2 4 6 5 : seq 2 arrives after 3 (displacement 1) and seq 5 after 6
        // (displacement 1). Two out-of-order arrivals out of seven -> a reordering rate of 2/7.
        uint[] arrivals = { 0, 1, 3, 2, 4, 6, 5 };
        for (int i = 0; i < arrivals.Length; i++)
        {
            observer.OnSequencedArrival(arrivals[i], senderSendTicks: Ms(i * 10), ReceiverTicksPerSecond, arrivalTicks: Ms(100 + i * 10));
        }

        Assert.Equal(new[] { 1.0, 1.0 }, sink.ValuesOf("net_downlink_reorder_displacement").ToArray());
    }

    [Fact]
    public void DisplacementIsTheDistanceBelowTheRunningHighWaterMark()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        // 0, 5, 1: seq 1 arrives when the high-water mark is 5, so displacement is 4 -- not 1,
        // which is the distance to the previous arrival. docs/metrics.md §3's "max displacement"
        // is the maximum of exactly these samples.
        uint[] arrivals = { 0, 5, 1 };
        for (int i = 0; i < arrivals.Length; i++)
        {
            observer.OnSequencedArrival(arrivals[i], senderSendTicks: Ms(i * 10), ReceiverTicksPerSecond, arrivalTicks: Ms(100 + i * 10));
        }

        Assert.Equal(new[] { 4.0 }, sink.ValuesOf("net_downlink_reorder_displacement").ToArray());
    }

    [Fact]
    public void ALossGapIsNotAReordering()
    {
        // Sequences 2 and 3 never arrive. Nothing is out of order; a gap must not be counted as
        // one, or the reordering rate would just be re-reporting the loss rate.
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        uint[] arrivals = { 0, 1, 4, 5 };
        for (int i = 0; i < arrivals.Length; i++)
        {
            observer.OnSequencedArrival(arrivals[i], senderSendTicks: Ms(i * 10), ReceiverTicksPerSecond, arrivalTicks: Ms(100 + i * 10));
        }

        Assert.Equal(0, sink.CountOf("net_downlink_reorder_displacement"));
    }

    [Fact]
    public void SequenceComparisonIsWrapSafeAcrossUintMaxValue()
    {
        // CommandFrame.Sequence wraps at uint.MaxValue and its doc forbids a plain unsigned
        // comparison. A naive `seq < highest` would call every sequence after the wrap a
        // reordering with a displacement near four billion, which would swamp the metric.
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        uint[] arrivals = { uint.MaxValue - 2, uint.MaxValue - 1, uint.MaxValue, 0, 1, 2 };
        for (int i = 0; i < arrivals.Length; i++)
        {
            observer.OnSequencedArrival(arrivals[i], senderSendTicks: Ms(i * 10), ReceiverTicksPerSecond, arrivalTicks: Ms(100 + i * 10));
        }

        Assert.Equal(0, sink.CountOf("net_downlink_reorder_displacement"));
    }

    [Fact]
    public void AReorderingAcrossTheWrapPointHasASmallDisplacement()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        // 0 then uint.MaxValue: the latter is the sequence immediately *before* 0, so it is one
        // step out of order, not 4294967295 steps.
        observer.OnSequencedArrival(0, Ms(10), ReceiverTicksPerSecond, Ms(110));
        observer.OnSequencedArrival(uint.MaxValue, Ms(0), ReceiverTicksPerSecond, Ms(120));

        Assert.Equal(new[] { 1.0 }, sink.ValuesOf("net_downlink_reorder_displacement").ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // RFC 3550 interarrival jitter (docs/metrics.md §3).
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void JitterIsNotEmittedForTheFirstArrivalBecauseThereIsNothingToDifferenceAgainst()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        observer.OnSequencedArrival(0, Ms(0), ReceiverTicksPerSecond, Ms(100));

        Assert.Equal(0, sink.CountOf("net_downlink_jitter_ms"));
    }

    [Fact]
    public void APerfectlyPacedLinkHasZeroJitterEvenAtLargeConstantDelay()
    {
        // The property that makes RFC 3550's estimator the right instrument here: it measures the
        // *variation* in transit time, so a link with 300 ms of rock-steady delay reports zero.
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        for (uint seq = 0; seq < 10; seq++)
        {
            observer.OnSequencedArrival(
                seq, senderSendTicks: Ms(seq * 10), ReceiverTicksPerSecond, arrivalTicks: Ms(300 + seq * 10));
        }

        Assert.All(sink.ValuesOf("net_downlink_jitter_ms"), v => Assert.Equal(0.0, v, 12));
    }

    [Fact]
    public void JitterMatchesRfc3550WorkedByHand()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        // S / R in ms. D(i-1,i) = (R_i - R_{i-1}) - (S_i - S_{i-1}); J += (|D| - J)/16.
        //   seq 0: S=0,  R=100                          -- no sample
        //   seq 1: S=10, R=110  dR=10 dS=10 D=0         -- J = 0 + (0 - 0)/16      = 0
        //   seq 2: S=20, R=135  dR=25 dS=10 D=+15       -- J = 0 + (15 - 0)/16     = 0.9375
        //   seq 3: S=30, R=140  dR=5  dS=10 D=-5, |D|=5 -- J = 0.9375 + (5 - 0.9375)/16
        //                                                    = 1.19140625
        (double S, double R)[] stream = { (0, 100), (10, 110), (20, 135), (30, 140) };
        for (uint i = 0; i < stream.Length; i++)
        {
            observer.OnSequencedArrival(
                i, Ms(stream[i].S), ReceiverTicksPerSecond, Ms(stream[i].R));
        }

        double[] jitter = sink.ValuesOf("net_downlink_jitter_ms").ToArray();
        Assert.Equal(3, jitter.Length);
        Assert.Equal(0.0, jitter[0], 9);
        Assert.Equal(0.9375, jitter[1], 9);
        Assert.Equal(1.19140625, jitter[2], 9);
    }

    [Fact]
    public void JitterNormalizesTheSendersTickRateRatherThanAssumingItMatchesTheReceivers()
    {
        // docs/adr/0008: a 10 MHz operator against a 1 GHz robot inflated every measured RTT 100x.
        // Here the sender counts at 1 GHz and the receiver at 10 MHz, and the stream is perfectly
        // paced in *physical* time. Treating the sender's ticks as the receiver's would produce a
        // wildly nonzero D on every sample; correct normalization produces exactly zero.
        const long SenderTicksPerSecond = 1_000_000_000;

        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        for (uint seq = 0; seq < 8; seq++)
        {
            observer.OnSequencedArrival(
                seq,
                senderSendTicks: Ms(seq * 10, SenderTicksPerSecond),
                senderTicksPerSecond: SenderTicksPerSecond,
                arrivalTicks: Ms(50 + seq * 10));
        }

        Assert.Equal(7, sink.CountOf("net_downlink_jitter_ms"));
        Assert.All(sink.ValuesOf("net_downlink_jitter_ms"), v => Assert.Equal(0.0, v, 9));
    }

    [Fact]
    public void JitterIsIndependentOfAConstantClockOffsetBetweenTheTwoEnds()
    {
        // The whole reason the raw sender stamp is fed in rather than a ClockSync-corrected one:
        // D is a difference of differences, so an arbitrary constant offset cancels exactly and no
        // offset estimate -- and therefore none of its estimation noise -- enters the figure.
        var withoutOffset = new RecordingSink();
        var withOffset = new RecordingSink();
        NetworkObserver a = Downlink(withoutOffset);
        NetworkObserver b = Downlink(withOffset);

        double[] sendMs = { 0, 10, 20, 30, 40 };
        double[] arrivalMs = { 100, 113, 118, 131, 140 };
        const double OffsetMs = 987_654;

        for (uint i = 0; i < sendMs.Length; i++)
        {
            a.OnSequencedArrival(i, Ms(sendMs[i]), ReceiverTicksPerSecond, Ms(arrivalMs[i]));
            b.OnSequencedArrival(i, Ms(sendMs[i] + OffsetMs), ReceiverTicksPerSecond, Ms(arrivalMs[i]));
        }

        Assert.Equal(
            withoutOffset.ValuesOf("net_downlink_jitter_ms").ToArray(),
            withOffset.ValuesOf("net_downlink_jitter_ms").ToArray());
    }

    [Fact]
    public void AnUnknownSenderTickRateSuppressesJitterButNotReordering()
    {
        // The honest degradation for a vantage that has no sender rate on the wire -- the robot,
        // for uplink traffic. It reports what it can measure and stays silent about what it
        // cannot, rather than assuming the rates match.
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        observer.OnSequencedArrival(0, Ms(0), senderTicksPerSecond: 0, arrivalTicks: Ms(100));
        observer.OnSequencedArrival(2, Ms(20), senderTicksPerSecond: 0, arrivalTicks: Ms(120));
        observer.OnSequencedArrival(1, Ms(10), senderTicksPerSecond: 0, arrivalTicks: Ms(130));

        Assert.Equal(0, sink.CountOf("net_downlink_jitter_ms"));
        Assert.Equal(new[] { 1.0 }, sink.ValuesOf("net_downlink_reorder_displacement").ToArray());
    }

    [Fact]
    public void JitterSamplesAreStampedAtArrival()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Downlink(sink);

        observer.OnSequencedArrival(0, Ms(0), ReceiverTicksPerSecond, Ms(100));
        observer.OnSequencedArrival(1, Ms(10), ReceiverTicksPerSecond, Ms(117));

        (string Name, double Value, long Ticks) sample =
            sink.Samples.Single(s => s.Name == "net_downlink_jitter_ms");
        Assert.Equal(Ms(117), sample.Ticks);
    }

    // ---------------------------------------------------------------------------------------
    // Vantage capability, and Reset.
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void TheUplinkVantageRefusesSequencedArrivalsRatherThanEmittingAnUndefinedMetric()
    {
        var sink = new RecordingSink();
        NetworkObserver observer = Uplink(sink);

        Assert.False(observer.SupportsSequencedArrival);
        Assert.Throws<InvalidOperationException>(
            () => observer.OnSequencedArrival(0, 0, ReceiverTicksPerSecond, 0));
        Assert.Empty(sink.Samples);
    }

    [Fact]
    public void TheDownlinkVantageSupportsSequencedArrivals()
    {
        Assert.True(Downlink(new RecordingSink()).SupportsSequencedArrival);
    }

    [Fact]
    public void ResetReturnsTheObserverToItsAsConstructedState()
    {
        // Core style requires a test proving Reset() restores the as-constructed state, because
        // sweeps reuse instances across trials. Here that means: the same event sequence produces
        // the same sample stream before and after a Reset.
        var first = new RecordingSink();
        var second = new RecordingSink();
        NetworkObserver fresh = Downlink(first);
        NetworkObserver reused = Downlink(second);

        void Drive(NetworkObserver observer)
        {
            observer.OnSendResult(accepted: false, nowTicks: 0);
            observer.OnSendResult(accepted: true, nowTicks: 1);
            uint[] arrivals = { 7, 9, 8 };
            for (uint i = 0; i < arrivals.Length; i++)
            {
                observer.OnSequencedArrival(
                    arrivals[i], Ms(i * 10), ReceiverTicksPerSecond, Ms(100 + i * 17));
            }
        }

        // Give the reused observer a completely different history first, then reset it.
        reused.OnSendResult(accepted: false, nowTicks: 900);
        reused.OnSequencedArrival(4_000, Ms(5000), ReceiverTicksPerSecond, Ms(9000));
        reused.OnSequencedArrival(4_001, Ms(5010), ReceiverTicksPerSecond, Ms(9100));
        reused.Reset();
        second.Samples.Clear();

        Drive(fresh);
        Drive(reused);

        Assert.Equal(
            first.Samples.Select(s => (s.Name, s.Value, s.Ticks)).ToArray(),
            second.Samples.Select(s => (s.Name, s.Value, s.Ticks)).ToArray());
    }

    [Fact]
    public void ConstructionRejectsANullSinkAndANullClock()
    {
        Assert.Throws<ArgumentNullException>(
            () => NetworkObserver.ForDownlink(null!, new ManualClock(ReceiverTicksPerSecond)));
        Assert.Throws<ArgumentNullException>(
            () => NetworkObserver.ForUplink(new RecordingSink(), null!));
    }

    // ---------------------------------------------------------------------------------------
    // Invariant 8: this runs once per datagram, in both directions.
    // ---------------------------------------------------------------------------------------

    [Fact]
    public void OnSendResultAllocatesNothing()
    {
        NetworkObserver observer = NetworkObserver.ForUplink(
            new NullMetricSink(), new ManualClock(ReceiverTicksPerSecond));
        long tick = 0;
        bool accept = false;

        AllocationAssert.Zero(() =>
        {
            // Alternating, so the burst-closing path is exercised inside the measured loop too.
            accept = !accept;
            observer.OnSendResult(accept, tick++);
        });
    }

    [Fact]
    public void OnDeliveryAndOnSequencedArrivalAllocateNothing()
    {
        NetworkObserver observer = NetworkObserver.ForDownlink(
            new NullMetricSink(), new ManualClock(ReceiverTicksPerSecond));
        uint sequence = 0;
        long tick = 0;

        AllocationAssert.Zero(() =>
        {
            observer.OnDelivery(tick);
            // Every third arrival is out of order, so the reorder-emitting branch is measured too.
            uint seq = sequence % 3 == 2 ? sequence - 1 : sequence;
            observer.OnSequencedArrival(seq, tick, ReceiverTicksPerSecond, tick);
            sequence++;
            tick += 100_000;
        });
    }
}
