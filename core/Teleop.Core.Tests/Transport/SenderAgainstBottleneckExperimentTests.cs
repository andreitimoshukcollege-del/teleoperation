using Teleop.Core.Contracts;
using Teleop.Core.Transport;
using Xunit.Abstractions;

namespace Teleop.Core.Tests.Transport;

/// <summary>
/// The measurement half of the bottleneck/sender candidate
/// (docs/research-log/2026-09-09-bottleneck-sender.md).
///
/// <b>Why the numbers live in a test rather than in <c>results/</c>.</b> <c>SweepCommand</c>
/// hardcodes its transport stack (<c>LoopbackTransport</c> + <c>EmulatedTransport</c>) and has no
/// way to select one by name, so a bottleneck cannot appear in a sweep today and no
/// <c>manifest.json</c> can describe it. Rather than leave the candidate unmeasured, the harness is
/// here — deterministic, seeded, and asserting the structural claims it was built to test. The
/// numbers it prints are reproducible from this file and are *not* citable in the repo's sense,
/// which is stated plainly in the log.
///
/// The stack under test, which is the physically correct ordering — the access link serializes and
/// queues, then the rest of the path adds propagation delay, jitter and loss:
///
/// <code>
/// sender-policy -> EmulatedTransport(profile) -> BottleneckTransport(C, B) -> LoopbackTransport
/// </code>
///
/// One consequence to know: <c>EmulatedTransport</c> applies loss at its own <c>Send</c>, so a lost
/// datagram never enters the bottleneck queue. That is the ordering <c>docs/adr/0013</c> insists on
/// ("a dropped packet that first occupied an in-flight slot would model a link that transmitted
/// bytes it was supposed to have lost"), and it means loss here does not consume buffer.
/// </summary>
public sealed class SenderAgainstBottleneckExperimentTests
{
    private const long TicksPerSecond = 10_000_000;
    private const long StepTicks = 100_000;          // 10 ms, the sweep's own cadence
    private const int PayloadBytes = 73;             // RawPoseCodec.EncodedSize
    private const long OfferedBytesPerSecond = 7300; // 73 B / 10 ms == 58.4 kbit/s

    private const int Steps = 3000;                  // 30 s
    private const int TransientSteps = 1000;         // discarded; the queue fill is not steady state

    private readonly ITestOutputHelper _output;

    public SenderAgainstBottleneckExperimentTests(ITestOutputHelper output) => _output = output;

    private enum SenderKind
    {
        /// <summary>The baseline: the host's offered load reaches the link unmodified.</summary>
        Greedy,

        /// <summary>Open-loop admission at a configured rate. The oracle — it is told the link rate.</summary>
        RateLimited,

        /// <summary>Closed-loop admission from the only signal a decorator can see: Send returning false.</summary>
        BacklogBackoff,
    }

    private sealed class TrialResult
    {
        public int Offered;
        public int Accepted;
        public int Delivered;
        public long SenderRefused;
        public long InnerRefused;
        public double MaxDeliveryGapMs;
        public List<double> DelayMs = new();

        public double P(double percentile)
        {
            if (DelayMs.Count == 0)
            {
                return double.NaN;
            }

            var sorted = DelayMs.OrderBy(d => d).ToList();
            int rank = (int)Math.Ceiling(percentile * sorted.Count) - 1;
            if (rank < 0)
            {
                rank = 0;
            }

            if (rank >= sorted.Count)
            {
                rank = sorted.Count - 1;
            }

            return sorted[rank];
        }
    }

    /// <summary>
    /// One trial. Deterministic given (kind, capacity, buffer, admit rate, profile, seed).
    /// Sends one 73-byte datagram per 10 ms step and drains the link every step, exactly as
    /// <c>SweepCommand</c> does with its uplink.
    /// </summary>
    private static TrialResult Run(
        SenderKind kind,
        long capacityBytesPerSecond,
        int bufferDatagrams,
        long admitBytesPerSecond,
        string profileName,
        ulong seed,
        int burstDatagrams = 1)
    {
        var loopback = new LoopbackTransport(PayloadBytes, 4096);
        var bottleneck = new BottleneckTransport(loopback, capacityBytesPerSecond, bufferDatagrams, TicksPerSecond);

        Assert.True(NetworkProfileCatalog.TryResolveImpairments(
            profileName, TicksPerSecond, out INetworkImpairment[] impairments, out _));

        // maxInFlight and the loopback ring are deliberately far larger than any occupancy these
        // trials reach, so that EmulatedTransport's own back-pressure (which engages only past
        // maxInFlight * stepInterval of one-way delay) cannot contaminate the measurement.
        var link = new EmulatedTransport(bottleneck, impairments, seed, maxInFlight: 1024);

        ITransport sender = kind switch
        {
            SenderKind.Greedy => link,
            SenderKind.RateLimited => new RateLimitedTransport(
                link, admitBytesPerSecond, PayloadBytes * burstDatagrams, TicksPerSecond),
            SenderKind.BacklogBackoff => new BacklogBackoffTransport(link, maxGap: 16, successesBeforeDecrease: 4),
            _ => throw new ArgumentOutOfRangeException(nameof(kind)),
        };

        var result = new TrialResult();
        var payload = new byte[PayloadBytes];
        var destination = new byte[PayloadBytes];
        long previousArrival = -1;

        for (int step = 0; step < Steps; step++)
        {
            long now = step * StepTicks;

            BitConverter.TryWriteBytes(payload.AsSpan(0, 8), now);
            bool accepted = sender.Send(payload, now);

            if (step >= TransientSteps)
            {
                result.Offered++;
                if (accepted)
                {
                    result.Accepted++;
                }
            }

            while (sender.TryReceive(now, destination, out _, out long arrivalTicks))
            {
                long sendTick = BitConverter.ToInt64(destination, 0);
                if (sendTick >= TransientSteps * StepTicks)
                {
                    result.Delivered++;
                    result.DelayMs.Add((arrivalTicks - sendTick) * 1000.0 / TicksPerSecond);

                    // Spacing of the delivered command stream. This is the place a mechanism that
                    // "wins" by delivering the same count in clumps would show up, and the reason
                    // delivered-per-second alone is not a sufficient cost measure.
                    if (previousArrival >= 0)
                    {
                        double gapMs = (arrivalTicks - previousArrival) * 1000.0 / TicksPerSecond;
                        if (gapMs > result.MaxDeliveryGapMs)
                        {
                            result.MaxDeliveryGapMs = gapMs;
                        }
                    }

                    previousArrival = arrivalTicks;
                }
            }
        }

        result.SenderRefused = sender switch
        {
            RateLimitedTransport limiter => limiter.RefusedCount,
            BacklogBackoffTransport controller => controller.RefusedCount,
            _ => 0,
        };

        result.InnerRefused = bottleneck.InnerRefusedCount;
        return result;
    }

    private void PrintHeader(string title)
    {
        _output.WriteLine(string.Empty);
        _output.WriteLine(title);
        _output.WriteLine(
            "sender                  C/offer  buf  accept/s  deliv/s  refused  maxgap  p50 ms   p95 ms   p99 ms");
    }

    private void PrintRow(string label, double capacityRatio, int buffer, TrialResult r)
    {
        double seconds = (Steps - TransientSteps) * StepTicks / (double)TicksPerSecond;
        _output.WriteLine(
            $"{label,-22}  {capacityRatio,6:0.00}  {buffer,3}  {r.Accepted / seconds,8:0.0}  " +
            $"{r.Delivered / seconds,7:0.0}  {r.SenderRefused,7}  {r.MaxDeliveryGapMs,6:0.0}  " +
            $"{r.P(0.50),6:0.0}  {r.P(0.95),6:0.0}  {r.P(0.99),6:0.0}");
    }

    /// <summary>
    /// <b>H1.</b> Below the offered load, delivered command rate is pinned at <c>C / payload</c>
    /// independent of how much is offered, and every delivered command pays the whole buffer as
    /// delay. Above the offered load there is no queue and nothing to reduce.
    ///
    /// Falsifier stated in the log: delay never exceeds one serialization time, or delivered rate
    /// tracks offered rate while C is below it.
    /// </summary>
    [Fact]
    public void H1_BelowCapacity_DeliveredRateIsPinnedAndDelayIsTheWholeBuffer()
    {
        PrintHeader("H1 -- greedy sender against a bottleneck, profile 'lan', seed 1");

        var ratios = new[] { 2.00, 1.00, 0.75, 0.50, 0.25 };
        const int buffer = 10;
        var byRatio = new Dictionary<double, TrialResult>();

        foreach (double ratio in ratios)
        {
            long capacity = (long)(OfferedBytesPerSecond * ratio);
            TrialResult r = Run(SenderKind.Greedy, capacity, buffer, capacity, "lan", seed: 1);
            byRatio[ratio] = r;
            PrintRow("greedy", ratio, buffer, r);

            // No hidden loss: everything this measurement attributes to tail drop really was tail
            // drop, not the inner transport quietly refusing.
            Assert.Equal(0, r.InnerRefused);
        }

        double seconds = (Steps - TransientSteps) * StepTicks / (double)TicksPerSecond;

        // Above capacity: no queue at all. 'lan' contributes 2 ms fixed delay plus up to 1 ms
        // jitter, and the link contributes one serialization time (5 ms at 2x rate).
        Assert.True(byRatio[2.00].P(0.95) < 12.0, $"unloaded p95 was {byRatio[2.00].P(0.95):0.0} ms");
        Assert.InRange(byRatio[2.00].Delivered / seconds, 99.0, 100.5);

        // Below capacity: delivered rate is C/73 per second, regardless of the 100/s offered.
        foreach (double ratio in new[] { 0.75, 0.50, 0.25 })
        {
            double expected = OfferedBytesPerSecond * ratio / PayloadBytes;
            double observed = byRatio[ratio].Delivered / seconds;
            Assert.InRange(observed, expected * 0.97, expected * 1.03);
        }

        // And the standing queue is the whole buffer. At C = 0.5x, serialization is 20 ms and the
        // buffer is 10 datagrams, so ~200 ms of pure self-inflicted delay on top of 'lan'.
        Assert.InRange(byRatio[0.50].P(0.50), 180.0, 210.0);
        Assert.InRange(byRatio[0.25].P(0.50), 380.0, 420.0);
    }

    /// <summary>
    /// <b>H2.</b> A sender admitting only what the link can carry delivers the same number of
    /// commands as the greedy sender at far lower delay — the queuing delay is self-inflicted and
    /// the sender owns it.
    ///
    /// Falsifier stated in the log: delivered count drops more than ~10%, or p95 falls by less than
    /// the buffer-drain time.
    /// </summary>
    [Fact]
    public void H2_AdmissionControlBuysBackTheStandingQueueAtNoCostInDeliveredCommands()
    {
        PrintHeader("H2 -- greedy vs open-loop admission, C = 0.5x offered, profile 'lan', seed 1");

        const double ratio = 0.50;
        const int buffer = 10;
        long capacity = (long)(OfferedBytesPerSecond * ratio);
        double seconds = (Steps - TransientSteps) * StepTicks / (double)TicksPerSecond;

        TrialResult greedy = Run(SenderKind.Greedy, capacity, buffer, capacity, "lan", seed: 1);
        PrintRow("greedy", ratio, buffer, greedy);

        TrialResult atCapacity = Run(SenderKind.RateLimited, capacity, buffer, capacity, "lan", seed: 1);
        PrintRow("rate-limited @1.00C", ratio, buffer, atCapacity);

        TrialResult belowCapacity = Run(
            SenderKind.RateLimited, capacity, buffer, (long)(capacity * 0.95), "lan", seed: 1);
        PrintRow("rate-limited @0.95C b1", ratio, buffer, belowCapacity);

        // Same run with a two-datagram bucket. The single-datagram bucket cannot carry fractional
        // credit -- cost and cap are the same quantity -- so it quantizes the achievable admitted
        // rate to offered/k. This row is here because the 0.95C figure above is that artifact, not a
        // property of admission control, and a reader comparing the two would otherwise conclude
        // that aiming just below capacity is expensive.
        TrialResult belowCapacityDeepBucket = Run(
            SenderKind.RateLimited, capacity, buffer, (long)(capacity * 0.95), "lan", seed: 1, burstDatagrams: 2);
        PrintRow("rate-limited @0.95C b2", ratio, buffer, belowCapacityDeepBucket);

        // Same delivered command rate, within a few percent.
        double greedyRate = greedy.Delivered / seconds;
        double limitedRate = atCapacity.Delivered / seconds;
        Assert.InRange(limitedRate, greedyRate * 0.95, greedyRate * 1.05);

        // The delivered command stream is no burstier: the mechanism is not buying its delay by
        // clumping deliveries somewhere the per-sample percentiles cannot see.
        Assert.True(
            atCapacity.MaxDeliveryGapMs <= greedy.MaxDeliveryGapMs + 1.0,
            $"greedy max delivery gap {greedy.MaxDeliveryGapMs:0.0} ms vs limited {atCapacity.MaxDeliveryGapMs:0.0} ms");

        // The quantization is the bucket's, not the mechanism's.
        Assert.True(
            belowCapacityDeepBucket.Delivered > belowCapacity.Delivered * 1.3,
            $"deep bucket delivered {belowCapacityDeepBucket.Delivered}, shallow {belowCapacity.Delivered}");

        // At a fraction of the delay. The buffer-drain time at this operating point is
        // 10 datagrams x 20 ms = 200 ms; recovering it should leave one serialization time plus the
        // profile's own 2 ms.
        Assert.True(
            atCapacity.P(0.95) < greedy.P(0.95) - 150.0,
            $"greedy p95 {greedy.P(0.95):0.0} ms vs limited p95 {atCapacity.P(0.95):0.0} ms");
        Assert.InRange(atCapacity.P(0.50), 0.0, 45.0);
    }

    /// <summary>
    /// <b>H2, continued.</b> The safety property, and the falsifier for the whole candidate: on an
    /// uncongested link the mechanism must cost nothing.
    /// </summary>
    [Fact]
    public void H2_OnAnUncongestedLink_AdmissionControlIsHarmless()
    {
        PrintHeader("H2b -- uncongested link (C = 2x offered), profile '50ms-5j', seed 1");

        const double ratio = 2.00;
        const int buffer = 10;
        long capacity = (long)(OfferedBytesPerSecond * ratio);
        double seconds = (Steps - TransientSteps) * StepTicks / (double)TicksPerSecond;

        TrialResult greedy = Run(SenderKind.Greedy, capacity, buffer, capacity, "50ms-5j", seed: 1);
        PrintRow("greedy", ratio, buffer, greedy);

        // Admitting at the link's own rate, which is twice the offered load: nothing is withheld.
        TrialResult limited = Run(SenderKind.RateLimited, capacity, buffer, capacity, "50ms-5j", seed: 1);
        PrintRow("rate-limited @1.00C", ratio, buffer, limited);

        TrialResult backoff = Run(SenderKind.BacklogBackoff, capacity, buffer, capacity, "50ms-5j", seed: 1);
        PrintRow("backlog-backoff", ratio, buffer, backoff);

        Assert.Equal(0, limited.SenderRefused);
        Assert.Equal(0, backoff.SenderRefused);
        Assert.Equal(greedy.Delivered, limited.Delivered);
        Assert.Equal(greedy.Delivered, backoff.Delivered);
        Assert.Equal(greedy.P(0.95), limited.P(0.95), 6);
        Assert.Equal(greedy.P(0.95), backoff.P(0.95), 6);
        Assert.InRange(greedy.Delivered / seconds, 99.0, 100.5);
    }

    /// <summary>
    /// <b>H3.</b> The controller that could actually run — one seeing only <c>Send</c> returning
    /// false — recovers the rate but not the delay, because a full buffer plus a matched rate
    /// produces no refusals to learn from. This is the bufferbloat result, reproduced here.
    ///
    /// Falsifier stated in the log: its p50 lands materially below greedy's.
    /// </summary>
    [Fact]
    public void H3_LossSignalledBackoffRecoversTheRateButNotTheDelay()
    {
        PrintHeader("H3 -- closed-loop vs open-loop, C = 0.5x offered, profile 'lan', seed 1");

        const double ratio = 0.50;
        const int buffer = 10;
        long capacity = (long)(OfferedBytesPerSecond * ratio);
        double seconds = (Steps - TransientSteps) * StepTicks / (double)TicksPerSecond;

        TrialResult greedy = Run(SenderKind.Greedy, capacity, buffer, capacity, "lan", seed: 1);
        TrialResult backoff = Run(SenderKind.BacklogBackoff, capacity, buffer, capacity, "lan", seed: 1);
        TrialResult oracle = Run(SenderKind.RateLimited, capacity, buffer, capacity, "lan", seed: 1);

        PrintRow("greedy", ratio, buffer, greedy);
        PrintRow("backlog-backoff", ratio, buffer, backoff);
        PrintRow("rate-limited @1.00C", ratio, buffer, oracle);

        _output.WriteLine(
            $"  delay recovered by backlog-backoff: {greedy.P(0.50) - backoff.P(0.50):0.0} ms of the " +
            $"{greedy.P(0.50) - oracle.P(0.50):0.0} ms available");

        // It does find the rate.
        Assert.InRange(backoff.Delivered / seconds, greedy.Delivered / seconds * 0.85, greedy.Delivered / seconds * 1.05);

        // The assertion is deliberately one-sided and loose: it pins the *shape* of the result
        // (the local signal recovers far less than the oracle) without pinning a number that a
        // controller-tuning change would have to chase.
        double available = greedy.P(0.50) - oracle.P(0.50);
        double recovered = greedy.P(0.50) - backoff.P(0.50);
        Assert.True(available > 100.0, $"expected a large recoverable queue, saw {available:0.0} ms");
        Assert.True(
            recovered < available * 0.5,
            $"loss-signalled backoff recovered {recovered:0.0} ms of {available:0.0} ms -- if this " +
            "fires, H3 is falsified and the mechanism needs explaining before it is believed");
    }

    /// <summary>
    /// Seed sensitivity. The bottleneck model is deterministic, so all variation comes from the
    /// profile's impairments; this runs the H2 comparison across five seeds on a jittered, lossy
    /// profile so the headline is not a single realization.
    /// </summary>
    [Fact]
    public void H2_HoldsAcrossSeedsOnAJitteredLossyProfile()
    {
        PrintHeader("H2c -- five seeds, C = 0.5x offered, profile '150ms-20j-0.5loss'");

        const double ratio = 0.50;
        const int buffer = 10;
        long capacity = (long)(OfferedBytesPerSecond * ratio);
        double seconds = (Steps - TransientSteps) * StepTicks / (double)TicksPerSecond;

        for (ulong seed = 1; seed <= 5; seed++)
        {
            TrialResult greedy = Run(SenderKind.Greedy, capacity, buffer, capacity, "150ms-20j-0.5loss", seed);
            TrialResult limited = Run(SenderKind.RateLimited, capacity, buffer, capacity, "150ms-20j-0.5loss", seed);
            TrialResult backoff = Run(SenderKind.BacklogBackoff, capacity, buffer, capacity, "150ms-20j-0.5loss", seed);

            PrintRow($"greedy seed={seed}", ratio, buffer, greedy);
            PrintRow($"rate-limited seed={seed}", ratio, buffer, limited);
            PrintRow($"backoff seed={seed}", ratio, buffer, backoff);

            Assert.InRange(limited.Delivered / seconds, greedy.Delivered / seconds * 0.93, greedy.Delivered / seconds * 1.07);
            Assert.True(
                limited.P(0.95) < greedy.P(0.95) - 150.0,
                $"seed {seed}: greedy p95 {greedy.P(0.95):0.0} vs limited p95 {limited.P(0.95):0.0}");
        }
    }

    /// <summary>
    /// Buffer depth is the whole story: the delay the sender can buy back is the buffer, so a
    /// shallow-buffered bottleneck leaves nothing on the table and a deep one leaves a great deal.
    /// This is the tradeoff-shape result rather than a percentage.
    /// </summary>
    [Fact]
    public void RecoverableDelayScalesWithBufferDepth()
    {
        PrintHeader("Buffer-depth response -- C = 0.5x offered, profile 'lan', seed 1");

        const double ratio = 0.50;
        long capacity = (long)(OfferedBytesPerSecond * ratio);

        var recovered = new List<(int Buffer, double Greedy, double Limited)>();
        foreach (int buffer in new[] { 2, 5, 10, 20, 40 })
        {
            TrialResult greedy = Run(SenderKind.Greedy, capacity, buffer, capacity, "lan", seed: 1);
            TrialResult limited = Run(SenderKind.RateLimited, capacity, buffer, capacity, "lan", seed: 1);
            PrintRow("greedy", ratio, buffer, greedy);
            PrintRow("rate-limited @1.00C", ratio, buffer, limited);
            recovered.Add((buffer, greedy.P(0.50), limited.P(0.50)));
        }

        // The limited sender's delay is flat in buffer depth -- it never builds a queue -- while the
        // greedy sender's grows linearly with it.
        double firstLimited = recovered[0].Limited;
        Assert.All(recovered, r => Assert.InRange(r.Limited, firstLimited - 25.0, firstLimited + 25.0));
        Assert.True(recovered[^1].Greedy > recovered[0].Greedy * 4.0);
    }
}
