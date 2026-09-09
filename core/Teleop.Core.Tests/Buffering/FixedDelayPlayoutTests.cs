using System;
using System.Numerics;
using Teleop.Core.Buffering;
using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;
using Xunit;

namespace Teleop.Core.Tests.Buffering;

/// <summary>
/// <c>fixed</c> buys reordering tolerance with delay, and these tests are organised around that
/// trade rather than around the API: what the budget costs every sample, what it recovers, and
/// where it stops recovering anything. The head-to-head against <c>immediate</c> at the bottom is
/// the axis's whole thesis in one assertion — same stream, same reordering, one policy loses the
/// samples and the other does not, and the difference is exactly the delay it paid.
/// </summary>
public class FixedDelayPlayoutTests
{
    private const long TicksPerSecond = 10_000_000;
    private const long OneMillisecond = TicksPerSecond / 1000;

    private static PlayoutPolicyConfig Config(long budgetTicks, int capacity = 16) =>
        new PlayoutPolicyConfig(
            historyCapacity: capacity, initialDelayBudgetTicks: budgetTicks, minDelayBudgetTicks: 0,
            maxDelayBudgetTicks: budgetTicks, targetPercentile: 0.95, delayWindowSamples: 64, delayProcessNoise: 0.01f,
            delayMeasurementNoise: 0.001f, maxAdaptationRatePerSecond: 0.0, lossWeight: 0.5);

    private static FixedDelayPlayout MakePolicy(
        out InMemoryMetricTracker metrics, long budgetTicks = 20 * OneMillisecond, int capacity = 16)
    {
        metrics = new InMemoryMetricTracker(capacity: 256);
        return new FixedDelayPlayout(Config(budgetTicks, capacity), metrics, new ManualClock(TicksPerSecond));
    }

    private static Stamped<Pose> Sample(long captureTicks, float x) =>
        new Stamped<Pose>(captureTicks, new Pose(new Vector3(x, 0f, 0f), Quaternion.Identity));

    private static double SumOf(InMemoryMetricTracker metrics, string name)
    {
        double total = 0.0;
        for (int i = 0; i < metrics.Count; i++)
        {
            if (metrics[i].Name == name)
            {
                total += metrics[i].Value;
            }
        }

        return total;
    }

    private static int CountOf(InMemoryMetricTracker metrics, string name)
    {
        int count = 0;
        for (int i = 0; i < metrics.Count; i++)
        {
            if (metrics[i].Name == name)
            {
                count++;
            }
        }

        return count;
    }

    [Fact]
    public void Constructor_RejectsANegativeBudget()
    {
        var metrics = new InMemoryMetricTracker(capacity: 8);

        // A negative budget would schedule a sample before it was captured.
        Assert.Throws<ArgumentException>(
            () => new FixedDelayPlayout(Config(-1), metrics, new ManualClock(TicksPerSecond)));
    }

    [Fact]
    public void TryDequeue_BeforeTheBudgetHasElapsed_HoldsTheSample()
    {
        FixedDelayPlayout policy = MakePolicy(out _, budgetTicks: 20 * OneMillisecond);

        policy.Enqueue(1, Sample(captureTicks: 0, x: 1f), arrivalTicks: 5 * OneMillisecond);

        Assert.False(policy.TryDequeue(19 * OneMillisecond, out _, out _, out _));
        Assert.Equal(1, policy.Diagnostics.BufferedCount);

        Assert.True(policy.TryDequeue(20 * OneMillisecond, out uint sequence, out _, out long playoutTicks));
        Assert.Equal(1u, sequence);
        Assert.Equal(20 * OneMillisecond, playoutTicks); // capture + budget, exactly.
    }

    [Fact]
    public void PlayoutDelay_IsTheHoldTime_NotTheTotalLatency()
    {
        FixedDelayPlayout policy = MakePolicy(out InMemoryMetricTracker metrics, budgetTicks: 20 * OneMillisecond);

        // Captured at 0, arrived after 5 ms of network delay, due at 20 ms: the buffer holds it for
        // 15 ms. The 5 ms belongs to owd_downlink_ms, which is why this metric measures from
        // arrival -- the two are separate, addable stages of docs/metrics.md section 2's breakdown.
        policy.Enqueue(1, Sample(captureTicks: 0, x: 1f), arrivalTicks: 5 * OneMillisecond);
        Assert.True(policy.TryDequeue(20 * OneMillisecond, out _, out _, out _));

        Assert.True(metrics.TryGetLatest("playout_delay_ms", out double delayMs, out _));
        Assert.Equal(15.0, delayMs, 6);

        Assert.True(metrics.TryGetLatest("playout_budget_ms", out double budgetMs, out _));
        Assert.Equal(20.0, budgetMs, 6);
    }

    [Fact]
    public void MetricsAreStampedAtPlayout_NotAtThePollTime()
    {
        FixedDelayPlayout policy = MakePolicy(out InMemoryMetricTracker metrics, budgetTicks: 20 * OneMillisecond);

        policy.Enqueue(1, Sample(captureTicks: 0, x: 1f), arrivalTicks: 5 * OneMillisecond);

        // The host polls late -- 30 ms, not the 20 ms the sample was due at. Reporting the poll time
        // would fold host frame time into every playout figure.
        Assert.True(policy.TryDequeue(30 * OneMillisecond, out _, out _, out long playoutTicks));
        Assert.Equal(20 * OneMillisecond, playoutTicks);

        Assert.True(metrics.TryGetLatest("playout_delay_ms", out double delayMs, out long stampTicks));
        Assert.Equal(15.0, delayMs, 6);
        Assert.Equal(20 * OneMillisecond, stampTicks);
    }

    /// <summary>
    /// The reason to pay the delay at all: a sample that arrives out of order but still inside the
    /// budget window is reinserted and released in its proper place, losing nothing.
    /// </summary>
    [Fact]
    public void Enqueue_OutOfOrderButWithinTheBudget_IsReleasedInCaptureOrder()
    {
        FixedDelayPlayout policy = MakePolicy(out _, budgetTicks: 20 * OneMillisecond);

        // Sequence 2 was captured later but arrives first.
        policy.Enqueue(2, Sample(captureTicks: 10 * OneMillisecond, x: 2f), arrivalTicks: 12 * OneMillisecond);
        policy.Enqueue(1, Sample(captureTicks: 5 * OneMillisecond, x: 1f), arrivalTicks: 14 * OneMillisecond);

        // Both are due by 30 ms. Order out is capture order, not arrival order.
        Assert.True(policy.TryDequeue(30 * OneMillisecond, out uint first, out _, out _));
        Assert.True(policy.TryDequeue(30 * OneMillisecond, out uint second, out _, out _));

        Assert.Equal(1u, first);
        Assert.Equal(2u, second);
        Assert.Equal(0.0, policy.Diagnostics.LateArrivalRate, 6);
    }

    [Fact]
    public void Enqueue_OutOfOrderPastTheBudget_IsCountedLate()
    {
        FixedDelayPlayout policy = MakePolicy(out InMemoryMetricTracker metrics, budgetTicks: 20 * OneMillisecond);

        policy.Enqueue(2, Sample(captureTicks: 10 * OneMillisecond, x: 2f), arrivalTicks: 12 * OneMillisecond);
        Assert.True(policy.TryDequeue(30 * OneMillisecond, out _, out _, out _));

        // Captured before the sample already played, so there is nowhere left to put it.
        policy.Enqueue(1, Sample(captureTicks: 5 * OneMillisecond, x: 1f), arrivalTicks: 31 * OneMillisecond);

        Assert.False(policy.TryDequeue(31 * OneMillisecond, out _, out _, out _));
        Assert.Equal(0.5, policy.Diagnostics.LateArrivalRate, 6);
        Assert.Equal(1, CountOf(metrics, "playout_late"));
    }

    /// <summary>
    /// A capacity too small for the budget makes the buffer discard samples it had room to hold in
    /// principle, and the late rate then measures the capacity rather than the policy. Counting
    /// those as late rather than dropping them silently is what makes the mistake visible; the
    /// sweep also rejects the configuration up front.
    /// </summary>
    [Fact]
    public void Enqueue_WhenTheBufferIsFull_IsCountedLateRatherThanDroppedSilently()
    {
        FixedDelayPlayout policy = MakePolicy(out _, budgetTicks: 100 * OneMillisecond, capacity: 2);

        policy.Enqueue(1, Sample(1 * OneMillisecond, 1f), arrivalTicks: 2 * OneMillisecond);
        policy.Enqueue(2, Sample(2 * OneMillisecond, 2f), arrivalTicks: 3 * OneMillisecond);
        policy.Enqueue(3, Sample(3 * OneMillisecond, 3f), arrivalTicks: 4 * OneMillisecond);

        Assert.Equal(2, policy.Diagnostics.BufferedCount);
        Assert.Equal(1f, policy.Diagnostics.OccupancyFraction, 4);
        Assert.Equal(1.0 / 3.0, policy.Diagnostics.LateArrivalRate, 6);
    }

    [Fact]
    public void AZeroBudget_BehavesExactlyLikeImmediate()
    {
        var fixedMetrics = new InMemoryMetricTracker(capacity: 256);
        var immediateMetrics = new InMemoryMetricTracker(capacity: 256);
        var clock = new ManualClock(TicksPerSecond);
        var fixedPolicy = new FixedDelayPlayout(Config(0), fixedMetrics, clock);
        var immediatePolicy = new ImmediatePlayout(Config(0), immediateMetrics, clock);

        Assert.Equal(Drive(fixedPolicy), Drive(immediatePolicy));

        static string Drive(IPlayoutPolicy<Pose> policy)
        {
            var log = new System.Text.StringBuilder();
            for (int i = 0; i < 10; i++)
            {
                long capture = (i % 3 == 2 ? i - 2 : i) * OneMillisecond;
                long arrival = i * OneMillisecond + 5;
                policy.Enqueue((uint)i, Sample(capture, i), arrival);

                while (policy.TryDequeue(arrival, out uint sequence, out Pose value, out long playout))
                {
                    log.Append($"{sequence}:{value.Position.X}:{playout};");
                }
            }

            log.Append(policy.Diagnostics.ToString());
            return log.ToString();
        }
    }

    /// <summary>
    /// The axis in one test. Identical reordered stream through both baselines: <c>immediate</c>
    /// discards every out-of-order sample because it has no window to reinsert them into;
    /// <c>fixed</c> keeps all of them, and the price is a delay every sample pays whether it needed
    /// the wait or not. Neither is "better" — they are two points on the curve, which is exactly
    /// why <c>docs/metrics.md</c> section 8 rule 1 wants both reported.
    /// </summary>
    [Fact]
    public void AgainstImmediate_OnAReorderedStream_TradesDelayForTheSamplesImmediateLoses()
    {
        var clock = new ManualClock(TicksPerSecond);
        var fixedMetrics = new InMemoryMetricTracker(capacity: 512);
        var immediateMetrics = new InMemoryMetricTracker(capacity: 512);
        var fixedPolicy = new FixedDelayPlayout(Config(20 * OneMillisecond), fixedMetrics, clock);
        var immediatePolicy = new ImmediatePlayout(Config(0), immediateMetrics, clock);

        // Ten samples, every adjacent pair swapped on the wire: captured in order 1,0,3,2,5,4...
        for (int i = 0; i < 10; i++)
        {
            int captureIndex = i % 2 == 0 ? i + 1 : i - 1;
            var sample = Sample(captureIndex * 10 * OneMillisecond, captureIndex);
            // +15 ms, not +1: at +1 the first sample would arrive before it was captured, which
            // no transport can do and which makes `immediate` hold it rather than release it.
            long arrival = i * 10 * OneMillisecond + 15 * OneMillisecond;

            fixedPolicy.Enqueue((uint)captureIndex, sample, arrival);
            immediatePolicy.Enqueue((uint)captureIndex, sample, arrival);

            while (fixedPolicy.TryDequeue(arrival, out _, out _, out _))
            {
            }

            while (immediatePolicy.TryDequeue(arrival, out _, out _, out _))
            {
            }
        }

        // immediate loses one of every swapped pair; fixed loses none.
        Assert.True(immediatePolicy.Diagnostics.LateArrivalRate > 0.4);
        Assert.Equal(0.0, fixedPolicy.Diagnostics.LateArrivalRate, 6);

        // And that is bought with delay. Summed over the run, not read off the last sample: the
        // final sample happens to arrive after its own due instant and so waits zero, which is a
        // real property of a fixed budget and exactly why one sample is not an operating point.
        double fixedDelayMs = SumOf(fixedMetrics, "playout_delay_ms");
        double immediateDelayMs = SumOf(immediateMetrics, "playout_delay_ms");
        Assert.True(
            fixedDelayMs > immediateDelayMs,
            $"fixed buffered {fixedDelayMs}ms total, immediate {immediateDelayMs}ms");
        Assert.Equal(0.0, immediateDelayMs, 6); // A zero buffer adds nothing, by construction.
    }

    [Fact]
    public void UnderrunCount_CountsAStarvedDrainOnce()
    {
        FixedDelayPlayout policy = MakePolicy(out InMemoryMetricTracker metrics, budgetTicks: 20 * OneMillisecond);

        policy.Enqueue(1, Sample(0, 1f), arrivalTicks: OneMillisecond);
        Assert.True(policy.TryDequeue(20 * OneMillisecond, out _, out _, out _));
        Assert.False(policy.TryDequeue(20 * OneMillisecond, out _, out _, out _));
        Assert.Equal(0, policy.Diagnostics.UnderrunCount);

        Assert.False(policy.TryDequeue(30 * OneMillisecond, out _, out _, out _));
        Assert.Equal(1, policy.Diagnostics.UnderrunCount);
        Assert.Equal(1, CountOf(metrics, "playout_underrun"));
    }

    /// <summary>
    /// Holding a sample that is not yet due is the steady state this policy is designed to be in,
    /// and must never be counted as starvation — otherwise <c>fixed</c> scores 100% underrun on a
    /// perfectly healthy stream and the counter is worthless.
    /// </summary>
    [Fact]
    public void UnderrunCount_DoesNotCountADrainThatFoundSomethingNotYetDue()
    {
        FixedDelayPlayout policy = MakePolicy(out _, budgetTicks: 20 * OneMillisecond);

        policy.Enqueue(1, Sample(0, 1f), arrivalTicks: OneMillisecond);
        Assert.True(policy.TryDequeue(20 * OneMillisecond, out _, out _, out _));

        policy.Enqueue(2, Sample(30 * OneMillisecond, 2f), arrivalTicks: 31 * OneMillisecond);
        Assert.False(policy.TryDequeue(31 * OneMillisecond, out _, out _, out _));

        Assert.Equal(1, policy.Diagnostics.BufferedCount);
        Assert.Equal(0, policy.Diagnostics.UnderrunCount);
    }

    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        FixedDelayPlayout policy = MakePolicy(out _, budgetTicks: 20 * OneMillisecond);

        policy.Enqueue(1, Sample(0, 1f), arrivalTicks: OneMillisecond);
        Assert.True(policy.TryDequeue(20 * OneMillisecond, out _, out _, out _));
        policy.Enqueue(2, Sample(50 * OneMillisecond, 2f), arrivalTicks: 51 * OneMillisecond);

        policy.Reset();

        PlayoutPolicyDiagnostics diagnostics = policy.Diagnostics;
        Assert.Equal(0, diagnostics.BufferedCount);
        Assert.Equal(0, diagnostics.UnderrunCount);
        Assert.Equal(0, diagnostics.DuplicatesRejected);
        Assert.Equal(0.0, diagnostics.LateArrivalRate, 6);

        // The budget is configuration, not state: it survives, unlike everything above.
        Assert.Equal(20 * OneMillisecond, diagnostics.DelayBudgetTicks);
    }

    [Fact]
    public void Reset_MakesASecondRunOfIdenticalInputProduceIdenticalOutput()
    {
        FixedDelayPlayout policy = MakePolicy(out _, budgetTicks: 20 * OneMillisecond);

        string first = Drive(policy);
        policy.Reset();
        string second = Drive(policy);

        Assert.Equal(first, second);

        static string Drive(FixedDelayPlayout policy)
        {
            var log = new System.Text.StringBuilder();
            for (int i = 0; i < 12; i++)
            {
                long capture = (i % 3 == 2 ? i - 2 : i) * 10 * OneMillisecond;
                long arrival = i * 10 * OneMillisecond + OneMillisecond;
                policy.Enqueue((uint)i, Sample(capture, i), arrival);

                while (policy.TryDequeue(arrival, out uint sequence, out Pose value, out long playout))
                {
                    log.Append($"{sequence}:{value.Position.X}:{playout};");
                }
            }

            log.Append(policy.Diagnostics.ToString());
            return log.ToString();
        }
    }

    [Fact]
    public void Enqueue_AndTryDequeue_Allocate_Zero_Bytes()
    {
        FixedDelayPlayout policy = MakePolicy(out _, budgetTicks: 20 * OneMillisecond);
        uint sequence = 0;

        AllocationAssert.Zero(() =>
        {
            sequence++;
            long capture = sequence * OneMillisecond;
            policy.Enqueue(sequence, Sample(capture, sequence), arrivalTicks: capture + 1);
            policy.TryDequeue(capture + 20 * OneMillisecond, out _, out _, out _);
        });
    }
}
