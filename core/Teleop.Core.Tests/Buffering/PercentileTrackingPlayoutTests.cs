using System;
using System.Numerics;
using Teleop.Core.Buffering;
using Teleop.Core.Metrics;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;
using Xunit;

namespace Teleop.Core.Tests.Buffering;

/// <summary>
/// The assertions that matter here are about the <i>estimator</i>, not the buffer — ordering,
/// duplicates and underruns are `PlayoutSampleBuffer`'s job and are covered through the two
/// baselines. What is specific to this policy is where the budget lands, how it moves, and that it
/// cannot ring.
///
/// Two of these tests exist to hold a claim made elsewhere honest:
/// <c>AtPercentileOne_ReproducesMaxOfTheLastW_Exactly</c> pins the correspondence with
/// <c>analysis/playout_bounds.py</c>, and <c>OnABimodalDelayDistribution_...</c> pins the reason
/// this is an order statistic rather than a mean.
/// </summary>
public class PercentileTrackingPlayoutTests
{
    private const long TicksPerSecond = 10_000_000;
    private const long OneMillisecond = TicksPerSecond / 1000;

    private static PlayoutPolicyConfig Config(
        double percentile = 0.95,
        int window = 4,
        long initialBudgetTicks = 5 * OneMillisecond,
        long minBudgetTicks = 0,
        long maxBudgetTicks = 1000 * OneMillisecond,
        int capacity = 64) =>
        new PlayoutPolicyConfig(
            historyCapacity: capacity, initialDelayBudgetTicks: initialBudgetTicks,
            minDelayBudgetTicks: minBudgetTicks, maxDelayBudgetTicks: maxBudgetTicks,
            targetPercentile: percentile, delayWindowSamples: window, delayProcessNoise: 0.01f,
            delayMeasurementNoise: 0.001f, maxAdaptationRatePerSecond: 0.0, lossWeight: 0.5);

    private static PercentileTrackingPlayout MakePolicy(PlayoutPolicyConfig config) =>
        new PercentileTrackingPlayout(
            config, new InMemoryMetricTracker(capacity: 1024), new ManualClock(TicksPerSecond));

    /// <summary>
    /// Monotonic across every <see cref="Feed"/> call in one test — xUnit constructs a fresh
    /// instance per test, so this cannot leak between them. Capture stamps must strictly increase
    /// across the whole test, not just within one call, or samples are discarded for arriving out
    /// of order and the assertions stop being about the estimator alone.
    /// </summary>
    private int _nextIndex;

    /// <summary>
    /// Feeds one sample per step with a chosen one-way delay, draining as a host would.
    /// </summary>
    private void Feed(PercentileTrackingPlayout policy, params long[] delaysMs)
    {
        for (int i = 0; i < delaysMs.Length; i++)
        {
            long capture = _nextIndex++ * 10 * OneMillisecond;
            long arrival = capture + (delaysMs[i] * OneMillisecond);
            policy.Enqueue(
                (uint)_nextIndex,
                new Stamped<Pose>(capture, new Pose(new Vector3(i, 0f, 0f), Quaternion.Identity)),
                arrival);

            while (policy.TryDequeue(arrival, out _, out _, out _))
            {
            }
        }
    }

    private static double BudgetMs(PercentileTrackingPlayout policy) =>
        policy.Diagnostics.DelayBudgetTicks * 1000.0 / TicksPerSecond;

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void Constructor_RejectsANonPositiveWindow(int window)
    {
        Assert.Throws<ArgumentException>(() => MakePolicy(Config(window: window)));
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-0.5)]
    [InlineData(1.5)]
    public void Constructor_RejectsAPercentileOutsideZeroToOne(double percentile)
    {
        Assert.Throws<ArgumentException>(() => MakePolicy(Config(percentile: percentile)));
    }

    [Fact]
    public void Constructor_AcceptsPercentileOne_BecauseItIsTheWindowMaximum()
    {
        // Not a degenerate case: p = 1.0 is the operating point the offline analysis measured.
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 1.0));

        Assert.NotNull(policy);
    }

    [Fact]
    public void Constructor_RejectsAnInvertedBudgetClamp()
    {
        Assert.Throws<ArgumentException>(
            () => MakePolicy(Config(minBudgetTicks: 50 * OneMillisecond, maxBudgetTicks: 10 * OneMillisecond)));
    }

    [Fact]
    public void BeforeTheWindowIsFull_TheBudgetIsTheConfiguredStartingValue()
    {
        PercentileTrackingPlayout policy = MakePolicy(Config(window: 4, initialBudgetTicks: 5 * OneMillisecond));

        Assert.Equal(5.0, BudgetMs(policy), 6);

        // Three of four: a quantile over a partial window is not the quantile that was asked for,
        // so it must not drive the budget yet.
        Feed(policy, 40, 40, 40);
        Assert.Equal(5.0, BudgetMs(policy), 6);

        Feed(policy, 40);
        Assert.Equal(40.0, BudgetMs(policy), 6);
    }

    [Fact]
    public void OnceFull_TheBudgetIsTheNearestRankQuantileOfTheWindow()
    {
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 0.5, window: 4));

        // Sorted [10, 20, 30, 40]; nearest rank at p=0.5 over 4 samples is index 2.
        Feed(policy, 30, 10, 40, 20);

        Assert.Equal(30.0, BudgetMs(policy), 6);
    }

    /// <summary>
    /// The claim this whole policy is built on. `analysis/playout_bounds.py` scored
    /// "budget = max of the last w delays" at 47-61% better than a matched-loss fixed budget; this
    /// asserts that policy is reachable here exactly, not approximately, so the offline result can
    /// be reproduced in the pipeline rather than only cited at it.
    /// </summary>
    [Fact]
    public void AtPercentileOne_ReproducesMaxOfTheLastW_Exactly()
    {
        const int window = 5;
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 1.0, window: window));

        long[] delays = { 20, 21, 250, 19, 20, 22, 21, 240, 20, 18, 19, 20 };

        for (int i = 0; i < delays.Length; i++)
        {
            Feed(policy, delays[i]);

            if (i + 1 < window)
            {
                continue;
            }

            long expected = 0;
            for (int j = i - window + 1; j <= i; j++)
            {
                expected = Math.Max(expected, delays[j]);
            }

            Assert.Equal((double)expected, BudgetMs(policy), 6);
        }
    }

    /// <summary>
    /// Requirement 6 (Buffering/CLAUDE.md): a step change in delay settles within a stated bound
    /// rather than ringing. The bound here is structural — an order statistic over a sliding window
    /// cannot overshoot, because the budget never influences the delays it is estimated from — so
    /// this asserts both halves: monotone approach, and settled within exactly `window` samples.
    /// </summary>
    [Fact]
    public void AfterAStepChangeInDelay_TheBudgetMovesMonotonically_AndSettlesWithinOneWindow()
    {
        const int window = 8;
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 1.0, window: window));

        for (int i = 0; i < window; i++)
        {
            Feed(policy, 20);
        }

        Assert.Equal(20.0, BudgetMs(policy), 6);

        // Step up. Monotone non-decreasing, settled by the time the window has turned over.
        double previous = BudgetMs(policy);
        for (int i = 0; i < window; i++)
        {
            Feed(policy, 200);
            double current = BudgetMs(policy);
            Assert.True(current >= previous, $"budget fell from {previous} to {current} during a step up");
            Assert.True(current <= 200.0, $"budget overshot to {current}, above the new delay level");
            previous = current;
        }

        Assert.Equal(200.0, BudgetMs(policy), 6);

        // Step back down. Monotone non-increasing, and settled within one more window -- the old
        // high samples have to age out, which is exactly `window` observations and no more.
        previous = BudgetMs(policy);
        for (int i = 0; i < window; i++)
        {
            Feed(policy, 20);
            double current = BudgetMs(policy);
            Assert.True(current <= previous, $"budget rose from {previous} to {current} during a step down");
            Assert.True(current >= 20.0, $"budget undershot to {current}, below the new delay level");
            previous = current;
        }

        Assert.Equal(20.0, BudgetMs(policy), 6);
    }

    /// <summary>
    /// Why an order statistic and not a mean-plus-sigma filter. The delay distribution this project
    /// owns is sharply bimodal — `synthetic-burst` has a baseline cluster near 20 ms, a burst
    /// cluster near 250 ms, and a 128 ms empty band between them. A mean of that sample sits in the
    /// band, describing a delay no packet ever had. A quantile lands in whichever mode holds it.
    /// </summary>
    [Fact]
    public void OnABimodalDelayDistribution_TheBudgetLandsInAMode_NotTheEmptyBandAMeanWouldPick()
    {
        const int window = 10;
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 0.5, window: window));

        // Seven baseline samples, three burst samples: mean ~89 ms, squarely inside the empty band.
        Feed(policy, 20, 20, 250, 21, 20, 250, 19, 20, 250, 20);

        double budget = BudgetMs(policy);
        Assert.True(budget <= 25.0, $"budget {budget}ms should sit in the baseline mode at p50");
        Assert.True(budget >= 19.0, $"budget {budget}ms fell below every observed baseline delay");
    }

    [Fact]
    public void TheBudgetIsClampedToTheConfiguredFloorAndCeiling()
    {
        PercentileTrackingPlayout ceiling = MakePolicy(
            Config(percentile: 1.0, window: 2, maxBudgetTicks: 50 * OneMillisecond));
        Feed(ceiling, 300, 300);
        Assert.Equal(50.0, BudgetMs(ceiling), 6);

        PercentileTrackingPlayout floor = MakePolicy(
            Config(percentile: 1.0, window: 2, minBudgetTicks: 30 * OneMillisecond));
        Feed(floor, 5, 5);
        Assert.Equal(30.0, BudgetMs(floor), 6);
    }

    /// <summary>
    /// A quiet link produces long runs of identical delays, so the sorted window is mostly
    /// duplicates. Removing "the first entry equal to the evicted value" is correct precisely
    /// because equal entries are indistinguishable — but it is also where an off-by-one in the
    /// insert/remove pair would hide, since every wrong answer still looks plausible.
    /// </summary>
    [Fact]
    public void ARunOfIdenticalDelays_KeepsTheSortedWindowConsistent()
    {
        const int window = 4;
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 1.0, window: window));

        Feed(policy, 20, 20, 20, 20);
        Assert.Equal(20.0, BudgetMs(policy), 6);

        // One spike, then it ages out over exactly `window` samples and the budget returns.
        Feed(policy, 90);
        Assert.Equal(90.0, BudgetMs(policy), 6);

        Feed(policy, 20, 20, 20);
        Assert.Equal(90.0, BudgetMs(policy), 6); // still in the window

        Feed(policy, 20);
        Assert.Equal(20.0, BudgetMs(policy), 6); // aged out
    }

    [Fact]
    public void ANegativeObservedDelay_IsClampedRatherThanRejected()
    {
        // ClockSync converts the capture stamp from the robot's domain and its offset estimate
        // moves, so a small negative delay is reachable early in a trial. Dropping the observation
        // would bias the window toward whatever the estimate happened to be.
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 1.0, window: 2));

        policy.Enqueue(0, new Stamped<Pose>(100 * OneMillisecond, Pose.Identity), arrivalTicks: 90 * OneMillisecond);
        policy.Enqueue(1, new Stamped<Pose>(110 * OneMillisecond, Pose.Identity), arrivalTicks: 110 * OneMillisecond);

        Assert.Equal(0.0, BudgetMs(policy), 6);
    }

    [Fact]
    public void Reset_RestoresTheStartingBudgetAndForgetsTheWindow()
    {
        PercentileTrackingPlayout policy = MakePolicy(
            Config(percentile: 1.0, window: 4, initialBudgetTicks: 5 * OneMillisecond));

        Feed(policy, 200, 200, 200, 200);
        Assert.Equal(200.0, BudgetMs(policy), 6);

        policy.Reset();

        Assert.Equal(5.0, BudgetMs(policy), 6);
        Assert.Equal(0, policy.Diagnostics.BufferedCount);
        Assert.Equal(0.0, policy.Diagnostics.LateArrivalRate, 6);

        // And the window is genuinely empty, not merely re-clamped: three samples must not be
        // enough to start tracking again.
        Feed(policy, 40, 40, 40);
        Assert.Equal(5.0, BudgetMs(policy), 6);
    }

    [Fact]
    public void Reset_MakesASecondRunOfIdenticalInputProduceIdenticalOutput()
    {
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 0.9, window: 8));

        string first = Drive(policy);
        policy.Reset();
        string second = Drive(policy);

        Assert.Equal(first, second);

        static string Drive(PercentileTrackingPlayout policy)
        {
            var log = new System.Text.StringBuilder();
            long[] delays = { 20, 21, 250, 19, 20, 240, 21, 20, 18, 250, 19, 20, 22, 20, 19, 21 };

            for (int i = 0; i < delays.Length; i++)
            {
                long capture = i * 10 * OneMillisecond;
                long arrival = capture + (delays[i] * OneMillisecond);
                policy.Enqueue((uint)i, new Stamped<Pose>(capture, Pose.Identity), arrival);

                while (policy.TryDequeue(arrival, out uint sequence, out _, out long playout))
                {
                    log.Append($"{sequence}:{playout};");
                }

                log.Append($"b{policy.Diagnostics.DelayBudgetTicks};");
            }

            return log.ToString();
        }
    }

    [Fact]
    public void Enqueue_AndTryDequeue_Allocate_Zero_Bytes()
    {
        // A full window on the hot path: every call does a binary search plus two Array.Copy
        // shifts, which is the steady state, not the cheap warm-up path.
        PercentileTrackingPlayout policy = MakePolicy(Config(percentile: 0.95, window: 64));
        Feed(policy, new long[64]);

        uint sequence = (uint)_nextIndex + 1000;
        AllocationAssert.Zero(() =>
        {
            sequence++;
            long capture = sequence * 10 * OneMillisecond;
            policy.Enqueue(sequence, new Stamped<Pose>(capture, Pose.Identity), capture + OneMillisecond);
            policy.TryDequeue(capture + 100 * OneMillisecond, out _, out _, out _);
        });
    }
}
