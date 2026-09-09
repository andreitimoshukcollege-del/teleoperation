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
/// <c>immediate</c> is the zero-latency end of the latency/loss curve, so almost every assertion
/// here is about what it does <i>not</i> do: it must not hold a sample, must not add delay, and
/// must not report a budget. The two that are about what it does are the interesting ones — it
/// still enforces capture order and still rejects duplicates, because <c>IPlayoutPolicy</c>
/// clause 2 requires that of every implementation and a baseline that quietly tolerated reordering
/// would make every buffered policy look like it had introduced loss it did not introduce.
/// </summary>
public class ImmediatePlayoutTests
{
    private const long TicksPerSecond = 10_000_000;
    private const long OneMillisecond = TicksPerSecond / 1000;

    private static ImmediatePlayout MakePolicy(out InMemoryMetricTracker metrics, int capacity = 8)
    {
        metrics = new InMemoryMetricTracker(capacity: 256);
        var config = new PlayoutPolicyConfig(
            historyCapacity: capacity, initialDelayBudgetTicks: 0, minDelayBudgetTicks: 0,
            maxDelayBudgetTicks: 0, targetPercentile: 0.95, delayProcessNoise: 0.01f,
            delayMeasurementNoise: 0.001f, maxAdaptationRatePerSecond: 0.0, lossWeight: 0.5);
        return new ImmediatePlayout(config, metrics, new ManualClock(TicksPerSecond));
    }

    private static Stamped<Pose> Sample(long captureTicks, float x) =>
        new Stamped<Pose>(captureTicks, new Pose(new Vector3(x, 0f, 0f), Quaternion.Identity));

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
    public void Constructor_RejectsNonPositiveCapacity()
    {
        var metrics = new InMemoryMetricTracker(capacity: 8);
        var config = new PlayoutPolicyConfig(
            historyCapacity: 0, initialDelayBudgetTicks: 0, minDelayBudgetTicks: 0,
            maxDelayBudgetTicks: 0, targetPercentile: 0.95, delayProcessNoise: 0.01f,
            delayMeasurementNoise: 0.001f, maxAdaptationRatePerSecond: 0.0, lossWeight: 0.5);

        Assert.Throws<ArgumentException>(
            () => new ImmediatePlayout(config, metrics, new ManualClock(TicksPerSecond)));
    }

    [Fact]
    public void Diagnostics_BeforeAnyEnqueue_IsEmpty()
    {
        ImmediatePlayout policy = MakePolicy(out _);

        Assert.Equal(PlayoutPolicyDiagnostics.Empty.ToString(), policy.Diagnostics.ToString());
    }

    [Fact]
    public void TryDequeue_AfterEnqueue_ReleasesAtArrival_AddingNoDelay()
    {
        ImmediatePlayout policy = MakePolicy(out InMemoryMetricTracker metrics);

        policy.Enqueue(sequence: 1, Sample(captureTicks: 100, x: 1f), arrivalTicks: 500);

        Assert.True(policy.TryDequeue(nowTicks: 500, out uint sequence, out Pose value, out long playoutTicks));
        Assert.Equal(1u, sequence);
        Assert.Equal(1f, value.Position.X, 4);

        // Due at capture, which is long past, so it is scheduled for the instant it actually
        // arrived -- never earlier, or t_playout - t_recv would be negative.
        Assert.Equal(500, playoutTicks);

        Assert.True(metrics.TryGetLatest("playout_delay_ms", out double delayMs, out _));
        Assert.Equal(0.0, delayMs, 6);
        Assert.True(metrics.TryGetLatest("playout_budget_ms", out double budgetMs, out _));
        Assert.Equal(0.0, budgetMs, 6);
    }

    [Fact]
    public void Diagnostics_BudgetIsAlwaysZero_AndBufferDrainsCompletely()
    {
        ImmediatePlayout policy = MakePolicy(out _);

        policy.Enqueue(1, Sample(100, 1f), arrivalTicks: 500);
        Assert.Equal(1, policy.Diagnostics.BufferedCount);

        Assert.True(policy.TryDequeue(500, out _, out _, out _));

        PlayoutPolicyDiagnostics diagnostics = policy.Diagnostics;
        Assert.Equal(0, diagnostics.DelayBudgetTicks);
        Assert.Equal(0, diagnostics.BufferedCount);
        Assert.Equal(0f, diagnostics.OccupancyFraction, 4);
        Assert.Equal(0.0, diagnostics.LateArrivalRate, 6);
    }

    /// <summary>
    /// The defining behaviour of a zero buffer: with no budget there is no window in which an
    /// out-of-order sample can be reinserted, so preserving capture order costs it the sample.
    /// This rate <i>is</i> the operating point <c>fixed</c> and every adaptive policy are trying to
    /// beat.
    /// </summary>
    [Fact]
    public void Enqueue_OutOfOrderAfterANewerSampleReleased_IsDiscardedAndCountedLate()
    {
        ImmediatePlayout policy = MakePolicy(out InMemoryMetricTracker metrics);

        policy.Enqueue(2, Sample(captureTicks: 200, x: 2f), arrivalTicks: 500);
        Assert.True(policy.TryDequeue(500, out _, out _, out _));

        // Captured earlier, arrived later -- the shape reordering actually takes.
        policy.Enqueue(1, Sample(captureTicks: 100, x: 1f), arrivalTicks: 510);

        Assert.False(policy.TryDequeue(510, out _, out _, out _));
        Assert.Equal(0.5, policy.Diagnostics.LateArrivalRate, 6);
        Assert.Equal(1, CountOf(metrics, "playout_late"));
    }

    [Fact]
    public void Enqueue_SameSequenceTwice_IsReleasedOnlyOnce()
    {
        ImmediatePlayout policy = MakePolicy(out InMemoryMetricTracker metrics);

        policy.Enqueue(1, Sample(100, 1f), arrivalTicks: 500);
        policy.Enqueue(1, Sample(100, 1f), arrivalTicks: 505);

        Assert.True(policy.TryDequeue(505, out _, out _, out _));
        Assert.False(policy.TryDequeue(505, out _, out _, out _));

        Assert.Equal(1, policy.Diagnostics.DuplicatesRejected);

        // A duplicate is a transport property, not an operating point -- it must not inflate the
        // late rate, which is what a buffered policy is judged on.
        Assert.Equal(0.0, policy.Diagnostics.LateArrivalRate, 6);
        Assert.Equal(0, CountOf(metrics, "playout_late"));
    }

    [Fact]
    public void Enqueue_DuplicateOfAnAlreadyReleasedSequence_IsAlsoRejected()
    {
        ImmediatePlayout policy = MakePolicy(out _);

        policy.Enqueue(1, Sample(100, 1f), arrivalTicks: 500);
        Assert.True(policy.TryDequeue(500, out _, out _, out _));

        // Checking only the live buffer would miss this: the original is long gone from it.
        policy.Enqueue(1, Sample(100, 1f), arrivalTicks: 600);

        Assert.False(policy.TryDequeue(600, out _, out _, out _));
        Assert.Equal(1, policy.Diagnostics.DuplicatesRejected);
    }

    /// <summary>
    /// One underrun per drain that gave the pipeline nothing, not one per false return — otherwise
    /// a policy whose buffer is empty by construction reports an underrun on every single step and
    /// the counter says nothing about starvation.
    /// </summary>
    [Fact]
    public void UnderrunCount_CountsDrainsThatReleasedNothing_NotFalseReturns()
    {
        ImmediatePlayout policy = MakePolicy(out InMemoryMetricTracker metrics);

        // A drain of a healthy step: one release, then the false return that ends the loop.
        policy.Enqueue(1, Sample(100, 1f), arrivalTicks: 500);
        Assert.True(policy.TryDequeue(500, out _, out _, out _));
        Assert.False(policy.TryDequeue(500, out _, out _, out _));
        Assert.Equal(0, policy.Diagnostics.UnderrunCount);

        // A starved step: drained, nothing to give.
        Assert.False(policy.TryDequeue(600, out _, out _, out _));
        Assert.Equal(1, policy.Diagnostics.UnderrunCount);
        Assert.Equal(1, CountOf(metrics, "playout_underrun"));

        // A second starved step is a second event: the loop exits on the first false, so one false
        // per starved drain is exactly one underrun per starved step.
        Assert.False(policy.TryDequeue(700, out _, out _, out _));
        Assert.Equal(2, policy.Diagnostics.UnderrunCount);
    }

    [Fact]
    public void TryDequeue_AfterAGapOfSeveralHundredMilliseconds_StillReleases()
    {
        ImmediatePlayout policy = MakePolicy(out _);

        policy.Enqueue(1, Sample(100, 1f), arrivalTicks: 500);
        Assert.True(policy.TryDequeue(500, out _, out _, out _));

        long muchLater = 500 + 500 * OneMillisecond;
        policy.Enqueue(2, Sample(captureTicks: muchLater - 10, x: 2f), arrivalTicks: muchLater);

        Assert.True(policy.TryDequeue(muchLater, out _, out Pose value, out _));
        Assert.Equal(2f, value.Position.X, 4);
    }

    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        ImmediatePlayout policy = MakePolicy(out _);

        policy.Enqueue(1, Sample(200, 1f), arrivalTicks: 500);
        Assert.True(policy.TryDequeue(500, out _, out _, out _));
        policy.Enqueue(2, Sample(100, 2f), arrivalTicks: 510); // late
        Assert.False(policy.TryDequeue(510, out _, out _, out _)); // underrun
        policy.Enqueue(3, Sample(300, 3f), arrivalTicks: 520);

        policy.Reset();

        Assert.Equal(PlayoutPolicyDiagnostics.Empty.ToString(), policy.Diagnostics.ToString());

        // The release history is forgotten too, so a sequence reused after Reset is not mistaken
        // for a duplicate -- a sweep reuses one instance across trials and restarts at sequence 0.
        policy.Enqueue(1, Sample(200, 9f), arrivalTicks: 600);
        Assert.True(policy.TryDequeue(600, out uint sequence, out Pose value, out _));
        Assert.Equal(1u, sequence);
        Assert.Equal(9f, value.Position.X, 4);
    }

    [Fact]
    public void Reset_MakesASecondRunOfIdenticalInputProduceIdenticalOutput()
    {
        ImmediatePlayout policy = MakePolicy(out _);

        string first = Drive(policy);
        policy.Reset();
        string second = Drive(policy);

        Assert.Equal(first, second);

        static string Drive(ImmediatePlayout policy)
        {
            var log = new System.Text.StringBuilder();
            for (int i = 0; i < 10; i++)
            {
                // Every third sample arrives out of order, so the run exercises the late path too.
                long capture = (i % 3 == 2 ? i - 2 : i) * OneMillisecond;
                policy.Enqueue((uint)i, Sample(capture, i), arrivalTicks: i * OneMillisecond + 5);

                while (policy.TryDequeue(i * OneMillisecond + 5, out uint sequence, out Pose value, out long playout))
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
        ImmediatePlayout policy = MakePolicy(out _);
        uint sequence = 0;

        AllocationAssert.Zero(() =>
        {
            sequence++;
            policy.Enqueue(sequence, Sample(sequence * OneMillisecond, sequence), arrivalTicks: sequence * OneMillisecond + 1);
            policy.TryDequeue(sequence * OneMillisecond + 1, out _, out _, out _);
        });
    }

    [Fact]
    public void Diagnostics_ReadRepeatedly_Allocates_Zero_Bytes()
    {
        ImmediatePlayout policy = MakePolicy(out _);
        policy.Enqueue(1, Sample(100, 1f), arrivalTicks: 500);

        AllocationAssert.Zero(() => _ = policy.Diagnostics.BufferedCount);
    }
}
