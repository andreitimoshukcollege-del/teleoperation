using Teleop.Core.Metrics;
using Teleop.Core.Prediction;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Metrics;

public class LiveJointScorerTests
{
    private const long Tps = 1000;   // 1 tick = 1 ms
    private const byte Arm = 1, Gripper = 10;
    private const float FractionPerRad = 0.4513f;
    private const float Deg = 57.29578f;

    private static JointPredictorConfig Config() =>
        new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: 5f,
            instantMotorIds: new[] { Gripper }, maxHorizonTicks: 1000, ticksPerSecond: Tps);

    /// <summary>Member 0 hold-last, member 1 servo-replay.</summary>
    private static LiveJointScorer Scorer(long windowMs = 30_000, int readingCapacity = 4096) =>
        new LiveJointScorer(JointPredictorBank.Create(new[]
        {
            new JointPredictorPreset("hold", "hold-last", Config()),
            new JointPredictorPreset("servo", "servo-replay", Config()),
        }, 0), windowMs, maxJoints: 6, gripperMotorIds: new[] { Gripper }, gripperFractionPerRadian: FractionPerRad,
            readingCapacity: readingCapacity);

    private static JointSample[] State(float arm, float gripper, bool gripperSensed = true) =>
        new[] { new JointSample(Arm, arm, true), new JointSample(Gripper, gripper, gripperSensed) };

    [Fact]
    public void TheError_IsThePredictionForTheReadingsTime_MadeBeforeTheReading()
    {
        var s = Scorer();
        s.ObserveState(0, 10, State(0f, 0f));               // the first reading: nothing to score yet
        s.ObserveState(100, 110, State(0.1f, 0f));          // hold-last predicted 0 for t=100: error 0.1 rad

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.True(stats.HasData);
        Assert.Equal(1, stats.Readings);
        Assert.Equal(0.1f * Deg, stats.ErrorP50Deg, 3);
        Assert.Equal(0.1f * Deg, stats.ErrorP95Deg, 3);
    }

    [Fact]
    public void TheCorrection_IsTheJumpInTheNowPrediction()
    {
        var s = Scorer();
        s.ObserveState(0, 10, State(0f, 0f));
        s.ObserveState(100, 110, State(0.2f, 0f));          // hold-last jumps from 0 to 0.2 at arrival

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.Equal(0.2f * Deg, stats.CorrectionP95Deg, 3);
    }

    [Fact]
    public void TheGripper_IsScoredApart_InServoDegrees()
    {
        var s = Scorer();
        s.ObserveState(0, 10, State(0f, 0.5f));
        s.ObserveState(100, 110, State(0f, 0.6f));          // a 0.1 gripper error, the arm exact

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.Equal(0f, stats.ErrorP95Deg, 4);             // not pooled with the arm
        Assert.Equal(0.1f / FractionPerRad * Deg, stats.GripperErrorP95ServoDeg, 2);
    }

    [Fact]
    public void AnUnsensedGripper_IsNotScored()
    {
        var s = Scorer();
        s.ObserveState(0, 10, State(0f, 0.5f, gripperSensed: false));
        s.ObserveState(100, 110, State(0f, 0.9f, gripperSensed: false));

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.True(float.IsNaN(stats.GripperErrorP95ServoDeg));
    }

    [Fact]
    public void ARepeatedOrOlderStamp_IsNotScored()
    {
        var s = Scorer();
        s.ObserveState(100, 110, State(0f, 0f));
        s.ObserveState(100, 120, State(0.3f, 0f));          // same sensed stamp
        s.ObserveState(50, 130, State(0.3f, 0f));           // older

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.False(stats.HasData);
        Assert.Equal(0, stats.Readings);
    }

    [Fact]
    public void TheWindow_DropsOldSamples()
    {
        var s = Scorer(windowMs: 1000);
        s.ObserveState(0, 0, State(0f, 0f));
        s.ObserveState(100, 100, State(1f, 0f));            // a 1 rad error at 100 ms
        for (int k = 2; k <= 30; k++)
        {
            s.ObserveState(k * 100, k * 100, State(1f, 0f)); // then exact
        }

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.Equal(0f, stats.ErrorP95Deg, 4);             // the big error is more than 1 s old
        Assert.Equal(11, stats.Readings);                    // readings at 2.0..3.0 s: the window includes its edge
    }

    [Fact]
    public void Percentiles_AreNearestRank()
    {
        var s = Scorer();
        s.ObserveState(0, 0, State(0f, 0f));
        // Errors of 1..20 degrees: hold-last's error is the step from the previous reading.
        float value = 0f;
        for (int k = 1; k <= 20; k++)
        {
            value += k / Deg;
            s.ObserveState(k * 10, k * 10, State(value, 0f));
        }

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.Equal(10f, stats.ErrorP50Deg, 2);            // ceil(0.5 * 20) = 10th
        Assert.Equal(19f, stats.ErrorP95Deg, 2);            // ceil(0.95 * 20) = 19th
    }

    [Fact]
    public void Jitter_IsTheSecondDifferenceOfThePrediction()
    {
        var s = Scorer();
        s.ObserveState(0, 0, State(0f, 0f));
        s.OnDisplayTick(10, 10);
        s.OnDisplayTick(20, 20);
        s.ObserveState(25, 25, State(0.01f, 0f));           // a step in hold-last's prediction
        s.OnDisplayTick(30, 30);                             // 0.01 - 2*0 + 0 = 0.01 rad

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.Equal(0.01f * Deg, stats.JitterP95Deg, 3);
    }

    [Fact]
    public void ADisabledMember_HasNoStats_AndReEnablingClearsItsNumbers()
    {
        var s = Scorer();
        s.ObserveState(0, 0, State(0f, 0f));
        s.ObserveState(100, 100, State(0.5f, 0f));          // servo (member 1) gets an error
        Assert.True(s.TryGetStats(1, out LiveJointStats before));
        Assert.True(before.HasData);

        Assert.True(s.Bank.TrySetEnabled(1, false));
        Assert.False(s.TryGetStats(1, out _));
        Assert.True(s.Bank.TrySetEnabled(1, true));          // reset by the bank: a new generation
        Assert.True(s.TryGetStats(1, out LiveJointStats after));
        Assert.False(after.HasData);
    }

    [Fact]
    public void Reset_ReturnsToAsConstructed()
    {
        var s = Scorer();
        s.ObserveState(0, 0, State(0f, 0f));
        s.ObserveState(100, 100, State(0.5f, 0f));
        s.OnDisplayTick(110, 110);
        s.Reset();

        Assert.True(s.TryGetStats(0, out LiveJointStats stats));
        Assert.False(stats.HasData);
        Assert.True(float.IsNaN(stats.ErrorP95Deg));

        // A stamp earlier than before the reset counts as fresh again.
        s.Bank.Reset();
        s.ObserveState(0, 0, State(0f, 0f));
        s.ObserveState(50, 50, State(0.2f, 0f));
        Assert.True(s.TryGetStats(0, out stats));
        Assert.Equal(1, stats.Readings);
    }

    [Fact]
    public void ScoringTickingAndReading_AllocateNothing()
    {
        var s = Scorer(readingCapacity: 512);
        var states = new[] { State(0f, 0.2f), State(0.1f, 0.3f) };
        long t = 0;
        AllocationAssert.Zero(() =>
        {
            t += 11;
            s.Bank.ObserveCommand(t, states[(int)(t / 11 % 2)], 0f);
            s.ObserveState(t, t + 3, states[(int)(t / 22 % 2)]);
            s.OnDisplayTick(t + 3, t + 3);
            s.TryGetStats((int)(t % 2), out _);
        });
    }
}
