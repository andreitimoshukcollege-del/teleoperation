using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Time;

public class ClockSyncTests
{
    // Every same-domain case below passes this for both the operator and the robot rate, which
    // makes ClockSync's rescale ratio exactly 1.0 and the arithmetic identical to the pre-ADR-0008
    // form. The deliberately mismatched-rate cases at the bottom of this file are the only ones
    // that exercise the rescale itself.
    private const long SharedRate = 10_000_000;

    // The real pairing docs/adr/0008-clocksync-cross-rate-normalization.md was written for: a
    // Windows dev machine's Stopwatch (10 MHz) talking to a Jetson running .NET on Linux ARM64
    // (1 GHz). A 100x mismatch, and the exact one that inflated Phase 3's first real-hardware
    // RTT figures 100x.
    private const long WindowsRate = 10_000_000;
    private const long JetsonRate = 1_000_000_000;

    private static ClockSyncConfig DefaultConfig(int historyCapacity = 16) =>
        new ClockSyncConfig(
            historyCapacity: historyCapacity,
            smoothingAlpha: 0.2f,
            maxAcceptableRttTicks: 1_000_000,
            outlierRttMultiple: 3.0,
            minSamplesBeforeTrusted: 3);

    [Fact]
    public void AddRoundTrip_ZeroOffsetSymmetricDelay_EstimatesZeroOffset()
    {
        var sync = new ClockSync(DefaultConfig());

        // Operator and robot clocks agree; 10-tick one-way transit each direction.
        bool accepted = sync.AddRoundTrip(
            operatorSendTicks: 0, operatorTicksPerSecond: SharedRate,
            robotRecvTicks: 10, robotSendTicks: 10, robotTicksPerSecond: SharedRate,
            operatorRecvTicks: 20);

        Assert.True(accepted);
        Assert.Equal(0, sync.Diagnostics.OffsetTicks);
        Assert.Equal(20, sync.Diagnostics.LastRttTicks);
        Assert.Equal(10, sync.Diagnostics.OffsetUncertaintyTicks);
    }

    [Fact]
    public void AddRoundTrip_KnownOffset_RecoversIt()
    {
        var sync = new ClockSync(DefaultConfig());

        // Robot clock is 1000 ticks ahead of operator clock; 10-tick transit each way.
        // operatorSend=0 (operator domain) -> robot receives at robot-domain 1010 (0+1000 offset +10 transit)
        // robot replies at robot-domain 1020 -> operator receives at operator-domain 30 (1020-1000+10)
        bool accepted = sync.AddRoundTrip(
            operatorSendTicks: 0, operatorTicksPerSecond: SharedRate,
            robotRecvTicks: 1010, robotSendTicks: 1020, robotTicksPerSecond: SharedRate,
            operatorRecvTicks: 30);

        Assert.True(accepted);
        // offset = ((t0-t1)+(t3-t2))/2 = ((0-1010)+(30-1020))/2 = (-1010 + -990)/2 = -1000
        Assert.Equal(-1000, sync.Diagnostics.OffsetTicks);
    }

    [Fact]
    public void ToOperatorTicks_AppliesCurrentOffset()
    {
        var sync = new ClockSync(DefaultConfig());
        sync.AddRoundTrip(
            operatorSendTicks: 0, operatorTicksPerSecond: SharedRate,
            robotRecvTicks: 1010, robotSendTicks: 1020, robotTicksPerSecond: SharedRate,
            operatorRecvTicks: 30);

        long converted = sync.ToOperatorTicks(
            robotTicks: 5000, robotTicksPerSecond: SharedRate, operatorTicksPerSecond: SharedRate);

        Assert.Equal(5000 + sync.Diagnostics.OffsetTicks, converted);
    }

    [Fact]
    public void ToOperatorTicks_BeforeAnySample_PassesThroughUnchanged()
    {
        var sync = new ClockSync(DefaultConfig());

        Assert.Equal(5000, sync.ToOperatorTicks(5000, SharedRate, SharedRate));
        Assert.False(sync.Diagnostics.IsSynced);
    }

    [Fact]
    public void AddRoundTrip_NegativeRtt_IsRejected()
    {
        var sync = new ClockSync(DefaultConfig());

        // t3 < t0 relative to t2 - t1 makes rtt negative: rtt = (t3-t0)-(t2-t1) = (5-0)-(100-0) = -95
        bool accepted = sync.AddRoundTrip(
            operatorSendTicks: 0, operatorTicksPerSecond: SharedRate,
            robotRecvTicks: 0, robotSendTicks: 100, robotTicksPerSecond: SharedRate,
            operatorRecvTicks: 5);

        Assert.False(accepted);
        Assert.Equal(1, sync.Diagnostics.RejectedSampleCount);
        Assert.Equal(0, sync.Diagnostics.AcceptedSampleCount);
    }

    [Fact]
    public void AddRoundTrip_OverHardCeiling_IsRejected()
    {
        var config = new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 100,
            outlierRttMultiple: 10.0, minSamplesBeforeTrusted: 1);
        var sync = new ClockSync(config);

        bool accepted = sync.AddRoundTrip(
            operatorSendTicks: 0, operatorTicksPerSecond: SharedRate,
            robotRecvTicks: 10, robotSendTicks: 10, robotTicksPerSecond: SharedRate,
            operatorRecvTicks: 1000);

        Assert.False(accepted);
        Assert.Equal(1, sync.Diagnostics.RejectedSampleCount);
    }

    [Fact]
    public void AddRoundTrip_FarSlowerThanRecentMinimum_IsRejectedAsOutlier()
    {
        var config = new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 1_000_000,
            outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 1);
        var sync = new ClockSync(config);

        // Establish a good baseline RTT of 20 ticks.
        sync.AddRoundTrip(0, SharedRate, 10, 10, SharedRate, 20);
        Assert.Equal(1, sync.Diagnostics.AcceptedSampleCount);

        // A round trip with RTT of 100 ticks (5x the 20-tick minimum, over the 3x multiple).
        bool accepted = sync.AddRoundTrip(0, SharedRate, 10, 10, SharedRate, 100);

        Assert.False(accepted);
        Assert.Equal(1, sync.Diagnostics.AcceptedSampleCount);
        Assert.Equal(1, sync.Diagnostics.RejectedSampleCount);
    }

    [Fact]
    public void IsSynced_BecomesTrueOnlyAfterMinSamplesBeforeTrusted()
    {
        var config = new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 1_000_000,
            outlierRttMultiple: 10.0, minSamplesBeforeTrusted: 3);
        var sync = new ClockSync(config);

        sync.AddRoundTrip(0, SharedRate, 10, 10, SharedRate, 20);
        Assert.False(sync.Diagnostics.IsSynced);
        sync.AddRoundTrip(0, SharedRate, 10, 10, SharedRate, 20);
        Assert.False(sync.Diagnostics.IsSynced);
        sync.AddRoundTrip(0, SharedRate, 10, 10, SharedRate, 20);
        Assert.True(sync.Diagnostics.IsSynced);
    }

    [Fact]
    public void SameSeedOfSamples_ProducesDeterministicOffset()
    {
        var a = new ClockSync(DefaultConfig());
        var b = new ClockSync(DefaultConfig());

        for (int i = 0; i < 20; i++)
        {
            a.AddRoundTrip(i * 100, SharedRate, i * 100 + 10, i * 100 + 15, SharedRate, i * 100 + 27);
            b.AddRoundTrip(i * 100, SharedRate, i * 100 + 10, i * 100 + 15, SharedRate, i * 100 + 27);
        }

        Assert.Equal(a.Diagnostics.OffsetTicks, b.Diagnostics.OffsetTicks);
        Assert.Equal(a.Diagnostics.AcceptedSampleCount, b.Diagnostics.AcceptedSampleCount);
    }

    /// <summary>
    /// The mismatched-rate counterpart of <see cref="SameSeedOfSamples_ProducesDeterministicOffset"/>:
    /// the rescale introduces floating-point arithmetic into a path that was previously pure
    /// integer math, so determinism has to be re-proven with a ratio that is not 1.0 and does not
    /// divide evenly (7 MHz operator vs 3 MHz robot).
    /// </summary>
    [Fact]
    public void MismatchedRates_SameSamplesTwice_ProduceIdenticalOffset()
    {
        var a = new ClockSync(DefaultConfig());
        var b = new ClockSync(DefaultConfig());

        for (int i = 0; i < 20; i++)
        {
            a.AddRoundTrip(i * 100, 7_000_000, i * 43 + 11, i * 43 + 17, 3_000_000, i * 100 + 27);
            b.AddRoundTrip(i * 100, 7_000_000, i * 43 + 11, i * 43 + 17, 3_000_000, i * 100 + 27);
        }

        Assert.Equal(a.Diagnostics.OffsetTicks, b.Diagnostics.OffsetTicks);
        Assert.Equal(a.Diagnostics.LastRttTicks, b.Diagnostics.LastRttTicks);
        Assert.Equal(a.Diagnostics.AcceptedSampleCount, b.Diagnostics.AcceptedSampleCount);
        Assert.Equal(a.Diagnostics.RejectedSampleCount, b.Diagnostics.RejectedSampleCount);
    }

    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        var sync = new ClockSync(DefaultConfig());
        sync.AddRoundTrip(0, SharedRate, 1010, 1020, SharedRate, 30);

        sync.Reset();

        var diag = sync.Diagnostics;
        Assert.Equal(0, diag.OffsetTicks);
        Assert.Equal(0, diag.OffsetUncertaintyTicks);
        Assert.Equal(0, diag.LastRttTicks);
        Assert.Equal(0, diag.MinRttTicks);
        Assert.Equal(0, diag.AcceptedSampleCount);
        Assert.Equal(0, diag.RejectedSampleCount);
        Assert.False(diag.IsSynced);
    }

    /// <summary>
    /// <c>Reset()</c> after a mismatched-rate round trip: nothing about the two rates persists
    /// across calls (both arrive fresh on every call, by design -- ADR 0008), so a reset estimator
    /// is indistinguishable from a freshly constructed one even though the last samples it saw
    /// carried a 100x ratio.
    /// </summary>
    [Fact]
    public void Reset_AfterMismatchedRateSamples_RestoresAsConstructedState()
    {
        var sync = new ClockSync(DefaultConfig());
        var fresh = new ClockSync(DefaultConfig());
        Assert.True(sync.AddRoundTrip(
            50_000_000, WindowsRate, 3_030_000_000, 3_040_000_000, JetsonRate, 50_700_000));

        sync.Reset();

        Assert.Equal(fresh.Diagnostics.OffsetTicks, sync.Diagnostics.OffsetTicks);
        Assert.Equal(fresh.Diagnostics.OffsetUncertaintyTicks, sync.Diagnostics.OffsetUncertaintyTicks);
        Assert.Equal(fresh.Diagnostics.LastRttTicks, sync.Diagnostics.LastRttTicks);
        Assert.Equal(fresh.Diagnostics.MinRttTicks, sync.Diagnostics.MinRttTicks);
        Assert.Equal(fresh.Diagnostics.AcceptedSampleCount, sync.Diagnostics.AcceptedSampleCount);
        Assert.Equal(fresh.Diagnostics.RejectedSampleCount, sync.Diagnostics.RejectedSampleCount);
        Assert.False(sync.Diagnostics.IsSynced);

        // And it still works afterwards, with the same rates, as if new.
        sync.AddRoundTrip(50_000_000, WindowsRate, 3_030_000_000, 3_040_000_000, JetsonRate, 50_700_000);
        fresh.AddRoundTrip(50_000_000, WindowsRate, 3_030_000_000, 3_040_000_000, JetsonRate, 50_700_000);
        Assert.Equal(fresh.Diagnostics.OffsetTicks, sync.Diagnostics.OffsetTicks);
        Assert.Equal(20_000_000, sync.Diagnostics.OffsetTicks);
    }

    [Fact]
    public void AddRoundTrip_Allocates_Zero_Bytes()
    {
        var sync = new ClockSync(DefaultConfig());
        AllocationAssert.Zero(() => sync.AddRoundTrip(0, SharedRate, 10, 10, SharedRate, 20));
    }

    [Fact]
    public void AddRoundTrip_MismatchedRates_Allocates_Zero_Bytes()
    {
        var sync = new ClockSync(DefaultConfig());
        AllocationAssert.Zero(() => sync.AddRoundTrip(0, WindowsRate, 1_000, 1_000, JetsonRate, 20));
    }

    [Fact]
    public void ToOperatorTicks_Allocates_Zero_Bytes()
    {
        var sync = new ClockSync(DefaultConfig());
        sync.AddRoundTrip(0, SharedRate, 10, 10, SharedRate, 20);
        AllocationAssert.Zero(() => sync.ToOperatorTicks(1000, SharedRate, SharedRate));
    }

    [Fact]
    public void ToOperatorTicks_MismatchedRates_Allocates_Zero_Bytes()
    {
        var sync = new ClockSync(DefaultConfig());
        AllocationAssert.Zero(() => sync.ToOperatorTicks(1_000_000, JetsonRate, WindowsRate));
    }

    /// <summary>
    /// The bug ADR 0008 fixes, stated as an arithmetic assertion. Hand-computed scenario, in
    /// physical time rather than ticks so the expected answer is independent of either rate:
    ///
    /// <list type="bullet">
    /// <item>The robot's clock reads 2.0s <b>behind</b> the operator's (offset = +2.0s, operator
    /// minus robot).</item>
    /// <item>The operator sends at operator-time 5.0s == 50,000,000 operator ticks (t0).</item>
    /// <item>Uplink transit 30ms: the robot receives at operator-time 5.03s, which its own clock
    /// reads as 3.03s == 3,030,000,000 robot ticks (t1).</item>
    /// <item>The robot spends 10ms replying: it sends at robot-time 3.04s ==
    /// 3,040,000,000 robot ticks (t2).</item>
    /// <item>Downlink transit 30ms: the operator receives at operator-time 5.07s ==
    /// 50,700,000 operator ticks (t3).</item>
    /// </list>
    ///
    /// Rescaled into operator ticks (ratio = 10,000,000 / 1,000,000,000 = 0.01):
    /// t1' = 30,300,000, t2' = 30,400,000. Then
    /// rtt = (50,700,000 - 50,000,000) - (30,400,000 - 30,300,000) = 700,000 - 100,000 = 600,000
    /// ticks = 60ms, the true 30+30ms of transit; and
    /// offset = ((50,000,000 - 30,300,000) + (50,700,000 - 30,400,000)) / 2
    ///        = (19,700,000 + 20,300,000) / 2 = 20,000,000 ticks = the true +2.0s.
    ///
    /// Without the rescale, the raw robot ticks (3.03e9, 3.04e9) would swamp the operator ticks
    /// and produce rtt = 700,000 - 10,000,000 = -9,300,000 -- a negative RTT, rejected outright --
    /// which is precisely the class of nonsense the real JetRover run produced.
    /// </summary>
    [Fact]
    public void AddRoundTrip_OperatorTenMHz_RobotOneGHz_RecoversTrueOffsetAndRttInOperatorTicks()
    {
        var config = new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 10_000_000,
            outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 1);
        var sync = new ClockSync(config);

        bool accepted = sync.AddRoundTrip(
            operatorSendTicks: 50_000_000, operatorTicksPerSecond: WindowsRate,
            robotRecvTicks: 3_030_000_000, robotSendTicks: 3_040_000_000, robotTicksPerSecond: JetsonRate,
            operatorRecvTicks: 50_700_000);

        Assert.True(accepted);
        Assert.Equal(600_000, sync.Diagnostics.LastRttTicks);   // 60ms at 10 MHz
        Assert.Equal(20_000_000, sync.Diagnostics.OffsetTicks); // +2.0s at 10 MHz
        Assert.Equal(300_000, sync.Diagnostics.OffsetUncertaintyTicks); // rtt/2 == 30ms
    }

    /// <summary>
    /// The pre-ADR-0008 behavior, reproduced by lying about the robot's rate (claiming it also
    /// ticks at 10 MHz), on the identical timestamps the test above feeds in honestly. The raw
    /// 1 GHz stamps swamp the operator's, giving <c>rtt = 700,000 - 10,000,000 = -9,300,000</c> --
    /// a negative, physically impossible RTT, rejected outright, no offset produced at all. This
    /// is the regression guard on the fix mattering: if <c>AddRoundTrip</c> ever silently ignored
    /// its rate arguments again, this test and the one above could not both pass.
    /// </summary>
    [Fact]
    public void AddRoundTrip_MismatchedRatesDeclaredEqual_ProducesNonsenseAndIsRejected()
    {
        var config = new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 10_000_000,
            outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 1);
        var honest = new ClockSync(config);
        var lying = new ClockSync(config);

        bool honestAccepted = honest.AddRoundTrip(
            50_000_000, WindowsRate, 3_030_000_000, 3_040_000_000, JetsonRate, 50_700_000);
        bool lyingAccepted = lying.AddRoundTrip(
            50_000_000, WindowsRate, 3_030_000_000, 3_040_000_000, WindowsRate, 50_700_000);

        Assert.True(honestAccepted);
        Assert.Equal(600_000, honest.Diagnostics.LastRttTicks);
        Assert.Equal(20_000_000, honest.Diagnostics.OffsetTicks);
        Assert.True(honest.Diagnostics.IsSynced);

        Assert.False(lyingAccepted);
        Assert.Equal(1, lying.Diagnostics.RejectedSampleCount);
        Assert.Equal(0, lying.Diagnostics.AcceptedSampleCount);
        Assert.Equal(0, lying.Diagnostics.OffsetTicks);
        Assert.False(lying.Diagnostics.IsSynced);
    }

    /// <summary>
    /// <see cref="ClockSync.ToOperatorTicks"/> must apply the identical rescale
    /// <see cref="ClockSync.AddRoundTrip"/> used, or a converted stamp and the offset estimated
    /// alongside it disagree. Proven by converting the very stamps the round trip was built from
    /// and checking they land on the operator-domain instants the scenario says they occurred at.
    /// </summary>
    [Fact]
    public void ToOperatorTicks_MismatchedRates_RescalesThenAppliesOffset()
    {
        var config = new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 10_000_000,
            outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 1);
        var sync = new ClockSync(config);
        sync.AddRoundTrip(50_000_000, WindowsRate, 3_030_000_000, 3_040_000_000, JetsonRate, 50_700_000);

        long robotRecvInOperatorDomain = sync.ToOperatorTicks(3_030_000_000, JetsonRate, WindowsRate);
        long robotSendInOperatorDomain = sync.ToOperatorTicks(3_040_000_000, JetsonRate, WindowsRate);

        // 30,300,000 rescaled + 20,000,000 offset = operator-time 5.03s, the true arrival instant.
        Assert.Equal(50_300_000, robotRecvInOperatorDomain);
        Assert.Equal(50_400_000, robotSendInOperatorDomain);

        // And the converted stamps sit inside the operator's own send/receive bracket, which is
        // the whole point of the conversion.
        Assert.True(50_000_000 <= robotRecvInOperatorDomain);
        Assert.True(robotRecvInOperatorDomain <= robotSendInOperatorDomain);
        Assert.True(robotSendInOperatorDomain <= 50_700_000);
    }

    /// <summary>
    /// The reverse ratio (robot slower than operator) must not truncate to nothing, which is what
    /// an integer ratio would do: 1,000,000 / 10,000,000 == 0 in integer arithmetic, mapping every
    /// robot stamp to 0. A 1 MHz robot stamp of 3,030,000 is 3.03s, the same instant as the case
    /// above, and must convert to the same operator-domain answer.
    /// </summary>
    [Fact]
    public void ToOperatorTicks_RobotSlowerThanOperator_DoesNotTruncateToZero()
    {
        const long slowRobotRate = 1_000_000;
        var config = new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 10_000_000,
            outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 1);
        var sync = new ClockSync(config);

        bool accepted = sync.AddRoundTrip(
            operatorSendTicks: 50_000_000, operatorTicksPerSecond: WindowsRate,
            robotRecvTicks: 3_030_000, robotSendTicks: 3_040_000, robotTicksPerSecond: slowRobotRate,
            operatorRecvTicks: 50_700_000);

        Assert.True(accepted);
        Assert.Equal(600_000, sync.Diagnostics.LastRttTicks);
        Assert.Equal(20_000_000, sync.Diagnostics.OffsetTicks);
        Assert.Equal(50_300_000, sync.ToOperatorTicks(3_030_000, slowRobotRate, WindowsRate));
    }

    /// <summary>
    /// Equal rates must leave the arithmetic bit-for-bit where it was before ADR 0008: the ratio
    /// is exactly 1.0, so the rescale is the identity and every existing same-domain caller
    /// (loopback, sweep, Unity's bridges) is unaffected. Asserted against the integer arithmetic
    /// spelled out longhand, at rates spanning the plausible range.
    /// </summary>
    [Theory]
    [InlineData(10_000_000L)]
    [InlineData(1_000_000_000L)]
    [InlineData(1_000_000L)]
    [InlineData(1L)]
    public void AddRoundTrip_EqualRates_MatchesUnscaledIntegerArithmetic(long rate)
    {
        const long t0 = 1_000;
        const long t1 = 1_510;
        const long t2 = 1_515;
        const long t3 = 1_025;
        var config = new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.5f, maxAcceptableRttTicks: 1_000_000,
            outlierRttMultiple: 10.0, minSamplesBeforeTrusted: 1);
        var sync = new ClockSync(config);

        bool accepted = sync.AddRoundTrip(t0, rate, t1, t2, rate, t3);

        Assert.True(accepted);
        Assert.Equal((t3 - t0) - (t2 - t1), sync.Diagnostics.LastRttTicks);
        Assert.Equal(((t0 - t1) + (t3 - t2)) / 2, sync.Diagnostics.OffsetTicks);
        Assert.Equal(9_999 + sync.Diagnostics.OffsetTicks, sync.ToOperatorTicks(9_999, rate, rate));
    }
}
