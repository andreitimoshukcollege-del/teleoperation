using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Time;

public class ClockSyncTests
{
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
        bool accepted = sync.AddRoundTrip(operatorSendTicks: 0, robotRecvTicks: 10, robotSendTicks: 10, operatorRecvTicks: 20);

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
        bool accepted = sync.AddRoundTrip(operatorSendTicks: 0, robotRecvTicks: 1010, robotSendTicks: 1020, operatorRecvTicks: 30);

        Assert.True(accepted);
        // offset = ((t0-t1)+(t3-t2))/2 = ((0-1010)+(30-1020))/2 = (-1010 + -990)/2 = -1000
        Assert.Equal(-1000, sync.Diagnostics.OffsetTicks);
    }

    [Fact]
    public void ToOperatorTicks_AppliesCurrentOffset()
    {
        var sync = new ClockSync(DefaultConfig());
        sync.AddRoundTrip(operatorSendTicks: 0, robotRecvTicks: 1010, robotSendTicks: 1020, operatorRecvTicks: 30);

        long converted = sync.ToOperatorTicks(robotTicks: 5000);

        Assert.Equal(5000 + sync.Diagnostics.OffsetTicks, converted);
    }

    [Fact]
    public void ToOperatorTicks_BeforeAnySample_PassesThroughUnchanged()
    {
        var sync = new ClockSync(DefaultConfig());

        Assert.Equal(5000, sync.ToOperatorTicks(5000));
        Assert.False(sync.Diagnostics.IsSynced);
    }

    [Fact]
    public void AddRoundTrip_NegativeRtt_IsRejected()
    {
        var sync = new ClockSync(DefaultConfig());

        // t3 < t0 relative to t2 - t1 makes rtt negative: rtt = (t3-t0)-(t2-t1) = (5-0)-(100-0) = -95
        bool accepted = sync.AddRoundTrip(operatorSendTicks: 0, robotRecvTicks: 0, robotSendTicks: 100, operatorRecvTicks: 5);

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

        bool accepted = sync.AddRoundTrip(operatorSendTicks: 0, robotRecvTicks: 10, robotSendTicks: 10, operatorRecvTicks: 1000);

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
        sync.AddRoundTrip(0, 10, 10, 20);
        Assert.Equal(1, sync.Diagnostics.AcceptedSampleCount);

        // A round trip with RTT of 100 ticks (5x the 20-tick minimum, over the 3x multiple).
        bool accepted = sync.AddRoundTrip(0, 10, 10, 100);

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

        sync.AddRoundTrip(0, 10, 10, 20);
        Assert.False(sync.Diagnostics.IsSynced);
        sync.AddRoundTrip(0, 10, 10, 20);
        Assert.False(sync.Diagnostics.IsSynced);
        sync.AddRoundTrip(0, 10, 10, 20);
        Assert.True(sync.Diagnostics.IsSynced);
    }

    [Fact]
    public void SameSeedOfSamples_ProducesDeterministicOffset()
    {
        var a = new ClockSync(DefaultConfig());
        var b = new ClockSync(DefaultConfig());

        for (int i = 0; i < 20; i++)
        {
            a.AddRoundTrip(i * 100, i * 100 + 10, i * 100 + 15, i * 100 + 27);
            b.AddRoundTrip(i * 100, i * 100 + 10, i * 100 + 15, i * 100 + 27);
        }

        Assert.Equal(a.Diagnostics.OffsetTicks, b.Diagnostics.OffsetTicks);
        Assert.Equal(a.Diagnostics.AcceptedSampleCount, b.Diagnostics.AcceptedSampleCount);
    }

    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        var sync = new ClockSync(DefaultConfig());
        sync.AddRoundTrip(0, 1010, 1020, 30);

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

    [Fact]
    public void AddRoundTrip_Allocates_Zero_Bytes()
    {
        var sync = new ClockSync(DefaultConfig());
        AllocationAssert.Zero(() => sync.AddRoundTrip(0, 10, 10, 20));
    }

    [Fact]
    public void ToOperatorTicks_Allocates_Zero_Bytes()
    {
        var sync = new ClockSync(DefaultConfig());
        sync.AddRoundTrip(0, 10, 10, 20);
        AllocationAssert.Zero(() => sync.ToOperatorTicks(1000));
    }
}
