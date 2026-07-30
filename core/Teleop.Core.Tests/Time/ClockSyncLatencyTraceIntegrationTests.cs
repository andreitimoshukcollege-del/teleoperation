using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Time;

/// <summary>
/// The test docs/adr/0002-latency-trace.md obligates: proof that LatencyTrace's robot-domain
/// fields (WithRobotRecvTicks, WithDownlinkSendTicks, WithClockSync) convert correctly through a
/// real ClockSync, before anything writes to those fields for real. LatencyTrace itself is not
/// modified here, only exercised.
/// </summary>
public class ClockSyncLatencyTraceIntegrationTests
{
    [Fact]
    public void RoundTrip_ConvertsRobotDomainStampsIntoOperatorDomain_ConsistentWithLatencyTrace()
    {
        // Scenario: the robot's clock reads 500 ticks ahead of the operator's clock. The uplink
        // command takes 10 ticks to arrive, the robot takes 5 ticks to prepare its reply, and the
        // reply takes 10 ticks to arrive back. All of this is unknown to ClockSync -- it only
        // ever sees the four timestamps below, which is exactly the shape a real uplink
        // command + downlink reply exchange produces.
        const long trueOffset = -500; // operator domain minus robot domain
        const long operatorSendTicks = 1000;      // t0, operator domain
        const long robotRecvTicksRaw = 1510;      // t1, robot domain  (1000 + 500 offset + 10 transit)
        const long robotSendTicksRaw = 1515;      // t2, robot domain  (1510 + 5 processing)
        const long operatorRecvTicks = 1025;      // t3, operator domain (1515 - 500 offset + 10 transit)

        var sync = new ClockSync(new ClockSyncConfig(
            historyCapacity: 16,
            smoothingAlpha: 0.5f,
            maxAcceptableRttTicks: 1_000_000,
            outlierRttMultiple: 10.0,
            minSamplesBeforeTrusted: 1));

        bool accepted = sync.AddRoundTrip(operatorSendTicks, robotRecvTicksRaw, robotSendTicksRaw, operatorRecvTicks);
        Assert.True(accepted);
        Assert.Equal(trueOffset, sync.Diagnostics.OffsetTicks);
        Assert.True(sync.Diagnostics.IsSynced);

        long robotRecvOperatorDomain = sync.ToOperatorTicks(robotRecvTicksRaw);
        long downlinkSendOperatorDomain = sync.ToOperatorTicks(robotSendTicksRaw);

        var trace = LatencyTrace.ForSequence(7)
            .WithCaptureTicks(operatorSendTicks)
            .WithUplinkSendTicks(operatorSendTicks)
            .WithRobotRecvTicks(robotRecvOperatorDomain)
            .WithDownlinkSendTicks(downlinkSendOperatorDomain)
            .WithOperatorRecvTicks(operatorRecvTicks)
            .WithClockSync(sync.Diagnostics.OffsetTicks, sync.Diagnostics.OffsetUncertaintyTicks);

        // The converted stamps land where the known transit/processing times predict.
        Assert.True(trace.TryGetRobotRecvTicks(out long robotRecv));
        Assert.Equal(1010, robotRecv); // 10 ticks of uplink transit past operatorSendTicks

        Assert.True(trace.TryGetDownlinkSendTicks(out long downlinkSend));
        Assert.Equal(1015, downlinkSend); // 5 ticks of robot processing past robotRecv

        Assert.True(trace.TryGetClockOffsetTicks(out long offset));
        Assert.Equal(trueOffset, offset);

        Assert.True(trace.TryGetClockOffsetUncertaintyTicks(out long uncertainty));
        Assert.Equal(10, uncertainty); // rtt=20 (25-5), uncertainty = rtt/2

        // Once converted, every stamp on this trace is in a single, causally ordered timeline --
        // this is the entire point of the conversion: before it, robot-domain 1510/1515 would
        // look nonsensically "in the future" next to operator-domain 1000..1025.
        Assert.True(trace.TryGetUplinkSendTicks(out long uplinkSend));
        Assert.True(trace.TryGetOperatorRecvTicks(out long recv));
        Assert.True(uplinkSend <= robotRecv);
        Assert.True(robotRecv <= downlinkSend);
        Assert.True(downlinkSend <= recv);
    }

    [Fact]
    public void UnsyncedClockSync_OffsetIsZero_ConversionIsIdentity()
    {
        var sync = new ClockSync(new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.5f, maxAcceptableRttTicks: 1_000_000,
            outlierRttMultiple: 10.0, minSamplesBeforeTrusted: 5));

        Assert.False(sync.Diagnostics.IsSynced);

        var trace = LatencyTrace.ForSequence(1)
            .WithRobotRecvTicks(sync.ToOperatorTicks(9999))
            .WithClockSync(sync.Diagnostics.OffsetTicks, sync.Diagnostics.OffsetUncertaintyTicks);

        Assert.True(trace.TryGetRobotRecvTicks(out long robotRecv));
        Assert.Equal(9999, robotRecv);
        Assert.True(trace.TryGetClockOffsetTicks(out long offset));
        Assert.Equal(0, offset);
    }
}
