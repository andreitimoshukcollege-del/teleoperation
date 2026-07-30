using Teleop.Core.Types;

namespace Teleop.Core.Tests.Types;

public class LatencyTraceTests
{
    [Fact]
    public void ForSequence_CarriesSequence_AllStampsUnset()
    {
        var trace = LatencyTrace.ForSequence(42);

        Assert.Equal(42u, trace.Sequence);
        Assert.False(trace.TryGetCaptureTicks(out _));
        Assert.False(trace.TryGetUplinkSendTicks(out _));
        Assert.False(trace.TryGetRobotRecvTicks(out _));
        Assert.False(trace.TryGetDownlinkSendTicks(out _));
        Assert.False(trace.TryGetOperatorRecvTicks(out _));
        Assert.False(trace.TryGetPlayoutTicks(out _));
        Assert.False(trace.TryGetRenderTicks(out _));
        Assert.False(trace.TryGetPhotonTicks(out _));
        Assert.False(trace.TryGetClockOffsetTicks(out _));
        Assert.False(trace.TryGetClockOffsetUncertaintyTicks(out _));
    }

    [Fact]
    public void WithCaptureTicks_IsImmutable_OriginalUnaffected()
    {
        var original = LatencyTrace.ForSequence(1);

        var withCapture = original.WithCaptureTicks(100);

        Assert.False(original.TryGetCaptureTicks(out _));
        Assert.True(withCapture.TryGetCaptureTicks(out var ticks));
        Assert.Equal(100, ticks);
    }

    [Fact]
    public void WithChain_AccumulatesEachFieldWithoutDisturbingTheOthers()
    {
        var trace = LatencyTrace.ForSequence(7)
            .WithCaptureTicks(100)
            .WithUplinkSendTicks(110)
            .WithRobotRecvTicks(250)
            .WithDownlinkSendTicks(260)
            .WithOperatorRecvTicks(400)
            .WithPlayoutTicks(410)
            .WithRenderTicks(420)
            .WithPhotonTicks(428)
            .WithClockSync(offsetTicks: 150, offsetUncertaintyTicks: 5);

        Assert.Equal(7u, trace.Sequence);

        Assert.True(trace.TryGetCaptureTicks(out var capture));
        Assert.Equal(100, capture);

        Assert.True(trace.TryGetUplinkSendTicks(out var uplinkSend));
        Assert.Equal(110, uplinkSend);

        Assert.True(trace.TryGetRobotRecvTicks(out var robotRecv));
        Assert.Equal(250, robotRecv);

        Assert.True(trace.TryGetDownlinkSendTicks(out var downlinkSend));
        Assert.Equal(260, downlinkSend);

        Assert.True(trace.TryGetOperatorRecvTicks(out var operatorRecv));
        Assert.Equal(400, operatorRecv);

        Assert.True(trace.TryGetPlayoutTicks(out var playout));
        Assert.Equal(410, playout);

        Assert.True(trace.TryGetRenderTicks(out var render));
        Assert.Equal(420, render);

        Assert.True(trace.TryGetPhotonTicks(out var photon));
        Assert.Equal(428, photon);

        Assert.True(trace.TryGetClockOffsetTicks(out var offset));
        Assert.Equal(150, offset);

        Assert.True(trace.TryGetClockOffsetUncertaintyTicks(out var offsetSigma));
        Assert.Equal(5, offsetSigma);
    }

    [Fact]
    public void WithChain_FieldsSetLaterInTheChainDoNotOverwriteEarlierOnes()
    {
        var trace = LatencyTrace.ForSequence(3)
            .WithCaptureTicks(10)
            .WithUplinkSendTicks(20)
            .WithRobotRecvTicks(30);

        Assert.True(trace.TryGetCaptureTicks(out var capture));
        Assert.Equal(10, capture);
        Assert.True(trace.TryGetUplinkSendTicks(out var uplinkSend));
        Assert.Equal(20, uplinkSend);
    }

    [Fact]
    public void WithClockSync_SetsOffsetAndUncertaintyTogether()
    {
        var trace = LatencyTrace.ForSequence(9).WithClockSync(offsetTicks: -40, offsetUncertaintyTicks: 3);

        Assert.True(trace.TryGetClockOffsetTicks(out var offset));
        Assert.Equal(-40, offset);
        Assert.True(trace.TryGetClockOffsetUncertaintyTicks(out var offsetSigma));
        Assert.Equal(3, offsetSigma);
    }

    [Fact]
    public void PartiallyFilledTrace_UnsetFieldsRemainUnset()
    {
        var trace = LatencyTrace.ForSequence(5).WithCaptureTicks(1).WithUplinkSendTicks(2);

        Assert.True(trace.TryGetCaptureTicks(out _));
        Assert.True(trace.TryGetUplinkSendTicks(out _));
        Assert.False(trace.TryGetRobotRecvTicks(out _));
        Assert.False(trace.TryGetDownlinkSendTicks(out _));
        Assert.False(trace.TryGetOperatorRecvTicks(out _));
        Assert.False(trace.TryGetPlayoutTicks(out _));
        Assert.False(trace.TryGetRenderTicks(out _));
        Assert.False(trace.TryGetPhotonTicks(out _));
        Assert.False(trace.TryGetClockOffsetTicks(out _));
        Assert.False(trace.TryGetClockOffsetUncertaintyTicks(out _));
    }
}
