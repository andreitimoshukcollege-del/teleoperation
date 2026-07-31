using System.Numerics;
using Teleop.Core.Metrics;
using Teleop.Core.Pipeline;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Pipeline;

public class OperatorEndpointTests
{
    private static OperatorEndpoint MakeEndpoint(
        out LoopbackTransport uplink,
        out LoopbackTransport downlink,
        out ClockSync clockSync,
        out InMemoryMetricTracker metrics,
        ManualClock? clock = null)
    {
        uplink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        downlink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        clockSync = new ClockSync(new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.5f, maxAcceptableRttTicks: 1_000_000,
            outlierRttMultiple: 10.0, minSamplesBeforeTrusted: 1));
        metrics = new InMemoryMetricTracker(capacity: 32);
        var operatorClock = clock ?? new ManualClock();

        return new OperatorEndpoint(
            new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink,
            operatorClock, metrics, clockSync, inFlightCapacity: 8);
    }

    [Fact]
    public void SubmitCommand_AssignsIncreasingSequence_AndOpensPartialTrace()
    {
        var endpoint = MakeEndpoint(out _, out _, out _, out _);

        LatencyTrace first = endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 100);
        LatencyTrace second = endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 200);

        Assert.Equal(0u, first.Sequence);
        Assert.Equal(1u, second.Sequence);

        Assert.True(first.TryGetCaptureTicks(out long capture));
        Assert.Equal(100, capture);
        Assert.True(first.TryGetUplinkSendTicks(out long uplinkSend));
        Assert.Equal(100, uplinkSend);

        Assert.False(first.TryGetRobotRecvTicks(out _));
        Assert.False(first.TryGetDownlinkSendTicks(out _));
        Assert.False(first.TryGetOperatorRecvTicks(out _));
        Assert.False(first.TryGetPlayoutTicks(out _));
    }

    [Fact]
    public void SubmitCommand_SendsAnEncodedFrameOnTheUplinkTransport()
    {
        var endpoint = MakeEndpoint(out LoopbackTransport uplink, out _, out _, out _);

        endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 100);

        byte[] buffer = new byte[128];
        bool received = uplink.TryReceive(100, buffer, out int byteCount, out long arrivalTicks);

        Assert.True(received);
        Assert.Equal(RawPoseCodec.EncodedSize, byteCount);
        Assert.Equal(100, arrivalTicks);
    }

    [Fact]
    public void TryReceiveState_EmptyChannel_ReturnsFalse()
    {
        var endpoint = MakeEndpoint(out _, out _, out _, out _);

        bool received = endpoint.TryReceiveState(100, out LatencyTrace trace);

        Assert.False(received);
        Assert.Equal(default, trace);
    }

    [Fact]
    public void TryReceiveState_UnknownSequence_IsSilentlyIgnored()
    {
        var endpoint = MakeEndpoint(out _, out LoopbackTransport downlink, out _, out _);

        // A reply for a sequence never submitted.
        var stateFrame = new RobotStateFrame(sequence: 999, robotRecvTicks: 10, downlinkSendTicks: 20, Pose.Identity);
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
        codec.TryEncode(stateFrame, buffer, out int n);
        downlink.Send(buffer.AsSpan(0, n), 30);

        bool received = endpoint.TryReceiveState(30, out LatencyTrace trace);

        Assert.False(received);
        Assert.Equal(default, trace);
    }

    [Fact]
    public void TryReceiveState_MatchingReply_CompletesTheTrace()
    {
        var endpoint = MakeEndpoint(out _, out LoopbackTransport downlink, out ClockSync clockSync, out InMemoryMetricTracker metrics);

        LatencyTrace opened = endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 100);
        Assert.True(opened.TryGetUplinkSendTicks(out long uplinkSendTicks));

        var stateFrame = new RobotStateFrame(opened.Sequence, robotRecvTicks: uplinkSendTicks + 10, downlinkSendTicks: uplinkSendTicks + 15, Pose.Identity);
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
        codec.TryEncode(stateFrame, buffer, out int n);
        downlink.Send(buffer.AsSpan(0, n), 130);

        bool received = endpoint.TryReceiveState(130, out LatencyTrace completed);

        Assert.True(received);
        Assert.True(completed.TryGetRobotRecvTicks(out _));
        Assert.True(completed.TryGetDownlinkSendTicks(out _));
        Assert.True(completed.TryGetOperatorRecvTicks(out long operatorRecv));
        Assert.Equal(130, operatorRecv);
        Assert.True(completed.TryGetPlayoutTicks(out long playout));
        Assert.Equal(operatorRecv, playout);
        Assert.True(completed.TryGetClockOffsetTicks(out _));
        Assert.True(completed.TryGetClockOffsetUncertaintyTicks(out _));
        Assert.True(clockSync.Diagnostics.AcceptedSampleCount >= 1);
        Assert.True(metrics.TryGetLatest("owd_uplink_ms", out _, out _));
        Assert.True(metrics.TryGetLatest("owd_downlink_ms", out _, out _));
    }

    [Fact]
    public void TryReceiveState_SameSequenceTwice_SecondReplyIsIgnored()
    {
        var endpoint = MakeEndpoint(out _, out LoopbackTransport downlink, out _, out _);
        LatencyTrace opened = endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 100);
        opened.TryGetUplinkSendTicks(out long uplinkSendTicks);

        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
        var stateFrame = new RobotStateFrame(opened.Sequence, uplinkSendTicks + 10, uplinkSendTicks + 15, Pose.Identity);
        codec.TryEncode(stateFrame, buffer, out int n);

        downlink.Send(buffer.AsSpan(0, n), 130);
        Assert.True(endpoint.TryReceiveState(130, out _));

        downlink.Send(buffer.AsSpan(0, n), 140);
        bool secondReceived = endpoint.TryReceiveState(140, out LatencyTrace secondTrace);

        Assert.False(secondReceived);
        Assert.Equal(default, secondTrace);
    }

    [Fact]
    public void Reset_RestoresSequenceCounterAndClearsInFlightTraces()
    {
        var endpoint = MakeEndpoint(out _, out LoopbackTransport downlink, out _, out _);
        endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 100);

        endpoint.Reset();

        LatencyTrace afterReset = endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 200);
        Assert.Equal(0u, afterReset.Sequence);
    }

    [Fact]
    public void SubmitCommand_Allocates_Zero_Bytes()
    {
        var endpoint = MakeEndpoint(out _, out _, out _, out _);
        long ticks = 0;
        AllocationAssert.Zero(() => endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, ticks += 1000));
    }

    [Fact]
    public void TryReceiveState_OnEmptyChannel_Allocates_Zero_Bytes()
    {
        var endpoint = MakeEndpoint(out _, out _, out _, out _);
        AllocationAssert.Zero(() => endpoint.TryReceiveState(1000, out _));
    }
}
