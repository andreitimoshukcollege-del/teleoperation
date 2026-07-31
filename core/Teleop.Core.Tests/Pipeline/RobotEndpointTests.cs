using System.Numerics;
using Teleop.Core.Pipeline;
using Teleop.Core.Plant;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Pipeline;

public class RobotEndpointTests
{
    private static RobotEndpoint MakeEndpoint(
        out LoopbackTransport uplink,
        out LoopbackTransport downlink,
        out RigidBodyPlant plant)
    {
        uplink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        downlink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        plant = new RigidBodyPlant(Pose.Identity, ticksPerSecond: 10_000_000);
        var robotClock = new ManualClock();

        return new RobotEndpoint(plant, new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink, robotClock);
    }

    private static void SendCommand(LoopbackTransport uplink, CommandFrame frame, long sendTicks)
    {
        var codec = new RawPoseCodec();
        byte[] buffer = new byte[RawPoseCodec.EncodedSize];
        codec.TryEncode(frame, buffer, out int n);
        uplink.Send(buffer.AsSpan(0, n), sendTicks);
    }

    [Fact]
    public void Step_WithEmptyUplinkQueue_StillStepsThePlant_AndSendsNothing()
    {
        var endpoint = MakeEndpoint(out _, out LoopbackTransport downlink, out RigidBodyPlant plant);
        var frame = new CommandFrame(0, 0, 0, Pose.Identity, new Vector3(1, 0, 0), Vector3.Zero, 0f);
        plant.Command(frame);

        endpoint.Step(1_000_000); // 0.1s at 10,000,000 ticks/sec

        Assert.Equal(1_000_000, plant.State.CaptureTicks);
        Assert.Equal(0.1f, plant.State.Value.Position.X, 3);

        byte[] buffer = new byte[128];
        bool anyReply = downlink.TryReceive(1_000_000, buffer, out _, out _);
        Assert.False(anyReply);
    }

    [Fact]
    public void Step_OneReceivedDatagram_ProducesExactlyOneReply_EchoingSequence()
    {
        var endpoint = MakeEndpoint(out LoopbackTransport uplink, out LoopbackTransport downlink, out _);
        var frame = new CommandFrame(42, 0, 0, Pose.Identity, Vector3.Zero, Vector3.Zero, 0f);
        SendCommand(uplink, frame, sendTicks: 0);

        endpoint.Step(100);

        byte[] buffer = new byte[128];
        var codec = new RobotStateFrameCodec();
        bool received = downlink.TryReceive(100, buffer, out int byteCount, out _);
        Assert.True(received);
        Assert.True(codec.TryDecode(buffer.AsSpan(0, byteCount), out RobotStateFrame reply));
        Assert.Equal(42u, reply.Sequence);

        bool secondReply = downlink.TryReceive(100, buffer, out _, out _);
        Assert.False(secondReply);
    }

    [Fact]
    public void Step_MultipleReceivedDatagrams_ProducesOneReplyEach()
    {
        var endpoint = MakeEndpoint(out LoopbackTransport uplink, out LoopbackTransport downlink, out _);
        SendCommand(uplink, new CommandFrame(1, 0, 0, Pose.Identity, Vector3.Zero, Vector3.Zero, 0f), 0);
        SendCommand(uplink, new CommandFrame(2, 1, 0, Pose.Identity, Vector3.Zero, Vector3.Zero, 0f), 0);

        endpoint.Step(100);

        byte[] buffer = new byte[128];
        var codec = new RobotStateFrameCodec();
        int replyCount = 0;
        while (downlink.TryReceive(100, buffer, out int byteCount, out _))
        {
            Assert.True(codec.TryDecode(buffer.AsSpan(0, byteCount), out _));
            replyCount++;
        }

        Assert.Equal(2, replyCount);
    }

    [Fact]
    public void Step_CorruptDatagram_IsDroppedWithoutThrowing_AndProducesNoReply()
    {
        var endpoint = MakeEndpoint(out LoopbackTransport uplink, out LoopbackTransport downlink, out _);
        byte[] garbage = new byte[RawPoseCodec.EncodedSize];
        garbage[0] = 99; // invalid version byte
        uplink.Send(garbage, 0);

        var exception = Record.Exception(() => endpoint.Step(100));

        Assert.Null(exception);
        byte[] buffer = new byte[128];
        Assert.False(downlink.TryReceive(100, buffer, out _, out _));
    }

    [Fact]
    public void Step_AppliesCommandBeforeReplying_ReplyReflectsSteppedState()
    {
        var endpoint = MakeEndpoint(out LoopbackTransport uplink, out LoopbackTransport downlink, out _);
        // Command a velocity of 1 m/s along X, captured at t=0.
        SendCommand(uplink, new CommandFrame(1, 0, 0, Pose.Identity, new Vector3(1, 0, 0), Vector3.Zero, 0f), sendTicks: 0);

        endpoint.Step(1_000_000); // steps 0.1s forward after applying the command

        byte[] buffer = new byte[128];
        var codec = new RobotStateFrameCodec();
        downlink.TryReceive(1_000_000, buffer, out int n, out _);
        codec.TryDecode(buffer.AsSpan(0, n), out RobotStateFrame reply);

        // The reply should reflect the plant's state AFTER this step's integration, not before.
        Assert.Equal(0.1f, reply.Pose.Position.X, 3);
    }

    [Fact]
    public void Reset_IsSafe_AndDoesNotAffectSubsequentSteps()
    {
        var endpoint = MakeEndpoint(out LoopbackTransport uplink, out LoopbackTransport downlink, out _);
        SendCommand(uplink, new CommandFrame(1, 0, 0, Pose.Identity, Vector3.Zero, Vector3.Zero, 0f), sendTicks: 0);

        endpoint.Reset();
        endpoint.Step(100);

        byte[] buffer = new byte[128];
        Assert.True(downlink.TryReceive(100, buffer, out _, out _));
    }

    [Fact]
    public void Step_Allocates_Zero_Bytes()
    {
        var endpoint = MakeEndpoint(out _, out _, out _);
        long ticks = 0;
        AllocationAssert.Zero(() => endpoint.Step(ticks += 1000));
    }
}
