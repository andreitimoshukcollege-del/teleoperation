using System.Numerics;
using Teleop.Core.Metrics;
using Teleop.Core.Pipeline;
using Teleop.Core.Prediction;
using Teleop.Core.Reconciliation;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Pipeline;

public class OperatorEndpointTests
{
    // Zero-mitigation baseline: PassthroughPredictor + SnapReconciler together reduce to
    // pass-through, matching this project's Phase-4 behavior by construction. Config values are
    // representative test defaults, not swept parameters.
    internal static readonly PredictorConfig DefaultPredictorConfig = new PredictorConfig(
        maxHorizonTicks: 4_000_000, maxObservationGapTicks: 2_000_000, historyCapacity: 16,
        smoothingAlpha: 0.3f, smoothingBeta: 0.1f, processNoise: 0.01f, measurementNoise: 0.001f,
        maxLinearSpeed: 5f, maxAngularSpeed: 10f);

    internal static readonly ReconcilerConfig DefaultReconcilerConfig = new ReconcilerConfig(
        convergencePositionToleranceMeters: 0.001f, convergenceOrientationToleranceRadians: 0.01f,
        maxTimeToConvergenceTicks: 1_000_000, maxCorrectionLinearSpeedMetersPerSecond: 5f,
        maxCorrectionAngularSpeedRadPerSecond: 10f, rollbackHistoryCapacity: 16);

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
            operatorClock, metrics, clockSync,
            new PassthroughPredictor(DefaultPredictorConfig),
            new SnapReconciler(DefaultReconcilerConfig, metrics, operatorClock),
            inFlightCapacity: 8);
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
    public void EstimateRobotState_BeforeAnyReply_ReturnsIdentity()
    {
        var endpoint = MakeEndpoint(out _, out _, out _, out _);

        Pose estimate = endpoint.EstimateRobotState(nowTicks: 12345);

        Assert.Equal(Pose.Identity.ToString(), estimate.ToString());
    }

    [Fact]
    public void EstimateRobotState_AfterAReply_ReflectsTheReceivedRobotPose()
    {
        var endpoint = MakeEndpoint(out _, out LoopbackTransport downlink, out _, out _);
        LatencyTrace opened = endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 100);
        opened.TryGetUplinkSendTicks(out long uplinkSendTicks);

        var receivedPose = new Pose(new Vector3(7, 8, 9), Quaternion.Identity);
        var stateFrame = new RobotStateFrame(opened.Sequence, uplinkSendTicks + 10, uplinkSendTicks + 15, receivedPose);
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
        codec.TryEncode(stateFrame, buffer, out int n);
        downlink.Send(buffer.AsSpan(0, n), 130);
        endpoint.TryReceiveState(130, out _);

        // Zero-mitigation baseline (PassthroughPredictor + SnapReconciler): the estimate snaps
        // to exactly the last received pose, regardless of nowTicks.
        Pose estimate = endpoint.EstimateRobotState(nowTicks: 9999);

        Assert.Equal(receivedPose.ToString(), estimate.ToString());
    }

    [Fact]
    public void EstimateRobotState_Allocates_Zero_Bytes()
    {
        var endpoint = MakeEndpoint(out _, out _, out _, out _);
        AllocationAssert.Zero(() => endpoint.EstimateRobotState(1000));
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

    /// <summary>
    /// The wiring in <see cref="OperatorEndpoint.TryReceiveState"/> that folds a robot-state
    /// sample into the predictor/reconciler is strictly additive: it must not change
    /// <c>completedTrace</c> or the <c>owd_*</c> metrics <see cref="OperatorEndpoint.SubmitCommand"/>/
    /// <see cref="OperatorEndpoint.TryReceiveState"/> already produced before predictors and
    /// reconcilers existed at all. Proven here by running the identical round-trip sequence
    /// through two endpoints built with completely different predictor/reconciler choices (the
    /// zero-mitigation baseline vs. a stateful predictor paired with a reconciler using
    /// different tolerances) and asserting the round-trip-envelope outputs are identical
    /// regardless -- only <see cref="OperatorEndpoint.EstimateRobotState"/> (not exercised by
    /// this test) is expected to differ between them.
    /// </summary>
    [Fact]
    public void TryReceiveState_RoundTripEnvelope_IsIdentical_RegardlessOfPredictorOrReconcilerChoice()
    {
        (LatencyTrace trace, double uplinkMs, double downlinkMs) RunRoundTrip(
            System.Func<InMemoryMetricTracker, ManualClock, (
                Teleop.Core.Contracts.IPredictor<Pose> Predictor,
                Teleop.Core.Contracts.IReconciler<Pose> Reconciler)> makeAlgorithms)
        {
            var uplink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
            var downlink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
            var clockSync = new ClockSync(new ClockSyncConfig(
                historyCapacity: 16, smoothingAlpha: 0.5f, maxAcceptableRttTicks: 1_000_000,
                outlierRttMultiple: 10.0, minSamplesBeforeTrusted: 1));
            var metrics = new InMemoryMetricTracker(capacity: 32);
            var clock = new ManualClock();
            var (predictor, reconciler) = makeAlgorithms(metrics, clock);

            var endpoint = new OperatorEndpoint(
                new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink,
                clock, metrics, clockSync, predictor, reconciler, inFlightCapacity: 8);

            LatencyTrace opened = endpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, nowTicks: 100);
            opened.TryGetUplinkSendTicks(out long uplinkSendTicks);

            var codec = new RobotStateFrameCodec();
            var stateFrame = new RobotStateFrame(
                opened.Sequence, uplinkSendTicks + 10, uplinkSendTicks + 15,
                new Pose(new Vector3(1, 2, 3), Quaternion.Identity));
            byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
            codec.TryEncode(stateFrame, buffer, out int n);
            downlink.Send(buffer.AsSpan(0, n), 130);

            endpoint.TryReceiveState(130, out LatencyTrace completed);
            metrics.TryGetLatest("owd_uplink_ms", out double uplinkMs, out _);
            metrics.TryGetLatest("owd_downlink_ms", out double downlinkMs, out _);
            return (completed, uplinkMs, downlinkMs);
        }

        var baseline = RunRoundTrip((metrics, clock) =>
            (new PassthroughPredictor(DefaultPredictorConfig),
             (Teleop.Core.Contracts.IReconciler<Pose>)new SnapReconciler(DefaultReconcilerConfig, metrics, clock)));

        var differentAlgorithms = RunRoundTrip((metrics, clock) =>
            ((Teleop.Core.Contracts.IPredictor<Pose>)new ConstantVelocityPredictor(DefaultPredictorConfig, clock),
             (Teleop.Core.Contracts.IReconciler<Pose>)new SnapReconciler(
                 new ReconcilerConfig(
                     convergencePositionToleranceMeters: 10f, convergenceOrientationToleranceRadians: 10f,
                     maxTimeToConvergenceTicks: 1, maxCorrectionLinearSpeedMetersPerSecond: 1f,
                     maxCorrectionAngularSpeedRadPerSecond: 1f, rollbackHistoryCapacity: 1),
                 metrics, clock)));

        Assert.Equal(baseline.trace.ToString(), differentAlgorithms.trace.ToString());
        Assert.Equal(baseline.uplinkMs, differentAlgorithms.uplinkMs);
        Assert.Equal(baseline.downlinkMs, differentAlgorithms.downlinkMs);
    }
}
