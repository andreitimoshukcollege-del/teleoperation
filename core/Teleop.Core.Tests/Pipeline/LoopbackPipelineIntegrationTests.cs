using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Pipeline;
using Teleop.Core.Plant;
using Teleop.Core.Prediction;
using Teleop.Core.Reconciliation;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Pipeline;

/// <summary>
/// The end-to-end headless proof that OperatorEndpoint + RobotEndpoint compose correctly:
/// a moving operator pose flows through encode -> transport -> plant -> encode -> transport ->
/// LatencyTrace, styled after Time/ClockSyncLatencyTraceIntegrationTests.cs.
///
/// Both endpoints share a single ManualClock here, deliberately. LoopbackTransport is a
/// zero-delay pass-through -- its "arrival tick" is inherited directly from whatever the sender
/// stamped at Send(), not independently measured by a receiving-side clock. That makes it
/// structurally unable to model two genuinely different clock domains (a real offset is "the
/// same physical instant read differently by two clocks," which a value-preserving pipe cannot
/// produce). ClockSync's own offset-computation correctness against hand-constructed
/// multi-domain timestamps is already covered by Time/ClockSyncLatencyTraceIntegrationTests.cs;
/// this test's job is to prove Pipeline's wiring is correct -- every field lands, in the right
/// place, in the right order -- not to re-derive ClockSync's algorithm with a different offset.
/// With a shared clock the true offset is zero, and asserting ClockSync reports (approximately)
/// zero is still a real, meaningful check that the round trip reaches it at all.
/// </summary>
public class LoopbackPipelineIntegrationTests
{
    private static ClockSyncConfig DefaultClockSyncConfig() => new ClockSyncConfig(
        historyCapacity: 16, smoothingAlpha: 0.5f, maxAcceptableRttTicks: 10_000_000,
        outlierRttMultiple: 10.0, minSamplesBeforeTrusted: 1);

    // Zero-mitigation baseline: PassthroughPredictor + SnapReconciler together reduce to
    // pass-through, matching this project's pre-Phase-5 behavior by construction. Same config
    // values as OperatorEndpointTests.MakeEndpoint, duplicated locally rather than shared across
    // files since these are representative test defaults, not a contract between the two files.
    private static IPredictor<Pose> MakePredictor() => new PassthroughPredictor(new PredictorConfig(
        maxHorizonTicks: 4_000_000, maxObservationGapTicks: 2_000_000, historyCapacity: 16,
        smoothingAlpha: 0.3f, smoothingBeta: 0.1f, processNoise: 0.01f, measurementNoise: 0.001f,
        maxLinearSpeed: 5f, maxAngularSpeed: 10f));

    private static IReconciler<Pose> MakeReconciler(IMetricSink metrics, ITimeAuthority clock) =>
        new SnapReconciler(new ReconcilerConfig(
            convergencePositionToleranceMeters: 0.001f, convergenceOrientationToleranceRadians: 0.01f,
            maxTimeToConvergenceTicks: 1_000_000, maxCorrectionLinearSpeedMetersPerSecond: 5f,
            maxCorrectionAngularSpeedRadPerSecond: 10f, rollbackHistoryCapacity: 16),
            metrics, clock);

    [Fact]
    public void Loopback_RoundTrip_PopulatesEveryHeadlessField_InCausalOrder()
    {
        const long ticksPerSecond = 10_000_000;
        var clock = new ManualClock(ticksPerSecond);

        var uplink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        var downlink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        var plant = new RigidBodyPlant(Pose.Identity, ticksPerSecond);
        var clockSync = new ClockSync(DefaultClockSyncConfig());
        var metrics = new InMemoryMetricTracker(capacity: 32);

        var operatorEndpoint = new OperatorEndpoint(
            new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink,
            clock, metrics, clockSync, MakePredictor(), MakeReconciler(metrics, clock),
            inFlightCapacity: 8);
        var robotEndpoint = new RobotEndpoint(
            plant, new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink, clock);

        LatencyTrace? lastCompleted = null;

        for (int i = 0; i < 20; i++)
        {
            clock.AdvanceTicks(100_000); // 10ms between commands
            long captureTicks = clock.NowTicks;
            var pose = new Pose(new Vector3(i * 0.01f, 0f, 1f), Quaternion.Identity);

            operatorEndpoint.SubmitCommand(pose, new Vector3(0.1f, 0, 0), Vector3.Zero, 0f, captureTicks);

            clock.AdvanceTicks(1_000); // robot processes a moment later
            robotEndpoint.Step(clock.NowTicks);

            clock.AdvanceTicks(1_000); // operator polls a moment after that
            bool received = operatorEndpoint.TryReceiveState(clock.NowTicks, out LatencyTrace completed);

            Assert.True(received, $"round trip {i} did not complete over a zero-delay transport");
            lastCompleted = completed;
        }

        Assert.NotNull(lastCompleted);
        LatencyTrace trace = lastCompleted!.Value;

        Assert.True(trace.TryGetCaptureTicks(out long capture));
        Assert.True(trace.TryGetUplinkSendTicks(out long uplinkSend));
        Assert.True(trace.TryGetRobotRecvTicks(out long robotRecv));
        Assert.True(trace.TryGetDownlinkSendTicks(out long downlinkSend));
        Assert.True(trace.TryGetOperatorRecvTicks(out long operatorRecv));
        Assert.True(trace.TryGetPlayoutTicks(out long playout));
        Assert.True(trace.TryGetClockOffsetTicks(out _));
        Assert.True(trace.TryGetClockOffsetUncertaintyTicks(out _));

        // Host/compositor-only fields: Core must never set these (docs/adr/0002-latency-trace.md).
        Assert.False(trace.TryGetRenderTicks(out _));
        Assert.False(trace.TryGetPhotonTicks(out _));

        Assert.True(capture <= uplinkSend);
        Assert.True(uplinkSend <= robotRecv);
        Assert.True(robotRecv <= downlinkSend);
        Assert.True(downlinkSend <= operatorRecv);
        Assert.True(operatorRecv <= playout);

        Assert.True(clockSync.Diagnostics.IsSynced);
        // A shared clock has zero true offset; ClockSync should recover (approximately) that.
        Assert.True(System.Math.Abs(clockSync.Diagnostics.OffsetTicks) < 10_000);
    }

    [Fact]
    public void Loopback_PlantTracksCommandedMotion()
    {
        const long ticksPerSecond = 10_000_000;
        var clock = new ManualClock(ticksPerSecond);
        var uplink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        var downlink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        var plant = new RigidBodyPlant(Pose.Identity, ticksPerSecond);
        var clockSync = new ClockSync(DefaultClockSyncConfig());
        var metrics = new InMemoryMetricTracker(capacity: 32);

        var operatorEndpoint = new OperatorEndpoint(
            new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink,
            clock, metrics, clockSync, MakePredictor(), MakeReconciler(metrics, clock),
            inFlightCapacity: 8);
        var robotEndpoint = new RobotEndpoint(
            plant, new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink, clock);

        // Command the robot to a specific pose and confirm it snaps there (zero mitigation).
        var target = new Pose(new Vector3(5f, 0f, 0f), Quaternion.Identity);
        clock.AdvanceTicks(100_000);
        operatorEndpoint.SubmitCommand(target, Vector3.Zero, Vector3.Zero, 0f, clock.NowTicks);
        robotEndpoint.Step(clock.NowTicks);

        Assert.Equal(5f, plant.State.Value.Position.X, 3);
    }

    [Fact]
    public void Loopback_TwoFreshRuns_ProduceIdenticalTraces_Deterministically()
    {
        LatencyTrace RunOnce()
        {
            const long ticksPerSecond = 10_000_000;
            var clock = new ManualClock(ticksPerSecond);
            var uplink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
            var downlink = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
            var plant = new RigidBodyPlant(Pose.Identity, ticksPerSecond);
            var clockSync = new ClockSync(DefaultClockSyncConfig());
            var metrics = new InMemoryMetricTracker(capacity: 32);

            var operatorEndpoint = new OperatorEndpoint(
                new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink,
                clock, metrics, clockSync, MakePredictor(), MakeReconciler(metrics, clock),
                inFlightCapacity: 8);
            var robotEndpoint = new RobotEndpoint(
                plant, new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink, clock);

            LatencyTrace completed = default;
            for (int i = 0; i < 5; i++)
            {
                clock.AdvanceTicks(100_000);
                operatorEndpoint.SubmitCommand(Pose.Identity, new Vector3(0.1f, 0, 0), Vector3.Zero, 0f, clock.NowTicks);
                clock.AdvanceTicks(1_000);
                robotEndpoint.Step(clock.NowTicks);
                clock.AdvanceTicks(1_000);
                operatorEndpoint.TryReceiveState(clock.NowTicks, out completed);
            }

            return completed;
        }

        LatencyTrace a = RunOnce();
        LatencyTrace b = RunOnce();

        Assert.Equal(a.ToString(), b.ToString());
    }

    [Fact]
    public void EmulatedTransport_InjectedDelay_ReflectedInMeasuredOneWayDelay()
    {
        const long ticksPerSecond = 10_000_000;
        const long injectedDelayTicks = 50_000; // 5ms
        var clock = new ManualClock(ticksPerSecond);

        var profile = new NetworkProfile(
            baseDelayTicks: injectedDelayTicks, jitterTicks: 0,
            lossProbabilityAfterDelivered: 0.0, lossProbabilityAfterLost: 0.0,
            reorderProbability: 0.0, reorderDelayTicks: 0);

        var uplinkInner = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        var uplink = new EmulatedTransport(uplinkInner, profile, new SeededRng(1), maxInFlight: 16);
        var downlinkInner = new LoopbackTransport(maxPayloadBytes: 128, capacity: 16);
        var downlink = new EmulatedTransport(downlinkInner, profile, new SeededRng(2), maxInFlight: 16);

        var plant = new RigidBodyPlant(Pose.Identity, ticksPerSecond);
        var clockSync = new ClockSync(DefaultClockSyncConfig());
        var metrics = new InMemoryMetricTracker(capacity: 32);

        var operatorEndpoint = new OperatorEndpoint(
            new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink,
            clock, metrics, clockSync, MakePredictor(), MakeReconciler(metrics, clock),
            inFlightCapacity: 8);
        var robotEndpoint = new RobotEndpoint(
            plant, new RawPoseCodec(), new RobotStateFrameCodec(), uplink, downlink, clock);

        operatorEndpoint.SubmitCommand(Pose.Identity, Vector3.Zero, Vector3.Zero, 0f, clock.NowTicks);

        bool received = false;
        LatencyTrace completed = default;
        for (int step = 0; step < 1000 && !received; step++)
        {
            clock.AdvanceTicks(1_000);
            robotEndpoint.Step(clock.NowTicks);
            if (operatorEndpoint.TryReceiveState(clock.NowTicks, out completed))
            {
                received = true;
            }
        }

        Assert.True(received, "round trip did not complete within the polling budget");
        Assert.True(metrics.TryGetLatest("owd_uplink_ms", out double uplinkMs, out _));
        Assert.True(metrics.TryGetLatest("owd_downlink_ms", out double downlinkMs, out _));

        double expectedMs = injectedDelayTicks * 1000.0 / ticksPerSecond;
        Assert.True(System.Math.Abs(uplinkMs - expectedMs) < 1.0, $"uplink OWD {uplinkMs}ms, expected ~{expectedMs}ms");
        Assert.True(System.Math.Abs(downlinkMs - expectedMs) < 1.0, $"downlink OWD {downlinkMs}ms, expected ~{expectedMs}ms");
    }
}
