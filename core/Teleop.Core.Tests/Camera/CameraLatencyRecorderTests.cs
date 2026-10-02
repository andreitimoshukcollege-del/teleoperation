using Teleop.Core.Camera;
using Teleop.Core.Contracts;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Camera;

/// <summary>
/// <see cref="CameraLatencyRecorder"/>: each docs/metrics.md §9 stage is the difference of adjacent
/// stamps, the stages sum to the headline figure, and nothing that crosses clocks is recorded until
/// <see cref="ClockSync"/> is trusted.
/// </summary>
public sealed class CameraLatencyRecorderTests
{
    private const long RobotTps = 1_000_000_000;   // Jetson Stopwatch
    private const long OperatorTps = 10_000_000;   // Windows Stopwatch
    private const long OpMs = OperatorTps / 1000;
    private const long RobotMs = RobotTps / 1000;

    private sealed class RecordingSink : IMetricSink
    {
        public readonly List<(string Name, double Value, long Ticks)> Samples = new();
        public void Record(string name, double value, long ticks) => Samples.Add((name, value, ticks));
        public double Only(string name) => Samples.Single(s => s.Name == name).Value;
        public bool Has(string name) => Samples.Any(s => s.Name == name);
    }

    /// <summary>A ClockSync trusted after one symmetric round trip, robot clock 0.5 s behind and 100x the rate.</summary>
    private static ClockSync Synced()
    {
        var sync = new ClockSync(new ClockSyncConfig(
            historyCapacity: 16, smoothingAlpha: 0.5f, maxAcceptableRttTicks: 10_000_000,
            outlierRttMultiple: 10.0, minSamplesBeforeTrusted: 1));
        const long offsetOp = 5_000_000; // operator = robot/100 + offset
        long opSend = 100_000_000, oneWay = 10 * OpMs;
        long robotAt = (opSend + oneWay - offsetOp) * (RobotTps / OperatorTps);
        Assert.True(sync.AddRoundTrip(opSend, OperatorTps, robotAt, robotAt, RobotTps, opSend + 2 * oneWay));
        Assert.True(sync.Diagnostics.IsSynced);
        return sync;
    }

    private static ClockSync Unsynced() => new(new ClockSyncConfig(16, 0.5f, 10_000_000, 10.0, minSamplesBeforeTrusted: 1));

    /// <summary>
    /// A frame captured at robot time <paramref name="capture"/>, sent 32 ms later, fully arrived 5 ms
    /// after that (operator clock), spread over 2 ms, then 1 ms handoff, 3 ms decode, 2 ms to render.
    /// </summary>
    private static (CameraFrameInfo Info, long DecodeStart, long DecodeEnd, long Render) Frame(
        ClockSync sync, long capture = 900 * RobotMs, bool unsetCapture = false)
    {
        long send = capture + 32 * RobotMs;
        long sendOp = sync.ToOperatorTicks(send, RobotTps, OperatorTps);
        long last = sendOp + 5 * OpMs, first = last - 2 * OpMs;
        var stamp = new CameraFrameStamp(1, unsetCapture ? LatencyTrace.Unset : capture, send, RobotTps, 640, 480);
        var info = new CameraFrameInfo(stamp, 42_000, 37, first, last);
        return (info, last + 1 * OpMs, last + 4 * OpMs, last + 6 * OpMs);
    }

    [Fact]
    public void EveryStage_IsTheDifferenceOfAdjacentStamps_AndTheyAddUp()
    {
        var sink = new RecordingSink();
        var recorder = new CameraLatencyRecorder(sink, OperatorTps);
        ClockSync sync = Synced();
        var f = Frame(sync);

        CameraLatencySample s = recorder.Record(f.Info, sync, f.DecodeStart, f.DecodeEnd, f.Render);

        Assert.True(s.Synced);
        Assert.True(s.StampsInOrder);
        Assert.Equal(32.0, s.CaptureToSend, 6);
        Assert.Equal(5.0, s.Owd, 6);
        Assert.Equal(2.0, s.Reassembly, 6);
        Assert.Equal(1.0, s.Handoff, 6);
        Assert.Equal(3.0, s.Decode, 6);
        Assert.Equal(2.0, s.DecodeToRender, 6);
        Assert.Equal(43.0, s.CaptureToRender, 3);
        Assert.Equal(s.CaptureToRender, s.CaptureToSend + s.Owd + s.Handoff + s.Decode + s.DecodeToRender, 3);

        Assert.Equal(7, sink.Samples.Count);
        Assert.All(sink.Samples, m => Assert.Equal(f.Render, m.Ticks));
        Assert.Equal(43.0, sink.Only(CameraLatencyRecorder.CaptureToRenderMetric), 3);
        Assert.Equal(5.0, sink.Only(CameraLatencyRecorder.OwdMetric), 6);
    }

    [Theory]
    [InlineData(1L)]
    [InlineData(7L)]
    [InlineData(123_456_789L)]
    public void TheStagesSumToCaptureToRender_WithinOneOperatorTick(long seed)
    {
        var rng = new SeededRng((ulong)seed);
        var recorder = new CameraLatencyRecorder(new RecordingSink(), OperatorTps);
        ClockSync sync = Synced();
        for (int i = 0; i < 200; i++)
        {
            long capture = 1_000 * RobotMs + (long)(rng.NextDouble() * 10_000 * RobotMs);
            long send = capture + (long)(rng.NextDouble() * 80 * RobotMs);
            long last = sync.ToOperatorTicks(send, RobotTps, OperatorTps) + (long)(rng.NextDouble() * 400 * OpMs);
            long first = last - (long)(rng.NextDouble() * 20 * OpMs);
            long decodeStart = last + (long)(rng.NextDouble() * 15 * OpMs);
            long decodeEnd = decodeStart + (long)(rng.NextDouble() * 15 * OpMs);
            long render = decodeEnd + (long)(rng.NextDouble() * 12 * OpMs);
            var info = new CameraFrameInfo(new CameraFrameStamp((uint)i, capture, send, RobotTps, 640, 480), 1, 1, first, last);

            CameraLatencySample s = recorder.Record(info, sync, decodeStart, decodeEnd, render);
            double sum = s.CaptureToSend + s.Owd + s.Handoff + s.Decode + s.DecodeToRender;
            Assert.True(Math.Abs(sum - s.CaptureToRender) <= 1000.0 / OperatorTps, $"sum {sum} vs {s.CaptureToRender}");
        }
    }

    [Fact]
    public void BeforeClockSyncIsTrusted_NothingThatCrossesClocksIsRecorded()
    {
        var sink = new RecordingSink();
        var recorder = new CameraLatencyRecorder(sink, OperatorTps);
        ClockSync sync = Unsynced();
        var f = Frame(sync);

        CameraLatencySample s = recorder.Record(f.Info, sync, f.DecodeStart, f.DecodeEnd, f.Render);

        Assert.False(s.Synced);
        Assert.True(double.IsNaN(s.Owd));
        Assert.True(double.IsNaN(s.CaptureToRender));
        Assert.False(sink.Has(CameraLatencyRecorder.OwdMetric));
        Assert.False(sink.Has(CameraLatencyRecorder.CaptureToRenderMetric));

        // Same-clock stages are still recorded.
        Assert.Equal(32.0, sink.Only(CameraLatencyRecorder.CaptureToSendMetric), 6);
        Assert.Equal(3.0, sink.Only(CameraLatencyRecorder.DecodeMetric), 6);
        Assert.Equal(5, sink.Samples.Count);
    }

    [Fact]
    public void AnUnsetCaptureStamp_RecordsNoCaptureBasedMetric()
    {
        var sink = new RecordingSink();
        var recorder = new CameraLatencyRecorder(sink, OperatorTps);
        ClockSync sync = Synced();
        var f = Frame(sync, unsetCapture: true);

        CameraLatencySample s = recorder.Record(f.Info, sync, f.DecodeStart, f.DecodeEnd, f.Render);

        Assert.True(double.IsNaN(s.CaptureToSend));
        Assert.True(double.IsNaN(s.CaptureToRender));
        Assert.False(sink.Has(CameraLatencyRecorder.CaptureToSendMetric));
        Assert.False(sink.Has(CameraLatencyRecorder.CaptureToRenderMetric));
        Assert.Equal(5.0, sink.Only(CameraLatencyRecorder.OwdMetric), 6);
    }

    [Fact]
    public void OutOfOrderStamps_RecordNothing()
    {
        var sink = new RecordingSink();
        var recorder = new CameraLatencyRecorder(sink, OperatorTps);
        ClockSync sync = Synced();
        var f = Frame(sync);

        CameraLatencySample s = recorder.Record(f.Info, sync, f.DecodeEnd, f.DecodeStart, f.Render); // decode swapped

        Assert.False(s.StampsInOrder);
        Assert.Empty(sink.Samples);
    }

    [Fact]
    public void RecordDropped_EmitsOneUnitSamplePerFrame()
    {
        var sink = new RecordingSink();
        new CameraLatencyRecorder(sink, OperatorTps).RecordDropped(3, ticks: 77);

        Assert.Equal(3, sink.Samples.Count);
        Assert.All(sink.Samples, m =>
        {
            Assert.Equal(CameraLatencyRecorder.FrameDroppedMetric, m.Name);
            Assert.Equal(1.0, m.Value);
            Assert.Equal(77, m.Ticks);
        });
    }

    [Fact]
    public void Record_IsAllocationFree()
    {
        var recorder = new CameraLatencyRecorder(new NullCountingSink(), OperatorTps);
        ClockSync sync = Synced();
        var f = Frame(sync);
        AllocationAssert.Zero(() => recorder.Record(f.Info, sync, f.DecodeStart, f.DecodeEnd, f.Render));
    }

    [Fact]
    public void Constructor_RefusesANullSinkOrABadRate()
    {
        Assert.Throws<ArgumentNullException>(() => new CameraLatencyRecorder(null!, OperatorTps));
        Assert.Throws<ArgumentOutOfRangeException>(() => new CameraLatencyRecorder(new RecordingSink(), 0));
    }

    private sealed class NullCountingSink : IMetricSink
    {
        public long Count;
        public void Record(string name, double value, long ticks) => Count++;
    }
}
