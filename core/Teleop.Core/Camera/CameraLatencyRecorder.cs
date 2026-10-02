using System;
using Teleop.Core.Contracts;
using Teleop.Core.Time;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>
    /// Turns one displayed camera frame's stamps into the <c>camera_*</c> metrics defined in
    /// docs/metrics.md §9 (docs/adr/0014-camera-frame-downlink.md §9). Pure arithmetic: the host
    /// supplies every stamp and the pose path's <see cref="ClockSync"/>, which this reads and never
    /// updates. A one-way video stream has no round trips of its own (ADR 0014 §5).
    ///
    /// <para>Robot-domain stamps (<c>t_capture</c>, <c>t_send</c>) are converted to the operator's
    /// clock once, through <see cref="ClockSync.ToOperatorTicks"/>. Metrics that cross the two clocks
    /// are recorded only while the estimate is trusted; the rest (capture to send, reassembly,
    /// handoff, decode, decode to render) never cross clocks and are recorded always.</para>
    ///
    /// Allocation-free; metric names are constants.
    /// </summary>
    public sealed class CameraLatencyRecorder
    {
        public const string CaptureToSendMetric = "camera_capture_to_send_ms";
        public const string OwdMetric = "camera_owd_ms";
        public const string ReassemblyMetric = "camera_reassembly_ms";
        public const string HandoffMetric = "camera_handoff_ms";
        public const string DecodeMetric = "camera_decode_ms";
        public const string DecodeToRenderMetric = "camera_decode_to_render_ms";
        public const string CaptureToRenderMetric = "camera_capture_to_render_ms";
        public const string FrameDroppedMetric = "camera_frame_dropped";

        private readonly IMetricSink _sink;
        private readonly long _operatorTicksPerSecond;

        /// <param name="operatorTicksPerSecond">Rate of every receiver-side stamp passed to <see cref="Record"/>.</param>
        public CameraLatencyRecorder(IMetricSink sink, long operatorTicksPerSecond)
        {
            if (operatorTicksPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(operatorTicksPerSecond), operatorTicksPerSecond, "Must be positive.");
            }

            _sink = sink ?? throw new ArgumentNullException(nameof(sink));
            _operatorTicksPerSecond = operatorTicksPerSecond;
        }

        /// <summary>
        /// Records one frame that reached the screen. Every metric is stamped at
        /// <paramref name="renderTicks"/>, operator domain.
        /// </summary>
        /// <param name="frame">From <see cref="CameraFrameReassembler.TryTakeLatest"/>; its arrival stamps are operator domain.</param>
        /// <param name="clockSync">The pose path's estimate. Read only.</param>
        /// <param name="decodeStartTicks">When the host began decoding the JPEG.</param>
        /// <param name="decodeEndTicks">When the decoded image was ready to draw.</param>
        /// <param name="renderTicks">
        /// <c>t_render</c>: when the first frame drawing it was submitted. Host-defined in
        /// docs/metrics.md §9, as for M2P.
        /// </param>
        public CameraLatencySample Record(
            in CameraFrameInfo frame, ClockSync clockSync,
            long decodeStartTicks, long decodeEndTicks, long renderTicks)
        {
            if (clockSync == null)
            {
                throw new ArgumentNullException(nameof(clockSync));
            }

            bool inOrder =
                frame.FirstChunkArrivalTicks <= frame.LastChunkArrivalTicks &&
                frame.LastChunkArrivalTicks <= decodeStartTicks &&
                decodeStartTicks <= decodeEndTicks &&
                decodeEndTicks <= renderTicks;

            bool synced = clockSync.Diagnostics.IsSynced;
            if (!inOrder)
            {
                return new CameraLatencySample(
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, synced, false);
            }

            CameraFrameStamp stamp = frame.Frame;
            bool hasCapture = stamp.CaptureTicks != LatencyTrace.Unset;

            double reassembly = OperatorMs(frame.LastChunkArrivalTicks - frame.FirstChunkArrivalTicks);
            double handoff = OperatorMs(decodeStartTicks - frame.LastChunkArrivalTicks);
            double decode = OperatorMs(decodeEndTicks - decodeStartTicks);
            double decodeToRender = OperatorMs(renderTicks - decodeEndTicks);

            // Same clock on both ends: no sync needed, only the robot's rate.
            double captureToSend = hasCapture
                ? (stamp.SendTicks - stamp.CaptureTicks) * 1000.0 / stamp.TicksPerSecond
                : double.NaN;

            double owd = double.NaN;
            double captureToRender = double.NaN;
            if (synced)
            {
                long sendOperator = clockSync.ToOperatorTicks(stamp.SendTicks, stamp.TicksPerSecond, _operatorTicksPerSecond);
                owd = OperatorMs(frame.LastChunkArrivalTicks - sendOperator);
                if (hasCapture)
                {
                    long captureOperator = clockSync.ToOperatorTicks(stamp.CaptureTicks, stamp.TicksPerSecond, _operatorTicksPerSecond);
                    captureToRender = OperatorMs(renderTicks - captureOperator);
                }
            }

            if (hasCapture)
            {
                _sink.Record(CaptureToSendMetric, captureToSend, renderTicks);
            }

            if (synced)
            {
                _sink.Record(OwdMetric, owd, renderTicks);
            }

            _sink.Record(ReassemblyMetric, reassembly, renderTicks);
            _sink.Record(HandoffMetric, handoff, renderTicks);
            _sink.Record(DecodeMetric, decode, renderTicks);
            _sink.Record(DecodeToRenderMetric, decodeToRender, renderTicks);
            if (synced && hasCapture)
            {
                _sink.Record(CaptureToRenderMetric, captureToRender, renderTicks);
            }

            return new CameraLatencySample(
                captureToSend, owd, reassembly, handoff, decode, decodeToRender, captureToRender, synced, true);
        }

        /// <summary>
        /// One <c>camera_frame_dropped</c> sample of value 1 per frame that completed nowhere on screen,
        /// as <c>latency_trace_evicted</c> does for round trips. The host passes the increase in its
        /// drop counters since the last call.
        /// </summary>
        public void RecordDropped(long count, long ticks)
        {
            for (long i = 0; i < count; i++)
            {
                _sink.Record(FrameDroppedMetric, 1.0, ticks);
            }
        }

        private double OperatorMs(long ticks) => ticks * 1000.0 / _operatorTicksPerSecond;
    }
}
