// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>
    /// One displayed camera frame's latency, by stage, in milliseconds (docs/metrics.md §9). A stage
    /// that could not be computed is <see cref="double.NaN"/>: the capture stamp was unset, or the clocks
    /// were not yet synced. NaN is never recorded as a metric; it is only reported here so a HUD can
    /// show "unknown" rather than a number.
    ///
    /// The five stages telescope: <c>CaptureToSend + Owd + Handoff + Decode + DecodeToRender ==
    /// CaptureToRender</c>, to within one operator tick of conversion rounding.
    /// <see cref="Reassembly"/> is not a sixth stage: it is the part of <see cref="Owd"/> between the
    /// frame's first and last chunk arriving.
    /// </summary>
    public readonly struct CameraLatencySample
    {
        public readonly double CaptureToSend;
        public readonly double Owd;
        public readonly double Reassembly;
        public readonly double Handoff;
        public readonly double Decode;
        public readonly double DecodeToRender;
        public readonly double CaptureToRender;

        /// <summary>The robot-to-operator conversion was trusted (<c>ClockSyncDiagnostics.IsSynced</c>).</summary>
        public readonly bool Synced;

        /// <summary>
        /// False when the host's stamps were out of order (arrival after decode, say). Nothing is
        /// recorded for such a frame: a negative stage is a host bug, not a latency.
        /// </summary>
        public readonly bool StampsInOrder;

        public CameraLatencySample(
            double captureToSend, double owd, double reassembly, double handoff, double decode,
            double decodeToRender, double captureToRender, bool synced, bool stampsInOrder)
        {
            CaptureToSend = captureToSend;
            Owd = owd;
            Reassembly = reassembly;
            Handoff = handoff;
            Decode = decode;
            DecodeToRender = decodeToRender;
            CaptureToRender = captureToRender;
            Synced = synced;
            StampsInOrder = stampsInOrder;
        }
    }
}
