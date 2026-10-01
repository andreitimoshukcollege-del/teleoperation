// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>
    /// What every chunk of one camera frame repeats about that frame
    /// (docs/adr/0014-camera-frame-downlink.md §3). All tick values are in the <b>robot's</b>
    /// clock domain, counted at <see cref="TicksPerSecond"/>, exactly as a
    /// <c>RobotStateFrame</c>'s are (docs/adr/0008); conversion to the operator's clock happens once,
    /// operator-side, through <c>ClockSync</c> (ADR 0014 §5).
    /// </summary>
    public readonly struct CameraFrameStamp
    {
        /// <summary>Increments per frame, wrapping; compared with wrap-aware arithmetic.</summary>
        public readonly uint FrameId;

        /// <summary>
        /// The driver's capture timestamp (start of exposure on the JetRover's camera), or
        /// <c>LatencyTrace.Unset</c> when the sender could not obtain it (ADR 0014 §4). Never the
        /// time the sender happened to read the frame.
        /// </summary>
        public readonly long CaptureTicks;

        /// <summary>When the frame's first chunk was handed to the socket.</summary>
        public readonly long SendTicks;

        /// <summary>The robot clock's rate, the unit of the two stamps above.</summary>
        public readonly long TicksPerSecond;

        public readonly ushort Width;
        public readonly ushort Height;

        public CameraFrameStamp(
            uint frameId, long captureTicks, long sendTicks, long ticksPerSecond, ushort width, ushort height)
        {
            FrameId = frameId;
            CaptureTicks = captureTicks;
            SendTicks = sendTicks;
            TicksPerSecond = ticksPerSecond;
            Width = width;
            Height = height;
        }

        public bool SameFrameAs(in CameraFrameStamp other) =>
            FrameId == other.FrameId &&
            CaptureTicks == other.CaptureTicks &&
            SendTicks == other.SendTicks &&
            TicksPerSecond == other.TicksPerSecond &&
            Width == other.Width &&
            Height == other.Height;
    }
}
