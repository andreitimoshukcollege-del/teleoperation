// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>
    /// A reassembled frame handed out by <see cref="CameraFrameReassembler.TryTakeLatest"/>. The
    /// stamp is in the robot's clock domain; the two arrival ticks are in the receiver's, exactly as
    /// the host passed them to <see cref="CameraFrameReassembler.Accept"/>.
    /// </summary>
    public readonly struct CameraFrameInfo
    {
        public readonly CameraFrameStamp Frame;
        public readonly int ByteCount;
        public readonly int ChunkCount;

        /// <summary>Arrival of the first chunk to arrive, which need not be chunk 0.</summary>
        public readonly long FirstChunkArrivalTicks;

        /// <summary>Arrival of the chunk that completed the frame: the frame's <c>t_recv</c>.</summary>
        public readonly long LastChunkArrivalTicks;

        public CameraFrameInfo(
            in CameraFrameStamp frame, int byteCount, int chunkCount,
            long firstChunkArrivalTicks, long lastChunkArrivalTicks)
        {
            Frame = frame;
            ByteCount = byteCount;
            ChunkCount = chunkCount;
            FirstChunkArrivalTicks = firstChunkArrivalTicks;
            LastChunkArrivalTicks = lastChunkArrivalTicks;
        }
    }
}
