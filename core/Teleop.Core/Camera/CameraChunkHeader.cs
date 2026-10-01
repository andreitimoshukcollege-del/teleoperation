// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>One decoded chunk header: the frame's stamp plus this chunk's place in it.</summary>
    public readonly struct CameraChunkHeader
    {
        public readonly CameraFrameStamp Frame;
        public readonly ushort ChunkIndex;
        public readonly ushort ChunkCount;
        public readonly ushort PayloadBytes;

        public CameraChunkHeader(in CameraFrameStamp frame, ushort chunkIndex, ushort chunkCount, ushort payloadBytes)
        {
            Frame = frame;
            ChunkIndex = chunkIndex;
            ChunkCount = chunkCount;
            PayloadBytes = payloadBytes;
        }

        public bool IsLastChunk => ChunkIndex == ChunkCount - 1;
    }
}
