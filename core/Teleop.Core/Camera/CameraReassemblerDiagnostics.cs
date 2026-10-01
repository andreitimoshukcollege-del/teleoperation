// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>
    /// Cumulative counts since construction or <see cref="CameraFrameReassembler.Reset"/>. Every
    /// frame that completes is eventually either taken or superseded, so
    /// <c>FramesCompleted == FramesTaken + DroppedSuperseded</c> once nothing is waiting to be taken
    /// (<see cref="FrameReady"/> false). Frames that never complete are counted in
    /// <see cref="DroppedIncomplete"/> when they are given up on, which is when a newer frame
    /// completes or their slot is needed.
    /// </summary>
    public readonly struct CameraReassemblerDiagnostics
    {
        public readonly long FramesCompleted;
        public readonly long FramesTaken;
        public readonly long DroppedIncomplete;
        public readonly long DroppedSuperseded;
        public readonly long LateChunks;
        public readonly long DuplicateChunks;
        public readonly long MalformedDatagrams;
        public readonly long InconsistentChunks;
        public readonly long TooLargeChunks;
        public readonly int FramesAssembling;
        public readonly bool FrameReady;

        public CameraReassemblerDiagnostics(
            long framesCompleted, long framesTaken, long droppedIncomplete, long droppedSuperseded,
            long lateChunks, long duplicateChunks, long malformedDatagrams, long inconsistentChunks,
            long tooLargeChunks, int framesAssembling, bool frameReady)
        {
            FramesCompleted = framesCompleted;
            FramesTaken = framesTaken;
            DroppedIncomplete = droppedIncomplete;
            DroppedSuperseded = droppedSuperseded;
            LateChunks = lateChunks;
            DuplicateChunks = duplicateChunks;
            MalformedDatagrams = malformedDatagrams;
            InconsistentChunks = inconsistentChunks;
            TooLargeChunks = tooLargeChunks;
            FramesAssembling = framesAssembling;
            FrameReady = frameReady;
        }
    }
}
