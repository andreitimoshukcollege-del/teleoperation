// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>What <see cref="CameraFrameReassembler.Accept"/> did with one datagram.</summary>
    public enum CameraChunkOutcome
    {
        /// <summary>Stored; its frame is still missing chunks.</summary>
        Stored,

        /// <summary>Stored, and it was the last missing chunk: a frame is ready to take.</summary>
        CompletedFrame,

        /// <summary>A chunk already held for its frame. Ignored.</summary>
        Duplicate,

        /// <summary>
        /// Belongs to a frame no newer than the newest frame already completed, or older than every
        /// frame being assembled when no slot is free. Newest wins, so it can never be shown.
        /// </summary>
        Late,

        /// <summary>Not a chunk at all (<see cref="CameraChunkCodec.TryDecode"/> refused it).</summary>
        Malformed,

        /// <summary>Its frame stamp disagrees with earlier chunks claiming the same frame id.</summary>
        Inconsistent,

        /// <summary>Its frame would exceed the reassembler's maximum frame size.</summary>
        TooLarge,
    }
}
