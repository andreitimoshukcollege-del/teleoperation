using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Turns a <see cref="CommandFrame"/> into bytes and back. Implementations live in
    /// <c>Transport/</c>.
    ///
    /// An underrated lever rather than plumbing: the wire format decides what mitigation is
    /// even possible downstream. A codec that sends an instantaneous pose leaves the receiver
    /// nothing to do through a lost packet; one that sends intended future motion leaves it a
    /// plan to follow. Codecs are therefore benchmarked as pairs with the predictor.
    ///
    /// Buffers are caller-owned spans so that encoding a frame every tick allocates nothing.
    /// Codecs may be stateful (delta against last acked, N-frame redundancy), so a decoder must
    /// be fed the stream in arrival order and <see cref="Reset"/> between trials.
    /// </summary>
    public interface ICommandCodec
    {
        /// <summary>
        /// Upper bound on the bytes <see cref="TryEncode"/> can produce for one frame. Callers
        /// size buffers from this and check it against <c>ITransport.MaxPayloadBytes</c> once at
        /// wiring time rather than per frame.
        /// </summary>
        int MaxEncodedBytes { get; }

        /// <summary>
        /// Encode one frame into <paramref name="destination"/>. Returns false when the buffer
        /// is too small, setting <paramref name="bytesWritten"/> to the length required and
        /// leaving codec state unchanged — a failed encode must not consume the delta baseline
        /// or a redundancy slot, or the stream desynchronizes.
        ///
        /// Deterministic: the same frame and the same codec state produce identical bytes.
        /// Allocation-free.
        /// </summary>
        bool TryEncode(in CommandFrame frame, Span<byte> destination, out int bytesWritten);

        /// <summary>
        /// Decode one frame from <paramref name="source"/>. Returns false on a truncated,
        /// corrupt, or unsupported-version payload; a decoder must reject rather than throw,
        /// because a lossy link legitimately delivers garbage and the loop cannot stop for it.
        ///
        /// <paramref name="frame"/> is <c>default</c> when this returns false. A codec carrying
        /// redundant copies may find several frames in one datagram; it returns the newest here
        /// and exposes the rest through its own surface, since recovering older frames is
        /// codec-specific and not part of this contract.
        /// Allocation-free.
        /// </summary>
        bool TryDecode(ReadOnlySpan<byte> source, out CommandFrame frame);

        /// <summary>
        /// Returns the codec to its as-constructed state: no delta baseline, no redundancy
        /// history, no quantization residual. Both ends must reset together — a decoder holding
        /// a baseline the encoder has forgotten produces plausible-looking wrong poses.
        /// </summary>
        void Reset();
    }
}
