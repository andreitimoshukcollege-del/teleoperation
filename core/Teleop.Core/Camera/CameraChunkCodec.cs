using System;
using System.Buffers.Binary;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>
    /// Wire format for one chunk of one camera frame (docs/adr/0014-camera-frame-downlink.md §3):
    /// a fixed 41-byte little-endian header, then a slice of the frame's JPEG. Every chunk but the
    /// last carries exactly <see cref="MaxPayloadBytes"/>, so a chunk's index alone places its bytes.
    ///
    /// <code>
    ///  0  u16  magic 0x5643 ("CV" on the wire)
    ///  2  u8   version
    ///  3  u32  frameId
    ///  7  u16  chunkIndex
    ///  9  u16  chunkCount
    /// 11  i64  captureTicks    (robot domain)
    /// 19  i64  sendTicks       (robot domain)
    /// 27  i64  ticksPerSecond  (robot clock rate, docs/adr/0008)
    /// 35  u16  width
    /// 37  u16  height
    /// 39  u16  payloadBytes
    /// 41  ...  payload
    /// </code>
    ///
    /// Pure and allocation-free: encodes into, and decodes from, caller-owned spans, with the same
    /// "false plus required length on a too-small buffer" contract as <c>ICommandCodec</c>.
    /// </summary>
    public sealed class CameraChunkCodec
    {
        public const ushort Magic = 0x5643;
        public const byte Version = 1;
        public const int HeaderSize = 41;

        /// <summary>
        /// Whole datagram, header included. Small enough never to fragment at the IP layer, with room
        /// for WireGuard's overhead when the path is Tailscale (ADR 0014 §3).
        /// </summary>
        public const int MaxDatagramBytes = 1200;

        public const int MaxPayloadBytes = MaxDatagramBytes - HeaderSize;

        /// <summary>Largest frame a chunk count can describe.</summary>
        public const int MaxFrameBytes = ushort.MaxValue * MaxPayloadBytes;

        /// <summary>How many chunks a frame of <paramref name="frameBytes"/> splits into; 0 for an empty frame.</summary>
        public static int ChunkCountFor(int frameBytes) =>
            frameBytes <= 0 ? 0 : (frameBytes + MaxPayloadBytes - 1) / MaxPayloadBytes;

        /// <summary>
        /// Writes chunk <paramref name="chunkIndex"/> of <paramref name="jpeg"/>. Returns false, with
        /// <paramref name="bytesWritten"/> set to the size needed, when <paramref name="destination"/>
        /// is too small; returns false with <paramref name="bytesWritten"/> 0 for an empty or oversized
        /// frame or an out-of-range index.
        /// </summary>
        public bool TryEncodeChunk(
            in CameraFrameStamp frame, ReadOnlySpan<byte> jpeg, int chunkIndex,
            Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = 0;
            int chunkCount = ChunkCountFor(jpeg.Length);
            if (chunkCount == 0 || jpeg.Length > MaxFrameBytes || chunkIndex < 0 || chunkIndex >= chunkCount)
            {
                return false;
            }

            int offset = chunkIndex * MaxPayloadBytes;
            int payloadBytes = Math.Min(MaxPayloadBytes, jpeg.Length - offset);
            int needed = HeaderSize + payloadBytes;
            if (destination.Length < needed)
            {
                bytesWritten = needed;
                return false;
            }

            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(0, 2), Magic);
            destination[2] = Version;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(3, 4), frame.FrameId);
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(7, 2), (ushort)chunkIndex);
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(9, 2), (ushort)chunkCount);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(11, 8), frame.CaptureTicks);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(19, 8), frame.SendTicks);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(27, 8), frame.TicksPerSecond);
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(35, 2), frame.Width);
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(37, 2), frame.Height);
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(39, 2), (ushort)payloadBytes);
            jpeg.Slice(offset, payloadBytes).CopyTo(destination.Slice(HeaderSize));

            bytesWritten = needed;
            return true;
        }

        /// <summary>
        /// Decodes one datagram. Rejects, without partial output, anything that is not exactly a
        /// well-formed chunk: wrong magic or version, an index outside the count, a payload length that
        /// disagrees with the datagram's size, or a non-final chunk that is not full.
        /// <paramref name="payload"/> aliases <paramref name="datagram"/>.
        /// </summary>
        public bool TryDecode(ReadOnlySpan<byte> datagram, out CameraChunkHeader header, out ReadOnlySpan<byte> payload)
        {
            header = default;
            payload = default;
            if (datagram.Length < HeaderSize ||
                BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(0, 2)) != Magic ||
                datagram[2] != Version)
            {
                return false;
            }

            uint frameId = BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(3, 4));
            ushort chunkIndex = BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(7, 2));
            ushort chunkCount = BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(9, 2));
            long captureTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(11, 8));
            long sendTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(19, 8));
            long ticksPerSecond = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(27, 8));
            ushort width = BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(35, 2));
            ushort height = BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(37, 2));
            ushort payloadBytes = BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(39, 2));

            bool isLast = chunkIndex == chunkCount - 1;
            if (chunkCount == 0 || chunkIndex >= chunkCount ||
                payloadBytes == 0 || payloadBytes > MaxPayloadBytes ||
                datagram.Length != HeaderSize + payloadBytes ||
                (!isLast && payloadBytes != MaxPayloadBytes))
            {
                return false;
            }

            var frame = new CameraFrameStamp(frameId, captureTicks, sendTicks, ticksPerSecond, width, height);
            header = new CameraChunkHeader(frame, chunkIndex, chunkCount, payloadBytes);
            payload = datagram.Slice(HeaderSize, payloadBytes);
            return true;
        }
    }
}
