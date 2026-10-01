using System;
using System.Buffers.Binary;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>
    /// The viewer's keepalive (docs/adr/0014-camera-frame-downlink.md §2): sent to the camera sender
    /// about once a second. The sender streams to the source of the most recent one and stops when
    /// none has arrived for a while, so it never pushes video at a viewer who has gone.
    ///
    /// <code>
    ///  0  u16  magic 0x5343 ("CS" on the wire)
    ///  2  u8   version
    ///  3  u16  maxFramesPerSecond  (0 = the sender's default)
    ///  5  i64  sendTicks           (viewer's clock; diagnostic only)
    /// </code>
    /// </summary>
    public sealed class CameraSubscribeCodec
    {
        public const ushort Magic = 0x5343;
        public const byte Version = 1;
        public const int EncodedSize = 13;

        public bool TryEncode(ushort maxFramesPerSecond, long sendTicks, Span<byte> destination, out int bytesWritten)
        {
            if (destination.Length < EncodedSize)
            {
                bytesWritten = EncodedSize;
                return false;
            }

            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(0, 2), Magic);
            destination[2] = Version;
            BinaryPrimitives.WriteUInt16LittleEndian(destination.Slice(3, 2), maxFramesPerSecond);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(5, 8), sendTicks);
            bytesWritten = EncodedSize;
            return true;
        }

        public bool TryDecode(ReadOnlySpan<byte> datagram, out ushort maxFramesPerSecond, out long sendTicks)
        {
            maxFramesPerSecond = 0;
            sendTicks = 0;
            if (datagram.Length != EncodedSize ||
                BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(0, 2)) != Magic ||
                datagram[2] != Version)
            {
                return false;
            }

            maxFramesPerSecond = BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(3, 2));
            sendTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(5, 8));
            return true;
        }
    }
}
