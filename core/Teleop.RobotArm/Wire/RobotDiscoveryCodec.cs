using System;
using System.Buffers.Binary;
using System.Net;
using System.Text;

namespace Teleop.RobotArm.Wire
{
    /// <summary>
    /// The discovery query and reply (docs/adr/0022-robot-discovery.md): the headset asks "where are
    /// you?" on the robot's discovery port, and RobotHost answers with its hostname and its IPv4 LAN
    /// addresses. Little-endian, versioned, stateless; a reply carries the query's nonce, so a stale or
    /// stray reply is told apart from the answer.
    ///
    /// <para>Query, <see cref="QuerySize"/> bytes: magic <c>0x5244</c>, version, type 1, nonce (u32).
    /// Reply: magic, version, type 2, nonce, address count (u8, at most <see cref="MaxAddresses"/>),
    /// 4 bytes per IPv4 address, name length (u8, at most <see cref="MaxNameBytes"/>), ASCII name.</para>
    /// </summary>
    public static class RobotDiscoveryCodec
    {
        public const ushort Magic = 0x5244;
        public const byte Version = 1;
        public const byte QueryType = 1;
        public const byte ReplyType = 2;
        public const int QuerySize = 2 + 1 + 1 + 4;
        public const int MaxAddresses = 8;
        public const int MaxNameBytes = 32;
        public const int MaxReplySize = QuerySize + 1 + MaxAddresses * 4 + 1 + MaxNameBytes;

        /// <summary>The robot's discovery port (RobotHost <c>--discovery-port</c>).</summary>
        public const int DefaultPort = 6005;

        public static bool TryEncodeQuery(uint nonce, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = QuerySize;
            if (destination.Length < QuerySize) return false;
            WriteHeader(destination, QueryType, nonce);
            return true;
        }

        public static bool TryDecodeQuery(ReadOnlySpan<byte> source, out uint nonce)
        {
            nonce = 0;
            if (source.Length < QuerySize || !HeaderIs(source, QueryType)) return false;
            nonce = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(4));
            return true;
        }

        /// <summary>A reply with the first <see cref="MaxAddresses"/> IPv4 addresses (others are skipped) and the name, cut to <see cref="MaxNameBytes"/> ASCII bytes.</summary>
        public static bool TryEncodeReply(uint nonce, ReadOnlySpan<IPAddress> addresses, string name, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = 0;
            if (destination.Length < MaxReplySize) return false;
            WriteHeader(destination, ReplyType, nonce);
            int pos = QuerySize;
            int countAt = pos++;
            int count = 0;
            Span<byte> ip = stackalloc byte[16];
            foreach (IPAddress address in addresses)
            {
                if (count == MaxAddresses) break;
                if (address.AddressFamily != System.Net.Sockets.AddressFamily.InterNetwork || !address.TryWriteBytes(ip, out int n) || n != 4) continue;
                ip.Slice(0, 4).CopyTo(destination.Slice(pos));
                pos += 4;
                count++;
            }

            destination[countAt] = (byte)count;
            byte[] nameBytes = Encoding.ASCII.GetBytes(name ?? string.Empty);
            int nameLength = Math.Min(nameBytes.Length, MaxNameBytes);
            destination[pos++] = (byte)nameLength;
            nameBytes.AsSpan(0, nameLength).CopyTo(destination.Slice(pos));
            bytesWritten = pos + nameLength;
            return true;
        }

        /// <summary>Decodes a reply into <paramref name="addresses"/> (at least <see cref="MaxAddresses"/> long). False on anything malformed.</summary>
        public static bool TryDecodeReply(ReadOnlySpan<byte> source, out uint nonce, Span<uint> addresses, out int addressCount, out string name)
        {
            nonce = 0;
            addressCount = 0;
            name = string.Empty;
            if (source.Length < QuerySize + 2 || !HeaderIs(source, ReplyType) || addresses.Length < MaxAddresses) return false;
            int pos = QuerySize;
            int count = source[pos++];
            if (count > MaxAddresses || source.Length < pos + count * 4 + 1) return false;
            for (int i = 0; i < count; i++)
            {
                // Network order on the wire, as IPAddress stores it: kept as the 4 bytes, read big-endian.
                addresses[i] = BinaryPrimitives.ReadUInt32BigEndian(source.Slice(pos));
                pos += 4;
            }

            int nameLength = source[pos++];
            if (nameLength > MaxNameBytes || source.Length < pos + nameLength) return false;
            nonce = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(4));
            addressCount = count;
            name = Encoding.ASCII.GetString(source.Slice(pos, nameLength));
            return true;
        }

        /// <summary>An address as <see cref="TryDecodeReply"/> returns it, back to an <see cref="IPAddress"/>.</summary>
        public static IPAddress ToAddress(uint address)
        {
            Span<byte> b = stackalloc byte[4];
            BinaryPrimitives.WriteUInt32BigEndian(b, address);
            return new IPAddress(b);
        }

        private static void WriteHeader(Span<byte> destination, byte type, uint nonce)
        {
            BinaryPrimitives.WriteUInt16LittleEndian(destination, Magic);
            destination[2] = Version;
            destination[3] = type;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(4), nonce);
        }

        private static bool HeaderIs(ReadOnlySpan<byte> source, byte type) =>
            BinaryPrimitives.ReadUInt16LittleEndian(source) == Magic && source[2] == Version && source[3] == type;
    }
}
