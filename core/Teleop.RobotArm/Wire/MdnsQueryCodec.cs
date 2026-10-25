using System;
using System.Buffers.Binary;
using System.Net;
using System.Text;

namespace Teleop.RobotArm.Wire
{
    /// <summary>
    /// A one-question mDNS client's wire format (docs/adr/0015-robot-address-discovery.md §2, used by
    /// docs/adr/0022 step 2): an A query for one name, sent to <see cref="Group"/>:<see cref="Port"/>
    /// from an ephemeral port, which makes it an RFC 6762 §6.7 "legacy unicast" query, so the robot's
    /// own mDNS responder answers by unicast to the asking port and no multicast has to be received.
    /// Not an mDNS implementation.
    ///
    /// <para>The parser treats malformed input as everyday input on a shared multicast group: it
    /// rejects pointer loops, labels or records running past the end, counts that disagree with the
    /// bytes, and answers for another name. It allocates only for the name comparison.</para>
    /// </summary>
    public static class MdnsQueryCodec
    {
        public const int Port = 5353;
        public static readonly IPAddress Group = IPAddress.Parse("224.0.0.251");
        public const int MaxMessageBytes = 512;
        private const int HeaderSize = 12;
        private const ushort TypeA = 1;
        private const ushort ClassIn = 1;
        private const int MaxPointerJumps = 16;

        /// <summary>A standard query (flags 0) for the A record of <paramref name="name"/> (e.g. <c>jetrover-sinrg.local</c>), with a non-zero <paramref name="id"/>.</summary>
        public static bool TryEncodeQuery(string name, ushort id, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = 0;
            if (id == 0 || string.IsNullOrEmpty(name) || destination.Length < HeaderSize) return false;
            destination.Slice(0, HeaderSize).Clear();
            BinaryPrimitives.WriteUInt16BigEndian(destination, id);
            BinaryPrimitives.WriteUInt16BigEndian(destination.Slice(4), 1); // one question
            int pos = HeaderSize;
            foreach (string label in name.TrimEnd('.').Split('.'))
            {
                byte[] bytes = Encoding.ASCII.GetBytes(label);
                if (bytes.Length == 0 || bytes.Length > 63 || pos + 1 + bytes.Length + 5 > destination.Length) return false;
                destination[pos++] = (byte)bytes.Length;
                bytes.CopyTo(destination.Slice(pos));
                pos += bytes.Length;
            }

            destination[pos++] = 0;
            BinaryPrimitives.WriteUInt16BigEndian(destination.Slice(pos), TypeA);
            BinaryPrimitives.WriteUInt16BigEndian(destination.Slice(pos + 2), ClassIn);
            bytesWritten = pos + 4;
            return true;
        }

        /// <summary>
        /// Whether <paramref name="message"/> is a response to query <paramref name="id"/> carrying an
        /// A record for <paramref name="name"/>; if so, its first such address and TTL.
        /// </summary>
        public static bool TryParseAnswer(ReadOnlySpan<byte> message, ushort id, string name, out IPAddress address, out uint ttlSeconds)
        {
            address = IPAddress.None;
            ttlSeconds = 0;
            if (message.Length < HeaderSize || BinaryPrimitives.ReadUInt16BigEndian(message) != id) return false;
            ushort flags = BinaryPrimitives.ReadUInt16BigEndian(message.Slice(2));
            if ((flags & 0x8000) == 0) return false; // a query, not a response
            int questions = BinaryPrimitives.ReadUInt16BigEndian(message.Slice(4));
            int answers = BinaryPrimitives.ReadUInt16BigEndian(message.Slice(6));
            int pos = HeaderSize;
            for (int q = 0; q < questions; q++)
            {
                if (!SkipName(message, ref pos) || pos + 4 > message.Length) return false;
                pos += 4;
            }

            string wanted = name.TrimEnd('.');
            for (int a = 0; a < answers; a++)
            {
                int nameAt = pos;
                if (!SkipName(message, ref pos) || pos + 10 > message.Length) return false;
                ushort type = BinaryPrimitives.ReadUInt16BigEndian(message.Slice(pos));
                ushort cls = (ushort)(BinaryPrimitives.ReadUInt16BigEndian(message.Slice(pos + 2)) & 0x7FFF); // less the cache-flush bit
                uint ttl = BinaryPrimitives.ReadUInt32BigEndian(message.Slice(pos + 4));
                int length = BinaryPrimitives.ReadUInt16BigEndian(message.Slice(pos + 8));
                pos += 10;
                if (pos + length > message.Length) return false;
                if (type == TypeA && cls == ClassIn && length == 4 &&
                    TryReadName(message, nameAt, out string answerName) && string.Equals(answerName, wanted, StringComparison.OrdinalIgnoreCase))
                {
                    address = new IPAddress(message.Slice(pos, 4));
                    ttlSeconds = ttl;
                    return true;
                }

                pos += length;
            }

            return false;
        }

        /// <summary>Moves past a name: labels, ending in a zero byte or a two-byte pointer.</summary>
        private static bool SkipName(ReadOnlySpan<byte> message, ref int pos)
        {
            while (pos < message.Length)
            {
                byte length = message[pos];
                if (length == 0)
                {
                    pos++;
                    return true;
                }

                if ((length & 0xC0) == 0xC0)
                {
                    if (pos + 2 > message.Length) return false;
                    pos += 2;
                    return true;
                }

                if ((length & 0xC0) != 0) return false;
                pos += 1 + length;
            }

            return false;
        }

        /// <summary>Reads the name at <paramref name="pos"/>, following compression pointers (at most <see cref="MaxPointerJumps"/>).</summary>
        private static bool TryReadName(ReadOnlySpan<byte> message, int pos, out string name)
        {
            name = string.Empty;
            var b = new StringBuilder();
            int jumps = 0;
            while (true)
            {
                if (pos >= message.Length) return false;
                byte length = message[pos];
                if (length == 0) break;
                if ((length & 0xC0) == 0xC0)
                {
                    if (pos + 2 > message.Length || ++jumps > MaxPointerJumps) return false;
                    pos = BinaryPrimitives.ReadUInt16BigEndian(message.Slice(pos)) & 0x3FFF;
                    continue;
                }

                if ((length & 0xC0) != 0 || pos + 1 + length > message.Length) return false;
                if (b.Length > 0) b.Append('.');
                b.Append(Encoding.ASCII.GetString(message.Slice(pos + 1, length)));
                pos += 1 + length;
            }

            name = b.ToString();
            return true;
        }
    }
}
