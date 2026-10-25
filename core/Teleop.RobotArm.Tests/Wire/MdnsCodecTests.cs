using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Net;
using System.Text;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    public class MdnsCodecTests
    {
        /// <summary>
        /// A response in the shape docs/adr/0015 measured from the robot: the question echoed, then one
        /// A record whose name is a compression pointer to it (0xC00C), class IN with the cache-flush
        /// bit, TTL 10 s.
        /// </summary>
        internal static byte[] Answer(ushort id, string name, IPAddress address, ushort flags = 0x8400, bool echoQuestion = true)
        {
            var b = new List<byte>();
            void U16(int v) { b.Add((byte)(v >> 8)); b.Add((byte)v); }
            U16(id);
            U16(flags);
            U16(echoQuestion ? 1 : 0);
            U16(1);
            U16(0);
            U16(0);
            if (echoQuestion)
            {
                foreach (string label in name.Split('.'))
                {
                    b.Add((byte)label.Length);
                    b.AddRange(Encoding.ASCII.GetBytes(label));
                }

                b.Add(0);
                U16(1);
                U16(1);
                b.Add(0xC0);
                b.Add(0x0C);
            }
            else
            {
                foreach (string label in name.Split('.'))
                {
                    b.Add((byte)label.Length);
                    b.AddRange(Encoding.ASCII.GetBytes(label));
                }

                b.Add(0);
            }

            U16(1);
            U16(0x8001);
            b.AddRange(new byte[] { 0, 0, 0, 10 });
            U16(4);
            b.AddRange(address.GetAddressBytes());
            return b.ToArray();
        }

        [Fact]
        public void TheQuery_IsOneQuestionForTheARecord()
        {
            var buffer = new byte[MdnsQueryCodec.MaxMessageBytes];
            Assert.True(MdnsQueryCodec.TryEncodeQuery("jetrover-sinrg.local", 0x1234, buffer, out int n));
            byte[] expected =
            {
                0x12, 0x34, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                14, (byte)'j', (byte)'e', (byte)'t', (byte)'r', (byte)'o', (byte)'v', (byte)'e', (byte)'r', (byte)'-',
                (byte)'s', (byte)'i', (byte)'n', (byte)'r', (byte)'g',
                5, (byte)'l', (byte)'o', (byte)'c', (byte)'a', (byte)'l', 0,
                0, 1, 0, 1,
            };
            Assert.Equal(expected, buffer.AsSpan(0, n).ToArray());
            Assert.False(MdnsQueryCodec.TryEncodeQuery("jetrover-sinrg.local", 0, buffer, out _));
        }

        [Fact]
        public void TheMeasuredShapeOfReply_Parses()
        {
            byte[] reply = Answer(0x1234, "jetrover-sinrg.local", IPAddress.Parse("10.188.62.69"));
            Assert.Equal(54, reply.Length); // ADR 0015's measured size for this name
            Assert.True(MdnsQueryCodec.TryParseAnswer(reply, 0x1234, "jetrover-sinrg.local", out IPAddress address, out uint ttl));
            Assert.Equal(IPAddress.Parse("10.188.62.69"), address);
            Assert.Equal(10u, ttl);
            Assert.True(MdnsQueryCodec.TryParseAnswer(Answer(7, "jetrover-sinrg.local", address, echoQuestion: false), 7, "JETROVER-SINRG.local.", out _, out _));
        }

        [Fact]
        public void TheWrongId_ANameForSomeoneElse_OrAQuery_AreRefused()
        {
            IPAddress a = IPAddress.Parse("10.188.62.69");
            Assert.False(MdnsQueryCodec.TryParseAnswer(Answer(0x1234, "jetrover-sinrg.local", a), 0x4321, "jetrover-sinrg.local", out _, out _));
            Assert.False(MdnsQueryCodec.TryParseAnswer(Answer(0x1234, "other-robot.local", a), 0x1234, "jetrover-sinrg.local", out _, out _));
            Assert.False(MdnsQueryCodec.TryParseAnswer(Answer(0x1234, "jetrover-sinrg.local", a, flags: 0), 0x1234, "jetrover-sinrg.local", out _, out _));
        }

        [Fact]
        public void TruncatedReplies_AndPointerLoops_AreRefused()
        {
            byte[] reply = Answer(0x1234, "jetrover-sinrg.local", IPAddress.Parse("10.188.62.69"));
            for (int length = 0; length < reply.Length; length++)
            {
                Assert.False(MdnsQueryCodec.TryParseAnswer(reply.AsSpan(0, length), 0x1234, "jetrover-sinrg.local", out _, out _), $"length {length}");
            }

            // The answer's name pointer aimed at itself.
            byte[] loop = (byte[])reply.Clone();
            int pointerAt = Array.LastIndexOf(loop, (byte)0xC0);
            BinaryPrimitives.WriteUInt16BigEndian(loop.AsSpan(pointerAt), (ushort)(0xC000 | pointerAt));
            Assert.False(MdnsQueryCodec.TryParseAnswer(loop, 0x1234, "jetrover-sinrg.local", out _, out _));

            // An answer count larger than the records present.
            byte[] overcount = (byte[])reply.Clone();
            overcount[7] = 3;
            Assert.True(MdnsQueryCodec.TryParseAnswer(overcount, 0x1234, "jetrover-sinrg.local", out _, out _)); // the first one is whole
            overcount[pointerAt + 3] = 5; // and now not an A record: the parser must run off the end, not past it
            Assert.False(MdnsQueryCodec.TryParseAnswer(overcount, 0x1234, "jetrover-sinrg.local", out _, out _));
        }
    }
}
