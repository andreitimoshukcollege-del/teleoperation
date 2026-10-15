using Teleop.Core.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    public class JointStateCodecTests
    {
        private static readonly JointSample[] Joints =
        {
            new JointSample(1, 0.25f, true), new JointSample(2, -1.1f, true), new JointSample(3, 0.9f, true),
            new JointSample(4, -0.2f, false), new JointSample(10, 0.75f, false), new JointSample(5, 0.3f, true),
        };

        [Fact]
        public void Frame_RoundTripsExactly()
        {
            var buffer = new byte[128];
            Assert.True(JointStateCodec.TryEncode(7, 1000, 990, 500, 800, 1_000_000_000, Joints, buffer, out int written));
            Assert.Equal(JointStateCodec.EncodedSize(6), written);
            Assert.True(written <= 128);

            var decoded = new JointSample[JointStateCodec.MaxJointsPerMessage];
            Assert.True(JointStateCodec.TryDecode(buffer.AsSpan(0, written), out uint seq, out long send, out long sensed,
                out long echoOp, out long echoRecv, out long tps, decoded, out int count));
            Assert.Equal((7u, 1000L, 990L, 500L, 800L, 1_000_000_000L, 6), (seq, send, sensed, echoOp, echoRecv, tps, count));
            for (int i = 0; i < 6; i++)
            {
                Assert.Equal(Joints[i].MotorId, decoded[i].MotorId);
                Assert.Equal(Joints[i].Value, decoded[i].Value);
                Assert.Equal(Joints[i].Sensed, decoded[i].Sensed);
            }
        }

        [Fact]
        public void Subscribe_RoundTrips_AndIsNotAJointCommand()
        {
            var buffer = new byte[JointStateCodec.SubscribeSize];
            Assert.True(JointStateCodec.TryEncodeSubscribe(123456, 10_000_000, buffer, out int written));
            Assert.True(JointStateCodec.IsSubscribe(buffer));
            Assert.True(JointStateCodec.TryDecodeSubscribe(buffer.AsSpan(0, written), out long ticks, out long tps));
            Assert.Equal((123456L, 10_000_000L), (ticks, tps));

            Assert.False(JointCommandCodec.TryDecode(buffer, out _, out _, new JointTarget[12], out _));
        }

        [Fact]
        public void AJointCommand_IsNotASubscribe_OrAFrame()
        {
            var buffer = new byte[128];
            Assert.True(JointCommandCodec.TryEncode(1, 2, new[] { new JointTarget(1, 0f, 0f) }, buffer, out int written));
            Assert.False(JointStateCodec.IsSubscribe(buffer.AsSpan(0, written)));
            Assert.False(JointStateCodec.TryDecode(buffer.AsSpan(0, written), out _, out _, out _, out _, out _, out _, new JointSample[12], out _));
        }

        [Fact]
        public void TruncatedOrOversizedFrames_AreRejected()
        {
            var buffer = new byte[128];
            JointStateCodec.TryEncode(1, 1, 1, 1, 1, 1000, Joints, buffer, out int written);
            Assert.False(JointStateCodec.TryDecode(buffer.AsSpan(0, written - 1), out _, out _, out _, out _, out _, out _, new JointSample[12], out _));
            Assert.False(JointStateCodec.TryDecode(buffer.AsSpan(0, written), out _, out _, out _, out _, out _, out _, new JointSample[2], out _));
            Assert.False(JointStateCodec.TryEncode(1, 1, 1, 1, 1, 1000, new JointSample[JointStateCodec.MaxJointsPerMessage + 1], buffer, out _));
        }
    }
}
