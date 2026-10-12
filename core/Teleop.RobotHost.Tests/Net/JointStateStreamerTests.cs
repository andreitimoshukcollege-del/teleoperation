using Teleop.Core.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Net;

namespace Teleop.RobotHost.Tests.Net
{
    public class JointStateStreamerTests
    {
        private const long Tps = 1000;

        private static int TwoJoints(JointSample[] buffer)
        {
            buffer[0] = new JointSample(1, 0.5f, true);
            buffer[1] = new JointSample(10, 0.2f, false);
            return 2;
        }

        private static byte[] Subscribe(long operatorTicks)
        {
            var buffer = new byte[JointStateCodec.SubscribeSize];
            JointStateCodec.TryEncodeSubscribe(operatorTicks, 10_000_000, buffer, out _);
            return buffer;
        }

        [Fact]
        public void SendsNothingUntilSubscribed()
        {
            var s = new JointStateStreamer(Tps);
            Assert.False(s.TryBuildFrame(100, TwoJoints, 90, new byte[128], out _));
        }

        [Fact]
        public void AfterASubscribe_SendsAtTheRate_EchoingTheKeepalive()
        {
            var s = new JointStateStreamer(Tps, rateHz: 50);
            Assert.True(s.TryHandleSubscribe(Subscribe(777), arrivalTicks: 100));

            var buffer = new byte[128];
            Assert.True(s.TryBuildFrame(105, TwoJoints, 103, buffer, out int written));
            Assert.False(s.TryBuildFrame(110, TwoJoints, 103, buffer, out _));   // 5 ms later: not due
            Assert.True(s.TryBuildFrame(125, TwoJoints, 120, buffer, out written)); // 20 ms later

            var joints = new JointSample[12];
            Assert.True(JointStateCodec.TryDecode(buffer.AsSpan(0, written), out uint seq, out long send, out long sensed,
                out long echoOp, out long echoRecv, out long tps, joints, out int count));
            Assert.Equal((1u, 125L, 120L, 777L, 100L, Tps, 2), (seq, send, sensed, echoOp, echoRecv, tps, count));
        }

        [Fact]
        public void StopsWhenTheKeepaliveGoesStale()
        {
            var s = new JointStateStreamer(Tps, subscriptionTimeoutSeconds: 3);
            s.TryHandleSubscribe(Subscribe(1), arrivalTicks: 0);
            Assert.True(s.IsSubscribed(3000));
            Assert.False(s.IsSubscribed(3001));
            Assert.False(s.TryBuildFrame(3001, TwoJoints, 0, new byte[128], out _));
        }

        [Fact]
        public void AJointCommand_IsNotTakenForAKeepalive()
        {
            var s = new JointStateStreamer(Tps);
            var buffer = new byte[128];
            JointCommandCodec.TryEncode(1, 2, new[] { new JointTarget(1, 0f, 0f) }, buffer, out int written);
            Assert.False(s.TryHandleSubscribe(buffer.AsSpan(0, written), 10));
            Assert.False(s.IsSubscribed(10));
        }
    }
}
