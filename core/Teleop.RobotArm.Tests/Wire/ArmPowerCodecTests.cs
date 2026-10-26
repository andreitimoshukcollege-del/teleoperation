using System;
using Teleop.Core.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    public class ArmPowerCodecTests
    {
        [Fact]
        public void Request_RoundTripsExactly()
        {
            var buffer = new byte[ArmPowerCodec.RequestSize];
            Assert.True(ArmPowerCodec.TryEncodeRequest(ArmPowerRequest.Relax, 0xDEADBEEF, -123456789012, buffer, out int written));
            Assert.Equal(15, written);
            Assert.True(ArmPowerCodec.IsRequest(buffer));
            Assert.False(ArmPowerCodec.IsStatus(buffer));

            Assert.True(ArmPowerCodec.TryDecodeRequest(buffer, out ArmPowerRequest request, out uint seq, out long ticks));
            Assert.Equal((ArmPowerRequest.Relax, 0xDEADBEEFu, -123456789012L), (request, seq, ticks));
            Assert.True(ArmPowerCodec.IsKnown(request));
        }

        [Fact]
        public void Request_LayoutIsFixed()
        {
            var buffer = new byte[ArmPowerCodec.RequestSize];
            Assert.True(ArmPowerCodec.TryEncodeRequest(ArmPowerRequest.Hold, 1, 2, buffer, out _));
            Assert.Equal(new byte[] { 0x81, 1, 2, 1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0 }, buffer);
        }

        [Fact]
        public void Status_RoundTripsExactly()
        {
            var buffer = new byte[ArmPowerCodec.StatusSize];
            Assert.True(ArmPowerCodec.TryEncodeStatus(ArmPowerState.Parking, ArmPowerReason.None, 42, 777, 888, 12.5f, buffer, out int written));
            Assert.Equal(28, written);
            Assert.True(ArmPowerCodec.IsStatus(buffer));

            Assert.True(ArmPowerCodec.TryDecodeStatus(buffer, out ArmPowerState state, out ArmPowerReason reason, out uint seq,
                out long echo, out long send, out float remaining));
            Assert.Equal((ArmPowerState.Parking, ArmPowerReason.None, 42u, 777L, 888L, 12.5f), (state, reason, seq, echo, send, remaining));
        }

        [Fact]
        public void Status_CarriesARefusalAndAnUnknownDistance()
        {
            var buffer = new byte[ArmPowerCodec.StatusSize];
            Assert.True(ArmPowerCodec.TryEncodeStatus(ArmPowerState.Holding, ArmPowerReason.PathCollides, 3, 4, 5, float.NaN, buffer, out _));
            Assert.True(ArmPowerCodec.TryDecodeStatus(buffer, out ArmPowerState state, out ArmPowerReason reason, out _, out _, out _,
                out float remaining));
            Assert.Equal(ArmPowerState.Holding, state);
            Assert.Equal(ArmPowerReason.PathCollides, reason);
            Assert.True(float.IsNaN(remaining));
            Assert.NotEqual("", ArmPowerCodec.Describe(reason));
        }

        [Fact]
        public void ShortBuffers_AreRefused_WithoutWriting()
        {
            var shortBuffer = new byte[ArmPowerCodec.RequestSize - 1];
            Assert.False(ArmPowerCodec.TryEncodeRequest(ArmPowerRequest.Relax, 1, 1, shortBuffer, out _));
            Assert.All(shortBuffer, b => Assert.Equal(0, b));
            Assert.False(ArmPowerCodec.TryEncodeStatus(ArmPowerState.Relaxed, 0, 1, 1, 1, 0f, new byte[ArmPowerCodec.StatusSize - 1], out _));
        }

        [Fact]
        public void Malformed_IsRejected()
        {
            var request = new byte[ArmPowerCodec.RequestSize];
            ArmPowerCodec.TryEncodeRequest(ArmPowerRequest.Relax, 1, 1, request, out _);
            Assert.False(ArmPowerCodec.TryDecodeRequest(request.AsSpan(0, ArmPowerCodec.RequestSize - 1), out _, out _, out _));
            Assert.False(ArmPowerCodec.TryDecodeRequest(ReadOnlySpan<byte>.Empty, out _, out _, out _));

            byte[] wrongVersion = (byte[])request.Clone();
            wrongVersion[1] = ArmPowerCodec.Version + 1;
            Assert.False(ArmPowerCodec.TryDecodeRequest(wrongVersion, out _, out _, out _));

            byte[] wrongTag = (byte[])request.Clone();
            wrongTag[0] = ArmPowerCodec.StatusTag;
            Assert.False(ArmPowerCodec.TryDecodeRequest(wrongTag, out _, out _, out _));

            var status = new byte[ArmPowerCodec.StatusSize];
            ArmPowerCodec.TryEncodeStatus(ArmPowerState.Relaxed, 0, 1, 1, 1, 0f, status, out _);
            Assert.False(ArmPowerCodec.TryDecodeStatus(status.AsSpan(0, ArmPowerCodec.StatusSize - 1), out _, out _, out _, out _, out _, out _));
            byte[] badState = (byte[])status.Clone();
            badState[2] = 3;
            Assert.False(ArmPowerCodec.TryDecodeStatus(badState, out _, out _, out _, out _, out _, out _));
            byte[] statusVersion = (byte[])status.Clone();
            statusVersion[1] = 0;
            Assert.False(ArmPowerCodec.TryDecodeStatus(statusVersion, out _, out _, out _, out _, out _, out _));
        }

        [Fact]
        public void UnknownRequestByte_DecodesRaw_SoItCanBeAnswered()
        {
            var buffer = new byte[ArmPowerCodec.RequestSize];
            Assert.True(ArmPowerCodec.TryEncodeRequest((ArmPowerRequest)9, 5, 6, buffer, out _));
            Assert.True(ArmPowerCodec.TryDecodeRequest(buffer, out ArmPowerRequest request, out _, out _));
            Assert.False(ArmPowerCodec.IsKnown(request));
        }

        [Fact]
        public void TheJointPortsOtherDecoders_IgnoreBoth()
        {
            var request = new byte[ArmPowerCodec.RequestSize];
            ArmPowerCodec.TryEncodeRequest(ArmPowerRequest.Relax, 1, 1, request, out _);
            var status = new byte[ArmPowerCodec.StatusSize];
            ArmPowerCodec.TryEncodeStatus(ArmPowerState.Relaxed, 0, 1, 1, 1, 0f, status, out _);

            foreach (byte[] message in new[] { request, status })
            {
                Assert.False(JointStateCodec.IsSubscribe(message));
                Assert.False(JointCommandCodec.TryDecode(message, out _, out _, new JointTarget[JointCommandCodec.MaxJointsPerMessage], out _));
                Assert.False(JointStateCodec.TryDecode(message, out _, out _, out _, out _, out _, out _,
                    new JointSample[JointStateCodec.MaxJointsPerMessage], out _));
            }

            // And the reverse: neither of these decoders takes the port's other messages.
            var subscribe = new byte[JointStateCodec.SubscribeSize];
            JointStateCodec.TryEncodeSubscribe(1, 1, subscribe, out _);
            var command = new byte[JointCommandCodec.EncodedSize(1)];
            JointCommandCodec.TryEncode(1, 1, new[] { new JointTarget(1, 0f, 0f) }, command, out _);
            var frame = new byte[128];
            JointStateCodec.TryEncode(1, 1, 1, 1, 1, 1, new[] { new JointSample(1, 0f, true) }, frame, out int frameBytes);
            foreach (byte[] other in new[] { subscribe, command, frame.AsSpan(0, frameBytes).ToArray() })
            {
                Assert.False(ArmPowerCodec.TryDecodeRequest(other, out _, out _, out _));
                Assert.False(ArmPowerCodec.TryDecodeStatus(other, out _, out _, out _, out _, out _, out _));
            }
        }
    }
}
