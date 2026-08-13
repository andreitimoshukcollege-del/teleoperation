using Teleop.JetRover.Wire;

namespace Teleop.JetRover.Tests.Wire
{
    public class JointCommandCodecTests
    {
        [Fact]
        public void TryEncode_ThenTryDecode_RoundTripsExactly()
        {
            var frame = new JointCommandFrame(
                sequence: 42,
                captureTicks: 123456789L,
                baseYaw: 0.7853982f,
                lowerPitch: -0.4903f,
                middlePitch: 1.857f,
                upperPitch: -1.2f,
                gripper: 0.5f);
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize];

            bool encoded = JointCommandCodec.TryEncode(frame, buffer, out int bytesWritten);
            bool decoded = JointCommandCodec.TryDecode(buffer, out JointCommandFrame result);

            Assert.True(encoded);
            Assert.Equal(JointCommandCodec.EncodedSize, bytesWritten);
            Assert.True(decoded);
            Assert.Equal(frame.Sequence, result.Sequence);
            Assert.Equal(frame.CaptureTicks, result.CaptureTicks);
            Assert.Equal(frame.BaseYaw, result.BaseYaw);
            Assert.Equal(frame.LowerPitch, result.LowerPitch);
            Assert.Equal(frame.MiddlePitch, result.MiddlePitch);
            Assert.Equal(frame.UpperPitch, result.UpperPitch);
            Assert.Equal(frame.Gripper, result.Gripper);
        }

        [Fact]
        public void TryEncode_DestinationTooShort_ReturnsFalseAndReportsRequiredSize()
        {
            Span<byte> tooShort = stackalloc byte[JointCommandCodec.EncodedSize - 1];

            bool encoded = JointCommandCodec.TryEncode(default, tooShort, out int bytesWritten);

            Assert.False(encoded);
            Assert.Equal(JointCommandCodec.EncodedSize, bytesWritten);
        }

        [Fact]
        public void TryDecode_SourceTooShort_ReturnsFalse()
        {
            Span<byte> tooShort = stackalloc byte[JointCommandCodec.EncodedSize - 1];

            bool decoded = JointCommandCodec.TryDecode(tooShort, out JointCommandFrame frame);

            Assert.False(decoded);
            Assert.Equal(default, frame);
        }

        [Fact]
        public void TryDecode_WrongVersionByte_ReturnsFalse()
        {
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize];
            JointCommandCodec.TryEncode(default, buffer, out _);
            buffer[0] = (byte)(JointCommandCodec.Version + 1);

            bool decoded = JointCommandCodec.TryDecode(buffer, out JointCommandFrame frame);

            Assert.False(decoded);
            Assert.Equal(default, frame);
        }
    }
}
