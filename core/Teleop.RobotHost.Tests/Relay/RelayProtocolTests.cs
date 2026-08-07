using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Relay
{
    public class RelayProtocolTests
    {
        [Fact]
        public void ArmCommand_RoundTripsThroughEncodeDecode()
        {
            var original = new LocalArmCommand(baseDirection: -3.25f);
            Span<byte> buffer = stackalloc byte[RelayProtocol.ArmCommandEncodedSize];

            int written = RelayProtocol.EncodeCommand(original, buffer);
            bool ok = RelayProtocol.TryDecodeCommand(buffer, out LocalArmCommand decoded);

            Assert.Equal(RelayProtocol.ArmCommandEncodedSize, written);
            Assert.True(ok);
            Assert.Equal(original.BaseDirection, decoded.BaseDirection);
        }

        [Fact]
        public void Feedback_RoundTripsThroughEncodeDecode()
        {
            var original = new LocalFeedback(baseDegreesValid: true, baseDegrees: -42);
            Span<byte> buffer = stackalloc byte[RelayProtocol.FeedbackEncodedSize];

            int written = RelayProtocol.EncodeFeedback(original, buffer);
            bool ok = RelayProtocol.TryDecodeFeedback(buffer, out LocalFeedback decoded);

            Assert.Equal(RelayProtocol.FeedbackEncodedSize, written);
            Assert.True(ok);
            Assert.Equal(original.BaseDegreesValid, decoded.BaseDegreesValid);
            Assert.Equal(original.BaseDegrees, decoded.BaseDegrees);
        }

        [Fact]
        public void TryDecodeCommand_RejectsWrongVersion()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.ArmCommandEncodedSize];
            RelayProtocol.EncodeCommand(new LocalArmCommand(1f), buffer);
            buffer[0] = RelayProtocol.Version + 1;

            bool ok = RelayProtocol.TryDecodeCommand(buffer, out _);

            Assert.False(ok);
        }

        [Fact]
        public void TryDecodeCommand_RejectsTooShortBuffer()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.ArmCommandEncodedSize - 1];

            bool ok = RelayProtocol.TryDecodeCommand(buffer, out _);

            Assert.False(ok);
        }

        [Fact]
        public void TryDecodeFeedback_RejectsWrongVersion()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.FeedbackEncodedSize];
            RelayProtocol.EncodeFeedback(new LocalFeedback(true, 1), buffer);
            buffer[0] = RelayProtocol.Version + 1;

            bool ok = RelayProtocol.TryDecodeFeedback(buffer, out _);

            Assert.False(ok);
        }
    }
}
