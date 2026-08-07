using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Relay
{
    public class RelayProtocolTests
    {
        [Fact]
        public void ArmCommand_RoundTripsThroughEncodeDecode()
        {
            var original = new LocalArmCommand(
                baseDirection: -3.25f, lowerDirection: 1.5f, middleDirection: -0.75f,
                upperDirection: 2.0f, gripperDegrees: 120f);
            Span<byte> buffer = stackalloc byte[RelayProtocol.ArmCommandEncodedSize];

            int written = RelayProtocol.EncodeCommand(original, buffer);
            bool ok = RelayProtocol.TryDecodeCommand(buffer, out LocalArmCommand decoded);

            Assert.Equal(RelayProtocol.ArmCommandEncodedSize, written);
            Assert.True(ok);
            Assert.Equal(original.BaseDirection, decoded.BaseDirection);
            Assert.Equal(original.LowerDirection, decoded.LowerDirection);
            Assert.Equal(original.MiddleDirection, decoded.MiddleDirection);
            Assert.Equal(original.UpperDirection, decoded.UpperDirection);
            Assert.Equal(original.GripperDegrees, decoded.GripperDegrees);
        }

        [Fact]
        public void Feedback_RoundTripsThroughEncodeDecode()
        {
            var original = new LocalFeedback(
                @base: new JointFeedback(true, -42),
                lower: new JointFeedback(false, 0),
                middle: new JointFeedback(true, 88),
                upper: new JointFeedback(true, -5));
            Span<byte> buffer = stackalloc byte[RelayProtocol.FeedbackEncodedSize];

            int written = RelayProtocol.EncodeFeedback(original, buffer);
            bool ok = RelayProtocol.TryDecodeFeedback(buffer, out LocalFeedback decoded);

            Assert.Equal(RelayProtocol.FeedbackEncodedSize, written);
            Assert.True(ok);
            Assert.Equal(original.Base.Valid, decoded.Base.Valid);
            Assert.Equal(original.Base.Degrees, decoded.Base.Degrees);
            Assert.Equal(original.Lower.Valid, decoded.Lower.Valid);
            Assert.Equal(original.Middle.Degrees, decoded.Middle.Degrees);
            Assert.Equal(original.Upper.Degrees, decoded.Upper.Degrees);
        }

        [Fact]
        public void TryDecodeCommand_RejectsWrongVersion()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.ArmCommandEncodedSize];
            RelayProtocol.EncodeCommand(new LocalArmCommand(1f, 0f, 0f, 0f, 90f), buffer);
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
            var feedback = new LocalFeedback(
                new JointFeedback(true, 1), new JointFeedback(true, 1),
                new JointFeedback(true, 1), new JointFeedback(true, 1));
            RelayProtocol.EncodeFeedback(feedback, buffer);
            buffer[0] = RelayProtocol.Version + 1;

            bool ok = RelayProtocol.TryDecodeFeedback(buffer, out _);

            Assert.False(ok);
        }
    }
}
