using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Relay
{
    public class RelayProtocolTests
    {
        private static readonly JointTarget[] ThreeTargets =
        {
            new JointTarget(motorId: 1, angle: -3.25f, speed: 300f),
            new JointTarget(motorId: 2, angle: 1.5f, speed: 300f),
            new JointTarget(motorId: 3, angle: -0.75f, speed: 250f),
        };

        [Fact]
        public void Command_RoundTripsThroughEncodeDecode()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];

            int written = RelayProtocol.EncodeCommand(ThreeTargets, buffer);
            Span<JointTarget> decodeBuffer = stackalloc JointTarget[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeCommand(buffer, decodeBuffer, out int targetCount);

            Assert.Equal(RelayProtocol.CommandEncodedSize(ThreeTargets.Length), written);
            Assert.True(ok);
            Assert.Equal(ThreeTargets.Length, targetCount);
            for (int i = 0; i < ThreeTargets.Length; i++)
            {
                Assert.Equal(ThreeTargets[i].MotorId, decodeBuffer[i].MotorId);
                Assert.Equal(ThreeTargets[i].Angle, decodeBuffer[i].Angle);
                Assert.Equal(ThreeTargets[i].Speed, decodeBuffer[i].Speed);
            }
        }

        [Fact]
        public void Feedback_RoundTripsThroughEncodeDecode()
        {
            var original = new[]
            {
                new JointFeedbackEntry(motorId: 1, valid: true, pulse: -42f),
                new JointFeedbackEntry(motorId: 2, valid: false, pulse: 0f),
                new JointFeedbackEntry(motorId: 3, valid: true, pulse: 88f),
                new JointFeedbackEntry(motorId: 4, valid: true, pulse: -5f),
            };
            Span<byte> buffer = stackalloc byte[RelayProtocol.FeedbackEncodedSize(original.Length)];

            int written = RelayProtocol.EncodeFeedback(original, buffer);
            Span<JointFeedbackEntry> decodeBuffer = stackalloc JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeFeedback(buffer, decodeBuffer, out int entryCount);

            Assert.Equal(RelayProtocol.FeedbackEncodedSize(original.Length), written);
            Assert.True(ok);
            Assert.Equal(original.Length, entryCount);
            for (int i = 0; i < original.Length; i++)
            {
                Assert.Equal(original[i].MotorId, decodeBuffer[i].MotorId);
                Assert.Equal(original[i].Valid, decodeBuffer[i].Valid);
                Assert.Equal(original[i].Pulse, decodeBuffer[i].Pulse);
            }
        }

        [Fact]
        public void TryDecodeCommand_RejectsWrongVersion()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];
            RelayProtocol.EncodeCommand(ThreeTargets, buffer);
            buffer[0] = RelayProtocol.Version + 1;

            Span<JointTarget> decodeBuffer = stackalloc JointTarget[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeCommand(buffer, decodeBuffer, out int targetCount);

            Assert.False(ok);
            Assert.Equal(0, targetCount);
        }

        [Fact]
        public void TryDecodeCommand_RejectsTooShortBuffer()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];
            RelayProtocol.EncodeCommand(ThreeTargets, buffer);
            Span<byte> truncated = buffer.Slice(0, buffer.Length - 1);

            Span<JointTarget> decodeBuffer = stackalloc JointTarget[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeCommand(truncated, decodeBuffer, out int targetCount);

            Assert.False(ok);
            Assert.Equal(0, targetCount);
        }

        [Fact]
        public void TryDecodeFeedback_RejectsWrongVersion()
        {
            var entries = new[]
            {
                new JointFeedbackEntry(1, true, 1f), new JointFeedbackEntry(2, true, 1f),
                new JointFeedbackEntry(3, true, 1f), new JointFeedbackEntry(4, true, 1f),
            };
            Span<byte> buffer = stackalloc byte[RelayProtocol.FeedbackEncodedSize(entries.Length)];
            RelayProtocol.EncodeFeedback(entries, buffer);
            buffer[0] = RelayProtocol.Version + 1;

            Span<JointFeedbackEntry> decodeBuffer = stackalloc JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage];
            bool ok = RelayProtocol.TryDecodeFeedback(buffer, decodeBuffer, out int entryCount);

            Assert.False(ok);
            Assert.Equal(0, entryCount);
        }

        [Fact]
        public void TryDecodeCommand_DeclaredCountExceedsCallersBuffer_ReturnsFalse()
        {
            Span<byte> buffer = stackalloc byte[RelayProtocol.CommandEncodedSize(ThreeTargets.Length)];
            RelayProtocol.EncodeCommand(ThreeTargets, buffer);

            Span<JointTarget> tooSmallBuffer = stackalloc JointTarget[ThreeTargets.Length - 1];
            bool ok = RelayProtocol.TryDecodeCommand(buffer, tooSmallBuffer, out int targetCount);

            Assert.False(ok);
            Assert.Equal(0, targetCount);
        }
    }
}
