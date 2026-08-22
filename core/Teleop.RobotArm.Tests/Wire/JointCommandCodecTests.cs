using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    public class JointCommandCodecTests
    {
        private static readonly JointTarget[] ThreeTargets =
        {
            new JointTarget(motorId: 1, angle: 0.7853982f, speed: 0.1f),
            new JointTarget(motorId: 2, angle: -0.4903f, speed: 0f),
            new JointTarget(motorId: 3, angle: 1.857f, speed: 0.25f),
        };

        [Fact]
        public void TryEncode_ThenTryDecode_RoundTripsExactly()
        {
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize(ThreeTargets.Length)];

            bool encoded = JointCommandCodec.TryEncode(
                sequence: 42, captureTicks: 123456789L, ThreeTargets, buffer, out int bytesWritten);

            Span<JointTarget> decodeBuffer = stackalloc JointTarget[JointCommandCodec.MaxJointsPerMessage];
            bool decoded = JointCommandCodec.TryDecode(
                buffer, out uint sequence, out long captureTicks, decodeBuffer, out int targetCount);

            Assert.True(encoded);
            Assert.Equal(JointCommandCodec.EncodedSize(ThreeTargets.Length), bytesWritten);
            Assert.True(decoded);
            Assert.Equal(42u, sequence);
            Assert.Equal(123456789L, captureTicks);
            Assert.Equal(ThreeTargets.Length, targetCount);
            for (int i = 0; i < ThreeTargets.Length; i++)
            {
                Assert.Equal(ThreeTargets[i].MotorId, decodeBuffer[i].MotorId);
                Assert.Equal(ThreeTargets[i].Angle, decodeBuffer[i].Angle);
                Assert.Equal(ThreeTargets[i].Speed, decodeBuffer[i].Speed);
            }
        }

        [Fact]
        public void TryEncode_ZeroTargets_StillProducesAValidHeaderOnlyRecord()
        {
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize(0)];

            bool encoded = JointCommandCodec.TryEncode(1, 0L, Array.Empty<JointTarget>(), buffer, out int bytesWritten);
            Span<JointTarget> decodeBuffer = stackalloc JointTarget[JointCommandCodec.MaxJointsPerMessage];
            bool decoded = JointCommandCodec.TryDecode(buffer, out _, out _, decodeBuffer, out int targetCount);

            Assert.True(encoded);
            Assert.True(decoded);
            Assert.Equal(0, targetCount);
        }

        [Fact]
        public void TryEncode_MoreTargetsThanMaxJointsPerMessage_ReturnsFalse()
        {
            var tooMany = new JointTarget[JointCommandCodec.MaxJointsPerMessage + 1];
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize(tooMany.Length)];

            bool encoded = JointCommandCodec.TryEncode(1, 0L, tooMany, buffer, out int bytesWritten);

            Assert.False(encoded);
            Assert.Equal(0, bytesWritten);
        }

        [Fact]
        public void TryEncode_DestinationTooShort_ReturnsFalseAndReportsRequiredSize()
        {
            Span<byte> tooShort = stackalloc byte[JointCommandCodec.EncodedSize(ThreeTargets.Length) - 1];

            bool encoded = JointCommandCodec.TryEncode(1, 0L, ThreeTargets, tooShort, out int bytesWritten);

            Assert.False(encoded);
            Assert.Equal(JointCommandCodec.EncodedSize(ThreeTargets.Length), bytesWritten);
        }

        [Fact]
        public void TryDecode_SourceTooShortForItsOwnDeclaredCount_ReturnsFalse()
        {
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize(ThreeTargets.Length)];
            JointCommandCodec.TryEncode(1, 0L, ThreeTargets, buffer, out _);
            Span<byte> truncated = buffer.Slice(0, buffer.Length - 1);

            Span<JointTarget> decodeBuffer = stackalloc JointTarget[JointCommandCodec.MaxJointsPerMessage];
            bool decoded = JointCommandCodec.TryDecode(truncated, out _, out _, decodeBuffer, out int targetCount);

            Assert.False(decoded);
            Assert.Equal(0, targetCount);
        }

        [Fact]
        public void TryDecode_WrongVersionByte_ReturnsFalse()
        {
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize(ThreeTargets.Length)];
            JointCommandCodec.TryEncode(1, 0L, ThreeTargets, buffer, out _);
            buffer[0] = (byte)(JointCommandCodec.Version + 1);

            Span<JointTarget> decodeBuffer = stackalloc JointTarget[JointCommandCodec.MaxJointsPerMessage];
            bool decoded = JointCommandCodec.TryDecode(buffer, out _, out _, decodeBuffer, out int targetCount);

            Assert.False(decoded);
            Assert.Equal(0, targetCount);
        }

        [Fact]
        public void TryDecode_DeclaredCountExceedsCallersBuffer_ReturnsFalse()
        {
            Span<byte> buffer = stackalloc byte[JointCommandCodec.EncodedSize(ThreeTargets.Length)];
            JointCommandCodec.TryEncode(1, 0L, ThreeTargets, buffer, out _);

            Span<JointTarget> tooSmallBuffer = stackalloc JointTarget[ThreeTargets.Length - 1];
            bool decoded = JointCommandCodec.TryDecode(buffer, out _, out _, tooSmallBuffer, out int targetCount);

            Assert.False(decoded);
            Assert.Equal(0, targetCount);
        }
    }
}
