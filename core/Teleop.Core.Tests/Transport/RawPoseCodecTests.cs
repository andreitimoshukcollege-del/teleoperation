using System.Numerics;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Transport;

public class RawPoseCodecTests
{
    private const int WireSize = 73;
    private const byte SentinelByte = 0xAB;

    /// <summary>
    /// A frame with every field distinct and nonzero, so a swapped-offset bug in the layout
    /// cannot pass by writing the right value into the wrong slot.
    /// </summary>
    private static CommandFrame RepresentativeFrame() => new CommandFrame(
        sequence: 4242u,
        ackSequence: 99u,
        captureTicks: 1_234_567_890L,
        pose: new Pose(
            new Vector3(0.125f, -2.5f, 3.75f),
            new Quaternion(0.1f, 0.2f, 0.3f, 0.9273618f)),
        linearVelocity: new Vector3(1.5f, -0.25f, 0.0625f),
        angularVelocity: new Vector3(-0.5f, 0.75f, -1.25f),
        gripper: 0.6f);

    /// <summary>
    /// Compares two frames on their raw bits, not on <c>==</c>. Floats here are copied as IEEE-754
    /// bit patterns rather than reformatted, so the round trip is exact and any tolerance would
    /// hide a real bug; bit comparison additionally catches the -0.0/+0.0 confusion that <c>==</c>
    /// would call equal.
    /// </summary>
    private static void AssertFramesIdentical(in CommandFrame expected, in CommandFrame actual)
    {
        Assert.Equal(expected.Sequence, actual.Sequence);
        Assert.Equal(expected.AckSequence, actual.AckSequence);
        Assert.Equal(expected.CaptureTicks, actual.CaptureTicks);
        AssertBitIdentical(expected.Pose.Position.X, actual.Pose.Position.X);
        AssertBitIdentical(expected.Pose.Position.Y, actual.Pose.Position.Y);
        AssertBitIdentical(expected.Pose.Position.Z, actual.Pose.Position.Z);
        AssertBitIdentical(expected.Pose.Rotation.X, actual.Pose.Rotation.X);
        AssertBitIdentical(expected.Pose.Rotation.Y, actual.Pose.Rotation.Y);
        AssertBitIdentical(expected.Pose.Rotation.Z, actual.Pose.Rotation.Z);
        AssertBitIdentical(expected.Pose.Rotation.W, actual.Pose.Rotation.W);
        AssertBitIdentical(expected.LinearVelocity.X, actual.LinearVelocity.X);
        AssertBitIdentical(expected.LinearVelocity.Y, actual.LinearVelocity.Y);
        AssertBitIdentical(expected.LinearVelocity.Z, actual.LinearVelocity.Z);
        AssertBitIdentical(expected.AngularVelocity.X, actual.AngularVelocity.X);
        AssertBitIdentical(expected.AngularVelocity.Y, actual.AngularVelocity.Y);
        AssertBitIdentical(expected.AngularVelocity.Z, actual.AngularVelocity.Z);
        AssertBitIdentical(expected.Gripper, actual.Gripper);
    }

    private static void AssertBitIdentical(float expected, float actual) =>
        Assert.Equal(BitConverter.SingleToInt32Bits(expected), BitConverter.SingleToInt32Bits(actual));

    private static byte[] Encode(RawPoseCodec codec, in CommandFrame frame)
    {
        var buffer = new byte[codec.MaxEncodedBytes];
        Assert.True(codec.TryEncode(frame, buffer, out int bytesWritten));
        Assert.Equal(WireSize, bytesWritten);
        return buffer;
    }

    [Fact]
    public void MaxEncodedBytes_IsTheFixedWireSizeAndMatchesWhatEncodeReports()
    {
        var codec = new RawPoseCodec();

        Assert.Equal(WireSize, codec.MaxEncodedBytes);
        Assert.Equal(WireSize, RawPoseCodec.EncodedSize);

        // Exact, not an upper bound: the reported length must equal MaxEncodedBytes for every
        // frame, because this codec has no variable-length case.
        var buffer = new byte[codec.MaxEncodedBytes];
        Assert.True(codec.TryEncode(RepresentativeFrame(), buffer, out int bytesWritten));
        Assert.Equal(codec.MaxEncodedBytes, bytesWritten);

        var zeroFrame = new CommandFrame(0u, 0u, 0L, Pose.Identity, Vector3.Zero, Vector3.Zero, 0f);
        Assert.True(codec.TryEncode(zeroFrame, buffer, out int zeroBytesWritten));
        Assert.Equal(codec.MaxEncodedBytes, zeroBytesWritten);
    }

    [Fact]
    public void RoundTrip_PreservesEveryFieldExactly()
    {
        var codec = new RawPoseCodec();
        CommandFrame original = RepresentativeFrame();

        byte[] wire = Encode(codec, original);

        Assert.True(codec.TryDecode(wire, out CommandFrame decoded));
        AssertFramesIdentical(original, decoded);
    }

    [Fact]
    public void RoundTrip_SurvivesSequenceNumbersNearWrap()
    {
        // Sequence wraps at uint.MaxValue; a signed-int mistake anywhere in the codec shows up
        // here as a negative or truncated value rather than at ordinary magnitudes.
        var codec = new RawPoseCodec();
        var original = new CommandFrame(
            sequence: uint.MaxValue - 1u,
            ackSequence: uint.MaxValue,
            captureTicks: 0L,
            pose: Pose.Identity,
            linearVelocity: Vector3.Zero,
            angularVelocity: Vector3.Zero,
            gripper: 1f);

        Assert.True(codec.TryDecode(Encode(codec, original), out CommandFrame decoded));
        AssertFramesIdentical(original, decoded);
        Assert.Equal(uint.MaxValue - 1u, decoded.Sequence);
        Assert.Equal(uint.MaxValue, decoded.AckSequence);
    }

    [Fact]
    public void RoundTrip_SurvivesNegativeCaptureTicks()
    {
        // CaptureTicks is on the sender's ITimeAuthority timebase and clock-sync offsets can put a
        // stamp before that timebase's origin, so negative is a legitimate value, not garbage.
        var codec = new RawPoseCodec();
        var original = new CommandFrame(
            sequence: 7u,
            ackSequence: 6u,
            captureTicks: -9_876_543_210L,
            pose: Pose.Identity,
            linearVelocity: Vector3.Zero,
            angularVelocity: Vector3.Zero,
            gripper: 0f);

        Assert.True(codec.TryDecode(Encode(codec, original), out CommandFrame decoded));
        AssertFramesIdentical(original, decoded);
        Assert.Equal(-9_876_543_210L, decoded.CaptureTicks);

        Assert.True(codec.TryDecode(Encode(codec, WithCaptureTicks(long.MinValue)), out CommandFrame extreme));
        Assert.Equal(long.MinValue, extreme.CaptureTicks);
    }

    private static CommandFrame WithCaptureTicks(long captureTicks) => new CommandFrame(
        1u, 1u, captureTicks, Pose.Identity, Vector3.Zero, Vector3.Zero, 0f);

    [Fact]
    public void RoundTrip_SurvivesNonIdentityRotation()
    {
        var codec = new RawPoseCodec();
        Quaternion rotation = Quaternion.Normalize(
            Quaternion.CreateFromYawPitchRoll(0.7f, -1.1f, 2.4f));
        var original = new CommandFrame(
            sequence: 12u,
            ackSequence: 11u,
            captureTicks: 500L,
            pose: new Pose(new Vector3(-1f, 2f, -3f), rotation),
            linearVelocity: new Vector3(0.01f, 0.02f, 0.03f),
            angularVelocity: new Vector3(-0.04f, 0.05f, -0.06f),
            gripper: 0.5f);

        Assert.True(codec.TryDecode(Encode(codec, original), out CommandFrame decoded));
        AssertFramesIdentical(original, decoded);

        // A quaternion the wire mangled would stop being unit-length; assert the property that
        // actually matters downstream, in addition to the bit equality above.
        Assert.Equal(1.0, decoded.Pose.Rotation.Length(), precision: 5);
    }

    [Fact]
    public void RoundTrip_SurvivesFloatExtremesBitExactly()
    {
        // Negative zero, subnormals and the representable extremes are exactly the values a
        // reformat-based codec would quietly lose. Raw IEEE-754 bits must survive untouched.
        var codec = new RawPoseCodec();
        var original = new CommandFrame(
            sequence: 3u,
            ackSequence: 2u,
            captureTicks: long.MaxValue,
            pose: new Pose(
                new Vector3(-0.0f, float.Epsilon, -float.MaxValue),
                new Quaternion(float.MinValue, float.MaxValue, -float.Epsilon, 0.0f)),
            linearVelocity: new Vector3(float.PositiveInfinity, float.NegativeInfinity, -0.0f),
            angularVelocity: new Vector3(1e-30f, -1e30f, 0.0f),
            gripper: -0.0f);

        Assert.True(codec.TryDecode(Encode(codec, original), out CommandFrame decoded));
        AssertFramesIdentical(original, decoded);
    }

    [Fact]
    public void TryEncode_DestinationTooSmall_ReportsRequiredLengthAndTouchesNothing()
    {
        var codec = new RawPoseCodec();
        CommandFrame frame = RepresentativeFrame();

        foreach (int shortLength in new[] { 0, 1, WireSize - 1 })
        {
            var buffer = new byte[WireSize];
            buffer.AsSpan().Fill(SentinelByte);

            Assert.False(codec.TryEncode(frame, buffer.AsSpan(0, shortLength), out int bytesWritten));
            Assert.Equal(WireSize, bytesWritten);

            // A partial record is indistinguishable from a corrupt one at the far end, so a
            // failed encode must leave the destination completely alone.
            Assert.All(buffer, b => Assert.Equal(SentinelByte, b));
        }
    }

    [Fact]
    public void TryEncode_AfterAFailedAttempt_StillEncodesCorrectly()
    {
        // This codec is stateless, so there is nothing a failed encode could corrupt -- exercised
        // anyway because that is the contract ICommandCodec.TryEncode states for every codec, and
        // a future delta baseline added here must not silently start consuming a slot on failure.
        var codec = new RawPoseCodec();
        CommandFrame frame = RepresentativeFrame();

        var tooSmall = new byte[WireSize - 1];
        Assert.False(codec.TryEncode(frame, tooSmall, out int required));
        Assert.Equal(WireSize, required);

        byte[] afterFailure = Encode(codec, frame);
        byte[] fromFreshCodec = Encode(new RawPoseCodec(), frame);
        Assert.Equal(fromFreshCodec, afterFailure);

        Assert.True(codec.TryDecode(afterFailure, out CommandFrame decoded));
        AssertFramesIdentical(frame, decoded);
    }

    [Fact]
    public void TryDecode_SourceTooShort_ReturnsFalseWithDefaultFrame()
    {
        var codec = new RawPoseCodec();
        byte[] valid = Encode(codec, RepresentativeFrame());

        foreach (int truncatedLength in new[] { 0, 1, 17, WireSize - 1 })
        {
            Assert.False(codec.TryDecode(valid.AsSpan(0, truncatedLength), out CommandFrame frame));
            AssertFramesIdentical(default, frame);
        }
    }

    [Fact]
    public void TryDecode_UnsupportedVersionByte_ReturnsFalseWithDefaultFrame()
    {
        var codec = new RawPoseCodec();
        byte[] wire = Encode(codec, RepresentativeFrame());
        Assert.Equal(RawPoseCodec.Version, wire[0]);

        // Well-formed in every respect except the version. Reject rather than reinterpret: the 72
        // bytes that follow mean whatever the *other* version says they mean.
        foreach (byte badVersion in new byte[] { 0, 99, 255 })
        {
            wire[0] = badVersion;
            Assert.False(codec.TryDecode(wire, out CommandFrame frame));
            AssertFramesIdentical(default, frame);
        }

        wire[0] = RawPoseCodec.Version;
        Assert.True(codec.TryDecode(wire, out CommandFrame recovered));
        AssertFramesIdentical(RepresentativeFrame(), recovered);
    }

    [Fact]
    public void TryDecode_IgnoresBytesBeyondTheRecord()
    {
        // A caller may hand over a whole receive buffer; trailing bytes are not the codec's.
        var codec = new RawPoseCodec();
        CommandFrame frame = RepresentativeFrame();
        var oversized = new byte[WireSize * 2];
        oversized.AsSpan().Fill(SentinelByte);
        Assert.True(codec.TryEncode(frame, oversized, out _));

        Assert.True(codec.TryDecode(oversized, out CommandFrame decoded));
        AssertFramesIdentical(frame, decoded);
    }

    [Fact]
    public void TryEncode_IsDeterministic()
    {
        var codec = new RawPoseCodec();
        CommandFrame frame = RepresentativeFrame();

        byte[] first = Encode(codec, frame);
        byte[] second = Encode(codec, frame);
        byte[] fromSecondInstance = Encode(new RawPoseCodec(), frame);

        Assert.Equal(first, second);
        Assert.Equal(first, fromSecondInstance);
    }

    [Fact]
    public void Reset_IsSafeAndLeavesEncodeAndDecodeUnchanged()
    {
        var codec = new RawPoseCodec();
        CommandFrame frame = RepresentativeFrame();

        byte[] before = Encode(codec, frame);
        Assert.True(codec.TryDecode(before, out _));

        codec.Reset();
        codec.Reset(); // idempotent, and safe to call on an untouched codec too.

        Assert.Equal(WireSize, codec.MaxEncodedBytes);

        byte[] after = Encode(codec, frame);
        Assert.Equal(before, after);

        Assert.True(codec.TryDecode(after, out CommandFrame decoded));
        AssertFramesIdentical(frame, decoded);

        // And is indistinguishable from a freshly constructed instance.
        Assert.Equal(Encode(new RawPoseCodec(), frame), after);
    }

    [Fact]
    public void Reset_OnAFreshCodec_ProducesTheSameBytesAsNoResetAtAll()
    {
        CommandFrame frame = RepresentativeFrame();

        var reset = new RawPoseCodec();
        reset.Reset();

        Assert.Equal(Encode(new RawPoseCodec(), frame), Encode(reset, frame));
    }

    [Fact]
    public void TryEncode_Allocates_Zero_Bytes()
    {
        var codec = new RawPoseCodec();
        CommandFrame frame = RepresentativeFrame();
        var buffer = new byte[codec.MaxEncodedBytes];

        AllocationAssert.Zero(() => codec.TryEncode(frame, buffer, out _));
    }

    [Fact]
    public void TryEncode_WhenDestinationTooSmall_Allocates_Zero_Bytes()
    {
        var codec = new RawPoseCodec();
        CommandFrame frame = RepresentativeFrame();
        var tooSmall = new byte[WireSize - 1];

        AllocationAssert.Zero(() => codec.TryEncode(frame, tooSmall, out _));
    }

    [Fact]
    public void TryDecode_Allocates_Zero_Bytes()
    {
        var codec = new RawPoseCodec();
        byte[] wire = Encode(codec, RepresentativeFrame());

        AllocationAssert.Zero(() => codec.TryDecode(wire, out _));
    }

    [Fact]
    public void TryDecode_OnRejectedPayloads_Allocates_Zero_Bytes()
    {
        var codec = new RawPoseCodec();
        byte[] wire = Encode(codec, RepresentativeFrame());
        var truncated = new byte[WireSize - 1];
        var badVersion = (byte[])wire.Clone();
        badVersion[0] = 99;

        AllocationAssert.Zero(() =>
        {
            codec.TryDecode(truncated, out _);
            codec.TryDecode(badVersion, out _);
        });
    }
}
