using System.Numerics;
using Teleop.Core.Pipeline;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Pipeline;

public class RobotStateFrameCodecTests
{
    private static RobotStateFrame SampleFrame(uint sequence = 7) => new RobotStateFrame(
        sequence: sequence,
        robotRecvTicks: 123_456,
        downlinkSendTicks: -789,
        pose: new Pose(new Vector3(1.5f, -2.25f, 3.0f), new Quaternion(0.1f, 0.2f, 0.3f, 0.9f)));

    [Fact]
    public void RoundTrip_ExactValues()
    {
        var codec = new RobotStateFrameCodec();
        Span<byte> buffer = new byte[RobotStateFrameCodec.EncodedSize];

        bool encoded = codec.TryEncode(SampleFrame(), buffer, out int bytesWritten);
        Assert.True(encoded);
        Assert.Equal(RobotStateFrameCodec.EncodedSize, bytesWritten);

        bool decoded = codec.TryDecode(buffer.Slice(0, bytesWritten), out RobotStateFrame frame);

        Assert.True(decoded);
        Assert.Equal(7u, frame.Sequence);
        Assert.Equal(123_456, frame.RobotRecvTicks);
        Assert.Equal(-789, frame.DownlinkSendTicks);
        Assert.Equal(1.5f, frame.Pose.Position.X);
        Assert.Equal(-2.25f, frame.Pose.Position.Y);
        Assert.Equal(3.0f, frame.Pose.Position.Z);
        Assert.Equal(0.1f, frame.Pose.Rotation.X);
        Assert.Equal(0.9f, frame.Pose.Rotation.W);
    }

    [Fact]
    public void RoundTrip_SequenceNearUIntMaxValue()
    {
        var codec = new RobotStateFrameCodec();
        Span<byte> buffer = new byte[RobotStateFrameCodec.EncodedSize];
        var original = SampleFrame(sequence: uint.MaxValue - 1);

        codec.TryEncode(original, buffer, out int n);
        codec.TryDecode(buffer.Slice(0, n), out RobotStateFrame decoded);

        Assert.Equal(uint.MaxValue - 1, decoded.Sequence);
    }

    [Fact]
    public void TryEncode_TooSmallDestination_ReportsRequiredLengthAndTouchesNothing()
    {
        var codec = new RobotStateFrameCodec();
        byte[] tiny = new byte[10];
        byte[] sentinel = (byte[])tiny.Clone();

        bool encoded = codec.TryEncode(SampleFrame(), tiny, out int required);

        Assert.False(encoded);
        Assert.Equal(RobotStateFrameCodec.EncodedSize, required);
        Assert.Equal(sentinel, tiny);
    }

    [Fact]
    public void TryEncode_ZeroLengthDestination_Fails()
    {
        var codec = new RobotStateFrameCodec();
        bool encoded = codec.TryEncode(SampleFrame(), Span<byte>.Empty, out int required);

        Assert.False(encoded);
        Assert.Equal(RobotStateFrameCodec.EncodedSize, required);
    }

    [Fact]
    public void TryDecode_TooShortSource_ReturnsFalseWithDefault()
    {
        var codec = new RobotStateFrameCodec();
        byte[] tooShort = new byte[RobotStateFrameCodec.EncodedSize - 1];

        bool decoded = codec.TryDecode(tooShort, out RobotStateFrame frame);

        Assert.False(decoded);
        Assert.Equal(default, frame);
    }

    [Fact]
    public void TryDecode_CorruptVersionByte_ReturnsFalseWithDefault()
    {
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
        codec.TryEncode(SampleFrame(), buffer, out _);
        buffer[0] = 99;

        bool decoded = codec.TryDecode(buffer, out RobotStateFrame frame);

        Assert.False(decoded);
        Assert.Equal(default, frame);
    }

    [Fact]
    public void Encode_IsDeterministic()
    {
        var codec = new RobotStateFrameCodec();
        byte[] a = new byte[RobotStateFrameCodec.EncodedSize];
        byte[] b = new byte[RobotStateFrameCodec.EncodedSize];

        codec.TryEncode(SampleFrame(), a, out _);
        codec.TryEncode(SampleFrame(), b, out _);

        Assert.Equal(a, b);
    }

    [Fact]
    public void Reset_IsSafeAndInert()
    {
        var codec = new RobotStateFrameCodec();
        byte[] before = new byte[RobotStateFrameCodec.EncodedSize];
        codec.TryEncode(SampleFrame(), before, out _);

        codec.Reset();

        byte[] after = new byte[RobotStateFrameCodec.EncodedSize];
        codec.TryEncode(SampleFrame(), after, out _);
        Assert.Equal(before, after);
    }

    [Fact]
    public void MaxEncodedBytes_MatchesActualBytesWritten()
    {
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];

        codec.TryEncode(SampleFrame(), buffer, out int bytesWritten);

        Assert.Equal(codec.MaxEncodedBytes, bytesWritten);
        Assert.Equal(49, codec.MaxEncodedBytes);
    }

    [Fact]
    public void TryEncode_Allocates_Zero_Bytes()
    {
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
        var frame = SampleFrame();
        AllocationAssert.Zero(() => codec.TryEncode(frame, buffer, out _));
    }

    [Fact]
    public void TryDecode_Allocates_Zero_Bytes()
    {
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
        codec.TryEncode(SampleFrame(), buffer, out _);
        AllocationAssert.Zero(() => codec.TryDecode(buffer, out _));
    }
}
