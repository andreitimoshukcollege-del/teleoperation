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
        ticksPerSecond: 1_000_000_000,
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
        Assert.Equal(1_000_000_000, frame.TicksPerSecond);
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

    /// <summary>
    /// The wire version is part of the contract, not an implementation detail: ADR 0008 moved it
    /// 1 -> 2 when <c>TicksPerSecond</c> was appended, and both ends of this hop must be built
    /// from the same Core revision. Pinned so bumping the format is a deliberate, visible act.
    /// </summary>
    [Fact]
    public void TryEncode_WritesWireVersionTwo()
    {
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];

        codec.TryEncode(SampleFrame(), buffer, out _);

        Assert.Equal(2, RobotStateFrameCodec.Version);
        Assert.Equal(2, buffer[0]);
    }

    /// <summary>
    /// A version-1 payload -- the 49-byte pre-ADR-0008 frame, which had no <c>TicksPerSecond</c>
    /// and would otherwise decode as a plausible-looking frame with a garbage rate -- is rejected
    /// by the version byte, not silently misread. This is the same rejection path the corrupt-byte
    /// test above covers, asserted for the one wrong version that actually existed in the field.
    /// </summary>
    [Fact]
    public void TryDecode_VersionOnePayload_IsRejected()
    {
        var codec = new RobotStateFrameCodec();
        byte[] v1 = new byte[49];
        v1[0] = 1;

        bool decoded = codec.TryDecode(v1, out RobotStateFrame frame);

        Assert.False(decoded);
        Assert.Equal(default, frame);
    }

    /// <summary>
    /// And a v1 payload padded out to v2's length is still rejected on the version byte alone --
    /// proving the check is on the version, not merely on the length.
    /// </summary>
    [Fact]
    public void TryDecode_VersionOnePayloadPaddedToCurrentLength_IsRejected()
    {
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];
        codec.TryEncode(SampleFrame(), buffer, out _);
        buffer[0] = 1;

        Assert.False(codec.TryDecode(buffer, out RobotStateFrame frame));
        Assert.Equal(default, frame);
    }

    [Fact]
    public void RoundTrip_PreservesTicksPerSecond_ForEitherHostsRate()
    {
        var codec = new RobotStateFrameCodec();
        byte[] buffer = new byte[RobotStateFrameCodec.EncodedSize];

        foreach (long rate in new long[] { 10_000_000L, 1_000_000_000L, 1L, long.MaxValue })
        {
            var original = new RobotStateFrame(3, 10, 20, rate, Pose.Identity);
            codec.TryEncode(original, buffer, out int n);
            Assert.True(codec.TryDecode(buffer.AsSpan(0, n), out RobotStateFrame decoded));
            Assert.Equal(rate, decoded.TicksPerSecond);
        }
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
        // 1 version + 4 sequence + 8 robotRecv + 8 downlinkSend + 28 pose + 8 ticksPerSecond.
        // Wire v2 (docs/adr/0008-clocksync-cross-rate-normalization.md) grew this from 49.
        Assert.Equal(57, codec.MaxEncodedBytes);
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
