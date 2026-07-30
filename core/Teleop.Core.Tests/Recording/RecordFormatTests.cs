using System.Numerics;
using Teleop.Core.Recording;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Recording;

public class RecordFormatTests
{
    private static CommandFrame SampleFrame(uint sequence = 42) => new CommandFrame(
        sequence: sequence,
        ackSequence: 41,
        captureTicks: 123_456_789,
        pose: new Pose(new Vector3(1.5f, -2.25f, 3.0f), new Quaternion(0.1f, 0.2f, 0.3f, 0.9f)),
        linearVelocity: new Vector3(0.5f, 0f, -0.5f),
        angularVelocity: new Vector3(0f, 1.0f, 0f),
        gripper: 0.75f);

    [Fact]
    public void CommandFrame_RoundTrips_ExactValues()
    {
        var writer = new SessionWriter();
        Span<byte> buffer = new byte[RecordFormat.MaxCommandFrameLineBytes];

        bool wrote = writer.TryWriteCommandFrame(SampleFrame(), buffer, out int bytesWritten);
        Assert.True(wrote);

        var reader = new SessionReader();
        bool read = reader.TryReadCommandFrame(buffer.Slice(0, bytesWritten), out CommandFrame decoded);

        Assert.True(read);
        Assert.Equal(42u, decoded.Sequence);
        Assert.Equal(41u, decoded.AckSequence);
        Assert.Equal(123_456_789, decoded.CaptureTicks);
        Assert.Equal(1.5f, decoded.Pose.Position.X);
        Assert.Equal(-2.25f, decoded.Pose.Position.Y);
        Assert.Equal(3.0f, decoded.Pose.Position.Z);
        Assert.Equal(0.1f, decoded.Pose.Rotation.X);
        Assert.Equal(0.9f, decoded.Pose.Rotation.W);
        Assert.Equal(0.75f, decoded.Gripper);
    }

    [Fact]
    public void StampedPose_RoundTrips_ExactValues()
    {
        var writer = new SessionWriter();
        Span<byte> buffer = new byte[RecordFormat.MaxStampedPoseLineBytes];
        var sample = new Stamped<Pose>(999, new Pose(new Vector3(1, 2, 3), new Quaternion(0, 0, 0, 1)));

        bool wrote = writer.TryWriteStampedPose(sample, buffer, out int bytesWritten);
        Assert.True(wrote);

        var reader = new SessionReader();
        bool read = reader.TryReadStampedPose(buffer.Slice(0, bytesWritten), out Stamped<Pose> decoded);

        Assert.True(read);
        Assert.Equal(999, decoded.CaptureTicks);
        Assert.Equal(1f, decoded.Value.Position.X);
        Assert.Equal(1f, decoded.Value.Rotation.W);
    }

    [Fact]
    public void LatencyTrace_FullyPopulated_RoundTrips()
    {
        var trace = LatencyTrace.ForSequence(7)
            .WithCaptureTicks(100)
            .WithUplinkSendTicks(110)
            .WithRobotRecvTicks(250)
            .WithDownlinkSendTicks(260)
            .WithOperatorRecvTicks(400)
            .WithPlayoutTicks(410)
            .WithRenderTicks(420)
            .WithPhotonTicks(428)
            .WithClockSync(-500, 10);

        var writer = new SessionWriter();
        Span<byte> buffer = new byte[RecordFormat.MaxLatencyTraceLineBytes];
        bool wrote = writer.TryWriteLatencyTrace(trace, buffer, out int bytesWritten);
        Assert.True(wrote);

        var reader = new SessionReader();
        bool read = reader.TryReadLatencyTrace(buffer.Slice(0, bytesWritten), out LatencyTrace decoded);

        Assert.True(read);
        Assert.Equal(7u, decoded.Sequence);
        Assert.True(decoded.TryGetCaptureTicks(out long capture)); Assert.Equal(100, capture);
        Assert.True(decoded.TryGetUplinkSendTicks(out long uplinkSend)); Assert.Equal(110, uplinkSend);
        Assert.True(decoded.TryGetRobotRecvTicks(out long robotRecv)); Assert.Equal(250, robotRecv);
        Assert.True(decoded.TryGetDownlinkSendTicks(out long downlinkSend)); Assert.Equal(260, downlinkSend);
        Assert.True(decoded.TryGetOperatorRecvTicks(out long operatorRecv)); Assert.Equal(400, operatorRecv);
        Assert.True(decoded.TryGetPlayoutTicks(out long playout)); Assert.Equal(410, playout);
        Assert.True(decoded.TryGetRenderTicks(out long render)); Assert.Equal(420, render);
        Assert.True(decoded.TryGetPhotonTicks(out long photon)); Assert.Equal(428, photon);
        Assert.True(decoded.TryGetClockOffsetTicks(out long offset)); Assert.Equal(-500, offset);
        Assert.True(decoded.TryGetClockOffsetUncertaintyTicks(out long uncertainty)); Assert.Equal(10, uncertainty);
    }

    [Fact]
    public void LatencyTrace_AllUnset_RoundTrips_AsUnset()
    {
        var trace = LatencyTrace.ForSequence(3);

        var writer = new SessionWriter();
        Span<byte> buffer = new byte[RecordFormat.MaxLatencyTraceLineBytes];
        bool wrote = writer.TryWriteLatencyTrace(trace, buffer, out int bytesWritten);
        Assert.True(wrote);

        var reader = new SessionReader();
        bool read = reader.TryReadLatencyTrace(buffer.Slice(0, bytesWritten), out LatencyTrace decoded);

        Assert.True(read);
        Assert.Equal(3u, decoded.Sequence);
        Assert.False(decoded.TryGetCaptureTicks(out _));
        Assert.False(decoded.TryGetUplinkSendTicks(out _));
        Assert.False(decoded.TryGetRobotRecvTicks(out _));
        Assert.False(decoded.TryGetDownlinkSendTicks(out _));
        Assert.False(decoded.TryGetOperatorRecvTicks(out _));
        Assert.False(decoded.TryGetPlayoutTicks(out _));
        Assert.False(decoded.TryGetRenderTicks(out _));
        Assert.False(decoded.TryGetPhotonTicks(out _));
        Assert.False(decoded.TryGetClockOffsetTicks(out _));
        Assert.False(decoded.TryGetClockOffsetUncertaintyTicks(out _));
    }

    [Fact]
    public void LatencyTrace_MixedSetAndUnset_RoundTrips()
    {
        var trace = LatencyTrace.ForSequence(uint.MaxValue)
            .WithCaptureTicks(5)
            .WithRenderTicks(-999);

        var writer = new SessionWriter();
        Span<byte> buffer = new byte[RecordFormat.MaxLatencyTraceLineBytes];
        writer.TryWriteLatencyTrace(trace, buffer, out int bytesWritten);

        var reader = new SessionReader();
        bool read = reader.TryReadLatencyTrace(buffer.Slice(0, bytesWritten), out LatencyTrace decoded);

        Assert.True(read);
        Assert.Equal(uint.MaxValue, decoded.Sequence);
        Assert.True(decoded.TryGetCaptureTicks(out long capture)); Assert.Equal(5, capture);
        Assert.False(decoded.TryGetUplinkSendTicks(out _));
        Assert.True(decoded.TryGetRenderTicks(out long render)); Assert.Equal(-999, render);
        Assert.False(decoded.TryGetPhotonTicks(out _));
    }

    [Fact]
    public void Header_RoundTrips()
    {
        var writer = new SessionWriter();
        Span<byte> buffer = new byte[RecordFormat.MaxHeaderLineBytes];
        bool wrote = writer.TryWriteHeader(10_000_000, 0xDEADBEEFCAFEUL, buffer, out int bytesWritten);
        Assert.True(wrote);

        var reader = new SessionReader();
        var result = reader.TryReadHeader(buffer.Slice(0, bytesWritten), out long ticksPerSecond, out ulong seed);

        Assert.Equal(SessionOpenResult.Ok, result);
        Assert.Equal(10_000_000, ticksPerSecond);
        Assert.Equal(0xDEADBEEFCAFEUL, seed);
    }

    [Fact]
    public void Header_WrongTag_ReportsBadTag()
    {
        var reader = new SessionReader();
        byte[] bogus = System.Text.Encoding.ASCII.GetBytes("NOPE|1|2|3");

        var result = reader.TryReadHeader(bogus, out _, out _);

        Assert.Equal(SessionOpenResult.BadTag, result);
    }

    [Fact]
    public void Header_UnsupportedVersion_IsReported()
    {
        var reader = new SessionReader();
        byte[] badVersion = System.Text.Encoding.ASCII.GetBytes("TLOG|999|10000000|123");

        var result = reader.TryReadHeader(badVersion, out _, out _);

        Assert.Equal(SessionOpenResult.UnsupportedVersion, result);
    }

    [Fact]
    public void ChecksumRoundTrip_MatchesAcrossWriterAndReader()
    {
        var writer = new SessionWriter();
        var reader = new SessionReader();
        Span<byte> buffer = new byte[RecordFormat.MaxLineBytes];

        writer.TryWriteHeader(10_000_000, 1, buffer, out int n1);
        reader.AccumulateChecksum(buffer.Slice(0, n1).ToArray());

        writer.TryWriteCommandFrame(SampleFrame(), buffer, out int n2);
        reader.AccumulateChecksum(buffer.Slice(0, n2).ToArray());

        writer.TryWriteEndOfSession(buffer, out int n3);
        bool readEos = reader.TryReadEndOfSession(buffer.Slice(0, n3), out ulong checksum);

        Assert.True(readEos);
        Assert.True(reader.TryVerifyChecksum(checksum));
    }

    [Fact]
    public void ChecksumMismatch_IsDetected()
    {
        var writer = new SessionWriter();
        var reader = new SessionReader();
        Span<byte> buffer = new byte[RecordFormat.MaxLineBytes];

        writer.TryWriteCommandFrame(SampleFrame(), buffer, out int n1);
        // Deliberately do not feed the line to the reader's checksum accumulator.

        writer.TryWriteEndOfSession(buffer, out int n2);
        reader.TryReadEndOfSession(buffer.Slice(0, n2), out ulong checksum);

        Assert.False(reader.TryVerifyChecksum(checksum));
    }

    [Fact]
    public void TooSmallBuffer_ReportsRequiredLengthAndWritesNothing()
    {
        var writer = new SessionWriter();
        Span<byte> tiny = new byte[4];

        bool wrote = writer.TryWriteCommandFrame(SampleFrame(), tiny, out int required);

        Assert.False(wrote);
        Assert.Equal(RecordFormat.MaxCommandFrameLineBytes, required);
        Assert.True(tiny.ToArray().AsSpan().SequenceEqual(new byte[4]));
    }

    [Fact]
    public void CorruptLine_IsRejectedNotThrown()
    {
        var reader = new SessionReader();
        byte[] corrupt = System.Text.Encoding.ASCII.GetBytes("CF|not-a-number|41|123");

        var exception = Record.Exception(() => reader.TryReadCommandFrame(corrupt, out _));

        Assert.Null(exception);
        Assert.False(reader.TryReadCommandFrame(corrupt, out CommandFrame decoded));
        Assert.Equal(default, decoded);
    }

    [Fact]
    public void SessionWriter_Reset_RestoresAsConstructedChecksum()
    {
        var writer = new SessionWriter();
        Span<byte> buffer = new byte[RecordFormat.MaxLineBytes];
        writer.TryWriteCommandFrame(SampleFrame(), buffer, out _);

        writer.Reset();

        // After reset, writing the same single record should produce the same checksum as a
        // brand-new writer would.
        var fresh = new SessionWriter();
        writer.TryWriteHeader(1, 1, buffer, out int n1);
        fresh.TryWriteHeader(1, 1, new byte[RecordFormat.MaxLineBytes], out int n2);
        writer.TryWriteEndOfSession(buffer, out int e1);
        fresh.TryWriteEndOfSession(new byte[RecordFormat.MaxLineBytes], out int e2);

        Assert.Equal(
            System.Text.Encoding.ASCII.GetString(buffer.Slice(0, e1)),
            System.Text.Encoding.ASCII.GetString(buffer.Slice(0, e2)));
    }

    [Fact]
    public void SessionReader_Reset_RestoresAsConstructedChecksum()
    {
        var reader = new SessionReader();
        reader.AccumulateChecksum(new byte[] { 1, 2, 3 });

        reader.Reset();

        Assert.True(reader.TryVerifyChecksum(RecordFormat.FnvOffsetBasis));
    }

    [Fact]
    public void TryWriteCommandFrame_Allocates_Zero_Bytes()
    {
        var writer = new SessionWriter();
        byte[] buffer = new byte[RecordFormat.MaxCommandFrameLineBytes];
        var frame = SampleFrame();
        AllocationAssert.Zero(() => writer.TryWriteCommandFrame(frame, buffer, out _));
    }

    [Fact]
    public void TryReadCommandFrame_Allocates_Zero_Bytes()
    {
        var writer = new SessionWriter();
        byte[] buffer = new byte[RecordFormat.MaxCommandFrameLineBytes];
        writer.TryWriteCommandFrame(SampleFrame(), buffer, out int bytesWritten);
        var reader = new SessionReader();
        byte[] line = buffer.AsSpan(0, bytesWritten).ToArray();
        AllocationAssert.Zero(() => reader.TryReadCommandFrame(line, out _));
    }
}
