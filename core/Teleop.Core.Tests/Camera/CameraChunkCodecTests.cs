using System.Buffers.Binary;
using Teleop.Core.Camera;

namespace Teleop.Core.Tests.Camera;

/// <summary>
/// <see cref="CameraChunkCodec"/> and <see cref="CameraSubscribeCodec"/>: the byte layout ADR 0014 §3
/// specifies, and that anything which is not exactly a well-formed chunk is refused whole.
/// </summary>
public sealed class CameraChunkCodecTests
{
    private static readonly CameraFrameStamp Stamp = new(7, 1_000_000_123, 1_000_500_456, 1_000_000_000, 640, 480);
    private readonly CameraChunkCodec _codec = new();

    private static byte[] Jpeg(int length) => Enumerable.Range(0, length).Select(i => (byte)(i * 31 + 5)).ToArray();

    [Theory]
    [InlineData(0, 0)]
    [InlineData(1, 1)]
    [InlineData(CameraChunkCodec.MaxPayloadBytes, 1)]
    [InlineData(CameraChunkCodec.MaxPayloadBytes + 1, 2)]
    [InlineData(42_000, 37)]
    public void ChunkCountFor_SplitsIntoFullChunksPlusARemainder(int frameBytes, int expected)
    {
        Assert.Equal(expected, CameraChunkCodec.ChunkCountFor(frameBytes));
    }

    [Fact]
    public void Sizes_FitTheAdrsDatagramBudget()
    {
        Assert.Equal(41, CameraChunkCodec.HeaderSize);
        Assert.Equal(1200, CameraChunkCodec.MaxDatagramBytes);
        Assert.Equal(1159, CameraChunkCodec.MaxPayloadBytes);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(CameraChunkCodec.MaxPayloadBytes)]
    [InlineData(CameraChunkCodec.MaxPayloadBytes * 3 + 17)]
    public void EveryChunk_RoundTripsAndTheirPayloadsRebuildTheFrame(int frameBytes)
    {
        byte[] jpeg = Jpeg(frameBytes);
        int count = CameraChunkCodec.ChunkCountFor(frameBytes);
        var rebuilt = new List<byte>();
        var buffer = new byte[CameraChunkCodec.MaxDatagramBytes];

        for (int i = 0; i < count; i++)
        {
            Assert.True(_codec.TryEncodeChunk(Stamp, jpeg, i, buffer, out int written));
            Assert.True(written <= CameraChunkCodec.MaxDatagramBytes);
            Assert.True(_codec.TryDecode(buffer.AsSpan(0, written), out CameraChunkHeader header, out ReadOnlySpan<byte> payload));

            Assert.True(header.Frame.SameFrameAs(Stamp));
            Assert.Equal(i, header.ChunkIndex);
            Assert.Equal(count, header.ChunkCount);
            Assert.Equal(i == count - 1, header.IsLastChunk);
            rebuilt.AddRange(payload.ToArray());
        }

        Assert.Equal(jpeg, rebuilt.ToArray());
    }

    [Fact]
    public void HeaderLayout_MatchesAdr0014()
    {
        var buffer = new byte[CameraChunkCodec.MaxDatagramBytes];
        Assert.True(_codec.TryEncodeChunk(Stamp, Jpeg(100), 0, buffer, out int written));

        Assert.Equal(new byte[] { 0x43, 0x56 }, buffer[..2]); // "CV"
        Assert.Equal(CameraChunkCodec.Version, buffer[2]);
        Assert.Equal(7u, BinaryPrimitives.ReadUInt32LittleEndian(buffer.AsSpan(3)));
        Assert.Equal((ushort)0, BinaryPrimitives.ReadUInt16LittleEndian(buffer.AsSpan(7)));
        Assert.Equal((ushort)1, BinaryPrimitives.ReadUInt16LittleEndian(buffer.AsSpan(9)));
        Assert.Equal(1_000_000_123L, BinaryPrimitives.ReadInt64LittleEndian(buffer.AsSpan(11)));
        Assert.Equal(1_000_500_456L, BinaryPrimitives.ReadInt64LittleEndian(buffer.AsSpan(19)));
        Assert.Equal(1_000_000_000L, BinaryPrimitives.ReadInt64LittleEndian(buffer.AsSpan(27)));
        Assert.Equal((ushort)640, BinaryPrimitives.ReadUInt16LittleEndian(buffer.AsSpan(35)));
        Assert.Equal((ushort)480, BinaryPrimitives.ReadUInt16LittleEndian(buffer.AsSpan(37)));
        Assert.Equal((ushort)100, BinaryPrimitives.ReadUInt16LittleEndian(buffer.AsSpan(39)));
        Assert.Equal(141, written);
    }

    [Fact]
    public void TryEncodeChunk_TooSmallDestination_ReportsTheSizeNeeded()
    {
        Assert.False(_codec.TryEncodeChunk(Stamp, Jpeg(100), 0, new byte[50], out int needed));
        Assert.Equal(141, needed);
    }

    [Theory]
    [InlineData(0, 0)]   // empty frame
    [InlineData(100, 1)] // index past the only chunk
    [InlineData(100, -1)]
    public void TryEncodeChunk_RefusesAnEmptyFrameOrAnIndexOutOfRange(int frameBytes, int index)
    {
        Assert.False(_codec.TryEncodeChunk(Stamp, Jpeg(frameBytes), index, new byte[2000], out int written));
        Assert.Equal(0, written);
    }

    public static IEnumerable<object[]> Corruptions()
    {
        yield return new object[] { "too short", (Func<byte[], byte[]>)(d => d[..40]) };
        yield return new object[] { "bad magic", (Func<byte[], byte[]>)(d => { d[0] ^= 0xFF; return d; }) };
        yield return new object[] { "bad version", (Func<byte[], byte[]>)(d => { d[2] = 99; return d; }) };
        yield return new object[] { "index >= count", (Func<byte[], byte[]>)(d => { BinaryPrimitives.WriteUInt16LittleEndian(d.AsSpan(7), 5); return d; }) };
        yield return new object[] { "zero count", (Func<byte[], byte[]>)(d => { BinaryPrimitives.WriteUInt16LittleEndian(d.AsSpan(9), 0); return d; }) };
        yield return new object[] { "payload length disagrees", (Func<byte[], byte[]>)(d => { BinaryPrimitives.WriteUInt16LittleEndian(d.AsSpan(39), 99); return d; }) };
        yield return new object[] { "trailing byte", (Func<byte[], byte[]>)(d => d.Append((byte)0).ToArray()) };
        yield return new object[] { "short non-final chunk", (Func<byte[], byte[]>)(d => { BinaryPrimitives.WriteUInt16LittleEndian(d.AsSpan(9), 2); return d; }) };
        yield return new object[] { "zero clock rate", (Func<byte[], byte[]>)(d => { BinaryPrimitives.WriteInt64LittleEndian(d.AsSpan(27), 0); return d; }) };
        yield return new object[] { "negative clock rate", (Func<byte[], byte[]>)(d => { BinaryPrimitives.WriteInt64LittleEndian(d.AsSpan(27), -1); return d; }) };
        yield return new object[] { "zero width", (Func<byte[], byte[]>)(d => { BinaryPrimitives.WriteUInt16LittleEndian(d.AsSpan(35), 0); return d; }) };
        yield return new object[] { "zero height", (Func<byte[], byte[]>)(d => { BinaryPrimitives.WriteUInt16LittleEndian(d.AsSpan(37), 0); return d; }) };
    }

    [Theory]
    [MemberData(nameof(Corruptions))]
    public void TryDecode_RefusesAnythingButAWellFormedChunk(string what, Func<byte[], byte[]> corrupt)
    {
        var buffer = new byte[CameraChunkCodec.MaxDatagramBytes];
        Assert.True(_codec.TryEncodeChunk(Stamp, Jpeg(100), 0, buffer, out int written));
        byte[] datagram = corrupt(buffer[..written]);

        Assert.False(_codec.TryDecode(datagram, out CameraChunkHeader header, out ReadOnlySpan<byte> payload), what);
        Assert.Equal(default, header.ChunkCount);
        Assert.True(payload.IsEmpty);
    }

    [Fact]
    public void Subscribe_RoundTripsAndRefusesOtherBytes()
    {
        var codec = new CameraSubscribeCodec();
        var buffer = new byte[CameraSubscribeCodec.EncodedSize];
        Assert.True(codec.TryEncode(30, 123_456_789, buffer, out int written));
        Assert.Equal(13, written);
        Assert.Equal(new byte[] { 0x43, 0x53 }, buffer[..2]); // "CS"

        Assert.True(codec.TryDecode(buffer, out ushort fps, out long sendTicks));
        Assert.Equal((ushort)30, fps);
        Assert.Equal(123_456_789L, sendTicks);

        var chunk = new byte[CameraChunkCodec.MaxDatagramBytes];
        Assert.True(_codec.TryEncodeChunk(Stamp, Jpeg(10), 0, chunk, out int chunkBytes));
        Assert.False(codec.TryDecode(chunk.AsSpan(0, chunkBytes), out _, out _));
        Assert.False(codec.TryDecode(buffer.AsSpan(0, 12), out _, out _));
        Assert.False(codec.TryEncode(30, 0, new byte[12], out int needed));
        Assert.Equal(13, needed);
    }
}
