using Teleop.Core.Camera;
using Teleop.Core.Contracts;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Transport;
using Teleop.Core.Transport.Impairments;

namespace Teleop.Core.Tests.Camera;

/// <summary>
/// <see cref="CameraFrameReassembler"/>: newest wins, every frame not shown is counted, and no chunk
/// can ever write into a frame it does not belong to (ADR 0014 §3, §6).
/// </summary>
public sealed class CameraFrameReassemblerTests
{
    private const long TicksPerSecond = 10_000_000;
    private const int MaxFrameBytes = 256 * 1024;
    private static readonly CameraChunkCodec Codec = new();

    private static CameraFrameStamp StampFor(uint id) =>
        new(id, captureTicks: 1_000 + id * 333_333L, sendTicks: 2_000 + id * 333_333L, TicksPerSecond, 640, 480);

    /// <summary>Content that differs per frame and per byte, so any mix-up shows as a mismatch.</summary>
    private static byte[] JpegFor(uint id, int length = 5_000) =>
        Enumerable.Range(0, length).Select(i => unchecked((byte)(i * 31 + id * 97 + (i >> 8)))).ToArray();

    private static List<byte[]> Chunks(uint id, int length = 5_000) => Chunks(StampFor(id), JpegFor(id, length));

    private static List<byte[]> Chunks(CameraFrameStamp stamp, byte[] jpeg)
    {
        var chunks = new List<byte[]>();
        var buffer = new byte[CameraChunkCodec.MaxDatagramBytes];
        for (int i = 0; i < CameraChunkCodec.ChunkCountFor(jpeg.Length); i++)
        {
            Assert.True(Codec.TryEncodeChunk(stamp, jpeg, i, buffer, out int written));
            chunks.Add(buffer[..written]);
        }

        return chunks;
    }

    private static CameraFrameReassembler New(int slots = 3) => new(slots, MaxFrameBytes);

    private static void AssertTakes(CameraFrameReassembler r, uint expectedId)
    {
        Assert.True(r.TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> jpeg));
        Assert.Equal(expectedId, info.Frame.FrameId);
        Assert.True(info.Frame.SameFrameAs(StampFor(expectedId)));
        Assert.Equal(JpegFor(expectedId, info.ByteCount), jpeg.ToArray());
    }

    [Fact]
    public void ChunksInOrder_CompleteOnTheLastOne_AndCarryTheirArrivalTicks()
    {
        var r = New();
        List<byte[]> chunks = Chunks(1);
        for (int i = 0; i < chunks.Count; i++)
        {
            CameraChunkOutcome expected = i == chunks.Count - 1 ? CameraChunkOutcome.CompletedFrame : CameraChunkOutcome.Stored;
            Assert.Equal(expected, r.Accept(chunks[i], arrivalTicks: 100 + i));
        }

        Assert.True(r.TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> jpeg));
        Assert.Equal(5_000, info.ByteCount);
        Assert.Equal(chunks.Count, info.ChunkCount);
        Assert.Equal(100, info.FirstChunkArrivalTicks);
        Assert.Equal(100 + chunks.Count - 1, info.LastChunkArrivalTicks);
        Assert.Equal(JpegFor(1), jpeg.ToArray());
        Assert.False(r.TryTakeLatest(out _, out _));
    }

    [Fact]
    public void ChunksInReverseOrder_StillRebuildTheExactFrame()
    {
        var r = New();
        List<byte[]> chunks = Chunks(2, length: 4 * CameraChunkCodec.MaxPayloadBytes + 3);
        for (int i = chunks.Count - 1; i >= 0; i--)
        {
            r.Accept(chunks[i], arrivalTicks: 10);
        }

        AssertTakes(r, 2);
    }

    [Fact]
    public void DuplicateChunks_AreCountedAndIgnored()
    {
        var r = New();
        List<byte[]> chunks = Chunks(3);
        Assert.Equal(CameraChunkOutcome.Stored, r.Accept(chunks[0], 0));
        Assert.Equal(CameraChunkOutcome.Duplicate, r.Accept(chunks[0], 0));
        foreach (byte[] chunk in chunks.Skip(1))
        {
            r.Accept(chunk, 0);
        }

        // A chunk of the frame that just completed is a duplicate too, not a new frame.
        Assert.Equal(CameraChunkOutcome.Duplicate, r.Accept(chunks[1], 0));
        Assert.Equal(2, r.Diagnostics.DuplicateChunks);
        AssertTakes(r, 3);
    }

    [Fact]
    public void AFrameMissingAChunk_IsGivenUpWhenANewerFrameCompletes()
    {
        var r = New();
        List<byte[]> lossy = Chunks(10);
        foreach (byte[] chunk in lossy.Skip(1))
        {
            r.Accept(chunk, 0);
        }

        foreach (byte[] chunk in Chunks(11))
        {
            r.Accept(chunk, 0);
        }

        AssertTakes(r, 11);
        Assert.Equal(1, r.Diagnostics.DroppedIncomplete);
        Assert.Equal(0, r.Diagnostics.FramesAssembling);

        // Its missing chunk arriving now is late: frame 10 can never be shown after frame 11.
        Assert.Equal(CameraChunkOutcome.Late, r.Accept(lossy[0], 0));
    }

    [Fact]
    public void AnUntakenFrame_IsSupersededByANewerOne()
    {
        var r = New();
        foreach (byte[] chunk in Chunks(20)) r.Accept(chunk, 0);
        foreach (byte[] chunk in Chunks(21)) r.Accept(chunk, 0);

        AssertTakes(r, 21);
        CameraReassemblerDiagnostics d = r.Diagnostics;
        Assert.Equal(2, d.FramesCompleted);
        Assert.Equal(1, d.FramesTaken);
        Assert.Equal(1, d.DroppedSuperseded);
        Assert.False(d.FrameReady);
    }

    [Fact]
    public void AnOlderFrame_CompletingAfterANewerOneIsShown_IsLate()
    {
        var r = New();
        List<byte[]> older = Chunks(30);
        r.Accept(older[0], 0);
        foreach (byte[] chunk in Chunks(31)) r.Accept(chunk, 0);
        AssertTakes(r, 31);

        foreach (byte[] chunk in older.Skip(1))
        {
            Assert.Equal(CameraChunkOutcome.Late, r.Accept(chunk, 0));
        }

        Assert.False(r.TryTakeLatest(out _, out _));
    }

    [Fact]
    public void WhenEverySlotIsBusy_TheOldestAssemblingFrameIsEvicted()
    {
        var r = New(slots: 2);
        r.Accept(Chunks(40)[0], 0);
        r.Accept(Chunks(41)[0], 0);
        Assert.Equal(2, r.Diagnostics.FramesAssembling);

        // Frame 42 needs a slot: frame 40, the oldest, gives way.
        Assert.Equal(CameraChunkOutcome.Stored, r.Accept(Chunks(42)[0], 0));
        Assert.Equal(1, r.Diagnostics.DroppedIncomplete);

        // An arrival older than everything being assembled is refused instead.
        Assert.Equal(CameraChunkOutcome.Late, r.Accept(Chunks(39)[0], 0));
    }

    [Fact]
    public void FrameIds_WrapWithoutLookingOld()
    {
        var r = New();
        foreach (byte[] chunk in Chunks(uint.MaxValue)) r.Accept(chunk, 0);
        AssertTakes(r, uint.MaxValue);

        foreach (byte[] chunk in Chunks(0)) Assert.NotEqual(CameraChunkOutcome.Late, r.Accept(chunk, 0));
        AssertTakes(r, 0);
    }

    [Fact]
    public void AChunkDisagreeingWithItsFramesStamp_IsRejected_NotMerged()
    {
        var r = New();
        List<byte[]> genuine = Chunks(50);
        r.Accept(genuine[0], 0);

        var forged = new CameraFrameStamp(50, captureTicks: 999, StampFor(50).SendTicks, TicksPerSecond, 640, 480);
        List<byte[]> impostor = Chunks(forged, JpegFor(77));
        Assert.Equal(CameraChunkOutcome.Inconsistent, r.Accept(impostor[1], 0));

        foreach (byte[] chunk in genuine.Skip(1)) r.Accept(chunk, 0);
        AssertTakes(r, 50);
        Assert.Equal(1, r.Diagnostics.InconsistentChunks);
    }

    [Fact]
    public void FramesLargerThanTheConfiguredMaximum_AreRefused()
    {
        var r = new CameraFrameReassembler(slotCount: 2, maxFrameBytes: 2 * CameraChunkCodec.MaxPayloadBytes);
        List<byte[]> big = Chunks(60, length: 2 * CameraChunkCodec.MaxPayloadBytes + 1);
        Assert.Equal(CameraChunkOutcome.TooLarge, r.Accept(big[0], 0));
        Assert.Equal(CameraChunkOutcome.TooLarge, r.Accept(big[^1], 0));

        List<byte[]> fits = Chunks(61, length: 2 * CameraChunkCodec.MaxPayloadBytes);
        foreach (byte[] chunk in fits) r.Accept(chunk, 0);
        AssertTakes(r, 61);
    }

    [Fact]
    public void AFrameExactlyAtTheMaximum_Fits_WhileOneByteMoreDoesNot()
    {
        // 2.5 chunks' worth, so the maximum chunk count (3) has a short last chunk and full non-last ones.
        int max = 2 * CameraChunkCodec.MaxPayloadBytes + CameraChunkCodec.MaxPayloadBytes / 2;
        var r = new CameraFrameReassembler(slotCount: 2, maxFrameBytes: max);

        List<byte[]> exact = Chunks(80, length: max);
        Assert.Equal(CameraChunkCodec.ChunkCountFor(max), exact.Count);
        foreach (byte[] chunk in exact) Assert.NotEqual(CameraChunkOutcome.TooLarge, r.Accept(chunk, 0));
        Assert.True(r.TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> jpeg));
        Assert.Equal(max, info.ByteCount);
        Assert.Equal(JpegFor(80, max), jpeg.ToArray());

        // Same chunk count, one byte longer: only the last chunk reveals it, and it is refused.
        List<byte[]> over = Chunks(81, length: max + 1);
        Assert.Equal(exact.Count, over.Count);
        Assert.Equal(CameraChunkOutcome.Stored, r.Accept(over[0], 0));
        Assert.Equal(CameraChunkOutcome.TooLarge, r.Accept(over[^1], 0));
    }

    [Fact]
    public void ASenderRestartingItsIds_FreezesTheViewWithoutResync()
    {
        var r = New();
        foreach (byte[] chunk in Chunks(5000)) r.Accept(chunk, 0);
        AssertTakes(r, 5000);

        foreach (uint id in new uint[] { 0, 1, 2 })
            foreach (byte[] chunk in Chunks(id)) Assert.Equal(CameraChunkOutcome.Late, r.Accept(chunk, 0));

        Assert.False(r.TryTakeLatest(out _, out _));
        Assert.Equal(0, r.Diagnostics.Resyncs);
    }

    [Fact]
    public void ARunOfLateChunks_Resyncs_SoARestartedSenderIsShownAgain()
    {
        var r = new CameraFrameReassembler(slotCount: 3, MaxFrameBytes, lateChunksBeforeResync: 5);
        foreach (byte[] chunk in Chunks(5000)) r.Accept(chunk, 0);
        AssertTakes(r, 5000);
        long completedBefore = r.Diagnostics.FramesCompleted;

        // The restarted sender's first frame: its first five chunks are late, the fifth triggers the
        // resync, and frame 0 is assembled from the next frame onward.
        foreach (byte[] chunk in Chunks(0)) r.Accept(chunk, 0);
        foreach (byte[] chunk in Chunks(1)) r.Accept(chunk, 0);

        AssertTakes(r, 1);
        CameraReassemblerDiagnostics d = r.Diagnostics;
        Assert.Equal(1, d.Resyncs);
        Assert.Equal(completedBefore + 1, d.FramesCompleted); // counters survive a resync
        Assert.True(d.LateChunks >= 5);
    }

    [Fact]
    public void OrdinaryReordering_NeverAccumulatesARunLongEnoughToResync()
    {
        var r = new CameraFrameReassembler(slotCount: 3, MaxFrameBytes, lateChunksBeforeResync: 5);
        for (uint id = 100; id < 110; id++)
        {
            List<byte[]> chunks = Chunks(id);
            foreach (byte[] chunk in chunks) r.Accept(chunk, 0);
            r.Accept(chunks[0], 0); // one stray straggler per frame
            r.TryTakeLatest(out _, out _);
        }

        Assert.Equal(0, r.Diagnostics.Resyncs);
        Assert.Equal(10, r.Diagnostics.FramesTaken);
    }

    [Fact]
    public void Reset_ClearsTheResyncCount_ButAResyncDoesNot()
    {
        var r = new CameraFrameReassembler(slotCount: 3, MaxFrameBytes, lateChunksBeforeResync: 2);
        foreach (byte[] chunk in Chunks(10)) r.Accept(chunk, 0);
        foreach (byte[] chunk in Chunks(1)) r.Accept(chunk, 0);
        Assert.Equal(1, r.Diagnostics.Resyncs);

        r.Reset();
        Assert.Equal(0, r.Diagnostics.Resyncs);
        Assert.Throws<ArgumentOutOfRangeException>(() => new CameraFrameReassembler(3, MaxFrameBytes, -1));
    }

    [Fact]
    public void Garbage_IsCountedAsMalformed()
    {
        var r = New();
        Assert.Equal(CameraChunkOutcome.Malformed, r.Accept(new byte[] { 1, 2, 3 }, 0));
        Assert.Equal(1, r.Diagnostics.MalformedDatagrams);
    }

    [Fact]
    public void Reset_ReturnsToTheAsConstructedState()
    {
        var r = New();
        foreach (byte[] chunk in Chunks(70)) r.Accept(chunk, 0);
        r.Accept(Chunks(71)[0], 0);
        r.Accept(new byte[] { 9 }, 0);

        r.Reset();

        Assert.Equal(new CameraReassemblerDiagnostics(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0), r.Diagnostics);
        Assert.False(r.TryTakeLatest(out _, out _));

        // A frame older than the pre-reset ones is accepted: nothing was remembered.
        foreach (byte[] chunk in Chunks(5)) r.Accept(chunk, 0);
        AssertTakes(r, 5);
    }

    [Fact]
    public void Constructor_RefusesTooFewSlotsOrABadSize()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new CameraFrameReassembler(1, MaxFrameBytes));
        Assert.Throws<ArgumentOutOfRangeException>(() => new CameraFrameReassembler(3, 0));
    }

    [Fact]
    public void EncodeAcceptAndTake_AreAllocationFree()
    {
        var r = New();
        byte[] jpeg = JpegFor(0, 3 * CameraChunkCodec.MaxPayloadBytes + 10);
        var datagram = new byte[CameraChunkCodec.MaxDatagramBytes];
        int chunkCount = CameraChunkCodec.ChunkCountFor(jpeg.Length);
        uint frameId = 0;
        int chunk = 0;

        AllocationAssert.Zero(() =>
        {
            var stamp = new CameraFrameStamp(frameId, 1, 2, TicksPerSecond, 640, 480);
            Codec.TryEncodeChunk(stamp, jpeg, chunk, datagram, out int written);
            if (r.Accept(new ReadOnlySpan<byte>(datagram, 0, written), 0) == CameraChunkOutcome.CompletedFrame)
            {
                r.TryTakeLatest(out _, out _);
            }

            if (++chunk == chunkCount)
            {
                chunk = 0;
                frameId++;
            }
        });

        Assert.True(r.Diagnostics.FramesTaken > 1000);
    }

    /// <summary>
    /// The end-to-end property ADR 0014 §6 asks for: frames chunked and sent through the emulator's
    /// burst loss, jitter and reordering come out either byte-identical or not at all, in strictly
    /// increasing order, with every frame accounted for.
    /// </summary>
    [Theory]
    [InlineData(1UL)]
    [InlineData(2UL)]
    [InlineData(3UL)]
    public void ThroughLossJitterAndReordering_FramesArriveIntactOrNotAtAll(ulong seed)
    {
        const long Millisecond = TicksPerSecond / 1000;
        var inner = new LoopbackTransport(maxPayloadBytes: CameraChunkCodec.MaxDatagramBytes, capacity: 4096);
        var impairments = new INetworkImpairment[]
        {
            new FixedDelayImpairment(10 * Millisecond),
            new UniformJitterImpairment(4 * Millisecond),
            new ReorderImpairment(probability: 0.05, extraDelayTicks: 15 * Millisecond),
            new GilbertElliottLossImpairment(lossProbabilityAfterDelivered: 0.01, lossProbabilityAfterLost: 0.4),
        };
        var link = new EmulatedTransport(inner, impairments, seed, maxInFlight: 4096);
        var r = New();

        const int Frames = 150;
        var sentIds = new HashSet<uint>();
        var receive = new byte[CameraChunkCodec.MaxDatagramBytes];
        long lastTaken = -1;
        int taken = 0;

        for (long now = 0; now < (Frames * 33 + 200) * Millisecond; now += Millisecond)
        {
            if (now % (33 * Millisecond) == 0 && now / (33 * Millisecond) < Frames)
            {
                uint id = (uint)(now / (33 * Millisecond));
                sentIds.Add(id);
                foreach (byte[] chunk in Chunks(id, length: 20_000 + (int)id * 37))
                {
                    link.Send(chunk, now);
                }
            }

            while (link.TryReceive(now, receive, out int byteCount, out long arrival))
            {
                r.Accept(new ReadOnlySpan<byte>(receive, 0, byteCount), arrival);
            }

            while (r.TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> jpeg))
            {
                uint id = info.Frame.FrameId;
                Assert.Contains(id, sentIds);
                Assert.True(id > lastTaken, $"frame {id} shown after frame {lastTaken}");
                Assert.True(info.Frame.SameFrameAs(StampFor(id)));
                Assert.Equal(JpegFor(id, 20_000 + (int)id * 37), jpeg.ToArray());
                lastTaken = id;
                taken++;
            }
        }

        CameraReassemblerDiagnostics d = r.Diagnostics;
        Assert.True(taken > Frames / 2, $"only {taken}/{Frames} frames survived");
        Assert.Equal(taken, d.FramesTaken);
        Assert.Equal(d.FramesCompleted, d.FramesTaken + d.DroppedSuperseded);
        Assert.True(d.DroppedIncomplete > 0, "the lossy link should have cost at least one frame");

        // Each sent frame is counted at most once: completed, given up as incomplete, or still being
        // assembled. Frames that lost every chunk never reach the reassembler and appear in none.
        Assert.True(d.FramesCompleted + d.DroppedIncomplete + d.FramesAssembling <= Frames,
            $"{d.FramesCompleted} completed + {d.DroppedIncomplete} incomplete + {d.FramesAssembling} assembling > {Frames} sent");
    }
}
