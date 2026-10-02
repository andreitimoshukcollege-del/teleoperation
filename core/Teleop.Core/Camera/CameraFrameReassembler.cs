using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Camera
{
    /// <summary>
    /// Turns camera chunks back into whole frames, newest wins
    /// (docs/adr/0014-camera-frame-downlink.md §3). There is no retransmission and no waiting: a frame
    /// is shown only if it completes before a newer one does, and nothing older than the newest
    /// completed frame is ever handed out. For a live view a late frame is worse than no frame.
    ///
    /// <para>Rules, all counted in <see cref="Diagnostics"/>:</para>
    /// <list type="bullet">
    /// <item>A completed frame waits in a ready slot until <see cref="TryTakeLatest"/> takes it. If a
    /// newer frame completes first, the waiting one is <b>superseded</b>.</item>
    /// <item>When a frame completes, every older frame still being assembled is given up as
    /// <b>incomplete</b>: it could no longer be shown.</item>
    /// <item>A chunk for a frame no newer than the newest completed one is <b>late</b> (or a
    /// <b>duplicate</b>, if it is that very frame). When every slot is busy, an arriving frame evicts
    /// the oldest one being assembled (<b>incomplete</b>), unless it is older still, in which case the
    /// arrival is <b>late</b>.</item>
    /// <item>Every chunk claiming a frame id must agree on that frame's stamp and chunk count, or it is
    /// <b>inconsistent</b> and ignored. A chunk can never corrupt a frame it does not belong to.</item>
    /// </list>
    ///
    /// <para><b>Recovering from a sender restart.</b> Newest wins forever, so a sender that restarts
    /// its frame ids below the newest one shown would be ignored until its counter caught up. With
    /// <c>lateChunksBeforeResync</c> set, that many consecutive late chunks (nothing stored in between)
    /// make the reassembler forget its frame history and start again from the next chunk. This is a
    /// <b>resync</b>, counted in <see cref="CameraReassemblerDiagnostics.Resyncs"/>; unlike
    /// <see cref="Reset"/> it keeps every counter. The price: having forgotten what it showed, the
    /// reassembler can let one frame older than the last one shown through right after a resync, if
    /// reordering was ever severe enough to make a whole frame's chunks late in a row.</para>
    ///
    /// Frame ids wrap; ordering uses serial-number arithmetic, so a wrap is not mistaken for an old
    /// frame. All storage is preallocated: <see cref="Accept"/> and <see cref="TryTakeLatest"/> are
    /// allocation-free. Time is a parameter (the host's arrival stamp), never read here.
    /// </summary>
    public sealed class CameraFrameReassembler
    {
        private const byte Free = 0;
        private const byte Assembling = 1;
        private const byte Ready = 2;

        private readonly CameraChunkCodec _codec = new CameraChunkCodec();
        private readonly int _maxFrameBytes;
        private readonly int _maxChunks;
        private readonly int _lateChunksBeforeResync;
        private int _lateRun;
        private long _resyncs;

        private readonly byte[] _state;
        private readonly CameraFrameStamp[] _frame;
        private readonly int[] _chunkCount;
        private readonly int[] _received;
        private readonly int[] _byteCount;
        private readonly long[] _firstArrival;
        private readonly long[] _lastArrival;
        private readonly byte[][] _data;
        private readonly bool[][] _seen;

        private int _readySlot = -1;
        private bool _hasCompleted;
        private uint _newestCompletedId;

        private long _completed;
        private long _taken;
        private long _droppedIncomplete;
        private long _droppedSuperseded;
        private long _late;
        private long _duplicate;
        private long _malformed;
        private long _inconsistent;
        private long _tooLarge;

        /// <param name="slotCount">
        /// Frames held at once: one ready to take plus the ones being assembled. At least 2. Three is
        /// enough for a link that reorders by less than a frame interval.
        /// </param>
        /// <param name="maxFrameBytes">Largest frame accepted; ADR 0014 §6 sizes it at 256 KB.</param>
        /// <param name="lateChunksBeforeResync">
        /// Consecutive late chunks that trigger a resync (see the class doc); 0, the default, never
        /// resyncs. A viewer of a live stream wants it on: about one frame's worth of chunks (a 640x480
        /// frame is ~37) is enough to tell a restarted sender from ordinary reordering.
        /// </param>
        public CameraFrameReassembler(int slotCount, int maxFrameBytes, int lateChunksBeforeResync = 0)
        {
            if (lateChunksBeforeResync < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(lateChunksBeforeResync), lateChunksBeforeResync, "Must be 0 (never) or positive.");
            }

            _lateChunksBeforeResync = lateChunksBeforeResync;

            if (slotCount < 2)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(slotCount), slotCount, "Need at least one slot to assemble in besides the ready one.");
            }

            if (maxFrameBytes <= 0 || maxFrameBytes > CameraChunkCodec.MaxFrameBytes)
            {
                throw new ArgumentOutOfRangeException(nameof(maxFrameBytes), maxFrameBytes, "Frame size out of range.");
            }

            _maxFrameBytes = maxFrameBytes;
            _maxChunks = CameraChunkCodec.ChunkCountFor(maxFrameBytes);

            _state = new byte[slotCount];
            _frame = new CameraFrameStamp[slotCount];
            _chunkCount = new int[slotCount];
            _received = new int[slotCount];
            _byteCount = new int[slotCount];
            _firstArrival = new long[slotCount];
            _lastArrival = new long[slotCount];
            _data = new byte[slotCount][];
            _seen = new bool[slotCount][];
            for (int i = 0; i < slotCount; i++)
            {
                _data[i] = new byte[maxFrameBytes];
                _seen[i] = new bool[_maxChunks];
            }
        }

        public int MaxFrameBytes => _maxFrameBytes;

        public CameraReassemblerDiagnostics Diagnostics
        {
            get
            {
                int assembling = 0;
                for (int i = 0; i < _state.Length; i++)
                {
                    if (_state[i] == Assembling)
                    {
                        assembling++;
                    }
                }

                return new CameraReassemblerDiagnostics(
                    _completed, _taken, _droppedIncomplete, _droppedSuperseded, _late, _duplicate,
                    _malformed, _inconsistent, _tooLarge, assembling, _readySlot >= 0, _resyncs);
            }
        }

        /// <summary>
        /// Takes one received datagram. <paramref name="arrivalTicks"/> is the host's receive stamp
        /// (taken at socket dequeue, docs/metrics.md §1), carried through to
        /// <see cref="CameraFrameInfo"/> unchanged. Allocation-free.
        /// </summary>
        public CameraChunkOutcome Accept(ReadOnlySpan<byte> datagram, long arrivalTicks)
        {
            if (!_codec.TryDecode(datagram, out CameraChunkHeader header, out ReadOnlySpan<byte> payload))
            {
                _malformed++;
                return CameraChunkOutcome.Malformed;
            }

            // The smallest frame this chunk count can describe must fit.
            long smallestFrame = ((long)(header.ChunkCount - 1) * CameraChunkCodec.MaxPayloadBytes) + 1;
            if (header.ChunkCount > _maxChunks || smallestFrame > _maxFrameBytes ||
                (header.IsLastChunk &&
                 ((long)(header.ChunkCount - 1) * CameraChunkCodec.MaxPayloadBytes) + header.PayloadBytes > _maxFrameBytes))
            {
                _tooLarge++;
                return CameraChunkOutcome.TooLarge;
            }

            uint id = header.Frame.FrameId;
            if (_hasCompleted && !IsNewer(id, _newestCompletedId))
            {
                if (id == _newestCompletedId)
                {
                    _duplicate++;
                    return CameraChunkOutcome.Duplicate;
                }

                return Late();
            }

            int slot = FindAssembling(id);
            if (slot < 0)
            {
                slot = ClaimSlot(id);
                if (slot < 0)
                {
                    return Late();
                }

                _state[slot] = Assembling;
                _frame[slot] = header.Frame;
                _chunkCount[slot] = header.ChunkCount;
                _received[slot] = 0;
                _byteCount[slot] = 0;
                _firstArrival[slot] = arrivalTicks;
                Array.Clear(_seen[slot], 0, header.ChunkCount);
            }
            else if (!_frame[slot].SameFrameAs(header.Frame) || _chunkCount[slot] != header.ChunkCount)
            {
                _inconsistent++;
                return CameraChunkOutcome.Inconsistent;
            }

            if (_seen[slot][header.ChunkIndex])
            {
                _duplicate++;
                return CameraChunkOutcome.Duplicate;
            }

            _lateRun = 0;
            payload.CopyTo(new Span<byte>(_data[slot], header.ChunkIndex * CameraChunkCodec.MaxPayloadBytes, payload.Length));
            _seen[slot][header.ChunkIndex] = true;
            _received[slot]++;
            _lastArrival[slot] = arrivalTicks;
            if (header.IsLastChunk)
            {
                _byteCount[slot] = ((header.ChunkCount - 1) * CameraChunkCodec.MaxPayloadBytes) + header.PayloadBytes;
            }

            if (_received[slot] < _chunkCount[slot])
            {
                return CameraChunkOutcome.Stored;
            }

            Complete(slot, id);
            return CameraChunkOutcome.CompletedFrame;
        }

        /// <summary>
        /// Hands out the newest completed frame not yet taken. The bytes alias the reassembler's own
        /// buffer and stay valid only until the next <see cref="Accept"/> or <see cref="Reset"/>, so
        /// decode or copy them first. Allocation-free.
        /// </summary>
        public bool TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> jpeg)
        {
            if (_readySlot < 0)
            {
                info = default;
                jpeg = default;
                return false;
            }

            int slot = _readySlot;
            info = new CameraFrameInfo(
                _frame[slot], _byteCount[slot], _chunkCount[slot], _firstArrival[slot], _lastArrival[slot]);
            jpeg = new ReadOnlySpan<byte>(_data[slot], 0, _byteCount[slot]);
            _state[slot] = Free;
            _readySlot = -1;
            _taken++;
            return true;
        }

        /// <summary>Back to the as-constructed state. Buffers are kept, not reallocated.</summary>
        public void Reset()
        {
            ForgetFrames();
            _resyncs = 0;
            _completed = 0;
            _taken = 0;
            _droppedIncomplete = 0;
            _droppedSuperseded = 0;
            _late = 0;
            _duplicate = 0;
            _malformed = 0;
            _inconsistent = 0;
            _tooLarge = 0;
        }

        private CameraChunkOutcome Late()
        {
            _late++;
            _lateRun++;
            if (_lateChunksBeforeResync > 0 && _lateRun >= _lateChunksBeforeResync)
            {
                ForgetFrames();
                _resyncs++;
            }

            return CameraChunkOutcome.Late;
        }

        /// <summary>Frame state back to empty; counters untouched (shared by Reset and resync).</summary>
        private void ForgetFrames()
        {
            Array.Clear(_state, 0, _state.Length);
            _readySlot = -1;
            _hasCompleted = false;
            _newestCompletedId = 0;
            _lateRun = 0;
        }

        private void Complete(int slot, uint id)
        {
            if (_readySlot >= 0)
            {
                _state[_readySlot] = Free;
                _droppedSuperseded++;
            }

            _state[slot] = Ready;
            _readySlot = slot;
            _hasCompleted = true;
            _newestCompletedId = id;
            _completed++;

            for (int i = 0; i < _state.Length; i++)
            {
                if (_state[i] == Assembling && !IsNewer(_frame[i].FrameId, id))
                {
                    _state[i] = Free;
                    _droppedIncomplete++;
                }
            }
        }

        private int FindAssembling(uint id)
        {
            for (int i = 0; i < _state.Length; i++)
            {
                if (_state[i] == Assembling && _frame[i].FrameId == id)
                {
                    return i;
                }
            }

            return -1;
        }

        /// <summary>A free slot, else the oldest assembling one if <paramref name="id"/> is newer than it; else -1.</summary>
        private int ClaimSlot(uint id)
        {
            int oldest = -1;
            for (int i = 0; i < _state.Length; i++)
            {
                if (_state[i] == Free)
                {
                    return i;
                }

                if (_state[i] == Assembling && (oldest < 0 || IsNewer(_frame[oldest].FrameId, _frame[i].FrameId)))
                {
                    oldest = i;
                }
            }

            if (oldest < 0 || !IsNewer(id, _frame[oldest].FrameId))
            {
                return -1;
            }

            _state[oldest] = Free;
            _droppedIncomplete++;
            return oldest;
        }

        /// <summary>Serial-number order (RFC 1982 style): true when <paramref name="a"/> comes after <paramref name="b"/>.</summary>
        private static bool IsNewer(uint a, uint b) => unchecked((int)(a - b)) > 0;
    }
}
