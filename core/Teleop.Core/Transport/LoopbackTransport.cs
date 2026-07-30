using System;
using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// The zero-impairment baseline <see cref="ITransport"/>: an in-process FIFO queue with no
    /// delay, no loss, and no reordering. A datagram handed to <see cref="Send"/> is immediately
    /// available to <see cref="TryReceive"/>, and its reported arrival tick equals its send tick
    /// exactly.
    ///
    /// Two jobs. First, it is the "no network at all" control condition — every latency figure
    /// measured through it is pipeline overhead and nothing else, so a nonzero one-way delay
    /// measured over this transport is a bug in the measurement, not a property of a link.
    /// Second, it is what <see cref="EmulatedTransport"/> wraps headlessly: the emulator adds its
    /// synthetic impairment on top of whatever the wrapped transport reports, and wrapping a
    /// transport that reports exactly zero is what makes the injected profile the whole of the
    /// measured delay.
    ///
    /// Storage is a fixed-capacity ring allocated once in the constructor. Nothing here allocates
    /// after construction, and the queue never grows: a send into a full ring returns false, the
    /// same ordinary "this datagram will not be delivered" outcome <see cref="ITransport.Send"/>
    /// documents for a full send queue. Growing instead would turn a capacity problem into an
    /// unbounded-latency problem, which is precisely the failure this project is measuring.
    ///
    /// <b>No config struct, deliberately.</b> Unlike <see cref="Teleop.Core.Types.NetworkProfile"/>
    /// this transport has no research knobs — its two constructor parameters are buffer sizing,
    /// not parameters anyone would sweep, and no result would ever be reported "at capacity 64".
    /// A one-field <c>LoopbackConfig</c> would be ceremony that implies a tunable where there is
    /// none. This is a considered choice, not an oversight.
    ///
    /// Not thread-safe, by contract. Time is a parameter, never a clock read.
    /// </summary>
    public sealed class LoopbackTransport : ITransport
    {
        private readonly int _maxPayloadBytes;
        private readonly int _capacity;

        /// <summary>Backing store for all slots, <c>_capacity * _maxPayloadBytes</c> bytes, allocated once.</summary>
        private readonly byte[] _payloads;

        /// <summary>Bytes valid in each slot's region of <see cref="_payloads"/>.</summary>
        private readonly int[] _lengths;

        /// <summary>Send tick of each slot's datagram, which is also its arrival tick here.</summary>
        private readonly long[] _sendTicks;

        /// <summary>Index of the oldest occupied slot; meaningless when <see cref="_count"/> is zero.</summary>
        private int _head;

        /// <summary>Occupied slot count, in [0, _capacity].</summary>
        private int _count;

        /// <param name="maxPayloadBytes">
        /// Largest datagram this transport carries. A send longer than this is rejected rather
        /// than truncated or throwing.
        /// </param>
        /// <param name="capacity">
        /// Number of datagrams that may be queued at once. Sends beyond this return false.
        /// </param>
        public LoopbackTransport(int maxPayloadBytes, int capacity)
        {
            if (maxPayloadBytes <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(maxPayloadBytes), maxPayloadBytes, "Max payload must be positive.");
            }

            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(capacity), capacity, "Capacity must be positive.");
            }

            _maxPayloadBytes = maxPayloadBytes;
            _capacity = capacity;
            _payloads = new byte[checked(maxPayloadBytes * capacity)];
            _lengths = new int[capacity];
            _sendTicks = new long[capacity];
            _head = 0;
            _count = 0;
        }

        /// <inheritdoc/>
        public int MaxPayloadBytes => _maxPayloadBytes;

        /// <summary>
        /// Datagrams currently queued. Exposed for tests and for a host that wants to know it is
        /// about to overflow; not a diagnostics channel and never logged.
        /// </summary>
        public int QueuedCount => _count;

        /// <summary>Slots in the ring, fixed at construction.</summary>
        public int Capacity => _capacity;

        /// <summary>
        /// Copies <paramref name="payload"/> into the next free slot in FIFO order. Returns false
        /// when the ring is full or the payload exceeds <see cref="MaxPayloadBytes"/> — both are
        /// ordinary non-delivery outcomes, not errors, and the caller must not retry (see
        /// <see cref="ITransport.Send"/>).
        ///
        /// <paramref name="nowTicks"/> is stored as this datagram's arrival tick, since a loopback
        /// adds no transit delay. Allocation-free.
        /// </summary>
        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            if (_count == _capacity)
            {
                return false;
            }

            if (payload.Length > _maxPayloadBytes)
            {
                return false;
            }

            int slot = _head + _count;
            if (slot >= _capacity)
            {
                slot -= _capacity;
            }

            payload.CopyTo(new Span<byte>(_payloads, slot * _maxPayloadBytes, _maxPayloadBytes));
            _lengths[slot] = payload.Length;
            _sendTicks[slot] = nowTicks;
            _count++;
            return true;
        }

        /// <summary>
        /// Pops the oldest queued datagram, reporting <paramref name="arrivalTicks"/> equal to its
        /// send tick. Returns false when the queue is empty.
        ///
        /// A datagram whose send tick is after <paramref name="nowTicks"/> is not yet visible.
        /// That cannot happen with a monotonic caller, but the check is here rather than assumed:
        /// <see cref="ITransport.TryReceive"/> says "arrived at or before nowTicks", and a
        /// transport that quietly hands back a datagram from the future would make a replay
        /// disagree with a live run in a way that looks like a prediction bug.
        ///
        /// If <paramref name="destination"/> is shorter than the pending datagram this returns
        /// false, sets <paramref name="byteCount"/> to the length required, and <b>leaves the
        /// datagram queued</b> — the caller can retry with a larger buffer and lose nothing.
        /// Allocation-free.
        /// </summary>
        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            byteCount = 0;
            arrivalTicks = 0;

            if (_count == 0)
            {
                return false;
            }

            long sendTicks = _sendTicks[_head];
            if (sendTicks > nowTicks)
            {
                return false;
            }

            int length = _lengths[_head];
            if (destination.Length < length)
            {
                byteCount = length;
                return false;
            }

            new ReadOnlySpan<byte>(_payloads, _head * _maxPayloadBytes, length).CopyTo(destination);
            byteCount = length;
            arrivalTicks = sendTicks;

            _lengths[_head] = 0;
            _sendTicks[_head] = 0;
            _head++;
            if (_head == _capacity)
            {
                _head = 0;
            }
            _count--;
            return true;
        }

        /// <summary>
        /// Empties the queue and returns the cursors to their as-constructed values. Slot bytes
        /// are cleared too: a stale payload behind a reset would be invisible in normal use but
        /// would surface as a phantom datagram the moment a length bookkeeping bug appeared, and
        /// a sweep reusing this instance across trials must not carry one trial's bytes into the
        /// next. This transport owns no RNG, so there is nothing to reseed.
        /// </summary>
        public void Reset()
        {
            Array.Clear(_payloads, 0, _payloads.Length);
            Array.Clear(_lengths, 0, _lengths.Length);
            Array.Clear(_sendTicks, 0, _sendTicks.Length);
            _head = 0;
            _count = 0;
        }
    }
}
