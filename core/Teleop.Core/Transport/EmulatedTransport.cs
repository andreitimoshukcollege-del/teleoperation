using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// A <see cref="ITransport"/> <b>decorator</b> that injects a reproducible synthetic
    /// impairment — fixed delay, uniform jitter, Gilbert-Elliott burst loss, and explicit
    /// reordering — on top of whatever transport it wraps. Most of this project's research runs
    /// through it. Wrapping <see cref="LoopbackTransport"/> gives a fully synthetic link for
    /// headless evaluation; wrapping the host's <c>Bridge/UdpTransport.cs</c> on a LAN gives a
    /// reproducible impairment layered over a real socket. Impairment is always <i>additive</i>:
    /// the wrapped transport's own transit delay and its own losses stand, and this decorator's
    /// model is applied on top rather than in place of them.
    ///
    /// All impairment is drawn from the injected <see cref="SeededRng"/>. Same seed, same profile
    /// and same sequence of calls produce a bit-identical sequence of receives, which is the
    /// property <c>Transport/CLAUDE.md</c> requires and the reason nothing here reads a clock or
    /// calls <c>System.Random</c>.
    ///
    /// <b>Why delay is implemented on the receive side.</b> <see cref="ITransport.Send"/> has no
    /// "deliver at a future time" parameter and Core has no threads and no timers, so a datagram
    /// cannot be handed to the wrapped transport with instructions to surface late. Instead the
    /// wrapped transport is drained opportunistically inside <see cref="TryReceive"/>, each drained
    /// datagram's synthetic arrival tick is computed at drain time as
    /// <c>innerArrivalTicks + delay</c>, and it is held in a fixed-capacity min-heap keyed by that
    /// tick until the caller polls at or after it. Note the delay is added to the tick the wrapped
    /// transport <i>reported</i>, never to the poll time: folding poll time in would make measured
    /// one-way delay depend on the host's frame rate, which is one of the concrete ways this
    /// project could produce confident wrong numbers.
    ///
    /// <b>Reordering falls out of the structure.</b> Delivery is by earliest synthetic arrival, not
    /// by send order, so any two datagrams whose synthetic arrivals invert — from a jitter draw or
    /// from the explicit reorder knob — are returned out of send order with no special case.
    ///
    /// <b>Not implemented here:</b> trace-driven replay of recorded one-way delays. That mode reads
    /// a frozen capture from <c>core/testdata/traces/</c>, which does not exist yet, and per
    /// <c>Transport/CLAUDE.md</c> the standard profile set cannot be extended without an ADR. This
    /// class covers the parametric profiles only.
    ///
    /// Everything is preallocated in the constructor; <see cref="Send"/> and
    /// <see cref="TryReceive"/> allocate nothing. Not thread-safe, by contract.
    /// </summary>
    public sealed class EmulatedTransport : ITransport
    {
        private readonly ITransport _inner;
        private readonly NetworkProfile _profile;
        private readonly int _maxInFlight;

        /// <summary>
        /// Cached at construction rather than read per drain. <c>ITransport.MaxPayloadBytes</c> is
        /// documented as a constant of the transport, and the slot arrays are sized from it, so
        /// re-reading it would only create a way for the two to disagree.
        /// </summary>
        private readonly int _innerMaxPayloadBytes;

        /// <summary>Owned by value. <see cref="SeededRng"/> is a mutable struct; it is never copied
        /// out of this field, because a copy and the original would silently diverge.</summary>
        private SeededRng _rng;

        // Payload slots. A drained datagram's bytes live in exactly one slot for its whole time in
        // flight; the heap moves 24-byte keys around, never these bytes.
        private readonly byte[] _slotPayloads;
        private readonly int[] _slotLengths;

        // Free-slot stack. Depth is the number of slots not currently held by the heap.
        private readonly int[] _freeSlots;
        private int _freeCount;

        // Min-heap over (arrival, sequence), storing the slot index that holds each datagram.
        private readonly long[] _heapArrivalTicks;
        private readonly long[] _heapSequence;
        private readonly int[] _heapSlots;
        private int _heapCount;

        /// <summary>
        /// Insertion counter used only to break ties between equal synthetic arrival ticks. Without
        /// it, two datagrams landing on the same tick would be ordered by heap-internal array
        /// layout, which is an implementation detail and would make delivery order depend on
        /// unrelated changes.
        /// </summary>
        private long _nextSequence;

        /// <summary>
        /// The one bit of Gilbert-Elliott state: whether the previous datagram sent through this
        /// instance was dropped by this decorator's loss model. Starts false — the chain begins in
        /// the good state.
        /// </summary>
        private bool _previousWasLost;

        /// <param name="inner">Transport to decorate. Its delay and losses are kept, not replaced.</param>
        /// <param name="profile">Impairment parameters; see <see cref="NetworkProfile"/>.</param>
        /// <param name="rng">
        /// Seeded generator driving every impairment decision. Taken by value and owned: this
        /// instance's <c>Reset()</c> reseeds its own copy and does not disturb the caller's.
        /// </param>
        /// <param name="maxInFlight">
        /// Number of delayed datagrams held at once. When full, the wrapped transport is simply not
        /// drained, so its datagrams stay queued there (back-pressure) rather than being silently
        /// destroyed by the emulator — an emulator that dropped them would add loss that is not in
        /// the profile and is therefore not in the manifest.
        /// </param>
        public EmulatedTransport(ITransport inner, NetworkProfile profile, SeededRng rng, int maxInFlight)
        {
            if (inner == null)
            {
                throw new ArgumentNullException(nameof(inner));
            }

            if (maxInFlight <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(maxInFlight), maxInFlight, "In-flight capacity must be positive.");
            }

            ValidateProfile(profile);

            _inner = inner;
            _profile = profile;
            _rng = rng;
            _maxInFlight = maxInFlight;
            _innerMaxPayloadBytes = inner.MaxPayloadBytes;

            if (_innerMaxPayloadBytes <= 0)
            {
                throw new ArgumentException(
                    "Wrapped transport reports a non-positive MaxPayloadBytes.", nameof(inner));
            }

            _slotPayloads = new byte[checked(_innerMaxPayloadBytes * maxInFlight)];
            _slotLengths = new int[maxInFlight];
            _freeSlots = new int[maxInFlight];
            _heapArrivalTicks = new long[maxInFlight];
            _heapSequence = new long[maxInFlight];
            _heapSlots = new int[maxInFlight];

            ResetLocalState();
        }

        /// <inheritdoc/>
        public int MaxPayloadBytes => _inner.MaxPayloadBytes;

        /// <summary>Datagrams currently held under synthetic delay. For tests and host sizing.</summary>
        public int InFlightCount => _heapCount;

        /// <summary>Slots available to hold delayed datagrams, fixed at construction.</summary>
        public int MaxInFlight => _maxInFlight;

        /// <summary>
        /// Rolls this datagram's loss decision, then either drops it or hands it to the wrapped
        /// transport unmodified.
        ///
        /// The loss roll is conditioned on whether the <i>previous</i> datagram through this
        /// instance was lost, which is what makes losses arrive in bursts
        /// (<see cref="NetworkProfile.ExpectedBurstLength"/>) rather than independently. A lost
        /// datagram is <b>not</b> passed to the wrapped transport at all: on a real link a dropped
        /// packet never occupies the wire, so sending and then discarding would misreport the
        /// wrapped transport's queue occupancy and, over a socket, would actually transmit bytes
        /// the model says were lost.
        ///
        /// Returns false on emulated loss and also when the wrapped transport itself refuses the
        /// datagram — both are the ordinary non-delivery outcome of
        /// <see cref="ITransport.Send"/>, and callers must not retry. A refusal by the wrapped
        /// transport does not advance the Gilbert-Elliott chain: that chain models this link's loss
        /// process, and a full queue downstream is a different mechanism which would distort the
        /// modelled burst statistics if folded in.
        ///
        /// <paramref name="nowTicks"/> is passed straight through; the synthetic delay is applied
        /// on the receive side, so it is deliberately not added here. Allocation-free.
        /// </summary>
        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            double lossProbability = _previousWasLost
                ? _profile.LossProbabilityAfterLost
                : _profile.LossProbabilityAfterDelivered;

            // NextDouble() is uniform on [0, 1), so probability 0 never fires and probability 1
            // always does — both endpoints behave exactly, with no epsilon anywhere.
            bool lost = _rng.NextDouble() < lossProbability;
            _previousWasLost = lost;

            if (lost)
            {
                return false;
            }

            return _inner.Send(payload, nowTicks);
        }

        /// <summary>
        /// Drains everything the wrapped transport has ready, assigning each drained datagram a
        /// synthetic arrival tick, then returns the single earliest datagram whose synthetic
        /// arrival is at or before <paramref name="nowTicks"/>.
        ///
        /// Returns false when nothing is due — either the heap is empty or its earliest entry is
        /// still in the future. Draining still happens on that call, which is what lets a host poll
        /// at whatever rate it likes without changing the measured delay of anything.
        ///
        /// <paramref name="arrivalTicks"/> is the synthetic arrival, which may be earlier than
        /// <paramref name="nowTicks"/> when the host polled late. Reporting the poll time instead
        /// would fold frame time into one-way delay.
        ///
        /// A <paramref name="destination"/> too short for the due datagram returns false, reports
        /// the required length in <paramref name="byteCount"/>, and leaves the datagram in the
        /// heap. Allocation-free.
        /// </summary>
        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            DrainInner(nowTicks);

            byteCount = 0;
            arrivalTicks = 0;

            if (_heapCount == 0)
            {
                return false;
            }

            long dueTicks = _heapArrivalTicks[0];
            if (dueTicks > nowTicks)
            {
                return false;
            }

            int slot = _heapSlots[0];
            int length = _slotLengths[slot];
            if (destination.Length < length)
            {
                byteCount = length;
                return false;
            }

            new ReadOnlySpan<byte>(_slotPayloads, slot * _innerMaxPayloadBytes, length).CopyTo(destination);
            byteCount = length;
            arrivalTicks = dueTicks;

            HeapPop();
            _slotLengths[slot] = 0;
            _freeSlots[_freeCount] = slot;
            _freeCount++;
            return true;
        }

        /// <summary>
        /// Returns this decorator and the transport it wraps to their as-constructed state:
        /// nothing in flight, the Gilbert-Elliott chain back in the good state, the tie-break
        /// counter back to zero, and the owned RNG reseeded to its construction seed so the next
        /// trial reproduces the previous one — all three requirements of
        /// <see cref="ITransport.Reset"/>. <c>_inner.Reset()</c> is called because a decorator
        /// resets what it wraps; leaving the wrapped transport holding a previous trial's
        /// datagrams would contaminate the next trial in a way that looks like spurious loss.
        /// </summary>
        public void Reset()
        {
            ResetLocalState();
            _rng.Reset();
            _inner.Reset();
        }

        private void ResetLocalState()
        {
            Array.Clear(_slotPayloads, 0, _slotPayloads.Length);
            Array.Clear(_slotLengths, 0, _slotLengths.Length);
            Array.Clear(_heapArrivalTicks, 0, _heapArrivalTicks.Length);
            Array.Clear(_heapSequence, 0, _heapSequence.Length);
            Array.Clear(_heapSlots, 0, _heapSlots.Length);

            for (int i = 0; i < _maxInFlight; i++)
            {
                _freeSlots[i] = i;
            }

            _freeCount = _maxInFlight;
            _heapCount = 0;
            _nextSequence = 0;
            _previousWasLost = false;
        }

        /// <summary>
        /// Pulls every datagram the wrapped transport has ready into the delay heap. Stops when the
        /// wrapped transport is empty or no in-flight slot is free; in the latter case the datagram
        /// stays where it is, un-drained, rather than being dropped.
        /// </summary>
        private void DrainInner(long nowTicks)
        {
            while (_freeCount > 0)
            {
                int slot = _freeSlots[_freeCount - 1];
                var destination = new Span<byte>(
                    _slotPayloads, slot * _innerMaxPayloadBytes, _innerMaxPayloadBytes);

                if (!_inner.TryReceive(nowTicks, destination, out int length, out long innerArrivalTicks))
                {
                    return;
                }

                _freeCount--;
                _slotLengths[slot] = length;
                HeapPush(innerArrivalTicks + DrawDelayTicks(), slot);
            }
        }

        /// <summary>
        /// One packet's synthetic delay: base, plus a uniform integer jitter draw on
        /// <c>[-JitterTicks, +JitterTicks]</c>, plus <c>ReorderDelayTicks</c> if the reorder roll
        /// fires. Clamped at zero, since a negative total delay would mean arriving before the
        /// wrapped transport actually delivered it.
        ///
        /// Both draws are made unconditionally, even when the corresponding knob is zero (a zero
        /// half-width draws from a one-element range and yields exactly 0; a zero probability never
        /// fires). That costs two draws per datagram and buys common random numbers across a sweep:
        /// RNG consumption depends only on how many datagrams flowed, not on the profile's values,
        /// so two profiles differing in one knob and sharing a seed see the same underlying stream
        /// for every other knob, and the difference between their results is the knob rather than
        /// re-randomization.
        /// </summary>
        private long DrawDelayTicks()
        {
            long halfWidth = _profile.JitterTicks;
            ulong span = ((ulong)halfWidth * 2UL) + 1UL;
            long jitter = (long)(_rng.NextUInt64() % span) - halfWidth;

            long delayTicks = _profile.BaseDelayTicks + jitter;

            if (_rng.NextDouble() < _profile.ReorderProbability)
            {
                delayTicks += _profile.ReorderDelayTicks;
            }

            return delayTicks < 0 ? 0 : delayTicks;
        }

        private void HeapPush(long arrivalTicks, int slot)
        {
            int index = _heapCount;
            _heapArrivalTicks[index] = arrivalTicks;
            _heapSequence[index] = _nextSequence;
            _heapSlots[index] = slot;
            _nextSequence++;
            _heapCount++;

            while (index > 0)
            {
                int parent = (index - 1) / 2;
                if (!IsBefore(index, parent))
                {
                    break;
                }

                Swap(index, parent);
                index = parent;
            }
        }

        private void HeapPop()
        {
            _heapCount--;
            if (_heapCount > 0)
            {
                _heapArrivalTicks[0] = _heapArrivalTicks[_heapCount];
                _heapSequence[0] = _heapSequence[_heapCount];
                _heapSlots[0] = _heapSlots[_heapCount];
            }

            _heapArrivalTicks[_heapCount] = 0;
            _heapSequence[_heapCount] = 0;
            _heapSlots[_heapCount] = 0;

            int index = 0;
            while (true)
            {
                int left = (2 * index) + 1;
                if (left >= _heapCount)
                {
                    break;
                }

                int smallest = IsBefore(left, index) ? left : index;
                int right = left + 1;
                if (right < _heapCount && IsBefore(right, smallest))
                {
                    smallest = right;
                }

                if (smallest == index)
                {
                    break;
                }

                Swap(index, smallest);
                index = smallest;
            }
        }

        /// <summary>Orders heap entries by arrival tick, then by insertion order for exact ties.</summary>
        private bool IsBefore(int a, int b)
        {
            long arrivalA = _heapArrivalTicks[a];
            long arrivalB = _heapArrivalTicks[b];
            if (arrivalA != arrivalB)
            {
                return arrivalA < arrivalB;
            }

            return _heapSequence[a] < _heapSequence[b];
        }

        private void Swap(int a, int b)
        {
            long arrival = _heapArrivalTicks[a];
            _heapArrivalTicks[a] = _heapArrivalTicks[b];
            _heapArrivalTicks[b] = arrival;

            long sequence = _heapSequence[a];
            _heapSequence[a] = _heapSequence[b];
            _heapSequence[b] = sequence;

            int slot = _heapSlots[a];
            _heapSlots[a] = _heapSlots[b];
            _heapSlots[b] = slot;
        }

        /// <summary>
        /// Rejects a profile that cannot be honoured, at construction rather than per datagram.
        /// A negative delay or an out-of-range probability is a configuration mistake, and one that
        /// would otherwise show up as a subtly wrong distribution in a result rather than as a
        /// failure.
        /// </summary>
        private static void ValidateProfile(NetworkProfile profile)
        {
            if (profile.BaseDelayTicks < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(profile), profile.BaseDelayTicks, "BaseDelayTicks must not be negative.");
            }

            if (profile.JitterTicks < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(profile), profile.JitterTicks, "JitterTicks must not be negative.");
            }

            // The uniform draw spans 2*JitterTicks+1 values; keep that inside long range.
            if (profile.JitterTicks > long.MaxValue / 4)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(profile), profile.JitterTicks, "JitterTicks is implausibly large.");
            }

            if (profile.ReorderDelayTicks < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(profile), profile.ReorderDelayTicks, "ReorderDelayTicks must not be negative.");
            }

            ValidateProbability(profile.LossProbabilityAfterDelivered, "LossProbabilityAfterDelivered");
            ValidateProbability(profile.LossProbabilityAfterLost, "LossProbabilityAfterLost");
            ValidateProbability(profile.ReorderProbability, "ReorderProbability");
        }

        private static void ValidateProbability(double value, string name)
        {
            // The NaN case is written as a failed in-range test rather than double.IsNaN so that a
            // NaN cannot slip through as "not less than 0 and not greater than 1".
            if (!(value >= 0.0 && value <= 1.0))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(NetworkProfile), value, name + " must be in [0, 1].");
            }
        }
    }
}
