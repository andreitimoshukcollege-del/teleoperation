using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// A <see cref="ITransport"/> <b>decorator</b> that injects a reproducible synthetic impairment
    /// on top of whatever transport it wraps. Most of this project's research runs through it.
    /// Wrapping <see cref="LoopbackTransport"/> gives a fully synthetic link for headless
    /// evaluation; wrapping the host's <c>Bridge/UdpTransport.cs</c> on a LAN gives a reproducible
    /// impairment layered over a real socket. Impairment is always <i>additive</i>: the wrapped
    /// transport's own transit delay and its own losses stand, and this decorator's model is applied
    /// on top rather than in place of them.
    ///
    /// <b>What impairment to apply is entirely the caller's, expressed as a set of
    /// <see cref="INetworkImpairment"/> objects</b> (docs/adr/0013). This class holds no impairment
    /// parameters of its own and knows nothing about profile names: it runs the send-stage
    /// impairments, asks the wrapped transport, runs the deliver-stage ones, and schedules the
    /// result. Adding a new kind of impairment is a new file implementing that interface, not an
    /// edit here. An empty set is legal and means an unimpaired decorator.
    ///
    /// All randomness comes from per-impairment substreams derived from the <c>seed</c> given here,
    /// so same seed plus same impairment set plus same call sequence produce a bit-identical
    /// sequence of receives — the property <c>Transport/CLAUDE.md</c> requires, and the reason
    /// nothing here reads a clock or calls <c>System.Random</c>. Because each impairment owns its
    /// stream, <b>adding or removing one leaves every other one's realization untouched</b>, which
    /// the previous single-shared-stream design could not offer.
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
    /// <b>Trace-driven replay is no longer a mode of this class.</b> It is
    /// <c>TraceDelayImpairment</c>, one member of the set like any other, so there is no second
    /// constructor and no cross-field validation. What used to be "a trace-mode profile must carry
    /// zero base delay and jitter, or we reject it" is now simply which impairments the caller
    /// chose to include: the composition is the configuration.
    ///
    /// <b>Two stages, because a datagram has two.</b> Send-stage impairments run inside
    /// <see cref="Send"/> and may drop, in which case the datagram never reaches the wrapped
    /// transport at all — what a real link does with a lost packet, and what
    /// <see cref="ITransport.Send"/>'s "returns false when the datagram will not be delivered"
    /// promises. Deliver-stage impairments run at drain time and may add delay. The set is
    /// partitioned by stage once, at construction, so an impairment costs nothing at a stage it did
    /// not declare.
    ///
    /// Everything is preallocated in the constructor; <see cref="Send"/> and
    /// <see cref="TryReceive"/> allocate nothing. Not thread-safe, by contract.
    /// </summary>
    public sealed class EmulatedTransport : ITransport
    {
        private readonly ITransport _inner;
        private readonly int _maxInFlight;

        /// <summary>
        /// The impairment set, in caller order, cloned so a caller mutating its array afterwards
        /// cannot change this transport's behaviour mid-run. Held as a concrete array and iterated
        /// with an indexed <c>for</c>, never as an interface-typed sequence with <c>foreach</c>:
        /// that would box an enumerator on <b>every datagram</b> and break the allocation tests
        /// this class is required to pass.
        /// </summary>
        private readonly INetworkImpairment[] _impairments;

        /// <summary>Indices into <see cref="_impairments"/> declaring <c>ImpairmentStages.Send</c>.</summary>
        private readonly int[] _sendStage;

        /// <summary>Indices into <see cref="_impairments"/> declaring <c>ImpairmentStages.Deliver</c>.</summary>
        private readonly int[] _deliverStage;

        /// <summary>
        /// Cached at construction rather than read per drain. <c>ITransport.MaxPayloadBytes</c> is
        /// documented as a constant of the transport, and the slot arrays are sized from it, so
        /// re-reading it would only create a way for the two to disagree.
        /// </summary>
        private readonly int _innerMaxPayloadBytes;

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

        /// <param name="inner">Transport to decorate. Its delay and losses are kept, not replaced.</param>
        /// <param name="impairments">
        /// The impairment set, applied in the order given. May be empty, which yields an unimpaired
        /// decorator -- useful as a structural baseline that still exercises this class's scheduling.
        /// Cloned defensively, and each element is bound to this transport: an instance may belong to
        /// exactly one transport, because it carries mutable model state and one RNG substream.
        ///
        /// Two impairments may not share an <c>AxisName</c>. That is what makes name-derived
        /// substream identity collision-proof: two axes silently drawing the same numbers would
        /// correlate, say, loss with jitter, and nothing in the source would look wrong.
        /// </param>
        /// <param name="seed">
        /// Trial seed. Each impairment gets its own substream derived from this and its axis name
        /// (<see cref="Impairments.ImpairmentStreams"/>), so adding or removing an impairment cannot
        /// perturb any other one's realization.
        /// </param>
        /// <param name="maxInFlight">
        /// Number of delayed datagrams held at once. When full, the wrapped transport is simply not
        /// drained, so its datagrams stay queued there (back-pressure) rather than being silently
        /// destroyed by the emulator — an emulator that dropped them would add loss no impairment
        /// asked for, and which is therefore in no manifest.
        /// </param>
        public EmulatedTransport(
            ITransport inner, INetworkImpairment[] impairments, ulong seed, int maxInFlight)
        {
            if (inner == null)
            {
                throw new ArgumentNullException(nameof(inner));
            }

            if (impairments == null)
            {
                throw new ArgumentNullException(nameof(impairments));
            }

            if (maxInFlight <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(maxInFlight), maxInFlight, "In-flight capacity must be positive.");
            }

            _inner = inner;
            _maxInFlight = maxInFlight;
            _innerMaxPayloadBytes = inner.MaxPayloadBytes;

            if (_innerMaxPayloadBytes <= 0)
            {
                throw new ArgumentException(
                    "Wrapped transport reports a non-positive MaxPayloadBytes.", nameof(inner));
            }

            _impairments = (INetworkImpairment[])impairments.Clone();

            int sendCount = 0;
            int deliverCount = 0;
            for (int i = 0; i < _impairments.Length; i++)
            {
                INetworkImpairment impairment = _impairments[i];
                if (impairment == null)
                {
                    throw new ArgumentException(
                        $"Impairment at index {i} is null.", nameof(impairments));
                }

                for (int j = 0; j < i; j++)
                {
                    if (string.Equals(_impairments[j].AxisName, impairment.AxisName, StringComparison.Ordinal))
                    {
                        throw new ArgumentException(
                            $"Two impairments share the axis name '{impairment.AxisName}'. Axis names " +
                            "must be unique -- they derive the RNG substreams, so a duplicate would " +
                            "make two axes draw identical numbers.",
                            nameof(impairments));
                    }
                }

                if ((impairment.Stages & ImpairmentStages.Send) != 0)
                {
                    sendCount++;
                }

                if ((impairment.Stages & ImpairmentStages.Deliver) != 0)
                {
                    deliverCount++;
                }
            }

            // Partitioned once, here, so an impairment costs nothing at a stage it did not declare
            // and neither loop has to test a flag per datagram.
            _sendStage = new int[sendCount];
            _deliverStage = new int[deliverCount];

            int sendAt = 0;
            int deliverAt = 0;
            for (int i = 0; i < _impairments.Length; i++)
            {
                ImpairmentStages stages = _impairments[i].Stages;

                if ((stages & ImpairmentStages.Send) != 0)
                {
                    _sendStage[sendAt++] = i;
                }

                if ((stages & ImpairmentStages.Deliver) != 0)
                {
                    _deliverStage[deliverAt++] = i;
                }

                _impairments[i].Bind(
                    Impairments.ImpairmentStreams.DeriveForAxis(seed, _impairments[i].AxisName));
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
            var fate = default(DatagramFate);
            for (int i = 0; i < _sendStage.Length; i++)
            {
                _impairments[_sendStage[i]].ApplyOnSend(ref fate);
            }

            if (fate.Dropped)
            {
                // Never reaches the wrapped transport: a real link does not put a dropped packet on
                // the wire, and over a real socket this is the difference between modelling loss and
                // transmitting bytes the model says were lost.
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
        /// Returns this decorator, every impairment it owns, and the transport it wraps to their
        /// as-constructed state: nothing in flight, the tie-break counter back to zero, and each
        /// impairment's model state cleared and substream reseeded so the next trial reproduces the
        /// previous one — all three requirements of <see cref="ITransport.Reset"/>.
        ///
        /// Each impairment resets itself, which is why this method no longer knows what a
        /// Gilbert-Elliott chain or a trace cursor is. That also makes "returns to as-constructed
        /// state" testable per impairment, and therefore actually exhaustive, instead of resting on
        /// one schedule-replay test hoping it covered every field.
        ///
        /// <c>_inner.Reset()</c> is called because a decorator resets what it wraps; leaving the
        /// wrapped transport holding a previous trial's datagrams would contaminate the next trial
        /// in a way that looks like spurious loss. Impairments stay bound — this is a reset, not a
        /// teardown.
        /// </summary>
        public void Reset()
        {
            ResetLocalState();

            for (int i = 0; i < _impairments.Length; i++)
            {
                _impairments[i].Reset();
            }

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

                var fate = default(DatagramFate);
                for (int i = 0; i < _deliverStage.Length; i++)
                {
                    _impairments[_deliverStage[i]].ApplyOnDeliver(ref fate);
                }

                if (fate.Dropped)
                {
                    // No shipped impairment sets Dropped at this stage -- loss belongs at send, where
                    // the datagram never reaches the wire. Honoured anyway rather than ignored,
                    // because silently discarding a decision an impairment made would be a trap for
                    // whoever writes the first receiver-side model (a late-arrival discard, say).
                    // Note it does NOT unsend: the wrapped transport already carried the bytes.
                    _slotLengths[slot] = 0;
                    _freeSlots[_freeCount] = slot;
                    _freeCount++;
                    continue;
                }

                // Clamped once, after the whole pipeline, so a jitter draw that ran before any delay
                // was added cannot make a datagram arrive before it was sent.
                long delayTicks = fate.DelayTicks < 0 ? 0 : fate.DelayTicks;
                HeapPush(innerArrivalTicks + delayTicks, slot);
            }
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
    }
}
