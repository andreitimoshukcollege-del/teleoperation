using System;
using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// A <see cref="ITransport"/> <b>decorator</b> that models a finite-rate bottleneck link with a
    /// finite FIFO queue in front of it: the missing piece that makes sender-side send-rate
    /// behaviour a measurable question on this platform rather than a no-op by construction.
    ///
    /// A datagram handed to <see cref="Send"/> is not forwarded immediately. It joins a queue and is
    /// handed to the wrapped transport at its <i>departure</i> tick — the instant the link has
    /// finished serializing it:
    ///
    /// <code>
    /// departure[i] = max(nowTicks[i], departure[i-1]) + length[i] * ticksPerSecond / capacityBytesPerSecond
    /// </code>
    ///
    /// so the queuing delay a datagram suffers is exactly the backlog ahead of it. When the queue is
    /// full the datagram is refused: <see cref="Send"/> returns false and nothing is forwarded. That
    /// is <b>tail drop</b>, and it is the case <see cref="ITransport.Send"/> already documents
    /// verbatim — "returns false when the datagram will not be delivered — emulated loss, or a full
    /// send queue". Refusing inside <c>Send</c>, before anything reaches the wrapped transport, is
    /// also what keeps this honest over a real socket: a datagram the model says was dropped never
    /// puts bytes on the wire, the same property <c>docs/adr/0013</c> protects for loss.
    ///
    /// <b>Why this is a transport and not an <see cref="INetworkImpairment"/>.</b> A queue's output
    /// depends on how much wall time elapsed since the previous datagram and on how many bytes that
    /// datagram was. <c>Types/DatagramFate.cs</c> deliberately carries neither, and
    /// <see cref="INetworkImpairment.ApplyOnSend"/>/<see cref="INetworkImpairment.ApplyOnDeliver"/>
    /// take no time parameter — while <see cref="ITransport"/> hands over <c>nowTicks</c> and
    /// <c>payload.Length</c> on every call already. Every impairment shipped today is memoryless in
    /// time (a constant, or one draw, or a Markov bit, or a per-datagram trace cursor); this is the
    /// first model that is not, which is why it belongs at a different seam. Widening
    /// <c>DatagramFate</c> would additionally have broken the order-independence rule that makes
    /// permuting an impairment array a non-change, because a queue's backlog depends on which
    /// datagrams an earlier loss axis let through.
    ///
    /// <b>Intended composition</b>, which is also the physically correct ordering — the access link
    /// serializes and queues, then the rest of the path adds propagation delay, jitter and loss:
    ///
    /// <code>
    /// new EmulatedTransport(new BottleneckTransport(new LoopbackTransport(...), ...), impairments, seed, maxInFlight)
    /// </code>
    ///
    /// <see cref="EmulatedTransport"/> needs no knowledge of this class: it already promises its
    /// impairment is "additive over the wrapped transport", and this transport reports each
    /// datagram's departure tick as its arrival tick, so the emulator's delay lands on top of the
    /// queuing delay exactly as a real path's would.
    ///
    /// <b>The queue drains on every call, not only on poll.</b> Both <see cref="Send"/> and
    /// <see cref="TryReceive"/> advance the departure schedule to <c>nowTicks</c> first. Draining
    /// only at poll time would make a datagram's acceptance depend on the host's polling schedule
    /// rather than on the link, which is the same class of error as folding poll time into measured
    /// one-way delay.
    ///
    /// <b>The buffer is measured in datagrams, not bytes.</b> That is not a simplification for
    /// preallocation's sake: Linux's own <c>txqueuelen</c> and most NIC rings are packet-limited,
    /// and for this project's fixed-size command stream the two limits coincide anyway. A
    /// byte-limited variant would be a different class, not a parameter on this one.
    ///
    /// No randomness — this model is fully deterministic given its arrival stream, so it consumes no
    /// RNG and takes no seed. Everything is preallocated in the constructor; <see cref="Send"/> and
    /// <see cref="TryReceive"/> allocate nothing. Not thread-safe, by contract. Time is always a
    /// parameter, never a clock read.
    /// </summary>
    public sealed class BottleneckTransport : ITransport
    {
        private readonly ITransport _inner;
        private readonly long _capacityBytesPerSecond;
        private readonly long _ticksPerSecond;
        private readonly int _bufferDatagrams;
        private readonly int _maxPayloadBytes;

        // Queue storage. A queued datagram's bytes live in exactly one slot until it departs.
        private readonly byte[] _payloads;
        private readonly int[] _lengths;
        private readonly long[] _departureTicks;

        private int _head;
        private int _count;

        /// <summary>
        /// Departure tick of the most recently *scheduled* datagram, which is when the link finishes
        /// serializing it. <see cref="NoPreviousDeparture"/> until the first send, so that the first
        /// datagram is serialized starting at its own arrival rather than at tick zero.
        /// </summary>
        private long _lastScheduledDeparture;

        private const long NoPreviousDeparture = long.MinValue;

        /// <param name="inner">
        /// Transport to decorate. Its own transit delay and losses stand; this decorator adds
        /// serialization and queuing delay in front of them.
        /// </param>
        /// <param name="capacityBytesPerSecond">
        /// Link rate available to this flow, in bytes per second. This is deliberately the
        /// *residual* capacity — what is left for this stream after whatever else shares the link —
        /// because nothing in this project models cross traffic, and a bottleneck below a 73-byte /
        /// 10 ms command stream's 7300 B/s only makes sense as a share of a contended link.
        /// </param>
        /// <param name="bufferDatagrams">
        /// Queue depth. The datagram currently being serialized counts against it, so the worst-case
        /// queuing delay this transport can impose is
        /// <c>bufferDatagrams * maxPayloadBytes / capacityBytesPerSecond</c> seconds.
        /// </param>
        /// <param name="ticksPerSecond">
        /// The host's tick rate, used only to convert bytes-per-second into ticks-per-byte. A
        /// constant, not a clock read — the same way <c>RigidBodyPlant</c> takes it.
        /// </param>
        public BottleneckTransport(
            ITransport inner, long capacityBytesPerSecond, int bufferDatagrams, long ticksPerSecond)
        {
            if (inner == null)
            {
                throw new ArgumentNullException(nameof(inner));
            }

            if (capacityBytesPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(capacityBytesPerSecond), capacityBytesPerSecond,
                    "Link capacity must be positive.");
            }

            if (bufferDatagrams <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(bufferDatagrams), bufferDatagrams, "Buffer depth must be positive.");
            }

            if (ticksPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(ticksPerSecond), ticksPerSecond, "Tick rate must be positive.");
            }

            _maxPayloadBytes = inner.MaxPayloadBytes;
            if (_maxPayloadBytes <= 0)
            {
                throw new ArgumentException(
                    "Wrapped transport reports a non-positive MaxPayloadBytes.", nameof(inner));
            }

            _inner = inner;
            _capacityBytesPerSecond = capacityBytesPerSecond;
            _bufferDatagrams = bufferDatagrams;
            _ticksPerSecond = ticksPerSecond;

            _payloads = new byte[checked(_maxPayloadBytes * bufferDatagrams)];
            _lengths = new int[bufferDatagrams];
            _departureTicks = new long[bufferDatagrams];

            ResetLocalState();
        }

        /// <inheritdoc/>
        public int MaxPayloadBytes => _inner.MaxPayloadBytes;

        /// <summary>Datagrams waiting in the bottleneck queue. For tests and diagnostics.</summary>
        public int QueuedCount => _count;

        /// <summary>Queue depth, fixed at construction.</summary>
        public int BufferDatagrams => _bufferDatagrams;

        /// <summary>Link rate in bytes per second, fixed at construction.</summary>
        public long CapacityBytesPerSecond => _capacityBytesPerSecond;

        /// <summary>
        /// Datagrams this transport handed to the wrapped transport that the wrapped transport
        /// refused, cumulative since the last <see cref="Reset"/>. Loss that belongs to the inner
        /// transport, not to this model — see <see cref="AdvanceTo"/>. Exists so an experiment can
        /// assert it is zero and therefore that every drop it measured was this model's tail drop.
        /// </summary>
        public long InnerRefusedCount { get; private set; }

        /// <summary>
        /// Serialization time of <paramref name="lengthBytes"/> on this link, in ticks. Integer
        /// division truncates, so the link is modelled at most one tick per datagram faster than its
        /// nominal rate — 100 ns against a 10 ms cadence at this project's 10 MHz timebase. Stated
        /// rather than corrected: a residual accumulator would add reset-visible state for a bias
        /// four orders of magnitude below anything measured here.
        /// </summary>
        public long SerializationTicks(int lengthBytes) =>
            lengthBytes * _ticksPerSecond / _capacityBytesPerSecond;

        /// <summary>
        /// Enqueues a datagram behind whatever backlog exists, or refuses it if the queue is full.
        ///
        /// Advances the departure schedule to <paramref name="nowTicks"/> first, so fullness is
        /// judged against the queue's true occupancy at this instant and not against whatever it
        /// held when the caller last polled.
        ///
        /// Returns false on a full queue (tail drop) and on an oversized payload — both the ordinary
        /// non-delivery outcome of <see cref="ITransport.Send"/>, and the caller must not retry.
        /// Allocation-free.
        /// </summary>
        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            AdvanceTo(nowTicks);

            if (payload.Length > _maxPayloadBytes)
            {
                return false;
            }

            if (_count == _bufferDatagrams)
            {
                return false;
            }

            long serviceStart = _lastScheduledDeparture == NoPreviousDeparture || _lastScheduledDeparture < nowTicks
                ? nowTicks
                : _lastScheduledDeparture;
            long departure = serviceStart + SerializationTicks(payload.Length);

            int slot = _head + _count;
            if (slot >= _bufferDatagrams)
            {
                slot -= _bufferDatagrams;
            }

            payload.CopyTo(new Span<byte>(_payloads, slot * _maxPayloadBytes, _maxPayloadBytes));
            _lengths[slot] = payload.Length;
            _departureTicks[slot] = departure;
            _count++;
            _lastScheduledDeparture = departure;
            return true;
        }

        /// <summary>
        /// Releases everything the link has finished serializing at or before
        /// <paramref name="nowTicks"/> into the wrapped transport, then returns whatever the wrapped
        /// transport has ready.
        ///
        /// <paramref name="arrivalTicks"/> therefore comes from the wrapped transport, which was
        /// handed each datagram stamped with its departure tick — so over a loopback the reported
        /// arrival <i>is</i> the departure, and over a delaying transport the two add. Allocation-free.
        /// </summary>
        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            AdvanceTo(nowTicks);
            return _inner.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks);
        }

        /// <summary>
        /// Empties the queue, clears the departure schedule, and resets the wrapped transport —
        /// a decorator resets what it wraps, for the same reason <see cref="EmulatedTransport"/>
        /// does: a previous trial's backlog would surface in the next one as spurious delay.
        /// This transport owns no RNG, so there is nothing to reseed.
        /// </summary>
        public void Reset()
        {
            ResetLocalState();
            _inner.Reset();
        }

        private void ResetLocalState()
        {
            Array.Clear(_payloads, 0, _payloads.Length);
            Array.Clear(_lengths, 0, _lengths.Length);
            Array.Clear(_departureTicks, 0, _departureTicks.Length);
            _head = 0;
            _count = 0;
            _lastScheduledDeparture = NoPreviousDeparture;
            InnerRefusedCount = 0;
        }

        /// <summary>
        /// Hands every datagram whose departure has passed to the wrapped transport, oldest first.
        ///
        /// <b>A datagram the wrapped transport refuses is discarded, not retried</b>, and counted in
        /// <see cref="InnerRefusedCount"/>. That is forced by <see cref="ITransport.Send"/>'s own
        /// contract — "callers must not retry on it or the loss model stops being the loss model" —
        /// and it is worth flagging that the contract's <c>false</c> is overloaded: it means both "I
        /// lost this" and "I have no room right now", and a queueing decorator has no way to tell
        /// them apart. Retrying would silently un-lose a datagram an inner loss model discarded;
        /// discarding attributes an inner back-pressure event as loss. The contract picks the
        /// second, so this does too, and the counter exists so an experiment can prove the case
        /// never arose rather than assume it.
        ///
        /// Note this differs from <see cref="EmulatedTransport"/>'s back-pressure, which is not the
        /// same situation: that class declines to *call* <c>TryReceive</c> when it is full, so it
        /// never has a refused <c>Send</c> to interpret.
        /// </summary>
        private void AdvanceTo(long nowTicks)
        {
            while (_count > 0)
            {
                long departure = _departureTicks[_head];
                if (departure > nowTicks)
                {
                    return;
                }

                int length = _lengths[_head];
                var payload = new ReadOnlySpan<byte>(_payloads, _head * _maxPayloadBytes, length);

                // Stamped with the departure tick, not nowTicks: the link finished transmitting it
                // when it finished, and reporting the poll time instead would fold the host's frame
                // rate into measured one-way delay.
                if (!_inner.Send(payload, departure))
                {
                    InnerRefusedCount++;
                }

                _lengths[_head] = 0;
                _departureTicks[_head] = 0;
                _head++;
                if (_head == _bufferDatagrams)
                {
                    _head = 0;
                }

                _count--;
            }
        }
    }
}
