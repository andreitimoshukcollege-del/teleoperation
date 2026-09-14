using System;
using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// An <see cref="ITransport"/> <b>decorator</b> that observes the link it wraps and changes
    /// nothing about it. Every call is forwarded verbatim; the return value, the payload, the
    /// arrival tick and the too-short-buffer protocol are the wrapped transport's, untouched. Its
    /// only effect is a stream of <c>docs/metrics.md</c> §3 samples pushed to the
    /// <see cref="NetworkObserver"/> it was given.
    ///
    /// <b>Where it goes in the stack decides what it measures, and there is one right answer.</b>
    /// Wrap it <i>outside</i> <c>EmulatedTransport</c>. There, <see cref="Send"/> returning false
    /// is exactly "this datagram will not be delivered" as <see cref="ITransport.Send"/> defines
    /// it — the union of emulated loss and the wrapped stack's own back-pressure, which is what
    /// §3's "fraction of sent datagrams never received" means for a sender. Wrapped
    /// <i>inside</i> the emulator it would see only the loopback's queue and never see a lost
    /// datagram at all, because a datagram the loss impairment drops is never handed down.
    ///
    /// <b>One instance measures one direction, from both ends.</b> That is not a compromise: in
    /// this pipeline a single <see cref="ITransport"/> object <i>is</i> one direction of the link —
    /// the operator endpoint calls <see cref="Send"/> on the uplink and the robot endpoint calls
    /// <see cref="TryReceive"/> on that same object. So one decorator sees both the sender vantage
    /// (refusals) and the receiver vantage (deliveries) of that direction, and the difference
    /// between the two counts is loss inflicted after acceptance, which no shipped impairment
    /// models.
    ///
    /// <b>What it cannot see, by construction.</b> Sequence numbers and sender timestamps: this
    /// class handles opaque bytes and is forbidden from parsing them — a transport that decoded
    /// payloads would be a codec, would have to be updated for every future codec, and would break
    /// the moment it wrapped a link carrying something else. Reordering and interarrival jitter
    /// therefore cannot be measured here and are fed to <see cref="NetworkObserver"/> from a
    /// pipeline endpoint instead. See that type's doc for the full vantage argument.
    ///
    /// Allocation-free; holds no buffers of its own because it never copies a payload.
    /// Not thread-safe, by the same contract as the rest of <c>Transport/</c>.
    /// </summary>
    public sealed class MeasuredTransport : ITransport
    {
        private readonly ITransport _inner;
        private readonly NetworkObserver _observer;

        /// <param name="inner">The transport to observe. Its behaviour is not modified in any way.</param>
        /// <param name="observer">
        /// Where the §3 samples go. Built by <see cref="NetworkObserver.ForUplink"/> or
        /// <see cref="NetworkObserver.ForDownlink"/>, which is what fixes the direction prefix on
        /// every metric name; this class holds no names of its own and so cannot disagree with
        /// <c>docs/metrics.md</c>.
        /// </param>
        public MeasuredTransport(ITransport inner, NetworkObserver observer)
        {
            if (inner == null)
            {
                throw new ArgumentNullException(nameof(inner));
            }

            if (observer == null)
            {
                throw new ArgumentNullException(nameof(observer));
            }

            _inner = inner;
            _observer = observer;
        }

        /// <inheritdoc/>
        public int MaxPayloadBytes => _inner.MaxPayloadBytes;

        /// <summary>
        /// Forwards the send and records its outcome. The return value is the wrapped transport's,
        /// unchanged — in particular a refusal is still an ordinary outcome and this class never
        /// retries, which would make the loss model stop being the loss model.
        /// </summary>
        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            bool accepted = _inner.Send(payload, nowTicks);
            _observer.OnSendResult(accepted, nowTicks);
            return accepted;
        }

        /// <summary>
        /// Forwards the receive and records a delivery only when one actually happened. A false
        /// return is not counted, which matters for the too-short-destination case: that returns
        /// false with <paramref name="byteCount"/> set to the required length and leaves the
        /// datagram queued, so counting it would count the same datagram again when the caller
        /// retries with a bigger buffer.
        ///
        /// The delivery is stamped at <paramref name="arrivalTicks"/>, the wrapped transport's
        /// <c>t_recv</c>, not at <paramref name="nowTicks"/>.
        /// </summary>
        public bool TryReceive(
            long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            bool received = _inner.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks);
            if (received)
            {
                _observer.OnDelivery(arrivalTicks);
            }

            return received;
        }

        /// <summary>
        /// Resets the observer and then the transport it wraps, because a decorator resets what it
        /// wraps. Resetting the observer is not optional bookkeeping: its sequence high-water mark
        /// and jitter estimator are per-link state, and carrying them into the next trial would
        /// make that trial's early samples depend on the previous seed's link.
        /// </summary>
        public void Reset()
        {
            _observer.Reset();
            _inner.Reset();
        }
    }
}
