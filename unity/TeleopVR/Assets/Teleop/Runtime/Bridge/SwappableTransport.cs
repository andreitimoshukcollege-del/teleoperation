using System;
using Teleop.Core.Contracts;
using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Bridge
{
    /// <summary>
    /// An <see cref="ITransport"/> whose impairment stage can be swapped while the session is
    /// running, so a checkbox can turn lag on and off without restarting Play mode.
    ///
    /// This is an adapter, not a transport: it owns no delay, loss or reorder behavior of its own
    /// and forwards every call unchanged. The impairment is always
    /// <see cref="EmulatedTransport"/>'s, applied to a profile
    /// <see cref="NetworkImpairmentSettings"/> composed. What this class adds is one level of
    /// indirection, and it exists for a specific reason: <c>OperatorEndpoint</c> and
    /// <c>RobotEndpoint</c> take their transports once, in their constructors, and
    /// <see cref="EmulatedTransport"/> holds its profile in a readonly field. Without an
    /// indirection here, changing any impairment setting would mean rebuilding the entire endpoint
    /// stack -- discarding <c>ClockSync</c>'s convergence, the predictor's history, and the
    /// recording session along with it. Those are precisely the things an operator comparing
    /// "50ms" to "200ms" needs held constant.
    ///
    /// <b>Swapping drops whatever the old emulator was holding.</b> A datagram already inside
    /// <see cref="EmulatedTransport"/>'s delay heap is gone when that instance is replaced, so
    /// flipping a checkbox costs up to <c>maxInFlight</c> datagrams as a one-off. This is not
    /// silently absorbed: <see cref="LastSwapDiscardedCount"/> reports it and
    /// <see cref="NetworkImpairmentController"/> logs it. The alternative -- draining the old
    /// instance into the new one -- is not possible through <see cref="ITransport"/>, which by
    /// design exposes no way to enumerate what is in flight, and adding one would be a Core change
    /// to serve a Unity convenience.
    ///
    /// The inner transport itself is never swapped, which bounds the damage: datagrams sitting in
    /// the underlying <c>LoopbackTransport</c>/<c>UdpTransport</c> queue are untouched, and only
    /// the decorator's own held-back set is lost.
    /// </summary>
    public sealed class SwappableTransport : ITransport
    {
        private readonly ITransport _inner;
        private readonly int _maxInFlight;
        private EmulatedTransport _emulator;

        /// <param name="inner">The real channel. Kept for the lifetime of this object and never replaced.</param>
        /// <param name="maxInFlight">
        /// Passed straight through to <see cref="EmulatedTransport"/>. Also the upper bound on how
        /// many datagrams a single swap can discard.
        /// </param>
        public SwappableTransport(ITransport inner, int maxInFlight)
        {
            _inner = inner ?? throw new ArgumentNullException(nameof(inner));

            if (maxInFlight <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(maxInFlight), maxInFlight, "In-flight capacity must be positive.");
            }

            _maxInFlight = maxInFlight;
        }

        /// <summary>Whether an emulator is currently installed. False means datagrams reach <c>inner</c> untouched.</summary>
        public bool IsImpaired => _emulator != null;

        /// <summary>
        /// How many in-flight datagrams the most recent <see cref="Install"/>/<see cref="Remove"/>
        /// threw away. Zero until the first swap.
        /// </summary>
        public int LastSwapDiscardedCount { get; private set; }

        /// <summary>
        /// Unchanged across swaps by construction: <see cref="EmulatedTransport"/> reports its
        /// wrapped transport's value, so this is always <c>inner</c>'s. Callers that sized a buffer
        /// from this before a swap stay correct after one.
        /// </summary>
        public int MaxPayloadBytes => _inner.MaxPayloadBytes;

        /// <summary>
        /// Replaces the impairment stage with one built from <paramref name="profile"/>. Safe to
        /// call when already impaired -- the previous emulator is discarded.
        /// </summary>
        /// <param name="seed">
        /// Seeds this direction's impairment stream. Give the two directions of a link *different*
        /// seeds: an identical seed makes uplink and downlink drop the same datagram indices in
        /// lockstep, which no real link does and which would make a loss result meaningless.
        /// </param>
        public void Install(NetworkProfile profile, ulong seed)
        {
            LastSwapDiscardedCount = _emulator?.InFlightCount ?? 0;
            _emulator = new EmulatedTransport(_inner, profile, new SeededRng(seed), _maxInFlight);
        }

        /// <summary>
        /// Installs a <b>trace-driven</b> impairment stage: delay comes from
        /// <paramref name="delayTraceTicks"/>, consumed in order and wrapped when exhausted, while
        /// <paramref name="profile"/> still supplies loss and reordering.
        ///
        /// <paramref name="profile"/> must have zero base delay and zero jitter --
        /// <see cref="EmulatedTransport"/> rejects anything else, on the grounds that synthetic
        /// jitter layered on an already-recorded delay double-models the same variance.
        /// <see cref="NetworkImpairmentSettings.ToProfile"/> zeroes both when trace mode is on, so
        /// callers coming from there satisfy this by construction.
        ///
        /// The samples must already be in this host's tick domain; see
        /// <see cref="DelayTraceLoader.TryLoad"/>, which rescales them and explains why skipping
        /// that step produces a 100x error that still looks plausible.
        /// </summary>
        public void Install(long[] delayTraceTicks, NetworkProfile profile, ulong seed)
        {
            LastSwapDiscardedCount = _emulator?.InFlightCount ?? 0;
            _emulator = new EmulatedTransport(
                _inner, delayTraceTicks, profile, new SeededRng(seed), _maxInFlight);
        }

        /// <summary>Removes the impairment stage, restoring the bare inner transport.</summary>
        public void Remove()
        {
            LastSwapDiscardedCount = _emulator?.InFlightCount ?? 0;
            _emulator = null;
        }

        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            return _emulator != null
                ? _emulator.Send(payload, nowTicks)
                : _inner.Send(payload, nowTicks);
        }

        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            return _emulator != null
                ? _emulator.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks)
                : _inner.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks);
        }

        /// <summary>
        /// Clears runtime state and <b>keeps the configured impairment</b>: delegated to the
        /// installed emulator, which per <see cref="EmulatedTransport.Reset"/> empties its in-flight
        /// set, returns the Gilbert-Elliott chain to the good state, reseeds its RNG to the
        /// construction seed, and resets what it wraps.
        ///
        /// Keeping the emulator installed is the deliberate reading of
        /// <see cref="ITransport.Reset"/>'s "as-constructed state" here. That clause exists so a
        /// sweep can reuse an instance across trials and have the next trial reproduce the previous
        /// one -- which requires the impairment to *survive*, not to be torn off. Removing the
        /// emulator would also silently switch off the operator's checkboxes, which no caller of
        /// Reset could possibly intend. What Reset clears is state; what
        /// <see cref="Install"/>/<see cref="Remove"/> change is configuration.
        /// </summary>
        public void Reset()
        {
            LastSwapDiscardedCount = 0;

            if (_emulator != null)
            {
                _emulator.Reset();
            }
            else
            {
                _inner.Reset();
            }
        }
    }
}
