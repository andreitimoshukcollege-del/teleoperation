using System;
using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// Closed-loop sender-side admission control driven by the <b>only</b> congestion signal a
    /// transport decorator can actually see: the wrapped transport returning false.
    ///
    /// <b>The signal, and its poverty.</b> The operator's uplink transport is write-only from Core's
    /// point of view — <c>OperatorEndpoint</c> calls <c>Send</c> on it and never receives on it, so a
    /// decorator here observes exactly one bit per datagram: accepted, or refused. It cannot see
    /// <c>owd_uplink_ms</c>; that is computed in <c>OperatorEndpoint</c> from the returned
    /// <c>RobotStateFrame</c>, one round trip later, and reaching it from here would need a new
    /// <c>Contracts/</c> interface and a <c>Pipeline/</c> change. This class is deliberately the
    /// best thing buildable *without* that, so that the gap between it and
    /// <see cref="RateLimitedTransport"/> measures what the missing signal is worth.
    ///
    /// <b>Algorithm — AIMD on the admission gap.</b> A gap of <c>N</c> means "refuse the next
    /// <c>N</c> offers after each one I forward", so the admitted fraction is <c>1/(N+1)</c> of the
    /// host's offered load. On a refusal from the link the gap grows multiplicatively
    /// (<c>2N+1</c>, i.e. the admitted rate roughly halves); after a run of successes it shrinks by
    /// one (the admitted rate creeps back up). Multiplicative decrease on the rate and additive
    /// increase is the standard shape, chosen here for the standard reason — it backs off fast
    /// enough to stop congestion collapse and probes back slowly enough not to re-cause it.
    ///
    /// <b>It counts offers, not seconds, and that is a real limitation.</b> With only a success/fail
    /// bit and no notion of how much time or how many bytes went by, this controller can express a
    /// *ratio* of the host's offered load and nothing else. It cannot target an absolute rate, so it
    /// cannot know whether it has undershot the link or merely matched it. That distinction is
    /// exactly what decides whether a standing queue drains, which is why this class exists as a
    /// measurement rather than as a recommendation.
    ///
    /// No randomness, no allocation, not thread-safe. Time is a parameter and is passed through
    /// untouched — this class makes no decision that depends on it, which is itself the finding.
    /// </summary>
    public sealed class BacklogBackoffTransport : ITransport
    {
        private readonly ITransport _inner;
        private readonly int _maxGap;
        private readonly int _successesBeforeDecrease;

        private int _gap;
        private int _pendingRefusals;
        private int _successRun;

        /// <param name="inner">Transport to decorate; usually a <see cref="BottleneckTransport"/>.</param>
        /// <param name="maxGap">
        /// Largest admission gap, i.e. the floor on admitted rate is <c>1/(maxGap+1)</c> of offered.
        /// A bound rather than a tuning knob: without it a run of refusals from an unrelated cause
        /// (a lossy link, say — which looks identical through this signal) would ratchet the sender
        /// to silence, which is the failure RFC 8083's circuit breakers exist to make deliberate
        /// rather than accidental.
        /// </param>
        /// <param name="successesBeforeDecrease">
        /// Consecutive forwarded-and-accepted datagrams before the gap shrinks by one. Larger is
        /// slower to recover and less likely to oscillate.
        /// </param>
        public BacklogBackoffTransport(ITransport inner, int maxGap, int successesBeforeDecrease)
        {
            if (inner == null)
            {
                throw new ArgumentNullException(nameof(inner));
            }

            if (maxGap < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(maxGap), maxGap, "Maximum gap must not be negative.");
            }

            if (successesBeforeDecrease <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(successesBeforeDecrease), successesBeforeDecrease,
                    "Successes before decrease must be positive.");
            }

            _inner = inner;
            _maxGap = maxGap;
            _successesBeforeDecrease = successesBeforeDecrease;
        }

        /// <inheritdoc/>
        public int MaxPayloadBytes => _inner.MaxPayloadBytes;

        /// <summary>Current admission gap: this sender forwards one offer in <c>Gap + 1</c>.</summary>
        public int Gap => _gap;

        /// <summary>
        /// Datagrams this sender refused itself, cumulative since the last <see cref="Reset"/>.
        /// Excludes datagrams it forwarded that the link then refused — those are the link's loss,
        /// not this controller's cost, and conflating them would let the mechanism hide its own bill.
        /// </summary>
        public long RefusedCount { get; private set; }

        /// <summary>
        /// Either withholds this datagram (returning false without forwarding) or forwards it and
        /// updates the controller from the outcome. Allocation-free.
        /// </summary>
        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            if (_pendingRefusals > 0)
            {
                _pendingRefusals--;
                RefusedCount++;
                return false;
            }

            bool accepted = _inner.Send(payload, nowTicks);

            if (accepted)
            {
                _successRun++;
                if (_successRun >= _successesBeforeDecrease)
                {
                    _successRun = 0;
                    if (_gap > 0)
                    {
                        _gap--;
                    }
                }
            }
            else
            {
                _successRun = 0;
                long grown = (2L * _gap) + 1L;
                _gap = grown > _maxGap ? _maxGap : (int)grown;
            }

            _pendingRefusals = _gap;
            return accepted;
        }

        /// <inheritdoc/>
        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks) =>
            _inner.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks);

        /// <summary>
        /// Returns the controller to its as-constructed state (gap zero — this sender starts
        /// greedy and learns) and resets the wrapped transport.
        /// </summary>
        public void Reset()
        {
            _gap = 0;
            _pendingRefusals = 0;
            _successRun = 0;
            RefusedCount = 0;
            _inner.Reset();
        }
    }
}
