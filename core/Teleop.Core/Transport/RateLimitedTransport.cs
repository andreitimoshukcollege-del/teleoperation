using System;
using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// Sender-side <b>admission control</b>: a <see cref="ITransport"/> decorator holding a token
    /// bucket that refuses datagrams the configured rate cannot carry, instead of handing them to a
    /// link that will queue them.
    ///
    /// <b>Why this can exist at all, given Core may not own a loop.</b> Cadence is the host's —
    /// <c>Pipeline/CLAUDE.md</c> is explicit that Core does not drive the step rate, and nothing
    /// here tries to. What this class owns is a different decision: given that the host has decided
    /// to send *now*, is it worth putting this datagram on a link that is already backed up? That
    /// decision is a per-call one, and <see cref="ITransport.Send"/> already models its outcome —
    /// "returns false when the datagram will not be delivered — emulated loss, or a full send
    /// queue... callers must not retry on it". A caller cannot distinguish a refusal here from a
    /// link that lost the datagram, and must not try to.
    ///
    /// <b>What it is for.</b> Against a <see cref="BottleneckTransport"/> whose capacity is below
    /// the offered load, the greedy sender's excess datagrams are tail-dropped anyway — so the
    /// excess buys no extra delivered commands, and pays for that with a standing queue that delays
    /// every command that *is* delivered. Admitting only what the link can carry is meant to deliver
    /// the same number of commands with far less of that self-inflicted queuing delay. This is
    /// open-loop: the rate is configured, not learned, so it is the *ceiling* on what any sender-side
    /// controller could achieve, and it is a fair mechanism only when someone knows the link rate.
    /// <see cref="BacklogBackoffTransport"/> is the closed-loop counterpart that has to find it.
    ///
    /// <b>Bucket arithmetic is in ticks, not bytes.</b> Credit is stored as ticks of link time and a
    /// datagram costs its serialization time, which makes refill exact integer arithmetic
    /// (<c>credit += elapsedTicks</c>) with no accumulating rounding drift. Storing bytes and
    /// refilling by <c>elapsed * rate / ticksPerSecond</c> truncates on every call, which at a 10 ms
    /// cadence would systematically under-admit.
    ///
    /// No randomness, no allocation after construction, not thread-safe. Time is always a parameter,
    /// never a clock read.
    /// </summary>
    public sealed class RateLimitedTransport : ITransport
    {
        private readonly ITransport _inner;
        private readonly long _admitBytesPerSecond;
        private readonly long _ticksPerSecond;
        private readonly long _burstTicks;

        private long _creditTicks;
        private long _lastRefillTicks;
        private bool _started;

        /// <param name="inner">Transport to decorate; usually a <see cref="BottleneckTransport"/>.</param>
        /// <param name="admitBytesPerSecond">
        /// Sustained rate this sender allows itself, in bytes per second. Above the link's own
        /// capacity this class is a no-op; at or just below it, it is the mechanism.
        /// </param>
        /// <param name="burstBytes">
        /// Bucket depth. One datagram's worth is the minimum that admits anything at all; larger
        /// values let a quiet period be spent as a burst, which against a bottleneck is exactly the
        /// backlog this class exists to avoid — so the interesting settings are small.
        /// </param>
        /// <param name="ticksPerSecond">
        /// Host tick rate, used only to convert bytes per second into ticks per byte. A constant,
        /// not a clock read.
        /// </param>
        public RateLimitedTransport(
            ITransport inner, long admitBytesPerSecond, long burstBytes, long ticksPerSecond)
        {
            if (inner == null)
            {
                throw new ArgumentNullException(nameof(inner));
            }

            if (admitBytesPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(admitBytesPerSecond), admitBytesPerSecond, "Admit rate must be positive.");
            }

            if (burstBytes <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(burstBytes), burstBytes, "Burst depth must be positive.");
            }

            if (ticksPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(ticksPerSecond), ticksPerSecond, "Tick rate must be positive.");
            }

            _inner = inner;
            _admitBytesPerSecond = admitBytesPerSecond;
            _ticksPerSecond = ticksPerSecond;
            _burstTicks = checked(burstBytes * ticksPerSecond) / admitBytesPerSecond;

            if (_burstTicks <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(burstBytes), burstBytes,
                    "Burst depth rounds to zero ticks at this rate and tick resolution.");
            }

            ResetLocalState();
        }

        /// <inheritdoc/>
        public int MaxPayloadBytes => _inner.MaxPayloadBytes;

        /// <summary>Sustained admission rate in bytes per second, fixed at construction.</summary>
        public long AdmitBytesPerSecond => _admitBytesPerSecond;

        /// <summary>
        /// Datagrams this sender refused itself, cumulative since the last <see cref="Reset"/>.
        /// Diagnostics for tests: it is the cost side of this mechanism and must always be reported
        /// next to whatever delay it saved.
        /// </summary>
        public long RefusedCount { get; private set; }

        /// <summary>Serialization time of <paramref name="lengthBytes"/> at the admitted rate, in ticks.</summary>
        public long AdmissionCostTicks(int lengthBytes) =>
            lengthBytes * _ticksPerSecond / _admitBytesPerSecond;

        /// <summary>
        /// Refills the bucket for the elapsed time, then either forwards the datagram or refuses it.
        ///
        /// A refusal returns false and <b>does not forward</b>, which is the whole point: the
        /// datagram never reaches the link and therefore never contributes to its backlog.
        /// A datagram the wrapped transport itself refuses is *not* refunded — the credit was spent
        /// on an attempt the link declined, which is what a real send that failed downstream costs.
        /// Allocation-free.
        /// </summary>
        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            if (!_started)
            {
                _started = true;
                _lastRefillTicks = nowTicks;
                _creditTicks = _burstTicks;
            }
            else if (nowTicks > _lastRefillTicks)
            {
                _creditTicks += nowTicks - _lastRefillTicks;
                if (_creditTicks > _burstTicks)
                {
                    _creditTicks = _burstTicks;
                }

                _lastRefillTicks = nowTicks;
            }

            long cost = AdmissionCostTicks(payload.Length);
            if (cost > _creditTicks)
            {
                RefusedCount++;
                return false;
            }

            _creditTicks -= cost;
            return _inner.Send(payload, nowTicks);
        }

        /// <inheritdoc/>
        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks) =>
            _inner.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks);

        /// <summary>
        /// Refills the bucket, clears the refusal counter, and resets the wrapped transport — a
        /// decorator resets what it wraps, so the next trial reproduces the previous one.
        /// </summary>
        public void Reset()
        {
            ResetLocalState();
            _inner.Reset();
        }

        private void ResetLocalState()
        {
            _creditTicks = _burstTicks;
            _lastRefillTicks = 0;
            _started = false;
            RefusedCount = 0;
        }
    }
}
