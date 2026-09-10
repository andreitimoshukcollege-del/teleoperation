using System;
using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// The <c>docs/metrics.md</c> §3 (Network) collector: a passive, allocation-free state machine
    /// that turns datagram events into the §3 metric names. It measures the link; it never changes
    /// it, and it never sends anything of its own — this is passive observation of traffic the
    /// system already carries, not active probing.
    ///
    /// <b>Why this is a plain class and not an <see cref="ITransport"/> decorator.</b> §3's five
    /// quantities do not all live at one vantage point, and that is the substantive finding this
    /// type encodes rather than papers over:
    ///
    /// <list type="bullet">
    /// <item><b>Loss and burst length live at the sender's <c>Send</c> boundary.</b>
    /// <c>GilbertElliottLossImpairment</c> declares <c>ImpairmentStages.Send</c> and
    /// <c>EmulatedTransport.Send</c> returns <c>false</c> for exactly the datagrams it destroyed,
    /// so a refusal count is the loss count — no estimation, no sequence bookkeeping, and
    /// structurally incapable of confusing link loss with a buffer's discard, because no buffer
    /// exists at that boundary. §3 is emphatic that late-arrival loss must never be added to
    /// network loss; measuring at <c>Send</c> makes that impossible rather than merely
    /// careful.</item>
    /// <item><b>Reordering lives at the receiver, and needs a sequence number</b>, which
    /// <see cref="ITransport"/> deliberately does not expose: <c>TryReceive</c> yields opaque bytes
    /// and <c>DatagramFate</c> documents the absence of a sequence field as intentional. Only a
    /// caller that has already decoded a frame can supply it.</item>
    /// <item><b>Interarrival jitter needs the sender's timestamp <i>and its tick rate</i> for the
    /// same datagram</b> — see <see cref="OnSequencedArrival"/>.</item>
    /// </list>
    ///
    /// So the type takes events from whoever holds the information: <c>MeasuredTransport</c> feeds
    /// the two transport-vantage methods, and a pipeline endpoint that has decoded a frame feeds
    /// the third. Nothing here polls, reads a clock, allocates, or looks at a payload.
    ///
    /// <b>Counts, not rates</b>, following the precedent <c>playout_late</c> set in
    /// <c>docs/metrics.md</c> §3: a direction that loses nothing emits nothing rather than a stream
    /// of zeroes, and the analyst divides. A rate computed in here would also be a rate over
    /// whatever window this object happened to live for, which is a trial in a sweep and a session
    /// in a host — two different denominators wearing one metric name.
    ///
    /// One instance observes one direction of one link. Not thread-safe, by the same contract as
    /// everything else in <c>Transport/</c>.
    /// </summary>
    public sealed class NetworkObserver
    {
        /// <summary>
        /// RFC 3550 A.8's single-pole gain: <c>J += (|D| − J)/16</c>. Named rather than inlined
        /// because it is the one number in this file that came from outside the repo, and because
        /// changing it would silently make every recorded jitter figure incomparable with every
        /// other one — and with the streaming literature, which is the entire reason
        /// <c>docs/metrics.md</c> §3 asks for this estimator alongside the OWD IQR.
        /// </summary>
        private const double Rfc3550JitterGainDivisor = 16.0;

        private readonly IMetricSink _metrics;
        private readonly long _ticksPerSecond;

        // All six names are string literals chosen by the factory, never built by concatenation:
        // IMetricSink.Record's own doc requires an interned literal because a concatenated name
        // allocates on the hot path, and literals are also what makes `grep net_ docs/metrics.md`
        // able to prove every emitted name is defined.
        private readonly string _sentName;
        private readonly string _droppedName;
        private readonly string _lossBurstName;
        private readonly string _receivedName;

        /// <summary>Null when this direction has no wired sequenced vantage; see the factories.</summary>
        private readonly string? _reorderDisplacementName;

        /// <summary>Null when this direction has no derivable interarrival jitter; see the factories.</summary>
        private readonly string? _jitterName;

        /// <summary>Length so far of the run of consecutive refused sends; 0 when none is open.</summary>
        private int _openLossRunLength;

        private bool _hasHighestSequence;
        private uint _highestSequence;

        private bool _hasPreviousArrival;
        private long _previousArrivalTicks;
        private long _previousSenderSendTicks;
        private long _previousSenderTicksPerSecond;
        private double _jitterMs;

        private NetworkObserver(
            IMetricSink metrics,
            long ticksPerSecond,
            string sentName,
            string droppedName,
            string lossBurstName,
            string receivedName,
            string? reorderDisplacementName,
            string? jitterName)
        {
            if (metrics == null)
            {
                throw new ArgumentNullException(nameof(metrics));
            }

            if (ticksPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(ticksPerSecond), ticksPerSecond, "Tick rate must be positive.");
            }

            _metrics = metrics;
            _ticksPerSecond = ticksPerSecond;
            _sentName = sentName;
            _droppedName = droppedName;
            _lossBurstName = lossBurstName;
            _receivedName = receivedName;
            _reorderDisplacementName = reorderDisplacementName;
            _jitterName = jitterName;
        }

        /// <summary>
        /// An observer for the operator→robot direction. <b>No sequenced vantage</b>: the only
        /// place a decoded uplink <c>CommandFrame.Sequence</c> exists is the robot endpoint, and
        /// nothing wires an observer there today, so <see cref="OnSequencedArrival"/> throws on an
        /// instance from this factory rather than silently emitting a metric name
        /// <c>docs/metrics.md</c> does not define. Uplink jitter is not merely unwired but
        /// structurally underivable — RFC 3550's <c>D</c> cancels a clock offset but not a clock
        /// rate, and <c>CommandFrame</c> carries no operator <c>TicksPerSecond</c> by the explicit
        /// decision in <c>docs/adr/0008</c>, so the robot cannot normalize the operator's stamps.
        /// </summary>
        public static NetworkObserver ForUplink(IMetricSink metrics, ITimeAuthority clock) =>
            new NetworkObserver(
                metrics,
                clock == null ? throw new ArgumentNullException(nameof(clock)) : clock.TicksPerSecond,
                "net_uplink_sent",
                "net_uplink_dropped",
                "net_uplink_loss_burst",
                "net_uplink_received",
                reorderDisplacementName: null,
                jitterName: null);

        /// <summary>
        /// An observer for the robot→operator direction, which has all three vantages: the operator
        /// holds the sending transport's refusals for its own uplink, the receiving transport's
        /// deliveries, and — after <c>RobotStateFrameCodec</c> has decoded a reply — the sequence,
        /// the robot's raw send stamp and the robot's tick rate that jitter needs.
        /// </summary>
        public static NetworkObserver ForDownlink(IMetricSink metrics, ITimeAuthority clock) =>
            new NetworkObserver(
                metrics,
                clock == null ? throw new ArgumentNullException(nameof(clock)) : clock.TicksPerSecond,
                "net_downlink_sent",
                "net_downlink_dropped",
                "net_downlink_loss_burst",
                "net_downlink_received",
                "net_downlink_reorder_displacement",
                "net_downlink_jitter_ms");

        /// <summary>Whether <see cref="OnSequencedArrival"/> may be called on this instance.</summary>
        public bool SupportsSequencedArrival => _reorderDisplacementName != null;

        /// <summary>
        /// One <see cref="ITransport.Send"/> result. <paramref name="accepted"/> is that method's
        /// own return value: true means the channel took the datagram, false means it will not be
        /// delivered.
        ///
        /// Emits one <c>net_&lt;dir&gt;_sent</c> sample per acceptance and one
        /// <c>net_&lt;dir&gt;_dropped</c> per refusal, and closes out a run of consecutive refusals
        /// by emitting its length as <c>net_&lt;dir&gt;_loss_burst</c> on the first acceptance that
        /// ends it. Every drop therefore belongs to exactly one run, so
        /// <c>sum(loss_burst) == count(dropped)</c> — an identity worth asserting in analysis,
        /// because a violation means one of the two counters is wrong.
        ///
        /// <b>Known censoring:</b> a run still open when <see cref="Reset"/> is called is discarded
        /// rather than emitted, because <see cref="Reset"/> has no tick to stamp it at and Core may
        /// not read a clock to invent one. That drops at most one run per trial and it is the
        /// *last* run, not the longest, so the burst-length distribution is censored on count and
        /// not on shape. At a 2% loss rate over a few hundred datagrams that is under one run in
        /// fifty trials; it is documented rather than corrected because correcting it means giving
        /// <see cref="Reset"/> a time parameter that <see cref="ITransport.Reset"/> does not have.
        ///
        /// <b>What a refusal means depends on what is being wrapped, and the caller owns that.</b>
        /// Over <c>EmulatedTransport</c> a refusal is exactly emulated loss or the wrapped
        /// transport's own queue-full back-pressure. Over a real socket it is a local send failure
        /// only: a datagram the kernel accepted and the network then destroyed returns true here.
        /// So <c>net_&lt;dir&gt;_dropped</c> is exact for the emulator and a lower bound in
        /// general, which is why <c>net_&lt;dir&gt;_received</c> exists alongside it — the two
        /// bracket the truth.
        /// </summary>
        public void OnSendResult(bool accepted, long nowTicks)
        {
            if (accepted)
            {
                if (_openLossRunLength > 0)
                {
                    _metrics.Record(_lossBurstName, _openLossRunLength, nowTicks);
                    _openLossRunLength = 0;
                }

                _metrics.Record(_sentName, 1.0, nowTicks);
                return;
            }

            _openLossRunLength++;
            _metrics.Record(_droppedName, 1.0, nowTicks);
        }

        /// <summary>
        /// One datagram delivered by <see cref="ITransport.TryReceive"/>. Emits
        /// <c>net_&lt;dir&gt;_received</c>. Stamped at the datagram's <c>arrivalTicks</c>
        /// (<c>t_recv</c>), never at the poll time: folding poll time in is the documented way this
        /// project produces frame-quantized network figures.
        ///
        /// Counted at the transport boundary, so it counts what the link delivered, including a
        /// datagram a codec later fails to decode. <c>net_&lt;dir&gt;_sent</c> minus this is
        /// in-flight loss — loss the link inflicted after accepting the datagram, which no shipped
        /// impairment models and which should therefore be exactly zero on the emulator. It is
        /// emitted anyway, because "should be zero" is a claim worth being able to check.
        /// </summary>
        public void OnDelivery(long arrivalTicks)
        {
            _metrics.Record(_receivedName, 1.0, arrivalTicks);
        }

        /// <summary>
        /// One decoded frame's arrival, from a caller that holds its sequence number and its
        /// sender's raw send stamp. Emits <c>net_&lt;dir&gt;_reorder_displacement</c> when this
        /// arrival is out of order, and <c>net_&lt;dir&gt;_jitter_ms</c> once a previous arrival
        /// exists to difference against.
        ///
        /// <b>Reordering</b> follows <c>docs/metrics.md</c> §3 literally: an arrival is out of
        /// sequence when its sequence is below the highest seen so far, and its displacement is
        /// <c>highest − sequence</c>. Emitted only for out-of-order arrivals, so the reordering
        /// *rate* is <c>count(reorder_displacement) / count(received)</c> and the *max
        /// displacement* §3 also asks for is the maximum of the same samples — one metric answering
        /// both halves of the definition, so they can never disagree about what counted. Comparison
        /// is wrap-safe (<c>CommandFrame.Sequence</c>'s own doc requires it): the signed difference
        /// is tested, never <c>&lt;</c> on the unsigned values.
        ///
        /// <b>A duplicate would be counted as a reordering</b> at displacement 0 or more. No
        /// shipped impairment duplicates datagrams and §3 defines no duplication metric, so this is
        /// noted rather than handled; a duplication impairment would need to revisit it.
        ///
        /// <b>Jitter</b> is RFC 3550 A.8 exactly: <c>D(i−1,i) = (R_i − R_{i−1}) − (S_i −
        /// S_{i−1})</c> and <c>J += (|D| − J)/16</c>, reported in milliseconds. Two deliberate
        /// choices:
        ///
        /// <list type="number">
        /// <item><b>Raw sender ticks plus the sender's rate, not a <c>ClockSync</c>-corrected
        /// stamp.</b> <c>D</c> is a difference of differences, so a constant clock offset cancels
        /// exactly and no offset estimate is needed. Feeding it an offset-corrected stamp would
        /// instead fold the *movement of the offset estimate* into the jitter figure — the
        /// estimator's noise reported as the network's — which is precisely the class of confident
        /// wrong number this repo keeps finding. Rate is normalized on the *difference*, not the
        /// absolute stamp, so nothing overflows and no epoch is assumed.</item>
        /// <item><b>The sender's tick rate is a parameter and must be positive.</b> A caller that
        /// does not know it (the robot, for uplink traffic — <c>docs/adr/0008</c>) passes a
        /// non-positive value and gets reordering without jitter, rather than a jitter figure
        /// computed against an assumed rate. Assuming the two ends tick alike is exactly the bug
        /// ADR 0008 was written for: a 10 MHz operator against a 1 GHz robot inflated every
        /// measured RTT 100×.</item>
        /// </list>
        ///
        /// Allocation-free. Throws only on misuse of a direction with no sequenced vantage.
        /// </summary>
        /// <param name="sequence">The decoded frame's sequence number, in the sender's sequence space.</param>
        /// <param name="senderSendTicks">
        /// The sender's own stamp for this datagram, in the sender's timebase and epoch,
        /// uncorrected. Only differences of it are used.
        /// </param>
        /// <param name="senderTicksPerSecond">
        /// The rate <paramref name="senderSendTicks"/> is counted in. Non-positive means "unknown",
        /// which suppresses jitter and is a legitimate, documented state — not an error.
        /// </param>
        /// <param name="arrivalTicks">
        /// <c>t_recv</c> on this observer's own timebase, as reported by
        /// <see cref="ITransport.TryReceive"/>.
        /// </param>
        public void OnSequencedArrival(
            uint sequence, long senderSendTicks, long senderTicksPerSecond, long arrivalTicks)
        {
            if (_reorderDisplacementName == null)
            {
                throw new InvalidOperationException(
                    "This observer has no sequenced vantage wired -- see NetworkObserver.ForUplink. " +
                    "Emitting here would produce a metric name docs/metrics.md does not define.");
            }

            if (_hasHighestSequence)
            {
                // Wrap-safe: CommandFrame.Sequence wraps at uint.MaxValue and its own doc forbids a
                // plain unsigned comparison. The signed cast makes "3 ahead of a value near the
                // wrap point" read as +3 rather than as ~4 billion.
                int relative = unchecked((int)(sequence - _highestSequence));
                if (relative > 0)
                {
                    _highestSequence = sequence;
                }
                else
                {
                    _metrics.Record(_reorderDisplacementName, -relative, arrivalTicks);
                }
            }
            else
            {
                _highestSequence = sequence;
                _hasHighestSequence = true;
            }

            if (_hasPreviousArrival && _jitterName != null &&
                senderTicksPerSecond > 0 && _previousSenderTicksPerSecond > 0)
            {
                double arrivalDeltaMs = (arrivalTicks - _previousArrivalTicks) * 1000.0 / _ticksPerSecond;
                double sendDeltaMs =
                    (senderSendTicks - _previousSenderSendTicks) * 1000.0 / senderTicksPerSecond;
                double transitDifferenceMs = arrivalDeltaMs - sendDeltaMs;
                double magnitude = transitDifferenceMs < 0.0 ? -transitDifferenceMs : transitDifferenceMs;

                _jitterMs += (magnitude - _jitterMs) / Rfc3550JitterGainDivisor;
                _metrics.Record(_jitterName, _jitterMs, arrivalTicks);
            }

            _previousArrivalTicks = arrivalTicks;
            _previousSenderSendTicks = senderSendTicks;
            _previousSenderTicksPerSecond = senderTicksPerSecond;
            _hasPreviousArrival = true;
        }

        /// <summary>
        /// Returns the observer to its as-constructed state so the next trial reproduces the
        /// previous one: no open loss run, no sequence high-water mark, no previous arrival, and
        /// the jitter estimator back at zero — RFC 3550 A.8 initializes <c>J = 0</c>, and carrying
        /// a previous trial's <c>J</c> across a trial boundary would make seed 2's early samples
        /// depend on seed 1's link.
        ///
        /// An open loss run is discarded rather than flushed; see <see cref="OnSendResult"/> for
        /// why and for how much that censors.
        /// </summary>
        public void Reset()
        {
            _openLossRunLength = 0;
            _hasHighestSequence = false;
            _highestSequence = 0;
            _hasPreviousArrival = false;
            _previousArrivalTicks = 0;
            _previousSenderSendTicks = 0;
            _previousSenderTicksPerSecond = 0;
            _jitterMs = 0.0;
        }
    }
}
