using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Time
{
    /// <summary>
    /// Estimates the offset between two <c>ITimeAuthority</c> domains — the operator's and the
    /// robot's — from round trips exchanged between them, so that a robot-origin tick can be
    /// converted into the operator's canonical domain (see docs/adr/0002-latency-trace.md, which
    /// requires exactly this before <c>LatencyTrace</c>'s robot-domain fields are populated for
    /// real).
    ///
    /// <b>Two timebases, two tick rates.</b> <c>ITimeAuthority</c>'s own doc states the rule the
    /// rest of Core obeys — "every long tick value elsewhere in Core is on the timebase of the
    /// authority injected into that component; mixing timebases is a bug." This class is the one
    /// component whose entire job is to reconcile two timebases, so it cannot avoid mixing them;
    /// it must instead do so explicitly. Two <c>ITimeAuthority</c> instances need not agree on
    /// <c>TicksPerSecond</c>, and across real machines they usually do not: Windows'
    /// <c>Stopwatch</c> commonly reports 10,000,000 while .NET on Linux reports 1,000,000,000, a
    /// 100x mismatch that silently inflated every RTT and offset this class produced before
    /// docs/adr/0008-clocksync-cross-rate-normalization.md. Both rates are therefore passed
    /// explicitly on <b>every</b> call — never cached at construction, because the robot's rate
    /// is a fact about a remote machine that arrives over the wire
    /// (<c>Pipeline/RobotStateFrame.TicksPerSecond</c>), and because "time is a parameter, not
    /// hidden state" is the same discipline <c>ITransport</c> and <c>IRobotPlant</c> already
    /// enforce for <c>nowTicks</c>.
    ///
    /// <b>Algorithm.</b> Cristian's algorithm / the NTP four-timestamp offset, applied only after
    /// both domains have been put on one tick rate. A round trip yields four stamps: <c>t0</c> a
    /// request left the operator, <c>t1</c> it arrived at the robot, <c>t2</c> the reply left the
    /// robot, <c>t3</c> the reply arrived back at the operator. <c>t0</c>/<c>t3</c> are operator
    /// domain; <c>t1</c>/<c>t2</c> are robot domain and are rescaled into
    /// <i>operator-tick-equivalent</i> units first:
    /// <code>
    /// ratio  = operatorTicksPerSecond / robotTicksPerSecond   // double, see below
    /// t1'    = round(t1 * ratio)                              // robot ticks -> operator-tick units
    /// t2'    = round(t2 * ratio)
    /// rtt    = (t3 - t0) - (t2' - t1')
    /// offset = ((t0 - t1') + (t3 - t2')) / 2    // operator domain minus robot domain
    /// </code>
    /// The ratio is a <c>double</c>, not integer arithmetic: neither rate divides the other in
    /// general (10,000,000 vs 1,000,000,000 happens to, but 1,000,000 vs 10,000,000 vs a Quest's
    /// rate need not), and an integer ratio would truncate to 0 whenever the robot ticks faster
    /// than the operator — turning a rate conversion into total data loss. A <c>double</c> holds
    /// tick counts exactly up to 2^53, which at 1 GHz is ~104 days of uptime, far beyond any
    /// session; <see cref="Math.Round"/> keeps the rescale deterministic and unbiased rather than
    /// truncating toward zero. Both rates are assumed positive, as <c>ITimeAuthority</c>
    /// guarantees.
    ///
    /// <c>offset</c> matches <c>LatencyTrace.WithClockSync</c>'s documented "operator domain
    /// minus robot domain" convention directly: <c>operatorTicks = round(robotTicks * ratio) + offset</c>,
    /// which is exactly what <see cref="ToOperatorTicks"/> computes — same rescale, same
    /// direction, so a converted stamp and the offset it was converted with always agree.
    /// The true offset lies within <c>± rtt/2</c> of this estimate when the two transit delays
    /// (request and reply) are symmetric, which is why <see cref="ClockSyncDiagnostics.OffsetUncertaintyTicks"/>
    /// tracks <c>rtt/2</c> rather than claiming a tighter bound the four timestamps cannot
    /// actually support. Every tick value this class stores or reports —
    /// <see cref="ClockSyncDiagnostics"/>'s offset, uncertainty, and RTTs alike — is in operator
    /// ticks, since that is the only domain the rescale leaves standing.
    ///
    /// <b>Smoothing and outlier rejection.</b> A single round trip's offset is noisy. Accepted
    /// samples are folded into an EWMA (<see cref="ClockSyncConfig.SmoothingAlpha"/>, same shape
    /// as <see cref="PredictorConfig.SmoothingAlpha"/>). Before that, a round trip is rejected —
    /// counted, never silently dropped — if its RTT is negative (a physically invalid
    /// measurement), exceeds <see cref="ClockSyncConfig.MaxAcceptableRttTicks"/> (an absolute
    /// ceiling), or exceeds <see cref="ClockSyncConfig.OutlierRttMultiple"/> times the best RTT
    /// recently observed (the NTP-style relative filter: a slower round trip bounds the offset
    /// less tightly, so blending it in at full weight would degrade an otherwise-good running
    /// estimate).
    ///
    /// Not behind a <c>Contracts/</c> interface: there is exactly one clock-sync algorithm in
    /// this project, not a family of competing implementations to select between, so this is a
    /// plain <c>sealed class</c> — the same reasoning that makes <c>Stamped&lt;T&gt;</c> a
    /// concrete struct rather than an interface.
    ///
    /// Allocation-free: the RTT history window is a fixed-size buffer allocated once in the
    /// constructor.
    /// </summary>
    public sealed class ClockSync
    {
        private readonly ClockSyncConfig _config;
        private readonly long[] _rttHistory;
        private int _historyCount;
        private int _historyNextIndex;

        private long _smoothedOffsetTicks;
        private long _smoothedUncertaintyTicks;
        private long _lastRttTicks;
        private int _acceptedCount;
        private int _rejectedCount;

        public ClockSync(ClockSyncConfig config)
        {
            _config = config;
            _rttHistory = new long[config.HistoryCapacity];
        }

        /// <summary>
        /// Offers one round trip's four timestamps — plus the tick rate each pair is expressed in
        /// — for the estimator to consider. Returns false if the round trip was rejected as
        /// invalid or as an outlier — see the type-level remarks for the rejection rules. A
        /// rejected round trip does not change <see cref="Diagnostics"/>'s offset/uncertainty,
        /// only its rejected-count. The two robot-domain stamps are rescaled into
        /// operator-tick-equivalent units before any cross-domain arithmetic (see the type-level
        /// remarks and docs/adr/0008-clocksync-cross-rate-normalization.md); when the two rates
        /// are equal the ratio is exactly 1.0 and this reduces to the plain four-timestamp form.
        /// Allocation-free.
        /// </summary>
        /// <param name="operatorSendTicks">t0: the request left the operator, operator domain.</param>
        /// <param name="operatorTicksPerSecond">The operator <c>ITimeAuthority</c>'s tick rate, which t0/t3 are in.</param>
        /// <param name="robotRecvTicks">t1: the request arrived at the robot, robot domain.</param>
        /// <param name="robotSendTicks">t2: the reply left the robot, robot domain.</param>
        /// <param name="robotTicksPerSecond">The robot <c>ITimeAuthority</c>'s tick rate, which t1/t2 are in.</param>
        /// <param name="operatorRecvTicks">t3: the reply arrived at the operator, operator domain.</param>
        public bool AddRoundTrip(
            long operatorSendTicks,
            long operatorTicksPerSecond,
            long robotRecvTicks,
            long robotSendTicks,
            long robotTicksPerSecond,
            long operatorRecvTicks)
        {
            double rateRatio = (double)operatorTicksPerSecond / robotTicksPerSecond;
            long robotRecvScaled = (long)Math.Round(robotRecvTicks * rateRatio);
            long robotSendScaled = (long)Math.Round(robotSendTicks * rateRatio);

            long rtt = (operatorRecvTicks - operatorSendTicks) - (robotSendScaled - robotRecvScaled);

            if (rtt < 0)
            {
                _rejectedCount++;
                return false;
            }

            _rttHistory[_historyNextIndex] = rtt;
            _historyNextIndex = (_historyNextIndex + 1) % _rttHistory.Length;
            if (_historyCount < _rttHistory.Length)
            {
                _historyCount++;
            }

            long minRtt = _rttHistory[0];
            for (int i = 1; i < _historyCount; i++)
            {
                if (_rttHistory[i] < minRtt)
                {
                    minRtt = _rttHistory[i];
                }
            }

            if (rtt > _config.MaxAcceptableRttTicks)
            {
                _rejectedCount++;
                return false;
            }

            if (_acceptedCount > 0 && rtt > (long)(_config.OutlierRttMultiple * minRtt))
            {
                _rejectedCount++;
                return false;
            }

            long offsetSample = ((operatorSendTicks - robotRecvScaled) + (operatorRecvTicks - robotSendScaled)) / 2;
            long uncertaintySample = rtt / 2;

            if (_acceptedCount == 0)
            {
                _smoothedOffsetTicks = offsetSample;
                _smoothedUncertaintyTicks = uncertaintySample;
            }
            else
            {
                float alpha = _config.SmoothingAlpha;
                _smoothedOffsetTicks = (long)(alpha * offsetSample + (1f - alpha) * _smoothedOffsetTicks);
                _smoothedUncertaintyTicks = (long)(alpha * uncertaintySample + (1f - alpha) * _smoothedUncertaintyTicks);
            }

            _lastRttTicks = rtt;
            _acceptedCount++;
            return true;
        }

        /// <summary>
        /// Converts a tick value on the robot's timebase into the operator's: rescale robot ticks
        /// into operator-tick-equivalent units, then add the current smoothed offset. The rescale
        /// is byte-for-byte the same operation <see cref="AddRoundTrip"/> applies to t1/t2, in the
        /// same direction, which is what makes the offset it estimated valid to add here. Before
        /// any round trip has been accepted the offset is zero, so this returns the rescaled
        /// <paramref name="robotTicks"/> and nothing more — a same-rate passthrough, but still a
        /// real unit conversion when the rates differ; check <see cref="Diagnostics"/>'s
        /// <c>IsSynced</c> before trusting the result. Allocation-free.
        /// </summary>
        /// <param name="robotTicks">A stamp on the robot's timebase, uncorrected.</param>
        /// <param name="robotTicksPerSecond">The robot <c>ITimeAuthority</c>'s tick rate, which <paramref name="robotTicks"/> is in.</param>
        /// <param name="operatorTicksPerSecond">The operator <c>ITimeAuthority</c>'s tick rate, which the result is in.</param>
        public long ToOperatorTicks(long robotTicks, long robotTicksPerSecond, long operatorTicksPerSecond)
        {
            double rateRatio = (double)operatorTicksPerSecond / robotTicksPerSecond;
            return (long)Math.Round(robotTicks * rateRatio) + _smoothedOffsetTicks;
        }

        /// <summary>Current estimate and its accounting. See <see cref="ClockSyncDiagnostics"/>.</summary>
        public ClockSyncDiagnostics Diagnostics
        {
            get
            {
                long minRtt = 0;
                if (_historyCount > 0)
                {
                    minRtt = _rttHistory[0];
                    for (int i = 1; i < _historyCount; i++)
                    {
                        if (_rttHistory[i] < minRtt)
                        {
                            minRtt = _rttHistory[i];
                        }
                    }
                }

                return new ClockSyncDiagnostics(
                    _smoothedOffsetTicks,
                    _smoothedUncertaintyTicks,
                    _lastRttTicks,
                    minRtt,
                    _acceptedCount,
                    _rejectedCount,
                    _acceptedCount >= _config.MinSamplesBeforeTrusted);
            }
        }

        /// <summary>
        /// Returns the estimator to its as-constructed state: no round-trip history, offset and
        /// uncertainty back to zero, accepted/rejected counters cleared. Configuration survives.
        /// </summary>
        public void Reset()
        {
            _historyCount = 0;
            _historyNextIndex = 0;
            _smoothedOffsetTicks = 0;
            _smoothedUncertaintyTicks = 0;
            _lastRttTicks = 0;
            _acceptedCount = 0;
            _rejectedCount = 0;
        }
    }
}
