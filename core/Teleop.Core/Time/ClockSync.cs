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
    /// <b>Algorithm.</b> Cristian's algorithm / the NTP four-timestamp offset. A round trip
    /// yields four stamps: <c>t0</c> a request left the operator, <c>t1</c> it arrived at the
    /// robot, <c>t2</c> the reply left the robot, <c>t3</c> the reply arrived back at the
    /// operator. Then:
    /// <code>
    /// rtt    = (t3 - t0) - (t2 - t1)
    /// offset = ((t0 - t1) + (t3 - t2)) / 2      // operator domain minus robot domain
    /// </code>
    /// <c>offset</c> matches <c>LatencyTrace.WithClockSync</c>'s documented "operator domain
    /// minus robot domain" convention directly: <c>operatorTicks = robotTicks + offset</c>.
    /// The true offset lies within <c>± rtt/2</c> of this estimate when the two transit delays
    /// (request and reply) are symmetric, which is why <see cref="ClockSyncDiagnostics.OffsetUncertaintyTicks"/>
    /// tracks <c>rtt/2</c> rather than claiming a tighter bound the four timestamps cannot
    /// actually support.
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
        /// Offers one round trip's four timestamps for the estimator to consider. Returns false
        /// if the round trip was rejected as invalid or as an outlier — see the type-level
        /// remarks for the rejection rules. A rejected round trip does not change
        /// <see cref="Diagnostics"/>'s offset/uncertainty, only its rejected-count. Allocation-free.
        /// </summary>
        /// <param name="operatorSendTicks">t0: the request left the operator, operator domain.</param>
        /// <param name="robotRecvTicks">t1: the request arrived at the robot, robot domain.</param>
        /// <param name="robotSendTicks">t2: the reply left the robot, robot domain.</param>
        /// <param name="operatorRecvTicks">t3: the reply arrived at the operator, operator domain.</param>
        public bool AddRoundTrip(
            long operatorSendTicks, long robotRecvTicks, long robotSendTicks, long operatorRecvTicks)
        {
            long rtt = (operatorRecvTicks - operatorSendTicks) - (robotSendTicks - robotRecvTicks);

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

            long offsetSample = ((operatorSendTicks - robotRecvTicks) + (operatorRecvTicks - robotSendTicks)) / 2;
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
        /// Converts a tick value on the robot's timebase into the operator's, by adding the
        /// current smoothed offset. Before any round trip has been accepted, the offset is zero
        /// and this passes <paramref name="robotTicks"/> through unchanged — check
        /// <see cref="Diagnostics"/>'s <c>IsSynced</c> before trusting the result. Allocation-free.
        /// </summary>
        public long ToOperatorTicks(long robotTicks) => robotTicks + _smoothedOffsetTicks;

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
