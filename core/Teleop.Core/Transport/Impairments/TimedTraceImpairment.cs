using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport.Impairments
{
    /// <summary>
    /// A recorded network replayed by time (docs/adr/0035 N4): a <c>.trace</c> version 2 (one delay or a loss per datagram
    /// sent every <c>sendIntervalTicks</c>; trainer/contract/NETWORK_DATASETS.md), looked up by when each datagram enters the
    /// link rather than by its position in the stream. A datagram sent <c>t</c> after the first one gets sample
    /// <c>floor(t / interval)</c>, wrapping at the end: dropped where that sample is a loss, delayed by it otherwise.
    ///
    /// <para><b>Why by time.</b> <see cref="TraceDelayImpairment"/> consumes a sample per datagram, which is right when the
    /// stream has the trace's own rate. A trace recorded at 50 Hz replayed on a 36 Hz command stream plus 1 Hz keepalives
    /// would stretch every stall and burst by the ratio; looked up by time, a 450 ms stall stays 450 ms, whoever sends
    /// through it.</para>
    ///
    /// <para>Reads <see cref="DatagramFate.SendTicks"/> at both stages (loss at <c>Send</c>, delay at <c>Deliver</c>);
    /// over the loopback every emulator here wraps, both stages see the same tick, so a datagram's loss and delay come from
    /// one sample. Consumes no random draws. Replays, never resamples (<c>Transport/CLAUDE.md</c>). The samples must be in
    /// the host's ticks (the caller rescales a trace written at another rate). Allocation-free.</para>
    /// </summary>
    public sealed class TimedTraceImpairment : INetworkImpairment
    {
        private readonly long[] _delayTicks;
        private readonly bool[] _lost;
        private readonly long _intervalTicks;
        private bool _bound, _started;
        private long _originTicks;

        /// <param name="delayTicks">One delay per sample, in the host's ticks; ignored where <paramref name="lost"/> is true.</param>
        /// <param name="lost">One flag per sample: the datagram sent in that interval is lost.</param>
        /// <param name="intervalTicks">How far apart the samples are, in the host's ticks.</param>
        public TimedTraceImpairment(long[] delayTicks, bool[] lost, long intervalTicks)
        {
            if (delayTicks == null) throw new ArgumentNullException(nameof(delayTicks));
            if (lost == null) throw new ArgumentNullException(nameof(lost));
            if (delayTicks.Length == 0) throw new ArgumentException("a trace needs at least one sample", nameof(delayTicks));
            if (lost.Length != delayTicks.Length) throw new ArgumentException("one loss flag per sample", nameof(lost));
            if (intervalTicks <= 0) throw new ArgumentOutOfRangeException(nameof(intervalTicks), "the sample interval must be positive");
            for (int i = 0; i < delayTicks.Length; i++)
            {
                if (!lost[i] && delayTicks[i] < 0) throw new ArgumentOutOfRangeException(nameof(delayTicks), delayTicks[i], "delays must not be negative");
            }

            _delayTicks = (long[])delayTicks.Clone();
            _lost = (bool[])lost.Clone();
            _intervalTicks = intervalTicks;
        }

        public int SampleCount => _delayTicks.Length;

        public long IntervalTicks => _intervalTicks;

        public string AxisName => "timed-trace";

        public ImpairmentStages Stages => ImpairmentStages.Send | ImpairmentStages.Deliver;

        public void Bind(ulong substreamSeed)
        {
            if (_bound)
            {
                throw new InvalidOperationException(
                    "This impairment is already bound to a transport. Each transport needs its own instances.");
            }

            _bound = true;
        }

        public void ApplyOnSend(ref DatagramFate fate)
        {
            if (_lost[IndexOf(fate.SendTicks)]) fate.Dropped = true;
        }

        public void ApplyOnDeliver(ref DatagramFate fate)
        {
            int i = IndexOf(fate.SendTicks);
            if (!_lost[i]) fate.DelayTicks += _delayTicks[i];
        }

        /// <summary>Back to the trace's start: the next datagram is sample 0 again.</summary>
        public void Reset()
        {
            _started = false;
            _originTicks = 0;
        }

        private int IndexOf(long ticks)
        {
            if (!_started)
            {
                _started = true;
                _originTicks = ticks;
            }

            long elapsed = ticks - _originTicks;
            if (elapsed < 0) return 0;
            return (int)((elapsed / _intervalTicks) % _delayTicks.Length);
        }
    }
}
