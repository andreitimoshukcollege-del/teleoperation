using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport.Impairments
{
    /// <summary>
    /// Delay replayed from a recorded one-way-delay trace, one sample per datagram, in order,
    /// wrapping when exhausted.
    ///
    /// <b>Replays, never resamples</b> (<c>Transport/CLAUDE.md</c>): the point of a trace is that its
    /// sample-to-sample structure -- burst onset, burst duration, autocorrelation -- is exactly what
    /// a parametric base+jitter model cannot express. Drawing randomly from the trace's marginal
    /// distribution would destroy the only thing it carries that the parametric axes do not.
    ///
    /// <b>Consumes no random draws.</b> The transport this replaces burned a jitter draw in trace
    /// mode purely to keep one shared RNG stream aligned; with per-impairment substreams
    /// (docs/adr/0013) that is no longer necessary, and not drawing is more honest about what a
    /// replayed trace is.
    ///
    /// <b>Combining this with delay or jitter is legal, not an error.</b> The transport used to
    /// reject a trace-mode profile carrying base delay or jitter, because one struct could not
    /// distinguish "recorded delay" from "synthetic delay" and layering them double-modelled the
    /// same variance. With separate objects there is nothing to contradict: including a fixed delay
    /// alongside a trace means a recorded link plus an extra fixed hop, and the delays simply sum.
    /// The composition is the configuration.
    ///
    /// Samples must already be in the host's tick domain. A trace's file header records the tick
    /// rate of the machine that wrote it, and rescaling is the caller's job -- getting that wrong
    /// inflates every delay by the ratio of the two clocks while each individual number still looks
    /// plausible.
    /// </summary>
    public sealed class TraceDelayImpairment : INetworkImpairment
    {
        private readonly long[] _samples;
        private int _cursor;
        private bool _bound;

        /// <param name="delayTraceTicks">
        /// One non-negative delay sample per datagram, in ticks. Copied defensively, so a caller
        /// mutating the original array afterwards cannot silently break this instance's determinism.
        /// </param>
        public TraceDelayImpairment(long[] delayTraceTicks)
        {
            if (delayTraceTicks == null)
            {
                throw new ArgumentNullException(nameof(delayTraceTicks));
            }

            if (delayTraceTicks.Length == 0)
            {
                throw new ArgumentException(
                    "Delay trace must contain at least one sample.", nameof(delayTraceTicks));
            }

            for (int i = 0; i < delayTraceTicks.Length; i++)
            {
                if (delayTraceTicks[i] < 0)
                {
                    throw new ArgumentOutOfRangeException(
                        nameof(delayTraceTicks), delayTraceTicks[i],
                        "Delay trace samples must not be negative.");
                }
            }

            _samples = (long[])delayTraceTicks.Clone();
        }

        /// <summary>Number of samples in the trace. After this many datagrams the replay wraps.</summary>
        public int SampleCount => _samples.Length;

        public string AxisName => "delay-trace";

        public ImpairmentStages Stages => ImpairmentStages.Deliver;

        public void Bind(ulong substreamSeed)
        {
            if (_bound)
            {
                throw new InvalidOperationException(
                    "This impairment is already bound to a transport. Each transport needs its own " +
                    "instances -- sharing them would make the two directions of a link replay the " +
                    "same trace position, which no real pair of paths does.");
            }

            _bound = true;
        }

        public void ApplyOnSend(ref DatagramFate fate)
        {
        }

        /// <summary>Additive rather than assignment; from a zero start that is identical, and it is what lets a trace compose with a fixed delay.</summary>
        public void ApplyOnDeliver(ref DatagramFate fate)
        {
            fate.DelayTicks += _samples[_cursor];
            _cursor = (_cursor + 1) % _samples.Length;
        }

        /// <summary>Back to the start of the trace, so the next trial replays the same sequence.</summary>
        public void Reset() => _cursor = 0;
    }
}
