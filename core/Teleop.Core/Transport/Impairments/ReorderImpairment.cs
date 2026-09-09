using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport.Impairments
{
    /// <summary>
    /// Reordering: a share of delivered datagrams get extra delay, pushing them behind datagrams
    /// sent after them.
    ///
    /// An independent knob rather than something left to emerge from jitter variance, so that
    /// "packets arrive out of order" can be studied with jitter at zero and vice versa. Those two
    /// hit a playout buffer differently, and conflating them into one parameter makes a result
    /// uninterpretable.
    ///
    /// <b>The extra delay must exceed the interval between consecutive sends to reorder anything.</b>
    /// Below that it is a delay spike on one datagram with everything still arriving in order --
    /// a configuration trap rather than a bug, and the reason the caller chooses the value.
    /// </summary>
    public sealed class ReorderImpairment : INetworkImpairment
    {
        private readonly double _probability;
        private readonly long _extraDelayTicks;
        private SeededRng _rng;
        private bool _bound;

        public ReorderImpairment(double probability, long extraDelayTicks)
        {
            if (!(probability >= 0.0 && probability <= 1.0))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(probability), probability, "Must be in [0, 1].");
            }

            if (extraDelayTicks < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(extraDelayTicks), extraDelayTicks, "Reorder delay must not be negative.");
            }

            _probability = probability;
            _extraDelayTicks = extraDelayTicks;
        }

        public double Probability => _probability;

        public long ExtraDelayTicks => _extraDelayTicks;

        public string AxisName => "reorder";

        public ImpairmentStages Stages => ImpairmentStages.Deliver;

        public void Bind(ulong substreamSeed)
        {
            if (_bound)
            {
                throw new InvalidOperationException(
                    "This impairment is already bound to a transport. Each transport needs its own " +
                    "instances -- sharing them would correlate the two directions of a link.");
            }

            _rng = new SeededRng(substreamSeed);
            _bound = true;
        }

        public void ApplyOnSend(ref DatagramFate fate)
        {
        }

        /// <summary>Exactly one draw per datagram, always -- including at probability zero. See <see cref="UniformJitterImpairment.ApplyOnDeliver"/> for why that matters.</summary>
        public void ApplyOnDeliver(ref DatagramFate fate)
        {
            if (_rng.NextDouble() < _probability)
            {
                fate.DelayTicks += _extraDelayTicks;
            }
        }

        public void Reset() => _rng.Reset();
    }
}
