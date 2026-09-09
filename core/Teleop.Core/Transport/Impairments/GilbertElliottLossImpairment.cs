using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport.Impairments
{
    /// <summary>
    /// Packet loss as a two-state Gilbert-Elliott Markov chain.
    ///
    /// Owns <b>both</b> probabilities because they are one model, not two axes. Splitting them would
    /// permit a configuration with a burst-continuation probability and no loss to continue, which
    /// is not a link.
    ///
    /// <b>Send stage, deliberately.</b> A lost datagram never reaches the wrapped transport, which
    /// is what a real link does with a dropped packet and what <c>ITransport.Send</c>'s "returns
    /// false when the datagram will not be delivered" promises. Deciding loss at delivery instead
    /// would let a datagram occupy an in-flight slot -- changing back-pressure -- and, over a real
    /// socket, would actually transmit bytes the model says were lost.
    ///
    /// A refusal by the wrapped transport (a full queue) is not a loss and must not advance the
    /// chain. That holds for free here, because this runs before the transport is asked.
    /// </summary>
    public sealed class GilbertElliottLossImpairment : INetworkImpairment
    {
        private readonly double _afterDelivered;
        private readonly double _afterLost;
        private SeededRng _rng;
        private bool _previousWasLost;
        private bool _bound;

        /// <param name="lossProbabilityAfterDelivered">
        /// Probability the next datagram is lost given the previous one was delivered -- the
        /// good-to-bad transition.
        /// </param>
        /// <param name="lossProbabilityAfterLost">
        /// Probability the loss continues given the previous datagram was lost. Setting this equal
        /// to <paramref name="lossProbabilityAfterDelivered"/> degenerates the chain to plain
        /// Bernoulli loss, which is how the frozen `150ms-20j-0.5loss` profile is expressed and why
        /// "bursty off" needs no special case anywhere.
        /// </param>
        public GilbertElliottLossImpairment(
            double lossProbabilityAfterDelivered, double lossProbabilityAfterLost)
        {
            ValidateProbability(lossProbabilityAfterDelivered, nameof(lossProbabilityAfterDelivered));
            ValidateProbability(lossProbabilityAfterLost, nameof(lossProbabilityAfterLost));

            _afterDelivered = lossProbabilityAfterDelivered;
            _afterLost = lossProbabilityAfterLost;
        }

        public double LossProbabilityAfterDelivered => _afterDelivered;

        public double LossProbabilityAfterLost => _afterLost;

        /// <summary>
        /// Expected number of consecutive datagrams lost once loss starts, <c>1 / (1 - p)</c>.
        /// Mirrors <c>NetworkProfile.ExpectedBurstLength</c>; a test asserts the two agree, so the
        /// figure in a manifest and the figure the live model implies come from the same expression.
        /// </summary>
        public double ExpectedBurstLength =>
            _afterLost >= 1.0 ? double.PositiveInfinity : 1.0 / (1.0 - _afterLost);

        public string AxisName => "loss";

        public ImpairmentStages Stages => ImpairmentStages.Send;

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

        /// <summary>
        /// Exactly one draw per datagram, always. <c>NextDouble()</c> is uniform on <c>[0, 1)</c>, so
        /// probability 0 never fires and probability 1 always does -- both endpoints behave exactly,
        /// with no epsilon anywhere.
        /// </summary>
        public void ApplyOnSend(ref DatagramFate fate)
        {
            double lossProbability = _previousWasLost ? _afterLost : _afterDelivered;
            bool lost = _rng.NextDouble() < lossProbability;
            _previousWasLost = lost;

            if (lost)
            {
                fate.Dropped = true;
            }
        }

        public void ApplyOnDeliver(ref DatagramFate fate)
        {
        }

        /// <summary>Chain back to the good state, substream back to its bind seed.</summary>
        public void Reset()
        {
            _previousWasLost = false;
            _rng.Reset();
        }

        /// <summary>
        /// Written as <c>!(v &gt;= 0 &amp;&amp; v &lt;= 1)</c> rather than a pair of comparisons so
        /// that NaN is rejected too -- NaN fails every ordered comparison, so the naive form would
        /// let it through. Preserved verbatim from the <c>ValidateProfile</c> rule this replaces.
        /// </summary>
        private static void ValidateProbability(double value, string parameterName)
        {
            if (!(value >= 0.0 && value <= 1.0))
            {
                throw new ArgumentOutOfRangeException(parameterName, value, "Must be in [0, 1].");
            }
        }
    }
}
