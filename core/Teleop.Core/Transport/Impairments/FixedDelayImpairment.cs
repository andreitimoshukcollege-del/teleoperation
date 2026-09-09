using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport.Impairments
{
    /// <summary>
    /// A fixed one-way delay added to every delivered datagram, on top of whatever transit delay the
    /// wrapped transport already has.
    ///
    /// Consumes no random draws, which makes it the anchor for tests asserting exact arrival ticks:
    /// with only this impairment installed, a datagram's synthetic arrival is pure arithmetic and is
    /// unaffected by any RNG change anywhere in the system.
    /// </summary>
    public sealed class FixedDelayImpairment : INetworkImpairment
    {
        private readonly long _delayTicks;
        private bool _bound;

        /// <param name="delayTicks">
        /// Delay in ticks on the host's <c>ITimeAuthority</c> timebase, never milliseconds --
        /// conversion happens at the caller, per <c>NetworkProfile</c>'s own units rule.
        /// </param>
        public FixedDelayImpairment(long delayTicks)
        {
            if (delayTicks < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(delayTicks), delayTicks, "Delay must not be negative.");
            }

            _delayTicks = delayTicks;
        }

        /// <summary>The configured delay, in ticks. Exposed so the catalog's equivalence test can assert on it.</summary>
        public long DelayTicks => _delayTicks;

        public string AxisName => "delay";

        public ImpairmentStages Stages => ImpairmentStages.Deliver;

        public void Bind(ulong substreamSeed)
        {
            if (_bound)
            {
                throw new InvalidOperationException(
                    "This impairment is already bound to a transport. Each transport needs its own " +
                    "instances -- sharing them would correlate the two directions of a link.");
            }

            _bound = true;
        }

        public void ApplyOnSend(ref DatagramFate fate)
        {
        }

        /// <summary>Additive, never assignment -- see <see cref="DatagramFate"/>'s order-independence rules.</summary>
        public void ApplyOnDeliver(ref DatagramFate fate)
        {
            fate.DelayTicks += _delayTicks;
        }

        /// <summary>Stateless and drawless, so there is nothing to restore.</summary>
        public void Reset()
        {
        }
    }
}
