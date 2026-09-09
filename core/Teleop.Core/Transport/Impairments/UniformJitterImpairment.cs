using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport.Impairments
{
    /// <summary>
    /// Uniform jitter: a draw from the inclusive integer range <c>[-halfWidth, +halfWidth]</c> added
    /// to a datagram's delay.
    ///
    /// Uniform rather than Gaussian or Pareto is the scoping choice <c>NetworkProfile.JitterTicks</c>
    /// already documents: a uniform integer draw is the one distribution whose every outcome can be
    /// hand-computed and asserted exactly in a deterministic test, which is what the delay path has
    /// to be trusted for before anything measured through it counts.
    ///
    /// Independent of <see cref="FixedDelayImpairment"/>. Jitter with no base delay is legal and
    /// occasionally wanted, but note the emulator clamps a negative total to zero rather than
    /// delivering a datagram before it was sent, so jitter alone is a half-sided distribution.
    /// </summary>
    public sealed class UniformJitterImpairment : INetworkImpairment
    {
        private readonly long _halfWidthTicks;
        private SeededRng _rng;
        private bool _bound;

        public UniformJitterImpairment(long halfWidthTicks)
        {
            if (halfWidthTicks < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(halfWidthTicks), halfWidthTicks, "Jitter half-width must not be negative.");
            }

            // Guards the (2 * halfWidth + 1) span computation below from overflowing. Lifted
            // verbatim from the ValidateProfile rule this impairment replaces.
            if (halfWidthTicks > long.MaxValue / 4)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(halfWidthTicks), halfWidthTicks, "Jitter half-width is implausibly large.");
            }

            _halfWidthTicks = halfWidthTicks;
        }

        /// <summary>The configured half-width, in ticks. A draw spans twice this, plus one.</summary>
        public long HalfWidthTicks => _halfWidthTicks;

        public string AxisName => "jitter";

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

        /// <summary>
        /// Exactly one draw per datagram, <b>always</b>, including at half-width zero where the
        /// result is necessarily exactly 0.
        ///
        /// That unconditional draw is what makes a jitter-magnitude sweep (`jitter-5ms` against
        /// `jitter-20ms`) see the same underlying uniforms, and what makes an axis at neutral
        /// settings observationally identical to its absence -- the property that lets
        /// <c>NetworkProfileCatalog</c> emit only non-zero axes while still reproducing the frozen
        /// profiles exactly (docs/adr/0013).
        ///
        /// The expression is preserved bit-for-bit from the <c>DrawDelayTicks</c> it replaces,
        /// modulo bias included, so frozen profiles keep drawing from the same distribution.
        /// </summary>
        public void ApplyOnDeliver(ref DatagramFate fate)
        {
            ulong span = ((ulong)_halfWidthTicks * 2UL) + 1UL;
            fate.DelayTicks += (long)(_rng.NextUInt64() % span) - _halfWidthTicks;
        }

        public void Reset() => _rng.Reset();
    }
}
