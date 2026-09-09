using System;
using Teleop.Core.Contracts;
using Teleop.Core.Transport.Impairments;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Packet loss, as a two-state Gilbert-Elliott chain.
    ///
    /// Owns both loss probabilities because they are one model, not two axes: the chain's behaviour
    /// is defined by the pair, and splitting them across files would let a configuration exist with
    /// a burst-continuation probability and no loss to continue.
    /// </summary>
    [Serializable]
    public sealed class LossImpairment : NetworkImpairment
    {
        /// <summary>Probability a datagram is dropped, given the previous one got through.</summary>
        [Range(0f, 100f)]
        public float LossPercent = 1f;

        /// <summary>
        /// When off, loss is plain Bernoulli (each datagram independent). When on, a drop makes the
        /// next drop far likelier — which is what real links do, and what actually breaks a jitter
        /// buffer: burst *length*, not average loss rate, is the thing that hurts.
        /// </summary>
        public bool Bursty;

        /// <summary>
        /// Probability the drop continues, given the previous datagram was dropped. Expected burst
        /// length is <c>1 / (1 - p)</c>, so 70% is bursts of ~3.3 and 90% is bursts of ~10.
        ///
        /// Capped below 100% deliberately: <c>NetworkProfile.LossProbabilityAfterLost</c> treats
        /// exactly 1.0 as an absorbing state — once anything drops, everything drops until the
        /// transport is rebuilt. A legitimate total-outage model, but reaching it by dragging a
        /// slider to the end would read as a hang rather than a setting.
        /// </summary>
        [Range(0f, 100f)]
        public float BurstContinuationPercent = 70f;

        /// <summary>See <see cref="BurstContinuationPercent"/> for why this is not 1.0.</summary>
        public const double MaxBurstContinuation = 0.99;

        public override string AxisName => "loss";

        public override INetworkImpairment ToCoreImpairment(long ticksPerSecond, long[] loadedTrace)
        {
            double afterDelivered = Clamp01(LossPercent / 100.0);

            // Equal after-delivered/after-lost degenerates the Gilbert-Elliott chain to plain
            // Bernoulli -- the same identity docs/adr/0004 relies on for `150ms-20j-0.5loss`. So
            // "bursty off" needs no special case, here or in Core.
            double afterLost = Bursty
                ? Math.Min(Clamp01(BurstContinuationPercent / 100.0), MaxBurstContinuation)
                : afterDelivered;

            return new GilbertElliottLossImpairment(afterDelivered, afterLost);
        }

        /// <summary>
        /// Clamped here because an Inspector value is operator-typed and Core's constructor throws
        /// on an out-of-range probability. A checkbox click must not surface as an exception.
        /// </summary>
        private static double Clamp01(double v) => v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);

        public override string DescribeSettings() => Bursty
            ? $"loss {LossPercent:0.##}% bursty({BurstContinuationPercent:0.#}%)"
            : $"loss {LossPercent:0.##}%";

        public override bool ValueEquals(NetworkImpairment other) =>
            other is LossImpairment o
            && Enabled == o.Enabled
            && Bursty == o.Bursty
            && Mathf.Approximately(LossPercent, o.LossPercent)
            && Mathf.Approximately(BurstContinuationPercent, o.BurstContinuationPercent);

        public override void CopyTo(NetworkImpairment destination)
        {
            var d = (LossImpairment)destination;
            d.Enabled = Enabled;
            d.LossPercent = LossPercent;
            d.Bursty = Bursty;
            d.BurstContinuationPercent = BurstContinuationPercent;
        }
    }
}
