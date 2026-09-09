using System;
using Teleop.Core.Types;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// The per-axis on/off state behind the network-disturbance checkboxes, and the one place that
    /// turns it into a Core <see cref="NetworkProfile"/>.
    ///
    /// <b>This type holds no impairment logic and must never grow any.</b> Every delay, jitter,
    /// loss and reorder decision is made by <c>Teleop.Core.Transport.EmulatedTransport</c> from the
    /// profile this produces; all that happens here is "unchecked axis contributes its neutral
    /// value" plus a milliseconds-to-ticks conversion. That keeps this on the right side of
    /// Teleop/CLAUDE.md's rule that a Bridge file containing a coefficient or a buffering decision
    /// is a bug -- and it is the same shape as <see cref="RobotArmProfileData"/>, which exists for
    /// the identical reason: a plain <see cref="SerializableAttribute"/> mirror that moves data
    /// into a constructor-only readonly Core struct Unity cannot serialize directly.
    ///
    /// The reason this exists at all, rather than reusing <c>NetworkProfileCatalog</c>: the catalog
    /// resolves a *frozen, named* profile suite (docs/adr/0004, 0005, 0006) whose whole point is
    /// that its numbers never move, because recorded results cite those names. That is exactly what
    /// you want for a sweep and exactly what you do not want for an operator sitting in a headset
    /// asking "what does 200ms feel like?". These are two different jobs; conflating them would
    /// mean either an unusable demo or a mutable profile suite, and the second would quietly
    /// invalidate every manifest that names a profile.
    /// </summary>
    [Serializable]
    public sealed class NetworkImpairmentSettings
    {
        [Header("Lag -- fixed one-way delay added to every datagram")]
        public bool EnableDelay;

        [Min(0f)]
        public float BaseDelayMs = 50f;

        [Header("Jitter -- uniform +/- variation on top of the delay above")]
        public bool EnableJitter;

        /// <summary>
        /// Half-width, not full width: the draw is uniform over <c>[-JitterMs, +JitterMs]</c>, so
        /// 10 here means a 20ms spread. See <see cref="NetworkProfile.JitterTicks"/>.
        ///
        /// Jitter is applied whether or not <see cref="EnableDelay"/> is checked. That combination
        /// is legal and occasionally what you want (pure arrival-time variance around zero base
        /// delay), but note the emulator clamps a negative total to zero rather than delivering a
        /// datagram before it was sent, so jitter alone is a half-sided distribution.
        /// </summary>
        [Min(0f)]
        public float JitterMs = 10f;

        [Header("Loss")]
        public bool EnableLoss;

        /// <summary>Probability a datagram is dropped, given the previous one got through.</summary>
        [Range(0f, 100f)]
        public float LossPercent = 1f;

        /// <summary>
        /// When off, loss is plain Bernoulli (each datagram independent). When on, a drop makes the
        /// next drop far likelier, which is what real links do and what actually breaks a jitter
        /// buffer -- <see cref="NetworkProfile.LossProbabilityAfterLost"/> notes that burst
        /// *length*, not average loss rate, is the thing that hurts.
        /// </summary>
        public bool EnableBurstLoss;

        /// <summary>
        /// Probability the drop continues, given the previous datagram was dropped. Expected burst
        /// length is <c>1 / (1 - p)</c>, so 70% is bursts of ~3 and 90% is bursts of ~10.
        ///
        /// Capped below 100% deliberately: <see cref="NetworkProfile.LossProbabilityAfterLost"/>
        /// treats exactly 1.0 as an absorbing state -- once anything drops, everything drops until
        /// the transport is rebuilt. That is a legitimate total-outage model, but reaching it by
        /// dragging a slider to the end would read as a hang rather than a setting, so
        /// <see cref="ToProfile"/> clamps it to <see cref="MaxBurstContinuation"/>.
        /// </summary>
        [Range(0f, 100f)]
        public float BurstContinuationPercent = 70f;

        [Header("Reorder")]
        public bool EnableReorder;

        [Range(0f, 100f)]
        public float ReorderPercent = 2f;

        /// <summary>
        /// Extra delay applied to the datagrams reordering selects.
        ///
        /// <b>This has to exceed the send interval to reorder anything at all</b>
        /// (<see cref="NetworkProfile.ReorderDelayTicks"/>) -- below that it is just a delay spike
        /// on one datagram, with everything still arriving in order. At the 48Hz this project sends
        /// at, the interval is ~21ms, so the 30ms default is the smallest round number that
        /// actually reorders. Lower it and the checkbox will appear to do nothing, which is a
        /// configuration trap rather than a bug.
        /// </summary>
        [Min(0f)]
        public float ReorderDelayMs = 30f;

        /// <summary>See <see cref="BurstContinuationPercent"/> for why this is not 1.0.</summary>
        public const double MaxBurstContinuation = 0.99;

        /// <summary>
        /// True when at least one axis is checked. When false the caller should install no
        /// emulator at all rather than one configured to a neutral profile -- see
        /// <see cref="NetworkImpairmentController"/>, which relies on this to keep the
        /// impairment-off path byte-identical to the unimpaired build rather than merely
        /// equivalent to it.
        /// </summary>
        public bool AnyEnabled => EnableDelay || EnableJitter || EnableLoss || EnableReorder;

        /// <summary>
        /// Composes the checked axes into a Core profile. Unchecked axes contribute their neutral
        /// value (zero), never a default. Every field is clamped into the range
        /// <c>EmulatedTransport.ValidateProfile</c> requires, so a nonsensical Inspector value
        /// produces a tame profile rather than an exception thrown out of a checkbox click.
        /// </summary>
        public NetworkProfile ToProfile(long ticksPerSecond)
        {
            double lossAfterDelivered = EnableLoss ? Clamp01(LossPercent / 100.0) : 0.0;

            // Equal after-delivered/after-lost probabilities degenerate the Gilbert-Elliott chain
            // to plain Bernoulli loss -- the same identity NetworkProfileCatalog relies on for its
            // own "not bursty in the name" case. So "burst loss off" needs no special case here.
            double lossAfterLost = EnableLoss
                ? (EnableBurstLoss
                    ? Math.Min(Clamp01(BurstContinuationPercent / 100.0), MaxBurstContinuation)
                    : lossAfterDelivered)
                : 0.0;

            return new NetworkProfile(
                baseDelayTicks: EnableDelay ? MsToTicks(BaseDelayMs, ticksPerSecond) : 0L,
                jitterTicks: EnableJitter ? MsToTicks(JitterMs, ticksPerSecond) : 0L,
                lossProbabilityAfterDelivered: lossAfterDelivered,
                lossProbabilityAfterLost: lossAfterLost,
                reorderProbability: EnableReorder ? Clamp01(ReorderPercent / 100.0) : 0.0,
                reorderDelayTicks: EnableReorder ? MsToTicks(ReorderDelayMs, ticksPerSecond) : 0L);
        }

        /// <summary>
        /// A one-line summary for the HUD and for the Play-mode log line that records what the
        /// operator actually had switched on -- the impairment state is not otherwise recoverable
        /// from a <c>.tlog</c>, and a session recorded under unknown conditions is not a result.
        /// </summary>
        public string Describe()
        {
            if (!AnyEnabled)
            {
                return "none";
            }

            string result = string.Empty;
            if (EnableDelay)
            {
                result += $"delay {BaseDelayMs:0.#}ms ";
            }

            if (EnableJitter)
            {
                result += $"jitter ±{JitterMs:0.#}ms ";
            }

            if (EnableLoss)
            {
                result += EnableBurstLoss
                    ? $"loss {LossPercent:0.##}% bursty({BurstContinuationPercent:0.#}%) "
                    : $"loss {LossPercent:0.##}% ";
            }

            if (EnableReorder)
            {
                result += $"reorder {ReorderPercent:0.##}%@{ReorderDelayMs:0.#}ms ";
            }

            return result.TrimEnd();
        }

        /// <summary>
        /// Value-equality over every field, so <see cref="NetworkImpairmentController"/> can detect
        /// an Inspector edit without rebuilding transports every frame. Hand-written rather than
        /// made a struct with generated equality: this is a serialized Unity type edited in place
        /// by the Inspector, which wants a reference type.
        /// </summary>
        public bool ValueEquals(NetworkImpairmentSettings other)
        {
            return other != null
                && EnableDelay == other.EnableDelay
                && EnableJitter == other.EnableJitter
                && EnableLoss == other.EnableLoss
                && EnableBurstLoss == other.EnableBurstLoss
                && EnableReorder == other.EnableReorder
                && Mathf.Approximately(BaseDelayMs, other.BaseDelayMs)
                && Mathf.Approximately(JitterMs, other.JitterMs)
                && Mathf.Approximately(LossPercent, other.LossPercent)
                && Mathf.Approximately(BurstContinuationPercent, other.BurstContinuationPercent)
                && Mathf.Approximately(ReorderPercent, other.ReorderPercent)
                && Mathf.Approximately(ReorderDelayMs, other.ReorderDelayMs);
        }

        /// <summary>Field-by-field copy into <paramref name="destination"/>, avoiding an allocation per change check.</summary>
        public void CopyTo(NetworkImpairmentSettings destination)
        {
            destination.EnableDelay = EnableDelay;
            destination.BaseDelayMs = BaseDelayMs;
            destination.EnableJitter = EnableJitter;
            destination.JitterMs = JitterMs;
            destination.EnableLoss = EnableLoss;
            destination.LossPercent = LossPercent;
            destination.EnableBurstLoss = EnableBurstLoss;
            destination.BurstContinuationPercent = BurstContinuationPercent;
            destination.EnableReorder = EnableReorder;
            destination.ReorderPercent = ReorderPercent;
            destination.ReorderDelayMs = ReorderDelayMs;
        }

        private static long MsToTicks(float milliseconds, long ticksPerSecond)
        {
            if (milliseconds <= 0f)
            {
                return 0L;
            }

            return (long)(milliseconds / 1000.0 * ticksPerSecond);
        }

        private static double Clamp01(double value)
        {
            if (value < 0.0)
            {
                return 0.0;
            }

            return value > 1.0 ? 1.0 : value;
        }
    }
}
