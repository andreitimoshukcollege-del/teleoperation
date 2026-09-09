using System;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Jitter: uniform variation added on top of the base delay.
    ///
    /// Independent of <see cref="DelayImpairment"/> on purpose. Jitter with delay off is legal and
    /// occasionally what you want (pure arrival-time variance around zero base delay), but note the
    /// emulator clamps a negative total to zero rather than delivering a datagram before it was
    /// sent, so jitter alone is a half-sided distribution.
    /// </summary>
    [Serializable]
    public sealed class JitterImpairment : NetworkImpairment
    {
        /// <summary>
        /// Half-width, not full width: the draw is uniform over <c>[-JitterMs, +JitterMs]</c>, so 10
        /// here means a 20 ms spread. See <c>NetworkProfile.JitterTicks</c>, which also explains why
        /// the distribution is uniform rather than Gaussian.
        /// </summary>
        [Min(0f)]
        public float JitterMs = 10f;

        public override string AxisName => "jitter";

        public override void Contribute(ref NetworkProfileDraft draft, long ticksPerSecond)
        {
            draft.JitterTicks = ImpairmentUnits.MsToTicks(JitterMs, ticksPerSecond);
        }

        public override string DescribeSettings() => $"jitter ±{JitterMs:0.#}ms";

        public override bool ValueEquals(NetworkImpairment other) =>
            other is JitterImpairment o && Enabled == o.Enabled && Mathf.Approximately(JitterMs, o.JitterMs);

        public override void CopyTo(NetworkImpairment destination)
        {
            var d = (JitterImpairment)destination;
            d.Enabled = Enabled;
            d.JitterMs = JitterMs;
        }
    }
}
