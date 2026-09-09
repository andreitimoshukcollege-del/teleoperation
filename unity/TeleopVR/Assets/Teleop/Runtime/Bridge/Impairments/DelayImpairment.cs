using System;
using Teleop.Core.Contracts;
using Teleop.Core.Transport.Impairments;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Lag: a fixed one-way delay added to every delivered datagram, on top of whatever transit
    /// delay the wrapped transport already has.
    /// </summary>
    [Serializable]
    public sealed class DelayImpairment : NetworkImpairment
    {
        [Min(0f)]
        public float BaseDelayMs = 50f;

        public override string AxisName => "delay";

        public override INetworkImpairment ToCoreImpairment(long ticksPerSecond, long[] loadedTrace) =>
            new FixedDelayImpairment(ImpairmentUnits.MsToTicks(BaseDelayMs, ticksPerSecond));

        public override string DescribeSettings() => $"delay {BaseDelayMs:0.#}ms";

        public override bool ValueEquals(NetworkImpairment other) =>
            other is DelayImpairment o && Enabled == o.Enabled && Mathf.Approximately(BaseDelayMs, o.BaseDelayMs);

        public override void CopyTo(NetworkImpairment destination)
        {
            var d = (DelayImpairment)destination;
            d.Enabled = Enabled;
            d.BaseDelayMs = BaseDelayMs;
        }
    }
}
