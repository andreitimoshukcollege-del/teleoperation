using System;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Reordering: a share of delivered datagrams get extra delay, pushing them behind datagrams
    /// sent after them.
    ///
    /// Kept independent of jitter rather than left to emerge from it, so "packets arrive out of
    /// order" can be studied with jitter at zero and vice versa. Those two hit a playout buffer
    /// differently and conflating them makes a result uninterpretable.
    /// </summary>
    [Serializable]
    public sealed class ReorderImpairment : NetworkImpairment
    {
        [Range(0f, 100f)]
        public float ReorderPercent = 2f;

        /// <summary>
        /// Extra delay applied to the datagrams reordering selects.
        ///
        /// <b>This has to exceed the send interval to reorder anything at all.</b> Below that it is
        /// just a delay spike on one datagram, with everything still arriving in order. At the 48Hz
        /// this project sends at, the interval is ~21 ms, so the 30 ms default is the smallest round
        /// number that actually reorders. Lower it and the checkbox appears to do nothing — a
        /// configuration trap rather than a bug.
        /// </summary>
        [Min(0f)]
        public float ReorderDelayMs = 30f;

        public override string AxisName => "reorder";

        public override void Contribute(ref NetworkProfileDraft draft, long ticksPerSecond)
        {
            draft.ReorderProbability = ReorderPercent / 100.0;
            draft.ReorderDelayTicks = ImpairmentUnits.MsToTicks(ReorderDelayMs, ticksPerSecond);
        }

        public override string DescribeSettings() =>
            $"reorder {ReorderPercent:0.##}%@{ReorderDelayMs:0.#}ms";

        public override bool ValueEquals(NetworkImpairment other) =>
            other is ReorderImpairment o
            && Enabled == o.Enabled
            && Mathf.Approximately(ReorderPercent, o.ReorderPercent)
            && Mathf.Approximately(ReorderDelayMs, o.ReorderDelayMs);

        public override void CopyTo(NetworkImpairment destination)
        {
            var d = (ReorderImpairment)destination;
            d.Enabled = Enabled;
            d.ReorderPercent = ReorderPercent;
            d.ReorderDelayMs = ReorderDelayMs;
        }
    }
}
