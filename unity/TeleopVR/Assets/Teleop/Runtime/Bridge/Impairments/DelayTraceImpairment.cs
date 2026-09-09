using System;
using Teleop.Core.Contracts;
using Teleop.Core.Transport.Impairments;

namespace Teleop.Bridge
{
    /// <summary>
    /// Delay replayed from a recorded trace instead of drawn from base + jitter.
    ///
    /// The odd one out, and deliberately so. Every other axis contributes numbers to the profile;
    /// this one contributes a *delay source*, which is why it sets
    /// <see cref="NetworkProfileDraft.DelayFromTrace"/> rather than any profile field, and why the
    /// samples themselves are loaded by <see cref="DelayTraceLoader"/> and handed to
    /// <see cref="SwappableTransport"/> separately. An impairment carrying a 2000-element array
    /// through Unity serialization would be the wrong shape entirely.
    ///
    /// This is the only way to reach burst *delay* structure: `synthetic-burst`'s bursts live in its
    /// recorded samples and are not expressible as base + jitter, which is exactly why it matters —
    /// the Buffering result rests on that structure.
    ///
    /// When enabled it supersedes <see cref="DelayImpairment"/> and <see cref="JitterImpairment"/>,
    /// resolved in <see cref="NetworkProfileDraft.Build"/> so the outcome does not depend on the
    /// order axes are visited.
    /// </summary>
    [Serializable]
    public sealed class DelayTraceImpairment : NetworkImpairment
    {
        /// <summary>
        /// Trace to replay, without the <c>.trace</c> extension. Resolved by
        /// <see cref="DelayTraceLoader"/> from <c>persistentDataPath</c> then <c>Resources</c>. The
        /// source of truth is <c>core/testdata/traces/</c>; no copy is committed under <c>unity/</c>,
        /// so install it with <c>just install-traces</c> (Editor) or <c>adb push</c> (device).
        /// </summary>
        public string TraceName = "synthetic-burst";

        public override string AxisName => "delay-trace";

        /// <summary>
        /// Needs the samples the host already loaded; returns null when none are available, so the
        /// caller installs nothing rather than silently running a different link.
        /// </summary>
        public override INetworkImpairment ToCoreImpairment(long ticksPerSecond, long[] loadedTrace) =>
            loadedTrace == null ? null : new TraceDelayImpairment(loadedTrace);

        public override string DescribeSettings() => $"delay-trace '{TraceName}'";

        public override bool ValueEquals(NetworkImpairment other) =>
            other is DelayTraceImpairment o && Enabled == o.Enabled && TraceName == o.TraceName;

        public override void CopyTo(NetworkImpairment destination)
        {
            var d = (DelayTraceImpairment)destination;
            d.Enabled = Enabled;
            d.TraceName = TraceName;
        }
    }
}
