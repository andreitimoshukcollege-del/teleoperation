using System;
using Teleop.Core.Types;

namespace Teleop.Bridge
{
    /// <summary>
    /// One axis of network impairment: an on/off flag, whatever parameters that axis needs, and the
    /// one method that folds them into the profile handed to <c>EmulatedTransport</c>.
    ///
    /// <b>An impairment never impairs anything.</b> It contributes numbers to a
    /// <see cref="NetworkProfileDraft"/>; every delay, drop and reorder decision is made by
    /// <c>Teleop.Core.Transport.EmulatedTransport</c>. Subclasses that start computing rather than
    /// describing are the leak Teleop/CLAUDE.md's "a Bridge file containing a coefficient is a bug"
    /// rule exists to catch.
    ///
    /// <b>Adding an axis.</b> One new file in <c>Impairments/</c> deriving from this, plus one field
    /// and three one-line entries in <see cref="NetworkImpairmentSettings"/>. Nothing else changes,
    /// and no existing axis is touched.
    ///
    /// <b>But note the real ceiling.</b> This only makes the *authoring* side extensible. A
    /// genuinely new kind of impairment — duplication, bandwidth throttling, corruption — is not a
    /// Unity change at all: <see cref="NetworkProfile"/> is a fixed six-field readonly struct and
    /// <c>EmulatedTransport</c> is what would have to grow the behaviour, both in Core, on the Linux
    /// box, with an ADR. What this shape buys is that when Core does gain such a field, the Unity
    /// side is one file and one line rather than an edit threaded through a monolith.
    /// </summary>
    [Serializable]
    public abstract class NetworkImpairment
    {
        /// <summary>The checkbox. When false the axis contributes nothing — its neutral value, never a default.</summary>
        public bool Enabled;

        /// <summary>Short lower-case label used in log lines and the HUD, e.g. "delay".</summary>
        public abstract string AxisName { get; }

        /// <summary>
        /// Folds this axis's parameters into <paramref name="draft"/>. Called only when
        /// <see cref="Enabled"/> is true, so implementations never need to check it.
        ///
        /// Implementations must be order-independent: each axis owns disjoint fields of the draft,
        /// and the one cross-axis interaction (a delay trace superseding base delay and jitter) is
        /// resolved in <see cref="NetworkProfileDraft.Build"/> rather than by relying on call order.
        /// </summary>
        public abstract void Contribute(ref NetworkProfileDraft draft, long ticksPerSecond);

        /// <summary>This axis's settings as one human-readable fragment, e.g. "delay 50ms". Enabled-only.</summary>
        public abstract string DescribeSettings();

        /// <summary>Value equality against another instance of the same concrete type, including <see cref="Enabled"/>.</summary>
        public abstract bool ValueEquals(NetworkImpairment other);

        /// <summary>Field-by-field copy into another instance of the same concrete type.</summary>
        public abstract void CopyTo(NetworkImpairment destination);
    }

    /// <summary>
    /// A mutable accumulator the axes write into, converted once into the immutable
    /// <see cref="NetworkProfile"/> Core actually consumes.
    ///
    /// This exists because <see cref="NetworkProfile"/> is a readonly struct with a single
    /// six-argument constructor — there is no way for five separate objects to each contribute part
    /// of one. Keeping the accumulator separate from the profile is also what lets
    /// <see cref="Build"/> own the one rule that spans axes, instead of scattering it.
    /// </summary>
    public struct NetworkProfileDraft
    {
        public long BaseDelayTicks;
        public long JitterTicks;
        public double LossProbabilityAfterDelivered;
        public double LossProbabilityAfterLost;
        public double ReorderProbability;
        public long ReorderDelayTicks;

        /// <summary>
        /// Set by a trace-driven delay axis to declare that delay comes from recorded samples.
        /// <see cref="Build"/> then zeroes base delay and jitter regardless of what those axes
        /// contributed, because <c>EmulatedTransport</c>'s trace constructor rejects a profile
        /// carrying either — synthetic jitter layered on an already-recorded delay would
        /// double-model the same variance.
        /// </summary>
        public bool DelayFromTrace;

        /// <summary>
        /// Produces the profile, clamping every field into the range
        /// <c>EmulatedTransport.ValidateProfile</c> requires. Clamping here rather than in each axis
        /// means a nonsensical Inspector value yields a tame profile instead of an exception thrown
        /// out of a checkbox click, and means a new axis cannot forget to do it.
        /// </summary>
        public NetworkProfile Build()
        {
            long baseDelay = DelayFromTrace ? 0L : Math.Max(0L, BaseDelayTicks);
            long jitter = DelayFromTrace ? 0L : Math.Max(0L, JitterTicks);

            return new NetworkProfile(
                baseDelayTicks: baseDelay,
                jitterTicks: jitter,
                lossProbabilityAfterDelivered: Clamp01(LossProbabilityAfterDelivered),
                lossProbabilityAfterLost: Clamp01(LossProbabilityAfterLost),
                reorderProbability: Clamp01(ReorderProbability),
                reorderDelayTicks: Math.Max(0L, ReorderDelayTicks));
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

    /// <summary>Shared millisecond-to-tick conversion, so every axis rounds identically.</summary>
    public static class ImpairmentUnits
    {
        public static long MsToTicks(float milliseconds, long ticksPerSecond)
        {
            if (milliseconds <= 0f)
            {
                return 0L;
            }

            return (long)(milliseconds / 1000.0 * ticksPerSecond);
        }

        public static float TicksToMs(long ticks, long ticksPerSecond) =>
            (float)(ticks * 1000.0 / ticksPerSecond);
    }
}
