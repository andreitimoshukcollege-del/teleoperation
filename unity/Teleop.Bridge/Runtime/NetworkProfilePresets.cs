using Teleop.Core.Transport;
using Teleop.Core.Types;

namespace Teleop.Bridge
{
    /// <summary>
    /// The frozen network-profile suite, as an Inspector dropdown.
    ///
    /// Selecting one <b>populates</b> <see cref="NetworkImpairmentSettings"/>'s per-axis fields
    /// with that profile's exact values; it does not lock them. Every axis stays independently
    /// editable afterwards, which is the point: "start from `300ms-60j-2loss-bursty`, then halve
    /// the jitter and leave everything else alone" is a question an operator in a headset actually
    /// asks, and no frozen suite can enumerate every answer to it.
    ///
    /// The names and numbers come from <see cref="NetworkProfileCatalog"/>, never from constants
    /// duplicated here. That matters more than it looks: docs/adr/0004-network-profile-suite.md
    /// records the four profiles' exact values precisely "so the numbers in a manifest and the
    /// numbers in this document never drift apart", and a second hardcoded copy in Unity is exactly
    /// the drift that argument is about. This file therefore owns only the enum-to-name mapping,
    /// which exists because the real names (`50ms-5j`, `150ms-20j-0.5loss`) are not legal C#
    /// identifiers and an enum is what gives Unity a dropdown without a custom editor.
    /// </summary>
    public enum NetworkProfilePreset
    {
        /// <summary>Hand-configured. Nothing is loaded and nothing is overwritten.</summary>
        Custom = 0,

        /// <summary>`lan` -- 2 ms base, 1 ms jitter, no loss. A good but real link.</summary>
        Lan = 1,

        /// <summary>`50ms-5j` -- 50 ms base, 5 ms jitter, no loss.</summary>
        Delay50Jitter5 = 2,

        /// <summary>`150ms-20j-0.5loss` -- 150 ms base, 20 ms jitter, 0.5% Bernoulli loss.</summary>
        Delay150Jitter20Loss05 = 3,

        /// <summary>`300ms-60j-2loss-bursty` -- 300 ms base, 60 ms jitter, ~2% loss in bursts of ~3.3.</summary>
        Delay300Jitter60Loss2Bursty = 4,

        /// <summary>
        /// `synthetic-burst` -- delay replayed from the committed trace fixture rather than drawn
        /// from base+jitter. Handled separately from the four above: it needs a file, so it goes
        /// through <see cref="DelayTraceLoader"/>, and in this mode the delay and jitter axes are
        /// unavailable (the trace supplies delay; see <see cref="NetworkImpairmentSettings"/>).
        /// </summary>
        SyntheticBurstTrace = 5,
    }

    /// <summary>Maps <see cref="NetworkProfilePreset"/> onto the catalog and back into per-axis settings.</summary>
    public static class NetworkProfilePresets
    {
        /// <summary>The catalog name a preset resolves to, or <c>null</c> for <see cref="NetworkProfilePreset.Custom"/>.</summary>
        public static string CatalogName(NetworkProfilePreset preset)
        {
            switch (preset)
            {
                case NetworkProfilePreset.Lan: return "lan";
                case NetworkProfilePreset.Delay50Jitter5: return "50ms-5j";
                case NetworkProfilePreset.Delay150Jitter20Loss05: return "150ms-20j-0.5loss";
                case NetworkProfilePreset.Delay300Jitter60Loss2Bursty: return "300ms-60j-2loss-bursty";
                case NetworkProfilePreset.SyntheticBurstTrace: return "synthetic-burst";
                default: return null;
            }
        }

        /// <summary>True for the one preset whose delay comes from a trace file rather than a profile.</summary>
        public static bool IsTraceDriven(NetworkProfilePreset preset) =>
            preset == NetworkProfilePreset.SyntheticBurstTrace;

        /// <summary>
        /// Overwrites <paramref name="target"/>'s axis fields with <paramref name="preset"/>'s
        /// values, resolved through Core's catalog. Returns false with a reason for
        /// <see cref="NetworkProfilePreset.Custom"/>, for the trace preset (whose delay is not in a
        /// profile at all), or for a name the catalog rejects.
        ///
        /// The conversion back from a <see cref="NetworkProfile"/>'s ticks into this type's
        /// milliseconds is lossy in the last decimal and deliberately not compensated: the profile
        /// that actually reaches <see cref="EmulatedTransport"/> is recomputed from these fields by
        /// <see cref="NetworkImpairmentSettings.ToProfile"/>, so what the operator sees in the
        /// Inspector is exactly what runs. Displaying the catalog's ticks while running something
        /// a rounding step away from them would be the worse failure.
        /// </summary>
        public static bool TryApply(
            NetworkProfilePreset preset, long ticksPerSecond, NetworkImpairmentSettings target, out string error)
        {
            if (target == null)
            {
                error = "no settings object to populate";
                return false;
            }

            if (preset == NetworkProfilePreset.Custom)
            {
                error = "Custom is hand-configured; nothing to load";
                return false;
            }

            if (IsTraceDriven(preset))
            {
                error = "trace-driven presets carry no delay/jitter to load; the trace supplies delay";
                return false;
            }

            string name = CatalogName(preset);
            if (!NetworkProfileCatalog.TryResolveParametric(
                    name, ticksPerSecond, out NetworkProfile profile, out string catalogError))
            {
                error = $"catalog rejected '{name}': {catalogError}";
                return false;
            }

            target.EnableDelayTrace = false;

            target.EnableDelay = profile.BaseDelayTicks > 0;
            target.BaseDelayMs = NetworkImpairmentSettings.TicksToMs(profile.BaseDelayTicks, ticksPerSecond);

            target.EnableJitter = profile.JitterTicks > 0;
            target.JitterMs = NetworkImpairmentSettings.TicksToMs(profile.JitterTicks, ticksPerSecond);

            target.EnableLoss = profile.LossProbabilityAfterDelivered > 0.0;
            target.LossPercent = (float)(profile.LossProbabilityAfterDelivered * 100.0);

            // Equal after-delivered/after-lost is the degenerate Bernoulli case (docs/adr/0004's
            // note on `150ms-20j-0.5loss`), so it round-trips to "bursty off" rather than to a
            // burst chain that happens to behave like Bernoulli. Unequal means a real chain.
            target.EnableBurstLoss =
                profile.LossProbabilityAfterLost != profile.LossProbabilityAfterDelivered;
            if (target.EnableBurstLoss)
            {
                target.BurstContinuationPercent = (float)(profile.LossProbabilityAfterLost * 100.0);
            }

            target.EnableReorder = profile.ReorderProbability > 0.0;
            target.ReorderPercent = (float)(profile.ReorderProbability * 100.0);
            if (profile.ReorderDelayTicks > 0)
            {
                target.ReorderDelayMs =
                    NetworkImpairmentSettings.TicksToMs(profile.ReorderDelayTicks, ticksPerSecond);
            }

            error = null;
            return true;
        }
    }
}
