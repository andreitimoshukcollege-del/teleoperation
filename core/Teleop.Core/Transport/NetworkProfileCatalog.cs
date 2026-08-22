using System;
using System.Globalization;
using System.Text.RegularExpressions;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport
{
    /// <summary>
    /// Name -&gt; <see cref="NetworkProfile"/> resolution for the frozen network-profile suite's
    /// parametric cases (the 4 named profiles from docs/adr/0004-network-profile-suite.md, the
    /// isolated single-variable families from docs/adr/0005-isolated-impairment-profiles.md, and
    /// the combined multi-axis family from docs/adr/0006-combined-impairment-profiles.md).
    ///
    /// Lives here, not only in <c>Teleop.Eval</c>, because a second host now needs the identical
    /// by-name resolution <c>sweep</c> already has -- promoted from
    /// <c>Teleop.Eval/Sweep/NetworkProfileCatalog.cs</c>, which still owns everything this class
    /// deliberately doesn't: trace-file-backed profiles (<c>synthetic-burst</c>), which need real
    /// file I/O, a host concern Core is forbidden from having. <c>Teleop.Eval</c>'s own
    /// <c>NetworkProfileCatalog.TryResolve</c> delegates to <see cref="TryResolveParametric"/> for
    /// every name this class recognizes and only handles the trace-file case itself
    /// (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md records this move).
    ///
    /// This is a pure function of (name, ticksPerSecond): no file I/O, no NuGet, no allocation
    /// beyond what <see cref="Regex"/> itself needs, safe under everything Core's own hard
    /// constraints already require. One real fix made during the move: the regexes below do
    /// <b>not</b> use <see cref="RegexOptions.Compiled"/> (unlike the original
    /// <c>Teleop.Eval</c> copy) -- that flag JIT-compiles the pattern via runtime code generation,
    /// which does not work under IL2CPP full-AOT (root CLAUDE.md invariant 5). Harmless in
    /// <c>Teleop.Eval</c> (`dotnet`-only, never compiled for Unity); a real bug waiting to happen
    /// now that this class is part of Core.
    /// </summary>
    public static class NetworkProfileCatalog
    {
        /// <summary>
        /// Resolves <paramref name="name"/> against the parametric portion of the catalog.
        /// Returns false for any name this class doesn't recognize -- including
        /// <c>synthetic-burst</c> and the reserved-pending-real-capture names, which are
        /// <c>Teleop.Eval</c>'s to report distinctly, not this class's.
        /// </summary>
        public static bool TryResolveParametric(
            string name, long ticksPerSecond, out NetworkProfile profile, out string? error)
        {
            switch (name)
            {
                case "lan":
                    profile = new NetworkProfile(
                        baseDelayTicks: MsToTicks(2, ticksPerSecond), jitterTicks: MsToTicks(1, ticksPerSecond),
                        lossProbabilityAfterDelivered: 0.0, lossProbabilityAfterLost: 0.0,
                        reorderProbability: 0.0, reorderDelayTicks: 0);
                    error = null;
                    return true;

                case "50ms-5j":
                    profile = new NetworkProfile(
                        baseDelayTicks: MsToTicks(50, ticksPerSecond), jitterTicks: MsToTicks(5, ticksPerSecond),
                        lossProbabilityAfterDelivered: 0.0, lossProbabilityAfterLost: 0.0,
                        reorderProbability: 0.0, reorderDelayTicks: 0);
                    error = null;
                    return true;

                case "150ms-20j-0.5loss":
                    // Not "bursty" in the name: equal after-delivered/after-lost probabilities
                    // degenerate the Gilbert-Elliott chain to plain Bernoulli loss at 0.5%.
                    profile = new NetworkProfile(
                        baseDelayTicks: MsToTicks(150, ticksPerSecond), jitterTicks: MsToTicks(20, ticksPerSecond),
                        lossProbabilityAfterDelivered: 0.005, lossProbabilityAfterLost: 0.005,
                        reorderProbability: 0.0, reorderDelayTicks: 0);
                    error = null;
                    return true;

                case "300ms-60j-2loss-bursty":
                    // Tuned so the chain's steady-state loss rate is ~2% (matching the name) with
                    // an expected burst length of 1/(1-0.7) ~ 3.33: solving
                    // p/(p+(1-0.7)) = 0.02 for p gives p ~ 0.00612.
                    profile = new NetworkProfile(
                        baseDelayTicks: MsToTicks(300, ticksPerSecond), jitterTicks: MsToTicks(60, ticksPerSecond),
                        lossProbabilityAfterDelivered: 0.00612, lossProbabilityAfterLost: 0.7,
                        reorderProbability: 0.0, reorderDelayTicks: 0);
                    error = null;
                    return true;

                default:
                    if (TryResolveIsolatedAxisProfile(name, ticksPerSecond, out NetworkProfile isolatedProfile))
                    {
                        profile = isolatedProfile;
                        error = null;
                        return true;
                    }

                    if (TryResolveCombinedProfile(name, ticksPerSecond, out NetworkProfile combinedProfile))
                    {
                        profile = combinedProfile;
                        error = null;
                        return true;
                    }

                    profile = default;
                    error = $"unknown network profile '{name}'";
                    return false;
            }
        }

        private static long MsToTicks(double ms, long ticksPerSecond) => (long)(ms / 1000.0 * ticksPerSecond);

        private static readonly Regex JitterAxisPattern = new Regex(@"^jitter-(\d+(?:\.\d+)?)ms$");
        private static readonly Regex DelayAxisPattern = new Regex(@"^delay-(\d+(?:\.\d+)?)ms$");
        private static readonly Regex LossAxisPattern = new Regex(@"^loss-(\d+(?:\.\d+)?)pct$");

        /// <summary>
        /// Isolated single-variable sensitivity profiles from
        /// docs/adr/0005-isolated-impairment-profiles.md -- "jitter-&lt;N&gt;ms",
        /// "delay-&lt;N&gt;ms", "loss-&lt;N&gt;pct", each varying exactly one
        /// <see cref="NetworkProfile"/> parameter while holding the other two at that family's
        /// fixed companion values. Resolved by pattern rather than one hand-written case per
        /// point, per that ADR's reasoning -- unlike the named profiles above, an individual
        /// point's value isn't itself a citable decision; the family's shape (fixed companions,
        /// even spacing) is.
        /// </summary>
        private static bool TryResolveIsolatedAxisProfile(
            string name, long ticksPerSecond, out NetworkProfile profile)
        {
            Match jitterMatch = JitterAxisPattern.Match(name);
            if (jitterMatch.Success)
            {
                double jitterMs = double.Parse(jitterMatch.Groups[1].Value, CultureInfo.InvariantCulture);
                profile = new NetworkProfile(
                    baseDelayTicks: MsToTicks(50, ticksPerSecond), jitterTicks: MsToTicks(jitterMs, ticksPerSecond),
                    lossProbabilityAfterDelivered: 0.0, lossProbabilityAfterLost: 0.0,
                    reorderProbability: 0.0, reorderDelayTicks: 0);
                return true;
            }

            Match delayMatch = DelayAxisPattern.Match(name);
            if (delayMatch.Success)
            {
                double delayMs = double.Parse(delayMatch.Groups[1].Value, CultureInfo.InvariantCulture);
                profile = new NetworkProfile(
                    baseDelayTicks: MsToTicks(delayMs, ticksPerSecond), jitterTicks: MsToTicks(5, ticksPerSecond),
                    lossProbabilityAfterDelivered: 0.0, lossProbabilityAfterLost: 0.0,
                    reorderProbability: 0.0, reorderDelayTicks: 0);
                return true;
            }

            Match lossMatch = LossAxisPattern.Match(name);
            if (lossMatch.Success)
            {
                double lossPercent = double.Parse(lossMatch.Groups[1].Value, CultureInfo.InvariantCulture);
                double lossProbability = lossPercent / 100.0;
                // Equal after-delivered/after-lost keeps ExpectedBurstLength ~1 at every point --
                // this family isolates loss rate, not burst shape (docs/adr/0005).
                profile = new NetworkProfile(
                    baseDelayTicks: MsToTicks(100, ticksPerSecond), jitterTicks: MsToTicks(10, ticksPerSecond),
                    lossProbabilityAfterDelivered: lossProbability, lossProbabilityAfterLost: lossProbability,
                    reorderProbability: 0.0, reorderDelayTicks: 0);
                return true;
            }

            profile = default;
            return false;
        }

        private static readonly Regex CombinedDelaySegmentPattern = new Regex(@"^delay-(\d+(?:\.\d+)?)ms$");
        private static readonly Regex CombinedJitterSegmentPattern = new Regex(@"^jitter-(\d+(?:\.\d+)?)ms$");
        private static readonly Regex CombinedLossSegmentPattern = new Regex(@"^loss-(\d+(?:\.\d+)?)pct$");

        /// <summary>
        /// Combined multi-axis profiles from docs/adr/0006-combined-impairment-profiles.md --
        /// <c>combo__delay-&lt;N&gt;ms__jitter-&lt;N&gt;ms__loss-&lt;N&gt;pct</c>, with any subset of the
        /// three axis segments present (each at most once) in any order. Unlike
        /// <see cref="TryResolveIsolatedAxisProfile"/>, an axis segment that's absent resolves to
        /// 0 (no impairment on that axis), not a nonzero baseline -- this family is one composite
        /// condition the caller chose, not a variable being isolated against fixed companions.
        /// The Python generator that produces these names (<c>experiment_builder.combined_points</c>)
        /// enforces at least 2 axes; this resolver itself accepts 1+ so it stays a pure grammar
        /// check, not a policy check.
        /// </summary>
        private static bool TryResolveCombinedProfile(
            string name, long ticksPerSecond, out NetworkProfile profile)
        {
            const string prefix = "combo__";
            if (!name.StartsWith(prefix, StringComparison.Ordinal))
            {
                profile = default;
                return false;
            }

            string[] segments = name.Substring(prefix.Length).Split(
                new[] { "__" }, StringSplitOptions.None);
            if (segments.Length == 0)
            {
                profile = default;
                return false;
            }

            double? delayMs = null;
            double? jitterMs = null;
            double? lossPercent = null;

            foreach (string segment in segments)
            {
                Match delayMatch = CombinedDelaySegmentPattern.Match(segment);
                if (delayMatch.Success)
                {
                    if (delayMs.HasValue) { profile = default; return false; }
                    delayMs = double.Parse(delayMatch.Groups[1].Value, CultureInfo.InvariantCulture);
                    continue;
                }

                Match jitterMatch = CombinedJitterSegmentPattern.Match(segment);
                if (jitterMatch.Success)
                {
                    if (jitterMs.HasValue) { profile = default; return false; }
                    jitterMs = double.Parse(jitterMatch.Groups[1].Value, CultureInfo.InvariantCulture);
                    continue;
                }

                Match lossMatch = CombinedLossSegmentPattern.Match(segment);
                if (lossMatch.Success)
                {
                    if (lossPercent.HasValue) { profile = default; return false; }
                    lossPercent = double.Parse(lossMatch.Groups[1].Value, CultureInfo.InvariantCulture);
                    continue;
                }

                // Unrecognized segment -- not a combo name, let the caller report "unknown profile".
                profile = default;
                return false;
            }

            if (!delayMs.HasValue && !jitterMs.HasValue && !lossPercent.HasValue)
            {
                profile = default;
                return false;
            }

            double lossProbability = (lossPercent ?? 0.0) / 100.0;
            profile = new NetworkProfile(
                baseDelayTicks: MsToTicks(delayMs ?? 0.0, ticksPerSecond),
                jitterTicks: MsToTicks(jitterMs ?? 0.0, ticksPerSecond),
                // Equal after-delivered/after-lost -> plain Bernoulli loss (ExpectedBurstLength ~1),
                // same reasoning as the isolated loss family: this grammar doesn't expose burst
                // shape, only rate.
                lossProbabilityAfterDelivered: lossProbability, lossProbabilityAfterLost: lossProbability,
                reorderProbability: 0.0, reorderDelayTicks: 0);
            return true;
        }
    }
}
