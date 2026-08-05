using System.Globalization;
using System.Text.RegularExpressions;
using Teleop.Core.Types;

namespace Teleop.Eval.Sweep
{
    /// <summary>
    /// A resolved named network profile: either purely parametric, or trace-driven with a loaded
    /// delay trace. See <see cref="NetworkProfileCatalog.Resolve"/>.
    /// </summary>
    public readonly struct NamedProfile
    {
        public readonly string Name;
        public readonly NetworkProfile Profile;

        /// <summary>Null for a parametric profile; the loaded trace for a trace-driven one.</summary>
        public readonly long[]? TraceTicks;

        public NamedProfile(string name, NetworkProfile profile, long[]? traceTicks)
        {
            Name = name;
            Profile = profile;
            TraceTicks = traceTicks;
        }
    }

    /// <summary>
    /// Name -&gt; <see cref="NetworkProfile"/>/trace resolution for the frozen network-profile suite
    /// (<c>Transport/CLAUDE.md</c>, <c>docs/adr/0004-network-profile-suite.md</c>). Lives in
    /// <c>Teleop.Eval</c>, not <c>Registry/Registries.cs</c>: a profile is either a
    /// <see cref="NetworkProfile"/> literal or a trace file loaded from disk, and file I/O is a
    /// host concern, not Core's -- <see cref="NetworkProfile"/> also isn't a <c>Contracts/</c>
    /// interface implementation, unlike everything else <c>Registries.cs</c> holds.
    ///
    /// Exact parameter values are recorded and justified in
    /// <c>docs/adr/0004-network-profile-suite.md</c>, not re-derived here.
    ///
    /// Also resolves the isolated single-variable sensitivity families from
    /// <c>docs/adr/0005-isolated-impairment-profiles.md</c> (<c>jitter-&lt;N&gt;ms</c>,
    /// <c>delay-&lt;N&gt;ms</c>, <c>loss-&lt;N&gt;pct</c>) via <see cref="TryResolveIsolatedAxisProfile"/>,
    /// by regex rather than one named case per point -- see that method's doc comment.
    ///
    /// Also resolves the combined multi-axis family from
    /// <c>docs/adr/0006-combined-impairment-profiles.md</c> (<c>combo__delay-&lt;N&gt;ms__jitter-&lt;N&gt;ms__loss-&lt;N&gt;pct</c>,
    /// any 2-or-3-axis subset) via <see cref="TryResolveCombinedProfile"/> -- unlike the isolated
    /// family, an axis absent from the name is 0, not a nonzero baseline, since this family
    /// represents one chosen composite condition rather than isolating a variable.
    ///
    /// <c>cellular-congested</c>, <c>leo-satellite</c>, and <c>long-haul</c> are reserved names
    /// from the frozen 7-name set that specifically imply a real network capture. They are not
    /// resolvable yet -- <see cref="Resolve"/> reports them as a distinct "not yet available"
    /// failure rather than silently treating an unknown name and a not-yet-captured real trace the
    /// same way.
    /// </summary>
    public static class NetworkProfileCatalog
    {
        private static readonly HashSet<string> ReservedPendingRealCapture = new HashSet<string>(
            StringComparer.Ordinal) { "cellular-congested", "leo-satellite", "long-haul" };

        /// <summary>
        /// Resolves <paramref name="name"/> against the catalog. Returns false for an unknown name
        /// or a name reserved pending a real capture, with <paramref name="error"/> distinguishing
        /// the two so a sweep's failure message doesn't conflate "you typo'd this" with "this
        /// doesn't exist yet."
        /// </summary>
        public static bool TryResolve(
            string name, long ticksPerSecond, string tracesDirectory, out NamedProfile profile, out string? error)
        {
            switch (name)
            {
                case "lan":
                    profile = new NamedProfile(name, new NetworkProfile(
                        baseDelayTicks: MsToTicks(2, ticksPerSecond), jitterTicks: MsToTicks(1, ticksPerSecond),
                        lossProbabilityAfterDelivered: 0.0, lossProbabilityAfterLost: 0.0,
                        reorderProbability: 0.0, reorderDelayTicks: 0), traceTicks: null);
                    error = null;
                    return true;

                case "50ms-5j":
                    profile = new NamedProfile(name, new NetworkProfile(
                        baseDelayTicks: MsToTicks(50, ticksPerSecond), jitterTicks: MsToTicks(5, ticksPerSecond),
                        lossProbabilityAfterDelivered: 0.0, lossProbabilityAfterLost: 0.0,
                        reorderProbability: 0.0, reorderDelayTicks: 0), traceTicks: null);
                    error = null;
                    return true;

                case "150ms-20j-0.5loss":
                    // Not "bursty" in the name: equal after-delivered/after-lost probabilities
                    // degenerate the Gilbert-Elliott chain to plain Bernoulli loss at 0.5%.
                    profile = new NamedProfile(name, new NetworkProfile(
                        baseDelayTicks: MsToTicks(150, ticksPerSecond), jitterTicks: MsToTicks(20, ticksPerSecond),
                        lossProbabilityAfterDelivered: 0.005, lossProbabilityAfterLost: 0.005,
                        reorderProbability: 0.0, reorderDelayTicks: 0), traceTicks: null);
                    error = null;
                    return true;

                case "300ms-60j-2loss-bursty":
                    // Tuned so the chain's steady-state loss rate is ~2% (matching the name) with
                    // an expected burst length of 1/(1-0.7) ≈ 3.33: solving
                    // p/(p+(1-0.7)) = 0.02 for p gives p ≈ 0.00612.
                    profile = new NamedProfile(name, new NetworkProfile(
                        baseDelayTicks: MsToTicks(300, ticksPerSecond), jitterTicks: MsToTicks(60, ticksPerSecond),
                        lossProbabilityAfterDelivered: 0.00612, lossProbabilityAfterLost: 0.7,
                        reorderProbability: 0.0, reorderDelayTicks: 0), traceTicks: null);
                    error = null;
                    return true;

                case "synthetic-burst":
                {
                    string path = Path.Combine(tracesDirectory, "synthetic-burst.trace");
                    if (!File.Exists(path))
                    {
                        profile = default;
                        error = $"trace file not found at {path} -- run 'gen-trace' first";
                        return false;
                    }

                    (long traceTicksPerSecond, long[] samples) = TraceFile.Read(path);
                    if (traceTicksPerSecond != ticksPerSecond)
                    {
                        profile = default;
                        error = $"trace was generated at {traceTicksPerSecond} ticks/sec, sweep clock is {ticksPerSecond}";
                        return false;
                    }

                    var zeroDelayProfile = new NetworkProfile(
                        baseDelayTicks: 0, jitterTicks: 0,
                        lossProbabilityAfterDelivered: 0.0, lossProbabilityAfterLost: 0.0,
                        reorderProbability: 0.0, reorderDelayTicks: 0);
                    profile = new NamedProfile(name, zeroDelayProfile, samples);
                    error = null;
                    return true;
                }

                default:
                    if (TryResolveIsolatedAxisProfile(name, ticksPerSecond, out NetworkProfile isolatedProfile))
                    {
                        profile = new NamedProfile(name, isolatedProfile, traceTicks: null);
                        error = null;
                        return true;
                    }

                    if (TryResolveCombinedProfile(name, ticksPerSecond, out NetworkProfile combinedProfile))
                    {
                        profile = new NamedProfile(name, combinedProfile, traceTicks: null);
                        error = null;
                        return true;
                    }

                    profile = default;
                    error = ReservedPendingRealCapture.Contains(name)
                        ? $"'{name}' is reserved for a real network capture, not yet available -- see docs/adr/0004-network-profile-suite.md"
                        : $"unknown network profile '{name}'";
                    return false;
            }
        }

        private static long MsToTicks(double ms, long ticksPerSecond) => (long)(ms / 1000.0 * ticksPerSecond);

        private static readonly Regex JitterAxisPattern = new Regex(@"^jitter-(\d+(?:\.\d+)?)ms$", RegexOptions.Compiled);
        private static readonly Regex DelayAxisPattern = new Regex(@"^delay-(\d+(?:\.\d+)?)ms$", RegexOptions.Compiled);
        private static readonly Regex LossAxisPattern = new Regex(@"^loss-(\d+(?:\.\d+)?)pct$", RegexOptions.Compiled);

        /// <summary>
        /// Isolated single-variable sensitivity profiles from
        /// <c>docs/adr/0005-isolated-impairment-profiles.md</c> -- "jitter-&lt;N&gt;ms",
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

        private static readonly Regex CombinedDelaySegmentPattern = new Regex(@"^delay-(\d+(?:\.\d+)?)ms$", RegexOptions.Compiled);
        private static readonly Regex CombinedJitterSegmentPattern = new Regex(@"^jitter-(\d+(?:\.\d+)?)ms$", RegexOptions.Compiled);
        private static readonly Regex CombinedLossSegmentPattern = new Regex(@"^loss-(\d+(?:\.\d+)?)pct$", RegexOptions.Compiled);

        /// <summary>
        /// Combined multi-axis profiles from
        /// <c>docs/adr/0006-combined-impairment-profiles.md</c> --
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
