using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

namespace Teleop.Eval.Sweep
{
    /// <summary>
    /// A resolved named network profile: either purely parametric, or trace-driven with a loaded
    /// delay trace. See <see cref="NetworkProfileCatalog.TryResolve"/>.
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

        /// <summary>
        /// The impairment set this named profile resolves to, which is what
        /// <c>EmulatedTransport</c> actually consumes. <see cref="Profile"/> is retained alongside
        /// it as the record of what the name means -- the manifest reports it, and ADR 0004 froze
        /// its numbers -- but nothing on the hot path reads it any more (docs/adr/0013).
        ///
        /// <b>Fresh instances on every call, and that is not an optimisation detail.</b> An
        /// impairment owns mutable model state and one RNG substream, and rejects a second
        /// <c>Bind</c>. Handing the same array to a link's uplink and downlink would either throw
        /// or -- if it did not -- make both directions lose and delay the same datagrams together,
        /// which no real pair of paths does. Call it once per transport.
        ///
        /// A trace-driven profile puts the trace axis first, then whatever loss and reordering the
        /// profile carries. Order does not affect the outcome (see <c>DatagramFate</c>), so this is
        /// for readability in a manifest, not for behaviour.
        /// </summary>
        public INetworkImpairment[] CreateImpairments()
        {
            INetworkImpairment[] parametric =
                Teleop.Core.Transport.NetworkProfileCatalog.CreateImpairments(Profile);

            if (TraceTicks == null)
            {
                return parametric;
            }

            var result = new INetworkImpairment[parametric.Length + 1];
            result[0] = Teleop.Core.Transport.NetworkProfileCatalog.CreateTraceDelay(TraceTicks);
            Array.Copy(parametric, 0, result, 1, parametric.Length);
            return result;
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
    /// The parametric cases (the 4 named profiles, plus the isolated and combined regex families)
    /// are resolved by <see cref="Teleop.Core.Transport.NetworkProfileCatalog.TryResolveParametric"/>
    /// -- promoted there (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md) once a
    /// second host (Unity, for the JetRover VR drag feature) needed the identical by-name
    /// resolution this class already gave <c>sweep</c>. This class now only adds what Core
    /// structurally cannot have: <c>synthetic-burst</c>'s trace-file loading, and the
    /// reserved-pending-real-capture distinction below.
    ///
    /// <c>cellular-congested</c>, <c>leo-satellite</c>, and <c>long-haul</c> are reserved names
    /// from the frozen 7-name set that specifically imply a real network capture. They are not
    /// resolvable yet -- <see cref="TryResolve"/> reports them as a distinct "not yet available"
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
            if (name == "synthetic-burst")
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

            if (Teleop.Core.Transport.NetworkProfileCatalog.TryResolveParametric(
                name, ticksPerSecond, out NetworkProfile parametricProfile, out _))
            {
                profile = new NamedProfile(name, parametricProfile, traceTicks: null);
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
}
