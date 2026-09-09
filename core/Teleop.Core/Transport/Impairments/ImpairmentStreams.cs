// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Transport.Impairments
{
    /// <summary>
    /// Derives each impairment's private RNG substream seed from the trial seed and its axis name.
    ///
    /// <b>Why substreams exist at all.</b> Before docs/adr/0013, common random numbers across a
    /// sweep was achieved by hardcoding the draw shape: <c>EmulatedTransport</c> drew for jitter and
    /// reorder on every datagram even when those knobs were zero, so that two profiles sharing a
    /// seed stayed aligned. That trick cannot survive composition — "do not include a jitter
    /// impairment" removes a draw and shifts every subsequent one for every other axis. Giving each
    /// axis its own stream replaces it with something stronger: adding or removing an axis leaves
    /// every other axis's realization untouched, which the old design could not do at all.
    ///
    /// <b>Everything here is paid once, at Bind.</b> Nothing in this class runs on the hot path.
    /// </summary>
    public static class ImpairmentStreams
    {
        private const ulong Fnv1a64OffsetBasis = 14695981039346656037UL;
        private const ulong Fnv1a64Prime = 1099511628211UL;

        /// <summary>Golden-ratio odd constant, the same one <c>SeededRng</c>'s SplitMix64 init uses.</summary>
        private const ulong GoldenRatio64 = 0x9E3779B97F4A7C15UL;

        /// <summary>
        /// FNV-1a-64 over <paramref name="axisName"/>'s UTF-16 code units, low byte then high byte.
        ///
        /// <b>Hand-rolled, and deliberately not <c>string.GetHashCode()</c>.</b> .NET Core randomises
        /// string hashing per process, so a <c>GetHashCode</c>-derived substream would produce a
        /// different stream on every run of the same code with the same seed. That failure is
        /// invisible to any test comparing two instances inside one process — both would agree, and
        /// both would be wrong tomorrow — so it would surface only as sweeps that mysteriously never
        /// reproduce. This is the single most likely way to get this design subtly wrong.
        ///
        /// Pure integer arithmetic, so CoreCLR, Mono and IL2CPP all compute it identically. The
        /// two-byte split is written explicitly rather than reinterpreting memory, so the result
        /// does not depend on endianness. Ordinal by construction: no culture, no casing rules —
        /// the same discipline as <c>Registries</c>' explicit <c>StringComparer.Ordinal</c>.
        /// </summary>
        public static ulong StreamIdFromName(string axisName)
        {
            if (axisName == null)
            {
                return Fnv1a64OffsetBasis;
            }

            unchecked
            {
                ulong hash = Fnv1a64OffsetBasis;
                for (int i = 0; i < axisName.Length; i++)
                {
                    char c = axisName[i];

                    hash ^= (ulong)(byte)(c & 0xFF);
                    hash *= Fnv1a64Prime;

                    hash ^= (ulong)(byte)((c >> 8) & 0xFF);
                    hash *= Fnv1a64Prime;
                }

                return hash;
            }
        }

        /// <summary>
        /// Combines a trial seed with a stream id into a substream seed.
        ///
        /// The SplitMix64 finalizer, applied to <c>seed + streamId * golden</c>. Two properties
        /// matter and both come from the finalizer's avalanche: adjacent trial seeds produce
        /// uncorrelated substreams (<c>SweepCommand</c> uses <c>seed</c> and <c>seed + 1</c> for the
        /// two link directions), and two different axes under the same trial seed do too.
        ///
        /// The result is fed to <c>new SeededRng(...)</c>, which runs SplitMix64 init over it again
        /// — two independent mixes. That is belt-and-braces rather than necessary, and it is free
        /// because it happens once per impairment per trial.
        /// </summary>
        public static ulong Derive(ulong trialSeed, ulong streamId)
        {
            unchecked
            {
                ulong z = trialSeed + (streamId * GoldenRatio64);
                z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
                z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
                return z ^ (z >> 31);
            }
        }

        /// <summary>Convenience for the transport: <c>Derive(trialSeed, StreamIdFromName(axisName))</c>.</summary>
        public static ulong DeriveForAxis(ulong trialSeed, string axisName) =>
            Derive(trialSeed, StreamIdFromName(axisName));
    }
}
