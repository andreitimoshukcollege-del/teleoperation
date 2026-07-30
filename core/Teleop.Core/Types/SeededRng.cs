// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// A self-contained, seeded pseudo-random generator: xorshift128+, seeded via splitmix64.
    /// Everywhere Core needs "randomness through an injected seeded RNG" (<c>Transport/CLAUDE.md</c>,
    /// the no-unseeded-randomness invariant), this is the type to inject.
    ///
    /// Hand-rolled rather than wrapping <c>System.Random</c> for one specific reason: this
    /// project's central law is one copy of Core, compiled by two runtimes (CoreCLR for
    /// <c>dotnet</c>, Mono/IL2CPP for the Quest build), and nothing guarantees
    /// <c>System.Random</c> produces bit-identical sequences for the same seed across those two
    /// runtimes — its algorithm is a BCL implementation detail, not a spec. "Same seed and same
    /// input produce the same output, every run" (this project's determinism requirement, and
    /// Gate 3's literal wording) would then silently depend on which runtime happened to run it.
    /// xorshift128+ and splitmix64 are both pure 64-bit integer arithmetic — add, xor, shift,
    /// multiply — which every .NET runtime is required to execute identically, so this type has
    /// no such gap.
    ///
    /// A mutable struct, unlike most of <c>Types/</c>: a generator's whole purpose is evolving
    /// state on every draw, so making it immutable would just relocate the mutation into an
    /// awkward <c>(SeededRng, ulong) NextUInt64(SeededRng)</c>-shaped API for no benefit. It is
    /// still a struct — stack-allocated, no heap allocation — consistent with the no-allocation
    /// hot-path invariant; callers store one as a field and call its methods directly on that
    /// field (never copy it mid-sequence, or the copy and the original silently diverge).
    /// </summary>
    public struct SeededRng
    {
        private readonly ulong _seed;
        private ulong _state0;
        private ulong _state1;

        /// <summary>
        /// Seeds the generator. Two instances constructed with the same
        /// <paramref name="seed"/> produce identical sequences, forever, on every runtime this
        /// project targets.
        /// </summary>
        public SeededRng(ulong seed)
        {
            _seed = seed;
            (_state0, _state1) = SplitMix64Init(seed);
        }

        /// <summary>Next 64 bits of the stream. Allocation-free.</summary>
        public ulong NextUInt64()
        {
            ulong x = _state0;
            ulong y = _state1;
            _state0 = y;
            x ^= x << 23;
            x ^= x >> 17;
            x ^= y ^ (y >> 26);
            _state1 = x;
            return unchecked(x + y);
        }

        /// <summary>
        /// Next draw, uniform on [0, 1). Built from the top 53 bits of <see cref="NextUInt64"/>
        /// so every representable <c>double</c> mantissa value is reachable. Allocation-free.
        /// </summary>
        public double NextDouble() => (NextUInt64() >> 11) * (1.0 / (1UL << 53));

        /// <summary>
        /// Restores the generator to the state it had immediately after construction, so the
        /// next trial reproduces the previous one — the guarantee
        /// <c>ITransport.Reset()</c>'s own contract requires of any RNG it owns.
        /// </summary>
        public void Reset() => (_state0, _state1) = SplitMix64Init(_seed);

        private static (ulong, ulong) SplitMix64Init(ulong seed)
        {
            ulong state = seed;
            ulong s0 = SplitMix64Next(ref state);
            ulong s1 = SplitMix64Next(ref state);
            // xorshift128+ never validly starts at the all-zero state; splitmix64 output is zero
            // only for a vanishing fraction of seeds, but guard it explicitly rather than trust that.
            if (s0 == 0 && s1 == 0)
            {
                s0 = 0x9E3779B97F4A7C15UL;
            }
            return (s0, s1);
        }

        private static ulong SplitMix64Next(ref ulong state)
        {
            state = unchecked(state + 0x9E3779B97F4A7C15UL);
            ulong z = state;
            z = unchecked((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL);
            z = unchecked((z ^ (z >> 27)) * 0x94D049BB133111EBUL);
            return z ^ (z >> 31);
        }
    }
}
