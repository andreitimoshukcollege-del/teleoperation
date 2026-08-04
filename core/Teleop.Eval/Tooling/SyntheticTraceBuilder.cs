using Teleop.Core.Types;
using Teleop.Eval.Sweep;

namespace Teleop.Eval.Tooling
{
    /// <summary>
    /// Builds the committed <c>synthetic-burst.trace</c> fixture deterministically, the same
    /// discipline <see cref="GoldenSessionBuilder"/> uses for the golden <c>.tlog</c>: never
    /// hand-author trace samples, generate them here from a seeded RNG, inspect the diff, commit
    /// the result.
    ///
    /// Models a bimodal delay pattern a parametric <see cref="NetworkProfile"/> cannot express: a
    /// low, tightly-clustered baseline delay punctuated by periodic congestion bursts of clearly
    /// elevated delay lasting several consecutive samples -- exactly the shape
    /// <c>docs/adr/0004-network-profile-suite.md</c> uses to justify why trace-driven mode earns
    /// its keep over four more parametric knobs.
    /// </summary>
    public static class SyntheticTraceBuilder
    {
        private const ulong Seed = 20260804UL;
        private const int SampleCount = 2000;
        private const long BaselineDelayTicks = 200_000; // 20ms @ 10,000,000 ticks/sec
        private const long BaselineJitterTicks = 20_000; // ±2ms
        private const long BurstDelayTicks = 2_000_000; // 200ms
        private const long BurstJitterTicks = 500_000; // ±50ms
        private const double BurstStartProbabilityPerSample = 0.01; // ~once per 100 samples
        private const int BurstMinLength = 5;
        private const int BurstMaxLength = 15;

        public static void Build(string path, long ticksPerSecond = 10_000_000)
        {
            var rng = new SeededRng(Seed);
            var samples = new long[SampleCount];

            int burstRemaining = 0;
            for (int i = 0; i < SampleCount; i++)
            {
                if (burstRemaining == 0 && rng.NextDouble() < BurstStartProbabilityPerSample)
                {
                    burstRemaining = BurstMinLength + (int)(rng.NextDouble() * (BurstMaxLength - BurstMinLength + 1));
                }

                long delay;
                if (burstRemaining > 0)
                {
                    delay = BurstDelayTicks + UniformJitter(ref rng, BurstJitterTicks);
                    burstRemaining--;
                }
                else
                {
                    delay = BaselineDelayTicks + UniformJitter(ref rng, BaselineJitterTicks);
                }

                samples[i] = delay < 0 ? 0 : delay;
            }

            TraceFile.Write(path, ticksPerSecond, samples);
        }

        private static long UniformJitter(ref SeededRng rng, long halfWidth)
        {
            ulong span = ((ulong)halfWidth * 2UL) + 1UL;
            return (long)(rng.NextUInt64() % span) - halfWidth;
        }
    }
}
