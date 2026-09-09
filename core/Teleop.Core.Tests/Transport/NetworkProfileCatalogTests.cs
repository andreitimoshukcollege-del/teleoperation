using Teleop.Core.Contracts;
using Teleop.Core.Transport;
using Teleop.Core.Transport.Impairments;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Transport
{
    public class NetworkProfileCatalogTests
    {
        private const long TicksPerSecond = 10_000_000;

        private static long MsToTicks(double ms) => (long)(ms / 1000.0 * TicksPerSecond);

        [Fact]
        public void TryResolveParametric_Lan_MatchesDocumentedValues()
        {
            bool ok = NetworkProfileCatalog.TryResolveParametric("lan", TicksPerSecond, out var profile, out string? error);

            Assert.True(ok);
            Assert.Null(error);
            Assert.Equal(MsToTicks(2), profile.BaseDelayTicks);
            Assert.Equal(MsToTicks(1), profile.JitterTicks);
            Assert.Equal(0.0, profile.LossProbabilityAfterDelivered);
            Assert.Equal(0.0, profile.LossProbabilityAfterLost);
        }

        [Fact]
        public void TryResolveParametric_300ms60j2lossBursty_MatchesDocumentedBurstShape()
        {
            bool ok = NetworkProfileCatalog.TryResolveParametric(
                "300ms-60j-2loss-bursty", TicksPerSecond, out var profile, out _);

            Assert.True(ok);
            Assert.Equal(MsToTicks(300), profile.BaseDelayTicks);
            Assert.Equal(MsToTicks(60), profile.JitterTicks);
            Assert.Equal(0.7, profile.LossProbabilityAfterLost);
            // Expected steady-state loss rate ~2%, per Transport/CLAUDE.md's documented tuning.
            double steadyStateLoss = profile.LossProbabilityAfterDelivered
                / (profile.LossProbabilityAfterDelivered + (1.0 - profile.LossProbabilityAfterLost));
            Assert.True(Math.Abs(steadyStateLoss - 0.02) < 0.001, $"expected ~2% steady-state loss, got {steadyStateLoss:P2}");
        }

        [Theory]
        [InlineData("jitter-15ms")]
        [InlineData("delay-75ms")]
        [InlineData("loss-3pct")]
        public void TryResolveParametric_IsolatedAxisNames_Resolve(string name)
        {
            bool ok = NetworkProfileCatalog.TryResolveParametric(name, TicksPerSecond, out _, out string? error);

            Assert.True(ok);
            Assert.Null(error);
        }

        [Fact]
        public void TryResolveParametric_IsolatedJitterAxis_HoldsDelayAtFixedCompanionValue()
        {
            NetworkProfileCatalog.TryResolveParametric("jitter-15ms", TicksPerSecond, out var profile, out _);

            Assert.Equal(MsToTicks(15), profile.JitterTicks);
            Assert.Equal(MsToTicks(50), profile.BaseDelayTicks); // fixed companion, per docs/adr/0005
        }

        [Fact]
        public void TryResolveParametric_CombinedProfile_LeavesAbsentAxesAtZero()
        {
            bool ok = NetworkProfileCatalog.TryResolveParametric(
                "combo__delay-100ms__loss-1pct", TicksPerSecond, out var profile, out _);

            Assert.True(ok);
            Assert.Equal(MsToTicks(100), profile.BaseDelayTicks);
            Assert.Equal(0L, profile.JitterTicks); // jitter absent from the name -> 0, not a baseline
            Assert.Equal(0.01, profile.LossProbabilityAfterDelivered);
        }

        [Fact]
        public void TryResolveParametric_CombinedProfileWithDuplicateAxis_Fails()
        {
            bool ok = NetworkProfileCatalog.TryResolveParametric(
                "combo__delay-100ms__delay-200ms", TicksPerSecond, out _, out _);

            Assert.False(ok);
        }

        [Fact]
        public void TryResolveParametric_UnknownName_ReturnsFalseWithError()
        {
            bool ok = NetworkProfileCatalog.TryResolveParametric("not-a-real-profile", TicksPerSecond, out _, out string? error);

            Assert.False(ok);
            Assert.NotNull(error);
        }

        [Fact]
        public void TryResolveParametric_TraceBackedName_IsNotResolvedHere()
        {
            // synthetic-burst needs real file I/O (Teleop.Eval's job, not Core's) -- this class
            // must not claim to resolve it.
            bool ok = NetworkProfileCatalog.TryResolveParametric("synthetic-burst", TicksPerSecond, out _, out _);

            Assert.False(ok);
        }

        /// <summary>
        /// <b>The test docs/adr/0013's central claim rests on: the frozen numbers did not move.</b>
        ///
        /// For every name in the frozen suite (ADRs 0004, 0005, 0006), the impairment set the
        /// catalog now emits must carry parameters exactly equal to the <c>NetworkProfile</c> the
        /// catalog has always returned. Exactly, not approximately -- a one-tick rounding difference
        /// would mean the frozen *numbers* changed, not merely their representation, and that is an
        /// amendment to three ADRs rather than a refactor.
        ///
        /// Note what this deliberately does not assert: that the two produce the same random
        /// realization. They do not, and cannot -- per-impairment substreams changed the draw
        /// sequence, which ADR 0013 authorises and records. Asserting stream equality here would
        /// fail for a reason unrelated to correctness, and the temptation would then be to weaken
        /// it. Parameters are the right thing to pin, because parameters are what the ADRs froze.
        /// </summary>
        [Theory]
        [InlineData("lan")]
        [InlineData("150ms-20j-0.5loss")]
        [InlineData("300ms-60j-2loss-bursty")]
        [InlineData("jitter-15ms")]
        [InlineData("delay-75ms")]
        [InlineData("loss-3pct")]
        [InlineData("combo__delay-100ms__loss-1pct")]
        [InlineData("combo__delay-50ms__jitter-10ms__loss-2pct")]
        public void CreateImpairments_ReproducesTheFrozenProfilesParametersExactly(string name)
        {
            Assert.True(NetworkProfileCatalog.TryResolveParametric(
                name, TicksPerSecond, out NetworkProfile profile, out _));

            INetworkImpairment[] impairments = NetworkProfileCatalog.CreateImpairments(profile);

            long delayTicks = 0;
            long jitterTicks = 0;
            double lossAfterDelivered = 0.0;
            double lossAfterLost = 0.0;
            double reorderProbability = 0.0;
            long reorderDelayTicks = 0;

            foreach (INetworkImpairment impairment in impairments)
            {
                switch (impairment)
                {
                    case FixedDelayImpairment d:
                        delayTicks = d.DelayTicks;
                        break;
                    case UniformJitterImpairment j:
                        jitterTicks = j.HalfWidthTicks;
                        break;
                    case GilbertElliottLossImpairment l:
                        lossAfterDelivered = l.LossProbabilityAfterDelivered;
                        lossAfterLost = l.LossProbabilityAfterLost;
                        break;
                    case ReorderImpairment r:
                        reorderProbability = r.Probability;
                        reorderDelayTicks = r.ExtraDelayTicks;
                        break;
                    default:
                        Assert.Fail($"Unexpected impairment type {impairment.GetType().Name} for '{name}'.");
                        break;
                }
            }

            Assert.Equal(profile.BaseDelayTicks, delayTicks);
            Assert.Equal(profile.JitterTicks, jitterTicks);
            Assert.Equal(profile.LossProbabilityAfterDelivered, lossAfterDelivered, 15);
            Assert.Equal(profile.LossProbabilityAfterLost, lossAfterLost, 15);
            Assert.Equal(profile.ReorderProbability, reorderProbability, 15);
            Assert.Equal(profile.ReorderDelayTicks, reorderDelayTicks);
        }

        /// <summary>
        /// A neutral axis is omitted rather than included at zero. Legal only because a neutral axis
        /// is observationally identical to its absence (asserted in <c>EmulatedTransportTests</c>),
        /// and worth pinning here because it is what keeps a manifest's impairment list honest --
        /// `lan` has no loss, so it should not list a loss axis at 0%.
        /// </summary>
        [Fact]
        public void CreateImpairments_OmitsNeutralAxesRatherThanZeroingThem()
        {
            Assert.True(NetworkProfileCatalog.TryResolveParametric(
                "lan", TicksPerSecond, out NetworkProfile lan, out _));

            INetworkImpairment[] impairments = NetworkProfileCatalog.CreateImpairments(lan);

            Assert.Equal(2, impairments.Length);
            Assert.Contains(impairments, i => i.AxisName == "delay");
            Assert.Contains(impairments, i => i.AxisName == "jitter");
            Assert.DoesNotContain(impairments, i => i.AxisName == "loss");
            Assert.DoesNotContain(impairments, i => i.AxisName == "reorder");
        }

        /// <summary>An all-zero profile yields an empty set: a legal, unimpaired decorator.</summary>
        [Fact]
        public void CreateImpairments_AllZeroProfile_YieldsAnEmptySet()
        {
            var zero = new NetworkProfile(0, 0, 0.0, 0.0, 0.0, 0);
            Assert.Empty(NetworkProfileCatalog.CreateImpairments(zero));
        }

        /// <summary>
        /// Each call must return new instances. An impairment owns mutable model state and one RNG
        /// substream and rejects a second bind, so a shared array would either throw or correlate a
        /// link's two directions.
        /// </summary>
        [Fact]
        public void CreateImpairments_ReturnsFreshInstancesEachCall()
        {
            Assert.True(NetworkProfileCatalog.TryResolveParametric(
                "300ms-60j-2loss-bursty", TicksPerSecond, out NetworkProfile profile, out _));

            INetworkImpairment[] first = NetworkProfileCatalog.CreateImpairments(profile);
            INetworkImpairment[] second = NetworkProfileCatalog.CreateImpairments(profile);

            for (int i = 0; i < first.Length; i++)
            {
                Assert.NotSame(first[i], second[i]);
            }
        }

        /// <summary>
        /// The burst-length figure a manifest reports (from the struct) and the one the live model
        /// implies (from the impairment) must come from the same expression, so a paper and a run
        /// cannot disagree about what "bursty" meant.
        /// </summary>
        [Fact]
        public void ExpectedBurstLength_AgreesBetweenTheProfileRecordAndTheLiveImpairment()
        {
            Assert.True(NetworkProfileCatalog.TryResolveParametric(
                "300ms-60j-2loss-bursty", TicksPerSecond, out NetworkProfile profile, out _));

            var loss = (GilbertElliottLossImpairment)Assert.Single(
                NetworkProfileCatalog.CreateImpairments(profile),
                i => i is GilbertElliottLossImpairment);

            Assert.Equal(profile.ExpectedBurstLength, loss.ExpectedBurstLength, 12);
        }
    }
}
