using Teleop.Core.Transport;

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
    }
}
