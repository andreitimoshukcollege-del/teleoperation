using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.Core.Registry;
using Teleop.Eval.Verification;
using Xunit;

namespace Teleop.Eval.Tests.Verification
{
    /// <summary>verify's algorithm replay (the 2026-11 bug hunt, A-03): it covers every registry entry, and it can fail.</summary>
    public sealed class DeterminismCheckTests
    {
        [Fact]
        public void EveryRegistryEntry_ReplaysIdentically_AndIsCounted()
        {
            List<string> failures = DeterminismCheck.Run(out int stacks);
            Assert.Empty(failures);
            // Every pose predictor, reconciler and playout policy once (the defaults are not run twice), and every joint
            // predictor the JetRover bench runs (franka-replay needs the FR3's shaper).
            int expected = Registries.Predictors.Count + (Registries.Reconcilers.Count - 1) + (Registries.PlayoutPolicies.Count - 1)
                + Registries.JointPredictors.Keys.Count(k => k != "franka-replay");
            Assert.Equal(expected, stacks);
        }

        [Fact]
        public void ARunThatDiffers_Throws_OrRecordsNothing_IsAFailure()
        {
            var failures = new List<string>();
            int call = 0;
            DeterminismCheck.Compare("leaks state", failures, () => new List<(string, long, long)> { ("x", BitConverter.DoubleToInt64Bits(++call), 0L) });
            DeterminismCheck.Compare("throws", failures, () => throw new InvalidOperationException("boom"));
            DeterminismCheck.Compare("silent", failures, () => new List<(string, long, long)>());
            DeterminismCheck.Compare("steady", failures, () => new List<(string, long, long)> { ("x", 1L, 5L) });
            Assert.Equal(3, failures.Count);
            Assert.Contains(failures, f => f.StartsWith("leaks state: sample 0 differs", StringComparison.Ordinal));
            Assert.Contains(failures, f => f.StartsWith("throws: threw InvalidOperationException", StringComparison.Ordinal));
            Assert.Contains(failures, f => f.StartsWith("silent: recorded nothing", StringComparison.Ordinal));
        }
    }
}
