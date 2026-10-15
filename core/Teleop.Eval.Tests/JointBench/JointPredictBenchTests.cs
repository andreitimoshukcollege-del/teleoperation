using Teleop.Core.Transport;
using Teleop.Core.Types;
using Teleop.Eval.JointBench;

namespace Teleop.Eval.Tests.JointBench
{
    public class JointPredictBenchTests
    {
        private static NetworkProfile Profile(string name)
        {
            Assert.True(NetworkProfileCatalog.TryResolveParametric(name, 1_000_000, out NetworkProfile p, out _));
            return p;
        }

        [Fact]
        public void TheSimulation_IsDeterministic()
        {
            var a = JointPredictBenchCommand.Simulate(Profile("50ms-5j"), "servo-replay", 3, 5, 4, 5);
            var b = JointPredictBenchCommand.Simulate(Profile("50ms-5j"), "servo-replay", 3, 5, 4, 5);
            Assert.Equal(a, b);
            Assert.NotEmpty(a);
        }

        [Fact]
        public void ServoReplay_BeatsHoldLast_WhenThereIsLatency()
        {
            static double P95(System.Collections.Generic.List<double> e) { e.Sort(); return e[(int)(0.95 * (e.Count - 1))]; }
            double hold = P95(JointPredictBenchCommand.Simulate(Profile("150ms-20j-0.5loss"), "hold-last", 1, 8, 4, 5));
            double replay = P95(JointPredictBenchCommand.Simulate(Profile("150ms-20j-0.5loss"), "servo-replay", 1, 8, 4, 5));
            Assert.True(replay < hold, $"servo-replay p95 {replay:0.00}° is not below hold-last {hold:0.00}°");
        }
    }
}
