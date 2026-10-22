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

        [Fact]
        public void UnderTheRelay_WithNoServo_RelayReplayServoIsRelayReplay_SampleForSample()
        {
            var options = new JointPredictBenchCommand.RelayOptions();
            var relay = JointPredictBenchCommand.SimulateRelay(Profile("50ms-5j"), "relay-replay", 2, 4, 6, options);
            var servo = JointPredictBenchCommand.SimulateRelay(Profile("50ms-5j"), "relay-replay-servo", 2, 4, 6, options);
            Assert.NotEmpty(relay.Errors);
            Assert.Equal(relay.Errors, servo.Errors);
            Assert.Equal(relay.Corrections, servo.Corrections);
            Assert.Equal(0, servo.ServoSteps);
        }

        [Fact]
        public void UnderTheRelay_WithAServoAndAPipeline_RelayReplayServoBeatsRelayReplay()
        {
            static double P95(System.Collections.Generic.List<double> e) { e.Sort(); return e[(int)(0.95 * (e.Count - 1))]; }
            var options = new JointPredictBenchCommand.RelayOptions { ServoMaxSpeedRadPerSec = 5, ServoLagMs = 20, PipelineMs = 20 };
            var relay = JointPredictBenchCommand.SimulateRelay(Profile("lan"), "relay-replay", 1, 6, 6, options);
            var servo = JointPredictBenchCommand.SimulateRelay(Profile("lan"), "relay-replay-servo", 1, 6, 6, options);
            Assert.True(P95(servo.Errors) < P95(relay.Errors),
                $"relay-replay-servo p95 {P95(servo.Errors):0.00}° is not below relay-replay {P95(relay.Errors):0.00}°");
            Assert.True(servo.ServoSteps > 0);
        }

        [Fact]
        public void FasterWaypoints_MakeTheSimulatedServosSpeedCapBind()
        {
            var options = new JointPredictBenchCommand.RelayOptions { ServoMaxSpeedRadPerSec = 5, ServoLagMs = 20 };
            var normal = JointPredictBenchCommand.SimulateRelay(Profile("lan"), "hold-last", 1, 4, 6, options);
            var fast = JointPredictBenchCommand.SimulateRelay(Profile("lan"), "hold-last", 1, 4, 6, options, legScale: 0.25);
            Assert.True(fast.ServoCappedSteps > 10 * normal.ServoCappedSteps + 100,
                $"the cap bound in {fast.ServoCappedSteps} steps with fast waypoints, {normal.ServoCappedSteps} without");
        }
    }
}
