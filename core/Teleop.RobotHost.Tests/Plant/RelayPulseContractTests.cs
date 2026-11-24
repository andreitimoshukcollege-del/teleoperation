using System;
using System.IO;
using System.Linq;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Backend;
using Teleop.RobotHost.Plant;

namespace Teleop.RobotHost.Tests.Plant
{
    /// <summary>
    /// The dataset builder's action is the command sent, a refused one included (docs/adr/0038 "M, motion", 2026-11-24): it
    /// turns a refused command's <c>JC</c> targets into relay pulses as this plant would have (robot/lerobot/teleop_lerobot/takes.py,
    /// <c>_jetrover_pulses</c>). Both sides pin the same vector, robot/lerobot/tests/test_takes.py's
    /// <c>test_the_action_of_a_refused_command_is_the_plants_own_conversion</c>. A change to the plant's conversion
    /// (<see cref="GenericArmPlantConfig.Default"/>, the profile's offsets or ranges) must change takes.py with it: the builder checks
    /// each take's accepted commands against their <c>RO</c> and refuses a take that disagrees.
    /// </summary>
    public sealed class RelayPulseContractTests
    {
        [Fact]
        public void AJointCommandsRelayPulses_AreTheDatasetBuildersConversion()
        {
            // The conversion alone: without the collision model the backstop takes no part, and the out-of-range targets reach it.
            RobotArmProfile profile = RobotArmProfileJson.Load(JetRoverProfile()).WithCollision(null);
            GenericArmPlantConfig d = GenericArmPlantConfig.Default;
            var config = new GenericArmPlantConfig(profile: profile, pulsePerRadian: d.PulsePerRadian, pulsesPerSecond: d.PulsesPerSecond,
                stepSizePulses: d.StepSizePulses, maxDirectionMagnitude: d.MaxDirectionMagnitude, zeroPulse: d.ZeroPulse, minPulse: d.MinPulse,
                maxPulse: d.MaxPulse, gripperOpenPulse: d.GripperOpenPulse, gripperClosedPulse: d.GripperClosedPulse);
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(config, relay);
            var targets = new[]
            {
                new JointTarget(1, 0f, 3.8f), new JointTarget(2, 0.64494798f, 3.8f), new JointTarget(3, 0.5f, 3.8f),
                new JointTarget(4, 3.0f, 3.8f), new JointTarget(5, -3.0f, 3.8f), new JointTarget(10, 0.5f, 3.8f),
            };

            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(targets, captureTicks: 10));

            // test_takes.py's numbers, by hand: 1000 pulses per 240 degrees from 500, the shoulder's mounting offset (-0.64494798 rad)
            // added, within 0..1000; the gripper's fraction from 116 (open) to 645 (closed).
            var pulses = Assert.Single(relay.SentCommands).ToDictionary(t => t.MotorId, t => t.Angle);
            byte[] motors = { 1, 2, 3, 4, 5, 10 };
            float[] expected = { 500.0f, 500.0f, 619.3662f, 1000.0f, 0.0f, 380.5f };
            Assert.Equal(motors.OrderBy(m => m), pulses.Keys.OrderBy(m => m));
            for (int i = 0; i < motors.Length; i++)
            {
                Assert.InRange(pulses[motors[i]], expected[i] - 1e-3f, expected[i] + 1e-3f);
            }
        }

        [Fact]
        public void AnFr3CommandsArmTargets_AreTheDatasetBuildersClamp()
        {
            // The FR3 Duo's action follows the same rule (takes.py, _fr3_targets): a refused command as this plant would have sent
            // it, each target within its joint's soft limits and a hand's fraction within 0..1. test_takes.py pins the same values.
            var backend = new CapturingBackend();
            var plant = new SerialChainPlant(SerialChainProfileJson.Load(Profile("fr3-duo.json")), backend); // no collision model: the clamp alone
            var targets = new[] { new JointTarget(1, 5.0f, 1f), new JointTarget(8, 1.7f, 1f), new JointTarget(4, 0.5f, 1f), new JointTarget(2, 0.25f, 1f) };

            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointTargets(targets, captureTicks: 10, nowTicks: 0));

            var sent = backend.Last.ToDictionary(t => t.MotorId, t => t.Value);
            Assert.Equal((float)2.9007400166666666, sent[1]);     // joint 1's soft upper limit
            Assert.Equal(1f, sent[8]);                             // the left hand, at most 1
            Assert.Equal((float)-0.11693708333333333, sent[4]);   // joint 4's soft upper limit: the FR3's elbow is always bent
            Assert.Equal(0.25f, sent[2]);                          // inside its limits: as sent
        }

        private static string JetRoverProfile() => Profile("jetrover.json");

        private static string Profile(string name)
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                string candidate = Path.Combine(dir.FullName, "core", "RobotProfiles", name);
                if (File.Exists(candidate)) return candidate;
            }

            throw new InvalidOperationException($"no core/RobotProfiles/{name} above {AppContext.BaseDirectory}");
        }

        private sealed class CapturingBackend : IArmBackend
        {
            public JointSample[] Last = Array.Empty<JointSample>();

            public bool Enabled => true;

            public Action<long, double[]>? TruthSink { get; set; }

            public string Summary => "capturing";

            public void Send(ReadOnlySpan<JointSample> targets, long hostTicks) => Last = targets.ToArray();

            public int ReadState(long hostTicks, Span<JointSample> output, out long sensedTicks)
            {
                sensedTicks = hostTicks;
                return 0;
            }

            public void SetEnabled(bool enabled, long hostTicks)
            {
            }

            public void Poll(long hostTicks)
            {
            }
        }
    }
}
