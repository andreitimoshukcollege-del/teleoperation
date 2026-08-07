using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Plant
{
    public class JetRoverPlantTests
    {
        private static CommandFrame Frame(long captureTicks, float positionX) =>
            new CommandFrame(
                sequence: 1,
                ackSequence: 0,
                captureTicks: captureTicks,
                pose: new Pose(new Vector3(positionX, 0f, 0f), Quaternion.Identity),
                linearVelocity: Vector3.Zero,
                angularVelocity: Vector3.Zero,
                gripper: 0f);

        [Fact]
        public void Command_SendsScaledDirectionToRelay()
        {
            var relay = new FakeRelayClient();
            var config = new JetRoverPlantConfig(positionXToDirectionScale: 2f, maxDirectionMagnitude: 100f);
            var plant = new JetRoverPlant(config, relay);

            plant.Command(Frame(captureTicks: 10, positionX: 3f));

            Assert.Single(relay.SentCommands);
            Assert.Equal(6f, relay.SentCommands[0].BaseDirection);
        }

        [Fact]
        public void Command_ClampsDirectionToConfiguredMaximum()
        {
            var relay = new FakeRelayClient();
            var config = new JetRoverPlantConfig(positionXToDirectionScale: 10f, maxDirectionMagnitude: 5f);
            var plant = new JetRoverPlant(config, relay);

            plant.Command(Frame(captureTicks: 10, positionX: 100f));
            plant.Command(Frame(captureTicks: 20, positionX: -100f));

            Assert.Equal(5f, relay.SentCommands[0].BaseDirection);
            Assert.Equal(-5f, relay.SentCommands[1].BaseDirection);
        }

        [Fact]
        public void Command_RejectsStaleOrDuplicateFramesWhole()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(JetRoverPlantConfig.Default, relay);

            plant.Command(Frame(captureTicks: 100, positionX: 1f));
            plant.Command(Frame(captureTicks: 100, positionX: 2f)); // duplicate stamp -- rejected whole
            plant.Command(Frame(captureTicks: 50, positionX: 3f));  // stale -- rejected whole

            Assert.Single(relay.SentCommands);
            Assert.Equal(1f, relay.SentCommands[0].BaseDirection);
        }

        [Fact]
        public void Step_UpdatesStateFromValidFeedback()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(JetRoverPlantConfig.Default, relay);
            relay.EnqueueFeedback(new LocalFeedback(baseDegreesValid: true, baseDegrees: 42));

            plant.Step(nowTicks: 1000);

            Assert.True(plant.IsBaseDegreesSensed);
            Assert.Equal(42f, plant.State.Value.Position.X);
            Assert.Equal(1000, plant.State.CaptureTicks);
        }

        [Fact]
        public void Step_IgnoresInvalidFeedback_StateStaysUnsensed()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(JetRoverPlantConfig.Default, relay);
            relay.EnqueueFeedback(new LocalFeedback(baseDegreesValid: false, baseDegrees: 999));

            plant.Step(nowTicks: 1000);

            Assert.False(plant.IsBaseDegreesSensed);
            Assert.True(float.IsNaN(plant.State.Value.Position.X));
        }

        [Fact]
        public void Step_AtOrBeforeCurrentStateTime_IsNoOp()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(JetRoverPlantConfig.Default, relay);
            relay.EnqueueFeedback(new LocalFeedback(baseDegreesValid: true, baseDegrees: 7));
            plant.Step(nowTicks: 1000);

            relay.EnqueueFeedback(new LocalFeedback(baseDegreesValid: true, baseDegrees: 999));
            plant.Step(nowTicks: 1000); // not after the current state time -- must not consume feedback

            Assert.Equal(1000, plant.State.CaptureTicks);
            Assert.Equal(7f, plant.State.Value.Position.X);
        }

        [Fact]
        public void Reset_ClearsBookkeeping_AcceptsNextFrameAtAnyCaptureTicks()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(JetRoverPlantConfig.Default, relay);
            plant.Command(Frame(captureTicks: 1000, positionX: 1f));
            relay.EnqueueFeedback(new LocalFeedback(baseDegreesValid: true, baseDegrees: 7));
            plant.Step(nowTicks: 1000);

            plant.Reset();

            Assert.False(plant.IsBaseDegreesSensed);
            Assert.Equal(0, plant.State.CaptureTicks);

            // A reused instance (sweeps reuse instances across trials) must accept the next
            // trial's first command whatever its stamp, even one lower than before Reset.
            // positionX kept below Default's MaxDirectionMagnitude (5) so this assertion tests
            // Reset's bookkeeping clear, not the separately-tested clamping behavior.
            plant.Command(Frame(captureTicks: 5, positionX: 3f));
            Assert.Equal(2, relay.SentCommands.Count);
            Assert.Equal(3f, relay.SentCommands[1].BaseDirection);
        }

        [Fact]
        public void Reset_DoesNotSendAnyRelayCommand()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(JetRoverPlantConfig.Default, relay);
            plant.Command(Frame(captureTicks: 1000, positionX: 1f));

            plant.Reset();

            Assert.Single(relay.SentCommands); // only the Command() call above -- Reset sent nothing
        }
    }
}
