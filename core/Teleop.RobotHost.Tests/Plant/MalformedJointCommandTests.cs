using System.Linq;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;

namespace Teleop.RobotHost.Tests.Plant
{
    /// <summary>
    /// Joint commands a sender can put on the wire that the plant must not pass on (the 2026-11 bug hunt): a motor named
    /// twice crashed RobotHost (C-01), a NaN gripper value became a NaN pulse that killed the servo driver (C-02), and a
    /// very slow speed wrapped the servo board's 16-bit move duration (C-03).
    /// </summary>
    public sealed class MalformedJointCommandTests
    {
        private static readonly GenericArmPlantConfig Config = GenericArmPlantConfig.Default;
        private static readonly RobotArmProfile JetRover = Config.Profile;
        private const byte GripperMotorId = 10;

        private static byte Motor(JointRole role) => JetRover.Joints[JetRover.TryGetJointIndex(role)].MotorId;

        private static JointTarget[] Home(float speed = 0f, float gripper = 0f)
        {
            ArmPose pose = JetRover.Collision!.Home;
            return new[]
            {
                new JointTarget(Motor(JointRole.BaseYaw), pose.BaseYaw, speed),
                new JointTarget(Motor(JointRole.Proximal), pose.Proximal, speed),
                new JointTarget(Motor(JointRole.Distal), pose.Distal, speed),
                new JointTarget(Motor(JointRole.Wrist), pose.Wrist, speed),
                new JointTarget(Motor(JointRole.GripperRotate), pose.Roll, speed),
                new JointTarget(GripperMotorId, gripper, speed),
            };
        }

        [Fact]
        public void AMotorNamedTwice_IsRefusedWhole_NotACrash()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            JointTarget[] twice = Home().Append(new JointTarget(Motor(JointRole.BaseYaw), 0.2f, 0f)).ToArray();

            Assert.Equal(JointCommandOutcome.Refused, plant.CommandJointAngles(twice, captureTicks: 10));

            Assert.Empty(relay.SentCommands);
            Assert.Equal(1, plant.InvalidJointCommands);
            Assert.Equal(0, plant.RefusedJointCommands); // not counted as a collision
        }

        [Theory]
        [InlineData(float.NaN)]
        [InlineData(float.PositiveInfinity)]
        [InlineData(float.NegativeInfinity)]
        public void ANonFiniteGripperValue_NeverReachesTheRelay(float gripper)
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            Assert.Equal(JointCommandOutcome.Refused, plant.CommandJointAngles(Home(gripper: gripper), captureTicks: 10));

            Assert.Empty(relay.SentCommands);
            Assert.Equal(1, plant.InvalidJointCommands);
        }

        [Theory]
        [InlineData(float.NaN, 0f)]
        [InlineData(0f, float.NaN)]
        [InlineData(0f, float.PositiveInfinity)]
        public void ANonFiniteArmAngleOrSpeed_IsRefusedWhole(float angle, float speed)
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            JointTarget[] targets = Home();
            targets[1] = new JointTarget(targets[1].MotorId, float.IsNaN(angle) ? angle : targets[1].Angle, speed);

            Assert.Equal(JointCommandOutcome.Refused, plant.CommandJointAngles(targets, captureTicks: 10));
            Assert.Empty(relay.SentCommands);
        }

        [Fact]
        public void ARefusedMalformedCommand_DoesNotUseUpItsCaptureStamp()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            plant.CommandJointAngles(Home(gripper: float.NaN), captureTicks: 10);

            // The same stamp with good values is the sender's corrected command, not a replay.
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(Home(), captureTicks: 10));
            Assert.Single(relay.SentCommands);
        }

        [Fact]
        public void AVerySlowSpeed_IsFlooredSoNoMoveOutlastsTheBoardsDurationField()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            // 0.01 rad/s is about 2.4 pulses/s: a 157-pulse step would plan 65.8 s, which the board's 16-bit millisecond
            // field wraps to 228 ms.
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(Home(speed: 0.01f), captureTicks: 10));

            JointTarget[] sent = relay.SentCommands.Single();
            Assert.All(sent, t => Assert.Equal(GenericArmPlant.MinPulsesPerSecond, t.Speed));
            // The board's whole range at the floor stays inside 65.535 s.
            Assert.True(1000f / GenericArmPlant.MinPulsesPerSecond < 65.535f);
        }

        [Fact]
        public void AnOrdinarySpeed_IsUnchanged_AndTheDeployedCapStillHolds()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            plant.CommandJointAngles(Home(speed: 1.5f), captureTicks: 10);
            plant.CommandJointAngles(Home(speed: 1000f), captureTicks: 11);

            Assert.Equal(System.Math.Min(1.5f * Config.PulsePerRadian, Config.PulsesPerSecond), relay.SentCommands[0][0].Speed, 3);
            Assert.Equal(Config.PulsesPerSecond, relay.SentCommands[1][0].Speed);
        }

        [Fact]
        public void AnUnknownMotorId_IsStillIgnored_NotRefused()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            JointTarget[] withForeign = Home().Append(new JointTarget(99, 0.5f, 0f)).ToArray();

            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(withForeign, captureTicks: 10));
            Assert.DoesNotContain(relay.SentCommands.Single(), t => t.MotorId == 99);
        }
    }
}
