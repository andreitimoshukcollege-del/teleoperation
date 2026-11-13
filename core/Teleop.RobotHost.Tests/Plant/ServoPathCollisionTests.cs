using System;
using System.Linq;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Plant
{
    /// <summary>
    /// The robot's collision backstop checks the path the servos take, not only where they end (the 2026-11 bug hunt, C-05:
    /// two clear poses in one command, and the servos' equal-speed path drove the distal link into the body 38% of the way).
    /// </summary>
    public sealed class ServoPathCollisionTests
    {
        private static readonly GenericArmPlantConfig Config = GenericArmPlantConfig.Default;
        private static readonly RobotArmProfile JetRover = Config.Profile;
        private static readonly ArmCollisionModel Model = JetRover.Collision!;

        private static float Toward(float delta, float tau) => delta >= 0f ? Math.Min(delta, tau) : Math.Max(delta, -tau);

        /// <summary>The servos' path, sampled finely here and independently of the plant: does it hit anywhere?</summary>
        private static bool PathHits(ArmPose a, ArmPose b)
        {
            float[] d = { b.BaseYaw - a.BaseYaw, b.Proximal - a.Proximal, b.Distal - a.Distal, b.Wrist - a.Wrist, b.Roll - a.Roll };
            float longest = d.Max(Math.Abs);
            int steps = Math.Max(2, (int)MathF.Ceiling(longest / (0.5f * MathF.PI / 180f)));
            for (int s = 1; s < steps; s++)
            {
                float tau = longest * s / steps;
                var p = new ArmPose(a.BaseYaw + Toward(d[0], tau), a.Proximal + Toward(d[1], tau), a.Distal + Toward(d[2], tau),
                    a.Wrist + Toward(d[3], tau), a.Roll + Toward(d[4], tau));
                if (!ArmCollision.IsValid(JetRover, Model, p)) return true;
            }

            return false;
        }

        private static ArmPose RandomValidPose(Random rng)
        {
            while (true)
            {
                float U(AngleRange r) => r.Min + (float)rng.NextDouble() * (r.Max - r.Min);
                var p = new ArmPose(U(Model.BaseYaw), U(Model.Proximal), U(Model.Distal), U(Model.Wrist), U(Model.Roll));
                if (ArmCollision.IsValid(JetRover, Model, p)) return p;
            }
        }

        private static byte Motor(JointRole role) => JetRover.Joints[JetRover.TryGetJointIndex(role)].MotorId;

        private static JointTarget[] Targets(ArmPose p) => new[]
        {
            new JointTarget(Motor(JointRole.BaseYaw), p.BaseYaw, 0f), new JointTarget(Motor(JointRole.Proximal), p.Proximal, 0f),
            new JointTarget(Motor(JointRole.Distal), p.Distal, 0f), new JointTarget(Motor(JointRole.Wrist), p.Wrist, 0f),
            new JointTarget(Motor(JointRole.GripperRotate), p.Roll, 0f),
        };

        /// <summary>A plant whose arm is measured at <paramref name="at"/> (the relay's feedback, as on the robot).</summary>
        private static GenericArmPlant PlantAt(ArmPose at, FakeRelayClient relay)
        {
            var plant = new GenericArmPlant(Config, relay);
            JointFeedbackEntry Entry(JointRole role, float angle)
            {
                JointHardwareSpec j = JetRover.Joints[JetRover.TryGetJointIndex(role)];
                return new JointFeedbackEntry(j.MotorId, true, Config.ZeroPulse + (angle + j.ZeroOffsetRadians) * Config.PulsePerRadian);
            }

            relay.EnqueueFeedback(Entry(JointRole.BaseYaw, at.BaseYaw), Entry(JointRole.Proximal, at.Proximal), Entry(JointRole.Distal, at.Distal),
                Entry(JointRole.Wrist, at.Wrist), Entry(JointRole.GripperRotate, at.Roll));
            plant.Step(1);
            return plant;
        }

        [Fact]
        public void AMoveBetweenTwoClearPoses_WhosePathHitsTheRobot_IsRefused_AndTheArmHolds()
        {
            var rng = new Random(20261113);
            for (int attempt = 0; attempt < 20000; attempt++)
            {
                ArmPose a = RandomValidPose(rng), b = RandomValidPose(rng);
                if (a.MaxJointDelta(b) < 20f * MathF.PI / 180f || !PathHits(a, b)) continue;

                var relay = new FakeRelayClient();
                GenericArmPlant plant = PlantAt(a, relay);
                Assert.Equal(JointCommandOutcome.Refused, plant.CommandJointAngles(Targets(b), captureTicks: 10));
                Assert.Empty(relay.SentCommands);
                Assert.Equal(1, plant.RefusedJointCommands);
                return;
            }

            Assert.Fail("no pair of clear poses with a colliding servo path was found to test with");
        }

        [Fact]
        public void AMoveWhosePathIsClear_IsAccepted()
        {
            var rng = new Random(7);
            for (int attempt = 0; attempt < 20000; attempt++)
            {
                ArmPose a = RandomValidPose(rng), b = RandomValidPose(rng);
                if (a.MaxJointDelta(b) < 20f * MathF.PI / 180f || PathHits(a, b)) continue;

                var relay = new FakeRelayClient();
                GenericArmPlant plant = PlantAt(a, relay);
                Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(Targets(b), captureTicks: 10));
                Assert.Single(relay.SentCommands);
                return;
            }

            Assert.Fail("no pair of clear poses with a clear servo path was found to test with");
        }

        [Fact]
        public void FromTheMeasuredRestPose_PastTheWristStop_ALongMoveIsStillPathChecked()
        {
            // The rest pose reads the wrist past its stop; the plant clamps the start into the ranges, so even the first
            // move out of rest is held to a clear path instead of skipping the check.
            const float deg = MathF.PI / 180f;
            var rest = new ArmPose(-1.20f * deg, 91.16f * deg, -78.48f * deg, -114.0f * deg, 5.76f * deg);
            var clampedRest = new ArmPose(rest.BaseYaw, rest.Proximal, rest.Distal, Model.Wrist.Clamp(rest.Wrist), rest.Roll);
            Assert.True(ArmCollision.IsValid(JetRover, Model, clampedRest));

            var rng = new Random(3);
            for (int attempt = 0; attempt < 20000; attempt++)
            {
                ArmPose b = RandomValidPose(rng);
                if (!PathHits(clampedRest, b)) continue;
                var relay = new FakeRelayClient();
                Assert.Equal(JointCommandOutcome.Refused, PlantAt(rest, relay).CommandJointAngles(Targets(b), captureTicks: 10));
                return;
            }

            Assert.Fail("no colliding move out of rest was found to test with");
        }
    }
}
