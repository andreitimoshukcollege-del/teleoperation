using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Tests.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>The FR3 Duo's two-hand operator (docs/adr/0029): clutch, solve, check, command.</summary>
    public class SerialChainTeleopTests
    {
        private static readonly double[] LeftReady = { 0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4 };
        private static readonly double[] RightReady = LeftReady.Select((q, j) => j % 2 == 0 ? -q : q).ToArray();

        private static SerialChainTeleop Teleop(out SerialChainProfile profile, out SerialChainCollisionModel collision, SerialChainTeleopOptions? options = null)
        {
            profile = SerialChainKinematicsTests.LoadFr3Duo();
            collision = SerialChainSolverTests.Collision();
            var teleop = new SerialChainTeleop(profile, collision, options);
            Assert.True(teleop.TryTakePose(Measured(profile, LeftReady, RightReady)));
            return teleop;
        }

        private static JointSample[] Measured(SerialChainProfile profile, double[] left, double[] right, float opening = 1f)
        {
            var samples = new List<JointSample>();
            double[][] arms = { left, right };
            for (int a = 0; a < profile.Arms.Length; a++)
            {
                ChainArm arm = profile.Arms[a];
                for (int j = 0; j < arm.Joints.Length; j++) samples.Add(new JointSample(arm.Joints[j].MotorId, (float)arms[a][j], true));
                if (arm.HasGripper) samples.Add(new JointSample(arm.Gripper.MotorId, opening, true));
            }

            return samples.ToArray();
        }

        private static RigidTransform Moved(in RigidTransform t, double dx, double dy, double dz) =>
            new RigidTransform(t.X + dx, t.Y + dy, t.Z + dz, t.Qw, t.Qx, t.Qy, t.Qz);

        private static double[] Pose(SerialChainTeleop teleop) => teleop.Commanded(0).ToArray().Concat(teleop.Commanded(1).ToArray()).ToArray();

        [Fact]
        public void BeforeTheRobotsPose_NothingIsCommanded_AndNoClutchEngages()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var teleop = new SerialChainTeleop(profile, SerialChainSolverTests.Collision());
            Assert.False(teleop.HasPose);
            Assert.False(teleop.Engage(0, RigidTransform.Identity));
            Assert.Equal(0, teleop.WriteTargets(new JointTarget[teleop.MotorCount]));

            // A pose missing a joint is not taken.
            JointSample[] partial = Measured(profile, LeftReady, RightReady).Where(s => s.MotorId != 13).ToArray();
            Assert.False(teleop.TryTakePose(partial));
            Assert.False(teleop.HasPose);
        }

        [Fact]
        public void TheFirstCommand_IsWhereTheRobotIs_InProfileOrder()
        {
            SerialChainTeleop teleop = Teleop(out SerialChainProfile profile, out _);
            var targets = new JointTarget[teleop.MotorCount];
            Assert.Equal(16, teleop.WriteTargets(targets));
            Assert.Equal(new byte[] { 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18 }, targets.Select(t => t.MotorId).ToArray());
            for (int j = 0; j < 7; j++)
            {
                Assert.Equal((float)LeftReady[j], targets[j].Angle);
                Assert.Equal((float)RightReady[j], targets[8 + j].Angle);
            }

            Assert.Equal(1f, targets[7].Angle);
            Assert.Equal(1f, targets[15].Angle);
            Assert.Equal(0, teleop.ArmIndex("left"));
            Assert.Equal(1, teleop.ArmIndex("right"));
            Assert.Equal(-1, teleop.ArmIndex("middle"));
        }

        [Fact]
        public void AClutchedArm_FollowsTheHand_ToAMillimetre_AndTheOtherArmDoesNotMove()
        {
            SerialChainTeleop teleop = Teleop(out SerialChainProfile profile, out _);
            double[] rightBefore = teleop.Commanded(1).ToArray();
            RigidTransform hand = RigidTransform.FromXyzRpy(0.2, 0.3, 1.0, 0.1, 0.2, 0.3);
            Assert.True(teleop.Engage(0, hand));

            // The hand moves 8 cm forward and 5 cm down over a second at 90 Hz, then stays.
            for (int k = 1; k <= 120; k++)
            {
                double s = Math.Min(1.0, k / 90.0);
                teleop.Follow(0, Moved(hand, 0.08 * s, 0, -0.05 * s));
                teleop.Step();
                Assert.NotEqual(ChainArmStatus.Blocked, teleop.Status(0));
            }

            RigidTransform target = teleop.Target(0);
            RigidTransform reached = teleop.Tool(0);
            double d = Math.Sqrt(Sq(reached.X - target.X) + Sq(reached.Y - target.Y) + Sq(reached.Z - target.Z));
            Assert.True(d < 1e-3, $"{d} m off");
            Assert.Equal(ChainArmStatus.Following, teleop.Status(0));
            Assert.Equal(rightBefore, teleop.Commanded(1).ToArray());
            Assert.Equal(ChainArmStatus.Idle, teleop.Status(1));
        }

        [Fact]
        public void TurningTheHand_TurnsTheGripper()
        {
            SerialChainTeleop teleop = Teleop(out _, out _);
            RigidTransform hand = RigidTransform.FromXyzRpy(0.2, -0.3, 1.0, 0, 0, 0);
            RigidTransform toolBefore = teleop.Tool(1);
            teleop.Engage(1, hand);
            RigidTransform turn = RigidTransform.Rotation(0, 0, 1, 0.35); // 20 degrees about the vertical
            for (int k = 1; k <= 120; k++)
            {
                double s = Math.Min(1.0, k / 60.0);
                RigidTransform r = RigidTransform.Rotation(0, 0, 1, 0.35 * s);
                teleop.Follow(1, new RigidTransform(hand.X, hand.Y, hand.Z, r.Qw, r.Qx, r.Qy, r.Qz));
                teleop.Step();
            }

            RigidTransform expected = turn.Compose(new RigidTransform(0, 0, 0, toolBefore.Qw, toolBefore.Qx, toolBefore.Qy, toolBefore.Qz));
            PoseClutchTests.AssertSameRotation(expected, teleop.Tool(1), toleranceRad: 2e-3);
        }

        [Fact]
        public void NoJoint_MovesMoreThanTheStepLimit_InOneStep()
        {
            SerialChainTeleop teleop = Teleop(out _, out _, new SerialChainTeleopOptions { MaxStepRadians = 0.1 });
            RigidTransform hand = RigidTransform.Identity;
            teleop.Engage(0, hand);
            teleop.Follow(0, Moved(hand, 0.25, 0.1, 0.2)); // the hand reappears 34 cm away
            for (int k = 0; k < 30; k++)
            {
                double[] before = teleop.Commanded(0).ToArray();
                teleop.Step();
                double[] after = teleop.Commanded(0).ToArray();
                for (int j = 0; j < 7; j++) Assert.True(Math.Abs(after[j] - before[j]) <= 0.1 + 1e-12, $"step {k} joint {j}: {after[j] - before[j]}");
            }
        }

        [Fact]
        public void AnArmDrivenIntoTheOther_IsBlocked_AndEveryCommittedPoseIsClear()
        {
            SerialChainTeleop teleop = Teleop(out SerialChainProfile profile, out SerialChainCollisionModel collision);
            RigidTransform hand = RigidTransform.Identity;
            RigidTransform start = teleop.Tool(0);
            RigidTransform other = teleop.Tool(1);
            teleop.Engage(0, hand);
            bool blocked = false;
            for (int k = 1; k <= 300; k++)
            {
                // Toward, and past, the right arm's tool.
                double s = Math.Min(1.5, k / 100.0);
                teleop.Follow(0, Moved(hand, s * (other.X - start.X), s * (other.Y - start.Y), s * (other.Z - start.Z)));
                teleop.Step();
                blocked |= teleop.Status(0) == ChainArmStatus.Blocked;
                ChainCollisionHit hit = SerialChainCollision.Check(profile, collision, Pose(teleop));
                Assert.False(hit.IsHit, $"step {k}: the committed pose collides");
            }

            Assert.True(blocked, "the arm was never blocked on its way into the other");
        }

        [Fact]
        public void Releasing_HoldsTheCommand_AndPutsTheTargetOnTheArm()
        {
            SerialChainTeleop teleop = Teleop(out _, out _);
            RigidTransform hand = RigidTransform.Identity;
            teleop.Engage(0, hand);
            teleop.Follow(0, Moved(hand, 0.5, 0, 0)); // far: the arm is still on its way when released
            teleop.Step();
            teleop.Release(0);
            double[] held = teleop.Commanded(0).ToArray();
            for (int k = 0; k < 10; k++)
            {
                teleop.Follow(0, Moved(hand, -0.3, 0, 0));
                teleop.Step();
            }

            Assert.Equal(held, teleop.Commanded(0).ToArray());
            Assert.Equal(ChainArmStatus.Idle, teleop.Status(0));
            PoseClutchTests.AssertSame(teleop.Tool(0), teleop.Target(0));
        }

        [Fact]
        public void TheGrippers_AreCommandedAsOpeningFractions_Clamped()
        {
            SerialChainTeleop teleop = Teleop(out _, out _);
            teleop.SetOpening(0, 0.25);
            teleop.SetOpening(1, 1.7);
            teleop.SetOpening(1, double.NaN); // leaves it
            var targets = new JointTarget[teleop.MotorCount];
            teleop.WriteTargets(targets);
            Assert.Equal(0.25f, targets[7].Angle);
            Assert.Equal(1f, targets[15].Angle);
            teleop.SetOpening(1, -3);
            teleop.WriteTargets(targets);
            Assert.Equal(0f, targets[15].Angle);
        }

        [Fact]
        public void TakingThePoseAgain_ReleasesTheClutches()
        {
            SerialChainTeleop teleop = Teleop(out SerialChainProfile profile, out _);
            teleop.Engage(0, RigidTransform.Identity);
            teleop.Engage(1, RigidTransform.Identity);
            Assert.True(teleop.TryTakePose(Measured(profile, LeftReady, RightReady, 0.5f)));
            Assert.False(teleop.IsEngaged(0));
            Assert.False(teleop.IsEngaged(1));
            Assert.Equal(0.5, teleop.Opening(0), 6);
        }

        [Fact]
        public void SteppingAndWriting_AreDeterministic_AndAllocateNothing()
        {
            double[] Run()
            {
                SerialChainTeleop t = Teleop(out _, out _);
                t.Engage(0, RigidTransform.Identity);
                t.Engage(1, RigidTransform.Identity);
                for (int k = 1; k <= 50; k++)
                {
                    t.Follow(0, Moved(RigidTransform.Identity, 0.001 * k, 0, 0));
                    t.Follow(1, Moved(RigidTransform.Identity, 0, -0.001 * k, 0.0005 * k));
                    t.Step();
                }

                return Pose(t);
            }

            Assert.Equal(Run().Select(BitConverter.DoubleToInt64Bits), Run().Select(BitConverter.DoubleToInt64Bits));

            SerialChainTeleop teleop = Teleop(out _, out _);
            var targets = new JointTarget[teleop.MotorCount];
            teleop.Engage(0, RigidTransform.Identity);
            RigidTransform moved = Moved(RigidTransform.Identity, 0.03, 0, 0);
            AllocationAssert.Zero(() =>
            {
                teleop.Follow(0, moved);
                teleop.Step();
                teleop.WriteTargets(targets);
            }, iterations: 200);
        }

        [Fact]
        public void ACollisionModelForAnotherRobot_IsRefused()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var other = new SerialChainCollisionModel("not-the-fr3", Array.Empty<ChainCapsule>(), Array.Empty<(int, int)>(), 0.01);
            Assert.Throws<ArgumentException>(() => new SerialChainTeleop(profile, other));
            Assert.Throws<ArgumentException>(() => new SerialChainTeleop(profile, null, new SerialChainTeleopOptions { MaxStepRadians = 0 }));
        }

        private static double Sq(double x) => x * x;
    }
}
