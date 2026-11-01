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
        public void NoJoint_MovesFasterThanItsVelocityLimit_EvenAcrossAHitch()
        {
            SerialChainTeleop teleop = Teleop(out SerialChainProfile profile, out _);
            RigidTransform hand = RigidTransform.Identity;
            teleop.Engage(0, hand);
            RigidTransform turn = RigidTransform.Rotation(0, 0, 1, 1.2);
            teleop.Follow(0, new RigidTransform(0.2, 0.1, -0.15, turn.Qw, turn.Qx, turn.Qy, turn.Qz)); // far, and turned
            ChainJoint[] joints = profile.Arms[0].Joints;
            foreach (double seconds in new[] { 1.0 / 90.0, 0.5 })
            {
                for (int k = 0; k < 40; k++)
                {
                    double[] before = teleop.Commanded(0).ToArray();
                    teleop.Step(seconds);
                    double[] after = teleop.Commanded(0).ToArray();
                    double allowed = Math.Min(seconds, SerialChainTeleop.LongestStep);
                    for (int j = 0; j < 7; j++)
                    {
                        double move = Math.Abs(after[j] - before[j]);
                        Assert.True(move <= joints[j].Limits.Velocity * allowed + 1e-12, $"{seconds} s step {k} joint {j}: {move} rad");
                    }
                }
            }
        }

        [Fact]
        public void TheGripper_TravelsTheStraightLine_ToAFarTarget_AtMostItsSpeed()
        {
            var options = new SerialChainTeleopOptions();
            SerialChainTeleop teleop = Teleop(out _, out _, options);
            RigidTransform start = teleop.Tool(0);
            teleop.Engage(0, RigidTransform.Identity);
            teleop.Follow(0, Moved(RigidTransform.Identity, 0.12, -0.08, -0.10)); // 18 cm away, the same orientation
            RigidTransform target = teleop.Target(0);
            double dx = target.X - start.X, dy = target.Y - start.Y, dz = target.Z - start.Z;
            double length = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            double travelled = 0, worst = 0;
            RigidTransform last = start;
            for (int k = 0; k < 120; k++)
            {
                teleop.Step(1.0 / 90.0);
                RigidTransform tool = teleop.Tool(0);
                double step = Math.Sqrt(Sq(tool.X - last.X) + Sq(tool.Y - last.Y) + Sq(tool.Z - last.Z));
                Assert.True(step <= options.MaxLinearSpeed / 90.0 + 1e-4, $"step {k}: {step} m in a frame");
                travelled += step;
                // Off the line from start to target: |(p - start) x direction|.
                double px = tool.X - start.X, py = tool.Y - start.Y, pz = tool.Z - start.Z;
                double cx = py * dz - pz * dy, cy = pz * dx - px * dz, cz = px * dy - py * dx;
                worst = Math.Max(worst, Math.Sqrt(cx * cx + cy * cy + cz * cz) / length);
                last = tool;
            }

            Assert.Equal(ChainArmStatus.Following, teleop.Status(0));
            Assert.True(Math.Sqrt(Sq(last.X - target.X) + Sq(last.Y - target.Y) + Sq(last.Z - target.Z)) < 1e-3, "it did not arrive");
            Assert.True(worst < 1e-3, $"the gripper left the straight line by {1000 * worst:0.00} mm");
            Assert.True(travelled < 1.01 * length, $"it travelled {travelled:0.000} m for {length:0.000} m");
        }

        [Fact]
        public void AJointNearItsLimit_IsSteeredOff_WhileTheGripperHoldsStill()
        {
            double[] left = (double[])LeftReady.Clone();
            left[4] = 2.7; // the forearm, inside the zone before its 2.876 rad limit
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var teleop = new SerialChainTeleop(profile, SerialChainSolverTests.Collision());
            Assert.True(teleop.TryTakePose(Measured(profile, left, RightReady)));
            RigidTransform held = teleop.Tool(0);
            teleop.EngageAt(0, held);
            for (int k = 0; k < 90; k++) teleop.Step(1.0 / 90.0);

            double q5 = teleop.Commanded(0)[4];
            Assert.True(q5 < 2.7 - 0.1, $"the forearm stayed at {q5:0.000} rad");
            RigidTransform now = teleop.Tool(0);
            Assert.True(Math.Sqrt(Sq(now.X - held.X) + Sq(now.Y - held.Y) + Sq(now.Z - held.Z)) < 1e-3, "the gripper moved");
            Assert.True(SerialChainTeleop.AngleBetween(now, held) < 0.01, "the gripper turned");
            Assert.Equal(ChainArmStatus.Following, teleop.Status(0));

            // The spare motion, the clearance and the speed caps allocate nothing.
            AllocationAssert.Zero(() => teleop.Step(1.0 / 90.0), iterations: 200);
        }

        [Fact]
        public void AnUnreachableTarget_IsOutOfReach_OnlyBeyondTheReachTolerance()
        {
            foreach (double tolerance in new[] { 0.01, 1.0 })
            {
                SerialChainTeleop teleop = Teleop(out _, out _, new SerialChainTeleopOptions { ReachTolerance = tolerance });
                teleop.Engage(0, RigidTransform.Identity);
                teleop.Follow(0, Moved(RigidTransform.Identity, 0.3, 0.9, 0.3)); // a metre out, beyond the arm's reach
                for (int k = 0; k < 400; k++) teleop.Step(1.0 / 90.0);
                RigidTransform tool = teleop.Tool(0), target = teleop.Target(0);
                double off = Math.Sqrt(Sq(tool.X - target.X) + Sq(tool.Y - target.Y) + Sq(tool.Z - target.Z));
                Assert.InRange(off, 0.05, 0.9);
                Assert.Equal(tolerance < off ? ChainArmStatus.StoppedShort : ChainArmStatus.Following, teleop.Status(0));
            }
        }

        [Fact]
        public void AnArmAtTheCollisionMargin_SlidesAlongTheObstacle_InsteadOfSticking()
        {
            // The right arm as the headset commanded it at 59.8 s of the 2026-11-01 Isaac session: its upper arm at the
            // margin of the head, where moving the hand away (-x) went 3 mm in 2 s before the IK could slide.
            double[] right = { -1.775942, -1.574056, -0.137502, -2.243987, -2.128081, 4.347246, 0.154379 };
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            SerialChainCollisionModel collision = SerialChainSolverTests.Collision();
            var teleop = new SerialChainTeleop(profile, collision);
            Assert.True(teleop.TryTakePose(Measured(profile, LeftReady, right)));
            Assert.InRange(SerialChainCollision.Clearance(profile, collision, Pose(teleop), 1), collision.Margin, collision.Margin + 0.002);

            RigidTransform start = teleop.Tool(1);
            teleop.EngageAt(1, Moved(start, -0.06, 0, 0));
            for (int k = 0; k < 180; k++)
            {
                teleop.Step(1.0 / 90.0);
                Assert.False(SerialChainCollision.Check(profile, collision, Pose(teleop)).IsHit, $"step {k}: the committed pose collides");
            }

            RigidTransform end = teleop.Tool(1);
            Assert.True(start.X - end.X > 0.05, $"the gripper moved {1000 * (start.X - end.X):0} mm of the 60 asked");
            Assert.True(SerialChainTeleop.AngleBetween(start, end) < 0.02, "the slide turned the gripper");
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
