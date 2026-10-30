using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Tests.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>The robot anchored to the operator's body (docs/adr/0029 amendment).</summary>
    public class BodyAnchoredMappingTests
    {
        private static BodyAnchoredMapping Mapping(out SerialChainProfile profile, BodyMappingOptions? options = null)
        {
            profile = SerialChainKinematicsTests.LoadFr3Duo();
            var m = new BodyAnchoredMapping(profile, BodyAnchoredMapping.NeutralTools(profile), options);
            m.Recentre(RigidTransform.Identity);
            return m;
        }

        /// <summary>A hand in the head's frame: forward, left, up (m), palm flat and pointing forward, then turned by <paramref name="turn"/>.</summary>
        private static RigidTransform Hand(double forward, double left, double up, RigidTransform? turn = null)
        {
            RigidTransform r = turn ?? RigidTransform.Identity;
            return new RigidTransform(forward, left, up, r.Qw, r.Qx, r.Qy, r.Qz);
        }

        private static void Axis(in RigidTransform t, double x, double y, double z, out double ax, out double ay, out double az) => t.Rotate(x, y, z, out ax, out ay, out az);

        [Fact]
        public void TheNeutralHand_PutsEachArmAtItsNeutralTool_WithTheGripperPointingForward()
        {
            BodyAnchoredMapping m = Mapping(out SerialChainProfile profile);
            RigidTransform[] neutral = BodyAnchoredMapping.NeutralTools(profile);
            int left = Array.FindIndex(profile.Arms, a => a.Name == "left"), right = Array.FindIndex(profile.Arms, a => a.Name == "right");
            RigidTransform l = m.Map(left, Hand(0.30, 0.20, -0.40));
            RigidTransform r = m.Map(right, Hand(0.30, -0.20, -0.40));
            Assert.Equal(neutral[left].X, l.X, 9);
            Assert.Equal(neutral[left].Y, l.Y, 9);
            Assert.Equal(neutral[left].Z, l.Z, 9);
            Assert.Equal(neutral[right].Y, r.Y, 9);

            // Palm flat, pointing forward: the approach (tool z) is forward and the fingers close sideways (tool y).
            Axis(l, 0, 0, 1, out double zx, out double zy, out double zz);
            Assert.Equal((1.0, 0.0, 0.0), (Math.Round(zx, 9), Math.Round(zy, 9), Math.Round(zz, 9)));
            Axis(l, 0, 1, 0, out double yx, out double yy, out double yz);
            Assert.Equal((0.0, 1.0, 0.0), (Math.Round(yx, 9), Math.Round(yy, 9), Math.Round(yz, 9)));
        }

        [Fact]
        public void WhereTheHandIs_DecidesWhereTheToolGoes_CentimetreForCentimetre()
        {
            BodyAnchoredMapping m = Mapping(out _);
            RigidTransform a = m.Map(0, Hand(0.30, 0.20, -0.40));
            RigidTransform b = m.Map(0, Hand(0.40, 0.15, -0.30));
            Assert.Equal(0.10, b.X - a.X, 9);
            Assert.Equal(-0.05, b.Y - a.Y, 9);
            Assert.Equal(0.10, b.Z - a.Z, 9);

            BodyAnchoredMapping half = Mapping(out _, new BodyMappingOptions { PositionScale = 0.5 });
            Assert.Equal(0.05, half.Map(0, Hand(0.40, 0.20, -0.40)).X - half.Map(0, Hand(0.30, 0.20, -0.40)).X, 9);
        }

        [Fact]
        public void TurningThePalm_TurnsTheGripper_AboutTheSameAxis()
        {
            BodyAnchoredMapping m = Mapping(out _);
            RigidTransform yaw30 = RigidTransform.Rotation(0, 0, 1, Math.PI / 6);
            RigidTransform t = m.Map(0, Hand(0.30, 0.20, -0.40, yaw30));
            Axis(t, 0, 0, 1, out double x, out double y, out double z);
            Assert.Equal(Math.Cos(Math.PI / 6), x, 9);
            Assert.Equal(Math.Sin(Math.PI / 6), y, 9);
            Assert.Equal(0, z, 9);
        }

        [Fact]
        public void TheBodyFrame_IsTheHeadsPositionAndHeadingOnly()
        {
            // The same hand relative to the head, wherever the operator stands and faces: the same tool.
            BodyAnchoredMapping here = Mapping(out SerialChainProfile profile);
            RigidTransform expected = here.Map(0, Hand(0.35, 0.10, -0.30));

            var elsewhere = new BodyAnchoredMapping(profile, BodyAnchoredMapping.NeutralTools(profile));
            RigidTransform face = RigidTransform.Rotation(0, 0, 1, 1.1);
            // Looking 40 degrees down at recentre changes nothing: only the heading is kept.
            RigidTransform look = face.Compose(RigidTransform.Rotation(0, 1, 0, 0.7));
            var head = new RigidTransform(2.0, -1.0, 1.7, look.Qw, look.Qx, look.Qy, look.Qz);
            elsewhere.Recentre(head);
            RigidTransform world = new RigidTransform(2.0, -1.0, 1.7, face.Qw, face.Qx, face.Qy, face.Qz).Compose(Hand(0.35, 0.10, -0.30));
            RigidTransform actual = elsewhere.Map(0, world);
            PoseClutchTests.AssertSame(expected, actual);
        }

        [Fact]
        public void AroundTheNeutralPose_TheArmsReachTheHands_AndNeverCollide()
        {
            // The default calibration must be usable: hands within 10 cm of neutral, palms forward, are reached.
            BodyAnchoredMapping m = Mapping(out SerialChainProfile profile);
            SerialChainCollisionModel collision = SerialChainSolverTests.Collision();
            int reached = 0, trials = 0;
            foreach (double dx in new[] { -0.1, 0, 0.1 })
            foreach (double dy in new[] { -0.1, 0, 0.1 })
            foreach (double dz in new[] { -0.1, 0, 0.1 })
            {
                var teleop = new SerialChainTeleop(profile, collision);
                Assert.True(teleop.TryTakePose(ReadyMeasured(profile)));
                for (int a = 0; a < 2; a++)
                {
                    double side = profile.Arms[a].Mount.Y >= 0 ? 0.20 : -0.20;
                    teleop.EngageAt(a, m.Map(a, Hand(0.30 + dx, side + (a == 0 ? dy : -dy), -0.40 + dz)));
                }

                for (int k = 0; k < 300; k++)
                {
                    teleop.Step();
                    double[] pose = teleop.Commanded(0).ToArray().Concat(teleop.Commanded(1).ToArray()).ToArray();
                    Assert.False(SerialChainCollision.Check(profile, collision, pose).IsHit);
                }

                for (int a = 0; a < 2; a++)
                {
                    trials++;
                    RigidTransform got = teleop.Tool(a), want = teleop.Target(a);
                    double mm = 1000 * Math.Sqrt(Math.Pow(got.X - want.X, 2) + Math.Pow(got.Y - want.Y, 2) + Math.Pow(got.Z - want.Z, 2));
                    if (mm < 1 && PoseClutchTests.AngleBetween(got, want) < 0.01) reached++;
                }
            }

            Assert.True(reached >= trials * 0.9, $"{reached} of {trials} neutral-pose targets reached");
        }

        [Fact]
        public void EngagingAt_ATarget_GetsThereThroughShortSteps_AndPreviewMovesNothing()
        {
            BodyAnchoredMapping m = Mapping(out SerialChainProfile profile);
            var teleop = new SerialChainTeleop(profile, SerialChainSolverTests.Collision());
            Assert.True(teleop.TryTakePose(ReadyMeasured(profile)));
            RigidTransform target = m.Map(0, Hand(0.30, 0.20, -0.40));

            double[] before = teleop.Commanded(0).ToArray();
            teleop.Preview(0, target);
            teleop.Step();
            Assert.Equal(before, teleop.Commanded(0).ToArray());
            PoseClutchTests.AssertSame(target, teleop.Target(0));

            Assert.True(teleop.EngageAt(0, target));
            teleop.Step();
            double[] after = teleop.Commanded(0).ToArray();
            Assert.All(Enumerable.Range(0, 7), j => Assert.True(Math.Abs(after[j] - before[j]) <= 0.1 + 1e-12));
            for (int k = 0; k < 200; k++) teleop.Step();
            Assert.True(PoseClutchTests.AngleBetween(teleop.Tool(0), target) < 0.01);

            teleop.Track(0, m.Map(0, Hand(0.35, 0.20, -0.40)));
            for (int k = 0; k < 50; k++) teleop.Step();
            Assert.True(Math.Abs(teleop.Tool(0).X - (target.X + 0.05)) < 1e-3, $"tool x {teleop.Tool(0).X}, wanted {target.X + 0.05}");
        }

        [Fact]
        public void MappingAllocatesNothing()
        {
            BodyAnchoredMapping m = Mapping(out _);
            RigidTransform hand = Hand(0.3, 0.2, -0.4);
            AllocationAssert.Zero(() => m.Map(0, hand), iterations: 1000);
        }

        private static JointSample[] ReadyMeasured(SerialChainProfile profile)
        {
            var samples = new List<JointSample>();
            foreach (ChainArm arm in profile.Arms)
            {
                bool mirrored = arm.Mount.Y < 0;
                for (int j = 0; j < arm.Joints.Length; j++)
                {
                    double q = mirrored && j % 2 == 0 ? -BodyAnchoredMapping.FrankaReady[j] : BodyAnchoredMapping.FrankaReady[j];
                    samples.Add(new JointSample(arm.Joints[j].MotorId, (float)q, true));
                }

                if (arm.HasGripper) samples.Add(new JointSample(arm.Gripper.MotorId, 1f, true));
            }

            return samples.ToArray();
        }
    }
}
