using System;
using System.Numerics;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using static Teleop.RobotArm.Tests.Collision.TestArm;

namespace Teleop.RobotArm.Tests.Collision
{
    public class ArmCollisionTests
    {
        private static readonly ArmCollisionModel Model = TestArm.Model();
        private static readonly RobotArmProfile Arm = Profile(Model);

        [Fact]
        public void Frames_PutTheWristWhereForwardKinematicsDoes()
        {
            var random = new Random(20261019);
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            for (int i = 0; i < 500; i++)
            {
                var pose = new ArmPose(Next(random, 2f), Next(random, 2f), Next(random, 2f), Next(random, 2f), Next(random, 1.5f));
                ArmCollision.ComputeFrames(Arm, pose, frames);
                Vector3 expected = ArmKinematics.Forward(Arm, pose.BaseYaw, pose.Proximal, pose.Distal);
                Assert.True(Vector3.Distance(expected, frames[3].Origin) < 1e-5f, $"pose {i}: {frames[3].Origin} vs {expected}");
                // The wrist's X axis points along the absolute pitch, and roll does not move it.
                Vector3 forward = Vector3.Transform(Vector3.UnitX, frames[4].Rotation);
                float pitch = MathF.Atan2(forward.Z, MathF.Sqrt(forward.X * forward.X + forward.Y * forward.Y));
                float expectedPitch = pose.Proximal + pose.Distal + pose.Wrist;
                Assert.True(MathF.Abs(MathF.Sin(pitch) - MathF.Sin(expectedPitch)) < 1e-4f, $"pose {i}: pitch {pitch} vs {expectedPitch}");
            }
        }

        [Fact]
        public void Roll_RaisesTheGrippersLeftSide()
        {
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ArmCollision.ComputeFrames(Arm, new ArmPose(0f, 0f, 0f, 0f, 30f * Deg), frames);
            Vector3 left = Vector3.Transform(Vector3.UnitY, frames[4].Rotation);
            Assert.True(left.Z > 0.4f, $"left side {left}");
        }

        [Fact]
        public void BoxesOverlap_KnownCases()
        {
            Vector3 x = Vector3.UnitX, y = Vector3.UnitY, z = Vector3.UnitZ, unit = new Vector3(0.5f);
            Assert.True(ArmCollision.BoxesOverlap(Vector3.Zero, x, y, z, unit, new Vector3(0.9f, 0f, 0f), x, y, z, unit));
            Assert.False(ArmCollision.BoxesOverlap(Vector3.Zero, x, y, z, unit, new Vector3(1.1f, 0f, 0f), x, y, z, unit));
            // A cube turned 45 degrees about Z reaches 0.707 along X: touching at 1.2, clear at 1.25.
            Quaternion turn = Quaternion.CreateFromAxisAngle(z, MathF.PI / 4f);
            Vector3 tx = Vector3.Transform(x, turn), ty = Vector3.Transform(y, turn);
            Assert.True(ArmCollision.BoxesOverlap(Vector3.Zero, x, y, z, unit, new Vector3(1.2f, 0f, 0f), tx, ty, z, unit));
            Assert.False(ArmCollision.BoxesOverlap(Vector3.Zero, x, y, z, unit, new Vector3(1.25f, 0f, 0f), tx, ty, z, unit));
            // Edge against edge: two long bars crossed at 90 degrees, one above the other.
            Vector3 bar = new Vector3(1f, 0.05f, 0.05f);
            Assert.True(ArmCollision.BoxesOverlap(Vector3.Zero, x, y, z, bar, new Vector3(0f, 0f, 0.09f), y, -x, z, bar));
            Assert.False(ArmCollision.BoxesOverlap(Vector3.Zero, x, y, z, bar, new Vector3(0f, 0f, 0.11f), y, -x, z, bar));
        }

        [Fact]
        public void BoxesOverlap_NeverMissesAnOverlapThatSamplingFinds()
        {
            // Ground truth: a fine grid of points inside box B, tested against box A. Pairs the
            // sampling finds overlapping must be reported overlapping. The exact test may report a
            // little more (an overlap between grid points), never less.
            var random = new Random(11);
            int agreed = 0;
            for (int c = 0; c < 400; c++)
            {
                Quaternion qa = RandomRotation(random), qb = RandomRotation(random);
                Vector3 ha = RandomHalf(random), hb = RandomHalf(random);
                var cb = new Vector3(Next(random, 0.25f), Next(random, 0.25f), Next(random, 0.25f));
                Vector3 ax = Vector3.Transform(Vector3.UnitX, qa), ay = Vector3.Transform(Vector3.UnitY, qa), az = Vector3.Transform(Vector3.UnitZ, qa);
                Vector3 bx = Vector3.Transform(Vector3.UnitX, qb), by = Vector3.Transform(Vector3.UnitY, qb), bz = Vector3.Transform(Vector3.UnitZ, qb);
                bool sampled = SampledOverlap(Vector3.Zero, ax, ay, az, ha, cb, bx, by, bz, hb);
                bool exact = ArmCollision.BoxesOverlap(Vector3.Zero, ax, ay, az, ha, cb, bx, by, bz, hb);
                if (sampled) Assert.True(exact, $"case {c}: the sampling found an overlap the exact test missed");
                if (sampled == exact) agreed++;
            }

            Assert.True(agreed > 380, $"only {agreed} of 400 agree: the exact test reports far more than the sampling");
        }

        [Fact]
        public void PointBoxDistance_IsZeroInsideAndEuclideanOutside()
        {
            Vector3 min = new Vector3(0f, 0f, 0f), max = new Vector3(1f, 1f, 1f);
            Assert.Equal(0f, ArmCollision.PointBoxDistanceSquared(new Vector3(0.5f, 0.5f, 0.5f), min, max));
            Assert.Equal(2f, ArmCollision.PointBoxDistanceSquared(new Vector3(2f, 2f, 0.5f), min, max), 5);
        }

        [Fact]
        public void JointOutsideItsRange_IsReportedAsAJointLimit()
        {
            CollisionHit hit = ArmCollision.Check(Arm, Model, new ArmPose(130f * Deg, 60f * Deg, -90f * Deg, 0f, 0f));
            Assert.Equal(CollisionHitKind.JointLimit, hit.Kind);
            Assert.Equal(0, hit.A);
        }

        [Fact]
        public void ArmPointingIntoTheDeck_Collides()
        {
            // Proximal straight down from the shoulder, then forward: through the deck, clear of the floor.
            CollisionHit hit = ArmCollision.Check(Arm, Model, new ArmPose(0f, -90f * Deg, 90f * Deg, 0f, 0f));
            Assert.True(hit.IsHit);
            Assert.Contains("deck", hit.Describe(Model));
        }

        [Fact]
        public void GripperFoldedOntoTheProximalLink_Collides()
        {
            // Elbow folded hard and the wrist folded back: the gripper lands on the proximal link.
            CollisionHit hit = ArmCollision.Check(Arm, Model, new ArmPose(0f, 60f * Deg, -115f * Deg, -110f * Deg, 0f));
            Assert.True(hit.IsHit, hit.Describe(Model));
        }

        [Fact]
        public void Margin_OnceBlocked_StaysBlockedAsItGrows()
        {
            // The same pose against the deck with a growing margin: once blocked, always blocked.
            var pose = new ArmPose(0f, -15f * Deg, 0f, 0f, 0f);
            bool blockedBefore = false;
            for (int mm = 0; mm <= 40; mm++)
            {
                ArmCollisionModel model = TestArm.Model(mm / 1000f);
                bool blocked = !ArmCollision.IsValid(Profile(model), model, pose);
                Assert.True(!blockedBefore || blocked, $"margin {mm} mm freed a pose a smaller margin blocked");
                blockedBefore |= blocked;
            }

            Assert.True(blockedBefore, "the pose never touched the deck: the test checks nothing");
        }

        [Fact]
        public void Home_IsValid()
        {
            Assert.True(ArmCollision.IsValid(Arm, Model, Model.Home), ArmCollision.Check(Arm, Model, Model.Home).Describe(Model));
        }

        [Fact]
        public void Check_AllocatesNothing()
        {
            var pose = new ArmPose(0.2f, 0.5f, -0.8f, 0.1f, 0.2f);
            ArmCollisionModel model = Model;
            RobotArmProfile arm = Arm;
            Teleop.Core.Tests.TestSupport.AllocationAssert.Zero(() => ArmCollision.IsValid(arm, model, pose));
        }

        private static float Next(Random random, float magnitude) => magnitude * (2f * (float)random.NextDouble() - 1f);

        private static Vector3 RandomHalf(Random random) => new Vector3(
            0.02f + 0.08f * (float)random.NextDouble(), 0.02f + 0.08f * (float)random.NextDouble(), 0.02f + 0.08f * (float)random.NextDouble());

        private static Quaternion RandomRotation(Random random) => Quaternion.Normalize(new Quaternion(
            Next(random, 1f), Next(random, 1f), Next(random, 1f), Next(random, 1f) + 1e-3f));

        private static bool SampledOverlap(Vector3 ca, Vector3 ax, Vector3 ay, Vector3 az, Vector3 ha,
            Vector3 cb, Vector3 bx, Vector3 by, Vector3 bz, Vector3 hb)
        {
            const int steps = 14;
            for (int i = 0; i <= steps; i++)
            {
                for (int j = 0; j <= steps; j++)
                {
                    for (int k = 0; k <= steps; k++)
                    {
                        Vector3 p = cb + bx * (hb.X * (2f * i / steps - 1f)) + by * (hb.Y * (2f * j / steps - 1f)) + bz * (hb.Z * (2f * k / steps - 1f));
                        Vector3 d = p - ca;
                        if (MathF.Abs(Vector3.Dot(d, ax)) <= ha.X && MathF.Abs(Vector3.Dot(d, ay)) <= ha.Y && MathF.Abs(Vector3.Dot(d, az)) <= ha.Z)
                        {
                            return true;
                        }
                    }
                }
            }

            return false;
        }
    }
}
