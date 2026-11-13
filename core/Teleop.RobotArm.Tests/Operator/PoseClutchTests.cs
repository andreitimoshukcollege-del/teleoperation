using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>The six-degree-of-freedom clutch (docs/adr/0029 decision 3).</summary>
    public class PoseClutchTests
    {
        private static readonly RigidTransform Tool = RigidTransform.FromXyzRpy(0.4, 0.1, 0.3, 3.0, 0.2, -0.5);
        private static readonly RigidTransform Hand = RigidTransform.FromXyzRpy(0.1, -0.2, 1.1, 0.3, -0.4, 1.2);

        [Fact]
        public void Released_TheTargetHolds()
        {
            var c = new PoseClutch(1.0, Tool);
            c.Follow(Hand);
            AssertSame(Tool, c.Target);
        }

        [Fact]
        public void AHandThatHasNotMoved_LeavesTheToolWhereItWas()
        {
            var c = new PoseClutch(1.0, RigidTransform.Identity);
            c.Engage(Hand, Tool);
            c.Follow(Hand);
            AssertSame(Tool, c.Target);
        }

        [Fact]
        public void TheHandsTranslation_IsScaled()
        {
            var c = new PoseClutch(0.5, RigidTransform.Identity);
            c.Engage(Hand, Tool);
            c.Follow(new RigidTransform(Hand.X + 0.1, Hand.Y - 0.04, Hand.Z + 0.02, Hand.Qw, Hand.Qx, Hand.Qy, Hand.Qz));
            Assert.Equal(Tool.X + 0.05, c.Target.X, 12);
            Assert.Equal(Tool.Y - 0.02, c.Target.Y, 12);
            Assert.Equal(Tool.Z + 0.01, c.Target.Z, 12);
            AssertSameRotation(Tool, c.Target);
        }

        [Fact]
        public void TheHandsRotation_TurnsTheTool_AboutTheSameRootAxis()
        {
            // Turning the hand 30 degrees about the root's vertical turns the tool 30 degrees about the root's vertical,
            // whatever either is pointing at.
            var c = new PoseClutch(1.0, RigidTransform.Identity);
            c.Engage(Hand, Tool);
            RigidTransform turn = RigidTransform.Rotation(0, 0, 1, Math.PI / 6);
            c.Follow(Rotated(turn, Hand));
            AssertSameRotation(Rotated(turn, Tool), c.Target);
            Assert.Equal(Tool.X, c.Target.X, 12);
        }

        [Fact]
        public void ReEngaging_Elsewhere_DoesNotJump()
        {
            var c = new PoseClutch(1.0, RigidTransform.Identity);
            c.Engage(Hand, Tool);
            c.Follow(Rotated(RigidTransform.Rotation(1, 0, 0, 0.4), Hand));
            RigidTransform reached = c.Target;
            c.Release();
            RigidTransform elsewhere = RigidTransform.FromXyzRpy(-0.3, 0.5, 0.9, 1.0, 1.0, 1.0);
            c.Engage(elsewhere, reached);
            c.Follow(elsewhere);
            AssertSame(reached, c.Target);
        }

        [Fact]
        public void AnUnnormalisedHand_StillGivesAUnitTarget()
        {
            var c = new PoseClutch(1.0, RigidTransform.Identity);
            c.Engage(Hand, Tool);
            c.Follow(new RigidTransform(Hand.X, Hand.Y, Hand.Z, Hand.Qw * 1.01, Hand.Qx * 1.01, Hand.Qy * 1.01, Hand.Qz * 1.01));
            RigidTransform t = c.Target;
            Assert.Equal(1.0, Math.Sqrt(t.Qw * t.Qw + t.Qx * t.Qx + t.Qy * t.Qy + t.Qz * t.Qz), 12);
        }

        [Fact]
        public void APositionScaleThatIsNotPositive_IsRefused()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new PoseClutch(0, Tool));
            Assert.Throws<ArgumentOutOfRangeException>(() => new PoseClutch(double.NaN, Tool));
        }

        private static RigidTransform Rotated(in RigidTransform turn, in RigidTransform t)
        {
            RigidTransform r = turn.Compose(new RigidTransform(0, 0, 0, t.Qw, t.Qx, t.Qy, t.Qz));
            return new RigidTransform(t.X, t.Y, t.Z, r.Qw, r.Qx, r.Qy, r.Qz);
        }

        internal static void AssertSame(in RigidTransform expected, in RigidTransform actual)
        {
            Assert.Equal(expected.X, actual.X, 12);
            Assert.Equal(expected.Y, actual.Y, 12);
            Assert.Equal(expected.Z, actual.Z, 12);
            AssertSameRotation(expected, actual);
        }

        internal static void AssertSameRotation(in RigidTransform expected, in RigidTransform actual, double toleranceRad = 1e-6)
        {
            Assert.True(AngleBetween(expected, actual) <= toleranceRad, $"rotations differ by {AngleBetween(expected, actual)} rad");
        }

        internal static double AngleBetween(in RigidTransform a, in RigidTransform b)
        {
            double dot = Math.Abs(a.Qw * b.Qw + a.Qx * b.Qx + a.Qy * b.Qy + a.Qz * b.Qz);
            return 2 * Math.Acos(Math.Min(1.0, dot));
        }
    }
}
