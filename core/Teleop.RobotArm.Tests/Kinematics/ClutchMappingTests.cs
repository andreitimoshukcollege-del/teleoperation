using System.Numerics;
using Teleop.Core.Tests.TestSupport;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Kinematics
{
    public class ClutchMappingTests
    {
        private static readonly RobotArmProfile JetRover = RobotArmProfile.JetRoverMeasuredDefault;
        private const float Floor = -0.05f;
        private static readonly Vector3 Start = new Vector3(0.15f, 0f, 0.08f); // the validated hardware target

        private static void AssertClose(Vector3 expected, Vector3 actual, float tolerance = 1e-5f)
        {
            Assert.True(Vector3.Distance(expected, actual) < tolerance,
                $"Expected {expected}, got {actual} (distance {Vector3.Distance(expected, actual)})");
        }

        private static float ShoulderDistance(Vector3 p) =>
            Vector3.Distance(p, new Vector3(0f, 0f, JetRover.BaseHeight));

        private static float MaxReach => JetRover.ProximalLinkLength + JetRover.DistalLinkLength;

        [Fact]
        public void Engage_DoesNotMoveTheTarget()
        {
            var clutch = new ClutchMapping(0.5f, Floor, Start, 0.2f);
            clutch.Engage(JetRover, handPosition: new Vector3(3f, -2f, 1.5f), handPitch: 1.0f);
            clutch.Update(JetRover, new Vector3(3f, -2f, 1.5f), 1.0f);

            AssertClose(Start, clutch.Target);
            Assert.Equal(0.2f, clutch.Pitch, 5);
        }

        [Fact]
        public void Update_AppliesTheScaleToHandMotion()
        {
            var clutch = new ClutchMapping(0.5f, Floor, Start, 0f);
            var hand = new Vector3(0.4f, 0.1f, 1.2f);
            clutch.Engage(JetRover, hand, 0f);
            clutch.Update(JetRover, hand + new Vector3(-0.04f, 0.02f, 0.06f), 0f);

            AssertClose(Start + new Vector3(-0.02f, 0.01f, 0.03f), clutch.Target);
        }

        [Fact]
        public void Update_PitchFollowsTheHandsPitchChangeOneToOne()
        {
            var clutch = new ClutchMapping(0.5f, Floor, Start, 0.1f);
            clutch.Engage(JetRover, Vector3.Zero, handPitch: -0.3f);
            clutch.Update(JetRover, Vector3.Zero, handPitch: 0.2f);

            Assert.Equal(0.6f, clutch.Pitch, 5);
        }

        [Fact]
        public void Update_WithNaNPitch_KeepsThePitchFromEngage()
        {
            var clutch = new ClutchMapping(0.5f, Floor, Start, 0.1f);
            clutch.Engage(JetRover, Vector3.Zero, handPitch: -0.3f);
            clutch.Update(JetRover, new Vector3(0.01f, 0f, 0f), float.NaN);

            Assert.Equal(0.1f, clutch.Pitch, 5);
        }

        [Fact]
        public void Release_HoldsTheTarget_AndUpdateIgnoresTheHand()
        {
            var clutch = new ClutchMapping(1f, Floor, Start, 0f);
            clutch.Engage(JetRover, Vector3.Zero, 0f);
            clutch.Update(JetRover, new Vector3(0.02f, 0f, 0f), 0f);
            Vector3 held = clutch.Target;

            clutch.Release();
            clutch.Update(JetRover, new Vector3(5f, 5f, 5f), 1f);

            Assert.False(clutch.IsEngaged);
            AssertClose(held, clutch.Target);
        }

        [Fact]
        public void ReEngageSomewhereElse_DoesNotJump()
        {
            var clutch = new ClutchMapping(0.5f, Floor, Start, 0f);
            clutch.Engage(JetRover, Vector3.Zero, 0f);
            clutch.Update(JetRover, new Vector3(0.02f, 0f, 0f), 0f);
            Vector3 held = clutch.Target;
            clutch.Release();

            clutch.Engage(JetRover, new Vector3(-1f, 2f, 0.5f), 0.4f);
            clutch.Update(JetRover, new Vector3(-1f, 2f, 0.5f), 0.4f);

            AssertClose(held, clutch.Target);
        }

        [Fact]
        public void PushingPastReach_StaysOnTheEdge_AndComingBackMovesImmediately()
        {
            var clutch = new ClutchMapping(1f, Floor, Start, 0f);
            clutch.Engage(JetRover, Vector3.Zero, 0f);
            clutch.Update(JetRover, new Vector3(1f, 0f, 0f), 0f);

            Assert.True(ShoulderDistance(clutch.Target) <= MaxReach + 1e-5f);
            Assert.True(ShoulderDistance(clutch.Target) > MaxReach - 1e-3f);

            // The projection is applied to the desired point from the anchor, so the target is a
            // function of the hand alone: coming back inside reach moves it inside straight away.
            clutch.Update(JetRover, Vector3.Zero, 0f);
            AssertClose(Start, clutch.Target);
        }

        [Fact]
        public void TheTargetNeverGoesBelowTheFloor()
        {
            var clutch = new ClutchMapping(1f, Floor, Start, 0f);
            clutch.Engage(JetRover, Vector3.Zero, 0f);
            clutch.Update(JetRover, new Vector3(0f, 0f, -1f), 0f);

            Assert.True(clutch.Target.Z >= Floor - 1e-6f, $"Z {clutch.Target.Z} is below the floor {Floor}");
            Assert.True(ShoulderDistance(clutch.Target) <= MaxReach + 1e-5f);
        }

        [Fact]
        public void Engage_PullsAnOutOfReachInitialTargetIn()
        {
            var clutch = new ClutchMapping(0.5f, Floor, new Vector3(2f, 0f, 0f), 0f);
            clutch.Engage(JetRover, Vector3.Zero, 0f);

            Assert.True(ShoulderDistance(clutch.Target) <= MaxReach + 1e-5f);
        }

        [Theory]
        [InlineData(0f)]
        [InlineData(-0.5f)]
        [InlineData(float.NaN)]
        [InlineData(float.PositiveInfinity)]
        public void Constructor_RejectsANonPositiveOrNonFiniteScale(float scale)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new ClutchMapping(scale, Floor, Start, 0f));
        }

        [Fact]
        public void Update_AllocatesNothing()
        {
            var clutch = new ClutchMapping(0.5f, Floor, Start, 0f);
            clutch.Engage(JetRover, Vector3.Zero, 0f);
            float x = 0f;
            AllocationAssert.Zero(() =>
            {
                x += 0.0001f;
                clutch.Update(JetRover, new Vector3(x % 0.1f, 0f, 0f), 0f);
            });
        }
    }

    public class ProjectToWorkspaceTests
    {
        private static readonly RobotArmProfile JetRover = RobotArmProfile.JetRoverMeasuredDefault;

        [Fact]
        public void APointInsideTheWorkspace_IsUnchanged()
        {
            var p = new Vector3(0.12f, 0.03f, 0.1f);
            Assert.Equal(p, ArmKinematics.ProjectToWorkspace(JetRover, p, -0.05f));
        }

        [Fact]
        public void APointBeyondReach_KeepsItsAzimuth()
        {
            Vector3 q = ArmKinematics.ProjectToWorkspace(JetRover, new Vector3(1f, 1f, JetRover.BaseHeight), -0.05f);
            Assert.Equal(q.X, q.Y, 5);
            Assert.Equal(JetRover.BaseHeight, q.Z, 5);
        }

        [Fact]
        public void TheProjectedPoint_IsSolvableWithoutClamping()
        {
            Span<float> wrist = stackalloc float[JetRover.WristJointCount];
            foreach (Vector3 p in new[] { new Vector3(1f, 0f, -1f), new Vector3(0f, 0f, 2f), new Vector3(-0.3f, 0.2f, -0.2f), Vector3.Zero })
            {
                Vector3 q = ArmKinematics.ProjectToWorkspace(JetRover, p, -0.05f);
                ArmKinematics.TryInverse(JetRover, q, 0f, out _, out _, out _, wrist, out bool clamped);
                Assert.False(clamped, $"{p} projected to {q}, which TryInverse still clamps");
                Assert.True(q.Z >= -0.05f - 1e-6f);
            }
        }
    }

    public class GripperMappingTests
    {
        [Theory]
        [InlineData(0.20f, 0f)]
        [InlineData(0.08f, 0f)]
        [InlineData(0.015f, 1f)]
        [InlineData(0.0f, 1f)]
        public void ClampsToZeroAndOne(float distance, float expected)
        {
            Assert.Equal(expected, GripperMapping.FromPinchDistance(distance, 0.08f, 0.015f), 5);
        }

        [Fact]
        public void IsMonotonic_TighterPinchClosesMore()
        {
            float previous = -1f;
            for (float d = 0.1f; d >= 0f; d -= 0.005f)
            {
                float g = GripperMapping.FromPinchDistance(d, 0.08f, 0.015f);
                Assert.True(g >= previous, $"{d}: {g} < {previous}");
                previous = g;
            }
        }

        [Fact]
        public void AnUntrackedHand_IsNotTurnedIntoOpen()
        {
            Assert.True(float.IsNaN(GripperMapping.FromPinchDistance(float.NaN, 0.08f, 0.015f)));
        }

        [Fact]
        public void RejectsAnInvertedRange()
        {
            Assert.Throws<ArgumentException>(() => GripperMapping.FromPinchDistance(0.05f, 0.01f, 0.08f));
        }
    }

    public class WristRollMappingTests
    {
        [Fact]
        public void AppliesTheSignAndClamps()
        {
            Assert.Equal(0.5f, WristRollMapping.FromHandRoll(0.5f, 1f, 1.5f), 5);
            Assert.Equal(-0.5f, WristRollMapping.FromHandRoll(0.5f, -1f, 1.5f), 5);
            Assert.Equal(1.5f, WristRollMapping.FromHandRoll(3f, 1f, 1.5f), 5);
            Assert.Equal(-1.5f, WristRollMapping.FromHandRoll(-3f, 1f, 1.5f), 5);
        }

        [Fact]
        public void AnUntrackedHand_StaysNaN()
        {
            Assert.True(float.IsNaN(WristRollMapping.FromHandRoll(float.NaN, 1f, 1.5f)));
        }

        [Fact]
        public void Delta_FollowsTheTwistSinceEngage_NotTheAbsoluteRoll()
        {
            // The hand reads -1.6 rad at engage (a runtime whose palm "up" is off by 90°); the wrist was at 0.2.
            Assert.Equal(0.2f, WristRollMapping.FromHandRollDelta(-1.6f, -1.6f, 0.2f, 1f, 1.5f), 5);
            Assert.Equal(0.5f, WristRollMapping.FromHandRollDelta(-1.3f, -1.6f, 0.2f, 1f, 1.5f), 5);
            Assert.Equal(-0.1f, WristRollMapping.FromHandRollDelta(-1.3f, -1.6f, 0.2f, -1f, 1.5f), 5);
        }

        [Fact]
        public void Delta_WrapsAcrossPi_AndClamps()
        {
            // From 3.0 to -3.0 is a small +0.28 twist, not -6.
            Assert.Equal(0.283f, WristRollMapping.FromHandRollDelta(-3.0f, 3.0f, 0f, 1f, 1.5f), 2);
            Assert.Equal(1.5f, WristRollMapping.FromHandRollDelta(2.5f, 0f, 0f, 1f, 1.5f), 5);
        }
    }
}
