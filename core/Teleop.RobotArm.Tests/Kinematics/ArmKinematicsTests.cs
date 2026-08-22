using System.Numerics;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Kinematics
{
    public class ArmKinematicsTests
    {
        // Regression baseline: the exact JetRover profile this codebase has always driven, so
        // default behavior is unchanged by the generalization from FourDofArmKinematics.
        private static readonly RobotArmProfile JetRover = RobotArmProfile.JetRoverMeasuredDefault;

        private static void AssertClose(Vector3 expected, Vector3 actual, float tolerance = 1e-4f)
        {
            Assert.True(
                Vector3.Distance(expected, actual) < tolerance,
                $"Expected {expected}, got {actual} (distance {Vector3.Distance(expected, actual)})");
        }

        [Theory]
        [InlineData(0.10f, 0.0f, 0.10f)]
        [InlineData(0.05f, 0.05f, 0.15f)]
        [InlineData(0.0f, 0.15f, 0.05f)]
        [InlineData(-0.08f, 0.08f, 0.20f)]
        [InlineData(0.20f, 0.0f, 0.04f)]
        public void Inverse_ThenForward_RoundTripsToTheOriginalTarget(float x, float y, float z)
        {
            var target = new Vector3(x, y, z);
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];

            bool ok = ArmKinematics.TryInverse(
                JetRover, target, desiredAbsolutePitchRadians: 0f,
                out float baseYaw, out float proximalPitch, out float distalPitch, wristPitches, out bool wasClamped);
            Assert.True(ok);
            Assert.False(wasClamped, "target is well within reach; should not have clamped");

            Vector3 recovered = ArmKinematics.Forward(JetRover, baseYaw, proximalPitch, distalPitch);

            AssertClose(target, recovered, tolerance: 1e-3f);
        }

        [Fact]
        public void Inverse_FullyExtendedReach_ProducesZeroDistalPitch()
        {
            // A target exactly at the sum of both link lengths, straight out horizontally, means
            // the arm is fully extended -- the elbow angle (distalPitch) should be ~0. The elbow
            // angle is extremely sensitive right at the reach boundary, so a target this close
            // ends up governed by TryInverse's own internal epsilon clamp rather than by this
            // offset -- ~0.055 rad is the resulting floor, not a bug.
            float fullReach = JetRover.ProximalLinkLength + JetRover.DistalLinkLength - 0.00001f;
            var target = new Vector3(fullReach, 0f, JetRover.BaseHeight);
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];

            ArmKinematics.TryInverse(
                JetRover, target, 0f, out _, out _, out float distalPitch, wristPitches, out _);

            Assert.True(MathF.Abs(distalPitch) < 0.1f, $"Expected distalPitch near 0, got {distalPitch}");
        }

        [Fact]
        public void Inverse_TargetBeyondMaxReach_ClampsInsteadOfFailing()
        {
            var farBeyondReach = new Vector3(10f, 0f, JetRover.BaseHeight);
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];

            bool ok = ArmKinematics.TryInverse(
                JetRover, farBeyondReach, 0f,
                out float baseYaw, out float proximalPitch, out float distalPitch, wristPitches, out bool wasClamped);

            Assert.True(ok);
            Assert.True(wasClamped, "a target far beyond max reach must report as clamped");
            Assert.False(float.IsNaN(baseYaw));
            Assert.False(float.IsNaN(proximalPitch));
            Assert.False(float.IsNaN(distalPitch));
        }

        [Fact]
        public void Inverse_TargetInsideMinReach_ClampsInsteadOfFailing()
        {
            // Equal link lengths => minReach is exactly 0, and TryInverse's own epsilon floor is
            // 1e-4 -- pick a reach well below that so this reliably exercises the clamp.
            var almostAtShoulder = new Vector3(0.00001f, 0f, JetRover.BaseHeight);
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];

            bool ok = ArmKinematics.TryInverse(
                JetRover, almostAtShoulder, 0f,
                out float baseYaw, out float proximalPitch, out float distalPitch, wristPitches, out bool wasClamped);

            Assert.True(ok);
            Assert.True(wasClamped, "a target essentially on top of the shoulder pivot must report as clamped");
            Assert.False(float.IsNaN(proximalPitch));
            Assert.False(float.IsNaN(distalPitch));
        }

        [Fact]
        public void Inverse_TargetWellWithinReach_DoesNotReportClamped()
        {
            var target = new Vector3(0.15f, 0f, 0.08f);
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];

            ArmKinematics.TryInverse(JetRover, target, 0f, out _, out _, out _, wristPitches, out bool wasClamped);

            Assert.False(wasClamped);
        }

        [Fact]
        public void Inverse_TargetNearlyDirectlyOverhead_BaseYawIsStableUnderTinyJitter()
        {
            var jitterA = new Vector3(0.0001f, 0.0001f, 0.2f);
            var jitterB = new Vector3(0.0001f, -0.0001f, 0.2f);
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];

            ArmKinematics.TryInverse(JetRover, jitterA, 0f, out float yawA, out _, out _, wristPitches, out _);
            ArmKinematics.TryInverse(JetRover, jitterB, 0f, out float yawB, out _, out _, wristPitches, out _);

            Assert.True(
                MathF.Abs(yawA - yawB) < 0.1f,
                $"Expected stable baseYaw under sub-millimeter jitter near the base axis, got {yawA} vs {yawB}");
        }

        [Fact]
        public void BaseYaw_PointsTowardTheTargetInTheHorizontalPlane()
        {
            var target = new Vector3(0f, 0.15f, JetRover.BaseHeight); // straight out along +Y
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];

            ArmKinematics.TryInverse(JetRover, target, 0f, out float baseYaw, out _, out _, wristPitches, out _);

            Assert.Equal(MathF.PI / 2f, baseYaw, 3);
        }

        [Theory]
        [InlineData(0.0)]
        [InlineData(0.3)]
        [InlineData(-0.3)]
        public void ExtractPitchRadians_IdentityForwardVector_RecoversZeroForNoRotation(double unused)
        {
            _ = unused;
            float pitch = ArmKinematics.ExtractPitchRadians(Quaternion.Identity);
            Assert.Equal(0f, pitch, 4);
        }

        [Fact]
        public void ExtractPitchRadians_RotationAroundY_RecoversThatAngle()
        {
            float angle = 0.4f;
            var rotation = Quaternion.CreateFromAxisAngle(-Vector3.UnitY, angle);

            float pitch = ArmKinematics.ExtractPitchRadians(rotation);

            Assert.Equal(angle, pitch, 3);
        }

        [Fact]
        public void Inverse_WristPitch_ClosesTheChainToTheDesiredAbsolutePitch()
        {
            // JetRover's one wrist joint must absorb whatever pitch the position-solving joints
            // don't already account for, so the full chain sums to exactly the desired pitch --
            // the behavior the old standalone InverseUpperPitch used to provide, now computed
            // directly inside TryInverse.
            var target = new Vector3(0.15f, 0f, 0.08f);
            float desired = 0.9f;
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];

            ArmKinematics.TryInverse(
                JetRover, target, desired, out _, out float proximalPitch, out float distalPitch, wristPitches, out _);

            Assert.Equal(desired, proximalPitch + distalPitch + wristPitches[0], 4);
        }

        // --- A second, structurally different profile: no rotating base, different link
        // lengths, no wrist joint, no gripper -- proves the generalization actually generalizes,
        // not just that JetRover's own numbers still work.

        private static readonly RobotArmProfile FixedBaseNoWrist = new RobotArmProfile(
            name: "fixed-base-no-wrist", hasRotatingBase: false, baseHeight: 0.02f,
            proximalLinkLength: 0.05f, distalLinkLength: 0.09f, wristJointCount: 0,
            hasGripper: false, gripperCanRotate: false,
            joints: new[]
            {
                new JointHardwareSpec(motorId: 1, role: JointRole.Proximal),
                new JointHardwareSpec(motorId: 2, role: JointRole.Distal),
            });

        [Fact]
        public void FixedBaseProfile_BaseYawIsAlwaysZero_EvenForAnOffPlaneTarget()
        {
            // A profile with no rotating base only ever operates in one fixed vertical plane --
            // an honest, structural limitation, not just a documentation promise.
            var offPlaneTarget = new Vector3(0.08f, 0.05f, 0.05f);
            Span<float> wristPitches = stackalloc float[FixedBaseNoWrist.WristJointCount];

            ArmKinematics.TryInverse(
                FixedBaseNoWrist, offPlaneTarget, 0f, out float baseYaw, out _, out _, wristPitches, out _);

            Assert.Equal(0f, baseYaw);
        }

        [Fact]
        public void FixedBaseProfile_ForwardIgnoresANonzeroBaseYawArgument()
        {
            Vector3 withZeroYaw = ArmKinematics.Forward(FixedBaseNoWrist, baseYaw: 0f, proximalPitch: 0.2f, distalPitch: -0.1f);
            Vector3 withNonzeroYaw = ArmKinematics.Forward(FixedBaseNoWrist, baseYaw: 1.2f, proximalPitch: 0.2f, distalPitch: -0.1f);

            AssertClose(withZeroYaw, withNonzeroYaw);
        }

        [Fact]
        public void ZeroWristJoints_TryInverse_NeverTouchesTheWristSpan()
        {
            var target = new Vector3(0.05f, 0f, 0.03f);
            Span<float> emptyWristSpan = Span<float>.Empty;

            bool ok = ArmKinematics.TryInverse(
                FixedBaseNoWrist, target, desiredAbsolutePitchRadians: 0f,
                out _, out _, out _, emptyWristSpan, out _);

            Assert.True(ok); // must not throw or index into the empty span
        }

        [Fact]
        public void Inverse_ThenForward_RoundTripsForTheFixedBaseProfileToo()
        {
            var target = new Vector3(0.10f, 0f, 0.06f);
            Span<float> wristPitches = stackalloc float[FixedBaseNoWrist.WristJointCount];

            ArmKinematics.TryInverse(
                FixedBaseNoWrist, target, 0f, out float baseYaw, out float proximalPitch, out float distalPitch,
                wristPitches, out bool wasClamped);
            Assert.False(wasClamped);

            Vector3 recovered = ArmKinematics.Forward(FixedBaseNoWrist, baseYaw, proximalPitch, distalPitch);

            AssertClose(target, recovered, tolerance: 1e-3f);
        }

        private static readonly RobotArmProfile ThreeWristJoints = new RobotArmProfile(
            name: "three-wrist-joints", hasRotatingBase: true, baseHeight: 0.035f,
            proximalLinkLength: 0.13f, distalLinkLength: 0.13f, wristJointCount: 3,
            hasGripper: false, gripperCanRotate: false,
            joints: new[]
            {
                new JointHardwareSpec(motorId: 1, role: JointRole.BaseYaw),
                new JointHardwareSpec(motorId: 2, role: JointRole.Proximal),
                new JointHardwareSpec(motorId: 3, role: JointRole.Distal),
                new JointHardwareSpec(motorId: 4, role: JointRole.Wrist, wristIndex: 0),
                new JointHardwareSpec(motorId: 5, role: JointRole.Wrist, wristIndex: 1),
                new JointHardwareSpec(motorId: 6, role: JointRole.Wrist, wristIndex: 2),
            });

        [Fact]
        public void MultipleWristJoints_OnlyTheFirstAbsorbsPitch_RestAreHeldAtZero()
        {
            var target = new Vector3(0.15f, 0f, 0.08f);
            float desired = 0.6f;
            Span<float> wristPitches = stackalloc float[ThreeWristJoints.WristJointCount];

            ArmKinematics.TryInverse(
                ThreeWristJoints, target, desired, out _, out float proximalPitch, out float distalPitch,
                wristPitches, out _);

            Assert.Equal(desired, proximalPitch + distalPitch + wristPitches[0], 4);
            Assert.Equal(0f, wristPitches[1]);
            Assert.Equal(0f, wristPitches[2]);
        }
    }
}
