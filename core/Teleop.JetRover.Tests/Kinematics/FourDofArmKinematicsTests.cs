using System.Numerics;
using Teleop.JetRover.Kinematics;

namespace Teleop.JetRover.Tests.Kinematics
{
    public class FourDofArmKinematicsTests
    {
        private static readonly ArmLinkLengths Links = new ArmLinkLengths(baseHeight: 0.035f, lower: 0.13f, middle: 0.13f);

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

            bool ok = FourDofArmKinematics.TryInverse(
                Links, target, out float baseYaw, out float lowerPitch, out float middlePitch, out bool wasClamped);
            Assert.True(ok);
            Assert.False(wasClamped, "target is well within reach; should not have clamped");

            Vector3 recovered = FourDofArmKinematics.Forward(Links, baseYaw, lowerPitch, middlePitch);

            AssertClose(target, recovered, tolerance: 1e-3f);
        }

        [Fact]
        public void Inverse_FullyExtendedReach_ProducesZeroMiddlePitch()
        {
            // A target exactly at the sum of both link lengths, straight out horizontally, means
            // the arm is fully extended -- the elbow angle (middlePitch) should be ~0, matching
            // the class doc's stated convention (middlePitch=0 <=> L2 continues straight from L1).
            // The elbow angle is extremely sensitive right at the reach boundary (its derivative
            // with respect to reach is unbounded exactly at full extension), so a target this
            // close ends up governed by TryInverse's own internal epsilon clamp (which exists to
            // avoid a numerically singular exact-boundary case) rather than by this offset --
            // ~0.055 rad is the resulting floor, not a bug; the tolerance below accounts for it.
            float fullReach = Links.Lower + Links.Middle - 0.00001f;
            var target = new Vector3(fullReach, 0f, Links.Base);

            FourDofArmKinematics.TryInverse(Links, target, out _, out _, out float middlePitch, out _);

            Assert.True(MathF.Abs(middlePitch) < 0.1f, $"Expected middlePitch near 0, got {middlePitch}");
        }

        [Fact]
        public void Inverse_TargetBeyondMaxReach_ClampsInsteadOfFailing()
        {
            var farBeyondReach = new Vector3(10f, 0f, Links.Base);

            bool ok = FourDofArmKinematics.TryInverse(
                Links, farBeyondReach, out float baseYaw, out float lowerPitch, out float middlePitch, out bool wasClamped);

            Assert.True(ok);
            Assert.True(wasClamped, "a target far beyond max reach must report as clamped");
            Assert.False(float.IsNaN(baseYaw));
            Assert.False(float.IsNaN(lowerPitch));
            Assert.False(float.IsNaN(middlePitch));
        }

        [Fact]
        public void Inverse_TargetInsideMinReach_ClampsInsteadOfFailing()
        {
            // Equal link lengths => minReach is exactly 0, and TryInverse's own epsilon floor is
            // 1e-4 -- pick a reach well below that so this reliably exercises the clamp rather
            // than landing right on the epsilon boundary by coincidence.
            var almostAtShoulder = new Vector3(0.00001f, 0f, Links.Base);

            bool ok = FourDofArmKinematics.TryInverse(
                Links, almostAtShoulder, out float baseYaw, out float lowerPitch, out float middlePitch, out bool wasClamped);

            Assert.True(ok);
            Assert.True(wasClamped, "a target essentially on top of the shoulder pivot must report as clamped");
            Assert.False(float.IsNaN(lowerPitch));
            Assert.False(float.IsNaN(middlePitch));
        }

        [Fact]
        public void Inverse_TargetWellWithinReach_DoesNotReportClamped()
        {
            var target = new Vector3(0.15f, 0f, 0.08f);

            FourDofArmKinematics.TryInverse(Links, target, out _, out _, out _, out bool wasClamped);

            Assert.False(wasClamped);
        }

        [Fact]
        public void Inverse_TargetNearlyDirectlyOverhead_BaseYawIsStableUnderTinyJitter()
        {
            // A real, observed bug (2026-08-13): when a target sits almost exactly above the
            // base-yaw axis (horizontal radius near zero), baseYaw = Atan2(y, x) is numerically
            // unstable -- a sub-millimeter jitter in x or y (well within real VR controller
            // tracking noise) can swing the result by up to 180 degrees even though the target
            // barely moved. Found via the JetRover VR drag feature: the Unity arm rig visibly
            // flashed between two poses while dragging a target that looked perfectly still.
            // baseYaw is physically meaningless when reach is directly overhead anyway (any yaw
            // reaches the same point), so TryInverse must stabilize it near the axis instead of
            // computing an ill-conditioned Atan2.
            var jitterA = new Vector3(0.0001f, 0.0001f, 0.2f);
            var jitterB = new Vector3(0.0001f, -0.0001f, 0.2f);

            FourDofArmKinematics.TryInverse(Links, jitterA, out float yawA, out _, out _, out _);
            FourDofArmKinematics.TryInverse(Links, jitterB, out float yawB, out _, out _, out _);

            Assert.True(
                MathF.Abs(yawA - yawB) < 0.1f,
                $"Expected stable baseYaw under sub-millimeter jitter near the base axis, got {yawA} vs {yawB}");
        }

        [Fact]
        public void BaseYaw_PointsTowardTheTargetInTheHorizontalPlane()
        {
            var target = new Vector3(0f, 0.15f, Links.Base); // straight out along +Y

            FourDofArmKinematics.TryInverse(Links, target, out float baseYaw, out _, out _, out _);

            Assert.Equal(MathF.PI / 2f, baseYaw, 3);
        }

        [Theory]
        [InlineData(0.0)]
        [InlineData(0.3)]
        [InlineData(-0.3)]
        public void ExtractPitchRadians_IdentityForwardVector_RecoversZeroForNoRotation(double unused)
        {
            _ = unused;
            float pitch = FourDofArmKinematics.ExtractPitchRadians(Quaternion.Identity);
            Assert.Equal(0f, pitch, 4);
        }

        [Fact]
        public void ExtractPitchRadians_RotationAroundY_RecoversThatAngle()
        {
            float angle = 0.4f;
            // Rotating -X-forward around Y matches how "pitching up" should read as a positive
            // elevation for this arm's convention (Z-up, X-forward, right-handed).
            var rotation = Quaternion.CreateFromAxisAngle(-Vector3.UnitY, angle);

            float pitch = FourDofArmKinematics.ExtractPitchRadians(rotation);

            Assert.Equal(angle, pitch, 3);
        }

        [Fact]
        public void InverseUpperPitch_ClosesTheChainToTheDesiredAbsolutePitch()
        {
            float lowerPitch = 0.5f;
            float middlePitch = -0.2f;
            float desired = 0.9f;

            float upperPitch = FourDofArmKinematics.InverseUpperPitch(lowerPitch, middlePitch, desired);

            Assert.Equal(desired, lowerPitch + middlePitch + upperPitch, 4);
        }
    }
}
