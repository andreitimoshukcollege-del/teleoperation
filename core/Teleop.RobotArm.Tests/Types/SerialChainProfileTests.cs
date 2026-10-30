using System.IO;
using System.Text.Json;
using Teleop.Core.Types;
using Teleop.RobotArm.Tests.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Types
{
    /// <summary>A malformed profile routes angles to the wrong motors, so every check in <see cref="SerialChainProfile.Validate"/> must fail loudly (docs/adr/0026 decision 1).</summary>
    public class SerialChainProfileTests
    {
        private static SerialChainProfileDocument Fr3Document() =>
            JsonSerializer.Deserialize<SerialChainProfileDocument>(
                File.ReadAllText(Path.Combine(SerialChainKinematicsTests.RepoRoot(), "core", "RobotProfiles", "fr3-duo.json")),
                new JsonSerializerOptions { IncludeFields = true })!;

        [Fact]
        public void TheCommittedProfileIsValid() => Assert.Null(Fr3Document().ToProfile().Validate());

        [Fact]
        public void AMotorIdUsedTwiceAcrossArmsIsRefused()
        {
            SerialChainProfileDocument d = Fr3Document();
            d.arms[1].joints[0].motorId = 3;
            AssertRefused(d, "Motor id 3");
        }

        [Fact]
        public void AGripperIdThatCollidesWithAJointIsRefused()
        {
            SerialChainProfileDocument d = Fr3Document();
            d.arms[0].gripper.motorId = 12;
            AssertRefused(d, "Motor id 12");
        }

        [Fact]
        public void ANonUnitAxisIsRefused()
        {
            SerialChainProfileDocument d = Fr3Document();
            d.arms[0].joints[2].axis = new double[] { 0, 0, 2 };
            AssertRefused(d, "unit vector");
        }

        [Fact]
        public void SoftLimitsOutsideTheHardLimitsAreRefused()
        {
            SerialChainProfileDocument d = Fr3Document();
            d.arms[0].joints[0].softUpper = d.arms[0].joints[0].upper + 0.1;
            AssertRefused(d, "soft limits");
        }

        [Fact]
        public void AMissingJerkLimitIsRefused()
        {
            SerialChainProfileDocument d = Fr3Document();
            d.arms[1].joints[6].jerk = 0;
            AssertRefused(d, "jerk");
        }

        [Fact]
        public void ADuplicateArmNameIsRefused()
        {
            SerialChainProfileDocument d = Fr3Document();
            d.arms[1].name = "left";
            AssertRefused(d, "used twice");
        }

        [Fact]
        public void AnUnknownJointTypeOrKindIsRefused()
        {
            SerialChainProfileDocument d = Fr3Document();
            d.arms[0].joints[0].type = "continuous";
            AssertRefused(d, "continuous");

            d = Fr3Document();
            d.kind = "";
            AssertRefused(d, "kind");
        }

        [Fact]
        public void AFrameWithTwoNumbersIsRefused()
        {
            SerialChainProfileDocument d = Fr3Document();
            d.arms[0].mount[1].xyz = new double[] { 0, 0 };
            AssertRefused(d, "three numbers");
        }

        [Fact]
        public void LayoutV4sRowsAndGravity_AreTheTrainers()
        {
            // The trainer computes the same rows and gravity directions from its copy of the profile; its golden records them.
            SerialChainProfile profile = Fr3Document().ToProfile();
            Teleop.Core.Types.LearnedChainLayout layout = SerialChainShaping.LearnedLayout(profile);
            using var golden = JsonDocument.Parse(File.ReadAllText(Path.Combine(SerialChainKinematicsTests.RepoRoot(), "core", "testdata", "learned", "golden-v4.json")));
            JsonElement chain = golden.RootElement.GetProperty("chain");
            int[] motors = chain.GetProperty("motors").EnumerateArray().Select(e => e.GetInt32()).ToArray();
            int[] arms = chain.GetProperty("armOfRow").EnumerateArray().Select(e => e.GetInt32()).ToArray();
            int[] identity = chain.GetProperty("identity").EnumerateArray().Select(e => e.GetInt32()).ToArray();
            double[] gravity = chain.GetProperty("gravity").EnumerateArray().Select(e => e.GetDouble()).ToArray();
            Assert.Equal(motors.Length, layout.Rows);
            for (int r = 0; r < layout.Rows; r++)
            {
                Assert.Equal((motors[r], arms[r], identity[r]), ((int)layout.MotorOf(r), layout.ArmOf(r), layout.IdentityOf(r)));
            }

            for (int a = 0; a < layout.Arms; a++)
            {
                for (int k = 0; k < 3; k++) Assert.True(Math.Abs(gravity[3 * a + k] - layout.Gravity(a, k)) < 1e-12, $"arm {a} axis {k}");
            }
        }

        [Fact]
        public void InverseUndoesATransformAndComposeAssociates()
        {
            var rng = new SeededRng(3);
            for (int i = 0; i < 100; i++)
            {
                RigidTransform a = Random(ref rng), b = Random(ref rng), c = Random(ref rng);

                AssertIdentity(a.Inverse().Compose(a));
                AssertIdentity(a.Compose(a.Inverse()));

                RigidTransform left = a.Compose(b).Compose(c), right = a.Compose(b.Compose(c));
                left.TransformPoint(0.3, -0.2, 0.1, out double lx, out double ly, out double lz);
                right.TransformPoint(0.3, -0.2, 0.1, out double rx, out double ry, out double rz);
                Assert.True(Math.Abs(lx - rx) + Math.Abs(ly - ry) + Math.Abs(lz - rz) < 1e-12);
            }
        }

        private static RigidTransform Random(ref SeededRng rng) => RigidTransform.FromXyzRpy(
            rng.NextDouble() - 0.5, rng.NextDouble() - 0.5, rng.NextDouble() - 0.5,
            6 * (rng.NextDouble() - 0.5), 3 * (rng.NextDouble() - 0.5), 6 * (rng.NextDouble() - 0.5));

        private static void AssertIdentity(in RigidTransform t)
        {
            Assert.True(Math.Abs(t.X) + Math.Abs(t.Y) + Math.Abs(t.Z) < 1e-12, $"translation {t}");
            Assert.True(Math.Abs(Math.Abs(t.Qw) - 1) < 1e-12, $"rotation {t}");
        }

        private static void AssertRefused(SerialChainProfileDocument d, string reason)
        {
            FormatException e = Assert.Throws<FormatException>(() => d.ToProfile());
            Assert.Contains(reason, e.Message);
        }
    }
}
