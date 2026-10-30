using System.IO;
using Teleop.Eval.ImportUrdf;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.Tests.ImportUrdf
{
    /// <summary>
    /// <c>import-urdf</c> (docs/adr/0026 decision 3). The committed FR3 Duo profile must be exactly what the
    /// importer makes of the vendored URDF, so a hand edit to either one fails here.
    /// </summary>
    public class ImportUrdfCommandTests
    {
        /// <summary>The arguments <c>just import-fr3-profile</c> passes; change both together.</summary>
        private static string[] Fr3DuoArgs(string urdf, string output) => new[]
        {
            "import-urdf",
            "--urdf", urdf, "--out", output, "--name", "fr3-duo",
            "--arm", "left:left_fr3v2_hand_tcp:1", "--arm", "right:right_fr3v2_hand_tcp:11",
            "--gripper", "left:left_fr3v2_finger_joint1:8", "--gripper", "right:right_fr3v2_finger_joint1:18",
            "--acceleration", "10", "--jerk", "5000",
            "--upstream", "frankarobotics/franka_description@7aeeddc449edf8d62b594f9e36a81da53e7796f9",
        };

        [Fact]
        public void ReimportingTheVendoredUrdfReproducesTheCommittedProfileByteForByte()
        {
            string root = RepoRoot();
            string output = Path.Combine(Path.GetTempPath(), $"fr3-duo-{Guid.NewGuid():N}.json");
            try
            {
                int exit = ImportUrdfCommand.Run(
                    Fr3DuoArgs(Path.Combine(root, "robot", "franka", "fr3_duo", "urdf", "fr3_duo.urdf"), output),
                    TextWriter.Null, TextWriter.Null);

                Assert.Equal(0, exit);
                byte[] committed = File.ReadAllBytes(Path.Combine(root, "core", "RobotProfiles", "fr3-duo.json"));
                Assert.True(committed.AsSpan().SequenceEqual(File.ReadAllBytes(output)),
                    "core/RobotProfiles/fr3-duo.json is not what import-urdf makes of the vendored URDF; regenerate it with `just import-fr3-profile`");
            }
            finally
            {
                File.Delete(output);
            }
        }

        [Fact]
        public void TheCommittedProfileLoadsAndRecordsItsSource()
        {
            SerialChainProfile p = SerialChainProfileJson.Load(Path.Combine(RepoRoot(), "core", "RobotProfiles", "fr3-duo.json"));
            Assert.Equal("robot/franka/fr3_duo/urdf/fr3_duo.urdf", p.SourceUrdf);
            Assert.Equal("5f1ab744deb0097f014163395aa828a7988edcc72526fd8e5cb0e6e6a6494162", p.SourceSha256);
            Assert.All(p.Arms[0].Joints, j => Assert.Equal((10.0, 5000.0), (j.Limits.Acceleration, j.Limits.Jerk)));
            Assert.Equal(2.62, p.Arms[1].Joints[0].Limits.Velocity);
            Assert.Equal(6.0, p.Arms[1].Joints[0].Limits.PositionBasedDeceleration);
        }

        [Fact]
        public void FixedJointsFoldIntoMountOriginsAndTool()
        {
            UrdfModel model = UrdfModel.Parse(@"<robot name='t'>
                <link name='root'/><link name='a'/><link name='b'/><link name='c'/><link name='d'/><link name='tip'/>
                <joint name='mount' type='fixed'><parent link='root'/><child link='a'/><origin xyz='1 0 0'/></joint>
                <joint name='j1' type='revolute'><parent link='a'/><child link='b'/><axis xyz='0 0 1'/>
                  <limit lower='-1' upper='1' velocity='2' effort='3'/></joint>
                <joint name='between' type='fixed'><parent link='b'/><child link='c'/><origin xyz='0 0 0.5' rpy='0 0 -0.0'/></joint>
                <joint name='j2' type='prismatic'><parent link='c'/><child link='d'/><axis xyz='1 0 0'/>
                  <limit lower='0' upper='0.2' velocity='0.1' effort='5'/></joint>
                <joint name='tcp' type='fixed'><parent link='d'/><child link='tip'/><origin xyz='0 0 0.1'/></joint>
              </robot>");
            var args = ImportUrdfArgs.TryParse(new[] { "import-urdf", "--urdf", "x", "--out", "y", "--name", "t",
                "--arm", "only:tip:5", "--acceleration", "1", "--jerk", "2" }, out _)!;

            SerialChainProfileDocument d = ImportUrdfCommand.Build(model, args, "x", "sha");

            ChainArmDocument arm = Assert.Single(d.arms);
            Assert.Equal(new[] { "mount" }, arm.mount.Select(o => o.joint));
            Assert.Equal(new[] { "j1" }, arm.joints[0].origin.Select(o => o.joint));
            Assert.Equal(new[] { "between", "j2" }, arm.joints[1].origin.Select(o => o.joint));
            Assert.Equal(new[] { "tcp" }, arm.tool.Select(o => o.joint));
            Assert.Equal(new byte[] { 5, 6 }, arm.joints.Select(j => j.motorId));
            Assert.Equal("prismatic", arm.joints[1].type);
            Assert.Equal(-1.0, arm.joints[0].softLower);
            Assert.Equal(0.0, arm.joints[1].origin[0].rpy[2]);
            Assert.False(double.IsNegative(arm.joints[1].origin[0].rpy[2]), "-0 must be written as 0");
            Assert.False(arm.hasGripper);
            Assert.Null(d.ToProfile().Validate());
        }

        [Theory]
        [InlineData("continuous", "only fixed, revolute and prismatic")]
        [InlineData("floating", "only fixed, revolute and prismatic")]
        public void AJointTypeTheProfileCannotDescribeIsAnError(string type, string message)
        {
            UrdfModel model = UrdfModel.Parse($@"<robot name='t'><link name='root'/><link name='tip'/>
                <joint name='j' type='{type}'><parent link='root'/><child link='tip'/><limit velocity='1' effort='1'/></joint></robot>");
            var args = ImportUrdfArgs.TryParse(new[] { "import-urdf", "--urdf", "x", "--out", "y", "--name", "t",
                "--arm", "a:tip:1", "--acceleration", "1", "--jerk", "1" }, out _)!;

            FormatException e = Assert.Throws<FormatException>(() => ImportUrdfCommand.Build(model, args, "x", "sha"));
            Assert.Contains(message, e.Message);
        }

        [Fact]
        public void AMissingTipLinkFailsWithoutWritingAFile()
        {
            string root = RepoRoot();
            string output = Path.Combine(Path.GetTempPath(), $"fr3-missing-{Guid.NewGuid():N}.json");
            string[] args = Fr3DuoArgs(Path.Combine(root, "robot", "franka", "fr3_duo", "urdf", "fr3_duo.urdf"), output);
            args[Array.IndexOf(args, "left:left_fr3v2_hand_tcp:1")] = "left:no_such_link:1";
            var error = new StringWriter();

            Assert.Equal(1, ImportUrdfCommand.Run(args, TextWriter.Null, error));
            Assert.Contains("no_such_link", error.ToString());
            Assert.False(File.Exists(output));
        }

        [Theory]
        [InlineData("--bogus", "x")]
        [InlineData("--arm", "left:tip")]
        [InlineData("--jerk", "-1")]
        public void BadArgumentsExitWithUsage(string flag, string value)
        {
            var error = new StringWriter();
            Assert.Equal(64, ImportUrdfCommand.Run(new[] { "import-urdf", flag, value }, TextWriter.Null, error));
            Assert.Contains("Usage", error.ToString());
        }

        private static string RepoRoot()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                if (Directory.Exists(Path.Combine(dir.FullName, "core", "RobotProfiles")) &&
                    Directory.Exists(Path.Combine(dir.FullName, "robot", "franka")))
                {
                    return dir.FullName;
                }
            }

            throw new InvalidOperationException($"no repo root (core/RobotProfiles and robot/franka) above {AppContext.BaseDirectory}");
        }
    }
}
