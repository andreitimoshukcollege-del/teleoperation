using System.IO;
using Teleop.Eval.BuildProfile;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.Tests.BuildProfile
{
    public class BuildProfileCommandTests
    {
        private static string TempJsonPath() =>
            Path.Combine(Path.GetTempPath(), $"teleop-eval-test-{Path.GetRandomFileName()}.json");

        private static void AssertNullableFloatClose(float? expected, float? actual)
        {
            Assert.Equal(expected.HasValue, actual.HasValue);
            if (expected.HasValue)
            {
                Assert.Equal(expected.Value, actual!.Value, 3);
            }
        }

        // Exactly reproduces RobotArmProfile.JetRoverMeasuredDefault through the interactive
        // prompt sequence, joint by joint (BaseYaw, Proximal, Distal, Wrist#0, GripperMain) --
        // the regression check that the wizard can express the one real profile this codebase has
        // always run, not just structurally simpler ones.
        private static readonly string JetRoverTranscript = string.Join("\n", new[]
        {
            "jetrover",       // name
            "y",              // has rotating base
            "0.035",          // base height
            "0.13",           // proximal link length
            "0.13",           // distal link length
            "1",               // wrist joint count
            "y",              // has gripper
            "n",              // gripper rotates
            "1",              // BaseYaw motor id
            "n",              // BaseYaw wants min
            "n",              // BaseYaw wants max
            "n",              // BaseYaw wants zero-offset
            "2",              // Proximal motor id
            "y",              // Proximal wants min
            "-1.8851179",     // Proximal min angle
            "n",              // Proximal wants max
            "y",              // Proximal wants zero-offset
            "0.12217305",     // Proximal zero-offset (7 degrees, real calibrated mounting correction)
            "3",              // Distal motor id
            "n",              // Distal wants min
            "n",              // Distal wants max
            "n",              // Distal wants zero-offset
            "4",              // Wrist#0 motor id
            "n",              // Wrist#0 wants min
            "n",              // Wrist#0 wants max
            "n",              // Wrist#0 wants zero-offset
            "5",              // GripperMain motor id (no limit/offset prompts for gripper roles)
            "y",              // confirm write
        }) + "\n";

        [Fact]
        public void CannedTranscript_ReproducesJetRoverMeasuredDefault()
        {
            string outputPath = TempJsonPath();
            try
            {
                var args = BuildProfileArgs.TryParse(new[] { "--output", outputPath });
                using var input = new StringReader(JetRoverTranscript);
                using var output = new StringWriter();

                int exitCode = BuildProfileCommand.Run(args, input, output);

                Assert.Equal(0, exitCode);
                Assert.True(File.Exists(outputPath));

                RobotArmProfile written = RobotArmProfileJson.Load(outputPath);
                RobotArmProfile expected = RobotArmProfile.JetRoverMeasuredDefault;

                Assert.Equal(expected.Name, written.Name);
                Assert.Equal(expected.HasRotatingBase, written.HasRotatingBase);
                Assert.Equal(expected.BaseHeight, written.BaseHeight, 4);
                Assert.Equal(expected.ProximalLinkLength, written.ProximalLinkLength, 4);
                Assert.Equal(expected.DistalLinkLength, written.DistalLinkLength, 4);
                Assert.Equal(expected.WristJointCount, written.WristJointCount);
                Assert.Equal(expected.HasGripper, written.HasGripper);
                Assert.Equal(expected.GripperCanRotate, written.GripperCanRotate);
                Assert.Equal(expected.JointCount, written.JointCount);
                Assert.Null(written.Validate());

                for (int i = 0; i < expected.Joints.Length; i++)
                {
                    Assert.Equal(expected.Joints[i].MotorId, written.Joints[i].MotorId);
                    Assert.Equal(expected.Joints[i].Role, written.Joints[i].Role);
                    Assert.Equal(expected.Joints[i].WristIndex, written.Joints[i].WristIndex);
                    AssertNullableFloatClose(expected.Joints[i].MinAngleRadians, written.Joints[i].MinAngleRadians);
                    AssertNullableFloatClose(expected.Joints[i].MaxAngleRadians, written.Joints[i].MaxAngleRadians);
                    Assert.Equal(expected.Joints[i].ZeroOffsetRadians, written.Joints[i].ZeroOffsetRadians, 3);
                }
            }
            finally
            {
                if (File.Exists(outputPath))
                {
                    File.Delete(outputPath);
                }
            }
        }

        [Fact]
        public void DecliningTheFinalConfirmation_WritesNoFile()
        {
            string outputPath = TempJsonPath();
            string transcriptDeclined = JetRoverTranscript.Substring(0, JetRoverTranscript.LastIndexOf("y\n")) + "n\n";

            var args = BuildProfileArgs.TryParse(new[] { "--output", outputPath });
            using var input = new StringReader(transcriptDeclined);
            using var output = new StringWriter();

            int exitCode = BuildProfileCommand.Run(args, input, output);

            Assert.NotEqual(0, exitCode);
            Assert.False(File.Exists(outputPath));
        }

        [Fact]
        public void ExistingFileWithoutForce_RefusesToOverwrite()
        {
            string outputPath = TempJsonPath();
            File.WriteAllText(outputPath, "{}");
            try
            {
                var args = BuildProfileArgs.TryParse(new[] { "--output", outputPath });
                using var input = new StringReader(JetRoverTranscript);
                using var output = new StringWriter();

                int exitCode = BuildProfileCommand.Run(args, input, output);

                Assert.NotEqual(0, exitCode);
                Assert.Equal("{}", File.ReadAllText(outputPath)); // untouched
            }
            finally
            {
                File.Delete(outputPath);
            }
        }
    }
}
