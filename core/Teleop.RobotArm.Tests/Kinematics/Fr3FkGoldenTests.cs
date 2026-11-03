using System.IO;
using System.Text;
using System.Text.Json;
using Teleop.Core.Types;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Kinematics
{
    /// <summary>
    /// The FR3 Duo's tool poses at seeded joint sets, as Core's kinematics compute them from the profile: the golden file the
    /// Isaac renderer's forward kinematics (robot/isaac/teleop_isaac/kinematics.py, from the URDF) is held to, so the
    /// rendered robot is the robot the IK and the predictors reason about (docs/adr/0032 decision 2).
    /// </summary>
    public class Fr3FkGoldenTests
    {
        [Fact]
        public void TheIsaacGoldenFile_IsWhatCoresKinematicsCompute()
        {
            string expected = GoldenJson();
            string path = Path.Combine(SerialChainKinematicsTests.RepoRoot(), "robot", "isaac", "golden", "fr3-duo-fk-v1.json");
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, expected, new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-isaac-goldens`");
            GoldenFile.AssertSame(File.ReadAllText(path), expected);
        }

        internal static string GoldenJson()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var rng = new SeededRng(2026);
            var sets = new List<Dictionary<string, object>>();
            for (int k = 0; k < 12; k++)
            {
                var joints = new Dictionary<string, double>();
                var tools = new Dictionary<string, double[]>();
                foreach (ChainArm arm in profile.Arms)
                {
                    var q = new double[arm.Joints.Length];
                    for (int j = 0; j < q.Length; j++)
                    {
                        ChainJointLimits l = arm.Joints[j].Limits;
                        q[j] = k == 0 ? 0 : l.SoftLower + rng.NextDouble() * (l.SoftUpper - l.SoftLower);
                        joints[arm.Joints[j].Name] = q[j];
                    }

                    RigidTransform t = SerialChainKinematics.Tool(arm, q);
                    tools[arm.Name] = new[] { t.X, t.Y, t.Z, t.Qw, t.Qx, t.Qy, t.Qz };
                }

                sets.Add(new Dictionary<string, object> { ["joints"] = joints, ["tools"] = tools });
            }

            var golden = new Dictionary<string, object>
            {
                ["about"] = "FR3 Duo tool poses (root frame; x y z qw qx qy qz) at seeded joint sets, from Core's SerialChainKinematics, written by Fr3FkGoldenTests. Set 0 is every joint at 0.",
                ["profile"] = profile.Name,
                ["sets"] = sets,
            };
            return JsonSerializer.Serialize(golden, new JsonSerializerOptions { WriteIndented = true }).Replace("\r\n", "\n") + "\n";
        }
    }
}
