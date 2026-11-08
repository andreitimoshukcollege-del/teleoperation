using System;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Text.Json.Nodes;
using System.Xml.Linq;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Sim;

namespace Teleop.RobotHost.Tests.Sim
{
    /// <summary>
    /// The JetRover's arm as Isaac simulates it (docs/adr/0034 decision 2): the committed URDF and sidecar are what the robot's
    /// own descriptions make, and the URDF's chain is the IK's arm. <c>TELEOP_WRITE_GOLDEN=1</c> rewrites the files.
    /// </summary>
    public class JetRoverSimModelTests
    {
        private static string Root()
        {
            for (DirectoryInfo? d = new DirectoryInfo(AppContext.BaseDirectory); d != null; d = d.Parent)
            {
                if (Directory.Exists(Path.Combine(d.FullName, "core", "RobotProfiles")) && Directory.Exists(Path.Combine(d.FullName, "robot"))) return d.FullName;
            }

            throw new InvalidOperationException("no repo root");
        }

        private static RobotArmProfile Profile() => RobotArmProfileJson.Load(Path.Combine(Root(), "core", "RobotProfiles", "jetrover.json"));

        private static GenericArmPlantConfig Plant(RobotArmProfile profile)
        {
            GenericArmPlantConfig d = GenericArmPlantConfig.Default;
            return new GenericArmPlantConfig(profile, d.PulsePerRadian, d.PulsesPerSecond, d.StepSizePulses, d.MaxDirectionMagnitude, d.ZeroPulse,
                d.MinPulse, d.MaxPulse, d.GripperOpenPulse, d.GripperClosedPulse);
        }

        [Theory]
        [InlineData("jetrover.urdf")]
        [InlineData("jetrover.sim.json")]
        public void TheCommittedFiles_AreWhatTheModelsMake(string file)
        {
            RobotArmProfile profile = Profile();
            string expected = file.EndsWith(".urdf", StringComparison.Ordinal)
                ? JetRoverSimModel.Urdf(profile, JetRoverCollisionModel.Instance, Plant(profile))
                : JetRoverSimModel.SimJson(profile, Plant(profile));
            string path = Path.Combine(Root(), "robot", "jetrover", "urdf", file);
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1") File.WriteAllText(path, expected, new UTF8Encoding(false));
            Assert.True(File.Exists(path), $"{path} is missing: TELEOP_WRITE_GOLDEN=1 dotnet test Teleop.RobotHost.Tests --filter JetRoverSimModelTests");
            Assert.Equal(expected, File.ReadAllText(path).Replace("\r\n", "\n"));
        }

        [Fact]
        public void TheUrdfsChain_PutsTheWristWhereTheIkDoes()
        {
            RobotArmProfile profile = Profile();
            XDocument urdf = XDocument.Parse(JetRoverSimModel.Urdf(profile, JetRoverCollisionModel.Instance, Plant(profile)));
            var joints = urdf.Root!.Elements("joint").ToDictionary(j => (string)j.Attribute("name")!);
            var rng = new Random(7);
            for (int k = 0; k < 50; k++)
            {
                float yaw = (float)(rng.NextDouble() * 2 - 1), prox = (float)(rng.NextDouble() * 1.5), dist = (float)(rng.NextDouble() * 2 - 1.5);
                // Walk the URDF: base_yaw, proximal, distal, then the wrist joint's origin (the wrist point).
                Matrix4x4 t = Matrix4x4.Identity;
                foreach ((string name, float angle) in new[] { ("base_yaw", yaw), ("proximal", prox), ("distal", dist), ("wrist", 0f) })
                {
                    XElement j = joints[name];
                    Vector3 o = Vec((string)j.Element("origin")!.Attribute("xyz")!), axis = Vec((string)j.Element("axis")!.Attribute("xyz")!);
                    t = Matrix4x4.CreateFromAxisAngle(Vector3.Normalize(axis), angle) * Matrix4x4.CreateTranslation(o) * t;
                }

                Vector3 wrist = Vector3.Transform(Vector3.Zero, t);
                Vector3 ik = ArmKinematics.Forward(profile, yaw, prox, dist);
                Assert.True(Vector3.Distance(wrist, ik) < 1e-5f, $"pose {k}: URDF {wrist}, IK {ik}");
            }
        }

        [Fact]
        public void TheArmWeighsWhatARobotArmOfThisSizeDoes_AndTheGripperNeverReports()
        {
            RobotArmProfile profile = Profile();
            XDocument urdf = XDocument.Parse(JetRoverSimModel.Urdf(profile, JetRoverCollisionModel.Instance, Plant(profile)));
            double arm = urdf.Root!.Elements("link").Where(l => (string)l.Attribute("name")! != "base_link")
                .Sum(l => (double)l.Element("inertial")!.Element("mass")!.Attribute("value")!);
            Assert.InRange(arm, 0.2, 1.5); // a 5-servo arm of 13 cm links

            JsonNode sim = JsonNode.Parse(JetRoverSimModel.SimJson(profile, Plant(profile)))!;
            var motors = sim["motors"]!.AsArray();
            Assert.Equal(new[] { 1, 2, 3, 4, 5, 10 }, motors.Select(m => (int)m!["motorId"]!).ToArray());
            Assert.False((bool)motors.Single(m => (int)m!["motorId"]! == 10)!["feedback"]!);
            Assert.Equal(0.12217305, (double)motors.Single(m => (int)m!["motorId"]! == 2)!["zeroOffsetRadians"]!, 6);
        }

        private static Vector3 Vec(string s)
        {
            float[] v = s.Split(' ').Select(x => float.Parse(x, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
            return new Vector3(v[0], v[1], v[2]);
        }
    }
}
