using System;
using System.Collections.Generic;
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

        /// <summary>The CAD meshes' manifest (docs/adr/0034 amendment 6), or none before <c>just jetrover-visuals</c> has run.</summary>
        private static IReadOnlyList<JetRoverSimModel.Visual> Visuals()
        {
            string path = Path.Combine(Root(), "robot", "jetrover", "urdf", "meshes", "visuals.json");
            return File.Exists(path) ? JetRoverSimModel.ParseVisuals(File.ReadAllText(path)) : Array.Empty<JetRoverSimModel.Visual>();
        }

        private static IReadOnlyList<double> Travel()
        {
            string path = Path.Combine(Root(), "robot", "jetrover", "urdf", "meshes", "visuals.json");
            return File.Exists(path) ? JetRoverSimModel.ParseFingerTravel(File.ReadAllText(path)) : Array.Empty<double>();
        }

        private static IReadOnlyList<JetRoverSimModel.Pad> Pads()
        {
            string path = Path.Combine(Root(), "robot", "jetrover", "urdf", "meshes", "visuals.json");
            return File.Exists(path) ? JetRoverSimModel.ParsePads(File.ReadAllText(path)) : Array.Empty<JetRoverSimModel.Pad>();
        }

        [Fact]
        public void TheFingers_CollideAsTheirPads_WideEnoughForTheCan_AndMeetClosed()
        {
            IReadOnlyList<JetRoverSimModel.Pad> pads = Pads();
            Assert.Equal(2, pads.Count);
            JetRoverSimModel.Pad left = pads.Single(p => p.Link == "left_finger"), right = pads.Single(p => p.Link == "right_finger");
            double gap = (left.Center.Y - left.Size.Y / 2) - (right.Center.Y + right.Size.Y / 2); // inner faces at the CAD's pose
            Assert.InRange(gap, 0.025, 0.045);
            IReadOnlyList<double> travel = Travel();
            Assert.Equal(21, travel.Count);
            for (int i = 1; i < travel.Count; i++) Assert.True(travel[i] > travel[i - 1], "the fingers close the more the gripper does");
            double open = gap - 2 * travel[0], closed = gap - 2 * travel[^1];
            Assert.True(open > 0.035 + 0.005, $"open, the pads are {open * 1000:F1} mm apart: the 35 mm can must fit");
            Assert.InRange(closed, -0.002, 0.002); // closed, they meet

            // The headset's twin opens its jaws to the same gap (docs/adr/0034 amendment 10): the table is the twin's own.
            JsonNode doc = JsonNode.Parse(File.ReadAllText(Path.Combine(Root(), "robot", "jetrover", "urdf", "meshes", "visuals.json")))!;
            Assert.Equal((double)doc["jawOpenGapMeters"]!, (double)doc["jawCadGapMeters"]! - 2 * travel[0], 3);
            Assert.InRange((double)doc["jawOpenDegrees"]!, 1.0, 80.0);
        }

        [Fact]
        public void EveryVisual_IsAUrdfLinksMesh_WithTheTrianglesItsManifestSays()
        {
            string dir = Path.Combine(Root(), "robot", "jetrover", "urdf");
            string manifest = Path.Combine(dir, "meshes", "visuals.json");
            Assert.True(File.Exists(manifest), $"{manifest} is missing: just jetrover-visuals");
            JsonNode doc = JsonNode.Parse(File.ReadAllText(manifest))!;
            XDocument urdf = XDocument.Parse(JetRoverSimModel.Urdf(Profile(), JetRoverCollisionModel.Instance, Plant(Profile()), Visuals(), Pads(), Travel()));
            var links = urdf.Root!.Elements("link").Select(l => (string)l.Attribute("name")!).ToHashSet();
            foreach (JsonNode? v in doc["visuals"]!.AsArray())
            {
                string link = (string)v!["link"]!, file = (string)v["file"]!;
                Assert.Contains(link, links);
                string obj = Path.Combine(dir, file);
                Assert.True(File.Exists(obj), obj);
                string[] lines = File.ReadAllLines(obj);
                Assert.Equal((int)v["triangles"]!, lines.Count(x => x.StartsWith("f ", StringComparison.Ordinal)));
                Assert.DoesNotContain(lines, x => x.StartsWith("version https://git-lfs", StringComparison.Ordinal)); // never an LFS pointer
            }

            // Each link with meshes is drawn by them, not by its boxes.
            Assert.All(urdf.Root.Elements("link").Where(l => l.Elements("visual").Any(x => x.Element("geometry")!.Element("mesh") != null)),
                l => Assert.DoesNotContain(l.Elements("visual"), x => x.Element("geometry")!.Element("box") != null));
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
                ? JetRoverSimModel.Urdf(profile, JetRoverCollisionModel.Instance, Plant(profile), Visuals(), Pads(), Travel())
                : JetRoverSimModel.SimJson(profile, Plant(profile), Travel());
            string path = Path.Combine(Root(), "robot", "jetrover", "urdf", file);
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1") File.WriteAllText(path, expected, new UTF8Encoding(false));
            Assert.True(File.Exists(path), $"{path} is missing: TELEOP_WRITE_GOLDEN=1 dotnet test Teleop.RobotHost.Tests --filter JetRoverSimModelTests");
            Assert.Equal(expected, File.ReadAllText(path).Replace("\r\n", "\n"));
        }

        [Fact]
        public void TheUrdfsChain_PutsTheWristWhereTheIkDoes()
        {
            RobotArmProfile profile = Profile();
            XDocument urdf = XDocument.Parse(JetRoverSimModel.Urdf(profile, JetRoverCollisionModel.Instance, Plant(profile), Visuals(), Pads(), Travel()));
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
            XDocument urdf = XDocument.Parse(JetRoverSimModel.Urdf(profile, JetRoverCollisionModel.Instance, Plant(profile), Visuals(), Pads(), Travel()));
            double arm = urdf.Root!.Elements("link").Where(l => (string)l.Attribute("name")! != "base_link")
                .Sum(l => (double)l.Element("inertial")!.Element("mass")!.Attribute("value")!);
            Assert.InRange(arm, 0.2, 1.5); // a 5-servo arm of 13 cm links

            JsonNode sim = JsonNode.Parse(JetRoverSimModel.SimJson(profile, Plant(profile), Travel()))!;
            var motors = sim["motors"]!.AsArray();
            Assert.Equal(new[] { 1, 2, 3, 4, 5, 10 }, motors.Select(m => (int)m!["motorId"]!).ToArray());
            Assert.True((bool)motors.Single(m => (int)m!["motorId"]! == 10)!["feedback"]!); // the real driver reads it (FEEDBACK_MOTOR_IDS)
            Assert.Equal(0.12217305, (double)motors.Single(m => (int)m!["motorId"]! == 2)!["zeroOffsetRadians"]!, 6);
        }

        private static Vector3 Vec(string s)
        {
            float[] v = s.Split(' ').Select(x => float.Parse(x, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
            return new Vector3(v[0], v[1], v[2]);
        }
    }
}
