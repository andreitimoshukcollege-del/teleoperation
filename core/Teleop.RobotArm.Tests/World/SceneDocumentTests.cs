using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Teleop.RobotArm.Tests.Kinematics;
using Teleop.RobotArm.World;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>The scene file (docs/adr/0031 amendment 2): what it accepts, what it refuses, and the copy Isaac is held to.</summary>
    public class SceneDocumentTests
    {
        private static readonly JsonSerializerOptions Options = new JsonSerializerOptions { IncludeFields = true };

        internal static string ScenePath(string name = "fr3-duo-bench") =>
            Path.Combine(SerialChainKinematicsTests.RepoRoot(), "core", "Scenes", name + ".json");

        internal static WorldScene LoadShipped() =>
            JsonSerializer.Deserialize<SceneDocument>(File.ReadAllText(ScenePath()), Options)!.ToScene();

        [Fact]
        public void TheVectors_AreAcceptedOrRefusedAsTheyExpect()
        {
            // core/Scenes/testdata/vectors.json: robot/isaac/tests/test_scene.py reads the same cases, so both readers agree.
            string path = Path.Combine(SerialChainKinematicsTests.RepoRoot(), "core", "Scenes", "testdata", "vectors.json");
            using JsonDocument vectors = JsonDocument.Parse(File.ReadAllText(path));
            int n = 0;
            foreach (JsonElement c in vectors.RootElement.GetProperty("cases").EnumerateArray())
            {
                string name = c.GetProperty("name").GetString()!;
                string expect = c.GetProperty("expect").GetString()!;
                SceneDocument document = JsonSerializer.Deserialize<SceneDocument>(c.GetProperty("scene").GetRawText(), Options)!;
                if (expect == "ok")
                {
                    WorldScene scene = document.ToScene();
                    Assert.True(scene.Objects.Length > 0, name);
                }
                else
                {
                    FormatException e = Assert.Throws<FormatException>(() => document.ToScene());
                    Assert.True(e.Message.StartsWith(expect + ":", StringComparison.Ordinal), $"{name}: expected '{expect}:', got '{e.Message}'");
                }

                n++;
            }

            Assert.True(n >= 30, $"only {n} vectors");
        }

        [Fact]
        public void TheShippedScene_IsTheBenchAndATableInFrontOfTheRobot()
        {
            WorldScene s = LoadShipped();
            Assert.Equal("fr3-duo-bench", s.Name);
            Assert.Equal(SerialChainKinematicsTests.LoadFr3Duo().Name, s.RobotProfile);
            Assert.NotNull(s.Bench);
            // The VR bench: 0.6 m deep, the robot 0.4 m in from its near edge (just ahead of the operator, docs/adr/0031
            // amendment 4), its top at the robot's base.
            Assert.Equal(-0.4, s.Bench!.XMin, 9);
            Assert.Equal(0.2, s.Bench.XMax, 9);
            Assert.Equal(0.0, s.Bench.Top, 9);
            Assert.Equal(0.2, s.Table.XMin, 9);
            Assert.Equal(0.9, s.Table.XMax, 9);
            Assert.Equal(0.0, s.Table.Top, 9);
            Assert.Equal(SurfaceDisplay.Builtin, s.Bench.Display);
            Assert.Equal(SurfaceDisplay.Show, s.Table.Display);
            Assert.Equal(4, s.Objects.Length);
            Assert.All(s.Objects, o => Assert.True(o.OperatorMovable));
            // Every object can be knocked over (the user, 2026-11-03), and lands on the floor if it falls off the table.
            Assert.All(s.Objects, o => Assert.True(o.Movable && o.Mass > 0 && o.Mass <= 1.0, o.Name));
            Assert.True(s.Surfaces.Single(x => x.Id == "floor").Collides);
        }

        [Fact]
        public void AVersion1File_StillReads_AndItsObjectsAreFixed()
        {
            string text = File.ReadAllText(ScenePath()).Replace("\"version\": 2", "\"version\": 1")
                .Replace("\"kind\": \"movable\", \"mass\": 0.4,", "\"kind\": \"fixed\",").Replace("\"kind\": \"movable\", \"mass\": 0.6,", "\"kind\": \"fixed\",")
                .Replace("\"kind\": \"movable\", \"mass\": 0.35,", "\"kind\": \"fixed\",");
            WorldScene s = JsonSerializer.Deserialize<SceneDocument>(text, Options)!.ToScene();
            Assert.All(s.Objects, o => Assert.False(o.Movable));
            Assert.All(s.Objects, o => Assert.Equal(0.0, o.Mass));
        }

        [Fact]
        public void TheIsaacGoldenFile_IsWhatThisReaderMakesOfTheScene()
        {
            string expected = GoldenJson();
            string path = Path.Combine(SerialChainKinematicsTests.RepoRoot(), "robot", "isaac", "golden", "scene-fr3-duo-bench.json");
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, expected, new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-isaac-goldens`");
            GoldenFile.AssertSame(File.ReadAllText(path), expected);
        }

        /// <summary>The canonical parse: what Python's reader (robot/isaac/teleop_isaac/scene.py) must make of the same bytes.</summary>
        internal static string GoldenJson()
        {
            byte[] bytes = File.ReadAllBytes(ScenePath());
            WorldScene s = LoadShipped();
            var golden = new Dictionary<string, object>
            {
                ["about"] = "The canonical parse of core/Scenes/fr3-duo-bench.json (docs/adr/0031 amendment 2), written by SceneDocumentTests.",
                ["scene"] = s.Name,
                ["sha256"] = Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant(),
                ["table"] = new[] { s.Table.XMin, s.Table.XMax, s.Table.YMin, s.Table.YMax, s.Table.Top },
                ["region"] = new[] { s.RegionXMin, s.RegionXMax, s.RegionYMin, s.RegionYMax, s.Clearance },
                ["objects"] = s.Objects.Select(o => new Dictionary<string, object>
                {
                    ["id"] = (int)o.Id,
                    ["name"] = o.Name,
                    ["shape"] = o.Shape.ToString().ToLowerInvariant(),
                    ["size"] = new[] { o.SizeX, o.SizeY, o.SizeZ },
                    ["nominal"] = new[] { o.Nominal.X, o.Nominal.Y, o.Nominal.Z, o.Nominal.Qw, o.Nominal.Qx, o.Nominal.Qy, o.Nominal.Qz },
                    ["footprintRadius"] = o.FootprintRadius,
                    ["movable"] = o.Movable,
                    ["mass"] = o.Mass,
                    ["jitter"] = new[] { o.JitterX, o.JitterY, o.JitterYaw },
                }).ToArray(),
            };
            return JsonSerializer.Serialize(golden, new JsonSerializerOptions { WriteIndented = true }).Replace("\r\n", "\n") + "\n";
        }
    }
}
