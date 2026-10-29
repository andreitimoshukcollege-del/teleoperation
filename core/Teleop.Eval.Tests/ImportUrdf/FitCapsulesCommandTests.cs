using System.IO;
using System.Text.Json;
using System.Xml.Linq;
using Teleop.Eval.ImportUrdf;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.Tests.ImportUrdf
{
    /// <summary><c>fit-capsules</c> (docs/adr/0026 amendment): the committed collision model is what the fit makes of the vendored URDF.</summary>
    public class FitCapsulesCommandTests
    {
        private static string Root()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                if (Directory.Exists(Path.Combine(dir.FullName, "core", "RobotProfiles")) && Directory.Exists(Path.Combine(dir.FullName, "robot", "franka"))) return dir.FullName;
            }

            throw new InvalidOperationException("no repo root above " + AppContext.BaseDirectory);
        }

        private static string Urdf => Path.Combine(Root(), "robot", "franka", "fr3_duo", "urdf", "fr3_duo.urdf");
        private static string Profile => Path.Combine(Root(), "core", "RobotProfiles", "fr3-duo.json");
        private static string Committed => Path.Combine(Root(), "core", "RobotProfiles", "fr3-duo.collision.json");

        [Fact]
        public void RefittingTheVendoredUrdf_ReproducesTheCommittedModel_ByteForByte()
        {
            string output = Path.Combine(Path.GetTempPath(), $"fr3-collision-{Guid.NewGuid():N}.json");
            try
            {
                Assert.Equal(0, FitCapsulesCommand.Run(new[] { "fit-capsules", "--urdf", Urdf, "--profile", Profile, "--out", output, "--margin", "0.01" },
                    TextWriter.Null, TextWriter.Null));
                Assert.True(File.ReadAllBytes(Committed).AsSpan().SequenceEqual(File.ReadAllBytes(output)),
                    "core/RobotProfiles/fr3-duo.collision.json is not what fit-capsules makes of the vendored URDF; `just import-fr3-profile` refits it");
            }
            finally
            {
                File.Delete(output);
            }
        }

        [Fact]
        public void EveryCollisionMeshVertex_IsInsideItsLinksCapsule()
        {
            // The fit's promise, checked on the arm links whose meshes sit at their link's origin (link 0 to link 7).
            var doc = JsonSerializer.Deserialize<SerialChainCollisionDocument>(File.ReadAllText(Committed), new JsonSerializerOptions { IncludeFields = true })!;
            string meshes = Path.Combine(Path.GetDirectoryName(Urdf)!, "resources", "franka_description", "meshes", "robots", "fr3v2", "collision");
            for (int k = 0; k <= 7; k++)
            {
                ChainCapsuleDocument c = doc.capsules.Single(x => x.name == $"left_fr3v2_link{k}");
                var seg = new[] { c.a[0], c.a[1], c.a[2], c.b[0], c.b[1], c.b[2] };
                foreach (double[] v in FitCapsulesCommand.StlVertices(Path.Combine(meshes, $"link{k}.stl")))
                {
                    double d = Math.Sqrt(SerialChainCollision.SegmentDistanceSquared(seg, new[] { v[0], v[1], v[2], v[0], v[1], v[2] }));
                    Assert.True(d <= c.radius + 1e-9, $"link{k}: a vertex {d - c.radius} m outside");
                }
            }
        }

        [Fact]
        public void TheModelCoversBothArms_TheBody_AndDisablesOnlyWithinAnArmOrOnTheMount()
        {
            var doc = JsonSerializer.Deserialize<SerialChainCollisionDocument>(File.ReadAllText(Committed), new JsonSerializerOptions { IncludeFields = true })!;
            SerialChainCollisionModel model = doc.ToModel();
            Assert.Equal(11, model.Capsules.Count(c => c.Arm == 0));
            Assert.Equal(11, model.Capsules.Count(c => c.Arm == 1));
            Assert.Equal(2, model.Capsules.Count(c => c.Arm < 0));
            foreach ((int a, int b) in model.DisabledPairs())
            {
                ChainCapsule ca = model.Capsules[a], cb = model.Capsules[b];
                bool acrossArms = ca.Arm >= 0 && cb.Arm >= 0 && ca.Arm != cb.Arm;
                Assert.True(!acrossArms || (ca.Frame < 0 && cb.Frame < 0), $"{ca.Name} / {cb.Name}: only the two link 0s on the mount may be disabled across arms");
            }
        }
    }
}
