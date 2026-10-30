using System.IO;
using System.Text.Json;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Tests.Kinematics;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>
    /// The headset's copies of the FR3 Duo's generated files (docs/adr/0029): Unity can only load what is under the
    /// project, so Bridge's Resources hold copies, and these tests hold the copies to the files they were made from.
    /// A regenerated profile, collision model or model that is not copied over fails here, not on the headset.
    /// </summary>
    public class BridgeResourcesTests
    {
        private static string Resources(string name) =>
            Path.Combine(SerialChainKinematicsTests.RepoRoot(), "unity", "Teleop.Bridge", "Runtime", "Resources", name);

        private static string Repo(params string[] parts) => Path.Combine(new[] { SerialChainKinematicsTests.RepoRoot() }.Concat(parts).ToArray());

        [Theory]
        [InlineData("fr3_duo_profile.json", "core/RobotProfiles/fr3-duo.json")]
        [InlineData("fr3_duo_collision.json", "core/RobotProfiles/fr3-duo.collision.json")]
        [InlineData("models/fr3-sim-1.bytes", "core/testdata/learned/fr3-sim-1.weights")]
        public void TheHeadsetsCopy_IsTheGeneratedFile(string resource, string source)
        {
            byte[] copy = File.ReadAllBytes(Resources(resource));
            byte[] original = File.ReadAllBytes(Repo(source.Split('/')));
            Assert.True(original.SequenceEqual(copy), $"unity/Teleop.Bridge/Runtime/Resources/{resource} is not {source}: copy it over");
        }

        [Fact]
        public void TheDefaultConfig_IsOneTheOperatorAccepts()
        {
            using JsonDocument doc = JsonDocument.Parse(File.ReadAllText(Resources("fr3_connection.json")));
            JsonElement c = doc.RootElement;
            var grab = new GrabGestureConfig
            {
                EngageBelowCurl = c.GetProperty("GrabEngageBelowCurl").GetSingle(),
                ReleaseAboveCurl = c.GetProperty("GrabReleaseAboveCurl").GetSingle(),
                PinchOpenMeters = c.GetProperty("PinchOpenMeters").GetSingle(),
                PinchClosedMeters = c.GetProperty("PinchClosedMeters").GetSingle(),
            };
            Assert.Null(grab.Validate());
            var options = new SerialChainTeleopOptions
            {
                PositionScale = c.GetProperty("PositionScale").GetDouble(),
                MaxStepRadians = c.GetProperty("MaxStepRadians").GetDouble(),
            };
            Assert.Null(options.Validate());
            Assert.InRange(options.PositionScale, 0.01, 2.0);
            Assert.InRange(options.MaxStepRadians, 0.001, 0.2);

            string active = c.GetProperty("JointPredictorActive").GetString()!;
            string[] labels = c.GetProperty("JointPredictorPresets").EnumerateArray().Select(p => p.GetProperty("Label").GetString()!).ToArray();
            Assert.Contains(active, labels);
            Assert.Contains("franka-replay", c.GetProperty("JointPredictorPresets").EnumerateArray().Select(p => p.GetProperty("Predictor").GetString()));
        }
    }
}
