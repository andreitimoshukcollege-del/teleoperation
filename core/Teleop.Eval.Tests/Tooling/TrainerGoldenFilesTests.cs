using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using Teleop.Eval.Tooling;

namespace Teleop.Eval.Tests.Tooling
{
    /// <summary>
    /// The trainer's golden files (trainer/contract/golden, docs/adr/0021 decision 5) are what Core
    /// produces now. A change to <c>SeededRng</c>, the impairments, the profile catalog,
    /// <c>ClockSync</c>, <c>relay-replay</c> or <c>relay-replay-servo</c> that alters any of them fails
    /// here, so it cannot reach the trainer unnoticed: regenerate them (<c>gen-trainer-golden</c>) and fix
    /// the trainer's copies in the same change.
    /// </summary>
    public class TrainerGoldenFilesTests
    {
        private static readonly string[] Expected =
        {
            "rng-v1.json", "impairments-v1.json", "clocksync-v1.json", "relay-replay-v1.json", "relay-replay-servo.json",
            "joint-shaper-v1.json", "franka-replay-v1.json",
        };

        [Fact]
        public void EveryGoldenFile_IsBuilt()
        {
            // TheCommittedGoldenFiles_AreWhatCoreProduces only checks what the builder writes; a file it
            // stopped writing would go stale unchecked.
            string trace = Path.Combine(RepoRoot(), "core", "testdata", "traces", "synthetic-burst.trace");
            Assert.Equal(Expected, TrainerGoldenBuilder.BuildAll(trace).Select(f => f.Name).ToArray());
        }

        [Fact]
        public void TheServoGoldenFile_AtNeutralParameters_IsRelayReplaysCallForCall_AndTheOtherSetsDiffer()
        {
            string trace = Path.Combine(RepoRoot(), "core", "testdata", "traces", "synthetic-burst.trace");
            var files = TrainerGoldenBuilder.BuildAll(trace);
            using JsonDocument relay = JsonDocument.Parse(files.Single(f => f.Name == "relay-replay-v1.json").Content);
            using JsonDocument servo = JsonDocument.Parse(files.Single(f => f.Name == "relay-replay-servo.json").Content);
            var servoCases = servo.RootElement.GetProperty("cases").EnumerateArray()
                .ToDictionary(c => c.GetProperty("name").GetString()!, c => c);
            Assert.Equal(9, servoCases.Count);

            // The neutral set has no cap: its speed is +infinity, exactly.
            JsonElement neutralSpeed = servoCases["neutral/headset-blind"].GetProperty("config").GetProperty("servoMaxSpeedRadPerSec");
            Assert.Equal(BitConverter.SingleToInt32Bits(float.PositiveInfinity), neutralSpeed[1].GetInt32());

            foreach (JsonElement relayCase in relay.RootElement.GetProperty("cases").EnumerateArray())
            {
                string name = relayCase.GetProperty("name").GetString()!;
                JsonElement neutral = servoCases["neutral/" + name];
                Assert.True(relayCase.GetProperty("calls").GetRawText() == neutral.GetProperty("calls").GetRawText(),
                    $"neutral/{name} is not relay-replay's {name}, call for call");
                Assert.Equal(relayCase.GetProperty("gate").GetRawText(), neutral.GetProperty("gate").GetRawText());

                var relayCalls = relayCase.GetProperty("calls").EnumerateArray().ToList();
                foreach (string set in new[] { "defaults", "custom" })
                {
                    var calls = servoCases[set + "/" + name].GetProperty("calls").EnumerateArray().ToList();
                    Assert.Equal(relayCalls.Count, calls.Count);
                    int differing = 0;
                    for (int i = 0; i < calls.Count; i++)
                    {
                        string op = relayCalls[i].GetProperty("op").GetString()!;
                        Assert.Equal(op, calls[i].GetProperty("op").GetString());
                        if (op != "predict")
                        {
                            // The same inputs, in the same order.
                            Assert.Equal(relayCalls[i].GetRawText(), calls[i].GetRawText());
                        }
                        else if (relayCalls[i].GetProperty("out").GetRawText() != calls[i].GetProperty("out").GetRawText())
                        {
                            differing++;
                        }
                    }

                    Assert.True(differing > calls.Count / 4, $"{set}/{name}: only {differing} of {calls.Count} calls predict differently from relay-replay");
                }
            }
        }

        [Fact]
        public void TheCommittedGoldenFiles_AreWhatCoreProduces()
        {
            string root = RepoRoot();
            string golden = Path.Combine(root, "trainer", "contract", "golden");
            string trace = Path.Combine(root, "core", "testdata", "traces", "synthetic-burst.trace");
            foreach ((string name, string content) in TrainerGoldenBuilder.BuildAll(trace))
            {
                string path = Path.Combine(golden, name);
                Assert.True(File.Exists(path), $"{path} is missing: run 'dotnet run --project Teleop.Eval -- gen-trainer-golden' from core/");
                string committed = File.ReadAllText(path).Replace("\r\n", "\n");
                Assert.True(committed == content,
                    $"{name} no longer matches what Core produces. If the Core change is intended, run " +
                    "'dotnet run --project Teleop.Eval -- gen-trainer-golden' from core/ and update the trainer's copies to match.");
            }
        }

        [Fact]
        public void TheIsaacBridgesShaperGolden_IsTheTrainers()
        {
            // The Isaac bridge's shaper copy is pinned to the same golden (docs/adr/0031 decision 2); robot/ reads nothing from trainer/.
            string root = RepoRoot();
            string trace = Path.Combine(root, "core", "testdata", "traces", "synthetic-burst.trace");
            string shaper = TrainerGoldenBuilder.BuildAll(trace).Single(f => f.Name == "joint-shaper-v1.json").Content;
            string copy = Path.Combine(root, "robot", "isaac", "golden", "joint-shaper-v1.json");
            Assert.True(File.Exists(copy), $"{copy} is missing: just gen-trainer-golden copies it there");
            Assert.True(File.ReadAllText(copy).Replace("\r\n", "\n") == shaper,
                "robot/isaac/golden/joint-shaper-v1.json is not what Core produces: run just gen-trainer-golden, and fix robot/isaac/teleop_isaac/shaper.py to match");
        }

        [Fact]
        public void TheTrainersTraceCopy_IsTheCommittedTrace()
        {
            string root = RepoRoot();
            byte[] original = File.ReadAllBytes(Path.Combine(root, "core", "testdata", "traces", "synthetic-burst.trace"));
            byte[] copy = File.ReadAllBytes(Path.Combine(root, "trainer", "contract", "traces", "synthetic-burst.trace"));
            Assert.Equal(original, copy);
        }

        [Fact]
        public void TheGoldenFiles_AreDeterministic()
        {
            string trace = Path.Combine(RepoRoot(), "core", "testdata", "traces", "synthetic-burst.trace");
            var a = TrainerGoldenBuilder.BuildAll(trace);
            var b = TrainerGoldenBuilder.BuildAll(trace);
            Assert.Equal(a.Count, b.Count);
            for (int i = 0; i < a.Count; i++) Assert.Equal(a[i], b[i]);
        }

        /// <summary>Walks up from the test binary (MSBuild output goes to the repo's build/) to the repo root.</summary>
        private static string RepoRoot()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                if (Directory.Exists(Path.Combine(dir.FullName, "trainer", "contract")) &&
                    Directory.Exists(Path.Combine(dir.FullName, "core", "testdata")))
                {
                    return dir.FullName;
                }
            }

            throw new InvalidOperationException($"no repo root (trainer/contract and core/testdata) above {AppContext.BaseDirectory}");
        }
    }
}
