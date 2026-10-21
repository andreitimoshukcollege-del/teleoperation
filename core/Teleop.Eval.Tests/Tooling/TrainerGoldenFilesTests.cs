using System;
using System.IO;
using Teleop.Eval.Tooling;

namespace Teleop.Eval.Tests.Tooling
{
    /// <summary>
    /// The trainer's golden files (trainer/contract/golden, docs/adr/0021 decision 5) are what Core
    /// produces now. A change to <c>SeededRng</c>, the impairments, the profile catalog,
    /// <c>ClockSync</c> or <c>relay-replay</c> that alters any of them fails here, so it cannot reach
    /// the trainer unnoticed: regenerate them (<c>gen-trainer-golden</c>) and fix the trainer's copies
    /// in the same change.
    /// </summary>
    public class TrainerGoldenFilesTests
    {
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
