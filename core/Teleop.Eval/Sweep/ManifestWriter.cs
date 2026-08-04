using System.Diagnostics;
using System.Text.Json;

namespace Teleop.Eval.Sweep
{
    /// <summary>
    /// Writes <c>results/&lt;id&gt;/&lt;timestamp&gt;/manifest.json</c> (schema documented in
    /// <c>results/CLAUDE.md</c> -- no prior example existed anywhere in the repo before this).
    /// Resolves the git SHA itself by shelling out to <c>git rev-parse HEAD</c>: this is exactly
    /// the file I/O and process execution Core is forbidden from doing and a host is not, and
    /// doing it here keeps <c>dotnet run --project Teleop.Eval -- sweep &lt;yaml&gt;</c>
    /// self-sufficient rather than depending on a caller to have already captured the SHA.
    /// </summary>
    public static class ManifestWriter
    {
        public static void Write(string path, ExperimentConfig config, string configPath, string commandLine)
        {
            var manifest = new
            {
                experimentId = config.Id,
                gitSha = TryGetGitSha(),
                seeds = config.Seeds,
                predictors = config.Predictors,
                reconciler = config.Reconciler,
                networkProfiles = config.NetworkProfiles,
                trialSteps = config.TrialSteps,
                stepIntervalTicks = config.StepIntervalTicks,
                configPath,
                machine = Environment.MachineName,
                command = commandLine,
                generatedAtUtc = DateTime.UtcNow.ToString("o"),
            };

            string json = JsonSerializer.Serialize(manifest, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(path, json);
        }

        private static string TryGetGitSha()
        {
            try
            {
                var startInfo = new ProcessStartInfo("git", "rev-parse HEAD")
                {
                    RedirectStandardOutput = true,
                    UseShellExecute = false,
                };

                using Process? process = Process.Start(startInfo);
                if (process == null)
                {
                    return "unknown";
                }

                string output = process.StandardOutput.ReadToEnd().Trim();
                process.WaitForExit();
                return process.ExitCode == 0 && output.Length > 0 ? output : "unknown";
            }
            catch (Exception)
            {
                return "unknown";
            }
        }
    }
}
