using System.Diagnostics;
using System.Text.Json;
using System.Text.RegularExpressions;

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
                var startInfo = new ProcessStartInfo("git")
                {
                    RedirectStandardOutput = true,
                    UseShellExecute = false,
                };

                string? gitDirOverride = TryResolveWindowsGitDir(Directory.GetCurrentDirectory());
                if (gitDirOverride != null)
                {
                    startInfo.ArgumentList.Add($"--git-dir={gitDirOverride}");
                }
                startInfo.ArgumentList.Add("rev-parse");
                startInfo.ArgumentList.Add("HEAD");

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

        private static readonly Regex WslMountPathPattern = new Regex(@"^/mnt/([a-zA-Z])/(.*)$", RegexOptions.Compiled);

        /// <summary>
        /// This repo's dev workflow (root CLAUDE.md's Environment section) is a WSL-native git
        /// creating worktrees, but this tool runs as a Windows process (<c>dotnet run</c>).
        /// A git worktree's ".git" is a text file, not a directory, containing
        /// "gitdir: &lt;path to the real one under the main checkout's .git/worktrees/&gt;" --
        /// written by WSL git as a POSIX path (e.g. "/mnt/c/Users/..."), which Windows git.exe
        /// cannot resolve on its own (confirmed: it fails with "not a git repository", not a
        /// silent misresolution). Detects exactly that case and returns an explicit,
        /// Windows-usable <c>--git-dir</c> override; null means "let git resolve normally,"
        /// which covers the ordinary non-worktree case and non-WSL-created worktrees alike.
        /// </summary>
        private static string? TryResolveWindowsGitDir(string startDirectory)
        {
            string? directory = startDirectory;
            while (directory != null)
            {
                string gitPath = Path.Combine(directory, ".git");
                if (File.Exists(gitPath))
                {
                    string content = File.ReadAllText(gitPath).Trim();
                    const string prefix = "gitdir: ";
                    if (!content.StartsWith(prefix, StringComparison.Ordinal))
                    {
                        return null;
                    }

                    string pointer = content.Substring(prefix.Length).Trim();
                    Match match = WslMountPathPattern.Match(pointer);
                    if (!match.Success)
                    {
                        return null;
                    }

                    string drive = match.Groups[1].Value.ToUpperInvariant();
                    string rest = match.Groups[2].Value.Replace('/', '\\');
                    return $"{drive}:\\{rest}";
                }

                if (Directory.Exists(gitPath))
                {
                    return null;
                }

                directory = Path.GetDirectoryName(directory);
            }

            return null;
        }
    }
}
