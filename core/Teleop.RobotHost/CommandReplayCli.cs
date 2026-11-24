using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Teleop.RobotArm.Types;
using Teleop.RobotHost.Plant;

namespace Teleop.RobotHost
{
    /// <summary>
    /// <c>Teleop.RobotHost --replay-commands [--host-check-onfail clamp|refuse] &lt;robothost.jlog&gt;...</c>: each recorded take's
    /// joint commands run back through the plant offline (<see cref="CommandReplay"/>, docs/adr/0020 amendment 2026-11-24).
    /// A take's <c>robot-profile.json</c> beside its log is its profile, else the JetRover's default. Prints the outcomes
    /// (recorded&gt;replayed), the clamps' alphas and the plant's time per command. Exits 1 when refuse mode does not reproduce
    /// every recorded outcome, when a clamp ever failed its re-check, or when a file cannot be read.
    /// </summary>
    internal static class CommandReplayCli
    {
        public static int Run(string[] args)
        {
            HostCheckOnFail onFail = HostCheckOnFail.Clamp;
            var files = new List<string>();
            for (int i = 1; i < args.Length; i++)
            {
                if (args[i] == "--host-check-onfail" && i + 1 < args.Length)
                {
                    string v = args[++i];
                    if (v == "clamp") onFail = HostCheckOnFail.Clamp;
                    else if (v == "refuse") onFail = HostCheckOnFail.Refuse;
                    else
                    {
                        Console.Error.WriteLine($"--host-check-onfail is clamp or refuse, not '{v}'");
                        return 2;
                    }
                }
                else files.Add(args[i]);
            }

            if (files.Count == 0)
            {
                Console.Error.WriteLine("usage: Teleop.RobotHost --replay-commands [--host-check-onfail clamp|refuse] <robothost.jlog>...");
                return 2;
            }

            var outcomes = new SortedDictionary<string, int>(StringComparer.Ordinal);
            var alphas = new List<float>();
            var checks = new List<int>();
            var micros = new Dictionary<char, List<double>>();
            int commands = 0, mismatches = 0, grid = 0, violations = 0, takesWithClamps = 0;
            foreach (string file in files)
            {
                if (!File.Exists(file))
                {
                    Console.Error.WriteLine($"{file}: not found");
                    return 1;
                }

                string profilePath = Path.Combine(Path.GetDirectoryName(Path.GetFullPath(file))!, "robot-profile.json");
                RobotArmProfile profile = File.Exists(profilePath) ? RobotArmProfileJson.Load(profilePath) : RobotArmProfile.JetRoverMeasuredDefault;
                GenericArmPlantConfig d = GenericArmPlantConfig.Default;
                var config = new GenericArmPlantConfig(profile, d.PulsePerRadian, d.PulsesPerSecond, d.StepSizePulses, d.MaxDirectionMagnitude,
                    d.ZeroPulse, d.MinPulse, d.MaxPulse, d.GripperOpenPulse, d.GripperClosedPulse);
                var replay = new CommandReplay(config, onFail);
                foreach (string line in File.ReadLines(file)) replay.Line(line);

                commands += replay.Commands;
                mismatches += replay.Mismatches;
                grid += replay.Plant.GridClamps;
                violations += replay.Plant.ClampInvariantViolations;
                alphas.AddRange(replay.Alphas);
                checks.AddRange(replay.ClampChecks);
                if (replay.Alphas.Count > 0) takesWithClamps++;
                foreach (KeyValuePair<string, int> kv in replay.Outcomes) outcomes[kv.Key] = outcomes.TryGetValue(kv.Key, out int c) ? c + kv.Value : kv.Value;
                foreach (KeyValuePair<char, List<double>> kv in replay.Micros)
                {
                    if (!micros.TryGetValue(kv.Key, out List<double>? list)) micros[kv.Key] = list = new List<double>();
                    list.AddRange(kv.Value);
                }

                Console.WriteLine($"{file}: {replay.Commands} commands, " +
                    string.Join(" ", replay.Outcomes.Select(kv => $"{kv.Key}:{kv.Value}")) +
                    (replay.Alphas.Count > 0 ? $", alpha p50 {Quantile(replay.Alphas.Select(a => (double)a).ToList(), 0.5):0.###}" : "") +
                    (replay.Mismatches > 0 ? $", differs at {string.Join(", ", replay.FirstMismatches)}" : ""));
            }

            Console.WriteLine();
            Console.WriteLine($"mode {(onFail == HostCheckOnFail.Clamp ? "clamp" : "refuse")}: {files.Count} take(s), {commands} commands");
            Console.WriteLine("outcomes (recorded>replayed): " + string.Join(" ", outcomes.Select(kv => $"{kv.Key}:{kv.Value}")));
            Console.WriteLine($"replayed differently from recorded: {mismatches}");
            if (alphas.Count > 0)
            {
                List<double> a = alphas.Select(x => (double)x).ToList();
                Console.WriteLine($"clamped: {alphas.Count} in {takesWithClamps} take(s), {grid} from the grid (non-monotone line); alpha " +
                    $"min {a.Min():0.###} p10 {Quantile(a, 0.1):0.###} p25 {Quantile(a, 0.25):0.###} p50 {Quantile(a, 0.5):0.###} " +
                    $"p75 {Quantile(a, 0.75):0.###} p90 {Quantile(a, 0.9):0.###} max {a.Max():0.###}");
                var bins = new int[10];
                foreach (float x in alphas) bins[Math.Min(9, (int)(x * 10))]++;
                Console.WriteLine("alpha histogram (tenths): " + string.Join(" ", bins.Select((n, i) => $"{i / 10.0:0.0}-{(i + 1) / 10.0:0.0}:{n}")));
            }

            if (checks.Count > 0)
            {
                List<double> c = checks.Select(x => (double)x).ToList();
                Console.WriteLine($"checks per clamp search: p50 {Quantile(c, 0.5)} p90 {Quantile(c, 0.9)} max {c.Max()} (bound {LineClamp.MaxChecks}, plus the command's own and the re-check)");
            }

            foreach (KeyValuePair<char, List<double>> kv in micros.OrderBy(kv => kv.Key))
            {
                Console.WriteLine($"plant time per {kv.Key} command, us: n {kv.Value.Count} p50 {Quantile(kv.Value, 0.5):0.#} p90 {Quantile(kv.Value, 0.9):0.#} " +
                    $"p99 {Quantile(kv.Value, 0.99):0.#} max {kv.Value.Max():0.#}");
            }

            Console.WriteLine($"clamp results that failed the re-check: {violations}");
            bool failed = violations > 0 || (onFail == HostCheckOnFail.Refuse && mismatches > 0);
            Console.WriteLine(failed ? "FAIL" : "OK");
            return failed ? 1 : 0;
        }

        private static double Quantile(List<double> values, double q)
        {
            if (values.Count == 0) return double.NaN;
            var sorted = values.OrderBy(v => v).ToList();
            double pos = q * (sorted.Count - 1);
            int lo = (int)Math.Floor(pos);
            int hi = Math.Min(sorted.Count - 1, lo + 1);
            return sorted[lo] + (sorted[hi] - sorted[lo]) * (pos - lo);
        }
    }
}
