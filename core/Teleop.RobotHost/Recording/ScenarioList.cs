using System;
using System.Collections.Generic;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Recording
{
    /// <summary>One scenario of <c>scenarios.txt</c>: its name (which names its folder) and what it asks of the operator.</summary>
    public sealed class Scenario
    {
        public Scenario(string name, string description)
        {
            Name = name;
            Description = description;
        }

        public string Name { get; }
        public string Description { get; }
    }

    /// <summary>
    /// The scenarios the headset may record (docs/adr/0033): <c>scenarios.txt</c> in the scenarios folder, which the person
    /// recording edits. One scenario a line, <c>name</c> or <c>name | what to do</c>; <c>#</c> starts a comment; blank lines
    /// are skipped. A name is 1 to 24 of letters, digits, '.', '_' and '-' (<see cref="RecordCodec.IsValidName"/>), not
    /// starting with '_' or '.'. A line that breaks a rule, a repeated name, and lines past the 16th are skipped, each with a
    /// problem the log shows. An empty or missing file is the one scenario <see cref="DefaultName"/>.
    /// </summary>
    public static class ScenarioList
    {
        public const string FileName = "scenarios.txt";
        public const string DefaultName = "free-play";

        /// <summary>What a fresh scenarios folder starts with.</summary>
        public const string Template =
            "# The scenarios the headset's RECORD button can record (docs/adr/0033). One a line:\n" +
            "#   name | what the operator does\n" +
            "# A name is up to 24 letters, digits, '.', '_' or '-', and names the folder its takes go in. At most 16.\n" +
            "# Edit this file at any time: the headset shows the new list within a few seconds.\n" +
            "free-play | anything: move the arms and the objects freely\n" +
            "knock-over | knock each object over with either arm\n" +
            "push-to-edge | push an object to the table's far edge without tipping it\n";

        public static IReadOnlyList<Scenario> Parse(string? text, List<string> problems)
        {
            if (problems == null) throw new ArgumentNullException(nameof(problems));
            var list = new List<Scenario>();
            if (text != null)
            {
                string[] lines = text.Replace("\r\n", "\n").Split('\n');
                for (int i = 0; i < lines.Length; i++)
                {
                    string line = lines[i];
                    int hash = line.IndexOf('#');
                    if (hash >= 0) line = line.Substring(0, hash);
                    line = line.Trim();
                    if (line.Length == 0) continue;
                    int bar = line.IndexOf('|');
                    string name = (bar >= 0 ? line.Substring(0, bar) : line).Trim();
                    string description = bar >= 0 ? line.Substring(bar + 1).Trim() : "";
                    if (!RecordCodec.IsValidName(name))
                    {
                        problems.Add($"line {i + 1}: '{name}' is not a scenario name (1 to {RecordCodec.MaxNameLength} letters, digits, '.', '_', '-'; not starting with '_' or '.')");
                        continue;
                    }

                    if (list.Exists(s => string.Equals(s.Name, name, StringComparison.OrdinalIgnoreCase)))
                    {
                        problems.Add($"line {i + 1}: '{name}' is already in the list");
                        continue;
                    }

                    if (list.Count == RecordCodec.MaxScenarios)
                    {
                        problems.Add($"line {i + 1}: more than {RecordCodec.MaxScenarios} scenarios; '{name}' and the rest are left out");
                        break;
                    }

                    list.Add(new Scenario(name, description));
                }
            }

            if (list.Count == 0) list.Add(new Scenario(DefaultName, "anything"));
            return list;
        }
    }
}
