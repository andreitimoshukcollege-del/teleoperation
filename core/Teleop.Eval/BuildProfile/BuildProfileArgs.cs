namespace Teleop.Eval.BuildProfile
{
    /// <summary>Parsed command-line arguments for <see cref="BuildProfileCommand"/>.</summary>
    internal readonly struct BuildProfileArgs
    {
        /// <summary>Defaults to <c>RobotProfiles/&lt;name&gt;.json</c> (relative to the current directory) once the wizard has asked for a name -- see <see cref="BuildProfileCommand.Run(BuildProfileArgs, System.IO.TextReader, System.IO.TextWriter)"/>.</summary>
        public readonly string? OutputPath;

        public readonly bool Force;

        public const string Usage =
            "Usage: Teleop.Eval build-profile [--output <path>] [--force]\n" +
            "  Interactively prompts for a robot's topology (rotating base, link lengths, wrist\n" +
            "  joint count, gripper) and writes a validated RobotArmProfile JSON file --\n" +
            "  see docs/adr/0011-generic-robot-arm-profiles.md. --output overrides the default\n" +
            "  location (RobotProfiles/<name>.json). --force allows overwriting an existing file.";

        private BuildProfileArgs(string? outputPath, bool force)
        {
            OutputPath = outputPath;
            Force = force;
        }

        public static BuildProfileArgs TryParse(string[] args)
        {
            string? outputPath = null;
            bool force = false;

            for (int i = 0; i < args.Length; i++)
            {
                switch (args[i])
                {
                    case "--output" when i + 1 < args.Length:
                        outputPath = args[++i];
                        break;
                    case "--force":
                        force = true;
                        break;
                }
            }

            return new BuildProfileArgs(outputPath, force);
        }
    }
}
