using System;
using System.IO;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.BuildProfile
{
    /// <summary>
    /// The <c>build-profile</c> verb (docs/adr/0011-generic-robot-arm-profiles.md): runs
    /// <see cref="ProfileBuilderWizard"/> against the real console and writes the resulting
    /// profile to <c>RobotProfiles/&lt;name&gt;.json</c> by convention (relative to the current
    /// directory -- normally invoked from <c>core/</c>, matching this project's other verbs), or
    /// to <c>--output</c>'s path when given.
    /// </summary>
    public static class BuildProfileCommand
    {
        public static int Run(string[] args) => Run(BuildProfileArgs.TryParse(args), Console.In, Console.Out);

        /// <summary>Testable entry point: <paramref name="input"/>/<paramref name="output"/> stand in for <see cref="Console"/> so a test can pipe a canned answer transcript and inspect what would have been printed.</summary>
        internal static int Run(BuildProfileArgs args, TextReader input, TextWriter output)
        {
            var wizard = new ProfileBuilderWizard(input, output);
            RobotArmProfile? built = wizard.Run();
            if (built is null)
            {
                output.WriteLine("Aborted -- no profile written.");
                return 1;
            }

            RobotArmProfile profile = built.Value;
            string outputPath = args.OutputPath ?? Path.Combine("RobotProfiles", $"{profile.Name}.json");

            if (File.Exists(outputPath) && !args.Force)
            {
                output.WriteLine($"Refusing to overwrite existing file '{outputPath}' without --force.");
                return 1;
            }

            RobotArmProfileJson.Save(outputPath, profile);
            output.WriteLine($"Wrote robot profile to {outputPath}");
            return 0;
        }
    }
}
