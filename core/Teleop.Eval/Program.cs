// core/Teleop.Eval/Program.cs
using System;
using System.IO;
using Teleop.Eval.BuildProfile;
using Teleop.Eval.ClockSyncCheck;
using Teleop.Eval.MoveArm;
using Teleop.Eval.Sweep;
using Teleop.Eval.Tooling;
using Teleop.Eval.Verification;

namespace Teleop.Eval
{
    public static class Program
    {
        public static int Main(string[] args)
        {
            string cmd = args.Length > 0 ? args[0] : "help";
            switch (cmd)
            {
                case "gen-golden":
                    return RunGenGolden(args);
                case "gen-trace":
                    return RunGenTrace(args);
                case "verify":
                    return VerifyCommand.Run();
                case "audit":
                    return AuditCommand.Run();
                case "sweep":
                    return SweepCommand.Run(args);
                case "clocksync-check":
                    return ClockSyncCheckCommand.Run(args);
                case "move-arm":
                    return MoveArmCommand.Run(args);
                case "build-profile":
                    return BuildProfileCommand.Run(args);
                case "replay":
                case "compare":
                    Console.Error.WriteLine($"'{cmd}' is NOT IMPLEMENTED. " +
                        "Do not treat this as a passing check. See docs/setup.md Phase 3.");
                    return 70;   // EX_SOFTWARE
                default:
                    Console.Error.WriteLine(
                        "usage: verify | audit | sweep | replay | compare | gen-golden | gen-trace | " +
                        "clocksync-check | move-arm | build-profile");
                    return 64;   // EX_USAGE
            }
        }

        // Not one of the five documented subcommands -- a small, explicit scope addition so the
        // golden .tlog fixture is generated deterministically rather than hand-authored (which
        // risks CRLF/encoding corruption when edited from WSL). Never invoked by verify/audit
        // themselves; a one-off tool run by a human, output committed by hand.
        private static int RunGenGolden(string[] args)
        {
            string outputPath = args.Length > 1 ? args[1] : Path.Combine("testdata", "golden", "basic-session.tlog");
            string fullPath = Path.GetFullPath(outputPath);

            string? directory = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(directory))
            {
                Directory.CreateDirectory(directory);
            }

            GoldenSessionBuilder.Build(fullPath);

            Console.WriteLine($"Wrote golden session to {fullPath}");
            return 0;
        }

        // Not one of the five documented subcommands, same reasoning as gen-golden: the
        // synthetic-burst network trace is generated deterministically and committed, never
        // hand-authored.
        private static int RunGenTrace(string[] args)
        {
            string outputPath = args.Length > 1 ? args[1] : Path.Combine("testdata", "traces", "synthetic-burst.trace");
            string fullPath = Path.GetFullPath(outputPath);

            string? directory = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(directory))
            {
                Directory.CreateDirectory(directory);
            }

            SyntheticTraceBuilder.Build(fullPath);

            Console.WriteLine($"Wrote synthetic trace to {fullPath}");
            return 0;
        }
    }
}