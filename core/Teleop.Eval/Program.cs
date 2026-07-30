// core/Teleop.Eval/Program.cs
using System;

namespace Teleop.Eval
{
    public static class Program
    {
        public static int Main(string[] args)
        {
            string cmd = args.Length > 0 ? args[0] : "help";
            switch (cmd)
            {
                case "verify":
                case "audit":
                case "sweep":
                case "replay":
                case "compare":
                    Console.Error.WriteLine($"'{cmd}' is NOT IMPLEMENTED. " +
                        "Do not treat this as a passing check. See docs/setup.md Phase 3.");
                    return 70;   // EX_SOFTWARE
                default:
                    Console.Error.WriteLine("usage: verify | audit | sweep | replay | compare");
                    return 64;   // EX_USAGE
            }
        }
    }
}