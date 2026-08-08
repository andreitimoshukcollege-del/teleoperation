using System;
using System.Net;

namespace Teleop.RobotHost
{
    /// <summary>Parsed command-line arguments for <see cref="Program"/>.</summary>
    internal readonly struct RobotHostArgs
    {
        public readonly int LocalPort;
        public readonly IPAddress RemoteHost;
        public readonly int RemotePort;
        public readonly string RelaySocketPath;
        public readonly string LocalRelaySocketPath;
        public readonly float? MaxDirectionMagnitude;

        /// <summary>Always resolved by <see cref="TryParse"/> -- <see cref="DefaultLowerArmMaxPulse"/>
        /// unless <c>--lower-arm-max-pulse</c> was passed. Never left unset: see that default's
        /// own doc comment for why this must be safe-by-default, not opt-in.</summary>
        public readonly int LowerArmMaxPulse;

        // 2026-08-08: the lower arm collided with the robot's own base plate at a real physical
        // target, straining the servo against the obstruction until manually corrected.
        // Iteratively calibrated against the real robot afterward, with a human confirming
        // clearance at each step -- see robot/README.md for the full process, including a real
        // timing bug found partway through (ServoController's 300ms per-move cooldown vs. a
        // faster sender means a final small correction can be silently dropped, so the first
        // calibration pass was invalidated by its own send rate and had to be redone at
        // --rate-hz below ~3.3). Safe-by-default rather than opt-in: forgetting to pass an
        // override flag must not silently re-expose a hazard that has already actually happened.
        // Override with --lower-arm-max-pulse only after confirming the mechanical setup has
        // genuinely changed (e.g. the plate is no longer in the way).
        public const int DefaultLowerArmMaxPulse = 50;

        public const string Usage =
            "Usage: Teleop.RobotHost --local-port <port> --remote-host <ip> --remote-port <port> " +
            "--relay-socket <path> --local-relay-socket <path> [--max-direction-magnitude <n>] " +
            "[--lower-arm-max-pulse <n>]\n" +
            "  --max-direction-magnitude overrides JetRoverPlantConfig.Default's clamp (5) on how far\n" +
            "  a single accepted command may move a joint's belief -- lower it (e.g. 1-2) for a\n" +
            "  visibly slower, gentler arm; omit it to keep the default.\n" +
            "  --lower-arm-max-pulse overrides the default (50) hard limit on the lower arm's pulse,\n" +
            "  in place since a real collision with the robot's own base plate --\n" +
            "  see JetRoverPlantConfig.LowerArmMaxPulse's doc comment. Only override this after\n" +
            "  confirming the mechanical setup has actually changed.";

        private RobotHostArgs(
            int localPort, IPAddress remoteHost, int remotePort,
            string relaySocketPath, string localRelaySocketPath, float? maxDirectionMagnitude,
            int lowerArmMaxPulse)
        {
            LocalPort = localPort;
            RemoteHost = remoteHost;
            RemotePort = remotePort;
            RelaySocketPath = relaySocketPath;
            LocalRelaySocketPath = localRelaySocketPath;
            MaxDirectionMagnitude = maxDirectionMagnitude;
            LowerArmMaxPulse = lowerArmMaxPulse;
        }

        public static RobotHostArgs? TryParse(string[] args, out string? error)
        {
            int? localPort = null;
            IPAddress? remoteHost = null;
            int? remotePort = null;
            string? relaySocketPath = null;
            string? localRelaySocketPath = null;
            float? maxDirectionMagnitude = null;
            int? lowerArmMaxPulse = null;

            for (int i = 0; i < args.Length; i++)
            {
                switch (args[i])
                {
                    case "--local-port" when i + 1 < args.Length:
                        localPort = ParseIntOrNull(args[++i]);
                        break;
                    case "--remote-host" when i + 1 < args.Length:
                        IPAddress.TryParse(args[++i], out remoteHost);
                        break;
                    case "--remote-port" when i + 1 < args.Length:
                        remotePort = ParseIntOrNull(args[++i]);
                        break;
                    case "--relay-socket" when i + 1 < args.Length:
                        relaySocketPath = args[++i];
                        break;
                    case "--local-relay-socket" when i + 1 < args.Length:
                        localRelaySocketPath = args[++i];
                        break;
                    case "--max-direction-magnitude" when i + 1 < args.Length:
                        if (float.TryParse(args[++i], out float parsedMagnitude))
                        {
                            maxDirectionMagnitude = parsedMagnitude;
                        }
                        break;
                    case "--lower-arm-max-pulse" when i + 1 < args.Length:
                        lowerArmMaxPulse = ParseIntOrNull(args[++i]);
                        break;
                }
            }

            if (localPort is null || remoteHost is null || remotePort is null ||
                relaySocketPath is null || localRelaySocketPath is null ||
                maxDirectionMagnitude is <= 0f || lowerArmMaxPulse is <= 0)
            {
                error = "Missing or invalid required argument (--max-direction-magnitude and " +
                    "--lower-arm-max-pulse must be positive).";
                return null;
            }

            error = null;
            return new RobotHostArgs(
                localPort.Value, remoteHost, remotePort.Value, relaySocketPath, localRelaySocketPath,
                maxDirectionMagnitude, lowerArmMaxPulse ?? DefaultLowerArmMaxPulse);
        }

        private static int? ParseIntOrNull(string s) => int.TryParse(s, out int value) ? value : null;
    }
}
