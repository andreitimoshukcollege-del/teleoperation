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

        /// <summary>
        /// Optional second listener for pre-computed joint-angle commands
        /// (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md) -- null (the default)
        /// disables it entirely, so existing deployments/scripts that only know about
        /// <see cref="LocalPort"/> are completely unaffected unless someone explicitly opts in.
        /// Uplink-only: this channel never replies, since a caller sending joint targets already
        /// gets robot state feedback through its own separate, unmodified Cartesian
        /// <see cref="RemotePort"/> connection.
        /// </summary>
        public readonly int? JointLocalPort;

        /// <summary>
        /// Path to a <c>RobotArmProfile</c> JSON file (<c>core/RobotProfiles/*.json</c> convention,
        /// docs/adr/0011-generic-robot-arm-profiles.md) -- null (the default) falls back to
        /// <c>RobotArmProfile.JetRoverMeasuredDefault</c>, a compatibility convenience, not the
        /// recommended path going forward. Replaces the old <c>--lower-arm-min-pulse</c> flag: a
        /// per-joint safety floor is now part of the profile itself
        /// (<c>JointHardwareSpec.MinAngleRadians</c>) rather than a single CLI override hardcoded
        /// to one specific joint by name.
        /// </summary>
        public readonly string? ProfilePath;

        public const string Usage =
            "Usage: Teleop.RobotHost --local-port <port> --remote-host <ip> --remote-port <port> " +
            "--relay-socket <path> --local-relay-socket <path> [--max-direction-magnitude <n>] " +
            "[--joint-local-port <port>] [--profile-path <path>]\n" +
            "  --max-direction-magnitude overrides GenericArmPlantConfig.Default's clamp (5) on how far\n" +
            "  a single accepted command may move a joint's belief -- lower it (e.g. 1-2) for a\n" +
            "  visibly slower, gentler arm; omit it to keep the default.\n" +
            "  --joint-local-port opens a second, optional listener for pre-computed joint-angle\n" +
            "  commands (GenericArmPlant.CommandJointAngles) instead of the default Cartesian-target\n" +
            "  path -- see docs/adr/0009-jetrover-operator-side-inverse-kinematics.md. Omit it to\n" +
            "  run exactly as before, with only the Cartesian path active.\n" +
            "  --profile-path loads a RobotArmProfile JSON file (core/RobotProfiles/*.json) describing\n" +
            "  the robot's topology and per-joint safety limits -- see\n" +
            "  docs/adr/0011-generic-robot-arm-profiles.md. Omit it to run against\n" +
            "  RobotArmProfile.JetRoverMeasuredDefault, the exact configuration this codebase has\n" +
            "  always used for the JetRover.";

        private RobotHostArgs(
            int localPort, IPAddress remoteHost, int remotePort,
            string relaySocketPath, string localRelaySocketPath, float? maxDirectionMagnitude,
            int? jointLocalPort, string? profilePath)
        {
            LocalPort = localPort;
            RemoteHost = remoteHost;
            RemotePort = remotePort;
            RelaySocketPath = relaySocketPath;
            LocalRelaySocketPath = localRelaySocketPath;
            MaxDirectionMagnitude = maxDirectionMagnitude;
            JointLocalPort = jointLocalPort;
            ProfilePath = profilePath;
        }

        public static RobotHostArgs? TryParse(string[] args, out string? error)
        {
            int? localPort = null;
            IPAddress? remoteHost = null;
            int? remotePort = null;
            string? relaySocketPath = null;
            string? localRelaySocketPath = null;
            float? maxDirectionMagnitude = null;
            int? jointLocalPort = null;
            string? profilePath = null;

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
                    case "--joint-local-port" when i + 1 < args.Length:
                        jointLocalPort = ParseIntOrNull(args[++i]);
                        break;
                    case "--profile-path" when i + 1 < args.Length:
                        profilePath = args[++i];
                        break;
                }
            }

            if (localPort is null || remoteHost is null || remotePort is null ||
                relaySocketPath is null || localRelaySocketPath is null ||
                maxDirectionMagnitude is <= 0f || jointLocalPort is <= 0)
            {
                error = "Missing or invalid required argument (--max-direction-magnitude and " +
                    "--joint-local-port, if given, must be positive).";
                return null;
            }

            error = null;
            return new RobotHostArgs(
                localPort.Value, remoteHost, remotePort.Value, relaySocketPath, localRelaySocketPath,
                maxDirectionMagnitude, jointLocalPort, profilePath);
        }

        private static int? ParseIntOrNull(string s) => int.TryParse(s, out int value) ? value : null;
    }
}
