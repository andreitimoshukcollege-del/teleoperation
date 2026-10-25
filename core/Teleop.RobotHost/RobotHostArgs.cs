using System;
using System.Net;

namespace Teleop.RobotHost
{
    /// <summary>Parsed command-line arguments for <see cref="Program"/>.</summary>
    internal readonly struct RobotHostArgs
    {
        public readonly int LocalPort;

        /// <summary>
        /// Where replies go before any command has arrived (and always, when
        /// <see cref="ReplyToSender"/> is false). Optional in sender mode: <c>RobotEndpoint</c>
        /// only replies to a received datagram, so in practice nothing is sent before the first
        /// command tells the transport where the operator is. Null when not given.
        /// </summary>
        public readonly IPAddress? RemoteHost;
        public readonly int? RemotePort;

        /// <summary>
        /// <c>--reply-to sender</c> (the default): reply to the source address and port of the most
        /// recent command, so the operator's address can change (DHCP, a different Wi-Fi, a Quest
        /// instead of a PC, a NAT rewriting the source port) without redeploying this host.
        /// <c>--reply-to fixed</c>: the original behaviour, always replying to
        /// <see cref="RemoteHost"/>:<see cref="RemotePort"/>, which are then required.
        /// </summary>
        public readonly bool ReplyToSender;
        public readonly string RelaySocketPath;
        public readonly string LocalRelaySocketPath;
        public readonly float? MaxDirectionMagnitude;

        /// <summary>
        /// Optional override of <c>GenericArmPlantConfig.Default.PulsesPerSecond</c>: how fast every
        /// servo is told to move (the relay turns a move into <c>pulseDelta / this</c> seconds). Lower
        /// is slower. Null keeps the default.
        /// </summary>
        public readonly float? PulsesPerSecond;

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

        /// <summary>
        /// The recording control file (docs/adr/0021): while it exists, RobotHost writes
        /// <c>robothost.jlog</c> into the session directory it names. Null (the default) never records.
        /// <c>just deploy-robothost</c> passes <c>~/recordings/.active</c>; <c>just record-start</c> and
        /// <c>record-stop</c> write and remove it, so recording needs no restart.
        /// </summary>
        public readonly string? RecordControlPath;

        /// <summary>
        /// The uplink control file (docs/adr/0021): while it holds <c>profile seed</c>, joint commands and
        /// keepalives pass through that Core network profile on their way to the plant
        /// (<see cref="Net.UplinkImpairedReceiver"/>). Absent, or <c>none</c>, impairs nothing. Null (the
        /// default) never impairs. <c>just uplink-impair</c> writes it.
        /// </summary>
        public readonly string? UplinkControlPath;

        /// <summary>
        /// The discovery port (docs/adr/0022-robot-discovery.md): RobotHost answers the headset's
        /// "where are you?" there with its hostname and LAN addresses. Null (the default) never answers.
        /// <c>just deploy-robothost</c> passes 6005.
        /// </summary>
        public readonly int? DiscoveryPort;

        public const string Usage =
            "Usage: Teleop.RobotHost --local-port <port> [--remote-host <ip> --remote-port <port>] " +
            "[--reply-to sender|fixed] --relay-socket <path> --local-relay-socket <path> " +
            "[--max-direction-magnitude <n>] [--pulses-per-second <n>] [--joint-local-port <port>] [--profile-path <path>] " +
            "[--record-control <path>] [--uplink-control <path>] [--discovery-port <port>]\n" +
            "  --reply-to sender (default) replies to wherever the latest command came from;\n" +
            "  --remote-host/--remote-port are then only the target before the first command.\n" +
            "  --reply-to fixed always replies to --remote-host:--remote-port, which it requires.\n" +
            "  --max-direction-magnitude overrides GenericArmPlantConfig.Default's clamp (5) on how far\n" +
            "  a single accepted command may move a joint's belief -- lower it (e.g. 1-2) for a\n" +
            "  visibly slower, gentler arm; omit it to keep the default.\n" +
            "  --pulses-per-second overrides how fast every servo is told to move (default 2200,\n" +
            "  about 9 rad/s commanded; 900 is about 3.8 rad/s). Lower is slower.\n" +
            "  --joint-local-port opens a second, optional listener for pre-computed joint-angle\n" +
            "  commands (GenericArmPlant.CommandJointAngles) instead of the default Cartesian-target\n" +
            "  path -- see docs/adr/0009-jetrover-operator-side-inverse-kinematics.md. Omit it to\n" +
            "  run exactly as before, with only the Cartesian path active.\n" +
            "  --profile-path loads a RobotArmProfile JSON file (core/RobotProfiles/*.json) describing\n" +
            "  the robot's topology and per-joint safety limits -- see\n" +
            "  docs/adr/0011-generic-robot-arm-profiles.md. Omit it to run against\n" +
            "  RobotArmProfile.JetRoverMeasuredDefault, the exact configuration this codebase has\n" +
            "  always used for the JetRover.\n" +
            "  --record-control names the recording control file (docs/adr/0021): while it exists,\n" +
            "  robothost.jlog is written into the session directory it names. Omit it to never record.\n" +
            "  --uplink-control names the uplink control file (docs/adr/0021): while it holds a Core\n" +
            "  network profile and a seed, joint commands reach the plant through that profile. Omit it\n" +
            "  to never impair the uplink.\n" +
            "  --discovery-port answers the headset's discovery query there with this machine's name and\n" +
            "  LAN addresses (docs/adr/0022). Omit it to not answer.";

        private RobotHostArgs(
            int localPort, IPAddress? remoteHost, int? remotePort, bool replyToSender,
            string relaySocketPath, string localRelaySocketPath, float? maxDirectionMagnitude,
            int? jointLocalPort, string? profilePath, float? pulsesPerSecond = null, string? recordControlPath = null,
            string? uplinkControlPath = null, int? discoveryPort = null)
        {
            LocalPort = localPort;
            RemoteHost = remoteHost;
            RemotePort = remotePort;
            ReplyToSender = replyToSender;
            RelaySocketPath = relaySocketPath;
            LocalRelaySocketPath = localRelaySocketPath;
            MaxDirectionMagnitude = maxDirectionMagnitude;
            PulsesPerSecond = pulsesPerSecond;
            JointLocalPort = jointLocalPort;
            ProfilePath = profilePath;
            RecordControlPath = recordControlPath;
            UplinkControlPath = uplinkControlPath;
            DiscoveryPort = discoveryPort;
        }

        public static RobotHostArgs? TryParse(string[] args, out string? error)
        {
            int? localPort = null;
            IPAddress? remoteHost = null;
            int? remotePort = null;
            string? relaySocketPath = null;
            string? localRelaySocketPath = null;
            float? maxDirectionMagnitude = null;
            float? pulsesPerSecond = null;
            int? jointLocalPort = null;
            string? profilePath = null;
            string? recordControlPath = null;
            string? uplinkControlPath = null;
            int? discoveryPort = null;
            string replyTo = "sender";

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
                    case "--reply-to" when i + 1 < args.Length:
                        replyTo = args[++i];
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
                    case "--pulses-per-second" when i + 1 < args.Length:
                        if (float.TryParse(args[++i], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float parsedPps))
                        {
                            pulsesPerSecond = parsedPps;
                        }
                        else
                        {
                            pulsesPerSecond = -1f; // rejected below
                        }
                        break;
                    case "--joint-local-port" when i + 1 < args.Length:
                        jointLocalPort = ParseIntOrNull(args[++i]);
                        break;
                    case "--profile-path" when i + 1 < args.Length:
                        profilePath = args[++i];
                        break;
                    case "--record-control" when i + 1 < args.Length:
                        recordControlPath = args[++i];
                        break;
                    case "--uplink-control" when i + 1 < args.Length:
                        uplinkControlPath = args[++i];
                        break;
                    case "--discovery-port" when i + 1 < args.Length:
                        discoveryPort = ParseIntOrNull(args[++i]) ?? -1;
                        break;
                }
            }

            if (localPort is null || relaySocketPath is null || localRelaySocketPath is null ||
                maxDirectionMagnitude is <= 0f || jointLocalPort is <= 0 || pulsesPerSecond is <= 0f || discoveryPort is <= 0 or > 65535)
            {
                error = "Missing or invalid required argument (--max-direction-magnitude, " +
                    "--pulses-per-second, --joint-local-port and --discovery-port, if given, must be positive).";
                return null;
            }

            if (replyTo != "sender" && replyTo != "fixed")
            {
                error = $"--reply-to must be 'sender' or 'fixed', not '{replyTo}'.";
                return null;
            }

            // A host without a port (or the reverse) is a typo, not a choice -- reject it rather
            // than silently replying nowhere until the first command arrives.
            if ((remoteHost is null) != (remotePort is null))
            {
                error = "--remote-host and --remote-port must be given together (and the host must be an IP address).";
                return null;
            }

            bool replyToSender = replyTo == "sender";
            if (!replyToSender && remoteHost is null)
            {
                error = "--reply-to fixed needs --remote-host and --remote-port.";
                return null;
            }

            error = null;
            return new RobotHostArgs(
                localPort.Value, remoteHost, remotePort, replyToSender, relaySocketPath, localRelaySocketPath,
                maxDirectionMagnitude, jointLocalPort, profilePath, pulsesPerSecond, recordControlPath, uplinkControlPath, discoveryPort);
        }

        private static int? ParseIntOrNull(string s) => int.TryParse(s, out int value) ? value : null;
    }
}
