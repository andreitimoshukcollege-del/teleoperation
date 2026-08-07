using System;
using System.Net;

namespace Teleop.Eval.ClockSyncCheck
{
    /// <summary>Parsed command-line arguments for <see cref="ClockSyncCheckCommand"/>.</summary>
    internal readonly struct ClockSyncCheckArgs
    {
        public readonly int LocalPort;
        public readonly IPAddress RemoteHost;
        public readonly int RemotePort;
        public readonly double DurationSeconds;
        public readonly double RateHz;
        public readonly bool ConfirmHardwareMotion;

        public const double DefaultDurationSeconds = 20.0;
        public const double DefaultRateHz = 20.0;

        public const string Usage =
            "Usage: Teleop.Eval clocksync-check --remote-host <ip> --remote-port <port> --local-port <port> " +
            "--confirm-hardware-motion [--duration-seconds <n>] [--rate-hz <n>]\n" +
            "  --confirm-hardware-motion is mandatory: the remote endpoint is a real RobotEndpoint/\n" +
            "  IRobotPlant. This tool sends a fixed CommandFrame repeatedly for the whole run --\n" +
            "  on a real JetRoverPlant that means the physical arm moves once to that pose and\n" +
            "  holds. Pass this flag only when a human is watching the hardware, per this repo's\n" +
            "  established supervised-hardware-test discipline (see robot/README.md).";

        private ClockSyncCheckArgs(
            int localPort, IPAddress remoteHost, int remotePort, double durationSeconds, double rateHz,
            bool confirmHardwareMotion)
        {
            LocalPort = localPort;
            RemoteHost = remoteHost;
            RemotePort = remotePort;
            DurationSeconds = durationSeconds;
            RateHz = rateHz;
            ConfirmHardwareMotion = confirmHardwareMotion;
        }

        public static ClockSyncCheckArgs? TryParse(string[] args, out string? error)
        {
            int? localPort = null;
            IPAddress? remoteHost = null;
            int? remotePort = null;
            double durationSeconds = DefaultDurationSeconds;
            double rateHz = DefaultRateHz;
            bool confirmHardwareMotion = false;

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
                    case "--duration-seconds" when i + 1 < args.Length:
                        double.TryParse(args[++i], out durationSeconds);
                        break;
                    case "--rate-hz" when i + 1 < args.Length:
                        double.TryParse(args[++i], out rateHz);
                        break;
                    case "--confirm-hardware-motion":
                        confirmHardwareMotion = true;
                        break;
                }
            }

            if (localPort is null || remoteHost is null || remotePort is null ||
                durationSeconds <= 0 || rateHz <= 0 || !confirmHardwareMotion)
            {
                error = "Missing or invalid required argument (--confirm-hardware-motion is mandatory).";
                return null;
            }

            error = null;
            return new ClockSyncCheckArgs(
                localPort.Value, remoteHost, remotePort.Value, durationSeconds, rateHz, confirmHardwareMotion);
        }

        private static int? ParseIntOrNull(string s) => int.TryParse(s, out int value) ? value : null;
    }
}
