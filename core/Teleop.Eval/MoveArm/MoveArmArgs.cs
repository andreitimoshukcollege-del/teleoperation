using System;
using System.Net;

namespace Teleop.Eval.MoveArm
{
    /// <summary>Parsed command-line arguments for <see cref="MoveArmCommand"/>.</summary>
    internal readonly struct MoveArmArgs
    {
        public readonly float X;
        public readonly float Y;
        public readonly float Z;
        public readonly float Gripper;
        public readonly int LocalPort;
        public readonly IPAddress RemoteHost;
        public readonly int RemotePort;
        public readonly double DurationSeconds;
        public readonly double RateHz;
        public readonly bool ConfirmHardwareMotion;

        public const float DefaultGripper = 0f;
        public const double DefaultDurationSeconds = 3.0;
        public const double DefaultRateHz = 5.0;

        public const string Usage =
            "Usage: Teleop.Eval move-arm --x <meters> --y <meters> --z <meters> " +
            "--remote-host <ip> --remote-port <port> --local-port <port> --confirm-hardware-motion " +
            "[--gripper <0-1>] [--duration-seconds <n>] [--rate-hz <n>]\n" +
            "  Position is in the arm's wrist frame (see Kinematics/FourDofArmKinematics.cs) --\n" +
            "  target point is the wrist, not the gripper fingertip. --gripper is 0 (open, default)\n" +
            "  to 1 (closed).\n" +
            "  --confirm-hardware-motion is mandatory: the remote endpoint is a real RobotEndpoint/\n" +
            "  IRobotPlant. This tool sends a fixed CommandFrame repeatedly for the whole run --\n" +
            "  on a real JetRoverPlant that means the physical arm moves to that pose and holds.\n" +
            "  Pass this flag only when a human is watching the hardware and has confirmed physical\n" +
            "  clearance for this exact target, per this repo's established supervised-hardware-test\n" +
            "  discipline (see robot/README.md).";

        private MoveArmArgs(
            float x, float y, float z, float gripper, int localPort, IPAddress remoteHost, int remotePort,
            double durationSeconds, double rateHz, bool confirmHardwareMotion)
        {
            X = x;
            Y = y;
            Z = z;
            Gripper = gripper;
            LocalPort = localPort;
            RemoteHost = remoteHost;
            RemotePort = remotePort;
            DurationSeconds = durationSeconds;
            RateHz = rateHz;
            ConfirmHardwareMotion = confirmHardwareMotion;
        }

        public static MoveArmArgs? TryParse(string[] args, out string? error)
        {
            float? x = null;
            float? y = null;
            float? z = null;
            float gripper = DefaultGripper;
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
                    case "--x" when i + 1 < args.Length:
                        x = ParseFloatOrNull(args[++i]);
                        break;
                    case "--y" when i + 1 < args.Length:
                        y = ParseFloatOrNull(args[++i]);
                        break;
                    case "--z" when i + 1 < args.Length:
                        z = ParseFloatOrNull(args[++i]);
                        break;
                    case "--gripper" when i + 1 < args.Length:
                        float.TryParse(args[++i], out gripper);
                        break;
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

            if (x is null || y is null || z is null || localPort is null || remoteHost is null ||
                remotePort is null || durationSeconds <= 0 || rateHz <= 0 || gripper < 0f || gripper > 1f ||
                !confirmHardwareMotion)
            {
                error = "Missing or invalid required argument (--confirm-hardware-motion is mandatory; " +
                    "--gripper must be within [0, 1]).";
                return null;
            }

            error = null;
            return new MoveArmArgs(
                x.Value, y.Value, z.Value, gripper, localPort.Value, remoteHost, remotePort.Value,
                durationSeconds, rateHz, confirmHardwareMotion);
        }

        private static int? ParseIntOrNull(string s) => int.TryParse(s, out int value) ? value : null;

        private static float? ParseFloatOrNull(string s) => float.TryParse(s, out float value) ? value : null;
    }
}
