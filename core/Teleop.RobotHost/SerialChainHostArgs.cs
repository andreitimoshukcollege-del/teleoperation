using System;
using System.Globalization;
using Teleop.RobotHost.Backend;
using Teleop.RobotHost.Recording;

namespace Teleop.RobotHost
{
    /// <summary>
    /// The command line of a serial-chain RobotHost (docs/adr/0027 decision 5): the joint port only, a profile,
    /// the simulated backend's tracking, recording and an impaired uplink. The JetRover's relay, Cartesian path
    /// and arm power have no meaning here, so they are not accepted.
    /// </summary>
    internal sealed class SerialChainHostArgs
    {
        public const string Usage =
            "Usage: Teleop.RobotHost --profile-path <serial-chain profile> --joint-local-port <port> --backend sim [--collision-model <file>]\n" +
            "  [--sim-track-hz <hz>] [--sim-track-damping <z>] [--sim-noise-rad <rad>] [--sim-seed <n>]\n" +
            "  [--discovery-port <port>] [--uplink-control <path>] [--record-control <path>]\n" +
            "  [--auto-record [--auto-record-root <dir>] [--auto-record-idle-seconds <s>] [--auto-record-max-minutes <min>]\n" +
            "   [--auto-record-headset-seconds <s>] [--build-info-robothost <path>]]\n" +
            "  A serial-chain profile (\"kind\": \"serial-chain\", docs/adr/0026) runs this host instead of the JetRover's:\n" +
            "  the joint port only (commands in, joint-state frames out at 50 Hz to whoever sends a keepalive), no relay,\n" +
            "  no Cartesian path, no arm power (docs/adr/0027 decision 5). --backend sim is the simulated arm\n" +
            "  (Fr3SimBackend: Core's trajectory shaper at 1 kHz, then a second-order tracking lag of --sim-track-hz (20)\n" +
            "  and damping --sim-track-damping (1), readings with --sim-noise-rad (0) of seeded noise). It is the only\n" +
            "  backend until the real arm's ADR. --collision-model (core/RobotProfiles/<profile>.collision.json, fit-capsules) refuses\n" +
            "  every command whose pose would collide (docs/adr/0026 amendment). Sessions are teleop-session v2 with source sim-host\n" +
            "  (trainer/contract/SESSION_FORMAT.md).";

        public string ProfilePath = "";
        public int JointLocalPort;
        public string Backend = "";
        public readonly Fr3SimOptions Sim = new Fr3SimOptions();
        public int? DiscoveryPort;
        public string? UplinkControlPath;
        public string? RecordControlPath;
        public string? CollisionModelPath;
        public AutoRecordOptions? AutoRecord;

        public static SerialChainHostArgs? TryParse(string[] args, out string error)
        {
            var a = new SerialChainHostArgs();
            var auto = new AutoRecordOptions { DriverWaitSeconds = 0, SessionFormat = "teleop-session", SessionVersion = 2, Source = "sim-host" };
            bool autoRecord = false;
            int? jointPort = null;
            error = "";
            for (int i = 0; i < args.Length; i++)
            {
                string flag = args[i];
                if (flag == "--auto-record")
                {
                    autoRecord = true;
                    continue;
                }

                if (i + 1 >= args.Length)
                {
                    error = $"{flag} needs a value";
                    return null;
                }

                string value = args[++i];
                switch (flag)
                {
                    case "--profile-path": a.ProfilePath = value; break;
                    case "--joint-local-port": jointPort = Port(value); break;
                    case "--backend": a.Backend = value; break;
                    case "--sim-track-hz": a.Sim.TrackHz = Number(value); break;
                    case "--sim-track-damping": a.Sim.TrackDamping = Number(value); break;
                    case "--sim-noise-rad": a.Sim.NoiseRad = Number(value); break;
                    case "--sim-seed":
                        if (!ulong.TryParse(value, NumberStyles.None, CultureInfo.InvariantCulture, out ulong seed)) { error = $"--sim-seed '{value}' is not a whole number"; return null; }
                        a.Sim.Seed = seed;
                        break;
                    case "--discovery-port": a.DiscoveryPort = Port(value); break;
                    case "--uplink-control": a.UplinkControlPath = value; break;
                    case "--record-control": a.RecordControlPath = value; break;
                    case "--collision-model": a.CollisionModelPath = value; break;
                    case "--auto-record-root": auto.RecordingsRoot = value; break;
                    case "--auto-record-idle-seconds": auto.IdleSeconds = Number(value); break;
                    case "--auto-record-max-minutes": auto.MaxMinutes = Number(value); break;
                    case "--auto-record-headset-seconds": auto.HeadsetSeconds = Number(value); break;
                    case "--build-info-robothost": auto.RobotHostBuildInfoPath = value; break;
                    default:
                        error = $"unknown argument '{flag}' for a serial-chain robot";
                        return null;
                }
            }

            if (a.ProfilePath.Length == 0) error = "--profile-path is required";
            else if (jointPort is null || jointPort <= 0) error = "--joint-local-port is required (1-65535)";
            else if (a.Backend != "sim") error = "--backend sim is required: the simulated arm is the only backend until the real arm's ADR (docs/adr/0027)";
            else if (a.DiscoveryPort.HasValue && a.DiscoveryPort <= 0) error = "--discovery-port must be 1-65535";
            else if (a.Sim.Validate() is string sim) error = "--sim: " + sim;
            else if (autoRecord && a.RecordControlPath == null) error = "--auto-record needs --record-control";
            else if (autoRecord && auto.Validate() is string bad) error = bad;

            if (error.Length > 0) return null;

            a.JointLocalPort = jointPort!.Value;
            if (autoRecord)
            {
                auto.ProfilePath = a.ProfilePath;
                a.AutoRecord = auto;
            }

            return a;
        }

        private static int Port(string text) =>
            int.TryParse(text, NumberStyles.None, CultureInfo.InvariantCulture, out int port) && port > 0 && port <= 65535 ? port : -1;

        private static double Number(string text) =>
            double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : double.NaN;
    }
}
