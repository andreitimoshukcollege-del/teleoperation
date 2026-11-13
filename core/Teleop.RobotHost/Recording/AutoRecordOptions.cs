using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// Another process writing its own log into each session, as the driver writes <c>driver.jlog</c>: Isaac's renderer writes
    /// <c>isaac.jlog</c> and its frame sidecars under <c>isaac/</c> (docs/adr/0032 decision 6). A closed session waits up to
    /// <see cref="Seconds"/> for its <c>END</c> trailer, checks its <c>FS</c> lines, and hashes its folder into <c>files.sha256</c>.
    /// </summary>
    public readonly struct WriterWait
    {
        public WriterWait(string name, double seconds)
        {
            Name = name;
            Seconds = seconds;
        }

        public string Name { get; }

        public double Seconds { get; }

        /// <summary><c>name:seconds</c>, the name lower-case letters, digits, '-' and '_' (it names files).</summary>
        public static bool TryParse(string spec, out WriterWait writer, out string error)
        {
            writer = default;
            error = "";
            int colon = spec.LastIndexOf(':');
            string name = colon > 0 ? spec.Substring(0, colon) : "";
            if (name.Length == 0 || !name.All(c => (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_') ||
                name == "robothost" || name == "driver" ||
                !double.TryParse(spec.Substring(colon + 1), NumberStyles.Float, CultureInfo.InvariantCulture, out double seconds) || !(seconds >= 0))
            {
                error = $"--auto-record-writer '{spec}': expected name:seconds (like isaac:60), a name other than robothost or driver";
                return false;
            }

            writer = new WriterWait(name, seconds);
            return true;
        }
    }

    /// <summary>
    /// The thresholds and paths of <see cref="AutoSessionRecorder"/> (docs/adr/0023 decision 4), one
    /// command-line argument each (<c>RobotHostArgs</c>). The defaults are the ADR's.
    /// </summary>
    public sealed class AutoRecordOptions
    {
        /// <summary>A session opens on the first joint command after at least this long without one, and closes after this long without one.</summary>
        public double IdleSeconds { get; set; } = 30;

        /// <summary>A session closes after this long and the next command opens a new one, so no file grows past about 200 MB.</summary>
        public double MaxMinutes { get; set; } = 60;

        /// <summary>An open session closes when the recordings' disk has less than this free.</summary>
        public double StopFreeGigabytes { get; set; } = 1;

        /// <summary>No session opens while the recordings' disk has less than this free.</summary>
        public double StartFreeGigabytes { get; set; } = 2;

        /// <summary>How long a closed session waits for the driver's <c>END</c> trailer before <c>files.sha256</c> is written anyway.</summary>
        public double DriverWaitSeconds { get; set; } = 15;

        /// <summary>Other writers each session waits for (<c>--auto-record-writer name:seconds</c>), after the driver.</summary>
        public List<WriterWait> Writers { get; } = new List<WriterWait>();

        /// <summary>
        /// A second sender or subscriber change this soon after one that split the session does not
        /// split it again: two endpoints taking turns on the joint port would otherwise make a session
        /// per datagram. The session then records both, and the trainer's check rejects it, honestly.
        /// </summary>
        public double MinSplitSeconds { get; set; } = 5;

        /// <summary>
        /// Only a headset's joint commands open or keep a session (2026-10-26): a sender counts as one
        /// while its last headset heartbeat (the operator message, docs/adr/0018 §9) is at most this old.
        /// <c>joint-drive</c>, <c>move-arm</c> and other tools send none, so they are never recorded
        /// automatically.
        /// </summary>
        public double HeadsetSeconds { get; set; } = 5;

        /// <summary>Where automatic sessions go: <c>&lt;root&gt;/&lt;UTC&gt;-auto/</c>. Null: <c>jetrover/</c> beside the control file.</summary>
        public string? RecordingsRoot { get; set; }

        /// <summary>The robot profile copied into each session as <c>robot-profile.json</c>. Null: none is copied, and the session says so.</summary>
        public string? ProfilePath { get; set; }

        /// <summary>
        /// What the session's <c>session.json</c> says it is: <c>jetrover-session</c> v1 from the robot, or, for a
        /// serial-chain robot (docs/adr/0027 decision 6), <c>teleop-session</c> v2 with its source (<c>robot</c>,
        /// or <c>sim-host</c> from the simulated backend) and the profile's name.
        /// </summary>
        public string SessionFormat { get; set; } = "jetrover-session";

        public int SessionVersion { get; set; } = 1;

        public string Source { get; set; } = "robot";

        /// <summary>The <c>source</c> values a session may carry (trainer/contract/SESSION_FORMAT.md).</summary>
        public static readonly string[] Sources = { "robot", "sim", "sim-host", "isaac", "arm-server-sim", "unknown" };

        /// <summary>The robot profile's name, written into a v2 <c>session.json</c>. Null for v1.</summary>
        public string? ProfileName { get; set; }

        /// <summary>The scene file (docs/adr/0031 amendment 2), copied into each session as <c>scene.json</c>. Null: no world.</summary>
        public string? ScenePath { get; set; }

        public string? SceneName { get; set; }

        /// <summary>The seed layout 1 was drawn from; later layouts are in the session's <c>WL</c> records.</summary>
        public ulong SceneSeed { get; set; }

        /// <summary>What the arm server said it is (kind and description: the Isaac version and drive gains). Null: no arm server.</summary>
        public string? ArmServerKind { get; set; }

        public string? ArmServerDescription { get; set; }

        /// <summary>RobotHost's own <c>BUILD_INFO</c>. Null: <c>BUILD_INFO</c> beside the running assembly.</summary>
        public string? RobotHostBuildInfoPath { get; set; }

        /// <summary>The driver's <c>BUILD_INFO</c> (<c>~/teleop_ros_current/BUILD_INFO</c>). Null: unknown (<c>{}</c>), which the trainer's check fails.</summary>
        public string? DriverBuildInfoPath { get; set; }

        public long BytesFromGigabytes(double gigabytes) => (long)(gigabytes * 1024 * 1024 * 1024);

        /// <summary>Null when every value is usable, else what is wrong.</summary>
        public string? Validate()
        {
            if (Array.IndexOf(Sources, Source) < 0) return $"--auto-record-source '{Source}' is not one of {string.Join(", ", Sources)}";
            if (!(IdleSeconds > 0)) return "--auto-record-idle-seconds must be positive";
            if (!(MaxMinutes > 0)) return "--auto-record-max-minutes must be positive";
            if (!(StopFreeGigabytes >= 0)) return "--auto-record-stop-free-gb must be zero or more";
            if (!(StartFreeGigabytes >= StopFreeGigabytes)) return "--auto-record-start-free-gb must be at least --auto-record-stop-free-gb";
            if (!(DriverWaitSeconds >= 0)) return "--auto-record-driver-wait-seconds must be zero or more";
            if (!(MinSplitSeconds >= 0)) return "--auto-record-min-split-seconds must be zero or more";
            if (!(HeadsetSeconds > 0)) return "--auto-record-headset-seconds must be positive";
            return null;
        }

        public override string ToString() =>
            FormattableString.Invariant(
                $"idle {IdleSeconds} s, rotate at {MaxMinutes} min, stop below {StopFreeGigabytes} GB free, start only above {StartFreeGigabytes} GB, ") +
            FormattableString.Invariant($"driver wait {DriverWaitSeconds} s, ") +
            string.Concat(Writers.Select(w => FormattableString.Invariant($"{w.Name} wait {w.Seconds} s, "))) +
            FormattableString.Invariant($"min split {MinSplitSeconds} s, headset only (heartbeat within {HeadsetSeconds} s)");
    }
}
