using System;
using System.Globalization;
using System.IO;
using Teleop.Core.Contracts;
using Teleop.Core.Transport;
using Teleop.Core.Transport.Impairments;

namespace Teleop.RobotHost.Net
{
    /// <summary>
    /// What an uplink profile name means (docs/adr/0021 decision 4, docs/adr/0035 N4): a Core profile from
    /// <see cref="NetworkProfileCatalog"/>, or <c>trace:&lt;dataset&gt;/&lt;id&gt;</c>, one of the network datasets' traces
    /// (trainer/contract/NETWORK_DATASETS.md) at <c>&lt;root&gt;/&lt;dataset&gt;/traces/&lt;id&gt;.trace</c> under
    /// <c>--uplink-traces</c>, replayed by time (<see cref="TimedTraceImpairment"/>): its delays and its losses, as the real
    /// link it was measured on had them. Only version-2 traces (they carry their send interval and losses); delays are
    /// rescaled from the file's tick rate to this host's. A name that does not resolve is refused with the reason, so a
    /// typo never records a session as impaired when it was not.
    /// </summary>
    public static class UplinkProfiles
    {
        public const string TracePrefix = "trace:";

        public static bool TryResolve(string name, long ticksPerSecond, string? tracesRoot, out INetworkImpairment[] impairments, out string? error)
        {
            if (name.StartsWith(TracePrefix, StringComparison.Ordinal))
            {
                return TryLoadTrace(name.Substring(TracePrefix.Length), ticksPerSecond, tracesRoot, out impairments, out error);
            }

            return NetworkProfileCatalog.TryResolveImpairments(name, ticksPerSecond, out impairments, out error);
        }

        private static bool Safe(string part) =>
            part.Length > 0 && part.Length <= 160 && part[0] != '.' && part.IndexOfAny(new[] { '/', '\\' }) < 0 &&
            Array.TrueForAll(part.ToCharArray(), c => char.IsLetterOrDigit(c) || c == '.' || c == '-' || c == '_');

        private static bool TryLoadTrace(string reference, long tps, string? root, out INetworkImpairment[] impairments, out string? error)
        {
            impairments = Array.Empty<INetworkImpairment>();
            error = null;
            string[] parts = reference.Split('/');
            if (parts.Length != 2 || !Safe(parts[0]) || !Safe(parts[1]))
            {
                error = $"'{TracePrefix}{reference}' is not {TracePrefix}<dataset>/<id>";
                return false;
            }

            if (root == null)
            {
                error = "trace profiles need --uplink-traces (the network datasets' root)";
                return false;
            }

            string path = Path.Combine(root, parts[0], "traces", parts[1] + ".trace");
            if (!File.Exists(path))
            {
                error = $"no trace at {path}";
                return false;
            }

            string[] lines = File.ReadAllLines(path);
            string[] head = lines.Length > 0 ? lines[0].Split('|') : Array.Empty<string>();
            if (head.Length != 4 || head[0] != "TRACE" || head[1] != "2" ||
                !long.TryParse(head[2], NumberStyles.None, CultureInfo.InvariantCulture, out long fileTps) || fileTps <= 0 ||
                !long.TryParse(head[3], NumberStyles.None, CultureInfo.InvariantCulture, out long fileInterval) || fileInterval <= 0)
            {
                error = $"{path}: not a version-2 trace header ('{(lines.Length > 0 ? lines[0] : "")}')";
                return false;
            }

            int n = 0;
            for (int i = 1; i < lines.Length; i++) if (lines[i].Length > 0) n++;
            var delays = new long[n];
            var lost = new bool[n];
            int k = 0;
            double scale = (double)tps / fileTps;
            for (int i = 1; i < lines.Length; i++)
            {
                string line = lines[i];
                if (line.Length == 0) continue;
                if (line == "L")
                {
                    lost[k++] = true;
                    continue;
                }

                if (!long.TryParse(line, NumberStyles.None, CultureInfo.InvariantCulture, out long d))
                {
                    error = $"{path}:{i + 1}: not a delay or L: '{line}'";
                    return false;
                }

                delays[k++] = (long)Math.Round(d * scale);
            }

            if (n == 0)
            {
                error = $"{path}: no samples";
                return false;
            }

            impairments = new[] { NetworkProfileCatalog.CreateTimedTrace(delays, lost, Math.Max(1L, (long)Math.Round(fileInterval * scale))) };
            return true;
        }
    }
}
