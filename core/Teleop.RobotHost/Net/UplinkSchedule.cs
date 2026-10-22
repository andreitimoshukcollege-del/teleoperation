using System;
using System.Collections.Generic;
using System.Globalization;
using Teleop.Core.Contracts;
using Teleop.Core.Transport;

namespace Teleop.RobotHost.Net
{
    /// <summary>
    /// A robot-side uplink impairment that changes on its own during a session (docs/adr/0021 decision
    /// 13), so a live recording holds uplink changes a network forecast could have announced. Core
    /// profiles from <see cref="NetworkProfileCatalog"/> (or <c>none</c>/<c>lan</c> for off), each from
    /// an offset after the schedule starts.
    ///
    /// <para>The uplink control file holds <c>schedule &lt;seed&gt; &lt;profile&gt;@&lt;seconds&gt;;...</c>:
    /// the trainer's grammar for a named schedule (<c>lan@0;150ms-20j-0.5loss@45;lan@90</c>). The
    /// first segment starts at 0 and the offsets increase. A name Core does not know refuses the whole
    /// schedule, so a typo never records a session as impaired when it was not.</para>
    /// </summary>
    public sealed class UplinkSchedule
    {
        public const string Prefix = "schedule ";

        private readonly double[] _starts;
        private readonly string[] _profiles;

        private UplinkSchedule(double[] starts, string[] profiles, ulong seed, string spec)
        {
            _starts = starts;
            _profiles = profiles;
            Seed = seed;
            Spec = spec;
        }

        public ulong Seed { get; }

        /// <summary>The schedule as written, for the log.</summary>
        public string Spec { get; }

        public int Count => _starts.Length;

        public string ProfileOf(int index) => _profiles[index];

        /// <summary>The segment in force <paramref name="elapsedSeconds"/> after the schedule started.</summary>
        public int IndexAt(double elapsedSeconds)
        {
            int index = 0;
            for (int i = 1; i < _starts.Length; i++)
            {
                if (_starts[i] <= elapsedSeconds) index = i;
            }

            return index;
        }

        public static bool IsSchedule(string? request) =>
            request != null && request.StartsWith(Prefix, StringComparison.Ordinal);

        /// <summary>Parses <c>schedule &lt;seed&gt; &lt;spec&gt;</c>; false, with the reason, on anything malformed.</summary>
        public static bool TryParse(string request, long ticksPerSecond, out UplinkSchedule? schedule, out string? error)
        {
            schedule = null;
            error = null;
            string[] parts = request.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != 3 || parts[0] != Prefix.Trim())
            {
                error = "expected 'schedule <seed> <profile>@<seconds>;...'";
                return false;
            }

            if (!ulong.TryParse(parts[1], NumberStyles.None, CultureInfo.InvariantCulture, out ulong seed))
            {
                error = $"'{parts[1]}' is not a seed";
                return false;
            }

            var starts = new List<double>();
            var profiles = new List<string>();
            foreach (string item in parts[2].Split(';', StringSplitOptions.RemoveEmptyEntries))
            {
                int at = item.LastIndexOf('@');
                if (at <= 0 || !double.TryParse(item.Substring(at + 1), NumberStyles.Float, CultureInfo.InvariantCulture, out double start)
                    || !(start >= 0.0))
                {
                    error = $"'{item}' is not <profile>@<seconds>";
                    return false;
                }

                string name = item.Substring(0, at);
                if (name != "none" && name != "lan" &&
                    !NetworkProfileCatalog.TryResolveImpairments(name, ticksPerSecond, out INetworkImpairment[] _, out string? why))
                {
                    error = why ?? $"unknown network profile '{name}'";
                    return false;
                }

                if (starts.Count > 0 && !(start > starts[starts.Count - 1]))
                {
                    error = $"'{item}' does not start after the segment before it";
                    return false;
                }

                starts.Add(start);
                profiles.Add(name);
            }

            if (starts.Count == 0 || starts[0] != 0.0)
            {
                error = "a schedule's first segment starts at 0";
                return false;
            }

            schedule = new UplinkSchedule(starts.ToArray(), profiles.ToArray(), seed, parts[2]);
            return true;
        }
    }
}
