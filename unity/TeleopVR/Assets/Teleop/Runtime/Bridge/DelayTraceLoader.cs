using System;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Loads a recorded one-way-delay trace (the <c>TRACE|version|ticksPerSecond</c> text format
    /// written by <c>core/Teleop.Eval/Sweep/TraceFile.cs</c>) so Unity can drive
    /// <c>EmulatedTransport</c>'s trace-driven mode with the same fixture the sweeps use.
    ///
    /// File I/O lives here rather than in Core for the usual reason: Core is forbidden from
    /// touching the filesystem (root CLAUDE.md invariant 3), which is exactly why
    /// <c>Teleop.Eval</c> owns the reader on that side and why <c>NetworkProfileCatalog</c> in Core
    /// resolves only the parametric profiles. This is Bridge doing the host's job, the same
    /// division <see cref="ConfigLoader"/> already follows.
    ///
    /// <b>Where the file comes from.</b> <c>Application.persistentDataPath</c> first (pushed with
    /// <c>adb push</c> on device, or installed by <c>just install-traces</c> for the Editor), then
    /// a <c>Resources</c> <c>TextAsset</c> if one was shipped in the build. No copy of the trace is
    /// committed under <c>unity/</c> on purpose: <c>core/testdata/traces/</c> is the one source, and
    /// a second copy would drift from it silently -- the same argument docs/adr/0004 makes about
    /// profile numbers drifting between a manifest and its documentation.
    ///
    /// Never throws. A missing or malformed trace returns <c>null</c> with a specific reason, and
    /// the caller declines to enter trace mode rather than falling back to some other impairment
    /// the operator did not ask for.
    /// </summary>
    public static class DelayTraceLoader
    {
        private const string HeaderTag = "TRACE";
        private const int SupportedVersion = 1;

        /// <summary>
        /// Reads <paramref name="traceName"/> (e.g. <c>synthetic-burst</c>) and returns its samples
        /// <b>already rescaled into <paramref name="localTicksPerSecond"/></b>.
        ///
        /// <b>The rescale is not optional and not cosmetic.</b> The header records the tick rate of
        /// the machine that wrote the file -- 10,000,000 on Windows, 1,000,000,000 on a Linux ARM64
        /// device -- and <c>EmulatedTransport</c> compares a sample directly against
        /// <c>ITimeAuthority</c>'s ticks. Replaying a Windows-written trace on a Quest without
        /// rescaling would inflate every delay by 100x while every individual number still looked
        /// entirely plausible. That exact failure has already happened once in this project, across
        /// this exact pair of tick rates: see robot/README.md's ClockSync finding, where uplink and
        /// downlink one-way delays came out in the tens of seconds and still summed to the correct
        /// RTT.
        /// </summary>
        public static long[] TryLoad(string traceName, long localTicksPerSecond, out string error)
        {
            if (string.IsNullOrEmpty(traceName))
            {
                error = "no trace name given";
                return null;
            }

            string text = ReadText(traceName, out string source);
            if (text == null)
            {
                error =
                    $"'{traceName}.trace' not found. Install it with `just install-traces` (Editor) " +
                    $"or `adb push` it to Application.persistentDataPath (device). The source of " +
                    $"truth is core/testdata/traces/{traceName}.trace; no copy is committed under unity/.";
                return null;
            }

            string[] lines = text.Split('\n');
            if (lines.Length < 2)
            {
                error = $"'{traceName}.trace' ({source}) has no samples";
                return null;
            }

            string[] header = lines[0].Trim().Split('|');
            if (header.Length != 3 || header[0] != HeaderTag)
            {
                error = $"'{traceName}.trace' ({source}) line 1 is not a TRACE header: {lines[0].Trim()}";
                return null;
            }

            if (!int.TryParse(header[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out int version)
                || version != SupportedVersion)
            {
                error = $"'{traceName}.trace' ({source}) has unsupported version '{header[1]}' (expected {SupportedVersion})";
                return null;
            }

            if (!long.TryParse(header[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out long fileTicksPerSecond)
                || fileTicksPerSecond <= 0)
            {
                error = $"'{traceName}.trace' ({source}) has an invalid ticksPerSecond '{header[2]}'";
                return null;
            }

            var samples = new long[lines.Length - 1];
            int count = 0;
            for (int i = 1; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (line.Length == 0)
                {
                    continue;   // trailing newline, and CRLF if the file crossed a box
                }

                if (!long.TryParse(line, NumberStyles.Integer, CultureInfo.InvariantCulture, out long sample)
                    || sample < 0)
                {
                    error = $"'{traceName}.trace' ({source}) line {i + 1} is not a non-negative tick integer: {line}";
                    return null;
                }

                samples[count++] = Rescale(sample, fileTicksPerSecond, localTicksPerSecond);
            }

            if (count == 0)
            {
                error = $"'{traceName}.trace' ({source}) has a valid header but no samples";
                return null;
            }

            Array.Resize(ref samples, count);
            error = null;
            Debug.Log(
                $"DelayTraceLoader: '{traceName}' loaded from {source} -- {count} samples, " +
                $"rescaled {fileTicksPerSecond} -> {localTicksPerSecond} ticks/s.");
            return samples;
        }

        /// <summary>
        /// Converts one sample between tick rates. Done in <see cref="double"/> rather than as
        /// <c>sample * local / file</c> in integers, which overflows for a long trace on a
        /// nanosecond clock, and rounded rather than truncated so a rescale cannot systematically
        /// shorten every delay in the trace.
        /// </summary>
        private static long Rescale(long sample, long fileTicksPerSecond, long localTicksPerSecond)
        {
            if (fileTicksPerSecond == localTicksPerSecond)
            {
                return sample;
            }

            return (long)Math.Round(sample * ((double)localTicksPerSecond / fileTicksPerSecond));
        }

        /// <summary>persistentDataPath first, then Resources -- the order <see cref="ConfigLoader"/> uses.</summary>
        private static string ReadText(string traceName, out string source)
        {
            string overridePath = Path.Combine(Application.persistentDataPath, traceName + ".trace");
            if (File.Exists(overridePath))
            {
                source = overridePath;
                return File.ReadAllText(overridePath);
            }

            TextAsset asset = Resources.Load<TextAsset>(traceName);
            if (asset != null)
            {
                source = $"Resources/{traceName}";
                return asset.text;
            }

            source = null;
            return null;
        }
    }
}
