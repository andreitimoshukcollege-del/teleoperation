using System.Globalization;

namespace Teleop.Eval.Sweep
{
    /// <summary>
    /// Reader/writer for the recorded-delay-trace text format
    /// (<c>docs/adr/0004-network-profile-suite.md</c>): a header line
    /// <c>TRACE|&lt;version&gt;|&lt;ticksPerSecond&gt;</c>, then one non-negative tick integer per
    /// line, in order -- the "recorded one-way delay" sequence
    /// <see cref="Teleop.Core.Transport.EmulatedTransport"/>'s trace-driven mode replays.
    ///
    /// Text, not binary, in the same "research data is text -- diff it" spirit as
    /// <c>Recording/RecordFormat.cs</c>'s <c>.tlog</c> format, without adopting that spec wholesale:
    /// a network trace has none of <c>.tlog</c>'s multi-record-type or checksum needs, so a
    /// dedicated minimal format is clearer than reusing one built for something else.
    ///
    /// Lives in <c>Teleop.Eval</c>, not Core: reading/writing an actual file is I/O, which Core is
    /// forbidden from doing (<c>Recording/CLAUDE.md</c> makes the identical argument for
    /// <c>TlogFileWriter</c>/<c>TlogFileReader</c>).
    /// </summary>
    public static class TraceFile
    {
        private const string HeaderTag = "TRACE";
        private const int Version = 1;

        public static void Write(string path, long ticksPerSecond, IReadOnlyList<long> delayTicks)
        {
            using var writer = new StreamWriter(path, append: false, System.Text.Encoding.ASCII) { NewLine = "\n" };
            writer.WriteLine($"{HeaderTag}|{Version}|{ticksPerSecond}");
            foreach (long sample in delayTicks)
            {
                writer.WriteLine(sample.ToString(CultureInfo.InvariantCulture));
            }
        }

        public static (long TicksPerSecond, long[] DelayTicks) Read(string path)
        {
            string[] lines = File.ReadAllLines(path);
            if (lines.Length == 0)
            {
                throw new InvalidDataException($"{path}: empty trace file");
            }

            string[] header = lines[0].Split('|');
            if (header.Length != 3 || header[0] != HeaderTag)
            {
                throw new InvalidDataException($"{path}: line 1 is not a valid trace header: {lines[0]}");
            }

            if (!int.TryParse(header[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out int version) || version != Version)
            {
                throw new InvalidDataException($"{path}: unsupported trace format version: {header[1]}");
            }

            if (!long.TryParse(header[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out long ticksPerSecond))
            {
                throw new InvalidDataException($"{path}: invalid ticksPerSecond in header: {header[2]}");
            }

            var samples = new long[lines.Length - 1];
            for (int i = 1; i < lines.Length; i++)
            {
                if (!long.TryParse(lines[i], NumberStyles.Integer, CultureInfo.InvariantCulture, out long sample))
                {
                    throw new InvalidDataException($"{path}: line {i + 1} is not a valid tick integer: {lines[i]}");
                }

                samples[i - 1] = sample;
            }

            return (ticksPerSecond, samples);
        }
    }
}
