using System;
using System.IO;
using System.Text;
using Teleop.Core.Recording;

namespace Teleop.Eval.Verification
{
    /// <summary>
    /// Implements <c>verify</c> exactly as scoped by root CLAUDE.md: "replay a golden log twice;
    /// assert identical." Concretely: decode every line of the committed golden <c>.tlog</c> and
    /// re-encode it, twice, independently; assert the two replays are byte-identical (the literal
    /// Gate 3 requirement), and -- a stricter, nearly-free check given the above -- that the
    /// first replay also matches the original file exactly. A lossless text format with no
    /// re-quantization anywhere should reproduce its own input on a single decode-encode pass; if
    /// it doesn't, that is a distinct format bug worth catching on its own.
    /// </summary>
    public static class VerifyCommand
    {
        public static int Run()
        {
            string goldenPath = Path.Combine(AppContext.BaseDirectory, "testdata", "golden", "basic-session.tlog");

            if (!File.Exists(goldenPath))
            {
                Console.Error.WriteLine($"verify: golden log not found at {goldenPath}");
                return 66; // EX_NOINPUT
            }

            byte[] original = File.ReadAllBytes(goldenPath);

            if (!TryReplay(goldenPath, out byte[] replayed1, out string? error1))
            {
                Console.Error.WriteLine($"verify: replay pass 1 failed: {error1}");
                return 1;
            }

            if (!TryReplay(goldenPath, out byte[] replayed2, out string? error2))
            {
                Console.Error.WriteLine($"verify: replay pass 2 failed: {error2}");
                return 1;
            }

            if (!SequenceEqualWithReport(replayed1, replayed2, "replay pass 1", "replay pass 2"))
            {
                return 1;
            }

            if (!SequenceEqualWithReport(replayed1, original, "replay pass 1", "the original golden file"))
            {
                return 1;
            }

            Console.WriteLine($"verify: PASS -- {goldenPath} replays byte-identical across two " +
                "independent passes, and matches the original file exactly.");
            return 0;
        }

        private static bool TryReplay(string path, out byte[] replayed, out string? error)
        {
            replayed = Array.Empty<byte>();
            error = null;

            string[] lines = File.ReadAllLines(path);
            if (lines.Length == 0)
            {
                error = "file is empty";
                return false;
            }

            var reader = new SessionReader();
            var writer = new SessionWriter();
            byte[] buffer = new byte[RecordFormat.MaxLineBytes];
            using var output = new MemoryStream();

            byte[] headerBytes = Encoding.ASCII.GetBytes(lines[0]);
            SessionOpenResult headerResult = reader.TryReadHeader(headerBytes, out long ticksPerSecond, out ulong seed);
            if (headerResult != SessionOpenResult.Ok)
            {
                error = $"header decode failed: {headerResult}";
                return false;
            }
            reader.AccumulateChecksum(headerBytes);

            if (!writer.TryWriteHeader(ticksPerSecond, seed, buffer, out int headerLen))
            {
                error = "header re-encode failed";
                return false;
            }
            AppendLine(output, buffer, headerLen);

            for (int i = 1; i < lines.Length; i++)
            {
                string line = lines[i];
                byte[] bytes = Encoding.ASCII.GetBytes(line);

                if (line.StartsWith(RecordFormat.CommandFrameTag + "|", StringComparison.Ordinal))
                {
                    if (!reader.TryReadCommandFrame(bytes, out var frame)) { error = $"line {i}: CommandFrame decode failed: {line}"; return false; }
                    if (!writer.TryWriteCommandFrame(frame, buffer, out int n)) { error = $"line {i}: CommandFrame re-encode failed"; return false; }
                    AppendLine(output, buffer, n);
                    reader.AccumulateChecksum(bytes);
                }
                else if (line.StartsWith(RecordFormat.StampedPoseTag + "|", StringComparison.Ordinal))
                {
                    if (!reader.TryReadStampedPose(bytes, out var sample)) { error = $"line {i}: StampedPose decode failed: {line}"; return false; }
                    if (!writer.TryWriteStampedPose(sample, buffer, out int n)) { error = $"line {i}: StampedPose re-encode failed"; return false; }
                    AppendLine(output, buffer, n);
                    reader.AccumulateChecksum(bytes);
                }
                else if (line.StartsWith(RecordFormat.LatencyTraceTag + "|", StringComparison.Ordinal))
                {
                    if (!reader.TryReadLatencyTrace(bytes, out var trace)) { error = $"line {i}: LatencyTrace decode failed: {line}"; return false; }
                    if (!writer.TryWriteLatencyTrace(trace, buffer, out int n)) { error = $"line {i}: LatencyTrace re-encode failed"; return false; }
                    AppendLine(output, buffer, n);
                    reader.AccumulateChecksum(bytes);
                }
                else if (line.StartsWith(RecordFormat.EndOfSessionTag + "|", StringComparison.Ordinal))
                {
                    if (!reader.TryReadEndOfSession(bytes, out ulong checksum)) { error = $"line {i}: EndOfSession decode failed: {line}"; return false; }
                    if (!reader.TryVerifyChecksum(checksum))
                    {
                        error = $"line {i}: checksum mismatch -- golden file itself is internally inconsistent";
                        return false;
                    }
                    if (!writer.TryWriteEndOfSession(buffer, out int n)) { error = $"line {i}: EndOfSession re-encode failed"; return false; }
                    AppendLine(output, buffer, n);
                    // The trailer line is never folded into the checksum -- it carries the
                    // checksum of everything before it, not of itself.
                }
                else
                {
                    error = $"line {i}: unrecognized tag: {line}";
                    return false;
                }
            }

            replayed = output.ToArray();
            return true;
        }

        private static void AppendLine(MemoryStream output, byte[] buffer, int length)
        {
            output.Write(buffer, 0, length);
            output.WriteByte((byte)'\n');
        }

        private static bool SequenceEqualWithReport(byte[] a, byte[] b, string aName, string bName)
        {
            if (a.Length == b.Length && a.AsSpan().SequenceEqual(b))
            {
                return true;
            }

            int minLen = Math.Min(a.Length, b.Length);
            int firstDiff = minLen;
            for (int i = 0; i < minLen; i++)
            {
                if (a[i] != b[i])
                {
                    firstDiff = i;
                    break;
                }
            }

            Console.Error.WriteLine(
                $"verify: {aName} ({a.Length} bytes) differs from {bName} ({b.Length} bytes) " +
                $"at byte offset {firstDiff}.");
            return false;
        }
    }
}
