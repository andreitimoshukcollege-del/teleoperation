using System;
using System.IO;
using System.Text;
using Teleop.Core.Recording;
using Teleop.Core.Types;

namespace Teleop.Eval.Recording
{
    /// <summary>
    /// Owns the actual <c>.tlog</c> file handle for reading. Reads line by line, dispatches each
    /// line's leading tag token to the matching <see cref="SessionReader"/> method, and folds
    /// every line -- including ones whose tag it doesn't recognize -- into the reader's checksum
    /// accumulator, so a truncated or corrupted file is still detectable end to end.
    /// </summary>
    public sealed class TlogFileReader : IDisposable
    {
        public enum RecordKind
        {
            CommandFrame,
            StampedPose,
            LatencyTrace,
            EndOfSession,
            Unrecognized,
            EndOfFile,
        }

        private readonly StreamReader _stream;
        private readonly SessionReader _reader;

        public TlogFileReader(string path)
        {
            _stream = new StreamReader(path, Encoding.ASCII);
            _reader = new SessionReader();
        }

        public SessionOpenResult ReadHeader(out long ticksPerSecond, out ulong randomSeed)
        {
            string? line = _stream.ReadLine();
            if (line == null)
            {
                ticksPerSecond = 0;
                randomSeed = 0;
                return SessionOpenResult.BadTag;
            }

            byte[] bytes = Encoding.ASCII.GetBytes(line);
            SessionOpenResult result = _reader.TryReadHeader(bytes, out ticksPerSecond, out randomSeed);
            if (result == SessionOpenResult.Ok)
            {
                _reader.AccumulateChecksum(bytes);
            }

            return result;
        }

        /// <summary>
        /// Reads and decodes the next line. <paramref name="endOfSessionChecksum"/> is only
        /// meaningful when the return value is <see cref="RecordKind.EndOfSession"/>.
        /// </summary>
        public RecordKind ReadNext(
            out CommandFrame commandFrame,
            out Stamped<Pose> stampedPose,
            out LatencyTrace latencyTrace,
            out ulong endOfSessionChecksum)
        {
            commandFrame = default;
            stampedPose = default;
            latencyTrace = default;
            endOfSessionChecksum = 0;

            string? line = _stream.ReadLine();
            if (line == null)
            {
                return RecordKind.EndOfFile;
            }

            byte[] bytes = Encoding.ASCII.GetBytes(line);

            if (line.StartsWith(RecordFormat.CommandFrameTag + "|", StringComparison.Ordinal))
            {
                _reader.AccumulateChecksum(bytes);
                return _reader.TryReadCommandFrame(bytes, out commandFrame)
                    ? RecordKind.CommandFrame : RecordKind.Unrecognized;
            }

            if (line.StartsWith(RecordFormat.StampedPoseTag + "|", StringComparison.Ordinal))
            {
                _reader.AccumulateChecksum(bytes);
                return _reader.TryReadStampedPose(bytes, out stampedPose)
                    ? RecordKind.StampedPose : RecordKind.Unrecognized;
            }

            if (line.StartsWith(RecordFormat.LatencyTraceTag + "|", StringComparison.Ordinal))
            {
                _reader.AccumulateChecksum(bytes);
                return _reader.TryReadLatencyTrace(bytes, out latencyTrace)
                    ? RecordKind.LatencyTrace : RecordKind.Unrecognized;
            }

            if (line.StartsWith(RecordFormat.EndOfSessionTag + "|", StringComparison.Ordinal))
            {
                // The EOS line itself is never folded into the checksum -- it carries the
                // checksum of everything before it, not of itself.
                return _reader.TryReadEndOfSession(bytes, out endOfSessionChecksum)
                    ? RecordKind.EndOfSession : RecordKind.Unrecognized;
            }

            // Unrecognized tag: fold it in anyway. A newer file may carry a record kind this
            // reader doesn't know about yet, and the checksum must still cover the whole stream.
            _reader.AccumulateChecksum(bytes);
            return RecordKind.Unrecognized;
        }

        public bool VerifyChecksum(ulong expected) => _reader.TryVerifyChecksum(expected);

        public void Dispose() => _stream.Dispose();
    }
}
