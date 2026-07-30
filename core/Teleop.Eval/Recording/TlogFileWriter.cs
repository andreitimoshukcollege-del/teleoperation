using System;
using System.IO;
using System.Text;
using Teleop.Core.Recording;
using Teleop.Core.Types;

namespace Teleop.Eval.Recording
{
    /// <summary>
    /// Owns the actual <c>.tlog</c> file handle -- the thing <c>Teleop.Core.Recording</c> is
    /// forbidden from touching. Calls into <see cref="SessionWriter"/> for encoding each line,
    /// writes it, and moves on. ASCII encoding, explicit "\n" line endings, matching
    /// <c>.gitattributes</c>' <c>*.tlog text eol=lf</c> declaration exactly (no BOM, no CRLF).
    /// </summary>
    public sealed class TlogFileWriter : IDisposable
    {
        private readonly StreamWriter _stream;
        private readonly SessionWriter _writer;
        private readonly byte[] _buffer;

        public TlogFileWriter(string path)
        {
            _stream = new StreamWriter(path, append: false, Encoding.ASCII) { NewLine = "\n" };
            _writer = new SessionWriter();
            _buffer = new byte[RecordFormat.MaxLineBytes];
        }

        public void WriteHeader(long ticksPerSecond, ulong randomSeed)
        {
            if (!_writer.TryWriteHeader(ticksPerSecond, randomSeed, _buffer, out int n))
            {
                throw new InvalidOperationException("header line exceeded RecordFormat.MaxLineBytes");
            }

            WriteLine(n);
        }

        public void WriteCommandFrame(in CommandFrame frame)
        {
            if (!_writer.TryWriteCommandFrame(frame, _buffer, out int n))
            {
                throw new InvalidOperationException("CommandFrame line exceeded RecordFormat.MaxLineBytes");
            }

            WriteLine(n);
        }

        public void WriteStampedPose(in Stamped<Pose> sample)
        {
            if (!_writer.TryWriteStampedPose(sample, _buffer, out int n))
            {
                throw new InvalidOperationException("StampedPose line exceeded RecordFormat.MaxLineBytes");
            }

            WriteLine(n);
        }

        public void WriteLatencyTrace(in LatencyTrace trace)
        {
            if (!_writer.TryWriteLatencyTrace(trace, _buffer, out int n))
            {
                throw new InvalidOperationException("LatencyTrace line exceeded RecordFormat.MaxLineBytes");
            }

            WriteLine(n);
        }

        public void WriteEndOfSession()
        {
            if (!_writer.TryWriteEndOfSession(_buffer, out int n))
            {
                throw new InvalidOperationException("EndOfSession line exceeded RecordFormat.MaxLineBytes");
            }

            WriteLine(n);
        }

        private void WriteLine(int length)
        {
            _stream.WriteLine(Encoding.ASCII.GetString(_buffer, 0, length));
        }

        public void Dispose()
        {
            _stream.Flush();
            _stream.Dispose();
        }
    }
}
