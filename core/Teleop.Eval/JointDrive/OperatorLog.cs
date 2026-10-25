using System;
using System.Globalization;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// <c>operator.jlog</c> (trainer/contract/SESSION_FORMAT.md, "Operator records"): every joint
    /// command sent, on this machine's clock, whether or not it reached the robot. The header names
    /// the clock <c>OPERATOR</c>; the trailer carries the record count, no drops (it writes
    /// synchronously) and a SHA-256 of every byte before it. Floats are round-trip, invariant culture,
    /// as RobotHost's <c>SessionLines</c> writes them.
    /// </summary>
    public sealed class OperatorLog : IDisposable
    {
        private readonly Stream _stream;
        private readonly IncrementalHash _hash = IncrementalHash.CreateHash(HashAlgorithmName.SHA256);
        private readonly StringBuilder _line = new StringBuilder(160);
        private long _records;
        private bool _closed;

        public OperatorLog(Stream stream, long ticksPerSecond, string sessionId)
        {
            if (sessionId.IndexOfAny(new[] { '|', '\n' }) >= 0) throw new ArgumentException("the session id may not hold | or a newline");
            _stream = stream;
            Write("JSES|1|operator|" + ticksPerSecond.ToString(CultureInfo.InvariantCulture) + "|OPERATOR|" + sessionId);
        }

        /// <summary><c>OC</c>: command <paramref name="sequence"/> sent at <paramref name="t"/>, stamped <paramref name="captureTicks"/>.</summary>
        public void Command(long t, uint sequence, long captureTicks, ReadOnlySpan<JointTarget> targets)
        {
            _line.Clear();
            _line.Append("OC|").Append(t.ToString(CultureInfo.InvariantCulture)).Append('|')
                .Append(sequence.ToString(CultureInfo.InvariantCulture)).Append('|')
                .Append(captureTicks.ToString(CultureInfo.InvariantCulture)).Append('|');
            for (int i = 0; i < targets.Length; i++)
            {
                if (i > 0) _line.Append(';');
                _line.Append(targets[i].MotorId.ToString(CultureInfo.InvariantCulture)).Append(':')
                    .Append(targets[i].Angle.ToString("R", CultureInfo.InvariantCulture)).Append(':')
                    .Append(targets[i].Speed.ToString("R", CultureInfo.InvariantCulture));
            }

            Write(_line.ToString());
            _records++;
        }

        /// <summary>Writes the trailer. Without it the file reads as cut off, and the session check fails it.</summary>
        public void Close()
        {
            if (_closed) return;
            _closed = true;
            string sha = Convert.ToHexString(_hash.GetHashAndReset()).ToLowerInvariant();
            byte[] trailer = Encoding.ASCII.GetBytes("END|" + _records.ToString(CultureInfo.InvariantCulture) + "|0|" + sha + "\n");
            _stream.Write(trailer, 0, trailer.Length);
            _stream.Flush();
        }

        public void Dispose()
        {
            Close();
            _hash.Dispose();
            _stream.Dispose();
        }

        private void Write(string line)
        {
            byte[] bytes = Encoding.ASCII.GetBytes(line + "\n");
            _hash.AppendData(bytes);
            _stream.Write(bytes, 0, bytes.Length);
        }
    }
}
