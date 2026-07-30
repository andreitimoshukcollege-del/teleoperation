using System;
using System.Buffers;
using System.Buffers.Text;
using System.Numerics;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Recording
{
    /// <summary>
    /// Encodes one <c>.tlog</c> line at a time into a caller-owned <see cref="Span{T}"/>, per
    /// <see cref="RecordFormat"/>. Mirrors <c>ICommandCodec.TryEncode</c> exactly: returns false
    /// and reports a safe upper bound on the length required when the destination is too small,
    /// touching no destination bytes in that case, rather than writing a partial line. Never
    /// touches <c>System.IO</c> or a file handle — the actual <c>.tlog</c> file lives in
    /// <c>Teleop.Eval</c>, which owns a real file stream and calls into this type per line.
    ///
    /// Accumulates a running FNV-1a checksum over every line it successfully writes (header
    /// through the last data line), folded into the trailer by <see cref="TryWriteEndOfSession"/>.
    /// This happens automatically here — unlike <see cref="SessionReader"/>, which requires an
    /// explicit fold call, this type is never asked to write something it does not understand,
    /// so there is no forward-compatibility case to leave room for.
    /// </summary>
    public sealed class SessionWriter
    {
        private ulong _checksum;

        public SessionWriter()
        {
            _checksum = RecordFormat.FnvOffsetBasis;
        }

        /// <summary>
        /// <c>TLOG|&lt;version&gt;|&lt;ticksPerSecond&gt;|&lt;randomSeed&gt;</c>. Written once, first.
        /// </summary>
        public bool TryWriteHeader(long ticksPerSecond, ulong randomSeed, Span<byte> destination, out int bytesWritten)
        {
            if (destination.Length < RecordFormat.MaxHeaderLineBytes)
            {
                bytesWritten = RecordFormat.MaxHeaderLineBytes;
                return false;
            }

            int pos = 0;
            WriteTag(RecordFormat.HeaderTag, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteInt(RecordFormat.Version, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteLong(ticksPerSecond, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteULong(randomSeed, destination, ref pos);

            bytesWritten = pos;
            Accumulate(destination.Slice(0, pos));
            return true;
        }

        /// <summary>One <see cref="CommandFrame"/>, tag <c>CF</c>.</summary>
        public bool TryWriteCommandFrame(in CommandFrame frame, Span<byte> destination, out int bytesWritten)
        {
            if (destination.Length < RecordFormat.MaxCommandFrameLineBytes)
            {
                bytesWritten = RecordFormat.MaxCommandFrameLineBytes;
                return false;
            }

            int pos = 0;
            WriteTag(RecordFormat.CommandFrameTag, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteUInt(frame.Sequence, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteUInt(frame.AckSequence, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteLong(frame.CaptureTicks, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WritePose(frame.Pose, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteVector3(frame.LinearVelocity, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteVector3(frame.AngularVelocity, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteFloat(frame.Gripper, destination, ref pos);

            bytesWritten = pos;
            Accumulate(destination.Slice(0, pos));
            return true;
        }

        /// <summary>One <see cref="Stamped{Pose}"/> ground-truth sample, tag <c>SP</c>.</summary>
        public bool TryWriteStampedPose(in Stamped<Pose> sample, Span<byte> destination, out int bytesWritten)
        {
            if (destination.Length < RecordFormat.MaxStampedPoseLineBytes)
            {
                bytesWritten = RecordFormat.MaxStampedPoseLineBytes;
                return false;
            }

            int pos = 0;
            WriteTag(RecordFormat.StampedPoseTag, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteLong(sample.CaptureTicks, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WritePose(sample.Value, destination, ref pos);

            bytesWritten = pos;
            Accumulate(destination.Slice(0, pos));
            return true;
        }

        /// <summary>One <see cref="LatencyTrace"/>, tag <c>LT</c>. Every unset field is written as <see cref="RecordFormat.UnsetToken"/>.</summary>
        public bool TryWriteLatencyTrace(in LatencyTrace trace, Span<byte> destination, out int bytesWritten)
        {
            if (destination.Length < RecordFormat.MaxLatencyTraceLineBytes)
            {
                bytesWritten = RecordFormat.MaxLatencyTraceLineBytes;
                return false;
            }

            int pos = 0;
            WriteTag(RecordFormat.LatencyTraceTag, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteUInt(trace.Sequence, destination, ref pos);

            WriteUnsetableField(trace.TryGetCaptureTicks(out long captureTicks), captureTicks, destination, ref pos);
            WriteUnsetableField(trace.TryGetUplinkSendTicks(out long uplinkSend), uplinkSend, destination, ref pos);
            WriteUnsetableField(trace.TryGetRobotRecvTicks(out long robotRecv), robotRecv, destination, ref pos);
            WriteUnsetableField(trace.TryGetDownlinkSendTicks(out long downlinkSend), downlinkSend, destination, ref pos);
            WriteUnsetableField(trace.TryGetOperatorRecvTicks(out long operatorRecv), operatorRecv, destination, ref pos);
            WriteUnsetableField(trace.TryGetPlayoutTicks(out long playout), playout, destination, ref pos);
            WriteUnsetableField(trace.TryGetRenderTicks(out long render), render, destination, ref pos);
            WriteUnsetableField(trace.TryGetPhotonTicks(out long photon), photon, destination, ref pos);
            WriteUnsetableField(trace.TryGetClockOffsetTicks(out long clockOffset), clockOffset, destination, ref pos);
            WriteUnsetableField(trace.TryGetClockOffsetUncertaintyTicks(out long clockOffsetUncertainty), clockOffsetUncertainty, destination, ref pos);

            bytesWritten = pos;
            Accumulate(destination.Slice(0, pos));
            return true;
        }

        /// <summary>
        /// <c>EOS|&lt;checksum-hex&gt;</c>, the trailer. The checksum covers every line written so
        /// far via this instance (header and data lines) — not the trailer line itself.
        /// </summary>
        public bool TryWriteEndOfSession(Span<byte> destination, out int bytesWritten)
        {
            if (destination.Length < RecordFormat.MaxEndOfSessionLineBytes)
            {
                bytesWritten = RecordFormat.MaxEndOfSessionLineBytes;
                return false;
            }

            int pos = 0;
            WriteTag(RecordFormat.EndOfSessionTag, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteHexULong(_checksum, destination, ref pos);

            bytesWritten = pos;
            return true;
        }

        /// <summary>Returns the writer to its as-constructed state: checksum cleared.</summary>
        public void Reset()
        {
            _checksum = RecordFormat.FnvOffsetBasis;
        }

        private void Accumulate(ReadOnlySpan<byte> bytes) => _checksum = RecordFormat.FoldFnv1a(_checksum, bytes);

        private static void WriteDelimiter(Span<byte> destination, ref int pos)
        {
            destination[pos] = RecordFormat.Delimiter;
            pos++;
        }

        private static void WriteTag(string tag, Span<byte> destination, ref int pos)
        {
            for (int i = 0; i < tag.Length; i++)
            {
                destination[pos + i] = (byte)tag[i];
            }
            pos += tag.Length;
        }

        private static void WriteLong(long value, Span<byte> destination, ref int pos)
        {
            Utf8Formatter.TryFormat(value, destination.Slice(pos), out int written);
            pos += written;
        }

        private static void WriteULong(ulong value, Span<byte> destination, ref int pos)
        {
            Utf8Formatter.TryFormat(value, destination.Slice(pos), out int written);
            pos += written;
        }

        private static void WriteUInt(uint value, Span<byte> destination, ref int pos)
        {
            Utf8Formatter.TryFormat(value, destination.Slice(pos), out int written);
            pos += written;
        }

        private static void WriteInt(int value, Span<byte> destination, ref int pos)
        {
            Utf8Formatter.TryFormat(value, destination.Slice(pos), out int written);
            pos += written;
        }

        private static void WriteFloat(float value, Span<byte> destination, ref int pos)
        {
            Utf8Formatter.TryFormat(value, destination.Slice(pos), out int written, new StandardFormat('G', 9));
            pos += written;
        }

        private static void WriteHexULong(ulong value, Span<byte> destination, ref int pos)
        {
            Utf8Formatter.TryFormat(value, destination.Slice(pos), out int written, new StandardFormat('X', 16));
            pos += written;
        }

        private static void WriteUnsetableField(bool isSet, long value, Span<byte> destination, ref int pos)
        {
            WriteDelimiter(destination, ref pos);
            if (isSet)
            {
                WriteLong(value, destination, ref pos);
            }
            else
            {
                destination[pos] = RecordFormat.UnsetToken;
                pos++;
            }
        }

        private static void WriteVector3(Vector3 v, Span<byte> destination, ref int pos)
        {
            WriteFloat(v.X, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteFloat(v.Y, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteFloat(v.Z, destination, ref pos);
        }

        private static void WritePose(Pose pose, Span<byte> destination, ref int pos)
        {
            WriteVector3(pose.Position, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteFloat(pose.Rotation.X, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteFloat(pose.Rotation.Y, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteFloat(pose.Rotation.Z, destination, ref pos);
            WriteDelimiter(destination, ref pos);
            WriteFloat(pose.Rotation.W, destination, ref pos);
        }
    }
}
