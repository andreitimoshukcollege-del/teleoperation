using System;
using System.Buffers.Text;
using System.Numerics;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Recording
{
    /// <summary>
    /// Decodes one already-line-split <c>.tlog</c> line at a time from a caller-supplied
    /// <see cref="ReadOnlySpan{T}"/> (no trailing <c>\n</c> — the caller already split on it),
    /// per <see cref="RecordFormat"/>. Mirrors <c>ICommandCodec.TryDecode</c>'s "reject rather
    /// than throw" contract exactly: a malformed, wrong-tag, or truncated line returns false
    /// instead of throwing, since a hand-corrupted or partially-written <c>.tlog</c> is a
    /// legitimate real-world input this type has to survive.
    ///
    /// Unrecognized tags within an otherwise-supported format version are the caller's concern
    /// (the line-reading loop, e.g. <c>Teleop.Eval</c>'s <c>TlogFileReader</c>), not this type's —
    /// each <c>TryReadX</c> only ever sees one line already identified as (probably) that kind,
    /// which is what lets a reader tolerate a newer file gaining a record kind it doesn't
    /// understand yet, by skipping it, without this type needing forward knowledge of tags that
    /// don't exist.
    ///
    /// Checksum accounting is explicit, unlike <see cref="SessionWriter"/>'s automatic
    /// accumulation: call <see cref="AccumulateChecksum"/> with every line's raw bytes, in
    /// session order, including lines whose tag the caller does not recognize and chooses to
    /// skip — the checksum must cover the whole byte stream to detect truncation, not just the
    /// lines this reader happened to successfully decode.
    /// </summary>
    public sealed class SessionReader
    {
        private ulong _checksum;

        public SessionReader()
        {
            _checksum = RecordFormat.FnvOffsetBasis;
        }

        /// <summary>Folds one line's raw bytes into the running checksum. See type-level remarks.</summary>
        public void AccumulateChecksum(ReadOnlySpan<byte> lineBytes)
        {
            _checksum = RecordFormat.FoldFnv1a(_checksum, lineBytes);
        }

        /// <summary>Decodes the header line. See <see cref="SessionOpenResult"/> for the outcomes.</summary>
        public SessionOpenResult TryReadHeader(ReadOnlySpan<byte> source, out long ticksPerSecond, out ulong randomSeed)
        {
            ticksPerSecond = 0;
            randomSeed = 0;

            int pos = 0;
            if (!TryConsumeTag(source, RecordFormat.HeaderTag, ref pos))
            {
                return SessionOpenResult.BadTag;
            }

            if (!TryReadInt(source, ref pos, isLast: false, out int version))
            {
                return SessionOpenResult.BadTag;
            }

            if (version != RecordFormat.Version)
            {
                return SessionOpenResult.UnsupportedVersion;
            }

            if (!TryReadLong(source, ref pos, isLast: false, out ticksPerSecond))
            {
                return SessionOpenResult.BadTag;
            }

            if (!TryReadULong(source, ref pos, isLast: true, out randomSeed))
            {
                return SessionOpenResult.BadTag;
            }

            return SessionOpenResult.Ok;
        }

        /// <summary>Decodes one <see cref="CommandFrame"/> line.</summary>
        public bool TryReadCommandFrame(ReadOnlySpan<byte> source, out CommandFrame frame)
        {
            frame = default;
            int pos = 0;

            if (!TryConsumeTag(source, RecordFormat.CommandFrameTag, ref pos)) return false;
            if (!TryReadUInt(source, ref pos, false, out uint sequence)) return false;
            if (!TryReadUInt(source, ref pos, false, out uint ackSequence)) return false;
            if (!TryReadLong(source, ref pos, false, out long captureTicks)) return false;
            if (!TryReadPose(source, ref pos, out Pose pose)) return false;
            if (!TryReadVector3(source, ref pos, false, out Vector3 linearVelocity)) return false;
            if (!TryReadVector3(source, ref pos, false, out Vector3 angularVelocity)) return false;
            if (!TryReadFloat(source, ref pos, true, out float gripper)) return false;

            frame = new CommandFrame(sequence, ackSequence, captureTicks, pose, linearVelocity, angularVelocity, gripper);
            return true;
        }

        /// <summary>Decodes one <see cref="Stamped{Pose}"/> line.</summary>
        public bool TryReadStampedPose(ReadOnlySpan<byte> source, out Stamped<Pose> sample)
        {
            sample = default;
            int pos = 0;

            if (!TryConsumeTag(source, RecordFormat.StampedPoseTag, ref pos)) return false;
            if (!TryReadLong(source, ref pos, false, out long captureTicks)) return false;
            if (!TryReadPoseLast(source, ref pos, out Pose pose)) return false;

            sample = new Stamped<Pose>(captureTicks, pose);
            return true;
        }

        /// <summary>Decodes one <see cref="LatencyTrace"/> line.</summary>
        public bool TryReadLatencyTrace(ReadOnlySpan<byte> source, out LatencyTrace trace)
        {
            trace = default;
            int pos = 0;

            if (!TryConsumeTag(source, RecordFormat.LatencyTraceTag, ref pos)) return false;
            if (!TryReadUInt(source, ref pos, false, out uint sequence)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long captureTicks)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long uplinkSend)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long robotRecv)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long downlinkSend)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long operatorRecv)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long playout)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long render)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long photon)) return false;
            if (!TryReadUnsetableLong(source, ref pos, false, out long clockOffset)) return false;
            if (!TryReadUnsetableLong(source, ref pos, true, out long clockOffsetUncertainty)) return false;

            trace = LatencyTrace.ForSequence(sequence)
                .WithCaptureTicks(captureTicks)
                .WithUplinkSendTicks(uplinkSend)
                .WithRobotRecvTicks(robotRecv)
                .WithDownlinkSendTicks(downlinkSend)
                .WithOperatorRecvTicks(operatorRecv)
                .WithPlayoutTicks(playout)
                .WithRenderTicks(render)
                .WithPhotonTicks(photon)
                .WithClockSync(clockOffset, clockOffsetUncertainty);
            return true;
        }

        /// <summary>Decodes the trailer line and reports the checksum it carries.</summary>
        public bool TryReadEndOfSession(ReadOnlySpan<byte> source, out ulong checksum)
        {
            checksum = 0;
            int pos = 0;

            if (!TryConsumeTag(source, RecordFormat.EndOfSessionTag, ref pos)) return false;
            if (!TryReadHexULong(source, ref pos, true, out checksum)) return false;
            return true;
        }

        /// <summary>
        /// True if the checksum accumulated so far via <see cref="AccumulateChecksum"/> matches
        /// <paramref name="expectedChecksum"/> (the value decoded from the trailer).
        /// </summary>
        public bool TryVerifyChecksum(ulong expectedChecksum) => _checksum == expectedChecksum;

        /// <summary>Returns the reader to its as-constructed state: checksum cleared.</summary>
        public void Reset()
        {
            _checksum = RecordFormat.FnvOffsetBasis;
        }

        private static bool TryConsumeTag(ReadOnlySpan<byte> source, string tag, ref int pos)
        {
            if (source.Length < tag.Length + 1)
            {
                return false;
            }

            for (int i = 0; i < tag.Length; i++)
            {
                if (source[i] != (byte)tag[i])
                {
                    return false;
                }
            }

            if (source[tag.Length] != RecordFormat.Delimiter)
            {
                return false;
            }

            pos = tag.Length + 1;
            return true;
        }

        private static bool TryReadToken(ReadOnlySpan<byte> source, ref int pos, bool isLast, out ReadOnlySpan<byte> token)
        {
            if (pos > source.Length)
            {
                token = default;
                return false;
            }

            ReadOnlySpan<byte> remaining = source.Slice(pos);

            if (isLast)
            {
                token = remaining;
                pos = source.Length + 1;
                return true;
            }

            int idx = remaining.IndexOf(RecordFormat.Delimiter);
            if (idx < 0)
            {
                token = default;
                return false;
            }

            token = remaining.Slice(0, idx);
            pos += idx + 1;
            return true;
        }

        private static bool TryReadLong(ReadOnlySpan<byte> source, ref int pos, bool isLast, out long value)
        {
            if (!TryReadToken(source, ref pos, isLast, out ReadOnlySpan<byte> token) ||
                !Utf8Parser.TryParse(token, out value, out int consumed) || consumed != token.Length)
            {
                value = 0;
                return false;
            }

            return true;
        }

        private static bool TryReadUnsetableLong(ReadOnlySpan<byte> source, ref int pos, bool isLast, out long value)
        {
            if (!TryReadToken(source, ref pos, isLast, out ReadOnlySpan<byte> token))
            {
                value = 0;
                return false;
            }

            if (token.Length == 1 && token[0] == RecordFormat.UnsetToken)
            {
                value = LatencyTrace.Unset;
                return true;
            }

            if (!Utf8Parser.TryParse(token, out value, out int consumed) || consumed != token.Length)
            {
                value = 0;
                return false;
            }

            return true;
        }

        private static bool TryReadULong(ReadOnlySpan<byte> source, ref int pos, bool isLast, out ulong value)
        {
            if (!TryReadToken(source, ref pos, isLast, out ReadOnlySpan<byte> token) ||
                !Utf8Parser.TryParse(token, out value, out int consumed) || consumed != token.Length)
            {
                value = 0;
                return false;
            }

            return true;
        }

        private static bool TryReadHexULong(ReadOnlySpan<byte> source, ref int pos, bool isLast, out ulong value)
        {
            if (!TryReadToken(source, ref pos, isLast, out ReadOnlySpan<byte> token) ||
                !Utf8Parser.TryParse(token, out value, out int consumed, 'X') || consumed != token.Length)
            {
                value = 0;
                return false;
            }

            return true;
        }

        private static bool TryReadUInt(ReadOnlySpan<byte> source, ref int pos, bool isLast, out uint value)
        {
            if (!TryReadToken(source, ref pos, isLast, out ReadOnlySpan<byte> token) ||
                !Utf8Parser.TryParse(token, out value, out int consumed) || consumed != token.Length)
            {
                value = 0;
                return false;
            }

            return true;
        }

        private static bool TryReadInt(ReadOnlySpan<byte> source, ref int pos, bool isLast, out int value)
        {
            if (!TryReadToken(source, ref pos, isLast, out ReadOnlySpan<byte> token) ||
                !Utf8Parser.TryParse(token, out value, out int consumed) || consumed != token.Length)
            {
                value = 0;
                return false;
            }

            return true;
        }

        private static bool TryReadFloat(ReadOnlySpan<byte> source, ref int pos, bool isLast, out float value)
        {
            if (!TryReadToken(source, ref pos, isLast, out ReadOnlySpan<byte> token) ||
                !Utf8Parser.TryParse(token, out value, out int consumed) || consumed != token.Length)
            {
                value = 0f;
                return false;
            }

            return true;
        }

        private static bool TryReadVector3(ReadOnlySpan<byte> source, ref int pos, bool isLast, out Vector3 value)
        {
            value = default;
            if (!TryReadFloat(source, ref pos, false, out float x)) return false;
            if (!TryReadFloat(source, ref pos, false, out float y)) return false;
            if (!TryReadFloat(source, ref pos, isLast, out float z)) return false;

            value = new Vector3(x, y, z);
            return true;
        }

        private static bool TryReadPose(ReadOnlySpan<byte> source, ref int pos, out Pose value)
        {
            value = default;
            if (!TryReadVector3(source, ref pos, false, out Vector3 position)) return false;
            if (!TryReadFloat(source, ref pos, false, out float rx)) return false;
            if (!TryReadFloat(source, ref pos, false, out float ry)) return false;
            if (!TryReadFloat(source, ref pos, false, out float rz)) return false;
            if (!TryReadFloat(source, ref pos, false, out float rw)) return false;

            value = new Pose(position, new Quaternion(rx, ry, rz, rw));
            return true;
        }

        private static bool TryReadPoseLast(ReadOnlySpan<byte> source, ref int pos, out Pose value)
        {
            value = default;
            if (!TryReadVector3(source, ref pos, false, out Vector3 position)) return false;
            if (!TryReadFloat(source, ref pos, false, out float rx)) return false;
            if (!TryReadFloat(source, ref pos, false, out float ry)) return false;
            if (!TryReadFloat(source, ref pos, false, out float rz)) return false;
            if (!TryReadFloat(source, ref pos, true, out float rw)) return false;

            value = new Pose(position, new Quaternion(rx, ry, rz, rw));
            return true;
        }
    }
}
