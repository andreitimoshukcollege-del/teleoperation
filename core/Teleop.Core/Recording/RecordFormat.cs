using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Recording
{
    /// <summary>
    /// The versioned <c>.tlog</c> spec: one record per line, terminated by <c>\n</c> (added by
    /// the caller — see <see cref="SessionWriter"/>/<see cref="SessionReader"/>, neither of which
    /// touches line endings), fields separated by <see cref="Delimiter"/>. Text, not binary:
    /// <c>.gitattributes</c> already declares <c>*.tlog text eol=lf</c> grouped with
    /// <c>*.csv</c> under "Research data is text — diff it," which is first-party evidence this
    /// format is meant to be line-oriented and human-diffable, not packed binary.
    ///
    /// The newline is sufficient framing on its own — no length prefix is needed once the format
    /// is textual.
    ///
    /// <see cref="UnsetToken"/> is written in place of a tick value that is unset (matching
    /// <c>LatencyTrace.Unset</c> exactly, so a diff shows the word "unset" as plain text instead
    /// of an opaque sentinel integer that happens to be <c>long.MinValue</c>).
    ///
    /// Numeric formatting: <c>float</c> fields use the <c>G9</c> format specifier, which
    /// guarantees an exact round trip for <see cref="float"/> per .NET's own documented
    /// guidance; integers are plain decimal. Both are written/read via
    /// <see cref="System.Buffers.Text.Utf8Formatter"/>/<see cref="System.Buffers.Text.Utf8Parser"/>
    /// — allocation-free, no NuGet dependency, already part of <c>netstandard2.1</c>.
    ///
    /// The FNV-1a fold function lives here rather than in <see cref="SessionWriter"/> or
    /// <see cref="SessionReader"/> because the checksum algorithm is a spec detail like the
    /// delimiter or the unset token — both the writer and the reader must agree on exactly the
    /// same function, so it is defined once, here.
    /// </summary>
    public static class RecordFormat
    {
        /// <summary>Current format version. A reader must reject any other version outright.</summary>
        public const int Version = 1;

        public const string HeaderTag = "TLOG";
        public const string CommandFrameTag = "CF";
        public const string StampedPoseTag = "SP";
        public const string LatencyTraceTag = "LT";
        public const string EndOfSessionTag = "EOS";

        public const byte Delimiter = (byte)'|';

        /// <summary>Written in place of a <c>LatencyTrace</c> tick field that is unset.</summary>
        public const byte UnsetToken = (byte)'_';

        /// <summary>Conservative upper bound on any single line this format produces.</summary>
        public const int MaxLineBytes = 512;

        public const int MaxHeaderLineBytes = 96;
        public const int MaxCommandFrameLineBytes = 400;
        public const int MaxStampedPoseLineBytes = 200;
        public const int MaxLatencyTraceLineBytes = 300;
        public const int MaxEndOfSessionLineBytes = 32;

        private const ulong FnvOffsetBasisValue = 0xcbf29ce484222325UL;
        private const ulong FnvPrime = 0x100000001b3UL;

        /// <summary>The FNV-1a starting value, before any bytes have been folded in.</summary>
        public const ulong FnvOffsetBasis = FnvOffsetBasisValue;

        /// <summary>
        /// Folds <paramref name="bytes"/> into <paramref name="hash"/> using FNV-1a. Called once
        /// per line (in session order, header through the last data line, excluding the trailer
        /// line itself) by both <see cref="SessionWriter"/> and whatever drives
        /// <see cref="SessionReader"/>, so that a truncated or corrupted <c>.tlog</c> is
        /// detectable by comparing the reader's accumulated value against the trailer.
        /// Allocation-free.
        /// </summary>
        public static ulong FoldFnv1a(ulong hash, ReadOnlySpan<byte> bytes)
        {
            for (int i = 0; i < bytes.Length; i++)
            {
                hash ^= bytes[i];
                hash = unchecked(hash * FnvPrime);
            }

            return hash;
        }
    }
}
