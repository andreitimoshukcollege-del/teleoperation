// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Recording
{
    /// <summary>
    /// Outcome of <c>SessionReader.TryReadHeader</c>. A bare <c>bool</c> cannot distinguish "this
    /// is not a <c>.tlog</c> header line at all" from "it is, but a version this reader does not
    /// support" — callers need that distinction to report a useful error rather than a generic
    /// parse failure.
    /// </summary>
    public enum SessionOpenResult
    {
        /// <summary>Header parsed and its version is supported.</summary>
        Ok,

        /// <summary>The line does not have the expected header tag/shape at all.</summary>
        BadTag,

        /// <summary>The line is a well-formed header, but for a format version this reader does not support.</summary>
        UnsupportedVersion,
    }
}
