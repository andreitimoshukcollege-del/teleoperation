using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// Which points in a datagram's life an impairment participates in.
    ///
    /// The transport has genuinely two, and this is not a stylistic split. Loss is decided when a
    /// datagram is <b>sent</b>, so a lost one never reaches the wrapped transport — a real link does
    /// not put a dropped packet on the wire, and <c>ITransport.Send</c> returning false is part of
    /// its contract. Delay is applied when a datagram is <b>delivered</b>, because
    /// <c>ITransport.Send</c> has no future-delivery parameter and Core has no threads, so the only
    /// place a synthetic arrival time can be honoured is the drain.
    ///
    /// An impairment declares its stages so <c>EmulatedTransport</c> can partition its array once at
    /// construction. A delay impairment then costs nothing on the send path, rather than costing a
    /// virtual call into an empty method on every datagram.
    /// </summary>
    [Flags]
    public enum ImpairmentStages
    {
        /// <summary>Participates in neither stage. Legal but pointless; a diagnostic value.</summary>
        None = 0,

        /// <summary>Runs inside <c>ITransport.Send</c>. May set <c>DatagramFate.Dropped</c>.</summary>
        Send = 1 << 0,

        /// <summary>Runs at drain time. May add to <c>DatagramFate.DelayTicks</c>.</summary>
        Deliver = 1 << 1,
    }
}
