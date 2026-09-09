// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// What is about to happen to one datagram, accumulated across the impairments applied to it.
    ///
    /// A <b>mutable</b> struct, which <c>Types/</c> otherwise avoids. The exception is deliberate
    /// and has precedent here: <see cref="SeededRng"/> is also a mutable struct in this folder, for
    /// the same reason — it exists to be written by successive calls, and copying it per call would
    /// be both slower and wrong. This is passed by <c>ref</c> to every impairment, never returned by
    /// value, so no impairment can silently discard what an earlier one decided.
    ///
    /// <b>Two rules make composition order-independent</b>, and every impairment must obey them:
    ///
    /// <list type="number">
    /// <item><see cref="DelayTicks"/> is <b>additive</b>. Write <c>fate.DelayTicks += x</c>, never
    /// <c>=</c>. Integer addition commutes, so the total does not depend on visit order.</item>
    /// <item><see cref="Dropped"/> is <b>monotone</b>. An impairment may set it true; nothing may
    /// set it back to false. Boolean OR commutes, so neither does this.</item>
    /// </list>
    ///
    /// Together those are why <c>EmulatedTransport</c> can iterate its impairment array in any order
    /// and why permuting the array is not a configuration change. An impairment that cannot be
    /// written within these rules is order-sensitive, which makes array order part of the
    /// configuration — legal, but it must say so in its own doc rather than depending on the
    /// emulator's current loop.
    ///
    /// <b>Deliberately absent:</b> send tick, sequence number, payload length. No impairment shipped
    /// today reads them, and "no invented knobs" (<c>Types/PredictorConfig.cs</c>'s reasoning)
    /// applies to struct fields too. Adding one later is source-compatible for every impairment that
    /// ignores it.
    /// </summary>
    public struct DatagramFate
    {
        /// <summary>
        /// Total delay to add to this datagram's arrival, in ticks on the host's
        /// <c>ITimeAuthority</c> timebase. Accumulated additively; see the type doc.
        ///
        /// May go negative in intermediate states — a jitter impairment draws from
        /// <c>[-halfWidth, +halfWidth]</c> and may run before any delay is added. The emulator
        /// clamps the total to zero after the whole pipeline has run, so a datagram is never
        /// delivered before it was sent, and clamping happens once rather than per impairment.
        /// </summary>
        public long DelayTicks;

        /// <summary>
        /// Whether this datagram is lost. Monotone; see the type doc.
        ///
        /// Set at the <c>Send</c> stage, this means the datagram never reaches the wrapped transport
        /// at all, which is what a real link does with a lost packet and what
        /// <c>ITransport.Send</c>'s "returns false when the datagram will not be delivered" promises.
        /// </summary>
        public bool Dropped;
    }
}
