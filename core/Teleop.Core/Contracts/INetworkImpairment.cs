using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// One axis of network impairment: delay, jitter, loss, reordering, or a recorded delay trace.
    ///
    /// An <c>EmulatedTransport</c> holds an array of these and applies them to every datagram. The
    /// set <b>is</b> the configuration — there is no profile struct behind it, and no name. A host
    /// that wants 200 ms of lag constructs a delay impairment; a sweep that wants a frozen benchmark
    /// row asks <c>NetworkProfileCatalog</c> for that name's set. Neither is privileged, and adding
    /// a new kind of impairment is a new file implementing this interface rather than a widening of
    /// a struct that every caller has to be edited for (docs/adr/0013).
    ///
    /// <b>Implementations describe; they do not decide policy.</b> An impairment owns its own
    /// parameters and its own random substream, and its whole job is to write into a
    /// <see cref="DatagramFate"/>. It does not know what other impairments exist, what order it runs
    /// in, or which transport owns it.
    ///
    /// <b>Order independence is a requirement, not a convention.</b> See <see cref="DatagramFate"/>:
    /// delay is additive and <c>Dropped</c> is monotone. An implementation that cannot obey those
    /// makes array order part of the configuration and must document that explicitly.
    ///
    /// <b>Determinism contract.</b> Every impairment draws only from the private substream
    /// <see cref="Bind"/> gives it, and must consume a <b>constant number of draws per datagram per
    /// stage</b>, fixed by its type and independent of its parameter values — including at neutral
    /// values. That is what makes an axis at zero observationally identical to its absence, which in
    /// turn is what lets the catalog emit only non-zero axes while reproducing the frozen profiles
    /// exactly. It also means adding or removing an axis cannot perturb any other axis's
    /// realization, which the previous shared-stream design could not offer.
    ///
    /// <b>Hot path.</b> <see cref="ApplyOnSend"/> and <see cref="ApplyOnDeliver"/> run once per
    /// datagram and must allocate nothing. No LINQ, no closures, no string work, no boxing.
    /// Implementations are reference types so that storing them in an array does not box, and are
    /// not thread-safe — the same contract <c>ITransport</c> itself sets.
    /// </summary>
    public interface INetworkImpairment
    {
        /// <summary>
        /// Stable ordinal identifier for this axis: <c>delay</c>, <c>jitter</c>, <c>loss</c>,
        /// <c>reorder</c>, <c>delay-trace</c>.
        ///
        /// This is frozen data, not a display string. It seeds this impairment's RNG substream, and
        /// it is what a manifest records, so <b>renaming one changes every stream it produces</b>
        /// and silently makes new runs incomparable with old ones. A test pins the derived
        /// identities to hardcoded literals precisely so a rename fails loudly.
        ///
        /// <c>EmulatedTransport</c> rejects two impairments sharing a name, which is what makes
        /// name-derived substream identity collision-proof.
        /// </summary>
        string AxisName { get; }

        /// <summary>
        /// Which stages this impairment participates in. Read once, at construction, and used to
        /// partition the transport's array — an impairment is never called at a stage it did not
        /// declare.
        /// </summary>
        ImpairmentStages Stages { get; }

        /// <summary>
        /// Hands this impairment its private RNG substream seed. Called exactly once, by the
        /// transport taking ownership.
        ///
        /// Implementations must throw <c>InvalidOperationException</c> on a second call. An
        /// impairment carries mutable model state — a Gilbert-Elliott chain bit, a trace cursor —
        /// so sharing one instance between the uplink and downlink transports would correlate the
        /// two directions in a way no real link does. Failing loudly beats producing a plausible
        /// wrong result.
        /// </summary>
        void Bind(ulong substreamSeed);

        /// <summary>
        /// Applies this impairment at send time. Called only when <see cref="Stages"/> includes
        /// <see cref="ImpairmentStages.Send"/>. Allocation-free.
        /// </summary>
        void ApplyOnSend(ref DatagramFate fate);

        /// <summary>
        /// Applies this impairment at delivery time. Called only when <see cref="Stages"/> includes
        /// <see cref="ImpairmentStages.Deliver"/>. Allocation-free.
        /// </summary>
        void ApplyOnDeliver(ref DatagramFate fate);

        /// <summary>
        /// Returns this impairment to its as-constructed state: model state cleared and the
        /// substream reseeded to the value <see cref="Bind"/> supplied, so the next trial reproduces
        /// the previous one. Does <b>not</b> unbind — the impairment still belongs to the same
        /// transport.
        /// </summary>
        void Reset();
    }
}
