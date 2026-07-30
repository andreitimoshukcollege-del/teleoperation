using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Decides *when* a received sample becomes usable — the instant stamped <c>t_playout</c>
    /// in docs/metrics.md and in <see cref="LatencyTrace"/>. Implementations live in
    /// <c>Buffering/</c>.
    ///
    /// This is the axis that trades latency against loss: holding a sample longer absorbs more
    /// jitter and reordering before it must be consumed, at the cost of added delay every
    /// sample pays regardless of whether it needed the wait. A policy that does not report
    /// where it sits on that curve is not evaluable, which is why every field of
    /// <see cref="PlayoutPolicyDiagnostics"/> is mandatory rather than optional.
    ///
    /// Correlated by the same <c>Sequence</c> <see cref="LatencyTrace"/> uses, not by
    /// <see cref="Stamped{TState}.CaptureTicks"/> — for the same reason: a duplicate or
    /// retransmitted sample carries the same capture stamp as the original, and only a sequence
    /// number survives that. <see cref="TryDequeue"/> returns the sequence of the sample it
    /// releases specifically so the pipeline can attribute the exact playout instant back to
    /// the <see cref="LatencyTrace"/> for that sequence via <c>WithPlayoutTicks</c>.
    ///
    /// Receive is poll-based, matching <c>ITransport</c>: nothing here calls back, and the host
    /// drains ready samples once per step at a time it chooses, which is what keeps replay
    /// bit-identical.
    ///
    /// Contract every implementation owes its callers:
    /// <list type="number">
    /// <item>Deterministic and allocation-free. Preallocate the buffer from
    /// <see cref="PlayoutPolicyConfig.HistoryCapacity"/> in the constructor.</item>
    /// <item>Tolerates reordering and duplicates without corrupting playout order: samples are
    /// released in capture-time order regardless of arrival order, and a duplicate (same
    /// sequence enqueued twice) must not be released twice.</item>
    /// <item>Never starves silently. A buffer underrun — <see cref="TryDequeue"/> has nothing to
    /// give when the pipeline expected something — is counted in
    /// <see cref="PlayoutPolicyDiagnostics.UnderrunCount"/> and, per Buffering/CLAUDE.md, pushed
    /// to the <c>IMetricSink</c> injected at construction. The sink is a constructor dependency
    /// rather than a parameter here, the same reason <c>IReconciler</c> keeps it off its
    /// per-frame signature.</item>
    /// <item>Adaptive policies must not oscillate: a step change in one-way delay settles the
    /// delay budget within a stated bound rather than ringing.</item>
    /// </list>
    /// </summary>
    /// <typeparam name="TState">
    /// The buffered state, typically <see cref="Pose"/>. Use a value type; both
    /// <see cref="Enqueue"/> and <see cref="TryDequeue"/> are on the per-frame path.
    /// </typeparam>
    public interface IPlayoutPolicy<TState>
    {
        /// <summary>
        /// A sample arrived. <paramref name="arrivalTicks"/> is <c>t_recv</c> — when the
        /// transport actually received it, not the time the host happens to poll at — because
        /// using poll time instead folds frame time into the delay-budget and one-way-delay
        /// statistics this policy computes from it (docs/metrics.md §1). May be called with a
        /// <paramref name="sequence"/> already buffered or already released; the implementation
        /// discards the duplicate and counts it rather than re-buffering or releasing it twice.
        /// Allocation-free.
        /// </summary>
        /// <param name="sequence">
        /// Correlation key, matching <see cref="LatencyTrace.Sequence"/> for this sample — the
        /// uplink command's sequence for a command-buffering policy, or the echoed sequence
        /// carried by a downlink state update. Never
        /// <see cref="Stamped{TState}.CaptureTicks"/>; see the type-level remarks.
        /// </param>
        /// <param name="sample">The received value and its capture stamp.</param>
        /// <param name="arrivalTicks">
        /// <c>t_recv</c>: when this sample was received, on the local <c>ITimeAuthority</c>
        /// timebase.
        /// </param>
        void Enqueue(uint sequence, Stamped<TState> sample, long arrivalTicks);

        /// <summary>
        /// Releases the next buffered sample whose target playout instant is at or before
        /// <paramref name="nowTicks"/>, if any. Returns false when nothing is due, which is the
        /// common case and not an error — call in a loop until it returns false to drain a step,
        /// mirroring <c>ITransport.TryReceive</c>. Samples are returned in capture-time order,
        /// which is not necessarily the order <see cref="Enqueue"/> was called in.
        /// </summary>
        /// <param name="nowTicks">Current time; nothing scheduled after it is released yet.</param>
        /// <param name="sequence">The released sample's correlation key.</param>
        /// <param name="value">The released value.</param>
        /// <param name="playoutTicks">
        /// <c>t_playout</c>: the instant this policy actually scheduled the sample for, which
        /// may be earlier than <paramref name="nowTicks"/> if the host polled late. Reporting
        /// <paramref name="nowTicks"/> here instead would fold step time into every downstream
        /// playout-latency figure — the same failure mode docs/metrics.md warns about for
        /// <c>t_recv</c> stamped on the wrong thread.
        /// </param>
        bool TryDequeue(long nowTicks, out uint sequence, out TState value, out long playoutTicks);

        /// <summary>
        /// Returns the policy to its as-constructed state: buffer empty, delay-budget estimate
        /// back to its initial value, all diagnostic counters zeroed. Configuration, the
        /// preallocated buffer, and the injected metric sink survive. Sweeps reuse instances
        /// across trials.
        /// </summary>
        void Reset();

        /// <summary>
        /// What this policy is currently doing to the stream: delay budget, occupancy, and the
        /// late-arrival rate that budget is inducing. Read after <see cref="Enqueue"/> or
        /// <see cref="TryDequeue"/>; before the first call it reports
        /// <see cref="PlayoutPolicyDiagnostics.Empty"/>.
        /// </summary>
        PlayoutPolicyDiagnostics Diagnostics { get; }
    }
}
