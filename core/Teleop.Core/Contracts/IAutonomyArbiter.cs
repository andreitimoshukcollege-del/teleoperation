using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Decides how much direct authority the operator's command carries, versus how much the
    /// robot resolves locally. Implementations live in <c>Autonomy/</c> and select a rung on
    /// Sheridan's supervisory-control ladder (Autonomy/CLAUDE.md): direct, rate-limited,
    /// waypoint, or intent primitive, from most to least direct — see
    /// <see cref="AutonomyRung"/>.
    ///
    /// <b>Latency-estimate dependency.</b> Rung selection is driven by measured round-trip
    /// time, not one-way delay: the quantity that determines how much local autonomy the robot
    /// needs is how long the operator waits to see the effect of a command, which is the full
    /// loop, not either half of it. RTT is supplied explicitly through <see cref="Observe"/> as
    /// a <see cref="Stamped{TState}">Stamped&lt;long&gt;</see> rather than read from an implicit
    /// source, so the caller — not this interface — owns computing it and the dependency is
    /// visible at every call site. The intended producer is the pipeline that owns round-trip
    /// bookkeeping: once both halves of a <see cref="LatencyTrace"/> for a given sequence are
    /// known, <c>OperatorRecvTicks - CaptureTicks</c> is that command's RTT, reported here
    /// stamped at <c>CaptureTicks</c> — the time the round trip *started* — so that, like
    /// <c>IPredictor.Observe</c>, a sample is attributed to the event it measures rather than to
    /// when this arbiter happened to learn about it. Until <c>Pipeline/</c> exists, tests and
    /// <c>Teleop.Eval</c> construct these samples directly from a recorded or synthetic RTT
    /// series.
    ///
    /// Contract every implementation owes its callers:
    /// <list type="number">
    /// <item><b>Monotonic in latency.</b> More measured RTT must never yield a more direct rung.
    /// Provable by a test sweeping latency and asserting the rung sequence never decreases in
    /// directness.</item>
    /// <item><b>Hysteretic at rung boundaries.</b> A latency signal dithering across a threshold
    /// must not flip the rung at the same rate — provable by a test asserting a bounded switch
    /// rate under a dithering input.</item>
    /// <item><b>Legible.</b> <see cref="Diagnostics"/> exposes the current rung and the reason
    /// for it, so a HUD can display why the robot is behaving as it is.</item>
    /// <item><b>Bounded authority transitions.</b> <see cref="Arbitrate"/> output must not
    /// contain a position or velocity discontinuity on the step a rung changes — the same
    /// C1-continuity discipline as <c>IReconciler</c>.</item>
    /// <item>Deterministic and allocation-free.</item>
    /// </list>
    /// </summary>
    public interface IAutonomyArbiter
    {
        /// <summary>
        /// A round-trip time measurement arrived. See the type-level remarks for what value to
        /// pass and why. May arrive at a lower rate than <see cref="Arbitrate"/> is called —
        /// most implementations smooth this internally rather than reacting to a single sample.
        /// Allocation-free.
        /// </summary>
        /// <param name="roundTripTicks">
        /// Measured round-trip time in <see cref="Stamped{TState}.Value"/>, ticks; stamped with
        /// <see cref="Stamped{TState}.CaptureTicks"/> of the command the measurement is for.
        /// </param>
        void Observe(Stamped<long> roundTripTicks);

        /// <summary>
        /// Applies the current rung to <paramref name="operatorCommand"/>, returning the command
        /// that should actually reach the plant. <see cref="CommandFrame.Sequence"/> is
        /// preserved unchanged — downstream correlation depends on it. Called every frame
        /// whether or not <see cref="Observe"/> was just called; two calls at the same
        /// <paramref name="nowTicks"/> with no intervening <see cref="Observe"/> return identical
        /// output. Allocation-free.
        /// </summary>
        CommandFrame Arbitrate(in CommandFrame operatorCommand, long nowTicks);

        /// <summary>
        /// Returns the arbiter to its as-constructed state: no latency history, rung back to its
        /// initial value, cleared hysteresis and dwell-time state. Configuration survives.
        /// Sweeps reuse instances across trials.
        /// </summary>
        void Reset();

        /// <summary>
        /// The current rung and why it is current. Read after <see cref="Arbitrate"/>; before
        /// the first call it reports the implementation's as-constructed rung (documented per
        /// implementation, since a sensible starting rung is a design choice, not a universal
        /// default).
        /// </summary>
        AutonomyArbiterDiagnostics Diagnostics { get; }
    }
}
