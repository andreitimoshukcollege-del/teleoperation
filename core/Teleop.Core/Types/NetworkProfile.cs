// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// The complete parameter set of the synthetic impairment an <c>EmulatedTransport</c> injects:
    /// one-way delay, jitter, burst loss, and reordering. Every number the emulator uses comes from
    /// here — no magic numbers in its body — so a sweep varies the link purely by varying this
    /// struct, and a run manifest records exactly which link produced a result.
    ///
    /// There is deliberately no <c>Default</c>, for the same reason
    /// <see cref="PredictorConfig"/> has none: a default network profile would be an invented
    /// number with no measurement behind it, and it would silently become the link every
    /// un-configured run was measured over. Values come from an experiment YAML or a test.
    ///
    /// Units are ticks on the injected <c>ITimeAuthority</c> timebase, never milliseconds — the
    /// tick rate is a property of the clock in use (<c>ITimeAuthority.TicksPerSecond</c>), and
    /// baking a millisecond assumption in here would make the same profile mean two different
    /// links on two different clocks. Conversion to the milliseconds used in docs/metrics.md
    /// happens at report time.
    ///
    /// This struct describes a <b>parametric</b> link only. Trace-driven replay of recorded
    /// one-way delays (<c>Transport/CLAUDE.md</c>, "trace-driven mode ... no resampling") is a
    /// separate mechanism and is deliberately not expressible here.
    /// </summary>
    public readonly struct NetworkProfile
    {
        /// <summary>
        /// Fixed one-way delay added to every delivered datagram, in ticks. This is added on top
        /// of whatever transit delay the wrapped transport reports on its own, so wrapping a real
        /// socket yields real delay plus this, not this alone.
        /// </summary>
        public readonly long BaseDelayTicks;

        /// <summary>
        /// Half-width of the jitter added to <see cref="BaseDelayTicks"/>, in ticks. The draw is
        /// <b>uniform</b> over the inclusive integer range
        /// <c>[-JitterTicks, +JitterTicks]</c>; zero means no jitter and an exactly constant delay.
        ///
        /// Uniform rather than Gaussian or Pareto is a deliberate Phase-3 scoping choice, not a
        /// claim about real links: a uniform integer draw is the one distribution whose every
        /// outcome can be hand-computed and asserted exactly in a deterministic test, which is
        /// what the delay path has to be trusted for before anything measured through it counts.
        /// A heavier-tailed delay model is a later, separately-registered profile shape.
        /// </summary>
        public readonly long JitterTicks;

        /// <summary>
        /// Probability in [0, 1] that the next datagram is lost <b>given the previous one through
        /// the same emulator instance was delivered</b> — the good-to-bad transition of a 2-state
        /// Gilbert-Elliott Markov chain. Alone (with
        /// <see cref="LossProbabilityAfterLost"/> equal to it) this degenerates to plain Bernoulli
        /// loss, which <c>Transport/CLAUDE.md</c> explicitly rejects as insufficient.
        /// </summary>
        public readonly double LossProbabilityAfterDelivered;

        /// <summary>
        /// Probability in [0, 1] that the next datagram is lost <b>given the previous one was
        /// lost</b> — the stay-in-bad-state probability of the chain. Setting this above
        /// <see cref="LossProbabilityAfterDelivered"/> is what produces bursts rather than
        /// independent drops, and burst length, not average loss rate, is what breaks a jitter
        /// buffer.
        ///
        /// The chain's expected burst length is the mean of a geometric distribution over the
        /// stay probability: <c>1 / (1 - LossProbabilityAfterLost)</c>. So 0.0 gives isolated
        /// single-packet drops, 0.5 gives bursts of 2 on average, 0.9 gives bursts of 10. A value
        /// of exactly 1.0 is an absorbing state: once a datagram is lost, every subsequent one is
        /// lost until <c>Reset()</c> — a total-outage model, and legitimate as such, but not a
        /// burst model.
        /// </summary>
        public readonly double LossProbabilityAfterLost;

        /// <summary>
        /// Probability in [0, 1] that a delivered datagram additionally has
        /// <see cref="ReorderDelayTicks"/> added to its delay, pushing it behind datagrams sent
        /// after it.
        ///
        /// Kept as an independent knob rather than left to emerge from jitter variance so that
        /// reordering can be studied with jitter held at zero (isolating "packets arrive out of
        /// order" from "packets arrive at varying times"), or jitter studied with reordering held
        /// off. Those two effects hit a playout buffer differently and conflating them into one
        /// parameter would make the result uninterpretable.
        /// </summary>
        public readonly double ReorderProbability;

        /// <summary>
        /// Extra delay, in ticks, applied to the datagrams selected by
        /// <see cref="ReorderProbability"/>. To actually reorder anything this must exceed the
        /// interval between consecutive sends; smaller than that, it is just a delay spike.
        /// </summary>
        public readonly long ReorderDelayTicks;

        public NetworkProfile(
            long baseDelayTicks,
            long jitterTicks,
            double lossProbabilityAfterDelivered,
            double lossProbabilityAfterLost,
            double reorderProbability,
            long reorderDelayTicks)
        {
            BaseDelayTicks = baseDelayTicks;
            JitterTicks = jitterTicks;
            LossProbabilityAfterDelivered = lossProbabilityAfterDelivered;
            LossProbabilityAfterLost = lossProbabilityAfterLost;
            ReorderProbability = reorderProbability;
            ReorderDelayTicks = reorderDelayTicks;
        }

        /// <summary>
        /// Expected number of consecutive datagrams lost once loss starts,
        /// <c>1 / (1 - LossProbabilityAfterLost)</c>. Reported rather than recomputed at each call
        /// site so that the burst-length figure in a manifest and the one in a paper come from the
        /// same expression. Returns <c>double.PositiveInfinity</c> for the absorbing case
        /// <see cref="LossProbabilityAfterLost"/> == 1.
        /// </summary>
        public double ExpectedBurstLength =>
            LossProbabilityAfterLost >= 1.0
                ? double.PositiveInfinity
                : 1.0 / (1.0 - LossProbabilityAfterLost);

        public override string ToString() =>
            $"NetworkProfile(base={BaseDelayTicks}t, jitter=±{JitterTicks}t, " +
            $"loss={LossProbabilityAfterDelivered:F4}/{LossProbabilityAfterLost:F4}, " +
            $"burst={ExpectedBurstLength:F2}, reorder={ReorderProbability:F4}@{ReorderDelayTicks}t)";
    }
}
