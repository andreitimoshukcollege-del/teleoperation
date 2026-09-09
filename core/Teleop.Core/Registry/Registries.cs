using System;
using System.Collections.Generic;
using Teleop.Core.Buffering;
using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Reconciliation;
using Teleop.Core.Transport;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Registry
{
    /// <summary>
    /// Static <c>string -&gt; factory</c> tables, one per research axis, hand-maintained per root
    /// CLAUDE.md invariant 5 ("Reflection-free construction. Add to Registries.cs by hand.").
    /// This is what lets an experiment YAML name an algorithm by a short key
    /// (<c>const-vel</c>, <c>snap</c>, ...) instead of the runtime needing
    /// <c>Activator.CreateInstance</c>, which IL2CPP's stripper would remove anyway for any type
    /// nothing references directly.
    ///
    /// Property names match <c>.claude/commands/new-impl.md</c>'s "Registry table" column
    /// exactly: <see cref="Predictors"/>, <see cref="Reconcilers"/>, <see cref="PlayoutPolicies"/>,
    /// <see cref="Codecs"/>, <see cref="Arbiters"/>, <see cref="Transports"/>. Every dictionary
    /// uses <see cref="StringComparer.Ordinal"/> — the same cross-runtime-determinism reasoning
    /// <see cref="SeededRng"/>'s hand-rolled PRNG documents for itself: a registry lookup must
    /// not depend on locale/culture behavior differing between <c>dotnet test</c> and the Quest
    /// build.
    ///
    /// <see cref="Arbiters"/> is declared, correctly typed, and empty — <c>Autonomy/</c> has no
    /// implementation yet, but the table exists so adding the first one is a one-line change here,
    /// not a new table to design. <see cref="PlayoutPolicies"/> was in that state until
    /// <c>docs/adr/0012-playout-policy-wiring.md</c>, and adding its first two entries was indeed a
    /// one-line change each.
    /// Non-generic by design: every contract's own doc already says <c>TState</c> is "typically
    /// <see cref="Pose"/>," and nothing in this project instantiates any of them against another
    /// state type. A second <c>TState</c> showing up later is a second table, not a generic
    /// rewrite of this one.
    /// </summary>
    public static class Registries
    {
        /// <summary>
        /// <see cref="IPredictor{TState}"/> factories, keyed by the name an experiment config
        /// uses. The factory shape is <c>(PredictorConfig, ITimeAuthority) -&gt; IPredictor&lt;Pose&gt;</c>
        /// even though <see cref="PassthroughPredictor"/> ignores the clock — the two
        /// extrapolating predictors need <c>ITimeAuthority.TicksPerSecond</c> to convert
        /// <see cref="PredictorConfig.MaxLinearSpeed"/>/<see cref="PredictorConfig.MaxAngularSpeed"/>
        /// (both per-second) against tick-stamped observations, so every entry takes the richer
        /// signature rather than having two incompatible dictionaries for a difference that is
        /// only sometimes used.
        /// </summary>
        public static readonly IReadOnlyDictionary<string, Func<PredictorConfig, ITimeAuthority, IPredictor<Pose>>> Predictors =
            new Dictionary<string, Func<PredictorConfig, ITimeAuthority, IPredictor<Pose>>>(StringComparer.Ordinal)
            {
                ["none"] = (config, clock) => new PassthroughPredictor(config),
                ["const-vel"] = (config, clock) => new ConstantVelocityPredictor(config, clock),
                ["double-exp"] = (config, clock) => new DoubleExponentialPredictor(config, clock),
            };

        /// <summary>
        /// <see cref="IReconciler{TState}"/> factories. <see cref="IMetricSink"/> is a
        /// constructor dependency here (not a per-call parameter) because
        /// <c>IReconciler{TState}.Observe</c>'s own doc mandates it, specifically so the
        /// per-frame <c>Reconcile</c> signature stays allocation-free.
        /// </summary>
        public static readonly IReadOnlyDictionary<string, Func<ReconcilerConfig, IMetricSink, ITimeAuthority, IReconciler<Pose>>> Reconcilers =
            new Dictionary<string, Func<ReconcilerConfig, IMetricSink, ITimeAuthority, IReconciler<Pose>>>(StringComparer.Ordinal)
            {
                ["snap"] = (config, metrics, clock) => new SnapReconciler(config, metrics, clock),
                ["spring"] = (config, metrics, clock) => new SpringReconciler(config, metrics, clock),
                ["budget-blend"] = (config, metrics, clock) => new TimeBudgetedBlendReconciler(config, metrics, clock),
                ["velocity-match"] = (config, metrics, clock) => new VelocityMatchedReconciler(config, metrics, clock),
                ["exp-smooth"] = (config, metrics, clock) => new ExponentialSmoothingReconciler(config, metrics, clock),
                ["exp-smooth-c1"] = (config, metrics, clock) => new EasedExponentialReconciler(config, metrics, clock),
                ["exp-smooth-track"] = (config, metrics, clock) => new TrackingLagReconciler(config, metrics, clock),
            };

        /// <summary><see cref="ICommandCodec"/> factories.</summary>
        public static readonly IReadOnlyDictionary<string, Func<ICommandCodec>> Codecs =
            new Dictionary<string, Func<ICommandCodec>>(StringComparer.Ordinal)
            {
                ["raw"] = () => new RawPoseCodec(),
            };

        /// <summary>
        /// <see cref="ITransport"/> factories. Shape is <c>(maxPayloadBytes, capacity) -&gt; ITransport</c>,
        /// specific to <see cref="LoopbackTransport"/>'s constructor — <c>EmulatedTransport</c>
        /// is a decorator over another <see cref="ITransport"/> plus a <see cref="NetworkProfile"/>
        /// and a <see cref="SeededRng"/>, a materially different shape, so it is deliberately not
        /// registered here yet rather than forced into this signature. Registering it is a
        /// follow-up once a sweep actually needs to select a transport by name rather than
        /// wiring one directly, as every current test does.
        /// </summary>
        public static readonly IReadOnlyDictionary<string, Func<int, int, ITransport>> Transports =
            new Dictionary<string, Func<int, int, ITransport>>(StringComparer.Ordinal)
            {
                ["loopback"] = (maxPayloadBytes, capacity) => new LoopbackTransport(maxPayloadBytes, capacity),
            };

        /// <summary>
        /// <see cref="IPlayoutPolicy{TState}"/> factories. The two baselines
        /// (<c>docs/adr/0012-playout-policy-wiring.md</c>); <c>percentile</c>, <c>kalman-jitter</c>,
        /// <c>adaptive</c> and <c>pareto</c> are still planned in <c>Buffering/CLAUDE.md</c>.
        ///
        /// <see cref="IMetricSink"/> is a constructor dependency because
        /// <see cref="IPlayoutPolicy{TState}"/> clause 3 mandates it — an underrun is pushed, not
        /// polled, for the same reason a reconciler pushes correction cost. <see cref="ITimeAuthority"/>
        /// is taken for the same reason <see cref="Predictors"/> takes it: both policies here need
        /// <c>TicksPerSecond</c> to report <c>playout_delay_ms</c> in milliseconds, and
        /// <see cref="PlayoutPolicyConfig.MaxAdaptationRatePerSecond"/> — specified per second of
        /// wall time — is unusable without one, so <c>adaptive</c> will need it too. Settling the
        /// shape once matters here beyond the usual: root CLAUDE.md names this file as a
        /// cross-machine conflict surface, so a second signature change is a second merge to
        /// coordinate.
        /// </summary>
        public static readonly IReadOnlyDictionary<string, Func<PlayoutPolicyConfig, IMetricSink, ITimeAuthority, IPlayoutPolicy<Pose>>> PlayoutPolicies =
            new Dictionary<string, Func<PlayoutPolicyConfig, IMetricSink, ITimeAuthority, IPlayoutPolicy<Pose>>>(StringComparer.Ordinal)
            {
                ["immediate"] = (config, metrics, clock) => new ImmediatePlayout(config, metrics, clock),
                ["fixed"] = (config, metrics, clock) => new FixedDelayPlayout(config, metrics, clock),
            };

        /// <summary>
        /// <see cref="IAutonomyArbiter"/> factories. Empty — <c>Autonomy/</c> has no
        /// implementation yet.
        /// </summary>
        public static readonly IReadOnlyDictionary<string, Func<AutonomyArbiterConfig, IAutonomyArbiter>> Arbiters =
            new Dictionary<string, Func<AutonomyArbiterConfig, IAutonomyArbiter>>(StringComparer.Ordinal);
    }
}
