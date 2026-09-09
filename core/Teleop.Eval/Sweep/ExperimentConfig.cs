using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace Teleop.Eval.Sweep
{
    /// <summary>
    /// One experiment's definition, loaded from an <c>experiments/*.yaml</c> file (schema
    /// documented in <c>experiments/CLAUDE.md</c>). Deliberately minimal. It expresses a study on
    /// one axis at a time, per <c>Reconciliation/CLAUDE.md</c>'s experiment-design note: sweep
    /// <c>predictors</c> with a fixed <c>reconciler</c>, or sweep <c>reconcilers</c> with a single
    /// fixed predictor. Nothing stops both being lists, but attributing the result then becomes the
    /// caller's problem.
    ///
    /// Property names are PascalCase to match the YAML (camelCase-to-PascalCase mapping via
    /// YamlDotNet's <c>CamelCaseNamingConvention</c> on the deserializer, so the YAML itself reads
    /// naturally lowercase).
    /// </summary>
    public sealed class ExperimentConfig
    {
        /// <summary>Identifies this experiment; also the results subdirectory name.</summary>
        public string Id { get; set; } = string.Empty;

        /// <summary>Seeds to run every (predictor, network profile) combination under.</summary>
        public List<ulong> Seeds { get; set; } = new List<ulong>();

        /// <summary><see cref="Teleop.Core.Registry.Registries.Predictors"/> keys to sweep.</summary>
        public List<string> Predictors { get; set; } = new List<string>();

        /// <summary>
        /// The single <see cref="Teleop.Core.Registry.Registries.Reconcilers"/> key held fixed
        /// across the sweep. The original schema, still supported and still what
        /// <c>analysis/experiment_builder.py</c> emits -- a predictor study holds the reconciler
        /// fixed, per <c>Reconciliation/CLAUDE.md</c>'s experiment-design note. Mutually exclusive
        /// with <see cref="Reconcilers"/>.
        /// </summary>
        public string Reconciler { get; set; } = string.Empty;

        /// <summary>
        /// <see cref="Teleop.Core.Registry.Registries.Reconcilers"/> keys to sweep, for a
        /// reconciler study -- the mirror image of <see cref="Predictors"/>, holding the predictor
        /// fixed instead. Mutually exclusive with <see cref="Reconciler"/>; specifying both is
        /// rejected rather than silently resolved, because which one wins is not something a reader
        /// of the YAML could predict.
        /// </summary>
        public List<string> Reconcilers { get; set; } = new List<string>();

        /// <summary><see cref="NetworkProfileCatalog"/> names to sweep.</summary>
        public List<string> NetworkProfiles { get; set; } = new List<string>();

        /// <summary>Command-submission steps per trial.</summary>
        public int TrialSteps { get; set; }

        /// <summary>Ticks between consecutive command-submission steps within a trial.</summary>
        public long StepIntervalTicks { get; set; }

        /// <summary>
        /// <see cref="Teleop.Core.Types.ReconcilerConfig.MaxTimeToConvergenceTicks"/>, in
        /// milliseconds -- the time a smoothed reconciler is given to absorb a correction.
        ///
        /// In milliseconds rather than ticks because the tick rate is <c>SweepCommand</c>'s own
        /// internal constant, which the author of a YAML file cannot see; docs/metrics.md reports
        /// every duration in ms for the same reason. (<see cref="StepIntervalTicks"/> predates this
        /// and stays in ticks.)
        ///
        /// Exposed because it is a genuine operating point, not an invented knob: it decides where
        /// a reconciler sits on the jerk-versus-convergence-time tradeoff, which is the whole
        /// question <c>Reconciliation/CLAUDE.md</c> exists to ask, and it cannot be swept while
        /// hardcoded. <c>snap</c> ignores it entirely (documented on <c>SnapReconciler</c>), so
        /// experiments that hold the reconciler at <c>snap</c> are unaffected by any value here.
        ///
        /// Defaults to 100 ms, a plausible VR correction budget. The previous hardcoded value was
        /// one full second against a 10 ms frame, which left a reconciler essentially never
        /// converged.
        /// </summary>
        public double ConvergenceBudgetMs { get; set; } = 100.0;

        /// <summary>
        /// <see cref="Teleop.Core.Types.ReconcilerConfig.MaxCorrectionLinearSpeedMetersPerSecond"/>.
        /// Bounds apparent motion during a correction, and takes precedence over
        /// <see cref="ConvergenceBudgetMs"/> when the two conflict. Ignored by <c>snap</c>.
        /// </summary>
        public float MaxCorrectionLinearSpeedMetersPerSecond { get; set; } = 5f;

        /// <summary>
        /// <see cref="Teleop.Core.Types.ReconcilerConfig.MaxCorrectionAngularSpeedRadPerSecond"/>.
        /// Same role and precedence as <see cref="MaxCorrectionLinearSpeedMetersPerSecond"/>.
        /// Ignored by <c>snap</c>.
        /// </summary>
        public float MaxCorrectionAngularSpeedRadiansPerSecond { get; set; } = 10f;

        /// <summary>
        /// The single <see cref="Teleop.Core.Registry.Registries.PlayoutPolicies"/> key held fixed
        /// while another axis varies. Defaults to <c>immediate</c> -- the zero buffer, which is the
        /// operating point every pre-Buffering run implicitly had, so an existing experiment YAML
        /// that says nothing about playout keeps meaning what it meant. Mutually exclusive with
        /// <see cref="PlayoutPolicies"/>.
        /// </summary>
        public string PlayoutPolicy { get; set; } = string.Empty;

        /// <summary>
        /// <see cref="Teleop.Core.Registry.Registries.PlayoutPolicies"/> keys to sweep, for a
        /// buffering study. Mutually exclusive with <see cref="PlayoutPolicy"/>.
        ///
        /// Sweeping this axis against a <b>single</b> budget is usually the wrong experiment:
        /// <c>fixed</c> is a family of operating points, not one, and comparing a single budget
        /// against <c>immediate</c> compares two arbitrary points on a curve. Vary
        /// <see cref="PlayoutBudgetMs"/> across runs to get the curve itself, which is what an
        /// adaptive policy's claim is stated against
        /// (docs/research-log/2026-09-09-playout-bounds-decisions.md).
        /// </summary>
        public List<string> PlayoutPolicies { get; set; } = new List<string>();

        /// <summary>
        /// <see cref="Teleop.Core.Types.PlayoutPolicyConfig.InitialDelayBudgetTicks"/>, in
        /// milliseconds. Ignored by <c>immediate</c>, whose budget is structurally zero. Defaults
        /// to zero so that a config which names no buffering intent gets none.
        /// </summary>
        public double PlayoutBudgetMs { get; set; } = 0.0;

        /// <summary>
        /// <see cref="Teleop.Core.Types.PlayoutPolicyConfig.HistoryCapacity"/>: how many samples a
        /// policy may hold, and therefore how far out of order a sample may arrive and still be
        /// reinserted. Not a research knob -- it is a bound that must exceed the budget divided by
        /// the step interval, or the buffer discards samples it had room to keep and the late rate
        /// measures the capacity rather than the policy.
        /// </summary>
        public int PlayoutHistoryCapacity { get; set; } = 64;

        /// <summary>
        /// The playout keys this config actually asks for, whichever of the two spellings it used.
        /// Mirrors <see cref="ResolveReconcilers"/>, including its fallback: an empty config means
        /// <c>immediate</c> here rather than an empty list, because unlike a reconciler there is no
        /// such thing as running without a playout policy once one is wired.
        /// </summary>
        public List<string> ResolvePlayoutPolicies()
        {
            if (PlayoutPolicies.Count > 0)
            {
                return PlayoutPolicies;
            }

            return string.IsNullOrWhiteSpace(PlayoutPolicy)
                ? new List<string> { "immediate" }
                : new List<string> { PlayoutPolicy };
        }

        /// <summary>Both playout spellings used at once -- a config error, not a merge.</summary>
        public bool HasConflictingPlayoutKeys =>
            PlayoutPolicies.Count > 0 && !string.IsNullOrWhiteSpace(PlayoutPolicy);

        /// <summary>
        /// The reconciler keys this config actually asks for, whichever of the two spellings it
        /// used. Returns empty when neither is set, which <c>SweepCommand</c>'s validation reports.
        /// </summary>
        public List<string> ResolveReconcilers()
        {
            if (Reconcilers.Count > 0)
            {
                return Reconcilers;
            }

            return string.IsNullOrWhiteSpace(Reconciler)
                ? new List<string>()
                : new List<string> { Reconciler };
        }

        /// <summary>True when the config sets both spellings, which is ambiguous.</summary>
        public bool HasConflictingReconcilerKeys =>
            Reconcilers.Count > 0 && !string.IsNullOrWhiteSpace(Reconciler);

        public static ExperimentConfig Load(string path)
        {
            string yaml = File.ReadAllText(path);
            var deserializer = new DeserializerBuilder()
                .WithNamingConvention(CamelCaseNamingConvention.Instance)
                .Build();
            return deserializer.Deserialize<ExperimentConfig>(yaml);
        }
    }
}
