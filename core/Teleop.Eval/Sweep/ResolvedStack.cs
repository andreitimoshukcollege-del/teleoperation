namespace Teleop.Eval.Sweep
{
    /// <summary>
    /// One fully-resolved mitigation stack: the exact combination of implementations a run
    /// exercised on every axis. A sweep's unit of comparison, and the name of the directory its
    /// <c>metrics.csv</c> lands in.
    ///
    /// <b>The schema is not new here.</b> <c>analysis/teleop_analysis/manifest.py</c> already reads
    /// a <c>stacks</c> array of exactly this shape and falls back to synthesizing one stack per
    /// predictor when a manifest predates it -- so the Python analysis layer has been ahead of this
    /// writer, with the richer path exercised only by test fixtures. Emitting it means
    /// <c>io_utils.discover_run</c> (which already reads
    /// <c>run_dir / stack.name / profile / metrics.csv</c>), <c>baseline.py</c>,
    /// <c>axis_diff.py</c> and <c>figures/stack_comparison.py</c> all start working on real runs
    /// with no Python change at all. Keep the JSON property names in step with that reader.
    /// </summary>
    public sealed class ResolvedStack
    {
        /// <summary>
        /// The stand-in <c>Buffering/</c> selection recorded for every stack until an
        /// <c>IPlayoutPolicy</c> exists. Matches <c>manifest.py</c>'s <c>LEGACY_PLAYOUT_POLICY</c>
        /// exactly: it names the behaviour <c>Pipeline/OperatorEndpoint</c> actually has today, its
        /// hardcoded <c>t_playout = t_operatorRecv</c>, and is deliberately <b>not</b> a
        /// <c>Registry/Registries.cs</c> key -- <c>PlayoutPolicies</c> is empty. Recording it keeps
        /// this writer and <c>manifest.py</c>'s legacy path from disagreeing about what a
        /// pre-Buffering run did.
        /// </summary>
        public const string StandInPlayoutPolicy = "immediate";

        /// <summary>
        /// The stand-in <c>Autonomy/</c> selection, matching <c>manifest.py</c>'s
        /// <c>LEGACY_ARBITER</c>. Same reasoning as <see cref="StandInPlayoutPolicy"/>: no arbiter
        /// is wired into <c>Pipeline/</c>, so every command carries full direct authority.
        /// </summary>
        public const string StandInArbiter = "direct";

        public ResolvedStack(string name, string predictor, string reconciler)
        {
            Name = name;
            Predictor = predictor;
            Reconciler = reconciler;
        }

        /// <summary>
        /// The results subdirectory and the label every figure groups by. See
        /// <see cref="Enumerate"/> for the naming rule.
        /// </summary>
        public string Name { get; }

        public string Predictor { get; }

        public string Reconciler { get; }

        public string PlayoutPolicy => StandInPlayoutPolicy;

        public string Arbiter => StandInArbiter;

        /// <summary>
        /// Every (predictor, reconciler) combination the config asks for.
        ///
        /// <b>Naming rule: a stack is named for the axes the experiment actually varies.</b> With a
        /// single reconciler held fixed the name is the bare predictor, which is byte-identical to
        /// the layout every existing run used -- so re-running <c>exp-001</c>/<c>exp-002</c> puts
        /// its <c>metrics.csv</c> in exactly the same place as before and nothing downstream moves.
        /// With more than one reconciler the name is <c>predictor__reconciler</c>, because the
        /// directory has to distinguish them. The alternative, always using the compound name,
        /// would silently relocate every existing experiment's output for no benefit.
        /// </summary>
        public static List<ResolvedStack> Enumerate(
            IReadOnlyList<string> predictors, IReadOnlyList<string> reconcilers)
        {
            bool reconcilerVaries = reconcilers.Count > 1;
            var stacks = new List<ResolvedStack>(predictors.Count * reconcilers.Count);

            foreach (string predictor in predictors)
            {
                foreach (string reconciler in reconcilers)
                {
                    string name = reconcilerVaries ? $"{predictor}__{reconciler}" : predictor;
                    stacks.Add(new ResolvedStack(name, predictor, reconciler));
                }
            }

            return stacks;
        }
    }
}
