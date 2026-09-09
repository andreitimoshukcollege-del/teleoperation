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
        /// What a run recorded before <c>IPlayoutPolicy</c> was wired in: no policy at all, and
        /// <c>Pipeline/OperatorEndpoint</c>'s hardcoded <c>t_playout = t_operatorRecv</c>. Matches
        /// <c>manifest.py</c>'s <c>LEGACY_PLAYOUT_POLICY</c> exactly, and is deliberately <b>not</b>
        /// a <c>Registry/Registries.cs</c> key.
        ///
        /// <b>It used to be the string <c>immediate</c>, and renaming it was the point.</b>
        /// <c>immediate</c> is now a real policy that enforces capture-time ordering, which the
        /// inline stand-in did not -- so leaving the old label would have made every pre-Buffering
        /// manifest claim its run used a policy that behaved differently and did not exist when it
        /// ran (docs/adr/0012-playout-policy-wiring.md). Manifests already on disk are untouched and
        /// still read correctly; only what new runs write changes.
        /// </summary>
        public const string LegacyInlinePlayout = "legacy-inline-playout";

        /// <summary>
        /// The stand-in <c>Autonomy/</c> selection, matching <c>manifest.py</c>'s
        /// <c>LEGACY_ARBITER</c>. Same reasoning as <see cref="LegacyInlinePlayout"/>: no arbiter
        /// is wired into <c>Pipeline/</c>, so every command carries full direct authority.
        /// </summary>
        public const string StandInArbiter = "direct";

        public ResolvedStack(string name, string predictor, string reconciler, string playoutPolicy)
        {
            Name = name;
            Predictor = predictor;
            Reconciler = reconciler;
            PlayoutPolicy = playoutPolicy;
        }

        /// <summary>
        /// The results subdirectory and the label every figure groups by. See
        /// <see cref="Enumerate"/> for the naming rule.
        /// </summary>
        public string Name { get; }

        public string Predictor { get; }

        public string Reconciler { get; }

        public string PlayoutPolicy { get; }

        public string Arbiter => StandInArbiter;

        /// <summary>
        /// Every (predictor, reconciler, playout policy) combination the config asks for.
        ///
        /// <b>Naming rule: a stack is named for the axes the experiment actually varies.</b> With a
        /// single reconciler and a single playout policy held fixed the name is the bare predictor,
        /// which is byte-identical to the layout every existing run used -- so re-running
        /// <c>exp-001</c>/<c>exp-002</c> puts its <c>metrics.csv</c> in exactly the same place as
        /// before and nothing downstream moves. Each axis that actually varies appends its key, in
        /// a fixed order: <c>predictor__reconciler__playout</c>. The alternative, always using the
        /// full compound name, would silently relocate every existing experiment's output for
        /// no benefit.
        /// </summary>
        public static List<ResolvedStack> Enumerate(
            IReadOnlyList<string> predictors,
            IReadOnlyList<string> reconcilers,
            IReadOnlyList<string> playoutPolicies)
        {
            bool reconcilerVaries = reconcilers.Count > 1;
            bool playoutVaries = playoutPolicies.Count > 1;
            var stacks = new List<ResolvedStack>(
                predictors.Count * reconcilers.Count * playoutPolicies.Count);

            foreach (string predictor in predictors)
            {
                foreach (string reconciler in reconcilers)
                {
                    foreach (string playout in playoutPolicies)
                    {
                        string name = predictor;
                        if (reconcilerVaries)
                        {
                            name += "__" + reconciler;
                        }

                        if (playoutVaries)
                        {
                            name += "__" + playout;
                        }

                        stacks.Add(new ResolvedStack(name, predictor, reconciler, playout));
                    }
                }
            }

            return stacks;
        }
    }
}
