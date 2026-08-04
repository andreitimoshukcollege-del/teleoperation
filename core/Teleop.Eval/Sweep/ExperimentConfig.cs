using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace Teleop.Eval.Sweep
{
    /// <summary>
    /// One experiment's definition, loaded from an <c>experiments/*.yaml</c> file (schema
    /// documented in <c>experiments/CLAUDE.md</c>). Deliberately minimal -- just enough to drive
    /// <c>exp-001-predictor-baseline.yaml</c>: predictor studies vary only the predictor, holding
    /// the reconciler fixed, per <c>Reconciliation/CLAUDE.md</c>'s experiment-design note.
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
        /// across the sweep.
        /// </summary>
        public string Reconciler { get; set; } = string.Empty;

        /// <summary><see cref="NetworkProfileCatalog"/> names to sweep.</summary>
        public List<string> NetworkProfiles { get; set; } = new List<string>();

        /// <summary>Command-submission steps per trial.</summary>
        public int TrialSteps { get; set; }

        /// <summary>Ticks between consecutive command-submission steps within a trial.</summary>
        public long StepIntervalTicks { get; set; }

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
