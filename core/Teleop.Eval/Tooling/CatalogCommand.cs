using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using Teleop.Core.Registry;
using Teleop.Core.Transport;

namespace Teleop.Eval.Tooling
{
    /// <summary>
    /// Implements <c>catalog</c>: prints everything a sweep can be configured with, as JSON on
    /// stdout.
    ///
    /// <b>Why this exists.</b> The experiment GUI (<c>analysis/test_gui.py</c>) hardcoded its own
    /// copy of the predictor list, with a comment conceding there was "no runtime way to query the
    /// C# registry from Python. Update both places by hand." Nobody did: by the time this was
    /// written, Core had fifteen registered implementations and the GUI offered three, silently
    /// omitting every reconciler and every playout policy. A researcher using the GUI could not
    /// sweep most of the algorithms this project has built.
    ///
    /// That is the failure this closes. Python cannot load a .NET assembly, but it can run a
    /// process and read JSON, so this is the seam. Everything below is read from the live
    /// <see cref="Registries"/> tables and <see cref="NetworkProfileCatalog"/> — nothing here is a
    /// second list that can drift from them.
    ///
    /// Output is deliberately plain and hand-written rather than serialized through a library:
    /// <c>Teleop.Eval</c> may take NuGet dependencies, but this is a flat object of string arrays
    /// and adding one for it would be disproportionate. Keys are sorted so the output diffs
    /// cleanly.
    /// </summary>
    public static class CatalogCommand
    {
        public static int Run()
        {
            var sb = new StringBuilder();
            sb.AppendLine("{");

            AppendArray(sb, "predictors", Registries.Predictors.Keys, comma: true);
            AppendArray(sb, "reconcilers", Registries.Reconcilers.Keys, comma: true);
            AppendArray(sb, "playoutPolicies", Registries.PlayoutPolicies.Keys, comma: true);
            AppendArray(sb, "codecs", Registries.Codecs.Keys, comma: true);
            AppendArray(sb, "transports", Registries.Transports.Keys, comma: true);
            AppendArray(sb, "arbiters", Registries.Arbiters.Keys, comma: true);
            AppendArray(sb, "namedProfiles", NetworkProfileCatalog.NamedProfiles, comma: true);

            // Trace-driven, and Teleop.Eval's rather than Core's because resolving it reads a file.
            // Listed separately so a caller can tell "a link the catalog can build from parameters"
            // from "a link that needs a recorded trace on disk".
            AppendArray(sb, "traceProfiles", new[] { "synthetic-burst" }, comma: true);

            sb.AppendLine("  \"isolatedAxes\": [");
            for (int i = 0; i < NetworkProfileCatalog.IsolatedAxes.Length; i++)
            {
                NetworkProfileCatalog.IsolatedAxis axis = NetworkProfileCatalog.IsolatedAxes[i];
                string tail = i == NetworkProfileCatalog.IsolatedAxes.Length - 1 ? string.Empty : ",";
                sb.AppendLine(
                    $"    {{ \"name\": {Quote(axis.Name)}, \"unitSuffix\": {Quote(axis.UnitSuffix)} }}{tail}");
            }

            sb.AppendLine("  ]");
            sb.AppendLine("}");

            Console.Write(sb.ToString());
            return 0;
        }

        private static void AppendArray(
            StringBuilder sb, string key, IEnumerable<string> values, bool comma)
        {
            // Sorted ordinally: the registries are insertion-ordered dictionaries, so without this
            // the output would reshuffle whenever someone reorders an entry, producing noisy diffs
            // that look like real changes.
            List<string> sorted = values.OrderBy(v => v, StringComparer.Ordinal).ToList();

            string joined = string.Join(", ", sorted.Select(Quote));
            sb.AppendLine($"  {Quote(key)}: [{joined}]{(comma ? "," : string.Empty)}");
        }

        private static string Quote(string value) =>
            "\"" + value.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
    }
}
