using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;

namespace Teleop.Eval.Verification
{
    /// <summary>
    /// Implements <c>audit</c>: "invariant check over the built assembly" (root CLAUDE.md).
    /// Replaces the exit-70 stub with real checks, split by what each mechanism actually answers
    /// authoritatively:
    /// <list type="bullet">
    /// <item>Source-text regex scans, re-implemented here in C# rather than shelling out to
    /// <c>scripts/hooks/core-guard.sh</c> -- <c>audit</c> must be self-contained under
    /// <c>dotnet run</c> alone, since a fresh clone or CI runner has no guarantee that hook is
    /// installed or executable.</item>
    /// <item>Reflection over the compiled <c>Teleop.Core.dll</c>, for questions text-grep cannot
    /// answer authoritatively: what the assembly actually references, what target framework it
    /// was actually built for, and an absence-of-token property (every public type sealed by
    /// default) that a regex cannot express cleanly.</item>
    /// <item>Plain existence/text checks (csproj settings, stray build output).</item>
    /// </list>
    /// Deliberately does not build Core itself -- "is the currently built artifact correct" is a
    /// distinct question from "did you remember to rebuild", and conflating them would let a
    /// stale-but-passing audit hide a real regression introduced since the last build.
    /// </summary>
    public static class AuditCommand
    {
        public static int Run()
        {
            string? coreDir = FindCoreDirectory();
            if (coreDir == null)
            {
                Console.Error.WriteLine("audit: could not locate core/Teleop.sln from the build output directory.");
                return 66; // EX_NOINPUT
            }

            string sourceDir = Path.Combine(coreDir, "Teleop.Core");
            string repoRoot = Path.GetDirectoryName(coreDir)!;

            string? dllPath = FindNewestBuiltDll(repoRoot);
            if (dllPath == null)
            {
                Console.Error.WriteLine(
                    $"audit: Teleop.Core.dll not found under {Path.Combine(repoRoot, "build", "Teleop.Core", "bin")}. " +
                    "Run `dotnet build Teleop.Core/Teleop.Core.csproj` first.");
                return 66; // EX_NOINPUT
            }

            var findings = new List<string>();

            RunSourceTextChecks(sourceDir, findings);
            RunAssemblyChecks(dllPath, findings);
            RunProjectFileChecks(sourceDir, findings);
            RunRegistryCompletenessCheck(sourceDir, dllPath, findings);
            RunBuildOutputCheck(sourceDir, findings);

            if (findings.Count == 0)
            {
                Console.WriteLine($"audit: PASS -- no invariant violations found in {sourceDir} or {dllPath}.");
                return 0;
            }

            Console.Error.WriteLine($"audit: FAIL -- {findings.Count} finding(s):");
            foreach (string finding in findings)
            {
                Console.Error.WriteLine($"  - {finding}");
            }
            return 1;
        }

        private static string? FindCoreDirectory()
        {
            // Directory.Build.props redirects every project's output to <repoRoot>/build/..., a
            // sibling of core/, not a descendant of it -- so this walks up looking for an
            // ancestor whose "core" subdirectory contains Teleop.sln, rather than looking for
            // Teleop.sln directly in an ancestor.
            string? dir = AppContext.BaseDirectory;
            for (int i = 0; i < 15 && dir != null; i++)
            {
                string candidate = Path.Combine(dir, "core", "Teleop.sln");
                if (File.Exists(candidate))
                {
                    return Path.Combine(dir, "core");
                }
                dir = Path.GetDirectoryName(dir);
            }
            return null;
        }

        private static string? FindNewestBuiltDll(string repoRoot)
        {
            string binDir = Path.Combine(repoRoot, "build", "Teleop.Core", "bin");
            if (!Directory.Exists(binDir))
            {
                return null;
            }

            string[] candidates = Directory.GetFiles(binDir, "Teleop.Core.dll", SearchOption.AllDirectories);
            if (candidates.Length == 0)
            {
                return null;
            }

            return candidates.OrderByDescending(File.GetLastWriteTimeUtc).First();
        }

        // Category 1-6 below mirror scripts/hooks/core-guard.sh's own ordering (highest
        // consequence first), re-implemented in C# so `audit` runs under `dotnet run` alone.
        private static void RunSourceTextChecks(string sourceDir, List<string> findings)
        {
            var checks = new (string Description, Regex Pattern)[]
            {
                ("UnityEngine referenced in Core", new Regex(@"using\s+UnityEngine|UnityEngine\.", RegexOptions.Compiled)),
                ("wall-clock read in Core (use ITimeAuthority)", new Regex(@"DateTime\.(Now|UtcNow)|Environment\.TickCount|new\s+Stopwatch|Time\.time", RegexOptions.Compiled)),
                ("I/O or threading in Core (belongs in the host)", new Regex(@"using\s+System\.IO|using\s+System\.Net|new\s+Thread|Task\.Run", RegexOptions.Compiled)),
                ("unseeded randomness in Core", new Regex(@"new\s+Random\(|Guid\.NewGuid", RegexOptions.Compiled)),
                ("reflection in Core (breaks IL2CPP AOT)", new Regex(@"Activator\.CreateInstance|Reflection\.Emit|Expression\.Compile|\.GetType\(\)\.GetMethod|\.GetMethod\(", RegexOptions.Compiled)),
                ("C#10+ file-scoped namespace in Core (breaks Unity 2022.3 / C# 9)", new Regex(@"^\s*namespace\s+[A-Za-z0-9_.]+\s*;", RegexOptions.Compiled)),
                ("C#10+ global using in Core (breaks Unity 2022.3 / C# 9)", new Regex(@"^\s*global\s+using\s", RegexOptions.Compiled)),
            };

            foreach (string file in Directory.GetFiles(sourceDir, "*.cs", SearchOption.AllDirectories))
            {
                string[] lines = File.ReadAllLines(file);
                for (int i = 0; i < lines.Length; i++)
                {
                    string trimmed = lines[i].TrimStart();
                    if (trimmed.StartsWith("//", StringComparison.Ordinal))
                    {
                        continue;
                    }

                    foreach (var (description, pattern) in checks)
                    {
                        if (pattern.IsMatch(lines[i]))
                        {
                            findings.Add($"{description}: {file}:{i + 1}: {lines[i].Trim()}");
                        }
                    }
                }
            }

            // Collection expressions ("[1, 2]") and primary constructors on classes are C#10+/12
            // features rejected by the compiler itself under this project's <LangVersion>9.0</>
            // -- a successful `dotnet build` of Teleop.Core.csproj is already authoritative
            // evidence neither is present, and regexing for them risks false positives against
            // ordinary array-literal or method-declaration syntax. Not checked here.
        }

        private static void RunAssemblyChecks(string dllPath, List<string> findings)
        {
            Assembly assembly;
            try
            {
                assembly = Assembly.LoadFrom(dllPath);
            }
            catch (Exception ex)
            {
                findings.Add($"could not load {dllPath} for reflection: {ex.Message}");
                return;
            }

            string[] allowedPrefixes = { "System", "netstandard", "mscorlib" };
            foreach (AssemblyName referenced in assembly.GetReferencedAssemblies())
            {
                string name = referenced.Name ?? string.Empty;
                if (!allowedPrefixes.Any(p => name.StartsWith(p, StringComparison.Ordinal)))
                {
                    findings.Add($"{Path.GetFileName(dllPath)} references non-BCL assembly '{name}' (zero NuGet dependencies invariant)");
                }
            }

            var targetFrameworkAttribute = assembly.GetCustomAttribute<System.Runtime.Versioning.TargetFrameworkAttribute>();
            string? frameworkName = targetFrameworkAttribute?.FrameworkName;
            if (frameworkName == null || !frameworkName.Contains(".NETStandard,Version=v2.1", StringComparison.Ordinal))
            {
                findings.Add($"{Path.GetFileName(dllPath)}'s TargetFrameworkAttribute is '{frameworkName ?? "missing"}', expected .NETStandard,Version=v2.1");
            }

            foreach (Type type in assembly.GetExportedTypes())
            {
                if (type.IsInterface || type.IsEnum || type.IsValueType || type.IsSealed)
                {
                    continue; // structs/enums/interfaces are implicitly sealed or N/A; delegates and static classes are IsSealed already.
                }

                if (type.IsClass && !type.IsAbstract)
                {
                    findings.Add($"{type.FullName} is a public class that is not sealed (style: sealed by default)");
                }
            }
        }

        private static void RunProjectFileChecks(string sourceDir, List<string> findings)
        {
            string csprojPath = Path.Combine(sourceDir, "Teleop.Core.csproj");
            if (!File.Exists(csprojPath))
            {
                findings.Add($"{csprojPath} not found");
                return;
            }

            string content = File.ReadAllText(csprojPath);

            if (!content.Contains("<TargetFramework>netstandard2.1</TargetFramework>", StringComparison.Ordinal))
            {
                findings.Add($"{csprojPath} does not declare <TargetFramework>netstandard2.1</TargetFramework>");
            }

            if (!content.Contains("<LangVersion>9.0</LangVersion>", StringComparison.Ordinal))
            {
                findings.Add($"{csprojPath} does not declare <LangVersion>9.0</LangVersion>");
            }

            if (content.Contains("<PackageReference", StringComparison.Ordinal))
            {
                findings.Add($"{csprojPath} declares a <PackageReference> (zero NuGet dependencies invariant)");
            }
        }

        // Axes checked for completeness: every public, non-abstract implementer of the interface
        // must be textually referenced somewhere in Registries.cs. `Transports` is deliberately
        // excluded -- `EmulatedTransport` implements `ITransport` but is documented in
        // `Registry/CLAUDE.md` as intentionally unregistered (its constructor shape doesn't fit
        // the simple `(maxPayloadBytes, capacity)` factory `LoopbackTransport` uses), so an
        // automated scan of that axis would flag a known, accepted gap on every run rather than
        // an actionable one.
        private static readonly (string InterfaceFullName, string RegistryPropertyName)[] RegistryCompletenessAxes =
        {
            ("Teleop.Core.Contracts.IPredictor`1", "Predictors"),
            ("Teleop.Core.Contracts.IReconciler`1", "Reconcilers"),
            ("Teleop.Core.Contracts.ICommandCodec", "Codecs"),
            ("Teleop.Core.Contracts.IPlayoutPolicy`1", "PlayoutPolicies"),
            ("Teleop.Core.Contracts.IAutonomyArbiter", "Arbiters"),
        };

        private static void RunRegistryCompletenessCheck(string sourceDir, string dllPath, List<string> findings)
        {
            string registriesPath = Path.Combine(sourceDir, "Registry", "Registries.cs");
            if (!File.Exists(registriesPath))
            {
                // Not a failure: nothing in this project has more than one implementation to
                // register yet. Reported for visibility, not as a finding.
                Console.WriteLine("audit: registry-completeness: N/A -- Registries.cs not introduced yet.");
                return;
            }

            Assembly assembly;
            try
            {
                assembly = Assembly.LoadFrom(dllPath);
            }
            catch (Exception ex)
            {
                findings.Add($"registry-completeness: could not load {dllPath}: {ex.Message}");
                return;
            }

            string registriesSource = File.ReadAllText(registriesPath);

            foreach (var (interfaceFullName, registryPropertyName) in RegistryCompletenessAxes)
            {
                Type? interfaceType = assembly.GetType(interfaceFullName);
                if (interfaceType == null)
                {
                    findings.Add($"registry-completeness: could not find {interfaceFullName} in {Path.GetFileName(dllPath)} -- has Contracts/ changed?");
                    continue;
                }

                foreach (Type candidate in assembly.GetTypes())
                {
                    if (!candidate.IsPublic || !candidate.IsClass || candidate.IsAbstract)
                    {
                        continue;
                    }

                    bool implementsAxis = candidate.GetInterfaces()
                        .Any(i => (i.IsGenericType ? i.GetGenericTypeDefinition() : i) == interfaceType);

                    if (!implementsAxis)
                    {
                        continue;
                    }

                    if (!registriesSource.Contains(candidate.Name, StringComparison.Ordinal))
                    {
                        findings.Add(
                            $"registry-completeness: {candidate.FullName} implements {interfaceFullName} but is not " +
                            $"referenced anywhere in {registriesPath} -- add it to Registries.{registryPropertyName}");
                    }
                }
            }
        }

        private static void RunBuildOutputCheck(string sourceDir, List<string> findings)
        {
            if (Directory.Exists(Path.Combine(sourceDir, "bin")) || Directory.Exists(Path.Combine(sourceDir, "obj")))
            {
                findings.Add($"bin/ or obj/ exists inside {sourceDir} -- Unity will import a stray DLL and duplicate every type");
            }
        }
    }
}
