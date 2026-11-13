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
    /// <item>Reflection over the compiled <c>Teleop.Core.dll</c> and <c>Teleop.RobotArm.dll</c>, for questions text-grep cannot
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

            string repoRoot = Path.GetDirectoryName(coreDir)!;
            var findings = new List<string>();
            var scanned = new List<string>();

            // Core and RobotArm both ship to the headset as Unity packages and are held to the same rules (root CLAUDE.md
            // invariants 1-7). RobotArm may name System.Net's value types (IPAddress: its discovery and mDNS codecs) and
            // may reference Core; it may not open a socket. Registry and impairment checks are Core's alone.
            foreach (Package package in Packages)
            {
                string sourceDir = Path.Combine(coreDir, package.Name);
                string? dllPath = FindNewestBuiltDll(repoRoot, package.Name);
                if (dllPath == null)
                {
                    Console.Error.WriteLine(
                        $"audit: {package.Name}.dll not found under {Path.Combine(repoRoot, "build", package.Name, "bin")}. " +
                        $"Run `dotnet build {package.Name}/{package.Name}.csproj` first.");
                    return 66; // EX_NOINPUT
                }

                RunSourceTextChecks(sourceDir, package, findings);
                RunAssemblyChecks(dllPath, package, findings);
                RunProjectFileChecks(sourceDir, package.Name, findings);
                RunBuildOutputCheck(sourceDir, findings);
                if (package.Name == "Teleop.Core")
                {
                    RunRegistryCompletenessCheck(sourceDir, dllPath, findings);
                    RunImpairmentReachabilityCheck(sourceDir, dllPath, findings);
                }

                scanned.Add($"{sourceDir} + {dllPath}");
            }

            if (findings.Count == 0)
            {
                Console.WriteLine($"audit: PASS -- no invariant violations found in {string.Join("; ", scanned)}.");
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

        private static string? FindNewestBuiltDll(string repoRoot, string name)
        {
            string binDir = Path.Combine(repoRoot, "build", name, "bin");
            if (!Directory.Exists(binDir))
            {
                return null;
            }

            string[] candidates = Directory.GetFiles(binDir, name + ".dll", SearchOption.AllDirectories);
            if (candidates.Length == 0)
            {
                return null;
            }

            return candidates.OrderByDescending(File.GetLastWriteTimeUtc).First();
        }

        /// <summary>A Unity-shipped package the audit holds to Core's invariants.</summary>
        private sealed class Package
        {
            public Package(string name, string[] allowedReferences, Regex ioPattern)
            {
                Name = name;
                AllowedReferences = allowedReferences;
                IoPattern = ioPattern;
            }

            public string Name { get; }

            /// <summary>Assembly-name prefixes the built DLL may reference, beyond the BCL.</summary>
            public string[] AllowedReferences { get; }

            public Regex IoPattern { get; }
        }

        private static readonly Package[] Packages =
        {
            new Package("Teleop.Core", Array.Empty<string>(),
                new Regex(@"using\s+System\.IO|using\s+System\.Net|new\s+Thread|Task\.Run", RegexOptions.Compiled)),
            new Package("Teleop.RobotArm", new[] { "Teleop.Core" },
                new Regex(@"using\s+System\.IO|System\.Net\.Sockets\.(Socket|UdpClient|TcpClient|TcpListener)\b|\b(Socket|UdpClient|TcpClient|TcpListener)\s*\(|\bDns\.|new\s+Thread|Task\.Run", RegexOptions.Compiled)),
        };

        // Category 1-6 below mirror scripts/hooks/core-guard.sh's own ordering (highest
        // consequence first), re-implemented in C# so `audit` runs under `dotnet run` alone.
        private static void RunSourceTextChecks(string sourceDir, Package package, List<string> findings)
        {
            string where = package.Name == "Teleop.Core" ? "Core" : package.Name;
            var checks = new (string Description, Regex Pattern)[]
            {
                ($"UnityEngine referenced in {where}", new Regex(@"using\s+UnityEngine|UnityEngine\.", RegexOptions.Compiled)),
                ($"wall-clock read in {where} (use ITimeAuthority)", new Regex(@"DateTime\.(Now|UtcNow)|Environment\.TickCount|new\s+Stopwatch|Time\.time", RegexOptions.Compiled)),
                ($"I/O or threading in {where} (belongs in the host)", package.IoPattern),
                ($"unseeded randomness in {where}", new Regex(@"new\s+Random\(|Guid\.NewGuid", RegexOptions.Compiled)),
                ($"reflection in {where} (breaks IL2CPP AOT)", new Regex(@"Activator\.CreateInstance|Reflection\.Emit|Expression\.Compile|\.GetType\(\)\.GetMethod|\.GetMethod\(", RegexOptions.Compiled)),
                ($"C#10+ file-scoped namespace in {where} (breaks Unity / C# 9)", new Regex(@"^\s*namespace\s+[A-Za-z0-9_.]+\s*;", RegexOptions.Compiled)),
                ($"C#10+ global using in {where} (breaks Unity / C# 9)", new Regex(@"^\s*global\s+using\s", RegexOptions.Compiled)),
            };

            if (!Directory.Exists(sourceDir))
            {
                findings.Add($"source folder {sourceDir} not found");
                return;
            }

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
            // -- a successful `dotnet build` of the csproj is already authoritative
            // evidence neither is present, and regexing for them risks false positives against
            // ordinary array-literal or method-declaration syntax. Not checked here.
        }

        private static void RunAssemblyChecks(string dllPath, Package package, List<string> findings)
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

            string[] bcl = { "System", "netstandard", "mscorlib" };
            foreach (AssemblyName referenced in assembly.GetReferencedAssemblies())
            {
                string name = referenced.Name ?? string.Empty;
                if (!bcl.Any(p => name.StartsWith(p, StringComparison.Ordinal)) &&
                    !package.AllowedReferences.Any(a => string.Equals(name, a, StringComparison.Ordinal)))
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

        private static void RunProjectFileChecks(string sourceDir, string name, List<string> findings)
        {
            string csprojPath = Path.Combine(sourceDir, name + ".csproj");
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
            ("Teleop.Core.Contracts.ICommandAwarePredictor", "JointPredictors"),
            ("Teleop.Core.Contracts.IReconciler`1", "Reconcilers"),
            ("Teleop.Core.Contracts.ICommandCodec", "Codecs"),
            ("Teleop.Core.Contracts.IPlayoutPolicy`1", "PlayoutPolicies"),
            ("Teleop.Core.Contracts.IAutonomyArbiter", "Arbiters"),
            ("Teleop.Core.Contracts.INetworkForecaster", "NetworkForecasters"),
        };

        private static void RunRegistryCompletenessCheck(string sourceDir, string dllPath, List<string> findings)
        {
            string registriesPath = Path.Combine(sourceDir, "Registry", "Registries.cs");
            if (!File.Exists(registriesPath))
            {
                // A finding, not "N/A": every axis has registered implementations, so a missing file means a moved file or
                // a wrong path, and passing then would be a gate that checks nothing (root CLAUDE.md invariant 10).
                findings.Add($"registry-completeness: {registriesPath} not found -- nothing could be checked");
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

            string registriesSource = StripLineComments(File.ReadAllText(registriesPath));

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

                    if (!NamesType(registriesSource, candidate.Name))
                    {
                        findings.Add(
                            $"registry-completeness: {candidate.FullName} implements {interfaceFullName} but is not " +
                            $"referenced anywhere in {registriesPath} -- add it to Registries.{registryPropertyName}");
                    }
                }
            }
        }

        /// <summary>
        /// Every <c>INetworkImpairment</c> implementation must be constructed by name somewhere in
        /// <c>Transport/NetworkProfileCatalog.cs</c>.
        ///
        /// This is the impairment axis's analogue of the registry-completeness check above, and it
        /// exists because that check reads a hardcoded list of <c>Contracts/</c> interfaces which
        /// deliberately does not include this one. Impairments get no <c>Registries.cs</c> table:
        /// their five constructors have five different shapes, and forcing them into one factory
        /// signature would produce a stringly-typed parameter bag -- the same objection
        /// <c>Registry/CLAUDE.md</c> already raises against registering <c>EmulatedTransport</c>.
        /// The catalog is this axis's registry, mapping a name to a whole set rather than to one
        /// type (docs/adr/0013).
        ///
        /// It doubles as an IL2CPP reachability check. Full AOT strips types nothing references and
        /// has no runtime codegen to recover them, so an impairment no code constructs directly is
        /// one that can vanish on device while every headless test still passes.
        /// </summary>
        private static void RunImpairmentReachabilityCheck(string sourceDir, string dllPath, List<string> findings)
        {
            string catalogPath = Path.Combine(sourceDir, "Transport", "NetworkProfileCatalog.cs");
            if (!File.Exists(catalogPath))
            {
                findings.Add($"impairment-reachability: {catalogPath} not found -- nothing could be checked");
                return;
            }

            Assembly assembly;
            try
            {
                assembly = Assembly.LoadFrom(dllPath);
            }
            catch (Exception ex)
            {
                findings.Add($"impairment-reachability: could not load {dllPath}: {ex.Message}");
                return;
            }

            Type? impairmentInterface = assembly.GetType("Teleop.Core.Contracts.INetworkImpairment");
            if (impairmentInterface == null)
            {
                findings.Add($"impairment-reachability: could not find Teleop.Core.Contracts.INetworkImpairment in {Path.GetFileName(dllPath)} -- has Contracts/ changed?");
                return;
            }

            string catalogSource = StripLineComments(File.ReadAllText(catalogPath));

            foreach (Type candidate in assembly.GetTypes())
            {
                if (!candidate.IsPublic || !candidate.IsClass || candidate.IsAbstract)
                {
                    continue;
                }

                if (!candidate.GetInterfaces().Any(i => i == impairmentInterface))
                {
                    continue;
                }

                if (!NamesType(catalogSource, candidate.Name))
                {
                    findings.Add(
                        $"impairment-reachability: {candidate.FullName} implements INetworkImpairment but is " +
                        $"never constructed in {catalogPath} -- add it there, or IL2CPP may strip it and no " +
                        "headless test will notice");
                }
            }
        }

        /// <summary>
        /// Whether <paramref name="source"/> names the type as a whole identifier: <c>Kalman</c> is not found inside
        /// <c>KalmanPosePredictor</c>, and a generic type's metadata name (<c>HoldLast`1</c>) is matched by its source name.
        /// </summary>
        internal static bool NamesType(string source, string metadataName)
        {
            int tick = metadataName.IndexOf('`');
            string name = tick < 0 ? metadataName : metadataName.Substring(0, tick);
            return Regex.IsMatch(source, @"(?<![A-Za-z0-9_])" + Regex.Escape(name) + @"(?![A-Za-z0-9_])");
        }

        /// <summary>The source without its <c>//</c> comments, so a type named only in a comment does not count as used.</summary>
        internal static string StripLineComments(string source)
        {
            var lines = source.Split('\n');
            for (int i = 0; i < lines.Length; i++)
            {
                int at = lines[i].IndexOf("//", StringComparison.Ordinal);
                if (at >= 0) lines[i] = lines[i].Substring(0, at);
            }

            return string.Join("\n", lines);
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
