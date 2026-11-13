using System;
using System.IO;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Builds the Galaxy XR APKs headless. The scene check runs first and a failing check means no build, because a scene
    /// whose pinch settings drifted must not reach a headset that drives a real arm. Outputs (gitignored) under
    /// <c>unity/TeleopXR/Builds/</c>:
    /// <list type="bullet">
    /// <item><c>teleop-jetrover-robot.apk</c> (<c>just build-galaxy</c>): "Teleop JetRover Robot", app id
    /// <see cref="XrProjectSetup.ApplicationId"/> (the id the app always had, so its pushed config, models and link traces
    /// stay where they were).</item>
    /// <item><c>teleop-jetrover-sim.apk</c> (<c>just build-galaxy-sim</c>): "Teleop JetRover Sim", app id
    /// <see cref="SimApplicationId"/>, for the Isaac JetRover (docs/adr/0034). The same scene; its built-in connection
    /// default has discovery off (it never finds the real robot, docs/adr/0022) and <c>-teleopRemoteHost</c> as its
    /// address. Both apps install side by side, each with its own files directory.</item>
    /// <item><c>TeleopXR-FR3.apk</c> (<c>just build-galaxy-fr3</c>): the FR3 Duo's scene, under the robot app's id.</item>
    /// </list>
    /// A variant's name, id and connection default are set for its build only, and put back afterwards.
    /// </summary>
    public static class XrBuild
    {
        public const string ApkPath = "Builds/teleop-jetrover-robot.apk";
        public const string SimApkPath = "Builds/teleop-jetrover-sim.apk";
        public const string Fr3ApkPath = "Builds/TeleopXR-FR3.apk";

        public const string RobotProductName = "Teleop JetRover Robot";
        public const string SimProductName = "Teleop JetRover Sim";
        public const string SimApplicationId = XrProjectSetup.ApplicationId + ".sim";

        /// <summary>The Bridge's built-in connection default, which the sim build replaces for its build only.</summary>
        private const string ConnectionResource = "Packages/com.teleop.bridge/Runtime/Resources/jetrover_connection.json";

        /// <summary>Batch entry point: <c>-buildTarget Android -executeMethod Teleop.XR.Editor.XrBuild.Run</c>.</summary>
        public static void Run() => Build(XrSceneCheck.Check, XrLabSceneBuilder.ScenePath, ApkPath, RobotProductName,
            XrProjectSetup.ApplicationId, null);

        /// <summary>
        /// The Isaac JetRover's app: <c>-executeMethod Teleop.XR.Editor.XrBuild.RunSim -teleopRemoteHost &lt;address&gt;</c>
        /// (the GPU machine as the headset reaches it). Refused without an address.
        /// </summary>
        public static void RunSim()
        {
            string host = Arg("-teleopRemoteHost");
            if (string.IsNullOrWhiteSpace(host) || !Regex.IsMatch(host, @"^[A-Za-z0-9.\-:]+$"))
            {
                Debug.LogError("[build] FAILED: the sim build needs -teleopRemoteHost <address> (the GPU machine running just isaac-jetrover-host)");
                EditorApplication.Exit(1);
                return;
            }
            Build(XrSceneCheck.Check, XrLabSceneBuilder.ScenePath, SimApkPath, SimProductName, SimApplicationId, host);
        }

        /// <summary>
        /// The FR3 Duo's APK (docs/adr/0029, <c>just build-galaxy-fr3</c>): its own scene and its own check, beside
        /// the JetRover's, which it leaves alone. Output: <c>Builds/TeleopXR-FR3.apk</c>.
        /// </summary>
        public static void RunFr3() => Build(XrFr3SceneCheck.Check, XrFr3SceneBuilder.ScenePath, Fr3ApkPath,
            XrProjectSetup.ProductName, XrProjectSetup.ApplicationId, null);

        private static string Arg(string name)
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
            {
                if (args[i] == name) return args[i + 1];
            }
            return null;
        }

        /// <summary>
        /// <paramref name="simHost"/>: null for the robot's own default; otherwise the connection default is rewritten for
        /// this build with discovery off and this address, and restored byte for byte afterwards.
        /// </summary>
        private static void Build(Func<System.Collections.Generic.List<string>> check, string scene, string apkPath,
            string productName, string applicationId, string simHost)
        {
            string connectionPath = null;
            byte[] connectionOriginal = null;
            int exit = 1;
            try
            {
                var problems = check();
                if (problems.Count > 0)
                {
                    Debug.LogError("[build] scene check failed; not building:\n  - " + string.Join("\n  - ", problems));
                    return;
                }

                if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android)
                {
                    EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Android, BuildTarget.Android);
                }

                if (simHost != null)
                {
                    // The Bridge is a file: package (../../Teleop.Bridge); its folder on disk is the package's resolved path.
                    var bridge = UnityEditor.PackageManager.PackageInfo.FindForAssetPath(ConnectionResource);
                    if (bridge == null) throw new InvalidOperationException($"{ConnectionResource}: the Bridge package is not resolved");
                    connectionPath = Path.Combine(bridge.resolvedPath, "Runtime", "Resources", "jetrover_connection.json");
                    connectionOriginal = File.ReadAllBytes(connectionPath);
                    string json = System.Text.Encoding.UTF8.GetString(connectionOriginal);
                    string sim = Replace(json, "RemoteHost", "\"" + simHost + "\"");
                    sim = Replace(sim, "DiscoverRobot", "false");
                    File.WriteAllText(connectionPath, sim, new System.Text.UTF8Encoding(false));
                    AssetDatabase.ImportAsset(ConnectionResource, ImportAssetOptions.ForceUpdate);
                    Debug.Log($"[build] sim connection default: RemoteHost {simHost}, DiscoverRobot false");
                }

                PlayerSettings.productName = productName;
                PlayerSettings.SetApplicationIdentifier(NamedBuildTarget.Android, applicationId);

                Directory.CreateDirectory(Path.GetDirectoryName(apkPath));
                BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                {
                    scenes = new[] { scene },
                    locationPathName = apkPath,
                    target = BuildTarget.Android,
                    options = BuildOptions.None,
                });

                BuildSummary summary = report.summary;
                if (summary.result != BuildResult.Succeeded)
                {
                    Debug.LogError($"[build] FAILED: {summary.result}, {summary.totalErrors} error(s)");
                    return;
                }

                Debug.Log($"[build] PASS: {apkPath} \"{productName}\" ({applicationId}), {new FileInfo(apkPath).Length / (1024 * 1024)} MB on disk, in {summary.totalTime.TotalSeconds:0} s");
                exit = 0;
            }
            catch (Exception e)
            {
                Debug.LogError($"[build] FAILED: {e}");
            }
            finally
            {
                PlayerSettings.productName = XrProjectSetup.ProductName;
                PlayerSettings.SetApplicationIdentifier(NamedBuildTarget.Android, XrProjectSetup.ApplicationId);
                if (connectionOriginal != null)
                {
                    File.WriteAllBytes(connectionPath, connectionOriginal);
                    AssetDatabase.ImportAsset(ConnectionResource, ImportAssetOptions.ForceUpdate);
                }
                EditorApplication.Exit(exit);
            }
        }

        /// <summary>Replaces one top-level <c>"key": value</c> of the connection JSON; throws when the key is not there once.</summary>
        private static string Replace(string json, string key, string value)
        {
            var pattern = new Regex("(\"" + key + "\"\\s*:\\s*)(\"[^\"]*\"|true|false)");
            if (pattern.Matches(json).Count != 1) throw new InvalidOperationException($"{ConnectionResource}: \"{key}\" is not there exactly once");
            return pattern.Replace(json, m => m.Groups[1].Value + value, 1);
        }
    }
}
