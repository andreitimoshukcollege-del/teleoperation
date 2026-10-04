using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Builds the Galaxy XR APK headless: <c>just build-galaxy</c>. The scene check runs first and a
    /// failing check means no build, because a scene whose pinch settings drifted must not reach a
    /// headset that drives a real arm. Output: <c>unity/TeleopXR/Builds/TeleopXR.apk</c> (gitignored).
    /// </summary>
    public static class XrBuild
    {
        public const string ApkPath = "Builds/TeleopXR.apk";

        /// <summary>Batch entry point: <c>-buildTarget Android -executeMethod Teleop.XR.Editor.XrBuild.Run</c>.</summary>
        public static void Run()
        {
            try
            {
                var problems = XrSceneCheck.Check();
                if (problems.Count > 0)
                {
                    Debug.LogError("[build] scene check failed; not building:\n  - " + string.Join("\n  - ", problems));
                    EditorApplication.Exit(1);
                    return;
                }

                if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.Android)
                {
                    EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Android, BuildTarget.Android);
                }

                Directory.CreateDirectory(Path.GetDirectoryName(ApkPath));
                BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                {
                    scenes = new[] { XrLabSceneBuilder.ScenePath },
                    locationPathName = ApkPath,
                    target = BuildTarget.Android,
                    options = BuildOptions.None,
                });

                BuildSummary summary = report.summary;
                if (summary.result != BuildResult.Succeeded)
                {
                    Debug.LogError($"[build] FAILED: {summary.result}, {summary.totalErrors} error(s)");
                    EditorApplication.Exit(1);
                    return;
                }

                Debug.Log($"[build] PASS: {ApkPath} ({new FileInfo(ApkPath).Length / (1024 * 1024)} MB on disk) in {summary.totalTime.TotalSeconds:0} s");
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[build] FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }
    }
}
