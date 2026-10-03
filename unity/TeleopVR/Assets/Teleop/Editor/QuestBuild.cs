using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Editor
{
    /// <summary>
    /// Builds the Quest APK headless: <c>just build-quest</c> (Unity 2022.3, ADR 0001). The Galaxy XR
    /// APK comes from the other project, <c>just build-galaxy</c> (docs/adr/0016). The scene check runs
    /// first, and a failing check means no build.
    ///
    /// <para>The scenes are this project's Build Settings, enabled ones only, as the editor's own Build
    /// would use them. To ship <c>JetRoverLab</c>, tick it there. Output:
    /// <c>unity/TeleopVR/Builds/TeleopVR-Quest.apk</c> (gitignored).</para>
    ///
    /// <para>Building for Android from a Library last used for Windows (Link from the editor)
    /// re-imports assets for Android once; the first build is slow, later ones are not.</para>
    /// </summary>
    public static class QuestBuild
    {
        public const string ApkPath = "Builds/TeleopVR-Quest.apk";

        /// <summary>Batch entry point: <c>-buildTarget Android -executeMethod Teleop.Editor.QuestBuild.Run</c>.</summary>
        public static void Run()
        {
            try
            {
                var problems = SceneIntegrityCheck.CheckScenes();
                if (problems.Count > 0)
                {
                    Debug.LogError("[build] scene check failed; not building:\n  - " + string.Join("\n  - ", problems));
                    EditorApplication.Exit(1);
                    return;
                }

                string[] scenes = EditorBuildSettings.scenes.Where(s => s.enabled).Select(s => s.path).ToArray();
                if (scenes.Length == 0)
                {
                    Debug.LogError("[build] no enabled scenes in Build Settings");
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
                    scenes = scenes,
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

                Debug.Log($"[build] PASS: {ApkPath} ({new FileInfo(ApkPath).Length / (1024 * 1024)} MB on disk, scenes: {string.Join(", ", scenes)}) in {summary.totalTime.TotalSeconds:0} s");
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
