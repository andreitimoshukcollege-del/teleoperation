using System.Collections.Generic;
using System.Text;
using Teleop.Bridge;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Editor
{
    /// <summary>
    /// A headless gate for this project's scenes: every scene in <see cref="Scenes"/> must open with
    /// zero missing scripts, and the JetRover scenes must still have their Bridge wiring.
    ///
    /// <para>It exists because moving Bridge into the shared <c>com.teleop.bridge</c> package
    /// (docs/adr/0016) is only safe if every script GUID survived the move, and a broken GUID does not
    /// fail a build. It shows up as a "missing script" in a scene nobody happens to open. Run it with
    /// <c>just unity-check-vr</c>, or <b>Teleop → Check Scenes</b> in the editor.</para>
    ///
    /// <para>Read-only: scenes are opened, inspected and closed without saving. It exits non-zero when
    /// it finds a problem *or* cannot look (a listed scene is missing), per root CLAUDE.md
    /// invariant 10.</para>
    /// </summary>
    public static class SceneIntegrityCheck
    {
        private static readonly string[] Scenes =
        {
            "Assets/Scenes/JetRoverControl.unity",
            "Assets/Scenes/JetRoverLab.unity",
            "Assets/Scenes/SampleScene.unity",
        };

        [MenuItem("Teleop/Check Scenes")]
        public static void RunFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                return;
            }

            string previous = SceneManager.GetActiveScene().path;
            List<string> problems = Check();
            if (!string.IsNullOrEmpty(previous))
            {
                EditorSceneManager.OpenScene(previous, OpenSceneMode.Single);
            }

            Report(problems);
        }

        /// <summary>Batch entry point: <c>-executeMethod Teleop.Editor.SceneIntegrityCheck.Run</c>.</summary>
        public static void Run()
        {
            List<string> problems;
            try
            {
                problems = Check();
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[scene-check] could not run: {e}");
                EditorApplication.Exit(2);
                return;
            }

            Report(problems);
            EditorApplication.Exit(problems.Count == 0 ? 0 : 1);
        }

        private static List<string> Check()
        {
            var problems = new List<string>();
            foreach (string path in Scenes)
            {
                if (AssetDatabase.LoadAssetAtPath<SceneAsset>(path) == null)
                {
                    problems.Add($"{path}: not found");
                    continue;
                }

                Scene scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
                int missing = 0;
                foreach (GameObject root in scene.GetRootGameObjects())
                {
                    foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
                    {
                        int count = GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject);
                        if (count > 0)
                        {
                            missing += count;
                            problems.Add($"{path}: {count} missing script(s) on '{PathOf(t)}'");
                        }
                    }
                }

                if (path.Contains("JetRover"))
                {
                    CheckJetRoverWiring(path, problems);
                }

                Debug.Log($"[scene-check] {path}: {missing} missing script(s)");
            }

            return problems;
        }

        private static void CheckJetRoverWiring(string path, List<string> problems)
        {
            JetRoverOperatorBridge bridge = Object.FindAnyObjectByType<JetRoverOperatorBridge>(FindObjectsInactive.Include);
            if (bridge == null)
            {
                problems.Add($"{path}: no JetRoverOperatorBridge");
                return;
            }

            var so = new SerializedObject(bridge);
            RequireReference(path, so, "dragTarget", problems);
            RequireReference(path, so, "armRig", problems);

            JetRoverArmRig rig = Object.FindAnyObjectByType<JetRoverArmRig>(FindObjectsInactive.Include);
            if (rig == null)
            {
                problems.Add($"{path}: no JetRoverArmRig");
            }
            else
            {
                RequireReference(path, new SerializedObject(rig), "baseAnchor", problems);
            }

            CameraFeedBridge feed = Object.FindAnyObjectByType<CameraFeedBridge>(FindObjectsInactive.Include);
            if (feed != null)
            {
                RequireReference(path, new SerializedObject(feed), "panel", problems);
            }
        }

        private static void RequireReference(string path, SerializedObject so, string field, List<string> problems)
        {
            SerializedProperty property = so.FindProperty(field);
            if (property == null)
            {
                problems.Add($"{path}: {so.targetObject.GetType().Name} has no serialized field '{field}'");
            }
            else if (property.objectReferenceValue == null)
            {
                problems.Add($"{path}: {so.targetObject.GetType().Name}.{field} is not assigned");
            }
        }

        private static void Report(List<string> problems)
        {
            if (problems.Count == 0)
            {
                Debug.Log($"[scene-check] PASS: {Scenes.Length} scenes, no missing scripts, JetRover wiring intact");
                return;
            }

            var message = new StringBuilder($"[scene-check] FAIL: {problems.Count} problem(s)");
            foreach (string problem in problems)
            {
                message.Append("\n  - ").Append(problem);
            }

            Debug.LogError(message.ToString());
        }

        private static string PathOf(Transform t)
        {
            var path = new StringBuilder(t.name);
            for (Transform p = t.parent; p != null; p = p.parent)
            {
                path.Insert(0, p.name + "/");
            }

            return path.ToString();
        }
    }
}
