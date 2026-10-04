using System;
using System.IO;
using System.Linq;
using Teleop.Bridge;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Dresses <c>JetRoverLabXR</c> as the night-shift lab: the room, the JetRover twin, the camera
    /// display, and the lighting. <b>Teleop → XR → Rebuild Lab Dressing</b>, or <c>just unity-lab-xr</c>
    /// headless (with a bake and preview screenshots).
    ///
    /// <para><b>What a rebuild touches, and only that:</b></para>
    /// <list type="bullet">
    /// <item>the <c>LabDressing</c> root, deleted and built again;</item>
    /// <item>the <c>TwinVisual</c> children under the arm's pivots;</item>
    /// <item>the old placeholder floor, bench and panel, if they are still there;</item>
    /// <item>three references: <c>CameraFeedBridge.panel</c> (the display's screen),
    /// <c>JetRoverArmRig.reachWarningRenderer</c> (the wrist LED), and the old segment cubes'
    /// renderers, disabled but kept.</item>
    /// </list>
    /// <para>Everything else in the scene is left as it is, including the rig, the target and the
    /// bridges. Edits made by hand <i>inside</i> <c>LabDressing</c> are lost on the next rebuild; to
    /// change the lab for good, change the builders.</para>
    /// </summary>
    public static class LabDressing
    {
        public const string RootName = "LabDressing";
        private const string ScreenMaterialPath = "Assets/Teleop/Materials/CameraPanel.mat";

        [MenuItem("Teleop/XR/Rebuild Lab Dressing")]
        public static void RebuildFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Scene scene = EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
            Apply(scene);
            EditorSceneManager.SaveScene(scene);
        }

        [MenuItem("Teleop/XR/Bake Lab Lighting")]
        public static void BakeFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Scene scene = EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
            if (LabLighting.Bake()) EditorSceneManager.SaveScene(scene);
        }

        [MenuItem("Teleop/XR/Capture Lab Previews")]
        public static void CaptureFromMenu()
        {
            string dir = DefaultOutDir();
            string sheet = LabCapture.Capture(dir);
            EditorUtility.RevealInFinder(sheet);
        }

        /// <summary>
        /// Batch entry point: <c>-executeMethod Teleop.XR.Editor.LabDressing.Run</c>, with optional
        /// <c>-labNoRebuild</c>, <c>-labBake</c>, <c>-labCapture</c>, and <c>-labOut &lt;dir&gt;</c>.
        /// </summary>
        public static void Run()
        {
            try
            {
                string[] args = Environment.GetCommandLineArgs();
                bool rebuild = !args.Contains("-labNoRebuild"), bake = args.Contains("-labBake"), capture = args.Contains("-labCapture");
                int outIndex = Array.IndexOf(args, "-labOut");
                string outDir = outIndex >= 0 && outIndex + 1 < args.Length ? args[outIndex + 1] : DefaultOutDir();

                Scene scene = EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
                if (rebuild)
                {
                    Apply(scene);
                    EditorSceneManager.SaveScene(scene);
                }

                if (bake)
                {
                    if (!LabLighting.Bake()) throw new InvalidOperationException("the lighting bake failed");
                    EditorSceneManager.SaveScene(scene);
                }

                if (capture)
                {
                    string sheet = LabCapture.Capture(outDir);
                    Debug.Log($"[lab] RESULT ok; contact sheet: {sheet}");
                }
                else
                {
                    Debug.Log("[lab] RESULT ok");
                }

                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[lab] RESULT FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        /// <summary>Builds the dressing into <paramref name="scene"/>, replacing a previous one.</summary>
        public static void Apply(Scene scene)
        {
            JetRoverArmRig rig = Object.FindAnyObjectByType<JetRoverArmRig>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no JetRoverArmRig; build it with `just unity-scene-xr` first");
            JetRoverOperatorBridge robot = Object.FindAnyObjectByType<JetRoverOperatorBridge>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no JetRoverOperatorBridge");
            CameraFeedBridge feed = Object.FindAnyObjectByType<CameraFeedBridge>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no CameraFeedBridge");
            Material screenMaterial = AssetDatabase.LoadAssetAtPath<Material>(ScreenMaterialPath)
                ?? throw new InvalidOperationException($"{ScreenMaterialPath} is missing; rebuild the scene with `just unity-scene-xr rebuild=true`");

            var rigSo = new SerializedObject(rig);
            Transform Pivot(string field) => rigSo.FindProperty(field).objectReferenceValue as Transform
                ?? throw new InvalidOperationException($"JetRoverArmRig.{field} is not assigned");
            Transform yaw = Pivot("baseYawPivot"), lower = Pivot("lowerPitchPivot"), middle = Pivot("middlePitchPivot"), upper = Pivot("upperPitchPivot");

            // Remove what a previous build (or the plain scene) left.
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                if (root.name == RootName || root.name == "Room" || root.name == "CameraPanel" || root.name == "ConnectionStatus")
                {
                    Object.DestroyImmediate(root);
                }
            }

            foreach (Transform pivot in new[] { yaw, lower, middle, upper })
            {
                foreach (Transform child in pivot.Cast<Transform>().Where(c => c.name == JetRoverTwin.VisualName).ToList())
                {
                    Object.DestroyImmediate(child.gameObject);
                }
            }

            // The old cube segments stay (they are the rig as TeleopVR has it) but are no longer drawn.
            foreach (Renderer segment in new[] { lower, middle, upper }.SelectMany(p => p.GetComponentsInChildren<Renderer>(true))
                         .Where(r => r.name.EndsWith("Segment", StringComparison.Ordinal)))
            {
                segment.enabled = false;
                EditorUtility.SetDirty(segment);
            }

            LabMeshLibrary.Begin();
            LabPalette palette = LabPalette.Create();
            Transform dressing = new GameObject(RootName).transform;

            LabRoom.Result room = LabRoom.Build(dressing, palette);
            Transform shoulder = lower.parent;
            Renderer wristLed = JetRoverTwin.Build(dressing, palette, yaw, lower, middle, upper, shoulder.localPosition);
            LabDisplay.Result display = LabDisplay.Build(dressing, palette, screenMaterial, feed, robot);

            var lamp = room.StatusRing.gameObject.AddComponent<LinkStatusLamp>();
            var lampSo = new SerializedObject(lamp);
            lampSo.FindProperty("robot").objectReferenceValue = robot;
            SetArray(lampSo.FindProperty("lamps"), new Object[] { room.StatusRing });
            lampSo.ApplyModifiedPropertiesWithoutUndo();

            var blink = dressing.Find("Room/Racks").gameObject.AddComponent<BlinkingLights>();
            var blinkSo = new SerializedObject(blink);
            SetArray(blinkSo.FindProperty("lamps"), room.RackLeds.Cast<Object>().ToArray());
            blinkSo.ApplyModifiedPropertiesWithoutUndo();

            Light key = Object.FindObjectsByType<Light>(FindObjectsInactive.Include).FirstOrDefault(l => l.type == LightType.Directional);
            LabLighting.Build(dressing, scene, key);
            LabMeshLibrary.End();

            // Wiring into objects this builder does not own.
            var feedSo = new SerializedObject(feed);
            feedSo.FindProperty("panel").objectReferenceValue = display.Screen;
            feedSo.FindProperty("panelHeightMeters").floatValue = LabDisplay.ScreenHeight;
            feedSo.FindProperty("armRig").objectReferenceValue = null; // the display places itself
            feedSo.ApplyModifiedPropertiesWithoutUndo();
            rigSo.FindProperty("reachWarningRenderer").objectReferenceValue = wristLed;
            rigSo.ApplyModifiedPropertiesWithoutUndo();

            EditorSceneManager.MarkSceneDirty(scene);
            Debug.Log($"[lab] dressing built: {LabMeshLibrary.Triangles} generated triangles");
        }

        private static void SetArray(SerializedProperty property, Object[] values)
        {
            property.arraySize = values.Length;
            for (int i = 0; i < values.Length; i++)
            {
                property.GetArrayElementAtIndex(i).objectReferenceValue = values[i];
            }
        }

        private static string DefaultOutDir() =>
            Path.Combine(Path.GetTempPath(), "teleop-lab", DateTime.Now.ToString("yyyyMMdd-HHmmss"));
    }
}
