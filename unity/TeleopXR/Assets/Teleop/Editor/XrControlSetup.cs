using System;
using System.Linq;
using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Puts clutch control and the off-hand gripper into the scene (docs/adr/0017). Idempotent, and
    /// touches only what it owns:
    /// <list type="bullet">
    /// <item>the drag target loses its grab interactable, rigidbody, collider and old highlight script, and
    /// shrinks to a marker. It stays the bridge's input; the clutch moves it now;</item>
    /// <item><see cref="HandClutchDriver"/> is added next to <see cref="JetRoverOperatorBridge"/> and
    /// wired to the arm rig, the XR Origin and the display's caption line;</item>
    /// <item><see cref="TwinGripper"/> is added on the wrist's twin visual and wired to its fingers.</item>
    /// </list>
    /// <see cref="LabDressing.Apply"/> calls it after every rebuild, because a rebuild replaces the twin.
    /// <b>Teleop → XR → Set Up Clutch Control</b>, or headless:
    /// <c>just _unity-batch TeleopXR Teleop.XR.Editor.XrControlSetup.Run</c>.
    /// </summary>
    public static class XrControlSetup
    {
        public const float MarkerDiameter = 0.02f;

        [MenuItem("Teleop/XR/Set Up Clutch Control")]
        public static void RunFromMenu()
        {
            Apply(SceneManager.GetActiveScene());
            EditorSceneManager.SaveOpenScenes();
        }

        public static void Run()
        {
            try
            {
                Scene scene = EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
                Apply(scene);
                EditorSceneManager.SaveScene(scene);
                Debug.Log("[xr-scene] clutch control set up");
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[xr-scene] FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        public static void Apply(Scene scene)
        {
            var bridge = Object.FindAnyObjectByType<JetRoverOperatorBridge>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no JetRoverOperatorBridge");
            var rig = Object.FindAnyObjectByType<JetRoverArmRig>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no JetRoverArmRig");
            var origin = Object.FindAnyObjectByType<XROrigin>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no XR Origin");
            var target = new SerializedObject(bridge).FindProperty("dragTarget").objectReferenceValue as Transform
                ?? throw new InvalidOperationException("JetRoverOperatorBridge.dragTarget is not assigned");

            StripGrab(target);

            HandClutchDriver driver = bridge.GetComponent<HandClutchDriver>() ?? bridge.gameObject.AddComponent<HandClutchDriver>();
            var driverSo = new SerializedObject(driver);
            driverSo.FindProperty("bridge").objectReferenceValue = bridge;
            driverSo.FindProperty("armRig").objectReferenceValue = rig;
            driverSo.FindProperty("origin").objectReferenceValue = origin;
            TMP_Text caption = Object.FindObjectsByType<TMP_Text>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .FirstOrDefault(t => t.name == "DisplayCaption");
            driverSo.FindProperty("statusLabel").objectReferenceValue = caption;
            driverSo.ApplyModifiedPropertiesWithoutUndo();

            var upper = new SerializedObject(rig).FindProperty("upperPitchPivot").objectReferenceValue as Transform;
            Transform visual = upper != null ? upper.Find(JetRoverTwin.VisualName) : null;
            if (visual != null)
            {
                TwinGripper twin = visual.GetComponent<TwinGripper>() ?? visual.gameObject.AddComponent<TwinGripper>();
                var twinSo = new SerializedObject(twin);
                twinSo.FindProperty("bridge").objectReferenceValue = bridge;
                twinSo.FindProperty("leftFinger").objectReferenceValue = visual.Find(JetRoverTwin.FingerName + "L");
                twinSo.FindProperty("rightFinger").objectReferenceValue = visual.Find(JetRoverTwin.FingerName + "R");
                twinSo.FindProperty("travelMeters").floatValue = JetRoverTwin.FingerTravel;
                twinSo.ApplyModifiedPropertiesWithoutUndo();
            }

            EditorSceneManager.MarkSceneDirty(scene);
        }

        /// <summary>Removes everything that made the target grabbable, dependents first.</summary>
        private static void StripGrab(Transform target)
        {
            // The old GrabHighlight script is gone, so an older scene shows it as a missing script.
            GameObjectUtility.RemoveMonoBehavioursWithMissingScript(target.gameObject);

            // The interactable, then its grab transformers (XRI adds them alongside), then what it required.
            foreach (MonoBehaviour behaviour in target.GetComponents<MonoBehaviour>()
                         .Where(b => b != null && b.GetType().Namespace != null
                                     && b.GetType().Namespace.StartsWith("UnityEngine.XR.Interaction.Toolkit", StringComparison.Ordinal))
                         .OrderBy(b => b is UnityEngine.XR.Interaction.Toolkit.Interactables.XRBaseInteractable ? 0 : 1)
                         .ToList())
            {
                Object.DestroyImmediate(behaviour);
            }

            foreach (Collider collider in target.GetComponents<Collider>())
            {
                Object.DestroyImmediate(collider);
            }

            if (target.TryGetComponent(out Rigidbody body))
            {
                Object.DestroyImmediate(body);
            }

            target.localScale = Vector3.one * MarkerDiameter;
            EditorUtility.SetDirty(target.gameObject);
        }
    }
}
