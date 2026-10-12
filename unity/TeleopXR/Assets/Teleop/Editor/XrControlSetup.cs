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
    /// <item><see cref="TwinGripper"/> is added on the wrist's twin visual and wired to its fingers and
    /// its roll group;</item>
    /// <item>the robot's start/stop button (<see cref="MotionToggleButton"/>) is built on the bench's
    /// front-left corner, and the bridge is set to start with motion <b>off</b>.</item>
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

            // Off until the button is pressed: nothing reaches the robot at launch.
            var bridgeSo = new SerializedObject(bridge);
            bridgeSo.FindProperty("motionEnabledAtStart").boolValue = false;
            bridgeSo.ApplyModifiedPropertiesWithoutUndo();
            BuildMotionButton(scene, bridge, origin);

            var upper = new SerializedObject(rig).FindProperty("upperPitchPivot").objectReferenceValue as Transform;
            Transform visual = upper != null ? upper.Find(JetRoverTwin.VisualName) : null;
            if (visual != null)
            {
                TwinGripper twin = visual.GetComponent<TwinGripper>() ?? visual.gameObject.AddComponent<TwinGripper>();
                var twinSo = new SerializedObject(twin);
                twinSo.FindProperty("bridge").objectReferenceValue = bridge;
                Transform roll = visual.Find(JetRoverTwin.RollName);
                twinSo.FindProperty("rollPivot").objectReferenceValue = roll;
                twinSo.FindProperty("leftFinger").objectReferenceValue = roll != null ? roll.Find(JetRoverTwin.FingerName + "L") : null;
                twinSo.FindProperty("rightFinger").objectReferenceValue = roll != null ? roll.Find(JetRoverTwin.FingerName + "R") : null;
                twinSo.FindProperty("travelMeters").floatValue = JetRoverTwin.FingerTravel;
                twinSo.ApplyModifiedPropertiesWithoutUndo();
            }

            EditorSceneManager.MarkSceneDirty(scene);
        }

        public const string MotionButtonName = "MotionButton";

        /// <summary>
        /// A red/green push button on the bench's front-left corner, out of the arm's reach and to the
        /// left of where the right hand clutches, with a label behind it. Rebuilt on every run.
        /// </summary>
        private static void BuildMotionButton(Scene scene, JetRoverOperatorBridge bridge, XROrigin origin)
        {
            foreach (GameObject root in scene.GetRootGameObjects().Where(r => r.name == MotionButtonName).ToList())
            {
                Object.DestroyImmediate(root);
            }

            float top = LabLayout.BenchTop;
            var button = new GameObject(MotionButtonName).transform;
            button.position = new Vector3(LabLayout.ArmBase.x - 0.5f, top, LabLayout.BenchNearZ + 0.12f);

            GameObject plinth = GameObject.CreatePrimitive(PrimitiveType.Cube);
            plinth.name = "Plinth";
            plinth.transform.SetParent(button, false);
            plinth.transform.localPosition = new Vector3(0f, 0.015f, 0f);
            plinth.transform.localScale = new Vector3(0.11f, 0.03f, 0.11f);
            plinth.GetComponent<Renderer>().sharedMaterial = LabMaterials.Lit("MotionButtonPlinth", new Color(0.06f, 0.062f, 0.068f), 0.6f, 0.5f);

            GameObject cap = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cap.name = "Cap";
            cap.transform.SetParent(button, false);
            cap.transform.localPosition = new Vector3(0f, 0.038f, 0f);
            cap.transform.localScale = new Vector3(0.06f, 0.008f, 0.06f); // a cylinder primitive is 2 units tall
            cap.GetComponent<Renderer>().sharedMaterial = LabMaterials.Unlit("MotionButtonCap", new Color(0.9f, 0.12f, 0.1f)); // red = off; recoloured at runtime

            foreach (Collider collider in button.GetComponentsInChildren<Collider>())
            {
                Object.DestroyImmediate(collider); // no XRI interactor may press it; the fingertip check does
            }

            TextMeshPro label = LabParts.Label("Label", button, new Vector3(0f, 0.11f, 0.075f), Quaternion.Euler(15f, 0f, 0f),
                "ROBOT OFF", 0.45f, Color.white, new Vector2(0.3f, 0.1f), TextAlignmentOptions.Center, FontStyles.Bold);

            var toggle = button.gameObject.AddComponent<MotionToggleButton>();
            var so = new SerializedObject(toggle);
            so.FindProperty("bridge").objectReferenceValue = bridge;
            so.FindProperty("origin").objectReferenceValue = origin;
            so.FindProperty("cap").objectReferenceValue = cap.GetComponent<Renderer>();
            so.FindProperty("label").objectReferenceValue = label;
            so.ApplyModifiedPropertiesWithoutUndo();
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
