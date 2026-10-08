using System;
using System.Collections.Generic;
using System.Text;
using Teleop.Bridge;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR.Interaction.Toolkit.Inputs;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// A headless gate for the Galaxy XR scene. <b>Teleop → XR → Check Scene</b>, or
    /// <c>just unity-check-xr</c>; <c>just build-galaxy</c> runs it before building.
    ///
    /// <para>It fails on:</para>
    /// <list type="bullet">
    /// <item>any missing script;</item>
    /// <item>broken Bridge wiring;</item>
    /// <item>clutch control not set up safely (docs/adr/0017): the target still grabbable, the
    /// clutch driver missing or unwired, a scale above 1, or the hand smoothing filter on (it filters
    /// the hand joints the clutch reads);</item>
    /// <item>controllers left in the modality manager;</item>
    /// <item>Active Input Handling without the Input System, which breaks XRI's hand interactions;</item>
    /// <item>Android XR Support or Hand Tracking off for Android.</item>
    /// </list>
    /// Those settings are easy to change by accident in an Inspector and invisible until the robot
    /// jumps. Read-only; exits non-zero when it finds a problem or cannot look (invariant 10).
    /// </summary>
    public static class XrSceneCheck
    {
        /// <summary>The rig's One Euro filter over every hand joint (XRI/XR Hands samples).</summary>
        public const string HandSmoothingTypeName = "HandsOneEuroFilterPostProcessor";

        [MenuItem("Teleop/XR/Check Scene")]
        public static void RunFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                return;
            }

            Report(Check());
        }

        /// <summary>Batch entry point: <c>-executeMethod Teleop.XR.Editor.XrSceneCheck.Run</c>.</summary>
        public static void Run()
        {
            List<string> problems;
            try
            {
                problems = Check();
            }
            catch (Exception e)
            {
                Debug.LogError($"[scene-check] could not run: {e}");
                EditorApplication.Exit(2);
                return;
            }

            Report(problems);
            EditorApplication.Exit(problems.Count == 0 ? 0 : 1);
        }

        public static List<string> Check()
        {
            var problems = new List<string>();
            string path = XrLabSceneBuilder.ScenePath;
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(path) == null)
            {
                problems.Add($"{path}: not found (run `just unity-scene-xr`)");
                return problems;
            }

            Scene scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
                {
                    int missing = GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject);
                    if (missing > 0)
                    {
                        problems.Add($"{missing} missing script(s) on '{t.name}'");
                    }

                    foreach (MonoBehaviour behaviour in t.GetComponents<MonoBehaviour>())
                    {
                        if (behaviour != null && behaviour.GetType().Name == HandSmoothingTypeName && behaviour.isActiveAndEnabled)
                        {
                            problems.Add($"hand smoothing ({HandSmoothingTypeName}) is active on '{t.name}': it filters the pinch that moves the robot (ADR 0016 §5)");
                        }
                    }
                }
            }

            var bridge = Object.FindAnyObjectByType<JetRoverOperatorBridge>(FindObjectsInactive.Include);
            if (bridge == null)
            {
                problems.Add("no JetRoverOperatorBridge");
            }
            else
            {
                RequireReference(new SerializedObject(bridge), "dragTarget", problems);
                RequireReference(new SerializedObject(bridge), "armRig", problems);
                if (bridge.GetComponent<HandTrackingSession>() == null)
                {
                    problems.Add("no HandTrackingSession next to JetRoverOperatorBridge (hand-tracking permission, modality log)");
                }
            }

            var rig = Object.FindAnyObjectByType<JetRoverArmRig>(FindObjectsInactive.Include);
            if (rig == null)
            {
                problems.Add("no JetRoverArmRig");
            }
            else
            {
                var so = new SerializedObject(rig);
                foreach (string field in new[] { "baseAnchor", "baseYawPivot", "lowerPitchPivot", "middlePitchPivot", "upperPitchPivot", "reachWarningRenderer" })
                {
                    RequireReference(so, field, problems);
                }

                // The reach warning only warns if it can be seen: the lab hides the old segment cubes,
                // so it must point at a renderer that is still drawn (the wrist LED).
                if (so.FindProperty("reachWarningRenderer").objectReferenceValue is Renderer warning
                    && (!warning.enabled || !warning.gameObject.activeInHierarchy))
                {
                    problems.Add($"JetRoverArmRig.reachWarningRenderer ('{warning.name}') is not drawn, so a clamped target would show nothing");
                }
            }

            var feed = Object.FindAnyObjectByType<CameraFeedBridge>(FindObjectsInactive.Include);
            if (feed == null)
            {
                problems.Add("no CameraFeedBridge");
            }
            else
            {
                var feedSo = new SerializedObject(feed);
                RequireReference(feedSo, "panel", problems);
                // With an arm rig set, CameraFeedBridge moves its panel beside the arm at start, which
                // would pull the lab display's screen out of its bezel.
                if (feedSo.FindProperty("panel").objectReferenceValue is Renderer panel && panel.GetComponentInParent<CameraStatusBar>() != null
                    && feedSo.FindProperty("armRig").objectReferenceValue != null)
                {
                    problems.Add("CameraFeedBridge.armRig is set while its panel is the lab display's screen; it would move the screen out of the display");
                }
            }

            CheckClutch(bridge, problems);

            CheckAndroidOpenXR(problems);

#if !ENABLE_INPUT_SYSTEM
            // XRI's hands rig reads its pinch and poke through Input System actions. With Active Input
            // Handling on the old Input Manager alone, hands are drawn but none of XRI's hand
            // interactions work. The project was committed like that once, because batch mode never
            // shows the dialog that offers to switch it. (The clutch reads XR Hands directly.)
            problems.Add("Player Settings > Active Input Handling excludes the Input System: XRI's hand interactions would not work (set it to Both, as in TeleopVR)");
#endif

            var modality = Object.FindAnyObjectByType<XRInputModalityManager>(FindObjectsInactive.Include);
            if (modality == null)
            {
                problems.Add("no XRInputModalityManager (the hands rig is missing)");
            }
            else if (modality.leftController != null || modality.rightController != null)
            {
                problems.Add("XRInputModalityManager still has controllers assigned; this scene is hands only (ADR 0016 §5)");
            }

            Debug.Log($"[scene-check] {path}: {problems.Count} problem(s)");
            return problems;
        }

        /// <summary>
        /// Android XR Support is what makes Unity's Android XR package mark the launcher activity
        /// immersive (PROPERTY_XR_ACTIVITY_START_MODE). Without it the headset starts the app as a
        /// 2D panel, refuses the OpenXR session ("Activity does not support immersive OpenXR
        /// sessions") and Unity aborts. Hand Tracking also brings the HAND_TRACKING permission.
        /// An editor once regenerated the OpenXR settings with both of them off.
        /// </summary>
        private static void CheckAndroidOpenXR(List<string> problems)
        {
            var settings = UnityEngine.XR.OpenXR.OpenXRSettings.GetSettingsForBuildTargetGroup(BuildTargetGroup.Android);
            if (settings == null)
            {
                problems.Add("no OpenXR settings for Android (run `just unity-setup-xr`)");
                return;
            }

            var axr = settings.GetFeature<UnityEngine.XR.OpenXR.Features.Android.AndroidXRSupportFeature>();
            if (axr == null || !axr.enabled)
                problems.Add("OpenXR (Android): Android XR Support is off, so the APK would launch as a 2D panel and crash on session creation (run `just unity-setup-xr`)");
            var hands = settings.GetFeature<UnityEngine.XR.Hands.OpenXR.HandTracking>();
            if (hands == null || !hands.enabled)
                problems.Add("OpenXR (Android): Hand Tracking is off, so no hands and no HAND_TRACKING permission (run `just unity-setup-xr`)");
        }

        /// <summary>
        /// Clutch control (docs/adr/0017) in place of the old pinch-grab. The target must no longer be
        /// grabbable: an interactable left on it would let a far pinch move it, and the robot, outside
        /// the clutch's scaling and workspace limits. The driver must be wired, and the config's
        /// scale must make the arm move no more than the hand does.
        /// </summary>
        private static void CheckClutch(JetRoverOperatorBridge bridge, List<string> problems)
        {
            if (bridge == null)
            {
                return;
            }

            var target = new SerializedObject(bridge).FindProperty("dragTarget")?.objectReferenceValue as Transform;
            if (target != null && target.GetComponent<XRBaseInteractable>() != null)
            {
                problems.Add("DragTarget still has an XR interactable: a pinch could grab it and move the robot outside the clutch's scale and workspace limits (run Teleop > XR > Set Up Clutch Control)");
            }

            var driver = Object.FindAnyObjectByType<HandClutchDriver>(FindObjectsInactive.Include);
            if (driver == null)
            {
                problems.Add("no HandClutchDriver: nothing drives the arm (run Teleop > XR > Set Up Clutch Control)");
            }
            else
            {
                var so = new SerializedObject(driver);
                foreach (string field in new[] { "bridge", "armRig", "origin" })
                {
                    RequireReference(so, field, problems);
                }

                if (so.FindProperty("bridge").objectReferenceValue != bridge)
                    problems.Add("HandClutchDriver.bridge is not the scene's JetRoverOperatorBridge");
                if (!(so.FindProperty("engageBelowMeters").floatValue < so.FindProperty("releaseAboveMeters").floatValue))
                    problems.Add("HandClutchDriver: engageBelowMeters must be below releaseAboveMeters, or the clutch chatters on and off");
                if (!driver.enabled)
                    problems.Add("HandClutchDriver is disabled");
            }

            var json = Resources.Load<TextAsset>("jetrover_connection");
            if (json == null)
            {
                problems.Add("Resources/jetrover_connection.json not found (Teleop.Bridge)");
                return;
            }

            var config = new JetRoverArmConfig();
            JsonUtility.FromJsonOverwrite(json.text, config);
            if (!(config.ClutchScale > 0f && config.ClutchScale <= 1f))
                problems.Add($"jetrover_connection.json: ClutchScale {config.ClutchScale} must be in (0, 1]; above 1 the arm moves further than the hand");
            if (!(config.GripperPinchOpenMeters > config.GripperPinchClosedMeters))
                problems.Add("jetrover_connection.json: GripperPinchOpenMeters must be greater than GripperPinchClosedMeters");
        }

        private static void RequireReference(SerializedObject so, string field, List<string> problems)
        {
            SerializedProperty property = so.FindProperty(field);
            if (property == null)
                problems.Add($"{so.targetObject.GetType().Name} has no serialized field '{field}'");
            else if (property.objectReferenceValue == null)
                problems.Add($"{so.targetObject.GetType().Name}.{field} is not assigned");
        }

        private static void Report(List<string> problems)
        {
            if (problems.Count == 0)
            {
                Debug.Log("[scene-check] PASS: JetRoverLabXR has no missing scripts, intact wiring, and clutch control set up safely");
                return;
            }

            var message = new StringBuilder($"[scene-check] FAIL: {problems.Count} problem(s)");
            foreach (string problem in problems)
            {
                message.Append("\n  - ").Append(problem);
            }

            Debug.LogError(message.ToString());
        }
    }
}
