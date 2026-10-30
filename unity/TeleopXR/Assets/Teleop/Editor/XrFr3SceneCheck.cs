using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Teleop.Bridge;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR.Interaction.Toolkit.Inputs;
using CoreQuat = System.Numerics.Quaternion;
using CoreVec = System.Numerics.Vector3;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// <b>Teleop → XR → Check FR3 Duo Scene</b> (or <c>just unity-check-fr3-xr</c>): fails on anything in
    /// <c>Fr3DuoXR.unity</c> that would let a hand move an arm it should not, or draw a twin that is not the robot
    /// (docs/adr/0029). <c>just build-galaxy-fr3</c> runs it first and refuses to build on a failure.
    /// <list type="bullet">
    /// <item>No missing scripts; hand smoothing inactive; no controllers in the modality manager.</item>
    /// <item><see cref="SerialChainOperatorBridge"/> wired (root, twin, two markers), motion <b>off at launch</b>.</item>
    /// <item><see cref="DuoHandDriver"/> and the <see cref="Fr3MotionButton"/> present, enabled, wired.</item>
    /// <item>The resources parse: the profile, the collision model (for that profile), the config (a grab that cannot
    /// chatter, a gripper that is not inverted, a position scale in (0, 2], a step limit in (0, 0.2]); every learned
    /// preset's model loads and runs.</item>
    /// <item><b>The twin is the profile:</b> at 25 seeded poses inside the soft limits, each arm's
    /// <c>*_hand_tcp</c> in the twin is within 1 mm and 0.1° of <see cref="SerialChainKinematics.Tool"/>.</item>
    /// </list>
    /// </summary>
    public static class XrFr3SceneCheck
    {
        [MenuItem("Teleop/XR/Check FR3 Duo Scene")]
        public static void RunFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Report(Check());
        }

        /// <summary>Batch entry point: <c>-executeMethod Teleop.XR.Editor.XrFr3SceneCheck.Run</c>.</summary>
        public static void Run()
        {
            List<string> problems;
            try
            {
                problems = Check();
            }
            catch (Exception e)
            {
                Debug.LogError($"[fr3-check] could not run: {e}");
                EditorApplication.Exit(2);
                return;
            }

            Report(problems);
            EditorApplication.Exit(problems.Count == 0 ? 0 : 1);
        }

        public static List<string> Check()
        {
            var problems = new List<string>();
            CheckResources(problems);
            string path = XrFr3SceneBuilder.ScenePath;
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(path) == null)
            {
                problems.Add($"{path}: not found (run `just unity-scene-fr3-xr`)");
                return problems;
            }

            Scene scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
                {
                    int missing = GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject);
                    if (missing > 0) problems.Add($"{missing} missing script(s) on '{t.name}'");
                    foreach (MonoBehaviour behaviour in t.GetComponents<MonoBehaviour>())
                    {
                        if (behaviour != null && behaviour.GetType().Name == XrSceneCheck.HandSmoothingTypeName && behaviour.isActiveAndEnabled)
                            problems.Add($"hand smoothing is active on '{t.name}': it filters the hand that moves the robot (ADR 0016 §5)");
                    }
                }
            }

            XRInputModalityManager modality = Object.FindAnyObjectByType<XRInputModalityManager>(FindObjectsInactive.Include);
            if (modality != null && (modality.leftController != null || modality.rightController != null))
                problems.Add("the modality manager has controllers: the scene is hands only");

            var bridge = Object.FindAnyObjectByType<SerialChainOperatorBridge>(FindObjectsInactive.Include);
            if (bridge == null)
            {
                problems.Add("no SerialChainOperatorBridge");
                return problems;
            }

            var so = new SerializedObject(bridge);
            RequireReference(so, "robotRoot", problems);
            RequireReference(so, "twin", problems);
            SerializedProperty markers = so.FindProperty("targetMarkers");
            if (markers == null || markers.arraySize != 2 || Enumerable.Range(0, 2).Any(i => markers.GetArrayElementAtIndex(i).objectReferenceValue == null))
                problems.Add("SerialChainOperatorBridge.targetMarkers must hold two markers (left, right)");
            if (so.FindProperty("motionEnabledAtStart")?.boolValue != false)
                problems.Add("SerialChainOperatorBridge.motionEnabledAtStart is on: the robot must not move at launch, before anyone presses the button");
            if (!bridge.enabled) problems.Add("SerialChainOperatorBridge is disabled");
            if (bridge.GetComponent<HandTrackingSession>() == null) problems.Add("no HandTrackingSession next to SerialChainOperatorBridge");

            DuoHandDriver[] drivers = Object.FindObjectsByType<DuoHandDriver>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            if (drivers.Length != 1) problems.Add($"{drivers.Length} DuoHandDriver(s): exactly one drives the arms");
            foreach (DuoHandDriver d in drivers)
            {
                if (!d.isActiveAndEnabled && d.gameObject.activeInHierarchy) problems.Add("DuoHandDriver is disabled");
                var dso = new SerializedObject(d);
                RequireReference(dso, "bridge", problems);
                RequireReference(dso, "origin", problems);
            }

            Fr3MotionButton[] buttons = Object.FindObjectsByType<Fr3MotionButton>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            int power = buttons.Count(b => b.ButtonRole == Fr3MotionButton.Role.Power);
            if (power != 1) problems.Add($"{power} power button(s): exactly one starts and stops the robot");
            foreach (Fr3MotionButton b in buttons)
            {
                if (!b.enabled) problems.Add("Fr3MotionButton is disabled");
                var bso = new SerializedObject(b);
                RequireReference(bso, "bridge", problems);
                RequireReference(bso, "origin", problems);
                RequireReference(bso, "cap", problems);
            }

            var twin = so.FindProperty("twin")?.objectReferenceValue as SerialChainArmRig;
            var robotRoot = so.FindProperty("robotRoot")?.objectReferenceValue as Transform;
            if (twin != null && robotRoot != null) CheckTwin(twin, robotRoot, problems);
            return problems;
        }

        private static void CheckResources(List<string> problems)
        {
            SerialChainProfile profile = null;
            try
            {
                profile = Fr3DuoModel.LoadProfile();
                if (profile.Validate() is string bad) problems.Add($"fr3_duo_profile: {bad}");
            }
            catch (Exception e)
            {
                problems.Add($"fr3_duo_profile does not load: {e.Message}");
            }

            var collisionAsset = Resources.Load<TextAsset>("fr3_duo_collision");
            if (collisionAsset == null)
            {
                problems.Add("Resources/fr3_duo_collision.json is missing");
            }
            else
            {
                try
                {
                    SerialChainCollisionModel model = JsonUtility.FromJson<SerialChainCollisionDocument>(collisionAsset.text).ToModel();
                    if (profile != null && model.Profile != profile.Name) problems.Add($"the collision model is for '{model.Profile}', not '{profile.Name}'");
                }
                catch (Exception e)
                {
                    problems.Add($"fr3_duo_collision does not load: {e.Message}");
                }
            }

            var configAsset = Resources.Load<TextAsset>("fr3_connection");
            SerialChainConnectionConfig config = configAsset != null ? JsonUtility.FromJson<SerialChainConnectionConfig>(configAsset.text) : null;
            if (config == null)
            {
                problems.Add("Resources/fr3_connection.json is missing or does not parse");
                return;
            }

            if (!(config.GrabEngageBelowCurl > 0f) || !(config.GrabReleaseAboveCurl > config.GrabEngageBelowCurl))
                problems.Add($"fr3_connection: the grab must engage below its release curl ({config.GrabEngageBelowCurl} / {config.GrabReleaseAboveCurl}), or it chatters");
            if (!(config.PinchOpenMeters > config.PinchClosedMeters) || config.PinchClosedMeters < 0f)
                problems.Add($"fr3_connection: the gripper's open pinch must be wider than its closed one ({config.PinchOpenMeters} / {config.PinchClosedMeters})");
            if (!(config.PositionScale > 0) || config.PositionScale > 2)
                problems.Add($"fr3_connection: PositionScale {config.PositionScale} is outside (0, 2]");
            if (!(config.MaxStepRadians > 0) || config.MaxStepRadians > 0.2)
                problems.Add($"fr3_connection: MaxStepRadians {config.MaxStepRadians} is outside (0, 0.2]");
            foreach (JointPredictorPresetData p in config.JointPredictorPresets ?? new JointPredictorPresetData[0])
            {
                if (p == null || p.Predictor != "learned" || !p.Enabled) continue;
                if (!LearnedModels.Check(p.Model, out string modelId, out string error)) problems.Add($"fr3 preset '{p.Label}': {error}");
                else Debug.Log($"[fr3-check] preset '{p.Label}' runs {modelId}");
            }
        }

        /// <summary>The twin's tool frames against Core's forward kinematics, at seeded poses.</summary>
        private static void CheckTwin(SerialChainArmRig twin, Transform robotRoot, List<string> problems)
        {
            SerialChainProfile profile = Fr3DuoModel.LoadProfile();
            int moving = profile.Arms.Sum(a => a.Joints.Length + (a.HasGripper && a.Gripper.SecondFingerMimics ? 2 : a.HasGripper ? 1 : 0));
            if (twin.Joints.Length != moving) problems.Add($"the twin has {twin.Joints.Length} moving joints; the profile {moving}");

            var random = new System.Random(29);
            double worstMm = 0, worstDeg = 0;
            for (int trial = 0; trial < 25; trial++)
            {
                foreach (ChainArm arm in profile.Arms)
                {
                    var q = new double[arm.Joints.Length];
                    for (int j = 0; j < q.Length; j++)
                    {
                        ChainJointLimits l = arm.Joints[j].Limits;
                        q[j] = l.SoftLower + random.NextDouble() * (l.SoftUpper - l.SoftLower);
                        int index = Array.FindIndex(twin.Joints, rj => rj.motorId == arm.Joints[j].MotorId);
                        if (index < 0)
                        {
                            problems.Add($"the twin has no joint for motor {arm.Joints[j].MotorId}");
                            return;
                        }

                        twin.SetJoint(index, (float)q[j]);
                    }

                    Transform tcp = twin.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == $"{arm.Name}_fr3v2_hand_tcp");
                    if (tcp == null)
                    {
                        problems.Add($"the twin has no {arm.Name}_fr3v2_hand_tcp");
                        return;
                    }

                    RigidTransform tool = SerialChainKinematics.Tool(arm, q);
                    Vector3 expected = new CoreVec((float)tool.X, (float)tool.Y, (float)tool.Z).ToUnity();
                    Quaternion expectedRotation = new CoreQuat((float)tool.Qx, (float)tool.Qy, (float)tool.Qz, (float)tool.Qw).ToUnity();
                    Vector3 actual = robotRoot.InverseTransformPoint(tcp.position);
                    Quaternion actualRotation = Quaternion.Inverse(robotRoot.rotation) * tcp.rotation;
                    worstMm = Math.Max(worstMm, 1000.0 * Vector3.Distance(expected, actual));
                    worstDeg = Math.Max(worstDeg, Quaternion.Angle(expectedRotation, actualRotation));
                }
            }

            // Back to zero, so the saved scene is not left at the last sample.
            for (int i = 0; i < twin.Joints.Length; i++) twin.SetJoint(i, 0f);
            if (worstMm > 1.0 || worstDeg > 0.1)
                problems.Add($"the twin is not the profile: its tool frames are up to {worstMm:0.###} mm and {worstDeg:0.###}° from Core's FK");
            else
                Debug.Log($"[fr3-check] twin matches Core's FK: worst {worstMm:0.####} mm, {worstDeg:0.####}° over 25 poses per arm");
        }

        private static void RequireReference(SerializedObject so, string field, List<string> problems)
        {
            SerializedProperty property = so.FindProperty(field);
            if (property == null) problems.Add($"{so.targetObject.GetType().Name} has no serialized field '{field}'");
            else if (property.objectReferenceValue == null) problems.Add($"{so.targetObject.GetType().Name}.{field} is not assigned");
        }

        private static void Report(List<string> problems)
        {
            if (problems.Count == 0)
            {
                Debug.Log("[fr3-check] PASS: Fr3DuoXR has no missing scripts, intact wiring, motion off at launch, and a twin that is the profile");
                return;
            }

            var message = new StringBuilder($"[fr3-check] FAIL: {problems.Count} problem(s)");
            foreach (string problem in problems) message.Append("\n  - ").Append(problem);
            Debug.LogError(message.ToString());
        }
    }
}
