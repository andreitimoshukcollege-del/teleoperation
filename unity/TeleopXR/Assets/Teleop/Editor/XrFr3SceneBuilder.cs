using System;
using System.IO;
using System.Linq;
using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR.Interaction.Toolkit;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// <b>Teleop → XR → Build FR3 Duo Scene</b> (or <c>just unity-scene-fr3-xr</c>): writes
    /// <c>Assets/Scenes/Fr3DuoXR.unity</c>, the two-hand FR3 Duo operator scene (docs/adr/0029), through the Editor
    /// API. The JetRover's scene is untouched: the two robots are two scenes, and each build names its own.
    ///
    /// <para><b>The same lab and the same controls as the JetRover's</b> (the user's request, docs/adr/0029 amendment):</para>
    /// <list type="bullet">
    /// <item>The night-shift room, the camera display with its status strip, the workcell's status ring and the baked
    /// lighting: <see cref="LabRoom"/>, <see cref="LabDisplay"/>, <see cref="LabLighting"/>, with the FR3's own mesh
    /// library (sharing the JetRover's would delete its robot's meshes) and light probes over the FR3's workspace.</item>
    /// <item>The bench is deeper (<see cref="BenchDepth"/>) and the FR3 Duo stands at its back, so its arms work beyond
    /// the controls instead of sweeping through them.</item>
    /// <item>The controls where the JetRover has them, built by the same code (<see cref="XrControlSetup"/>): power and
    /// pause on the front-left, the twin's three switches beside them with the link's status line, the operator console
    /// (predictors, the network sliders and the panel) behind them, and the added-delay slider. RECENTRE stands on the
    /// front-right, where the JetRover's training course starts.</item>
    /// <item>The twin's three arms, as the JetRover's: <b>predicted</b>, solid, with Franka's visual meshes;
    /// <b>commanded</b>, a blue ghost; <b>measured</b>, faint amber. The ghosts are the low-poly collision shapes and
    /// draw no mount: three visual twins would be 1.5M triangles. The headset guard turns the robot off when the headset
    /// comes off.</item>
    /// </list>
    /// <para>Not here, because they do not apply: the arm-speed slider (the FR3's shaper sets its own speed) and the
    /// training course (built for the JetRover's arm).</para>
    /// <para>Create-only, like the lab: once built it is edited by hand, so this refuses to overwrite it unless run
    /// headless with <c>-teleopRebuildScene</c>; <c>-labBake</c> bakes the lighting after building.
    /// <c>XrFr3SceneCheck</c> holds what must not change.</para>
    /// </summary>
    public static class XrFr3SceneBuilder
    {
        public const string ScenePath = "Assets/Scenes/Fr3DuoXR.unity";
        public const string MeshLibraryPath = LabPaths.Generated + "/Fr3LabMeshes.asset";
        private const string RebuildFlag = "-teleopRebuildScene";
        private const string BakeFlag = "-labBake";
        private const string ScreenMaterialPath = "Assets/Teleop/Materials/CameraPanel.mat";

        /// <summary>
        /// The bench runs 1.3 m from its near edge (the JetRover's runs 0.8 m), and the robot's base stands 1.1 m in: far
        /// enough back that, from the operator's eye, its arms rise above the console's panel instead of showing through it.
        /// </summary>
        public const float BenchDepth = 1.3f;
        private const float RobotDepth = 1.1f;

        private static Vector3 RobotBase => new Vector3(LabLayout.ArmBase.x, LabLayout.BenchTop, LabLayout.BenchNearZ + RobotDepth);

        /// <summary>RECENTRE, on the bench's front-right, where the JetRover's training course has its START cap.</summary>
        private static Vector3 RecentreButtonAt => new Vector3(LabLayout.ArmBase.x + 0.42f, LabLayout.BenchTop, LabLayout.BenchNearZ + 0.12f);

        [MenuItem("Teleop/XR/Build FR3 Duo Scene")]
        public static void BuildFromMenu()
        {
            if (File.Exists(ScenePath))
            {
                EditorUtility.DisplayDialog("Scene already exists",
                    $"{ScenePath} exists and is edited by hand now; the builder never overwrites it. Rename or delete it to build a fresh one.", "OK");
                return;
            }

            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                return;
            }

            Build();
        }

        [MenuItem("Teleop/XR/Bake FR3 Duo Lighting")]
        public static void BakeFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            if (LabLighting.Bake()) EditorSceneManager.SaveScene(scene);
        }

        /// <summary>Batch entry point: <c>-executeMethod Teleop.XR.Editor.XrFr3SceneBuilder.Run</c>, with optional <c>-teleopRebuildScene</c> and <c>-labBake</c>.</summary>
        public static void Run()
        {
            try
            {
                string[] args = Environment.GetCommandLineArgs();
                if (File.Exists(ScenePath))
                {
                    if (!args.Contains(RebuildFlag))
                    {
                        Debug.Log($"[fr3-scene] {ScenePath} exists; left as is (pass {RebuildFlag} to replace it).");
                        EditorApplication.Exit(0);
                        return;
                    }

                    AssetDatabase.DeleteAsset(ScenePath);
                }

                Build();
                if (args.Contains(BakeFlag))
                {
                    Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
                    if (!LabLighting.Bake()) throw new InvalidOperationException("the lighting bake failed");
                    EditorSceneManager.SaveScene(scene);
                }

                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[fr3-scene] FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        private static void Build()
        {
            GameObject rigPrefab = XrLabSceneBuilder.FindHandsRigPrefab();
            XrProjectSetup.EnsureFolder("Assets/Scenes");
            Material screenMaterial = AssetDatabase.LoadAssetAtPath<Material>(ScreenMaterialPath)
                ?? throw new InvalidOperationException($"{ScreenMaterialPath} is missing; build the JetRover's lab first (`just unity-scene-xr`)");
            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            XrLabSceneBuilder.BuildLighting();
            GameObject handsRig = XrLabSceneBuilder.BuildHandsRig(rigPrefab);
            XROrigin origin = handsRig.GetComponent<XROrigin>() ?? handsRig.GetComponentInChildren<XROrigin>(true);
            new GameObject("XR Interaction Manager").AddComponent<XRInteractionManager>();

            // The robot: the solid twin with Franka's visual meshes, and two low-poly ghosts on the same base.
            Material links = LabMaterials.Lit("Fr3Link", new Color(0.9f, 0.91f, 0.92f), 0.5f);
            Material body = LabMaterials.Lit("Fr3Body", new Color(0.22f, 0.23f, 0.25f), 0.35f);
            var robot = new GameObject("FR3Duo").transform;
            robot.position = RobotBase;
            SerialChainArmRig twin = Fr3DuoModel.Build(robot, "Twin", links, body);
            SerialChainArmRig commanded = Fr3DuoModel.Build(robot, "CommandedTwin", links, body, visualMeshes: false, fixedBody: false);
            SerialChainArmRig measured = Fr3DuoModel.Build(robot, "MeasuredTwin", links, body, visualMeshes: false, fixedBody: false);
            Color ghost = XrControlSetup.TwinColors.Commanded;
            ghost.a = 0.25f;
            XrControlSetup.SetAppearance(commanded.gameObject, LabMaterials.Transparent("TwinGhost", ghost), null);
            Color faint = XrControlSetup.TwinColors.Measured;
            faint.a = 0.3f;
            XrControlSetup.SetAppearance(measured.gameObject, LabMaterials.Transparent("TwinMeasured", faint), null);
            // The bridge shows them once there is something to show: previews and the editor see the solid twin alone.
            commanded.gameObject.SetActive(false);
            measured.gameObject.SetActive(false);

            Material marker = LabMaterials.Lit("Fr3Target", new Color(0.55f, 0.6f, 0.7f), 0.6f);
            Transform left = Marker("LeftTarget", marker);
            Transform right = Marker("RightTarget", marker);

            var operatorObject = new GameObject("Fr3Operator");
            var bridge = operatorObject.AddComponent<SerialChainOperatorBridge>();
            SetReference(bridge, "robotRoot", twin.transform);
            SetReference(bridge, "twin", twin);
            SetReference(bridge, "commandedTwin", commanded);
            SetReference(bridge, "measuredTwin", measured);
            SetReferences(bridge, "targetMarkers", left, right);
            var driver = operatorObject.AddComponent<DuoHandDriver>();
            SetReference(driver, "bridge", bridge);
            SetReference(driver, "origin", origin);
            operatorObject.AddComponent<HandTrackingSession>();
            var guard = operatorObject.AddComponent<HeadsetRemovalGuard>();
            SetReference(guard, "bridge", bridge);

            // The lab, as the JetRover's, around a deeper bench.
            LabMeshLibrary.Begin(MeshLibraryPath);
            LabPalette palette = LabPalette.Create();
            Transform dressing = new GameObject(LabDressing.RootName).transform;
            LabRoom.Result room = LabRoom.Build(dressing, palette, BenchDepth, "FR3 Duo");
            LabDisplay.Build(dressing, palette, screenMaterial, null, bridge, "robot camera  ·  none yet (the FR3 Duo is simulated)");
            var lamp = room.StatusRing.gameObject.AddComponent<LinkStatusLamp>();
            SetReference(lamp, "robot", bridge);
            SetReferences(lamp, "lamps", room.StatusRing);
            var blink = dressing.Find("Room/Racks").gameObject.AddComponent<BlinkingLights>();
            SetReferences(blink, "lamps", room.RackLeds.Cast<Object>().ToArray());
            Light key = Object.FindObjectsByType<Light>(FindObjectsInactive.Include).FirstOrDefault(l => l.type == LightType.Directional);
            LabLighting.Build(dressing, scene, key, new LabLighting.ProbeRegion
            {
                // Where the FR3's arms move: from the bench up past its head, either side, and out over the floor ahead.
                Centre = new Vector3(LabLayout.ArmBase.x, LabLayout.BenchTop + 0.5f, LabLayout.BenchNearZ + RobotDepth + 0.25f),
                HalfSize = new Vector3(1.0f, 0.5f, 0.75f),
                Spacing = 0.25f,
            }, BenchDepth);
            LabMeshLibrary.End();

            // The controls, where the JetRover has them, by the same code.
            XrControlSetup.BuildMotionButton(scene, bridge, origin);
            TextMeshPro linkLine = XrControlSetup.BuildTwinSwitches(scene, bridge, origin);
            // Edit-time labels from the shipped config, so previews show the console; the runtime replaces them.
            TextAsset configAsset = Resources.Load<TextAsset>("fr3_connection");
            SerialChainConnectionConfig config = configAsset != null ? JsonUtility.FromJson<SerialChainConnectionConfig>(configAsset.text) : new SerialChainConnectionConfig();
            XrControlSetup.BuildConsole(scene, bridge, origin, (config.JointPredictorPresets ?? new JointPredictorPresetData[0]).Select(p => p.Label).ToArray());
            XrControlSetup.BuildDelaySlider(scene, bridge, origin);
            BuildRecentreButton(bridge, origin);

            // The status lines: the arms on the display's caption line, the link above the switches.
            TMP_Text caption = Object.FindObjectsByType<TMP_Text>(FindObjectsInactive.Include, FindObjectsSortMode.None).FirstOrDefault(t => t.name == "DisplayCaption");
            SetReference(driver, "statusLabel", caption);
            SetReference(driver, "linkLabel", linkLine);

            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
            Debug.Log($"[fr3-scene] built {ScenePath}: {twin.Joints.Length} moving joints on the twin; lab {LabMeshLibrary.Triangles} generated triangles");
        }

        private static void BuildRecentreButton(SerialChainOperatorBridge bridge, XROrigin origin)
        {
            var root = new GameObject("RecentreButton").transform;
            root.position = RecentreButtonAt;
            GameObject plinth = GameObject.CreatePrimitive(PrimitiveType.Cube);
            plinth.name = "Plinth";
            Object.DestroyImmediate(plinth.GetComponent<Collider>());
            plinth.transform.SetParent(root, false);
            plinth.transform.localPosition = new Vector3(0f, 0.015f, 0f);
            plinth.transform.localScale = new Vector3(0.09f, 0.03f, 0.09f);
            plinth.GetComponent<Renderer>().sharedMaterial = LabMaterials.Lit("MotionButtonPlinth", new Color(0.06f, 0.062f, 0.068f), 0.6f, 0.5f);
            GameObject cap = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cap.name = "Cap";
            Object.DestroyImmediate(cap.GetComponent<Collider>());
            cap.transform.SetParent(root, false);
            cap.transform.localScale = new Vector3(0.055f, 0.008f, 0.055f); // a cylinder primitive is 2 units tall
            cap.transform.localPosition = new Vector3(0f, 0.038f, 0f);
            cap.GetComponent<Renderer>().sharedMaterial = LabMaterials.Unlit("Fr3RecentreCap", new Color(0.25f, 0.55f, 1f));
            TextMeshPro label = LabParts.Label("Label", root, new Vector3(0f, 0.11f, 0.075f), Quaternion.Euler(15f, 0f, 0f),
                "RECENTRE", 0.3f, Color.white, new Vector2(0.3f, 0.08f), TextAlignmentOptions.Center, FontStyles.Bold);

            var button = root.gameObject.AddComponent<Fr3MotionButton>();
            SetReference(button, "bridge", bridge);
            SetReference(button, "origin", origin);
            SetReference(button, "cap", cap.GetComponent<Renderer>());
            SetReference(button, "label", label);
            var so = new SerializedObject(button);
            so.FindProperty("role").enumValueIndex = (int)Fr3MotionButton.Role.Recentre;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        private static Transform Marker(string name, Material material)
        {
            // A 2.5 cm ball with a stalk along the tool's approach axis (Core's +Z, Unity's +Y in the marker's frame).
            GameObject ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            ball.name = name;
            Object.DestroyImmediate(ball.GetComponent<Collider>());
            ball.transform.localScale = Vector3.one * 0.025f;
            ball.GetComponent<Renderer>().sharedMaterial = material;
            GameObject stalk = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            stalk.name = "Approach";
            Object.DestroyImmediate(stalk.GetComponent<Collider>());
            stalk.transform.SetParent(ball.transform, false);
            stalk.transform.localScale = new Vector3(0.25f, 1.2f, 0.25f);
            stalk.transform.localPosition = new Vector3(0f, 1.2f, 0f);
            stalk.GetComponent<Renderer>().sharedMaterial = material;
            ball.SetActive(false); // shown once the robot's pose is in
            return ball.transform;
        }

        private static void SetReference(Object component, string field, Object value)
        {
            var so = new SerializedObject(component);
            SerializedProperty property = so.FindProperty(field)
                ?? throw new InvalidOperationException($"{component.GetType().Name} has no serialized field '{field}'.");
            property.objectReferenceValue = value;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void SetReferences(Object component, string field, params Object[] values)
        {
            var so = new SerializedObject(component);
            SerializedProperty property = so.FindProperty(field)
                ?? throw new InvalidOperationException($"{component.GetType().Name} has no serialized field '{field}'.");
            property.arraySize = values.Length;
            for (int i = 0; i < values.Length; i++) property.GetArrayElementAtIndex(i).objectReferenceValue = values[i];
            so.ApplyModifiedPropertiesWithoutUndo();
        }
    }
}
