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
    /// <para><b>What it contains:</b></para>
    /// <list type="bullet">
    /// <item>The same hands-only XRI rig as the JetRover's lab, its hand smoothing off (ADR 0016 §5).</item>
    /// <item>A floor and a table, and on it the FR3 Duo's twin, built from the URDF by <see cref="Fr3DuoModel"/>,
    /// facing away from the operator so each hand is on the side of its arm.</item>
    /// <item>Two target markers (where each clutch asks its tool to be), coloured by the clutch's state.</item>
    /// <item><c>Fr3Operator</c>: <see cref="SerialChainOperatorBridge"/>, <see cref="DuoHandDriver"/> and
    /// <see cref="HandTrackingSession"/>; a status line over the robot; and on the table's front-left corner the motion
    /// button and, beside it, RECENTRE (<see cref="Fr3MotionButton"/>).</item>
    /// </list>
    /// <para>Create-only, like the lab: once built it is edited by hand, so this refuses to overwrite it unless run
    /// headless with <c>-teleopRebuildScene</c>. <c>XrFr3SceneCheck</c> holds what must not change.</para>
    /// </summary>
    public static class XrFr3SceneBuilder
    {
        public const string ScenePath = "Assets/Scenes/Fr3DuoXR.unity";
        private const string RebuildFlag = "-teleopRebuildScene";

        /// <summary>The table's top, and the robot's base on it, 75 cm in front of the operator: at the ready pose its elbows sweep out to the sides and back toward you, and its hands are about a metre ahead.</summary>
        private const float TableHeight = 0.75f;
        private static readonly Vector3 RobotBase = new Vector3(0f, TableHeight, 0.75f);
        private static readonly Vector3 TableSize = new Vector3(1.9f, TableHeight, 1.2f);
        private static readonly Vector3 TableCentre = new Vector3(0f, TableHeight / 2f, 1.05f);
        private static readonly Vector3 MotionButtonAt = new Vector3(-0.55f, TableHeight, 0.55f);
        private static readonly Vector3 RecentreButtonAt = new Vector3(-0.38f, TableHeight, 0.55f);

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

        /// <summary>Batch entry point: <c>-executeMethod Teleop.XR.Editor.XrFr3SceneBuilder.Run</c>.</summary>
        public static void Run()
        {
            try
            {
                if (File.Exists(ScenePath))
                {
                    if (!Environment.GetCommandLineArgs().Contains(RebuildFlag))
                    {
                        Debug.Log($"[fr3-scene] {ScenePath} exists; left as is (pass {RebuildFlag} to replace it).");
                        EditorApplication.Exit(0);
                        return;
                    }

                    AssetDatabase.DeleteAsset(ScenePath);
                }

                Build();
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
            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            Material links = LabMaterials.Lit("Fr3Link", new Color(0.9f, 0.91f, 0.92f), 0.5f);
            Material body = LabMaterials.Lit("Fr3Body", new Color(0.22f, 0.23f, 0.25f), 0.35f);
            Material floor = LabMaterials.Matte("Fr3Floor", new Color(0.3f, 0.31f, 0.33f));
            Material table = LabMaterials.Matte("Fr3Table", new Color(0.52f, 0.45f, 0.36f), 0.25f);
            Material marker = LabMaterials.Lit("Fr3Target", new Color(0.55f, 0.6f, 0.7f), 0.6f);
            Material button = LabMaterials.Lit("Fr3MotionCap", new Color(0.9f, 0.12f, 0.1f), 0.5f);

            XrLabSceneBuilder.BuildLighting();
            GameObject handsRig = XrLabSceneBuilder.BuildHandsRig(rigPrefab);
            XROrigin origin = handsRig.GetComponent<XROrigin>() ?? handsRig.GetComponentInChildren<XROrigin>(true);
            new GameObject("XR Interaction Manager").AddComponent<XRInteractionManager>();

            Box("Floor", null, new Vector3(0f, -0.01f, 1f), new Vector3(6f, 0.02f, 6f), floor);
            Box("Table", null, TableCentre, TableSize, table);

            var robot = new GameObject("FR3Duo").transform;
            robot.position = RobotBase;
            SerialChainArmRig twin = Fr3DuoModel.Build(robot, "Twin", links, body);
            Transform robotRoot = twin.transform;

            Transform left = Marker("LeftTarget", marker);
            Transform right = Marker("RightTarget", marker);

            var operatorObject = new GameObject("Fr3Operator");
            var bridge = operatorObject.AddComponent<SerialChainOperatorBridge>();
            SetReference(bridge, "robotRoot", robotRoot);
            SetReference(bridge, "twin", twin);
            SetReferences(bridge, "targetMarkers", left, right);
            var driver = operatorObject.AddComponent<DuoHandDriver>();
            SetReference(driver, "bridge", bridge);
            SetReference(driver, "origin", origin);
            SetReference(driver, "statusLabel", Label("Status", null, new Vector3(0f, 1.8f, 1.6f), 0.5f, new Vector2(2.4f, 0.3f)));
            operatorObject.AddComponent<HandTrackingSession>();

            BuildMotionButton(bridge, origin, button, "MotionButton", MotionButtonAt, Fr3MotionButton.Role.Power);
            BuildMotionButton(bridge, origin, LabMaterials.Lit("Fr3RecentreCap", new Color(0.25f, 0.55f, 1f), 0.5f), "RecentreButton", RecentreButtonAt, Fr3MotionButton.Role.Recentre);

            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
            Debug.Log($"[fr3-scene] built {ScenePath}: {twin.Joints.Length} moving joints on the twin");
        }

        private static void BuildMotionButton(SerialChainOperatorBridge bridge, XROrigin origin, Material capMaterial, string name, Vector3 at, Fr3MotionButton.Role role)
        {
            var root = new GameObject(name).transform;
            root.position = at;
            GameObject cap = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cap.name = "Cap";
            Object.DestroyImmediate(cap.GetComponent<Collider>());
            cap.transform.SetParent(root, false);
            // A cylinder primitive is 2 units tall: 1.5 cm tall, 7 cm across, sitting on the table.
            cap.transform.localScale = new Vector3(0.07f, 0.0075f, 0.07f);
            cap.transform.localPosition = new Vector3(0f, 0.0075f, 0f);
            cap.GetComponent<Renderer>().sharedMaterial = capMaterial;
            TMP_Text label = Label("Label", root, new Vector3(0f, 0.09f, 0.02f), 0.22f, new Vector2(0.4f, 0.12f));

            var switchComponent = root.gameObject.AddComponent<Fr3MotionButton>();
            SetReference(switchComponent, "bridge", bridge);
            SetReference(switchComponent, "origin", origin);
            SetReference(switchComponent, "cap", cap.GetComponent<Renderer>());
            SetReference(switchComponent, "label", label);
            var so = new SerializedObject(switchComponent);
            so.FindProperty("role").enumValueIndex = (int)role;
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

        private static void Box(string name, Transform parent, Vector3 centre, Vector3 size, Material material)
        {
            GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            Object.DestroyImmediate(box.GetComponent<Collider>());
            if (parent != null) box.transform.SetParent(parent, false);
            box.transform.localPosition = centre;
            box.transform.localScale = size;
            box.GetComponent<Renderer>().sharedMaterial = material;
        }

        private static TMP_Text Label(string name, Transform parent, Vector3 position, float fontSize, Vector2 size)
        {
            var go = new GameObject(name);
            if (parent != null) go.transform.SetParent(parent, false);
            go.transform.localPosition = position;
            var text = go.AddComponent<TextMeshPro>();
            text.fontSize = fontSize;
            text.alignment = TextAlignmentOptions.Center;
            text.rectTransform.sizeDelta = size;
            text.text = "";
            return text;
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
