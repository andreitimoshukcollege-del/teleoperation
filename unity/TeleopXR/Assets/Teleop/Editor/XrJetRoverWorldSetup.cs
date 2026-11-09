using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.Bridge;
using Teleop.RobotArm.World;
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
    /// <b>Teleop → XR → Set Up JetRover World</b> (or <c>just unity-world-jetrover-xr</c>): gives the JetRover's scene what the FR3
    /// Duo's has for Isaac (docs/adr/0034 J6), from <c>Resources/jetrover_scene.json</c>, a copy of <c>core/Scenes/jetrover-bench.json</c>:
    /// <list type="bullet">
    /// <item>the <b>world twin</b>: the scene file's objects, at the arm's base frame (its rig's <c>BaseAnchor</c>), each shown once
    /// Isaac reports it. The bench and the table are <c>builtin</c>: the lab already has them, so no surface is built.</item>
    /// <item><b>NEW LAYOUT</b> and the <b>RECORD</b> buttons (RECORD, DISCARD LAST, &lt; PREV, NEXT &gt;) on the bench's right wing,
    /// behind the training course.</item>
    /// <item>the <b>hand pointer</b>: with the robot off, a pinch clicks buttons, drags objects and moves displays.</item>
    /// <item>the <b>third-person feeds</b> (6008 and 6009, local 6015 and 6016) on two more displays: front-left on its own on the
    /// operator's left, front-right above the wrist camera's display. All three displays float and move with the pointer.</item>
    /// </list>
    /// The JetRover scene is edited by hand, so this replaces only its own roots (<see cref="RootNames"/>) and adds a
    /// <see cref="DisplayMover"/> to the lab's display; everything else is left as it is. The real robot sends no world, so with it
    /// the twin stays empty and the world buttons say so. <c>XrSceneCheck</c> checks the result.
    /// </summary>
    public static class XrJetRoverWorldSetup
    {
        public const string SceneResource = "jetrover_scene";
        public const string WorldRootName = "JetRoverWorld";
        public const string RecordGroupName = "JetRoverRecording";
        public const string FeedsRootName = "JetRoverCameraFeeds";
        public const string PointerName = "HandPointer";
        public static readonly string[] RootNames = { WorldRootName, RecordGroupName, FeedsRootName, PointerName };

        public const string ThirdLeftFeedName = "3RD PERSON LEFT (REC)", ThirdRightFeedName = "3RD PERSON RIGHT (REC)";
        public const string ThirdLeftDisplayName = "CameraDisplayThirdLeft", ThirdRightDisplayName = "CameraDisplayThirdRight";
        public const int ThirdLeftFeedPort = 6008, ThirdLeftFeedLocalPort = 6015, ThirdRightFeedPort = 6009, ThirdRightFeedLocalPort = 6016;
        private const string FeedMeshLibraryPath = LabPaths.Generated + "/JetRoverFeedMeshes.asset";

        [MenuItem("Teleop/XR/Set Up JetRover World")]
        public static void RunFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Scene scene = EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
            Apply();
            EditorSceneManager.SaveScene(scene);
        }

        /// <summary>Batch entry point: <c>-executeMethod Teleop.XR.Editor.XrJetRoverWorldSetup.Run</c>.</summary>
        public static void Run()
        {
            try
            {
                Scene scene = EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
                Apply();
                EditorSceneManager.SaveScene(scene);
                AssetDatabase.SaveAssets();
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[jetrover-world] FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        public const string VisualsPath = "../../robot/jetrover/urdf/meshes/visuals.json";

        /// <summary>How far the twin's jaws open past the CAD's pose (deg), as <c>just jetrover-visuals</c> measured for Isaac's opening.</summary>
        public static float JawOpenDegrees()
        {
            string text = System.IO.File.ReadAllText(VisualsPath);
            var m = System.Text.RegularExpressions.Regex.Match(text, "\"jawOpenDegrees\":\\s*([0-9.]+)");
            if (!m.Success) throw new InvalidOperationException($"{VisualsPath} has no jawOpenDegrees: run just jetrover-visuals");
            return float.Parse(m.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture);
        }

        /// <summary>The scene file the headset ships with, validated by Core's reader.</summary>
        public static WorldScene LoadScene()
        {
            TextAsset asset = Resources.Load<TextAsset>(SceneResource)
                ?? throw new InvalidOperationException($"Resources/{SceneResource}.json is missing (a copy of core/Scenes/jetrover-bench.json)");
            return JsonUtility.FromJson<SceneDocument>(asset.text).ToScene();
        }

        /// <summary>The arm's base frame: the commanded rig's <c>baseAnchor</c>, where the world's poses are given.</summary>
        public static Transform ArmBase(JetRoverOperatorBridge bridge)
        {
            var rig = new SerializedObject(bridge).FindProperty("armRig")?.objectReferenceValue as JetRoverArmRig
                ?? throw new InvalidOperationException("JetRoverOperatorBridge.armRig is not set");
            return new SerializedObject(rig).FindProperty("baseAnchor")?.objectReferenceValue as Transform
                ?? throw new InvalidOperationException("JetRoverArmRig.baseAnchor is not set");
        }

        public static void Apply()
        {
            WorldScene world = LoadScene();
            var bridge = Object.FindAnyObjectByType<JetRoverOperatorBridge>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no JetRoverOperatorBridge");
            XROrigin origin = Object.FindAnyObjectByType<XROrigin>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no XROrigin");
            Transform armBase = ArmBase(bridge);
            Scene scene = SceneManager.GetActiveScene();
            foreach (GameObject old in scene.GetRootGameObjects().Where(g => RootNames.Contains(g.name)).ToArray())
            {
                Object.DestroyImmediate(old);
            }

            // The twin, at the arm's base frame: positions only from the anchor (it is never rotated, see JetRoverArmRig.BaseAnchor).
            var root = new GameObject(WorldRootName).transform;
            root.SetPositionAndRotation(armBase.position, armBase.rotation);
            XrFr3WorldSetup.BuildTwin(root, world, bridge, "JetRoverWorld");

            BuildButtons(bridge, origin);

            var pointer = new GameObject(PointerName).AddComponent<Fr3HandPointer>();
            XrFr3SceneBuilder.SetReference(pointer, "bridge", bridge);
            XrFr3SceneBuilder.SetReference(pointer, "origin", origin);
            XrFr3SceneBuilder.SetReference(pointer, "rayMaterial", LabMaterials.Transparent("PointerRay", Color.white));

            // Every twin gripper opens as far as Isaac's (docs/adr/0034 amendment 10): the angle JetRoverVisualExport measured.
            float openDegrees = JawOpenDegrees();
            foreach (TwinGripper g in Object.FindObjectsByType<TwinGripper>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                var gso = new SerializedObject(g);
                gso.FindProperty("jawOpenDegrees").floatValue = openDegrees;
                gso.ApplyModifiedPropertiesWithoutUndo();
            }

            DisplayMover[] movers = BuildFeeds(bridge);
            XrFr3SceneBuilder.SetReferences(pointer, "displays", movers);

            // The clutch is the right hand's pinch too: it leaves the pointer's pinches alone, and the displays to it.
            HandClutchDriver clutch = Object.FindAnyObjectByType<HandClutchDriver>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no HandClutchDriver");
            XrFr3SceneBuilder.SetReference(clutch, "handPointer", pointer);
            Debug.Log($"[jetrover-world] built '{world.Name}': a twin of {world.Objects.Length} object(s), NEW LAYOUT and RECORD, the hand pointer, " +
                      $"and the {ThirdLeftFeedName} and {ThirdRightFeedName} feeds; {movers.Length} movable displays");
        }

        /// <summary>
        /// NEW LAYOUT and the RECORD buttons on the bench's right wing, behind the training course's START and TRAIN: the left
        /// wing behind power is the console's. RECORD, nearest, stays in reach while driving (STOP); the wing is beyond the arm's
        /// reach (about 0.35 m from its base). RECORD and DISCARD LAST in front; &lt; PREV, NEXT &gt; and NEW LAYOUT behind; SIZE −
        /// and SIZE + at the back.
        /// </summary>
        private static void BuildButtons(JetRoverOperatorBridge bridge, XROrigin origin)
        {
            var root = new GameObject(RecordGroupName).transform;
            // Rows 0.22 m apart: a label stands behind and above its cap, and closer rows read as the next row's.
            float top = LabLayout.BenchTop, z = LabLayout.BenchNearZ + 0.24f, x = LabLayout.ArmBase.x;
            XrFr3SceneBuilder.BuildButton(root, "RecordButton", new Vector3(x + 0.48f, top, z), "Fr3RecordCap", new Color(0.85f, 0.15f, 0.15f), "RECORD",
                Fr3MotionButton.Role.Record, bridge, origin);
            XrFr3SceneBuilder.BuildButton(root, "DiscardLastButton", new Vector3(x + 0.70f, top, z), "Fr3DiscardCap", new Color(0.55f, 0.56f, 0.6f), "DISCARD LAST",
                Fr3MotionButton.Role.DiscardLast, bridge, origin);
            XrFr3SceneBuilder.BuildButton(root, "ScenarioPrevious", new Vector3(x + 0.42f, top, z + 0.22f), "Fr3ScenarioCap", new Color(0.25f, 0.55f, 1f), "< PREV",
                Fr3MotionButton.Role.ScenarioPrevious, bridge, origin);
            XrFr3SceneBuilder.BuildButton(root, "ScenarioNext", new Vector3(x + 0.565f, top, z + 0.22f), "Fr3ScenarioCap", new Color(0.25f, 0.55f, 1f), "NEXT >",
                Fr3MotionButton.Role.ScenarioNext, bridge, origin);
            XrFr3SceneBuilder.BuildButton(root, "NewLayoutButton", new Vector3(x + 0.765f, top, z + 0.22f), "Fr3NewLayoutCap", new Color(0.95f, 0.65f, 0.15f), "NEW LAYOUT",
                Fr3MotionButton.Role.NewLayout, bridge, origin);
            // Resizing the object a pointer last grabbed (docs/adr/0034 amendment 6): robot off, so the pointer reaches them.
            XrFr3SceneBuilder.BuildButton(root, "SizeDownButton", new Vector3(x + 0.48f, top, z + 0.44f), "Fr3SizeCap", new Color(0.3f, 0.75f, 0.55f), "SIZE \u2212",
                Fr3MotionButton.Role.SizeDown, bridge, origin);
            XrFr3SceneBuilder.BuildButton(root, "SizeUpButton", new Vector3(x + 0.70f, top, z + 0.44f), "Fr3SizeCap", new Color(0.3f, 0.75f, 0.55f), "SIZE +",
                Fr3MotionButton.Role.SizeUp, bridge, origin);
            WarnOverlaps(root);
        }

        /// <summary>The scene is edited by hand: says which existing renderers a new button's footprint would sit in.</summary>
        private static void WarnOverlaps(Transform buttons)
        {
            var mine = new HashSet<Renderer>(buttons.GetComponentsInChildren<Renderer>(true));
            foreach (Transform button in buttons.Cast<Transform>())
            {
                var footprint = new Bounds(button.position + new Vector3(0f, 0.04f, 0f), new Vector3(0.09f, 0.06f, 0.09f));
                foreach (Renderer r in Object.FindObjectsByType<Renderer>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                {
                    if (mine.Contains(r) || !r.enabled || r.bounds.size.y > 0.5f || !r.bounds.Intersects(footprint)) continue;
                    if (r.bounds.max.y <= LabLayout.BenchTop + 0.002f) continue; // the bench top itself
                    Debug.LogWarning($"[jetrover-world] {button.name} overlaps '{r.name}' at {r.bounds.center}: move one of them by hand");
                }
            }
        }

        /// <summary>The two third-person feeds on new displays, and every display (the lab's too) floating and movable.</summary>
        private static DisplayMover[] BuildFeeds(JetRoverOperatorBridge bridge)
        {
            Scene scene = SceneManager.GetActiveScene();
            GameObject dressing = scene.GetRootGameObjects().FirstOrDefault(g => g.name == LabDressing.RootName)
                ?? throw new InvalidOperationException("the scene has no LabDressing");
            Transform wrist = dressing.transform.Find("CameraDisplay") ?? throw new InvalidOperationException("the lab has no CameraDisplay");
            Material screenMaterial = AssetDatabase.LoadAssetAtPath<Material>(XrFr3WorldSetup.ScreenMaterialAssetPath)
                ?? throw new InvalidOperationException($"{XrFr3WorldSetup.ScreenMaterialAssetPath} is missing");

            var root = new GameObject(FeedsRootName).transform;
            LabMeshLibrary.Begin(FeedMeshLibraryPath);
            CameraFeedBridge left = XrFr3WorldSetup.Feed(root, "ThirdLeftFeed", ThirdLeftFeedName, ThirdLeftFeedPort, ThirdLeftFeedLocalPort, null,
                XrFr3WorldSetup.ThirdFeedMaxFps, CameraFeedBridge.AddressSource.JetRover);
            LabDisplay.Result l = LabDisplay.Build(root, LabPalette.Create(), screenMaterial, left, bridge,
                "third-person camera, front-left  ·  Isaac, recorded", ThirdLeftDisplayName, mirrored: true, at: XrFr3WorldSetup.LeftOf(LabLayout.DisplayFoot));
            CameraFeedBridge right = XrFr3WorldSetup.Feed(root, "ThirdRightFeed", ThirdRightFeedName, ThirdRightFeedPort, ThirdRightFeedLocalPort, null,
                XrFr3WorldSetup.ThirdFeedMaxFps, CameraFeedBridge.AddressSource.JetRover);
            LabDisplay.Result r = LabDisplay.Build(root, LabPalette.Create(), screenMaterial, right, bridge,
                "third-person camera, front-right  ·  Isaac, recorded", ThirdRightDisplayName, at: XrFr3WorldSetup.Above(LabLayout.DisplayFoot));
            LabMeshLibrary.End();
            foreach ((CameraFeedBridge f, Renderer screen) in new[] { (left, l.Screen), (right, r.Screen) })
            {
                var feed = new SerializedObject(f);
                feed.FindProperty("panel").objectReferenceValue = screen;
                feed.ApplyModifiedPropertiesWithoutUndo();
            }

            // The upper display without its nameplate, which would hang over the wrist camera's display below.
            foreach (Transform child in root.Find(ThirdRightDisplayName).GetComponentsInChildren<Transform>(true))
            {
                if (child.name.StartsWith("Nameplate", StringComparison.Ordinal) || child.name.EndsWith("Nameplate", StringComparison.Ordinal)) child.gameObject.SetActive(false);
            }

            Transform[] displays = { wrist, root.Find(ThirdLeftDisplayName), root.Find(ThirdRightDisplayName) };
            var movers = new DisplayMover[displays.Length];
            for (int i = 0; i < displays.Length; i++)
            {
                Transform d = displays[i] ?? throw new InvalidOperationException("a camera display was not built");
                foreach (Transform child in d.GetComponentsInChildren<Transform>(true))
                {
                    if (child != d && child.name.EndsWith("Stand", StringComparison.Ordinal)) child.gameObject.SetActive(false);
                }

                DisplayMover mover = d.GetComponent<DisplayMover>();
                if (mover == null) mover = d.gameObject.AddComponent<DisplayMover>();
                var mso = new SerializedObject(mover);
                mso.FindProperty("freeVertical").boolValue = true;
                mso.FindProperty("grabMargin").floatValue = 0.05f;
                mso.ApplyModifiedPropertiesWithoutUndo();
                movers[i] = mover;
            }

            return movers;
        }
    }
}
