using System;
using System.Linq;
using Teleop.Bridge;
using Teleop.RobotArm.World;
using Unity.XR.CoreUtils;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using CoreVec = System.Numerics.Vector3;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// <b>Teleop → XR → Set Up FR3 World</b> (or <c>just unity-world-fr3-xr</c>): builds the FR3 scene's world from the scene file,
    /// <c>Resources/fr3_scene.json</c>, a copy of <c>core/Scenes/fr3-duo-bench.json</c> (docs/adr/0031 amendment 2). The headset
    /// simulates nothing: Isaac builds the same scene's physics from the same file. This builds every surface the file
    /// marks <c>show</c> (the work table in front of the robot) under one root, <see cref="RootName"/>, in the robot's root
    /// frame, and replaces only that root, so hand edits elsewhere in the scene survive. The scene builder calls it as well.
    /// <c>XrFr3SceneCheck</c> holds the result to the file within 1 mm.
    /// </summary>
    public static class XrFr3WorldSetup
    {
        public const string RootName = "World";
        public const string SceneResource = "fr3_scene";

        [MenuItem("Teleop/XR/Set Up FR3 World")]
        public static void RunFromMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Scene scene = EditorSceneManager.OpenScene(XrFr3SceneBuilder.ScenePath, OpenSceneMode.Single);
            Apply();
            EditorSceneManager.SaveScene(scene);
        }

        /// <summary>Batch entry point: <c>-executeMethod Teleop.XR.Editor.XrFr3WorldSetup.Run</c>.</summary>
        public static void Run()
        {
            try
            {
                Scene scene = EditorSceneManager.OpenScene(XrFr3SceneBuilder.ScenePath, OpenSceneMode.Single);
                Apply();
                EditorSceneManager.SaveScene(scene);
                AssetDatabase.SaveAssets();
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[fr3-world] FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        /// <summary>The scene file the headset ships with, validated by Core's reader.</summary>
        public static WorldScene LoadScene()
        {
            TextAsset asset = Resources.Load<TextAsset>(SceneResource)
                ?? throw new InvalidOperationException($"Resources/{SceneResource}.json is missing (a copy of core/Scenes/fr3-duo-bench.json)");
            return JsonUtility.FromJson<SceneDocument>(asset.text).ToScene();
        }

        /// <summary>The robot's root frame in the open scene: the bridge's <c>robotRoot</c>, where the twin is built.</summary>
        public static Transform RobotRoot()
        {
            var bridge = Object.FindAnyObjectByType<SerialChainOperatorBridge>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no SerialChainOperatorBridge");
            return new SerializedObject(bridge).FindProperty("robotRoot")?.objectReferenceValue as Transform
                ?? throw new InvalidOperationException("SerialChainOperatorBridge.robotRoot is not set");
        }

        /// <summary>Builds (or rebuilds) the world root in the open scene.</summary>
        public static void Apply()
        {
            WorldScene world = LoadScene();
            Transform robotRoot = RobotRoot();
            foreach (GameObject old in SceneManager.GetActiveScene().GetRootGameObjects().Where(g => g.name == RootName).ToArray())
            {
                Object.DestroyImmediate(old);
            }

            var root = new GameObject(RootName).transform;
            root.SetPositionAndRotation(robotRoot.position, robotRoot.rotation);
            foreach (SceneSurface s in world.Surfaces.Where(s => s.Display == SurfaceDisplay.Show))
            {
                BuildSurface(root, s);
            }

            if (world.Table.Display == SurfaceDisplay.Show) BuildLegs(root, world.Table);
            BuildTwin(root, world);
            var bridge = Object.FindAnyObjectByType<SerialChainOperatorBridge>(FindObjectsInactive.Include);
            XROrigin origin = Object.FindAnyObjectByType<XROrigin>(FindObjectsInactive.Include);
            XrFr3SceneBuilder.BuildButton(root, NewLayoutButtonName, XrFr3SceneBuilder.NewLayoutButtonAt, "Fr3NewLayoutCap", new Color(0.95f, 0.65f, 0.15f),
                "NEW LAYOUT", Teleop.XR.Fr3MotionButton.Role.NewLayout, bridge, origin);
            XrFr3SceneBuilder.BuildButton(root, EditObjectsButtonName, XrFr3SceneBuilder.EditObjectsButtonAt, "Fr3EditObjectsCap", new Color(0.55f, 0.35f, 0.9f),
                "EDIT OBJECTS", Teleop.XR.Fr3MotionButton.Role.EditObjects, bridge, origin);

            BuildFeeds(bridge);
            Debug.Log($"[fr3-world] built '{world.Name}': {world.Surfaces.Count(s => s.Display == SurfaceDisplay.Show)} surface(s) and a twin of {world.Objects.Length} object(s) with their ghosts from the scene file, NEW LAYOUT, EDIT OBJECTS, and the {RightFeedName} and {LeftFeedName} feeds");
        }

        public const string FeedsRootName = "Fr3CameraFeeds";
        public const string RightFeedName = "RIGHT WRIST (REC)";
        public const string LeftFeedName = "LEFT WRIST (REC)";
        public const string LeftDisplayName = "CameraDisplayLeft";
        public const int RightFeedPort = 6003, RightFeedLocalPort = 6004, LeftFeedPort = 6007, LeftFeedLocalPort = 6014;
        public const string ThirdLeftFeedName = "3RD PERSON LEFT (REC)", ThirdRightFeedName = "3RD PERSON RIGHT (REC)";
        public const string ThirdLeftDisplayName = "CameraDisplayThirdLeft", ThirdRightDisplayName = "CameraDisplayThirdRight";
        public const int ThirdLeftFeedPort = 6008, ThirdLeftFeedLocalPort = 6015, ThirdRightFeedPort = 6009, ThirdRightFeedLocalPort = 6016;

        /// <summary>The third-person feeds' rate (fps): half the wrists', so four feeds cost the headset's decoder three.</summary>
        public const int ThirdFeedMaxFps = 15;
        private const string FeedMeshLibraryPath = LabPaths.Generated + "/Fr3FeedMeshes.asset";
        private const string ScreenMaterialPath = "Assets/Teleop/Materials/CameraPanel.mat";

        /// <summary>
        /// Two of Isaac's four feeds (docs/adr/0032 decision 3, amendment 3), outside the joint link's disturbance: the right
        /// wrist camera on the lab's display, on the operator's right (built by the scene builder), and the left wrist camera
        /// on a second display, mirrored on the left, which this builds with its own mesh library. Each <see cref="CameraFeedBridge"/> asks
        /// <c>fr3_connection</c>'s <c>RemoteHost</c>, Isaac's renderer on the GPU machine. Both are under <see cref="FeedsRootName"/>,
        /// replaced on every run.
        /// </summary>
        private static void BuildFeeds(SerialChainOperatorBridge bridge)
        {
            foreach (GameObject old in SceneManager.GetActiveScene().GetRootGameObjects().Where(g => g.name == FeedsRootName).ToArray())
            {
                Object.DestroyImmediate(old);
            }

            GameObject dressing = SceneManager.GetActiveScene().GetRootGameObjects().FirstOrDefault(g => g.name == LabDressing.RootName)
                ?? throw new InvalidOperationException("the scene has no LabDressing (build the FR3 scene first)");
            Transform right = dressing.transform.Find("CameraDisplay") ?? throw new InvalidOperationException("the lab has no CameraDisplay");
            Renderer rightScreen = right.Find("Panel/Screen")?.GetComponent<Renderer>() ?? throw new InvalidOperationException("CameraDisplay has no Panel/Screen");
            Material screenMaterial = AssetDatabase.LoadAssetAtPath<Material>(ScreenMaterialPath)
                ?? throw new InvalidOperationException($"{ScreenMaterialPath} is missing");

            var root = new GameObject(FeedsRootName).transform;
            // The wrists on the two displays beside you; the third-person cameras on two more, each stacked above the wrist display
            // on its side (the user, 2026-11-06), so the view ahead over the robot stays clear. All four float, without their floor stands, and move with a pointing pinch while the robot is off,
            // up and down too (Fr3HandPointer, DisplayMover).
            CameraFeedBridge head = Feed(root, "RightWristFeed", RightFeedName, RightFeedPort, RightFeedLocalPort, rightScreen);
            var bar = right.GetComponent<CameraStatusBar>();
            if (bar != null)
            {
                var so = new SerializedObject(bar);
                so.FindProperty("feed").objectReferenceValue = head;
                so.ApplyModifiedPropertiesWithoutUndo();
            }

            Transform caption = right.Find("Panel/DisplayCaption");
            if (caption != null) caption.GetComponent<TMPro.TMP_Text>().text = "right wrist camera  ·  Isaac, recorded";

            LabMeshLibrary.Begin(FeedMeshLibraryPath);
            CameraFeedBridge third = Feed(root, "LeftWristFeed", LeftFeedName, LeftFeedPort, LeftFeedLocalPort, null);
            LabDisplay.Result left = LabDisplay.Build(root, LabPalette.Create(), screenMaterial, third, bridge,
                "left wrist camera  ·  Isaac, recorded", LeftDisplayName, mirrored: true);
            CameraFeedBridge thirdLeft = Feed(root, "ThirdLeftFeed", ThirdLeftFeedName, ThirdLeftFeedPort, ThirdLeftFeedLocalPort, null, ThirdFeedMaxFps);
            LabDisplay.Result tl = LabDisplay.Build(root, LabPalette.Create(), screenMaterial, thirdLeft, bridge,
                "third-person camera, front-left  ·  Isaac, recorded", ThirdLeftDisplayName, at: Above(LeftOf(LabLayout.DisplayFoot)));
            CameraFeedBridge thirdRight = Feed(root, "ThirdRightFeed", ThirdRightFeedName, ThirdRightFeedPort, ThirdRightFeedLocalPort, null, ThirdFeedMaxFps);
            LabDisplay.Result tr = LabDisplay.Build(root, LabPalette.Create(), screenMaterial, thirdRight, bridge,
                "third-person camera, front-right  ·  Isaac, recorded", ThirdRightDisplayName, at: Above(LabLayout.DisplayFoot));
            LabMeshLibrary.End();
            foreach ((CameraFeedBridge f, Renderer screen) in new[] { (third, left.Screen), (thirdLeft, tl.Screen), (thirdRight, tr.Screen) })
            {
                var feed = new SerializedObject(f);
                feed.FindProperty("panel").objectReferenceValue = screen;
                feed.ApplyModifiedPropertiesWithoutUndo();
            }

            // The upper pair without their nameplates, which would hang over the displays below.
            foreach (string upper in new[] { ThirdLeftDisplayName, ThirdRightDisplayName })
            {
                foreach (Transform child in root.Find(upper).GetComponentsInChildren<Transform>(true))
                {
                    if (child.name.StartsWith("Nameplate", StringComparison.Ordinal) || child.name.EndsWith("Nameplate", StringComparison.Ordinal)) child.gameObject.SetActive(false);
                }
            }

            // Movable, floating: each display's stand hidden, and a mover the hand pointer drives.
            Transform[] displays = { right, root.Find(LeftDisplayName), root.Find(ThirdLeftDisplayName), root.Find(ThirdRightDisplayName) };
            var movers = new DisplayMover[displays.Length];
            for (int i = 0; i < displays.Length; i++)
            {
                Transform d = displays[i] ?? throw new InvalidOperationException("a camera display was not built");
                foreach (Transform child in d.GetComponentsInChildren<Transform>(true))
                {
                    if (child != d && child.name.EndsWith("Stand", StringComparison.Ordinal)) child.gameObject.SetActive(false);
                }

                DisplayMover mover = d.GetComponent<DisplayMover>() ?? d.gameObject.AddComponent<DisplayMover>();
                var mso = new SerializedObject(mover);
                mso.FindProperty("freeVertical").boolValue = true;
                mso.FindProperty("grabMargin").floatValue = 0.05f;
                mso.ApplyModifiedPropertiesWithoutUndo();
                movers[i] = mover;
            }

            Fr3HandPointer pointer = Object.FindAnyObjectByType<Fr3HandPointer>(FindObjectsInactive.Include);
            if (pointer != null) XrFr3SceneBuilder.SetReferences(pointer, "displays", movers);
        }

        /// <summary>The upper display's origin: its screen's bottom just clears the top of the one below, the upper one having no nameplate.</summary>
        private static Vector3 Above(Vector3 foot) => foot + new Vector3(0f, 1.15f, 0f);

        private static Vector3 LeftOf(Vector3 foot) => new Vector3(2f * LabLayout.OperatorEye.x - foot.x, foot.y, foot.z);

        private static CameraFeedBridge Feed(Transform root, string objectName, string feedName, int port, int localPort, Renderer screen, int maxFps = 0)
        {
            var go = new GameObject(objectName);
            go.transform.SetParent(root, false);
            var feed = go.AddComponent<CameraFeedBridge>();
            var so = new SerializedObject(feed);
            so.FindProperty("panel").objectReferenceValue = screen;
            // The display's screen is 1.2 x 0.9 m (4:3); a 16:9 feed fills its width.
            so.FindProperty("panelHeightMeters").floatValue = LabDisplay.ScreenHeight;
            so.FindProperty("panelMaxWidthMeters").floatValue = LabDisplay.ScreenWidth;
            so.FindProperty("feedName").stringValue = feedName;
            so.FindProperty("addressSource").enumValueIndex = (int)CameraFeedBridge.AddressSource.SerialChain;
            so.FindProperty("robotPort").intValue = port;
            so.FindProperty("localPort").intValue = localPort;
            so.FindProperty("maxFramesPerSecond").intValue = maxFps;
            so.ApplyModifiedPropertiesWithoutUndo();
            return feed;
        }

        public const string TwinName = "WorldTwin";
        public const string NewLayoutButtonName = "NewLayoutButton";
        public const string EditObjectsButtonName = "EditObjectsButton";

        /// <summary>
        /// The world twin: one visual per scene object (the built-in primitive of its shape, its colour, no collider), at its nominal
        /// pose so previews show it, under a <see cref="SerialChainWorldTwin"/> that hides each at runtime until the robot reports it.
        /// </summary>
        private static void BuildTwin(Transform root, WorldScene world)
        {
            var twinRoot = new GameObject(TwinName).transform;
            twinRoot.SetParent(root, false);
            var objects = new Transform[world.Objects.Length];
            var ghosts = new Transform[world.Objects.Length];
            var ids = new int[world.Objects.Length];
            Color ghostColor = new Color(0.75f, 0.6f, 1f, 0.35f);
            Material ghostMaterial = LabMaterials.Transparent("Fr3WorldGhost", ghostColor);
            for (int i = 0; i < world.Objects.Length; i++)
            {
                SceneObject o = world.Objects[i];
                PrimitiveType type = o.Shape == SceneShape.Box ? PrimitiveType.Cube : o.Shape == SceneShape.Cylinder ? PrimitiveType.Cylinder : PrimitiveType.Sphere;
                GameObject go = GameObject.CreatePrimitive(type);
                Object.DestroyImmediate(go.GetComponent<Collider>());
                go.name = o.Name;
                go.transform.SetParent(twinRoot, false);
                Vector3 size = new CoreVec((float)o.SizeX, (float)o.SizeY, (float)o.SizeZ).ExtentsToUnity();
                if (type == PrimitiveType.Cylinder) size.y /= 2f; // a cylinder primitive is 2 units tall
                go.transform.localScale = size;
                go.transform.localPosition = new CoreVec((float)o.NominalX, (float)o.NominalY, (float)o.NominalZ).ToUnity();
                RigidTransformToUnity(o, go.transform);
                var renderer = go.GetComponent<MeshRenderer>();
                renderer.sharedMaterial = LabMaterials.Lit("Fr3World_" + o.Name, new Color(o.R, o.G, o.B), 0.4f);
                renderer.lightProbeUsage = UnityEngine.Rendering.LightProbeUsage.BlendProbes;
                objects[i] = go.transform;
                ids[i] = o.Id;

                // Its ghost: where EDIT OBJECTS is dragging it, see-through, until the world frames show the move.
                GameObject ghost = Object.Instantiate(go, twinRoot, false);
                ghost.name = o.Name + "_ghost";
                ghost.GetComponent<MeshRenderer>().sharedMaterial = ghostMaterial;
                ghost.SetActive(false);
                ghosts[i] = ghost.transform;
            }

            var twin = twinRoot.gameObject.AddComponent<SerialChainWorldTwin>();
            var so = new SerializedObject(twin);
            so.FindProperty("bridge").objectReferenceValue = Object.FindAnyObjectByType<SerialChainOperatorBridge>(FindObjectsInactive.Include);
            SerializedProperty list = so.FindProperty("objects");
            list.arraySize = objects.Length;
            for (int i = 0; i < objects.Length; i++) list.GetArrayElementAtIndex(i).objectReferenceValue = objects[i];
            SerializedProperty ghostList = so.FindProperty("ghosts");
            ghostList.arraySize = ghosts.Length;
            for (int i = 0; i < ghosts.Length; i++) ghostList.GetArrayElementAtIndex(i).objectReferenceValue = ghosts[i];
            SerializedProperty idList = so.FindProperty("ids");
            idList.arraySize = ids.Length;
            for (int i = 0; i < ids.Length; i++) idList.GetArrayElementAtIndex(i).intValue = ids[i];
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void RigidTransformToUnity(SceneObject o, Transform t)
        {
            var q = new System.Numerics.Quaternion((float)o.Nominal.Qx, (float)o.Nominal.Qy, (float)o.Nominal.Qz, (float)o.Nominal.Qw);
            t.localRotation = q.ToUnity();
        }

        /// <summary>The name a surface's object takes in the scene: the work table is <c>WorkTable</c>.</summary>
        public static string SurfaceObjectName(string id) => id == "table" ? "WorkTable" : "Surface_" + id;

        /// <summary>
        /// Two legs under the table's far corners, down to the floor: display only, and not in the scene file (the near edge
        /// rests on the bench). Siblings of the table, not its children, so they keep their own scale.
        /// </summary>
        private static void BuildLegs(Transform root, SceneSurface table)
        {
            var legs = new GameObject("WorkTableLegs").transform;
            legs.SetParent(root, false);
            Material metal = LabMaterials.Lit("Fr3World_legs", new Color(0.55f, 0.57f, 0.6f), 0.6f, 0.8f);
            float bottom = root.position.y + (float)(table.CenterZ - table.SizeZ / 2), floor = 0f;
            foreach (int side in new[] { -1, 1 })
            {
                GameObject leg = GameObject.CreatePrimitive(PrimitiveType.Cube);
                Object.DestroyImmediate(leg.GetComponent<Collider>());
                leg.name = side < 0 ? "Right" : "Left";
                leg.transform.SetParent(legs, false);
                var foot = new CoreVec((float)(table.XMax - 0.06), (float)(side * (table.SizeY / 2 - 0.06)), 0f).ToUnity();
                leg.transform.position = new Vector3(root.position.x + foot.x, (bottom + floor) / 2f, root.position.z + foot.z);
                leg.transform.localScale = new Vector3(0.045f, bottom - floor, 0.045f);
                leg.GetComponent<MeshRenderer>().sharedMaterial = metal;
            }
        }

        private static void BuildSurface(Transform root, SceneSurface s)
        {
            // The built-in cube mesh, saved by reference in the scene: nothing generated, nothing simulated (no collider).
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Object.DestroyImmediate(go.GetComponent<Collider>());
            go.name = SurfaceObjectName(s.Id);
            go.transform.SetParent(root, false);
            go.transform.localPosition = new CoreVec((float)s.CenterX, (float)s.CenterY, (float)s.CenterZ).ToUnity();
            go.transform.localRotation = Quaternion.identity;
            go.transform.localScale = new CoreVec((float)s.SizeX, (float)s.SizeY, (float)s.SizeZ).ExtentsToUnity();
            var renderer = go.GetComponent<MeshRenderer>();
            renderer.sharedMaterial = LabMaterials.Lit("Fr3World_" + s.Id, new Color(s.R, s.G, s.B), 0.35f);
            renderer.lightProbeUsage = UnityEngine.Rendering.LightProbeUsage.BlendProbes;
        }
    }
}
