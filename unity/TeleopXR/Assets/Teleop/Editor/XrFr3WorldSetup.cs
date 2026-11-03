using System;
using System.Linq;
using Teleop.Bridge;
using Teleop.RobotArm.World;
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

            Debug.Log($"[fr3-world] built '{world.Name}': {world.Surfaces.Count(s => s.Display == SurfaceDisplay.Show)} surface(s) from the scene file");
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
