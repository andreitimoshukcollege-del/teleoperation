using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Reports how the JetRover CAD parts in <see cref="Folder"/> imported: each file's hierarchy,
    /// every mesh's bounds in the part's own frame and in a shared frame (all parts at the origin, as
    /// exported), vertex counts and materials. It is how the parts' joint locations are found
    /// before they are fitted to the arm rig. Read-only.
    /// <c>just _unity-batch TeleopXR Teleop.XR.Editor.RobotModelInspector.Run</c>.
    /// </summary>
    public static class RobotModelInspector
    {
        public const string Folder = "Assets/Teleop/Lab/RobotModel";

        public static void Run()
        {
            var report = new StringBuilder();
            foreach (string path in Directory.GetFiles(Folder, "*.fbx").OrderBy(p => p))
            {
                string assetPath = path.Replace('\\', '/');
                var importer = AssetImporter.GetAtPath(assetPath) as ModelImporter;
                var root = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
                report.AppendLine($"== {Path.GetFileName(assetPath)}  globalScale={importer?.globalScale} useFileScale={importer?.useFileScale} fileScale={importer?.fileScale} bakeAxis={importer?.bakeAxisConversion}");
                if (root == null)
                {
                    report.AppendLine("   (did not import)");
                    continue;
                }

                report.AppendLine($"   ROOT as exported: localPos {V(root.transform.localPosition)} localEuler {V(root.transform.localEulerAngles)} localScale {V(root.transform.localScale)}");
                var instance = (GameObject)Object.Instantiate(root);
                instance.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
                foreach (Transform t in instance.GetComponentsInChildren<Transform>(true))
                {
                    string depth = new string(' ', 3 + 2 * Depth(t, instance.transform));
                    report.Append($"{depth}{t.name}: localPos {V(t.localPosition)} localEuler {V(t.localEulerAngles)} localScale {V(t.localScale)}");
                    if (t.TryGetComponent(out MeshFilter mf) && mf.sharedMesh != null)
                    {
                        Bounds world = t.GetComponent<Renderer>().bounds;
                        report.Append($"\n{depth}  mesh '{mf.sharedMesh.name}' verts {mf.sharedMesh.vertexCount} localBounds c{V(mf.sharedMesh.bounds.center)} s{V(mf.sharedMesh.bounds.size)}");
                        report.Append($"\n{depth}  WORLD bounds min{V(world.min)} max{V(world.max)} size{V(world.size)}");
                        string mats = string.Join(", ", t.GetComponent<Renderer>().sharedMaterials.Select(m => m == null ? "null" : $"{m.name}({(m.HasProperty("_BaseColor") ? m.GetColor("_BaseColor") : m.HasProperty("_Color") ? m.GetColor("_Color") : Color.clear)})"));
                        report.Append($"\n{depth}  materials: {mats}");
                    }

                    report.AppendLine();
                }

                Object.DestroyImmediate(instance);
            }

            Debug.Log("[lab] robot model report:\n" + report);
            File.WriteAllText(Path.Combine(Path.GetTempPath(), "teleop-robotmodel-report.txt"), report.ToString());
            EditorApplication.Exit(0);
        }

        /// <summary>
        /// Renders each part, at real scale and in Unity's frame, from the side (+x), the front (+z)
        /// and above, with a 1 cm grid behind, into %TEMP%/teleop-robotmodel/. For fitting the parts.
        /// </summary>
        public static void Render()
        {
            string outDir = Path.Combine(Path.GetTempPath(), "teleop-robotmodel");
            Directory.CreateDirectory(outDir);
            var camObject = new GameObject("cam") { hideFlags = HideFlags.HideAndDontSave };
            var cam = camObject.AddComponent<Camera>();
            cam.orthographic = true;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.85f, 0.87f, 0.9f);
            cam.enabled = false;
            var light = new GameObject("light") { hideFlags = HideFlags.HideAndDontSave }.AddComponent<Light>();
            light.type = LightType.Directional;
            light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            light.intensity = 1.2f;

            foreach (string path in Directory.GetFiles(Folder, "*.fbx").OrderBy(p => p))
            {
                var root = AssetDatabase.LoadAssetAtPath<GameObject>(path.Replace('\\', '/'));
                var part = (GameObject)Object.Instantiate(root);
                part.transform.localScale = Vector3.one * RealScale(root);
                Bounds b = Bounds(part);
                var views = new (string name, Vector3 dir, Vector3 up)[] { ("side", Vector3.right, Vector3.up), ("front", Vector3.forward, Vector3.up), ("top", Vector3.up, Vector3.forward) };
                foreach (var v in views)
                {
                    cam.transform.position = b.center + v.dir * 2f;
                    cam.transform.rotation = Quaternion.LookRotation(-v.dir, v.up);
                    cam.orthographicSize = Mathf.Max(b.extents.x, b.extents.y, b.extents.z) * 1.25f;
                    cam.nearClipPlane = 0.01f;
                    cam.farClipPlane = 5f;
                    var rt = new RenderTexture(800, 800, 24);
                    cam.targetTexture = rt;
                    cam.Render();
                    RenderTexture.active = rt;
                    var tex = new Texture2D(800, 800, TextureFormat.RGB24, false);
                    tex.ReadPixels(new Rect(0, 0, 800, 800), 0, 0);
                    tex.Apply();
                    RenderTexture.active = null;
                    File.WriteAllBytes(Path.Combine(outDir, $"{Path.GetFileNameWithoutExtension(path)}_{v.name}.png"), tex.EncodeToPNG());
                    Object.DestroyImmediate(tex);
                    rt.Release();
                }

                Debug.Log($"[lab] {Path.GetFileName(path)} at real scale: bounds min{V(b.min)} max{V(b.max)} (Unity axes, origin = the part's own frame)");
                Object.DestroyImmediate(part);
            }

            Object.DestroyImmediate(camObject);
            Object.DestroyImmediate(light.gameObject);
            Debug.Log($"[lab] rendered to {outDir}");
            EditorApplication.Exit(0);
        }

        /// <summary>The parts import about 4.44× too large (the wheels come out 0.43 m across, the real ones 0.097 m): their root carries 444.35 where 100 is metres.</summary>
        public static float RealScale(GameObject root) => root.transform.localScale.x * (100f / 444.3494f);

        private static Bounds Bounds(GameObject go)
        {
            Renderer[] rs = go.GetComponentsInChildren<Renderer>();
            Bounds b = rs[0].bounds;
            foreach (Renderer r in rs) b.Encapsulate(r.bounds);
            return b;
        }

        private static int Depth(Transform t, Transform root)
        {
            int d = 0;
            for (Transform p = t; p != null && p != root; p = p.parent) d++;
            return d;
        }

        private static string V(Vector3 v) => $"({v.x:0.####}, {v.y:0.####}, {v.z:0.####})";
    }
}
