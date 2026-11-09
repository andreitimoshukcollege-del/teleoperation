using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using Teleop.Bridge;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// The JetRover's look for Isaac (docs/adr/0034 amendment 6): the lab twin's CAD meshes, written as one OBJ per URDF link and
    /// colour into <c>robot/jetrover/urdf/meshes/</c>, with <c>visuals.json</c> naming each file's link and colour, from which
    /// <c>JetRoverSimModel</c> writes the URDF's <c>&lt;visual&gt;</c> elements. <c>just jetrover-visuals</c>. Reads the lab scene;
    /// saves nothing in it.
    /// <list type="bullet">
    /// <item><b>Frames.</b> As <see cref="RobotCollisionExtractor"/>: the commanded arm at every joint 0 with the gripper open, each
    /// part in its own pivot's frame, in Core's axes. Those are the URDF's link frames: its chain is Core's forward kinematics
    /// (<c>JetRoverSimModelTests</c>), and the extractor checks the pivots against Core's frames within 1 mm. The chassis goes
    /// to <c>base_link</c>, the turntable to <c>turntable</c>, the three arm parts to their links, the gripper body to
    /// <c>gripper_link</c>, and each jaw to the finger on its side (which slides in the URDF where the real jaw turns).</item>
    /// <item><b>Decimated</b> by vertex clustering on a <see cref="Cell"/> grid, a cell's vertices kept apart by which way they
    /// face, so edges stay sharp: the CAD has about 170k vertices an arm, the renderer draws it at 640 × 360.</item>
    /// <item><b>Colours</b> are the lab palette's (the materials <see cref="JetRoverCad"/> gives the parts), one file per colour.</item>
    /// <item><b>Plain text in git, not LFS:</b> the GPU machine clones from GitHub, and an LFS pointer would import as nothing.</item>
    /// </list>
    /// </summary>
    public static class JetRoverVisualExport
    {
        /// <summary>The decimation grid (m).</summary>
        private const float Cell = 0.0015f;

        /// <summary>How much of each jaw, back from its tip, is its pad (m).</summary>
        private const float PadLength = 0.03f;
        private const string OutputFolder = "../../robot/jetrover/urdf/meshes";

        public static void Run()
        {
            try
            {
                Debug.Log("[visuals] " + Export());
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError("[visuals] FAILED: " + e);
                EditorApplication.Exit(1);
            }
        }

        private sealed class Group
        {
            public string Link;
            public Color Colour;
            public readonly List<Vector3> Positions = new List<Vector3>(); // Core axes, metres
            public readonly List<Vector3> Normals = new List<Vector3>();
            public readonly List<int> Triangles = new List<int>();
        }

        private static string Export()
        {
            EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
            JetRoverArmRig rig = XrControlSetup.CommandedRig() ?? throw new InvalidOperationException("no commanded arm rig in the scene");
            var rigSo = new SerializedObject(rig);
            Transform Pivot(string field) => rigSo.FindProperty(field).objectReferenceValue as Transform
                ?? throw new InvalidOperationException($"the rig's {field} is not set");
            Transform baseAnchor = Pivot("baseAnchor"), yaw = Pivot("baseYawPivot"), lower = Pivot("lowerPitchPivot"),
                middle = Pivot("middlePitchPivot"), upper = Pivot("upperPitchPivot");
            TwinGripper gripper = rig.GetComponentsInChildren<TwinGripper>(true).FirstOrDefault()
                ?? Object.FindObjectsByType<TwinGripper>(FindObjectsInactive.Include).FirstOrDefault(g => g.transform.IsChildOf(rig.transform) || g.gameObject == rig.gameObject)
                ?? throw new InvalidOperationException("no TwinGripper on the commanded arm");
            Transform roll = upper.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == JetRoverTwin.RollName)
                ?? throw new InvalidOperationException("no TwinRoll group on the upper link");
            Transform chassis = Object.FindObjectsByType<Transform>(FindObjectsInactive.Include).FirstOrDefault(t => t.name == "JetRoverChassis")
                ?? throw new InvalidOperationException("no JetRoverChassis in the scene: rebuild the lab with the CAD (just unity-lab-xr)");
            if (!JetRoverCad.Available) throw new InvalidOperationException($"the CAD files are not in {JetRoverCad.Folder} (git lfs pull)");

            var gso = new SerializedObject(gripper);
            Transform Jaw(string field) => gso.FindProperty(field).objectReferenceValue as Transform;
            Transform[] left = { Jaw("leftJaw"), Jaw("leftJawOuter") }, right = { Jaw("rightJaw"), Jaw("rightJawOuter") };
            if (left.Any(t => t == null) || right.Any(t => t == null)) throw new InvalidOperationException("the twin's gripper has no hinged jaws (the CAD gripper)");

            rig.ApplyAngles(0f, 0f, 0f, 0f, false);
            gripper.Show(0f, 0f);

            var groups = new Dictionary<(string, int), Group>();
            int rawTriangles = 0;
            void Add(string link, Transform root, Transform frame, Func<Transform, bool> skip)
            {
                foreach (MeshRenderer r in root.GetComponentsInChildren<MeshRenderer>(true))
                {
                    if (!r.enabled || r.name.Contains("ReachLed") || skip(r.transform)) continue;
                    MeshFilter mf = r.GetComponent<MeshFilter>();
                    if (mf == null || mf.sharedMesh == null) continue;
                    rawTriangles += AddMesh(groups, link, mf.sharedMesh, r.sharedMaterials, frame.worldToLocalMatrix * mf.transform.localToWorldMatrix);
                }
            }

            bool InJaw(Transform t, Transform[] jaw) => jaw.Any(j => t == j || t.IsChildOf(j));
            foreach (Transform part in chassis.Cast<Transform>()) Add("base_link", part, baseAnchor, _ => false);
            Add("turntable", Visual(yaw), yaw, _ => false);
            Add("proximal_link", Visual(lower), lower, _ => false);
            Add("distal_link", Visual(middle), middle, _ => false);
            Add("wrist_link", Visual(upper), upper, t => t.IsChildOf(roll));
            Add("gripper_link", roll, roll, t => InJaw(t, left) || InJaw(t, right));
            foreach (Transform j in left.Distinct()) Add("left_finger", j, roll, _ => false);
            foreach (Transform j in right.Distinct()) Add("right_finger", j, roll, _ => false);

            // Each jaw on its own finger's side: the URDF's left finger is at +y and closes toward -y (its collision box says so),
            // and the twin's "left" jaw is not always there. Measured, not named: swap them when the left one lies at -y.
            double MeanY(string link)
            {
                double sum = 0;
                int n = 0;
                foreach (Group g in groups.Values.Where(x => x.Link == link)) foreach (Vector3 p in g.Positions) { sum += p.y; n++; }
                return n > 0 ? sum / n : 0;
            }

            if (MeanY("left_finger") < 0 && MeanY("right_finger") > 0)
            {
                foreach (Group g in groups.Values) g.Link = g.Link == "left_finger" ? "right_finger" : g.Link == "right_finger" ? "left_finger" : g.Link;
            }
            else if (MeanY("left_finger") < 0 || MeanY("right_finger") > 0)
            {
                throw new InvalidOperationException("the jaws are not one on each side of the gripper");
            }

            string folder = Path.GetFullPath(OutputFolder);
            Directory.CreateDirectory(folder);
            foreach (string old in Directory.GetFiles(folder, "*.obj")) File.Delete(old);
            var entries = new List<string>();
            var report = new StringBuilder();
            int kept = 0;
            foreach (var byLink in groups.Values.GroupBy(g => g.Link).OrderBy(g => LinkOrder(g.Key)))
            {
                int k = 0;
                foreach (Group g in byLink.OrderByDescending(x => x.Triangles.Count))
                {
                    Group d = Decimate(g);
                    if (d.Triangles.Count == 0) continue;
                    string file = $"{g.Link}_{k}.obj";
                    File.WriteAllText(Path.Combine(folder, file), Obj(d, $"{g.Link}_{k}"), new UTF8Encoding(false));
                    entries.Add($"    {{\"link\": \"{g.Link}\", \"file\": \"meshes/{file}\", \"rgba\": [{F(g.Colour.r)}, {F(g.Colour.g)}, {F(g.Colour.b)}, 1], \"triangles\": {d.Triangles.Count / 3}}}");
                    report.Append($"{file} {g.Triangles.Count / 3}->{d.Triangles.Count / 3}; ");
                    kept += d.Triangles.Count / 3;
                    k++;
                }
            }

            // Each finger's pad (docs/adr/0034 amendment 8): the box round its jaw's last PadLength towards the tip, in the finger's
            // frame. The URDF's fingers collide as these, so an object fits between the pads, not between padded jaw boxes.
            var pads = new List<string>();
            foreach (string finger in new[] { "left_finger", "right_finger" })
            {
                List<Vector3> pts = groups.Values.Where(x => x.Link == finger).SelectMany(x => x.Positions).ToList();
                float tip = pts.Max(p => p.x);
                List<Vector3> pad = pts.Where(p => p.x >= tip - PadLength).ToList();
                Vector3 lo = new Vector3(pad.Min(p => p.x), pad.Min(p => p.y), pad.Min(p => p.z));
                Vector3 hi = new Vector3(pad.Max(p => p.x), pad.Max(p => p.y), pad.Max(p => p.z));
                Vector3 c = (lo + hi) / 2, sz = hi - lo;
                pads.Add($"    {{\"link\": \"{finger}\", \"center\": [{P(c.x)}, {P(c.y)}, {P(c.z)}], \"size\": [{P(sz.x)}, {P(sz.y)}, {P(sz.z)}]}}");
            }

            string hashes = string.Join(", ", new[] { "Body", "Base", "LowerArm", "CenterArm", "UpperArm", "Gripper" }.Select(n =>
                $"\"{n}.fbx\": \"{Sha(Path.Combine(JetRoverCad.Folder, n + ".fbx"))}\""));
            string manifest = "{\n  \"format\": \"jetrover-visuals\",\n  \"version\": 1,\n" +
                              "  \"about\": \"Written by unity/TeleopXR JetRoverVisualExport (just jetrover-visuals, docs/adr/0034 amendment 6) from the lab twin's CAD. Do not edit.\",\n" +
                              $"  \"cellMeters\": {F(Cell)},\n  \"sources\": {{{hashes}}},\n  \"visuals\": [\n" + string.Join(",\n", entries) + "\n  ],\n" +
                              "  \"fingerPads\": [\n" + string.Join(",\n", pads) + "\n  ]\n}\n";
            File.WriteAllText(Path.Combine(folder, "visuals.json"), manifest, new UTF8Encoding(false));
            return $"wrote {entries.Count} meshes, {rawTriangles} -> {kept} triangles, to {folder}: {report}";
        }

        private static int LinkOrder(string link) => Array.IndexOf(new[] { "base_link", "turntable", "proximal_link", "distal_link", "wrist_link", "gripper_link", "left_finger", "right_finger" }, link);

        private static Transform Visual(Transform pivot) =>
            pivot.Cast<Transform>().FirstOrDefault(t => t.name == JetRoverTwin.VisualName)
            ?? throw new InvalidOperationException($"{pivot.name} has no {JetRoverTwin.VisualName}");

        /// <summary>Unity local (x right, y up, z forward) to Core (x forward, y left, z up). A reflection, so Unity's clockwise front faces come out anticlockwise, as OBJ wants.</summary>
        private static Vector3 ToCoreAxes(Vector3 v) => new Vector3(v.z, -v.x, v.y);

        private static int AddMesh(Dictionary<(string, int), Group> groups, string link, Mesh mesh, Material[] materials, Matrix4x4 toFrame)
        {
            Vector3[] v = mesh.vertices;
            Vector3[] n = mesh.normals;
            Matrix4x4 normalMatrix = toFrame.inverse.transpose;
            int count = 0;
            for (int s = 0; s < mesh.subMeshCount; s++)
            {
                Material m = materials.Length > 0 ? materials[Math.Min(s, materials.Length - 1)] : null;
                Color c = m == null ? Color.gray : m.HasProperty("_BaseColor") ? m.GetColor("_BaseColor") : m.HasProperty("_Color") ? m.color : Color.gray;
                int key = (Mathf.RoundToInt(c.r * 255) << 16) | (Mathf.RoundToInt(c.g * 255) << 8) | Mathf.RoundToInt(c.b * 255);
                if (!groups.TryGetValue((link, key), out Group g)) groups[(link, key)] = g = new Group { Link = link, Colour = c };
                int[] tri = mesh.GetTriangles(s);
                var map = new Dictionary<int, int>();
                foreach (int i in tri)
                {
                    if (!map.TryGetValue(i, out int j))
                    {
                        j = g.Positions.Count;
                        map[i] = j;
                        g.Positions.Add(ToCoreAxes(toFrame.MultiplyPoint3x4(v[i])));
                        Vector3 normal = n.Length == v.Length ? normalMatrix.MultiplyVector(n[i]).normalized : Vector3.up;
                        g.Normals.Add(ToCoreAxes(normal));
                    }

                    g.Triangles.Add(j);
                }

                count += tri.Length / 3;
            }

            return count;
        }

        /// <summary>Vertex clustering: one vertex per grid cell and facing (the dominant axis of its normal, with its sign), at their mean.</summary>
        private static Group Decimate(Group g)
        {
            var d = new Group { Link = g.Link, Colour = g.Colour };
            var cellOf = new Dictionary<(int, int, int, int), int>();
            var sums = new List<(Vector3 P, Vector3 N, int K)>();
            var remap = new int[g.Positions.Count];
            for (int i = 0; i < g.Positions.Count; i++)
            {
                Vector3 p = g.Positions[i], nm = g.Normals[i];
                float ax = Mathf.Abs(nm.x), ay = Mathf.Abs(nm.y), az = Mathf.Abs(nm.z);
                int facing = ax >= ay && ax >= az ? (nm.x >= 0 ? 0 : 1) : ay >= az ? (nm.y >= 0 ? 2 : 3) : (nm.z >= 0 ? 4 : 5);
                var key = (Mathf.FloorToInt(p.x / Cell), Mathf.FloorToInt(p.y / Cell), Mathf.FloorToInt(p.z / Cell), facing);
                if (!cellOf.TryGetValue(key, out int c))
                {
                    c = sums.Count;
                    cellOf[key] = c;
                    sums.Add((Vector3.zero, Vector3.zero, 0));
                }

                var s = sums[c];
                sums[c] = (s.P + p, s.N + nm, s.K + 1);
                remap[i] = c;
            }

            foreach (var s in sums)
            {
                d.Positions.Add(s.P / s.K);
                d.Normals.Add(s.N.sqrMagnitude > 1e-12f ? s.N.normalized : Vector3.forward);
            }

            var seen = new HashSet<(int, int, int)>();
            for (int t = 0; t < g.Triangles.Count; t += 3)
            {
                int a = remap[g.Triangles[t]], b = remap[g.Triangles[t + 1]], c = remap[g.Triangles[t + 2]];
                if (a == b || b == c || a == c) continue;
                // One copy of each triangle whichever vertex it starts from.
                int lo = Math.Min(a, Math.Min(b, c));
                var key = lo == a ? (a, b, c) : lo == b ? (b, c, a) : (c, a, b);
                if (!seen.Add(key)) continue;
                d.Triangles.Add(a);
                d.Triangles.Add(b);
                d.Triangles.Add(c);
            }

            // Drop the vertices no triangle kept.
            var used = new int[d.Positions.Count];
            for (int i = 0; i < used.Length; i++) used[i] = -1;
            var compact = new Group { Link = d.Link, Colour = d.Colour };
            foreach (int i in d.Triangles)
            {
                if (used[i] < 0)
                {
                    used[i] = compact.Positions.Count;
                    compact.Positions.Add(d.Positions[i]);
                    compact.Normals.Add(d.Normals[i]);
                }

                compact.Triangles.Add(used[i]);
            }

            return compact;
        }

        private static string Obj(Group g, string name)
        {
            var s = new StringBuilder();
            s.Append("# The JetRover's CAD, ").Append(name).Append(", in the URDF link's frame (metres, x forward, y left, z up).\n");
            s.Append("# Written by unity/TeleopXR JetRoverVisualExport (just jetrover-visuals). Do not edit.\n");
            s.Append("o ").Append(name).Append('\n');
            foreach (Vector3 p in g.Positions) s.Append("v ").Append(P(p.x)).Append(' ').Append(P(p.y)).Append(' ').Append(P(p.z)).Append('\n');
            foreach (Vector3 n in g.Normals) s.Append("vn ").Append(N(n.x)).Append(' ').Append(N(n.y)).Append(' ').Append(N(n.z)).Append('\n');
            for (int t = 0; t < g.Triangles.Count; t += 3)
            {
                int a = g.Triangles[t] + 1, b = g.Triangles[t + 1] + 1, c = g.Triangles[t + 2] + 1;
                s.Append("f ").Append(a).Append("//").Append(a).Append(' ').Append(b).Append("//").Append(b).Append(' ').Append(c).Append("//").Append(c).Append('\n');
            }

            return s.ToString();
        }

        private static string P(float v) => v.ToString("0.######", CultureInfo.InvariantCulture);
        private static string N(float v) => v.ToString("0.####", CultureInfo.InvariantCulture);
        private static string F(float v) => v.ToString("0.####", CultureInfo.InvariantCulture);

        private static string Sha(string path)
        {
            using var sha = SHA256.Create();
            return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", "").ToLowerInvariant();
        }
    }
}
