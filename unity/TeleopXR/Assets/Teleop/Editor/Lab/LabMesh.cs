using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Accumulates simple shapes into one mesh: chamfered boxes, lathed solids (cylinders, discs,
    /// rollers), and tori. Each prop of the lab is one <see cref="LabMesh"/> per material, so a desk is a
    /// handful of draw calls rather than dozens of cubes.
    ///
    /// <para>The shapes are cheap on purpose: a chamfered box is 44 triangles. The chamfer is what lets
    /// light catch an edge, which a plain cube never does. Texture coordinates are planar and in metres
    /// (times <see cref="UvScale"/>), so a tiling texture keeps one scale across every prop.</para>
    /// </summary>
    internal sealed class LabMesh
    {
        private readonly List<Vector3> _positions = new List<Vector3>();
        private readonly List<Vector3> _normals = new List<Vector3>();
        private readonly List<Vector2> _uvs = new List<Vector2>();
        private readonly List<int> _triangles = new List<int>();

        /// <summary>Texture repeats per metre for the planar UVs.</summary>
        public float UvScale = 1f;

        public int TriangleCount => _triangles.Count / 3;
        public bool IsEmpty => _triangles.Count == 0;

        // ---- boxes --------------------------------------------------------------------------

        public LabMesh Box(Vector3 center, Vector3 size) => Chamfer(center, size, 0f, Quaternion.identity);

        public LabMesh Box(Vector3 center, Vector3 size, Quaternion rotation) => Chamfer(center, size, 0f, rotation);

        public LabMesh Chamfer(Vector3 center, Vector3 size, float bevel) => Chamfer(center, size, bevel, Quaternion.identity);

        /// <summary>A box whose 12 edges are cut at 45°, <paramref name="bevel"/> metres deep.</summary>
        public LabMesh Chamfer(Vector3 center, Vector3 size, float bevel, Quaternion rotation)
        {
            Vector3 h = size * 0.5f;
            float b = Mathf.Clamp(bevel, 0f, Mathf.Min(h.x, Mathf.Min(h.y, h.z)) * 0.95f);
            Vector3 P(float x, float y, float z) => center + rotation * new Vector3(x, y, z);
            Vector3 N(Vector3 n) => (rotation * n).normalized;

            // Six faces, each inset by the bevel on its four sides.
            for (int axis = 0; axis < 3; axis++)
            {
                for (int s = -1; s <= 1; s += 2)
                {
                    Vector3 n = Vector3.zero;
                    n[axis] = s;
                    int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
                    Vector3 c = Vector3.zero;
                    c[axis] = s * h[axis];
                    Vector3 e1 = Vector3.zero, e2 = Vector3.zero;
                    e1[a1] = h[a1] - b;
                    e2[a2] = h[a2] - b;
                    Quad(P(c.x - e1.x - e2.x, c.y - e1.y - e2.y, c.z - e1.z - e2.z),
                         P(c.x + e1.x - e2.x, c.y + e1.y - e2.y, c.z + e1.z - e2.z),
                         P(c.x + e1.x + e2.x, c.y + e1.y + e2.y, c.z + e1.z + e2.z),
                         P(c.x - e1.x + e2.x, c.y - e1.y + e2.y, c.z - e1.z + e2.z), N(n));
                }
            }

            if (b <= 0f)
            {
                return this;
            }

            // Twelve edge strips: between the face of axis a (sign sa) and of axis c (sign sc), running along axis r.
            for (int r = 0; r < 3; r++)
            {
                int a = (r + 1) % 3, c = (r + 2) % 3;
                for (int sa = -1; sa <= 1; sa += 2)
                {
                    for (int sc = -1; sc <= 1; sc += 2)
                    {
                        Vector3 n = Vector3.zero;
                        n[a] = sa;
                        n[c] = sc;
                        Vector3 onA = Vector3.zero, onC = Vector3.zero;
                        onA[a] = sa * h[a];
                        onA[c] = sc * (h[c] - b);
                        onC[a] = sa * (h[a] - b);
                        onC[c] = sc * h[c];
                        Vector3 run = Vector3.zero;
                        run[r] = h[r] - b;
                        Quad(P((onA - run).x, (onA - run).y, (onA - run).z),
                             P((onA + run).x, (onA + run).y, (onA + run).z),
                             P((onC + run).x, (onC + run).y, (onC + run).z),
                             P((onC - run).x, (onC - run).y, (onC - run).z), N(n.normalized));
                    }
                }
            }

            // Eight corner triangles.
            for (int sx = -1; sx <= 1; sx += 2)
            {
                for (int sy = -1; sy <= 1; sy += 2)
                {
                    for (int sz = -1; sz <= 1; sz += 2)
                    {
                        Vector3 px = new Vector3(sx * h.x, sy * (h.y - b), sz * (h.z - b));
                        Vector3 py = new Vector3(sx * (h.x - b), sy * h.y, sz * (h.z - b));
                        Vector3 pz = new Vector3(sx * (h.x - b), sy * (h.y - b), sz * h.z);
                        Triangle(P(px.x, px.y, px.z), P(py.x, py.y, py.z), P(pz.x, pz.y, pz.z), N(new Vector3(sx, sy, sz).normalized));
                    }
                }
            }

            return this;
        }

        // ---- solids of revolution -----------------------------------------------------------

        /// <summary>A cylinder along <paramref name="axis"/>, centred on <paramref name="center"/>.</summary>
        public LabMesh Cylinder(Vector3 center, Vector3 axis, float radius, float length, int segments = 24, float bevel = 0f)
        {
            float h = length * 0.5f;
            float b = Mathf.Min(bevel, Mathf.Min(radius, h) * 0.9f);
            var profile = b > 0f
                ? new[] { new Vector2(0f, -h), new Vector2(radius - b, -h), new Vector2(radius, -h + b), new Vector2(radius, h - b), new Vector2(radius - b, h), new Vector2(0f, h) }
                : new[] { new Vector2(0f, -h), new Vector2(radius, -h), new Vector2(radius, h), new Vector2(0f, h) };
            return Lathe(center, axis, profile, segments);
        }

        /// <summary>
        /// Revolves a profile of (radius, position along the axis) points around <paramref name="axis"/>.
        /// Smooth around, faceted along the profile, so each profile corner reads as an edge.
        /// Points on the axis (radius 0) close the solid.
        /// </summary>
        public LabMesh Lathe(Vector3 center, Vector3 axis, IReadOnlyList<Vector2> profile, int segments = 24)
        {
            Vector3 w = axis.normalized;
            Vector3 u = Vector3.Cross(w, Mathf.Abs(Vector3.Dot(w, Vector3.up)) > 0.9f ? Vector3.right : Vector3.up).normalized;
            Vector3 v = Vector3.Cross(w, u);
            for (int p = 0; p < profile.Count - 1; p++)
            {
                Vector2 a = profile[p], c = profile[p + 1];
                Vector2 d = c - a;
                if (d.sqrMagnitude < 1e-12f)
                {
                    continue;
                }

                // Outward normal in the (radial, axial) plane, for a profile walked in +axis order.
                Vector2 n2 = new Vector2(d.y, -d.x).normalized;
                float perimeter = Mathf.PI * (a.x + c.x);
                for (int s = 0; s < segments; s++)
                {
                    float t0 = s / (float)segments * Mathf.PI * 2f, t1 = (s + 1) / (float)segments * Mathf.PI * 2f;
                    Vector3 r0 = u * Mathf.Cos(t0) + v * Mathf.Sin(t0);
                    Vector3 r1 = u * Mathf.Cos(t1) + v * Mathf.Sin(t1);
                    Vector3 p00 = center + w * a.y + r0 * a.x, p01 = center + w * a.y + r1 * a.x;
                    Vector3 p10 = center + w * c.y + r0 * c.x, p11 = center + w * c.y + r1 * c.x;
                    Vector3 n0 = (r0 * n2.x + w * n2.y).normalized, n1 = (r1 * n2.x + w * n2.y).normalized;
                    float uu0 = s / (float)segments * perimeter, uu1 = (s + 1) / (float)segments * perimeter;
                    AddQuadSmooth(p00, p01, p11, p10, n0, n1, n1, n0,
                        new Vector2(uu0, a.y), new Vector2(uu1, a.y), new Vector2(uu1, c.y), new Vector2(uu0, c.y));
                }
            }

            return this;
        }

        public LabMesh Torus(Vector3 center, Vector3 axis, float majorRadius, float minorRadius, int majorSegments = 32, int minorSegments = 10)
        {
            Vector3 w = axis.normalized;
            Vector3 u = Vector3.Cross(w, Mathf.Abs(Vector3.Dot(w, Vector3.up)) > 0.9f ? Vector3.right : Vector3.up).normalized;
            Vector3 v = Vector3.Cross(w, u);
            Vector3 Point(int i, int j, out Vector3 normal)
            {
                float a = i / (float)majorSegments * Mathf.PI * 2f, b = j / (float)minorSegments * Mathf.PI * 2f;
                Vector3 radial = u * Mathf.Cos(a) + v * Mathf.Sin(a);
                normal = (radial * Mathf.Cos(b) + w * Mathf.Sin(b)).normalized;
                return center + radial * majorRadius + normal * minorRadius;
            }

            for (int i = 0; i < majorSegments; i++)
            {
                for (int j = 0; j < minorSegments; j++)
                {
                    Vector3 a = Point(i, j, out Vector3 na), b = Point(i + 1, j, out Vector3 nb);
                    Vector3 c = Point(i + 1, j + 1, out Vector3 nc), d = Point(i, j + 1, out Vector3 nd);
                    AddQuadSmooth(a, b, c, d, na, nb, nc, nd,
                        new Vector2(i, j), new Vector2(i + 1, j), new Vector2(i + 1, j + 1), new Vector2(i, j + 1));
                }
            }

            return this;
        }

        /// <summary>A flat rectangle facing <paramref name="normal"/>, <paramref name="up"/> along its height.</summary>
        public LabMesh Panel(Vector3 center, Vector3 normal, Vector3 up, Vector2 size)
        {
            Vector3 n = normal.normalized;
            Vector3 y = Vector3.ProjectOnPlane(up, n).normalized * size.y * 0.5f;
            Vector3 x = Vector3.Cross(n, y).normalized * size.x * 0.5f;
            return Quad(center - x - y, center + x - y, center + x + y, center - x + y, n);
        }

        /// <summary>
        /// A rectangle facing <paramref name="normal"/> whose texture fills it exactly (UVs 0..1,
        /// upright as seen from the front). For screens, signs and windows, where the image must not
        /// tile or be shared between panels. <paramref name="uvRect"/> picks one cell of an atlas instead.
        /// </summary>
        public LabMesh Screen(Vector3 center, Vector3 normal, Vector3 up, Vector2 size, Rect? uvRect = null)
        {
            Vector3 n = normal.normalized;
            Vector3 y = Vector3.ProjectOnPlane(up, n).normalized * size.y * 0.5f;
            Vector3 x = Vector3.Cross(n, y).normalized * size.x * 0.5f; // the viewer's right, seen from the front
            int i = _positions.Count;
            _positions.Add(center - x - y); _positions.Add(center + x - y); _positions.Add(center + x + y); _positions.Add(center - x + y);
            for (int k = 0; k < 4; k++) _normals.Add(n);
            Rect r = uvRect ?? new Rect(0f, 0f, 1f, 1f);
            _uvs.Add(new Vector2(r.xMin, r.yMin)); _uvs.Add(new Vector2(r.xMax, r.yMin)); _uvs.Add(new Vector2(r.xMax, r.yMax)); _uvs.Add(new Vector2(r.xMin, r.yMax));
            bool front = Vector3.Dot(Vector3.Cross(_positions[i + 1] - _positions[i], _positions[i + 2] - _positions[i]), n) > 0f;
            if (front)
            {
                _triangles.Add(i); _triangles.Add(i + 1); _triangles.Add(i + 2);
                _triangles.Add(i); _triangles.Add(i + 2); _triangles.Add(i + 3);
            }
            else
            {
                _triangles.Add(i); _triangles.Add(i + 2); _triangles.Add(i + 1);
                _triangles.Add(i); _triangles.Add(i + 3); _triangles.Add(i + 2);
            }

            return this;
        }

        // ---- primitives ---------------------------------------------------------------------

        /// <summary>One flat quad; the winding is fixed to face <paramref name="normal"/> whatever the vertex order.</summary>
        public LabMesh Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 normal)
        {
            if (Vector3.Dot(Vector3.Cross(b - a, c - a), normal) < 0f)
            {
                (b, d) = (d, b);
            }

            Vector3 t = (b - a).sqrMagnitude > 1e-12f ? (b - a).normalized : Vector3.Cross(normal, Vector3.up).normalized;
            Vector3 bt = Vector3.Cross(normal, t);
            Vector2 UV(Vector3 p) => new Vector2(Vector3.Dot(p, t), Vector3.Dot(p, bt)) * UvScale;
            int i = _positions.Count;
            _positions.Add(a); _positions.Add(b); _positions.Add(c); _positions.Add(d);
            _normals.Add(normal); _normals.Add(normal); _normals.Add(normal); _normals.Add(normal);
            _uvs.Add(UV(a)); _uvs.Add(UV(b)); _uvs.Add(UV(c)); _uvs.Add(UV(d));
            _triangles.Add(i); _triangles.Add(i + 1); _triangles.Add(i + 2);
            _triangles.Add(i); _triangles.Add(i + 2); _triangles.Add(i + 3);
            return this;
        }

        public LabMesh Triangle(Vector3 a, Vector3 b, Vector3 c, Vector3 normal)
        {
            if (Vector3.Dot(Vector3.Cross(b - a, c - a), normal) < 0f)
            {
                (b, c) = (c, b);
            }

            int i = _positions.Count;
            _positions.Add(a); _positions.Add(b); _positions.Add(c);
            _normals.Add(normal); _normals.Add(normal); _normals.Add(normal);
            _uvs.Add(new Vector2(a.x + a.z, a.y) * UvScale); _uvs.Add(new Vector2(b.x + b.z, b.y) * UvScale); _uvs.Add(new Vector2(c.x + c.z, c.y) * UvScale);
            _triangles.Add(i); _triangles.Add(i + 1); _triangles.Add(i + 2);
            return this;
        }

        private void AddQuadSmooth(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 na, Vector3 nb, Vector3 nc, Vector3 nd,
            Vector2 ua, Vector2 ub, Vector2 uc, Vector2 ud)
        {
            Vector3 average = na + nb + nc + nd;
            bool flip = Vector3.Dot(Vector3.Cross(b - a, c - a), average) < 0f;
            int i = _positions.Count;
            _positions.Add(a); _positions.Add(b); _positions.Add(c); _positions.Add(d);
            _normals.Add(na); _normals.Add(nb); _normals.Add(nc); _normals.Add(nd);
            _uvs.Add(ua * UvScale); _uvs.Add(ub * UvScale); _uvs.Add(uc * UvScale); _uvs.Add(ud * UvScale);
            if (flip)
            {
                _triangles.Add(i); _triangles.Add(i + 2); _triangles.Add(i + 1);
                _triangles.Add(i); _triangles.Add(i + 3); _triangles.Add(i + 2);
            }
            else
            {
                _triangles.Add(i); _triangles.Add(i + 1); _triangles.Add(i + 2);
                _triangles.Add(i); _triangles.Add(i + 2); _triangles.Add(i + 3);
            }
        }

        /// <summary>Writes the accumulated geometry into <paramref name="mesh"/>, replacing what it held.</summary>
        public void WriteTo(Mesh mesh, bool lightmapUvs)
        {
            mesh.Clear();
            mesh.indexFormat = _positions.Count > 65000 ? IndexFormat.UInt32 : IndexFormat.UInt16;
            mesh.SetVertices(_positions);
            mesh.SetNormals(_normals);
            mesh.SetUVs(0, _uvs);
            mesh.SetTriangles(_triangles, 0);
            mesh.RecalculateBounds();
            if (lightmapUvs)
            {
                UnwrapParam.SetDefaults(out UnwrapParam settings);
                settings.hardAngle = 80f;
                settings.packMargin = 0.01f;
                if (!Unwrapping.GenerateSecondaryUVSet(mesh, settings))
                {
                    Debug.LogWarning($"[lab] lightmap UVs failed for mesh '{mesh.name}'");
                }
            }
        }
    }

    /// <summary>
    /// Every generated mesh lives as a named sub-asset of one library asset, refilled in place on each
    /// rebuild, so scene references and asset IDs stay stable and the scene file stays small. Meshes
    /// the current build did not produce are removed at <see cref="End"/>.
    /// </summary>
    internal static class LabMeshLibrary
    {
        public const string Path = LabPaths.Generated + "/LabMeshes.asset";

        private static Dictionary<string, Mesh> s_existing = new Dictionary<string, Mesh>();
        private static readonly HashSet<string> s_written = new HashSet<string>();
        private static int s_triangles;

        public static int Triangles => s_triangles;

        public static void Begin()
        {
            LabPaths.EnsureFolder(LabPaths.Generated);
            if (AssetDatabase.LoadMainAssetAtPath(Path) == null)
            {
                var placeholder = new Mesh { name = "_library" };
                AssetDatabase.CreateAsset(placeholder, Path);
            }

            s_existing = AssetDatabase.LoadAllAssetsAtPath(Path).OfType<Mesh>()
                .Where(m => m.name != "_library")
                .GroupBy(m => m.name).ToDictionary(g => g.Key, g => g.First());
            s_written.Clear();
            s_triangles = 0;
        }

        public static Mesh Store(string name, LabMesh shape, bool lightmapUvs)
        {
            if (!s_written.Add(name))
            {
                throw new System.InvalidOperationException($"[lab] two meshes are both named '{name}'");
            }

            if (!s_existing.TryGetValue(name, out Mesh mesh) || mesh == null)
            {
                mesh = new Mesh { name = name };
                AssetDatabase.AddObjectToAsset(mesh, Path);
            }

            shape.WriteTo(mesh, lightmapUvs);
            s_triangles += shape.TriangleCount;
            EditorUtility.SetDirty(mesh);
            return mesh;
        }

        public static void End()
        {
            foreach (KeyValuePair<string, Mesh> stale in s_existing.Where(e => !s_written.Contains(e.Key)).ToList())
            {
                AssetDatabase.RemoveObjectFromAsset(stale.Value);
                Object.DestroyImmediate(stale.Value, true);
            }

            AssetDatabase.SaveAssets();
        }
    }
}
