using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Text.Json;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// A scene's mesh objects as they really are (docs/adr/0037 amendment 5, third revision): <c>core/Scenes/&lt;scene&gt;.shapes.json</c>,
    /// written by <c>robot/isaac/shape_survey.py</c> from each asset's render mesh. Each asset is cut along its standing z into
    /// slabs, each a convex polygon extruded from its z0 to its z1, in fractions of the asset's box, so one entry serves every
    /// size of it. Only joint-drive reads it: collide_and_recover's touch is planned against it (<see cref="ObjectShape"/>);
    /// every clearance stays on the object's box.
    /// </summary>
    public sealed class ObjectShapes
    {
        public const string Format = "teleop-object-shapes";
        public const int Version = 1;

        private readonly Dictionary<string, AssetShape> _assets;

        private ObjectShapes(string source, Dictionary<string, AssetShape> assets)
        {
            Source = source;
            _assets = assets;
        }

        /// <summary>What the file says it was measured from.</summary>
        public string Source { get; }

        public IReadOnlyDictionary<string, AssetShape> Assets => _assets;

        /// <summary>The shapes file that goes with a scene file: <c>x.json</c> beside <c>x.shapes.json</c>.</summary>
        public static string PathFor(string scenePath) =>
            Path.Combine(Path.GetDirectoryName(scenePath) ?? "", Path.GetFileNameWithoutExtension(scenePath) + ".shapes.json");

        public static ObjectShapes Load(string path) => Parse(File.ReadAllBytes(path), path);

        /// <summary>A shapes file, checked: its format, every asset's size, and every slab (bottom to top, each where the one below ends, its outline convex and counter-clockwise).</summary>
        public static ObjectShapes Parse(byte[] json, string what = "the shapes file")
        {
            using JsonDocument doc = JsonDocument.Parse(json);
            JsonElement root = doc.RootElement;
            if (!root.TryGetProperty("format", out JsonElement format) || format.GetString() != Format)
                throw new InvalidDataException($"{what} is not a {Format} file");
            if (!root.TryGetProperty("version", out JsonElement version) || version.GetInt32() != Version)
                throw new InvalidDataException($"{what}: only version {Version} is read");
            string source = root.TryGetProperty("source", out JsonElement s) ? s.GetString() ?? "" : "";
            var assets = new Dictionary<string, AssetShape>(StringComparer.Ordinal);
            foreach (JsonProperty a in root.GetProperty("assets").EnumerateObject())
            {
                string name = a.Name;
                float[] size = Floats(a.Value.GetProperty("size"), 3, $"{what}: {name}'s size");
                if (!(size[0] > 0 && size[1] > 0 && size[2] > 0)) throw new InvalidDataException($"{what}: {name}'s size must be positive");
                double excess = a.Value.TryGetProperty("excessMm", out JsonElement e) ? e.GetDouble() : double.NaN;
                var slabs = new List<ShapeSlab>();
                foreach (JsonElement slab in a.Value.GetProperty("slabs").EnumerateArray())
                {
                    float z0 = slab.GetProperty("z0").GetSingle(), z1 = slab.GetProperty("z1").GetSingle();
                    var outline = new List<Vector2>();
                    foreach (JsonElement p in slab.GetProperty("outline").EnumerateArray())
                    {
                        float[] xy = Floats(p, 2, $"{what}: {name}'s outline");
                        outline.Add(new Vector2(xy[0], xy[1]));
                    }

                    string at = $"{what}: {name}'s slab {slabs.Count}";
                    if (!(z1 > z0)) throw new InvalidDataException($"{at} is empty (z1 {z1} <= z0 {z0})");
                    if (slabs.Count > 0 && MathF.Abs(slabs[slabs.Count - 1].Z1 - z0) > 1e-5f) throw new InvalidDataException($"{at} does not start where the one below ends");
                    if (!IsConvex(outline)) throw new InvalidDataException($"{at}'s outline is not a convex counter-clockwise polygon");
                    slabs.Add(new ShapeSlab(z0, z1, outline.ToArray()));
                }

                if (slabs.Count == 0) throw new InvalidDataException($"{what}: {name} has no slabs");
                assets[name] = new AssetShape(name, new Vector3(size[0], size[1], size[2]), excess, slabs.ToArray());
            }

            return new ObjectShapes(source, assets);
        }

        /// <summary>The shape of <paramref name="o"/>, a mesh object whose asset this file has; null for any other.</summary>
        public AssetShape? For(SceneObject o) => o.Shape == SceneShape.Mesh && _assets.TryGetValue(o.Asset, out AssetShape? a) ? a : null;

        private static float[] Floats(JsonElement array, int count, string what)
        {
            if (array.ValueKind != JsonValueKind.Array || array.GetArrayLength() != count) throw new InvalidDataException($"{what} must be {count} numbers");
            var v = new float[count];
            int i = 0;
            foreach (JsonElement x in array.EnumerateArray()) v[i++] = x.GetSingle();
            return v;
        }

        /// <summary>Whether <paramref name="p"/> has at least three corners, each turning left.</summary>
        internal static bool IsConvex(IReadOnlyList<Vector2> p)
        {
            if (p.Count < 3) return false;
            for (int i = 0; i < p.Count; i++)
            {
                Vector2 a = p[i], b = p[(i + 1) % p.Count], c = p[(i + 2) % p.Count];
                if (Cross(b - a, c - b) <= 0f) return false;
            }

            return true;
        }

        internal static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;
    }

    /// <summary>One slab of an asset's shape: a convex polygon (counter-clockwise, seen from above) extruded from <see cref="Z0"/> to <see cref="Z1"/>, in fractions of the asset's box.</summary>
    public sealed class ShapeSlab
    {
        public ShapeSlab(float z0, float z1, Vector2[] outline)
        {
            Z0 = z0;
            Z1 = z1;
            Outline = outline;
        }

        public float Z0 { get; }
        public float Z1 { get; }
        public IReadOnlyList<Vector2> Outline { get; }
    }

    /// <summary>One asset's shape from a shapes file: its box (the manifest's size, metres) and its slabs.</summary>
    public sealed class AssetShape
    {
        private readonly ConcurrentDictionary<Vector3, ObjectShape> _sized = new ConcurrentDictionary<Vector3, ObjectShape>();

        public AssetShape(string asset, Vector3 size, double excessMm, ShapeSlab[] slabs)
        {
            Asset = asset;
            Size = size;
            ExcessMm = excessMm;
            Slabs = slabs;
        }

        public string Asset { get; }

        /// <summary>The asset's box, metres: the size every fraction is of.</summary>
        public Vector3 Size { get; }

        /// <summary>How far a slab's prism may stand off the mesh, at <see cref="Size"/> (the survey's measure), mm.</summary>
        public double ExcessMm { get; }

        public IReadOnlyList<ShapeSlab> Slabs { get; }

        /// <summary>This shape at an object's size (its box's half extents), in the object's frame; the same one for the same size.</summary>
        public ObjectShape At(Vector3 halfExtents) => _sized.GetOrAdd(halfExtents, h => new ObjectShape(this, h));
    }

    /// <summary>An arm box in an object's frame (its box's centre and axes): centre, unit axes, half extents, and its axis-aligned reach.</summary>
    public readonly struct ShapeBox
    {
        public readonly Vector3 Center, AxisX, AxisY, AxisZ, Half, Reach;

        public ShapeBox(Vector3 center, Vector3 axisX, Vector3 axisY, Vector3 axisZ, Vector3 half)
        {
            Center = center;
            AxisX = axisX;
            AxisY = axisY;
            AxisZ = axisZ;
            Half = half;
            Reach = Vector3.Abs(axisX) * half.X + Vector3.Abs(axisY) * half.Y + Vector3.Abs(axisZ) * half.Z;
        }

        /// <summary>Its radius along <paramref name="axis"/> (any length): how far it reaches either side of its centre, in units of that length.</summary>
        public float Radius(Vector3 axis) =>
            MathF.Abs(Vector3.Dot(AxisX, axis)) * Half.X + MathF.Abs(Vector3.Dot(AxisY, axis)) * Half.Y + MathF.Abs(Vector3.Dot(AxisZ, axis)) * Half.Z;
    }

    /// <summary>
    /// An object's own shape at its size (docs/adr/0037 amendment 5, third revision), in its frame: its box's centre and axes,
    /// metres; the slabs of its <see cref="AssetShape"/> scaled to the box. A touch is measured against it as
    /// <see cref="GapLadder"/> measures a box: the shape grown or shrunk by a distance, every slab's polygon moved out (or
    /// in) by it edge by edge, and the stack's bottom and top with it (shrunk, every slab is cut to the stack's new height; a
    /// slab shrunk past its inner radius is gone). So a point's depth inside is the least of its distances to its slab's sides,
    /// to the bottom and to the top. Offsets come in steps of <see cref="Step"/>, each slab's built when first asked for.
    /// </summary>
    public sealed class ObjectShape
    {
        public const float Step = GapLadder.Step;

        /// <summary>The ladder's reach either way: grown up to <see cref="GapLadder.MaxGap"/>, shrunk up to as much (or its <see cref="MaxDepth"/>).</summary>
        public static readonly int MaxSteps = (int)MathF.Round(GapLadder.MaxGap / Step);

        private readonly float[] _z0, _z1;
        private readonly Vector2[][] _outline;
        private readonly float[] _inner;
        private readonly Vector2[]?[][] _ladder; // [slab][steps + MaxSteps]: the polygon offset by steps * Step, or null when gone
        private readonly bool[][] _built;
        private readonly object _lock = new object();

        public ObjectShape(AssetShape asset, Vector3 halfExtents)
        {
            Asset = asset;
            int n = asset.Slabs.Count;
            Vector3 size = 2f * halfExtents;
            _z0 = new float[n];
            _z1 = new float[n];
            _outline = new Vector2[n][];
            _inner = new float[n];
            _ladder = new Vector2[]?[n][];
            _built = new bool[n][];
            for (int s = 0; s < n; s++)
            {
                ShapeSlab slab = asset.Slabs[s];
                _z0[s] = slab.Z0 * size.Z;
                _z1[s] = slab.Z1 * size.Z;
                var poly = new Vector2[slab.Outline.Count];
                for (int i = 0; i < poly.Length; i++) poly[i] = slab.Outline[i] * new Vector2(size.X, size.Y);
                _outline[s] = poly;
                _inner[s] = InnerRadiusOf(poly);
                _ladder[s] = new Vector2[2 * MaxSteps + 1][];
                _built[s] = new bool[2 * MaxSteps + 1];
            }

            float widest = 0f;
            foreach (float r in _inner) widest = MathF.Max(widest, r);
            MaxDepth = MathF.Max(0f, MathF.Min(widest, (Top - Bottom) / 2f) - 0.0005f);
            ShrinkSteps = Math.Min(MaxSteps, (int)MathF.Floor(MaxDepth / Step + 1e-3f));
        }

        public AssetShape Asset { get; }
        public int SlabCount => _z0.Length;
        public float Bottom => _z0[0];
        public float Top => _z1[_z1.Length - 1];
        public float Z0(int slab) => _z0[slab];
        public float Z1(int slab) => _z1[slab];
        public IReadOnlyList<Vector2> Outline(int slab) => _outline[slab];

        /// <summary>How far slab <paramref name="slab"/>'s polygon can shrink before it is gone: the radius of the largest circle in it, metres.</summary>
        public float InnerRadius(int slab) => _inner[slab];

        /// <summary>The deepest a pose can be measured inside the shape: its widest slab's inner radius (at most half its height), less 0.5 mm.</summary>
        public float MaxDepth { get; }

        /// <summary>How many steps the shape can shrink and still be measured (up to <see cref="MaxSteps"/>).</summary>
        public int ShrinkSteps { get; }

        /// <summary>Whether any of <paramref name="boxes"/> overlaps the shape offset by <paramref name="steps"/> (grown when positive, shrunk when negative).</summary>
        public bool Overlaps(ReadOnlySpan<ShapeBox> boxes, int steps)
        {
            float d = steps * Step;
            float bottom = Bottom - d, top = Top + d;
            if (!(top > bottom)) return false;
            foreach (ref readonly ShapeBox b in boxes)
            {
                float lo = MathF.Max(b.Center.Z - b.Reach.Z, bottom), hi = MathF.Min(b.Center.Z + b.Reach.Z, top);
                if (lo > hi) continue;
                // The slabs the box's height meets (the bottom one reaching down to the stack's bottom, the top one up to its top).
                for (int s = FirstSlabAbove(lo); s < _z0.Length && (s == 0 || _z0[s] <= hi); s++)
                {
                    if (BoxOverlaps(b, s, steps)) return true;
                }
            }

            return false;
        }

        /// <summary>The smallest inner radius among the slabs any of <paramref name="boxes"/> overlaps, as the shape is (+inf when none).</summary>
        public float TouchedInnerRadius(ReadOnlySpan<ShapeBox> boxes)
        {
            float least = float.PositiveInfinity;
            foreach (ref readonly ShapeBox b in boxes)
            {
                float lo = b.Center.Z - b.Reach.Z, hi = b.Center.Z + b.Reach.Z;
                for (int s = FirstSlabAbove(lo); s < _z0.Length && _z0[s] <= hi; s++)
                {
                    if (BoxOverlaps(b, s, 0)) least = MathF.Min(least, _inner[s]);
                }
            }

            return least;
        }

        /// <summary>The first slab whose top is at or above <paramref name="z"/> (the slabs run bottom to top).</summary>
        private int FirstSlabAbove(float z)
        {
            int lo = 0, hi = _z1.Length - 1;
            while (lo < hi)
            {
                int mid = (lo + hi) / 2;
                if (_z1[mid] < z) lo = mid + 1;
                else hi = mid;
            }

            return lo;
        }

        private bool BoxOverlaps(in ShapeBox b, int slab, int steps)
        {
            // The slab's height in the offset stack: the bottom one from the stack's bottom, the top one to its top, and each cut
            // to the stack when it shrinks.
            float d = steps * Step;
            float z0 = slab == 0 ? Bottom - d : MathF.Max(_z0[slab], Bottom - d);
            float z1 = slab == _z1.Length - 1 ? Top + d : MathF.Min(_z1[slab], Top + d);
            if (!(z1 > z0)) return false;
            Vector2[]? poly = Polygon(slab, steps);
            return poly != null && BoxPrismOverlap(b, poly, z0, z1);
        }

        /// <summary>Slab <paramref name="slab"/>'s polygon offset by <paramref name="steps"/> steps (each edge moved out by that much; in when negative), or null when it is gone.</summary>
        public Vector2[]? Polygon(int slab, int steps)
        {
            if (steps == 0) return _outline[slab];
            int k = Math.Max(-MaxSteps, Math.Min(MaxSteps, steps)) + MaxSteps;
            if (System.Threading.Volatile.Read(ref _built[slab][k])) return _ladder[slab][k];
            lock (_lock)
            {
                if (!_built[slab][k])
                {
                    _ladder[slab][k] = Offset(_outline[slab], (k - MaxSteps) * Step);
                    System.Threading.Volatile.Write(ref _built[slab][k], true);
                }

                return _ladder[slab][k];
            }
        }

        /// <summary>
        /// Whether an oriented box and a prism (a convex counter-clockwise polygon extruded from <paramref name="z0"/> to
        /// <paramref name="z1"/>) intersect: the separating-axis test over the prism's faces, the box's, and the cross
        /// products of their edges (the prism's upright edges and its polygon's sides with the box's three axes). Exact, as
        /// <see cref="ArmCollision.BoxesOverlap"/> is for two boxes.
        /// </summary>
        public static bool BoxPrismOverlap(in ShapeBox b, Vector2[] poly, float z0, float z1)
        {
            // The prism's top and bottom.
            if (b.Center.Z - b.Reach.Z > z1 || b.Center.Z + b.Reach.Z < z0) return false;
            // Its bounds across.
            float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
            foreach (Vector2 v in poly)
            {
                minX = MathF.Min(minX, v.X);
                maxX = MathF.Max(maxX, v.X);
                minY = MathF.Min(minY, v.Y);
                maxY = MathF.Max(maxY, v.Y);
            }

            if (b.Center.X - b.Reach.X > maxX || b.Center.X + b.Reach.X < minX || b.Center.Y - b.Reach.Y > maxY || b.Center.Y + b.Reach.Y < minY) return false;

            int m = poly.Length;
            // The prism's sides.
            for (int i = 0; i < m; i++)
            {
                Vector2 e = poly[(i + 1) % m] - poly[i];
                var n = new Vector3(e.Y, -e.X, 0f); // outward: the polygon runs counter-clockwise
                if (Separates(b, n, poly, z0, z1)) return false;
            }

            // The box's faces.
            if (Separates(b, b.AxisX, poly, z0, z1) || Separates(b, b.AxisY, poly, z0, z1) || Separates(b, b.AxisZ, poly, z0, z1)) return false;
            // Edge against edge: each box axis with the prism's upright edges, and with each of its polygon's sides.
            for (int k = 0; k < 3; k++)
            {
                Vector3 a = k == 0 ? b.AxisX : k == 1 ? b.AxisY : b.AxisZ;
                var up = new Vector3(a.Y, -a.X, 0f); // a x z
                if (up.LengthSquared() > 1e-12f && Separates(b, up, poly, z0, z1)) return false;
                for (int i = 0; i < m; i++)
                {
                    Vector2 e = poly[(i + 1) % m] - poly[i];
                    var axis = new Vector3(-a.Z * e.Y, a.Z * e.X, a.X * e.Y - a.Y * e.X); // a x (e, 0)
                    // Parallel edges (an angle below 1e-4 rad) give no axis of their own: the faces' axes already cover them.
                    if (axis.LengthSquared() > 1e-8f * e.LengthSquared() && Separates(b, axis, poly, z0, z1)) return false;
                }
            }

            return true;
        }

        /// <summary>Whether <paramref name="axis"/> (any length) separates the box from the prism.</summary>
        private static bool Separates(in ShapeBox b, Vector3 axis, Vector2[] poly, float z0, float z1)
        {
            float lo = float.MaxValue, hi = float.MinValue;
            foreach (Vector2 v in poly)
            {
                float p = axis.X * v.X + axis.Y * v.Y;
                lo = MathF.Min(lo, p);
                hi = MathF.Max(hi, p);
            }

            float za = axis.Z * z0, zb = axis.Z * z1;
            lo += MathF.Min(za, zb);
            hi += MathF.Max(za, zb);
            float c = Vector3.Dot(b.Center, axis), r = b.Radius(axis);
            return c - r > hi || c + r < lo;
        }

        /// <summary>A convex counter-clockwise polygon with every edge moved out by <paramref name="d"/> (in when negative): the half-planes' intersection, or null when it is empty.</summary>
        public static Vector2[]? Offset(Vector2[] poly, float d)
        {
            float big = 1f;
            foreach (Vector2 v in poly) big = MathF.Max(big, MathF.Max(MathF.Abs(v.X), MathF.Abs(v.Y)));
            big = 4f * (big + MathF.Abs(d));
            var current = new List<Vector2> { new Vector2(-big, -big), new Vector2(big, -big), new Vector2(big, big), new Vector2(-big, big) };
            var next = new List<Vector2>(poly.Length + 4);
            int m = poly.Length;
            for (int i = 0; i < m && current.Count > 0; i++)
            {
                Vector2 e = poly[(i + 1) % m] - poly[i];
                float len = e.Length();
                if (len < 1e-9f) continue;
                var n = new Vector2(e.Y, -e.X) / len;
                float c = Vector2.Dot(n, poly[i]) + d; // inside: n . p <= c
                next.Clear();
                for (int j = 0; j < current.Count; j++)
                {
                    Vector2 p = current[j], q = current[(j + 1) % current.Count];
                    float fp = Vector2.Dot(n, p) - c, fq = Vector2.Dot(n, q) - c;
                    if (fp <= 0f) next.Add(p);
                    if ((fp < 0f && fq > 0f) || (fp > 0f && fq < 0f)) next.Add(p + (q - p) * (fp / (fp - fq)));
                }

                (current, next) = (next, current);
            }

            // Drop corners closer than a micron to the one before: a vanishing edge.
            var result = new List<Vector2>(current.Count);
            foreach (Vector2 p in current)
            {
                if (result.Count == 0 || Vector2.DistanceSquared(result[result.Count - 1], p) > 1e-12f) result.Add(p);
            }

            while (result.Count > 1 && Vector2.DistanceSquared(result[0], result[result.Count - 1]) <= 1e-12f) result.RemoveAt(result.Count - 1);
            if (result.Count < 3) return null;
            float area = 0f;
            for (int i = 0; i < result.Count; i++) area += ObjectShapes.Cross(result[i], result[(i + 1) % result.Count]);
            return area > 1e-12f ? result.ToArray() : null;
        }

        /// <summary>The radius of the largest circle inside a convex polygon: how far it can shrink before it is gone (to 0.05 mm).</summary>
        public static float InnerRadiusOf(Vector2[] poly)
        {
            // The polygon lies within its corners' farthest distance from their mean, and so does any circle inside it.
            Vector2 mean = Vector2.Zero;
            foreach (Vector2 v in poly) mean += v / poly.Length;
            float lo = 0f, hi = 0f;
            foreach (Vector2 v in poly) hi = MathF.Max(hi, Vector2.Distance(v, mean));
            while (hi - lo > 0.00005f)
            {
                float mid = 0.5f * (lo + hi);
                if (Offset(poly, -mid) != null) lo = mid;
                else hi = mid;
            }

            return lo;
        }
    }

    /// <summary>
    /// How close a pose's arm comes to one object's own shape (<see cref="SafetyObject.Shape"/>), to <see cref="Step"/>, as
    /// <see cref="GapLadder"/> measures its box (docs/adr/0037 amendment 5, third revision). The arm is its boxes as Isaac's
    /// links are (<see cref="TouchBodies"/>: the model's bodies, but its two CAD finger boxes replaced by the fingers' pads
    /// where the open gripper holds them). Clear of the shape: the most it can grow and stay clear (up to
    /// <see cref="GapLadder.MaxGap"/>). Touching it: minus the most it can shrink and still be touched (up to as much, or its
    /// <see cref="ObjectShape.MaxDepth"/>), a hair below zero at a first touch.
    /// </summary>
    public sealed class ShapeGap
    {
        public const float Step = ObjectShape.Step;

        /// <summary>
        /// The JetRover's finger pads in the twin (robot/jetrover/urdf/jetrover.urdf): <c>left_finger</c>'s pad, a box
        /// 29.996 x 11.332 x 7.39 mm at (0.147724, 0.022089, 0.002667) m, rides the prismatic joint <c>gripper</c> along -Y;
        /// <c>right_finger</c>'s, 29.787 x 11.285 x 7.074 mm at (0.147565, -0.02266, 0.003051) m, <c>gripper_mirror</c> along
        /// +Y. Open (<see cref="SafetyScript.ApproachGripper"/> 0, the sidecar's first <c>fingerTravelByFraction</c>) both
        /// joints are at -<see cref="FingerOpenMeters"/>, so each pad sits that much further out, about 32 mm either side.
        /// </summary>
        public const float FingerOpenMeters = 0.009781f;

        public static readonly CollisionBody LeftPad = new CollisionBody("left_finger.pad", ArmLink.Gripper,
            new Vector3(0.147724f, 0.022089f + FingerOpenMeters, 0.002667f), Quaternion.Identity, new Vector3(0.029996f, 0.011332f, 0.00739f) / 2f);

        public static readonly CollisionBody RightPad = new CollisionBody("right_finger.pad", ArmLink.Gripper,
            new Vector3(0.147565f, -0.02266f - FingerOpenMeters, 0.003051f), Quaternion.Identity, new Vector3(0.029787f, 0.011285f, 0.007074f) / 2f);

        private readonly RobotArmProfile _profile;
        private readonly CollisionBody[] _bodies;
        private readonly ObjectShape _shape;
        private readonly Vector3 _center;
        private readonly Quaternion _toObject;

        public ShapeGap(in RobotArmProfile profile, ArmCollisionModel model, in SafetyObject o)
        {
            _profile = profile;
            _bodies = TouchBodies(model);
            _shape = o.Shape ?? throw new ArgumentException($"object {o.Id} has no shape", nameof(o));
            _center = o.Center;
            _toObject = Quaternion.Conjugate(Quaternion.Normalize(o.Rotation));
        }

        public ObjectShape Shape => _shape;

        /// <summary>
        /// The arm's boxes for a touch: <paramref name="model"/>'s bodies, but the JetRover CAD's two finger boxes
        /// (<c>gripper.1</c>, <c>gripper.2</c>, which the twin does not have) replaced by its two pads, open
        /// (<see cref="LeftPad"/>, <see cref="RightPad"/>). Another model's bodies are as they are.
        /// </summary>
        public static CollisionBody[] TouchBodies(ArmCollisionModel model)
        {
            var bodies = new List<CollisionBody>(model.Bodies.Length + 2);
            bool fingers = false;
            foreach (CollisionBody b in model.Bodies)
            {
                if (b.Link == ArmLink.Gripper && (b.Name == "gripper.1" || b.Name == "gripper.2"))
                {
                    fingers = true;
                    continue;
                }

                bodies.Add(b);
            }

            if (fingers)
            {
                bodies.Add(LeftPad);
                bodies.Add(RightPad);
            }

            return bodies.ToArray();
        }

        /// <summary>The arm's touch boxes at <paramref name="pose"/>, in the object's frame.</summary>
        public void Boxes(in ArmPose pose, Span<ShapeBox> boxes)
        {
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ArmCollision.ComputeFrames(_profile, pose, frames);
            for (int i = 0; i < _bodies.Length; i++)
            {
                CollisionBody body = _bodies[i];
                LinkFrame frame = frames[(int)body.Link];
                Quaternion turn = _toObject * frame.Rotation * body.Rotation;
                Vector3 c = Vector3.Transform(frame.ToBase(body.Center) - _center, _toObject);
                boxes[i] = new ShapeBox(c, Vector3.Transform(Vector3.UnitX, turn), Vector3.Transform(Vector3.UnitY, turn), Vector3.Transform(Vector3.UnitZ, turn),
                    body.HalfExtents);
            }
        }

        public int BoxCount => _bodies.Length;

        /// <summary>Whether the arm at <paramref name="pose"/> touches the shape.</summary>
        public bool Touches(in ArmPose pose)
        {
            Span<ShapeBox> boxes = stackalloc ShapeBox[_bodies.Length];
            Boxes(pose, boxes);
            return _shape.Overlaps(boxes, 0);
        }

        /// <summary>The smallest inner radius of the slabs the arm touches at <paramref name="pose"/> (+inf when it touches none).</summary>
        public float TouchedInnerRadius(in ArmPose pose)
        {
            Span<ShapeBox> boxes = stackalloc ShapeBox[_bodies.Length];
            Boxes(pose, boxes);
            return _shape.TouchedInnerRadius(boxes);
        }

        public float SignedGap(in ArmPose pose)
        {
            Span<ShapeBox> boxes = stackalloc ShapeBox[_bodies.Length];
            Boxes(pose, boxes);
            if (!_shape.Overlaps(boxes, 0))
            {
                int lo = 0, hi = ObjectShape.MaxSteps + 1; // clear at lo, touching at hi (past the ladder: clear all the way)
                while (hi - lo > 1)
                {
                    int mid = (lo + hi) / 2;
                    if (_shape.Overlaps(boxes, mid)) hi = mid;
                    else lo = mid;
                }

                return lo * Step;
            }

            int a = 0, b = _shape.ShrinkSteps + 1; // touching shrunk by a, clear shrunk by b (past the ladder: touching all the way)
            while (b - a > 1)
            {
                int mid = (a + b) / 2;
                if (_shape.Overlaps(boxes, -mid)) a = mid;
                else b = mid;
            }

            return -a * Step - 1e-5f;
        }
    }
}
