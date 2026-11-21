using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using Teleop.Core.Types;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>The scene's objects as they really are (docs/adr/0037 amendment 5, third revision): the shapes file, the shape test, and a touch planned against a shape.</summary>
    public class ObjectShapeTests
    {
        private static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;
        private static readonly ArmCollisionModel Model = Profile.Collision!;
        private static readonly DriveEnvelope Envelope = DriveEnvelope.FromModel(Model, 15f * MathF.PI / 180f);

        /// <summary>A file of the repository, found from the test's own folder upwards.</summary>
        internal static string RepoFile(params string[] parts)
        {
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !File.Exists(Path.Combine(new[] { dir }.Concat(parts).ToArray()))) dir = Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            return Path.Combine(new[] { dir! }.Concat(parts).ToArray());
        }

        /// <summary>The bench scene and the shapes loaded with it.</summary>
        internal static WorldScene Bench(out ObjectShapes shapes)
        {
            WorldScene scene = WorldView.LoadScene(RepoFile("core", "Scenes", "jetrover-bench.json"), out _);
            ObjectShapes? s = WorldView.ShapesOf(scene);
            Assert.True(s != null, "core/Scenes/jetrover-bench.shapes.json did not load with the scene");
            shapes = s!;
            return scene;
        }

        internal static Vector3 Half(SceneObject o, double scale = 1.0) =>
            new Vector3((float)(o.SizeX * scale / 2), (float)(o.SizeY * scale / 2), (float)(o.SizeZ * scale / 2));

        [Fact]
        public void TheShapesFile_CoversEveryMeshObjectOfTheScene()
        {
            WorldScene scene = Bench(out ObjectShapes shapes);
            int meshes = 0;
            foreach (SceneObject o in scene.Objects)
            {
                if (o.Shape != SceneShape.Mesh) continue;
                meshes++;
                AssetShape? a = shapes.For(o);
                Assert.True(a != null, $"{o.Name}: no shape for {o.Asset}");
                // The object is its asset scaled the same way every way (world.py's mesh_factor), so fractions of the asset's box
                // are fractions of the object's.
                double kx = o.SizeX / a!.Size.X, ky = o.SizeY / a.Size.Y, kz = o.SizeZ / a.Size.Z;
                Assert.True(Math.Abs(kx / kz - 1) < 0.01 && Math.Abs(ky / kz - 1) < 0.01, $"{o.Name}: {o.Asset} scaled by {kx:0.000}, {ky:0.000}, {kz:0.000}");
                // Its slabs stand from the bottom of its box to the top, and it can be touched. Raised clutter (docs/adr/0037 "Raised
                // clutter") lies flat, 1.4-2.7 cm tall: a touch, which meets an object at least 10 mm below its top and the arm never
                // closer than 10 mm to the table, has little or none of it to go into. It is an obstacle first.
                Assert.InRange(a.Slabs.Count, 8, 48);
                Assert.InRange(a.Slabs[0].Z0, -0.501f, -0.499f);
                Assert.InRange(a.Slabs[a.Slabs.Count - 1].Z1, 0.49f, 0.501f);
                ObjectShape shape = a.At(Half(o));
                if (!o.Clutter) Assert.True(shape.MaxDepth > 0.006f, $"{o.Name}: the deepest it can be entered is {shape.MaxDepth * 1000:0.0} mm");
                Assert.Same(shape, a.At(Half(o)));
            }

            // 20 movable objects and fixtures, and 4 pieces of clutter that are scans (the phone and the pen are boxes).
            Assert.Equal(28, meshes);
        }

        [Fact]
        public void AShapesFile_ThatIsNotOne_IsRefused()
        {
            static ObjectShapes Parse(string json) => ObjectShapes.Parse(Encoding.UTF8.GetBytes(json));
            const string Square = "[[-0.5,-0.5],[0.5,-0.5],[0.5,0.5],[-0.5,0.5]]";
            string Doc(string outline, string z = "\"z0\":-0.5,\"z1\":0.5") =>
                "{\"format\":\"teleop-object-shapes\",\"version\":1,\"assets\":{\"a/b\":{\"size\":[0.1,0.1,0.1],\"slabs\":[{" + z + ",\"outline\":" + outline + "}]}}}";
            Assert.Single(Parse(Doc(Square)).Assets);
            Assert.Throws<InvalidDataException>(() => Parse(Doc("[[-0.5,-0.5],[-0.5,0.5],[0.5,0.5],[0.5,-0.5]]"))); // clockwise
            Assert.Throws<InvalidDataException>(() => Parse(Doc("[[-0.5,-0.5],[0.5,-0.5],[0.0,-0.4],[-0.5,0.5]]"))); // not convex
            Assert.Throws<InvalidDataException>(() => Parse(Doc(Square, "\"z0\":0.5,\"z1\":-0.5")));                  // upside down
            Assert.Throws<InvalidDataException>(() => Parse(Doc(Square).Replace("teleop-object-shapes", "teleop-scene")));
        }

        /// <summary>
        /// A point's depth inside a shape, by brute force: the least of its distances to its slab's sides (each side's line,
        /// as the shape is offset edge by edge) and to the bottom; negative outside; nothing above the touch's top (which no
        /// offset moves). Its slab is the one whose height holds it (the bottom one below the bottom); at a boundary, the
        /// better of two.
        /// </summary>
        internal static float Depth(ObjectShape s, Vector3 p)
        {
            float best = float.NegativeInfinity;
            if (p.Z > s.TouchTop) return best;
            for (int k = 0; k < s.SlabCount; k++)
            {
                if ((p.Z < s.Z0(k) && k > 0) || p.Z > s.Z1(k)) continue;
                IReadOnlyList<Vector2> poly = s.Outline(k);
                float lateral = float.PositiveInfinity;
                for (int i = 0; i < poly.Count; i++)
                {
                    Vector2 e = poly[(i + 1) % poly.Count] - poly[i];
                    var n = Vector2.Normalize(new Vector2(e.Y, -e.X));
                    lateral = MathF.Min(lateral, Vector2.Dot(n, poly[i] - new Vector2(p.X, p.Y)));
                }

                best = MathF.Max(best, MathF.Min(lateral, p.Z - s.Bottom));
            }

            return best;
        }

        /// <summary>The deepest of a grid of points through a box (every <paramref name="spacing"/>, and at every height a slab starts in it, when it is upright).</summary>
        internal static float SampledDepth(ObjectShape s, in ShapeBox b, float spacing)
        {
            static float[] Grid(float half, float spacing)
            {
                int n = Math.Max(1, (int)MathF.Ceiling(2 * half / spacing));
                var g = new float[n + 1];
                for (int i = 0; i <= n; i++) g[i] = -half + 2 * half * i / n;
                return g;
            }

            float[] us = Grid(b.Half.X, spacing), vs = Grid(b.Half.Y, spacing);
            var ws = Grid(b.Half.Z, spacing).ToList();
            if (MathF.Abs(b.AxisZ.Z) > 0.99999f)
            {
                // Upright: a slab's start inside the box is a height of its own, just above and just below, and so is the touch's top.
                for (int k = 0; k <= s.SlabCount; k++)
                foreach (float z in k < s.SlabCount ? new[] { s.Z0(k) - 1e-5f, s.Z0(k) + 1e-5f } : new[] { s.TouchTop - 1e-5f })
                {
                    float w = (z - b.Center.Z) * MathF.Sign(b.AxisZ.Z);
                    if (MathF.Abs(w) <= b.Half.Z) ws.Add(w);
                }
            }

            float best = float.NegativeInfinity;
            foreach (float u in us)
            foreach (float v in vs)
            foreach (float w in ws)
            {
                best = MathF.Max(best, Depth(s, b.Center + u * b.AxisX + v * b.AxisY + w * b.AxisZ));
            }

            return best;
        }

        [Fact]
        public void TheShapeTest_AgreesWithPointsSampledThroughTheBox()
        {
            // The planner measures a touch with the separating-axis test against offset slabs (ObjectShape.Overlaps); this checks
            // it by brute force, boxes near each shape's surface against a grid of points through them. Upright boxes (turned
            // about z only) both ways: the shape offset by the sampled depth is touched, and offset past it (by the grid's
            // error) it is not. Boxes turned any way one way: the grid's depth is a depth the box surely reaches.
            WorldScene scene = Bench(out ObjectShapes shapes);
            const float Spacing = 0.0006f, GridError = 0.0008f;
            var rng = new SeededRng(17);
            float Uniform(float lo, float hi) => lo + (hi - lo) * (float)rng.NextDouble();
            int both = 0, deep = 0;
            foreach (string name in new[] { "vintage-electric-kettle", "garden-gnome", "large-marker", "wooden-elephant", "marble-bust-01" })
            {
                SceneObject o = scene.Objects.First(x => x.Name == name);
                ObjectShape shape = shapes.For(o)!.At(Half(o));
                for (int i = 0; i < 24; i++)
                {
                    // A point on a slab's side, at a height in it, moved out (or in) along the side's normal.
                    int k = (int)(rng.NextDouble() * shape.SlabCount) % shape.SlabCount;
                    IReadOnlyList<Vector2> poly = shape.Outline(k);
                    int j = (int)(rng.NextDouble() * poly.Count) % poly.Count;
                    Vector2 a = poly[j], e = poly[(j + 1) % poly.Count] - a;
                    var n = Vector2.Normalize(new Vector2(e.Y, -e.X));
                    Vector2 at = a + e * Uniform(0f, 1f) + n * Uniform(-0.012f, 0.008f);
                    var c = new Vector3(at, Uniform(shape.Z0(k), shape.Z1(k)));
                    var half = new Vector3(Uniform(0.002f, 0.012f), Uniform(0.002f, 0.012f), Uniform(0.002f, 0.012f));
                    bool upright = i % 3 != 2;
                    Quaternion turn = upright
                        ? Quaternion.CreateFromAxisAngle(Vector3.UnitZ, Uniform(-MathF.PI, MathF.PI))
                        : Quaternion.Normalize(new Quaternion(Uniform(-1, 1), Uniform(-1, 1), Uniform(-1, 1), Uniform(-1, 1)));
                    var box = new ShapeBox(c, Vector3.Transform(Vector3.UnitX, turn), Vector3.Transform(Vector3.UnitY, turn), Vector3.Transform(Vector3.UnitZ, turn), half);
                    ShapeBox[] boxes = { box };
                    float sampled = SampledDepth(shape, box, Spacing);
                    string what = $"{name}: a box at ({c.X * 1000:0.0}, {c.Y * 1000:0.0}, {c.Z * 1000:0.0}) mm, sampled {sampled * 1000:0.00} mm deep";

                    // Offset so that the sampled depth is just inside: touched.
                    int reach = (int)MathF.Floor(sampled / ObjectShape.Step); // shrunk by up to this many steps, still touched
                    if (-reach >= -shape.ShrinkSteps && -reach <= ObjectShape.MaxSteps)
                    {
                        Assert.True(shape.Overlaps(boxes, -reach), $"{what}: not touched shrunk by {reach * ObjectShape.Step * 1000:0.0} mm");
                        deep++;
                    }

                    if (!upright) continue;
                    int past = (int)MathF.Ceiling((sampled + GridError) / ObjectShape.Step) + 1; // shrunk this far, surely clear
                    if (-past >= -shape.ShrinkSteps && -past <= ObjectShape.MaxSteps)
                    {
                        Assert.False(shape.Overlaps(boxes, -past), $"{what}: touched shrunk by {past * ObjectShape.Step * 1000:0.0} mm");
                        both++;
                    }
                }
            }

            Assert.True(both >= 60 && deep >= 90, $"{both} boxes checked both ways, {deep} one way");
        }

        [Fact]
        public void TheOffsetPolygons_ShrinkToTheirInnerRadius()
        {
            // A 40 x 20 mm rectangle: its inner radius is 10 mm; grown by 2 mm it is 44 x 24.
            Vector2[] rect = { new Vector2(-0.02f, -0.01f), new Vector2(0.02f, -0.01f), new Vector2(0.02f, 0.01f), new Vector2(-0.02f, 0.01f) };
            Assert.InRange(ObjectShape.InnerRadiusOf(rect), 0.0099f, 0.0100f);
            Vector2[] grown = ObjectShape.Offset(rect, 0.002f)!;
            Assert.Equal(4, grown.Length);
            Assert.InRange(grown.Max(v => v.X), 0.0219f, 0.0221f);
            Assert.InRange(grown.Max(v => v.Y), 0.0119f, 0.0121f);
            Assert.NotNull(ObjectShape.Offset(rect, -0.0099f));
            Assert.Null(ObjectShape.Offset(rect, -0.0101f));
        }

        [Fact]
        public void TheTouchBodies_AreTheTwinsLinks_WithItsFingersPads()
        {
            // The model's two CAD finger boxes are not in the twin; its pads are: open, about 32 mm either side of the gripper's
            // axis, closed about 6 mm. Each finger's travel is the twin's own table.
            CollisionBody[] bodies = ShapeGap.TouchBodies(Model);
            Assert.Equal(Model.Bodies.Length, bodies.Length);
            Assert.DoesNotContain(bodies, b => b.Name == "gripper.1" || b.Name == "gripper.2");
            CollisionBody left = bodies.Single(b => b.Name == "left_finger.pad"), right = bodies.Single(b => b.Name == "right_finger.pad");
            Assert.InRange(left.Center.Y, 0.0318f, 0.0319f);
            Assert.InRange(right.Center.Y, -0.0325f, -0.0324f);
            CollisionBody closed = ShapeGap.TouchBodies(Model, 1f).Single(b => b.Name == "left_finger.pad");
            Assert.InRange(closed.Center.Y, 0.0058f, 0.0059f);

            using var sidecar = System.Text.Json.JsonDocument.Parse(File.ReadAllBytes(RepoFile("robot", "jetrover", "urdf", "jetrover.sim.json")));
            System.Text.Json.JsonElement gripper = sidecar.RootElement.GetProperty("motors").EnumerateArray().Single(m => m.GetProperty("joint").GetString() == "gripper");
            float[] table = gripper.GetProperty("fingerTravelByFraction").EnumerateArray().Select(v => v.GetSingle()).ToArray();
            Assert.Equal(table, ShapeGap.FingerTravelByFraction);
            Assert.Equal(-ShapeGap.FingerTravelByFraction[0], gripper.GetProperty("fingerOpenMeters").GetSingle());
        }

        /// <summary>
        /// The deepest <paramref name="plan"/> goes into <paramref name="o"/>'s shape (by <see cref="ShapeGap"/>, the pads where
        /// the plan's gripper holds them, sampled every 10 ms), and when.
        /// </summary>
        internal static float ShapeDepth(DrivePlan plan, in SafetyObject o, out double when)
        {
            var gap = new ShapeGap(Profile, Model, o);
            float deepest = float.NegativeInfinity;
            when = double.NaN;
            for (double t = 0; t <= plan.TotalSeconds; t += 0.01)
            {
                float d = -gap.SignedGap(plan.Sample(t), plan.GripperAt(t));
                if (d > deepest)
                {
                    deepest = d;
                    when = t;
                }
            }

            return deepest;
        }

        [Fact]
        public void ARoundObjectInsideItsBox_IsTouchedAbout9mmIntoItsShape()
        {
            // The cleaner tin (a can 88 mm across) turned 45 degrees in front of the robot: its box's corner faces the arm, 18 mm
            // outside the can. A touch planned 20 mm into the box meets the can at a few millimetres; planned against the can
            // itself, it goes ShapeTouchDepthMeters in (2026-11-16: the round fixtures and the elephant were never touched).
            WorldScene scene = Bench(out ObjectShapes shapes);
            SceneObject tin = scene.Objects.First(x => x.Name == "cleaner-tin");
            var turn = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 4f);
            float z = Model.FloorZ + (float)tin.SizeZ / 2f;
            var roles = new SafetyOptions { KnockMaxKg = scene.KnockMaxMass };
            int planned = 0;
            var report = new List<string>();
            foreach ((float x, float y) in new[] { (0.24f, 0.04f), (0.22f, -0.06f), (0.27f, 0.0f) })
            {
                SafetyObject Tin(bool shaped) => WorldView.Bounds(tin, 1.0, x, y, z, turn.W, turn.X, turn.Y, turn.Z, shaped ? shapes : null);
                SafetyObject real = Tin(true);
                Assert.NotNull(real.Shape);
                for (ulong seed = 1; seed <= 2; seed++)
                {
                    if (!DrivePlan.TryBuild(DriveScript.CollideAndRecover, Profile, Model.Home, 2.5f, 20, seed, Envelope, out DrivePlan? plan, out _, new[] { real }, roles))
                        continue;
                    planned++;
                    float depth = ShapeDepth(plan!, real, out double when);
                    report.Add($"({x}, {y}) seed {seed}: {depth * 1000:0.0} mm at {when:0.00} s, gripper {plan!.GripperAt(when):0.00}");
                    Assert.InRange(depth, SafetyScript.ShapeTouchDepthMeters - 0.0015f, SafetyScript.ShapeTouchDepthMeters + 0.0015f);
                }
            }

            Assert.True(planned >= 3, $"{planned} of 6 plans: {string.Join("; ", report)}");
        }
    }
}
