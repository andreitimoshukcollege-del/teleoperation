using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>
    /// joint-drive's view of the objects (docs/adr/0036 amendment 1) as they rest (docs/adr/0037 amendment 6): a layout lays some
    /// on a side and stands one by the bench's edge, and physics settles them all before the take; the planner takes each as it is.
    /// </summary>
    public class WorldViewTests
    {
        private static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;
        private static readonly ArmCollisionModel Model = Profile.Collision!;
        private static readonly DriveEnvelope Envelope = DriveEnvelope.FromModel(Model, 15f * MathF.PI / 180f);
        private static readonly WorldScene Bench = LoadBench();

        private static WorldScene LoadBench()
        {
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !System.IO.File.Exists(System.IO.Path.Combine(dir, "core", "Scenes", "jetrover-bench.json"))) dir = System.IO.Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            return WorldView.LoadScene(System.IO.Path.Combine(dir!, "core", "Scenes", "jetrover-bench.json"), out _);
        }

        private static int Index(string name) => Array.FindIndex(Bench.Objects, o => o.Name == name);

        private static SafetyObject At(int i, in RigidTransform p) => WorldView.Bounds(Bench.Objects[i], 1.0, p.X, p.Y, p.Z, p.Qw, p.Qx, p.Qy, p.Qz);

        [Fact]
        public void AnObjectIsTippy_AsItRests_NotAsTheFileStandsIt()
        {
            // Pushed, an object tips before it slides when it is taller than 1.25 times its narrower side as it rests (docs/adr/0037
            // P4). The marker standing is (12 cm on 2 cm); lying on its side it is 2 cm tall and slides; so does the cleaner tin
            // lying (8.8 cm up, 8.8 by 13 cm flat), which stands tippy (13 cm on 8.8).
            int marker = Index("large-marker"), tin = Index("cleaner-tin"), tuna = Index("tuna-fish-can");
            foreach ((int i, int rest, bool tippy) in new[] { (marker, 0, true), (marker, 1, false), (marker, 2, false), (marker, 5, false),
                         (tin, 0, true), (tin, 1, false), (tin, 4, false), (tin, 2, false), (tin, 3, true), (tuna, 0, false), (tuna, 1, true) })
            {
                RigidTransform p = Bench.PoseAt(i, 0.25, 0.0, 0.6, 1.0, rest);
                Assert.True(tippy == At(i, p).Tippy, $"{Bench.Objects[i].Name} on rest {rest}");
                // Settled 15 degrees off its rest (on a facet of its hull, say), it is the same.
                RigidTransform settled = RigidTransform.Rotation(0.6, 0.8, 0, 0.26).Compose(p);
                Assert.Equal(tippy, WorldView.IsTippy(Bench.Objects[i], settled.Qw, settled.Qx, settled.Qy, settled.Qz));
            }

            // The box the planner sees is the object's own, turned as it lies: the tin lying is long along the table, low above it.
            SafetyObject lying = At(tin, Bench.PoseAt(tin, 0.25, 0.0, 0.0, 1.0, 1));
            Assert.Equal(0.0876, lying.Max.Z - lying.Min.Z, 4);
            Assert.Equal(0.1304, lying.Max.Y - lying.Min.Y, 4);
        }

        [Fact]
        public void ARoundObjectOnItsSide_Rolls_SoItIsNoSteadyTarget()
        {
            // The 2026-11-21 twin check: a collide_and_recover touch rolled the mug, lying, 34 degrees, and the take was set aside.
            // Lying on a round side an object rolls when pushed. With the bench's shapes the tin, the soup can and the mug lying
            // are tippy (no steady target for a touch); the foam brick and a parcel lying still slide, and standing nothing changes.
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !System.IO.File.Exists(System.IO.Path.Combine(dir, "core", "Scenes", "jetrover-bench.json"))) dir = System.IO.Path.GetDirectoryName(dir);
            ObjectShapes shapes = ObjectShapes.Load(ObjectShapes.PathFor(System.IO.Path.Combine(dir!, "core", "Scenes", "jetrover-bench.json")));
            var options = new SafetyOptions { KnockMaxKg = 0.7 }; // the bench pool's knockMaxMass
            foreach ((string name, int rest, bool tippy) in new[] { ("cleaner-tin", 1, true), ("tomato-soup-can", 2, true), ("mug", 4, true),
                         ("foam-brick", 1, false), ("parcel-medium", 2, false), ("cleaner-tin", 0, true), ("tuna-fish-can", 0, false) })
            {
                int i = Index(name);
                RigidTransform p = Bench.PoseAt(i, 0.25, 0.0, 0.6, 1.0, rest);
                SafetyObject o = WorldView.Bounds(Bench.Objects[i], 1.0, p.X, p.Y, p.Z, p.Qw, p.Qx, p.Qy, p.Qz, shapes);
                Assert.True(tippy == o.Tippy, $"{name} on rest {rest}");
                Assert.True(tippy != options.Steady(o) || o.Mass > options.KnockMaxKg, $"{name} on rest {rest}: steady");
            }

            Assert.True(shapes.For(Bench.Objects[Index("mug")])!.Round);
            Assert.False(shapes.For(Bench.Objects[Index("parcel-medium")])!.Round);
        }

        [Fact]
        public void AnObjectByTheBenchsBackEdge_CannotBeApproached()
        {
            // docs/adr/0037 amendment 6: the object by the edge is scenery for the cameras, out of the arm's reach on purpose, so
            // no stop, touch or knock aims at it. Mapped on 2026-11-17 (every object that may stand there, x -0.12 to -0.18 m,
            // |y| in 2 cm steps, two yaws): each was approachable out to |y| 0.32-0.34 m and none beyond 0.36, so the regions start
            // at 0.40. Here the worst cases, alone on the table: the widest such object (nearest the arm) upright and lying, and
            // the tallest (highest top), at the spot of an edge rectangle nearest the base, a corner of its footprint turned
            // towards the base. Each failed plan explores every sweep (seconds each), so one rectangle: the robot and its yaw
            // range are mirror-images across x, as the map found.
            var reached = new List<string>();
            var roles = new SafetyOptions { KnockMaxKg = Bench.KnockMaxMass };
            int[] candidates = Enumerable.Range(0, Bench.Objects.Length).Where(i => Bench.HasEdgeRoom(i, 1.0, WorldScene.RestUpright)).ToArray();
            int widest = candidates.OrderByDescending(i => Bench.FootprintRadiusAt(i, 0)).First();
            int tallest = candidates.OrderByDescending(i => Bench.Objects[i].SizeZ).First();
            int lie = Enumerable.Range(1, 5).First(r => Bench.CanLie(widest, r) && Bench.HasEdgeRoom(widest, 1.0, r));
            int k = Enumerable.Range(0, Bench.EdgeRegions.Length / 4).Select(n => 4 * n).First(n => Bench.EdgeRegions[n + 2] > 0);
            foreach ((int i, int rest) in new[] { (widest, 0), (widest, lie), (tallest, 0) })
            {
                Bench.RestExtents(i, rest, out double ex, out double ey, out _);
                double x = Bench.EdgeRegions[k + 1];
                double y = Math.Max(Bench.EdgeRegions[k + 2], Bench.RobotKeepOut[3] + Bench.FootprintRadiusAt(i, rest) + 1e-4);
                double yaw = Math.Atan2(-y, -x) - Math.Atan2(-ey, ex); // its corner (ex/2, -ey/2) towards the base
                Assert.True(Bench.FitsOnTable(i, x, y, 1.0, rest), $"{Bench.Objects[i].Name} at ({x}, {y})");
                SafetyObject o = At(i, Bench.PoseAt(i, x, y, yaw, 1.0, rest));
                foreach (DriveScript script in new[] { DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover })
                {
                    if (DrivePlan.TryBuild(script, Profile, Model.Home, 2.5f, 20, 1, Envelope, out _, out _, new[] { o }, roles))
                        reached.Add(FormattableString.Invariant($"{SafetyScript.Category(script)} {Bench.Objects[i].Name} rest {rest} at ({x:0.000}, {y:0.000}) yaw {yaw:0.00}"));
                }
            }

            Assert.True(reached.Count == 0, string.Join(" | ", reached));
        }
    }
}
