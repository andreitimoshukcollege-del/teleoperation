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
        public void AnObjectByTheBenchsBackEdge_CannotBeApproached()
        {
            // docs/adr/0037 amendment 6: the object by the edge is scenery for the cameras, out of the arm's reach on purpose, so
            // no stop, touch or knock aims at it. Every object that may stand there, at the spots of each edge rectangle nearest
            // the arm (its footprint just clear of the robot's keep-out, the rectangle's side nearest the base), upright and on
            // each side it lies on there, turned two ways, alone on the table: no script plans.
            var reached = new List<string>();
            var roles = new SafetyOptions { KnockMaxKg = Bench.KnockMaxMass };
            int[] candidates = Enumerable.Range(0, Bench.Objects.Length).Where(i => Bench.HasEdgeRoom(i, 1.0, WorldScene.RestUpright)).ToArray();
            // The largest footprint and the tallest of them: the nearest any object comes to the arm, and the highest top.
            int widest = candidates.OrderByDescending(i => Bench.FootprintRadiusAt(i, 0)).First();
            int tallest = candidates.OrderByDescending(i => Bench.Objects[i].SizeZ).First();
            foreach (int i in new[] { widest, tallest }.Distinct())
            {
                foreach (int rest in new[] { 0, 1, 2 }.Where(r => r == 0 || (Bench.CanLie(i, r) && Bench.HasEdgeRoom(i, 1.0, r))))
                {
                    double r0 = Bench.FootprintRadiusAt(i, rest);
                    for (int k = 0; k < Bench.EdgeRegions.Length; k += 4)
                    {
                        double x = Bench.EdgeRegions[k + 1], side = Bench.EdgeRegions[k + 2] > 0 ? 1 : -1;
                        double near = side > 0 ? Bench.EdgeRegions[k + 2] : -Bench.EdgeRegions[k + 3];
                        foreach (double more in new[] { 0.0, 0.04 })
                        foreach (double yaw in new[] { 0.0, Math.PI / 4 })
                        {
                            double y = side * (Math.Max(near, Bench.RobotKeepOut[3] + r0 + 1e-4) + more);
                            Assert.True(Bench.FitsOnTable(i, x, y, 1.0, rest), $"{Bench.Objects[i].Name} at ({x}, {y})");
                            SafetyObject o = At(i, Bench.PoseAt(i, x, y, yaw, 1.0, rest));
                            foreach (DriveScript script in new[] { DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover, DriveScript.CollideWithObstacle })
                            {
                                if (DrivePlan.TryBuild(script, Profile, Model.Home, 2.5f, 20, 1, Envelope, out _, out _, new[] { o }, roles))
                                    reached.Add(FormattableString.Invariant($"{SafetyScript.Category(script)} {Bench.Objects[i].Name} rest {rest} at ({x:0.000}, {y:0.000}) yaw {yaw:0.00}"));
                            }
                        }
                    }
                }
            }

            Assert.True(reached.Count == 0, string.Join(" | ", reached));
        }
    }
}
