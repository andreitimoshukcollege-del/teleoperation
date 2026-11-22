using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit.Abstractions;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>
    /// docs/adr/0037 amendment 6: how an object rests (upright, or lying on a side), and layouts that look like a lab table:
    /// clusters, objects lying on their side, and one by the bench's edge. The shipped JetRover bench is the pool.
    /// </summary>
    public class SceneRestTests
    {
        private static readonly JsonSerializerOptions Options = new JsonSerializerOptions { IncludeFields = true };
        private static readonly WorldScene Pool = LoadBench(withShares: true);
        private readonly ITestOutputHelper _output;

        public SceneRestTests(ITestOutputHelper output) => _output = output;

        /// <summary>The shipped bench, or the same pool with amendment 6's shares at 0 (its layouts as they were before it).</summary>
        private static WorldScene LoadBench(bool withShares)
        {
            SceneDocument d = JsonSerializer.Deserialize<SceneDocument>(File.ReadAllText(SceneDocumentTests.ScenePath("jetrover-bench")), Options)!;
            if (!withShares)
            {
                d.pool!.clusterShare = d.pool.lyingShare = d.pool.edgeShare = 0;
                d.pool.edgeRegions = Array.Empty<double>();
            }

            return d.ToScene();
        }

        private static int Index(string name) => Array.FindIndex(Pool.Objects, o => o.Name == name);

        [Fact]
        public void EveryRest_RoundTripsThroughItsPose_RestingOnTheTable()
        {
            foreach (int i in Enumerable.Range(0, Pool.Objects.Length))
            {
                for (int rest = 0; rest < WorldScene.RestCount; rest++)
                {
                    foreach (double yaw in new[] { -3.0, -1.2, 0.0, 0.4, 2.9, Math.PI })
                    {
                        RigidTransform p = Pool.PoseAt(i, 0.25, -0.05, yaw, 1.0, rest);
                        Assert.Equal(1.0, p.Qw * p.Qw + p.Qx * p.Qx + p.Qy * p.Qy + p.Qz * p.Qz, 12);
                        Assert.Equal(rest, WorldScene.RestOf(p, out double back));
                        Assert.Equal(0.0, Math.IEEERemainder(back - yaw, 2 * Math.PI), 9);
                        Assert.InRange(back, -Math.PI, Math.PI);
                        Pool.RestExtents(i, rest, out _, out _, out double up);
                        Assert.Equal(Pool.Table.Top + up / 2, p.Z, 12); // its lowest face on the table top
                    }
                }

                // Upright is the pose it always had.
                Assert.Equal(Pool.PoseAt(i, 0.3, 0.1, 0.7, 1.0).ToString(), Pool.PoseAt(i, 0.3, 0.1, 0.7, 1.0, WorldScene.RestUpright).ToString());
            }
        }

        [Fact]
        public void ASettledObjectTiltedALittle_KeepsItsRestAndItsHeading()
        {
            // Physics settles an object a little off its rest (a can on its side rolls onto a facet of its hull); the planner takes
            // the pose as it is, and a placement from it snaps to the nearest rest. A tilt about any level axis keeps the heading.
            int tin = Index("cleaner-tin");
            for (int rest = 0; rest < WorldScene.RestCount; rest++)
            {
                RigidTransform p = Pool.PoseAt(tin, 0.25, 0.0, 1.1, 1.0, rest);
                foreach (double axis in new[] { 0.0, 0.8, 2.0, 4.5 })
                foreach (double tilt in new[] { 0.1, 0.4, 0.7 }) // up to 40 degrees
                {
                    RigidTransform t = RigidTransform.Rotation(Math.Cos(axis), Math.Sin(axis), 0, tilt).Compose(p);
                    Assert.Equal(rest, WorldScene.RestOf(t, out double yaw));
                    Assert.Equal(1.1, yaw, 9);
                }
            }
        }

        [Fact]
        public void TheObjectsThatLie_AreTheCansTheTinTheMugTheBrickTheElephantAndTheParcels()
        {
            // A side is drawn when the object lying on it is not tippy (at most 1.25 times as tall as its narrower side) and
            // stands at least 3 cm (a touch needs it): the tuna can and the boxes would stand on an edge, the marker lies 2 cm low.
            string[] lie = Pool.Objects.Where((o, i) => Enumerable.Range(1, 5).Any(r => Pool.CanLie(i, r))).Select(o => o.Name).ToArray();
            Assert.Equal(new[] { "tomato-soup-can", "potted-meat-can", "mug", "foam-brick", "cleaner-tin", "wooden-elephant",
                "parcel-mid", "parcel-large", "parcel-small", "parcel-medium" }, lie);
            Assert.False(Pool.CanLie(Index("tomato-soup-can"), 3)); // upside down is not a side
            Assert.All(Enumerable.Range(0, Pool.Objects.Length).Where(i => !Pool.Objects[i].Movable), i => Assert.DoesNotContain(Enumerable.Range(0, 6), r => Pool.CanLie(i, r)));
            // Lying, a footprint changes: the tin's grows from its diameter to its length.
            int tin = Index("cleaner-tin");
            Assert.Equal(Pool.Objects[tin].FootprintRadius, Pool.FootprintRadiusAt(tin, 0), 12);
            Assert.Equal(0.5 * Math.Sqrt(0.0876 * 0.0876 + 0.1304 * 0.1304), Pool.FootprintRadiusAt(tin, 1), 12);
        }

        [Fact]
        public void TheEdgeRegions_LieOnTheBenchBehindTheArm_AndHoldAllButTheLargestParcels()
        {
            Assert.True(Pool.EdgeRegions.Length >= 4 && Pool.EdgeShare > 0);
            for (int k = 0; k < Pool.EdgeRegions.Length; k += 4)
            {
                // Behind the arm's base and outside the region where the movable objects in reach stand.
                Assert.True(Pool.EdgeRegions[k + 1] < 0 && Pool.EdgeRegions[k + 1] < Pool.RegionXMin);
                Assert.True(Pool.Bench != null && Pool.EdgeRegions[k] >= Pool.Bench!.XMin);
            }

            string[] none = Pool.Objects.Where((o, i) => o.Movable && !Pool.HasEdgeRoom(i, 1.0, WorldScene.RestUpright)).Select(o => o.Name).ToArray();
            Assert.Equal(new[] { "parcel-mid", "parcel-large", "parcel-medium" }, none);
            Assert.True(Pool.HasEdgeRoom(Index("parcel-mid"), 1.0, 2)); // lying on its end, it fits
            Assert.False(Pool.HasEdgeRoom(Array.FindIndex(Pool.Objects, o => !o.Movable), 1.0, 0)); // a fixture never stands there
        }

        [Fact]
        public void Layouts_Cluster_Lie_AndStandOneByTheEdge_AtAboutTheirShares()
        {
            const int Layouts = 2000;
            Stats now = Measure(Pool, Layouts), before = Measure(LoadBench(withShares: false), Layouts);
            _output.WriteLine($"amendment 6 over {Layouts} layouts: {now}");
            _output.WriteLine($"the same pool without its shares:  {before}");

            // Measured 2026-11-17: lying 0.335; clustered 0.756 (0.427 without); one by the edge in 0.498 of layouts; 2.24 movable
            // objects in reach a layout (2.21 without), 2.74 with the edge one.
            // A movable object that can lie does so about as often as the pool says.
            Assert.InRange(now.Lying, Pool.LyingShare - 0.07, Pool.LyingShare + 0.07);
            Assert.Equal(0, before.Lying);
            // About half the layouts have one by the edge (fewer only when no object left over has room there).
            Assert.InRange(now.EdgeLayouts, Pool.EdgeShare - 0.08, Pool.EdgeShare + 0.02);
            Assert.Equal(0, before.EdgeLayouts);
            // Clusters: far more of the objects in reach stand right beside another than chance puts there.
            Assert.True(now.Clustered > before.Clustered + 0.2, $"{now.Clustered:0.00} clustered, {before.Clustered:0.00} without");
            // And they take no room from the reach band.
            Assert.True(now.MeanInReach >= before.MeanInReach - 0.05, $"{now.MeanInReach:0.00} in reach, {before.MeanInReach:0.00} without");
        }

        private sealed class Stats
        {
            public double Lying, EdgeLayouts, Clustered, MeanInReach, MeanOnTable;
            public int[] InReach = new int[8], OnTable = new int[8];

            public override string ToString() =>
                FormattableString.Invariant($"lying {Lying:0.000} of the movable objects that can; clustered {Clustered:0.000} of those in reach beside another; ") +
                FormattableString.Invariant($"by the edge in {EdgeLayouts:0.000} of layouts; movable in reach {MeanInReach:0.00} a layout ({MeanOnTable:0.00} with the edge one), ") +
                $"layouts with 0-5 in reach: {string.Join("/", InReach.Take(6))}, 0-6 on the table with the edge one: {string.Join("/", OnTable.Take(7))}";
        }

        /// <summary>
        /// Over seeds 1 to <paramref name="layouts"/>: the share of movable objects that can lie which lie, of layouts with one by
        /// the edge, of movable objects in reach (in layouts with two or more) standing beside another (footprint circles at most
        /// the clearance plus <see cref="WorldScene.ClusterGap"/> apart), and the movable objects in reach a layout.
        /// </summary>
        private static Stats Measure(WorldScene scene, int layouts)
        {
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            var s = new Stats();
            int canLie = 0, lying = 0, edge = 0, inReachTotal = 0, onTable = 0, clusterCandidates = 0, clustered = 0;
            for (ulong seed = 1; seed <= (ulong)layouts; seed++)
            {
                SceneLayout.Draw(scene, seed, poses, parked, scales);
                var inReach = Enumerable.Range(0, poses.Length).Where(i => !parked[i] && scene.Objects[i].Movable && !scene.InEdgeRegions(poses[i].X, poses[i].Y)).ToArray();
                bool hasEdge = Enumerable.Range(0, poses.Length).Any(i => !parked[i] && scene.Objects[i].Movable && scene.InEdgeRegions(poses[i].X, poses[i].Y));
                edge += hasEdge ? 1 : 0;
                inReachTotal += inReach.Length;
                onTable += inReach.Length + (hasEdge ? 1 : 0);
                s.InReach[inReach.Length]++;
                s.OnTable[inReach.Length + (hasEdge ? 1 : 0)]++;
                foreach (int i in Enumerable.Range(0, poses.Length).Where(i => !parked[i] && scene.Objects[i].Movable))
                {
                    if (!Enumerable.Range(1, 5).Any(r => scene.CanLie(i, r))) continue;
                    canLie++;
                    lying += WorldScene.RestOf(poses[i]) != WorldScene.RestUpright ? 1 : 0;
                }

                if (inReach.Length < 2) continue;
                foreach (int i in inReach)
                {
                    clusterCandidates++;
                    double ri = scene.FootprintRadiusAt(i, WorldScene.RestOf(poses[i]));
                    clustered += inReach.Any(k => k != i && Math.Sqrt(Sq(poses[i].X - poses[k].X) + Sq(poses[i].Y - poses[k].Y))
                        <= ri + scene.FootprintRadiusAt(k, WorldScene.RestOf(poses[k])) + scene.Clearance + WorldScene.ClusterGap + 1e-5) ? 1 : 0;
                }
            }

            s.Lying = canLie > 0 ? (double)lying / canLie : 0;
            s.EdgeLayouts = (double)edge / layouts;
            s.Clustered = clusterCandidates > 0 ? (double)clustered / clusterCandidates : 0;
            s.MeanInReach = (double)inReachTotal / layouts;
            s.MeanOnTable = (double)onTable / layouts;
            return s;
        }

        private static double Sq(double v) => v * v;

        [Fact]
        public void TheRestsGoldenFile_IsWhatThisReaderMakesOfTheBench()
        {
            // robot/isaac's scene.py must stand every object on the same rests, footprints and poses, and accept the same spots
            // (by the edge too), or Isaac refuses what RobotHost draws (OUTSIDE_REGION) or stands it up.
            string expected = RestsGoldenJson();
            string path = Path.Combine(Kinematics.SerialChainKinematicsTests.RepoRoot(), "robot", "isaac", "golden", "scene-rests-jetrover-bench.json");
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, expected, new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-isaac-goldens`");
            GoldenFile.AssertSame(File.ReadAllText(path), expected);
        }

        /// <summary>
        /// The rests of the shipped bench as this reader has them, for robot/isaac/tests/test_scene.py: every object's extents,
        /// footprint radius and whether it may lie, on each rest; poses at rests; the rest and yaw of poses, tilted ones too; and
        /// spots that fit or not, and pairs that overlap or not, each on its rest. Rows of numbers (a flag is 0 or 1).
        /// </summary>
        internal static string RestsGoldenJson()
        {
            WorldScene s = Pool;
            byte[] bytes = File.ReadAllBytes(SceneDocumentTests.ScenePath("jetrover-bench"));
            var rng = new SeededRng(20261117);
            double U(double lo, double hi) => lo + rng.NextDouble() * (hi - lo);
            double minX = Math.Min(s.Table.XMin, s.Bench!.XMin), maxX = Math.Max(s.Table.XMax, s.Bench.XMax);
            var sb = new StringBuilder();
            sb.Append("{\n \"about\": \"docs/adr/0037 amendment 6: the rests of core/Scenes/jetrover-bench.json, written by SceneRestTests (just gen-isaac-goldens). ");
            sb.Append("objects: name, then per rest 0-5 its extents (x, y, up), footprint radius and whether a layout may lay it there. ");
            sb.Append("poses: index, rest, x, y, yaw, then PoseAt's x, y, z, qw, qx, qy, qz. restOf: qw, qx, qy, qz, then RestOf's rest and yaw. ");
            sb.Append("fits: index, rest, x, y, FitsOnTable. overlaps: a, x, y, rest, b, x, y, rest, Overlaps.\",\n");
            sb.Append($" \"scene\": \"{s.Name}\",\n");
            sb.Append($" \"sha256\": \"{Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant()}\",\n");
            sb.Append($" \"shares\": {Row(s.ClusterShare, s.LyingShare, s.EdgeShare)},\n");
            sb.Append($" \"edgeRegions\": {Row(s.EdgeRegions)},\n");
            sb.Append(" \"objects\": [\n");
            for (int i = 0; i < s.Objects.Length; i++)
            {
                var rests = Enumerable.Range(0, WorldScene.RestCount).Select(r =>
                {
                    s.RestExtents(i, r, out double ex, out double ey, out double ez);
                    return Row(ex, ey, ez, s.FootprintRadiusAt(i, r), s.CanLie(i, r) ? 1 : 0);
                });
                sb.Append($"  {{\"name\": \"{s.Objects[i].Name}\", \"rests\": [{string.Join(", ", rests)}]}}{(i < s.Objects.Length - 1 ? "," : "")}\n");
            }

            sb.Append(" ],\n");
            var poses = new System.Collections.Generic.List<string>();
            var restOf = new System.Collections.Generic.List<string>();
            for (int i = 0; i < s.Objects.Length; i++)
            {
                for (int rest = 0; rest < (s.Objects[i].Movable ? WorldScene.RestCount : 1); rest++)
                {
                    foreach (double yaw in new[] { -2.5, 1.3 })
                    {
                        double x = U(0.15, 0.40), y = U(-0.2, 0.2);
                        RigidTransform p = s.PoseAt(i, x, y, yaw, 1.0, rest);
                        poses.Add(Row(i, rest, x, y, yaw, p.X, p.Y, p.Z, p.Qw, p.Qx, p.Qy, p.Qz));
                        // Tilted about a level axis by up to 40 degrees, as a settled object may rest: the same rest and heading.
                        double axis = U(-Math.PI, Math.PI);
                        RigidTransform t = RigidTransform.Rotation(Math.Cos(axis), Math.Sin(axis), 0, U(0, 0.7)).Compose(p);
                        foreach (RigidTransform q in new[] { p, t })
                        {
                            int r = WorldScene.RestOf(q, out double back);
                            restOf.Add(Row(q.Qw, q.Qx, q.Qy, q.Qz, r, back));
                        }
                    }
                }
            }

            var fits = new System.Collections.Generic.List<string>();
            for (int i = 0; i < s.Objects.Length; i++)
            {
                foreach (int rest in s.Objects[i].Movable ? new[] { 0, 1, 2, 4 } : new[] { 0 })
                {
                    for (int n = 0; n < 8; n++)
                    {
                        // Half by an edge region (5 cm beyond it), half anywhere over the bench and the table.
                        int k = 4 * (int)(rng.NextDouble() * (s.EdgeRegions.Length / 4));
                        double x = n % 2 == 0 ? U(s.EdgeRegions[k] - 0.05, s.EdgeRegions[k + 1] + 0.05) : U(minX, maxX);
                        double y = n % 2 == 0 ? U(s.EdgeRegions[k + 2] - 0.05, s.EdgeRegions[k + 3] + 0.05) : U(s.Table.YMin, s.Table.YMax);
                        fits.Add(Row(i, rest, x, y, s.FitsOnTable(i, x, y, 1.0, rest) ? 1 : 0));
                    }
                }
            }

            var overlaps = new System.Collections.Generic.List<string>();
            int[] movable = Enumerable.Range(0, s.Objects.Length).Where(i => s.Objects[i].Movable).ToArray();
            for (int n = 0; n < 120; n++)
            {
                int a = movable[(int)(rng.NextDouble() * movable.Length)], b = movable[(int)(rng.NextDouble() * movable.Length)];
                int ra = (int)(rng.NextDouble() * WorldScene.RestCount), rb = (int)(rng.NextDouble() * WorldScene.RestCount);
                double ax = U(0.15, 0.40), ay = U(-0.2, 0.2), bearing = U(-Math.PI, Math.PI);
                double d = (s.FootprintRadiusAt(a, ra) + s.FootprintRadiusAt(b, rb) + s.Clearance) * U(0.9, 1.1);
                double bx = ax + d * Math.Cos(bearing), by = ay + d * Math.Sin(bearing);
                overlaps.Add(Row(a, ax, ay, ra, b, bx, by, rb, s.Overlaps(a, ax, ay, b, bx, by, 1.0, 1.0, ra, rb) ? 1 : 0));
            }

            void Rows(string name, System.Collections.Generic.List<string> rows, bool last)
            {
                sb.Append($" \"{name}\": [\n  ").Append(string.Join(",\n  ", rows)).Append(last ? "\n ]\n" : "\n ],\n");
            }

            Rows("poses", poses, false);
            Rows("restOf", restOf, false);
            Rows("fits", fits, false);
            Rows("overlaps", overlaps, true);
            sb.Append("}\n");
            return sb.ToString();
        }

        private static string Row(params double[] v) => "[" + string.Join(", ", v.Select(x => x.ToString("R", CultureInfo.InvariantCulture))) + "]";
    }
}
