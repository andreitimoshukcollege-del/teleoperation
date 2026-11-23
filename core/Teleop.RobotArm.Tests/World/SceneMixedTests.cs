using System;
using System.Collections.Generic;
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
    /// docs/adr/0038 decision 3: a mixed pool (<c>core/Scenes/jetrover-mixed.json</c> and its held-out family). Each layout stands 0-3
    /// obstacles centred uniformly by area over the arm's reachable band, some raised on a stand, some a plank on its supports, and
    /// 2-8 distractors beyond every arm pose's reach; and the bench, which is no mixed pool, draws exactly what it drew before.
    /// </summary>
    public class SceneMixedTests
    {
        private static readonly JsonSerializerOptions Options = new JsonSerializerOptions { IncludeFields = true };
        internal static readonly WorldScene Mixed = Load("jetrover-mixed");
        internal static readonly WorldScene Holdout = Load("jetrover-mixed-holdout");
        private readonly ITestOutputHelper _output;

        /// <summary>
        /// The farthest any collider of the arm comes from the base's yaw axis, over every pose robot/isaac/reach_envelope.py keeps
        /// (Core's floor rule, self-collision not checked), m, measured 2026-11-22: 0.3644 at 1 cm above the table, 0.4061 at 10 cm,
        /// 0.4270 at any height.
        /// </summary>
        internal const double ArmReach = 0.4270, ArmReachAt1cm = 0.3644;

        public SceneMixedTests(ITestOutputHelper output) => _output = output;

        internal static WorldScene Load(string name) =>
            JsonSerializer.Deserialize<SceneDocument>(File.ReadAllText(SceneDocumentTests.ScenePath(name)), Options)!.ToScene();

        private static (RigidTransform[] Poses, bool[] Parked, double[] Scales, int Missed) Draw(WorldScene scene, ulong seed)
        {
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            int missed = SceneLayout.Draw(scene, seed, poses, parked, scales);
            return (poses, parked, scales, missed);
        }

        private static double YawOf(in RigidTransform p)
        {
            WorldScene.RestOf(p, out double yaw);
            return yaw;
        }

        public static IEnumerable<object[]> Scenes() => new[] { new object[] { "jetrover-mixed" }, new object[] { "jetrover-mixed-holdout" } };

        [Theory]
        [MemberData(nameof(Scenes))]
        public void TheMixedScenes_AreMixedPools_EveryObjectWithARole_AndEverythingWithARealSurface(string name)
        {
            WorldScene s = Load(name);
            Assert.True(s.Mixed && s.IsPool && s.DrawsSizes);
            Assert.Equal("jetrover", s.RobotProfile);
            Assert.Equal((0, 3), (s.ObstacleMin, s.ObstacleMax));
            Assert.Equal((2, 6), (s.DistractorMin, s.DistractorMax));
            Assert.All(s.Objects, o => Assert.NotEqual(SceneRole.None, o.Role));
            // At every yaw the arm reaches (its base turns 120 degrees either way), and to the reach of a low touch.
            Assert.Equal(2.0944, s.ReachBearing, 4);
            Assert.True(s.ReachMax < ArmReachAt1cm - 0.02, "an obstacle's near side within reach of a touch 1 cm up");
            // Distractors beyond the farthest any collider of the arm comes, at any height.
            Assert.True(s.DistractorReach > ArmReach + 0.02, $"distractorReach {s.DistractorReach}");
            Assert.Contains(s.Objects, o => o.Role == SceneRole.Stand);
            Assert.Contains(s.Objects, o => o.Role == SceneRole.Plank && o.SupportIds.Length == 2); // a bridge
            Assert.Contains(s.Objects, o => o.Role == SceneRole.Plank && o.SupportIds.Length == 1); // a cantilever

            // Nothing without a real surface (docs/adr/0037 amendment 5): every simple shape wears looks by name (the renderer's
            // materials.objects, each a real texture); every scan has its shape (the shapes file beside the scene).
            using JsonDocument file = JsonDocument.Parse(File.ReadAllText(SceneDocumentTests.ScenePath(name)));
            JsonElement worn = file.RootElement.GetProperty("materials").GetProperty("objects");
            JsonElement looks = file.RootElement.GetProperty("materials").GetProperty("looks");
            foreach (SceneObject o in s.Objects.Where(o => o.Shape != SceneShape.Mesh))
            {
                Assert.True(worn.TryGetProperty(o.Name, out JsonElement ids) && ids.GetArrayLength() > 0, $"{name}: {o.Name} wears no looks");
                Assert.All(ids.EnumerateArray(), id => Assert.True(looks.GetProperty(id.GetString()!).GetProperty("texture").GetString()!.Length > 0));
            }

            string shapesPath = Path.Combine(Path.GetDirectoryName(SceneDocumentTests.ScenePath(name))!, name + ".shapes.json");
            using JsonDocument shapes = JsonDocument.Parse(File.ReadAllText(shapesPath));
            JsonElement assets = shapes.RootElement.GetProperty("assets");
            string[] surveyed = assets.EnumerateObject().Select(a => a.Name).ToArray();
            Assert.All(s.Objects.Where(o => o.Shape == SceneShape.Mesh), o => Assert.Contains(o.Asset, surveyed));
            Assert.Equal(s.Objects.Where(o => o.Shape == SceneShape.Mesh).Select(o => o.Asset).Distinct().Count(), assets.EnumerateObject().Count());

            // Sizes are drawn from each object's own steps: boxes and cylinders only, fixed ones too; scans keep their size.
            Assert.All(s.Objects.Where(o => o.HasSteps), o => Assert.True(o.Shape == SceneShape.Box || o.Shape == SceneShape.Cylinder, o.Name));
            Assert.All(s.Objects.Where(o => o.Shape == SceneShape.Mesh), o => Assert.False(s.Resizable(Array.IndexOf(s.Objects, o)), o.Name));
        }

        [Fact]
        public void TheHeldOutFamily_SharesNoAssetNoLookAndNoKindOfSimpleShapeWithTheTrainFamily()
        {
            // docs/adr/0038 item 22: a test session on layouts of objects the training data never had.
            string[] Assets(WorldScene s) => s.Objects.Where(o => o.Shape == SceneShape.Mesh).Select(o => o.Asset).Distinct().ToArray();
            Assert.Empty(Assets(Mixed).Intersect(Assets(Holdout)));
            Assert.Empty(Mixed.Objects.Select(o => o.Name).Intersect(Holdout.Objects.Select(o => o.Name)));
            Assert.Empty(Mixed.Objects.Select(o => o.Id).Intersect(Holdout.Objects.Select(o => o.Id)));
            // The looks the simple shapes wear: textures of their own.
            HashSet<string> Textures(string name)
            {
                using JsonDocument file = JsonDocument.Parse(File.ReadAllText(SceneDocumentTests.ScenePath(name)));
                JsonElement m = file.RootElement.GetProperty("materials");
                WorldScene s = Load(name);
                return s.Objects.Where(o => o.Shape != SceneShape.Mesh && !o.Clutter)
                    .SelectMany(o => m.GetProperty("objects").GetProperty(o.Name).EnumerateArray().Select(id => m.GetProperty("looks").GetProperty(id.GetString()!).GetProperty("texture").GetString()!))
                    .ToHashSet();
            }

            Assert.Empty(Textures("jetrover-mixed").Intersect(Textures("jetrover-mixed-holdout")));
            // Heights the train family never draws: its overhangs 4.1-19.5 cm up, the held-out bench's 2.6-4 cm and its cantilever 19-24 cm;
            // its stands 3.8-11.7 cm, the held-out ones 12-15 cm and a 3.5-5.5 cm riser; its tall thin things round, the held-out post square.
            (double Lo, double Hi) Heights(WorldScene s, SceneRole role) => (
                s.Objects.Where(o => o.Role == role).Min(o => o.SizeZ * WorldScene.ScaleOfStep(o.StepMin)),
                s.Objects.Where(o => o.Role == role).Max(o => o.SizeZ * WorldScene.ScaleOfStep(o.StepMax)));
            Assert.Equal(0.0256, Heights(Holdout, SceneRole.Support).Lo, 4);
            Assert.True(Heights(Mixed, SceneRole.Support).Lo > Heights(Holdout, SceneRole.Support).Lo);
            Assert.True(Heights(Mixed, SceneRole.Support).Hi < Heights(Holdout, SceneRole.Support).Hi);
            Assert.True(Holdout.Objects.Any(o => o.Role == SceneRole.Stand && o.SizeZ * WorldScene.ScaleOfStep(o.StepMin) > Heights(Mixed, SceneRole.Stand).Hi));
            Assert.All(Mixed.Objects.Where(o => o.Role == SceneRole.Obstacle && o.SizeZ >= 0.15), o => Assert.Equal(SceneShape.Cylinder, o.Shape));
            Assert.Contains(Holdout.Objects, o => o.Role == SceneRole.Obstacle && o.Shape == SceneShape.Box && o.SizeZ >= 0.3);
        }

        [Theory]
        [MemberData(nameof(Scenes))]
        public void EveryLayout_StandsItsObstaclesInReach_ItsDistractorsBeyondIt_AndNothingIntoAnything(string name)
        {
            WorldScene s = Load(name);
            for (ulong seed = 1; seed <= 400; seed++)
            {
                (RigidTransform[] p, bool[] parked, double[] scales, int missed) = Draw(s, seed);
                int units = 0, distractors = 0;
                for (int i = 0; i < p.Length; i++)
                {
                    SceneObject o = s.Objects[i];
                    if (parked[i])
                    {
                        Assert.Equal(s.ParkedPose(i).ToString(), p[i].ToString());
                        continue;
                    }

                    int rest = WorldScene.RestOf(p[i], out double yaw);
                    Assert.True(rest == WorldScene.RestUpright || s.CanLie(i, rest), $"{name} seed {seed}: {o.Name} on rest {rest}");
                    Assert.True(o.HasSteps ? s.InSteps(i, scales[i]) : scales[i] == 1.0, $"{name} seed {seed}: {o.Name} at {scales[i]}");
                    Assert.True(s.FitsOnTable(i, p[i].X, p[i].Y, yaw, scales[i], rest), $"{name} seed {seed}: {o.Name} does not fit");
                    for (int k = 0; k < i; k++)
                    {
                        if (parked[k]) continue;
                        int restK = WorldScene.RestOf(p[k], out double yawK);
                        Assert.False(s.OverlapsAtHeights(i, p[i].X, p[i].Y, p[i].Z, yaw, k, p[k].X, p[k].Y, p[k].Z, yawK, scales[i], scales[k], rest, restK),
                            $"{name} seed {seed}: {o.Name} into {s.Objects[k].Name}");
                    }

                    double r = Math.Sqrt(p[i].X * p[i].X + p[i].Y * p[i].Y), bearing = Math.Abs(Math.Atan2(p[i].Y, p[i].X));
                    bool raised = s.IsElevated(i, p[i].Z, scales[i], rest);
                    switch (o.Role)
                    {
                        case SceneRole.Distractor:
                            distractors++;
                            Assert.False(raised);
                            Assert.True(s.InDistractorRegions(p[i].X, p[i].Y));
                            break;
                        case SceneRole.Obstacle when raised:
                            // On a stand that holds it, the stand where an obstacle stands.
                            units++;
                            Assert.True(o.Movable, $"{name} seed {seed}: fixed {o.Name} raised");
                            Assert.Contains(Enumerable.Range(0, p.Length), k => !parked[k] && s.HoldsOnTop(k, p[k].X, p[k].Y, p[k].Z, YawOf(p[k]),
                                scales[k], WorldScene.RestUpright, i, p[i].X, p[i].Y, p[i].Z, scales[i], rest));
                            break;
                        case SceneRole.Obstacle:
                            units++;
                            Assert.InRange(r, s.ReachMin - 1e-9, s.ReachMax + 1e-9);
                            Assert.InRange(bearing, 0, s.ReachBearing + 1e-9);
                            Assert.Equal(s.RestingZ(i, scales[i], rest), p[i].Z, 9);
                            break;
                        case SceneRole.Stand:
                            Assert.InRange(r, s.ReachMin - 1e-9, s.ReachMax + 1e-9);
                            Assert.InRange(bearing, 0, s.ReachBearing + 1e-9);
                            Assert.Equal(s.RestingZ(i, scales[i], rest), p[i].Z, 9);
                            Assert.Contains(Enumerable.Range(0, p.Length), k => !parked[k] && s.HoldsOnTop(i, p[i].X, p[i].Y, p[i].Z, yaw, scales[i], rest, k, p[k].X, p[k].Y, p[k].Z,
                                scales[k], WorldScene.RestOf(p[k])));
                            break;
                        case SceneRole.Plank:
                            // Centred as an obstacle is, its underside on its supports' tops, they under its ends, the gripper's room between.
                            units++;
                            Assert.InRange(r, s.ReachMin - 1e-9, s.ReachMax + 1e-9);
                            Assert.InRange(bearing, 0, s.ReachBearing + 1e-9);
                            Assert.True(raised, $"{name} seed {seed}: {o.Name} on the table");
                            double c = Math.Cos(yaw), sn = Math.Sin(yaw);
                            foreach (byte id in o.SupportIds)
                            {
                                int k = s.IndexOf(id);
                                Assert.False(parked[k], $"{name} seed {seed}: {o.Name} without {s.Objects[k].Name}");
                                Assert.Equal(p[i].Z - o.SizeZ * scales[i] / 2, s.Table.Top + s.Objects[k].SizeZ * scales[k], 9);
                                double along = (p[k].X - p[i].X) * c + (p[k].Y - p[i].Y) * sn, across = -(p[k].X - p[i].X) * sn + (p[k].Y - p[i].Y) * c;
                                Assert.Equal(o.SizeX * scales[i] / 2 - s.Objects[k].SizeX * scales[k] / 2, Math.Abs(along), 9);
                                Assert.Equal(0.0, across, 9);
                                Assert.Equal(0.0, Math.IEEERemainder(YawOf(p[k]) - yaw, 2 * Math.PI), 9);
                            }

                            double legs = o.SupportIds.Sum(id => s.Objects[s.IndexOf(id)].SizeX * scales[s.IndexOf(id)]);
                            Assert.True(o.SizeX * scales[i] - legs >= SceneLayout.MinRigSpan - 1e-9);
                            break;
                        case SceneRole.Support:
                            Assert.Contains(s.Objects, q => q.Role == SceneRole.Plank && Array.IndexOf(q.SupportIds, o.Id) >= 0 && !parked[Array.IndexOf(s.Objects, q)]);
                            break;
                    }
                }

                Assert.InRange(units, Math.Max(0, s.ObstacleMin - missed), s.ObstacleMax);
                Assert.InRange(distractors, Math.Max(0, s.DistractorMin - missed), s.DistractorMax);
            }
        }

        [Theory]
        [MemberData(nameof(Scenes))]
        public void TheSameSeed_GivesTheSameLayout_WithoutAllocating(string name)
        {
            WorldScene s = Load(name);
            var a = Draw(s, 17);
            var b = Draw(s, 17);
            Assert.Equal(a.Parked, b.Parked);
            Assert.Equal(a.Scales, b.Scales);
            Assert.Equal(a.Poses.Select(x => x.ToString()), b.Poses.Select(x => x.ToString()));
            Assert.NotEqual(a.Poses.Select(x => x.ToString()), Draw(s, 18).Poses.Select(x => x.ToString()));
            var poses = new RigidTransform[s.Objects.Length];
            var parked = new bool[s.Objects.Length];
            var scales = new double[s.Objects.Length];
            ulong seed = 1;
            Teleop.Core.Tests.TestSupport.AllocationAssert.Zero(() => SceneLayout.Draw(s, seed++, poses, parked, scales), iterations: 500);
        }

        [Fact]
        public void AStandHoldsWhatSitsOnItsTop_AndNothingOneAboveTheOtherOverlaps()
        {
            WorldScene s = Mixed;
            int stand = Array.FindIndex(s.Objects, o => o.Name == "stand-block"), can = Array.FindIndex(s.Objects, o => o.Name == "tomato-soup-can");
            int plank = Array.FindIndex(s.Objects, o => o.Name == "shelf-plank"), leg = Array.FindIndex(s.Objects, o => o.Name == "shelf-leg-a");
            double standZ = s.RestingZ(stand, 1.0, 0), top = s.Table.Top + s.Objects[stand].SizeZ, canZ = top + s.Objects[can].SizeZ / 2;
            Assert.True(s.HoldsOnTop(stand, 0.25, 0.0, standZ, 0.3, 1.0, 0, can, 0.26, 0.01, canZ, 1.0, 0));
            Assert.True(s.HoldsOnTop(stand, 0.25, 0.0, standZ, 0.3, 1.0, 0, can, 0.25, 0.0, canZ + 0.0019, 1.0, 0)); // within 2 mm
            Assert.False(s.HoldsOnTop(stand, 0.25, 0.0, standZ, 0.3, 1.0, 0, can, 0.25, 0.0, canZ + 0.0025, 1.0, 0)); // floating
            Assert.False(s.HoldsOnTop(stand, 0.25, 0.0, standZ, 0.0, 1.0, 0, can, 0.25 + 0.051, 0.0, canZ, 1.0, 0)); // past its edge
            Assert.False(s.HoldsOnTop(can, 0.25, 0.0, standZ, 0.0, 1.0, 0, stand, 0.25, 0.0, canZ, 1.0, 0)); // only a stand holds
            Assert.False(s.OverlapsAtHeights(stand, 0.25, 0.0, standZ, 0.3, can, 0.25, 0.0, canZ, 0.3, 1.0, 1.0, 0, 0));
            Assert.True(s.OverlapsAtHeights(stand, 0.25, 0.0, standZ, 0.3, can, 0.25, 0.0, canZ - 0.002, 0.3, 1.0, 1.0, 0, 0)); // 2 mm into it
            // A plank on its leg, and a can under a plank 15 cm up: one above the other. The can beside the leg: their rectangles apart.
            double legZ = s.RestingZ(leg, 1.0, 0), plankZ = s.Table.Top + s.Objects[leg].SizeZ + s.Objects[plank].SizeZ / 2;
            Assert.False(s.OverlapsAtHeights(plank, 0.3, 0.0, plankZ, 0.0, leg, 0.3 - 0.13, 0.0, legZ, 0.0, 1.0, 1.0, 0, 0));
            Assert.False(s.OverlapsAtHeights(plank, 0.3, 0.0, s.Table.Top + 0.15 + 0.009, 0.0, can, 0.3, 0.0, s.RestingZ(can, 1.0, 0), 0.7, 1.0, 1.0, 0, 0));
            Assert.True(s.OverlapsAtHeights(plank, 0.3, 0.0, plankZ, 0.0, can, 0.3, 0.0, s.RestingZ(can, 1.0, 0), 0.7, 1.0, 1.0, 0, 0)); // the can is taller
            // Elevated: only more than a millimetre up, and only in a mixed pool.
            Assert.False(s.IsElevated(can, s.RestingZ(can, 1.0, 0) + 0.0009, 1.0, 0));
            Assert.True(s.IsElevated(can, s.RestingZ(can, 1.0, 0) + 0.0011, 1.0, 0));
            WorldScene bench = Load("jetrover-bench");
            Assert.False(bench.IsElevated(0, 1.0, 1.0, 0));
        }

        [Fact]
        public void ADistractorFitsOnlyBeyondTheArmsReach_InItsRegions()
        {
            WorldScene s = Mixed;
            int phone = Array.FindIndex(s.Objects, o => o.Name == "phone"), kettle = Array.FindIndex(s.Objects, o => o.Name == "vintage-electric-kettle");
            // The phone, 7 by 15 cm, across y at the region's near edge: its near side 0.47 - 0.036 from the axis, short of 0.45.
            Assert.False(s.FitsOnTable(phone, 0.10, 0.47, Math.PI / 2, 1.0, 0));
            Assert.True(s.FitsOnTable(phone, 0.10, 0.50, Math.PI / 2, 1.0, 0));
            Assert.False(s.FitsOnTable(phone, 0.50, 0.10, 0, 1.0, 0)); // beyond the reach, but outside the regions
            Assert.False(s.FitsOnTable(kettle, 0.20, 0.70, 0, 1.0, 0)); // off the table's side edge
            Assert.True(s.FitsOnTable(kettle, 0.20, 0.62, 0, 1.0, 0));
            // An obstacle never stands there: the placement region holds it, and the draws centre it within the reach band.
            int can = Array.FindIndex(s.Objects, o => o.Name == "tomato-soup-can");
            Assert.True(s.FitsOnTable(can, 0.30, 0.0, 0.4, 1.0, 0));
            Assert.False(s.FitsOnTable(can, 0.10, 0.0, 0.4, 1.0, 0)); // on the robot's keep-out
        }

        [Fact]
        public void AMixedLayout_SpreadsOverTheReachableBand_AtItsShares()
        {
            // docs/adr/0038: what a mixed layout puts in front of the arm, over 2000 layouts, and how far apart the layouts' obstacles are.
            foreach (WorldScene s in new[] { Mixed, Holdout })
            {
                Stats m = Measure(s, 2000);
                _output.WriteLine($"{s.Name}: {m}");
                // 0-3 obstacles uniform, fewer only by the ones that found no spot; 2-6 distractors the same.
                Assert.InRange(m.MeanUnits, 1.3, 1.5);
                Assert.InRange(m.MeanDistractors, 3.6, 4.0);
                Assert.True(m.MissedPerLayout < 0.4, $"{m.MissedPerLayout:0.00} found no spot a layout");
                // Raised on stands, and planks on their supports, in a share of the obstacles; every object takes part.
                Assert.InRange(m.RaisedShare, 0.08, 0.30);
                Assert.InRange(m.RigShare, 0.10, 0.30);
                Assert.Equal(0, m.NeverPlaced);
                // Uniform by area over the band: the share of centres nearer than its middle radius is about what the area holds.
                Assert.InRange(m.InnerShare, m.InnerArea - 0.08, m.InnerArea + 0.08);
                Assert.True(m.SideShare > 0.25, $"{m.SideShare:0.00} beside the robot"); // beyond 60 degrees of straight ahead
                // Contacts at many heights: the tops the arm meets range over 20 cm.
                Assert.True(m.TopHeights[2] - m.TopHeights[0] > 0.15, string.Join(", ", m.TopHeights));
            }
        }

        [Fact]
        public void AMixedLayoutsObstacles_LieFartherFromEarlierLayouts_ThanTheBenchsObjectsInReach()
        {
            // docs/adr/0038 (world-models: val and test contacts sat a median 2.5 cm from the nearest train contact): how near an object a
            // new layout puts within the arm's reach comes to one an earlier layout put there, at the same number of objects (1000
            // earlier, 200 new), and how high their tops stand. The bench's are its movable objects in the reach band and its fixture.
            WorldScene bench = Load("jetrover-bench");
            (double Median, double P90, double TopLo, double TopHi) mixed = Spread(Mixed, (s, i, p, scale) => s.Objects[i].Role == SceneRole.Obstacle || s.Objects[i].Role == SceneRole.Plank);
            (double Median, double P90, double TopLo, double TopHi) old = Spread(bench, (s, i, p, scale) => !s.Objects[i].Clutter && !(s.Objects[i].Movable && s.InEdgeRegions(p.X, p.Y)));
            _output.WriteLine(FormattableString.Invariant(
                $"a new layout's objects in reach to the nearest of 1000 earlier ones: jetrover-mixed median {mixed.Median * 1000:0.0} mm, 90% {mixed.P90 * 1000:0.0} mm; ") +
                FormattableString.Invariant($"jetrover-bench median {old.Median * 1000:0.0} mm, 90% {old.P90 * 1000:0.0} mm. ") +
                FormattableString.Invariant($"Their tops 5-95%: mixed {mixed.TopLo:0.000}-{mixed.TopHi:0.000} m, bench {old.TopLo:0.000}-{old.TopHi:0.000} m"));
            Assert.True(mixed.Median > old.Median, "the mixed layouts' obstacles spread over more of the arm's reach");
        }

        /// <summary>
        /// Centres of the objects <paramref name="select"/> takes from layouts 1, 2, ...: the first 1000 are the earlier ones, the next 200
        /// the new; the median and 90th percentile of each new one's distance to the nearest earlier one, and the 5th and 95th percentile
        /// of their tops above the table.
        /// </summary>
        private static (double, double, double, double) Spread(WorldScene s, Func<WorldScene, int, RigidTransform, double, bool> select)
        {
            var centres = new List<(double X, double Y)>();
            var tops = new List<double>();
            for (ulong seed = 1; centres.Count < 1200; seed++)
            {
                (RigidTransform[] p, bool[] parked, double[] scales, _) = Draw(s, seed);
                for (int i = 0; i < p.Length && centres.Count < 1200; i++)
                {
                    if (parked[i] || !select(s, i, p[i], scales[i])) continue;
                    centres.Add((p[i].X, p[i].Y));
                    s.RestExtents(i, WorldScene.RestOf(p[i]), out _, out _, out double ez);
                    tops.Add(p[i].Z + ez * scales[i] / 2 - s.Table.Top);
                }
            }

            var nn = centres.Skip(1000).Select(c => Math.Sqrt(centres.Take(1000).Min(e => (c.X - e.X) * (c.X - e.X) + (c.Y - e.Y) * (c.Y - e.Y)))).OrderBy(d => d).ToArray();
            tops.Sort();
            return (nn[nn.Length / 2], nn[(int)(0.9 * nn.Length)], tops[(int)(0.05 * tops.Count)], tops[(int)(0.95 * tops.Count)]);
        }

        private sealed class Stats
        {
            public double MeanUnits, MeanDistractors, MissedPerLayout, RaisedShare, RigShare, InnerShare, InnerArea, SideShare, NnMedian, Nn90;
            public int NeverPlaced;
            public int[] Units = new int[4], Distractors = new int[7];
            public double[] TopHeights = new double[3]; // the 5th, 50th and 95th percentile of the obstacles' tops above the table, m

            public override string ToString() => FormattableString.Invariant(
                $"obstacles {MeanUnits:0.00} a layout ({string.Join("/", Units)} with 0-3), distractors {MeanDistractors:0.00} ({string.Join("/", Distractors.Skip(2))} with 2-6), ") +
                FormattableString.Invariant($"{MissedPerLayout:0.000} found no spot a layout; raised on a stand {RaisedShare:0.000} and a plank {RigShare:0.000} of the obstacles; ") +
                FormattableString.Invariant($"centres inside the band's middle radius {InnerShare:0.000} (area {InnerArea:0.000}), beyond 60 degrees {SideShare:0.000}; ") +
                FormattableString.Invariant($"tops {TopHeights[0]:0.000}/{TopHeights[1]:0.000}/{TopHeights[2]:0.000} m (5/50/95%); ") +
                FormattableString.Invariant($"nearest obstacle of another layout (first 1000): median {NnMedian * 1000:0.0} mm, 90% {Nn90 * 1000:0.0} mm; never placed {NeverPlaced}");
        }

        /// <summary>
        /// Over seeds 1 to <paramref name="layouts"/>: obstacles (an object, an object on its stand, or a plank on its supports, each one)
        /// and distractors a layout, the misses, the shares of raised objects and planks, where the obstacles' centres stand in the band,
        /// how high their tops are, and how near each obstacle of the first 1000 layouts comes to one of another layout.
        /// </summary>
        private static Stats Measure(WorldScene s, int layouts)
        {
            var m = new Stats();
            var placedEver = new bool[s.Objects.Length];
            var centres = new List<(int Layout, double X, double Y)>();
            var tops = new List<double>();
            int units = 0, distractors = 0, missed = 0, raised = 0, rigs = 0, inner = 0, side = 0;
            double mid = Math.Sqrt((s.ReachMin * s.ReachMin + s.ReachMax * s.ReachMax) / 2); // half the band's area inside it
            for (ulong seed = 1; seed <= (ulong)layouts; seed++)
            {
                (RigidTransform[] p, bool[] parked, double[] scales, int miss) = Draw(s, seed);
                missed += miss;
                int u = 0, d = 0;
                for (int i = 0; i < p.Length; i++)
                {
                    if (parked[i]) continue;
                    placedEver[i] = true;
                    SceneObject o = s.Objects[i];
                    int rest = WorldScene.RestOf(p[i]);
                    s.RestExtents(i, rest, out _, out _, out double ez);
                    if (o.Role == SceneRole.Distractor) d++;
                    if (o.Role != SceneRole.Obstacle && o.Role != SceneRole.Plank) continue;
                    u++;
                    bool up = s.IsElevated(i, p[i].Z, scales[i], rest);
                    if (o.Role == SceneRole.Plank) rigs++;
                    else if (up) raised++;
                    tops.Add(p[i].Z + ez * scales[i] / 2 - s.Table.Top);
                    double r = Math.Sqrt(p[i].X * p[i].X + p[i].Y * p[i].Y);
                    // The stand's centre for a raised object (its offset aside), the plank's own.
                    inner += r < mid ? 1 : 0;
                    side += Math.Abs(Math.Atan2(p[i].Y, p[i].X)) > Math.PI / 3 ? 1 : 0;
                    if (seed <= 1000) centres.Add(((int)seed, p[i].X, p[i].Y));
                }

                units += u;
                distractors += d;
                m.Units[u]++;
                m.Distractors[d]++;
            }

            m.MeanUnits = (double)units / layouts;
            m.MeanDistractors = (double)distractors / layouts;
            m.MissedPerLayout = (double)missed / layouts;
            m.RaisedShare = (double)raised / units;
            m.RigShare = (double)rigs / units;
            m.InnerShare = (double)inner / units;
            m.SideShare = (double)side / units;
            m.NeverPlaced = placedEver.Count(x => !x);
            // The area inside the middle radius of the part of the band where an obstacle can stand: estimated over the same disc
            // sector the draw samples, kept where the first scan's footprint fits (the robot's keep-out and the surfaces cut it).
            var rng = new SeededRng(99);
            int fit = 0, fitInner = 0, can = Array.FindIndex(s.Objects, o => o.Role == SceneRole.Obstacle && o.Shape == SceneShape.Mesh);
            for (int k = 0; k < 20000; k++)
            {
                s.DrawObstacleSpot(ref rng, out double x, out double y, out double yaw);
                if (!s.FitsOnTable(can, x, y, yaw, 1.0, WorldScene.RestUpright)) continue;
                fit++;
                fitInner += Math.Sqrt(x * x + y * y) < mid ? 1 : 0;
            }

            m.InnerArea = (double)fitInner / fit;
            tops.Sort();
            m.TopHeights = new[] { tops[(int)(0.05 * tops.Count)], tops[tops.Count / 2], tops[(int)(0.95 * tops.Count)] };
            var nn = new List<double>();
            foreach ((int layout, double x, double y) in centres)
            {
                double best = double.MaxValue;
                foreach ((int other, double ox, double oy) in centres)
                {
                    if (other != layout) best = Math.Min(best, (x - ox) * (x - ox) + (y - oy) * (y - oy));
                }

                nn.Add(Math.Sqrt(best));
            }

            nn.Sort();
            m.NnMedian = nn[nn.Count / 2];
            m.Nn90 = nn[(int)(0.9 * nn.Count)];
            return m;
        }

        [Fact]
        public void TheBenchsPoolLayouts_AreFrozen_SoAMixedPoolChangesNoDrawOfThem()
        {
            // docs/adr/0038: a mixed pool's draws are opt-in. core/Scenes/testdata/layout-jetrover-bench.txt holds seeds 1-25 of the
            // JetRover bench (which is no mixed pool) as the code before them drew them, to the last bit: written once from that
            // commit, rewritten only on purpose (`just gen-isaac-goldens`).
            WorldScene bench = Load("jetrover-bench");
            var lines = new List<string>();
            for (ulong seed = 1; seed <= 25; seed++)
            {
                (RigidTransform[] p, bool[] parked, double[] scales, int missed) = Draw(bench, seed);
                lines.Add(FormattableString.Invariant($"seed {seed} missed {missed}"));
                for (int i = 0; i < p.Length; i++)
                {
                    if (parked[i]) continue;
                    lines.Add(string.Join(" ", new[] { bench.Objects[i].Id.ToString(CultureInfo.InvariantCulture) }.Concat(
                        new[] { p[i].X, p[i].Y, p[i].Z, p[i].Qw, p[i].Qx, p[i].Qy, p[i].Qz, scales[i] }.Select(v => v.ToString("R", CultureInfo.InvariantCulture)))));
                }
            }

            string path = Path.Combine(Kinematics.SerialChainKinematicsTests.RepoRoot(), "core", "Scenes", "testdata", "layout-jetrover-bench.txt");
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, string.Join("\n", lines) + "\n", new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-isaac-goldens`");
            Assert.Equal(File.ReadAllText(path).Replace("\r\n", "\n").TrimEnd('\n').Split('\n'), lines);
        }

        [Fact]
        public void TheRulesGoldenFile_IsWhatThisReaderMakesOfTheMixedScene()
        {
            // robot/isaac's scene.py must take a mixed pool's footprints, heights and stands as this reader does, or Isaac refuses
            // what RobotHost draws (OUTSIDE_REGION, OVERLAPS_OBJECT) or stands a raised object on the table.
            string expected = RulesGoldenJson();
            string path = Path.Combine(Kinematics.SerialChainKinematicsTests.RepoRoot(), "robot", "isaac", "golden", "scene-rules-jetrover-mixed.json");
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, expected, new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-isaac-goldens`");
            GoldenFile.AssertSame(File.ReadAllText(path), expected);
        }

        /// <summary>
        /// A mixed pool's rules as this reader has them, for robot/isaac/tests/test_scene.py: every object's role, steps and supports;
        /// spots that fit or not, with a yaw and without, a third of them by the edges of the distractor regions; pairs that overlap or
        /// not at their heights, a third of them one above the other; stands that hold an object or not; and heights that stand an
        /// object above the table or not. Rows of numbers (a flag is 0 or 1).
        /// </summary>
        internal static string RulesGoldenJson()
        {
            WorldScene s = Mixed;
            byte[] bytes = File.ReadAllBytes(SceneDocumentTests.ScenePath("jetrover-mixed"));
            var rng = new SeededRng(20261122);
            double U(double lo, double hi) => lo + rng.NextDouble() * (hi - lo);
            int I(int n) => Math.Min(n - 1, (int)(rng.NextDouble() * n));
            double minX = Math.Min(s.Table.XMin, s.Bench!.XMin), maxX = Math.Max(s.Table.XMax, s.Bench.XMax);
            int[] Steps(int i) => s.Objects[i].HasSteps ? Enumerable.Range(s.Objects[i].StepMin, s.Objects[i].StepMax - s.Objects[i].StepMin + 1).ToArray() : new[] { 0 };
            double Scale(int i) => WorldScene.ScaleOfStep(Steps(i)[I(Steps(i).Length)]);
            int Rest(int i) => s.Objects[i].Movable && !s.Objects[i].Clutter ? new[] { 0, 0, 1, 2, 4 }[I(5)] : 0;
            var sb = new StringBuilder();
            sb.Append("{\n \"about\": \"docs/adr/0038 decision 3: the rules of core/Scenes/jetrover-mixed.json, a mixed pool, written by SceneMixedTests (just gen-isaac-goldens). ");
            sb.Append("objects: name, role, steps (min, max, or empty), supports (ids). fits: index, rest, scale, x, y, yaw, FitsOnTable at the yaw, FitsOnTable without one. ");
            sb.Append("overlaps: a, x, y, z, yaw, rest, scale, b, x, y, z, yaw, rest, scale, OverlapsAtHeights. holds: stand, x, y, z, yaw, scale, object, x, y, z, rest, scale, HoldsOnTop. ");
            sb.Append("elevated: index, rest, scale, z, RestingZ, IsElevated.\",\n");
            sb.Append($" \"scene\": \"{s.Name}\",\n");
            sb.Append($" \"sha256\": \"{Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant()}\",\n");
            sb.Append($" \"pool\": {Row(s.ObstacleMin, s.ObstacleMax, s.DistractorMin, s.DistractorMax, s.ReachMin, s.ReachMax, s.ReachBearing, s.StandShare, s.RigShare, s.DistractorReach)},\n");
            sb.Append($" \"distractorRegions\": {Row(s.DistractorRegions)},\n");
            sb.Append(" \"objects\": [\n");
            for (int i = 0; i < s.Objects.Length; i++)
            {
                SceneObject o = s.Objects[i];
                string steps = o.HasSteps ? Row(o.StepMin, o.StepMax) : "[]";
                sb.Append($"  [\"{o.Name}\", \"{o.Role.ToString().ToLowerInvariant()}\", {steps}, {Row(o.SupportIds.Select(b => (double)b).ToArray())}]{(i < s.Objects.Length - 1 ? "," : "")}\n");
            }

            sb.Append(" ],\n");
            var fits = new List<string>();
            for (int i = 0; i < s.Objects.Length; i++)
            {
                for (int n = 0; n < 12; n++)
                {
                    int rest = Rest(i);
                    double scale = Scale(i), x, y;
                    if (n % 3 == 0)
                    {
                        // By a distractor region's edges, and the reach beyond which a distractor stands.
                        int k = 4 * I(s.DistractorRegions.Length / 4);
                        x = U(s.DistractorRegions[k] - 0.06, s.DistractorRegions[k + 1] + 0.06);
                        y = U(s.DistractorRegions[k + 2] - 0.06, s.DistractorRegions[k + 3] + 0.06);
                    }
                    else
                    {
                        x = U(minX, maxX);
                        y = U(-0.6, 0.6);
                    }

                    double yaw = U(-Math.PI, Math.PI);
                    fits.Add(Row(i, rest, scale, x, y, yaw, s.FitsOnTable(i, x, y, yaw, scale, rest) ? 1 : 0, s.FitsOnTable(i, x, y, scale, rest) ? 1 : 0));
                }
            }

            var overlaps = new List<string>();
            for (int n = 0; n < 240; n++)
            {
                int a = I(s.Objects.Length), b = I(s.Objects.Length);
                int ra = Rest(a), rb = Rest(b);
                double sa = Scale(a), sb2 = Scale(b);
                double ax = U(0.0, 0.4), ay = U(-0.3, 0.3), bearing = U(-Math.PI, Math.PI), yawA = U(-Math.PI, Math.PI), yawB = U(-Math.PI, Math.PI);
                double d = (s.FootprintRadiusAt(a, ra) * sa + s.FootprintRadiusAt(b, rb) * sb2 + s.Clearance) * U(0.1, 1.1);
                double bx = ax + d * Math.Cos(bearing), by = ay + d * Math.Sin(bearing);
                s.RestExtents(a, ra, out _, out _, out double eza);
                s.RestExtents(b, rb, out _, out _, out double ezb);
                double az = s.RestingZ(a, sa, ra), bz = s.RestingZ(b, sb2, rb);
                if (n % 3 == 0) bz = az + eza * sa / 2 + ezb * sb2 / 2 + U(-0.003, 0.003); // b on a's top, give or take 3 mm
                overlaps.Add(Row(a, ax, ay, az, yawA, ra, sa, b, bx, by, bz, yawB, rb, sb2,
                    s.OverlapsAtHeights(a, ax, ay, az, yawA, b, bx, by, bz, yawB, sa, sb2, ra, rb) ? 1 : 0));
            }

            var holds = new List<string>();
            int[] stands = Enumerable.Range(0, s.Objects.Length).Where(i => s.Objects[i].Role == SceneRole.Stand).ToArray();
            int[] movable = Enumerable.Range(0, s.Objects.Length).Where(i => s.Objects[i].Movable).ToArray();
            for (int n = 0; n < 160; n++)
            {
                int k = n % 8 == 0 ? movable[I(movable.Length)] : stands[I(stands.Length)], i = movable[I(movable.Length)];
                int rest = Rest(i);
                double ks = Scale(k), sc = Scale(i), kx = U(0.1, 0.3), ky = U(-0.2, 0.2), kyaw = U(-Math.PI, Math.PI);
                double kz = s.RestingZ(k, ks, 0), half = s.Objects[k].SizeX * ks / 2;
                s.RestExtents(i, rest, out _, out _, out double ez);
                double x = kx + U(-1.3, 1.3) * half, y = ky + U(-1.3, 1.3) * half;
                double z = s.Table.Top + s.Objects[k].SizeZ * ks + ez * sc / 2 + U(-0.004, 0.004);
                holds.Add(Row(k, kx, ky, kz, kyaw, ks, i, x, y, z, rest, sc, s.HoldsOnTop(k, kx, ky, kz, kyaw, ks, 0, i, x, y, z, sc, rest) ? 1 : 0));
            }

            var elevated = new List<string>();
            for (int n = 0; n < 60; n++)
            {
                int i = I(s.Objects.Length), rest = Rest(i);
                double sc = Scale(i), z = s.RestingZ(i, sc, rest) + U(-0.002, 0.004);
                elevated.Add(Row(i, rest, sc, z, s.RestingZ(i, sc, rest), s.IsElevated(i, z, sc, rest) ? 1 : 0));
            }

            void Rows(string name, List<string> rows, bool last) => sb.Append($" \"{name}\": [\n  ").Append(string.Join(",\n  ", rows)).Append(last ? "\n ]\n" : "\n ],\n");
            Rows("fits", fits, false);
            Rows("overlaps", overlaps, false);
            Rows("holds", holds, false);
            Rows("elevated", elevated, true);
            sb.Append("}\n");
            return sb.ToString();
        }

        private static string Row(params double[] v) => "[" + string.Join(", ", v.Select(x => x.ToString("R", CultureInfo.InvariantCulture))) + "]";
    }
}
