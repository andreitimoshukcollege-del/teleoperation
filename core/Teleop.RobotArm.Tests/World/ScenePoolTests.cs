using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>
    /// A scene whose objects are a pool (docs/adr/0037 decision 3): each layout puts a few of each kind on the table, anywhere
    /// in the region and apart, and parks the rest off-stage. The shipped JetRover bench is one.
    /// </summary>
    public class ScenePoolTests
    {
        private static readonly WorldScene Pool = JsonSerializer.Deserialize<SceneDocument>(
            File.ReadAllText(SceneDocumentTests.ScenePath("jetrover-bench")), new JsonSerializerOptions { IncludeFields = true })!.ToScene();

        private static (RigidTransform[] Poses, bool[] Parked, int Missed) Draw(ulong seed) => DrawSized(seed).Item1;

        private static ((RigidTransform[] Poses, bool[] Parked, int Missed), double[] Scales) DrawSized(ulong seed)
        {
            var poses = new RigidTransform[Pool.Objects.Length];
            var parked = new bool[Pool.Objects.Length];
            var scales = new double[Pool.Objects.Length];
            int missed = SceneLayout.Draw(Pool, seed, poses, parked, scales);
            return ((poses, parked, missed), scales);
        }

        [Fact]
        public void TheShippedBench_IsAPool()
        {
            Assert.True(Pool.IsPool);
            Assert.True(Pool.Objects.Count(o => o.Movable) >= Pool.MovableMax);
            Assert.True(Pool.Objects.Count(o => !o.Movable) >= Pool.FixedMax);
        }

        [Fact]
        public void EveryLayout_TakesEachKindWithinItsRange_StandsThemApartOnTheTable_AndParksTheRest()
        {
            for (ulong seed = 1; seed <= 200; seed++)
            {
                ((RigidTransform[] poses, bool[] parked, int missed), double[] scales) = DrawSized(seed);
                int movable = 0, fixeds = 0;
                for (int i = 0; i < poses.Length; i++)
                {
                    if (parked[i])
                    {
                        Assert.Equal(Pool.ParkedPose(i).ToString(), poses[i].ToString());
                        continue;
                    }

                    if (Pool.Objects[i].Movable) movable++;
                    else fixeds++;
                    Assert.True(Pool.FitsOnTable(i, poses[i].X, poses[i].Y, scales[i]), $"seed {seed}: {Pool.Objects[i].Name} off the table");
                    Assert.True(Pool.ClearOfRobot(poses[i].X, poses[i].Y, Pool.Objects[i].FootprintRadius * scales[i]), $"seed {seed}: {Pool.Objects[i].Name} on the robot");
                    // A fixture stands beside the robot, its near side within the arm's reach (amendment 5).
                    if (Pool.InFixedRegions(i)) Assert.True(Math.Sqrt(poses[i].X * poses[i].X + poses[i].Y * poses[i].Y) - Pool.Objects[i].FootprintRadius <= Pool.FixedNearMax + 1e-9, $"seed {seed}: {Pool.Objects[i].Name} out of reach");
                    // A movable object stands where the arm reaches it (amendment 3): within the band from the robot's base.
                    if (Pool.Objects[i].Movable) Assert.InRange(Math.Sqrt(poses[i].X * poses[i].X + poses[i].Y * poses[i].Y), Pool.ReachMin, Pool.ReachMax);
                    Assert.Equal(Pool.Table.Top + Pool.Objects[i].SizeZ * scales[i] / 2, poses[i].Z, 9);
                    for (int k = 0; k < i; k++)
                    {
                        if (!parked[k]) Assert.False(Pool.Overlaps(i, poses[i].X, poses[i].Y, k, poses[k].X, poses[k].Y, scales[i], scales[k]), $"seed {seed}: {Pool.Objects[i].Name} on {Pool.Objects[k].Name}");
                    }
                }

                // An object that found no spot is parked too: never more on the table than the range, and fewer only by the misses.
                Assert.InRange(movable, Pool.MovableMin - missed, Pool.MovableMax);
                Assert.InRange(fixeds, Pool.FixedMin - missed, Pool.FixedMax);
            }
        }

        [Fact]
        public void Layouts_DifferInWhichObjects_HowMany_AndWhere()
        {
            var sets = Enumerable.Range(1, 60).Select(s => Draw((ulong)s)).ToArray();
            string Key((RigidTransform[] Poses, bool[] Parked, int Missed) d) => string.Join(",", d.Parked.Select((p, i) => p ? "" : Pool.Objects[i].Name).Where(n => n != ""));
            Assert.True(sets.Select(Key).Distinct().Count() > 50, "almost every layout takes a different set of objects");
            Assert.True(sets.Select(d => d.Parked.Count(p => !p)).Distinct().Count() >= 4, "the count on the table varies");
            // Every object of the pool takes part in some layout. The large ones less often (2026-11-15, 2000 layouts: the mug
            // 7.5 %, the small primitives about 20 %): their round footprint more often finds no free spot in the reach band.
            var many = Enumerable.Range(1, 300).Select(s => Draw((ulong)s)).ToArray();
            Assert.All(Enumerable.Range(0, Pool.Objects.Length), i => Assert.Contains(many, d => !d.Parked[i]));
            // Anywhere in the reach band, not around a nominal pose: the movable objects spread over most of its depth and
            // swing, and the fixed ones over most of the region.
            double[] rs = sets.SelectMany(d => d.Poses.Where((p, i) => !d.Parked[i] && Pool.Objects[i].Movable).Select(p => Math.Sqrt(p.X * p.X + p.Y * p.Y))).ToArray();
            Assert.True(rs.Max() - rs.Min() > 0.7 * (Pool.ReachMax - Pool.ReachMin));
            double[] ys = sets.SelectMany(d => d.Poses.Where((p, i) => !d.Parked[i] && Pool.Objects[i].Movable).Select(p => p.Y)).ToArray();
            Assert.True(ys.Max() - ys.Min() > 0.3);
        }

        [Fact]
        public void EachLayout_DrawsSizesFromTheResizeSteps_KeepingTheLargestSideInThePoolsRange()
        {
            if (!Pool.DrawsSizes || !Enumerable.Range(0, Pool.Objects.Length).Any(Pool.Resizable))
            {
                // Every object a real scan at its real size (amendment 5): no layout resizes anything.
                for (ulong seed = 1; seed <= 50; seed++) Assert.All(DrawSized(seed).Item2, k => Assert.Equal(1.0, k));
                return;
            }

            var seen = new System.Collections.Generic.Dictionary<string, System.Collections.Generic.HashSet<double>>();
            for (ulong seed = 1; seed <= 300; seed++)
            {
                ((RigidTransform[] _, bool[] parked, int _), double[] scales) = DrawSized(seed);
                for (int i = 0; i < scales.Length; i++)
                {
                    SceneObject o = Pool.Objects[i];
                    if (parked[i] || !Pool.Resizable(i))
                    {
                        Assert.Equal(1.0, scales[i]); // a fixed wall, or one off the table
                        continue;
                    }

                    Assert.True(WorldScene.IsValidScale(scales[i]), $"{o.Name}: {scales[i]} is no resize step");
                    double side = Math.Max(o.SizeX, Math.Max(o.SizeY, o.SizeZ)) * scales[i];
                    Assert.InRange(side, Pool.SizeMin - 1e-9, Pool.SizeMax + 1e-9);
                    if (!seen.TryGetValue(o.Name, out var s)) seen[o.Name] = s = new System.Collections.Generic.HashSet<double>();
                    s.Add(scales[i]);
                }
            }

            // The 2.5 cm cube runs from 2 cm to about 12 cm: it takes most of the steps between, not one size.
            Assert.True(seen["cube"].Count >= 6, string.Join(", ", seen["cube"]));
            int resizable = Enumerable.Range(0, Pool.Objects.Length).Count(Pool.Resizable); // the real objects keep their real size
            Assert.True(seen.Values.Count(v => v.Count >= 3) >= resizable * 3 / 4, "most resizable objects take several sizes");
        }

        [Fact]
        public void TheSameSeed_GivesTheSameLayout()
        {
            var a = Draw(11);
            var b = Draw(11);
            Assert.Equal(a.Parked, b.Parked);
            Assert.Equal(a.Poses.Select(p => p.ToString()), b.Poses.Select(p => p.ToString()));
        }

        [Fact]
        public void APool_NeedsTheParkedOverload_AndDrawsWithoutAllocating()
        {
            Assert.Throws<ArgumentException>(() => SceneLayout.Draw(Pool, 1, new RigidTransform[Pool.Objects.Length]));
            var poses = new RigidTransform[Pool.Objects.Length];
            var parked = new bool[Pool.Objects.Length];
            ulong s = 1;
            Teleop.Core.Tests.TestSupport.AllocationAssert.Zero(() => SceneLayout.Draw(Pool, s++, poses, parked), iterations: 500);
        }

        [Fact]
        public void AParkedPose_IsOffStage_BeyondEveryCameraAndArm()
        {
            for (int i = 0; i < Pool.Objects.Length; i++)
            {
                RigidTransform p = Pool.ParkedPose(i);
                Assert.True(p.Z < Pool.Table.Top - 4 && p.X > 10, Pool.Objects[i].Name);
                for (int k = 0; k < i; k++) Assert.True(Math.Abs(Pool.ParkedPose(k).Y - p.Y) >= 0.999, "a metre apart on the shelf");
            }
        }
    }
}
