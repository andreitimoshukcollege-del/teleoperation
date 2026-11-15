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

        private static (RigidTransform[] Poses, bool[] Parked, int Missed) Draw(ulong seed)
        {
            var poses = new RigidTransform[Pool.Objects.Length];
            var parked = new bool[Pool.Objects.Length];
            int missed = SceneLayout.Draw(Pool, seed, poses, parked);
            return (poses, parked, missed);
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
                (RigidTransform[] poses, bool[] parked, int missed) = Draw(seed);
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
                    Assert.True(Pool.FitsOnTable(i, poses[i].X, poses[i].Y), $"seed {seed}: {Pool.Objects[i].Name} off the table");
                    Assert.Equal(Pool.Table.Top + Pool.Objects[i].SizeZ / 2, poses[i].Z, 9);
                    for (int k = 0; k < i; k++)
                    {
                        if (!parked[k]) Assert.False(Pool.Overlaps(i, poses[i].X, poses[i].Y, k, poses[k].X, poses[k].Y), $"seed {seed}: {Pool.Objects[i].Name} on {Pool.Objects[k].Name}");
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
            // Every object of the pool takes part in some layout.
            Assert.All(Enumerable.Range(0, Pool.Objects.Length), i => Assert.Contains(sets, d => !d.Parked[i]));
            // Anywhere in the region, not around a nominal pose: the movable objects' x spread over most of it.
            double[] xs = sets.SelectMany(d => d.Poses.Where((p, i) => !d.Parked[i] && Pool.Objects[i].Movable).Select(p => p.X)).ToArray();
            Assert.True(xs.Max() - xs.Min() > 0.7 * (Pool.RegionXMax - Pool.RegionXMin));
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
