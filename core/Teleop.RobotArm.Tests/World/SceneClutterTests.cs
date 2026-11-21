using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.Json;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit.Abstractions;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>
    /// docs/adr/0037 "Raised clutter": desk clutter tall enough for the arm to hit (tape, scissors, pliers, screwdrivers, a phone, a
    /// pen) is real scene objects of the JetRover bench, stood by a pass of its own after every other object, anywhere in the
    /// placement region. The bench's other objects stand exactly where they stood before clutter came.
    /// </summary>
    public class SceneClutterTests
    {
        private static readonly JsonSerializerOptions Options = new JsonSerializerOptions { IncludeFields = true };
        private static readonly WorldScene Bench = Load(_ => { });
        private readonly ITestOutputHelper _output;

        public SceneClutterTests(ITestOutputHelper output) => _output = output;

        private static WorldScene Load(Action<SceneDocument> change)
        {
            SceneDocument d = JsonSerializer.Deserialize<SceneDocument>(File.ReadAllText(SceneDocumentTests.ScenePath("jetrover-bench")), Options)!;
            change(d);
            return d.ToScene();
        }

        /// <summary>The bench as it was before clutter: its other objects alone, and no clutter range.</summary>
        private static WorldScene Before() => Load(d =>
        {
            d.objects = d.objects.Where(o => !o.clutter).ToArray();
            d.pool.clutterMin = d.pool.clutterMax = 0;
        });

        private static (RigidTransform[] Poses, bool[] Parked, int Missed) Draw(WorldScene scene, ulong seed)
        {
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            int missed = SceneLayout.Draw(scene, seed, poses, parked, scales);
            return (poses, parked, missed);
        }

        [Fact]
        public void TheBenchsClutter_IsSixFlatMovableObjects_ThatNeverLieOnASide()
        {
            string[] clutter = Bench.Objects.Where(o => o.Clutter).Select(o => o.Name).ToArray();
            Assert.Equal(new[] { "tape-roll", "scissors", "pliers", "screwdrivers", "phone", "pen" }, clutter);
            Assert.Equal((1, 3), (Bench.ClutterMin, Bench.ClutterMax));
            // The pool's movable objects are the others: clutter takes none of the 2-5 a layout stands in reach.
            Assert.True(Bench.Objects.Count(o => o.Movable && !o.Clutter) >= Bench.MovableMax);
            foreach (int i in Enumerable.Range(0, Bench.Objects.Length).Where(i => Bench.Objects[i].Clutter))
            {
                Assert.DoesNotContain(Enumerable.Range(0, WorldScene.RestCount), r => Bench.CanLie(i, r));
                // Held to no reach band: the far corner of the region is out of the arm's reach, and clutter may stand there.
                Assert.True(Bench.InReach(i, Bench.RegionXMax - 0.02, Bench.RegionYMax - 0.02), Bench.Objects[i].Name);
            }

            int can = Array.FindIndex(Bench.Objects, o => o.Name == "tomato-soup-can");
            Assert.False(Bench.InReach(can, Bench.RegionXMax - 0.02, Bench.RegionYMax - 0.02));
            Assert.True(Bench.Objects.Length <= WorldScene.MaxObjects && WorldScene.MaxObjects == 32);
        }

        [Fact]
        public void EveryOtherObject_StandsExactlyWhereItStoodBeforeClutter()
        {
            // The clutter pass draws from a generator of its own, after every other object: without clutter (none in the file, or
            // a range of 0) a layout is exactly what it was, and with it only the clutter is new. A parked object's shelf slot
            // depends on how many objects the scene has, so only the objects on the table are compared.
            WorldScene before = Before();
            WorldScene none = Load(d => d.pool.clutterMin = d.pool.clutterMax = 0);
            int[] others = Enumerable.Range(0, Bench.Objects.Length).Where(i => !Bench.Objects[i].Clutter).ToArray();
            Assert.Equal(before.Objects.Select(o => o.Name), others.Select(i => Bench.Objects[i].Name));
            for (ulong seed = 1; seed <= 500; seed++)
            {
                var b = Draw(before, seed);
                var n = Draw(none, seed);
                var c = Draw(Bench, seed);
                Assert.All(Enumerable.Range(0, Bench.Objects.Length).Where(i => Bench.Objects[i].Clutter), i => Assert.True(n.Parked[i]));
                for (int k = 0; k < others.Length; k++)
                {
                    int i = others[k];
                    Assert.Equal(b.Parked[k], n.Parked[i]);
                    Assert.Equal(b.Parked[k], c.Parked[i]);
                    if (b.Parked[k]) continue;
                    Assert.Equal(b.Poses[k].ToString(), n.Poses[i].ToString());
                    Assert.Equal(b.Poses[k].ToString(), c.Poses[i].ToString());
                }

                int clutterMissed = c.Missed - b.Missed;
                Assert.True(clutterMissed >= 0 && n.Missed == b.Missed, $"seed {seed}");
            }
        }

        [Fact]
        public void Clutter_TakesPartAnywhereInTheRegion_SomeInReachAndSomeBeyondIt()
        {
            const int Layouts = 2000;
            int[] clutter = Enumerable.Range(0, Bench.Objects.Length).Where(i => Bench.Objects[i].Clutter).ToArray();
            var onTable = new int[Bench.Objects.Length];
            var perLayout = new int[Bench.ClutterMax + 1];
            int inReach = 0, beyond = 0, total = 0;
            for (ulong seed = 1; seed <= Layouts; seed++)
            {
                var d = Draw(Bench, seed);
                int n = 0;
                foreach (int i in clutter.Where(i => !d.Parked[i]))
                {
                    n++;
                    onTable[i]++;
                    double r = Math.Sqrt(d.Poses[i].X * d.Poses[i].X + d.Poses[i].Y * d.Poses[i].Y);
                    if (r >= Bench.ReachMin && r <= Bench.ReachMax) inReach++;
                    else beyond++;
                }

                perLayout[n]++;
                total += n;
            }

            string shares = string.Join(", ", clutter.Select(i => FormattableString.Invariant($"{Bench.Objects[i].Name} {onTable[i] / (double)Layouts:0.000}")));
            _output.WriteLine(FormattableString.Invariant($"over {Layouts} layouts: {total / (double)Layouts:0.00} clutter objects a layout ") +
                              $"(layouts with 0-{Bench.ClutterMax}: {string.Join("/", perLayout)}); " +
                              FormattableString.Invariant($"their centres in the reach band {inReach / (double)total:0.000}, beyond it {beyond / (double)total:0.000}; ") +
                              $"on the table in a share of layouts: {shares}");
            // Every piece takes part, and a layout stands about the range's middle (fewer only when no spot is clear: the long tools'
            // footprint circles, 25-29 cm across, fit only near the middle of the 32 x 48 cm region).
            Assert.All(clutter, i => Assert.True(onTable[i] > Layouts / 100, Bench.Objects[i].Name));
            Assert.InRange(total / (double)Layouts, 1.0, Bench.ClutterMax);
            // Some lands in the arm's reach (an obstacle, a possible target), some beyond it (the cameras' foreground).
            Assert.True(inReach > total / 5 && beyond > total / 5, $"{inReach} in reach, {beyond} beyond");
        }
    }
}
