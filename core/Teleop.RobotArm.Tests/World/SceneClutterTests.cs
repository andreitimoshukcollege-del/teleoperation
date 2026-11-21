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
        public void APieceOfClutter_IsItsOwnRectangle_InEveryRuleThatKnowsItsYaw()
        {
            // The circle that holds the screwdrivers at any yaw is 29 cm across: in the 32 cm deep region, clear of the robot, it fit
            // only at its very middle, and beside the objects in reach almost never (none in 2000 layouts). Turned along y, the pair
            // is 10 cm deep and stands by the region's far edge.
            int screwdrivers = Index("screwdrivers"), pen = Index("pen"), can = Index("tomato-soup-can"), tape = Index("tape-roll");
            SceneObject s = Bench.Objects[screwdrivers];
            double x = Bench.RegionXMax - s.SizeX / 2 - 0.001;
            Assert.False(Bench.FitsOnTable(screwdrivers, x, 0.0, 1.0, WorldScene.RestUpright));        // its circle: past the far edge
            Assert.True(Bench.FitsOnTable(screwdrivers, x, 0.0, 0.0, 1.0, WorldScene.RestUpright));    // its rectangle, along y
            Assert.False(Bench.FitsOnTable(screwdrivers, x, 0.0, Math.PI / 2, 1.0, WorldScene.RestUpright)); // across: 27 cm deep
            Assert.True(Bench.FitsOnTable(screwdrivers, x, 0.0, Math.PI, 1.0, WorldScene.RestUpright));
            // The robot's keep-out (x up to 0.14 m): a pen along x at 0.20 m reaches 0.13; turned across, it is clear.
            Assert.False(Bench.FitsOnTable(pen, 0.20, 0.0, 0.0, 1.0, WorldScene.RestUpright));
            Assert.True(Bench.FitsOnTable(pen, 0.20, 0.0, Math.PI / 2, 1.0, WorldScene.RestUpright));
            // Against another object's circle: a can 7 cm beside a pen lying along x is 6.5 cm from it, apart; its circle was not.
            Assert.True(Bench.Overlaps(pen, 0.30, 0.0, can, 0.30, 0.07, 1.0, 1.0, 0, 0));
            Assert.False(Bench.Overlaps(pen, 0.30, 0.0, 0.0, can, 0.30, 0.07, 0.7, 1.0, 1.0, 0, 0));
            Assert.True(Bench.Overlaps(pen, 0.30, 0.0, Math.PI / 2, can, 0.30, 0.07, 0.7, 1.0, 1.0, 0, 0)); // turned towards it
            Assert.Equal(Bench.Overlaps(can, 0.30, 0.07, 0.7, pen, 0.30, 0.0, 0.0, 1.0, 1.0, 0, 0), Bench.Overlaps(pen, 0.30, 0.0, 0.0, can, 0.30, 0.07, 0.7, 1.0, 1.0, 0, 0));
            // Two pieces of clutter: two pens side by side, the clearance (1 cm) between them or less.
            Assert.False(Bench.Overlaps(pen, 0.30, 0.0, 0.0, pen, 0.30, 0.0195, 0.0, 1.0, 1.0, 0, 0));
            Assert.True(Bench.Overlaps(pen, 0.30, 0.0, 0.0, pen, 0.30, 0.0185, 0.0, 1.0, 1.0, 0, 0));
            Assert.True(Bench.Overlaps(tape, 0.30, 0.0, 0.3, pen, 0.30, 0.0, 1.2, 1.0, 1.0, 0, 0));
            // Objects that are not clutter keep their circles, yaw or not.
            int mug = Index("mug");
            foreach (double d in new[] { 0.09, 0.10, 0.12, 0.13 })
            {
                Assert.Equal(Bench.Overlaps(can, 0.3, 0.0, mug, 0.3 + d, 0.0, 1.0, 1.0, 0, 1), Bench.Overlaps(can, 0.3, 0.0, 1.0, mug, 0.3 + d, 0.0, -2.0, 1.0, 1.0, 0, 1));
                Assert.Equal(Bench.FitsOnTable(mug, 0.12 + d, 0.2, 1.0, 1), Bench.FitsOnTable(mug, 0.12 + d, 0.2, 0.8, 1.0, 1));
            }
        }

        private static int Index(string name) => Array.FindIndex(Bench.Objects, o => o.Name == name);

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
            // Measured 2026-11-21: 1.21 pieces a layout (layouts with 0-3: 313/1045/560/82), their centres in the reach band 0.126
            // and beyond it 0.874; on the table in a share of layouts: the tape 0.332, the scissors 0.105, the pliers 0.158, the
            // screwdrivers 0.056, the phone 0.226, the pen 0.329. A piece is drawn last, so it finds room where nothing else stands:
            // mostly past the reach band, where the 2-5 movable objects in reach leave the region free. Every piece takes part.
            Assert.All(clutter, i => Assert.True(onTable[i] > Layouts / 50, Bench.Objects[i].Name));
            Assert.InRange(total / (double)Layouts, 0.9, Bench.ClutterMax);
            // Some lands in the arm's reach (an obstacle, a possible target), most beyond it (the cameras' foreground).
            Assert.True(inReach > total / 20 && beyond > total / 2, $"{inReach} in reach, {beyond} beyond");
        }
    }
}
