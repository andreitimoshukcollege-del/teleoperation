using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit.Abstractions;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>
    /// docs/adr/0038, the scene's third pass (S3): at which heights a mixed layout's obstacles stand where the arm could meet them, over
    /// 2000 layouts of each mixed scene. World-models found the pilot's contacts at one height (the tool point a median 12 cm below the
    /// base frame, 80% within 9 cm), so a model could learn "low means unsafe". The figures in the ADR's "S, scene" section come from
    /// <see cref="Report"/> (run it with <c>--logger "console;verbosity=detailed"</c>). Set <c>TELEOP_MEASURE_SCENES</c> to more scene
    /// files (separated by the path separator) to measure them too, e.g. an earlier commit's.
    /// </summary>
    public class SceneHeightMeasureTests
    {
        /// <summary>The bands' lower edges start here, m in the base frame (the table top is 0.1962 below it), each <see cref="Band"/> tall.</summary>
        internal const double BandFrom = -0.20, Band = 0.05;

        /// <summary>How many bands, up to 0.30 m above the base frame: the wrist reaches 0.30 with the arm straight up.</summary>
        internal const int Bands = 10;

        /// <summary>The most any one band may hold of the obstacles' heights (world-models asked for no 5 cm band above about 40%).</summary>
        internal const double MaxBandShare = 0.40;

        private static readonly JsonSerializerOptions Options = new JsonSerializerOptions { IncludeFields = true };
        private readonly ITestOutputHelper _output;

        public SceneHeightMeasureTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void Report()
        {
            var figures = new Dictionary<string, Figures>();
            foreach (string name in new[] { "jetrover-mixed", "jetrover-mixed-holdout" })
            {
                Figures f = Measure(SceneMixedTests.Load(name), 2000);
                figures[name] = f;
                _output.WriteLine(name + ":\n" + f);
                // Every 5 cm band holds at most 40% of the obstacles' heights (the pilot's scene put 54% in the lowest band, 2026-11-24).
                Assert.True(f.Shares(f.Spread).Max() <= MaxBandShare, $"{name}: a band holds {f.Shares(f.Spread).Max():0.000}");
                // Bars are drawn, stand above the base frame's level or near it, and their posts beyond the obstacles' band.
                Assert.True(f.Bars > 0.15 * f.Units, $"{name}: bars {(double)f.Bars / f.Units:0.000} of the obstacles");
                Assert.Equal(0, f.PostsInBand);
                // Nothing a layout stands meets the arm at home (1 cm): the box where the arm stands between takes keeps it clear.
                Assert.Equal(0, f.HomeObjects);
                Assert.True(f.Missed < 0.3 * f.Layouts, $"{name}: {(double)f.Missed / f.Layouts:0.000} found no spot a layout");
            }

            // The held-out family's bars and stands stand at heights the train family's never do (docs/adr/0038, the held-out split).
            Assert.Empty(figures["jetrover-mixed"].BarHeights.Intersect(figures["jetrover-mixed-holdout"].BarHeights));
            string? more = Environment.GetEnvironmentVariable("TELEOP_MEASURE_SCENES");
            foreach (string path in (more ?? "").Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries))
            {
                WorldScene s = JsonSerializer.Deserialize<SceneDocument>(File.ReadAllText(path), Options)!.ToScene();
                _output.WriteLine(path + ":\n" + Measure(s, 2000));
            }
        }

        internal sealed class Figures
        {
            public int Layouts, LayoutsWithObstacles, Units, Tall, Rigs, Bars, Raised, OnTable, Missed, HomeObjects, HomeLayouts, PostsInBand;
            public readonly double[] Spread = new double[Bands]; // each obstacle's height span, its weight 1 spread over the bands it covers
            public readonly double[] Point = new double[Bands]; // each obstacle's one height: a plank's underside, a tall one's middle, else its top
            public readonly double[] WithFixtures = new double[Bands]; // Spread, with the stands and the supports too
            public readonly int[] BarBearing = new int[7]; // the bar's nearest point to the axis, 15-degree bins from straight ahead
            public readonly SortedSet<int> BarHeights = new SortedSet<int>(); // bars' undersides, mm above the table
            public readonly List<double> BarUnders = new List<double>();

            public double[] Shares(double[] bands)
            {
                double sum = bands.Sum();
                return bands.Select(b => sum > 0 ? b / sum : 0).ToArray();
            }

            private string Row(double[] bands) => string.Join(" ", Shares(bands).Select(v => v.ToString("0.000", CultureInfo.InvariantCulture)));

            public override string ToString()
            {
                var b = new StringBuilder();
                double u = Math.Max(1, Units);
                b.Append(FormattableString.Invariant($"  layouts {Layouts}, with obstacles {LayoutsWithObstacles}; obstacles {Units} ({Units / (double)Layouts:0.00} a layout)\n"));
                b.Append(FormattableString.Invariant($"  share of the obstacles: on the table {OnTable / u:0.000}, raised on a stand {Raised / u:0.000}, tall {Tall / u:0.000}, rigs {Rigs / u:0.000}, bars {Bars / u:0.000}\n"));
                b.Append("  bands, cm from the base frame: ");
                for (int k = 0; k < Bands; k++) b.Append(FormattableString.Invariant($"{(BandFrom + k * Band) * 100:0}..{(BandFrom + (k + 1) * Band) * 100:0} "));
                b.Append('\n');
                b.Append("  obstacles' height spans, share a band: " + Row(Spread) + "\n");
                b.Append("  one height each (a plank's underside, a tall one's middle, else the top): " + Row(Point) + "\n");
                b.Append("  spans with the stands and supports too: " + Row(WithFixtures) + "\n");
                b.Append("  bars' nearest point, 15-degree bins from straight ahead: " + string.Join(" ", BarBearing) + "\n");
                b.Append("  bars' undersides, cm from the base frame: " + string.Join(" ", BarUnders.GroupBy(v => Math.Round(v * 100, 1)).OrderBy(g => g.Key)
                    .Select(g => FormattableString.Invariant($"{g.Key:0.0} ({g.Count()})"))) + "\n");
                b.Append(FormattableString.Invariant($"  found no spot: {Missed / (double)Layouts:0.000} objects a layout; posts inside the obstacles' band: {PostsInBand}\n"));
                b.Append(FormattableString.Invariant($"  placements the arm at home would meet (1 cm): {HomeObjects} objects in {HomeLayouts} layouts"));
                return b.ToString();
            }
        }

        /// <summary>Adds weight 1 spread uniformly over [lo, hi] (base frame, m) to the bands, clipped to them; a point when they meet.</summary>
        private static void Spread(double[] bands, double lo, double hi)
        {
            double top = BandFrom + Bands * Band;
            lo = Math.Max(lo, BandFrom);
            hi = Math.Min(hi, top);
            if (hi <= lo + 1e-9)
            {
                double z = Math.Min(Math.Max(lo, BandFrom), top - 1e-9);
                bands[Math.Min(Bands - 1, Math.Max(0, (int)Math.Floor((z - BandFrom) / Band)))] += 1;
                return;
            }

            for (int k = 0; k < Bands; k++)
            {
                double a = Math.Max(lo, BandFrom + k * Band), c = Math.Min(hi, BandFrom + (k + 1) * Band);
                if (c > a) bands[k] += (c - a) / (hi - lo);
            }
        }

        /// <summary>Over seeds 1 to <paramref name="layouts"/>: see <see cref="Figures"/>.</summary>
        internal static Figures Measure(WorldScene s, int layouts)
        {
            var f = new Figures { Layouts = layouts };
            SceneMixedMeasureTests.ArmBox[] home = SceneMixedMeasureTests.ArmAtHome();
            var poses = new RigidTransform[s.Objects.Length];
            var parked = new bool[s.Objects.Length];
            var scales = new double[s.Objects.Length];
            for (ulong seed = 1; seed <= (ulong)layouts; seed++)
            {
                f.Missed += SceneLayout.Draw(s, seed, poses, parked, scales);
                int units = 0;
                bool homeHit = false;
                for (int i = 0; i < poses.Length; i++)
                {
                    if (parked[i]) continue;
                    SceneObject o = s.Objects[i];
                    int rest = WorldScene.RestOf(poses[i], out double yaw);
                    s.RestExtents(i, rest, out double ex, out double ey, out double ez);
                    double lo = poses[i].Z - ez * scales[i] / 2, hi = poses[i].Z + ez * scales[i] / 2;
                    if (SceneMixedMeasureTests.Meets(home, poses[i].X, poses[i].Y, poses[i].Z, yaw, ex * scales[i] / 2, ey * scales[i] / 2, ez * scales[i] / 2, s.Clearance))
                    {
                        f.HomeObjects++;
                        homeHit = true;
                    }

                    if (o.Role == SceneRole.Stand || o.Role == SceneRole.Support) Spread(f.WithFixtures, lo, hi);
                    if (o.Role == SceneRole.Support && IsBarPost(s, o) && !s.BeyondBand(i, poses[i].X, poses[i].Y, yaw, scales[i])) f.PostsInBand++;
                    if (o.Role != SceneRole.Obstacle && o.Role != SceneRole.Plank) continue;
                    units++;
                    Spread(f.Spread, lo, hi);
                    Spread(f.WithFixtures, lo, hi);
                    bool tall = s.IsTall(i), plank = o.Role == SceneRole.Plank;
                    Spread(f.Point, plank ? lo : tall ? (lo + Math.Min(hi, BandFrom + Bands * Band)) / 2 : hi, plank ? lo : tall ? (lo + Math.Min(hi, BandFrom + Bands * Band)) / 2 : hi);
                    if (plank && o.Overhead)
                    {
                        f.Bars++;
                        f.BarUnders.Add(lo);
                        f.BarHeights.Add((int)Math.Round((lo - s.Table.Top) * 1000));
                        // The bar's line's nearest point to the base's axis: its bearing.
                        double c = Math.Cos(yaw), sn = Math.Sin(yaw), along = poses[i].X * c + poses[i].Y * sn;
                        double nx = poses[i].X - along * c, ny = poses[i].Y - along * sn;
                        f.BarBearing[Math.Min(6, (int)(Math.Abs(Math.Atan2(ny, nx)) / (15 * Math.PI / 180)))]++;
                    }
                    else if (plank) f.Rigs++;
                    else if (tall) f.Tall++;
                    else if (s.IsElevated(i, poses[i].Z, scales[i], rest)) f.Raised++;
                    else f.OnTable++;
                }

                f.Units += units;
                if (units > 0) f.LayoutsWithObstacles++;
                f.HomeLayouts += homeHit ? 1 : 0;
            }

            return f;
        }

        private static bool IsBarPost(WorldScene s, SceneObject support)
        {
            foreach (SceneObject q in s.Objects)
            {
                if (q.Overhead && Array.IndexOf(q.SupportIds, support.Id) >= 0) return true;
            }

            return false;
        }
    }
}
