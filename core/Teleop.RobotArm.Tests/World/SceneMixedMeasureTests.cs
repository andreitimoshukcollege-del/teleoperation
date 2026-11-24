using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Text;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit.Abstractions;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>
    /// docs/adr/0038, the scene's second pass (S2): what a mixed layout puts where the arm works, measured over 2000 layouts of each
    /// mixed scene, and how far round the arm's IK reaches. The figures in the ADR's "S, scene" section come from <see cref="Report"/>
    /// (run it with <c>--logger "console;verbosity=detailed"</c> to see them).
    /// </summary>
    public class SceneMixedMeasureTests
    {
        /// <summary>An obstacle this tall or taller at its file size, m, is one of the tall thin kinds: the rod, the dowel, the post, the slat.</summary>
        internal const double TallHeight = 0.15;

        /// <summary>
        /// The arm's working band, for the measure: centres 0.16-0.30 m from the base's axis, within 60 degrees of straight ahead, where
        /// the pads reach at many heights and joint-drive's aims may come round the base from either side (MixedTake: beyond 60 degrees
        /// an arc comes from the front only).
        /// </summary>
        internal const double WorkMin = 0.16, WorkMax = 0.30, WorkBearing = 60 * Math.PI / 180;

        /// <summary>
        /// The widest bearing a mixed layout centres an obstacle at, rad, and the measure counts one beyond as out of reach: between the 90
        /// degrees joint-drive's IK reaches from home and the 105 the arm reaches turned that way first (<see cref="IkReach"/>).
        /// </summary>
        internal const double ReachableBearing = 95 * Math.PI / 180;

        private readonly ITestOutputHelper _output;

        public SceneMixedMeasureTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void Report()
        {
            foreach (string name in new[] { "jetrover-mixed", "jetrover-mixed-holdout" })
            {
                Figures f = Measure(SceneMixedTests.Load(name), 2000);
                _output.WriteLine(name + ":\n" + f);
                Assert.True(f.LayoutsWithObstacles > 1000);
                // The second pass (docs/adr/0038), measured 2026-11-23 against the first's 0.163 and 0.179: a tall thin obstacle or a rig in the
                // working band in about half the layouts with obstacles or more (0.592 and 0.627), but not in nearly all of them.
                double share = (double)f.WithTallOrRigInWork / f.LayoutsWithObstacles;
                Assert.InRange(share, 0.5, 0.75);
                // Nothing centred where joint-drive's aims cannot reach (16% and 14% of the obstacles before).
                Assert.Equal(0, f.BeyondReach);
                // Every raised object overhangs its stand all round (no stand was narrower before; the tops were 1.4-43 times the footprint).
                Assert.Equal(f.Raised, f.RaisedNarrower);
                Assert.True(f.StandAreaRatios.Max() < 1);
                Assert.True(f.Missed < 0.3 * f.Layouts, $"{(double)f.Missed / f.Layouts:0.000} found no spot a layout");
            }

            (string table, double home, double turned) = IkReach();
            _output.WriteLine(table);
            _output.WriteLine(FormattableString.Invariant($"widest bearing the IK reaches from home: {home:0.0} deg; with the base turned that way first: {turned:0.0} deg"));
            // Measured 2026-11-23: 90.0 and 105.0 (ArmSolver.MaxApproach is 90 degrees from home; joint-drive's envelope is the yaw's 120 less
            // 15). The obstacles' bearing lies between: an obstacle centred there has its near side where aims reach.
            Assert.Equal(90.0, home, 1);
            Assert.Equal(105.0, turned, 1);
            foreach (WorldScene s in new[] { SceneMixedTests.Mixed, SceneMixedTests.Holdout })
            {
                Assert.InRange(s.ReachBearing * 180 / Math.PI, home, turned);
                Assert.True(s.ReachBearing <= ReachableBearing);
            }
        }

        internal sealed class Figures
        {
            public int Layouts, LayoutsWithObstacles, WithTallOrRig, WithTallOrRigInWork, WithTallOrRigReachable, Units, Tall, Rigs, Raised;
            public int TallInWork, RigsInWork, UnitsInWork, Missed, BeyondReach, Beyond60, RaisedNarrower, HomeLayouts, HomeObjects;
            public readonly int[] BearingBins = new int[8]; // 15 degrees each, from straight ahead
            public readonly int[] UnitCounts = new int[4];
            public readonly List<double> Tops = new List<double>(), TallRigTops = new List<double>(), RaisedMiddles = new List<double>();
            public readonly List<double> StandAreaRatios = new List<double>(), RigHeights = new List<double>(), StandHeights = new List<double>();
            public readonly int[] RigHeightBins = new int[5]; // underside 0-5, 5-10, 10-15, 15-20, 20+ cm above the table

            private static string P(List<double> v, double scale = 100)
            {
                if (v.Count == 0) return "none";
                double[] s = v.OrderBy(x => x).ToArray();
                double Q(double q) => s[Math.Min(s.Length - 1, (int)(q * s.Length))] * scale;
                return FormattableString.Invariant($"{Q(0.05):0.0} / {Q(0.5):0.0} / {Q(0.95):0.0}");
            }

            public override string ToString()
            {
                var b = new StringBuilder();
                double l = LayoutsWithObstacles;
                b.Append(FormattableString.Invariant($"  layouts {Layouts}, with obstacles {LayoutsWithObstacles} (0/1/2/3 obstacles: {string.Join("/", UnitCounts)})\n"));
                b.Append(FormattableString.Invariant($"  with a tall obstacle or a rig: anywhere {WithTallOrRig / l:0.000}, in the working band {WithTallOrRigInWork / l:0.000}, within 95 deg {WithTallOrRigReachable / l:0.000}\n"));
                b.Append(FormattableString.Invariant($"  obstacles {Units} ({(double)Units / Layouts:0.00} a layout): tall {Tall} ({(double)Tall / Units:0.000}), rigs {Rigs} ({(double)Rigs / Units:0.000}), raised {Raised} ({(double)Raised / Units:0.000})\n"));
                b.Append(FormattableString.Invariant($"  in the working band: obstacles {(double)UnitsInWork / Units:0.000}, tall {(Tall > 0 ? (double)TallInWork / Tall : 0):0.000}, rigs {(Rigs > 0 ? (double)RigsInWork / Rigs : 0):0.000}\n"));
                b.Append(FormattableString.Invariant($"  bearing from straight ahead, 15-degree bins: {string.Join(" ", BearingBins)}; beyond 60 deg {(double)Beyond60 / Units:0.000}, beyond 95 deg {(double)BeyondReach / Units:0.000}\n"));
                b.Append($"  obstacle tops above the table, 5/50/95 %: {P(Tops)} cm; tall and rigs: {P(TallRigTops)} cm; raised objects' middles: {P(RaisedMiddles)} cm\n");
                b.Append($"  stands: height {P(StandHeights)} cm; top area over the object's footprint {P(StandAreaRatios, 1)}; ");
                b.Append(FormattableString.Invariant($"narrower than the object both ways {(Raised > 0 ? (double)RaisedNarrower / Raised : 0):0.000}\n"));
                b.Append($"  rigs' undersides above the table: {P(RigHeights)} cm; 0-5/5-10/10-15/15-20/20+ cm: {string.Join("/", RigHeightBins)}\n");
                b.Append(FormattableString.Invariant($"  found no spot: {(double)Missed / Layouts:0.000} objects a layout\n"));
                b.Append(FormattableString.Invariant($"  placements the arm at home would meet (1 cm): {HomeObjects} objects in {HomeLayouts} layouts"));
                return b.ToString();
            }
        }

        private static (RigidTransform[] Poses, bool[] Parked, double[] Scales, int Missed) Draw(WorldScene scene, ulong seed)
        {
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            int missed = SceneLayout.Draw(scene, seed, poses, parked, scales);
            return (poses, parked, scales, missed);
        }

        internal static bool IsTall(SceneObject o) => o.Role == SceneRole.Obstacle && o.SizeZ >= TallHeight - 1e-12;

        private static bool InWork(double x, double y)
        {
            double r = Math.Sqrt(x * x + y * y);
            return r >= WorkMin - 1e-9 && r <= WorkMax + 1e-9 && Math.Abs(Math.Atan2(y, x)) <= WorkBearing + 1e-9;
        }

        /// <summary>Over seeds 1 to <paramref name="layouts"/>: see <see cref="Figures"/>.</summary>
        internal static Figures Measure(WorldScene s, int layouts)
        {
            var f = new Figures { Layouts = layouts };
            ArmBox[] home = ArmAtHome();
            for (ulong seed = 1; seed <= (ulong)layouts; seed++)
            {
                (RigidTransform[] p, bool[] parked, double[] scales, int missed) = Draw(s, seed);
                f.Missed += missed;
                int units = 0;
                bool tallOrRig = false, tallOrRigInWork = false, tallOrRigReachable = false, homeHit = false;
                for (int i = 0; i < p.Length; i++)
                {
                    if (parked[i]) continue;
                    SceneObject o = s.Objects[i];
                    int rest = WorldScene.RestOf(p[i], out double yaw);
                    s.RestExtents(i, rest, out double ex, out double ey, out double ez);
                    if (Meets(home, p[i].X, p[i].Y, p[i].Z, yaw, ex * scales[i] / 2, ey * scales[i] / 2, ez * scales[i] / 2, s.Clearance))
                    {
                        f.HomeObjects++;
                        homeHit = true;
                    }

                    if (o.Role == SceneRole.Stand) f.StandHeights.Add(o.SizeZ * scales[i]);
                    if (o.Role != SceneRole.Obstacle && o.Role != SceneRole.Plank) continue;
                    units++;
                    double top = p[i].Z + ez * scales[i] / 2 - s.Table.Top, bearing = Math.Abs(Math.Atan2(p[i].Y, p[i].X));
                    f.Tops.Add(top);
                    f.BearingBins[Math.Min(7, (int)(bearing / (15 * Math.PI / 180)))]++;
                    f.Beyond60 += bearing > Math.PI / 3 ? 1 : 0;
                    f.BeyondReach += bearing > ReachableBearing ? 1 : 0;
                    bool work = InWork(p[i].X, p[i].Y);
                    f.UnitsInWork += work ? 1 : 0;
                    bool tall = IsTall(o), rig = o.Role == SceneRole.Plank;
                    if (tall || rig)
                    {
                        tallOrRig = true;
                        tallOrRigInWork |= work;
                        tallOrRigReachable |= bearing <= ReachableBearing;
                        f.TallRigTops.Add(top);
                    }

                    if (tall)
                    {
                        f.Tall++;
                        f.TallInWork += work ? 1 : 0;
                    }

                    if (rig)
                    {
                        f.Rigs++;
                        f.RigsInWork += work ? 1 : 0;
                        int a = s.IndexOf(o.SupportIds[0]);
                        double under = s.Objects[a].SizeZ * scales[a];
                        f.RigHeights.Add(under);
                        f.RigHeightBins[Math.Min(4, (int)(under / 0.05))]++;
                    }

                    if (o.Role == SceneRole.Obstacle && s.IsElevated(i, p[i].Z, scales[i], rest))
                    {
                        f.Raised++;
                        f.RaisedMiddles.Add(p[i].Z - s.Table.Top);
                        for (int k = 0; k < p.Length; k++)
                        {
                            if (parked[k] || s.Objects[k].Role != SceneRole.Stand) continue;
                            int restK = WorldScene.RestOf(p[k], out double yawK);
                            if (!s.HoldsOnTop(k, p[k].X, p[k].Y, p[k].Z, yawK, scales[k], restK, i, p[i].X, p[i].Y, p[i].Z, scales[i], rest)) continue;
                            s.RestExtents(k, restK, out double kx, out double ky, out _);
                            kx *= scales[k];
                            ky *= scales[k];
                            double ox = ex * scales[i], oy = ey * scales[i];
                            if (Math.Abs(Math.Cos(yaw - yawK)) < Math.Sqrt(0.5)) (ox, oy) = (oy, ox); // turned a quarter on it
                            f.StandAreaRatios.Add(kx * ky / (ox * oy));
                            f.RaisedNarrower += kx < ox - 1e-9 && ky < oy - 1e-9 ? 1 : 0;
                            break;
                        }
                    }
                }

                f.Units += units;
                f.UnitCounts[Math.Min(3, units)]++;
                if (units == 0) continue;
                f.LayoutsWithObstacles++;
                f.WithTallOrRig += tallOrRig ? 1 : 0;
                f.WithTallOrRigInWork += tallOrRigInWork ? 1 : 0;
                f.WithTallOrRigReachable += tallOrRigReachable ? 1 : 0;
                f.HomeLayouts += homeHit ? 1 : 0;
            }

            return f;
        }

        /// <summary>One of the collision model's arm bodies at a pose: its centre, axes and half extents in the base frame.</summary>
        private readonly struct ArmBox
        {
            public readonly Vector3 Centre, X, Y, Z, Half;

            public ArmBox(Vector3 centre, Vector3 x, Vector3 y, Vector3 z, Vector3 half)
            {
                Centre = centre;
                X = x;
                Y = y;
                Z = z;
                Half = half;
            }
        }

        /// <summary>The JetRover model's bodies with the arm at its home pose (where a layout may find it standing).</summary>
        private static ArmBox[] ArmAtHome()
        {
            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            ArmCollisionModel model = profile.Collision!;
            var frames = new LinkFrame[5];
            ArmCollision.ComputeFrames(profile, model.Home, frames);
            var boxes = new List<ArmBox>();
            foreach (CollisionBody body in model.Bodies)
            {
                if (body.Link == ArmLink.Turntable) continue; // the base turns in place, under the bench's keep-out
                LinkFrame frame = frames[(int)body.Link];
                Quaternion turn = frame.Rotation * body.Rotation;
                boxes.Add(new ArmBox(frame.ToBase(body.Center), Vector3.Transform(Vector3.UnitX, turn), Vector3.Transform(Vector3.UnitY, turn),
                    Vector3.Transform(Vector3.UnitZ, turn), body.HalfExtents));
            }

            return boxes.ToArray();
        }

        /// <summary>Whether a box (centre, yaw about the vertical, half extents) comes within <paramref name="margin"/> of any arm body.</summary>
        private static bool Meets(ArmBox[] arm, double x, double y, double z, double yaw, double hx, double hy, double hz, double margin)
        {
            var c = new Vector3((float)x, (float)y, (float)z);
            var ax = new Vector3((float)Math.Cos(yaw), (float)Math.Sin(yaw), 0f);
            var ay = new Vector3((float)-Math.Sin(yaw), (float)Math.Cos(yaw), 0f);
            var h = new Vector3((float)(hx + margin), (float)(hy + margin), (float)(hz + margin));
            foreach (ArmBox b in arm)
            {
                if (ArmCollision.BoxesOverlap(c, ax, ay, Vector3.UnitZ, h, b.Centre, b.X, b.Y, b.Z, b.Half)) return true;
            }

            return false;
        }

        /// <summary>
        /// How far round the base the headset's IK puts the wrist, as joint-drive asks it (docs/adr/0038, M): wrist targets 8-24 cm from the
        /// axis, 10 cm below to 10 cm above the base frame, pitched 0 to -70 degrees, at each bearing; solved by <see cref="ArmSolver"/> from
        /// the model's home pose (MixedTake solves where an aim meets its object so) and from home turned to that bearing, each kept when it
        /// gets there exactly, inside joint-drive's envelope (each range less 15 degrees) and clear by the model. The table of bearings and
        /// the widest each reaches, degrees.
        /// </summary>
        internal static (string Table, double FromHome, double Turned) IkReach()
        {
            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            ArmCollisionModel model = profile.Collision!;
            const float margin = 15f * MathF.PI / 180f;
            bool InEnvelope(in ArmPose q) =>
                q.BaseYaw >= model.BaseYaw.Min + margin && q.BaseYaw <= model.BaseYaw.Max - margin && q.Proximal >= model.Proximal.Min + margin &&
                q.Proximal <= model.Proximal.Max - margin && q.Distal >= model.Distal.Min + margin && q.Distal <= model.Distal.Max - margin &&
                q.Wrist >= model.Wrist.Min + margin && q.Wrist <= model.Wrist.Max - margin;
            bool Reaches(in ArmPose from, Vector3 target, float pitch)
            {
                var solver = new ArmSolver();
                solver.Reset(profile, from);
                for (int k = 0; k < 40; k++)
                {
                    ArmSolution a = solver.Solve(profile, target, pitch, 0f);
                    if (a.Status == ArmSolveStatus.Approaching) continue;
                    return (a.Status == ArmSolveStatus.Exact || a.Status == ArmSolveStatus.PitchAdjusted) && !a.ReachClamped &&
                           Vector3.Distance(a.Position, target) <= 0.003f && InEnvelope(a.Pose) && ArmCollision.IsValid(profile, model, a.Pose);
                }

                return false;
            }

            var table = new StringBuilder("bearing (deg): wrist targets reached from home / from home turned that way, of 125\n");
            double fromHome = 0, turned = 0;
            for (double deg = 60; deg <= 120 + 1e-9; deg += 2.5)
            {
                float b = (float)(deg * Math.PI / 180);
                ArmPose start = new ArmPose(Math.Clamp(b, model.BaseYaw.Min + margin, model.BaseYaw.Max - margin), model.Home.Proximal, model.Home.Distal,
                    model.Home.Wrist, model.Home.Roll);
                int home = 0, turn = 0;
                for (float r = 0.08f; r <= 0.2401f; r += 0.04f)
                {
                    for (float z = -0.10f; z <= 0.1001f; z += 0.05f)
                    {
                        for (float pitch = 0f; pitch >= -1.2301f; pitch -= 0.3054f)
                        {
                            var target = new Vector3(r * MathF.Cos(b), r * MathF.Sin(b), z);
                            home += Reaches(model.Home, target, pitch) ? 1 : 0;
                            turn += Reaches(start, target, pitch) ? 1 : 0;
                        }
                    }
                }

                table.Append(FormattableString.Invariant($"  {deg:0.0}: {home} / {turn}\n"));
                if (home > 0) fromHome = deg;
                if (turn > 0) turned = deg;
            }

            return (table.ToString(), fromHome, turned);
        }
    }
}
