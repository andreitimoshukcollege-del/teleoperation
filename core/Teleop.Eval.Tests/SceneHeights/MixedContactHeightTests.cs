using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit;
using Xunit.Abstractions;

namespace Teleop.Eval.Tests.SceneHeights
{
    /// <summary>
    /// docs/adr/0038, the scene's third pass (S3): at which heights joint-drive's mixed plans touch a mixed scene's objects by the model,
    /// the dry run's contacts (<c>joint-drive --script mixed --speed 3.8 --seconds 15 --seed 1 --dry-run --layouts a-b</c>, its
    /// <c>MixedDryRun.ModelContacts</c>: every tick of the commanded plan, each object in reach that is not scenery, the twin's links), with
    /// two heights a contact: the tool point's, which the dry run reports, and the touching link's point (the box of the arm that overlaps
    /// the object's box, its centre clamped into the object's box). By default 12 layouts of <c>jetrover-mixed</c>; set
    /// <c>TELEOP_CONTACT_LAYOUTS</c> (e.g. <c>1-200</c>) and <c>TELEOP_CONTACT_SCENES</c> (scene files, separated by the path separator) for
    /// the ADR's figures, with <c>--logger "console;verbosity=detailed"</c>.
    /// </summary>
    public class MixedContactHeightTests
    {
        private const double BandFrom = -0.20, Band = 0.05;
        private const int Bands = 10;
        private readonly ITestOutputHelper _output;

        public MixedContactHeightTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void ContactHeights_ByTheToolPoint_AndByTheTouchingLink()
        {
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !File.Exists(Path.Combine(dir, "core", "Scenes", "jetrover-mixed.json"))) dir = Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            string? scenes = Environment.GetEnvironmentVariable("TELEOP_CONTACT_SCENES");
            string[] paths = string.IsNullOrEmpty(scenes) ? new[] { Path.Combine(dir!, "core", "Scenes", "jetrover-mixed.json") } : scenes.Split(Path.PathSeparator);
            string range = Environment.GetEnvironmentVariable("TELEOP_CONTACT_LAYOUTS") ?? "1-12";
            int from = int.Parse(range.Split('-')[0], CultureInfo.InvariantCulture), to = int.Parse(range.Split('-')[1], CultureInfo.InvariantCulture);
            foreach (string path in paths)
            {
                Heights h = Measure(path, from, to);
                _output.WriteLine(path + ":\n" + h);
                Assert.Equal(h.Onsets, h.Tool.Sum());
                Assert.Equal(h.Onsets, h.Link.Sum());
                Assert.True(h.Planned > 0, "nothing planned");
            }
        }

        private sealed class Heights
        {
            public int Planned, Total, Onsets;
            public readonly int[] Tool = new int[Bands], Link = new int[Bands];
            public readonly Dictionary<string, int> Links = new Dictionary<string, int>(), Kinds = new Dictionary<string, int>();

            private static string Row(int[] v)
            {
                double sum = Math.Max(1, v.Sum());
                return string.Join(" ", v.Select(x => (x / sum).ToString("0.000", CultureInfo.InvariantCulture))) + " (" + string.Join(" ", v) + ")";
            }

            public override string ToString()
            {
                var b = new StringBuilder();
                b.Append(FormattableString.Invariant($"  planned {Planned} of {Total}; contacts by the model: {Onsets} onsets\n"));
                b.Append("  bands, cm from the base frame: ");
                for (int k = 0; k < Bands; k++) b.Append(FormattableString.Invariant($"{(BandFrom + k * Band) * 100:0}..{(BandFrom + (k + 1) * Band) * 100:0} "));
                b.Append('\n');
                b.Append("  the tool point, share a band: " + Row(Tool) + "\n");
                b.Append("  the touching link's point, share a band: " + Row(Link) + "\n");
                b.Append("  touching links: " + string.Join(", ", Links.OrderByDescending(k => k.Value).Select(k => $"{k.Key} {k.Value}")) + "\n");
                b.Append("  touched kinds: " + string.Join(", ", Kinds.OrderByDescending(k => k.Value).Select(k => $"{k.Key} {k.Value}")));
                return b.ToString();
            }
        }

        private static int BandOf(double z) => Math.Min(Bands - 1, Math.Max(0, (int)Math.Floor((z - BandFrom) / Band)));

        private static Heights Measure(string path, int from, int to)
        {
            WorldScene scene = WorldView.LoadScene(path, out _);
            ObjectShapes? shapes = WorldView.ShapesOf(scene);
            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            ArmCollisionModel model = profile.Collision!;
            var envelope = DriveEnvelope.FromModel(model, (float)(15 * Math.PI / 180)); // joint-drive's --margin-deg default
            var safety = new SafetyOptions { KnockMaxKg = scene.KnockMaxMass };
            float table = (float)scene.Table.Top;
            CollisionBody[] bodies = ShapeGap.TouchBodies(model, 0f);
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            var h = new Heights();
            var frames = new LinkFrame[5];
            for (int layout = from; layout <= to; layout++)
            {
                h.Total++;
                SceneLayout.Draw(scene, (ulong)layout, poses, parked, scales);
                var objects = new List<SafetyObject>();
                for (int i = 0; i < scene.Objects.Length; i++)
                {
                    if (parked[i]) continue;
                    SceneObject o = scene.Objects[i];
                    objects.Add(WorldView.Bounds(o, scales[i], poses[i].X, poses[i].Y, poses[i].Z, poses[i].Qw, poses[i].Qx, poses[i].Qy, poses[i].Qz, shapes,
                        (o.Movable && scene.InEdgeRegions(poses[i].X, poses[i].Y)) || o.Role == SceneRole.Distractor));
                }

                // As MixedDryRun: seed 1, the batch's take seed, speed 3.8, 15 s, from home, the default options.
                if (!DrivePlan.TryBuild(DriveScript.Mixed, profile, model.Home, 3.8f, 15, 100000UL + (ulong)layout, envelope, out DrivePlan? plan, out _, objects, safety,
                        new MixedOptions()))
                {
                    continue;
                }

                h.Planned++;
                var arm = new ArmTouch(profile, model);
                var reach = new List<int>();
                for (int i = 0; i < objects.Count; i++)
                {
                    SafetyObject o = objects[i];
                    if (!o.Scenery && new Vector2(o.Center.X, o.Center.Y).Length() - MathF.Max(o.HalfExtents.X, o.HalfExtents.Y) < 0.45f) reach.Add(i);
                }

                var touching = new bool[objects.Count];
                foreach (TrackPoint p in ReplayAdd.Commanded(plan!))
                {
                    arm.At(p.Pose, p.Gripper);
                    foreach (int i in reach)
                    {
                        bool now = arm.Touches(objects[i]);
                        if (now && !touching[i])
                        {
                            h.Onsets++;
                            h.Tool[BandOf(arm.Tool.Z)]++;
                            (string link, double z) = TouchingLink(profile, bodies, frames, p.Pose, objects[i]);
                            h.Link[BandOf(z)]++;
                            h.Links[link] = h.Links.TryGetValue(link, out int n) ? n + 1 : 1;
                            string kind = MixedDryRun.KindOf(objects[i], scene, table);
                            int si = scene.IndexOf(objects[i].Id);
                            if (si >= 0 && scene.Objects[si].Overhead) kind = "bar";
                            h.Kinds[kind] = h.Kinds.TryGetValue(kind, out int m) ? m + 1 : 1;
                        }

                        touching[i] = now;
                    }
                }
            }

            return h;
        }

        /// <summary>
        /// The arm's box that overlaps <paramref name="o"/>'s box at <paramref name="pose"/> (the gripper open: its pads where
        /// <see cref="ShapeGap.TouchBodies"/> puts them), the nearest to the object's centre when more do, and its centre clamped into the
        /// object's axis-aligned bounds: its name and that point's height, base frame. The tool point's height when none overlaps.
        /// </summary>
        private static (string, double) TouchingLink(in RobotArmProfile profile, CollisionBody[] bodies, LinkFrame[] frames, in ArmPose pose, in SafetyObject o)
        {
            ArmCollision.ComputeFrames(profile, pose, frames);
            Vector3 ax = Vector3.Transform(Vector3.UnitX, o.Rotation), ay = Vector3.Transform(Vector3.UnitY, o.Rotation), az = Vector3.Transform(Vector3.UnitZ, o.Rotation);
            Vector3 grow = new Vector3(0.002f); // the touch's own margin: the shapes touch within a millimetre or so
            string best = "none";
            float bestDistance = float.MaxValue;
            double z = double.NaN;
            foreach (CollisionBody b in bodies)
            {
                LinkFrame f = frames[(int)b.Link];
                Quaternion turn = f.Rotation * b.Rotation;
                Vector3 c = f.ToBase(b.Center);
                if (!ArmCollision.BoxesOverlap(c, Vector3.Transform(Vector3.UnitX, turn), Vector3.Transform(Vector3.UnitY, turn), Vector3.Transform(Vector3.UnitZ, turn),
                        b.HalfExtents + grow, o.Center, ax, ay, az, o.HalfExtents)) continue;
                float d = Vector3.Distance(c, o.Center);
                if (d >= bestDistance) continue;
                bestDistance = d;
                best = b.Name;
                z = Math.Min(Math.Max(c.Z, o.Min.Z), o.Max.Z);
            }

            if (double.IsNaN(z))
            {
                Span<LinkFrame> span = frames;
                z = span[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper).Z;
            }

            return (best, z);
        }
    }
}
