using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text.Json;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit;
using Xunit.Abstractions;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>
    /// The batch that set aside 12 of 25 collide_and_recover takes as "never touched an object" (2026-11-16, batch seed 10),
    /// re-planned from its record (testdata/touch-misses.json): its plans went into the target's box and stopped short of the
    /// object itself; planned against the object's own shape (docs/adr/0037 amendment 5, third revision), the touch reaches it.
    /// </summary>
    public class TouchMissesTests
    {
        private static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;
        private static readonly ArmCollisionModel Model = Profile.Collision!;
        private static readonly DriveEnvelope Envelope = DriveEnvelope.FromModel(Model, 15f * MathF.PI / 180f);
        private readonly ITestOutputHelper _output;

        public TouchMissesTests(ITestOutputHelper output) => _output = output;

        /// <summary>One recorded take: its seed, speed, start and layout, and some of the commands the twin was sent.</summary>
        private sealed class Take
        {
            public string Name = "";
            public ulong Seed;
            public float Speed;
            public ArmPose Start;
            public (byte Id, float[] Pose, float Scale)[] Objects = Array.Empty<(byte, float[], float)>();
            public (double T, ArmPose Pose)[] Commands = Array.Empty<(double, ArmPose)>();
        }

        private static List<Take> Takes(out double seconds)
        {
            using JsonDocument doc = JsonDocument.Parse(File.ReadAllBytes(ObjectShapeTests.RepoFile("core", "Teleop.Eval.Tests", "JointDrive", "testdata", "touch-misses.json")));
            seconds = doc.RootElement.GetProperty("seconds").GetDouble();
            var takes = new List<Take>();
            static float[] Floats(JsonElement a) => a.EnumerateArray().Select(x => x.GetSingle()).ToArray();
            static ArmPose Pose(float[] q, int at) => new ArmPose(q[at], q[at + 1], q[at + 2], q[at + 3], q[at + 4]);
            foreach (JsonElement t in doc.RootElement.GetProperty("takes").EnumerateArray())
            {
                takes.Add(new Take
                {
                    Name = t.GetProperty("take").GetString()!,
                    Seed = t.GetProperty("seed").GetUInt64(),
                    Speed = t.GetProperty("speed").GetSingle(),
                    Start = Pose(Floats(t.GetProperty("start")), 0),
                    Objects = t.GetProperty("objects").EnumerateArray()
                        .Select(o => ((byte)o.GetProperty("id").GetInt32(), Floats(o.GetProperty("pose")), o.GetProperty("scale").GetSingle())).ToArray(),
                    Commands = t.GetProperty("commands").EnumerateArray()
                        .Select(c => { float[] v = Floats(c); return ((double)v[0], Pose(v, 1)); }).ToArray(),
                });
            }

            return takes;
        }

        /// <summary>The take's objects as joint-drive had them: in the scene's order, each from its pose (with its shape, given <paramref name="shapes"/>).</summary>
        private static SafetyObject[] Layout(WorldScene scene, Take take, ObjectShapes? shapes)
        {
            var objects = new List<SafetyObject>();
            foreach (SceneObject o in scene.Objects)
            {
                foreach ((byte id, float[] p, float scale) in take.Objects)
                {
                    if (id == o.Id) objects.Add(WorldView.Bounds(o, scale, p[0], p[1], p[2], p[3], p[4], p[5], p[6], shapes));
                }
            }

            return objects.ToArray();
        }

        /// <summary>How far <paramref name="q"/> is from the nearest point of <paramref name="plan"/>'s path, joint by joint (the largest), rad.</summary>
        private static float OffPath(DrivePlan plan, in ArmPose q)
        {
            float best = float.MaxValue;
            foreach (DriveLeg leg in plan.Legs)
            {
                float[] a = Joints(leg.From), b = Joints(leg.To), x = Joints(q);
                float dot = 0f, len = 0f;
                for (int j = 0; j < 5; j++)
                {
                    dot += (x[j] - a[j]) * (b[j] - a[j]);
                    len += (b[j] - a[j]) * (b[j] - a[j]);
                }

                float u = len > 0f ? Math.Clamp(dot / len, 0f, 1f) : 0f, worst = 0f;
                for (int j = 0; j < 5; j++) worst = MathF.Max(worst, MathF.Abs(x[j] - (a[j] + u * (b[j] - a[j]))));
                best = MathF.Min(best, worst);
            }

            return best;
        }

        private static float[] Joints(in ArmPose p) => new[] { p.BaseYaw, p.Proximal, p.Distal, p.Wrist, p.Roll };

        /// <summary>The deepest <paramref name="plan"/> goes into <paramref name="o"/>'s box, by the collision model (sampled every 10 ms).</summary>
        private static float BoxDepth(DrivePlan plan, in SafetyObject o)
        {
            var ladder = new GapLadder(Profile, Model, o);
            float deepest = float.NegativeInfinity;
            for (double t = 0; t <= plan.TotalSeconds; t += 0.01) deepest = MathF.Max(deepest, -ladder.SignedGap(plan.Sample(t)));
            return deepest;
        }

        [Fact]
        public void EveryRecordedMiss_ReplannedAgainstTheShapes_TouchesTheObjectItself()
        {
            List<Take> takes = Takes(out double seconds);
            WorldScene scene = ObjectShapeTests.Bench(out ObjectShapes shapes);
            var roles = new SafetyOptions { KnockMaxKg = scene.KnockMaxMass }; // as joint-drive plans it (--scene)
            var rows = new List<string> { "take | target | old plan into box | old plan into shape | new target | new plan into shape" };
            int replanned = 0;
            foreach (Take take in takes)
            {
                SafetyObject[] boxes = Layout(scene, take, null), real = Layout(scene, take, shapes);
                Assert.Equal(take.Objects.Length, boxes.Length);

                // The batch's own plan, from its record: every command the twin was sent lies on it (so the record is right).
                Assert.True(DrivePlan.TryBuild(DriveScript.CollideAndRecover, Profile, take.Start, take.Speed, seconds, take.Seed, Envelope,
                    out DrivePlan? old, out string? error, boxes, roles), $"{take.Name}: {error}");
                foreach ((double t, ArmPose q) in take.Commands)
                {
                    float off = OffPath(old!, q);
                    Assert.True(off < 2e-4f, $"{take.Name}: the command at {t:0.000} s is {off:0.00000} rad off the re-planned path");
                }

                // What it did: into its target's box, and not as far into the object itself.
                int target = Enumerable.Range(0, boxes.Length).OrderByDescending(i => BoxDepth(old!, boxes[i])).First();
                float oldBox = BoxDepth(old!, boxes[target]);
                float oldShape = real[target].Shape != null ? ObjectShapeTests.ShapeDepth(old!, real[target], out _) : float.NaN;
                string name = scene.Objects.First(o => o.Id == boxes[target].Id).Name;

                // Planned against the shapes: the object it touches is touched about ShapeTouchDepthMeters in (less for a thin one).
                string now;
                if (DrivePlan.TryBuild(DriveScript.CollideAndRecover, Profile, take.Start, take.Speed, seconds, take.Seed, Envelope,
                        out DrivePlan? plan, out error, real, roles))
                {
                    replanned++;
                    var depths = real.Select(o => o.Shape != null ? ObjectShapeTests.ShapeDepth(plan!, o, out _) : BoxDepth(plan!, o)).ToArray();
                    int touched = Array.IndexOf(depths, depths.Max());
                    string what = scene.Objects.First(o => o.Id == real[touched].Id).Name;
                    now = $"{what} | {depths[touched] * 1000:0.0} mm";
                    Assert.True(depths[touched] >= 0.007f, $"{take.Name}: the new plan goes only {depths[touched] * 1000:0.0} mm into {what}");
                    Assert.True(depths[touched] <= SafetyScript.ShapeTouchDepthMeters + 0.0015f, $"{take.Name}: the new plan pushes {depths[touched] * 1000:0.0} mm into {what}");
                    for (int i = 0; i < real.Length; i++)
                    {
                        if (i != touched) Assert.True(depths[i] < 0f, $"{take.Name}: the new plan touches a second object too");
                    }
                }
                else
                {
                    now = $"none ({error}) |";
                }

                rows.Add($"{take.Name} | {name} | {oldBox * 1000:0.0} mm | {oldShape * 1000:0.0} mm | {now}");
            }

            foreach (string row in rows) _output.WriteLine(row);
            // Every one plans (2026-11-17). Were one not to, the batch would draw another layout rather than drive a miss; but
            // then this test would no longer show that layout touched.
            Assert.True(replanned == takes.Count, string.Join("\n", rows));
        }
    }
}
