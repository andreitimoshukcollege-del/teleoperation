using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>joint-drive's safety scripts (docs/adr/0036 amendment 1), planned around the bench scene's objects at their nominal places.</summary>
    public class SafetyScriptTests
    {
        private static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;
        private static readonly ArmCollisionModel Model = Profile.Collision!;
        private static readonly DriveEnvelope Envelope = DriveEnvelope.FromModel(Model, 15f * MathF.PI / 180f);

        // core/Scenes/jetrover-bench.json's objects at their nominal poses: centre and size, metres, arm base frame.
        private static readonly SafetyObject[] Bench =
        {
            Box(1, new Vector3(0.20f, 0.08f, -0.1837f), new Vector3(0.025f, 0.025f, 0.025f)),
            Box(2, new Vector3(0.24f, -0.08f, -0.1512f), new Vector3(0.025f, 0.025f, 0.09f)),
            Box(3, new Vector3(0.17f, -0.02f, -0.1637f), new Vector3(0.035f, 0.035f, 0.065f)),
            Box(4, new Vector3(0.29f, 0.03f, -0.1712f), new Vector3(0.015f, 0.10f, 0.05f)),
        };

        private static SafetyObject Box(byte id, Vector3 centre, Vector3 size) => new SafetyObject(id, centre - size / 2, centre + size / 2);

        private static DrivePlan Plan(DriveScript script, ulong seed = 1)
        {
            Assert.True(DrivePlan.TryBuild(script, Profile, Model.Home, 2.5f, 20, seed, Envelope, out DrivePlan? plan, out string? error, Bench), error);
            return plan!;
        }

        private static IEnumerable<ArmPose> Poses(DrivePlan plan)
        {
            for (double t = 0; t <= plan.TotalSeconds; t += 0.01) yield return plan.Sample(t);
        }

        /// <summary>Whether <paramref name="pose"/> comes within <paramref name="gap"/> of any of <paramref name="objects"/> (gap below 0: that far inside).</summary>
        private static bool Within(in ArmPose pose, IReadOnlyList<SafetyObject> objects, float gap)
        {
            // Inside by -gap on each axis, but never past the box's middle (1 mm short of it), so a small box stays a box.
            var grown = objects.Select(o =>
            {
                if (gap >= 0) return o;
                Vector3 half = (o.Max - o.Min) / 2f, s = Vector3.Min(new Vector3(-gap), half - new Vector3(0.001f));
                return new SafetyObject(o.Id, o.Min + s, o.Max - s);
            }).ToArray();
            return !ArmCollision.IsValid(Profile, SafetyScript.WithObjects(Model, grown, Math.Max(gap, 0f), 0f), pose);
        }

        [Fact]
        public void ReachNoObstacle_KeepsEveryLinkClearOfEveryObject()
        {
            DrivePlan plan = Plan(DriveScript.ReachNoObstacle);
            Assert.True(plan.TotalSeconds > 15);
            Assert.DoesNotContain(Poses(plan), p => Within(p, Bench, SafetyScript.ClearanceMeters - 0.005f));
        }

        [Fact]
        public void StopBeforeCollision_ComesClose_ButNeverTouches()
        {
            DrivePlan plan = Plan(DriveScript.StopBeforeCollision);
            ArmPose[] poses = Poses(plan).ToArray();
            Assert.DoesNotContain(poses, p => Within(p, Bench, 0f));
            Assert.Contains(poses, p => Within(p, Bench, SafetyScript.StopGapMaxMeters + 0.005f));
        }

        [Fact]
        public void CollideAndRecover_TouchesBriefly_ThenIsClearAgain()
        {
            DrivePlan plan = Plan(DriveScript.CollideAndRecover);
            ArmPose[] poses = Poses(plan).ToArray();
            int touching = poses.Count(p => Within(p, Bench, 0f));
            Assert.InRange(touching * 0.01, 0.05, 2.0); // a touch, not a push
            // Never deep: at most TouchDepthMeters past the model's first touch (each box only shrunk to 1 mm short of its middle).
            Assert.DoesNotContain(poses, p => Within(p, Bench, -(SafetyScript.TouchDepthMeters + 0.003f)));
            Assert.False(Within(poses[^1], Bench, 0f));
        }

        [Fact]
        public void CollideWithObstacle_DrivesDeepIntoTheObject()
        {
            DrivePlan plan = Plan(DriveScript.CollideWithObstacle);
            Assert.Contains(Poses(plan), p => Within(p, Bench, -(SafetyScript.TouchDepthMeters + 0.003f))); // deeper than any recovering touch
        }

        [Fact]
        public void EveryScript_IsDeterministic_AndStaysInTheEnvelopeAndModel()
        {
            foreach (DriveScript script in new[] { DriveScript.ReachNoObstacle, DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover, DriveScript.CollideWithObstacle })
            {
                DrivePlan a = Plan(script, 7), b = Plan(script, 7);
                Assert.Equal(a.Legs.Count, b.Legs.Count);
                for (int i = 0; i < a.Legs.Count; i++)
                {
                    Assert.Equal(a.Legs[i].Start, b.Legs[i].Start);
                    Assert.Equal(0f, a.Legs[i].To.MaxJointDelta(b.Legs[i].To));
                }

                foreach (DriveLeg leg in a.Legs.Where(l => l.Phase == LegPhase.Script))
                {
                    Assert.True(Envelope.Contains(leg.To), $"{script}: {DrivePlan.Describe(leg.To)} leaves the envelope");
                    Assert.True(ArmCollision.IsValid(Profile, Model, leg.To), $"{script}: {DrivePlan.Describe(leg.To)} hits the floor, the body or itself");
                }
            }
        }

        [Fact]
        public void OnRandomLayouts_EveryScriptUsuallyFindsAnApproach()
        {
            // RobotHost draws a new layout per take (SceneLayout, the bench scene's region and jitter): each category must plan on
            // most of them. A layout a script cannot use is skipped by the batch, not driven badly.
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !System.IO.File.Exists(System.IO.Path.Combine(dir, "core", "Scenes", "jetrover-bench.json"))) dir = System.IO.Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            var scene = WorldView.LoadScene(System.IO.Path.Combine(dir!, "core", "Scenes", "jetrover-bench.json"), out _);
            var poses = new RigidTransform[scene.Objects.Length];
            const int Layouts = 16; // all 40 of seeds 1-40 planned for every script (2026-11-14); 16 keep the test short
            var planned = new Dictionary<DriveScript, int>();
            var why = new List<string>();
            for (ulong seed = 1; seed <= Layouts; seed++)
            {
                SceneLayout.Draw(scene, seed, poses);
                var objects = scene.Objects.Select((o, i) => WorldView.Bounds(o, 1.0, poses[i].X, poses[i].Y, poses[i].Z, poses[i].Qw, poses[i].Qx, poses[i].Qy, poses[i].Qz)).ToArray();
                foreach (DriveScript script in new[] { DriveScript.ReachNoObstacle, DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover, DriveScript.CollideWithObstacle })
                {
                    if (DrivePlan.TryBuild(script, Profile, Model.Home, 2.5f, 20, seed, Envelope, out _, out string? error, objects))
                        planned[script] = planned.GetValueOrDefault(script) + 1;
                    else if (why.Count < 6) why.Add($"layout {seed} {script}: {error}");
                }
            }

            string report = string.Join(", ", planned.Select(kv => $"{SafetyScript.Category(kv.Key)} {kv.Value}/{Layouts}"));
            foreach (DriveScript script in new[] { DriveScript.ReachNoObstacle, DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover, DriveScript.CollideWithObstacle })
            {
                Assert.True(planned.GetValueOrDefault(script) >= Layouts * 8 / 10, $"{report}; e.g. {string.Join(" | ", why)}");
            }
        }

        [Fact]
        public void ASafetyScript_WithoutTheObjects_IsRefused()
        {
            Assert.False(DrivePlan.TryBuild(DriveScript.StopBeforeCollision, Profile, Model.Home, 2.5f, 20, 1, Envelope, out _, out string? error));
            Assert.Contains("--scene", error);
            Assert.Equal("collide_and_recover", SafetyScript.Category(DriveScript.CollideAndRecover));
        }
    }
}
