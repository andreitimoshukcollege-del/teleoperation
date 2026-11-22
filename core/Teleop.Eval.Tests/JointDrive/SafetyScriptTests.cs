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
        private readonly Xunit.Abstractions.ITestOutputHelper _output;

        public SafetyScriptTests(Xunit.Abstractions.ITestOutputHelper output) => _output = output;

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
            // A touch, not a push: easing in, a moment held, easing out (by the model's boxes). Several a take since 2026-11-21
            // (docs/adr/0036 amendment 5): each stretch in contact is one touch, and there is one for each event.
            var stretches = new List<int>();
            for (int i = 0, run = 0; i <= poses.Length; i++)
            {
                if (i < poses.Length && Within(poses[i], Bench, 0f)) run++;
                else if (run > 0)
                {
                    stretches.Add(run);
                    run = 0;
                }
            }

            Assert.NotEmpty(stretches);
            Assert.All(stretches, n => Assert.InRange(n * 0.01, 0.05, 3.0));
            Assert.True(stretches.Count >= plan.Events.Count, $"{stretches.Count} stretches in contact, {plan.Events.Count} touches planned");
            // Never deep: each object at most 2 mm past its own touch depth (TouchDepthMeters, kept 3 mm short of its middle).
            foreach (SafetyObject o in Bench)
            {
                float limit = SafetyScript.TouchTarget(o, SafetyScript.TouchDepthMeters) + 0.002f;
                Assert.DoesNotContain(poses, p => Within(p, new[] { o }, -limit));
            }
            Assert.False(Within(poses[^1], Bench, 0f));
        }

        [Fact]
        public void CollideWithObstacle_DrivesDeepIntoTheObject()
        {
            DrivePlan plan = Plan(DriveScript.CollideWithObstacle);
            Assert.Contains(Poses(plan), p => Within(p, Bench, -0.008f)); // well inside the box: through it, not grazing it
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
            // RobotHost draws a new layout per take (SceneLayout, the bench scene's pool): each category must plan on enough of them.
            // A layout a script cannot use is skipped by the batch, not driven badly.
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !System.IO.File.Exists(System.IO.Path.Combine(dir, "core", "Scenes", "jetrover-bench.json"))) dir = System.IO.Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            var scene = WorldView.LoadScene(System.IO.Path.Combine(dir!, "core", "Scenes", "jetrover-bench.json"), out _);
            var shapes = WorldView.ShapesOf(scene); // the objects' own shapes, which collide_and_recover touches (docs/adr/0037 amendment 5, third revision)
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            const int Layouts = 16; // all 40 of seeds 1-40 planned for every script (2026-11-14); 16 keep the test short
            var planned = new Dictionary<DriveScript, int>();
            var why = new List<string>();
            // docs/adr/0037 "Raised clutter": each category's targets (the object the planned arm first comes within 3.5 cm of), and
            // how many of them are clutter.
            var targets = new Dictionary<DriveScript, int>();
            var clutterTargets = new Dictionary<DriveScript, int>();
            for (ulong seed = 1; seed <= Layouts; seed++)
            {
                SceneLayout.Draw(scene, seed, poses, parked, scales); // a pool's (docs/adr/0037): the objects on the table, at their sizes
                int[] on = Enumerable.Range(0, scene.Objects.Length).Where(i => !parked[i]).ToArray();
                var objects = on.Select(i => WorldView.Bounds(scene.Objects[i], scales[i], poses[i].X, poses[i].Y, poses[i].Z, poses[i].Qw, poses[i].Qx, poses[i].Qy, poses[i].Qz, shapes)).ToArray();
                foreach (DriveScript script in new[] { DriveScript.ReachNoObstacle, DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover, DriveScript.CollideWithObstacle })
                {
                    // As joint-drive plans it (--scene), but for the first event alone: a take plans when its first event does (later
                    // ones that find no approach are left out, docs/adr/0036 amendment 5), and the count is drawn before any of the first event's
                    // draws, so it is the one a take with up to three would plan. Three would only make this test slower.
                    var roles = new SafetyOptions { KnockMaxKg = scene.KnockMaxMass, MaxEvents = 1 };
                    if (DrivePlan.TryBuild(script, Profile, Model.Home, 2.5f, 20, seed, Envelope, out DrivePlan? plan, out string? error, objects, roles))
                    {
                        planned[script] = planned.GetValueOrDefault(script) + 1;
                        if (script == DriveScript.ReachNoObstacle) continue;
                        foreach (int k in Targets(plan!, objects))
                        {
                            targets[script] = targets.GetValueOrDefault(script) + 1;
                            if (scene.Objects[on[k]].Clutter) clutterTargets[script] = clutterTargets.GetValueOrDefault(script) + 1;
                        }
                    }
                    else if (why.Count < 6) why.Add($"layout {seed} {script}: {error}");
                }
            }

            string report = string.Join(", ", planned.Select(kv => $"{SafetyScript.Category(kv.Key)} {kv.Value}/{Layouts}")) + "; clutter targets " +
                            string.Join(", ", targets.Select(kv => $"{SafetyScript.Category(kv.Key)} {clutterTargets.GetValueOrDefault(kv.Key)}/{kv.Value}"));
            // The bars (docs/adr/0037 amendment 5, 2026-11-16). A fixture bigger than the robot stands within the arm's reach
            // beside it every take (the user's choice); it takes approach paths away, and since it is swept along its side it is
            // a target itself. Over 40 layouts the scripts planned: reach 100%, collide_with_obstacle 85%,
            // stop_before_collision 98% (the fixture the target in 17), collide_and_recover 98% (the fixture in 16). A layout a
            // script cannot use costs the batch a few seconds and another layout, never a take.
            var bar = new Dictionary<DriveScript, int>
            {
                [DriveScript.ReachNoObstacle] = Layouts * 8 / 10, [DriveScript.CollideWithObstacle] = Layouts * 6 / 10,
                [DriveScript.StopBeforeCollision] = Layouts * 8 / 10, [DriveScript.CollideAndRecover] = Layouts * 8 / 10,
            };
            foreach (DriveScript script in new[] { DriveScript.ReachNoObstacle, DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover, DriveScript.CollideWithObstacle })
            {
                Assert.True(planned.GetValueOrDefault(script) >= bar[script], $"{report}; e.g. {string.Join(" | ", why)}");
                // Knocking a pen or a phone is realistic; clutter taking most of a category's targets is not. Over seeds 1-40
                // (2026-11-21, docs/adr/0037 "Raised clutter") it was at most 3 of 39 plans' targets in any category: 6 of the 49
                // pieces on those tables stood in reach, and the phone and the pen lie below the arm's 10 mm floor.
                Assert.True(2 * clutterTargets.GetValueOrDefault(script) <= targets.GetValueOrDefault(script), report);
            }

            _output.WriteLine(report);
        }

        /// <summary>
        /// The objects a plan aims at: of <paramref name="objects"/>, the one its script part first brings the arm within 3.5 cm of
        /// (by the collision model's bodies), and any met at that same instant; none when it never comes that near.
        /// </summary>
        private static IEnumerable<int> Targets(DrivePlan plan, IReadOnlyList<SafetyObject> objects)
        {
            double start = plan.Legs.First(l => l.Phase == LegPhase.Script).Start;
            var near = objects.Select(o => SafetyScript.WithObjects(Model, new[] { o }, 0.035f, 0f)).ToArray();
            for (double t = start; t <= plan.TotalSeconds; t += 0.01)
            {
                ArmPose pose = plan.Sample(t);
                int[] met = Enumerable.Range(0, objects.Count).Where(k => !ArmCollision.IsValid(Profile, near[k], pose)).ToArray();
                if (met.Length > 0) return met;
            }

            return Array.Empty<int>();
        }

        [Fact]
        public void ATurnedObject_IsCheckedAsItIsTurned_NotAsItsBounds()
        {
            // 2026-11-16: an object was its axis-aligned bounds, and a touch at a turned jerrycan's corner stopped 10 mm short
            // of it to 5 mm in, never touching in Isaac. A thin slab turned 135 degrees about z, 10 cm ahead of the gripper: its
            // bounds take in the gripper, the slab does not.
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ArmCollision.ComputeFrames(Profile, Model.Home, frames);
            LinkFrame gf = frames[(int)ArmLink.Gripper];
            System.Numerics.Vector3 p = Model.Bodies.Where(b => b.Link == ArmLink.Gripper).Select(b => gf.ToBase(b.Center)).OrderBy(c => c.X + c.Y).Last(); // its foremost part
            var ahead = System.Numerics.Vector3.Normalize(new System.Numerics.Vector3(1f, 1f, 0f)); // ahead of the gripper and to its left
            var turn = System.Numerics.Quaternion.CreateFromAxisAngle(System.Numerics.Vector3.UnitZ, 3f * MathF.PI / 4f); // thin along ahead
            var slab = new SafetyObject(70, p + 0.10f * ahead, turn, new System.Numerics.Vector3(0.12f, 0.005f, 0.04f), movable: false);
            Assert.True(p.X > slab.Min.X && p.X < slab.Max.X && p.Y > slab.Min.Y && p.Y < slab.Max.Y, "the slab's bounds take in the gripper");
            var turned = SafetyScript.WithObjects(Model, new[] { slab }, 0f, 0f);
            Assert.True(ArmCollision.IsValid(Profile, turned, Model.Home), ArmCollision.Check(Profile, turned, Model.Home).Describe(turned));
            var bounds = new SafetyObject(70, slab.Min, slab.Max, movable: false);
            Assert.False(ArmCollision.IsValid(Profile, SafetyScript.WithObjects(Model, new[] { bounds }, 0f, 0f), Model.Home));
            Assert.Equal(slab.HalfExtents.Y + 0.01f, slab.Grown(0.01f).HalfExtents.Y, 5);
        }

        [Fact]
        public void AFixtureTallerThanTheArmReaches_IsMetAlongItsSide()
        {
            // docs/adr/0037 amendment 5: a fixture's top is out of the wrist's reach, so it is swept along its side; seen from
            // the base it is wide and near, so the sweep starts beyond its edge (2026-11-16: neither, and no fixture was ever
            // approached). A 22 x 22 x 50 cm box standing on the table beside the robot, in front of it and off to its side.
            const float Half = 0.11f, Table = -0.1962f, Height = 0.5f;
            foreach ((float bearingDeg, float near) in new[] { (60f, 0.15f), (90f, 0.20f), (90f, 0.28f), (100f, 0.24f), (-80f, 0.20f) })
            {
                float b = bearingDeg * MathF.PI / 180f;
                var c = new System.Numerics.Vector2(MathF.Cos(b), MathF.Sin(b)) * (near + Half);
                var box = new SafetyObject(60, new System.Numerics.Vector3(c.X - Half, c.Y - Half, Table),
                    new System.Numerics.Vector3(c.X + Half, c.Y + Half, Table + Height), movable: false);
                var roles = new SafetyOptions { KnockMaxKg = 0.7 };
                foreach (DriveScript script in new[] { DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover })
                {
                    Assert.True(DrivePlan.TryBuild(script, Profile, Model.Home, 2.5f, 20, 1, Envelope, out _, out string? error, new[] { box }, roles),
                        $"{script}, the box at {bearingDeg} degrees, its near side {near} m out: {error}");
                }

                Assert.False(DrivePlan.TryBuild(DriveScript.CollideWithObstacle, Profile, Model.Home, 2.5f, 20, 1, Envelope, out _, out _, new[] { box }, roles));
            }
        }

        [Fact]
        public void Roles_AimEachCollideScriptOnlyAtObjectsThatCanDoWhatItNeeds()
        {
            // docs/adr/0037 P4: collide_with_obstacle needs an object the arm knocks over (movable, light enough at its size);
            // collide_and_recover prefers one a touch does not tip (a wall, one too heavy, or a squat one), and takes another
            // only when none of those can be approached. With only the other kind in reach, a script plans
            // nothing, and the batch draws another layout.
            SafetyObject Post(double massKg, bool movable = true) // 2.5 x 2.5 x 9 cm: tippy
            {
                SafetyObject b = Bench[1];
                return new SafetyObject(b.Id, b.Min, b.Max, movable, massKg, tippy: true);
            }

            SafetyObject Squat(double massKg) // the post's box, taken as squat: only the role differs (a 2.5 cm cube is too low to touch)
            {
                SafetyObject b = Bench[1];
                return new SafetyObject(b.Id, b.Min, b.Max, true, massKg, tippy: false);
            }

            var roles = new SafetyOptions { KnockMaxKg = 0.2 };
            bool Plans(DriveScript script, SafetyObject o) =>
                DrivePlan.TryBuild(script, Profile, Model.Home, 2.5f, 20, 1, Envelope, out _, out _, new[] { o }, roles);

            Assert.True(Plans(DriveScript.CollideWithObstacle, Post(0.03)));
            Assert.False(Plans(DriveScript.CollideWithObstacle, Post(1.5)));            // too heavy to knock over
            Assert.False(Plans(DriveScript.CollideWithObstacle, Post(0, movable: false))); // a wall
            Assert.True(Plans(DriveScript.CollideAndRecover, Post(1.5)));
            Assert.True(Plans(DriveScript.CollideAndRecover, Post(0, movable: false)));
            Assert.True(Plans(DriveScript.CollideAndRecover, Post(0.03)));              // only a tippy one in reach: tried after all
            Assert.True(Plans(DriveScript.CollideAndRecover, Squat(0.03)));             // light but squat: it slides
            // No roles (a scene without knockMaxMass): collide_with_obstacle takes any movable object, as before.
            Assert.True(DrivePlan.TryBuild(DriveScript.CollideWithObstacle, Profile, Model.Home, 2.5f, 20, 1, Envelope, out _, out _, new[] { Post(1.5) }, new SafetyOptions()));
        }

        [Fact]
        public void EveryScript_DragsTheWrist_AlongAContinuousPath()
        {
            // Teleop is a dragged point: a target every 48 Hz tick along a continuous path, never a jump from pose to pose
            // (the user, 2026-11-15). The plan's script part, sampled as joint-drive sends it, never moves the wrist faster
            // than a hand does (HandModel.MaxWristSpeed), and all of its motion comes as dense ticks rather than long moves.
            foreach (DriveScript script in new[] { DriveScript.ReachNoObstacle, DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover, DriveScript.CollideWithObstacle })
            {
                DrivePlan plan = Plan(script, 3);
                var script_ = plan.Legs.Where(l => l.Phase == LegPhase.Script).ToList();
                double moving = script_.Where(l => l.Duration > 0).Sum(l => l.Duration);
                double inTicks = script_.Where(l => l.Duration > 0 && Math.Abs(l.Duration - HandModel.TickSeconds) < 1e-9).Sum(l => l.Duration);
                Assert.True(inTicks / moving > 0.99, $"{script}: {inTicks:0.0} of {moving:0.0} s moving is one target a tick");
                double t0 = script_[0].Start, t1 = script_[^1].End;
                Vector3 last = HandModel.Wrist(Profile, plan.Sample(t0));
                float worst = 0f;
                for (double t = t0 + HandModel.TickSeconds; t <= t1; t += HandModel.TickSeconds)
                {
                    Vector3 w = HandModel.Wrist(Profile, plan.Sample(t));
                    worst = Math.Max(worst, Vector3.Distance(w, last));
                    last = w;
                }

                Assert.True(worst <= HandModel.MaxWristSpeed * HandModel.TickSeconds + 0.0005f, $"{script}: the wrist jumped {worst * 1000:0.0} mm in a tick");
            }
        }

        [Fact]
        public void TheHand_MovesContinuously_InOverlappingSubmovements()
        {
            // docs/adr/0036 amendment 3 (the user, 2026-11-16): the motion flows. The hand never lets go of the clutch mid-take,
            // is almost always moving, and a stillness lasts at most about half a second. The submovements still overlap as a
            // person's do (amendment 2, below), at the speed people move.
            // People at the headset (2026-11-15, 31 sessions, docs/adr/0036 amendment 2) chain overlapping submovements: 5.4
            // speed peaks a bout, a quarter of bouts with a single one; they let go of the clutch while still moving (75% of
            // holds begin with a step faster than 30 mm/s); they hold it let go 27% of the time, and move the wrist at ~98 mm/s
            // (median, while moving). The script before (one minimum-jerk bell to rest, then the next) had 1.1 peaks a bout, 88%
            // single-peak bouts, no release at speed and 32 mm/s. The bands here are loose: the measured ones are the analysis's.
            var hand = new HumanOperator(Profile, Envelope, Model.Home, 11);
            Assert.True(hand.FreeUntil(180, Model));
            IReadOnlyList<HandCommand> c = hand.Commands;
            int n = c.Count;
            double dt = HandModel.TickSeconds;
            var step = new double[n];
            for (int k = 1; k < n; k++) step[k] = Vector3.Distance(c[k].Target, c[k - 1].Target) / dt;
            double[] speed = Smooth(step, 0.040 / dt);

            int holds = 0, atSpeed = 0, held = 0;
            for (int k = 1; k < n; k++)
            {
                if (c[k].Held) held++;
                if (c[k].Held && !c[k - 1].Held)
                {
                    holds++;
                    if (step[k - 1] > 0.030) atSpeed++;
                }
            }

            var peaks = new List<int>();
            var inMotion = new List<double>();
            for (int k = 0; k < n;)
            {
                if (c[k].Held || speed[k] < 0.010) { k++; continue; }
                int end = k;
                while (end < n && !c[end].Held && speed[end] >= 0.005) end++;
                double top = 0;
                for (int i = k; i < end; i++) top = Math.Max(top, speed[i]);
                if (end - k >= 5) peaks.Add(Peaks(speed, k, end, Math.Max(0.1 * top, 0.003)));
                for (int i = k; i < end; i++) inMotion.Add(speed[i]);
                k = end;
            }

            inMotion.Sort();
            double median = inMotion[inMotion.Count / 2];
            double perBout = peaks.Average(), single = peaks.Count(p => p <= 1) / (double)peaks.Count;
            string report = $"{peaks.Count} bouts, {perBout:0.0} peaks a bout, {single:P0} single-peak; {atSpeed}/{holds} holds entered at speed; held {held * dt / (n * dt):P0}; median speed moving {median * 1000:0} mm/s";
            int still = 0, longest = 0, run = 0;
            for (int k = 0; k < n; k++)
            {
                run = speed[k] < 0.005 ? run + 1 : 0;
                if (run > 0) still++;
                longest = Math.Max(longest, run);
            }

            report += $"; still {still / (double)n:P0}, longest stillness {longest * dt:0.00} s";
            Assert.True(perBout >= 2.0, report);
            Assert.True(single <= 0.5, report);
            Assert.True(holds == 0 && held == 0, report);            // the clutch is never let go mid-take
            Assert.True(still <= 0.10 * n, report);                  // almost always moving
            Assert.True(longest * dt <= 0.7, report);                // a pause is brief (at most 0.5 s, plus the smoothing)
            Assert.True(median >= 0.045 && median <= 0.2, report);
        }

        /// <summary>A Gaussian moving average with an sd of <paramref name="sigma"/> samples.</summary>
        private static double[] Smooth(double[] x, double sigma)
        {
            int r = (int)Math.Ceiling(3 * sigma);
            var w = Enumerable.Range(-r, 2 * r + 1).Select(i => Math.Exp(-0.5 * i * i / (sigma * sigma))).ToArray();
            var y = new double[x.Length];
            for (int k = 0; k < x.Length; k++)
            {
                double s = 0, ws = 0;
                for (int i = -r; i <= r; i++)
                {
                    int j = k + i;
                    if (j < 0 || j >= x.Length) continue;
                    s += w[i + r] * x[j];
                    ws += w[i + r];
                }

                y[k] = s / ws;
            }

            return y;
        }

        /// <summary>The peaks of <paramref name="x"/> in [from, to): each rising by <paramref name="h"/> from the valley before it and falling by it after.</summary>
        private static int Peaks(double[] x, int from, int to, double h)
        {
            int count = 0;
            double valley = x[from], top = x[from];
            bool rising = true;
            for (int i = from; i < to; i++)
            {
                if (rising)
                {
                    top = Math.Max(top, x[i]);
                    if (top - valley >= h && top - x[i] >= h)
                    {
                        count++;
                        rising = false;
                        valley = x[i];
                    }
                }
                else
                {
                    valley = Math.Min(valley, x[i]);
                    if (x[i] - valley >= h)
                    {
                        rising = true;
                        top = x[i];
                    }
                }
            }

            if (rising && top - valley >= h) count++; // the bout ended on a peak (cut off by a release)
            return Math.Max(count, 1);
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
