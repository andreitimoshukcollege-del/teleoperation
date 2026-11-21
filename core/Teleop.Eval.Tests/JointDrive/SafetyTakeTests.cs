using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;
using Xunit;
using Xunit.Abstractions;
using static Teleop.Eval.Tests.JointDrive.Takes;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>What the tests of docs/adr/0036 amendment 5 share: the arm, the bench at its nominal poses, layouts, and a take driven through its knocks.</summary>
    internal static class Takes
    {
        public static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;
        public static readonly ArmCollisionModel Model = Profile.Collision!;
        public static readonly DriveEnvelope Envelope = DriveEnvelope.FromModel(Model, 15f * MathF.PI / 180f);
        public static readonly DriveScript[] EventScripts = { DriveScript.StopBeforeCollision, DriveScript.CollideAndRecover, DriveScript.CollideWithObstacle };
        public const double Tick = HandModel.TickSeconds;

        // core/Scenes/jetrover-bench.json's first four objects at their nominal poses, as SafetyScriptTests has them.
        public static readonly SafetyObject[] Bench =
        {
            Box(1, new Vector3(0.20f, 0.08f, -0.1837f), new Vector3(0.025f, 0.025f, 0.025f)),
            Box(2, new Vector3(0.24f, -0.08f, -0.1512f), new Vector3(0.025f, 0.025f, 0.09f)),
            Box(3, new Vector3(0.17f, -0.02f, -0.1637f), new Vector3(0.035f, 0.035f, 0.065f)),
            Box(4, new Vector3(0.29f, 0.03f, -0.1712f), new Vector3(0.015f, 0.10f, 0.05f)),
        };

        public static SafetyObject Box(byte id, Vector3 centre, Vector3 size) => new SafetyObject(id, centre - size / 2, centre + size / 2);

        public static DrivePlan Plan(DriveScript script, IReadOnlyList<SafetyObject> objects, ulong seed, SafetyOptions? options = null, ArmPose? start = null)
        {
            Assert.True(DrivePlan.TryBuild(script, Profile, start ?? Model.Home, 2.5f, 15, seed, Envelope, out DrivePlan? plan, out string? error, objects, options), error);
            return plan!;
        }

        /// <summary>Whether <paramref name="pose"/> comes within <paramref name="gap"/> of any of <paramref name="objects"/> (below 0: that far inside, never past a box's middle).</summary>
        public static bool Within(in ArmPose pose, IReadOnlyList<SafetyObject> objects, float gap)
        {
            var grown = objects.Select(o => gap >= 0 ? o : o.Grown(Math.Max(gap, -SafetyScript.MaxDepth(o)))).ToArray();
            return !ArmCollision.IsValid(Profile, SafetyScript.WithObjects(Model, grown, Math.Max(gap, 0f), 0f), pose);
        }

        public static WorldScene BenchScene(out ObjectShapes? shapes)
        {
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !System.IO.File.Exists(System.IO.Path.Combine(dir, "core", "Scenes", "jetrover-bench.json"))) dir = System.IO.Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            WorldScene scene = WorldView.LoadScene(System.IO.Path.Combine(dir!, "core", "Scenes", "jetrover-bench.json"), out _);
            shapes = WorldView.ShapesOf(scene);
            return scene;
        }

        /// <summary>RobotHost's layout <paramref name="seed"/> of the bench's pool: the objects on the table, as joint-drive sees them.</summary>
        public static SafetyObject[] Layout(WorldScene scene, ObjectShapes? shapes, ulong seed)
        {
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            SceneLayout.Draw(scene, seed, poses, parked, scales);
            return scene.Objects.Select((o, i) => (o, i)).Where(x => !parked[x.i])
                .Select(x => WorldView.Bounds(x.o, scales[x.i], poses[x.i].X, poses[x.i].Y, poses[x.i].Z, poses[x.i].Qw, poses[x.i].Qx, poses[x.i].Qy, poses[x.i].Qz, shapes))
                .ToArray();
        }

        /// <summary>The knocked object as a knock may leave it, for the planner: sent 8 cm along the swipe and over on its side, on the table.</summary>
        public static SafetyObject KnockedOver(in SafetyObject o, Vector2 way)
        {
            Vector3 w = way.Length() > 1e-6f ? Vector3.Normalize(new Vector3(way.X, way.Y, 0f)) : Vector3.UnitY;
            Quaternion q = Quaternion.Normalize(Quaternion.Concatenate(o.Rotation, Quaternion.CreateFromAxisAngle(Vector3.Normalize(Vector3.Cross(Vector3.UnitZ, w)), MathF.PI / 2f)));
            Vector3 c = o.Center + 0.08f * w;
            var probe = new SafetyObject(o.Id, c, q, o.HalfExtents, o.Movable, o.Mass, false, o.Shape);
            return new SafetyObject(o.Id, c + new Vector3(0f, 0f, o.Min.Z - probe.Min.Z), q, o.HalfExtents, o.Movable, o.Mass, false, o.Shape);
        }

        /// <summary>
        /// The whole of a take as joint-drive drives it: the plan, and after each knock the objects as it left them (the knocked one
        /// over on its side, <see cref="KnockedOver"/>) and the next event planned against them once they have settled, the planner
        /// given joint-drive's budget. The plans it went through, last the one driven to the end.
        /// </summary>
        public static List<DrivePlan> Drive(DrivePlan plan, IReadOnlyList<SafetyObject> objects, double settleSeconds = 2.0)
        {
            var plans = new List<DrivePlan> { plan };
            var now = objects.ToList();
            while (plan.Pending != null)
            {
                SafetyContinuation c = plan.Pending;
                int k = now.FindIndex(o => o.Id == c.Knocked);
                now[k] = KnockedOver(now[k], c.KnockWay);
                double at = c.SwitchAfter(c.KnockEnd + settleSeconds + 4.0);
                if (double.IsNaN(at)) break;
                DrivePlan next = c.Continue(now, at, out _);
                for (double t = 0; t < at; t += 0.05) Assert.Equal(0f, plan.Sample(t).MaxJointDelta(next.Sample(t))); // the same until it takes over
                plan = next;
                plans.Add(plan);
            }

            return plans;
        }
    }

    /// <summary>
    /// docs/adr/0036 amendment 5 over RobotHost's random layouts of the bench: how many events, where in the take, how long the take.
    /// The rows and the summary go to the test's output (a TRX logger shows them).
    /// </summary>
    public class SafetyTakeLayoutTests
    {
        private readonly ITestOutputHelper _output;

        public SafetyTakeLayoutTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void OnRandomLayouts_EventsFollowTheDraw_AndLandAnywhereInTheTake()
        {
            // RobotHost draws a new layout per take (the bench's pool). Each event category draws 1-3 events (uniform); each lands
            // after free movement (the lead, or between events) and the take ends after a tail, never inside one. A knock's next
            // event is planned against the objects as the knock left them (here, its object over on its side 8 cm on).
            WorldScene scene = BenchScene(out ObjectShapes? shapes);
            var options = new SafetyOptions { KnockMaxKg = scene.KnockMaxMass };
            const int Layouts = 8;
            var drawn = new List<int>();
            var planned = new List<int>();
            var lengths = new List<double>();
            var where = new List<double>(); // each contact as a share of its take
            var firstAt = new List<double>(); // the first event's contact, seconds into the take
            var rows = new List<string>();
            int unplanned = 0;
            var watch = new Stopwatch();
            foreach (DriveScript script in EventScripts)
            {
                for (ulong seed = 1; seed <= Layouts; seed++)
                {
                    SafetyObject[] objects = Layout(scene, shapes, seed);
                    watch.Restart();
                    if (!DrivePlan.TryBuild(script, Profile, Model.Home, 3.8f, 15, 100 * seed + (ulong)script, Envelope, out DrivePlan? first, out string? error, objects, options))
                    {
                        unplanned++;
                        rows.Add($"{SafetyScript.Category(script)} layout {seed}: not planned ({error})");
                        continue;
                    }

                    double firstSeconds = watch.Elapsed.TotalSeconds;
                    List<DrivePlan> plans = Drive(first!, objects);
                    double allSeconds = watch.Elapsed.TotalSeconds;
                    DrivePlan plan = plans[plans.Count - 1];
                    IReadOnlyList<SafetyEvent> events = plan.Events;
                    double start = plan.ScriptStart, length = plan.TotalSeconds - start;
                    Assert.InRange(plan.EventsDrawn, options.MinEvents, options.MaxEvents);
                    Assert.InRange(events.Count, 1, plan.EventsDrawn);
                    Assert.Equal(0.0, start); // from home, clear of the objects: no move home first
                    Assert.Null(plan.Pending);
                    // Not in the lead: the first approach comes after at least the shortest lead.
                    Assert.True(events[0].ApproachStart >= start + options.LeadMinSeconds - Tick, $"{script} {seed}: {events[0]}");
                    for (int i = 0; i < events.Count; i++)
                    {
                        SafetyEvent e = events[i];
                        Assert.Equal(i + 1, e.Index);
                        Assert.Equal(script == DriveScript.StopBeforeCollision ? ScriptEventKind.Stop : script == DriveScript.CollideAndRecover ? ScriptEventKind.Touch : ScriptEventKind.Knock, e.Kind);
                        Assert.True(e.ApproachStart <= e.FinalStart && e.FinalStart <= e.Contact && e.Contact <= e.End && e.End <= e.Clear + 1e-9, e.ToString());
                        if (i > 0)
                        {
                            // Free movement between: at least the shortest stretch (after a knock, until the objects settle and the next is planned).
                            Assert.True(e.ApproachStart - events[i - 1].Clear >= options.BetweenMinSeconds - Tick, $"{script} {seed}: {events[i - 1]} then {e}");
                        }

                        where.Add((e.Contact - start) / length);
                    }

                    // Not in the tail, and the cut never inside an event: free movement after the last for at least the shortest tail.
                    Assert.True(plan.TotalSeconds >= events[events.Count - 1].Clear + options.TailMinSeconds - Tick, $"{script} {seed}: ends at {plan.TotalSeconds:0.00} s, {events[events.Count - 1]}");
                    Assert.InRange(length, options.MinTakeSeconds - Tick, options.MaxTakeSeconds + 1.0);
                    drawn.Add(plan.EventsDrawn);
                    planned.Add(events.Count);
                    lengths.Add(length);
                    firstAt.Add(events[0].Contact - start);
                    rows.Add(FormattableString.Invariant(
                        $"{SafetyScript.Category(script)} layout {seed}: {events.Count} of {plan.EventsDrawn} events, take {length:0.0} s, contacts at {string.Join(", ", events.Select(e => $"{e.Contact - start:0.0} s ({(e.Contact - start) / length:P0}) on {e.ObjectId}"))}; planned in {firstSeconds:0.0} s (all {allSeconds:0.0} s, {plans.Count} plan(s))"));
                }
            }

            foreach (string row in rows) _output.WriteLine(row);
            where.Sort();
            lengths.Sort();
            string summary = FormattableString.Invariant(
                $"{planned.Count} takes ({unplanned} not planned): drawn 1/2/3 {drawn.Count(d => d == 1)}/{drawn.Count(d => d == 2)}/{drawn.Count(d => d == 3)}, planned 1/2/3 {planned.Count(d => d == 1)}/{planned.Count(d => d == 2)}/{planned.Count(d => d == 3)}; ") +
                FormattableString.Invariant($"take {lengths[0]:0.0}-{lengths[lengths.Count - 1]:0.0} s (median {lengths[lengths.Count / 2]:0.0}); contacts at {where[0]:P0}-{where[where.Count - 1]:P0} of the take (median {where[where.Count / 2]:P0}), the first {firstAt.Min():0.0}-{firstAt.Max():0.0} s in");
            _output.WriteLine(summary);
            // Several events a take, and not only at the end: two thirds of the takes draw more than one; most plan all they draw.
            Assert.True(planned.Count(p => p >= 2) >= planned.Count / 3, summary);
            Assert.True(planned.Sum() >= 0.75 * drawn.Sum(), summary);
            Assert.True(where[where.Count / 2] < 0.7, summary);
            Assert.True(where.Count(w => w < 0.5) >= where.Count / 4, summary);
            Assert.True(unplanned <= Layouts, summary);
        }
    }

    /// <summary>
    /// docs/adr/0036 amendment 5 (the user, 2026-11-21): "the collisions always happen at the end of the video and there is only
    /// one; remove the idle state in the first few seconds; collide multiple times in one video". Measured on the 16 takes of the
    /// 2026-11-21 check: 3.1-3.9 s of a still arm at every take's start, and one contact a take, at 81-92% of it.
    /// </summary>
    public class SafetyTakeTests
    {
        [Theory]
        [InlineData(DriveScript.StopBeforeCollision)]
        [InlineData(DriveScript.CollideAndRecover)]
        public void FreeMovement_BeforeBetweenAndAfterEvents_KeepsClearOfEveryObject(DriveScript script)
        {
            // Three events on the bench at its nominal poses: between and after them the arm moves freely and touches nothing
            // (by the collision model, the approach clearance less 2 mm); the lead keeps the scripts' clearance as before.
            var three = new SafetyOptions { MinEvents = 3, MaxEvents = 3 };
            int checkedTakes = 0;
            for (ulong seed = 1; seed <= 3; seed++)
            {
                DrivePlan plan = Plan(script, Bench, seed, three);
                IReadOnlyList<SafetyEvent> events = plan.Events;
                Assert.Equal(3, plan.EventsDrawn);
                Assert.True(events.Count >= 2, $"seed {seed}: {events.Count} events");
                checkedTakes++;
                for (double t = plan.ScriptStart; t < events[0].ApproachStart; t += Tick)
                {
                    Assert.False(Within(plan.Sample(t), Bench, SafetyScript.ClearanceMeters - 0.005f), $"seed {seed}: the lead at {t:0.00} s");
                }

                for (int i = 0; i < events.Count; i++)
                {
                    double to = i + 1 < events.Count ? events[i + 1].ApproachStart : plan.TotalSeconds;
                    for (double t = events[i].Clear; t < to; t += Tick)
                    {
                        Assert.False(Within(plan.Sample(t), Bench, SafetyScript.ApproachClearanceMeters - 0.002f), $"seed {seed}: free movement after event {i + 1} at {t:0.00} s");
                    }
                }

                // The events themselves do what the category says: a stop never touches, a touch touches.
                bool touched = Enumerable.Range(0, (int)(plan.TotalSeconds / 0.01)).Any(k => Within(plan.Sample(k * 0.01), Bench, 0f));
                Assert.Equal(script == DriveScript.CollideAndRecover, touched);
            }

            Assert.Equal(3, checkedTakes);
        }

        [Fact]
        public void SuccessiveEvents_AimAtDifferentObjects_WhenThereAreSeveral()
        {
            // Three posts in reach, each one a touch can meet: three touches go to the three of them, not one post three times.
            // (An object already aimed at is only tried once no other works: on the bench, whose 2.5 cm cube is too low to touch,
            // a third touch can go back to one of the other three.)
            SafetyObject Post(byte id, float bearingDeg)
            {
                float b = bearingDeg * MathF.PI / 180f;
                var c = new Vector3(0.23f * MathF.Cos(b), 0.23f * MathF.Sin(b), -0.1512f);
                return Box(id, c, new Vector3(0.025f, 0.025f, 0.09f));
            }

            SafetyObject[] posts = { Post(11, -35f), Post(12, 0f), Post(13, 35f) };
            var three = new SafetyOptions { MinEvents = 3, MaxEvents = 3 };
            var report = new List<string>();
            int distinct = 0, events = 0;
            for (ulong seed = 1; seed <= 2; seed++)
            {
                DrivePlan plan = Plan(DriveScript.CollideAndRecover, posts, seed, three);
                events += plan.Events.Count;
                distinct += plan.Events.Select(e => e.ObjectId).Distinct().Count();
                report.Add(string.Join(", ", plan.Events.Select(e => e.ObjectId)));
            }

            Assert.True(events >= 4 && distinct == events, string.Join(" | ", report));
        }

        [Theory]
        [InlineData(DriveScript.ReachNoObstacle)]
        [InlineData(DriveScript.StopBeforeCollision)]
        [InlineData(DriveScript.CollideAndRecover)]
        [InlineData(DriveScript.CollideWithObstacle)]
        public void TheTake_BeginsWithTheArmMoving(DriveScript script)
        {
            // The 2026-11-21 check's takes began with 3.1-3.9 s of a still arm: the 3 s countdown inside the recording and a
            // reaction time of 0.15-0.45 s. The countdown is before the take now, and the operator is already moving as it starts:
            // from the script's first tick a joint is 0.5 degrees from where it began (take_timing.py's measure) within 0.3 s.
            var first = new SafetyOptions { MaxEvents = 1 }; // later events do not change how a take begins
            foreach (ArmPose? start in new ArmPose?[] { null, new ArmPose(0.3f, 1.2f, -1.2f, 0.2f, 0.1f) })
            {
                for (ulong seed = 1; seed <= 3; seed++)
                {
                    DrivePlan plan = Plan(script, Bench, seed, first, start);
                    ArmPose at = plan.Sample(plan.ScriptStart);
                    double moved = double.NaN;
                    for (double t = plan.ScriptStart; t <= plan.ScriptStart + 1.0; t += Tick / 4)
                    {
                        if (plan.Sample(t).MaxJointDelta(at) <= 0.5 * Math.PI / 180) continue;
                        moved = t - plan.ScriptStart;
                        break;
                    }

                    Assert.True(moved <= 0.3, $"{script} seed {seed} from {(start == null ? "home" : "a pose not clear")}: still for {moved:0.00} s after the script starts at {plan.ScriptStart:0.00} s");
                    // From a pose not clear of the objects the plan moves home first; the take starts after it, the arm still.
                    if (start != null && plan.ScriptStart > 0) Assert.True(plan.Sample(plan.ScriptStart - 0.01).MaxJointDelta(Model.Home) < 1e-4f);
                }
            }
        }

        [Fact]
        public void TheOneEventPlanner_IsStillThere_ForWhatWasRecordedWithIt()
        {
            // TouchMissesTests replans the takes of 2026-11-16 from their records with it: one event, at the end.
            var old = new SafetyOptions { OneEventAtTheEnd = true };
            DrivePlan plan = Plan(DriveScript.CollideAndRecover, Bench, 1, old);
            Assert.Empty(plan.Events);
            Assert.Equal(0, plan.EventsDrawn);
            Assert.Null(plan.Pending);
        }

        [Fact]
        public void TheEventCount_IsAnOption()
        {
            var o = new SafetyOptions();
            JointDriveCommand.ParseEvents("2", o);
            Assert.Equal((2, 2), (o.MinEvents, o.MaxEvents));
            JointDriveCommand.ParseEvents("1-3", o);
            Assert.Equal((1, 3), (o.MinEvents, o.MaxEvents));
            Assert.Throws<ArgumentException>(() => JointDriveCommand.ParseEvents("x", o));
            Assert.Throws<ArgumentException>(() => JointDriveCommand.ParseEvents("1-2-3", o));
            o.MinEvents = 3;
            o.MaxEvents = 2;
            Assert.Throws<ArgumentException>(() => o.Validate());
            var two = new SafetyOptions { MinEvents = 2, MaxEvents = 2 };
            Assert.Equal(2, Plan(DriveScript.StopBeforeCollision, Bench, 4, two).EventsDrawn);
        }

        [Fact]
        public void RestWatch_WaitsUntilEveryObjectHasStayedPut()
        {
            WorldScene scene = BenchScene(out _);
            var world = new WorldState(scene, new byte[32]);
            var rest = new RestWatch(0.6);
            void Put(double x)
            {
                foreach (SceneObject o in scene.Objects)
                {
                    world.Apply(new ObjectPose(o.Id, 0, o.Id == scene.Objects[0].Id ? (float)x : 0.2f, 0f, -0.15f, 1f, 0f, 0f, 0f), 1);
                }
            }

            Assert.False(rest.Still(world, 0.0)); // nothing posed yet
            Put(0.20);
            Assert.False(rest.Still(world, 0.1));
            Assert.False(rest.Still(world, 0.5));
            Assert.True(rest.Still(world, 0.71));
            Put(0.2005); // half a millimetre: still at rest
            Assert.True(rest.Still(world, 0.8));
            Put(0.23); // rolled 3 cm: the wait starts again
            Assert.False(rest.Still(world, 0.9));
            Assert.False(rest.Still(world, 1.4));
            Assert.True(rest.Still(world, 1.51));
        }
    }

    /// <summary>docs/adr/0036 amendment 5: after a knock, the next event is planned once the objects have settled, against them as they are.</summary>
    public class SafetyTakeKnockTests
    {
        [Fact]
        public void AfterAKnock_TheNextIsPlannedAgainstTheObjectsWhereTheyNowAre()
        {
            // A knock sends its object wherever physics takes it, so the next event is planned only once the objects have settled,
            // from a point of the free movement that follows the knock (the runway), against the objects as they now are.
            var two = new SafetyOptions { MinEvents = 2, MaxEvents = 2 };
            int checkedTakes = 0;
            for (ulong seed = 1; seed <= 4 && checkedTakes < 2; seed++)
            {
                DrivePlan plan = Plan(DriveScript.CollideWithObstacle, Bench, seed, two);
                Assert.Single(plan.Events);
                SafetyContinuation? c = plan.Pending;
                Assert.NotNull(c);
                Assert.Equal(plan.Events[0].ObjectId, c!.Knocked);
                Assert.Equal(2, c.NextIndex);
                Assert.Equal(plan.Events[0].End, c.KnockEnd);
                // Until something takes over, the runway is free movement, away from the knocked object, and ends the take.
                Assert.True(plan.TotalSeconds - plan.Events[0].Clear >= 8.0, $"the runway is {plan.TotalSeconds - plan.Events[0].Clear:0.0} s");

                // The knocked object over on its side 8 cm on, and another of the objects pushed 3 cm and turned 30 degrees by it.
                SafetyObject[] now = Bench.ToArray();
                int k = Array.FindIndex(now, o => o.Id == c.Knocked);
                now[k] = KnockedOver(now[k], c.KnockWay);
                int m = (k + 1) % now.Length;
                now[m] = new SafetyObject(now[m].Id, now[m].Center + new Vector3(0.02f, 0.02f, 0f),
                    Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 6f), now[m].HalfExtents);
                double at = c.SwitchAfter(c.KnockEnd + 2.0 + 4.0);
                Assert.False(double.IsNaN(at));
                DrivePlan next = c.Continue(now, at, out string outcome);
                DrivePlan again = c.Continue(now, at, out _);
                Assert.Equal(next.Legs.Count, again.Legs.Count); // deterministic: the same world, the same plan
                for (int i = 0; i < next.Legs.Count; i++) Assert.Equal(0f, next.Legs[i].To.MaxJointDelta(again.Legs[i].To));
                for (double t = 0; t < at; t += Tick) Assert.Equal(0f, plan.Sample(t).MaxJointDelta(next.Sample(t))); // nothing jumps

                if (next.Events.Count < 2) continue; // this layout as the knock left it had no approach: the take ends after a tail
                checkedTakes++;
                SafetyEvent second = next.Events[1];
                Assert.True(second.ApproachStart >= at - 1e-9, outcome);
                Assert.NotEqual(c.Knocked, second.ObjectId); // another object first
                SafetyObject target = now.First(o => o.Id == second.ObjectId);
                // The swipe goes through the target where it now is, and the arm keeps clear of the knocked one where it now lies.
                Assert.Contains(Enumerable.Range(0, (int)((second.End - second.FinalStart) / Tick) + 1),
                    i => Within(next.Sample(second.FinalStart + i * Tick), new[] { target }, -0.008f));
                for (double t = at; t < second.Contact; t += Tick)
                {
                    Assert.False(Within(next.Sample(t), new[] { now[k] }, SafetyScript.ApproachClearanceMeters - 0.002f), $"near the knocked object at {t:0.00} s ({outcome})");
                }

                Assert.Null(next.Pending); // the last of the two
                Assert.True(next.TotalSeconds >= second.Clear + two.TailMinSeconds - Tick);
            }

            Assert.True(checkedTakes >= 1);
        }

        [Fact]
        public void AfterAKnock_WithNothingLeftToApproach_TheTakeEndsAfterATail()
        {
            // The only object that can be knocked over went off the table (it lies on the floor), and the other is too heavy to
            // knock over: no approach, so the take ends with free movement soon after the point it went on from, one knock being a
            // whole take. (Were the knocked object still in reach, the next knock could aim at it again: preferred last, not barred.)
            var roles = new SafetyOptions { MinEvents = 2, MaxEvents = 2, KnockMaxKg = 0.2 };
            SafetyObject light = new SafetyObject(2, Bench[1].Min, Bench[1].Max, true, 0.03, tippy: true);
            SafetyObject heavy = new SafetyObject(3, Bench[2].Min, Bench[2].Max, true, 1.5);
            DrivePlan plan = Plan(DriveScript.CollideWithObstacle, new[] { light, heavy }, 1, roles);
            Assert.Equal(2, plan.Events[0].ObjectId);
            SafetyContinuation c = plan.Pending!;
            var floor = new SafetyObject(2, new Vector3(0.30f, -0.45f, -0.95f), Quaternion.Identity, light.HalfExtents, true, 0.03);
            double at = c.SwitchAfter(c.KnockEnd + 6.0);
            DrivePlan next = c.Continue(new[] { floor, heavy }, at, out string outcome);
            Assert.Single(next.Events);
            Assert.Null(next.Pending);
            Assert.True(next.TotalSeconds > at && next.TotalSeconds <= at + roles.TailMaxSeconds + 1.0, $"{outcome}: ends at {next.TotalSeconds:0.0} s, going on from {at:0.0} s");
        }
    }
}
