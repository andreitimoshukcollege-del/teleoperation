using System;
using System.Collections.Generic;
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
    /// <summary>
    /// The mixed script (docs/adr/0038 decision 1) over RobotHost's random layouts of the bench: deterministic, its timing rules,
    /// one continuous range of offsets, contacts at many heights and at any point of a move, and its speed caps.
    /// </summary>
    public class MixedTakeTests
    {
        private readonly ITestOutputHelper _output;

        public MixedTakeTests(ITestOutputHelper output) => _output = output;

        private static readonly Lazy<(WorldScene Scene, ObjectShapes? Shapes)> Bench_ = new Lazy<(WorldScene, ObjectShapes?)>(() =>
        {
            WorldScene scene = BenchScene(out ObjectShapes? shapes);
            return (scene, shapes);
        });

        private static SafetyObject[] LayoutOf(ulong seed) => Layout(Bench_.Value.Scene, Bench_.Value.Shapes, seed);

        /// <summary>core/Scenes/jetrover-mixed.json (docs/adr/0038 decision 3) and its shapes: stands, planks on supports, rods, raised objects.</summary>
        internal static readonly Lazy<(WorldScene Scene, ObjectShapes? Shapes)> MixedScene = new Lazy<(WorldScene, ObjectShapes?)>(() =>
        {
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !System.IO.File.Exists(System.IO.Path.Combine(dir, "core", "Scenes", "jetrover-mixed.json"))) dir = System.IO.Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            WorldScene scene = WorldView.LoadScene(System.IO.Path.Combine(dir!, "core", "Scenes", "jetrover-mixed.json"), out _);
            return (scene, WorldView.ShapesOf(scene));
        });

        /// <summary>RobotHost's layout <paramref name="seed"/> of jetrover-mixed as joint-drive sees it on the twin (a distractor is scenery, as WorldView.TryObjects has it).</summary>
        internal static SafetyObject[] MixedLayout(ulong seed)
        {
            (WorldScene scene, ObjectShapes? shapes) = MixedScene.Value;
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            SceneLayout.Draw(scene, seed, poses, parked, scales);
            return scene.Objects.Select((o, i) => (o, i)).Where(x => !parked[x.i])
                .Select(x => WorldView.Bounds(x.o, scales[x.i], poses[x.i].X, poses[x.i].Y, poses[x.i].Z, poses[x.i].Qw, poses[x.i].Qx, poses[x.i].Qy, poses[x.i].Qz, shapes,
                    (x.o.Movable && scene.InEdgeRegions(poses[x.i].X, poses[x.i].Y)) || x.o.Role == SceneRole.Distractor))
                .ToArray();
        }

        internal static bool TryMixed(IReadOnlyList<SafetyObject> objects, ulong seed, out DrivePlan? plan, out string? error, MixedOptions? options = null, ArmPose? start = null) =>
            DrivePlan.TryBuild(DriveScript.Mixed, Profile, start ?? Model.Home, 3.8f, 15, seed, Envelope, out plan, out error, objects, mixedOptions: options);

        internal static DrivePlan Mixed(IReadOnlyList<SafetyObject> objects, ulong seed, MixedOptions? options = null)
        {
            Assert.True(TryMixed(objects, seed, out DrivePlan? plan, out string? error, options), error);
            return plan!;
        }

        /// <summary>The plan of the first of ten seeds from <paramref name="seed"/> that plans on <paramref name="objects"/> (about 3% of plans find no tail).</summary>
        internal static DrivePlan AnyMixed(IReadOnlyList<SafetyObject> objects, ulong seed, MixedOptions? options = null)
        {
            string? error = null;
            for (ulong s = seed; s < seed + 10; s++)
            {
                if (TryMixed(objects, s, out DrivePlan? plan, out error, options)) return plan!;
            }

            Assert.Fail(error);
            return null!;
        }

        [Fact]
        public void AMixedTake_IsDeterministic_AndItsMovesTileIt()
        {
            SafetyObject[] objects = LayoutOf(3);
            DrivePlan a = AnyMixed(objects, 11), b = AnyMixed(objects, 11), c = AnyMixed(objects, 31);
            Assert.Equal(DriveScript.Mixed, a.Script);
            Assert.Equal(a.Legs.Count, b.Legs.Count);
            for (int i = 0; i < a.Legs.Count; i++)
            {
                Assert.Equal(a.Legs[i].Start, b.Legs[i].Start);
                Assert.Equal(0f, a.Legs[i].To.MaxJointDelta(b.Legs[i].To));
            }

            Assert.Equal(a.Moves.Select(m => m.ToString()), b.Moves.Select(m => m.ToString()));
            Assert.NotEqual(a.Moves.Select(m => m.ToString()), c.Moves.Select(m => m.ToString()));
            // The moves: numbered in the order they begin, each a kind the record knows, an object only for an aim or a pass.
            Assert.NotEmpty(a.Moves);
            for (int i = 0; i < a.Moves.Count; i++)
            {
                PlannedMove m = a.Moves[i];
                Assert.Equal(i + 1, m.Index);
                if (i > 0) Assert.True(m.Start >= a.Moves[i - 1].Start);
                Assert.True(m.End >= m.Start, m.ToString());
                if (ScriptMoveCodec.IsAccident(m.Kind))
                {
                    // An accident overlies the move it came in, from its onset, and keeps that move's object (docs/adr/0038 "M, motion").
                    PlannedMove over = a.Moves.Single(x => x.Index == m.Overlies);
                    Assert.False(ScriptMoveCodec.IsAccident(over.Kind));
                    Assert.True(over.Start <= m.Start, m.ToString());
                    Assert.Equal(over.ObjectId, m.ObjectId);
                }
                else Assert.Equal(m.Kind == ScriptMoveKind.Aim || m.Kind == ScriptMoveKind.Pass, m.ObjectId != 0);
                Assert.True(m.Start < a.TotalSeconds, m.ToString());
            }

            Assert.True(a.Moves[0].Start <= a.ScriptStart, "the first move begins with the take (the arm already moving)");
        }

        [Fact]
        public void OnRandomLayouts_TheTimingRulesHold_AndTheOffsetsAreOneContinuousRange()
        {
            // RobotHost draws a new layout per take. Every mixed take: the arm moving within 0.3 s of the take's start; nothing
            // within 1 cm of an object's box (the margin is 2 cm) before 1.5 s or in the last 1.0 s; the last planned contact at least
            // 1.0 s before the end. Over them all, the aims' offsets fill one range across the surface, contacts by the model come at
            // many heights, and some come early in their move.
            const int Layouts = 24;
            var offsets = new List<float>();
            var heights = new List<float>(); // the lowest point of the arm at each planned first contact, m above the table
            var where = new List<double>(); // each planned contact's place in its move: 0 its start, 1 its end
            int planned = 0, aims = 0, passes = 0, contacts = 0;
            var why = new List<string>();
            WorldScene scene = Bench_.Value.Scene;
            float table = (float)scene.Table.Top;
            for (ulong seed = 1; seed <= Layouts; seed++)
            {
                SafetyObject[] objects = LayoutOf(seed);
                if (!TryMixed(objects, 1000 + seed, out DrivePlan? plan, out string? error))
                {
                    why.Add($"layout {seed}: {error}");
                    continue;
                }

                planned++;
                double start = plan!.ScriptStart, end = plan.TotalSeconds;
                ArmPose first = plan.Sample(start);
                double moving = double.NaN;
                for (double t = start; t <= start + 1.0 && double.IsNaN(moving); t += Tick / 4) if (plan.Sample(t).MaxJointDelta(first) > 0.5 * Math.PI / 180) moving = t - start;
                Assert.True(moving <= 0.3, $"layout {seed}: still for {moving:0.00} s");
                for (double t = start; t < end; t += Tick)
                {
                    if (t - start >= 1.5 && t < end - 1.0) continue;
                    Assert.False(Within(plan.Sample(t), objects, 0.01f), $"layout {seed}: within 1 cm of an object at {t - start:0.00} s of {end - start:0.00} s");
                }

                foreach (PlannedMove m in plan.Moves.Where(m => m.ObjectId != 0))
                {
                    // An accident on an aim or a pass keeps the timing rules too; the figures are the aims' and passes' own.
                    bool accident = ScriptMoveCodec.IsAccident(m.Kind);
                    if (!double.IsNaN(m.Contact))
                    {
                        Assert.True(m.Contact - start >= 1.5 - 1e-9, $"layout {seed}: {m}");
                        Assert.True(end - m.Contact >= 1.0 - 1e-9, $"layout {seed}: {m}, the take ends at {end:0.00} s");
                    }

                    if (accident) continue;
                    if (m.Kind == ScriptMoveKind.Aim)
                    {
                        aims++;
                        offsets.Add(m.OffsetMeters);
                    }
                    else passes++;

                    if (double.IsNaN(m.Contact)) continue;
                    contacts++;
                    where.Add((m.Contact - m.Start) / Math.Max(1e-6, m.End - m.Start));
                    heights.Add(Lowest(plan.Sample(m.Contact)) - table);
                }
            }

            offsets.Sort();
            heights.Sort();
            string report = FormattableString.Invariant(
                $"{planned}/{Layouts} planned; {aims} aims, {passes} passes, {contacts} with a planned contact; offsets {offsets.FirstOrDefault() * 1000:0}..{offsets.LastOrDefault() * 1000:0} mm; ") +
                FormattableString.Invariant($"contact heights {heights.FirstOrDefault() * 1000:0}..{heights.LastOrDefault() * 1000:0} mm above the table; contacts in the first third of their move {where.Count(w => w < 1 / 3.0)} of {where.Count}; ") +
                string.Join(" | ", why);
            _output.WriteLine(report);
            // The bars (docs/adr/0038, "M, motion": over 200 layouts of the bench). Nearly every layout plans, and its takes aim at
            // objects: at least one object-directed move a take on average, passes among them.
            Assert.True(planned >= Layouts * 9 / 10, report);
            Assert.True(aims + passes >= planned, report);
            Assert.True(passes >= 1, report);
            // One continuous range: both signs, and no gap wider than 25 mm anywhere in it, across zero included.
            Assert.True(offsets.First() < -0.010f && offsets.Last() > 0.040f, report);
            for (int i = 1; i < offsets.Count; i++) Assert.True(offsets[i] - offsets[i - 1] < 0.025f, $"a gap from {offsets[i - 1] * 1000:0} to {offsets[i] * 1000:0} mm; {report}");
            // Contacts at many heights, not one: at least 2 cm between the lowest and highest quarter, or 6 cm between the extremes.
            Assert.True(heights.Count >= 6, report);
            Assert.True(heights[heights.Count * 3 / 4] - heights[heights.Count / 4] >= 0.02f || heights[heights.Count - 1] - heights[0] >= 0.06f, report);
            // Some contacts early in their move (a pass's mid-move, an aim started close), not only at its end.
            Assert.True(where.Count(w => w < 0.6) >= 1, report);
        }

        /// <summary>The lowest point of the arm's bodies at <paramref name="pose"/> (their boxes' corners), arm base frame z.</summary>
        internal static float Lowest(in ArmPose pose)
        {
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ArmCollision.ComputeFrames(Profile, pose, frames);
            float low = float.MaxValue;
            foreach (CollisionBody b in ShapeGap.TouchBodies(Model))
            {
                LinkFrame f = frames[(int)b.Link];
                for (int k = 0; k < 8; k++)
                {
                    var corner = new Vector3((k & 1) == 0 ? -b.HalfExtents.X : b.HalfExtents.X, (k & 2) == 0 ? -b.HalfExtents.Y : b.HalfExtents.Y, (k & 4) == 0 ? -b.HalfExtents.Z : b.HalfExtents.Z);
                    low = MathF.Min(low, f.ToBase(b.Center + Vector3.Transform(corner, b.Rotation)).Z);
                }
            }

            return low;
        }

        [Fact]
        public void TheCaps_HoldOnEveryTick_AndWithoutThemTheHandIsFaster()
        {
            SafetyObject[] objects = LayoutOf(5);
            // Without accidents, whose ticks keep to the arm's own limits instead (AccidentTests).
            var capped = new MixedOptions { AccidentShare = 0, AccidentNearShare = 0 };
            MotionCaps caps = capped.Caps!;
            DrivePlan plan = Mixed(objects, 21, capped);
            // Every tick of the take within each joint's caps as they are measured: speed and acceleration over three ticks (about
            // the 25 ms smoothing people's figures were measured with; the legs are linear a tick apart, so the ticks are the plan's
            // own). A single tick may go faster with the commands' own jitter, at most 2.5 times.
            var ticks = new List<ArmPose>();
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds + 1e-9; t += Tick) ticks.Add(plan.Sample(t));
            int m = MotionCaps.AccelerationTicks;
            for (int k = 1; k < ticks.Count; k++)
            {
                for (int j = 0; j < DriveEnvelope.JointCount; j++)
                {
                    double q0 = DrivePlan.Joint(ticks[k], j);
                    double v1 = Math.Abs(q0 - DrivePlan.Joint(ticks[k - 1], j)) / Tick;
                    Assert.True(v1 <= 2.5 * caps.JointSpeed[j], $"{DriveEnvelope.JointName(j)} at {v1:0.00} rad/s in one tick, tick {k}");
                    if (k < m) continue;
                    double v = Math.Abs(q0 - DrivePlan.Joint(ticks[k - m], j)) / (m * Tick);
                    Assert.True(v <= caps.JointSpeed[j] * 1.02 + 0.02, $"{DriveEnvelope.JointName(j)} at {v:0.00} rad/s over {m} ticks, tick {k}");
                    if (k < 2 * m) continue;
                    double a = Math.Abs(q0 - 2 * DrivePlan.Joint(ticks[k - m], j) + DrivePlan.Joint(ticks[k - 2 * m], j)) / (m * m * Tick * Tick);
                    Assert.True(a <= caps.JointAcceleration[j] * 1.02 + 0.5, $"{DriveEnvelope.JointName(j)} at {a:0.0} rad/s2, tick {k}");
                }
            }

            var uncapped = new MixedOptions { Caps = null, Style = null, AccidentShare = 0, AccidentNearShare = 0 };
            Assert.True(TryMixed(objects, 21, out DrivePlan? free, out string? error, uncapped), error);
            Assert.NotEqual(plan.Legs.Count, free!.Legs.Count);
        }

        [Fact]
        public void AimsAndPassesMoveAsFreeMovesDo_AndEveryKindFollowsOneLawAfterIt()
        {
            // docs/adr/0038 "M, motion": a pull back or a pause after a move by one law whatever its kind, the hand always pulling back out
            // of a fixture, aims and passes turning the wrist on the way as free moves do, and the layout-blind goals as low as aims go.
            WorldScene scene = Bench_.Value.Scene;
            float table = (float)scene.Table.Top;
            int blindPullBacks = 0, objectMoves = 0, turned = 0, intoFixtures = 0, blindTransports = 0, lowBlind = 0, highBlind = 0;
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            var options = new MixedOptions { AccidentShare = 0, AccidentNearShare = 0 }; // the moves' own law; accidents overlie it (AccidentTests)
            // 20 layouts: since the 2026-11-24 review the blind goals spread over five bands, and on the crowded bench few end low.
            for (ulong seed = 1; seed <= 20; seed++)
            {
                SafetyObject[] objects = LayoutOf(seed);
                if (!TryMixed(objects, 2000 + seed, out DrivePlan? plan, out _, options)) continue;
                IReadOnlyList<PlannedMove> moves = plan!.Moves;
                for (int i = 0; i < moves.Count; i++)
                {
                    PlannedMove m = moves[i];
                    PlannedMove? next = i + 1 < moves.Count ? moves[i + 1] : (PlannedMove?)null;
                    bool blind = m.Part == MovePart.BlindTransport || m.Part == MovePart.BlindAdjust || m.Part == MovePart.BlindStrokes;
                    if (blind && next.HasValue && next.Value.Part == MovePart.PullBack) blindPullBacks++;
                    double low = double.MaxValue, pitch0 = double.MaxValue, pitch1 = double.MinValue;
                    for (double t = Math.Max(m.Start, plan.ScriptStart); t <= Math.Min(m.End, plan.TotalSeconds); t += Tick)
                    {
                        ArmPose p = plan.Sample(t);
                        ArmCollision.ComputeFrames(Profile, p, frames);
                        low = Math.Min(low, frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper).Z - table);
                        pitch0 = Math.Min(pitch0, HandModel.PitchOf(p));
                        pitch1 = Math.Max(pitch1, HandModel.PitchOf(p));
                    }

                    if (m.Part == MovePart.BlindTransport && m.End < plan.TotalSeconds)
                    {
                        // Where it ends: blind goals at every height (docs/adr/0038 "M, motion", the 2026-11-23 review), not all low.
                        blindTransports++;
                        ArmCollision.ComputeFrames(Profile, plan.Sample(m.End), frames);
                        double end = frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper).Z - table;
                        if (end < 0.08) lowBlind++;
                        if (end > 0.12) highBlind++;
                    }

                    if (m.ObjectId == 0) continue;
                    objectMoves++;
                    if (pitch1 - pitch0 > 3 * Math.PI / 180) turned++;
                    if (objects.First(o => o.Id == m.ObjectId).Movable || m.OffsetMeters >= 0.005f) continue;
                    intoFixtures++;
                    Assert.True(next.HasValue && next.Value.Kind == ScriptMoveKind.Reverse, $"layout {seed}: {m} is not pulled back out of the fixture");
                }
            }

            string report = $"{blindPullBacks} blind moves pulled back from; {turned} of {objectMoves} aims and passes turn the wrist; of {blindTransports} blind moves to a goal {lowBlind} end below 8 cm, {highBlind} above 12 cm; {intoFixtures} into fixtures";
            _output.WriteLine(report);
            Assert.True(blindPullBacks >= 3, report);
            Assert.True(objectMoves >= 8 && turned * 4 >= objectMoves, report);
            // Some end low and some high: in the quiet start and the tail low goals near an object are refused, and the tail after its exit
            // keeps off the low band, so on the crowded bench most end high.
            Assert.True(lowBlind >= 2 && highBlind >= 2, report);
        }

        [Fact]
        public void OnTheMixedScene_NoAimIsAtAStandOrASupport_AimsAtARaisedObjectKeepOffItsStand_AndContactsComeAtEveryHeight()
        {
            // docs/adr/0038 "M, motion" (the 2026-11-23 review of the pilot dataset): the stands under raised objects took 18 of 41 contacts, no
            // tall object or plank was hit, and the tool point sat about 14.5 cm below the base frame at contact.
            (WorldScene scene, _) = MixedScene.Value;
            var tool = new List<float>(); // the tool point at each planned contact, base frame z
            int aims = 0, raisedAims = 0, tallOrPlank = 0, planned = 0;
            var holderWalls = new Dictionary<int, int>();
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            for (ulong layout = 1; layout <= 40; layout++)
            {
                SafetyObject[] objects = MixedLayout(layout);
                if (!TryMixed(objects, 3000 + layout, out DrivePlan? plan, out _)) continue;
                planned++;
                foreach (PlannedMove m in plan!.Moves.Where(m => m.ObjectId != 0))
                {
                    if (!ScriptMoveCodec.IsAccident(m.Kind)) aims++;
                    SafetyObject o = objects.First(x => x.Id == m.ObjectId);
                    Assert.NotEqual(SceneRole.Stand, o.Role);
                    Assert.NotEqual(SceneRole.Support, o.Role);
                    if (o.Role == SceneRole.Plank || o.Max.Z - o.Min.Z >= 0.12f) tallOrPlank++;
                    if (!double.IsNaN(m.Contact) && !ScriptMoveCodec.IsAccident(m.Kind))
                    {
                        ArmCollision.ComputeFrames(Profile, plan.Sample(m.Contact), frames);
                        tool.Add(frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper).Z);
                    }

                    // What holds a raised object up is kept 5 mm off (the model's bodies) from where the move starts to where it ends.
                    SafetyObject[] stands = objects.Where(s => s.Id != o.Id && (s.Role == SceneRole.Stand || s.Role == SceneRole.Support) && s.Max.Z <= o.Min.Z + 0.015f
                        && s.Max.Z >= o.Min.Z - 0.03f && s.Max.X > o.Min.X && s.Min.X < o.Max.X && s.Max.Y > o.Min.Y && s.Min.Y < o.Max.Y).ToArray();
                    if (stands.Length == 0) continue;
                    if (!ScriptMoveCodec.IsAccident(m.Kind)) raisedAims++;
                    ArmCollisionModel off = SafetyScript.WithObjects(Model, stands, MixedTake.HolderClearMeters - 0.0015f, 0f);
                    // Until the move after it begins (they overlap, and a pull back is its own move).
                    double until = Math.Min(m.End, plan.Moves.Where(n => n.Index == m.Index + 1).Select(n => n.Start).DefaultIfEmpty(m.End).First());
                    for (double t = m.Start; t <= until; t += Tick) Assert.True(ArmCollision.IsValid(Profile, off, plan.Sample(t)), $"layout {layout}: {m} meets what holds it up at {t:0.00} s");
                }
            }

            tool.Sort();
            int high = tool.Count(z => z > -0.08f);
            string report = FormattableString.Invariant($"{planned} planned, {aims} aims and passes ({raisedAims} at raised objects, {tallOrPlank} at tall ones or planks); ") +
                            FormattableString.Invariant($"{tool.Count} planned contacts, tool point {tool.FirstOrDefault() * 100:0.0}..{tool.LastOrDefault() * 100:0.0} cm, {high} above -8 cm");
            _output.WriteLine(report);
            Assert.True(planned >= 36, report);
            Assert.True(raisedAims >= 1 && tallOrPlank >= 6, report);
            Assert.True(tool.Count >= 10 && high * 5 >= tool.Count, report); // contacts high as well as low
        }

        [Fact]
        public void BlindGoals_AreDrawnByTheToolPointsHeight_AndTheBandsAreChecked()
        {
            SafetyObject[] objects = MixedLayout(4);
            var options = new MixedOptions();
            Assert.NotNull(options.Reach.ToolBands);
            DrivePlan plan = AnyMixed(objects, 41, options);
            // The tool point where blind moves to a goal end: spread over the bands, low, middle and high.
            var ends = new List<float>();
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            for (ulong seed = 41; seed < 47; seed++)
            {
                if (!TryMixed(objects, seed, out DrivePlan? p, out _, options)) continue;
                foreach (PlannedMove m in p!.Moves.Where(m => m.Part == MovePart.BlindTransport && m.End < p.TotalSeconds && !ScriptMoveCodec.IsAccident(m.Kind)))
                {
                    ArmCollision.ComputeFrames(Profile, p.Sample(m.End), frames);
                    ends.Add(frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper).Z);
                }
            }

            int low = ends.Count(z => z < -0.12f), mid = ends.Count(z => z >= -0.12f && z < -0.06f), high = ends.Count(z => z >= -0.06f);
            Assert.True(low >= 2 && mid >= 2 && high >= 2, $"blind goals' tool point: {low} low, {mid} middle, {high} high");
            Assert.Throws<ArgumentException>(() => new MixedOptions { Reach = new BlindReach { ToolBands = new[] { -0.1f, -0.2f, 1f } } }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { Reach = new BlindReach { ToolBands = new[] { -0.1f, 0.0f } } }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { QuietReach = new BlindReach { ToolBands = new[] { -0.1f, 0.0f, 0f } } }.Validate());
            Assert.Equal(options.Reach.ToolBands!.Length - 3, options.Reach.WithoutLow().ToolBands!.Length);
        }

        [Fact]
        public void TheMixedOptions_AreChecked()
        {
            Assert.Throws<ArgumentException>(() => new MixedOptions { OffsetMinMeters = 0.05f, OffsetMaxMeters = 0.01f }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { TailMinSeconds = 0.5 }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { Caps = new MotionCaps().AllSpeeds(0f) }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { PullBackShare = 1.5 }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { TurnDegreesPerMeter = -1 }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { AccidentShare = 1.5 }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { AccidentNearShare = -0.1 }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { AccidentWeights = new double[] { 1, 1 } }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { AccidentWeights = new double[] { 0, 0, 0, 0, 0 } }.Validate());
            new MixedOptions { AccidentShare = 0, AccidentNearShare = 0, AccidentWeights = new double[] { 0, 0, 0, 0, 0 } }.Validate(); // none: no kinds needed
            Assert.False(TryMixed(LayoutOf(1), 1, out _, out string? error, new MixedOptions { PassShare = 2 }));
            Assert.Contains("pass share", error);
            Assert.Equal("mixed", SafetyScript.Category(DriveScript.Mixed));
            Assert.True(SafetyScript.IsSafety(DriveScript.Mixed));
        }
    }
}
