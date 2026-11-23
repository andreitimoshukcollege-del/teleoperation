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

        internal static bool TryMixed(IReadOnlyList<SafetyObject> objects, ulong seed, out DrivePlan? plan, out string? error, MixedOptions? options = null, ArmPose? start = null) =>
            DrivePlan.TryBuild(DriveScript.Mixed, Profile, start ?? Model.Home, 3.8f, 15, seed, Envelope, out plan, out error, objects, mixedOptions: options);

        internal static DrivePlan Mixed(IReadOnlyList<SafetyObject> objects, ulong seed, MixedOptions? options = null)
        {
            Assert.True(TryMixed(objects, seed, out DrivePlan? plan, out string? error, options), error);
            return plan!;
        }

        [Fact]
        public void AMixedTake_IsDeterministic_AndItsMovesTileIt()
        {
            SafetyObject[] objects = LayoutOf(3);
            DrivePlan a = Mixed(objects, 11), b = Mixed(objects, 11), c = Mixed(objects, 12);
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
                Assert.Equal(m.Kind == ScriptMoveKind.Aim || m.Kind == ScriptMoveKind.Pass, m.ObjectId != 0);
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
                    if (m.Kind == ScriptMoveKind.Aim)
                    {
                        aims++;
                        offsets.Add(m.OffsetMeters);
                    }
                    else passes++;

                    if (double.IsNaN(m.Contact)) continue;
                    contacts++;
                    Assert.True(m.Contact - start >= 1.5 - 1e-9, $"layout {seed}: {m}");
                    Assert.True(end - m.Contact >= 1.0 - 1e-9, $"layout {seed}: {m}, the take ends at {end:0.00} s");
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
            var capped = new MixedOptions();
            MotionCaps caps = capped.Caps!;
            DrivePlan plan = Mixed(objects, 21, capped);
            // Every tick of the take within each joint's speed cap (the legs are linear a tick apart, so the ticks are the plan's own).
            var ticks = new List<ArmPose>();
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds + 1e-9; t += Tick) ticks.Add(plan.Sample(t));
            for (int k = 1; k < ticks.Count; k++)
            {
                for (int j = 0; j < DriveEnvelope.JointCount; j++)
                {
                    double v = Math.Abs(DrivePlan.Joint(ticks[k], j) - DrivePlan.Joint(ticks[k - 1], j)) / Tick;
                    Assert.True(v <= caps.JointSpeed[j] * 1.02 + 0.02, $"{DriveEnvelope.JointName(j)} at {v:0.00} rad/s, tick {k}");
                }
            }

            var uncapped = new MixedOptions { Caps = null, Style = null };
            Assert.True(TryMixed(objects, 21, out DrivePlan? free, out string? error, uncapped), error);
            Assert.NotEqual(plan.Legs.Count, free!.Legs.Count);
        }

        [Fact]
        public void TheMixedOptions_AreChecked()
        {
            Assert.Throws<ArgumentException>(() => new MixedOptions { OffsetMinMeters = 0.05f, OffsetMaxMeters = 0.01f }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { TailMinSeconds = 0.5 }.Validate());
            Assert.Throws<ArgumentException>(() => new MixedOptions { Caps = new MotionCaps().AllSpeeds(0f) }.Validate());
            Assert.False(TryMixed(LayoutOf(1), 1, out _, out string? error, new MixedOptions { PassShare = 2 }));
            Assert.Contains("pass share", error);
            Assert.Equal("mixed", SafetyScript.Category(DriveScript.Mixed));
            Assert.True(SafetyScript.IsSafety(DriveScript.Mixed));
        }
    }
}
