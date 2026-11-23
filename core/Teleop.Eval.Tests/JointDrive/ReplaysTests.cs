using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;
using Xunit;
using static Teleop.Eval.Tests.JointDrive.Takes;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>docs/adr/0038 decision 2: a mixed take's replays send its commands again, on its layout with one edit each.</summary>
    public class ReplaysTests
    {
        private static (WorldScene Scene, ObjectShapes? Shapes) Bench()
        {
            WorldScene scene = BenchScene(out ObjectShapes? shapes);
            return (scene, shapes);
        }

        /// <summary>The commands joint-drive would send while the take records: the plan sampled every tick, with a jittery loop.</summary>
        private static List<SentCommand> Tape(DrivePlan plan, ulong seed)
        {
            var tape = new List<SentCommand>();
            var rng = new SeededRng(seed);
            for (double t = 0; t <= plan.TotalSeconds - plan.ScriptStart + 0.5; t += Tick * (0.8 + 0.4 * rng.NextDouble()))
            {
                double at = plan.ScriptStart + t;
                float g = plan.GripperAt(at);
                tape.Add(new SentCommand(t, plan.Sample(at), float.IsNaN(g) ? 0f : g, plan.SpeedAt(at)));
            }

            return tape;
        }

        [Fact]
        public void AReplay_SendsTheSameCommands_InTheSameOrder_NoneEarly()
        {
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            SafetyObject[] objects = Layout(scene, shapes, 4);
            DrivePlan plan = MixedTakeTests.AnyMixed(objects, 31);
            List<SentCommand> tape = Tape(plan, 7);
            // The replay's loop runs on its own jittery clock, sometimes late by several ticks.
            var player = new TapePlayer(tape);
            var sent = new List<(double Clock, SentCommand Command)>();
            var rng = new SeededRng(99);
            double clock = 0;
            while (!player.Done(clock, 0.5))
            {
                clock += 0.001 + (rng.NextDouble() < 0.02 ? 0.08 : 0.004 * rng.NextDouble());
                while (player.TryNext(clock, out SentCommand c)) sent.Add((clock, c));
            }

            Assert.Equal(tape.Count, sent.Count);
            var targets = new JointTarget[Profile.JointCount];
            var again = new JointTarget[Profile.JointCount];
            for (int i = 0; i < tape.Count; i++)
            {
                Assert.True(sent[i].Clock >= tape[i].At, $"command {i} came out at {sent[i].Clock:0.000} s, before {tape[i].At:0.000} s");
                Assert.True(sent[i].Clock - tape[i].At < 0.1, $"command {i} came out {sent[i].Clock - tape[i].At:0.000} s late");
                // The datagram's targets are the source's, bit for bit.
                int n = JointDriveCommand.Targets(Profile, tape[i].Pose, tape[i].Gripper, tape[i].Speed, targets);
                int m = JointDriveCommand.Targets(Profile, sent[i].Command.Pose, sent[i].Command.Gripper, sent[i].Command.Speed, again);
                Assert.Equal(targets.Take(n), again.Take(m));
            }
        }

        [Fact]
        public void TheEdits_AreDeterministic_TakeEachKindInTurn_AndKeepToTheirRules()
        {
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            var kinds = new List<ReplayEdit>();
            int checkedGroups = 0;
            for (ulong layout = 1; layout <= 6; layout++)
            {
                SafetyObject[] objects = Layout(scene, shapes, layout);
                if (!MixedTakeTests.TryMixed(objects, 40 + layout, out DrivePlan? plan, out _)) continue;
                checkedGroups++;
                ulong group = 500 + layout;
                var touched = new HashSet<byte>();
                var seen = new List<ReplayEdit>();
                for (int replay = 1; replay <= 3; replay++)
                {
                    ReplayEditDraw a = ReplayEdits.Draw(group, replay, 0, plan!, objects, scene, touched, Profile);
                    ReplayEditDraw b = ReplayEdits.Draw(group, replay, 0, plan!, objects, scene, touched, Profile);
                    Assert.Equal(a.Record, b.Record); // the same take, objects and draw: the same edit
                    Assert.Equal(ReplayRole.Replay, a.Record.Role);
                    Assert.Equal(group, a.Record.Group);
                    seen.Add(a.Record.Edit);
                    int i = Array.FindIndex(objects, o => o.Id == a.Record.ObjectId);
                    Assert.True(i >= 0);
                    Assert.False(ReplayEdits.Holds(objects, i), "an object something stands on is never edited");
                    SafetyObject o = objects[i];
                    switch (a.Record.Edit)
                    {
                        case ReplayEdit.Remove:
                            Assert.Equal(WorldEditOp.Park, a.Op);
                            Assert.Null(a.After);
                            break;
                        case ReplayEdit.Shift:
                            Assert.Equal(WorldEditOp.Move, a.Op);
                            float d = new Vector2(a.Record.A, a.Record.B).Length();
                            Assert.InRange(d, ReplayEdits.ShiftMinMeters - 1e-5f, ReplayEdits.ShiftMaxMeters + 1e-5f);
                            Assert.Equal(o.Center.X + a.Record.A, a.Pose.X, 4);
                            Assert.True(scene.Objects[scene.IndexOf(o.Id)].OperatorMovable && o.Min.Z <= (float)scene.Table.Top + 0.01f);
                            break;
                        case ReplayEdit.Move:
                            Assert.Equal(WorldEditOp.Move, a.Op);
                            float r = new Vector2(a.Record.A, a.Record.B).Length();
                            Assert.InRange(r, ReplayEdits.MoveMinRadius - 1e-4f, ReplayEdits.MoveMaxRadius + 1e-4f);
                            Assert.InRange(a.Record.A, (float)scene.RegionXMin, (float)scene.RegionXMax);
                            Assert.True(scene.Objects[scene.IndexOf(o.Id)].OperatorMovable);
                            break;
                    }
                }

                // Three replays of a group take the three kinds, when a movable object stands in reach (else removals).
                if (seen.Distinct().Count() == 3) kinds.AddRange(seen);
                Assert.True(seen.Distinct().Count() == 3 || seen.All(k => k == ReplayEdit.Remove || seen.Count(x => x == k) == 1), string.Join(", ", seen));
            }

            Assert.True(checkedGroups >= 4);
            Assert.NotEmpty(kinds);
        }

        [Fact]
        public void AnObjectTheSourceMoved_IsTheOneARemovalTakes()
        {
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            SafetyObject[] objects = Layout(scene, shapes, 2);
            DrivePlan plan = MixedTakeTests.AnyMixed(objects, 51);
            ulong group = 77;
            int removal = Enumerable.Range(1, 3).First(r => ReplayEdits.Draw(group, r, 0, plan, objects, scene, new HashSet<byte>(), Profile).Record.Edit == ReplayEdit.Remove);
            SafetyObject knocked = objects.First(o => !o.Scenery && !ReplayEdits.Holds(objects, Array.IndexOf(objects, o)));
            ReplayEditDraw e = ReplayEdits.Draw(group, removal, 0, plan, objects, scene, new HashSet<byte> { knocked.Id }, Profile);
            Assert.Equal(ReplayEdit.Remove, e.Record.Edit);
            Assert.Equal(knocked.Id, e.Record.ObjectId);
            Assert.Equal($"remove:{knocked.Id}", ReplayCodec.Describe(e.Record));
        }

        [Fact]
        public void AnEdit_ThatWouldMeetTheQuietStartOrEnd_IsSeen()
        {
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            SafetyObject[] objects = Layout(scene, shapes, 3);
            DrivePlan plan = MixedTakeTests.AnyMixed(objects, 61);
            var options = new MixedOptions();
            // A box where the arm is as the take begins: in the quiet start's way. One far off: not.
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ArmCollision.ComputeFrames(Profile, plan.Sample(plan.ScriptStart), frames);
            Vector3 pads = frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper);
            var inTheWay = new SafetyObject(200, pads, Quaternion.Identity, new Vector3(0.02f));
            var faraway = new SafetyObject(201, new Vector3(-0.3f, 0.6f, -0.17f), Quaternion.Identity, new Vector3(0.02f));
            Assert.False(ReplayEdits.KeepsQuiet(plan, inTheWay, Profile, Model, options));
            Assert.True(ReplayEdits.KeepsQuiet(plan, faraway, Profile, Model, options));
        }

        [Fact]
        public void APlannedMovesRecord_SaysItsTimesFromWhenItIsSent()
        {
            var m = new PlannedMove(3, ScriptMoveKind.Aim, 41, -0.012f, -0.008f, 5.0, 5.4, 5.6, 6.1, 90f);
            ScriptMove r = m.Record(5.02, 17);
            Assert.Equal((3, 17, ScriptMoveKind.Aim, (byte)41), (r.Index, r.Count, r.Kind, r.ObjectId));
            Assert.Equal(-12f, r.OffsetMm, 3);
            Assert.Equal(-8f, r.GapMm, 3);
            Assert.Equal(-0.02f, r.StartSeconds, 4);
            Assert.Equal(0.38f, r.ContactSeconds, 4);
            Assert.Equal(0.58f, r.ClosestSeconds, 4);
            Assert.Equal(1.08f, r.EndSeconds, 4);
            ScriptMove free = new PlannedMove(1, ScriptMoveKind.Free, 0, float.NaN, float.NaN, -0.1, double.NaN, double.NaN, 0.7).Record(0, 17);
            Assert.True(float.IsNaN(free.OffsetMm) && float.IsNaN(free.ContactSeconds) && float.IsNaN(free.ClosestSeconds));
            Assert.Equal(-0.1f, free.StartSeconds, 4);
        }
    }
}
