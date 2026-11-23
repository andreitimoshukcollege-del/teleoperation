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
        public void TheEdits_AreDeterministic_NeverRepeatInAGroup_AndKeepToTheirRules()
        {
            // The twin check of 2026-11-23: both replays of a group often removed the same object. A group's replays now never make
            // one edit twice, and when nothing new is left there is no edit.
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            int checkedGroups = 0, kinds = 0;
            for (ulong layout = 1; layout <= 6; layout++)
            {
                SafetyObject[] objects = Layout(scene, shapes, layout);
                if (!MixedTakeTests.TryMixed(objects, 40 + layout, out DrivePlan? plan, out _)) continue;
                checkedGroups++;
                ulong group = 500 + layout;
                var touched = new HashSet<byte>();
                var used = new HashSet<string>();
                var seen = new List<ReplayEdit>();
                for (int replay = 1; replay <= 8; replay++)
                {
                    ReplayEditDraw a = ReplayEdits.Draw(group, replay, 0, plan!, objects, scene, touched, Profile, used);
                    ReplayEditDraw b = ReplayEdits.Draw(group, replay, 0, plan!, objects, scene, touched, Profile, new HashSet<string>(used));
                    Assert.Equal(a.Record, b.Record); // the same take, objects, draw and edits so far: the same edit
                    if (a.Pose.Id == 0) break; // nothing new is left
                    string key = ReplayEdits.Key(a.Record);
                    Assert.DoesNotContain(key, used);
                    used.Add(key);
                    Assert.Equal(ReplayRole.Replay, a.Record.Role);
                    Assert.Equal(group, a.Record.Group);
                    seen.Add(a.Record.Edit);
                    int i = Array.FindIndex(objects, o => o.Id == a.Record.ObjectId);
                    Assert.True(i >= 0);
                    Assert.False(ReplayEdits.Holds(objects, i), "an object something stands on is never edited");
                    Assert.False(objects[i].Scenery);
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
                            Assert.InRange(d, ReplayEdits.ShiftMinMeters - 1e-4f, ReplayEdits.ShiftMaxMeters + 1e-4f);
                            Assert.Contains(plan!.Moves, m => m.ObjectId == o.Id); // along a move the take made at it
                            Assert.True(scene.Objects[scene.IndexOf(o.Id)].OperatorMovable && o.Min.Z <= (float)scene.Table.Top + 0.01f);
                            break;
                        case ReplayEdit.Move:
                            Assert.Equal(WorldEditOp.Move, a.Op);
                            Assert.True(new Vector2(a.Record.A - o.Center.X, a.Record.B - o.Center.Y).Length() >= 0.05f, "elsewhere, not a nudge");
                            int k = scene.IndexOf(o.Id);
                            Assert.True(scene.Objects[k].OperatorMovable && scene.InReach(k, a.Record.A, a.Record.B));
                            break;
                    }
                }

                kinds += seen.Distinct().Count();
            }

            Assert.True(checkedGroups >= 4);
            Assert.True(kinds >= checkedGroups, "the groups' replays take more than one kind of edit");
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

        /// <summary>The objects RobotHost's layout <paramref name="seed"/> of <paramref name="scene"/> leaves out that an add may stand on the table, as joint-drive takes them.</summary>
        private static List<byte> ParkedOf(WorldScene scene, ulong seed)
        {
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            SceneLayout.Draw(scene, seed, poses, parked, scales);
            return Enumerable.Range(0, scene.Objects.Length)
                .Where(i => parked[i] && scene.Objects[i].Movable && scene.Objects[i].OperatorMovable && (!scene.Mixed || scene.Objects[i].Role == SceneRole.Obstacle))
                .Select(i => scene.Objects[i].Id).ToList();
        }

        private static ReplayAdd AddFor(DrivePlan plan, IReadOnlyList<byte> parked) =>
            new ReplayAdd(ReplayAdd.Commanded(plan), plan.TotalSeconds - plan.ScriptStart, parked, Model, new MixedOptions());

        [Fact]
        public void AnAdd_StandsAMovableObstacleWhereTheArmPassedLow_ByTheScenesRules_AndTheArmMeetsIt_ButNotInTheQuietStartOrEnd()
        {
            // docs/adr/0038 decision 2 (the twin check's review): an obstacle put into the path, a safe take's unsafe twin.
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            var options = new MixedOptions();
            float table = (float)scene.Table.Top;
            int adds = 0;
            for (ulong layout = 1; layout <= 8; layout++)
            {
                SafetyObject[] objects = Layout(scene, shapes, layout);
                if (!MixedTakeTests.TryMixed(objects, 70 + layout, out DrivePlan? plan, out _)) continue;
                ReplayAdd add = AddFor(plan!, ParkedOf(scene, layout));
                var r = new SeededRng(layout);
                ReplayEditDraw a = ReplayEdits.Add(900 + layout, ref r, plan!, objects, scene, new HashSet<byte>(), Profile, new HashSet<string>(), null, add);
                var again = new SeededRng(layout);
                Assert.Equal(a.Record, ReplayEdits.Add(900 + layout, ref again, plan!, objects, scene, new HashSet<byte>(), Profile, new HashSet<string>(), null, add).Record);
                if (a.Pose.Id == 0) continue;
                adds++;
                Assert.Equal((ReplayRole.Replay, (ulong)(900 + layout), ReplayEdit.Add), (a.Record.Role, a.Record.Group, a.Record.Edit));
                Assert.StartsWith($"add:{a.Record.ObjectId}:", ReplayCodec.Describe(a.Record));
                int k = scene.IndexOf(a.Record.ObjectId);
                Assert.True(scene.Objects[k].Movable && scene.Objects[k].OperatorMovable);
                SafetyObject after = a.After!.Value;
                Assert.True(scene.InReach(k, after.Center.X, after.Center.Y), "in reach");
                Assert.InRange(after.Min.Z, table - 0.002f, table + 0.002f); // standing on the table
                Assert.Equal(a.Record.A, after.Center.X, 4);
                Assert.Equal(a.Record.B, after.Center.Y, 4);
                Assert.True(ReplayEdits.KeepsQuiet(plan!, after, Profile, Model, options), "clear of the quiet start and end");
                double met = MixedDryRun.FirstMeeting(Profile, Model, plan!, after);
                Assert.False(double.IsNaN(met), $"layout {layout}: the arm never meets {ReplayCodec.Describe(a.Record)}");
                Assert.True(met >= options.QuietStartSeconds, $"layout {layout}: met at {met:0.00} s");
                if (a.Op == WorldEditOp.Move) Assert.Contains(objects, o => o.Id == a.Record.ObjectId); // one of the layout's, moved there
                else
                {
                    Assert.Equal(WorldEditOp.Unpark, a.Op); // one the layout left out, stood there
                    Assert.Contains(a.Record.ObjectId, add.Parked);
                    Assert.DoesNotContain(objects, o => o.Id == a.Record.ObjectId);
                }
            }

            Assert.True(adds >= 4, $"{adds} adds in 8 layouts");
        }

        [Fact]
        public void InAGroupOfTwo_OneReplayIsTheAdd_WhenOneFits_AndTheOtherRemovesShiftsOrMoves()
        {
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            int groups = 0, withAdd = 0;
            for (ulong layout = 1; layout <= 8; layout++)
            {
                SafetyObject[] objects = Layout(scene, shapes, layout);
                if (!MixedTakeTests.TryMixed(objects, 70 + layout, out DrivePlan? plan, out _)) continue;
                groups++;
                ulong group = 1200 + layout;
                ReplayAdd add = AddFor(plan!, ParkedOf(scene, layout));
                var used = new HashSet<string>();
                var edits = new List<ReplayEditDraw>();
                for (int replay = 1; replay <= 2; replay++)
                {
                    ReplayEditDraw e = ReplayEdits.Draw(group, replay, 0, plan!, objects, scene, new HashSet<byte>(), Profile, used, null, add);
                    if (e.Pose.Id == 0) break;
                    used.Add(ReplayEdits.Key(e.Record));
                    edits.Add(e);
                }

                int at = edits.FindIndex(e => e.Record.Edit == ReplayEdit.Add);
                Assert.True(edits.Count(e => e.Record.Edit == ReplayEdit.Add) <= 1);
                if (at >= 0)
                {
                    withAdd++;
                    Assert.Equal(ReplayEdits.AddReplay(group), at + 1); // the replay the group drew for it
                }

                foreach (ReplayEditDraw e in edits.Where(e => e.Record.Edit != ReplayEdit.Add))
                {
                    Assert.Contains(e.Record.Edit, new[] { ReplayEdit.Remove, ReplayEdit.Shift, ReplayEdit.Move });
                }

                Assert.Equal(edits.Count, edits.Select(e => ReplayEdits.Key(e.Record)).Distinct().Count()); // never one edit twice
            }

            Assert.True(groups >= 4);
            Assert.True(withAdd * 3 >= groups * 2, $"an add in {withAdd} of {groups} groups");
            Assert.Contains(1, Enumerable.Range(0, 40).Select(g => ReplayEdits.AddReplay((ulong)g)));
            Assert.Contains(2, Enumerable.Range(0, 40).Select(g => ReplayEdits.AddReplay((ulong)g)));
        }

        [Fact]
        public void AnAdd_NeverTakesWhatTheTakeTouched_WhatHoldsAnother_WhatIsRaised_OrWhatTheGroupAddedAlready()
        {
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            SafetyObject[] objects = Layout(scene, shapes, 5);
            DrivePlan plan = MixedTakeTests.AnyMixed(objects, 75);
            List<byte> parked = ParkedOf(scene, 5);
            ReplayAdd add = AddFor(plan, parked), nothingParked = AddFor(plan, Array.Empty<byte>());
            var everything = new HashSet<byte>(objects.Select(o => o.Id));
            for (int attempt = 0; attempt < 6; attempt++)
            {
                // Everything on the table touched: only a parked one may be stood there.
                var r = new SeededRng((ulong)attempt + 1);
                ReplayEditDraw e = ReplayEdits.Add(31, ref r, plan, objects, scene, everything, Profile, new HashSet<string>(), null, add);
                if (e.Pose.Id != 0) Assert.Equal(WorldEditOp.Unpark, e.Op);
                // And with nothing parked, nothing at all.
                r = new SeededRng((ulong)attempt + 1);
                Assert.Equal(0, ReplayEdits.Add(31, ref r, plan, objects, scene, everything, Profile, new HashSet<string>(), null, nothingParked).Pose.Id);
            }

            // Every object added already: nothing new.
            var added = new HashSet<string>(parked.Select(id => "add:" + id).Concat(objects.Select(o => "add:" + o.Id)));
            var rng = new SeededRng(9);
            Assert.Equal(0, ReplayEdits.Add(31, ref rng, plan, objects, scene, new HashSet<byte>(), Profile, added, null, add).Pose.Id);

            // One object standing on another: the one below holds it, the one above is raised; neither is moved for an add.
            int a = Array.FindIndex(objects, o => !o.Scenery && scene.Objects[scene.IndexOf(o.Id)].OperatorMovable);
            int b = Array.FindIndex(objects, o => !o.Scenery && o.Id != objects[a].Id && scene.Objects[scene.IndexOf(o.Id)].OperatorMovable);
            Assert.True(a >= 0 && b >= 0);
            SafetyObject below = objects[a], top = objects[b];
            var stacked = (SafetyObject[])objects.Clone();
            stacked[b] = new SafetyObject(top.Id, new Vector3(below.Center.X, below.Center.Y, below.Max.Z + (top.Max.Z - top.Min.Z) / 2f), top.Rotation, top.HalfExtents, top.Movable,
                top.Mass, top.Tippy, top.Shape, top.Scenery);
            Assert.True(ReplayEdits.Holds(stacked, a));
            var others = new HashSet<byte>(stacked.Where((o, i) => i != a && i != b).Select(o => o.Id));
            for (int attempt = 0; attempt < 6; attempt++)
            {
                var r = new SeededRng((ulong)attempt + 20);
                Assert.Equal(0, ReplayEdits.Add(31, ref r, plan, stacked, scene, others, Profile, new HashSet<string>(), null, nothingParked).Pose.Id);
            }
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
