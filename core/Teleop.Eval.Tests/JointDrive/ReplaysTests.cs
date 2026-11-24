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
            // Every command's capture stamp newer than the one before, even those sent in one pass (RobotHost drops a stamp not newer as stale).
            var stamps = new TapePlayer(tape);
            long last = long.MinValue;
            foreach (long now in new long[] { 100, 100, 100, 101, 250, 250, 240, 400 })
            {
                long s = stamps.Stamp(now);
                Assert.True(s > last && s >= Math.Min(now, last + 1), $"stamp {s} after {last} at {now}");
                last = s;
            }

            Assert.Equal(400, last);
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
            var allKinds = new HashSet<ReplayEdit>();
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
                            Assert.InRange(d, 0.005f, ReplayEdits.ShiftFarthestMeters + 1e-4f);
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

                foreach (ReplayEdit k in seen) allKinds.Add(k);
                kinds = allKinds.Count;
            }

            Assert.True(checkedGroups >= 4);
            Assert.True(kinds >= 2, "the groups' replays take more than one kind of edit");
        }

        [Fact]
        public void AnObjectTheSourceMoved_IsTheOneARemovalTakes()
        {
            (WorldScene scene, ObjectShapes? shapes) = Bench();
            SafetyObject[] objects = Layout(scene, shapes, 2);
            DrivePlan plan = MixedTakeTests.AnyMixed(objects, 51);
            ulong group = 77;
            // The object the source moved is the path's own, whatever the model says of it: the replay whose turn is a removal takes it.
            SafetyObject knocked = objects.First(o => !o.Scenery && !ReplayEdits.Holds(objects, Array.IndexOf(objects, o)));
            var touched = new HashSet<byte> { knocked.Id };
            ReplayEditDraw[] e = Enumerable.Range(1, 3).Select(r => ReplayEdits.Draw(group, r, 0, plan, objects, scene, touched, Profile)).ToArray();
            Assert.Contains(e, x => x.Record.Edit == ReplayEdit.Remove);
            ReplayEditDraw removal = e.First(x => x.Record.Edit == ReplayEdit.Remove);
            Assert.Equal(knocked.Id, removal.Record.ObjectId);
            Assert.Equal($"remove:{knocked.Id}", ReplayCodec.Describe(removal.Record));
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

        /// <summary>core/Scenes/jetrover-mixed.json (docs/adr/0038 decision 3), the scene the mixed script and its replays run on, and its shapes.</summary>
        private static readonly Lazy<(WorldScene Scene, ObjectShapes? Shapes)> Mixed_ = new Lazy<(WorldScene, ObjectShapes?)>(() =>
        {
            string? dir = AppContext.BaseDirectory;
            while (dir != null && !System.IO.File.Exists(System.IO.Path.Combine(dir, "core", "Scenes", "jetrover-mixed.json"))) dir = System.IO.Path.GetDirectoryName(dir);
            Assert.NotNull(dir);
            WorldScene scene = WorldView.LoadScene(System.IO.Path.Combine(dir!, "core", "Scenes", "jetrover-mixed.json"), out _);
            return (scene, WorldView.ShapesOf(scene));
        });

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
            (WorldScene scene, ObjectShapes? shapes) = Mixed_.Value;
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
            (WorldScene scene, ObjectShapes? shapes) = Mixed_.Value;
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
                    // The replay the group drew for it, or the first when that had nothing else near its path to edit.
                    Assert.True(at + 1 == ReplayEdits.AddReplay(group) || at == 0, $"layout {layout}: the add came at replay {at + 1}, drawn for {ReplayEdits.AddReplay(group)}");
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
            (WorldScene scene, ObjectShapes? shapes) = Mixed_.Value;
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

            // One object standing on another: the one below holds it, the one above is raised; neither is moved for an add, though each
            // would be on its own (the arm far off, nothing touched).
            var far = new ReplayAdd(new[] { new TrackPoint(0, Model.Home, 0f), new TrackPoint(20, Model.Home, 0f) }, 20, parked, Model, new MixedOptions());
            for (ulong layout = 1; layout <= 40; layout++)
            {
                SafetyObject[] laid = Layout(scene, shapes, layout);
                List<int> alone = ReplayEdits.AddCandidatesOnTable(laid, scene, new HashSet<byte>(), new HashSet<string>(), far, Profile);
                if (alone.Count < 2) continue;
                int a = alone[0], b = alone[1];
                SafetyObject below = laid[a], top = laid[b];
                var stacked = (SafetyObject[])laid.Clone();
                stacked[b] = new SafetyObject(top.Id, new Vector3(below.Center.X, below.Center.Y, below.Max.Z + (top.Max.Z - top.Min.Z) / 2f), top.Rotation, top.HalfExtents,
                    top.Movable, top.Mass, top.Tippy, top.Shape, top.Scenery);
                Assert.True(ReplayEdits.Holds(stacked, a));
                List<int> now = ReplayEdits.AddCandidatesOnTable(stacked, scene, new HashSet<byte>(), new HashSet<string>(), far, Profile);
                Assert.DoesNotContain(a, now);
                Assert.DoesNotContain(b, now);
                // Nor one the take touched, nor one the group added already.
                Assert.DoesNotContain(a, ReplayEdits.AddCandidatesOnTable(laid, scene, new HashSet<byte> { below.Id }, new HashSet<string>(), far, Profile));
                Assert.DoesNotContain(a, ReplayEdits.AddCandidatesOnTable(laid, scene, new HashSet<byte>(), new HashSet<string> { "add:" + below.Id }, far, Profile));
                return;
            }

            Assert.Fail("no layout has two objects an add may move");
        }

        [Fact]
        public void EveryEdit_ChangesWhatTheModelSaysTheTakeTouches_AndNoneIsFarFromThePath()
        {
            // docs/adr/0038 "M, motion" (the 2026-11-23 review: a group whose removal and move left the same stand contact both times): what the
            // arm touched or came within 1 cm of is removed or moved off the path, what it passed within 5 cm of is moved onto it, nothing
            // farther is edited.
            (WorldScene scene, ObjectShapes? shapes) = Mixed_.Value;
            int edits = 0, onto = 0, off = 0;
            for (ulong layout = 1; layout <= 16; layout++)
            {
                SafetyObject[] objects = MixedTakeTests.MixedLayout(layout);
                if (!MixedTakeTests.TryMixed(objects, 90 + layout, out DrivePlan? plan, out _)) continue;
                ReplayAdd add = AddFor(plan!, ParkedOf(scene, layout));
                var near = new PathProximity(Profile, Model, add.Track, objects, 2);
                var used = new HashSet<string>();
                for (int replay = 1; replay <= 3; replay++)
                {
                    ReplayEditDraw e = ReplayEdits.Draw(1500 + layout, replay, 0, plan!, objects, scene, new HashSet<byte>(), Profile, used, null, add, near);
                    if (e.Pose.Id == 0)
                    {
                        Assert.False(string.IsNullOrEmpty(e.Note), "no edit says why");
                        break;
                    }

                    used.Add(ReplayEdits.Key(e.Record));
                    edits++;
                    if (e.Record.Edit == ReplayEdit.Add) continue;
                    int i = Array.FindIndex(objects, o => o.Id == e.Record.ObjectId);
                    Assert.True(near.Touched(i) || near.Near(i), $"layout {layout}: {ReplayCodec.Describe(e.Record)} edits an object {near.Gap(i) * 1000:0} mm from the path");
                    if (e.Record.Edit == ReplayEdit.Remove)
                    {
                        Assert.True(near.Touched(i), $"layout {layout}: {ReplayCodec.Describe(e.Record)} removes what the arm did not come within 1 cm of");
                        continue;
                    }

                    float after = PathProximity.GapOver(near.Arm, add.Track, e.After!.Value, 2, out _);
                    if (near.Touched(i))
                    {
                        off++;
                        Assert.True(after >= ReplayEdits.ClearMeters, $"layout {layout}: {ReplayCodec.Describe(e.Record)} leaves it {after * 1000:0} mm from the path");
                    }
                    else
                    {
                        onto++;
                        Assert.True(after <= -ReplayEdits.MeetMeters, $"layout {layout}: {ReplayCodec.Describe(e.Record)} leaves it {after * 1000:0} mm from the path");
                    }

                    Assert.True(ReplayEdits.KeepsQuiet(plan!, e.After.Value, Profile, Model, new MixedOptions()));
                }
            }

            Assert.True(edits >= 12, $"{edits} edits ({off} off the path, {onto} onto it)");
        }

        [Fact]
        public void AGroupWithNothingNearItsPath_GetsItsAddAtTheFirstReplay_AndATiltedObjectOnAStandIsHeld()
        {
            (WorldScene scene, ObjectShapes? shapes) = Mixed_.Value;
            for (ulong layout = 1; layout <= 40; layout++)
            {
                // Only what is far from the path stays (none of it can be edited), so the add's replay may be either.
                SafetyObject[] all = MixedTakeTests.MixedLayout(layout);
                if (!MixedTakeTests.TryMixed(all, 120 + layout, out DrivePlan? plan, out _)) continue;
                ReplayAdd add = AddFor(plan!, ParkedOf(scene, layout));
                var whole = new PathProximity(Profile, Model, add.Track, all, 2);
                SafetyObject[] objects = all.Where((o, i) => o.Scenery || float.IsPositiveInfinity(whole.Gap(i))).ToArray();
                ulong group = Enumerable.Range(1, 40).Select(g => (ulong)g).First(g => ReplayEdits.AddReplay(g) == 2);
                ReplayEditDraw first = ReplayEdits.Draw(group, 1, 0, plan!, objects, scene, new HashSet<byte>(), Profile, new HashSet<string>(), null, add);
                if (first.Pose.Id == 0)
                {
                    Assert.StartsWith("nothing near the path", first.Note);
                    Assert.StartsWith("no add", first.AddNote);
                    continue;
                }

                Assert.Equal(ReplayEdit.Add, first.Record.Edit);
                ReplayEditDraw second = ReplayEdits.Draw(group, 2, 0, plan!, objects, scene, new HashSet<byte>(), Profile, new HashSet<string> { ReplayEdits.Key(first.Record) }, null, add);
                Assert.NotEqual(ReplayEdit.Add, second.Record.Edit); // one add a group
                break;
            }

            // The twin check's elephant lying on a stand: its box's bottom 1.5 cm below the stand's top, yet it rests there.
            var stand = new SafetyObject(75, new Vector3(-0.1007f, -0.3224f, -0.1587f), Quaternion.Identity, new Vector3(0.0625f, 0.0625f, 0.0375f), false);
            var lying = new SafetyObject(52, new Vector3(-0.1151f, -0.3261f, -0.0938f), Quaternion.Identity, new Vector3(0.05f, 0.03f, 0.0425f));
            Assert.True(ReplayEdits.Holds(new[] { stand, lying }, 0));
            Assert.False(ReplayEdits.Holds(new[] { stand, lying }, 1));
            // An object on the table beside a stand is not on it.
            var beside = new SafetyObject(38, new Vector3(-0.05f, -0.3224f, -0.1700f), Quaternion.Identity, new Vector3(0.03f, 0.03f, 0.0262f));
            Assert.False(ReplayEdits.Holds(new[] { stand, beside }, 0));
        }

        [Fact]
        public void AnAddRobotHostRefused_LeavesAnotherObjectToAdd_ButAnAddMadeEndsTheGroupsAdds()
        {
            (WorldScene scene, ObjectShapes? shapes) = Mixed_.Value;
            for (ulong layout = 1; layout <= 20; layout++)
            {
                SafetyObject[] objects = MixedTakeTests.MixedLayout(layout);
                if (!MixedTakeTests.TryMixed(objects, 150 + layout, out DrivePlan? plan, out _)) continue;
                ReplayAdd add = AddFor(plan!, ParkedOf(scene, layout));
                ulong group = 1700 + layout;
                int at = ReplayEdits.AddReplay(group);
                ReplayEditDraw first = ReplayEdits.Draw(group, at, 0, plan!, objects, scene, new HashSet<byte>(), Profile, new HashSet<string>(), null, add);
                if (first.Record.Edit != ReplayEdit.Add) continue;
                string key = ReplayEdits.Key(first.Record);
                ReplayEditDraw again = ReplayEdits.Draw(group, at, 1, plan!, objects, scene, new HashSet<byte>(), Profile, new HashSet<string>(), null, add, null, new HashSet<string> { key });
                Assert.NotEqual(key, ReplayEdits.Key(again.Record)); // refused: never that one again, though another may be added
                ReplayEditDraw after = ReplayEdits.Draw(group, 2, 0, plan!, objects, scene, new HashSet<byte>(), Profile, new HashSet<string> { key }, null, add);
                Assert.NotEqual(ReplayEdit.Add, after.Record.Edit); // made: one add a group
                return;
            }

            Assert.Fail("no layout gave an add");
        }

        [Fact]
        public void APathsProximity_MeasuresTheArmAsTheTwinsLinksAre()
        {
            DrivePlan plan = MixedTakeTests.AnyMixed(MixedTakeTests.MixedLayout(2), 7);
            List<TrackPoint> track = ReplayAdd.Commanded(plan);
            var arm = new ArmTouch(Profile, Model);
            TrackPoint p = track[track.Count / 2];
            Vector3 tool = arm.At(p.Pose, p.Gripper).Tool;
            // A box round the tool point (the pads either side of it) is touched; moved out from the base axis beyond it, it is clear, and the gap
            // grows as it goes.
            var at = new SafetyObject(201, tool, Quaternion.Identity, new Vector3(0.04f));
            Assert.True(arm.Touches(at));
            Assert.True(arm.Gap(at) < 0f);
            var outward = Vector3.Normalize(new Vector3(tool.X, tool.Y, 0f));
            float last = float.NegativeInfinity;
            foreach (float d in new[] { 0.10f, 0.14f, 0.18f })
            {
                var beyond = new SafetyObject(202, tool + d * outward, Quaternion.Identity, new Vector3(0.01f));
                float g = arm.Gap(beyond, 0.5f);
                Assert.True(g > 0f && g >= last, $"{g} after {last}");
                last = g;
            }

            var far = new SafetyObject(203, new Vector3(-0.3f, 0.6f, -0.17f), Quaternion.Identity, new Vector3(0.02f));
            Assert.True(float.IsPositiveInfinity(arm.Gap(far)));
            var near = new PathProximity(Profile, Model, track, new[] { at, far }, 1);
            Assert.True(near.Contact(0) && near.Touched(0) && !near.Near(0));
            Assert.False(near.Touched(1) || near.Near(1));
            Assert.True(near.ClosestAt(0) >= 0 && near.ClosestAt(1) < 0);
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
