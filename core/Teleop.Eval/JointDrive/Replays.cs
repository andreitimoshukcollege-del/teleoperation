using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;

namespace Teleop.Eval.JointDrive
{
    /// <summary>One command joint-drive sent during a take: when (seconds from the take's start), the pose, the gripper and the speed it carried.</summary>
    public readonly struct SentCommand
    {
        public readonly double At;
        public readonly ArmPose Pose;
        public readonly float Gripper, Speed;

        public SentCommand(double at, in ArmPose pose, float gripper, float speed)
        {
            At = at;
            Pose = pose;
            Gripper = gripper;
            Speed = speed;
        }
    }

    /// <summary>
    /// Plays a take's commands back (docs/adr/0038 decision 2): each comes out once the replay's clock (seconds from its take's start)
    /// reaches the time it was sent at in the source take, in the order they were sent, none early and none twice. Behind the clock,
    /// the commands due come out one after another.
    /// </summary>
    public sealed class TapePlayer
    {
        private readonly IReadOnlyList<SentCommand> _tape;
        private int _next;

        public TapePlayer(IReadOnlyList<SentCommand> tape) => _tape = tape ?? throw new ArgumentNullException(nameof(tape));

        /// <summary>How many have come out.</summary>
        public int Played => _next;

        /// <summary>Whether every command has come out and the clock is past the last by <paramref name="after"/> seconds.</summary>
        public bool Done(double clock, double after) => _next >= _tape.Count && (_tape.Count == 0 || clock > _tape[_tape.Count - 1].At + after);

        /// <summary>The next command due at <paramref name="clock"/>, if one is.</summary>
        public bool TryNext(double clock, out SentCommand command)
        {
            command = default;
            if (_next >= _tape.Count || _tape[_next].At > clock) return false;
            command = _tape[_next++];
            return true;
        }
    }

    /// <summary>A point of the arm's way through a take: seconds from its start, the pose, and the gripper fraction commanded.</summary>
    public readonly struct TrackPoint
    {
        public readonly double At;
        public readonly ArmPose Pose;
        public readonly float Gripper;

        public TrackPoint(double at, in ArmPose pose, float gripper)
        {
            At = at;
            Pose = pose;
            Gripper = gripper;
        }
    }

    /// <summary>
    /// What a replay's <c>add</c> needs besides what every edit has (docs/adr/0038 decision 2; the twin check's review asked for an edit
    /// that puts an obstacle into the path: a safe take's unsafe twin): the arm's way through the source take (as measured when the take
    /// measured it, else as commanded), how long the take ran, the pool's parked objects it may stand on the table, and the model and
    /// options the take's quiet start and end are kept with.
    /// </summary>
    public sealed class ReplayAdd
    {
        public ReplayAdd(IReadOnlyList<TrackPoint> track, double takeSeconds, IReadOnlyList<byte> parked, ArmCollisionModel model, MixedOptions options)
        {
            Track = track ?? throw new ArgumentNullException(nameof(track));
            TakeSeconds = takeSeconds;
            Parked = parked ?? Array.Empty<byte>();
            Model = model ?? throw new ArgumentNullException(nameof(model));
            Options = options ?? throw new ArgumentNullException(nameof(options));
        }

        public IReadOnlyList<TrackPoint> Track { get; }
        public double TakeSeconds { get; }

        /// <summary>The ids of objects the layout left out that an add may stand on the table (movable obstacles the headset may move).</summary>
        public IReadOnlyList<byte> Parked { get; }

        public ArmCollisionModel Model { get; }
        public MixedOptions Options { get; }

        /// <summary>A planned take's commanded way, every tick from its start (the dry run's, and joint-drive's when the take measured too little of its own).</summary>
        public static List<TrackPoint> Commanded(DrivePlan plan)
        {
            var track = new List<TrackPoint>();
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds + 1e-9; t += HandModel.TickSeconds)
            {
                float g = plan.GripperAt(t);
                track.Add(new TrackPoint(t - plan.ScriptStart, plan.Sample(t), float.IsNaN(g) ? 0f : g));
            }

            return track;
        }
    }

    /// <summary>One replay's edit (docs/adr/0038 decision 2): the record that says it, the world edit that makes it, and the edited object's box afterwards (none for a removal).</summary>
    public readonly struct ReplayEditDraw
    {
        public readonly ReplayRecord Record;
        public readonly WorldEditOp Op;
        public readonly ObjectPose Pose;
        public readonly SafetyObject? After;

        public ReplayEditDraw(in ReplayRecord record, WorldEditOp op, in ObjectPose pose, SafetyObject? after)
        {
            Record = record;
            Op = op;
            Pose = pose;
            After = after;
        }
    }

    /// <summary>
    /// The edits of a mixed take's replays (docs/adr/0038 decision 2): each replay of a group takes the next of remove, shift and
    /// move in an order drawn from the group, and of that kind the first edit the group has not made yet (<c>used</c>), over the
    /// objects in the order the take came close to them (a removal first of an object the take moved, which it touched). A shift
    /// moves an object an aim or a pass was made at, along that move's approach; a move stands an object at a spot the scene's own
    /// rules give (in reach, on the table, clear of the others). When a kind has nothing new, the next kind; when no kind has, no
    /// edit. Its numbers come from the group, the replay and the try (a placement RobotHost refuses is drawn again). Deterministic.
    /// <para>With a <see cref="ReplayAdd"/>, one of the group's first two replays (drawn from the group) is an <c>add</c> whenever a
    /// spot for one exists (<see cref="Add"/>): an obstacle stood where the source's arm passed low; the other replays are as above.</para>
    /// </summary>
    public static class ReplayEdits
    {
        /// <summary>A shift moves its object this far along the approach of the move that came closest to it, m (uniform between the two).</summary>
        public const float ShiftMinMeters = 0.010f, ShiftMaxMeters = 0.040f;

        /// <summary>An edit's key in a group's record of edits made: its kind and object, whatever its numbers.</summary>
        public static string Key(in ReplayRecord r) => r.Edit switch
        {
            ReplayEdit.Remove => "remove:",
            ReplayEdit.Shift => "shift:",
            ReplayEdit.Move => "move:",
            ReplayEdit.Add => "add:",
            _ => "none:",
        } + r.ObjectId.ToString(System.Globalization.CultureInfo.InvariantCulture);

        /// <summary>Which of a group's replays is its <c>add</c> (when a spot for one exists): the first or the second, drawn from the group.</summary>
        public static int AddReplay(ulong group)
        {
            var rng = new SeededRng(unchecked(group * 0x9E3779B97F4A7C15UL + 0xADD));
            return rng.NextDouble() < 0.5 ? 1 : 2;
        }

        /// <summary>How far below an object's top the arm's lowest point must pass for an add to stand it there, m: the arm goes at least that deep into it.</summary>
        public const float AddDepthMeters = 0.015f;

        /// <summary>An object on the table is moved for an add only when the source take kept this far from its box throughout, m: where it stood made no difference to what the take touched.</summary>
        public const float AddFarMeters = 0.01f;

        /// <summary>The add's moments keep this far inside the take's quiet start and end, s: the arm meets the object's box a little before its lowest point under it.</summary>
        public const double AddQuietSlackSeconds = 0.25;

        /// <summary>
        /// Replay <paramref name="group"/>'s <c>add</c> (docs/adr/0038 decision 2): a movable obstacle stood with its footprint centred under
        /// the arm's lowest point (the twin's links, the pads where the gripper held them) at a moment of the source take outside its
        /// quiet start and end, where that point passed lower over the table than the object is tall less <see cref="AddDepthMeters"/>;
        /// by the scene's own rules (on the table, in reach, clear of the others as they stand) and clear of the take's quiet start and
        /// end (as planned, and as the arm went). First an object of the layout on the table (moved there, <see cref="WorldEditOp.Move"/>)
        /// that the take did not touch and kept <see cref="AddFarMeters"/> from, holding nothing and not raised; when none will do, one the
        /// layout left out (<see cref="WorldEditOp.Unpark"/>), upright. The moment is drawn among those that work, not the lowest, so
        /// contacts come at every height the take passed at. Nothing the group has added already. No edit (<c>Pose.Id</c> 0) when nothing
        /// works.
        /// </summary>
        public static ReplayEditDraw Add(ulong group, ref SeededRng r, DrivePlan plan, IReadOnlyList<SafetyObject> objects, WorldScene scene, ICollection<byte> touched,
            in RobotArmProfile profile, ICollection<string> used, Func<byte, double>? scaleOf, ReplayAdd add)
        {
            MixedOptions options = add.Options;
            float table = (float)scene.Table.Top;
            // Where the arm passed lowest, every tick of the take but its quiet start and end.
            var spots = new List<Vector3>(add.Track.Count);
            double from = options.QuietStartSeconds + AddQuietSlackSeconds, to = add.TakeSeconds - options.QuietEndSeconds - AddQuietSlackSeconds;
            foreach (TrackPoint p in add.Track)
            {
                if (p.At < from || p.At > to) continue;
                Vector3 low = LowestPoint(profile, add.Model, p.Pose, p.Gripper);
                spots.Add(new Vector3(low.X, low.Y, low.Z - table));
            }

            if (spots.Count == 0) return default;
            // Objects of the layout on the table the take kept clear of, then parked ones: each in an order drawn from the replay.
            var onTable = new List<int>();
            for (int i = 0; i < objects.Count; i++)
            {
                SafetyObject o = objects[i];
                int k = scene.IndexOf(o.Id);
                if (k < 0 || o.Scenery || Holds(objects, i) || o.Min.Z > table + 0.01f || !scene.Objects[k].OperatorMovable || touched.Contains(o.Id)) continue;
                if (used.Contains("add:" + o.Id) || !KeptClear(add, profile, o)) continue;
                onTable.Add(i);
            }

            var parked = new List<byte>();
            foreach (byte id in add.Parked)
            {
                int k = scene.IndexOf(id);
                if (k >= 0 && scene.Objects[k].OperatorMovable && scene.Objects[k].Movable && !used.Contains("add:" + id) && IndexOf(objects, id) < 0) parked.Add(id);
            }

            Shuffle(onTable, ref r);
            Shuffle(parked, ref r);
            ObjectShapes? shapes = WorldView.ShapesOf(scene);
            for (int pass = 0; pass < 2; pass++)
            {
                int n = pass == 0 ? onTable.Count : parked.Count;
                for (int c = 0; c < n; c++)
                {
                    int i = pass == 0 ? onTable[c] : -1;
                    byte id = pass == 0 ? objects[i].Id : parked[c];
                    int k = scene.IndexOf(id);
                    double scale = scaleOf?.Invoke(id) ?? 1.0, yawNow = 0;
                    int rest = pass == 0 ? WorldScene.RestOf(objects[i].Rotation.W, objects[i].Rotation.X, objects[i].Rotation.Y, objects[i].Rotation.Z, out yawNow) : WorldScene.RestUpright;
                    float height = (float)(2 * (scene.RestingZ(k, scale, rest) - scene.Table.Top));
                    var deep = new List<int>();
                    for (int m = 0; m < spots.Count; m++) if (spots[m].Z < height - AddDepthMeters) deep.Add(m);
                    for (int tries = 0; tries < 12 && deep.Count > 0; tries++)
                    {
                        Vector3 at = spots[deep[(int)(r.NextDouble() * deep.Count) % deep.Count]];
                        double yaw = pass == 0 && tries == 0 ? yawNow : (2 * r.NextDouble() - 1) * Math.PI;
                        if (scene.ReachBearing > 0 && Math.Abs(Math.Atan2(at.Y, at.X)) > scene.ReachBearing) continue;
                        if (!Fits(scene, objects, k, i, at.X, at.Y, yaw, scale, rest, scaleOf, out RigidTransform pose)) continue;
                        SafetyObject after = pass == 0
                            ? Moved(objects[i], pose)
                            : WorldView.Bounds(scene.Objects[k], scale, pose.X, pose.Y, pose.Z, pose.Qw, pose.Qx, pose.Qy, pose.Qz, shapes);
                        if (!KeepsQuiet(plan, after, profile, add.Model, options) || !KeepsQuietAsWent(add, profile, after)) continue;
                        var record = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Add, id, (float)pose.X, (float)pose.Y, (float)yaw);
                        return new ReplayEditDraw(record, pass == 0 ? WorldEditOp.Move : WorldEditOp.Unpark, Pose(id, pose), after);
                    }
                }
            }

            return default;
        }

        /// <summary>The lowest point of the arm's bodies as the twin's links are (the pads where <paramref name="gripper"/> holds them), arm base frame.</summary>
        public static Vector3 LowestPoint(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose pose, float gripper)
        {
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ArmCollision.ComputeFrames(profile, pose, frames);
            Vector3 low = new Vector3(0f, 0f, float.MaxValue);
            foreach (CollisionBody b in ShapeGap.TouchBodies(model, gripper))
            {
                LinkFrame f = frames[(int)b.Link];
                for (int c = 0; c < 8; c++)
                {
                    var corner = new Vector3((c & 1) == 0 ? -b.HalfExtents.X : b.HalfExtents.X, (c & 2) == 0 ? -b.HalfExtents.Y : b.HalfExtents.Y, (c & 4) == 0 ? -b.HalfExtents.Z : b.HalfExtents.Z);
                    Vector3 p = f.ToBase(b.Center + Vector3.Transform(corner, b.Rotation));
                    if (p.Z < low.Z) low = p;
                }
            }

            return low;
        }

        /// <summary>Whether the arm, as it went through the take (every other tick), kept <see cref="AddFarMeters"/> from <paramref name="o"/>'s box.</summary>
        private static bool KeptClear(ReplayAdd add, in RobotArmProfile profile, in SafetyObject o)
        {
            ArmCollisionModel check = SafetyScript.WithObjects(add.Model, new[] { o }, AddFarMeters, 0f);
            for (int t = 0; t < add.Track.Count; t += 2) if (!ArmCollision.IsValid(profile, check, add.Track[t].Pose)) return false;
            return true;
        }

        /// <summary>Whether the arm as it went (<see cref="ReplayAdd.Track"/>) kept the quiet margin from <paramref name="after"/> in the take's quiet start and end.</summary>
        private static bool KeepsQuietAsWent(ReplayAdd add, in RobotArmProfile profile, in SafetyObject after)
        {
            ArmCollisionModel check = SafetyScript.WithObjects(add.Model, new[] { after }, add.Options.QuietMarginMeters);
            foreach (TrackPoint p in add.Track)
            {
                if (p.At >= add.Options.QuietStartSeconds && p.At <= add.TakeSeconds - add.Options.QuietEndSeconds) continue;
                if (!ArmCollision.IsValid(profile, check, p.Pose)) return false;
            }

            return true;
        }

        private static void Shuffle<T>(List<T> items, ref SeededRng r)
        {
            for (int i = items.Count - 1; i > 0; i--)
            {
                int j = (int)(r.NextDouble() * (i + 1)) % (i + 1);
                (items[i], items[j]) = (items[j], items[i]);
            }
        }

        /// <summary>
        /// Whether scene object <paramref name="k"/> may stand at (<paramref name="x"/>, <paramref name="y"/>) turned to <paramref name="yaw"/> on
        /// rest <paramref name="rest"/> at <paramref name="scale"/>, by the scene's rules (on the table, in reach) and clear of every other of
        /// <paramref name="objects"/> as it stands (all but index <paramref name="skip"/>, the object itself when it is on the table); the
        /// pose it would have there.
        /// </summary>
        private static bool Fits(WorldScene scene, IReadOnlyList<SafetyObject> objects, int k, int skip, double x, double y, double yaw, double scale, int rest,
            Func<byte, double>? scaleOf, out RigidTransform at)
        {
            at = default;
            if (!scene.FitsOnTable(k, x, y, yaw, scale, rest) || !scene.InReach(k, x, y)) return false;
            for (int b = 0; b < objects.Count; b++)
            {
                if (b == skip) continue;
                int kb = scene.IndexOf(objects[b].Id);
                if (kb < 0) continue;
                Quaternion q = objects[b].Rotation;
                int restB = WorldScene.RestOf(q.W, q.X, q.Y, q.Z, out double yawB);
                if (scene.Overlaps(k, x, y, yaw, kb, objects[b].Center.X, objects[b].Center.Y, yawB, scale, scaleOf?.Invoke(objects[b].Id) ?? 1.0, rest, restB)) return false;
            }

            at = scene.PoseAt(k, x, y, yaw, scale, rest);
            return true;
        }

        /// <summary>
        /// Replay <paramref name="replay"/> (from 1) of group <paramref name="group"/>, try <paramref name="attempt"/> (from 0): an edit
        /// of <paramref name="objects"/> (the source take's, as they stood at its start) for the take <paramref name="plan"/> planned
        /// that is not among <paramref name="used"/> (the group's edits so far, <see cref="Key"/>); <paramref name="touched"/>, the
        /// objects the source take moved; <paramref name="scaleOf"/>, each object's size factor (1 when not given). No edit
        /// (<c>Pose.Id</c> 0) when nothing new is left.
        /// </summary>
        public static ReplayEditDraw Draw(ulong group, int replay, int attempt, DrivePlan plan, IReadOnlyList<SafetyObject> objects, WorldScene scene,
            ICollection<byte> touched, in RobotArmProfile profile, ICollection<string>? used = null, Func<byte, double>? scaleOf = null, ReplayAdd? add = null)
        {
            var rng = new SeededRng(unchecked(group * 0x9E3779B97F4A7C15UL + 0x5EED));
            ReplayEdit[] order = { ReplayEdit.Remove, ReplayEdit.Shift, ReplayEdit.Move };
            for (int i = order.Length - 1; i > 0; i--)
            {
                int j = (int)(rng.NextDouble() * (i + 1)) % (i + 1);
                (order[i], order[j]) = (order[j], order[i]);
            }

            var r = new SeededRng(unchecked(group * 0x9E3779B97F4A7C15UL ^ ((ulong)replay * 1000 + (ulong)attempt)));
            used ??= Array.Empty<string>();
            if (add != null && replay == AddReplay(group))
            {
                ReplayEditDraw a = Add(group, ref r, plan, objects, scene, touched, profile, used, scaleOf, add);
                if (a.Pose.Id != 0) return a;
            }

            float table = (float)scene.Table.Top;
            int[] ranked = Ranked(plan, objects, profile, out Dictionary<int, PlannedMove> closest);
            // An object something stands on (a stand, a plank's support) is never edited: what it holds would fall. A raised one
            // (on a stand) is only removed: shifted off its stand, RobotHost would refuse it in the air.
            bool Free(int i) => !objects[i].Scenery && !Holds(objects, i);
            bool Movable(int i)
            {
                int k = scene.IndexOf(objects[i].Id);
                return Free(i) && objects[i].Min.Z <= table + 0.01f && k >= 0 && scene.Objects[k].OperatorMovable;
            }

            for (int t = 0; t < order.Length; t++)
            {
                ReplayEdit kind = order[(replay - 1 + t) % order.Length];
                foreach (int i in Candidates(kind, ranked, objects, touched))
                {
                    SafetyObject o = objects[i];
                    var remove = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Remove, o.Id);
                    switch (kind)
                    {
                        case ReplayEdit.Remove:
                            if (!Free(i) || used.Contains(Key(remove))) continue;
                            return new ReplayEditDraw(remove, WorldEditOp.Park, new ObjectPose(o.Id, 0, 0, 0, 0, 1, 0, 0, 0), null);
                        case ReplayEdit.Shift:
                        {
                            if (!Movable(i) || !closest.TryGetValue(i, out PlannedMove m) || used.Contains("shift:" + o.Id)) continue;
                            // Along the approach of that move: toward the arm when it stopped short (a miss made a hit), away when it went
                            // in (a hit made a miss), each three times in four; the other way, or shorter, when the scene does not take it.
                            float heading = MathF.Atan2(o.Center.Y, o.Center.X) + (float.IsNaN(m.ApproachDegrees) ? 0f : m.ApproachDegrees * MathF.PI / 180f);
                            var way = new Vector2(MathF.Cos(heading), MathF.Sin(heading));
                            float sign = (m.GapMeters < 0 ? 1f : -1f) * (r.NextDouble() < 0.75 ? 1f : -1f);
                            float d = ShiftMinMeters + (float)r.NextDouble() * (ShiftMaxMeters - ShiftMinMeters);
                            foreach ((float sg, float dd) in new[] { (sign, d), (-sign, d), (sign, ShiftMinMeters), (-sign, ShiftMinMeters) })
                            {
                                Vector2 delta = sg * dd * way;
                                if (!Placeable(scene, objects, i, o.Center.X + delta.X, o.Center.Y + delta.Y, null, scaleOf, out RigidTransform at)) continue;
                                var shift = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Shift, o.Id, delta.X, delta.Y);
                                return new ReplayEditDraw(shift, WorldEditOp.Move, Pose(o.Id, at), Moved(o, at));
                            }

                            continue;
                        }

                        case ReplayEdit.Move:
                        {
                            if (!Movable(i) || used.Contains("move:" + o.Id)) continue;
                            int k = scene.IndexOf(o.Id);
                            int rest = WorldScene.RestOf(o.Rotation.W, o.Rotation.X, o.Rotation.Y, o.Rotation.Z, out _);
                            double scale = scaleOf?.Invoke(o.Id) ?? 1.0;
                            for (int tries = 0; tries < 40; tries++)
                            {
                                if (!scene.DrawSpot(k, scale, rest, ref r, out double x, out double y, out double yaw)) continue;
                                if (Math.Sqrt((x - o.Center.X) * (x - o.Center.X) + (y - o.Center.Y) * (y - o.Center.Y)) < 0.05) continue; // elsewhere, not a nudge
                                if (!Placeable(scene, objects, i, x, y, yaw, scaleOf, out RigidTransform at)) continue;
                                var move = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Move, o.Id, (float)at.X, (float)at.Y, (float)yaw);
                                return new ReplayEditDraw(move, WorldEditOp.Move, Pose(o.Id, at), Moved(o, at));
                            }

                            continue;
                        }
                    }
                }
            }

            return default;
        }

        /// <summary>The objects a kind of edit tries, in order: for a removal those the take moved first; then every one as the take came close to it.</summary>
        private static IEnumerable<int> Candidates(ReplayEdit kind, int[] ranked, IReadOnlyList<SafetyObject> objects, ICollection<byte> touched)
        {
            if (kind == ReplayEdit.Remove)
            {
                foreach (int i in ranked) if (touched.Contains(objects[i].Id)) yield return i;
            }

            foreach (int i in ranked)
            {
                if (kind == ReplayEdit.Remove && touched.Contains(objects[i].Id)) continue;
                yield return i;
            }
        }

        /// <summary>
        /// The objects (not scenery) in the order the take came close to them: those its aims and passes went at, by their planned
        /// closest approach, then the rest by how near the pads passed their box's centre; and each aimed-at one's closest move.
        /// </summary>
        public static int[] Ranked(DrivePlan plan, IReadOnlyList<SafetyObject> objects, in RobotArmProfile profile, out Dictionary<int, PlannedMove> closest)
        {
            closest = new Dictionary<int, PlannedMove>();
            foreach (PlannedMove m in plan.Moves)
            {
                if (m.ObjectId == 0 || float.IsNaN(m.GapMeters)) continue;
                int i = IndexOf(objects, m.ObjectId);
                if (i < 0 || objects[i].Scenery) continue;
                if (!closest.TryGetValue(i, out PlannedMove best) || m.GapMeters < best.GapMeters) closest[i] = m;
            }

            var nearest = new float[objects.Count];
            for (int i = 0; i < nearest.Length; i++) nearest[i] = float.MaxValue;
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds; t += 0.1)
            {
                ArmCollision.ComputeFrames(profile, plan.Sample(t), frames);
                Vector3 pads = frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper);
                for (int i = 0; i < objects.Count; i++) nearest[i] = MathF.Min(nearest[i], Vector3.Distance(pads, objects[i].Center));
            }

            var order = new List<int>();
            for (int i = 0; i < objects.Count; i++) if (!objects[i].Scenery) order.Add(i);
            Dictionary<int, PlannedMove> c = closest;
            order.Sort((a, b) =>
            {
                bool ma = c.TryGetValue(a, out PlannedMove pa), mb = c.TryGetValue(b, out PlannedMove pb);
                if (ma != mb) return ma ? -1 : 1;
                int byGap = ma ? pa.GapMeters.CompareTo(pb.GapMeters) : 0;
                return byGap != 0 ? byGap : nearest[a] != nearest[b] ? nearest[a].CompareTo(nearest[b]) : a.CompareTo(b);
            });
            return order.ToArray();
        }

        /// <summary>
        /// Whether object <paramref name="i"/> may stand at (<paramref name="x"/>, <paramref name="y"/>), turned to
        /// <paramref name="yaw"/> (null: as it is) on its rest, by the scene's rules (on the table, in reach) and clear of every other
        /// object as it stands; the pose it would have there.
        /// </summary>
        private static bool Placeable(WorldScene scene, IReadOnlyList<SafetyObject> objects, int i, double x, double y, double? yaw, Func<byte, double>? scaleOf,
            out RigidTransform at)
        {
            at = default;
            SafetyObject o = objects[i];
            int k = scene.IndexOf(o.Id);
            if (k < 0) return false;
            int rest = WorldScene.RestOf(o.Rotation.W, o.Rotation.X, o.Rotation.Y, o.Rotation.Z, out double yawNow);
            double turn = yaw ?? yawNow, scale = scaleOf?.Invoke(o.Id) ?? 1.0;
            if (!scene.FitsOnTable(k, x, y, turn, scale, rest) || !scene.InReach(k, x, y)) return false;
            for (int b = 0; b < objects.Count; b++)
            {
                if (b == i) continue;
                int kb = scene.IndexOf(objects[b].Id);
                if (kb < 0) continue;
                Quaternion q = objects[b].Rotation;
                int restB = WorldScene.RestOf(q.W, q.X, q.Y, q.Z, out double yawB);
                if (scene.Overlaps(k, x, y, turn, kb, objects[b].Center.X, objects[b].Center.Y, yawB, scale, scaleOf?.Invoke(objects[b].Id) ?? 1.0, rest, restB)) return false;
            }

            at = scene.PoseAt(k, x, y, turn, scale, rest);
            return true;
        }

        private static ObjectPose Pose(byte id, in RigidTransform t) =>
            new ObjectPose(id, 0, (float)t.X, (float)t.Y, (float)t.Z, (float)t.Qw, (float)t.Qx, (float)t.Qy, (float)t.Qz);

        private static SafetyObject Moved(in SafetyObject o, in RigidTransform t) =>
            Moved(o, new Vector3((float)t.X, (float)t.Y, (float)t.Z), Quaternion.Normalize(new Quaternion((float)t.Qx, (float)t.Qy, (float)t.Qz, (float)t.Qw)));

        /// <summary>Whether another of <paramref name="objects"/> stands on object <paramref name="i"/>: its bottom within 1 cm of this one's top, their footprints overlapping.</summary>
        public static bool Holds(IReadOnlyList<SafetyObject> objects, int i)
        {
            SafetyObject o = objects[i];
            for (int k = 0; k < objects.Count; k++)
            {
                if (k == i) continue;
                SafetyObject p = objects[k];
                if (MathF.Abs(p.Min.Z - o.Max.Z) < 0.01f && p.Min.X < o.Max.X && p.Max.X > o.Min.X && p.Min.Y < o.Max.Y && p.Max.Y > o.Min.Y) return true;
            }

            return false;
        }

        private static SafetyObject Moved(in SafetyObject o, Vector3 center, Quaternion rotation) =>
            new SafetyObject(o.Id, center, rotation, o.HalfExtents, o.Movable, o.Mass, o.Tippy, o.Shape, o.Scenery);

        private static int IndexOf(IReadOnlyList<SafetyObject> objects, byte id)
        {
            for (int i = 0; i < objects.Count; i++) if (objects[i].Id == id) return i;
            return -1;
        }

        /// <summary>
        /// Whether a replay of <paramref name="plan"/> keeps its quiet start and end (<see cref="MixedOptions"/>) clear of
        /// <paramref name="after"/>, an edited object where it will stand: no contact planned before the quiet start's end, nor in the
        /// last second.
        /// </summary>
        public static bool KeepsQuiet(DrivePlan plan, in SafetyObject after, in RobotArmProfile profile, ArmCollisionModel model, MixedOptions options)
        {
            ArmCollisionModel check = SafetyScript.WithObjects(model, new[] { after }, options.QuietMarginMeters);
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds; t += HandModel.TickSeconds)
            {
                if (t - plan.ScriptStart >= options.QuietStartSeconds && t <= plan.TotalSeconds - options.QuietEndSeconds) continue;
                if (!ArmCollision.IsValid(profile, check, plan.Sample(t))) return false;
            }

            return true;
        }
    }
}
