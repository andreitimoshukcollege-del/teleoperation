using System;
using System.Collections.Generic;
using System.Linq;
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

        private long _stamp = long.MinValue;

        /// <summary>
        /// The capture stamp for a command sent at sender ticks <paramref name="now"/>: <paramref name="now"/>, or one tick past the last stamp
        /// when that is not earlier. RobotHost refuses a command whose stamp is not newer than the last it accepted (stale), and behind its
        /// clock the replay sends the commands due one after another in one pass of its loop: stamped alike, all but the first were dropped
        /// (docs/adr/0038 "M, motion": 4-19 commands a replay on the twin, none in their sources).
        /// </summary>
        public long Stamp(long now)
        {
            _stamp = _stamp == long.MinValue || now > _stamp ? now : _stamp + 1;
            return _stamp;
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

    /// <summary>
    /// One replay's edit (docs/adr/0038 decision 2): the record that says it, the world edit that makes it, and the edited object's box
    /// afterwards (none for a removal). No edit when <c>Pose.Id</c> is 0, and then <see cref="Note"/> says why; <see cref="AddNote"/> says
    /// why the group's add was tried here and found no spot (docs/adr/0038 "M, motion": the twin check logged neither).
    /// </summary>
    public readonly struct ReplayEditDraw
    {
        public readonly ReplayRecord Record;
        public readonly WorldEditOp Op;
        public readonly ObjectPose Pose;
        public readonly SafetyObject? After;
        public readonly string? Note;
        public readonly string? AddNote;

        public ReplayEditDraw(in ReplayRecord record, WorldEditOp op, in ObjectPose pose, SafetyObject? after, string? note = null, string? addNote = null)
        {
            Record = record;
            Op = op;
            Pose = pose;
            After = after;
            Note = note;
            AddNote = addNote;
        }

        /// <summary>No edit, and why.</summary>
        public static ReplayEditDraw Nothing(string why, string? addNote = null) => new ReplayEditDraw(default, default, default, null, why, addNote);

        /// <summary>This edit, saying why the add tried before it found nothing.</summary>
        public ReplayEditDraw WithAddNote(string? addNote) => new ReplayEditDraw(Record, Op, Pose, After, Note, addNote);
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
            List<Vector3> spots = LowSpots(add, profile, (float)scene.Table.Top);
            if (spots.Count == 0) return ReplayEditDraw.Nothing("no add: the take has no moment outside its quiet start and end");
            // Objects of the layout on the table the take kept clear of, then parked ones: each in an order drawn from the replay.
            List<int> onTable = AddCandidatesOnTable(objects, scene, touched, used, add, profile);
            var parked = new List<byte>();
            foreach (byte id in add.Parked)
            {
                int k = scene.IndexOf(id);
                if (k >= 0 && scene.Objects[k].OperatorMovable && scene.Objects[k].Movable && !used.Contains("add:" + id) && IndexOf(objects, id) < 0) parked.Add(id);
            }

            if (onTable.Count == 0 && parked.Count == 0)
                return ReplayEditDraw.Nothing(FormattableString.Invariant($"no add: no object to stand there (none on the table that the take kept {AddFarMeters * 100:0} cm from and holds nothing, none parked)"));
            Shuffle(onTable, ref r);
            Shuffle(parked, ref r);
            ObjectShapes? shapes = WorldView.ShapesOf(scene);
            var tally = new SpotTally();
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
                    // At the yaw it would stand at: its own on the table, else one drawn.
                    double yaw = pass == 0 ? yawNow : (2 * r.NextDouble() - 1) * Math.PI;
                    SafetyObject Placed(RigidTransform pose) => pass == 0
                        ? Moved(objects[i], pose)
                        : WorldView.Bounds(scene.Objects[k], scale, pose.X, pose.Y, pose.Z, pose.Qw, pose.Qx, pose.Qy, pose.Qz, shapes);
                    if (!SpotOnPath(scene, objects, k, i, spots, yaw, scale, rest, scaleOf, plan, profile, add, ref r, tally, Placed, out RigidTransform at, out SafetyObject after)) continue;
                    var record = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Add, id, (float)at.X, (float)at.Y, (float)yaw);
                    return new ReplayEditDraw(record, pass == 0 ? WorldEditOp.Move : WorldEditOp.Unpark, Pose(id, at), after);
                }
            }

            return ReplayEditDraw.Nothing(FormattableString.Invariant($"no add: {spots.Count} moments, {onTable.Count} object(s) on the table and {parked.Count} parked tried; ") + tally);
        }

        /// <summary>Where the arm's lowest point passed at every other moment of the add's track outside the quiet start and end (and their slack): x, y, and z above the table.</summary>
        private static List<Vector3> LowSpots(ReplayAdd add, in RobotArmProfile profile, float table)
        {
            MixedOptions options = add.Options;
            var spots = new List<Vector3>(add.Track.Count / 2 + 1);
            double from = options.QuietStartSeconds + AddQuietSlackSeconds, to = add.TakeSeconds - options.QuietEndSeconds - AddQuietSlackSeconds;
            for (int t = 0; t < add.Track.Count; t += 2)
            {
                TrackPoint p = add.Track[t];
                if (p.At < from || p.At > to) continue;
                Vector3 low = LowestPoint(profile, add.Model, p.Pose, p.Gripper);
                spots.Add(new Vector3(low.X, low.Y, low.Z - table));
            }

            return spots;
        }

        /// <summary>Why spots under the arm did not take an object (counted over every moment and object tried), for the log.</summary>
        private sealed class SpotTally
        {
            public int TooHigh, Bearing, Rules, Quiet;

            public override string ToString() => FormattableString.Invariant(
                $"moments too high over the table for the object {TooHigh}, beyond the reach bearing {Bearing}, off the table, out of reach or on another object {Rules}; spots that met the quiet start or end {Quiet}");
        }

        /// <summary>
        /// Scene object <paramref name="k"/> (index <paramref name="skip"/> in <paramref name="objects"/> when it stands on the table, else -1) stood
        /// with its footprint centred under the arm's lowest point at one of <paramref name="spots"/> where that point passed lower than the
        /// object is tall less <see cref="AddDepthMeters"/>, by the scene's rules, clear of the take's quiet start and end as planned and as the
        /// arm went: a moment drawn among those that work. The pose, and the object there (<paramref name="placed"/> makes it).
        /// </summary>
        private static bool SpotOnPath(WorldScene scene, IReadOnlyList<SafetyObject> objects, int k, int skip, List<Vector3> spots, double yaw, double scale, int rest,
            Func<byte, double>? scaleOf, DrivePlan plan, in RobotArmProfile profile, ReplayAdd add, ref SeededRng r, SpotTally tally, Func<RigidTransform, SafetyObject> placed,
            out RigidTransform pose, out SafetyObject after)
        {
            pose = default;
            after = default;
            float height = (float)(2 * (scene.RestingZ(k, scale, rest) - scene.Table.Top));
            var fits = new List<int>();
            for (int m = 0; m < spots.Count; m++)
            {
                Vector3 at = spots[m];
                if (at.Z >= height - AddDepthMeters) { tally.TooHigh++; continue; }
                if (scene.ReachBearing > 0 && Math.Abs(Math.Atan2(at.Y, at.X)) > scene.ReachBearing) { tally.Bearing++; continue; }
                if (Fits(scene, objects, k, skip, at.X, at.Y, yaw, scale, rest, scaleOf, out _)) fits.Add(m);
                else tally.Rules++;
            }

            for (int tries = 0; tries < 12 && fits.Count > 0; tries++)
            {
                int pick = (int)(r.NextDouble() * fits.Count) % fits.Count;
                Vector3 at = spots[fits[pick]];
                fits[pick] = fits[fits.Count - 1];
                fits.RemoveAt(fits.Count - 1);
                Fits(scene, objects, k, skip, at.X, at.Y, yaw, scale, rest, scaleOf, out pose);
                after = placed(pose);
                if (KeepsQuiet(plan, after, profile, add.Model, add.Options) && KeepsQuietAsWent(add, profile, after)) return true;
                tally.Quiet++;
            }

            return false;
        }

        /// <summary>
        /// The objects of the layout on the table an add may move (indices into <paramref name="objects"/>): those the headset may move, not
        /// scenery, holding nothing, not raised (their bottom within 1 cm of the table), not touched by the take (<paramref name="touched"/>),
        /// kept <see cref="AddFarMeters"/> from all through it, and not added by the group already.
        /// </summary>
        internal static List<int> AddCandidatesOnTable(IReadOnlyList<SafetyObject> objects, WorldScene scene, ICollection<byte> touched, ICollection<string> used, ReplayAdd add,
            in RobotArmProfile profile)
        {
            float table = (float)scene.Table.Top;
            var list = new List<int>();
            for (int i = 0; i < objects.Count; i++)
            {
                SafetyObject o = objects[i];
                int k = scene.IndexOf(o.Id);
                if (k < 0 || o.Scenery || Holds(objects, i) || o.Min.Z > table + 0.01f || !scene.Objects[k].OperatorMovable || touched.Contains(o.Id)) continue;
                if (used.Contains("add:" + o.Id) || !KeptClear(add, profile, o)) continue;
                list.Add(i);
            }

            return list;
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
        /// Replay <paramref name="replay"/> (from 1) of group <paramref name="group"/>, try <paramref name="attempt"/> (from 0): an edit of
        /// <paramref name="objects"/> (the source take's, as they stood at its start) that is not among <paramref name="used"/> (the group's
        /// edits so far, <see cref="Key"/>) and that the collision model expects to change what the source's commands touch (docs/adr/0038
        /// "M, motion", the 2026-11-23 review: a removal and a move far from the path left the take's one contact as it was):
        /// <list type="bullet">
        /// <item>an object the arm touched or came within <see cref="PathProximity.TouchMeters"/> of (by the model on the source's track,
        /// <paramref name="near"/>; or one the source moved, <paramref name="touched"/>) is removed, shifted away until the track keeps
        /// <see cref="ClearMeters"/> from it, or moved elsewhere <see cref="AwayMeters"/> clear of it: a contact made a miss;</item>
        /// <item>one it passed within <see cref="PathProximity.NearMeters"/> of is shifted toward the path or moved onto it, until the track goes
        /// <see cref="MeetMeters"/> into it: a miss made a contact;</item>
        /// <item>with <paramref name="add"/>, one of the group's first two replays is an <c>add</c> (<see cref="Add"/>): its drawn one
        /// (<see cref="AddReplay"/>), or the first when that has nothing else to edit, or the second when the first found no spot;</item>
        /// <item>nothing farther from the path is edited: with nothing else left, no edit, and <see cref="ReplayEditDraw.Note"/> says why.</item>
        /// </list>
        /// The kinds come in an order drawn from the group, each replay starting one further on; within a kind, the objects nearest the path
        /// first. Nothing that holds another object is edited, a raised one is only removed, and a moved or shifted one must keep the take's
        /// quiet start and end. Deterministic from the group, the replay and the try (a placement RobotHost refuses is drawn again).
        /// </summary>
        public static ReplayEditDraw Draw(ulong group, int replay, int attempt, DrivePlan plan, IReadOnlyList<SafetyObject> objects, WorldScene scene,
            ICollection<byte> touched, in RobotArmProfile profile, ICollection<string>? used = null, Func<byte, double>? scaleOf = null, ReplayAdd? add = null,
            PathProximity? near = null)
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
            ArmCollisionModel model = add?.Model ?? profile.Collision!;
            near ??= new PathProximity(profile, model, add?.Track ?? ReplayAdd.Commanded(plan), objects, 2);
            bool added = false;
            foreach (string k in used) added |= k.StartsWith("add:", StringComparison.Ordinal);
            bool mayAdd = add != null && !added && replay <= 2, addTurn = mayAdd && (replay == AddReplay(group) || replay == 2);
            string? addNote = null;
            if (addTurn)
            {
                ReplayEditDraw a = Add(group, ref r, plan, objects, scene, touched, profile, used, scaleOf, add!);
                if (a.Pose.Id != 0) return a;
                addNote = a.Note;
            }

            ReplayEditDraw e = PathEdit(group, order, replay, ref r, plan, objects, scene, touched, profile, used, scaleOf, add, near);
            if (e.Pose.Id != 0) return e.WithAddNote(addNote);
            if (mayAdd && !addTurn)
            {
                // Nothing else to edit in the replay before the add's: the add now, so a group with nothing near its path still gets one.
                ReplayEditDraw a = Add(group, ref r, plan, objects, scene, touched, profile, used, scaleOf, add!);
                if (a.Pose.Id != 0) return a;
                addNote = a.Note;
            }

            return ReplayEditDraw.Nothing(e.Note ?? "no edit", addNote);
        }

        /// <summary>A shift or a move away from the path leaves the source's track at least this far from the object, m: a contact made a miss.</summary>
        public const float ClearMeters = 0.005f;

        /// <summary>A move elsewhere leaves the object at least this far from the track, m.</summary>
        public const float AwayMeters = 0.02f;

        /// <summary>A shift or a move onto the path puts the object where the source's track goes at least this far into it, m: a miss made a contact.</summary>
        public const float MeetMeters = 0.003f;

        /// <summary>The longest shift, m: a near object shifted toward the path goes as far as its gap and up to 2.5 cm more.</summary>
        public const float ShiftFarthestMeters = 0.08f;

        /// <summary>The remove, shift or move of <see cref="Draw"/>, by how near the path each object is; no edit (with why) when nothing near it is left.</summary>
        private static ReplayEditDraw PathEdit(ulong group, ReplayEdit[] order, int replay, ref SeededRng r, DrivePlan plan, IReadOnlyList<SafetyObject> objects, WorldScene scene,
            ICollection<byte> touched, in RobotArmProfile profile, ICollection<string> used, Func<byte, double>? scaleOf, ReplayAdd? add, PathProximity near)
        {
            float table = (float)scene.Table.Top;
            ArmCollisionModel model = add?.Model ?? profile.Collision!;
            MixedOptions options = add?.Options ?? new MixedOptions();
            // The path's own objects (touched, within 1 cm, or moved by the source) and those near it, each nearest first.
            var own = new List<int>();
            var close = new List<int>();
            var far = new List<string>();
            for (int i = 0; i < objects.Count; i++)
            {
                if (objects[i].Scenery) continue;
                if (near.Touched(i) || touched.Contains(objects[i].Id)) own.Add(i);
                else if (near.Near(i)) close.Add(i);
                else far.Add(objects[i].Id.ToString(System.Globalization.CultureInfo.InvariantCulture));
            }

            own.Sort((a, b) => (touched.Contains(objects[b].Id) ? 1 : 0).CompareTo(touched.Contains(objects[a].Id) ? 1 : 0) is int c && c != 0 ? c : near.Gap(a).CompareTo(near.Gap(b)));
            close.Sort((a, b) => near.Gap(a).CompareTo(near.Gap(b)));
            // An object something stands on (a stand, a plank's support) is never edited: what it holds would fall. A raised one (on a stand)
            // is only removed: shifted off its stand, RobotHost would refuse it in the air.
            bool Free(int i) => !objects[i].Scenery && !Holds(objects, i);
            bool Movable(int i)
            {
                int k = scene.IndexOf(objects[i].Id);
                return Free(i) && objects[i].Min.Z <= table + 0.01f && k >= 0 && scene.Objects[k].OperatorMovable;
            }

            List<Vector3>? spots = null;
            var tally = new SpotTally();
            for (int t = 0; t < order.Length; t++)
            {
                ReplayEdit kind = order[(replay - 1 + t) % order.Length];
                foreach (int i in kind == ReplayEdit.Remove ? own : own.Concat(close))
                {
                    SafetyObject o = objects[i];
                    bool mine = own.Contains(i);
                    var remove = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Remove, o.Id);
                    switch (kind)
                    {
                        case ReplayEdit.Remove:
                            if (!Free(i) || used.Contains(Key(remove))) continue;
                            return new ReplayEditDraw(remove, WorldEditOp.Park, new ObjectPose(o.Id, 0, 0, 0, 0, 1, 0, 0, 0), null);
                        case ReplayEdit.Shift:
                        {
                            if (!Movable(i) || used.Contains("shift:" + o.Id)) continue;
                            if (Shifted(group, i, mine, ref r, plan, objects, scene, profile, model, options, scaleOf, near, out ReplayEditDraw shift)) return shift;
                            continue;
                        }

                        case ReplayEdit.Move:
                        {
                            if (!Movable(i) || used.Contains("move:" + o.Id)) continue;
                            int k = scene.IndexOf(o.Id);
                            int rest = WorldScene.RestOf(o.Rotation.W, o.Rotation.X, o.Rotation.Y, o.Rotation.Z, out double yawNow);
                            double scale = scaleOf?.Invoke(o.Id) ?? 1.0;
                            if (!mine)
                            {
                                // Onto the path: under the arm's way where it passed low enough to go into it, as an add stands one.
                                ReplayAdd onto = add ?? new ReplayAdd(near.Track, plan.TotalSeconds - plan.ScriptStart, Array.Empty<byte>(), model, options);
                                spots ??= LowSpots(onto, profile, table);
                                if (SpotOnPath(scene, objects, k, i, spots, yawNow, scale, rest, scaleOf, plan, profile, onto, ref r, tally, p => Moved(o, p), out RigidTransform at,
                                        out SafetyObject after)
                                    && PathProximity.GapOver(near.Arm, near.Track, after, 2, out _) <= -MeetMeters)
                                {
                                    var move = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Move, o.Id, (float)at.X, (float)at.Y, (float)yawNow);
                                    return new ReplayEditDraw(move, WorldEditOp.Move, Pose(o.Id, at), after);
                                }

                                continue;
                            }

                            // Elsewhere, clear of the path: a spot the scene's rules give, the track kept AwayMeters from it.
                            for (int tries = 0; tries < 40; tries++)
                            {
                                if (!scene.DrawSpot(k, scale, rest, ref r, out double x, out double y, out double yaw)) continue;
                                if (Math.Sqrt((x - o.Center.X) * (x - o.Center.X) + (y - o.Center.Y) * (y - o.Center.Y)) < 0.05) continue; // elsewhere, not a nudge
                                if (!Placeable(scene, objects, i, x, y, yaw, scaleOf, out RigidTransform at)) continue;
                                SafetyObject after = Moved(o, at);
                                if (PathProximity.GapOver(near.Arm, near.Track, after, 2, out _, AwayMeters + 0.001f) < AwayMeters) continue;
                                if (!KeepsQuiet(plan, after, profile, model, options)) continue;
                                var move = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Move, o.Id, (float)at.X, (float)at.Y, (float)yaw);
                                return new ReplayEditDraw(move, WorldEditOp.Move, Pose(o.Id, at), after);
                            }

                            continue;
                        }
                    }
                }
            }

            string List(IEnumerable<int> xs) => string.Join(" ", xs.Select(i => objects[i].Id.ToString(System.Globalization.CultureInfo.InvariantCulture)));
            return ReplayEditDraw.Nothing(FormattableString.Invariant(
                $"nothing near the path left to edit (touched or within {PathProximity.TouchMeters * 100:0} cm: {(own.Count > 0 ? List(own) : "none")}; within {PathProximity.NearMeters * 100:0} cm: {(close.Count > 0 ? List(close) : "none")}; farther, not edited: {(far.Count > 0 ? string.Join(" ", far) : "none")}; done: {(used.Count > 0 ? string.Join(", ", used) : "none")})"));
        }

        /// <summary>
        /// Object <paramref name="i"/> shifted on the table: away from the path (<paramref name="away"/>, the path's own object) until the
        /// source's track keeps <see cref="ClearMeters"/> from it, or toward it (a near one) until the track goes <see cref="MeetMeters"/> into
        /// it; along the way from the arm to the object where the track came closest, or up to 90 degrees off it, the shortest shift that does
        /// it from a few, by the scene's rules and clear of the take's quiet start and end.
        /// </summary>
        private static bool Shifted(ulong group, int i, bool away, ref SeededRng r, DrivePlan plan, IReadOnlyList<SafetyObject> objects, WorldScene scene,
            in RobotArmProfile profile, ArmCollisionModel model, MixedOptions options, Func<byte, double>? scaleOf, PathProximity near, out ReplayEditDraw shift)
        {
            shift = default;
            SafetyObject o = objects[i];
            int at = Math.Max(0, near.ClosestAt(i));
            Vector3 tool = near.Arm.At(near.Track[at].Pose, near.Track[at].Gripper).Tool;
            var toObject = new Vector2(o.Center.X - tool.X, o.Center.Y - tool.Y);
            toObject = toObject.Length() > 1e-4f ? Vector2.Normalize(toObject) : new Vector2(o.Center.X, o.Center.Y) / MathF.Max(1e-4f, new Vector2(o.Center.X, o.Center.Y).Length());
            Vector2 way = away ? toObject : -toObject;
            float gap = MathF.Max(0f, near.Gap(i));
            float extra = (float)(0.005 + 0.020 * r.NextDouble());
            float[] distances = away ? new[] { 0.015f, 0.025f, 0.035f, 0.05f, 0.065f } : new[] { gap + extra, gap + 0.5f * extra + 0.005f, gap + 0.03f };
            float[] turns = { 0f, 30f, -30f, 60f, -60f, 90f, -90f };
            foreach (float d in distances)
            {
                if (d > ShiftFarthestMeters) continue;
                foreach (float turn in turns)
                {
                    float c = MathF.Cos(turn * MathF.PI / 180f), s = MathF.Sin(turn * MathF.PI / 180f);
                    Vector2 delta = d * new Vector2(c * way.X - s * way.Y, s * way.X + c * way.Y);
                    if (!Placeable(scene, objects, i, o.Center.X + delta.X, o.Center.Y + delta.Y, null, scaleOf, out RigidTransform pose)) continue;
                    SafetyObject after = Moved(o, pose);
                    float g = PathProximity.GapOver(near.Arm, near.Track, after, 2, out _, ClearMeters + 0.001f);
                    if (away ? g < ClearMeters : g > -MeetMeters) continue;
                    if (!KeepsQuiet(plan, after, profile, model, options)) continue;
                    var record = new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Shift, o.Id, delta.X, delta.Y);
                    shift = new ReplayEditDraw(record, WorldEditOp.Move, Pose(o.Id, pose), after);
                    return true;
                }
            }

            return false;
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

        /// <summary>Whether another of <paramref name="objects"/> stands on object <paramref name="i"/>: its bottom at this one's top, their footprints overlapping.</summary>
        public static bool Holds(IReadOnlyList<SafetyObject> objects, int i)
        {
            SafetyObject o = objects[i];
            for (int k = 0; k < objects.Count; k++)
            {
                if (k == i) continue;
                SafetyObject p = objects[k];
                // Its bottom at this one's top: within 1.5 cm above it, or up to 3 cm below it (a tilted object's box reaches below what it
                // rests on: the twin check's elephant lying on a stand read 1.5 cm low, the stand was taken for free, removed, and its layout
                // could not be put back).
                if (p.Min.Z <= o.Max.Z + 0.015f && p.Min.Z >= o.Max.Z - 0.03f && p.Center.Z > o.Max.Z && p.Min.X < o.Max.X && p.Max.X > o.Min.X && p.Min.Y < o.Max.Y && p.Max.Y > o.Min.Y)
                    return true;
            }

            return false;
        }

        private static SafetyObject Moved(in SafetyObject o, Vector3 center, Quaternion rotation) =>
            new SafetyObject(o.Id, center, rotation, o.HalfExtents, o.Movable, o.Mass, o.Tippy, o.Shape, o.Scenery, o.Role);

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
