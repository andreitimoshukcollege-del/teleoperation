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
    /// move in an order drawn from the group, at the object the take came closest to (a removal prefers one the take moved, which
    /// it touched), and its numbers from the group, the replay and the attempt (a placement RobotHost refuses is drawn again).
    /// Deterministic: the same take, objects and draw give the same edit.
    /// </summary>
    public static class ReplayEdits
    {
        /// <summary>A shift moves its object this far along the approach of the move that came closest to it, m (uniform between the two).</summary>
        public const float ShiftMinMeters = 0.010f, ShiftMaxMeters = 0.040f;

        /// <summary>A move stands its object this far from the base axis, m (uniform), within the scene's placement region.</summary>
        public const float MoveMinRadius = 0.15f, MoveMaxRadius = 0.30f;

        /// <summary>
        /// Replay <paramref name="replay"/> (from 1) of group <paramref name="group"/>, try <paramref name="attempt"/> (from 0): its edit
        /// of <paramref name="objects"/> (the source take's, as they stood at its start) for the take <paramref name="plan"/> planned;
        /// <paramref name="touched"/>, the objects the source take moved. Removal when nothing else can be done.
        /// </summary>
        public static ReplayEditDraw Draw(ulong group, int replay, int attempt, DrivePlan plan, IReadOnlyList<SafetyObject> objects, WorldScene scene,
            ICollection<byte> touched, in RobotArmProfile profile)
        {
            var rng = new SeededRng(unchecked(group * 0x9E3779B97F4A7C15UL + 0x5EED));
            ReplayEdit[] order = { ReplayEdit.Remove, ReplayEdit.Shift, ReplayEdit.Move };
            for (int i = order.Length - 1; i > 0; i--)
            {
                int j = (int)(rng.NextDouble() * (i + 1)) % (i + 1);
                (order[i], order[j]) = (order[j], order[i]);
            }

            ReplayEdit kind = order[(replay - 1) % order.Length];
            var r = new SeededRng(unchecked(group * 0xBF58476D1CE4E5B9UL + (ulong)replay * 1000 + (ulong)attempt));
            int closest = Closest(plan, objects, profile, out PlannedMove? move, null);
            if (closest < 0) return default;
            int movable = scene.IndexOf(objects[closest].Id) >= 0 && scene.Objects[scene.IndexOf(objects[closest].Id)].OperatorMovable ? closest
                : Closest(plan, objects, profile, out move, i => scene.IndexOf(objects[i].Id) >= 0 && scene.Objects[scene.IndexOf(objects[i].Id)].OperatorMovable);
            if (kind != ReplayEdit.Remove && movable < 0) kind = ReplayEdit.Remove;
            if (attempt >= 4) kind = ReplayEdit.Remove; // placements kept being refused

            if (kind == ReplayEdit.Remove)
            {
                int which = closest;
                for (int i = 0; i < objects.Count; i++) if (touched.Contains(objects[i].Id) && !objects[i].Scenery) which = i;
                byte id = objects[which].Id;
                return new ReplayEditDraw(new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Remove, id), WorldEditOp.Park, new ObjectPose(id, 0, 0, 0, 0, 1, 0, 0, 0), null);
            }

            SafetyObject o = objects[movable];
            if (kind == ReplayEdit.Shift)
            {
                // Along the approach of the move that came closest to it: toward the arm when that move stopped short (a miss made a
                // hit), away when it went in (a hit made a miss), each three times in four.
                PlannedMove? m = move.HasValue && move.Value.ObjectId == o.Id ? move : null;
                float bearing = MathF.Atan2(o.Center.Y, o.Center.X);
                float heading = bearing + (m.HasValue && !float.IsNaN(m.Value.ApproachDegrees) ? m.Value.ApproachDegrees * MathF.PI / 180f : 0f);
                var way = new Vector2(MathF.Cos(heading), MathF.Sin(heading));
                bool missed = !m.HasValue || !(m.Value.GapMeters < 0);
                float sign = (missed ? -1f : 1f) * (r.NextDouble() < 0.75 ? 1f : -1f);
                float d = ShiftMinMeters + (float)r.NextDouble() * (ShiftMaxMeters - ShiftMinMeters);
                Vector2 delta = sign * d * way;
                var pose = new ObjectPose(o.Id, 0, o.Center.X + delta.X, o.Center.Y + delta.Y, o.Center.Z, o.Rotation.W, o.Rotation.X, o.Rotation.Y, o.Rotation.Z);
                return new ReplayEditDraw(new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Shift, o.Id, delta.X, delta.Y), WorldEditOp.Move, pose,
                    Moved(o, new Vector3(o.Center.X + delta.X, o.Center.Y + delta.Y, o.Center.Z), o.Rotation));
            }

            // Elsewhere in reach, in the placement region, turned to a drawn yaw on the rest it has.
            for (int k = 0; k < 40; k++)
            {
                float radius = MoveMinRadius + (float)r.NextDouble() * (MoveMaxRadius - MoveMinRadius);
                float azimuth = (float)((r.NextDouble() * 2 - 1) * 60 * Math.PI / 180);
                float x = radius * MathF.Cos(azimuth), y = radius * MathF.Sin(azimuth);
                if (x < scene.RegionXMin || x > scene.RegionXMax || y < scene.RegionYMin || y > scene.RegionYMax) continue;
                float turn = (float)((r.NextDouble() * 2 - 1) * Math.PI);
                Quaternion q = Quaternion.Normalize(Quaternion.Concatenate(o.Rotation, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, turn)));
                float yaw = MathF.Atan2(2f * (q.W * q.Z + q.X * q.Y), 1f - 2f * (q.Y * q.Y + q.Z * q.Z));
                var pose = new ObjectPose(o.Id, 0, x, y, o.Center.Z, q.W, q.X, q.Y, q.Z);
                return new ReplayEditDraw(new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Move, o.Id, x, y, yaw), WorldEditOp.Move, pose,
                    Moved(o, new Vector3(x, y, o.Center.Z), q));
            }

            return new ReplayEditDraw(new ReplayRecord(ReplayRole.Replay, group, ReplayEdit.Remove, o.Id), WorldEditOp.Park, new ObjectPose(o.Id, 0, 0, 0, 0, 1, 0, 0, 0), null);
        }

        private static SafetyObject Moved(in SafetyObject o, Vector3 center, Quaternion rotation) =>
            new SafetyObject(o.Id, center, rotation, o.HalfExtents, o.Movable, o.Mass, o.Tippy, o.Shape, o.Scenery);

        /// <summary>
        /// The object (of those <paramref name="allowed"/>, not scenery) the take came closest to: the one of its aims and passes whose
        /// planned approach was closest (that move), else the one whose box's centre the pads passed nearest. -1 when there is none.
        /// </summary>
        public static int Closest(DrivePlan plan, IReadOnlyList<SafetyObject> objects, in RobotArmProfile profile, out PlannedMove? move, Func<int, bool>? allowed)
        {
            move = null;
            int best = -1;
            float bestGap = float.MaxValue;
            foreach (PlannedMove m in plan.Moves)
            {
                if (m.ObjectId == 0 || float.IsNaN(m.GapMeters)) continue;
                int i = IndexOf(objects, m.ObjectId);
                if (i < 0 || objects[i].Scenery || (allowed != null && !allowed(i))) continue;
                if (m.GapMeters < bestGap)
                {
                    bestGap = m.GapMeters;
                    best = i;
                    move = m;
                }
            }

            if (best >= 0) return best;
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            float nearest = float.MaxValue;
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds; t += 0.1)
            {
                ArmCollision.ComputeFrames(profile, plan.Sample(t), frames);
                Vector3 pads = frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper);
                for (int i = 0; i < objects.Count; i++)
                {
                    if (objects[i].Scenery || (allowed != null && !allowed(i))) continue;
                    float d = Vector3.Distance(pads, objects[i].Center);
                    if (d < nearest)
                    {
                        nearest = d;
                        best = i;
                    }
                }
            }

            return best;
        }

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
