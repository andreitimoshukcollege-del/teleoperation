using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// The arm at one pose as the twin's links are (<see cref="ShapeGap.TouchBodies"/>: the model's bodies, the finger pads where the
    /// gripper holds them), in the base frame, and how close it comes to an object: its own shape when it has one
    /// (<see cref="ShapeGap"/>), else its box. For measuring a take after it is planned or recorded (docs/adr/0038: the dry run's
    /// contacts, a replay's edits), never for planning a move.
    /// </summary>
    public sealed class ArmTouch
    {
        private readonly RobotArmProfile _profile;
        private readonly CollisionBody[] _bodies; // the model's without its CAD fingers; the pads last when it had them
        private readonly bool _pads;
        private readonly int _count;
        private readonly Vector3[] _c, _x, _y, _z, _h, _lo, _hi;
        private readonly Dictionary<(byte, Vector3), ShapeGap> _shapes = new Dictionary<(byte, Vector3), ShapeGap>();
        private readonly ArmCollisionModel _model;
        private ArmPose _pose;
        private float _gripper;

        public ArmTouch(in RobotArmProfile profile, ArmCollisionModel model)
        {
            _profile = profile;
            _model = model;
            CollisionBody[] all = ShapeGap.TouchBodies(model, 0f);
            _pads = all.Length >= 2 && all[all.Length - 1].Name == ShapeGap.RightPadAtZero.Name && all[all.Length - 2].Name == ShapeGap.LeftPadAtZero.Name;
            _bodies = all;
            _count = all.Length;
            _c = new Vector3[_count];
            _x = new Vector3[_count];
            _y = new Vector3[_count];
            _z = new Vector3[_count];
            _h = new Vector3[_count];
            _lo = new Vector3[_count];
            _hi = new Vector3[_count];
        }

        /// <summary>The arm's profile and collision model this measures with.</summary>
        public RobotArmProfile Profile => _profile;

        public ArmCollisionModel Model => _model;

        /// <summary>The pads' midpoint at the last <see cref="At"/>, base frame: the tool point.</summary>
        public Vector3 Tool { get; private set; }

        /// <summary>The lowest corner of the arm's boxes at the last <see cref="At"/>, base frame z.</summary>
        public float Lowest { get; private set; }

        /// <summary>Puts the arm at <paramref name="pose"/>, the gripper at <paramref name="gripper"/> (open when NaN).</summary>
        public ArmTouch At(in ArmPose pose, float gripper)
        {
            _pose = pose;
            _gripper = float.IsNaN(gripper) ? SafetyScript.ApproachGripper : gripper;
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ArmCollision.ComputeFrames(_profile, pose, frames);
            // The pads were placed for an open gripper (TouchBodies at 0): each moves along the gripper's Y by its finger's travel since.
            float moved = ShapeGap.FingerTravel(_gripper) - ShapeGap.FingerTravel(0f), low = float.MaxValue;
            for (int i = 0; i < _count; i++)
            {
                CollisionBody b = _bodies[i];
                Vector3 centre = b.Center;
                if (_pads && i == _count - 2) centre += new Vector3(0f, -moved, 0f); // the left pad, on the gripper along -Y
                if (_pads && i == _count - 1) centre += new Vector3(0f, moved, 0f); // the right, along +Y
                LinkFrame f = frames[(int)b.Link];
                Quaternion turn = f.Rotation * b.Rotation;
                _c[i] = f.ToBase(centre);
                _x[i] = Vector3.Transform(Vector3.UnitX, turn);
                _y[i] = Vector3.Transform(Vector3.UnitY, turn);
                _z[i] = Vector3.Transform(Vector3.UnitZ, turn);
                _h[i] = b.HalfExtents;
                Vector3 reach = Vector3.Abs(_x[i]) * _h[i].X + Vector3.Abs(_y[i]) * _h[i].Y + Vector3.Abs(_z[i]) * _h[i].Z;
                _lo[i] = _c[i] - reach;
                _hi[i] = _c[i] + reach;
                low = MathF.Min(low, _lo[i].Z);
            }

            Lowest = low;
            Tool = frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper);
            return this;
        }

        /// <summary>Whether any of the arm's boxes overlaps <paramref name="o"/>'s box grown by <paramref name="grow"/> (shrunk when below 0).</summary>
        public bool Overlaps(in SafetyObject o, float grow)
        {
            Vector3 half = grow >= 0 ? o.HalfExtents + new Vector3(grow) : o.HalfExtents - Vector3.Min(new Vector3(-grow), o.HalfExtents - new Vector3(0.0005f));
            Vector3 ax = Vector3.Transform(Vector3.UnitX, o.Rotation), ay = Vector3.Transform(Vector3.UnitY, o.Rotation), az = Vector3.Transform(Vector3.UnitZ, o.Rotation);
            Vector3 reach = Vector3.Abs(ax) * half.X + Vector3.Abs(ay) * half.Y + Vector3.Abs(az) * half.Z;
            Vector3 lo = o.Center - reach, hi = o.Center + reach;
            for (int i = 0; i < _count; i++)
            {
                if (_hi[i].X < lo.X || _lo[i].X > hi.X || _hi[i].Y < lo.Y || _lo[i].Y > hi.Y || _hi[i].Z < lo.Z || _lo[i].Z > hi.Z) continue;
                if (ArmCollision.BoxesOverlap(_c[i], _x[i], _y[i], _z[i], _h[i], o.Center, ax, ay, az, half)) return true;
            }

            return false;
        }

        /// <summary>
        /// When the arm at the last <see cref="At"/> touches <paramref name="o"/> (<see cref="Touches"/>), the link that does and a point of
        /// the contact (docs/adr/0038 "M, motion": contacts by height and link): of the arm's boxes that overlap the object's box, the one
        /// whose centre comes nearest the object's centre, and the point of the object's box nearest that centre. False when it does not touch.
        /// </summary>
        public bool Touching(in SafetyObject o, out ArmLink link, out Vector3 point)
        {
            link = ArmLink.Gripper;
            point = default;
            if (!Touches(o)) return false;
            Vector3 ax = Vector3.Transform(Vector3.UnitX, o.Rotation), ay = Vector3.Transform(Vector3.UnitY, o.Rotation), az = Vector3.Transform(Vector3.UnitZ, o.Rotation);
            float best = float.MaxValue;
            for (int i = 0; i < _count; i++)
            {
                if (!ArmCollision.BoxesOverlap(_c[i], _x[i], _y[i], _z[i], _h[i], o.Center, ax, ay, az, o.HalfExtents)) continue;
                Vector3 d = _c[i] - o.Center;
                Vector3 local = new Vector3(Math.Clamp(Vector3.Dot(d, ax), -o.HalfExtents.X, o.HalfExtents.X), Math.Clamp(Vector3.Dot(d, ay), -o.HalfExtents.Y, o.HalfExtents.Y),
                    Math.Clamp(Vector3.Dot(d, az), -o.HalfExtents.Z, o.HalfExtents.Z));
                float far = d.Length();
                if (far >= best) continue;
                best = far;
                link = _bodies[i].Link;
                point = o.Center + ax * local.X + ay * local.Y + az * local.Z;
            }

            return best < float.MaxValue;
        }

        /// <summary>Whether the arm at the last <see cref="At"/> touches <paramref name="o"/>: its own shape when it has one, else its box.</summary>
        public bool Touches(in SafetyObject o)
        {
            if (!Overlaps(o, 0f)) return false;
            return o.Shape == null || ShapeOf(o).Touches(_pose, _gripper);
        }

        private ShapeGap ShapeOf(in SafetyObject o)
        {
            var key = (o.Id, o.Center);
            if (!_shapes.TryGetValue(key, out ShapeGap? shape))
            {
                shape = new ShapeGap(_profile, _model, o);
                _shapes[key] = shape;
            }

            return shape;
        }

        /// <summary>
        /// The arm's signed gap to <paramref name="o"/> at the last <see cref="At"/>, m: above 0 clear (to 1 mm, at most
        /// <paramref name="cap"/>: farther reads as <see cref="float.PositiveInfinity"/>), below 0 into it. Against its own shape when it
        /// has one (<see cref="ShapeGap.SignedGap"/>, 0.5 mm steps, its own ladder's reach), else its box (1 mm steps, at most its
        /// smallest half size deep).
        /// </summary>
        public float Gap(in SafetyObject o, float cap = PathProximity.NearMeters)
        {
            if (!Overlaps(o, cap)) return float.PositiveInfinity;
            if (o.Shape != null)
            {
                float g = ShapeOf(o).SignedGap(_pose, _gripper);
                return g >= GapLadder.MaxGap - 1e-4f ? MathF.Min(cap, BoxGap(o, cap)) : g;
            }

            return BoxGap(o, cap);
        }

        private float BoxGap(in SafetyObject o, float cap)
        {
            if (Overlaps(o, 0f))
            {
                float deepest = SafetyScript.MaxDepth(o);
                int a = 0, b = (int)MathF.Floor(deepest / 0.001f) + 1; // touching shrunk by a mm, clear shrunk by b (past: touching all the way)
                while (b - a > 1)
                {
                    int mid = (a + b) / 2;
                    if (Overlaps(o, -mid * 0.001f)) a = mid;
                    else b = mid;
                }

                return -a * 0.001f - 1e-5f;
            }

            int lo = 0, hi = (int)MathF.Round(cap / 0.001f); // clear grown by lo mm, touching grown by hi
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (Overlaps(o, mid * 0.001f)) hi = mid;
                else lo = mid;
            }

            return lo * 0.001f;
        }
    }

    /// <summary>
    /// How close the arm came to each object along a take's track (docs/adr/0038 decision 2: a replay's edit should plausibly change what
    /// the take touches): for each object, the smallest signed gap over the track (<see cref="ArmTouch.Gap"/>, every
    /// <c>stride</c>-th point), and where. An object it touched or came within <see cref="TouchMeters"/> of is the path's own; one within
    /// <see cref="NearMeters"/> is near it; the rest are far.
    /// </summary>
    public sealed class PathProximity
    {
        /// <summary>Touched, or within this of it: an edit there can turn a contact into a miss.</summary>
        public const float TouchMeters = 0.01f;

        /// <summary>Near the path: an edit there can turn a miss into a contact.</summary>
        public const float NearMeters = 0.05f;

        private readonly float[] _gap;
        private readonly int[] _at;

        public PathProximity(in RobotArmProfile profile, ArmCollisionModel model, IReadOnlyList<TrackPoint> track, IReadOnlyList<SafetyObject> objects, int stride = 2)
        {
            Track = track;
            Objects = objects;
            Arm = new ArmTouch(profile, model);
            _gap = new float[objects.Count];
            _at = new int[objects.Count];
            for (int i = 0; i < objects.Count; i++)
            {
                _gap[i] = float.PositiveInfinity;
                _at[i] = -1;
            }

            for (int t = 0; t < track.Count; t += Math.Max(1, stride))
            {
                Arm.At(track[t].Pose, track[t].Gripper);
                for (int i = 0; i < objects.Count; i++)
                {
                    float g = Arm.Gap(objects[i]);
                    if (g < _gap[i])
                    {
                        _gap[i] = g;
                        _at[i] = t;
                    }
                }
            }
        }

        public IReadOnlyList<TrackPoint> Track { get; }
        public IReadOnlyList<SafetyObject> Objects { get; }
        public ArmTouch Arm { get; }

        /// <summary>The smallest signed gap the track came to object <paramref name="i"/>, m (+inf beyond <see cref="NearMeters"/>).</summary>
        public float Gap(int i) => _gap[i];

        /// <summary>The track's index at <see cref="Gap"/> (-1 when far).</summary>
        public int ClosestAt(int i) => _at[i];

        /// <summary>Whether the arm touched object <paramref name="i"/> by the model.</summary>
        public bool Contact(int i) => _gap[i] <= 0f;

        /// <summary>Touched, or within <see cref="TouchMeters"/>.</summary>
        public bool Touched(int i) => _gap[i] <= TouchMeters;

        /// <summary>Not touched, but within <see cref="NearMeters"/>.</summary>
        public bool Near(int i) => _gap[i] > TouchMeters && _gap[i] <= NearMeters;

        /// <summary>The smallest signed gap the track (every <paramref name="stride"/>-th point) comes to <paramref name="o"/> (+inf beyond <paramref name="cap"/>), and where.</summary>
        public static float GapOver(ArmTouch arm, IReadOnlyList<TrackPoint> track, in SafetyObject o, int stride, out int at, float cap = NearMeters)
        {
            float best = float.PositiveInfinity;
            at = -1;
            for (int t = 0; t < track.Count; t += Math.Max(1, stride))
            {
                float g = arm.At(track[t].Pose, track[t].Gripper).Gap(o, cap);
                if (g < best)
                {
                    best = g;
                    at = t;
                }
            }

            return best;
        }
    }
}
