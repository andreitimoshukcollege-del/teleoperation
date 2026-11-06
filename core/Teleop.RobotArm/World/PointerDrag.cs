using System;
using Teleop.RobotArm.Types;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.World
{
    /// <summary>
    /// Dragging the scene's objects with a pointing ray, while the robot is off (docs/adr/0031 amendment 5): what a hand's
    /// pinch does when its ray hits an object instead of a button.
    ///
    /// <para><b>The ray</b> is the headset's, in the robot's base frame (origin and direction, any length). <b>Grabbing:</b> a
    /// pinch that starts while the ray passes within an operator-movable object's bounding sphere (half its size's diagonal,
    /// so a fallen object is caught as well as a standing one) takes the nearest such object along the ray. <b>Dragging:</b>
    /// the object follows the ray's point on the horizontal plane through its centre, by the offset it had at the grab, kept
    /// inside the table and the placement region as <see cref="ObjectEditGesture"/> keeps it, at the yaw it had; a fallen
    /// object is stood up there. <b>Releasing</b> the pinch asks once for that pose (<see cref="TryTakeRequest"/>) when it moved
    /// at least <see cref="MinMoveMeters"/>, or when it was not standing where it may stand (so a fallen object can be stood
    /// up by a click). A ray nearly parallel to the plane, or one that loses tracking, cancels.</para>
    ///
    /// <para>One object at a time, by one hand. Allocation-free and deterministic; no clock.</para>
    /// </summary>
    public sealed class PointerDrag
    {
        public const int Hands = 2;

        /// <summary>A release closer than this to the grab asks nothing (an upright object on the table).</summary>
        public const double MinMoveMeters = 0.005;

        private readonly WorldScene _scene;
        private readonly bool[] _wasPinched = new bool[Hands];
        private int _owner = -1, _index = -1;
        private double _planeZ, _offsetX, _offsetY, _startX, _startY, _yaw;
        private double _x, _y;
        private bool _wasUpright;
        private bool _hasRequest;
        private ObjectPose _request;

        public PointerDrag(WorldScene scene)
        {
            _scene = scene ?? throw new ArgumentNullException(nameof(scene));
        }

        public bool IsDragging => _owner >= 0;

        /// <summary>The hand dragging (0 or 1), or -1.</summary>
        public int Owner => _owner;

        public byte DraggedId => _owner >= 0 ? _scene.Objects[_index].Id : (byte)0;

        /// <summary>Where the dragged object would be put now: upright on the table.</summary>
        public RigidTransform Preview => _index >= 0 ? _scene.PoseAt(_index, _x, _y, _yaw) : RigidTransform.Identity;

        /// <summary>
        /// The object the ray from (<paramref name="ox"/>, <paramref name="oy"/>, <paramref name="oz"/>) along
        /// (<paramref name="dx"/>, <paramref name="dy"/>, <paramref name="dz"/>) hits first, and how far along the ray; -1 when none.
        /// Only operator-movable objects the world has posed.
        /// </summary>
        public int Hit(double ox, double oy, double oz, double dx, double dy, double dz, WorldState world, out double distance)
        {
            distance = double.MaxValue;
            double n = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            if (!(n > 1e-9)) return -1;
            dx /= n;
            dy /= n;
            dz /= n;
            int best = -1;
            for (int i = 0; i < _scene.Objects.Length; i++)
            {
                SceneObject o = _scene.Objects[i];
                if (!o.OperatorMovable || !world.TryGet(o.Id, out ObjectPose p, out _)) continue;
                double r = 0.5 * Math.Sqrt(o.SizeX * o.SizeX + o.SizeY * o.SizeY + o.SizeZ * o.SizeZ);
                double cx = p.X - ox, cy = p.Y - oy, cz = p.Z - oz;
                double along = cx * dx + cy * dy + cz * dz;
                if (along <= 0) continue;
                double off2 = cx * cx + cy * cy + cz * cz - along * along;
                if (off2 > r * r) continue;
                double enter = along - Math.Sqrt(r * r - off2);
                if (enter < distance)
                {
                    distance = enter;
                    best = i;
                }
            }

            return best;
        }

        /// <summary>One hand's frame: whether it is tracked, whether it pinches, and its ray. Call for each hand every frame.</summary>
        public void Update(int hand, bool tracked, bool pinched, double ox, double oy, double oz, double dx, double dy, double dz, WorldState world)
        {
            if (hand < 0 || hand >= Hands) throw new ArgumentOutOfRangeException(nameof(hand));
            bool rising = pinched && tracked && !_wasPinched[hand];
            _wasPinched[hand] = pinched && tracked;

            if (_owner == hand)
            {
                if (!tracked || !Drag(ox, oy, oz, dx, dy, dz))
                {
                    _owner = -1; // lost, or pointing along the plane: nothing is asked
                    return;
                }

                if (!pinched)
                {
                    _owner = -1;
                    double moved = Math.Sqrt((_x - _startX) * (_x - _startX) + (_y - _startY) * (_y - _startY));
                    if (moved >= MinMoveMeters || !_wasUpright)
                    {
                        _request = ObjectPose.Of(_scene.Objects[_index].Id, 0, Preview);
                        _hasRequest = true;
                    }
                }

                return;
            }

            if (_owner < 0 && rising) TryGrab(hand, ox, oy, oz, dx, dy, dz, world);
        }

        /// <summary>The move the last release asks for, once.</summary>
        public bool TryTakeRequest(out ObjectPose pose)
        {
            pose = _request;
            if (!_hasRequest) return false;
            _hasRequest = false;
            return true;
        }

        /// <summary>Back to as constructed: nothing held, nothing asked.</summary>
        public void Reset()
        {
            _owner = _index = -1;
            _hasRequest = false;
            for (int i = 0; i < Hands; i++) _wasPinched[i] = false;
        }

        private void TryGrab(int hand, double ox, double oy, double oz, double dx, double dy, double dz, WorldState world)
        {
            int index = Hit(ox, oy, oz, dx, dy, dz, world, out double distance);
            if (index < 0 || !world.TryGet(_scene.Objects[index].Id, out ObjectPose p, out _)) return;
            // The drag plane is level through the object's centre: the ray's point there, plus the offset of the centre from
            // where the ray crossed it at the grab, is where the object goes. A ray that never crosses it grabs nothing.
            if (Math.Abs(dz) < 1e-3 || (p.Z - oz) / dz <= 0) return;
            double t = (p.Z - oz) / dz;
            _owner = hand;
            _index = index;
            _planeZ = p.Z;
            _offsetX = p.X - (ox + dx * t);
            _offsetY = p.Y - (oy + dy * t);
            _yaw = Math.Atan2(2.0 * (p.Qw * p.Qz + p.Qx * p.Qy), 1.0 - 2.0 * (p.Qy * p.Qy + p.Qz * p.Qz));
            RigidTransform upright = _scene.PoseAt(index, p.X, p.Y, _yaw);
            double tilt = Math.Abs(p.Qw * upright.Qw + p.Qx * upright.Qx + p.Qy * upright.Qy + p.Qz * upright.Qz);
            _wasUpright = Math.Abs(p.Z - upright.Z) < 0.01 && tilt > Math.Cos(0.5 * 0.1) && _scene.FitsOnTable(index, p.X, p.Y);
            Clamp(p.X, p.Y);
            _startX = _x;
            _startY = _y;
        }

        private bool Drag(double ox, double oy, double oz, double dx, double dy, double dz)
        {
            if (Math.Abs(dz) < 1e-3) return false;
            double t = (_planeZ - oz) / dz;
            if (t <= 0) return false;
            Clamp(ox + dx * t + _offsetX, oy + dy * t + _offsetY);
            return true;
        }

        private void Clamp(double x, double y)
        {
            SceneSurface t = _scene.Table;
            // 1 mm inside the bounds: the pose goes out as float32, whose rounding must not land it a hair outside.
            double r = _scene.Objects[_index].FootprintRadius + 0.001;
            double x0 = Math.Max(t.XMin, _scene.RegionXMin) + r, x1 = Math.Min(t.XMax, _scene.RegionXMax) - r;
            double y0 = Math.Max(t.YMin, _scene.RegionYMin) + r, y1 = Math.Min(t.YMax, _scene.RegionYMax) - r;
            _x = x0 <= x1 ? Math.Min(Math.Max(x, x0), x1) : (x0 + x1) / 2;
            _y = y0 <= y1 ? Math.Min(Math.Max(y, y0), y1) : (y0 + y1) / 2;
        }
    }
}
