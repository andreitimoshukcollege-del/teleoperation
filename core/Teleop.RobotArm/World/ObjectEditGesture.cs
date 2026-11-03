using System;
using Teleop.RobotArm.Types;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.World
{
    /// <summary>The thresholds of <see cref="ObjectEditGesture"/> (docs/adr/0032 plan, phase P5).</summary>
    public sealed class ObjectEditConfig
    {
        /// <summary>How far beyond an object's footprint (metres, in the plane) the cursor may be and still grab it.</summary>
        public double GrabReachMeters { get; set; } = 0.06;

        /// <summary>How far above an object's top the cursor may be and still grab it.</summary>
        public double GrabAboveMeters { get; set; } = 0.10;

        /// <summary>A drag shorter than this, and turned less than <see cref="MinTurnRadians"/>, asks for nothing.</summary>
        public double MinMoveMeters { get; set; } = 0.005;

        public double MinTurnRadians { get; set; } = 0.035;

        public string? Validate()
        {
            if (!(GrabReachMeters >= 0) || !(GrabAboveMeters >= 0)) return "the grab reach and height must be zero or more";
            if (!(MinMoveMeters >= 0) || !(MinTurnRadians >= 0)) return "the smallest move and turn must be zero or more";
            return null;
        }
    }

    /// <summary>
    /// Moving the scene's objects by hand from the headset, while the robot is off (docs/adr/0031 amendment 2, phase P5).
    ///
    /// <para><b>The cursor</b> is where the hand would put the gripper: the operator mapping's tool pose for that hand, in the
    /// robot's base frame. A held curl (<see cref="GrabGesture"/>'s, not the double-curl lock) grabs the nearest operator-movable
    /// object whose footprint is within <see cref="ObjectEditConfig.GrabReachMeters"/> of the cursor and whose top is at most
    /// <see cref="ObjectEditConfig.GrabAboveMeters"/> below it. While held, the object follows the cursor's motion in x and y
    /// and turns with its heading, from where both were at the grab, stood upright on the table and kept inside the table and
    /// the placement region (<see cref="WorldScene.FitsOnTable"/>'s bounds). Opening the hand asks once for the object there
    /// (<see cref="TryTakeRequest"/>); a drag shorter than the minimum asks nothing, and losing the hand's tracking cancels.</para>
    ///
    /// <para><b>One object at a time,</b> by one hand: the other hand cannot grab until it is let go. The gesture decides nothing
    /// about other objects or the arms: RobotHost and Isaac refuse a pose that overlaps them, and the headset's twin shows the
    /// object where the world frames put it, so the operator sees the move cross the network (or not happen).</para>
    ///
    /// <para>Allocation-free and deterministic; no clock.</para>
    /// </summary>
    public sealed class ObjectEditGesture
    {
        public const int Hands = 2;

        private readonly WorldScene _scene;
        private readonly ObjectEditConfig _config;
        private readonly bool[] _wasHeld = new bool[Hands];
        private int _owner = -1, _index = -1;
        private bool _useApproach;
        private double _startX, _startY, _startYaw, _cursorX, _cursorY, _cursorHeading;
        private double _x, _y, _yaw;
        private bool _hasRequest;
        private ObjectPose _request;

        public ObjectEditGesture(WorldScene scene, ObjectEditConfig? config = null)
        {
            _scene = scene ?? throw new ArgumentNullException(nameof(scene));
            _config = config ?? new ObjectEditConfig();
            if (_config.Validate() is string bad) throw new ArgumentException(bad, nameof(config));
        }

        /// <summary>Whether an object is being dragged.</summary>
        public bool IsDragging => _owner >= 0;

        /// <summary>The hand dragging (0 or 1), or -1.</summary>
        public int Owner => _owner;

        /// <summary>The dragged object's id, or 0.</summary>
        public byte DraggedId => _index >= 0 && _owner >= 0 ? _scene.Objects[_index].Id : (byte)0;

        /// <summary>Where the dragged object would be put now: upright on the table at the dragged x, y and yaw.</summary>
        public RigidTransform Preview => _index >= 0 ? _scene.PoseAt(_index, _x, _y, _yaw) : RigidTransform.Identity;

        /// <summary>
        /// One hand's frame: whether it is tracked, whether its curl is held, and its cursor. <paramref name="world"/> says where
        /// each object is now; an object the world has not posed cannot be grabbed.
        /// </summary>
        public void Update(int hand, bool tracked, bool held, in RigidTransform cursor, WorldState world)
        {
            if (hand < 0 || hand >= Hands) throw new ArgumentOutOfRangeException(nameof(hand));
            bool rising = held && tracked && !_wasHeld[hand];
            _wasHeld[hand] = held && tracked;

            if (_owner == hand)
            {
                if (!tracked)
                {
                    _owner = -1; // the hand was lost mid-drag: nothing is asked
                    return;
                }

                Drag(cursor);
                if (!held)
                {
                    _owner = -1;
                    double moved = Math.Sqrt((_x - _startX) * (_x - _startX) + (_y - _startY) * (_y - _startY));
                    if (moved >= _config.MinMoveMeters || Math.Abs(Wrap(_yaw - _startYaw)) >= _config.MinTurnRadians)
                    {
                        _request = ObjectPose.Of(_scene.Objects[_index].Id, 0, Preview);
                        _hasRequest = true;
                    }
                }

                return;
            }

            if (_owner < 0 && rising) TryGrab(hand, cursor, world);
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
            for (int i = 0; i < Hands; i++) _wasHeld[i] = false;
        }

        private void TryGrab(int hand, in RigidTransform cursor, WorldState world)
        {
            int best = -1;
            double bestDistance = double.MaxValue;
            ObjectPose bestPose = default;
            for (int i = 0; i < _scene.Objects.Length; i++)
            {
                SceneObject o = _scene.Objects[i];
                if (!o.OperatorMovable || !world.TryGet(o.Id, out ObjectPose p, out _)) continue;
                double dx = cursor.X - p.X, dy = cursor.Y - p.Y;
                double planar = Math.Sqrt(dx * dx + dy * dy) - o.FootprintRadius;
                double top = p.Z + o.SizeZ / 2;
                if (planar > _config.GrabReachMeters || cursor.Z > top + _config.GrabAboveMeters) continue;
                double distance = Math.Max(0, planar);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    best = i;
                    bestPose = p;
                }
            }

            if (best < 0) return;
            _owner = hand;
            _index = best;
            _startX = _x = bestPose.X;
            _startY = _y = bestPose.Y;
            _startYaw = _yaw = Math.Atan2(2.0 * (bestPose.Qw * bestPose.Qz + bestPose.Qx * bestPose.Qy),
                1.0 - 2.0 * (bestPose.Qy * bestPose.Qy + bestPose.Qz * bestPose.Qz));
            _cursorX = cursor.X;
            _cursorY = cursor.Y;
            // The heading of the tool's approach axis (its z), unless it points nearly straight down: then its x. Fixed at the
            // grab, so the heading never jumps between the two mid-drag.
            cursor.Rotate(0, 0, 1, out double ax, out double ay, out _);
            _useApproach = ax * ax + ay * ay > 0.09;
            _cursorHeading = Heading(cursor);
        }

        private void Drag(in RigidTransform cursor)
        {
            double x = _startX + (cursor.X - _cursorX), y = _startY + (cursor.Y - _cursorY);
            SceneSurface t = _scene.Table;
            double r = _scene.Objects[_index].FootprintRadius;
            double x0 = Math.Max(t.XMin, _scene.RegionXMin) + r, x1 = Math.Min(t.XMax, _scene.RegionXMax) - r;
            double y0 = Math.Max(t.YMin, _scene.RegionYMin) + r, y1 = Math.Min(t.YMax, _scene.RegionYMax) - r;
            _x = x0 <= x1 ? Math.Min(Math.Max(x, x0), x1) : (x0 + x1) / 2;
            _y = y0 <= y1 ? Math.Min(Math.Max(y, y0), y1) : (y0 + y1) / 2;
            _yaw = Wrap(_startYaw + Wrap(Heading(cursor) - _cursorHeading));
        }

        private double Heading(in RigidTransform cursor)
        {
            double vx, vy;
            if (_useApproach) cursor.Rotate(0, 0, 1, out vx, out vy, out _);
            else cursor.Rotate(1, 0, 0, out vx, out vy, out _);
            return Math.Atan2(vy, vx);
        }

        private static double Wrap(double a)
        {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a <= -Math.PI) a += 2 * Math.PI;
            return a;
        }
    }
}
