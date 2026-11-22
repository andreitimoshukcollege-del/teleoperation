using System;
using Teleop.RobotArm.World;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// Whether the objects have come to rest, by the newest poses RobotHost sent (docs/adr/0036 amendment 5): every object on the
    /// table within <see cref="MovedMeters"/> and <see cref="TurnedRadians"/> of where it was <see cref="Seconds"/> ago. After a
    /// knock, joint-drive plans the next event against the objects only once they are (<see cref="SafetyContinuation"/>), as it
    /// plans a take only once a new layout has settled. An object that moves further starts the wait again; a parked one does
    /// not count. No clock of its own: the caller says when.
    /// </summary>
    public sealed class RestWatch
    {
        public const float MovedMeters = 0.001f;
        public const float TurnedRadians = MathF.PI / 180f;

        private readonly ObjectPose[] _anchor = new ObjectPose[256];
        private readonly bool[] _anchored = new bool[256];
        private double _since = double.NaN;

        public RestWatch(double seconds)
        {
            if (!(seconds > 0)) throw new ArgumentOutOfRangeException(nameof(seconds), seconds, "a positive time");
            Seconds = seconds;
        }

        /// <summary>How long every object must stay put, seconds.</summary>
        public double Seconds { get; }

        /// <summary>Whether every object of <paramref name="world"/>'s scene has stayed put for <see cref="Seconds"/> by <paramref name="now"/> (seconds, any clock that rises).</summary>
        public bool Still(WorldState world, double now)
        {
            bool moved = double.IsNaN(_since);
            foreach (SceneObject o in world.Scene.Objects)
            {
                if (!world.TryGet(o.Id, out ObjectPose p, out _))
                {
                    moved = true; // not yet posed: nothing is known to be at rest
                    continue;
                }

                if (p.Parked) continue;
                if (!_anchored[o.Id] || Moved(_anchor[o.Id], p)) moved = true;
            }

            if (moved)
            {
                foreach (SceneObject o in world.Scene.Objects)
                {
                    if (!world.TryGet(o.Id, out ObjectPose p, out _)) continue;
                    _anchor[o.Id] = p;
                    _anchored[o.Id] = true;
                }

                _since = now;
                return false;
            }

            return now - _since >= Seconds;
        }

        private static bool Moved(in ObjectPose a, in ObjectPose b)
        {
            float dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
            if (dx * dx + dy * dy + dz * dz > MovedMeters * MovedMeters) return true;
            float dot = MathF.Min(1f, MathF.Abs(a.Qw * b.Qw + a.Qx * b.Qx + a.Qy * b.Qy + a.Qz * b.Qz));
            return 2f * MathF.Acos(dot) > TurnedRadians;
        }
    }
}
