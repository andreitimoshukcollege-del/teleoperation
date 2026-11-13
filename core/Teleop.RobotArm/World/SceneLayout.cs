using System;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.World
{
    /// <summary>
    /// A seeded layout of a scene's objects (docs/adr/0031 amendment 2): RobotHost draws one at start and on NEW LAYOUT, and
    /// sends each pose to Isaac as a placement. Each object, in file order, takes three draws per try from one
    /// <see cref="SeededRng"/> (x, y and yaw, each uniform within ± its jitter); a try that leaves the table or the placement
    /// region, or comes within the clearance of an object already placed, is drawn again, up to <see cref="MaxTries"/>
    /// times, and after that the object keeps its nominal pose. The same scene and seed give the same layout on every
    /// runtime. Allocates nothing.
    /// </summary>
    public static class SceneLayout
    {
        public const int MaxTries = 16;

        /// <summary>
        /// Writes each object's pose (root frame, in <see cref="WorldScene.Objects"/>' order) into <paramref name="poses"/>,
        /// and returns how many objects found no free spot and kept their nominal pose.
        /// </summary>
        public static int Draw(WorldScene scene, ulong seed, Span<RigidTransform> poses)
        {
            if (scene == null) throw new ArgumentNullException(nameof(scene));
            int n = scene.Objects.Length;
            if (poses.Length < n) throw new ArgumentException($"room for {n} poses is needed", nameof(poses));
            Span<double> xs = stackalloc double[WorldScene.MaxObjects];
            Span<double> ys = stackalloc double[WorldScene.MaxObjects];
            var rng = new SeededRng(seed);
            int nominal = 0;
            for (int i = 0; i < n; i++)
            {
                SceneObject o = scene.Objects[i];
                bool placed = false;
                for (int t = 0; t < MaxTries && !placed; t++)
                {
                    double x = o.NominalX + (2 * rng.NextDouble() - 1) * o.JitterX;
                    double y = o.NominalY + (2 * rng.NextDouble() - 1) * o.JitterY;
                    double yaw = o.NominalYaw + (2 * rng.NextDouble() - 1) * o.JitterYaw;
                    if (!scene.FitsOnTable(i, x, y) || OverlapsPlaced(scene, i, x, y, xs, ys)) continue;
                    xs[i] = x;
                    ys[i] = y;
                    poses[i] = scene.PoseAt(i, x, y, yaw);
                    placed = true;
                }

                if (!placed)
                {
                    xs[i] = o.NominalX;
                    ys[i] = o.NominalY;
                    poses[i] = o.Nominal;
                    nominal++;
                }
            }

            return nominal;
        }

        private static bool OverlapsPlaced(WorldScene scene, int i, double x, double y, ReadOnlySpan<double> xs, ReadOnlySpan<double> ys)
        {
            for (int k = 0; k < i; k++)
            {
                if (scene.Overlaps(i, x, y, k, xs[k], ys[k])) return true;
            }

            return false;
        }
    }
}
