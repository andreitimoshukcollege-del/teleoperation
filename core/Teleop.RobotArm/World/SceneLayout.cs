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

        /// <summary>How many spots a pool's layout draws for one object before it leaves that object parked.</summary>
        public const int MaxPoolTries = 64;

        /// <summary>
        /// Writes each object's pose (root frame, in <see cref="WorldScene.Objects"/>' order) into <paramref name="poses"/>,
        /// and returns how many objects found no free spot and kept their nominal pose. For a scene that is not a pool.
        /// </summary>
        public static int Draw(WorldScene scene, ulong seed, Span<RigidTransform> poses)
        {
            if (scene == null) throw new ArgumentNullException(nameof(scene));
            if (scene.IsPool) throw new ArgumentException("a pool's layout also says which objects are parked: use the overload with parked", nameof(scene));
            return DrawJittered(scene, seed, poses);
        }

        /// <summary>
        /// As <see cref="Draw(WorldScene, ulong, Span{RigidTransform})"/>, and for a pool (docs/adr/0037 decision 3) also which
        /// objects take part: <paramref name="parked"/>[i] is true for one this layout leaves out, whose pose is then its
        /// <see cref="WorldScene.ParkedPose"/>. A pool's layout draws, from one <see cref="SeededRng"/>: how many movable
        /// objects (uniform in the pool's range), how many fixed ones, which (a partial Fisher-Yates shuffle of each kind in
        /// file order), then a spot for each, the fixed ones first: x and y uniform where its footprint fits on the table and
        /// in the region (a movable one also within the pool's reach band, if it has one), and yaw uniform in ±π, redrawn while it comes within the clearance of one already placed, up to
        /// <see cref="MaxPoolTries"/> times. One that finds no spot is parked too. Returns how many chosen objects found no
        /// spot. Allocates nothing.
        /// </summary>
        public static int Draw(WorldScene scene, ulong seed, Span<RigidTransform> poses, Span<bool> parked)
        {
            if (scene == null) throw new ArgumentNullException(nameof(scene));
            Span<double> scales = stackalloc double[WorldScene.MaxObjects];
            return Draw(scene, seed, poses, parked, scales);
        }

        /// <summary>
        /// As <see cref="Draw(WorldScene, ulong, Span{RigidTransform}, Span{bool})"/>, with each object's size (docs/adr/0037
        /// amendment 2): for a pool that draws sizes, each object on the table that may be resized first takes a resize step,
        /// uniform among those that keep its largest side in the pool's range, and its spot is drawn at that size. Every other
        /// object's is 1 (a parked one keeps whatever size it has). The poses are at the drawn sizes.
        /// </summary>
        public static int Draw(WorldScene scene, ulong seed, Span<RigidTransform> poses, Span<bool> parked, Span<double> scales)
        {
            if (scene == null) throw new ArgumentNullException(nameof(scene));
            int n = scene.Objects.Length;
            if (poses.Length < n || parked.Length < n || scales.Length < n) throw new ArgumentException($"room for {n} poses, flags and sizes is needed");
            scales.Slice(0, n).Fill(1.0);
            if (!scene.IsPool)
            {
                parked.Slice(0, n).Clear();
                return DrawJittered(scene, seed, poses);
            }

            Span<int> movable = stackalloc int[WorldScene.MaxObjects];
            Span<int> fixedOnes = stackalloc int[WorldScene.MaxObjects];
            int movables = 0, fixeds = 0;
            for (int i = 0; i < n; i++)
            {
                parked[i] = true;
                poses[i] = scene.ParkedPose(i);
                if (scene.Objects[i].Movable) movable[movables++] = i;
                else fixedOnes[fixeds++] = i;
            }

            var rng = new SeededRng(seed);
            int takeMovable = Count(ref rng, scene.MovableMin, scene.MovableMax);
            int takeFixed = Count(ref rng, scene.FixedMin, scene.FixedMax);
            Choose(ref rng, movable.Slice(0, movables), takeMovable);
            Choose(ref rng, fixedOnes.Slice(0, fixeds), takeFixed);

            // docs/adr/0037 amendment 6: sometimes one more movable object, by the table's edge. Drawn only for a pool that asks,
            // so every other pool's layouts take exactly the draws they always took.
            int edge = -1;
            if (scene.EdgeShare > 0 && rng.NextDouble() < scene.EdgeShare && movables > takeMovable)
            {
                Choose(ref rng, movable.Slice(takeMovable, movables - takeMovable), 1);
                edge = movable[takeMovable];
            }

            Span<int> order = stackalloc int[WorldScene.MaxObjects];
            int count = 0;
            for (int k = 0; k < takeFixed; k++) order[count++] = fixedOnes[k];
            for (int k = 0; k < takeMovable; k++) order[count++] = movable[k];
            if (edge >= 0) order[count++] = edge;
            Span<int> placed = stackalloc int[WorldScene.MaxObjects];
            Span<double> xs = stackalloc double[WorldScene.MaxObjects];
            Span<double> ys = stackalloc double[WorldScene.MaxObjects];
            Span<int> rests = stackalloc int[WorldScene.MaxObjects];
            Span<int> anchors = stackalloc int[WorldScene.MaxObjects];
            int placedCount = 0, anchorCount = 0, failed = 0;
            for (int k = 0; k < count; k++)
            {
                int i = order[k];
                bool movableOne = scene.Objects[i].Movable, atEdge = i == edge;
                double scale = 1.0;
                if (scene.Resizable(i))
                {
                    scene.SizeSteps(i, out int lo, out int hi);
                    int step = Math.Min(hi, lo + (int)(rng.NextDouble() * (hi - lo + 1)));
                    scale = WorldScene.ScaleOfStep(step);
                }

                int rest = movableOne && scene.LyingShare > 0 ? DrawRest(scene, i, ref rng) : WorldScene.RestUpright;
                bool beside = movableOne && !atEdge && anchorCount > 0 && scene.ClusterShare > 0 && rng.NextDouble() < scene.ClusterShare;
                bool done = false;
                for (int t = 0; t < MaxPoolTries && !done; t++)
                {
                    double x, y, yaw;
                    if (atEdge)
                    {
                        if (!scene.DrawEdgeSpot(i, scale, rest, ref rng, out x, out y, out yaw)) continue;
                    }
                    else if (beside && t < MaxPoolTries / 2)
                    {
                        // Half the tries beside one already placed; then anywhere, as for one that joins no cluster.
                        int a = placed[anchors[Math.Min(anchorCount - 1, (int)(rng.NextDouble() * anchorCount))]];
                        if (!DrawBeside(scene, i, scale, rest, a, poses[a].X, poses[a].Y, scales[a], WorldScene.RestOf(poses[a]), ref rng, out x, out y, out yaw)) continue;
                    }
                    else if (!scene.DrawSpot(i, scale, rest, ref rng, out x, out y, out yaw)) continue;

                    bool clear = true;
                    for (int q = 0; q < placedCount && clear; q++)
                    {
                        if (scene.Overlaps(i, x, y, placed[q], xs[q], ys[q], scale, scales[placed[q]], rest, rests[q])) clear = false;
                    }

                    if (!clear) continue;
                    if (movableOne && !atEdge) anchors[anchorCount++] = placedCount;
                    placed[placedCount] = i;
                    xs[placedCount] = x;
                    ys[placedCount] = y;
                    rests[placedCount] = rest;
                    placedCount++;
                    parked[i] = false;
                    scales[i] = scale;
                    poses[i] = scene.PoseAt(i, x, y, yaw, scale, rest);
                    done = true;
                }

                if (!done) failed++;
            }

            return failed;
        }

        /// <summary>
        /// The rest a layout gives movable object <paramref name="index"/> (docs/adr/0037 amendment 6): one of its stable sides
        /// (<see cref="WorldScene.CanLie"/>), uniform among them, with the pool's <see cref="WorldScene.LyingShare"/>; otherwise
        /// upright. Draws nothing for an object with no stable side.
        /// </summary>
        private static int DrawRest(WorldScene scene, int index, ref SeededRng rng)
        {
            Span<int> sides = stackalloc int[4];
            int n = 0;
            for (int rest = 1; rest < WorldScene.RestCount; rest++)
            {
                if (scene.CanLie(index, rest)) sides[n++] = rest;
            }

            if (n == 0 || rng.NextDouble() >= scene.LyingShare) return WorldScene.RestUpright;
            return sides[Math.Min(n - 1, (int)(rng.NextDouble() * n))];
        }

        /// <summary>
        /// A spot for object <paramref name="index"/> next to object <paramref name="anchor"/> at (ax, ay) (a cluster, docs/adr/0037
        /// amendment 6): their footprint circles apart by the clearance and up to <see cref="WorldScene.ClusterGap"/> more, at a
        /// uniform bearing, yaw uniform; three draws. False when the spot leaves the table, the region or the reach band.
        /// </summary>
        private static bool DrawBeside(WorldScene scene, int index, double scale, int rest, int anchor, double ax, double ay, double anchorScale, int anchorRest,
            ref SeededRng rng, out double x, out double y, out double yaw)
        {
            double bearing = rng.NextDouble() * 2 * Math.PI;
            double d = scene.FootprintRadiusAt(index, rest) * scale + scene.FootprintRadiusAt(anchor, anchorRest) * anchorScale + scene.Clearance +
                       rng.NextDouble() * WorldScene.ClusterGap + 1e-6;
            x = ax + d * Math.Cos(bearing);
            y = ay + d * Math.Sin(bearing);
            yaw = (2 * rng.NextDouble() - 1) * Math.PI;
            return scene.FitsOnTable(index, x, y, scale, rest) && scene.InReach(index, x, y);
        }

        private static int Count(ref SeededRng rng, int min, int max)
        {
            int k = min + (int)(rng.NextDouble() * (max - min + 1));
            return Math.Min(k, max);
        }

        /// <summary>Moves <paramref name="take"/> entries of <paramref name="items"/>, drawn without replacement, to its front.</summary>
        private static void Choose(ref SeededRng rng, Span<int> items, int take)
        {
            for (int k = 0; k < take && k < items.Length; k++)
            {
                int j = k + (int)(rng.NextDouble() * (items.Length - k));
                if (j >= items.Length) j = items.Length - 1;
                (items[k], items[j]) = (items[j], items[k]);
            }
        }

        private static int DrawJittered(WorldScene scene, ulong seed, Span<RigidTransform> poses)
        {
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
