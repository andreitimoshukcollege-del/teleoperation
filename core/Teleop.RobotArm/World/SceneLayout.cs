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
        /// <para>docs/adr/0037 amendment 6, each only for a pool that sets its share (any other takes exactly the draws it always
        /// took): after which objects, whether one more movable object stands by the bench's edge
        /// (<see cref="WorldScene.EdgeShare"/>), chosen among those left off the table that have room there, and placed last, out
        /// of reach; for each movable object after its size, its rest (<see cref="WorldScene.LyingShare"/>: one of the sides it
        /// lies on stably, uniform), then for one in reach after the first, whether it joins a cluster
        /// (<see cref="WorldScene.ClusterShare"/>: half its tries beside one already standing, footprint circles the clearance and
        /// up to <see cref="WorldScene.ClusterGap"/> apart).</para>
        /// <para>docs/adr/0037 "Raised clutter": a pool's clutter objects (<see cref="SceneObject.Clutter"/>) are none of its movable
        /// objects. After every other object, <see cref="SceneClutter"/> stands <see cref="WorldScene.ClutterMin"/> to
        /// <see cref="WorldScene.ClutterMax"/> of them anywhere in the region, from a generator of its own: a pool without clutter
        /// draws exactly what it drew before, and with it, every other object stands where it would without it.</para>
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
            Span<int> clutter = stackalloc int[WorldScene.MaxObjects];
            int movables = 0, fixeds = 0, clutters = 0;
            for (int i = 0; i < n; i++)
            {
                parked[i] = true;
                poses[i] = scene.ParkedPose(i);
                if (scene.Objects[i].Clutter) clutter[clutters++] = i; // its own pass, last (docs/adr/0037 "Raised clutter")
                else if (scene.Objects[i].Movable) movable[movables++] = i;
                else fixedOnes[fixeds++] = i;
            }

            var rng = new SeededRng(seed);
            int takeMovable = Count(ref rng, scene.MovableMin, scene.MovableMax);
            int takeFixed = Count(ref rng, scene.FixedMin, scene.FixedMax);
            Choose(ref rng, movable.Slice(0, movables), takeMovable);
            Choose(ref rng, fixedOnes.Slice(0, fixeds), takeFixed);

            // docs/adr/0037 amendment 6: sometimes one more movable object, by the bench's edge, from those left off the table, so
            // it takes no object from the reach band.
            int edge = scene.EdgeShare > 0 && rng.NextDouble() < scene.EdgeShare ? ChooseEdge(scene, ref rng, movable.Slice(takeMovable, movables - takeMovable)) : -1;

            Span<int> placed = stackalloc int[WorldScene.MaxObjects];
            Span<double> xs = stackalloc double[WorldScene.MaxObjects];
            Span<double> ys = stackalloc double[WorldScene.MaxObjects];
            Span<int> rests = stackalloc int[WorldScene.MaxObjects];
            Span<int> anchors = stackalloc int[WorldScene.MaxObjects];
            Span<double> yaws = stackalloc double[WorldScene.MaxObjects];
            var standing = new Standing(scene, poses, parked, scales, placed, xs, ys, yaws, rests, anchors);

            // In passes, each object of a pass in turn: the fixed ones, the movable ones in reach (after the first, each may join a
            // cluster), then the one by the edge, then the clutter (docs/adr/0037 "Raised clutter"; its own generator, so none of the
            // others moves for it).
            int failed = 0;
            for (int k = 0; k < takeFixed; k++) failed += standing.Stand(fixedOnes[k], false, ref rng) ? 0 : 1;
            for (int k = 0; k < takeMovable; k++) failed += standing.Stand(movable[k], false, ref rng) ? 0 : 1;
            if (edge >= 0) failed += standing.Stand(edge, true, ref rng) ? 0 : 1;
            failed += SceneClutter.Stand(scene, seed, clutter.Slice(0, clutters), ref standing);
            return failed;
        }

        /// <summary>
        /// The objects a pool's layout has stood on the table so far, in order: what each next one keeps clear of, and the movable
        /// ones in reach that a cluster grows from. <see cref="Stand"/> and <see cref="StandClutter"/> write each one's pose, size
        /// and parked flag.
        /// </summary>
        internal ref struct Standing
        {
            private readonly WorldScene _scene;
            private readonly Span<RigidTransform> _poses;
            private readonly Span<bool> _parked;
            private readonly Span<double> _scales;
            private readonly Span<int> _index;
            private readonly Span<double> _x;
            private readonly Span<double> _y;
            private readonly Span<double> _yaw; // a piece of clutter is its own rectangle at its yaw (docs/adr/0037 "Raised clutter")
            private readonly Span<int> _rest;
            private readonly Span<int> _anchors; // positions in _index of the movable objects in reach
            private int _count;
            private int _anchorCount;

            public Standing(WorldScene scene, Span<RigidTransform> poses, Span<bool> parked, Span<double> scales, Span<int> index, Span<double> x,
                Span<double> y, Span<double> yaw, Span<int> rest, Span<int> anchors)
            {
                _yaw = yaw;
                _scene = scene;
                _poses = poses;
                _parked = parked;
                _scales = scales;
                _index = index;
                _x = x;
                _y = y;
                _rest = rest;
                _anchors = anchors;
                _count = 0;
                _anchorCount = 0;
            }

            /// <summary>
            /// Draws object <paramref name="i"/>'s size, rest and spot (by the edge when <paramref name="atEdge"/>), clear of every
            /// object standing, and stands it there; false, leaving it parked, when no try of <see cref="MaxPoolTries"/> finds one.
            /// </summary>
            public bool Stand(int i, bool atEdge, ref SeededRng rng)
            {
                double scale = 1.0;
                if (_scene.Resizable(i))
                {
                    _scene.SizeSteps(i, out int lo, out int hi);
                    int step = Math.Min(hi, lo + (int)(rng.NextDouble() * (hi - lo + 1)));
                    scale = WorldScene.ScaleOfStep(step);
                }

                bool movable = _scene.Objects[i].Movable;
                int rest = movable && _scene.LyingShare > 0 ? DrawRest(_scene, i, scale, atEdge, ref rng) : WorldScene.RestUpright;
                bool beside = movable && !atEdge && _anchorCount > 0 && _scene.ClusterShare > 0 && rng.NextDouble() < _scene.ClusterShare;
                for (int t = 0; t < MaxPoolTries; t++)
                {
                    double x, y, yaw;
                    if (atEdge)
                    {
                        if (!_scene.DrawEdgeSpot(i, scale, rest, ref rng, out x, out y, out yaw)) continue;
                    }
                    else if (beside && t < MaxPoolTries / 2)
                    {
                        // Half its tries beside one already standing (a cluster); then anywhere, as one that joins none.
                        int a = _anchors[Math.Min(_anchorCount - 1, (int)(rng.NextDouble() * _anchorCount))];
                        if (!DrawBeside(_scene, i, scale, rest, _index[a], _x[a], _y[a], _scales[_index[a]], _rest[a], ref rng, out x, out y, out yaw)) continue;
                    }
                    else if (!_scene.DrawSpot(i, scale, rest, ref rng, out x, out y, out yaw)) continue;

                    if (!Clear(i, x, y, yaw, scale, rest)) continue;
                    if (movable && !atEdge) _anchors[_anchorCount++] = _count;
                    Add(i, x, y, yaw, scale, rest);
                    return true;
                }

                return false;
            }

            /// <summary>
            /// Draws clutter object <paramref name="i"/>'s spot (docs/adr/0037 "Raised clutter"): at its file size and flat as the
            /// file has it (its upright rest), its yaw uniform, anywhere in the placement region where its own rectangle fits at that
            /// yaw (<see cref="WorldScene.DrawClutterSpot"/>: no reach band), clear of every object standing, three draws a try;
            /// false, leaving it parked, when no try of <see cref="MaxPoolTries"/> finds one. It joins no cluster, and none grows from it.
            /// </summary>
            public bool StandClutter(int i, ref SeededRng rng)
            {
                for (int t = 0; t < MaxPoolTries; t++)
                {
                    if (!_scene.DrawClutterSpot(i, ref rng, out double x, out double y, out double yaw)) continue;
                    if (!Clear(i, x, y, yaw, 1.0, WorldScene.RestUpright)) continue;
                    Add(i, x, y, yaw, 1.0, WorldScene.RestUpright);
                    return true;
                }

                return false;
            }

            /// <summary>Stands object <paramref name="i"/> at (x, y), turned <paramref name="yaw"/>, at its size and rest: what the next ones keep clear of.</summary>
            private void Add(int i, double x, double y, double yaw, double scale, int rest)
            {
                _index[_count] = i;
                _x[_count] = x;
                _y[_count] = y;
                _yaw[_count] = yaw;
                _rest[_count] = rest;
                _count++;
                _parked[i] = false;
                _scales[i] = scale;
                _poses[i] = _scene.PoseAt(i, x, y, yaw, scale, rest);
            }

            /// <summary>
            /// Whether object <paramref name="i"/> at (x, y), its yaw, size and rest, keeps the clearance from every object standing:
            /// circles, but a piece of clutter its own rectangle (<see cref="WorldScene.Overlaps(int, double, double, double, int, double, double, double, double, double, int, int)"/>).
            /// </summary>
            private bool Clear(int i, double x, double y, double yaw, double scale, int rest)
            {
                for (int q = 0; q < _count; q++)
                {
                    if (_scene.Overlaps(i, x, y, yaw, _index[q], _x[q], _y[q], _yaw[q], scale, _scales[_index[q]], rest, _rest[q])) return false;
                }

                return true;
            }
        }

        /// <summary>
        /// The movable object a layout stands by the edge (docs/adr/0037 amendment 6): one of <paramref name="left"/> (those it
        /// leaves off the table) with room there upright at its file size (<see cref="WorldScene.HasEdgeRoom"/>), uniform among them;
        /// -1 when none has. Moves those with room to the front of <paramref name="left"/>.
        /// </summary>
        private static int ChooseEdge(WorldScene scene, ref SeededRng rng, Span<int> left)
        {
            int n = 0;
            for (int k = 0; k < left.Length; k++)
            {
                if (!scene.HasEdgeRoom(left[k], 1.0, WorldScene.RestUpright)) continue;
                (left[n], left[k]) = (left[k], left[n]);
                n++;
            }

            if (n == 0) return -1;
            Choose(ref rng, left.Slice(0, n), 1);
            return left[0];
        }

        /// <summary>
        /// The rest a layout gives movable object <paramref name="index"/> (docs/adr/0037 amendment 6): with the pool's
        /// <see cref="WorldScene.LyingShare"/>, one of the sides it lies on stably (<see cref="WorldScene.CanLie"/>; by the edge, only
        /// one that leaves it room there), uniform among them; otherwise upright. Draws nothing for an object with no such side.
        /// </summary>
        private static int DrawRest(WorldScene scene, int index, double scale, bool atEdge, ref SeededRng rng)
        {
            Span<int> sides = stackalloc int[4];
            int n = 0;
            for (int rest = 1; rest < WorldScene.RestCount; rest++)
            {
                if (scene.CanLie(index, rest) && (!atEdge || scene.HasEdgeRoom(index, scale, rest))) sides[n++] = rest;
            }

            if (n == 0 || rng.NextDouble() >= scene.LyingShare) return WorldScene.RestUpright;
            return sides[Math.Min(n - 1, (int)(rng.NextDouble() * n))];
        }

        /// <summary>
        /// A spot for object <paramref name="index"/> next to object <paramref name="anchor"/> at (ax, ay) (a cluster, docs/adr/0037
        /// amendment 6): their footprint circles apart by the clearance and up to <see cref="WorldScene.ClusterGap"/> more, at a
        /// uniform bearing, yaw uniform; three draws. False when the spot leaves the table, the region or the reach band (by the edge
        /// is not in reach).
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
            return !scene.InEdgeRegions(x, y) && scene.FitsOnTable(index, x, y, scale, rest) && scene.InReach(index, x, y);
        }

        /// <summary>A count uniform in [<paramref name="min"/>, <paramref name="max"/>], one draw.</summary>
        internal static int Count(ref SeededRng rng, int min, int max)
        {
            int k = min + (int)(rng.NextDouble() * (max - min + 1));
            return Math.Min(k, max);
        }

        /// <summary>Moves <paramref name="take"/> entries of <paramref name="items"/>, drawn without replacement, to its front.</summary>
        internal static void Choose(ref SeededRng rng, Span<int> items, int take)
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
