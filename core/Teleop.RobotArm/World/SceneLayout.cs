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

            if (scene.Mixed) return DrawMixed(scene, seed, poses, parked, scales);

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
            Span<double> zs = stackalloc double[WorldScene.MaxObjects];
            var standing = new Standing(scene, poses, parked, scales, placed, xs, ys, yaws, rests, anchors, zs);

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
            private readonly Span<double> _z; // each one's centre height: a mixed pool's rules take it (docs/adr/0038)
            private int _count;
            private int _anchorCount;

            public Standing(WorldScene scene, Span<RigidTransform> poses, Span<bool> parked, Span<double> scales, Span<int> index, Span<double> x,
                Span<double> y, Span<double> yaw, Span<int> rest, Span<int> anchors, Span<double> z)
            {
                _z = z;
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
            private void Add(int i, double x, double y, double yaw, double scale, int rest) =>
                AddAt(i, x, y, _scene.RestingZ(i, scale, rest), yaw, scale, rest);

            /// <summary>As <see cref="Add"/>, its centre at height <paramref name="z"/> (on a stand, or a plank on its supports: docs/adr/0038).</summary>
            private void AddAt(int i, double x, double y, double z, double yaw, double scale, int rest)
            {
                _index[_count] = i;
                _x[_count] = x;
                _y[_count] = y;
                _z[_count] = z;
                _yaw[_count] = yaw;
                _rest[_count] = rest;
                _count++;
                _parked[i] = false;
                _scales[i] = scale;
                _poses[i] = _scene.PoseAtHeight(i, x, y, z, yaw, scale, rest);
            }

            /// <summary>
            /// Whether object <paramref name="i"/> with its centre at (x, y, z), its yaw, size and rest, keeps a mixed pool's clearance from
            /// every object standing (docs/adr/0038: rectangles, and nothing one above the other overlaps).
            /// </summary>
            private bool ClearAt(int i, double x, double y, double z, double yaw, double scale, int rest)
            {
                for (int q = 0; q < _count; q++)
                {
                    if (_scene.OverlapsAtHeights(i, x, y, z, yaw, _index[q], _x[q], _y[q], _z[q], _yaw[q], scale, _scales[_index[q]], rest, _rest[q])) return false;
                }

                return true;
            }

            /// <summary>
            /// A mixed pool's obstacle <paramref name="i"/> (docs/adr/0038) on the table at its size and rest: centred uniformly by area over
            /// the arm's reachable band (<see cref="WorldScene.DrawObstacleSpot"/>), where it fits and keeps clear; false, leaving it
            /// parked, when no try of <see cref="MaxPoolTries"/> finds one.
            /// </summary>
            public bool StandObstacle(int i, double scale, int rest, ref SeededRng rng)
            {
                for (int t = 0; t < MaxPoolTries; t++)
                {
                    _scene.DrawObstacleSpot(ref rng, out double x, out double y, out double yaw);
                    double z = _scene.RestingZ(i, scale, rest);
                    if (!_scene.FitsOnTable(i, x, y, yaw, scale, rest) || !ClearAt(i, x, y, z, yaw, scale, rest)) continue;
                    AddAt(i, x, y, z, yaw, scale, rest);
                    return true;
                }

                return false;
            }

            /// <summary>
            /// A mixed pool's distractor <paramref name="i"/> (docs/adr/0038) at its size and rest: centred in its regions
            /// (<see cref="WorldScene.DrawDistractorSpot"/>), where it fits beyond every arm pose's reach and keeps clear; false, leaving it
            /// parked, when no try of <see cref="MaxPoolTries"/> finds one.
            /// </summary>
            public bool StandDistractor(int i, double scale, int rest, ref SeededRng rng)
            {
                for (int t = 0; t < MaxPoolTries; t++)
                {
                    _scene.DrawDistractorSpot(ref rng, out double x, out double y, out double yaw);
                    double z = _scene.RestingZ(i, scale, rest);
                    if (!_scene.FitsOnTable(i, x, y, yaw, scale, rest) || !ClearAt(i, x, y, z, yaw, scale, rest)) continue;
                    AddAt(i, x, y, z, yaw, scale, rest);
                    return true;
                }

                return false;
            }

            /// <summary>
            /// Movable obstacle <paramref name="i"/> raised on stand <paramref name="k"/> (docs/adr/0038): the stand at
            /// <paramref name="standScale"/> centred as an obstacle is, the object on its top at its size and rest, turned with it (a
            /// quarter more when <paramref name="turned"/>) and off its middle by up to the room its top leaves; both kept clear of
            /// everything standing. False, leaving both parked, when no try of <see cref="MaxPoolTries"/> finds a spot.
            /// </summary>
            public bool StandRaised(int i, double scale, int rest, int k, double standScale, bool turned, ref SeededRng rng)
            {
                _scene.RestExtents(i, rest, out double ex, out double ey, out double ez);
                _scene.RestExtents(k, WorldScene.RestUpright, out double kx, out double ky, out double kz);
                double ox = turned ? ey : ex, oy = turned ? ex : ey; // the object's sides along the stand's x and y
                double offX = (2 * rng.NextDouble() - 1) * Math.Max(0, kx * standScale - ox * scale) / 2;
                double offY = (2 * rng.NextDouble() - 1) * Math.Max(0, ky * standScale - oy * scale) / 2;
                double standZ = _scene.RestingZ(k, standScale, WorldScene.RestUpright), z = _scene.Table.Top + kz * standScale + ez * scale / 2;
                for (int t = 0; t < MaxPoolTries; t++)
                {
                    _scene.DrawObstacleSpot(ref rng, out double x, out double y, out double yaw);
                    if (!_scene.FitsOnTable(k, x, y, yaw, standScale, WorldScene.RestUpright) || !ClearAt(k, x, y, standZ, yaw, standScale, WorldScene.RestUpright)) continue;
                    double c = Math.Cos(yaw), s = Math.Sin(yaw);
                    double px = x + offX * c - offY * s, py = y + offX * s + offY * c, pyaw = yaw + (turned ? Math.PI / 2 : 0);
                    if (pyaw > Math.PI) pyaw -= 2 * Math.PI;
                    if (!_scene.FitsOnTable(i, px, py, pyaw, scale, rest) || !ClearAt(i, px, py, z, pyaw, scale, rest)) continue;
                    AddAt(k, x, y, standZ, yaw, standScale, WorldScene.RestUpright);
                    AddAt(i, px, py, z, pyaw, scale, rest);
                    return true;
                }

                return false;
            }

            /// <summary>
            /// Plank <paramref name="p"/> on its supports (docs/adr/0038), an overhang: the plank at <paramref name="plankScale"/>, centred as
            /// an obstacle is and lying flat as the file has it, its supports at <paramref name="supportScale"/> under its ends (a bridge's
            /// two, a cantilever's one under its back end), its underside on their tops. All of them fit and keep clear of everything
            /// standing. False, leaving them parked, when no try of <see cref="MaxPoolTries"/> finds a spot.
            /// </summary>
            public bool StandRig(int p, double plankScale, double supportScale, ref SeededRng rng)
            {
                byte[] ids = _scene.Objects[p].SupportIds;
                int a = _scene.IndexOf(ids[0]), b = ids.Length > 1 ? _scene.IndexOf(ids[1]) : -1;
                SceneObject plank = _scene.Objects[p], leg = _scene.Objects[a];
                double length = plank.SizeX * plankScale, legX = leg.SizeX * supportScale, height = leg.SizeZ * supportScale;
                double off = length / 2 - legX / 2;
                double legZ = _scene.RestingZ(a, supportScale, WorldScene.RestUpright), z = _scene.Table.Top + height + plank.SizeZ * plankScale / 2;
                for (int t = 0; t < MaxPoolTries; t++)
                {
                    _scene.DrawObstacleSpot(ref rng, out double x, out double y, out double yaw);
                    double c = Math.Cos(yaw), s = Math.Sin(yaw);
                    double ax = x - off * c, ay = y - off * s, bx = x + off * c, by = y + off * s;
                    if (!_scene.FitsOnTable(p, x, y, yaw, plankScale, WorldScene.RestUpright) || !ClearAt(p, x, y, z, yaw, plankScale, WorldScene.RestUpright)) continue;
                    if (!_scene.FitsOnTable(a, ax, ay, yaw, supportScale, WorldScene.RestUpright) || !ClearAt(a, ax, ay, legZ, yaw, supportScale, WorldScene.RestUpright)) continue;
                    if (b >= 0 && (!_scene.FitsOnTable(b, bx, by, yaw, supportScale, WorldScene.RestUpright) || !ClearAt(b, bx, by, legZ, yaw, supportScale, WorldScene.RestUpright) ||
                                   _scene.OverlapsAtHeights(a, ax, ay, legZ, yaw, b, bx, by, legZ, yaw, supportScale, supportScale, WorldScene.RestUpright, WorldScene.RestUpright))) continue;
                    AddAt(a, ax, ay, legZ, yaw, supportScale, WorldScene.RestUpright);
                    if (b >= 0) AddAt(b, bx, by, legZ, yaw, supportScale, WorldScene.RestUpright);
                    AddAt(p, x, y, z, yaw, plankScale, WorldScene.RestUpright);
                    return true;
                }

                return false;
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

        /// <summary>The narrowest clear span a plank leaves under it, m (docs/adr/0038): between a bridge's legs, or past a cantilever's base, room for the gripper.</summary>
        public const double MinRigSpan = 0.10;

        /// <summary>
        /// A mixed pool's layout (docs/adr/0038 decision 3), from one <see cref="SeededRng"/>: how many obstacles (uniform in the pool's
        /// range, at most as many as there are obstacles and planks), then for each, whether it is a plank on its supports
        /// (<see cref="WorldScene.RigShare"/>, when one is free; always when no obstacle object is) or an obstacle object, which (uniform
        /// among those free), its size (<see cref="SceneObject.HasSteps"/>), its rest (<see cref="WorldScene.LyingShare"/>), and for a
        /// movable one whether it is raised on a free stand that holds it (<see cref="WorldScene.StandShare"/>), and its spot: centred
        /// uniformly by area over the arm's reachable band, yaw uniform, where it fits and keeps clear (rectangles, and nothing one above
        /// the other overlaps). Then how many distractors (uniform in the range), which (a partial shuffle), and each one's size, rest
        /// and spot in the distractor regions. One that finds no spot is parked with its stand or supports. Returns how many obstacles
        /// and distractors found no spot.
        /// </summary>
        private static int DrawMixed(WorldScene scene, ulong seed, Span<RigidTransform> poses, Span<bool> parked, Span<double> scales)
        {
            int n = scene.Objects.Length;
            Span<int> obstacles = stackalloc int[WorldScene.MaxObjects];
            Span<int> distractors = stackalloc int[WorldScene.MaxObjects];
            Span<int> stands = stackalloc int[WorldScene.MaxObjects];
            Span<int> planks = stackalloc int[WorldScene.MaxObjects];
            int freeObstacles = 0, distractorCount = 0, freeStands = 0, freePlanks = 0;
            for (int i = 0; i < n; i++)
            {
                parked[i] = true;
                poses[i] = scene.ParkedPose(i);
                switch (scene.Objects[i].Role)
                {
                    case SceneRole.Obstacle: obstacles[freeObstacles++] = i; break;
                    case SceneRole.Distractor: distractors[distractorCount++] = i; break;
                    case SceneRole.Stand: stands[freeStands++] = i; break;
                    case SceneRole.Plank: planks[freePlanks++] = i; break;
                }
            }

            Span<int> placed = stackalloc int[WorldScene.MaxObjects];
            Span<double> xs = stackalloc double[WorldScene.MaxObjects];
            Span<double> ys = stackalloc double[WorldScene.MaxObjects];
            Span<double> zs = stackalloc double[WorldScene.MaxObjects];
            Span<int> rests = stackalloc int[WorldScene.MaxObjects];
            Span<int> anchors = stackalloc int[WorldScene.MaxObjects];
            Span<double> yaws = stackalloc double[WorldScene.MaxObjects];
            var standing = new Standing(scene, poses, parked, scales, placed, xs, ys, yaws, rests, anchors, zs);
            var rng = new SeededRng(seed);
            int units = Count(ref rng, scene.ObstacleMin, Math.Min(scene.ObstacleMax, freeObstacles + freePlanks));
            int failed = 0;
            for (int u = 0; u < units && freeObstacles + freePlanks > 0; u++)
            {
                if (freePlanks > 0 && (freeObstacles == 0 || (scene.RigShare > 0 && rng.NextDouble() < scene.RigShare)))
                {
                    int p = Take(ref rng, planks, ref freePlanks);
                    failed += DrawRigSizes(scene, p, ref rng, out double plankScale, out double supportScale) && standing.StandRig(p, plankScale, supportScale, ref rng) ? 0 : 1;
                    continue;
                }

                int i = Take(ref rng, obstacles, ref freeObstacles);
                double scale = DrawScale(scene, i, ref rng);
                int rest = scene.Objects[i].Movable && scene.LyingShare > 0 ? DrawRest(scene, i, scale, false, ref rng) : WorldScene.RestUpright;
                int stand = -1;
                double standScale = 1.0;
                bool turned = false;
                if (scene.Objects[i].Movable && freeStands > 0 && scene.StandShare > 0 && rng.NextDouble() < scene.StandShare)
                {
                    stand = ChooseStand(scene, i, scale, rest, stands, ref freeStands, ref rng, out standScale, out turned);
                }

                bool stood = stand >= 0 ? standing.StandRaised(i, scale, rest, stand, standScale, turned, ref rng) : standing.StandObstacle(i, scale, rest, ref rng);
                failed += stood ? 0 : 1;
            }

            int take = Count(ref rng, scene.DistractorMin, Math.Min(scene.DistractorMax, distractorCount));
            Choose(ref rng, distractors.Slice(0, distractorCount), take);
            for (int k = 0; k < take; k++)
            {
                int i = distractors[k];
                double scale = DrawScale(scene, i, ref rng);
                int rest = scene.Objects[i].Movable && scene.LyingShare > 0 ? DrawRest(scene, i, scale, false, ref rng) : WorldScene.RestUpright;
                failed += standing.StandDistractor(i, scale, rest, ref rng) ? 0 : 1;
            }

            return failed;
        }

        /// <summary>One of the first <paramref name="count"/> of <paramref name="free"/>, uniform; it leaves the free ones (the last takes its place). One draw.</summary>
        private static int Take(ref SeededRng rng, Span<int> free, ref int count)
        {
            int j = Math.Min(count - 1, (int)(rng.NextDouble() * count));
            int item = free[j];
            free[j] = free[count - 1];
            count--;
            return item;
        }

        /// <summary>Object <paramref name="i"/>'s size a layout draws: one of its own steps, uniform (one draw), or 1 without them.</summary>
        private static double DrawScale(WorldScene scene, int i, ref SeededRng rng)
        {
            if (!scene.Resizable(i)) return 1.0;
            scene.SizeSteps(i, out int lo, out int hi);
            return WorldScene.ScaleOfStep(Math.Min(hi, lo + (int)(rng.NextDouble() * (hi - lo + 1))));
        }

        /// <summary>
        /// A plank's size and its supports' (docs/adr/0038), one draw: uniform among the pairs of their steps that leave a clear span of
        /// at least <see cref="MinRigSpan"/> under it. False when no pair does.
        /// </summary>
        private static bool DrawRigSizes(WorldScene scene, int p, ref SeededRng rng, out double plankScale, out double supportScale)
        {
            SceneObject plank = scene.Objects[p];
            int a = scene.IndexOf(plank.SupportIds[0]);
            int legs = plank.SupportIds.Length;
            scene.SizeSteps(p, out int plo, out int phi);
            scene.SizeSteps(a, out int slo, out int shi);
            if (!scene.Resizable(p)) plo = phi = 0;
            if (!scene.Resizable(a)) slo = shi = 0;
            int pairs = 0;
            for (int ps = plo; ps <= phi; ps++)
            {
                for (int ss = slo; ss <= shi; ss++) pairs += Span(scene, p, a, legs, ps, ss) >= MinRigSpan ? 1 : 0;
            }

            plankScale = supportScale = 1.0;
            if (pairs == 0) return false;
            int pick = Math.Min(pairs - 1, (int)(rng.NextDouble() * pairs));
            for (int ps = plo; ps <= phi; ps++)
            {
                for (int ss = slo; ss <= shi; ss++)
                {
                    if (Span(scene, p, a, legs, ps, ss) < MinRigSpan) continue;
                    if (pick-- > 0) continue;
                    plankScale = WorldScene.ScaleOfStep(ps);
                    supportScale = WorldScene.ScaleOfStep(ss);
                    return true;
                }
            }

            return false;
        }

        /// <summary>The clear span under plank <paramref name="p"/> at step <paramref name="ps"/> on <paramref name="legs"/> supports like <paramref name="a"/> at step <paramref name="ss"/>.</summary>
        private static double Span(WorldScene scene, int p, int a, int legs, int ps, int ss) =>
            scene.Objects[p].SizeX * WorldScene.ScaleOfStep(ps) - legs * scene.Objects[a].SizeX * WorldScene.ScaleOfStep(ss);

        /// <summary>
        /// The stand a layout raises movable object <paramref name="i"/> on (docs/adr/0038), at its size and rest: uniform among the pairs of
        /// a free stand and one of its steps whose top holds the object's footprint, as it is or turned a quarter, one draw (and one more
        /// for the turn when both hold it); -1 when none does. The chosen stand leaves the free ones.
        /// </summary>
        private static int ChooseStand(WorldScene scene, int i, double scale, int rest, Span<int> stands, ref int freeStands, ref SeededRng rng,
            out double standScale, out bool turned)
        {
            scene.RestExtents(i, rest, out double ex, out double ey, out _);
            ex *= scale;
            ey *= scale;
            int pairs = 0;
            for (int k = 0; k < freeStands; k++)
            {
                StandSteps(scene, stands[k], out int lo, out int hi);
                for (int step = lo; step <= hi; step++) pairs += Holds(scene, stands[k], step, ex, ey) != 0 ? 1 : 0;
            }

            standScale = 1.0;
            turned = false;
            if (pairs == 0) return -1;
            int pick = Math.Min(pairs - 1, (int)(rng.NextDouble() * pairs));
            for (int k = 0; k < freeStands; k++)
            {
                StandSteps(scene, stands[k], out int lo, out int hi);
                for (int step = lo; step <= hi; step++)
                {
                    int holds = Holds(scene, stands[k], step, ex, ey);
                    if (holds == 0 || pick-- > 0) continue;
                    int stand = stands[k];
                    stands[k] = stands[freeStands - 1];
                    freeStands--;
                    standScale = WorldScene.ScaleOfStep(step);
                    turned = holds == 2 || (holds == 3 && rng.NextDouble() < 0.5);
                    return stand;
                }
            }

            return -1;
        }

        private static void StandSteps(WorldScene scene, int k, out int lo, out int hi)
        {
            if (scene.Resizable(k)) scene.SizeSteps(k, out lo, out hi);
            else lo = hi = 0;
        }

        /// <summary>Whether stand <paramref name="k"/>'s top at <paramref name="step"/> holds a footprint <paramref name="ex"/> by <paramref name="ey"/>: 1 as it is, 2 turned a quarter, 3 both ways, 0 neither.</summary>
        private static int Holds(WorldScene scene, int k, int step, double ex, double ey)
        {
            double f = WorldScene.ScaleOfStep(step), sx = scene.Objects[k].SizeX * f, sy = scene.Objects[k].SizeY * f;
            return (ex <= sx + 1e-12 && ey <= sy + 1e-12 ? 1 : 0) | (ey <= sx + 1e-12 && ex <= sy + 1e-12 ? 2 : 0);
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
