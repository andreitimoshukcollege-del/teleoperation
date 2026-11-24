using System;
using Teleop.RobotArm.Types;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.World
{
    public enum SceneShape
    {
        Box,
        Cylinder,
        Sphere,

        /// <summary>
        /// A real object's scanned mesh (docs/adr/0037 decision 4): Isaac draws and collides <see cref="SceneObject.Asset"/>;
        /// everything else (footprints, the headset, joint-drive's plans) takes its axis-aligned box, the file's size.
        /// </summary>
        Mesh,
    }

    /// <summary>Who draws a surface: the headset's lab already has it, the headset builds it from the file, or only Isaac.</summary>
    public enum SurfaceDisplay
    {
        Builtin,
        Show,
        Hide,
    }

    /// <summary>
    /// What a mixed pool's layout does with an object (docs/adr/0038 decision 3): <see cref="None"/> for every object of a scene that
    /// is not one.
    /// </summary>
    public enum SceneRole
    {
        None,

        /// <summary>One of the 0-3 things a layout stands in the arm's reach, anywhere over its reachable workspace.</summary>
        Obstacle,

        /// <summary>In the cameras' view and beyond every arm pose's reach.</summary>
        Distractor,

        /// <summary>A box fixture a movable obstacle is raised on, so contacts come at other heights.</summary>
        Stand,

        /// <summary>A box fixture held up on its supports: an overhang the arm meets from the side or from below.</summary>
        Plank,

        /// <summary>A plank's leg (a bridge has two) or its base (a cantilever has one): a box fixture under one of its ends.</summary>
        Support,
    }

    /// <summary>A flat box in the root frame (ROS convention: Z up, X forward), axis-aligned.</summary>
    public sealed class SceneSurface
    {
        internal SceneSurface(string id, double cx, double cy, double cz, double sx, double sy, double sz,
            float r, float g, float b, double staticFriction, double dynamicFriction, bool collides, SurfaceDisplay display)
        {
            Id = id;
            CenterX = cx;
            CenterY = cy;
            CenterZ = cz;
            SizeX = sx;
            SizeY = sy;
            SizeZ = sz;
            R = r;
            G = g;
            B = b;
            StaticFriction = staticFriction;
            DynamicFriction = dynamicFriction;
            Collides = collides;
            Display = display;
        }

        public string Id { get; }
        public double CenterX { get; }
        public double CenterY { get; }
        public double CenterZ { get; }
        public double SizeX { get; }
        public double SizeY { get; }
        public double SizeZ { get; }
        public float R { get; }
        public float G { get; }
        public float B { get; }
        public double StaticFriction { get; }
        public double DynamicFriction { get; }
        public bool Collides { get; }
        public SurfaceDisplay Display { get; }
        public double XMin => CenterX - SizeX / 2;
        public double XMax => CenterX + SizeX / 2;
        public double YMin => CenterY - SizeY / 2;
        public double YMax => CenterY + SizeY / 2;
        public double Top => CenterZ + SizeZ / 2;
    }

    /// <summary>An object standing upright on the work table at its nominal pose: a fixed obstacle, or a movable one.</summary>
    public sealed class SceneObject
    {
        internal SceneObject(byte id, string name, SceneShape shape, double sx, double sy, double sz, double x, double y, double z, double yaw,
            bool movable, double mass, bool operatorMovable, double staticFriction, double dynamicFriction, float r, float g, float b, double jx, double jy, double jyaw,
            string asset = "", bool clutter = false, SceneRole role = SceneRole.None, byte[]? supports = null, int stepMin = 0, int stepMax = 0,
            bool hasSteps = false, bool overhead = false)
        {
            Overhead = overhead;
            Asset = asset;
            Clutter = clutter;
            Role = role;
            SupportIds = supports ?? Array.Empty<byte>();
            HasSteps = hasSteps;
            StepMin = hasSteps ? stepMin : 0;
            StepMax = hasSteps ? stepMax : 0;
            Id = id;
            Name = name;
            Shape = shape;
            SizeX = sx;
            SizeY = sy;
            SizeZ = sz;
            NominalX = x;
            NominalY = y;
            NominalZ = z;
            NominalYaw = yaw;
            Movable = movable;
            Mass = mass;
            OperatorMovable = operatorMovable;
            StaticFriction = staticFriction;
            DynamicFriction = dynamicFriction;
            R = r;
            G = g;
            B = b;
            JitterX = jx;
            JitterY = jy;
            JitterYaw = jyaw;
            FootprintRadius = shape == SceneShape.Box || shape == SceneShape.Mesh ? 0.5 * Math.Sqrt(sx * sx + sy * sy) : 0.5 * sx;
            RigidTransform turn = RigidTransform.Rotation(0, 0, 1, yaw);
            Nominal = new RigidTransform(x, y, z, turn.Qw, turn.Qx, turn.Qy, turn.Qz);
        }

        public byte Id { get; }
        public string Name { get; }
        public SceneShape Shape { get; }

        /// <summary>A <see cref="SceneShape.Mesh"/>'s asset, <c>library/name</c> (<c>ycb/025_mug</c>); empty for the other shapes.</summary>
        public string Asset { get; }

        /// <summary>Full extents: a box's sides; a cylinder's diameter twice, then its height; a sphere's diameter three times.</summary>
        public double SizeX { get; }
        public double SizeY { get; }
        public double SizeZ { get; }
        public double NominalX { get; }
        public double NominalY { get; }
        public double NominalZ { get; }
        public double NominalYaw { get; }
        public RigidTransform Nominal { get; }

        /// <summary>
        /// A body physics moves (version 2): it has <see cref="Mass"/>, falls, and an arm can push or knock it over. A fixed one
        /// moves only by a placement. Only a physics backend (Isaac) tells them apart; a placement stands either upright.
        /// </summary>
        public bool Movable { get; }

        /// <summary>kg; 0 for a fixed object.</summary>
        public double Mass { get; }

        /// <summary>Whether the headset may move it (EDIT OBJECTS); independent of <see cref="Movable"/>.</summary>
        public bool OperatorMovable { get; }

        /// <summary>
        /// Raised desk clutter (docs/adr/0037 "Raised clutter"): a movable object low enough to lie flat as the file has it (a roll
        /// of tape, tools, a phone, a pen) yet tall enough for the arm to hit. A pool's layout stands it in its own pass, after
        /// every other object, anywhere in the placement region (<see cref="WorldScene.InReach"/> holds it to no band); it never
        /// lies on a side and is not one of the pool's movable objects.
        /// </summary>
        public bool Clutter { get; }

        /// <summary>What a mixed pool's layout does with it (docs/adr/0038 decision 3); <see cref="SceneRole.None"/> otherwise.</summary>
        public SceneRole Role { get; }

        /// <summary>A plank's supports, by id (docs/adr/0038): one (a cantilever) or two (a bridge); empty for any other object.</summary>
        public byte[] SupportIds { get; }

        /// <summary>
        /// A plank on two supports that a mixed layout lays across the arm's reach as an overhead bar (docs/adr/0038, the scene's third
        /// pass): <see cref="WorldScene.DrawBarSpot"/>, its posts beyond the obstacles' band.
        /// </summary>
        public bool Overhead { get; }

        /// <summary>
        /// Whether a mixed pool's layout draws its size (docs/adr/0038): a resize step uniform in <see cref="StepMin"/> to
        /// <see cref="StepMax"/>, the factor 1.25 to that power on every axis. A box or a cylinder, movable or fixed.
        /// </summary>
        public bool HasSteps { get; }
        public int StepMin { get; }
        public int StepMax { get; }
        public double StaticFriction { get; }
        public double DynamicFriction { get; }
        public float R { get; }
        public float G { get; }
        public float B { get; }
        public double JitterX { get; }
        public double JitterY { get; }
        public double JitterYaw { get; }

        /// <summary>The radius of a circle that holds the footprint at any yaw: what placement and overlap checks use.</summary>
        public double FootprintRadius { get; }
    }

    /// <summary>
    /// The world a scene file describes (docs/adr/0031 amendment 2), validated: the surfaces the robot stands on and works
    /// over, and the fixed obstacles on the work table. Root frame, metres and radians. Built by
    /// <see cref="SceneDocument.ToScene"/>; nothing here reads a file. Every check that decides where an object may stand
    /// (the file's own layout, <see cref="SceneLayout"/>, a move from the headset) goes through <see cref="FitsOnTable"/>
    /// and <see cref="Overlaps"/>, so all of them agree.
    /// </summary>
    public sealed class WorldScene
    {
        public const string Format = "teleop-scene";

        /// <summary>The newest version this reader knows; it reads every version from 1. Version 2 adds movable objects.</summary>
        public const int Version = 2;

        /// <summary>
        /// The most objects a scene holds (robot/isaac's <c>scene.MAX_OBJECTS</c> too): 32 since raised clutter joined the JetRover
        /// bench's 24 (docs/adr/0037 "Raised clutter"). Every buffer that holds a scene's objects is sized from it; the world
        /// frames split a snapshot into parts (<see cref="Wire.WorldCodec.MaxObjectsPerFrame"/> to the headset, 16 to Isaac).
        /// </summary>
        public const int MaxObjects = 32;

        /// <summary>
        /// The sizes the headset may give an object (docs/adr/0034 amendment 6): a factor on the scene file's size, in every
        /// direction, <see cref="ResizeStep"/> to the power <see cref="MinResizeStep"/> to <see cref="MaxResizeStep"/> (0.512 to
        /// 3.05). Steps, not any factor: Isaac cannot change a collider while it simulates, so it builds a body for every step
        /// and swaps them.
        /// </summary>
        public const double ResizeStep = 1.25;
        public const int MinResizeStep = -3, MaxResizeStep = 8;
        public static readonly double MinObjectScale = Math.Pow(ResizeStep, MinResizeStep);
        public static readonly double MaxObjectScale = Math.Pow(ResizeStep, MaxResizeStep);

        /// <summary>The size factor of step <paramref name="step"/>, clamped to the steps there are.</summary>
        public static double ScaleOfStep(int step) => Math.Pow(ResizeStep, Math.Max(MinResizeStep, Math.Min(MaxResizeStep, step)));

        /// <summary>The step nearest <paramref name="scale"/>.</summary>
        public static int StepOf(double scale) => scale > 0 ? (int)Math.Round(Math.Log(scale) / Math.Log(ResizeStep)) : MinResizeStep;

        /// <summary>The heaviest movable object, kg: anything heavier is not something the arms are meant to knock over.</summary>
        public const double MaxMass = 20.0;

        /// <summary>How far a surface or a resting object may be from where it must be, m.</summary>
        public const double Tolerance = 0.001;

        /// <summary>
        /// Where a parked object rests (docs/adr/0037 decision 3): upright on the bridge's hidden shelf, 5 m below the floor and
        /// 17 m out, a metre from the next, beyond every camera's far plane and every arm. robot/isaac's <c>Scene.parked_pose</c>
        /// (and <c>IsaacWorld.parked</c> at the file's size) is the same pose.
        /// </summary>
        public const double ParkX = 17.0, ParkZ = -5.0;

        private WorldScene(string name, string robotProfile, SceneSurface[] surfaces, SceneSurface table, SceneSurface? bench,
            SceneObject[] objects, double xMin, double xMax, double yMin, double yMax, double clearance,
            int movableMin, int movableMax, int fixedMin, int fixedMax, double sizeMin, double sizeMax, double reachMin, double reachMax,
            double knockMaxMass, double[] fixedRegions, double[] robotKeepOut, double fixedNearMax,
            double clusterShare, double lyingShare, double[] edgeRegions, double edgeShare, int clutterMin, int clutterMax,
            int obstacleMin, int obstacleMax, double reachBearing, double standShare, double rigShare, int distractorMin, int distractorMax,
            double[] distractorRegions, double distractorReach, double tallShare, double workMin, double workMax, double workBearing,
            double standTopMin, double standTopMax, double rigUnderMin, double rigUnderMax, double overheadShare, double[] armRest)
        {
            OverheadShare = overheadShare;
            ArmRest = armRest;
            TallShare = tallShare;
            WorkMin = workMin;
            WorkMax = workMax;
            WorkBearing = workBearing;
            StandTopMin = standTopMin;
            StandTopMax = standTopMax;
            RigUnderMin = rigUnderMin;
            RigUnderMax = rigUnderMax;
            ObstacleMin = obstacleMin;
            ObstacleMax = obstacleMax;
            ReachBearing = reachBearing;
            StandShare = standShare;
            RigShare = rigShare;
            DistractorMin = distractorMin;
            DistractorMax = distractorMax;
            DistractorRegions = distractorRegions;
            DistractorReach = distractorReach;
            foreach (SceneObject o in objects) _anySteps |= o.HasSteps;
            ClutterMin = clutterMin;
            ClutterMax = clutterMax;
            ClusterShare = clusterShare;
            LyingShare = lyingShare;
            EdgeRegions = edgeRegions;
            EdgeShare = edgeShare;
            FixedNearMax = fixedNearMax;
            KnockMaxMass = knockMaxMass;
            FixedRegions = fixedRegions;
            RobotKeepOut = robotKeepOut;
            ReachMin = reachMin;
            ReachMax = reachMax;
            SizeMin = sizeMin;
            SizeMax = sizeMax;
            MovableMin = movableMin;
            MovableMax = movableMax;
            FixedMin = fixedMin;
            FixedMax = fixedMax;
            Name = name;
            RobotProfile = robotProfile;
            Surfaces = surfaces;
            Table = table;
            Bench = bench;
            Objects = objects;
            RegionXMin = xMin;
            RegionXMax = xMax;
            RegionYMin = yMin;
            RegionYMax = yMax;
            Clearance = clearance;
        }

        public string Name { get; }
        public string RobotProfile { get; }
        public SceneSurface[] Surfaces { get; }
        public SceneSurface Table { get; }
        public SceneSurface? Bench { get; }
        public SceneObject[] Objects { get; }
        public double RegionXMin { get; }
        public double RegionXMax { get; }
        public double RegionYMin { get; }
        public double RegionYMax { get; }
        public double Clearance { get; }

        /// <summary>Whether the objects are a pool that each layout chooses from (docs/adr/0037 decision 3; a mixed pool is one too).</summary>
        public bool IsPool => MovableMax + FixedMax > 0 || Mixed;

        /// <summary>
        /// Whether the pool is a mixed one (docs/adr/0038 decision 3): each layout stands <see cref="ObstacleMin"/> to
        /// <see cref="ObstacleMax"/> obstacles over the arm's reachable workspace and <see cref="DistractorMin"/> to
        /// <see cref="DistractorMax"/> distractors beyond its reach, every object by its <see cref="SceneObject.Role"/>.
        /// </summary>
        public bool Mixed => ObstacleMax + DistractorMax > 0;
        public int ObstacleMin { get; }
        public int ObstacleMax { get; }
        public int DistractorMin { get; }
        public int DistractorMax { get; }

        /// <summary>How far either side of straight ahead (+x) a mixed layout centres an obstacle, rad: the base's yaw range.</summary>
        public double ReachBearing { get; }

        /// <summary>The chance that a movable obstacle is raised on a free stand that holds it (docs/adr/0038).</summary>
        public double StandShare { get; }

        /// <summary>The chance that an obstacle is a plank on its supports, when one is free (docs/adr/0038).</summary>
        public double RigShare { get; }

        /// <summary>Rectangles <c>[xMin, xMax, yMin, yMax, ...]</c> that hold a distractor's centre (docs/adr/0038).</summary>
        public double[] DistractorRegions { get; }

        /// <summary>The nearest a distractor's footprint comes to the base's yaw axis, m: beyond every arm pose's reach.</summary>
        public double DistractorReach { get; }

        /// <summary>
        /// An obstacle at least this tall at its file size, m, is one of a mixed pool's tall thin kinds (docs/adr/0038, the scene's second
        /// pass): a rod, a dowel, a post, a slat. joint-drive's <c>SafetyScript.TallObjectMeters</c> is the same height.
        /// </summary>
        public const double TallHeight = 0.15;

        /// <summary>The chance that an obstacle that is not a plank is a tall one, when one is free (docs/adr/0038, second pass); 0: none drawn so.</summary>
        public double TallShare { get; }

        /// <summary>
        /// The arm's working band (docs/adr/0038, second pass): tall obstacles and planks are centred <see cref="WorkMin"/>-<see cref="WorkMax"/> m
        /// from the base's axis within <see cref="WorkBearing"/> rad of straight ahead. All 0: the obstacles' band.
        /// </summary>
        public double WorkMin { get; }
        public double WorkMax { get; }
        public double WorkBearing { get; }

        /// <summary>
        /// A stand's top sides as fractions of the narrower side of the object it holds (docs/adr/0038, second pass): above 0, a stand is
        /// narrower than its object, which overhangs it and stands upright on it; 0: a stand's top holds the whole footprint.
        /// </summary>
        public double StandTopMin { get; }
        public double StandTopMax { get; }

        /// <summary>The range a rig's underside height is drawn from, uniform, m (docs/adr/0038, second pass); 0, 0: its steps' pairs, uniform.</summary>
        public double RigUnderMin { get; }
        public double RigUnderMax { get; }

        /// <summary>The chance that an obstacle is an overhead bar, when one is free (docs/adr/0038, third pass); 0: none drawn so.</summary>
        public double OverheadShare { get; }

        /// <summary>
        /// Where the arm stands between takes, <c>[xMin, xMax, yMin, yMax, zMin, zMax]</c> (docs/adr/0038, third pass): a mixed layout
        /// stands nothing within the clearance of it (<see cref="InArmRest"/>). Empty: no such box.
        /// </summary>
        public double[] ArmRest { get; }

        /// <summary>The longest a bar's spot is looked for, tries (docs/adr/0038, third pass): its posts and its span leave few spots.</summary>
        public const int MaxBarTries = 256;

        /// <summary>
        /// Whether object <paramref name="index"/> centred at (x, y, z), turned <paramref name="yaw"/>, at its size and rest, comes within
        /// the clearance of <see cref="ArmRest"/> (docs/adr/0038, third pass): its rectangle against the box's in plan, and its heights
        /// against the box's. False when the scene has no such box.
        /// </summary>
        public bool InArmRest(int index, double x, double y, double z, double yaw, double scale, int rest)
        {
            if (ArmRest.Length != 6) return false;
            RestExtents(index, rest, out double ex, out double ey, out double ez);
            double hz = ez * scale / 2;
            if (z - hz >= ArmRest[5] + Clearance - Tolerance || z + hz <= ArmRest[4] - Clearance + Tolerance) return false;
            double kx = (ArmRest[0] + ArmRest[1]) / 2, ky = (ArmRest[2] + ArmRest[3]) / 2, kw = (ArmRest[1] - ArmRest[0]) / 2, kh = (ArmRest[3] - ArmRest[2]) / 2;
            return !RectsApart(x, y, Math.Cos(yaw), Math.Sin(yaw), ex * scale / 2, ey * scale / 2, kx, ky, 1, 0, kw, kh, Clearance);
        }

        /// <summary>
        /// A random spot for overhead bar <paramref name="index"/> (docs/adr/0038, third pass), across the arm's reach: the point of its
        /// line nearest the base's axis <see cref="WorkMin"/>-<see cref="WorkMax"/> m out (uniform) within <see cref="ReachBearing"/> of
        /// straight ahead, the bar square to that bearing, its centre along its line uniform within half its length
        /// (<paramref name="scale"/>) of that point; three draws. Whether it and its posts fit and keep clear is the caller's to check.
        /// </summary>
        public void DrawBarSpot(int index, double scale, ref Teleop.Core.Types.SeededRng rng, out double x, out double y, out double yaw)
        {
            double d = WorkMin + rng.NextDouble() * (WorkMax - WorkMin);
            double bearing = (2 * rng.NextDouble() - 1) * ReachBearing;
            double along = (2 * rng.NextDouble() - 1) * Objects[index].SizeX * scale / 2;
            yaw = bearing + Math.PI / 2;
            if (yaw > Math.PI) yaw -= 2 * Math.PI;
            x = d * Math.Cos(bearing) - along * Math.Sin(bearing);
            y = d * Math.Sin(bearing) + along * Math.Cos(bearing);
        }

        /// <summary>
        /// Whether object <paramref name="index"/>'s rectangle at (x, y), turned <paramref name="yaw"/>, at its size, comes no nearer the
        /// base's axis than <see cref="ReachMax"/> (docs/adr/0038, third pass: an overhead bar's posts stand beyond the obstacles' band,
        /// where the arm's turn at home never reaches).
        /// </summary>
        public bool BeyondBand(int index, double x, double y, double yaw, double scale)
        {
            RestExtents(index, RestUpright, out double ex, out double ey, out _);
            double c = Math.Cos(yaw), s = Math.Sin(yaw);
            double px = -x * c - y * s, py = x * s - y * c;
            double qx = Math.Max(Math.Abs(px) - ex * scale / 2, 0), qy = Math.Max(Math.Abs(py) - ey * scale / 2, 0);
            return Math.Sqrt(qx * qx + qy * qy) >= ReachMax - 1e-9;
        }

        /// <summary>Whether object <paramref name="index"/> is one of a mixed pool's tall thin obstacles (<see cref="TallHeight"/> or taller at its file size).</summary>
        public bool IsTall(int index) => Objects[index].Role == SceneRole.Obstacle && Objects[index].SizeZ >= TallHeight - 1e-12;

        /// <summary>Whether a layout centres object <paramref name="index"/> in the working band: a tall obstacle or a plank, when the pool has one.</summary>
        public bool InWorkBand(int index) => WorkMax > 0 && (IsTall(index) || Objects[index].Role == SceneRole.Plank);

        private readonly bool _anySteps;

        /// <summary>A pool's layout puts <see cref="MovableMin"/> to <see cref="MovableMax"/> movable objects on the table, and <see cref="FixedMin"/> to <see cref="FixedMax"/> fixed ones.</summary>
        public int MovableMin { get; }
        public int MovableMax { get; }
        public int FixedMin { get; }
        public int FixedMax { get; }

        /// <summary>
        /// Whether a pool's layouts also draw sizes (docs/adr/0037 amendment 2): each movable object the headset may resize
        /// gets a resize step that keeps its largest side within <see cref="SizeMin"/> to <see cref="SizeMax"/>, m.
        /// </summary>
        public bool DrawsSizes => SizeMax > 0 || _anySteps;
        public double SizeMin { get; }
        public double SizeMax { get; }

        /// <summary>Where a pool's fixed objects stand, flat <c>[xMin, xMax, yMin, yMax, ...]</c> (docs/adr/0037 amendment 5); empty: the placement region.</summary>
        public double[] FixedRegions { get; }

        /// <summary>A fixed object's near side comes within this of the robot's base in a layout, m; 0: no bound.</summary>
        public double FixedNearMax { get; }

        /// <summary>The robot's footprint no object may reach, <c>[xMin, xMax, yMin, yMax]</c>; empty: none.</summary>
        public double[] RobotKeepOut { get; }

        /// <summary>The chance that each movable object in reach after a layout's first joins a cluster (docs/adr/0037 amendment 6); 0: none.</summary>
        public double ClusterShare { get; }

        /// <summary>The chance that a movable object with a stable side lies on it (docs/adr/0037 amendment 6); 0: every one upright.</summary>
        public double LyingShare { get; }

        /// <summary>
        /// Rectangles <c>[xMin, xMax, yMin, yMax, ...]</c> outside the placement region that hold the centre of a movable object by
        /// the bench's edge, out of the arm's reach (docs/adr/0037 amendment 6); its footprint lies on the table or the bench. Empty: none.
        /// </summary>
        public double[] EdgeRegions { get; }

        /// <summary>The chance that a layout also stands one more movable object in <see cref="EdgeRegions"/>.</summary>
        public double EdgeShare { get; }

        /// <summary>
        /// How many clutter objects (<see cref="SceneObject.Clutter"/>) a pool's layout stands on the table, inclusive (docs/adr/0037
        /// "Raised clutter"): <see cref="SceneClutter"/>'s pass, after every other object. 0, 0: none.
        /// </summary>
        public int ClutterMin { get; }
        public int ClutterMax { get; }

        /// <summary>The widest a cluster's gap between two footprint circles may be beyond <see cref="Clearance"/>, m.</summary>
        public const double ClusterGap = 0.02;

        /// <summary>
        /// The ways an object may rest on the table (docs/adr/0037 amendment 6), each a turn of the object's own frame before its
        /// yaw: 0 upright (its +z up), 1 its +y up (+90 degrees about x), 2 its +x up (-90 degrees about y), 3 upside down,
        /// 4 its -y up, 5 its -x up. Rests 1, 2, 4 and 5 lie on a side. A pose's rest is the one whose up axis is nearest the
        /// vertical (<see cref="RestOf(double, double, double, double, out double)"/>).
        /// </summary>
        public const int RestUpright = 0, RestCount = 6;

        /// <summary>
        /// A side rest is stable enough to be drawn when the object lying on it is at most this many times as tall as its narrower
        /// side: not tippy, as joint-drive's <c>SafetyObject.TippyAspect</c> decides from the rest it is in.
        /// </summary>
        public const double LieAspect = 1.25;

        /// <summary>A side rest lower than this is not drawn, m: too low for the arm to touch (the marker on its side, 2 cm).</summary>
        public const double MinLyingHeight = 0.03;

        /// <summary>The turn of rest <paramref name="rest"/>, a unit quaternion w, x, y, z.</summary>
        public static void RestRotation(int rest, out double qw, out double qx, out double qy, out double qz)
        {
            const double h = 0.70710678118654752;
            qw = 1;
            qx = qy = qz = 0;
            switch (rest)
            {
                case 1: qw = h; qx = h; break;
                case 2: qw = h; qy = -h; break;
                case 3: qw = 0; qx = 1; break;
                case 4: qw = h; qx = -h; break;
                case 5: qw = h; qy = h; break;
            }
        }

        /// <summary>Object <paramref name="index"/>'s extents at rest <paramref name="rest"/> before its yaw, at its file size: along x and y (its footprint) and up.</summary>
        public void RestExtents(int index, int rest, out double ex, out double ey, out double ez)
        {
            SceneObject o = Objects[index];
            switch (rest % 3)
            {
                case 1: ex = o.SizeX; ey = o.SizeZ; ez = o.SizeY; break;
                case 2: ex = o.SizeZ; ey = o.SizeY; ez = o.SizeX; break;
                default: ex = o.SizeX; ey = o.SizeY; ez = o.SizeZ; break;
            }
        }

        /// <summary>The radius of a circle that holds object <paramref name="index"/>'s footprint at rest <paramref name="rest"/> and any yaw, at its file size.</summary>
        public double FootprintRadiusAt(int index, int rest)
        {
            SceneObject o = Objects[index];
            if (rest % 3 == 0 || o.Shape == SceneShape.Sphere) return o.FootprintRadius;
            RestExtents(index, rest, out double ex, out double ey, out _);
            return 0.5 * Math.Sqrt(ex * ex + ey * ey);
        }

        /// <summary>
        /// Whether a layout may lay object <paramref name="index"/> on side rest <paramref name="rest"/>: movable, stable on it
        /// (<see cref="LieAspect"/>) and at least <see cref="MinLyingHeight"/> tall. Never clutter: it lies flat as the file has it.
        /// </summary>
        public bool CanLie(int index, int rest)
        {
            if (rest < 0 || rest >= RestCount || rest % 3 == 0 || !Objects[index].Movable || Objects[index].Clutter || Objects[index].Shape == SceneShape.Sphere) return false;
            RestExtents(index, rest, out double ex, out double ey, out double ez);
            return ez <= LieAspect * Math.Min(ex, ey) + 1e-12 && ez >= MinLyingHeight - 1e-12;
        }

        /// <summary>
        /// The rest of a pose's rotation (w, x, y, z): the one whose up axis is nearest the vertical. <paramref name="yaw"/> is
        /// the turn about the vertical left once that rest is taken out, in (-pi, pi]:
        /// <see cref="PoseAt(int, double, double, double, double, int)"/> gives the pose back. A settled object tilted a little
        /// keeps its rest.
        /// </summary>
        public static int RestOf(double qw, double qx, double qy, double qz, out double yaw)
        {
            // The world z of each of the object's axes: the third row of its rotation matrix.
            double zx = 2 * (qx * qz - qw * qy), zy = 2 * (qy * qz + qw * qx), zz = 1 - 2 * (qx * qx + qy * qy);
            double ax = Math.Abs(zx), ay = Math.Abs(zy), az = Math.Abs(zz);
            int rest = az >= ax && az >= ay ? (zz >= 0 ? 0 : 3) : ay >= ax ? (zy >= 0 ? 1 : 4) : (zx >= 0 ? 2 : 5);
            RestRotation(rest, out double rw, out double rx, out double ry, out double rz);
            double w = qw * rw + qx * rx + qy * ry + qz * rz, z = qz * rw - qw * rz - qx * ry + qy * rx; // q times the rest's inverse
            yaw = 2 * Math.Atan2(z, w);
            if (yaw > Math.PI) yaw -= 2 * Math.PI;
            else if (yaw <= -Math.PI) yaw += 2 * Math.PI;
            return rest;
        }

        /// <summary>The rest of <paramref name="pose"/> (<see cref="RestOf(double, double, double, double, out double)"/>).</summary>
        public static int RestOf(in RigidTransform pose) => RestOf(pose.Qw, pose.Qx, pose.Qy, pose.Qz, out _);

        /// <summary>The rest of <paramref name="pose"/>, and its yaw once that rest is taken out (<see cref="RestOf(double, double, double, double, out double)"/>).</summary>
        public static int RestOf(in RigidTransform pose, out double yaw) => RestOf(pose.Qw, pose.Qx, pose.Qy, pose.Qz, out yaw);

        /// <summary>Whether (x, y) lies in one of <see cref="EdgeRegions"/>.</summary>
        public bool InEdgeRegions(double x, double y)
        {
            for (int k = 0; k + 3 < EdgeRegions.Length; k += 4)
            {
                if (x >= EdgeRegions[k] - 1e-9 && x <= EdgeRegions[k + 1] + 1e-9 && y >= EdgeRegions[k + 2] - 1e-9 && y <= EdgeRegions[k + 3] + 1e-9) return true;
            }

            return false;
        }

        /// <summary>
        /// Whether movable object <paramref name="index"/> at <paramref name="scale"/>, on rest <paramref name="rest"/>, has room by the
        /// edge: some rectangle of <see cref="EdgeRegions"/> holds a centre whose footprint lies on the table or the bench (the
        /// spots <see cref="DrawEdgeSpot"/> draws from). The largest parcels have none.
        /// </summary>
        public bool HasEdgeRoom(int index, double scale, int rest)
        {
            if (!Objects[index].Movable) return false;
            double r = FootprintRadiusAt(index, rest) * scale;
            SurfaceBounds(out double sx0, out double sx1, out double sy0, out double sy1);
            for (int k = 0; k + 3 < EdgeRegions.Length; k += 4)
            {
                if (Math.Max(EdgeRegions[k], sx0 + r) <= Math.Min(EdgeRegions[k + 1], sx1 - r) &&
                    Math.Max(EdgeRegions[k + 2], sy0 + r) <= Math.Min(EdgeRegions[k + 3], sy1 - r)) return true;
            }

            return false;
        }

        /// <summary>Whether a footprint of radius <paramref name="r"/> at (x, y) stays out of <see cref="RobotKeepOut"/>.</summary>
        public bool ClearOfRobot(double x, double y, double r)
        {
            if (RobotKeepOut.Length != 4) return true;
            double dx = Math.Max(Math.Max(RobotKeepOut[0] - x, 0), x - RobotKeepOut[1]);
            double dy = Math.Max(Math.Max(RobotKeepOut[2] - y, 0), y - RobotKeepOut[3]);
            return dx * dx + dy * dy >= r * r - 1e-12;
        }

        /// <summary>Whether object <paramref name="index"/> stands in <see cref="FixedRegions"/> rather than the placement region.</summary>
        public bool InFixedRegions(int index) => FixedRegions.Length > 0 && !Objects[index].Movable;

        /// <summary>
        /// A random spot for object <paramref name="index"/> at <paramref name="scale"/> where it fits (its footprint on the table
        /// and in its region; a movable one also in the reach band, but clutter), drawn from <paramref name="rng"/>: three draws a try (a
        /// fixed-region object first picks its rectangle). A fixed-region object faces the robot's base, give or take
        /// 20 degrees; any other turns at random. False when the draw misses (the caller tries again); the clearance from other
        /// objects is the caller's.
        /// </summary>
        public bool DrawSpot(int index, double scale, ref Teleop.Core.Types.SeededRng rng, out double x, out double y, out double yaw) =>
            DrawSpot(index, scale, RestUpright, ref rng, out x, out y, out yaw);

        /// <summary>As <see cref="DrawSpot(int, double, ref Teleop.Core.Types.SeededRng, out double, out double, out double)"/>, for the object at rest <paramref name="rest"/> (its footprint there).</summary>
        public bool DrawSpot(int index, double scale, int rest, ref Teleop.Core.Types.SeededRng rng, out double x, out double y, out double yaw)
        {
            double r = FootprintRadiusAt(index, rest) * scale;
            double x0, x1, y0, y1;
            if (InFixedRegions(index))
            {
                // A fixed-region rectangle holds the centre; the footprint lies on the table or the bench. Draw x there, then y
                // only where the object clears the robot's keep-out and its near side is within reach (a thin band beside the
                // robot: drawn from the whole rectangle, 3-4% of tries landed in it). FitsOnTable and InReach still decide.
                int k = Math.Min(FixedRegions.Length / 4 - 1, (int)(rng.NextDouble() * (FixedRegions.Length / 4)));
                SurfaceBounds(out double sx0, out double sx1, out double sy0, out double sy1);
                x0 = Math.Max(FixedRegions[4 * k], sx0 + r);
                x1 = Math.Min(FixedRegions[4 * k + 1], sx1 - r);
                x = x0 + rng.NextDouble() * (x1 - x0);
                y0 = Math.Max(FixedRegions[4 * k + 2], sy0 + r);
                y1 = Math.Min(FixedRegions[4 * k + 3], sy1 - r);
                if (FixedNearMax > 0)
                {
                    double reach = FixedNearMax + r, half = reach * reach - x * x;
                    double ymax = half > 0 ? Math.Sqrt(half) : 0;
                    y0 = Math.Max(y0, -ymax);
                    y1 = Math.Min(y1, ymax);
                }

                if (RobotKeepOut.Length == 4 && x >= RobotKeepOut[0] - r && x <= RobotKeepOut[1] + r)
                {
                    if (FixedRegions[4 * k + 2] >= RobotKeepOut[3]) y0 = Math.Max(y0, RobotKeepOut[3] + r);
                    else if (FixedRegions[4 * k + 3] <= RobotKeepOut[2]) y1 = Math.Min(y1, RobotKeepOut[2] - r);
                }

                y = y0 + rng.NextDouble() * (y1 - y0);
            }
            else
            {
                x0 = Math.Max(Table.XMin, RegionXMin) + r;
                x1 = Math.Min(Table.XMax, RegionXMax) - r;
                y0 = Math.Max(Table.YMin, RegionYMin) + r;
                y1 = Math.Min(Table.YMax, RegionYMax) - r;
                x = x0 + rng.NextDouble() * (x1 - x0);
                y = y0 + rng.NextDouble() * (y1 - y0);
            }

            double turn = (2 * rng.NextDouble() - 1) * Math.PI;
            yaw = InFixedRegions(index) ? Math.Atan2(-y, -x) + turn * (20.0 / 180.0) : turn;
            return x0 <= x1 && y0 <= y1 && FitsOnTable(index, x, y, scale, rest) && InReach(index, x, y);
        }

        /// <summary>
        /// A random spot for movable object <paramref name="index"/> at rest <paramref name="rest"/> by the bench's edge, in one of
        /// <see cref="EdgeRegions"/> (docs/adr/0037 amendment 6): its rectangle first, then x and y uniform where its footprint lies
        /// on the table or the bench, and yaw uniform; four draws. False when the draw misses (that rectangle has no room, or the
        /// footprint reaches the robot's keep-out); the clearance from other objects is the caller's. The reach band does not apply.
        /// </summary>
        public bool DrawEdgeSpot(int index, double scale, int rest, ref Teleop.Core.Types.SeededRng rng, out double x, out double y, out double yaw)
        {
            x = y = yaw = 0;
            int rects = EdgeRegions.Length / 4;
            if (rects == 0) return false;
            double r = FootprintRadiusAt(index, rest) * scale;
            int k = Math.Min(rects - 1, (int)(rng.NextDouble() * rects));
            SurfaceBounds(out double sx0, out double sx1, out double sy0, out double sy1);
            double x0 = Math.Max(EdgeRegions[4 * k], sx0 + r), x1 = Math.Min(EdgeRegions[4 * k + 1], sx1 - r);
            double y0 = Math.Max(EdgeRegions[4 * k + 2], sy0 + r), y1 = Math.Min(EdgeRegions[4 * k + 3], sy1 - r);
            x = x0 + rng.NextDouble() * (x1 - x0);
            y = y0 + rng.NextDouble() * (y1 - y0);
            yaw = (2 * rng.NextDouble() - 1) * Math.PI;
            return x0 <= x1 && y0 <= y1 && FitsOnTable(index, x, y, scale, rest);
        }

        /// <summary>The table and the bench beside it, together: where a fixture's or an edge object's footprint may lie.</summary>
        private void SurfaceBounds(out double sx0, out double sx1, out double sy0, out double sy1)
        {
            sx0 = Bench != null ? Math.Min(Table.XMin, Bench.XMin) : Table.XMin;
            sx1 = Bench != null ? Math.Max(Table.XMax, Bench.XMax) : Table.XMax;
            sy0 = Bench != null ? Math.Max(Table.YMin, Bench.YMin) : Table.YMin;
            sy1 = Bench != null ? Math.Min(Table.YMax, Bench.YMax) : Table.YMax;
        }

        /// <summary>The heaviest a movable object may be at its size, kg, and be one the arm knocks over (docs/adr/0037 P4); 0: no roles.</summary>
        public double KnockMaxMass { get; }

        /// <summary>A pool's band for its movable objects, m from the robot's base (0, 0 when there is none): where the arm reaches them.</summary>
        public double ReachMin { get; }
        public double ReachMax { get; }

        /// <summary>
        /// Whether a pool's layout may stand object <paramref name="index"/> at (x, y): a movable object within the reach band, when
        /// there is one; any other anywhere. A movable object by the edge (<see cref="DrawEdgeSpot"/>) is out of reach on purpose,
        /// and its draw does not ask. Clutter (docs/adr/0037 "Raised clutter") goes anywhere in the region: some in reach, some in
        /// the cameras' foreground.
        /// </summary>
        public bool InReach(int index, double x, double y)
        {
            if (InFixedRegions(index) && FixedNearMax > 0) return Math.Sqrt(x * x + y * y) - Objects[index].FootprintRadius <= FixedNearMax;
            if (!(ReachMax > 0) || !Objects[index].Movable || Objects[index].Clutter) return true;
            double r = Math.Sqrt(x * x + y * y);
            return r >= ReachMin && r <= ReachMax;
        }

        /// <summary>
        /// Whether a layout may give object <paramref name="index"/> another size: a movable object the headset may resize, in a pool
        /// that draws sizes; or one with steps of its own (docs/adr/0038), movable or fixed.
        /// </summary>
        public bool Resizable(int index) =>
            Objects[index].HasSteps || (SizeMax > 0 && Objects[index].Movable && Objects[index].OperatorMovable && Objects[index].Shape != SceneShape.Mesh);

        /// <summary>
        /// The resize steps a layout may give object <paramref name="index"/>: its own (docs/adr/0038), or those that keep its largest
        /// side within the pool's size range (step 0 when none is).
        /// </summary>
        public void SizeSteps(int index, out int min, out int max)
        {
            SceneObject o = Objects[index];
            if (o.HasSteps)
            {
                min = o.StepMin;
                max = o.StepMax;
                return;
            }

            double largest = Math.Max(o.SizeX, Math.Max(o.SizeY, o.SizeZ));
            min = MaxResizeStep + 1;
            max = MinResizeStep - 1;
            for (int k = MinResizeStep; k <= MaxResizeStep; k++)
            {
                double side = largest * ScaleOfStep(k);
                if (side < SizeMin - 1e-9 || side > SizeMax + 1e-9) continue;
                min = Math.Min(min, k);
                max = Math.Max(max, k);
            }

            if (min > max) min = max = 0;
        }

        /// <summary>Where object <paramref name="index"/> rests while parked: see <see cref="ParkX"/>.</summary>
        public RigidTransform ParkedPose(int index) => ParkedPose(index, 1.0);

        /// <summary>As <see cref="ParkedPose(int)"/>, at size <paramref name="scale"/>.</summary>
        public RigidTransform ParkedPose(int index, double scale) =>
            new RigidTransform(ParkX, (index - Objects.Length / 2.0) * 1.0, ParkZ + Objects[index].SizeZ * scale / 2 + 0.001, 1, 0, 0, 0);

        /// <summary>The object's index in <see cref="Objects"/>, or -1.</summary>
        public int IndexOf(byte id)
        {
            for (int i = 0; i < Objects.Length; i++)
            {
                if (Objects[i].Id == id) return i;
            }

            return -1;
        }

        /// <summary>
        /// Whether object <paramref name="index"/>'s footprint, centred at (x, y), lies on the table and inside the placement region
        /// (a pool's fixed object: in <see cref="FixedRegions"/>; a movable one may also stand in <see cref="EdgeRegions"/>), clear
        /// of the robot's keep-out. Upright: see the overload with a rest for one lying on a side.
        /// </summary>
        public bool FitsOnTable(int index, double x, double y) => FitsOnTable(index, x, y, 1.0);

        /// <summary>As <see cref="FitsOnTable(int, double, double)"/>, for the object resized by <paramref name="scale"/>.</summary>
        public bool FitsOnTable(int index, double x, double y, double scale) => FitsOnTable(index, x, y, scale, RestUpright);

        /// <summary>
        /// As <see cref="FitsOnTable(int, double, double, double)"/>, for the object at rest <paramref name="rest"/> (its footprint
        /// there). A movable object may also stand in one of <see cref="EdgeRegions"/>: its centre there, its footprint on the
        /// table or the bench (docs/adr/0037 amendment 6).
        /// </summary>
        public bool FitsOnTable(int index, double x, double y, double scale, int rest)
        {
            double r = FootprintRadiusAt(index, rest) * scale;
            if (Mixed) return FitsMixed(index, x, y, r);
            if (!ClearOfRobot(x, y, r)) return false;
            if (InFixedRegions(index))
            {
                // Its centre in a rectangle, its footprint on the table or on the bench beside it (flush with it).
                SurfaceBounds(out double sx0, out double sx1, out double sy0, out double sy1);
                if (x - r < sx0 - 1e-9 || x + r > sx1 + 1e-9 || y - r < sy0 - 1e-9 || y + r > sy1 + 1e-9) return false;
                for (int k = 0; k + 3 < FixedRegions.Length; k += 4)
                {
                    if (x >= FixedRegions[k] - 1e-9 && x <= FixedRegions[k + 1] + 1e-9 && y >= FixedRegions[k + 2] - 1e-9 && y <= FixedRegions[k + 3] + 1e-9)
                    {
                        return true;
                    }
                }

                return false;
            }

            if (x - r >= Math.Max(Table.XMin, RegionXMin) - 1e-9 && x + r <= Math.Min(Table.XMax, RegionXMax) + 1e-9 &&
                y - r >= Math.Max(Table.YMin, RegionYMin) - 1e-9 && y + r <= Math.Min(Table.YMax, RegionYMax) + 1e-9)
            {
                return true;
            }

            if (!Objects[index].Movable || Objects[index].Clutter || !InEdgeRegions(x, y)) return false;
            SurfaceBounds(out double ex0, out double ex1, out double ey0, out double ey1);
            return x - r >= ex0 - 1e-9 && x + r <= ex1 + 1e-9 && y - r >= ey0 - 1e-9 && y + r <= ey1 + 1e-9;
        }

        /// <summary>
        /// As <see cref="FitsOnTable(int, double, double, double, int)"/>, for the object turned <paramref name="yaw"/> about the vertical
        /// (on its rest). Clutter (docs/adr/0037 "Raised clutter") is taken as its own footprint, the rectangle of its sides on that
        /// rest turned by the yaw: inside the placement region on the table, clear of the robot's keep-out (the circle that holds a
        /// 27 cm pair of screwdrivers at any yaw is wider than the region allows, beside the objects in reach). Every other object
        /// is taken as its circle, exactly as the overload without a yaw takes it.
        /// </summary>
        public bool FitsOnTable(int index, double x, double y, double yaw, double scale, int rest)
        {
            if (Mixed) return FitsMixed(index, x, y, yaw, scale, rest);
            if (!Objects[index].Clutter) return FitsOnTable(index, x, y, scale, rest);
            ClutterHalf(index, rest, scale, out double hx, out double hy);
            double c = Math.Cos(yaw), s = Math.Sin(yaw);
            double bx = Math.Abs(c) * hx + Math.Abs(s) * hy, by = Math.Abs(s) * hx + Math.Abs(c) * hy;
            if (!(x - bx >= Math.Max(Table.XMin, RegionXMin) - 1e-9 && x + bx <= Math.Min(Table.XMax, RegionXMax) + 1e-9 &&
                  y - by >= Math.Max(Table.YMin, RegionYMin) - 1e-9 && y + by <= Math.Min(Table.YMax, RegionYMax) + 1e-9))
            {
                return false;
            }

            if (RobotKeepOut.Length != 4) return true;
            double kx = (RobotKeepOut[0] + RobotKeepOut[1]) / 2, ky = (RobotKeepOut[2] + RobotKeepOut[3]) / 2;
            double kw = (RobotKeepOut[1] - RobotKeepOut[0]) / 2, kh = (RobotKeepOut[3] - RobotKeepOut[2]) / 2;
            return RectsApart(x, y, c, s, hx, hy, kx, ky, 1, 0, kw, kh, 0);
        }

        /// <summary>
        /// As <see cref="Overlaps(int, double, double, int, double, double, double, double, int, int)"/>, for the two objects turned
        /// <paramref name="yawA"/> and <paramref name="yawB"/> (on their rests). A piece of clutter (docs/adr/0037 "Raised clutter") is
        /// its own footprint, the rectangle of its sides on its rest turned by its yaw: against another object's circle, the circle
        /// may come no closer to the rectangle than the clearance; two pieces of clutter must lie the clearance apart along one of
        /// their four sides' directions (which may keep two apart that are a little further than that at a corner). Two objects that
        /// are not clutter are their circles, exactly as the overload without yaws takes them.
        /// </summary>
        public bool Overlaps(int a, double ax, double ay, double yawA, int b, double bx, double by, double yawB, double scaleA, double scaleB, int restA, int restB)
        {
            bool ca = Objects[a].Clutter, cb = Objects[b].Clutter;
            if (!ca && !cb) return Overlaps(a, ax, ay, b, bx, by, scaleA, scaleB, restA, restB);
            if (ca && cb)
            {
                ClutterHalf(a, restA, scaleA, out double hxa, out double hya);
                ClutterHalf(b, restB, scaleB, out double hxb, out double hyb);
                return !RectsApart(ax, ay, Math.Cos(yawA), Math.Sin(yawA), hxa, hya, bx, by, Math.Cos(yawB), Math.Sin(yawB), hxb, hyb, Clearance);
            }

            // One piece of clutter and one circle: how near the circle's centre comes to the rectangle, in the rectangle's frame.
            int rect = ca ? a : b, circle = ca ? b : a;
            double rx = ca ? ax : bx, ry = ca ? ay : by, ryaw = ca ? yawA : yawB, cx = ca ? bx : ax, cy = ca ? by : ay;
            ClutterHalf(rect, ca ? restA : restB, ca ? scaleA : scaleB, out double hx, out double hy);
            double c = Math.Cos(ryaw), s = Math.Sin(ryaw), dx = cx - rx, dy = cy - ry;
            double qx = Math.Max(Math.Abs(dx * c + dy * s) - hx, 0), qy = Math.Max(Math.Abs(-dx * s + dy * c) - hy, 0);
            double reach = FootprintRadiusAt(circle, ca ? restB : restA) * (ca ? scaleB : scaleA) + Clearance;
            return qx * qx + qy * qy < reach * reach;
        }

        /// <summary>
        /// A random spot for clutter object <paramref name="index"/> (docs/adr/0037 "Raised clutter"), flat as the file has it: its yaw
        /// uniform, then x and y uniform where its rectangle at that yaw lies inside the placement region on the table; three
        /// draws. False when the draw misses (no room at that yaw, or the robot's keep-out); the clearance from other objects is the
        /// caller's. No reach band applies.
        /// </summary>
        public bool DrawClutterSpot(int index, ref Teleop.Core.Types.SeededRng rng, out double x, out double y, out double yaw)
        {
            yaw = (2 * rng.NextDouble() - 1) * Math.PI;
            ClutterHalf(index, RestUpright, 1.0, out double hx, out double hy);
            double c = Math.Abs(Math.Cos(yaw)), s = Math.Abs(Math.Sin(yaw));
            double bx = c * hx + s * hy, by = s * hx + c * hy;
            double x0 = Math.Max(Table.XMin, RegionXMin) + bx, x1 = Math.Min(Table.XMax, RegionXMax) - bx;
            double y0 = Math.Max(Table.YMin, RegionYMin) + by, y1 = Math.Min(Table.YMax, RegionYMax) - by;
            x = x0 + rng.NextDouble() * (x1 - x0);
            y = y0 + rng.NextDouble() * (y1 - y0);
            return x0 <= x1 && y0 <= y1 && FitsOnTable(index, x, y, yaw, 1.0, RestUpright);
        }

        /// <summary>Half the sides of object <paramref name="index"/>'s footprint on rest <paramref name="rest"/> at <paramref name="scale"/>, along its own x and y.</summary>
        private void ClutterHalf(int index, int rest, double scale, out double hx, out double hy)
        {
            RestExtents(index, rest, out double ex, out double ey, out _);
            hx = ex * scale / 2;
            hy = ey * scale / 2;
        }

        /// <summary>
        /// Whether two rectangles, each its centre, the cosine and sine of its turn, and its half sides, lie at least
        /// <paramref name="gap"/> apart along one of their four sides' directions (the separating-axis test, touching counts as apart).
        /// </summary>
        private static bool RectsApart(double ax, double ay, double ac, double asn, double ahx, double ahy,
            double bx, double by, double bc, double bsn, double bhx, double bhy, double gap)
        {
            double dx = bx - ax, dy = by - ay;
            for (int k = 0; k < 4; k++)
            {
                // The axis: A's x, A's y, B's x, B's y.
                double nx = k == 0 ? ac : k == 1 ? -asn : k == 2 ? bc : -bsn;
                double ny = k == 0 ? asn : k == 1 ? ac : k == 2 ? bsn : bc;
                double pa = ahx * Math.Abs(nx * ac + ny * asn) + ahy * Math.Abs(-nx * asn + ny * ac);
                double pb = bhx * Math.Abs(nx * bc + ny * bsn) + bhy * Math.Abs(-nx * bsn + ny * bc);
                if (Math.Abs(nx * dx + ny * dy) - pa - pb >= gap - 1e-12) return true;
            }

            return false;
        }

        /// <summary>
        /// The nearest point to (x, y) where object <paramref name="index"/>, resized by <paramref name="scale"/>, fits on the
        /// table and inside the placement region (1 mm in, for float32's rounding): where a resize puts an object that would
        /// otherwise reach past an edge (docs/adr/0034 amendment 7). False when it fits nowhere.
        /// </summary>
        public bool TryClampOnTable(int index, double x, double y, double scale, out double cx, out double cy)
        {
            double r = Objects[index].FootprintRadius * scale + 0.001;
            double x0 = Math.Max(Table.XMin, RegionXMin) + r, x1 = Math.Min(Table.XMax, RegionXMax) - r;
            double y0 = Math.Max(Table.YMin, RegionYMin) + r, y1 = Math.Min(Table.YMax, RegionYMax) - r;
            cx = Math.Min(Math.Max(x, x0), x1);
            cy = Math.Min(Math.Max(y, y0), y1);
            return x0 <= x1 && y0 <= y1;
        }

        /// <summary>
        /// The nearest point to (x, y), on a <paramref name="step"/> grid over where object <paramref name="index"/> at
        /// <paramref name="scale"/> fits (the clamped point first), that <paramref name="clear"/> accepts: where a resize finds
        /// room (docs/adr/0034 amendment 7). As robot/isaac's <c>Scene.spots_near</c>. False when none is.
        /// </summary>
        public bool TryFindSpot(int index, double x, double y, double scale, Func<double, double, bool> clear, out double fx, out double fy, double step = 0.01)
        {
            fx = fy = 0;
            if (!TryClampOnTable(index, x, y, scale, out double cx, out double cy)) return false;
            if (clear(cx, cy))
            {
                fx = cx;
                fy = cy;
                return true;
            }

            double r = Objects[index].FootprintRadius * scale + 0.001;
            double x0 = Math.Max(Table.XMin, RegionXMin) + r, x1 = Math.Min(Table.XMax, RegionXMax) - r;
            double y0 = Math.Max(Table.YMin, RegionYMin) + r, y1 = Math.Min(Table.YMax, RegionYMax) - r;
            int nx = (int)((x1 - x0) / step) + 1, ny = (int)((y1 - y0) / step) + 1;
            double best = double.MaxValue;
            for (int i = 0; i < nx; i++)
            {
                for (int j = 0; j < ny; j++)
                {
                    double px = x0 + i * step, py = y0 + j * step, d = (px - x) * (px - x) + (py - y) * (py - y);
                    if (d < best && clear(px, py))
                    {
                        best = d;
                        fx = px;
                        fy = py;
                    }
                }
            }

            return best < double.MaxValue;
        }

        /// <summary>Whether two objects' footprints, centred where given, come closer than <see cref="Clearance"/>.</summary>
        public bool Overlaps(int a, double ax, double ay, int b, double bx, double by) => Overlaps(a, ax, ay, b, bx, by, 1.0, 1.0);

        /// <summary>As <see cref="Overlaps(int, double, double, int, double, double)"/>, for the two objects resized.</summary>
        public bool Overlaps(int a, double ax, double ay, int b, double bx, double by, double scaleA, double scaleB) =>
            Overlaps(a, ax, ay, b, bx, by, scaleA, scaleB, RestUpright, RestUpright);

        /// <summary>As <see cref="Overlaps(int, double, double, int, double, double, double, double)"/>, for the two objects at their rests (their footprints there).</summary>
        public bool Overlaps(int a, double ax, double ay, int b, double bx, double by, double scaleA, double scaleB, int restA, int restB)
        {
            double reach = FootprintRadiusAt(a, restA) * scaleA + FootprintRadiusAt(b, restB) * scaleB + Clearance;
            double dx = ax - bx, dy = ay - by;
            return dx * dx + dy * dy < reach * reach;
        }

        /// <summary>Object <paramref name="index"/> standing upright on the table at (x, y), turned <paramref name="yaw"/> about the vertical.</summary>
        public RigidTransform PoseAt(int index, double x, double y, double yaw) => PoseAt(index, x, y, yaw, 1.0);

        /// <summary>As <see cref="PoseAt(int, double, double, double)"/>, for the object resized by <paramref name="scale"/>.</summary>
        public RigidTransform PoseAt(int index, double x, double y, double yaw, double scale)
        {
            RigidTransform turn = RigidTransform.Rotation(0, 0, 1, yaw);
            return new RigidTransform(x, y, Table.Top + Objects[index].SizeZ * scale / 2, turn.Qw, turn.Qx, turn.Qy, turn.Qz);
        }

        /// <summary>
        /// Object <paramref name="index"/> resting on the table at (x, y) on rest <paramref name="rest"/> (docs/adr/0037 amendment 6),
        /// then turned <paramref name="yaw"/> about the vertical, resized by <paramref name="scale"/>: its box's lowest face on the
        /// table top. Rest 0 is <see cref="PoseAt(int, double, double, double, double)"/> exactly.
        /// </summary>
        public RigidTransform PoseAt(int index, double x, double y, double yaw, double scale, int rest)
        {
            if (rest == RestUpright) return PoseAt(index, x, y, yaw, scale);
            RestRotation(rest, out double rw, out double rx, out double ry, out double rz);
            double c = Math.Cos(yaw / 2), s = Math.Sin(yaw / 2);
            RestExtents(index, rest, out _, out _, out double ez);
            return new RigidTransform(x, y, Table.Top + ez * scale / 2, c * rw - s * rz, c * rx - s * ry, c * ry + s * rx, c * rz + s * rw);
        }

        /// <summary>Whether <paramref name="scale"/> is a size the headset may ask for.</summary>
        public static bool IsValidScale(double scale)
        {
            if (!(scale > 0) || double.IsInfinity(scale)) return false;
            int step = StepOf(scale);
            return step >= MinResizeStep && step <= MaxResizeStep && Math.Abs(scale - ScaleOfStep(step)) <= 1e-4 * scale;
        }

        // --- docs/adr/0038 decision 3: a mixed pool -----------------------------------------------------------------------

        /// <summary>Whether <paramref name="scale"/> is one of object <paramref name="index"/>'s own steps (<see cref="SceneObject.HasSteps"/>).</summary>
        public bool InSteps(int index, double scale)
        {
            SceneObject o = Objects[index];
            if (!o.HasSteps || !IsValidScale(scale)) return false;
            int step = StepOf(scale);
            return step >= o.StepMin && step <= o.StepMax;
        }

        /// <summary>The height of object <paramref name="index"/>'s centre resting on the table on rest <paramref name="rest"/> at <paramref name="scale"/>.</summary>
        public double RestingZ(int index, double scale, int rest)
        {
            RestExtents(index, rest, out _, out _, out double ez);
            return Table.Top + ez * scale / 2;
        }

        /// <summary>
        /// Whether a placement with its centre at height <paramref name="z"/> stands object <paramref name="index"/> above the table, not
        /// on it (docs/adr/0038): only in a mixed pool, and more than <see cref="Tolerance"/> up. Every other placement is stood on the
        /// table whatever height it asks, as always. The placement rules take it only from RobotHost's own layouts, never the headset.
        /// </summary>
        public bool IsElevated(int index, double z, double scale, int rest) => Mixed && z > RestingZ(index, scale, rest) + Tolerance;

        /// <summary>As <see cref="PoseAt(int, double, double, double, double, int)"/>, with the centre at height <paramref name="z"/> (on a stand, or a plank on its supports).</summary>
        public RigidTransform PoseAtHeight(int index, double x, double y, double z, double yaw, double scale, int rest)
        {
            RigidTransform p = PoseAt(index, x, y, yaw, scale, rest);
            return new RigidTransform(p.X, p.Y, z, p.Qw, p.Qx, p.Qy, p.Qz);
        }

        /// <summary>
        /// Whether stand <paramref name="k"/>, at (<paramref name="kx"/>, <paramref name="ky"/>, <paramref name="kz"/>) turned
        /// <paramref name="kyaw"/> on rest <paramref name="krest"/> at <paramref name="kscale"/>, holds object <paramref name="index"/> at
        /// (<paramref name="x"/>, <paramref name="y"/>, <paramref name="z"/>) on rest <paramref name="rest"/> at <paramref name="scale"/>:
        /// its bottom on the stand's top (within 2 mm) and its centre over that top. A movable object stands above the table only so
        /// (docs/adr/0038); anything else falls.
        /// </summary>
        public bool HoldsOnTop(int k, double kx, double ky, double kz, double kyaw, double kscale, int krest, int index, double x, double y, double z, double scale, int rest)
        {
            if (Objects[k].Role != SceneRole.Stand) return false;
            RestExtents(k, krest, out double kex, out double key, out double kez);
            RestExtents(index, rest, out _, out _, out double ez);
            if (Math.Abs(kz + kez * kscale / 2 - (z - ez * scale / 2)) > 2 * Tolerance) return false;
            double c = Math.Cos(kyaw), s = Math.Sin(kyaw), dx = x - kx, dy = y - ky;
            return Math.Abs(dx * c + dy * s) <= kex * kscale / 2 + 1e-9 && Math.Abs(-dx * s + dy * c) <= key * kscale / 2 + 1e-9;
        }

        /// <summary>
        /// As <see cref="Overlaps(int, double, double, double, int, double, double, double, double, double, int, int)"/>, with each object's
        /// centre height (docs/adr/0038). In a mixed pool every footprint is the rectangle of its sides on its rest turned by its yaw, and
        /// two objects one above the other (their heights touching within <see cref="Tolerance"/>, or apart) never overlap: a plank on
        /// its legs, an object on its stand, a can under a bridge. Any other pool ignores the heights, exactly as that overload.
        /// </summary>
        public bool OverlapsAtHeights(int a, double ax, double ay, double az, double yawA, int b, double bx, double by, double bz, double yawB,
            double scaleA, double scaleB, int restA, int restB)
        {
            if (!Mixed) return Overlaps(a, ax, ay, yawA, b, bx, by, yawB, scaleA, scaleB, restA, restB);
            RestExtents(a, restA, out double eax, out double eay, out double eaz);
            RestExtents(b, restB, out double ebx, out double eby, out double ebz);
            double ha = eaz * scaleA / 2, hb = ebz * scaleB / 2;
            if (az - ha >= bz + hb - Tolerance || bz - hb >= az + ha - Tolerance) return false;
            return !RectsApart(ax, ay, Math.Cos(yawA), Math.Sin(yawA), eax * scaleA / 2, eay * scaleA / 2,
                bx, by, Math.Cos(yawB), Math.Sin(yawB), ebx * scaleB / 2, eby * scaleB / 2, Clearance);
        }

        /// <summary>Whether (x, y) lies in one of <see cref="DistractorRegions"/>.</summary>
        public bool InDistractorRegions(double x, double y)
        {
            for (int k = 0; k + 3 < DistractorRegions.Length; k += 4)
            {
                if (x >= DistractorRegions[k] - 1e-9 && x <= DistractorRegions[k + 1] + 1e-9 && y >= DistractorRegions[k + 2] - 1e-9 && y <= DistractorRegions[k + 3] + 1e-9) return true;
            }

            return false;
        }

        /// <summary>
        /// A random centre for an obstacle (docs/adr/0038): uniform by area over the band <see cref="ReachMin"/>-<see cref="ReachMax"/>
        /// from the base's yaw axis within <see cref="ReachBearing"/> of straight ahead, and a yaw uniform; three draws. Whether it fits
        /// and keeps clear is the caller's to check.
        /// </summary>
        public void DrawObstacleSpot(ref Teleop.Core.Types.SeededRng rng, out double x, out double y, out double yaw)
        {
            double r = Math.Sqrt(ReachMin * ReachMin + rng.NextDouble() * (ReachMax * ReachMax - ReachMin * ReachMin));
            double bearing = (2 * rng.NextDouble() - 1) * ReachBearing;
            x = r * Math.Cos(bearing);
            y = r * Math.Sin(bearing);
            yaw = (2 * rng.NextDouble() - 1) * Math.PI;
        }

        /// <summary>
        /// As <see cref="DrawObstacleSpot(ref Teleop.Core.Types.SeededRng, out double, out double, out double)"/>, for object
        /// <paramref name="index"/> (docs/adr/0038, second pass): a tall obstacle or a plank uniform by area over the working band when the
        /// pool has one (<see cref="InWorkBand"/>), anything else over the obstacles' band; three draws either way.
        /// </summary>
        public void DrawObstacleSpot(int index, ref Teleop.Core.Types.SeededRng rng, out double x, out double y, out double yaw)
        {
            if (!InWorkBand(index))
            {
                DrawObstacleSpot(ref rng, out x, out y, out yaw);
                return;
            }

            double r = Math.Sqrt(WorkMin * WorkMin + rng.NextDouble() * (WorkMax * WorkMax - WorkMin * WorkMin));
            double bearing = (2 * rng.NextDouble() - 1) * WorkBearing;
            x = r * Math.Cos(bearing);
            y = r * Math.Sin(bearing);
            yaw = (2 * rng.NextDouble() - 1) * Math.PI;
        }

        /// <summary>
        /// A random centre for a distractor (docs/adr/0038): one of <see cref="DistractorRegions"/> by its area, x and y uniform in it,
        /// and a yaw uniform; four draws. Whether it fits and keeps clear is the caller's to check.
        /// </summary>
        public void DrawDistractorSpot(ref Teleop.Core.Types.SeededRng rng, out double x, out double y, out double yaw)
        {
            double total = 0;
            for (int k = 0; k + 3 < DistractorRegions.Length; k += 4) total += (DistractorRegions[k + 1] - DistractorRegions[k]) * (DistractorRegions[k + 3] - DistractorRegions[k + 2]);
            double pick = rng.NextDouble() * total;
            int at = 0;
            for (int k = 0; k + 3 < DistractorRegions.Length; k += 4)
            {
                at = k;
                double area = (DistractorRegions[k + 1] - DistractorRegions[k]) * (DistractorRegions[k + 3] - DistractorRegions[k + 2]);
                if (pick < area) break;
                pick -= area;
            }

            x = DistractorRegions[at] + rng.NextDouble() * (DistractorRegions[at + 1] - DistractorRegions[at]);
            y = DistractorRegions[at + 2] + rng.NextDouble() * (DistractorRegions[at + 3] - DistractorRegions[at + 2]);
            yaw = (2 * rng.NextDouble() - 1) * Math.PI;
        }

        /// <summary>
        /// A mixed pool's rule for a footprint of radius <paramref name="r"/> at (x, y) (docs/adr/0038): on the table or the bench, clear
        /// of the robot's keep-out, and a distractor centred in <see cref="DistractorRegions"/> no nearer the base's axis than
        /// <see cref="DistractorReach"/>; anything else inside the placement region.
        /// </summary>
        private bool FitsMixed(int index, double x, double y, double r)
        {
            if (!ClearOfRobot(x, y, r)) return false;
            SurfaceBounds(out double sx0, out double sx1, out double sy0, out double sy1);
            if (Objects[index].Role == SceneRole.Distractor)
            {
                return InDistractorRegions(x, y) && x - r >= sx0 - 1e-9 && x + r <= sx1 + 1e-9 && y - r >= sy0 - 1e-9 && y + r <= sy1 + 1e-9 &&
                       Math.Sqrt(x * x + y * y) - r >= DistractorReach - 1e-9;
            }

            return x - r >= Math.Max(sx0, RegionXMin) - 1e-9 && x + r <= Math.Min(sx1, RegionXMax) + 1e-9 &&
                   y - r >= Math.Max(sy0, RegionYMin) - 1e-9 && y + r <= Math.Min(sy1, RegionYMax) + 1e-9;
        }

        /// <summary>As <see cref="FitsMixed(int, double, double, double)"/>, the footprint the rectangle of the object's sides on its rest turned by <paramref name="yaw"/>.</summary>
        private bool FitsMixed(int index, double x, double y, double yaw, double scale, int rest)
        {
            ClutterHalf(index, rest, scale, out double hx, out double hy);
            double c = Math.Cos(yaw), s = Math.Sin(yaw);
            double bx = Math.Abs(c) * hx + Math.Abs(s) * hy, by = Math.Abs(s) * hx + Math.Abs(c) * hy;
            SurfaceBounds(out double sx0, out double sx1, out double sy0, out double sy1);
            bool distractor = Objects[index].Role == SceneRole.Distractor;
            double x0 = distractor ? sx0 : Math.Max(sx0, RegionXMin), x1 = distractor ? sx1 : Math.Min(sx1, RegionXMax);
            double y0 = distractor ? sy0 : Math.Max(sy0, RegionYMin), y1 = distractor ? sy1 : Math.Min(sy1, RegionYMax);
            if (!(x - bx >= x0 - 1e-9 && x + bx <= x1 + 1e-9 && y - by >= y0 - 1e-9 && y + by <= y1 + 1e-9)) return false;
            if (distractor)
            {
                // The rectangle's nearest point to the base's axis: the axis in the rectangle's frame, clamped to it.
                double px = -x * c - y * s, py = x * s - y * c;
                double qx = Math.Max(Math.Abs(px) - hx, 0), qy = Math.Max(Math.Abs(py) - hy, 0);
                if (!InDistractorRegions(x, y) || Math.Sqrt(qx * qx + qy * qy) < DistractorReach - 1e-9) return false;
            }

            if (RobotKeepOut.Length != 4) return true;
            double kx = (RobotKeepOut[0] + RobotKeepOut[1]) / 2, ky = (RobotKeepOut[2] + RobotKeepOut[3]) / 2;
            double kw = (RobotKeepOut[1] - RobotKeepOut[0]) / 2, kh = (RobotKeepOut[3] - RobotKeepOut[2]) / 2;
            return RectsApart(x, y, c, s, hx, hy, kx, ky, 1, 0, kw, kh, 0);
        }

        internal static WorldScene Build(SceneDocument d)
        {
            if (d == null) throw new ArgumentNullException(nameof(d));
            if (d.format != Format) throw Bad("format", $"'{d.format}', not '{Format}'");
            if (d.version < 1 || d.version > Version) throw Bad("version", $"{d.version}; this reader knows versions 1 to {Version}");
            if (string.IsNullOrWhiteSpace(d.name)) throw Bad("name", "the scene has no name");
            if (string.IsNullOrWhiteSpace(d.robotProfile)) throw Bad("robotProfile", "the scene names no robot profile");

            SceneSurfaceDocument[] sd = d.surfaces ?? Array.Empty<SceneSurfaceDocument>();
            var surfaces = new SceneSurface[sd.Length];
            SceneSurface? table = null, bench = null;
            for (int i = 0; i < sd.Length; i++)
            {
                SceneSurfaceDocument s = sd[i] ?? throw Bad("surface", $"surface {i} is missing");
                if (string.IsNullOrWhiteSpace(s.id)) throw Bad("surface", $"surface {i} has no id");
                for (int k = 0; k < i; k++)
                {
                    if (surfaces[k].Id == s.id) throw Bad("surface", $"two surfaces are called '{s.id}'");
                }

                double[] c = Vector(s.center, 3, "surface", $"'{s.id}' center");
                double[] z = Vector(s.size, 3, "surface", $"'{s.id}' size");
                if (!(z[0] > 0 && z[1] > 0 && z[2] > 0)) throw Bad("surface", $"'{s.id}' has a size that is not positive");
                double[] col = Colour(s.colour, $"surface '{s.id}'");
                Friction(s.staticFriction, s.dynamicFriction, $"surface '{s.id}'");
                surfaces[i] = new SceneSurface(s.id, c[0], c[1], c[2], z[0], z[1], z[2], (float)col[0], (float)col[1], (float)col[2],
                    s.staticFriction, s.dynamicFriction, s.collides, Display(s.headset, s.id));
                if (s.id == "table") table = surfaces[i];
                if (s.id == "bench") bench = surfaces[i];
            }

            if (table == null) throw Bad("table", "the scene has no surface called 'table'");
            if (bench != null)
            {
                if (Math.Abs(table.XMin - bench.XMax) > Tolerance)
                {
                    throw Bad("table", $"it must abut the bench's far edge: it starts at x = {table.XMin:0.###}, the bench ends at {bench.XMax:0.###}");
                }

                if (Math.Abs(table.Top - bench.Top) > Tolerance) throw Bad("table", $"its top ({table.Top:0.####}) must be flush with the bench's ({bench.Top:0.####})");
            }

            ScenePlacementDocument p = d.placement ?? new ScenePlacementDocument();
            if (!(Finite(p.xMin, p.xMax, p.yMin, p.yMax, p.clearance) && p.xMin < p.xMax && p.yMin < p.yMax && p.clearance >= 0))
            {
                throw Bad("placement", "the region must have min < max on both axes and a clearance of 0 or more");
            }

            if (p.xMax < table.XMin || p.xMin > table.XMax || p.yMax < table.YMin || p.yMin > table.YMax)
            {
                throw Bad("placement", "the region does not overlap the table");
            }

            SceneObjectDocument[] od = d.objects ?? Array.Empty<SceneObjectDocument>();
            if (od.Length < 1 || od.Length > MaxObjects) throw Bad("objects", $"{od.Length}; a scene holds 1 to {MaxObjects}");
            var objects = new SceneObject[od.Length];
            for (int i = 0; i < od.Length; i++)
            {
                SceneObjectDocument o = od[i] ?? throw Bad("objects", $"object {i} is missing");
                if (o.id < 1 || o.id > 254) throw Bad("object id", $"{o.id}; ids run from 1 to 254");
                if (!ValidName(o.name)) throw Bad("object name", $"'{o.name}': letters, digits, '.', '_' and '-' only");
                for (int k = 0; k < i; k++)
                {
                    if (objects[k].Id == o.id) throw Bad("object id", $"two objects have id {o.id}");
                    if (objects[k].Name == o.name) throw Bad("object name", $"two objects are called '{o.name}'");
                }

                SceneShape shape = o.shape switch
                {
                    "box" => SceneShape.Box,
                    "cylinder" => SceneShape.Cylinder,
                    "sphere" => SceneShape.Sphere,
                    "mesh" => SceneShape.Mesh,
                    _ => throw Bad("shape", $"'{o.name}' has shape '{o.shape}': box, cylinder, sphere or mesh"),
                };
                string asset = o.asset ?? "";
                if (shape == SceneShape.Mesh ? !ValidAsset(asset) : asset.Length > 0)
                {
                    throw Bad("asset", shape == SceneShape.Mesh ? $"mesh '{o.name}' needs an asset, library/name in lowercase letters, digits and '_'" : $"'{o.name}' is a {o.shape}: only a mesh has an asset");
                }

                double[] z = Vector(o.size, 3, "size", $"'{o.name}'");
                if (!(z[0] > 0 && z[1] > 0 && z[2] > 0)) throw Bad("size", $"'{o.name}' has a size that is not positive");
                if (shape == SceneShape.Cylinder && Math.Abs(z[0] - z[1]) > 1e-9) throw Bad("size", $"cylinder '{o.name}' must be [d, d, h]");
                if (shape == SceneShape.Sphere && (Math.Abs(z[0] - z[1]) > 1e-9 || Math.Abs(z[0] - z[2]) > 1e-9)) throw Bad("size", $"sphere '{o.name}' must be [d, d, d]");
                bool movable = o.kind == "movable";
                if (o.kind != "fixed" && !(movable && d.version >= 2))
                {
                    throw Bad("kind", d.version >= 2 ? $"'{o.name}' is '{o.kind}': fixed or movable" : $"'{o.name}' is '{o.kind}'; version 1 has fixed objects only");
                }

                if (movable ? !(o.mass > 0 && o.mass <= MaxMass) : o.mass != 0)
                {
                    throw Bad("mass", movable ? $"movable '{o.name}' has mass {o.mass}: more than 0 and at most {MaxMass} kg" : $"fixed '{o.name}' has a mass: only a movable object has one");
                }

                // docs/adr/0037 "Raised clutter": clutter is something the arm can push about, so it is a movable body.
                if (o.clutter && !movable) throw Bad("kind", $"'{o.name}' is clutter, which is movable, not '{o.kind}'");

                // docs/adr/0038 decision 3: a mixed pool's roles, a plank's supports and the steps a layout draws a size from.
                SceneRole role = (o.role ?? "") switch
                {
                    "" => SceneRole.None,
                    "obstacle" => SceneRole.Obstacle,
                    "distractor" => SceneRole.Distractor,
                    "stand" => SceneRole.Stand,
                    "plank" => SceneRole.Plank,
                    "support" => SceneRole.Support,
                    _ => throw Bad("role", $"'{o.name}' has role '{o.role}': obstacle, distractor, stand, plank or support"),
                };
                if ((role == SceneRole.Stand || role == SceneRole.Plank || role == SceneRole.Support) && (movable || shape != SceneShape.Box))
                {
                    throw Bad("role", $"'{o.name}' is a {o.role}, which is a fixed box");
                }

                int[] sup = o.supports ?? Array.Empty<int>();
                if (role == SceneRole.Plank ? sup.Length < 1 || sup.Length > 2 : sup.Length > 0)
                {
                    throw Bad("role", role == SceneRole.Plank ? $"plank '{o.name}' needs one or two supports" : $"'{o.name}' is no plank: only a plank has supports");
                }

                foreach (int id in sup)
                {
                    if (id < 1 || id > 254) throw Bad("role", $"plank '{o.name}' names support {id}; ids run from 1 to 254");
                }

                // docs/adr/0038, the scene's third pass: an overhead bar is a bridge.
                if (o.overhead && !(role == SceneRole.Plank && sup.Length == 2)) throw Bad("role", $"'{o.name}' is overhead, which only a plank on two supports is");

                int[] st = o.steps ?? Array.Empty<int>();
                if (st.Length != 0 && st.Length != 2) throw Bad("steps", $"'{o.name}': steps is [min, max]");
                if (st.Length == 2 && !(st[0] >= MinResizeStep && st[0] <= st[1] && st[1] <= MaxResizeStep))
                {
                    throw Bad("steps", $"'{o.name}': steps run from {MinResizeStep} to {MaxResizeStep}, min <= max");
                }

                if (st.Length == 2 && ((shape != SceneShape.Box && shape != SceneShape.Cylinder) || o.clutter))
                {
                    throw Bad("steps", $"'{o.name}': only a box or a cylinder that is not clutter takes steps");
                }

                double[] pos = Vector(o.position, 3, "pose", $"'{o.name}' position");
                double[] rpy = Vector(o.rpy, 3, "pose", $"'{o.name}' rpy");
                if (Math.Abs(rpy[0]) > 1e-9 || Math.Abs(rpy[1]) > 1e-9) throw Bad("pose", $"'{o.name}' must stand upright (roll and pitch 0)");
                if (Math.Abs(pos[2] - (table.Top + z[2] / 2)) > Tolerance) throw Bad("rests", $"'{o.name}' must rest on the table: its centre at z = {table.Top + z[2] / 2:0.####}, not {pos[2]:0.####}");
                double[] col = Colour(o.colour, $"object '{o.name}'");
                Friction(o.staticFriction, o.dynamicFriction, $"object '{o.name}'");
                double[] j = Vector(o.jitter, 3, "jitter", $"'{o.name}'");
                if (!(j[0] >= 0 && j[1] >= 0 && j[2] >= 0)) throw Bad("jitter", $"'{o.name}' has a negative range");
                var supportIds = new byte[sup.Length];
                for (int k = 0; k < sup.Length; k++) supportIds[k] = (byte)sup[k];
                objects[i] = new SceneObject((byte)o.id, o.name, shape, z[0], z[1], z[2], pos[0], pos[1], pos[2], rpy[2], movable, o.mass, o.operatorMovable,
                    o.staticFriction, o.dynamicFriction, (float)col[0], (float)col[1], (float)col[2], j[0], j[1], j[2], asset, o.clutter, role, supportIds,
                    st.Length == 2 ? st[0] : 0, st.Length == 2 ? st[1] : 0, st.Length == 2, o.overhead);
            }

            ScenePoolDocument pool = d.pool ?? new ScenePoolDocument();
            int movables = 0, fixeds = 0, clutters = 0;
            foreach (SceneObject o in objects)
            {
                if (o.Clutter) clutters++; // not one of the pool's movable objects (docs/adr/0037 "Raised clutter")
                else if (o.Movable) movables++;
                else fixeds++;
            }

            if (pool.movableMin < 0 || pool.fixedMin < 0 || pool.clutterMin < 0 || pool.movableMin > pool.movableMax || pool.fixedMin > pool.fixedMax ||
                pool.clutterMin > pool.clutterMax || pool.obstacleMin < 0 || pool.obstacleMin > pool.obstacleMax || pool.distractorMin < 0 ||
                pool.distractorMin > pool.distractorMax)
            {
                throw Bad("pool", "each kind needs 0 <= min <= max");
            }

            if (pool.movableMax > movables || pool.fixedMax > fixeds)
            {
                throw Bad("pool", $"it asks for up to {pool.movableMax} movable and {pool.fixedMax} fixed objects; the scene has {movables} and {fixeds}");
            }

            if (pool.clutterMax > clutters) throw Bad("pool", $"it asks for up to {pool.clutterMax} clutter objects; the scene has {clutters}");
            bool mixed = pool.obstacleMax + pool.distractorMax > 0; // docs/adr/0038 decision 3
            bool isPool = pool.movableMax + pool.fixedMax > 0 || mixed;
            if (isPool && d.version < 2) throw Bad("pool", "a pool needs version 2");
            if ((clutters > 0 || pool.clutterMax > 0) && !isPool) throw Bad("pool", "clutter is only for a pool, whose layouts choose it");
            if (!Finite(pool.sizeMin, pool.sizeMax) || !((pool.sizeMin == 0 && pool.sizeMax == 0) || (pool.sizeMin > 0 && pool.sizeMin < pool.sizeMax)))
            {
                throw Bad("pool", "sizeMin and sizeMax are both 0, or 0 < sizeMin < sizeMax");
            }

            if (pool.sizeMax > 0 && !isPool) throw Bad("pool", "sizes are drawn only for a pool");
            if (!Finite(pool.reachMin, pool.reachMax) || !((pool.reachMin == 0 && pool.reachMax == 0) || (pool.reachMin >= 0 && pool.reachMin < pool.reachMax && pool.reachMax > 0)))
            {
                throw Bad("pool", "reachMin and reachMax are both 0, or 0 <= reachMin < reachMax");
            }

            if (pool.reachMax > 0 && !isPool) throw Bad("pool", "a reach band is only for a pool");
            if (!Finite(pool.knockMaxMass) || pool.knockMaxMass < 0 || pool.knockMaxMass > MaxMass) throw Bad("pool", $"knockMaxMass is 0 (no roles) to {MaxMass} kg");
            if (pool.knockMaxMass > 0 && !isPool) throw Bad("pool", "roles are only for a pool");
            double[] fr = pool.fixedRegions ?? Array.Empty<double>();
            if (fr.Length % 4 != 0) throw Bad("pool", "fixedRegions holds xMin, xMax, yMin, yMax for each rectangle");
            for (int k = 0; k < fr.Length; k += 4)
            {
                if (!Finite(fr[k], fr[k + 1], fr[k + 2], fr[k + 3]) || !(fr[k] < fr[k + 1] && fr[k + 2] < fr[k + 3]))
                {
                    throw Bad("pool", "each fixedRegions rectangle needs min < max on both axes");
                }

                double sx0 = bench != null ? Math.Min(table.XMin, bench.XMin) : table.XMin, sx1 = bench != null ? Math.Max(table.XMax, bench.XMax) : table.XMax;
                if (fr[k + 1] < sx0 || fr[k] > sx1 || fr[k + 3] < table.YMin || fr[k + 2] > table.YMax)
                {
                    throw Bad("pool", "a fixedRegions rectangle does not overlap the table or the bench");
                }
            }

            if (fr.Length > 0 && !isPool) throw Bad("pool", "fixedRegions are only for a pool");
            double[] ko = pool.robotKeepOut ?? Array.Empty<double>();
            if (ko.Length != 0 && (ko.Length != 4 || !Finite(ko[0], ko[1], ko[2], ko[3]) || !(ko[0] < ko[1] && ko[2] < ko[3])))
            {
                throw Bad("pool", "robotKeepOut is [xMin, xMax, yMin, yMax] with min < max, or absent");
            }

            double near = ValidNear(pool.fixedNearMax);

            // docs/adr/0037 amendment 6, checked in this order by robot/isaac's scene.py too.
            double clusterShare = Share(pool.clusterShare, "clusterShare", isPool);
            double lyingShare = Share(pool.lyingShare, "lyingShare", isPool);
            double[] er = pool.edgeRegions ?? Array.Empty<double>();
            if (er.Length % 4 != 0) throw Bad("pool", "edgeRegions holds xMin, xMax, yMin, yMax for each rectangle");
            for (int k = 0; k < er.Length; k += 4)
            {
                if (!Finite(er[k], er[k + 1], er[k + 2], er[k + 3]) || !(er[k] < er[k + 1] && er[k + 2] < er[k + 3]))
                {
                    throw Bad("pool", "each edgeRegions rectangle needs min < max on both axes");
                }

                double sx0 = bench != null ? Math.Min(table.XMin, bench.XMin) : table.XMin, sx1 = bench != null ? Math.Max(table.XMax, bench.XMax) : table.XMax;
                if (er[k + 1] < sx0 || er[k] > sx1 || er[k + 3] < table.YMin || er[k + 2] > table.YMax)
                {
                    throw Bad("pool", "an edgeRegions rectangle does not overlap the table or the bench");
                }

                if (er[k] < p.xMax && er[k + 1] > p.xMin && er[k + 2] < p.yMax && er[k + 3] > p.yMin)
                {
                    throw Bad("pool", "an edgeRegions rectangle overlaps the placement region: edge objects stand outside it, out of reach");
                }
            }

            if (er.Length > 0 && !isPool) throw Bad("pool", "edgeRegions are only for a pool");
            double edgeShare = Share(pool.edgeShare, "edgeShare", isPool);
            if (edgeShare > 0 && er.Length == 0) throw Bad("pool", "edgeShare needs edgeRegions");

            // docs/adr/0038 decision 3, a mixed pool, checked in this order by robot/isaac's scene.py too.
            if (mixed && (pool.movableMax > 0 || pool.fixedMax > 0 || pool.clutterMax > 0))
            {
                throw Bad("pool", "a mixed pool draws obstacles and distractors: its movable, fixed and clutter counts are 0");
            }

            if (mixed && (clusterShare > 0 || edgeShare > 0 || fr.Length > 0 || er.Length > 0 || pool.sizeMax > 0))
            {
                throw Bad("pool", "a mixed pool has no clusters, edge objects, fixed regions or pool sizes");
            }

            if (!Finite(pool.reachBearing) || pool.reachBearing < 0 || pool.reachBearing > Math.PI) throw Bad("pool", "reachBearing is 0 to pi rad");
            if (pool.reachBearing > 0 && !mixed) throw Bad("pool", "reachBearing is only for a mixed pool");
            if (pool.obstacleMax > 0 && !(pool.reachMax > 0 && pool.reachBearing > 0)) throw Bad("pool", "obstacles need reachMin, reachMax and reachBearing");
            double standShare = MixedShare(pool.standShare, "standShare", mixed);
            double rigShare = MixedShare(pool.rigShare, "rigShare", mixed);

            // docs/adr/0038, the scene's second pass: tall obstacles more often, them and the planks in the arm's working band, stands narrower
            // than what they hold, and rigs' heights uniform over a range. Checked in this order by robot/isaac's scene.py too.
            double tallShare = MixedShare(pool.tallShare, "tallShare", mixed);
            bool work = pool.workMin != 0 || pool.workMax != 0 || pool.workBearing != 0;
            if (work && !mixed) throw Bad("pool", "a work band (workMin, workMax, workBearing) is only for a mixed pool");
            if (work && !(Finite(pool.workMin, pool.workMax, pool.workBearing) && pool.workMin >= pool.reachMin && pool.workMin < pool.workMax &&
                          pool.workMax <= pool.reachMax && pool.workBearing > 0 && pool.workBearing <= pool.reachBearing))
            {
                throw Bad("pool", "a work band lies within the obstacles' band: reachMin <= workMin < workMax <= reachMax and 0 < workBearing <= reachBearing");
            }

            bool narrow = pool.standTopMin != 0 || pool.standTopMax != 0;
            if (narrow && !mixed) throw Bad("pool", "standTopMin and standTopMax are only for a mixed pool");
            if (narrow && !(Finite(pool.standTopMin, pool.standTopMax) && pool.standTopMin > 0 && pool.standTopMin <= pool.standTopMax && pool.standTopMax <= 1))
            {
                throw Bad("pool", "standTopMin and standTopMax are both 0, or 0 < standTopMin <= standTopMax <= 1");
            }

            bool under = pool.rigUnderMin != 0 || pool.rigUnderMax != 0;
            if (under && !mixed) throw Bad("pool", "rigUnderMin and rigUnderMax are only for a mixed pool");
            if (under && !(Finite(pool.rigUnderMin, pool.rigUnderMax) && pool.rigUnderMin > 0 && pool.rigUnderMin < pool.rigUnderMax))
            {
                throw Bad("pool", "rigUnderMin and rigUnderMax are both 0, or 0 < rigUnderMin < rigUnderMax");
            }

            // docs/adr/0038, the scene's third pass: overhead bars, and the box where the arm stands between takes. Checked in this order by
            // robot/isaac's scene.py too.
            double overheadShare = MixedShare(pool.overheadShare, "overheadShare", mixed);
            double[] armRest = pool.armRest ?? Array.Empty<double>();
            if (armRest.Length > 0 && !mixed) throw Bad("pool", "armRest is only for a mixed pool");
            if (armRest.Length > 0 && !(armRest.Length == 6 && Finite(armRest[0], armRest[1], armRest[2], armRest[3]) && Finite(armRest[4], armRest[5]) &&
                                        armRest[0] < armRest[1] && armRest[2] < armRest[3] && armRest[4] < armRest[5]))
            {
                throw Bad("pool", "armRest is [xMin, xMax, yMin, yMax, zMin, zMax] with min < max, or absent");
            }

            double[] dr = pool.distractorRegions ?? Array.Empty<double>();
            if (dr.Length % 4 != 0) throw Bad("pool", "distractorRegions holds xMin, xMax, yMin, yMax for each rectangle");
            for (int k = 0; k < dr.Length; k += 4)
            {
                if (!Finite(dr[k], dr[k + 1], dr[k + 2], dr[k + 3]) || !(dr[k] < dr[k + 1] && dr[k + 2] < dr[k + 3]))
                {
                    throw Bad("pool", "each distractorRegions rectangle needs min < max on both axes");
                }

                double sx0 = bench != null ? Math.Min(table.XMin, bench.XMin) : table.XMin, sx1 = bench != null ? Math.Max(table.XMax, bench.XMax) : table.XMax;
                if (dr[k + 1] < sx0 || dr[k] > sx1 || dr[k + 3] < table.YMin || dr[k + 2] > table.YMax)
                {
                    throw Bad("pool", "a distractorRegions rectangle does not overlap the table or the bench");
                }
            }

            if (dr.Length > 0 && !mixed) throw Bad("pool", "distractorRegions are only for a mixed pool");
            if (!Finite(pool.distractorReach) || pool.distractorReach < 0) throw Bad("pool", "distractorReach is 0 or a distance in metres");
            if (pool.distractorReach > 0 && !mixed) throw Bad("pool", "distractorReach is only for a mixed pool");
            if (pool.distractorMax > 0 && (dr.Length == 0 || !(pool.distractorReach > 0))) throw Bad("pool", "distractors need distractorRegions and distractorReach");
            if (pool.distractorReach > 0 && pool.distractorReach < pool.reachMax) throw Bad("pool", "distractorReach lies within the obstacles' band (reachMax)");

            int obstacles = 0, distractors = 0, planks = 0, overheads = 0;
            for (int i = 0; i < objects.Length; i++)
            {
                SceneObject o = objects[i];
                overheads += o.Overhead ? 1 : 0;
                if (mixed && o.Role == SceneRole.None) throw Bad("role", $"'{o.Name}' has no role: every object of a mixed pool has one");
                if (!mixed && o.Role != SceneRole.None) throw Bad("role", $"'{o.Name}' has a role: roles are for a mixed pool");
                if (!mixed && o.HasSteps) throw Bad("steps", $"'{o.Name}' has steps: they are for a mixed pool's layouts");
                if (o.Role == SceneRole.Obstacle) obstacles++;
                if (o.Role == SceneRole.Distractor) distractors++;
                if (o.Role != SceneRole.Plank) continue;
                planks++;
                foreach (byte id in o.SupportIds)
                {
                    int k = Array.FindIndex(objects, x => x.Id == id);
                    if (k < 0) throw Bad("role", $"plank '{o.Name}' names support {id}, which is no object of the scene");
                    if (objects[k].Role != SceneRole.Support) throw Bad("role", $"plank '{o.Name}' names '{objects[k].Name}', which is no support");
                    int named = 0;
                    foreach (SceneObject q in objects)
                    {
                        if (q.Role != SceneRole.Plank) continue;
                        foreach (byte other in q.SupportIds) named += other == id ? 1 : 0;
                    }

                    if (named > 1) throw Bad("role", $"support '{objects[k].Name}' holds up two planks, or one twice");
                }

                if (o.SupportIds.Length == 2)
                {
                    SceneObject l = objects[Array.FindIndex(objects, x => x.Id == o.SupportIds[0])], r = objects[Array.FindIndex(objects, x => x.Id == o.SupportIds[1])];
                    if (Math.Abs(l.SizeX - r.SizeX) > 1e-9 || Math.Abs(l.SizeY - r.SizeY) > 1e-9 || Math.Abs(l.SizeZ - r.SizeZ) > 1e-9 ||
                        l.HasSteps != r.HasSteps || l.StepMin != r.StepMin || l.StepMax != r.StepMax)
                    {
                        throw Bad("role", $"plank '{o.Name}''s two supports differ in size or steps: they hold it level");
                    }
                }
            }

            foreach (SceneObject o in objects)
            {
                bool held = false;
                foreach (SceneObject q in objects) held |= q.Role == SceneRole.Plank && Array.IndexOf(q.SupportIds, o.Id) >= 0;
                if (o.Role == SceneRole.Support && !held) throw Bad("role", $"support '{o.Name}' holds up no plank");
            }

            if (overheads > 0 && !(overheadShare > 0)) throw Bad("pool", "overhead planks need overheadShare");
            if (overheadShare > 0 && overheads == 0) throw Bad("pool", "overheadShare needs an overhead plank");
            if (pool.obstacleMax > obstacles + planks) throw Bad("pool", $"it asks for up to {pool.obstacleMax} obstacles; the scene has {obstacles} and {planks} planks");
            if (pool.distractorMax > distractors) throw Bad("pool", $"it asks for up to {pool.distractorMax} distractors; the scene has {distractors}");
            var scene = new WorldScene(d.name, d.robotProfile, surfaces, table, bench, objects, p.xMin, p.xMax, p.yMin, p.yMax, p.clearance,
                pool.movableMin, pool.movableMax, pool.fixedMin, pool.fixedMax, pool.sizeMin, pool.sizeMax, pool.reachMin, pool.reachMax,
                pool.knockMaxMass, fr, ko, near, clusterShare, lyingShare, er, edgeShare, pool.clutterMin, pool.clutterMax,
                pool.obstacleMin, pool.obstacleMax, pool.reachBearing, standShare, rigShare, pool.distractorMin, pool.distractorMax, dr, pool.distractorReach,
                tallShare, pool.workMin, pool.workMax, pool.workBearing, pool.standTopMin, pool.standTopMax, pool.rigUnderMin, pool.rigUnderMax,
                overheadShare, armRest);
            for (int i = 0; i < objects.Length; i++)
            {
                SceneObject o = objects[i];
                for (int sx = -1; sx <= 1; sx++)
                {
                    for (int sy = -1; sy <= 1; sy++)
                    {
                        bool fits = mixed
                            ? scene.FitsOnTable(i, o.NominalX + sx * o.JitterX, o.NominalY + sy * o.JitterY, o.NominalYaw, 1.0, RestUpright)
                            : scene.FitsOnTable(i, o.NominalX + sx * o.JitterX, o.NominalY + sy * o.JitterY);
                        if (!fits)
                        {
                            throw Bad("fits", $"'{o.Name}' leaves the table or the placement region at its nominal pose or the edge of its jitter");
                        }
                    }
                }

                // A pool's objects take part a few at a time and start parked, so their nominal poses may share the table.
                for (int k = 0; k < i && !isPool; k++)
                {
                    if (scene.Overlaps(i, o.NominalX, o.NominalY, k, objects[k].NominalX, objects[k].NominalY))
                    {
                        throw Bad("overlap", $"'{o.Name}' and '{objects[k].Name}' are closer than the clearance at their nominal poses");
                    }
                }
            }

            return scene;
        }

        private static double Share(double v, string name, bool isPool)
        {
            if (!Finite(v) || v < 0 || v > 1) throw Bad("pool", $"{name} is a chance from 0 to 1");
            if (v > 0 && !isPool) throw Bad("pool", $"{name} is only for a pool");
            return v;
        }

        private static double MixedShare(double v, string name, bool mixed)
        {
            if (!Finite(v) || v < 0 || v > 1) throw Bad("pool", $"{name} is a chance from 0 to 1");
            if (v > 0 && !mixed) throw Bad("pool", $"{name} is only for a mixed pool");
            return v;
        }

        private static double ValidNear(double v)
        {
            if (!Finite(v) || v < 0) throw Bad("pool", "fixedNearMax is 0 (no bound) or a distance in metres");
            return v;
        }

        private static FormatException Bad(string what, string detail) => new FormatException(what + ": " + detail);

        private static double[] Vector(double[]? v, int length, string what, string where)
        {
            if (v == null || v.Length != length) throw Bad(what, $"{where} must have {length} numbers");
            foreach (double x in v)
            {
                if (double.IsNaN(x) || double.IsInfinity(x)) throw Bad(what, $"{where} must be finite");
            }

            return v;
        }

        private static double[] Colour(double[]? c, string where)
        {
            if (c == null || c.Length != 3) throw Bad("colour", $"{where} needs [r, g, b]");
            foreach (double x in c)
            {
                if (!(x >= 0 && x <= 1)) throw Bad("colour", $"{where}: each part runs from 0 to 1");
            }

            return c;
        }

        private static void Friction(double s, double d, string where)
        {
            if (!(s >= 0 && d >= 0) || double.IsInfinity(s + d)) throw Bad("friction", $"{where}: 0 or more, and finite");
        }

        private static SurfaceDisplay Display(string? headset, string id) => headset switch
        {
            "builtin" => SurfaceDisplay.Builtin,
            "show" => SurfaceDisplay.Show,
            "hide" => SurfaceDisplay.Hide,
            _ => throw Bad("surface", $"'{id}' has headset '{headset}': builtin, show or hide"),
        };

        private static bool Finite(params double[] xs)
        {
            foreach (double x in xs)
            {
                if (double.IsNaN(x) || double.IsInfinity(x)) return false;
            }

            return true;
        }

        /// <summary><c>library/name</c>, each part 1-64 of a-z, 0-9 and '_' (an asset names a file under the GPU machine's assets folder).</summary>
        private static bool ValidAsset(string asset)
        {
            int slash = asset.IndexOf('/');
            if (slash < 1 || slash != asset.LastIndexOf('/') || slash == asset.Length - 1 || asset.Length > 129) return false;
            foreach (char ch in asset)
            {
                bool ok = (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '_' || ch == '/';
                if (!ok) return false;
            }

            return slash <= 64 && asset.Length - slash - 1 <= 64;
        }

        private static bool ValidName(string? name)
        {
            if (string.IsNullOrEmpty(name) || name!.Length > 64) return false;
            foreach (char ch in name)
            {
                bool ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '.' || ch == '_' || ch == '-';
                if (!ok) return false;
            }

            return true;
        }
    }
}
