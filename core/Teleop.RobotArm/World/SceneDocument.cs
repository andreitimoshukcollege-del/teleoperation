using System;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.World
{
    /// <summary>
    /// The file form of a <see cref="WorldScene"/> (<c>core/Scenes/*.json</c>, docs/adr/0031 amendment 2). Plain public
    /// fields named as the JSON names them, like <see cref="Types.SerialChainProfileDocument"/>: RobotHost reads the file
    /// with System.Text.Json (fields included), the headset with <c>JsonUtility</c>, and Isaac with Python's <c>json</c>.
    /// No nulls and no nested arrays, so <c>JsonUtility</c> can read it. The Isaac-only sections (<c>backdrops</c>,
    /// <c>cameras</c>, <c>lights</c>, docs/adr/0032) are not mirrored here: both C# readers skip fields they don't know.
    /// </summary>
    [Serializable]
    public sealed class SceneDocument
    {
        public string format = "";
        public int version;
        public string name = "";
        public string robotProfile = "";
        public SceneSurfaceDocument[] surfaces = Array.Empty<SceneSurfaceDocument>();
        public SceneObjectDocument[] objects = Array.Empty<SceneObjectDocument>();
        public ScenePlacementDocument placement = new ScenePlacementDocument();

        /// <summary>
        /// docs/adr/0037 decision 3: when set (any maximum above 0), the objects are a pool and each layout chooses which take
        /// part; the rest are parked. Absent (all 0): every object takes part, jittered around its nominal pose.
        /// </summary>
        public ScenePoolDocument pool = new ScenePoolDocument();

        /// <summary>Builds and validates the scene.</summary>
        /// <exception cref="FormatException">The document is not a valid scene; the message starts with what is wrong.</exception>
        public WorldScene ToScene() => WorldScene.Build(this);
    }

    /// <summary>A flat box the world stands on (the bench, the work table), or one only Isaac draws.</summary>
    [Serializable]
    public sealed class SceneSurfaceDocument
    {
        public string id = "";
        public double[] center = Array.Empty<double>();
        public double[] size = Array.Empty<double>();
        public double[] colour = Array.Empty<double>();
        public double staticFriction;
        public double dynamicFriction;
        public bool collides = true;

        /// <summary><c>builtin</c>: the headset's lab already has it (the scene check compares); <c>show</c>: built from the file; <c>hide</c>: Isaac only.</summary>
        public string headset = "show";
    }

    /// <summary>One object standing upright on the work table: a fixed obstacle, or (version 2) a movable body.</summary>
    [Serializable]
    public sealed class SceneObjectDocument
    {
        public int id;
        public string name = "";

        /// <summary>
        /// <c>box</c> (size is its full extents), <c>cylinder</c> (<c>[d, d, h]</c> along its own z), <c>sphere</c> (<c>[d, d, d]</c>),
        /// or <c>mesh</c> (docs/adr/0037 decision 4: a real object's <see cref="asset"/>; size is its axis-aligned box, standing).
        /// </summary>
        public string shape = "";

        /// <summary>A mesh's asset, <c>library/name</c> (<c>ycb/025_mug</c>): Isaac's files for it are under the GPU machine's assets folder. Empty otherwise.</summary>
        public string asset = "";
        public double[] size = Array.Empty<double>();

        /// <summary>The nominal pose, root frame: its centre, and URDF roll/pitch/yaw (roll and pitch 0 in version 1).</summary>
        public double[] position = Array.Empty<double>();
        public double[] rpy = Array.Empty<double>();

        /// <summary>
        /// <c>fixed</c>: a kinematic body the arms collide with, moved only by a placement. <c>movable</c> (version 2): a body with
        /// <see cref="mass"/> that falls and that an arm can push or knock over (docs/adr/0031 amendment 4).
        /// </summary>
        public string kind = "";

        /// <summary>kg: more than 0 for a movable object, absent (0) for a fixed one.</summary>
        public double mass;
        public bool operatorMovable;
        public double staticFriction;
        public double dynamicFriction;
        public double[] colour = Array.Empty<double>();

        /// <summary>The seeded layout's ranges, ±: x and y (m), and yaw (rad).</summary>
        public double[] jitter = Array.Empty<double>();

        /// <summary>
        /// docs/adr/0037 "Raised clutter": desk clutter tall enough for the arm to hit (a roll of tape, tools, a phone, a pen), a
        /// movable object a pool's clutter pass stands anywhere in the placement region (<see cref="ScenePoolDocument.clutterMin"/>
        /// to <see cref="ScenePoolDocument.clutterMax"/> a layout), flat as the file has it. It is not one of the pool's movable
        /// objects (<see cref="ScenePoolDocument.movableMax"/> counts the others).
        /// </summary>
        public bool clutter;

        /// <summary>
        /// docs/adr/0038 decision 3, a mixed pool's (<see cref="ScenePoolDocument.obstacleMax"/>): what a layout does with the object.
        /// <c>obstacle</c>: one of 0-3 things in the arm's reach, anywhere over its reachable workspace (raised on a stand sometimes);
        /// <c>distractor</c>: in the cameras' view, beyond every arm pose's reach; <c>stand</c>: a box fixture an obstacle is raised
        /// on; <c>plank</c>: a box fixture held up on its <see cref="supports"/>, an overhang; <c>support</c>: a plank's leg or base.
        /// Empty: every object of a scene that is not a mixed pool, which draws as it always did.
        /// </summary>
        public string role = "";

        /// <summary>A plank's supports (docs/adr/0038): the ids of the one (a cantilever) or two (a bridge) box fixtures under its ends.</summary>
        public int[] supports = Array.Empty<int>();

        /// <summary>
        /// docs/adr/0038: the resize steps a mixed pool's layout draws this object's size from, uniform, <c>[min, max]</c> of
        /// <see cref="WorldScene.MinResizeStep"/> to <see cref="WorldScene.MaxResizeStep"/> (the factor 1.25 to that power, every
        /// axis), for a box or a cylinder, movable or fixed. Empty: its file size.
        /// </summary>
        public int[] steps = Array.Empty<int>();

        /// <summary>
        /// docs/adr/0038 (the scene's third pass): a plank on two supports that a layout lays across the arm's reach as an overhead bar,
        /// its posts beyond the obstacles' band (<see cref="ScenePoolDocument.overheadShare"/>). False: a plank centred as before.
        /// </summary>
        public bool overhead;
    }

    /// <summary>How many objects of each kind a pool's layout puts on the table, inclusive (docs/adr/0037 decision 3).</summary>
    [Serializable]
    public sealed class ScenePoolDocument
    {
        public int movableMin;
        public int movableMax;
        public int fixedMin;
        public int fixedMax;

        /// <summary>
        /// When both are above 0: each layout also draws a size for every movable object the headset may resize, one of the
        /// resize steps that keeps its largest side between these, m (docs/adr/0037 amendment 2). Absent (0): the file's sizes.
        /// </summary>
        public double sizeMin;
        public double sizeMax;

        /// <summary>
        /// When both are above 0: a layout stands each movable object this far from the robot's base (the root frame's origin,
        /// horizontally), m, where its arm can reach it (docs/adr/0037 amendment 3). Absent (0): anywhere in the region.
        /// </summary>
        public double reachMin;
        public double reachMax;

        /// <summary>
        /// docs/adr/0037 P4: the heaviest a movable object may be, at its drawn size, kg, and still be one the arm knocks over
        /// (measured on the twin). The safety scripts aim collide_with_obstacle only at such objects and collide_and_recover only
        /// at others. Absent (0): no roles.
        /// </summary>
        public double knockMaxMass;

        /// <summary>
        /// docs/adr/0037 amendment 5: where a pool's fixed objects stand instead of the placement region, as flat rectangles
        /// <c>[xMin, xMax, yMin, yMax, ...]</c> in the root frame that hold the object's centre (beside the robot, within the
        /// arm's reach and clear of the cameras' view); its footprint lies on the table or the bench beside it. Empty: the
        /// placement region, as for every object.
        /// </summary>
        public double[] fixedRegions = Array.Empty<double>();

        /// <summary>The robot's own footprint, <c>[xMin, xMax, yMin, yMax]</c>, root frame, that no object's footprint may reach. Empty: none.</summary>
        public double[] robotKeepOut = Array.Empty<double>();

        /// <summary>
        /// When above 0: a layout stands each fixed object (in <see cref="fixedRegions"/>) so its near side, by its footprint's
        /// circle, comes within this distance of the robot's base, m: where the arm reaches it. 0: anywhere in its region.
        /// </summary>
        public double fixedNearMax;

        /// <summary>
        /// docs/adr/0037 amendment 6: the chance, 0-1, that each movable object in reach after a layout's first stands next to
        /// one already placed (a cluster), rather than anywhere in the reach band. 0: every one anywhere, as before.
        /// </summary>
        public double clusterShare;

        /// <summary>
        /// docs/adr/0037 amendment 6: the chance, 0-1, that a movable object with a stable side at least 3 cm tall lies on it (a
        /// can, a parcel) rather than standing upright. 0: every one upright, as before.
        /// </summary>
        public double lyingShare;

        /// <summary>
        /// docs/adr/0037 amendment 6: rectangles <c>[xMin, xMax, yMin, yMax, ...]</c>, root frame, outside the placement region,
        /// that hold the centre of a movable object standing by the bench's edge, out of the arm's reach; its footprint lies on
        /// the table or the bench.
        /// </summary>
        public double[] edgeRegions = Array.Empty<double>();

        /// <summary>docs/adr/0037 amendment 6: the chance, 0-1, that a layout also stands one more movable object in <see cref="edgeRegions"/>.</summary>
        public double edgeShare;

        /// <summary>
        /// docs/adr/0037 "Raised clutter": how many of the scene's clutter objects (<see cref="SceneObjectDocument.clutter"/>) a
        /// layout stands on the table, inclusive, after every other object, anywhere in the placement region (no reach band).
        /// Absent (0): none, and the layouts are exactly those drawn before clutter.
        /// </summary>
        public int clutterMin;
        public int clutterMax;

        /// <summary>
        /// docs/adr/0038 decision 3: a mixed pool. Each layout stands <see cref="obstacleMin"/> to <see cref="obstacleMax"/> obstacles
        /// (an object of role <c>obstacle</c>, maybe raised on a stand, or a plank on its supports), each centred uniformly by area
        /// over the band <see cref="reachMin"/>-<see cref="reachMax"/> within <see cref="reachBearing"/> of straight ahead, then
        /// <see cref="distractorMin"/> to <see cref="distractorMax"/> distractors in <see cref="distractorRegions"/>. Every object has
        /// a role, and the movable, fixed and clutter counts are 0. Absent (0): not a mixed pool.
        /// </summary>
        public int obstacleMin;
        public int obstacleMax;

        /// <summary>docs/adr/0038: how far either side of straight ahead (+x) an obstacle's centre may stand, rad (the base's yaw range).</summary>
        public double reachBearing;

        /// <summary>docs/adr/0038: the chance, 0-1, that a movable obstacle is raised on a free stand that holds it.</summary>
        public double standShare;

        /// <summary>docs/adr/0038: the chance, 0-1, that an obstacle is a plank on its supports, when one is free.</summary>
        public double rigShare;

        /// <summary>
        /// docs/adr/0038 (the scene's second pass): the chance, 0-1, that an obstacle that is not a plank is one of the tall thin ones
        /// (<see cref="WorldScene.TallHeight"/> or taller at its file size: a rod, a dowel, a post), when one is free. 0: every obstacle
        /// object uniform among those free, as before.
        /// </summary>
        public double tallShare;

        /// <summary>
        /// docs/adr/0038 (the scene's second pass): the arm's working band, inside the obstacles' band. Tall obstacles and planks are centred
        /// <see cref="workMin"/>-<see cref="workMax"/> m from the base's axis within <see cref="workBearing"/> rad of straight ahead, where the
        /// arm reaches them at every height, not at the band's edges. All 0: the obstacles' band, as before.
        /// </summary>
        public double workMin;
        public double workMax;
        public double workBearing;

        /// <summary>
        /// docs/adr/0038 (the scene's second pass): when set, a stand holds an object only when its top's sides are each between these
        /// fractions of the object's narrower side, so the object overhangs it all round and is met before its stand; the object stands
        /// upright on it. Both 0: a stand's top holds the object's whole footprint, as before.
        /// </summary>
        public double standTopMin;
        public double standTopMax;

        /// <summary>
        /// docs/adr/0038 (the scene's second pass): when set, each rig draws the height of its plank's underside uniform between these, m
        /// (within the heights its supports' steps give), and takes the support step nearest it. Both 0: its sizes uniform among the pairs
        /// of steps that leave the gripper room under it, as before.
        /// </summary>
        public double rigUnderMin;
        public double rigUnderMax;

        /// <summary>
        /// docs/adr/0038 (the scene's third pass): the chance, 0-1, that an obstacle is an overhead bar (a plank marked
        /// <see cref="SceneObjectDocument.overhead"/>), when one is free; drawn before <see cref="rigShare"/>. 0: no bars.
        /// </summary>
        public double overheadShare;

        /// <summary>
        /// docs/adr/0038 (the scene's third pass): a box <c>[xMin, xMax, yMin, yMax, zMin, zMax]</c>, root frame, where the arm stands
        /// between takes (home, and the tails that end in front). A layout stands nothing that comes within the clearance of it. Empty:
        /// no such box, as before.
        /// </summary>
        public double[] armRest = Array.Empty<double>();

        /// <summary>docs/adr/0038: how many distractors a mixed layout stands, inclusive.</summary>
        public int distractorMin;
        public int distractorMax;

        /// <summary>
        /// docs/adr/0038: rectangles <c>[xMin, xMax, yMin, yMax, ...]</c>, root frame, that hold a distractor's centre; its footprint
        /// lies on the table or the bench, and no nearer the robot's base than <see cref="distractorReach"/>.
        /// </summary>
        public double[] distractorRegions = Array.Empty<double>();

        /// <summary>
        /// docs/adr/0038: the nearest a distractor's footprint may come to the base's yaw axis, horizontally, m: beyond the farthest
        /// any arm pose reaches (robot/isaac/reach_envelope.py), so no accepted pose touches one.
        /// </summary>
        public double distractorReach;
    }

    /// <summary>Where objects may stand: every footprint lies inside this region and on the table, this far apart.</summary>
    [Serializable]
    public sealed class ScenePlacementDocument
    {
        public double xMin;
        public double xMax;
        public double yMin;
        public double yMax;
        public double clearance;
    }
}
