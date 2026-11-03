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

    /// <summary>One object: a fixed obstacle standing upright on the work table (version 1).</summary>
    [Serializable]
    public sealed class SceneObjectDocument
    {
        public int id;
        public string name = "";

        /// <summary><c>box</c> (size is its full extents), <c>cylinder</c> (<c>[d, d, h]</c> along its own z) or <c>sphere</c> (<c>[d, d, d]</c>).</summary>
        public string shape = "";
        public double[] size = Array.Empty<double>();

        /// <summary>The nominal pose, root frame: its centre, and URDF roll/pitch/yaw (roll and pitch 0 in version 1).</summary>
        public double[] position = Array.Empty<double>();
        public double[] rpy = Array.Empty<double>();

        /// <summary><c>fixed</c>: a kinematic body the arms collide with, moved only by a placement. <c>movable</c> is reserved.</summary>
        public string kind = "";
        public bool operatorMovable;
        public double staticFriction;
        public double dynamicFriction;
        public double[] colour = Array.Empty<double>();

        /// <summary>The seeded layout's ranges, ±: x and y (m), and yaw (rad).</summary>
        public double[] jitter = Array.Empty<double>();
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
