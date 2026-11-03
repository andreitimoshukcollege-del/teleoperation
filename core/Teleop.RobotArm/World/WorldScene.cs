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
    }

    /// <summary>Who draws a surface: the headset's lab already has it, the headset builds it from the file, or only Isaac.</summary>
    public enum SurfaceDisplay
    {
        Builtin,
        Show,
        Hide,
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
            bool movable, double mass, bool operatorMovable, double staticFriction, double dynamicFriction, float r, float g, float b, double jx, double jy, double jyaw)
        {
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
            FootprintRadius = shape == SceneShape.Box ? 0.5 * Math.Sqrt(sx * sx + sy * sy) : 0.5 * sx;
            RigidTransform turn = RigidTransform.Rotation(0, 0, 1, yaw);
            Nominal = new RigidTransform(x, y, z, turn.Qw, turn.Qx, turn.Qy, turn.Qz);
        }

        public byte Id { get; }
        public string Name { get; }
        public SceneShape Shape { get; }

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
        public const int MaxObjects = 24;

        /// <summary>The heaviest movable object, kg: anything heavier is not something the arms are meant to knock over.</summary>
        public const double MaxMass = 20.0;

        /// <summary>How far a surface or a resting object may be from where it must be, m.</summary>
        public const double Tolerance = 0.001;

        private WorldScene(string name, string robotProfile, SceneSurface[] surfaces, SceneSurface table, SceneSurface? bench,
            SceneObject[] objects, double xMin, double xMax, double yMin, double yMax, double clearance)
        {
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

        /// <summary>The object's index in <see cref="Objects"/>, or -1.</summary>
        public int IndexOf(byte id)
        {
            for (int i = 0; i < Objects.Length; i++)
            {
                if (Objects[i].Id == id) return i;
            }

            return -1;
        }

        /// <summary>Whether object <paramref name="index"/>'s footprint, centred at (x, y), lies on the table and inside the placement region.</summary>
        public bool FitsOnTable(int index, double x, double y)
        {
            double r = Objects[index].FootprintRadius;
            return x - r >= Math.Max(Table.XMin, RegionXMin) - 1e-9 && x + r <= Math.Min(Table.XMax, RegionXMax) + 1e-9 &&
                   y - r >= Math.Max(Table.YMin, RegionYMin) - 1e-9 && y + r <= Math.Min(Table.YMax, RegionYMax) + 1e-9;
        }

        /// <summary>Whether two objects' footprints, centred where given, come closer than <see cref="Clearance"/>.</summary>
        public bool Overlaps(int a, double ax, double ay, int b, double bx, double by)
        {
            double reach = Objects[a].FootprintRadius + Objects[b].FootprintRadius + Clearance;
            double dx = ax - bx, dy = ay - by;
            return dx * dx + dy * dy < reach * reach;
        }

        /// <summary>Object <paramref name="index"/> standing upright on the table at (x, y), turned <paramref name="yaw"/> about the vertical.</summary>
        public RigidTransform PoseAt(int index, double x, double y, double yaw)
        {
            RigidTransform turn = RigidTransform.Rotation(0, 0, 1, yaw);
            return new RigidTransform(x, y, Table.Top + Objects[index].SizeZ / 2, turn.Qw, turn.Qx, turn.Qy, turn.Qz);
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
                    _ => throw Bad("shape", $"'{o.name}' has shape '{o.shape}': box, cylinder or sphere"),
                };
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

                double[] pos = Vector(o.position, 3, "pose", $"'{o.name}' position");
                double[] rpy = Vector(o.rpy, 3, "pose", $"'{o.name}' rpy");
                if (Math.Abs(rpy[0]) > 1e-9 || Math.Abs(rpy[1]) > 1e-9) throw Bad("pose", $"'{o.name}' must stand upright (roll and pitch 0)");
                if (Math.Abs(pos[2] - (table.Top + z[2] / 2)) > Tolerance) throw Bad("rests", $"'{o.name}' must rest on the table: its centre at z = {table.Top + z[2] / 2:0.####}, not {pos[2]:0.####}");
                double[] col = Colour(o.colour, $"object '{o.name}'");
                Friction(o.staticFriction, o.dynamicFriction, $"object '{o.name}'");
                double[] j = Vector(o.jitter, 3, "jitter", $"'{o.name}'");
                if (!(j[0] >= 0 && j[1] >= 0 && j[2] >= 0)) throw Bad("jitter", $"'{o.name}' has a negative range");
                objects[i] = new SceneObject((byte)o.id, o.name, shape, z[0], z[1], z[2], pos[0], pos[1], pos[2], rpy[2], movable, o.mass, o.operatorMovable,
                    o.staticFriction, o.dynamicFriction, (float)col[0], (float)col[1], (float)col[2], j[0], j[1], j[2]);
            }

            var scene = new WorldScene(d.name, d.robotProfile, surfaces, table, bench, objects, p.xMin, p.xMax, p.yMin, p.yMax, p.clearance);
            for (int i = 0; i < objects.Length; i++)
            {
                SceneObject o = objects[i];
                for (int sx = -1; sx <= 1; sx++)
                {
                    for (int sy = -1; sy <= 1; sy++)
                    {
                        if (!scene.FitsOnTable(i, o.NominalX + sx * o.JitterX, o.NominalY + sy * o.JitterY))
                        {
                            throw Bad("fits", $"'{o.Name}' leaves the table or the placement region at its nominal pose or the edge of its jitter");
                        }
                    }
                }

                for (int k = 0; k < i; k++)
                {
                    if (scene.Overlaps(i, o.NominalX, o.NominalY, k, objects[k].NominalX, objects[k].NominalY))
                    {
                        throw Bad("overlap", $"'{o.Name}' and '{objects[k].Name}' are closer than the clearance at their nominal poses");
                    }
                }
            }

            return scene;
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
