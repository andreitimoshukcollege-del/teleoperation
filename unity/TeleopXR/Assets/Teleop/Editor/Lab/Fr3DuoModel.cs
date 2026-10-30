using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using Teleop.Bridge;
using Teleop.RobotArm.Types;
using UnityEditor;
using UnityEngine;
using CoreQuat = System.Numerics.Quaternion;
using CoreVec = System.Numerics.Vector3;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// The FR3 Duo's twin (docs/adr/0029 decision 6), built from the vendored URDF
    /// (<c>robot/franka/fr3_duo/urdf/fr3_duo.urdf</c>), the file <c>core/RobotProfiles/fr3-duo.json</c> was generated
    /// from, so the twin's frames are the profile's.
    /// <list type="bullet">
    /// <item><b>Frames:</b> one GameObject per URDF link, named after it, under its parent link at the joint's origin
    /// (<see cref="CoordConversion"/>, the only conversion). Each moving joint becomes a
    /// <see cref="SerialChainRigJoint"/> with the motor id the profile gives it (a gripper's two fingers share
    /// one).</item>
    /// <item><b>Geometry: the collision meshes, not the visual ones.</b> The visual COLLADA files are 59 MB of dense
    /// CAD, too heavy for a mobile headset twin; the collision STLs are a few hundred triangles each and boxes for the
    /// fingers, and they are the shapes the collision model was fitted to (<c>fit-capsules</c>). Each STL becomes a
    /// <see cref="Mesh"/> asset in <see cref="MeshFolder"/>, rewritten in place so its GUID stays stable.</item>
    /// </list>
    /// Regenerated, never hand-edited: <see cref="XrFr3SceneBuilder"/> calls it.
    /// </summary>
    internal static class Fr3DuoModel
    {
        /// <summary>From the project folder (unity/TeleopXR) to the vendored URDF.</summary>
        public const string UrdfPath = "../../robot/franka/fr3_duo/urdf/fr3_duo.urdf";
        public const string MeshFolder = "Assets/Teleop/Lab/FR3";
        public const string ProfileResource = "fr3_duo_profile";

        private sealed class UrdfJoint
        {
            public string Name, Type, Parent, Child;
            public RigidTransform Origin;
            public double[] Axis = { 1, 0, 0 };
        }

        private sealed class UrdfCollision
        {
            public RigidTransform Origin;
            public string Mesh;
            public double[] Box;
        }

        /// <summary>Builds the twin under <paramref name="parent"/>: its root is the URDF's <c>base</c> link.</summary>
        public static SerialChainArmRig Build(Transform parent, string name, Material links, Material body)
        {
            string urdfFile = Path.GetFullPath(UrdfPath);
            if (!File.Exists(urdfFile)) throw new FileNotFoundException($"the FR3 Duo's URDF is not at {urdfFile}");
            XElement robot = XDocument.Load(urdfFile).Root ?? throw new InvalidOperationException("empty URDF");
            string urdfDir = Path.GetDirectoryName(urdfFile);

            var joints = robot.Elements("joint").Select(ParseJoint).ToList();
            var collisions = robot.Elements("link").ToDictionary(l => (string)l.Attribute("name"), l => l.Elements("collision").Select(ParseCollision).ToList());
            var children = new HashSet<string>(joints.Select(j => j.Child));
            string rootLink = collisions.Keys.Single(l => !children.Contains(l));

            SerialChainProfile profile = LoadProfile();
            var rigJoints = new List<SerialChainRigJoint>();
            var meshes = new Dictionary<string, Mesh>();
            XrProjectSetup.EnsureFolder(MeshFolder);

            var root = new GameObject(name).transform;
            root.SetParent(parent, false);
            var pending = new Stack<(string Link, Transform At)>();
            pending.Push((rootLink, root));
            while (pending.Count > 0)
            {
                (string link, Transform at) = pending.Pop();
                bool armLink = link.Contains("fr3v2");
                foreach (UrdfCollision c in collisions[link])
                {
                    AddGeometry(at, c, urdfDir, meshes, armLink ? links : body);
                }

                foreach (UrdfJoint j in joints.Where(j => j.Parent == link))
                {
                    var child = new GameObject(j.Child).transform;
                    child.SetParent(at, false);
                    ToUnityPose(j.Origin, out Vector3 position, out Quaternion rotation);
                    child.localPosition = position;
                    child.localRotation = rotation;
                    if (j.Type == "revolute" || j.Type == "continuous" || j.Type == "prismatic")
                    {
                        rigJoints.Add(RigJoint(profile, j, child, position, rotation));
                    }

                    pending.Push((j.Child, child));
                }
            }

            var rig = root.gameObject.AddComponent<SerialChainArmRig>();
            rig.Configure(rigJoints.ToArray());
            AssetDatabase.SaveAssets();
            return rig;
        }

        /// <summary>The profile the runtime loads (Bridge's Resources copy, held to core/RobotProfiles by a test).</summary>
        public static SerialChainProfile LoadProfile()
        {
            TextAsset asset = Resources.Load<TextAsset>(ProfileResource) ?? throw new InvalidOperationException($"Resources/{ProfileResource}.json is missing");
            return JsonUtility.FromJson<SerialChainProfileDocument>(asset.text).ToProfile();
        }

        private static SerialChainRigJoint RigJoint(SerialChainProfile profile, UrdfJoint j, Transform t, Vector3 position, Quaternion rotation)
        {
            int motor = -1;
            float scale = 1f;
            foreach (ChainArm arm in profile.Arms)
            {
                foreach (ChainJoint cj in arm.Joints)
                {
                    if (cj.Name == j.Name) motor = cj.MotorId;
                }

                // Both fingers of a hand: the first is the gripper's joint, the second mimics it (the URDF's <mimic>).
                if (arm.HasGripper && j.Type == "prismatic" && SameHand(arm.Gripper.Name, j.Name))
                {
                    motor = arm.Gripper.MotorId;
                    scale = (float)arm.Gripper.FingerTravel;
                }
            }

            if (motor < 0) throw new InvalidOperationException($"URDF joint '{j.Name}' moves, but the profile has no motor for it");
            return new SerialChainRigJoint
            {
                name = j.Name, motorId = motor, transform = t, prismatic = j.Type == "prismatic",
                axis = new Vector3((float)j.Axis[0], (float)j.Axis[1], (float)j.Axis[2]), scale = scale,
                restLocalPosition = position, restLocalRotation = rotation,
            };
        }

        /// <summary><c>left_fr3v2_finger_joint1</c> and <c>left_fr3v2_finger_joint2</c> are the same hand.</summary>
        private static bool SameHand(string gripperJoint, string joint)
        {
            int cut = gripperJoint.LastIndexOf("finger_joint", StringComparison.Ordinal);
            return cut > 0 && joint.StartsWith(gripperJoint.Substring(0, cut), StringComparison.Ordinal);
        }

        private static void AddGeometry(Transform at, UrdfCollision c, string urdfDir, Dictionary<string, Mesh> meshes, Material material)
        {
            ToUnityPose(c.Origin, out Vector3 position, out Quaternion rotation);
            GameObject shape;
            if (c.Box != null)
            {
                shape = GameObject.CreatePrimitive(PrimitiveType.Cube);
                Object.DestroyImmediate(shape.GetComponent<Collider>()); // the twin has no colliders
                // A ROS box (x, y, z) is a Unity box (y, z, x): CoordConversion's axes, unsigned.
                Vector3 size = new CoreVec((float)c.Box[0], (float)c.Box[1], (float)c.Box[2]).ToUnity();
                shape.transform.localScale = new Vector3(Mathf.Abs(size.x), Mathf.Abs(size.y), Mathf.Abs(size.z));
            }
            else
            {
                string file = Path.GetFullPath(Path.Combine(urdfDir, c.Mesh));
                if (!meshes.TryGetValue(file, out Mesh mesh))
                {
                    mesh = StoreStl(file);
                    meshes[file] = mesh;
                }

                shape = new GameObject(mesh.name);
                shape.AddComponent<MeshFilter>().sharedMesh = mesh;
                shape.AddComponent<MeshRenderer>();
            }

            shape.name = c.Box != null ? "box" : shape.name;
            shape.transform.SetParent(at, false);
            shape.transform.localPosition = position;
            shape.transform.localRotation = rotation;
            shape.GetComponent<Renderer>().sharedMaterial = material;
        }

        /// <summary>A binary STL (metres, the link's frame) as a Mesh asset, its vertices through CoordConversion.</summary>
        private static Mesh StoreStl(string file)
        {
            byte[] bytes = File.ReadAllBytes(file);
            int triangles = BitConverter.ToInt32(bytes, 80);
            if (bytes.Length != 84 + 50 * triangles) throw new InvalidDataException($"{file} is not a binary STL");
            var vertices = new Vector3[triangles * 3];
            var indices = new int[triangles * 3];
            for (int t = 0; t < triangles; t++)
            {
                int at = 84 + 50 * t + 12; // past the facet normal
                for (int v = 0; v < 3; v++)
                {
                    var p = new CoreVec(BitConverter.ToSingle(bytes, at), BitConverter.ToSingle(bytes, at + 4), BitConverter.ToSingle(bytes, at + 8));
                    vertices[t * 3 + v] = p.ToUnity();
                    at += 12;
                }

                // The conversion mirrors handedness, so each triangle's winding is reversed to keep it facing out.
                indices[t * 3] = t * 3;
                indices[t * 3 + 1] = t * 3 + 2;
                indices[t * 3 + 2] = t * 3 + 1;
            }

            // robots/fr3v2/collision/link3.stl -> fr3v2-link3
            string dir = Path.GetFileName(Path.GetDirectoryName(Path.GetDirectoryName(file)));
            string name = $"{dir}-{Path.GetFileNameWithoutExtension(file)}";
            string path = $"{MeshFolder}/{name}.asset";
            Mesh mesh = AssetDatabase.LoadAssetAtPath<Mesh>(path);
            bool created = mesh == null;
            if (created) mesh = new Mesh();
            mesh.Clear();
            mesh.name = name;
            mesh.vertices = vertices;
            mesh.triangles = indices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            if (created) AssetDatabase.CreateAsset(mesh, path);
            else EditorUtility.SetDirty(mesh);
            return mesh;
        }

        private static void ToUnityPose(in RigidTransform t, out Vector3 position, out Quaternion rotation)
        {
            position = new CoreVec((float)t.X, (float)t.Y, (float)t.Z).ToUnity();
            rotation = new CoreQuat((float)t.Qx, (float)t.Qy, (float)t.Qz, (float)t.Qw).ToUnity();
        }

        private static UrdfJoint ParseJoint(XElement e) => new UrdfJoint
        {
            Name = (string)e.Attribute("name"),
            Type = (string)e.Attribute("type"),
            Parent = (string)e.Element("parent")?.Attribute("link"),
            Child = (string)e.Element("child")?.Attribute("link"),
            Origin = Origin(e.Element("origin")),
            Axis = Numbers((string)e.Element("axis")?.Attribute("xyz"), 3) ?? new double[] { 1, 0, 0 },
        };

        private static UrdfCollision ParseCollision(XElement e)
        {
            XElement geometry = e.Element("geometry") ?? throw new InvalidDataException("a collision without geometry");
            XElement shape = geometry.Elements().First();
            var c = new UrdfCollision { Origin = Origin(e.Element("origin")) };
            if (shape.Name.LocalName == "mesh") c.Mesh = (string)shape.Attribute("filename");
            else if (shape.Name.LocalName == "box") c.Box = Numbers((string)shape.Attribute("size"), 3);
            else throw new InvalidDataException($"collision geometry '{shape.Name.LocalName}' is not drawn by the twin");
            return c;
        }

        private static RigidTransform Origin(XElement origin)
        {
            double[] xyz = Numbers((string)origin?.Attribute("xyz"), 3) ?? new double[3];
            double[] rpy = Numbers((string)origin?.Attribute("rpy"), 3) ?? new double[3];
            return RigidTransform.FromXyzRpy(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
        }

        private static double[] Numbers(string text, int count)
        {
            if (string.IsNullOrWhiteSpace(text)) return null;
            double[] v = text.Split((char[])null, StringSplitOptions.RemoveEmptyEntries)
                .Select(s => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture)).ToArray();
            if (v.Length != count) throw new InvalidDataException($"'{text}' is not {count} numbers");
            return v;
        }
    }
}
