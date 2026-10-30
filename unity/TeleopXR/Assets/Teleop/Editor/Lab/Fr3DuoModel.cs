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
    /// <item><b>Geometry: the arms' visual meshes, the mount's and head's collision shapes</b> (the user's choice,
    /// docs/adr/0029 amendment). The arms and hands are Franka's visual COLLADA meshes, about 255k triangles per arm,
    /// imported by Unity and aligned by <see cref="AddVisuals"/>. The mount and head stay as their collision STLs (a few
    /// hundred triangles each): they are decorative, and 395k triangles as visuals. Each STL becomes a <see cref="Mesh"/>
    /// asset in <see cref="MeshFolder"/>, rewritten in place so its GUID stays stable.</item>
    /// </list>
    /// Regenerated, never hand-edited: <see cref="XrFr3SceneBuilder"/> calls it.
    /// </summary>
    internal static class Fr3DuoModel
    {
        /// <summary>From the project folder (unity/TeleopXR) to the vendored URDF.</summary>
        public const string UrdfPath = "../../robot/franka/fr3_duo/urdf/fr3_duo.urdf";
        public const string MeshFolder = "Assets/Teleop/Lab/FR3";

        /// <summary>The arms' visual COLLADA meshes, copied in so Unity imports them (LFS, as every .dae).</summary>
        public const string VisualFolder = "Assets/Teleop/Lab/FR3/Visual";
        public const string ProfileResource = "fr3_duo_profile";

        private sealed class UrdfJoint
        {
            public string Name, Type, Parent, Child;
            public RigidTransform Origin;
            public double[] Axis = { 1, 0, 0 };
        }

        private sealed class UrdfVisual
        {
            public RigidTransform Origin;
            public string Mesh;
        }

        /// <summary>One arm link drawn with its visual mesh: the link, its collision shapes (to align against, then removed) and its visuals.</summary>
        private sealed class VisualLink
        {
            public string Name;
            public Transform Link;
            public Transform Collision;
            public List<UrdfVisual> Visuals;
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
            var visuals = robot.Elements("link").ToDictionary(l => (string)l.Attribute("name"), l => l.Elements("visual").Select(ParseVisual).Where(v => v.Mesh != null).ToList());
            var children = new HashSet<string>(joints.Select(j => j.Child));
            string rootLink = collisions.Keys.Single(l => !children.Contains(l));

            SerialChainProfile profile = LoadProfile();
            var rigJoints = new List<SerialChainRigJoint>();
            var meshes = new Dictionary<string, Mesh>();
            XrProjectSetup.EnsureFolder(MeshFolder);

            var root = new GameObject(name).transform;
            root.SetParent(parent, false);
            var visualLinks = new List<VisualLink>();
            var pending = new Stack<(string Link, Transform At)>();
            pending.Push((rootLink, root));
            while (pending.Count > 0)
            {
                (string link, Transform at) = pending.Pop();
                bool armLink = link.Contains("fr3v2");
                // The arms are drawn with their visual meshes (the user's choice, docs/adr/0029 amendment); their collision
                // shapes are built too, under one container, to align the visuals against, and then removed. The mount and
                // the head keep their collision shapes: decorative, and 395k triangles as visuals.
                Transform geometryParent = at;
                if (armLink && visuals[link].Count > 0)
                {
                    geometryParent = new GameObject("collision").transform;
                    geometryParent.SetParent(at, false);
                    visualLinks.Add(new VisualLink { Name = link, Link = at, Collision = geometryParent, Visuals = visuals[link] });
                }

                foreach (UrdfCollision c in collisions[link])
                {
                    AddGeometry(geometryParent, c, urdfDir, meshes, armLink ? links : body);
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

            AddVisuals(visualLinks, urdfDir, links);

            var rig = root.gameObject.AddComponent<SerialChainArmRig>();
            rig.Configure(rigJoints.ToArray());
            AssetDatabase.SaveAssets();
            return rig;
        }

        /// <summary>
        /// Draws each arm link with its visual mesh, imported by Unity from the vendored COLLADA file. A COLLADA file says
        /// its own up axis and unit, and how Unity's importer turns that into Unity's frame is the importer's business, not
        /// <see cref="CoordConversion"/>'s. So the turn is found, not assumed: of the 24 axis-aligned rotations (and the
        /// unit scales a file could be off by), the one that best lays every link's visual bounds over its collision shapes
        /// (which <see cref="CoordConversion"/> placed, and the FK check proved) is applied to all of them, and the build
        /// fails if even the best leaves a link more than 3 cm or 25 % off.
        /// </summary>
        private static void AddVisuals(List<VisualLink> links, string urdfDir, Material material)
        {
            if (links.Count == 0) return;
            XrProjectSetup.EnsureFolder(VisualFolder);
            var prefabs = new Dictionary<string, GameObject>();
            var instances = new List<(VisualLink Link, Transform Wrapper, Quaternion Origin)>();
            foreach (VisualLink link in links)
            {
                foreach (UrdfVisual v in link.Visuals)
                {
                    string file = Path.GetFullPath(Path.Combine(urdfDir, v.Mesh));
                    if (!prefabs.TryGetValue(file, out GameObject prefab))
                    {
                        prefab = ImportDae(file);
                        prefabs[file] = prefab;
                    }

                    ToUnityPose(v.Origin, out Vector3 position, out Quaternion rotation);
                    var wrapper = new GameObject("visual").transform;
                    wrapper.SetParent(link.Link, false);
                    wrapper.localPosition = position;
                    wrapper.localRotation = rotation;
                    var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab, wrapper);
                    PrefabUtility.UnpackPrefabInstance(instance, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
                    foreach (Renderer r in instance.GetComponentsInChildren<Renderer>(true))
                    {
                        r.sharedMaterials = Enumerable.Repeat(material, r.sharedMaterials.Length).ToArray();
                    }

                    instances.Add((link, wrapper, rotation));
                }
            }

            Quaternion[] turns = AxisRotations();
            float[] scales = { 1f, 0.01f, 0.001f, 100f };
            double bestScore = double.PositiveInfinity;
            Quaternion bestTurn = Quaternion.identity;
            float bestScale = 1f;
            foreach (Quaternion turn in turns)
            {
                foreach (float scale in scales)
                {
                    double score = 0;
                    foreach ((VisualLink link, Transform wrapper, Quaternion origin) in instances)
                    {
                        wrapper.localRotation = origin * turn;
                        wrapper.localScale = Vector3.one * scale;
                        score += Mismatch(BoundsIn(link.Link, wrapper), BoundsIn(link.Link, link.Collision), out _, out _);
                    }

                    if (score < bestScore)
                    {
                        bestScore = score;
                        bestTurn = turn;
                        bestScale = scale;
                    }
                }
            }

            double worstCentre = 0, worstSize = 0;
            foreach ((VisualLink link, Transform wrapper, Quaternion origin) in instances)
            {
                wrapper.localRotation = origin * bestTurn;
                wrapper.localScale = Vector3.one * bestScale;
                Mismatch(BoundsIn(link.Link, wrapper), BoundsIn(link.Link, link.Collision), out double centre, out double size);
                worstCentre = Math.Max(worstCentre, centre);
                worstSize = Math.Max(worstSize, size);
            }

            Debug.Log($"[fr3-scene] visual meshes: importer turn {bestTurn.eulerAngles}, scale {bestScale}; worst link {worstCentre * 1000:0.0} mm off-centre, {worstSize * 100:0.0} % off in size from its collision shape");
            if (worstCentre > 0.03 || worstSize > 0.25)
            {
                throw new InvalidOperationException($"the visual meshes do not line up with the collision shapes (worst {worstCentre * 1000:0} mm, {worstSize * 100:0} %): the COLLADA import turned them some way this builder cannot undo");
            }

            foreach (VisualLink link in links) Object.DestroyImmediate(link.Collision.gameObject);
        }

        /// <summary>A COLLADA file copied into <see cref="VisualFolder"/> (when changed) and imported: no animation, cameras, lights or materials.</summary>
        private static GameObject ImportDae(string file)
        {
            // robots/fr3v2/visual/link3.dae -> fr3v2-link3.dae
            string dir = Path.GetFileName(Path.GetDirectoryName(Path.GetDirectoryName(file)));
            string path = $"{VisualFolder}/{dir}-{Path.GetFileName(file)}";
            byte[] bytes = File.ReadAllBytes(file);
            if (!File.Exists(path) || !File.ReadAllBytes(path).SequenceEqual(bytes))
            {
                File.WriteAllBytes(path, bytes);
                AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceSynchronousImport);
            }

            var importer = (ModelImporter)AssetImporter.GetAtPath(path);
            bool changed = importer.importAnimation || importer.importCameras || importer.importLights || importer.materialImportMode != ModelImporterMaterialImportMode.None || importer.isReadable;
            if (changed)
            {
                importer.importAnimation = false;
                importer.importCameras = false;
                importer.importLights = false;
                importer.materialImportMode = ModelImporterMaterialImportMode.None;
                importer.isReadable = false;
                importer.SaveAndReimport();
            }

            return AssetDatabase.LoadAssetAtPath<GameObject>(path) ?? throw new InvalidOperationException($"Unity did not import {path} as a model");
        }

        /// <summary>The bounds of every mesh under <paramref name="of"/>, in <paramref name="frame"/>'s local space.</summary>
        private static Bounds BoundsIn(Transform frame, Transform of)
        {
            bool any = false;
            var b = new Bounds();
            Matrix4x4 toFrame = frame.worldToLocalMatrix;
            foreach (MeshFilter f in of.GetComponentsInChildren<MeshFilter>(true))
            {
                if (f.sharedMesh == null) continue;
                Bounds m = f.sharedMesh.bounds;
                Matrix4x4 toLocal = toFrame * f.transform.localToWorldMatrix;
                for (int c = 0; c < 8; c++)
                {
                    var corner = new Vector3((c & 1) == 0 ? m.min.x : m.max.x, (c & 2) == 0 ? m.min.y : m.max.y, (c & 4) == 0 ? m.min.z : m.max.z);
                    Vector3 p = toLocal.MultiplyPoint3x4(corner);
                    if (!any) { b = new Bounds(p, Vector3.zero); any = true; }
                    else b.Encapsulate(p);
                }
            }

            return b;
        }

        /// <summary>How far apart two boxes are: centre distance (m) plus size difference relative to the reference's size.</summary>
        private static double Mismatch(Bounds visual, Bounds reference, out double centre, out double size)
        {
            centre = Vector3.Distance(visual.center, reference.center);
            float r = Mathf.Max(reference.size.magnitude, 1e-4f);
            size = (visual.size - reference.size).magnitude / r;
            return centre + 0.1 * size;
        }

        /// <summary>The 24 rotations that take axes to axes.</summary>
        private static Quaternion[] AxisRotations()
        {
            var result = new List<Quaternion>();
            Vector3[] axes = { Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back };
            foreach (Vector3 forward in axes)
            {
                foreach (Vector3 up in axes)
                {
                    if (Mathf.Abs(Vector3.Dot(forward, up)) > 0.5f) continue;
                    result.Add(Quaternion.LookRotation(forward, up));
                }
            }

            return result.ToArray();
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

        private static UrdfVisual ParseVisual(XElement e)
        {
            XElement shape = e.Element("geometry")?.Elements().FirstOrDefault();
            return new UrdfVisual
            {
                Origin = Origin(e.Element("origin")),
                Mesh = shape != null && shape.Name.LocalName == "mesh" ? (string)shape.Attribute("filename") : null,
            };
        }

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
