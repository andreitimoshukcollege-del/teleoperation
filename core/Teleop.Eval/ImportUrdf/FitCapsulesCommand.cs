using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text.Json;
using System.Xml.Linq;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.ImportUrdf
{
    /// <summary>
    /// The <c>fit-capsules</c> verb (docs/adr/0026 amendment). It fits a serial-chain robot's collision model to its URDF's
    /// collision geometry, headlessly. The geometry is binary STL meshes and boxes.
    /// <list type="bullet">
    /// <item><b>The capsules:</b> one per URDF link that has collision geometry. Its segment runs along the vertices'
    /// principal axis. Its radius is the largest distance from the segment to any vertex, so every vertex lies inside
    /// it.</item>
    /// <item><b>Where a capsule lives:</b> in the frame of the arm joint that moves its link, the arm's base for link 0,
    /// or the robot's root for the fixed body. A gripper's fingers are taken fully open, in the arm's last joint
    /// frame.</item>
    /// <item><b>The disabled pairs:</b> adjacent links, an arm's link 0 with the body and with the other arm's link 0
    /// (all on the mount), and any pair the fit makes touch at the ready pose.</item>
    /// <item><b>Byte-stable output:</b> numbers are rounded to a micrometre, so a re-fit is the same bytes on every
    /// platform. <c>FitCapsulesCommandTests</c> requires that of the committed file.</item>
    /// </list>
    /// </summary>
    public static class FitCapsulesCommand
    {
        private const string Usage =
            "usage: fit-capsules --urdf <file> --profile <serial-chain profile> --out <file> [--margin 0.01]\n" +
            "  Fits one capsule per URDF link to its collision meshes (binary STL) and boxes, and writes the robot's collision\n" +
            "  model (docs/adr/0026 amendment): capsules in the frames that move them, the pairs never checked, the margin.";

        public static int Run(string[] args) => Run(args, Console.Out, Console.Error);

        internal static int Run(string[] args, TextWriter output, TextWriter error)
        {
            string? urdf = null, profilePath = null, outPath = null;
            double margin = 0.01;
            for (int i = 1; i < args.Length; i++)
            {
                if (i + 1 >= args.Length)
                {
                    error.WriteLine($"{args[i]} needs a value");
                    error.WriteLine(Usage);
                    return 64;
                }

                string value = args[++i];
                switch (args[i - 1])
                {
                    case "--urdf": urdf = value; break;
                    case "--profile": profilePath = value; break;
                    case "--out": outPath = value; break;
                    case "--margin":
                        if (!double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out margin) || !(margin >= 0))
                        {
                            error.WriteLine($"--margin '{value}' is not a distance");
                            return 64;
                        }

                        break;
                    default:
                        error.WriteLine($"unknown argument '{args[i - 1]}'");
                        error.WriteLine(Usage);
                        return 64;
                }
            }

            if (urdf == null || profilePath == null || outPath == null)
            {
                error.WriteLine(Usage);
                return 64;
            }

            SerialChainCollisionDocument document;
            try
            {
                SerialChainProfile profile = SerialChainProfileJson.Load(profilePath);
                byte[] bytes = File.ReadAllBytes(urdf);
                document = Fit(XDocument.Parse(System.Text.Encoding.UTF8.GetString(bytes)), Path.GetDirectoryName(Path.GetFullPath(urdf))!, profile, margin);
                document.urdf = ImportUrdfCommand.RepoRelativePath(urdf);
                document.sha256 = Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant();
            }
            catch (Exception e) when (e is IOException || e is FormatException || e is InvalidDataException || e is System.Xml.XmlException)
            {
                error.WriteLine($"fit-capsules: {e.Message}");
                return 1;
            }

            string json = JsonSerializer.Serialize(document, new JsonSerializerOptions { IncludeFields = true, WriteIndented = true }).Replace("\r\n", "\n") + "\n";
            File.WriteAllText(outPath, json);
            output.WriteLine($"Wrote {outPath}: {document.capsules.Length} capsules, {document.disabled.Length} disabled pairs, margin {margin} m");
            foreach (DisabledPairDocument p in document.disabled.Where(p => p.reason != "adjacent"))
            {
                output.WriteLine($"  disabled ({p.reason}): {p.a} / {p.b}");
            }

            return 0;
        }

        private sealed class Joint
        {
            public string Name = "", Type = "", Parent = "", Child = "";
            public RigidTransform Origin = RigidTransform.Identity;
            public double Ax = 1, Ay, Az, Upper;
        }

        /// <summary>The collision model for <paramref name="urdf"/>, its meshes found relative to <paramref name="baseDir"/>.</summary>
        internal static SerialChainCollisionDocument Fit(XDocument urdf, string baseDir, SerialChainProfile profile, double margin)
        {
            XElement robot = urdf.Root ?? throw new FormatException("URDF has no root");
            var joints = new Dictionary<string, Joint>(StringComparer.Ordinal);
            foreach (XElement e in robot.Elements("joint"))
            {
                var j = new Joint
                {
                    Name = (string)e.Attribute("name")!,
                    Type = (string)e.Attribute("type")!,
                    Parent = (string)e.Element("parent")!.Attribute("link")!,
                    Child = (string)e.Element("child")!.Attribute("link")!,
                    Origin = Origin(e.Element("origin")),
                };
                double[] axis = Vector(e.Element("axis")?.Attribute("xyz")?.Value, new[] { 1.0, 0, 0 });
                (j.Ax, j.Ay, j.Az) = (axis[0], axis[1], axis[2]);
                j.Upper = double.TryParse(e.Element("limit")?.Attribute("upper")?.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out double u) ? u : 0;
                joints[j.Name] = j;
            }

            var byChild = joints.Values.ToDictionary(j => j.Child, StringComparer.Ordinal);
            var armJoint = new Dictionary<string, (int Arm, int Index)>(StringComparer.Ordinal);
            var fingers = new Dictionary<string, int>(StringComparer.Ordinal);
            var armBase = new Dictionary<string, int>(StringComparer.Ordinal);
            for (int a = 0; a < profile.Arms.Length; a++)
            {
                ChainArm arm = profile.Arms[a];
                for (int k = 0; k < arm.Joints.Length; k++) armJoint[arm.Joints[k].Name] = (a, k);
                armBase[joints[arm.Joints[0].Name].Parent] = a;
                if (arm.HasGripper)
                {
                    fingers[arm.Gripper.Name] = a;
                    foreach (Joint m in joints.Values)
                    {
                        XElement? mimic = robot.Elements("joint").First(x => (string)x.Attribute("name")! == m.Name).Element("mimic");
                        if (mimic != null && (string)mimic.Attribute("joint")! == arm.Gripper.Name) fingers[m.Name] = a;
                    }
                }
            }

            // Every link's pose in the root at zero (fingers open), and what moves it.
            RigidTransform PoseOf(string link, out int arm, out int frame)
            {
                var path = new List<Joint>();
                for (string l = link; byChild.TryGetValue(l, out Joint? j); l = j.Parent) path.Add(j);
                path.Reverse();
                arm = -1;
                frame = -1;
                RigidTransform pose = RigidTransform.Identity;
                foreach (Joint j in path)
                {
                    if (armBase.TryGetValue(j.Child, out int baseArm) && arm < 0) arm = baseArm;
                    pose = pose.Compose(j.Origin);
                    if (armJoint.TryGetValue(j.Name, out var aj))
                    {
                        arm = aj.Arm;
                        frame = aj.Index;
                    }
                    else if (fingers.ContainsKey(j.Name))
                    {
                        pose = pose.Compose(RigidTransform.Translation(j.Ax * j.Upper, j.Ay * j.Upper, j.Az * j.Upper));
                    }
                }

                return pose;
            }

            RigidTransform FramePose(int arm, int frame)
            {
                ChainArm chain = profile.Arms[arm];
                string link = frame < 0 ? joints[chain.Joints[0].Name].Parent : joints[chain.Joints[frame].Name].Child;
                return PoseOf(link, out _, out _);
            }

            var capsules = new List<ChainCapsuleDocument>();
            foreach (XElement link in robot.Elements("link"))
            {
                var cols = link.Elements("collision").ToList();
                if (cols.Count == 0) continue;
                string name = (string)link.Attribute("name")!;
                RigidTransform linkPose = PoseOf(name, out int arm, out int frame);
                RigidTransform toFrame = (arm < 0 ? RigidTransform.Identity : FramePose(arm, frame)).Inverse().Compose(linkPose);
                var points = new List<double[]>();
                foreach (XElement col in cols)
                {
                    RigidTransform t = toFrame.Compose(Origin(col.Element("origin")));
                    XElement geometry = col.Element("geometry") ?? throw new FormatException($"link {name}: a collision without geometry");
                    if (geometry.Element("mesh") is XElement mesh)
                    {
                        string file = Path.Combine(baseDir, ((string)mesh.Attribute("filename")!).Replace('/', Path.DirectorySeparatorChar));
                        foreach (double[] v in StlVertices(file)) points.Add(Apply(t, v));
                    }
                    else if (geometry.Element("box") is XElement box)
                    {
                        double[] s = Vector(box.Attribute("size")?.Value, null);
                        for (int c = 0; c < 8; c++)
                        {
                            points.Add(Apply(t, new[] { ((c & 1) != 0 ? 0.5 : -0.5) * s[0], ((c & 2) != 0 ? 0.5 : -0.5) * s[1], ((c & 4) != 0 ? 0.5 : -0.5) * s[2] }));
                        }
                    }
                    else
                    {
                        throw new FormatException($"link {name}: only mesh and box collision geometry is supported");
                    }
                }

                capsules.Add(FitCapsule(name, arm, frame, points));
            }

            var doc = new SerialChainCollisionDocument { profile = profile.Name, margin = margin, capsules = capsules.ToArray() };
            var disabled = new List<DisabledPairDocument>();
            for (int i = 0; i < capsules.Count; i++)
            {
                for (int k = i + 1; k < capsules.Count; k++)
                {
                    ChainCapsuleDocument a = capsules[i], b = capsules[k];
                    bool adjacent = a.arm == b.arm && a.arm >= 0 && Math.Abs(a.frame - b.frame) == 1;
                    bool onMount = (a.arm < 0 && b.arm >= 0 && b.frame < 0) || (b.arm < 0 && a.arm >= 0 && a.frame < 0) ||
                        (a.arm >= 0 && b.arm >= 0 && a.arm != b.arm && a.frame < 0 && b.frame < 0);
                    if (adjacent || onMount) disabled.Add(new DisabledPairDocument { a = a.name, b = b.name, reason = "adjacent" });
                }
            }

            doc.disabled = disabled.ToArray();
            SerialChainCollisionModel model = doc.ToModel();
            double[] ready = ReadyPose(profile);
            var touching = new HashSet<(int, int, int, int)>();
            for (int i = 0; i < capsules.Count; i++)
            {
                for (int k = i + 1; k < capsules.Count; k++)
                {
                    if (!model.IsChecked(i, k) || capsules[i].arm != capsules[k].arm && capsules[i].arm >= 0 && capsules[k].arm >= 0) continue;
                    if (PairTooClose(profile, model, ready, i, k)) touching.Add(Key(capsules, i, k));
                }
            }

            // A pair touching at the ready pose is disabled on every arm alike, so the arms (mirror images) are checked the same way.
            for (int i = 0; i < capsules.Count; i++)
            {
                for (int k = i + 1; k < capsules.Count; k++)
                {
                    bool acrossArms = capsules[i].arm >= 0 && capsules[k].arm >= 0 && capsules[i].arm != capsules[k].arm;
                    if (!acrossArms && model.IsChecked(i, k) && touching.Contains(Key(capsules, i, k)))
                    {
                        disabled.Add(new DisabledPairDocument { a = capsules[i].name, b = capsules[k].name, reason = "touching-at-ready" });
                    }
                }
            }

            doc.disabled = disabled.ToArray();
            return doc;
        }

        /// <summary>A pair's shape, whatever its arm: each capsule's frame and its place among that frame's capsules, body or arm.</summary>
        private static (int, int, int, int) Key(List<ChainCapsuleDocument> capsules, int i, int k)
        {
            int Rank(int c) => capsules.Take(c).Count(o => o.arm == capsules[c].arm && o.frame == capsules[c].frame);
            int Code(int c) => capsules[c].arm < 0 ? -1000 - c : capsules[c].frame;
            (int, int) a = (Code(i), Rank(i)), b = (Code(k), Rank(k));
            return a.CompareTo(b) <= 0 ? (a.Item1, a.Item2, b.Item1, b.Item2) : (b.Item1, b.Item2, a.Item1, a.Item2);
        }

        /// <summary>Franka's ready pose for every arm, mirrored for every second arm, as RobotHost's SerialChainHost.StartPose.</summary>
        internal static double[] ReadyPose(SerialChainProfile profile)
        {
            double[] ready = { 0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4 };
            var pose = new List<double>();
            for (int a = 0; a < profile.Arms.Length; a++)
            {
                ChainJoint[] js = profile.Arms[a].Joints;
                for (int j = 0; j < js.Length; j++)
                {
                    double q = j < ready.Length ? ready[j] : 0;
                    if (a % 2 == 1 && j % 2 == 0) q = -q;
                    pose.Add(Math.Min(Math.Max(q, js[j].Limits.SoftLower), js[j].Limits.SoftUpper));
                }
            }

            return pose.ToArray();
        }

        private static bool PairTooClose(SerialChainProfile profile, SerialChainCollisionModel model, double[] pose, int i, int k)
        {
            // Only this pair: a model of the two capsules alone, nothing disabled.
            var pair = new SerialChainCollisionModel(model.Profile, new[] { model.Capsules[i], model.Capsules[k] }, Array.Empty<(int, int)>(), model.Margin);
            return SerialChainCollision.Check(profile, pair, pose).IsHit;
        }

        private static ChainCapsuleDocument FitCapsule(string name, int arm, int frame, List<double[]> points)
        {
            int n = points.Count;
            double mx = points.Average(p => p[0]), my = points.Average(p => p[1]), mz = points.Average(p => p[2]);
            var cov = new double[3, 3];
            foreach (double[] p in points)
            {
                double[] d = { p[0] - mx, p[1] - my, p[2] - mz };
                for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) cov[r, c] += d[r] * d[c] / n;
            }

            // The principal axis by power iteration, from a fixed start: deterministic.
            double ax = 1, ay = 1, az = 1;
            for (int it = 0; it < 200; it++)
            {
                double nx = cov[0, 0] * ax + cov[0, 1] * ay + cov[0, 2] * az;
                double ny = cov[1, 0] * ax + cov[1, 1] * ay + cov[1, 2] * az;
                double nz = cov[2, 0] * ax + cov[2, 1] * ay + cov[2, 2] * az;
                double len = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                if (len < 1e-30) break;
                (ax, ay, az) = (nx / len, ny / len, nz / len);
            }

            double lo = double.PositiveInfinity, hi = double.NegativeInfinity, radius = 0;
            foreach (double[] p in points)
            {
                double s = (p[0] - mx) * ax + (p[1] - my) * ay + (p[2] - mz) * az;
                lo = Math.Min(lo, s);
                hi = Math.Max(hi, s);
                double px = p[0] - mx - s * ax, py = p[1] - my - s * ay, pz = p[2] - mz - s * az;
                radius = Math.Max(radius, Math.Sqrt(px * px + py * py + pz * pz));
            }

            // Shrink the segment by the radius at each end (the caps cover it), then grow the radius until every vertex is inside.
            double s0 = lo + radius, s1 = hi - radius;
            if (s0 > s1) s0 = s1 = 0.5 * (lo + hi);
            var a = new[] { mx + s0 * ax, my + s0 * ay, mz + s0 * az };
            var b = new[] { mx + s1 * ax, my + s1 * ay, mz + s1 * az };
            a = a.Select(Round).ToArray();
            b = b.Select(Round).ToArray();
            double worst = 0;
            var seg = new[] { a[0], a[1], a[2], b[0], b[1], b[2] };
            foreach (double[] p in points)
            {
                double d = Math.Sqrt(SerialChainCollision.SegmentDistanceSquared(seg, new[] { p[0], p[1], p[2], p[0], p[1], p[2] }));
                worst = Math.Max(worst, d);
            }

            return new ChainCapsuleDocument { name = name, arm = arm, frame = frame, a = a, b = b, radius = Math.Ceiling(worst * 1e6) / 1e6 };
        }

        private static double Round(double v) => Math.Round(v, 6) + 0.0;

        private static double[] Apply(in RigidTransform t, double[] v)
        {
            t.TransformPoint(v[0], v[1], v[2], out double x, out double y, out double z);
            return new[] { x, y, z };
        }

        internal static IEnumerable<double[]> StlVertices(string path)
        {
            byte[] d = File.ReadAllBytes(path);
            if (d.Length < 84) throw new InvalidDataException($"{path}: not a binary STL");
            uint triangles = BitConverter.ToUInt32(d, 80);
            if (84 + 50L * triangles != d.Length) throw new InvalidDataException($"{path}: not a binary STL ({triangles} triangles, {d.Length} bytes)");
            for (int t = 0; t < triangles; t++)
            {
                int at = 84 + 50 * t + 12;
                for (int k = 0; k < 3; k++, at += 12)
                {
                    yield return new double[] { BitConverter.ToSingle(d, at), BitConverter.ToSingle(d, at + 4), BitConverter.ToSingle(d, at + 8) };
                }
            }
        }

        private static RigidTransform Origin(XElement? origin)
        {
            if (origin == null) return RigidTransform.Identity;
            double[] xyz = Vector(origin.Attribute("xyz")?.Value, new double[3]);
            double[] rpy = Vector(origin.Attribute("rpy")?.Value, new double[3]);
            return RigidTransform.FromXyzRpy(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
        }

        private static double[] Vector(string? text, double[]? fallback)
        {
            if (text == null) return fallback ?? throw new FormatException("a vector is missing");
            double[] v = text.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries).Select(x => double.Parse(x, NumberStyles.Float, CultureInfo.InvariantCulture)).ToArray();
            if (v.Length != 3) throw new FormatException($"'{text}' is not three numbers");
            return v;
        }
    }
}
