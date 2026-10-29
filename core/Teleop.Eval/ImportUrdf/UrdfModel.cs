using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Xml.Linq;

namespace Teleop.Eval.ImportUrdf
{
    /// <summary>One URDF <c>&lt;joint&gt;</c>, with the parts the importer reads (docs/adr/0026 decision 3).</summary>
    internal sealed class UrdfJoint
    {
        public string Name = "";
        public string Type = "";
        public string Parent = "";
        public string Child = "";
        public double[] Xyz = { 0, 0, 0 };
        public double[] Rpy = { 0, 0, 0 };
        public double[] Axis = { 1, 0, 0 };

        public bool HasLimit;
        public double Lower;
        public double Upper;
        public double Velocity;
        public double Effort;

        public bool HasSafetyController;
        public double SoftLower;
        public double SoftUpper;

        public double PositionBasedDeceleration;
        public double PositionBasedVelocityOffset;

        /// <summary>The joint this one mimics, or null.</summary>
        public string? Mimics;

        public bool IsMoving => Type == "revolute" || Type == "prismatic" || Type == "continuous";
    }

    /// <summary>
    /// The kinematic tree of a URDF: its joints, indexed by child link. It reads only what a
    /// serial-chain profile needs. Visual, collision and inertial elements are left for the tools that
    /// use them.
    /// </summary>
    internal sealed class UrdfModel
    {
        public string RobotName { get; }
        public string RootLink { get; }
        public IReadOnlyList<UrdfJoint> Joints { get; }

        private readonly Dictionary<string, UrdfJoint> _byChild;
        private readonly Dictionary<string, UrdfJoint> _byName;

        private UrdfModel(string robotName, string rootLink, List<UrdfJoint> joints)
        {
            RobotName = robotName;
            RootLink = rootLink;
            Joints = joints;
            _byChild = joints.ToDictionary(j => j.Child, StringComparer.Ordinal);
            _byName = joints.ToDictionary(j => j.Name, StringComparer.Ordinal);
        }

        public static UrdfModel Parse(string xml)
        {
            XElement robot = XDocument.Parse(xml).Root ?? throw new FormatException("URDF has no root element");
            if (robot.Name.LocalName != "robot") throw new FormatException($"URDF root is <{robot.Name.LocalName}>, not <robot>");

            var links = new HashSet<string>(StringComparer.Ordinal);
            foreach (XElement link in robot.Elements("link"))
            {
                string name = Required(link, "name", "link");
                if (!links.Add(name)) throw new FormatException($"link '{name}' is defined twice");
            }

            var joints = new List<UrdfJoint>();
            var children = new HashSet<string>(StringComparer.Ordinal);
            var names = new HashSet<string>(StringComparer.Ordinal);
            foreach (XElement e in robot.Elements("joint"))
            {
                var j = new UrdfJoint
                {
                    Name = Required(e, "name", "joint"),
                    Type = Required(e, "type", "joint"),
                };
                string where = $"joint '{j.Name}'";
                if (!names.Add(j.Name)) throw new FormatException($"{where} is defined twice");

                j.Parent = Required(e.Element("parent") ?? throw new FormatException($"{where} has no <parent>"), "link", where + " parent");
                j.Child = Required(e.Element("child") ?? throw new FormatException($"{where} has no <child>"), "link", where + " child");
                if (!links.Contains(j.Parent) || !links.Contains(j.Child)) throw new FormatException($"{where} names a link that is not defined");
                if (!children.Add(j.Child)) throw new FormatException($"link '{j.Child}' has two parent joints");

                XElement? origin = e.Element("origin");
                if (origin != null)
                {
                    j.Xyz = Vector(origin.Attribute("xyz")?.Value, where + " origin xyz");
                    j.Rpy = Vector(origin.Attribute("rpy")?.Value, where + " origin rpy");
                }

                XElement? axis = e.Element("axis");
                if (axis != null) j.Axis = Vector(axis.Attribute("xyz")?.Value, where + " axis");

                XElement? limit = e.Element("limit");
                if (limit != null)
                {
                    j.HasLimit = true;
                    j.Lower = Number(limit.Attribute("lower")?.Value ?? "0", where + " lower");
                    j.Upper = Number(limit.Attribute("upper")?.Value ?? "0", where + " upper");
                    j.Velocity = Number(Required(limit, "velocity", where + " limit"), where + " velocity");
                    j.Effort = Number(Required(limit, "effort", where + " limit"), where + " effort");
                }

                XElement? safety = e.Element("safety_controller");
                if (safety?.Attribute("soft_lower_limit") != null && safety.Attribute("soft_upper_limit") != null)
                {
                    j.HasSafetyController = true;
                    j.SoftLower = Number(safety.Attribute("soft_lower_limit")!.Value, where + " soft_lower_limit");
                    j.SoftUpper = Number(safety.Attribute("soft_upper_limit")!.Value, where + " soft_upper_limit");
                }

                XElement? pbv = e.Element("position_based_velocity_limits");
                if (pbv != null)
                {
                    j.PositionBasedDeceleration = Number(Required(pbv, "deceleration_limit", where), where + " deceleration_limit");
                    j.PositionBasedVelocityOffset = Number(Required(pbv, "velocity_offset", where), where + " velocity_offset");
                }

                j.Mimics = e.Element("mimic")?.Attribute("joint")?.Value;
                joints.Add(j);
            }

            string[] roots = links.Where(l => !children.Contains(l)).OrderBy(l => l, StringComparer.Ordinal).ToArray();
            if (roots.Length != 1) throw new FormatException($"URDF must have exactly one root link; found {roots.Length} ({string.Join(", ", roots)})");

            return new UrdfModel(Required(robot, "name", "robot"), roots[0], joints);
        }

        public bool TryGetJoint(string name, out UrdfJoint joint) => _byName.TryGetValue(name, out joint!);

        /// <summary>The joints from the root link down to <paramref name="link"/>, root first.</summary>
        public List<UrdfJoint> PathTo(string link)
        {
            var path = new List<UrdfJoint>();
            string current = link;
            while (current != RootLink)
            {
                if (!_byChild.TryGetValue(current, out UrdfJoint? joint)) throw new FormatException($"link '{link}' is not in the URDF");
                path.Add(joint);
                current = joint.Parent;
            }

            path.Reverse();
            return path;
        }

        public bool IsMimicked(string jointName) => Joints.Any(j => j.Mimics == jointName);

        private static string Required(XElement e, string attribute, string where) =>
            e.Attribute(attribute)?.Value ?? throw new FormatException($"{where} has no '{attribute}'");

        private static double[] Vector(string? text, string where)
        {
            if (text == null) return new double[] { 0, 0, 0 };
            string[] parts = text.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != 3) throw new FormatException($"{where}: '{text}' is not three numbers");
            return new[] { Number(parts[0], where), Number(parts[1], where), Number(parts[2], where) };
        }

        /// <summary>Parses a number exactly as written. <c>-0</c> becomes 0, so the profile never says "-0".</summary>
        private static double Number(string text, string where)
        {
            if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double v) || double.IsNaN(v) || double.IsInfinity(v))
            {
                throw new FormatException($"{where}: '{text}' is not a number");
            }

            return v + 0.0;
        }
    }
}
