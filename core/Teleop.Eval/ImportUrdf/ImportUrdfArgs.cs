using System;
using System.Collections.Generic;
using System.Globalization;

namespace Teleop.Eval.ImportUrdf
{
    /// <summary>One <c>--arm name:tipLink:firstMotorId</c>.</summary>
    internal readonly struct ArmSpec
    {
        public readonly string Name;
        public readonly string TipLink;
        public readonly byte FirstMotorId;

        public ArmSpec(string name, string tipLink, byte firstMotorId)
        {
            Name = name;
            TipLink = tipLink;
            FirstMotorId = firstMotorId;
        }
    }

    /// <summary>One <c>--gripper arm:fingerJoint:motorId</c>.</summary>
    internal readonly struct GripperSpec
    {
        public readonly string Arm;
        public readonly string FingerJoint;
        public readonly byte MotorId;

        public GripperSpec(string arm, string fingerJoint, byte motorId)
        {
            Arm = arm;
            FingerJoint = fingerJoint;
            MotorId = motorId;
        }
    }

    /// <summary>Parsed command-line arguments for <see cref="ImportUrdfCommand"/>.</summary>
    internal sealed class ImportUrdfArgs
    {
        public const string Usage =
            "Usage: Teleop.Eval import-urdf --urdf <file> --out <file> --name <profile>\n" +
            "         --arm <name>:<tipLink>:<firstMotorId> [--arm ...]\n" +
            "         [--gripper <arm>:<fingerJoint>:<motorId> ...]\n" +
            "         --acceleration <unit/s^2> --jerk <unit/s^3> [--upstream <repo@commit>]\n" +
            "  Writes a serial-chain robot profile (docs/adr/0026) from a URDF. Each arm is the chain of\n" +
            "  moving joints from the URDF's root link to <tipLink>, numbered from <firstMotorId>. The\n" +
            "  URDF has no acceleration or jerk limits, so they are given here and recorded per joint.";

        public string UrdfPath = "";
        public string OutPath = "";
        public string Name = "";
        public string Upstream = "";
        public double Acceleration;
        public double Jerk;
        public readonly List<ArmSpec> Arms = new List<ArmSpec>();
        public readonly List<GripperSpec> Grippers = new List<GripperSpec>();

        /// <summary>The parsed arguments, or null with <paramref name="error"/> set.</summary>
        public static ImportUrdfArgs? TryParse(string[] args, out string error)
        {
            var a = new ImportUrdfArgs();
            error = "";
            for (int i = 1; i < args.Length; i++)
            {
                string flag = args[i];
                if (i + 1 >= args.Length)
                {
                    error = $"{flag} needs a value";
                    return null;
                }

                string value = args[++i];
                switch (flag)
                {
                    case "--urdf": a.UrdfPath = value; break;
                    case "--out": a.OutPath = value; break;
                    case "--name": a.Name = value; break;
                    case "--upstream": a.Upstream = value; break;
                    case "--acceleration":
                        if (!TryPositive(value, out a.Acceleration)) { error = $"--acceleration '{value}' is not a positive number"; return null; }
                        break;
                    case "--jerk":
                        if (!TryPositive(value, out a.Jerk)) { error = $"--jerk '{value}' is not a positive number"; return null; }
                        break;
                    case "--arm":
                        if (!TrySplit(value, out string armName, out string tip, out byte first)) { error = $"--arm '{value}' is not <name>:<tipLink>:<firstMotorId>"; return null; }
                        a.Arms.Add(new ArmSpec(armName, tip, first));
                        break;
                    case "--gripper":
                        if (!TrySplit(value, out string arm, out string finger, out byte id)) { error = $"--gripper '{value}' is not <arm>:<fingerJoint>:<motorId>"; return null; }
                        a.Grippers.Add(new GripperSpec(arm, finger, id));
                        break;
                    default:
                        error = $"unknown argument '{flag}'";
                        return null;
                }
            }

            if (a.UrdfPath.Length == 0 || a.OutPath.Length == 0 || a.Name.Length == 0) error = "--urdf, --out and --name are required";
            else if (a.Arms.Count == 0) error = "at least one --arm is required";
            else if (!(a.Acceleration > 0) || !(a.Jerk > 0)) error = "--acceleration and --jerk are required";
            return error.Length == 0 ? a : null;
        }

        private static bool TryPositive(string text, out double value) =>
            double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) && value > 0 && !double.IsInfinity(value);

        private static bool TrySplit(string text, out string first, out string second, out byte id)
        {
            first = second = "";
            id = 0;
            string[] parts = text.Split(':');
            if (parts.Length != 3 || parts[0].Length == 0 || parts[1].Length == 0) return false;
            first = parts[0];
            second = parts[1];
            return byte.TryParse(parts[2], NumberStyles.None, CultureInfo.InvariantCulture, out id) && id > 0;
        }
    }
}
