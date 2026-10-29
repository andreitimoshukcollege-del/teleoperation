using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.ImportUrdf
{
    /// <summary>
    /// The <c>import-urdf</c> verb (docs/adr/0026 decision 3). It turns a robot's URDF into a
    /// serial-chain profile, so the FR3 Duo's profile is generated, never hand-written:
    /// <c>just import-fr3-profile</c> writes <c>core/RobotProfiles/fr3-duo.json</c>, and a test
    /// re-imports the vendored URDF and requires the committed bytes.
    ///
    /// <para>Each arm is the URDF path from the root link to its tip link:
    /// <list type="bullet">
    /// <item>the fixed joints before the first moving joint are the mount;</item>
    /// <item>fixed joints between two moving joints are folded into the next joint's origin;</item>
    /// <item>the fixed joints after the last moving joint are the tool.</item>
    /// </list>
    /// A <c>continuous</c>, <c>floating</c> or <c>planar</c> joint on the path, or a mimic joint, is an
    /// error, not something guessed at.</para>
    /// </summary>
    public static class ImportUrdfCommand
    {
        public static int Run(string[] args) => Run(args, Console.Out, Console.Error);

        internal static int Run(string[] args, TextWriter output, TextWriter error)
        {
            ImportUrdfArgs? parsed = ImportUrdfArgs.TryParse(args, out string usageError);
            if (parsed == null)
            {
                error.WriteLine(usageError);
                error.WriteLine(ImportUrdfArgs.Usage);
                return 64;
            }

            SerialChainProfileDocument document;
            SerialChainProfile profile;
            try
            {
                byte[] bytes = File.ReadAllBytes(parsed.UrdfPath);
                string sha256 = Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant();
                UrdfModel model = UrdfModel.Parse(System.Text.Encoding.UTF8.GetString(bytes));
                document = Build(model, parsed, RepoRelativePath(parsed.UrdfPath), sha256);
                profile = document.ToProfile();
            }
            catch (Exception e) when (e is IOException || e is FormatException || e is System.Xml.XmlException || e is UnauthorizedAccessException)
            {
                error.WriteLine($"import-urdf: {e.Message}");
                return 1;
            }

            string? directory = Path.GetDirectoryName(Path.GetFullPath(parsed.OutPath));
            if (directory != null) Directory.CreateDirectory(directory);
            File.WriteAllText(parsed.OutPath, SerialChainProfileJson.Serialize(document));

            output.WriteLine($"Wrote {parsed.OutPath}: profile '{profile.Name}', {profile.MotorCount} motor ids");
            foreach (ChainArm arm in profile.Arms)
            {
                string ids = string.Join(",", arm.Joints.Select(j => j.MotorId.ToString(System.Globalization.CultureInfo.InvariantCulture)));
                output.WriteLine($"  {arm.Name}: {arm.Joints.Length} joints (ids {ids})" +
                    (arm.HasGripper ? $", gripper id {arm.Gripper.MotorId}" : ", no gripper"));
            }

            return 0;
        }

        /// <summary>The profile document for <paramref name="model"/>. Pure: everything it needs is passed in.</summary>
        internal static SerialChainProfileDocument Build(UrdfModel model, ImportUrdfArgs args, string sourcePath, string sha256)
        {
            var arms = new List<ChainArmDocument>();
            foreach (ArmSpec spec in args.Arms)
            {
                if (arms.Any(a => a.name == spec.Name)) throw new FormatException($"arm '{spec.Name}' is given twice");
                arms.Add(BuildArm(model, spec, args));
            }

            foreach (GripperSpec g in args.Grippers)
            {
                ChainArmDocument arm = arms.FirstOrDefault(a => a.name == g.Arm)
                    ?? throw new FormatException($"--gripper names arm '{g.Arm}', which no --arm defines");
                if (arm.hasGripper) throw new FormatException($"arm '{g.Arm}' is given two grippers");
                arm.hasGripper = true;
                arm.gripper = BuildGripper(model, g);
            }

            return new SerialChainProfileDocument
            {
                name = args.Name,
                kind = SerialChainProfile.Kind,
                source = new ProfileSourceDocument { urdf = sourcePath, sha256 = sha256, upstream = args.Upstream },
                arms = arms.ToArray(),
            };
        }

        private static ChainArmDocument BuildArm(UrdfModel model, ArmSpec spec, ImportUrdfArgs args)
        {
            string where = $"arm '{spec.Name}'";
            List<UrdfJoint> path = model.PathTo(spec.TipLink);

            var mount = new List<UrdfOriginDocument>();
            var pending = new List<UrdfOriginDocument>();
            var joints = new List<ChainJointDocument>();

            foreach (UrdfJoint j in path)
            {
                if (j.Type == "fixed")
                {
                    (joints.Count == 0 ? mount : pending).Add(Origin(j));
                    continue;
                }

                if (j.Type != "revolute" && j.Type != "prismatic")
                {
                    throw new FormatException($"{where}: joint '{j.Name}' is '{j.Type}'; only fixed, revolute and prismatic joints are supported");
                }

                if (j.Mimics != null) throw new FormatException($"{where}: joint '{j.Name}' mimics '{j.Mimics}'; a mimic joint cannot be on an arm's chain");
                if (!j.HasLimit) throw new FormatException($"{where}: joint '{j.Name}' has no <limit>");

                int id = spec.FirstMotorId + joints.Count;
                if (id > byte.MaxValue) throw new FormatException($"{where}: motor ids run past 255");

                pending.Add(Origin(j));
                joints.Add(new ChainJointDocument
                {
                    motorId = (byte)id,
                    name = j.Name,
                    type = j.Type,
                    origin = pending.ToArray(),
                    axis = (double[])j.Axis.Clone(),
                    lower = j.Lower,
                    upper = j.Upper,
                    softLower = j.HasSafetyController ? j.SoftLower : j.Lower,
                    softUpper = j.HasSafetyController ? j.SoftUpper : j.Upper,
                    velocity = j.Velocity,
                    acceleration = args.Acceleration,
                    jerk = args.Jerk,
                    effort = j.Effort,
                    positionBasedDeceleration = j.PositionBasedDeceleration,
                    positionBasedVelocityOffset = j.PositionBasedVelocityOffset,
                });
                pending.Clear();
            }

            if (joints.Count == 0) throw new FormatException($"{where}: no moving joint between the root link and '{spec.TipLink}'");

            return new ChainArmDocument
            {
                name = spec.Name,
                mount = mount.ToArray(),
                joints = joints.ToArray(),
                tool = pending.ToArray(),
            };
        }

        private static ChainGripperDocument BuildGripper(UrdfModel model, GripperSpec spec)
        {
            string where = $"arm '{spec.Arm}' gripper '{spec.FingerJoint}'";
            if (!model.TryGetJoint(spec.FingerJoint, out UrdfJoint j)) throw new FormatException($"{where}: no such joint");
            if (j.Type != "prismatic") throw new FormatException($"{where}: is '{j.Type}', not prismatic");
            if (j.Mimics != null) throw new FormatException($"{where}: mimics '{j.Mimics}'; name the driven finger");
            if (!j.HasLimit || j.Lower != 0 || !(j.Upper > 0)) throw new FormatException($"{where}: needs a limit from 0 (closed) to its open travel");

            return new ChainGripperDocument
            {
                motorId = spec.MotorId,
                name = j.Name,
                fingerTravel = j.Upper,
                speed = j.Velocity,
                secondFingerMimics = model.IsMimicked(j.Name),
            };
        }

        private static UrdfOriginDocument Origin(UrdfJoint j) => new UrdfOriginDocument
        {
            joint = j.Name,
            xyz = (double[])j.Xyz.Clone(),
            rpy = (double[])j.Rpy.Clone(),
        };

        /// <summary>
        /// The URDF's path relative to the repository root, with forward slashes. The root is the first
        /// directory above the file that holds both <c>core/</c> and <c>robot/</c>. The recorded source
        /// is then the same on every machine and from every working directory, or just the file name
        /// outside a repository.
        /// </summary>
        internal static string RepoRelativePath(string path)
        {
            string full = Path.GetFullPath(path);
            for (DirectoryInfo? dir = new FileInfo(full).Directory; dir != null; dir = dir.Parent)
            {
                if (Directory.Exists(Path.Combine(dir.FullName, "core")) && Directory.Exists(Path.Combine(dir.FullName, "robot")))
                {
                    return Path.GetRelativePath(dir.FullName, full).Replace('\\', '/');
                }
            }

            return Path.GetFileName(full);
        }
    }
}
