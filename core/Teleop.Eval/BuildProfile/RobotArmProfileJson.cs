using System;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.BuildProfile
{
    /// <summary>
    /// Loads and saves a <see cref="RobotArmProfile"/> as JSON (<c>core/RobotProfiles/*.json</c>
    /// convention, docs/adr/0011-generic-robot-arm-profiles.md). Deliberately a separate, small
    /// copy of the load half of <c>Teleop.RobotHost.Plant.RobotArmProfileJson</c> rather than a
    /// shared dependency between these two sibling host processes -- see that class's own doc
    /// comment for why (the same reasoning this codebase already applies to
    /// <c>MonotonicClock</c>). This copy also needs to *write* the format, which
    /// <c>Teleop.RobotHost</c> never does.
    /// </summary>
    public static class RobotArmProfileJson
    {
        private static JsonSerializerOptions Options()
        {
            var options = new JsonSerializerOptions { PropertyNameCaseInsensitive = true, WriteIndented = true };
            options.Converters.Add(new JsonStringEnumConverter());
            return options;
        }

        public static RobotArmProfile Load(string path)
        {
            string json = File.ReadAllText(path);
            RobotArmProfileFile? file = JsonSerializer.Deserialize<RobotArmProfileFile>(json, Options());
            if (file is null)
            {
                throw new InvalidDataException($"Robot arm profile at '{path}' deserialized to null.");
            }

            return file.ToProfile();
        }

        public static void Save(string path, in RobotArmProfile profile)
        {
            var file = RobotArmProfileFile.FromProfile(profile);
            string json = JsonSerializer.Serialize(file, Options());
            string? directory = Path.GetDirectoryName(Path.GetFullPath(path));
            if (!string.IsNullOrEmpty(directory))
            {
                Directory.CreateDirectory(directory);
            }

            File.WriteAllText(path, json);
        }
    }

    /// <summary>Plain, JSON-friendly mirror of <see cref="RobotArmProfile"/> -- a constructor-only readonly struct isn't a convenient serialization target in either direction, so this DTO exists purely to move data through JSON.</summary>
    internal sealed class RobotArmProfileFile
    {
        public string Name { get; set; } = "";
        public bool HasRotatingBase { get; set; }
        public float BaseHeight { get; set; }
        public float ProximalLinkLength { get; set; }
        public float DistalLinkLength { get; set; }
        public int WristJointCount { get; set; }
        public bool HasGripper { get; set; }
        public bool GripperCanRotate { get; set; }
        public JointHardwareSpecFile[] Joints { get; set; } = Array.Empty<JointHardwareSpecFile>();

        public RobotArmProfile ToProfile()
        {
            var joints = new JointHardwareSpec[Joints.Length];
            for (int i = 0; i < Joints.Length; i++)
            {
                JointHardwareSpecFile j = Joints[i];
                joints[i] = new JointHardwareSpec(
                    j.MotorId, j.Role, j.WristIndex, j.MinAngleRadians, j.MaxAngleRadians, j.ZeroOffsetRadians);
            }

            return new RobotArmProfile(
                Name, HasRotatingBase, BaseHeight, ProximalLinkLength, DistalLinkLength,
                WristJointCount, HasGripper, GripperCanRotate, joints);
        }

        public static RobotArmProfileFile FromProfile(in RobotArmProfile profile)
        {
            var joints = new JointHardwareSpecFile[profile.Joints.Length];
            for (int i = 0; i < profile.Joints.Length; i++)
            {
                JointHardwareSpec j = profile.Joints[i];
                joints[i] = new JointHardwareSpecFile
                {
                    MotorId = j.MotorId,
                    Role = j.Role,
                    WristIndex = j.WristIndex,
                    MinAngleRadians = j.MinAngleRadians,
                    MaxAngleRadians = j.MaxAngleRadians,
                    ZeroOffsetRadians = j.ZeroOffsetRadians,
                };
            }

            return new RobotArmProfileFile
            {
                Name = profile.Name,
                HasRotatingBase = profile.HasRotatingBase,
                BaseHeight = profile.BaseHeight,
                ProximalLinkLength = profile.ProximalLinkLength,
                DistalLinkLength = profile.DistalLinkLength,
                WristJointCount = profile.WristJointCount,
                HasGripper = profile.HasGripper,
                GripperCanRotate = profile.GripperCanRotate,
                Joints = joints,
            };
        }
    }

    internal sealed class JointHardwareSpecFile
    {
        public byte MotorId { get; set; }
        public JointRole Role { get; set; }
        public int WristIndex { get; set; }
        public float? MinAngleRadians { get; set; }
        public float? MaxAngleRadians { get; set; }
        public float ZeroOffsetRadians { get; set; }
    }
}
