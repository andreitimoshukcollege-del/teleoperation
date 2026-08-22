using System;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;
using Teleop.RobotArm.Types;

namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// Loads a <see cref="RobotArmProfile"/> from a JSON file (<c>core/RobotProfiles/*.json</c>
    /// convention, docs/adr/0011-generic-robot-arm-profiles.md). <see cref="RobotArmProfile"/>
    /// itself can't carry this logic -- it lives in <c>Teleop.RobotArm</c>, which is compiled by
    /// Unity too and stays at zero NuGet packages / no file I/O, the same discipline
    /// <c>Teleop.Core</c> holds itself to. Deliberately not shared with <c>Teleop.Eval</c>'s own
    /// copy (<c>BuildProfile/RobotArmProfileJson.cs</c>) -- each host process owning a small,
    /// independent copy of a utility this thin is this codebase's established precedent (see
    /// <c>MonotonicClock</c>, duplicated per-host for the same reason: each is a separate
    /// deployable, and a shared dependency edge between two sibling host processes that don't
    /// otherwise need each other isn't worth avoiding ~40 lines of duplication for.
    /// </summary>
    public static class RobotArmProfileJson
    {
        public static RobotArmProfile Load(string path)
        {
            string json = File.ReadAllText(path);
            var options = new JsonSerializerOptions { PropertyNameCaseInsensitive = true };
            options.Converters.Add(new JsonStringEnumConverter());
            RobotArmProfileFile? file = JsonSerializer.Deserialize<RobotArmProfileFile>(json, options);
            if (file is null)
            {
                throw new InvalidDataException($"Robot arm profile at '{path}' deserialized to null.");
            }

            RobotArmProfile profile = file.ToProfile();
            string? validationError = profile.Validate();
            if (validationError != null)
            {
                throw new InvalidDataException($"Robot arm profile at '{path}' is invalid: {validationError}");
            }

            return profile;
        }
    }

    /// <summary>Plain, JSON-friendly mirror of <see cref="RobotArmProfile"/> -- a constructor-only readonly struct isn't a convenient deserialization target, so this DTO exists purely to load one.</summary>
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
                    motorId: j.MotorId, role: j.Role, wristIndex: j.WristIndex,
                    minAngleRadians: j.MinAngleRadians, maxAngleRadians: j.MaxAngleRadians,
                    zeroOffsetRadians: j.ZeroOffsetRadians);
            }

            return new RobotArmProfile(
                Name, HasRotatingBase, BaseHeight, ProximalLinkLength, DistalLinkLength,
                WristJointCount, HasGripper, GripperCanRotate, joints);
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
