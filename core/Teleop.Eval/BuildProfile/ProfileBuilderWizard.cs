using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.BuildProfile
{
    /// <summary>
    /// The interactive prompt sequence behind <c>build-profile</c>
    /// (docs/adr/0011-generic-robot-arm-profiles.md): walks an operator through a robot's
    /// topology and writes a validated <see cref="RobotArmProfile"/>. Deliberately not a hardware
    /// scan -- physical dimensions can't be auto-detected, so every question is a guided prompt,
    /// not an inspection.
    ///
    /// Reads from an injected <see cref="TextReader"/>/writes to an injected
    /// <see cref="TextWriter"/> rather than <see cref="Console"/> directly, so a test can pipe a
    /// canned answer transcript through a <see cref="StringReader"/> and assert on the result --
    /// this repo's "an algorithm that cannot be evaluated headlessly does not count" ethos,
    /// extended to an interactive CLI tool.
    /// </summary>
    public sealed class ProfileBuilderWizard
    {
        private static readonly Regex NamePattern = new Regex("^[a-z0-9_-]+$", RegexOptions.Compiled);

        private readonly TextReader _input;
        private readonly TextWriter _output;

        public ProfileBuilderWizard(TextReader input, TextWriter output)
        {
            _input = input;
            _output = output;
        }

        /// <summary>Runs the full prompt sequence. Returns null if the operator declines the final confirmation, the topology exceeds the wire protocol's joint cap, or the input stream ends before the sequence completes.</summary>
        public RobotArmProfile? Run()
        {
            string? name = PromptString("Profile name (lowercase letters, digits, dashes, underscores): ", NamePattern.IsMatch);
            if (name is null)
            {
                return null;
            }

            bool? hasRotatingBase = PromptYesNo("Does the arm have a rotating base? (y/n): ");
            if (hasRotatingBase is null)
            {
                return null;
            }

            float? baseHeight = PromptFloat("Base/shoulder pivot height above the base-yaw axis, metres: ", v => v >= 0f);
            float? proximalLength = PromptFloat("Proximal (shoulder-to-elbow) link length, metres: ", v => v > 0f);
            float? distalLength = PromptFloat("Distal (elbow-to-wrist) link length, metres: ", v => v > 0f);
            if (baseHeight is null || proximalLength is null || distalLength is null)
            {
                return null;
            }

            int? wristJointCount = PromptInt(
                "Number of wrist (orientation-only) joints -- 0 = none, 1 = a single wrist pitch " +
                "(most common), 2+ = extra joints but only the first absorbs commanded pitch, the " +
                "rest are held at zero: ", v => v >= 0);
            if (wristJointCount is null)
            {
                return null;
            }

            bool? hasGripper = PromptYesNo("Does it have a gripper? (y/n): ");
            if (hasGripper is null)
            {
                return null;
            }

            bool gripperCanRotate = false;
            if (hasGripper.Value)
            {
                bool? canRotate = PromptYesNo("Does the gripper rotate independently? (y/n): ");
                if (canRotate is null)
                {
                    return null;
                }

                gripperCanRotate = canRotate.Value;
            }

            List<(JointRole Role, int WristIndex)> roles = BuildRoleOrder(
                hasRotatingBase.Value, wristJointCount.Value, hasGripper.Value, gripperCanRotate);

            if (roles.Count > JointCommandCodec.MaxJointsPerMessage)
            {
                _output.WriteLine(
                    $"This topology needs {roles.Count} joints, above the wire protocol's cap of " +
                    $"{JointCommandCodec.MaxJointsPerMessage}. Reduce the wrist joint count or contact " +
                    "a maintainer to raise the cap.");
                return null;
            }

            var joints = new List<JointHardwareSpec>();
            var usedMotorIds = new HashSet<byte>();
            foreach ((JointRole role, int wristIndex) in roles)
            {
                string roleLabel = DescribeRole(role, wristIndex);
                byte? motorId = PromptMotorId($"Motor id for {roleLabel} (0-255): ", usedMotorIds);
                if (motorId is null)
                {
                    return null;
                }

                usedMotorIds.Add(motorId.Value);

                float? minAngle = null;
                float? maxAngle = null;
                bool isGripperRole = role == JointRole.GripperMain || role == JointRole.GripperRotate;
                if (!isGripperRole)
                {
                    // Min and max are asked independently (not one combined "needs a limit"
                    // question) since a joint may genuinely need only one -- e.g. a mechanical
                    // collision floor with no corresponding ceiling.
                    bool? wantsMin = PromptYesNo($"Does {roleLabel} need a custom minimum angle limit, radians? (y/n): ");
                    if (wantsMin is null)
                    {
                        return null;
                    }

                    if (wantsMin.Value)
                    {
                        minAngle = PromptFloat("  Min angle, radians: ", _ => true);
                        if (minAngle is null)
                        {
                            return null;
                        }
                    }

                    bool? wantsMax = PromptYesNo($"Does {roleLabel} need a custom maximum angle limit, radians? (y/n): ");
                    if (wantsMax is null)
                    {
                        return null;
                    }

                    if (wantsMax.Value)
                    {
                        float minValue = minAngle ?? float.NegativeInfinity;
                        maxAngle = PromptFloat("  Max angle, radians: ", v => v > minValue);
                        if (maxAngle is null)
                        {
                            return null;
                        }
                    }
                }

                float zeroOffset = 0f;
                if (!isGripperRole)
                {
                    // A servo horn only mounts at discrete spline positions, so it may not land
                    // exactly on the kinematic model's assumed zero -- see
                    // JointHardwareSpec.ZeroOffsetRadians's own doc. Zero (no correction) unless
                    // this joint has already been calibrated against real hardware.
                    bool? wantsOffset = PromptYesNo(
                        $"Does {roleLabel} need a mounting zero-offset correction, radians (0 if unknown/uncalibrated)? (y/n): ");
                    if (wantsOffset is null)
                    {
                        return null;
                    }

                    if (wantsOffset.Value)
                    {
                        float? offset = PromptFloat("  Zero-offset, radians: ", _ => true);
                        if (offset is null)
                        {
                            return null;
                        }

                        zeroOffset = offset.Value;
                    }
                }

                joints.Add(new JointHardwareSpec(motorId.Value, role, wristIndex, minAngle, maxAngle, zeroOffset));
            }

            var profile = new RobotArmProfile(
                name, hasRotatingBase.Value, baseHeight.Value, proximalLength.Value, distalLength.Value,
                wristJointCount.Value, hasGripper.Value, gripperCanRotate, joints.ToArray());

            string? validationError = profile.Validate();
            if (validationError != null)
            {
                _output.WriteLine($"Internal error building profile: {validationError}");
                return null;
            }

            PrintSummary(profile);
            bool? confirmed = PromptYesNo("Write this profile? (y/n): ");
            return confirmed == true ? profile : null;
        }

        private static List<(JointRole Role, int WristIndex)> BuildRoleOrder(
            bool hasRotatingBase, int wristJointCount, bool hasGripper, bool gripperCanRotate)
        {
            var roles = new List<(JointRole, int)>();
            if (hasRotatingBase)
            {
                roles.Add((JointRole.BaseYaw, 0));
            }

            roles.Add((JointRole.Proximal, 0));
            roles.Add((JointRole.Distal, 0));

            for (int i = 0; i < wristJointCount; i++)
            {
                roles.Add((JointRole.Wrist, i));
            }

            if (hasGripper)
            {
                roles.Add((JointRole.GripperMain, 0));
                if (gripperCanRotate)
                {
                    roles.Add((JointRole.GripperRotate, 0));
                }
            }

            return roles;
        }

        private static string DescribeRole(JointRole role, int wristIndex) => role switch
        {
            JointRole.BaseYaw => "the rotating base",
            JointRole.Proximal => "the proximal (shoulder) joint",
            JointRole.Distal => "the distal (elbow) joint",
            JointRole.Wrist => $"wrist joint #{wristIndex}",
            JointRole.GripperMain => "the gripper",
            JointRole.GripperRotate => "the gripper's rotation actuator",
            _ => role.ToString(),
        };

        private void PrintSummary(RobotArmProfile profile)
        {
            _output.WriteLine($"--- {profile.Name} ({profile.JointCount} joints) ---");
            foreach (JointHardwareSpec joint in profile.Joints)
            {
                string limits = joint.MinAngleRadians.HasValue || joint.MaxAngleRadians.HasValue
                    ? $" [{joint.MinAngleRadians?.ToString("0.###") ?? "-inf"}, {joint.MaxAngleRadians?.ToString("0.###") ?? "+inf"}]"
                    : "";
                string offset = joint.ZeroOffsetRadians != 0f ? $" offset={joint.ZeroOffsetRadians:0.###}rad" : "";
                _output.WriteLine($"  motor {joint.MotorId}: {joint.Role}{(joint.Role == JointRole.Wrist ? $"#{joint.WristIndex}" : "")}{limits}{offset}");
            }
        }

        private string? PromptString(string prompt, Func<string, bool> isValid)
        {
            while (true)
            {
                _output.Write(prompt);
                string? line = _input.ReadLine();
                if (line is null)
                {
                    return null;
                }

                line = line.Trim();
                if (isValid(line))
                {
                    return line;
                }

                _output.WriteLine("Invalid input, try again.");
            }
        }

        private bool? PromptYesNo(string prompt)
        {
            while (true)
            {
                _output.Write(prompt);
                string? line = _input.ReadLine()?.Trim().ToLowerInvariant();
                if (line is null)
                {
                    return null;
                }

                if (line == "y" || line == "yes")
                {
                    return true;
                }

                if (line == "n" || line == "no")
                {
                    return false;
                }

                _output.WriteLine("Please answer y or n.");
            }
        }

        private float? PromptFloat(string prompt, Func<float, bool> isValid)
        {
            while (true)
            {
                _output.Write(prompt);
                string? line = _input.ReadLine();
                if (line is null)
                {
                    return null;
                }

                if (float.TryParse(line.Trim(), out float value) && isValid(value))
                {
                    return value;
                }

                _output.WriteLine("Invalid value, try again.");
            }
        }

        private int? PromptInt(string prompt, Func<int, bool> isValid)
        {
            while (true)
            {
                _output.Write(prompt);
                string? line = _input.ReadLine();
                if (line is null)
                {
                    return null;
                }

                if (int.TryParse(line.Trim(), out int value) && isValid(value))
                {
                    return value;
                }

                _output.WriteLine("Invalid value, try again.");
            }
        }

        private byte? PromptMotorId(string prompt, HashSet<byte> usedMotorIds)
        {
            while (true)
            {
                _output.Write(prompt);
                string? line = _input.ReadLine();
                if (line is null)
                {
                    return null;
                }

                if (byte.TryParse(line.Trim(), out byte value))
                {
                    if (usedMotorIds.Contains(value))
                    {
                        _output.WriteLine($"Motor id {value} is already used by another joint -- pick a different one.");
                        continue;
                    }

                    return value;
                }

                _output.WriteLine("Invalid motor id, must be an integer 0-255.");
            }
        }
    }
}
