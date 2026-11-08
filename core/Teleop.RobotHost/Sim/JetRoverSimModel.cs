using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotHost.Plant;

namespace Teleop.RobotHost.Sim
{
    /// <summary>
    /// The JetRover's arm as Isaac Sim simulates it (docs/adr/0034 decision 2), made from the robot's own descriptions so the
    /// simulated arm is the arm the IK and RobotHost reason about:
    /// <list type="bullet">
    /// <item><b>Kinematics</b> from the profile (<c>core/RobotProfiles/jetrover.json</c>): the yaw axis at the arm base frame's
    /// origin, the shoulder <see cref="RobotArmProfile.BaseHeight"/> above it, two links of the profile's lengths, one wrist
    /// pitch, the gripper's roll about the wrist's x. Each pitch joint turns about −y, so a positive angle raises the link, and
    /// at 0 a link points along +x: <c>ArmKinematics.Forward</c>'s convention.</item>
    /// <item><b>Collision shapes</b> from the collision model the IK and RobotHost already check against
    /// (<see cref="JetRoverCollisionModel"/>, fitted to the CAD): every link body a box in its link's frame, the chassis's
    /// boxes on the fixed base. The gripper's two jaw bodies become fingers on prismatic joints.</item>
    /// <item><b>Joint ranges</b> from the same model; <b>velocity limits</b> from the plant's fastest pulse rate.</item>
    /// <item><b>Mass</b> per link from what it carries (<see cref="LinkMassKg"/>: a bus servo is about 60 g, plus brackets), and
    /// <b>inertia</b> from that mass spread over the link's boxes by volume. The boxes alone would overstate it: they overlap
    /// and are padded for the collision check. First guesses, fitted later (docs/adr/0034 decision 6).</item>
    /// <item><b>Effort limits</b> from Hiwonder's bus servos' stall torque (<see cref="StallTorqueNm"/>): first guesses too.</item>
    /// </list>
    /// <see cref="SimJson"/> is its sidecar: how the bridge turns the relay's pulses into joint angles and back, exactly as
    /// <see cref="GenericArmPlant"/> does, which motors it may report, and when. A test holds both files to what this makes.
    /// Pure: no file is read or written here.
    /// </summary>
    public static class JetRoverSimModel
    {
        /// <summary>
        /// Each link's mass (kg): the servo it carries and its bracket. The yaw servo rides the chassis, the shoulder's servo the
        /// turntable, the elbow's the proximal link, the wrist pitch's the distal link, the roll's the wrist, the gripper's the
        /// gripper; about 0.7 kg in all, as a 5-servo arm of 13 cm links weighs.
        /// </summary>
        public static readonly IReadOnlyDictionary<string, double> LinkMassKg = new Dictionary<string, double>
        {
            ["turntable"] = 0.15, ["proximal_link"] = 0.12, ["distal_link"] = 0.12, ["wrist_link"] = 0.10, ["gripper_link"] = 0.12,
            ["left_finger"] = 0.01, ["right_finger"] = 0.01,
        };

        /// <summary>The chassis's mass (kg); it is pinned to the world, so it only matters to the contacts.</summary>
        public const double ChassisMass = 3.0;

        /// <summary>Each finger's travel from open to closed (m): the jaw bodies' gap, less a margin, halved.</summary>
        public const double FingerTravelMeters = 0.011;

        /// <summary>
        /// Stall torque (N·m) per joint: Hiwonder's 35 kg·cm bus servos on the yaw, shoulder, elbow and wrist pitch, its smaller
        /// ones on the roll and the gripper (first guesses from the product pages; docs/adr/0034 decision 3).
        /// </summary>
        public static readonly IReadOnlyDictionary<string, double> StallTorqueNm = new Dictionary<string, double>
        {
            ["base_yaw"] = 3.4, ["proximal"] = 3.4, ["distal"] = 3.4, ["wrist"] = 3.4, ["roll"] = 1.5, ["gripper"] = 1.5,
        };

        /// <summary>How often the real relay sends feedback (s): <c>relay_node.py</c>'s <c>_FEEDBACK_SEND_PERIOD_SECONDS</c>, 20 ms since 2026-10-25.</summary>
        public const double FeedbackPeriodSeconds = 0.02;

        /// <summary>The real driver's feedback cycle (s, <c>robot_controller_node.py</c>'s <c>pollRate</c>): motors 1 to 5 read back to back every 20 ms.</summary>
        public const double DriverCycleSeconds = 0.02;

        /// <summary>One bus position read (s): the trainer's fitted 2.9 ms (<c>armpred/sim/params.py</c>).</summary>
        public const double ReadSeconds = 0.0029;

        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        private sealed class LinkSpec
        {
            public string Name = "";
            public readonly List<(string Name, Vector3 Center, Quaternion Rotation, Vector3 Size)> Boxes = new List<(string, Vector3, Quaternion, Vector3)>();
            public double Mass;
        }

        /// <summary>The URDF, from the profile, the collision model and the plant's pulse rate.</summary>
        public static string Urdf(RobotArmProfile profile, ArmCollisionModel collision, GenericArmPlantConfig plant)
        {
            if (profile.WristJointCount != 1 || !profile.HasRotatingBase || !profile.HasGripper || !profile.GripperCanRotate)
                throw new ArgumentException("the JetRover's model: a rotating base, one wrist pitch, a rotating gripper");
            double maxRadPerSec = plant.PulsesPerSecond / plant.PulsePerRadian;
            var links = new Dictionary<ArmLink, LinkSpec>
            {
                [ArmLink.Turntable] = new LinkSpec { Name = "turntable" },
                [ArmLink.Proximal] = new LinkSpec { Name = "proximal_link" },
                [ArmLink.Distal] = new LinkSpec { Name = "distal_link" },
                [ArmLink.Wrist] = new LinkSpec { Name = "wrist_link" },
                [ArmLink.Gripper] = new LinkSpec { Name = "gripper_link" },
            };
            var left = new LinkSpec { Name = "left_finger" };
            var right = new LinkSpec { Name = "right_finger" };
            foreach (CollisionBody b in collision.Bodies)
            {
                var box = (b.Name, b.Center, b.Rotation, b.HalfExtents * 2);
                if (b.Name == "gripper.1") left.Boxes.Add(box);
                else if (b.Name == "gripper.2") right.Boxes.Add(box);
                else links[b.Link].Boxes.Add(box);
            }

            var sb = new StringBuilder();
            sb.Append("<?xml version=\"1.0\"?>\n");
            sb.Append("<!-- Generated by Teleop.RobotHost's JetRoverSimModel (docs/adr/0034 decision 2) from core/RobotProfiles/jetrover.json,\n");
            sb.Append("     JetRoverCollisionModel (" + collision.Source.Split('|')[0].Trim() + ") and the plant's pulse rate. Do not edit:\n");
            sb.Append("     JetRoverSimModelTests rewrites it (TELEOP_WRITE_GOLDEN=1). -->\n");
            sb.Append("<robot name=\"jetrover\">\n");

            // The chassis: the collision model's boxes, axis-aligned in the arm base frame.
            sb.Append("  <link name=\"base_link\">\n");
            Vector3 chassisCentre = Vector3.Zero;
            double chassisVolume = 0;
            foreach (CollisionBox box in collision.Boxes)
            {
                Vector3 size = box.Max - box.Min, centre = (box.Max + box.Min) / 2;
                double v = size.X * size.Y * size.Z;
                chassisCentre += centre * (float)v;
                chassisVolume += v;
            }

            chassisCentre /= (float)chassisVolume;
            sb.Append(Inertial(ChassisMass, chassisCentre, new Vector3(0.02f, 0.02f, 0.02f) * (float)ChassisMass));
            foreach (CollisionBox box in collision.Boxes)
            {
                Vector3 size = box.Max - box.Min, centre = (box.Max + box.Min) / 2;
                sb.Append(Geometry("collision", box.Name, centre, Quaternion.Identity, size));
            }

            sb.Append("  </link>\n");

            foreach (LinkSpec link in links.Values.Concat(new[] { left, right })) sb.Append(Link(link));

            float baseHeight = profile.BaseHeight, l1 = profile.ProximalLinkLength, l2 = profile.DistalLinkLength;
            sb.Append(Joint("base_yaw", "revolute", "base_link", "turntable", Vector3.Zero, new Vector3(0, 0, 1), collision.BaseYaw, maxRadPerSec));
            sb.Append(Joint("proximal", "revolute", "turntable", "proximal_link", new Vector3(0, 0, baseHeight), new Vector3(0, -1, 0), collision.Proximal, maxRadPerSec));
            sb.Append(Joint("distal", "revolute", "proximal_link", "distal_link", new Vector3(l1, 0, 0), new Vector3(0, -1, 0), collision.Distal, maxRadPerSec));
            sb.Append(Joint("wrist", "revolute", "distal_link", "wrist_link", new Vector3(l2, 0, 0), new Vector3(0, -1, 0), collision.Wrist, maxRadPerSec));
            sb.Append(Joint("roll", "revolute", "wrist_link", "gripper_link", Vector3.Zero, new Vector3(1, 0, 0), collision.Roll, maxRadPerSec));
            var travel = new AngleRange(0f, (float)FingerTravelMeters);
            sb.Append(Joint("gripper", "prismatic", "gripper_link", "left_finger", Vector3.Zero, new Vector3(0, -1, 0), travel, 0.1, "gripper"));
            sb.Append(Joint("gripper_mirror", "prismatic", "gripper_link", "right_finger", Vector3.Zero, new Vector3(0, 1, 0), travel, 0.1, "gripper"));
            sb.Append("</robot>\n");
            return sb.ToString();
        }

        /// <summary>The bridge's sidecar: the pulse conversion, each motor's joint, which motors report, and the servo limits.</summary>
        public static string SimJson(RobotArmProfile profile, GenericArmPlantConfig plant)
        {
            static string JointOf(JointRole role) => role switch
            {
                JointRole.BaseYaw => "base_yaw",
                JointRole.Proximal => "proximal",
                JointRole.Distal => "distal",
                JointRole.Wrist => "wrist",
                JointRole.GripperRotate => "roll",
                JointRole.GripperMain => "gripper",
                _ => throw new ArgumentException($"no sim joint for {role}"),
            };

            var motors = new JsonArray();
            foreach (JointHardwareSpec j in profile.Joints.OrderBy(j => j.MotorId))
            {
                var m = new JsonObject
                {
                    ["motorId"] = (int)j.MotorId,
                    ["joint"] = JointOf(j.Role),
                    ["zeroOffsetRadians"] = Math.Round(j.ZeroOffsetRadians, 9),
                    // The driver reads motors 1 to 5; the gripper (10) never answers a position read (ADR 0018).
                    ["feedback"] = j.Role != JointRole.GripperMain,
                    ["stallTorqueNm"] = StallTorqueNm[JointOf(j.Role)],
                };
                if (j.Role == JointRole.GripperMain)
                {
                    m["openPulse"] = plant.GripperOpenPulse;
                    m["closedPulse"] = plant.GripperClosedPulse;
                    m["fingerTravelMeters"] = FingerTravelMeters;
                    m["fingers"] = new JsonArray("gripper", "gripper_mirror");
                }

                motors.Add(m);
            }

            var doc = new JsonObject
            {
                ["format"] = "jetrover-sim",
                ["version"] = 1,
                ["about"] = "The sidecar of the Isaac JetRover bridge (docs/adr/0034), written by JetRoverSimModel: angle = (pulse - zeroPulse) / pulsePerRadian - zeroOffsetRadians, as GenericArmPlant converts.",
                ["urdf"] = "jetrover.urdf",
                ["zeroPulse"] = plant.ZeroPulse,
                ["pulsePerRadian"] = Math.Round(plant.PulsePerRadian, 6),
                ["minPulse"] = plant.MinPulse,
                ["maxPulse"] = plant.MaxPulse,
                ["defaultPulsesPerSecond"] = plant.PulsesPerSecond,
                ["feedbackPeriodSeconds"] = FeedbackPeriodSeconds,
                ["driverCycleSeconds"] = DriverCycleSeconds,
                ["readSeconds"] = ReadSeconds,
                ["motors"] = motors,
            };
            return doc.ToJsonString(new JsonSerializerOptions { WriteIndented = true }).Replace("\r\n", "\n") + "\n";
        }

        private static string Link(LinkSpec link)
        {
            double volume = link.Boxes.Sum(b => (double)b.Size.X * b.Size.Y * b.Size.Z);
            double mass = LinkMassKg[link.Name];
            Vector3 com = Vector3.Zero;
            foreach (var b in link.Boxes) com += b.Center * (float)(b.Size.X * b.Size.Y * b.Size.Z / volume);

            // Each box's inertia about its centre, turned into the link frame, moved to the link's centre of mass.
            Matrix4x4 inertia = default;
            foreach (var b in link.Boxes)
            {
                double m = mass * (b.Size.X * b.Size.Y * b.Size.Z) / volume;
                float ix = (float)(m / 12 * (b.Size.Y * b.Size.Y + b.Size.Z * b.Size.Z));
                float iy = (float)(m / 12 * (b.Size.X * b.Size.X + b.Size.Z * b.Size.Z));
                float iz = (float)(m / 12 * (b.Size.X * b.Size.X + b.Size.Y * b.Size.Y));
                Matrix4x4 r = Matrix4x4.CreateFromQuaternion(b.Rotation);
                var local = new Matrix4x4(ix, 0, 0, 0, 0, iy, 0, 0, 0, 0, iz, 0, 0, 0, 0, 0);
                Matrix4x4 turned = Matrix4x4.Transpose(r) * local * r; // System.Numerics is row-vector: R^T I R is R I R^T in column form
                Vector3 d = b.Center - com;
                float dd = d.LengthSquared();
                var shift = new Matrix4x4(
                    (float)m * (dd - d.X * d.X), (float)m * -d.X * d.Y, (float)m * -d.X * d.Z, 0,
                    (float)m * -d.Y * d.X, (float)m * (dd - d.Y * d.Y), (float)m * -d.Y * d.Z, 0,
                    (float)m * -d.Z * d.X, (float)m * -d.Z * d.Y, (float)m * (dd - d.Z * d.Z), 0,
                    0, 0, 0, 0);
                inertia += turned + shift;
            }

            var sb = new StringBuilder();
            sb.Append($"  <link name=\"{link.Name}\">\n");
            sb.Append($"    <inertial>\n      <origin xyz=\"{F(com.X)} {F(com.Y)} {F(com.Z)}\" rpy=\"0 0 0\"/>\n      <mass value=\"{F(mass)}\"/>\n");
            sb.Append($"      <inertia ixx=\"{F(Math.Max(1e-7, inertia.M11))}\" ixy=\"{F(inertia.M12)}\" ixz=\"{F(inertia.M13)}\" iyy=\"{F(Math.Max(1e-7, inertia.M22))}\" iyz=\"{F(inertia.M23)}\" izz=\"{F(Math.Max(1e-7, inertia.M33))}\"/>\n    </inertial>\n");
            foreach (var b in link.Boxes)
            {
                sb.Append(Geometry("visual", b.Name, b.Center, b.Rotation, b.Size));
                sb.Append(Geometry("collision", b.Name, b.Center, b.Rotation, b.Size));
            }

            sb.Append("  </link>\n");
            return sb.ToString();
        }

        private static string Inertial(double mass, Vector3 com, Vector3 diagonal) =>
            $"    <inertial>\n      <origin xyz=\"{F(com.X)} {F(com.Y)} {F(com.Z)}\" rpy=\"0 0 0\"/>\n      <mass value=\"{F(mass)}\"/>\n" +
            $"      <inertia ixx=\"{F(diagonal.X)}\" ixy=\"0\" ixz=\"0\" iyy=\"{F(diagonal.Y)}\" iyz=\"0\" izz=\"{F(diagonal.Z)}\"/>\n    </inertial>\n";

        private static string Geometry(string kind, string name, Vector3 centre, Quaternion rotation, Vector3 size)
        {
            Vector3 rpy = Rpy(rotation);
            return $"    <{kind} name=\"{name}\">\n      <origin xyz=\"{F(centre.X)} {F(centre.Y)} {F(centre.Z)}\" rpy=\"{F(rpy.X)} {F(rpy.Y)} {F(rpy.Z)}\"/>\n" +
                   $"      <geometry><box size=\"{F(size.X)} {F(size.Y)} {F(size.Z)}\"/></geometry>\n    </{kind}>\n";
        }

        private static string Joint(string name, string type, string parent, string child, Vector3 origin, Vector3 axis, AngleRange range, double velocity, string? effortKey = null)
        {
            double effort = StallTorqueNm[effortKey ?? name];
            if (type == "prismatic") effort = 20; // N: the gripper's servo through its linkage (a guess)
            return $"  <joint name=\"{name}\" type=\"{type}\">\n    <parent link=\"{parent}\"/>\n    <child link=\"{child}\"/>\n" +
                   $"    <origin xyz=\"{F(origin.X)} {F(origin.Y)} {F(origin.Z)}\" rpy=\"0 0 0\"/>\n    <axis xyz=\"{F(axis.X)} {F(axis.Y)} {F(axis.Z)}\"/>\n" +
                   $"    <limit lower=\"{F(range.Min)}\" upper=\"{F(range.Max)}\" effort=\"{F(effort)}\" velocity=\"{F(velocity)}\"/>\n  </joint>\n";
        }

        /// <summary>URDF's fixed-axis roll, pitch, yaw of a rotation (R = Rz(yaw) Ry(pitch) Rx(roll)).</summary>
        internal static Vector3 Rpy(Quaternion q)
        {
            q = Quaternion.Normalize(q);
            double sinp = 2 * (q.W * q.Y - q.Z * q.X);
            double pitch = Math.Abs(sinp) >= 1 ? Math.CopySign(Math.PI / 2, sinp) : Math.Asin(sinp);
            double roll = Math.Atan2(2 * (q.W * q.X + q.Y * q.Z), 1 - 2 * (q.X * q.X + q.Y * q.Y));
            double yaw = Math.Atan2(2 * (q.W * q.Z + q.X * q.Y), 1 - 2 * (q.Y * q.Y + q.Z * q.Z));
            return new Vector3((float)roll, (float)pitch, (float)yaw);
        }

        private static string F(double v) => Math.Round(v, 7).ToString("0.#######", Inv);
    }
}
