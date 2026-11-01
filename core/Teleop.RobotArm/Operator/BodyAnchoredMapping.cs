using System;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>The body-anchored mapping's settings (docs/adr/0029 amendment). Defaults are first guesses, tuned on the headset.</summary>
    public sealed class BodyMappingOptions
    {
        /// <summary>Hand motion → tool motion about the neutral pose; 1 is one to one.</summary>
        public double PositionScale { get; set; } = 1.0;

        /// <summary>The neutral hand, from the head at recentre, in metres: forward, to the side (each hand its own side) and down.</summary>
        public double NeutralForward { get; set; } = 0.30;
        public double NeutralSide { get; set; } = 0.20;
        public double NeutralDown { get; set; } = 0.40;

        public string? Validate()
        {
            if (!(PositionScale > 0) || double.IsInfinity(PositionScale)) return $"PositionScale must be positive and finite ({PositionScale})";
            if (double.IsNaN(NeutralForward + NeutralSide + NeutralDown) || double.IsInfinity(NeutralForward + NeutralSide + NeutralDown)) return "the neutral hand must be finite";
            return null;
        }
    }

    /// <summary>
    /// The robot anchored to the operator's body (docs/adr/0029 amendment, the user's choice over the relative clutch):
    /// the hand's pose relative to the body <i>is</i> the tool's pose relative to the robot, so a palm held flat and
    /// pointing forward points the gripper forward, and where the hand is decides where the gripper goes, not how far
    /// it has moved since a grab.
    /// <list type="bullet">
    /// <item><b>The body frame</b> is taken from the pose given to <see cref="Recentre"/>: its position, and its heading
    /// only (yaw about the vertical), so looking down or tilting the head changes nothing. The Galaxy XR passes the
    /// headset's position with the robot's heading (docs/adr/0029 amendment 4), so where the operator happened to look
    /// does not decide which way is forward.</item>
    /// <item><b>Position, per arm:</b> the neutral hand (<see cref="BodyMappingOptions"/>: forward, down, and to that
    /// hand's side) maps onto the arm's neutral tool; every centimetre from there, times
    /// <see cref="BodyMappingOptions.PositionScale"/>, is a centimetre of tool. Per arm, because the robot's shoulders
    /// are not a person's: the FR3 Duo's are 0.6 m apart.</item>
    /// <item><b>Orientation, absolute:</b> the tool's rotation is the hand's (in the body frame) times
    /// <see cref="ToolFromHand"/>. Core's view of an XR Hands palm is x toward the fingertips, y to the hand's left and
    /// z out of the back of the hand; the Franka tool is z along the approach and y along the fingers' travel. So the
    /// tool's z is the fingertips' direction, its y the hand's left, and a flat hand pointing forward is a gripper
    /// pointing forward with its fingers closing sideways, as a thumb and index do.</item>
    /// </list>
    /// Everything is Core's convention (Z up, X forward); the host converts the head and the hands once
    /// (<c>CoordConversion</c>). A plain class: no clock, no I/O, nothing allocated after construction.
    /// </summary>
    public sealed class BodyAnchoredMapping
    {
        /// <summary>Franka's ready pose (the arm upright, the gripper down in front); the right arm of a duo mirrors it.</summary>
        public static readonly double[] FrankaReady = { 0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4 };

        /// <summary>Tool from hand: +90° about y, so the hand's x (fingertips) becomes the tool's z (approach).</summary>
        public static readonly RigidTransform ToolFromHand = RigidTransform.Rotation(0, 1, 0, Math.PI / 2);

        private readonly BodyMappingOptions _options;
        private readonly RigidTransform[] _neutralTool;
        private readonly double[] _neutralSide;
        private RigidTransform _bodyFromWorld = RigidTransform.Identity;

        /// <param name="neutralTools">Each arm's tool at the neutral hand, root frame; <see cref="NeutralTools"/> gives the FR3 Duo's.</param>
        public BodyAnchoredMapping(SerialChainProfile profile, RigidTransform[] neutralTools, BodyMappingOptions? options = null)
        {
            if (profile == null) throw new ArgumentNullException(nameof(profile));
            if (neutralTools == null || neutralTools.Length != profile.Arms.Length) throw new ArgumentException("one neutral tool per arm", nameof(neutralTools));
            _options = options ?? new BodyMappingOptions();
            if (_options.Validate() is string bad) throw new ArgumentException(bad, nameof(options));
            _neutralTool = (RigidTransform[])neutralTools.Clone();
            _neutralSide = new double[profile.Arms.Length];
            for (int a = 0; a < profile.Arms.Length; a++)
            {
                // Each hand on its own arm's side: the arm whose base is on the robot's left (+Y) takes the left hand.
                _neutralSide[a] = profile.Arms[a].Mount.Y >= 0 ? _options.NeutralSide : -_options.NeutralSide;
            }
        }

        /// <summary>
        /// Each arm's neutral tool: <paramref name="forward"/> ahead of its shoulder (the first joint's origin) and
        /// <paramref name="up"/> above it, in the root frame, pointing forward (<see cref="ToolFromHand"/>). Not the ready
        /// pose's tool: that is 31 cm ahead of the shoulder, where a seven-joint arm can barely point its 21 cm hand forward.
        /// 55 cm ahead and 30 cm up is the middle of the region where hands within 10 cm of neutral are reached from the
        /// ready pose without folding the elbow into the upper arm: 53 of 54 such targets (45/20 reached 34, 65/30 49;
        /// BodyAnchoredMappingTests).
        /// </summary>
        public static RigidTransform[] NeutralTools(SerialChainProfile profile, double forward = 0.55, double up = 0.30)
        {
            var tools = new RigidTransform[profile.Arms.Length];
            var frames = new RigidTransform[16];
            RigidTransform r = ToolFromHand;
            for (int a = 0; a < profile.Arms.Length; a++)
            {
                ChainArm arm = profile.Arms[a];
                var zero = new double[arm.Joints.Length];
                SerialChainKinematics.ComputeFrames(arm, zero, frames.AsSpan(0, arm.Joints.Length + 1));
                RigidTransform shoulder = frames[0];
                tools[a] = new RigidTransform(shoulder.X + forward, shoulder.Y, shoulder.Z + up, r.Qw, r.Qx, r.Qy, r.Qz);
            }

            return tools;
        }

        public bool IsCentred { get; private set; }

        /// <summary>Takes the body frame from <paramref name="head"/> (world): its position, and its heading about the vertical.</summary>
        public void Recentre(in RigidTransform head)
        {
            head.Rotate(1, 0, 0, out double fx, out double fy, out _);
            double yaw = Math.Abs(fx) + Math.Abs(fy) > 1e-9 ? Math.Atan2(fy, fx) : 0;
            RigidTransform heading = RigidTransform.Rotation(0, 0, 1, yaw);
            var body = new RigidTransform(head.X, head.Y, head.Z, heading.Qw, heading.Qx, heading.Qy, heading.Qz);
            _bodyFromWorld = body.Inverse();
            IsCentred = true;
        }

        /// <summary>Where arm <paramref name="arm"/>'s tool goes (root frame) for a hand at <paramref name="hand"/> (world).</summary>
        public RigidTransform Map(int arm, in RigidTransform hand)
        {
            RigidTransform inBody = _bodyFromWorld.Compose(hand);
            double s = _options.PositionScale;
            RigidTransform n = _neutralTool[arm];
            double x = n.X + s * (inBody.X - _options.NeutralForward);
            double y = n.Y + s * (inBody.Y - _neutralSide[arm]);
            double z = n.Z + s * (inBody.Z + _options.NeutralDown);
            RigidTransform r = new RigidTransform(0, 0, 0, inBody.Qw, inBody.Qx, inBody.Qy, inBody.Qz).Compose(ToolFromHand);
            double norm = Math.Sqrt(r.Qw * r.Qw + r.Qx * r.Qx + r.Qy * r.Qy + r.Qz * r.Qz);
            if (!(norm > 1e-9)) norm = 1;
            return new RigidTransform(x, y, z, r.Qw / norm, r.Qx / norm, r.Qy / norm, r.Qz / norm);
        }
    }
}
