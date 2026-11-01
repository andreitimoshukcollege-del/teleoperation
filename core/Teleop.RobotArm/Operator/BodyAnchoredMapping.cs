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

        /// <summary>
        /// Walking carries the body frame (<see cref="BodyAnchoredMapping.Follow"/>, docs/adr/0029 amendment 5): a head
        /// moving horizontally faster than this (m/s, smoothed over <see cref="WalkSpeedSeconds"/>) is walking, and the
        /// frame then moves with it, so the target stays where it was. Slower is standing, sway or a lean to reach, and
        /// the frame stays put, so that motion is the hand's. 0 turns it off.
        /// </summary>
        public double WalkSpeed { get; set; } = 0.35;

        /// <summary>The time constant of the head speed compared with <see cref="WalkSpeed"/> (s): tracking jitter is not a step.</summary>
        public double WalkSpeedSeconds { get; set; } = 0.1;

        /// <summary>How long after the head slows below <see cref="WalkSpeed"/> it still counts as walking (s): the steps that stop a walk.</summary>
        public double WalkLingerSeconds { get; set; } = 0.5;

        /// <summary>
        /// The head's motion not yet taken by the frame (the start of a walk, before its speed told) is taken over this
        /// long once walking (s), so after a walk the target is back where it was; when no walk follows, that motion is
        /// forgotten over <see cref="WalkForgetSeconds"/>.
        /// </summary>
        public double WalkCatchUpSeconds { get; set; } = 0.25;

        public double WalkForgetSeconds { get; set; } = 1.0;

        public string? Validate()
        {
            if (!(PositionScale > 0) || double.IsInfinity(PositionScale)) return $"PositionScale must be positive and finite ({PositionScale})";
            if (double.IsNaN(NeutralForward + NeutralSide + NeutralDown) || double.IsInfinity(NeutralForward + NeutralSide + NeutralDown)) return "the neutral hand must be finite";
            if (!(WalkSpeed >= 0) || double.IsInfinity(WalkSpeed)) return $"WalkSpeed must be 0 (off) or positive ({WalkSpeed})";
            if (!(WalkSpeedSeconds > 0 && WalkLingerSeconds >= 0 && WalkCatchUpSeconds > 0 && WalkForgetSeconds > 0) ||
                double.IsInfinity(WalkSpeedSeconds + WalkLingerSeconds + WalkCatchUpSeconds + WalkForgetSeconds)) return "the walking times must be positive and finite";
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
        private double _bodyX, _bodyY, _bodyZ, _bodyYaw;
        private double _headX, _headY, _headSpeed, _pendingX, _pendingY, _walking;

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
            _bodyYaw = Math.Abs(fx) + Math.Abs(fy) > 1e-9 ? Math.Atan2(fy, fx) : 0;
            _bodyX = head.X;
            _bodyY = head.Y;
            _bodyZ = head.Z;
            _headX = head.X;
            _headY = head.Y;
            _headSpeed = _pendingX = _pendingY = _walking = 0;
            SetBody();
            IsCentred = true;
        }

        /// <summary>
        /// The head now, every frame, <paramref name="seconds"/> after the last: the body frame walks with the operator
        /// (docs/adr/0029 amendment 5). Standing (the head slower than <see cref="BodyMappingOptions.WalkSpeed"/>), the
        /// frame stays put, so a lean or a reach is the hand's. Walking, and for <see cref="BodyMappingOptions.WalkLingerSeconds"/>
        /// after, the frame moves with the head's horizontal motion, and also takes the motion from before the walk's speed
        /// told, so the target ends where it was. Height and heading stay as <see cref="Recentre"/> set them. Before
        /// <see cref="Recentre"/>, nothing.
        /// </summary>
        public void Follow(in RigidTransform head, double seconds)
        {
            if (!IsCentred) return;
            double dx = head.X - _headX, dy = head.Y - _headY;
            _headX = head.X;
            _headY = head.Y;
            if (!(seconds > 0) || double.IsInfinity(seconds) || !(_options.WalkSpeed > 0)) return;
            double speed = Math.Sqrt(dx * dx + dy * dy) / seconds;
            _headSpeed += (1 - Math.Exp(-seconds / _options.WalkSpeedSeconds)) * (speed - _headSpeed);
            _walking = _headSpeed >= _options.WalkSpeed ? _options.WalkLingerSeconds + seconds : Math.Max(0, _walking - seconds);
            _pendingX += dx;
            _pendingY += dy;
            if (_walking > 0)
            {
                // Take this frame's motion, and what was left over, a share each frame: the walk's start catches up.
                double take = Math.Min(1.0, seconds / _options.WalkCatchUpSeconds);
                double tx = dx + take * (_pendingX - dx), ty = dy + take * (_pendingY - dy);
                _bodyX += tx;
                _bodyY += ty;
                _pendingX -= tx;
                _pendingY -= ty;
                SetBody();
            }
            else
            {
                // Standing: the motion is the hand's, and what no walk claimed is forgotten.
                double keep = Math.Exp(-seconds / _options.WalkForgetSeconds);
                _pendingX *= keep;
                _pendingY *= keep;
            }
        }

        /// <summary>The body frame's position in the world (its height and heading are the last recentre's).</summary>
        public RigidTransform Body => new RigidTransform(_bodyX, _bodyY, _bodyZ,
            Math.Cos(0.5 * _bodyYaw), 0, 0, Math.Sin(0.5 * _bodyYaw));

        private void SetBody()
        {
            RigidTransform heading = RigidTransform.Rotation(0, 0, 1, _bodyYaw);
            _bodyFromWorld = new RigidTransform(_bodyX, _bodyY, _bodyZ, heading.Qw, heading.Qx, heading.Qy, heading.Qz).Inverse();
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
