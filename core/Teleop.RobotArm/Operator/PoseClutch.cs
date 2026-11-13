using System;
using Teleop.RobotArm.Types;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>
    /// A six-degree-of-freedom clutch (docs/adr/0029 decision 3): while engaged, the tool's target follows the operator's
    /// hand <i>relative to where both were at engage</i>, position scaled, rotation one to one; while released, it holds.
    /// Re-engaging anywhere never jumps the arm, because the anchor is re-taken from the current hand and tool.
    ///
    /// <para>Everything is in the robot's root frame (Core convention: Z up, X forward), the frame
    /// <see cref="Kinematics.SerialChainSolver"/> solves in. The hand's rotation change is applied in that frame, on the
    /// left of the tool's rotation at engage: turning the hand about the vertical turns the gripper about the vertical,
    /// whatever its orientation. Unlike the JetRover's <see cref="Kinematics.ClutchMapping"/>, which drops yaw and roll
    /// for a five-joint arm, every rotation comes through: a seven-joint arm can follow it.</para>
    ///
    /// <para>The target is not projected onto the workspace here: the solver reaches as close as it can, and the host
    /// reports how far it fell short. A plain struct: no allocations, no clock, no I/O.</para>
    /// </summary>
    public struct PoseClutch
    {
        private RigidTransform _handAtEngage;
        private RigidTransform _toolAtEngage;

        /// <summary>Hand motion → tool motion. 0.5 means 10 cm of hand moves the tool 5 cm.</summary>
        public double PositionScale { get; }

        public bool IsEngaged { get; private set; }

        /// <summary>The current target: where the hand asks the tool to be, held while released.</summary>
        public RigidTransform Target { get; private set; }

        public PoseClutch(double positionScale, in RigidTransform initialTarget)
        {
            if (!(positionScale > 0) || double.IsInfinity(positionScale))
            {
                throw new ArgumentOutOfRangeException(nameof(positionScale), positionScale, "the position scale must be a positive, finite number");
            }

            PositionScale = positionScale;
            Target = initialTarget;
            IsEngaged = false;
            _handAtEngage = RigidTransform.Identity;
            _toolAtEngage = RigidTransform.Identity;
        }

        /// <summary>Starts following the hand from where it is now, with the tool where it is now.</summary>
        public void Engage(in RigidTransform hand, in RigidTransform tool)
        {
            _handAtEngage = hand;
            _toolAtEngage = tool;
            Target = tool;
            IsEngaged = true;
        }

        /// <summary>Moves the target with the hand. Does nothing while released.</summary>
        public void Follow(in RigidTransform hand)
        {
            if (!IsEngaged)
            {
                return;
            }

            double x = _toolAtEngage.X + PositionScale * (hand.X - _handAtEngage.X);
            double y = _toolAtEngage.Y + PositionScale * (hand.Y - _handAtEngage.Y);
            double z = _toolAtEngage.Z + PositionScale * (hand.Z - _handAtEngage.Z);

            // The hand's rotation since engage, in the root frame: hand · handAtEngage⁻¹, then onto the tool's.
            RigidTransform turn = RotationOf(hand).Compose(RotationOf(_handAtEngage).Inverse());
            RigidTransform r = turn.Compose(RotationOf(_toolAtEngage));
            // A tracked hand's rotation is not exactly unit; the target's must be.
            double n = Math.Sqrt(r.Qw * r.Qw + r.Qx * r.Qx + r.Qy * r.Qy + r.Qz * r.Qz);
            if (!(n > 1e-9) || double.IsNaN(x + y + z))
            {
                return;
            }

            Target = new RigidTransform(x, y, z, r.Qw / n, r.Qx / n, r.Qy / n, r.Qz / n);
        }

        /// <summary>
        /// Sets the target outright while engaged: the absolute mapping (docs/adr/0029 amendment), where the target is
        /// where the hand maps to rather than how far it has moved. Does nothing while released.
        /// </summary>
        public void Track(in RigidTransform target)
        {
            if (IsEngaged) Target = target;
        }

        /// <summary>Stops following; the target holds where it is.</summary>
        public void Release() => IsEngaged = false;

        /// <summary>Sets the held target, for when the arm ends up somewhere else than asked (stopped short, blocked).</summary>
        public void Hold(in RigidTransform target)
        {
            if (!IsEngaged)
            {
                Target = target;
            }
        }

        private static RigidTransform RotationOf(in RigidTransform t) => new RigidTransform(0, 0, 0, t.Qw, t.Qx, t.Qy, t.Qz);
    }
}
