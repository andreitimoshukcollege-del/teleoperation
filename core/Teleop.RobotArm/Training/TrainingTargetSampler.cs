using System;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Training
{
    /// <summary>
    /// Draws course targets the arm can reach (docs/adr/0024 decision 3): joint angles inside the
    /// collision model's ranges less a margin, the wrist position from the forward kinematics, kept only
    /// if the headset's own <see cref="ArmSolver"/> gets the wrist there exactly (or with the pitch
    /// tilted) and the point is above the clutch's floor. Positions are in the arm's base frame (Core's
    /// ROS convention: X forward, Z up, metres). Plain C#: draws come from the caller's
    /// <see cref="SeededRng"/>, so a course is reproducible from its seed.
    /// </summary>
    public sealed class TrainingTargetSampler
    {
        /// <summary>How far inside each joint range a drawn angle stays.</summary>
        public const float JointMarginRad = 12f * MathF.PI / 180f;

        /// <summary>Base yaw is drawn within this of straight ahead: targets stay in front of the operator.</summary>
        public const float MaxYawRad = 75f * MathF.PI / 180f;

        /// <summary>How far above the clutch's floor a target must be.</summary>
        public const float FloorClearanceMeters = 0.02f;

        /// <summary>A target nearer the yaw axis than this (horizontally) is refused: it sits in the arm's own column.</summary>
        public const float MinRadiusMeters = 0.06f;

        /// <summary>How close the solved wrist must come to a target for it to count as reached.</summary>
        public const float ReachToleranceMeters = 0.003f;

        /// <summary>How many solves a check allows: one solve moves a joint at most <see cref="ArmSolver.MaxJumpPerSolve"/>.</summary>
        public const int MaxSolvesPerCheck = 40;

        private readonly RobotArmProfile _profile;
        private readonly float _minHeight;

        /// <param name="minHeightMeters">The clutch's floor, the profile's Z (the headset's <c>ClutchMinHeightMeters</c>).</param>
        public TrainingTargetSampler(RobotArmProfile profile, float minHeightMeters)
        {
            if (profile.Collision == null)
            {
                throw new ArgumentException("the course needs a profile with a collision model (docs/adr/0020)", nameof(profile));
            }

            _profile = profile;
            _minHeight = minHeightMeters;
        }

        public RobotArmProfile Profile => _profile;

        /// <summary>The lowest a target may be: the clutch's floor plus <see cref="FloorClearanceMeters"/>.</summary>
        public float MinTargetHeight => _minHeight + FloorClearanceMeters;

        /// <summary>
        /// Whether the arm reaches <paramref name="target"/> from the model's home pose, as the headset's
        /// IK would, and the point is above the floor and outside the arm's own column.
        /// </summary>
        public bool IsReachable(Vector3 target)
        {
            if (target.Z < MinTargetHeight || MathF.Sqrt(target.X * target.X + target.Y * target.Y) < MinRadiusMeters)
            {
                return false;
            }

            var solver = new ArmSolver();
            solver.Reset(_profile, _profile.Collision!.Home);
            return Converges(ref solver, target);
        }

        /// <summary>
        /// Whether the arm can follow <paramref name="points"/> in order, each from where the last left
        /// it (so a curve is checked the way an operator would drive along it), starting from home.
        /// </summary>
        public bool IsFollowable(ReadOnlySpan<Vector3> points)
        {
            var solver = new ArmSolver();
            solver.Reset(_profile, _profile.Collision!.Home);
            for (int i = 0; i < points.Length; i++)
            {
                Vector3 p = points[i];
                if (p.Z < MinTargetHeight || MathF.Sqrt(p.X * p.X + p.Y * p.Y) < MinRadiusMeters || !Converges(ref solver, p))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Draws a reachable target at least <paramref name="minSpacing"/> from <paramref name="avoid"/>
        /// (pass <c>null</c> for none). False after <paramref name="attempts"/> refused draws.
        /// </summary>
        public bool TrySample(ref SeededRng rng, out Vector3 target, Vector3? avoid = null, float minSpacing = 0f, int attempts = 400)
        {
            ArmCollisionModel model = _profile.Collision!;
            for (int i = 0; i < attempts; i++)
            {
                float yaw = Draw(ref rng, Shrink(model.BaseYaw), -MaxYawRad, MaxYawRad);
                float proximal = Draw(ref rng, Shrink(model.Proximal), float.NegativeInfinity, float.PositiveInfinity);
                float distal = Draw(ref rng, Shrink(model.Distal), float.NegativeInfinity, float.PositiveInfinity);
                Vector3 p = ArmKinematics.Forward(_profile, yaw, proximal, distal);
                if (avoid.HasValue && Vector3.Distance(p, avoid.Value) < minSpacing)
                {
                    continue;
                }

                if (IsReachable(p))
                {
                    target = p;
                    return true;
                }
            }

            target = default;
            return false;
        }

        private bool Converges(ref ArmSolver solver, Vector3 target)
        {
            for (int i = 0; i < MaxSolvesPerCheck; i++)
            {
                ArmSolution s = solver.Solve(_profile, target, 0f, 0f);
                if (s.Status == ArmSolveStatus.Approaching)
                {
                    continue;
                }

                return (s.Status == ArmSolveStatus.Exact || s.Status == ArmSolveStatus.PitchAdjusted) && !s.ReachClamped &&
                       Vector3.Distance(s.Position, target) <= ReachToleranceMeters;
            }

            return false;
        }

        private static AngleRange Shrink(AngleRange r)
        {
            float min = r.Min + JointMarginRad, max = r.Max - JointMarginRad;
            return min < max ? new AngleRange(min, max) : new AngleRange((r.Min + r.Max) / 2f, (r.Min + r.Max) / 2f);
        }

        private static float Draw(ref SeededRng rng, AngleRange r, float lo, float hi)
        {
            float min = MathF.Max(r.Min, lo), max = MathF.Min(r.Max, hi);
            if (max < min) min = max = MathF.Max(MathF.Min((r.Min + r.Max) / 2f, hi), lo);
            return min + (float)rng.NextDouble() * (max - min);
        }
    }
}
