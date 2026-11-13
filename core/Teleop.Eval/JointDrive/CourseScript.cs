using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Training;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// joint-drive's <c>course</c> script: the robot drives the headset's training course itself
    /// (docs/adr/0024), so scripted sessions have the shape of teleop ones while there is little teleop
    /// data. Each round is drawn by <see cref="TrainingCourse.TryDraw"/> (the course's own targets, traces
    /// and gripper goals) and solved by the headset's own IK (<see cref="ArmSolver"/>, from where the arm
    /// is), in turn REACH, TRACE, SHUTTLE, at the course's levels 1.0, 2.5, 3.8, 6.0 rad/s or varied, in an
    /// order that covers every task at every level (3 tasks and 5 levels: 15 rounds). Moves are a person's
    /// rather than a planner's: a reach overshoots or falls short and corrects, the peak speed is 50-130% of
    /// the level (the hand can outrun the slider), and holds vary.
    ///
    /// <para>Every leg goes through <see cref="DrivePlan.PathValid"/> (collision model and envelope), as every
    /// other script's do; a target whose IK pose or path fails is skipped. The gripper opens and closes for
    /// REACH targets that ask for it, but closes only to <see cref="GripperClosed"/> and never stays closed
    /// longer than <see cref="MaxClosedSeconds"/>: its pulses are uncalibrated and it has run hot (72 °C).</para>
    /// </summary>
    public static class CourseScript
    {
        public static readonly float[] Levels = { 1.0f, 2.5f, 3.8f, 6.0f };
        public const int LevelCount = 5; // the four, then varied
        public const float GripperOpen = 0f;
        public const float GripperClosed = 0.75f;
        public const double MaxClosedSeconds = 1.5;
        public const double GripperRampSeconds = 0.3;
        public const float MinHeightMeters = -0.05f; // the headset's ClutchMinHeightMeters
        public const int SegmentPoints = TrainingCourse.TracePoints;

        /// <summary>No course move asks a joint for more than this, rad/s: the headset slider's top (JointDriveCommand.MaxSpeedRadPerSec).</summary>
        public const float MaxPeakRadPerSec = 6f;

        /// <summary>A leg's peak joint speed: <paramref name="fraction"/> of the level, never above <see cref="MaxPeakRadPerSec"/>.</summary>
        private static float Peak(float speed, double fraction) => (float)Math.Min(MaxPeakRadPerSec, speed * fraction);

        /// <summary>
        /// Appends rounds from <paramref name="t"/> until <paramref name="end"/>, moving
        /// <paramref name="current"/> along. False, with the reason, if no round can be drawn at all.
        /// </summary>
        public static bool Append(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, ref ArmPose current,
            ref double t, double end, ref SeededRng rng, List<DriveLeg> legs, out string? error)
        {
            error = null;
            var course = new TrainingCourse(profile, MinHeightMeters);
            float gripper = GripperOpen;
            int round = 0, failed = 0;
            while (t < end)
            {
                TrainingTask task = (round % 3) switch { 0 => TrainingTask.Reach, 1 => TrainingTask.Trace, _ => TrainingTask.Shuttle };
                int level = round % LevelCount;
                round++;
                int before = legs.Count;
                bool ok = course.TryDraw(task, rng.NextUInt64()) && task switch
                {
                    TrainingTask.Reach => Reach(profile, model, envelope, course, level, ref current, ref t, ref gripper, ref rng, legs),
                    TrainingTask.Trace => Trace(profile, model, envelope, course, level, ref current, ref t, ref gripper, ref rng, legs),
                    _ => Shuttle(profile, model, envelope, course, level, ref current, ref t, ref gripper, ref rng, legs),
                };
                if (!ok || legs.Count == before)
                {
                    if (++failed > 30)
                    {
                        error = $"no course round could be driven from {DrivePlan.Describe(current)} in 30 tries";
                        return false;
                    }

                    continue;
                }

                failed = 0;
                // Between rounds, as the course's announcement: open the gripper and pause.
                double pause = 1.0 + 2.0 * rng.NextDouble();
                legs.Add(new DriveLeg(t, 0, pause, current, current, LegPhase.Script, float.NaN, gripper, GripperOpen));
                gripper = GripperOpen;
                t += pause;
            }

            return true;
        }

        private static float SpeedOf(int level, ref SeededRng rng) =>
            level < Levels.Length ? Levels[level] : Levels[(int)(rng.NextDouble() * Levels.Length) % Levels.Length];

        private static bool Reach(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, TrainingCourse course,
            int level, ref ArmPose current, ref double t, ref float gripper, ref SeededRng rng, List<DriveLeg> legs)
        {
            for (int i = 0; i < course.TargetCount; i++)
            {
                if (!TrySolve(profile, current, course.Target(i), Pitch(ref rng), Roll(envelope, ref rng), out ArmPose pose)) continue;
                if (!DrivePlan.PathValid(profile, model, envelope, current, pose)) continue;
                float speed = SpeedOf(level, ref rng);
                float goalGripper = course.Goal(i) switch
                {
                    GripperGoal.Open => GripperOpen,
                    GripperGoal.Closed => GripperClosed,
                    _ => GripperOpen, // a target that asks nothing opens a closed gripper: short closures only
                };
                // A person overshoots or falls short, then corrects.
                float reach = (float)(0.85 + 0.2 * rng.NextDouble());
                ArmPose first = ArmPose.Lerp(current, pose, reach);
                if (!DrivePlan.PathValid(profile, model, envelope, current, first) || !DrivePlan.PathValid(profile, model, envelope, first, pose))
                {
                    first = pose;
                }

                // On the way the gripper opens (or stays open); a target that asks for it closed closes it
                // on arrival, for the hold only, so it is never shut for long.
                float moving = goalGripper == GripperClosed ? GripperOpen : goalGripper;
                double d1 = DrivePlan.SmoothDuration(current.MaxJointDelta(first), Peak(speed, 0.5 + 0.8 * rng.NextDouble()));
                legs.Add(new DriveLeg(t, d1, 0, current, first, LegPhase.Script, speed, gripper, moving));
                t += d1;
                if (first.MaxJointDelta(pose) > 0.005f)
                {
                    double d2 = Math.Max(0.25 + 0.25 * rng.NextDouble(), DrivePlan.SmoothDuration(first.MaxJointDelta(pose), Peak(speed, 1.0)));
                    legs.Add(new DriveLeg(t, d2, 0, first, pose, LegPhase.Script, speed, moving, moving));
                    t += d2;
                }

                double hold = 0.4 + 0.8 * rng.NextDouble();
                if (goalGripper == GripperClosed) hold = Math.Min(hold, MaxClosedSeconds);
                legs.Add(new DriveLeg(t, 0, hold, pose, pose, LegPhase.Script, speed, moving, goalGripper));
                t += hold;
                gripper = goalGripper;
                current = pose;
            }

            return true;
        }

        private static bool Shuttle(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, TrainingCourse course,
            int level, ref ArmPose current, ref double t, ref float gripper, ref SeededRng rng, List<DriveLeg> legs)
        {
            if (course.TargetCount < 2) return false;
            float pitch = Pitch(ref rng), roll = Roll(envelope, ref rng);
            if (!TrySolve(profile, current, course.Target(0), pitch, roll, out ArmPose a)) return false;
            if (!TrySolve(profile, a, course.Target(1), pitch, roll, out ArmPose b)) return false;
            if (!DrivePlan.PathValid(profile, model, envelope, current, a) || !DrivePlan.PathValid(profile, model, envelope, a, b)) return false;
            float approachSpeed = SpeedOf(level, ref rng);
            double d0 = DrivePlan.SmoothDuration(current.MaxJointDelta(a), Peak(approachSpeed, 1.0));
            legs.Add(new DriveLeg(t, d0, 0.3, current, a, LegPhase.Script, approachSpeed, gripper, GripperOpen));
            t += d0 + 0.3;
            gripper = GripperOpen;
            double end = t + TrainingCourse.ShuttleRoundSeconds;
            ArmPose from = a, to = b;
            while (t < end)
            {
                float speed = SpeedOf(level, ref rng);
                double d = DrivePlan.SmoothDuration(from.MaxJointDelta(to), Peak(speed, 0.8 + 0.5 * rng.NextDouble()));
                double hold = 0.15 * rng.NextDouble();
                legs.Add(new DriveLeg(t, d, hold, from, to, LegPhase.Script, speed, gripper, gripper));
                t += d + hold;
                (from, to) = (to, from);
            }

            current = from;
            return true;
        }

        private static bool Trace(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, TrainingCourse course,
            int level, ref ArmPose current, ref double t, ref float gripper, ref SeededRng rng, List<DriveLeg> legs)
        {
            int n = course.PathCount;
            if (n < 3) return false;
            float pitch = Pitch(ref rng), roll = Roll(envelope, ref rng);
            var poses = new ArmPose[n];
            ArmPose at = current;
            for (int i = 0; i < n; i++)
            {
                if (!TrySolve(profile, at, course.PathPoint(i), pitch, roll, out poses[i])) return false;
                at = poses[i];
            }

            if (!DrivePlan.PathValid(profile, model, envelope, current, poses[0])) return false;
            for (int i = 0; i < n; i++)
            {
                if (!DrivePlan.PathValid(profile, model, envelope, poses[i], poses[(i + 1) % n])) return false;
            }

            float approachSpeed = SpeedOf(level, ref rng);
            double d0 = DrivePlan.SmoothDuration(current.MaxJointDelta(poses[0]), Peak(approachSpeed, 1.0));
            legs.Add(new DriveLeg(t, d0, 0.5, current, poses[0], LegPhase.Script, approachSpeed, gripper, GripperOpen));
            t += d0 + 0.5;
            gripper = GripperOpen;
            double end = t + TrainingCourse.TraceRoundSeconds;
            int k = 0;
            float speedNow = SpeedOf(level, ref rng);
            double nextChange = t + TrainingCourse.VariedMinSeconds;
            while (t < end)
            {
                if (level >= Levels.Length && t >= nextChange)
                {
                    speedNow = SpeedOf(level, ref rng);
                    nextChange = t + TrainingCourse.VariedMinSeconds + rng.NextDouble() * (TrainingCourse.VariedMaxSeconds - TrainingCourse.VariedMinSeconds);
                }

                Vector3 p = course.PathPoint(k % n), q = course.PathPoint((k + 1) % n);
                // At the bead's speed, unless the IK turns a joint fast there: never above 1.3 x the level.
                ArmPose a0 = poses[k % n], a1 = poses[(k + 1) % n];
                double d = Math.Max(Math.Max(0.01, Vector3.Distance(p, q) / TrainingCourse.BeadSpeed(speedNow)),
                    a0.MaxJointDelta(a1) / Peak(speedNow, 1.3));
                legs.Add(new DriveLeg(t, d, 0, poses[k % n], poses[(k + 1) % n], LegPhase.Script, speedNow, gripper, gripper, linear: true));
                t += d;
                k++;
            }

            current = poses[k % n];
            return true;
        }

        // A palm's pitch varies; the IK tilts it further when it must (docs/adr/0020).
        private static float Pitch(ref SeededRng rng) => (float)(-0.6 + 0.9 * rng.NextDouble());

        // The left hand's twist turns the wrist roll; kept well inside the envelope.
        private static float Roll(in DriveEnvelope envelope, ref SeededRng rng)
        {
            AngleRange r = envelope[4];
            float mid = (r.Min + r.Max) / 2f, half = Math.Min(0.6f, (r.Max - r.Min) / 2f);
            return mid + (float)((rng.NextDouble() * 2 - 1) * half);
        }

        /// <summary>The headset's IK from <paramref name="from"/> to the wrist <paramref name="target"/>, until it gets there.</summary>
        private static bool TrySolve(in RobotArmProfile profile, in ArmPose from, Vector3 target, float pitch, float roll, out ArmPose pose)
        {
            var solver = new ArmSolver();
            solver.Reset(profile, from);
            pose = from;
            for (int i = 0; i < TrainingTargetSampler.MaxSolvesPerCheck; i++)
            {
                ArmSolution s = solver.Solve(profile, target, pitch, roll);
                if (s.Status == ArmSolveStatus.Approaching) continue;
                pose = s.Pose;
                return (s.Status == ArmSolveStatus.Exact || s.Status == ArmSolveStatus.PitchAdjusted) && !s.ReachClamped &&
                       Vector3.Distance(s.Position, target) <= TrainingTargetSampler.ReachToleranceMeters;
            }

            return false;
        }
    }
}
