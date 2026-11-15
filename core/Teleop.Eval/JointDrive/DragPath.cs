using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// The arm driven as an operator drives it: a point dragged along a continuous path, not moves between poses. On the headset
    /// the hand drags the clutch's target (half the hand's motion, JetRoverArmConfig.ClutchScale), and every frame the IK turns
    /// where it is now into joint targets, so the robot is sent a dense list of targets along a path. Here the dragged point is
    /// the wrist: strokes through a few points on a smooth curve (Catmull-Rom), each timed by a hand's speed profile (minimum
    /// jerk, or easing in and keeping its speed), a small tremor on top, and one solve of the headset's IK
    /// (<see cref="ArmSolver"/>) every command tick from the pose before it, exactly as the headset solves.
    /// </summary>
    public static class DragPath
    {
        /// <summary>The headset's command rate: one target a tick.</summary>
        public const double TickSeconds = 1.0 / 48.0;

        /// <summary>A stroke's peak wrist speed, m/s: a hand's 6-30 cm/s at the clutch's half scale.</summary>
        public const float MinStrokeSpeed = 0.03f, MaxStrokeSpeed = 0.15f;

        /// <summary>The tremor's amplitude on the wrist, metres (a hand's ~1 mm, halved by the clutch), at 8-12 Hz.</summary>
        public const float TremorMeters = 0.0005f;

        /// <summary>How close the solved wrist must end to the stroke's last point, metres: the IK caught up.</summary>
        public const float EndToleranceMeters = 0.004f;

        private const float CurveStepMeters = 0.001f;

        /// <summary>How a stroke's speed runs: a hand moving to a stop (minimum jerk), or easing in and keeping its speed to the end.</summary>
        public enum Profile
        {
            MinimumJerk,
            EaseIn,
        }

        /// <summary>Where the wrist is at <paramref name="pose"/> (the IK's target point), arm base frame.</summary>
        public static Vector3 Wrist(in RobotArmProfile profile, in ArmPose pose) => ArmKinematics.Forward(profile, pose.BaseYaw, pose.Proximal, pose.Distal);

        /// <summary>
        /// The wrist's position every tick along a smooth curve through <paramref name="points"/> (the first is where it starts and
        /// is not repeated; the last is where it ends), timed by <paramref name="profile"/> at a peak speed of
        /// <paramref name="peak"/> m/s. With <paramref name="tremorSeed"/> not 0, a tremor rides on it (and fades at the end, so
        /// the stroke ends where it was meant to).
        /// </summary>
        public static List<Vector3> Stroke(IReadOnlyList<Vector3> points, float peak, Profile profile, ulong tremorSeed = 0)
        {
            if (points.Count < 2) throw new ArgumentException("a stroke needs a start and an end", nameof(points));
            if (!(peak > 0f)) throw new ArgumentOutOfRangeException(nameof(peak));

            // The curve, densely: Catmull-Rom through every point (the ends doubled), then its length point by point.
            var curve = new List<Vector3> { points[0] };
            for (int i = 0; i < points.Count - 1; i++)
            {
                Vector3 p0 = points[Math.Max(0, i - 1)], p1 = points[i], p2 = points[i + 1], p3 = points[Math.Min(points.Count - 1, i + 2)];
                int n = Math.Max(1, (int)MathF.Ceiling(Vector3.Distance(p1, p2) / CurveStepMeters));
                for (int k = 1; k <= n; k++) curve.Add(CatmullRom(p0, p1, p2, p3, k / (float)n));
            }

            var length = new double[curve.Count];
            for (int i = 1; i < curve.Count; i++) length[i] = length[i - 1] + Vector3.Distance(curve[i - 1], curve[i]);
            double total = length[^1];
            var out_ = new List<Vector3>();
            if (total < 1e-6)
            {
                out_.Add(points[^1]);
                return out_;
            }

            // Minimum jerk peaks at 15/8 of its mean speed; easing in (a smooth start, then the peak to the end) at 1/0.85.
            double seconds = profile == Profile.MinimumJerk ? 1.875 * total / peak : total / peak / 0.85;
            int ticks = Math.Max(1, (int)Math.Ceiling(seconds / TickSeconds));
            var rng = new SeededRng(tremorSeed == 0 ? 1 : tremorSeed);
            double f1 = 8 + 4 * rng.NextDouble(), f2 = 8 + 4 * rng.NextDouble(), ph1 = rng.NextDouble() * 2 * Math.PI, ph2 = rng.NextDouble() * 2 * Math.PI;
            var axis1 = Vector3.Normalize(new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f) + new Vector3(1e-3f));
            var axis2 = Vector3.Normalize(new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f) + new Vector3(1e-3f));
            int j = 0;
            for (int k = 1; k <= ticks; k++)
            {
                double u = k / (double)ticks;
                double s = (profile == Profile.MinimumJerk ? u * u * u * (10 - u * (15 - 6 * u)) : EaseIn(u)) * total;
                while (j < curve.Count - 2 && length[j + 1] < s) j++;
                double seg = length[j + 1] - length[j];
                float a = seg > 0 ? (float)((s - length[j]) / seg) : 0f;
                Vector3 p = Vector3.Lerp(curve[j], curve[j + 1], Math.Clamp(a, 0f, 1f));
                if (tremorSeed != 0 && k < ticks)
                {
                    double tt = k * TickSeconds, fade = Math.Min(1.0, (ticks - k) / 12.0);
                    p += (float)(TremorMeters * fade * Math.Sin(2 * Math.PI * f1 * tt + ph1)) * axis1
                       + (float)(TremorMeters * 0.6 * fade * Math.Sin(2 * Math.PI * f2 * tt + ph2)) * axis2;
                }

                out_.Add(p);
            }

            out_[^1] = points[^1];
            return out_;
        }

        /// <summary>
        /// The headset's IK along <paramref name="wrist"/>, one solve a tick from <paramref name="from"/>: the pose sent each
        /// tick. The hand's pitch turns from <paramref name="from"/>'s to <paramref name="pitch"/> over the stroke, smoothly, as
        /// a hand tilts while it moves (asked for at once, the solver swings the wrist up to 30 degrees a tick). False when a
        /// pose leaves the envelope or fails <paramref name="model"/>, the solver is blocked or out of reach, or the wrist does
        /// not end within <see cref="EndToleranceMeters"/> of the last point (a few extra ticks of holding the last point are
        /// allowed for it to catch up, as the headset's would).
        /// </summary>
        public static bool Follow(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, in ArmPose from,
            IReadOnlyList<Vector3> wrist, float pitch, out List<ArmPose> poses)
        {
            poses = new List<ArmPose>(wrist.Count + 8);
            var solver = new ArmSolver();
            solver.Reset(profile, from);
            ArmSolution s = default;
            float startPitch = PitchOf(from);
            for (int i = 0; i < wrist.Count + 24; i++)
            {
                Vector3 target = wrist[Math.Min(i, wrist.Count - 1)];
                float u = Math.Min(1f, (i + 1) / (float)wrist.Count);
                float p = startPitch + (pitch - startPitch) * (u * u * (3f - 2f * u));
                s = solver.Solve(profile, target, p, 0f);
                if (s.Status == ArmSolveStatus.Blocked || s.ReachClamped) return false;
                if (!envelope.Contains(s.Pose) || !ArmCollision.IsValid(profile, model, s.Pose)) return false;
                poses.Add(s.Pose);
                if (i >= wrist.Count - 1 && Vector3.Distance(s.Position, wrist[^1]) <= EndToleranceMeters) return true;
            }

            return false;
        }

        /// <summary>The wrist's absolute pitch at <paramref name="pose"/>, as the IK counts it (proximal + distal + wrist).</summary>
        public static float PitchOf(in ArmPose pose) => pose.Proximal + pose.Distal + pose.Wrist;

        /// <summary>Appends one linear leg a tick through <paramref name="poses"/>, from <paramref name="from"/>: the commands are these poses, one a tick.</summary>
        public static void AddTicks(List<DriveLeg> legs, ref double t, in ArmPose from, IReadOnlyList<ArmPose> poses)
        {
            ArmPose previous = from;
            foreach (ArmPose p in poses)
            {
                legs.Add(new DriveLeg(t, TickSeconds, 0, previous, p, LegPhase.Script, linear: true));
                t += TickSeconds;
                previous = p;
            }
        }

        /// <summary>A hand that lets go of the clutch: the arm holds where it is.</summary>
        public static void AddHold(List<DriveLeg> legs, ref double t, in ArmPose at, double seconds)
        {
            legs.Add(new DriveLeg(t, 0, seconds, at, at, LegPhase.Script));
            t += seconds;
        }

        /// <summary>The fraction of a stroke's length covered at the fraction <paramref name="u"/> of its time.</summary>
        public static double Shape(Profile profile, double u) => profile == Profile.MinimumJerk ? u * u * u * (10 - u * (15 - 6 * u)) : EaseIn(u);

        /// <summary>How long a stroke of <paramref name="meters"/> takes at a peak of <paramref name="peak"/> m/s.</summary>
        public static double Seconds(Profile profile, double meters, float peak) => profile == Profile.MinimumJerk ? 1.875 * meters / peak : meters / peak / 0.85;

        // Speed rising smoothly over the first 30% of the time, then the peak: position is its integral, normalized.
        private static double EaseIn(double u)
        {
            const double R = 0.3;
            double total = 1 - R / 2;
            if (u >= R) return (R / 2 + (u - R)) / total;
            double x = u / R;
            return R * (x * x * x - x * x * x * x / 2) / total; // the integral of 3x^2 - 2x^3 (smoothstep), times R
        }

        private static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            float t2 = t * t, t3 = t2 * t;
            return 0.5f * (2f * p1 + (-p0 + p2) * t + (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 + (-p0 + 3f * p1 - 3f * p2 + p3) * t3);
        }
    }
}
