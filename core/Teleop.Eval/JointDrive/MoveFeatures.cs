using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Numerics;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// How each planned move of a mixed take moves (docs/adr/0038, "M, motion": a path-only model must not tell a reach at an object
    /// from a free move): <c>joint-drive --script mixed --dry-run --layouts a-b --features &lt;csv&gt;</c> writes one row a move, measured
    /// on the plan's commands at the command rate (48 Hz) from the take's start, the tool point (the pads' midpoint,
    /// <see cref="MixedTake.ToolInGripper"/>) smoothed with a 25 ms Gaussian as workstream R measured people. A move's span is its
    /// <c>MV</c> record's (it overlaps the next, as a hand's submovements do), clipped to the take. <c>scripts/gpu/move_features.py</c>
    /// reads it.
    /// <list type="bullet">
    /// <item><c>duration_s</c>, <c>path_m</c> and <c>chord_m</c>: the span, the tool's path length over it and its start-to-end
    /// distance.</item>
    /// <item><c>peak_speed</c>, <c>mean_speed</c> (path over span), m/s; <c>peak_accel</c>, m/s².</item>
    /// <item><c>end_ratio</c>: the mean speed over the span's last 30% against the peak.</item>
    /// <item><c>stop_s</c>: from the peak until the speed is below a fifth of it (looked for up to 1 s past the span; NaN when the next
    /// move keeps it up).</item>
    /// <item><c>peaks</c>: the speed's local maxima over 30% of the peak.</item>
    /// <item><c>low_mm</c>: the tool's lowest point above the table.</item>
    /// <item><c>roll_range_deg</c>, <c>pitch_range_deg</c> (the wrist's absolute pitch) and <c>grip_range</c> over the span.</item>
    /// <item><c>next_kind</c>, <c>pause_after</c> (the next move is a pause), <c>reverse_after</c> (a reverse) and <c>pullback_after</c>
    /// (a reverse that heads back against this move's first 70%: the next move begins before this one ends, so its last part already
    /// turns).</item>
    /// <item><c>part</c>: what part of the plan it is (<see cref="MovePart"/>), which the record does not say.</item>
    /// </list>
    /// </summary>
    internal static class MoveFeatures
    {
        public const string Header =
            "layout,seed,move,kind,object,offset_mm,gap_mm,contact,start_s,end_s,duration_s,path_m,chord_m,peak_speed,mean_speed,peak_accel,end_ratio,stop_s,peaks,low_mm," +
            "roll_range_deg,pitch_range_deg,grip_range,next_kind,pause_after,reverse_after,pullback_after,part";

        /// <summary>The smoothing of the tool point, s (sigma of the Gaussian): workstream R's 25 ms.</summary>
        public const double SmoothSeconds = 0.025;

        /// <summary>One row a move of <paramref name="plan"/> (layout <paramref name="layout"/>, take seed <paramref name="seed"/>) to <paramref name="output"/>; how many.</summary>
        public static int Write(TextWriter output, int layout, ulong seed, in RobotArmProfile profile, DrivePlan plan, float table)
        {
            double tick = HandModel.TickSeconds, start = plan.ScriptStart, end = plan.TotalSeconds;
            int n = Math.Max(2, (int)Math.Floor((end - start) / tick + 1e-9) + 1);
            var poses = new ArmPose[n];
            var grips = new float[n];
            var tool = new Vector3[n];
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            for (int k = 0; k < n; k++)
            {
                double t = Math.Min(end, start + k * tick);
                poses[k] = plan.Sample(t);
                grips[k] = plan.GripperAt(t);
                ArmCollision.ComputeFrames(profile, poses[k], frames);
                tool[k] = frames[(int)ArmLink.Gripper].ToBase(MixedTake.ToolInGripper);
            }

            // The tool smoothed, then its speed and acceleration by central differences.
            int r = (int)Math.Ceiling(3 * SmoothSeconds / tick);
            var w = new double[2 * r + 1];
            for (int i = -r; i <= r; i++) w[i + r] = Math.Exp(-0.5 * (i * tick / SmoothSeconds) * (i * tick / SmoothSeconds));
            var s = new Vector3[n];
            for (int k = 0; k < n; k++)
            {
                double x = 0, y = 0, z = 0, ws = 0;
                for (int i = -r; i <= r; i++)
                {
                    int m = k + i;
                    if (m < 0 || m >= n) continue;
                    x += w[i + r] * tool[m].X;
                    y += w[i + r] * tool[m].Y;
                    z += w[i + r] * tool[m].Z;
                    ws += w[i + r];
                }

                s[k] = new Vector3((float)(x / ws), (float)(y / ws), (float)(z / ws));
            }

            var speed = new double[n];
            var accel = new double[n];
            for (int k = 1; k + 1 < n; k++)
            {
                speed[k] = Vector3.Distance(s[k + 1], s[k - 1]) / (2 * tick);
                accel[k] = (s[k + 1] - 2f * s[k] + s[k - 1]).Length() / (tick * tick);
            }

            speed[0] = speed[Math.Min(1, n - 1)];
            speed[n - 1] = speed[Math.Max(0, n - 2)];
            accel[0] = accel[Math.Min(1, n - 1)];
            accel[n - 1] = accel[Math.Max(0, n - 2)];

            IReadOnlyList<PlannedMove> moves = plan.Moves;
            int rows = 0;
            for (int i = 0; i < moves.Count; i++)
            {
                PlannedMove m = moves[i];
                if (!Span(m, start, end, tick, n, out int k0, out int k1, out double a, out double b)) continue;
                double path = 0;
                for (int k = k0 + 1; k <= k1; k++) path += Vector3.Distance(s[k], s[k - 1]);
                double peak = 0, top = 0;
                int kp = k0;
                double lowest = double.MaxValue, roll0 = double.MaxValue, roll1 = double.MinValue, pitch0 = double.MaxValue, pitch1 = double.MinValue;
                double grip0 = double.MaxValue, grip1 = double.MinValue;
                for (int k = k0; k <= k1; k++)
                {
                    if (speed[k] > peak)
                    {
                        peak = speed[k];
                        kp = k;
                    }

                    top = Math.Max(top, accel[k]);
                    lowest = Math.Min(lowest, tool[k].Z);
                    roll0 = Math.Min(roll0, poses[k].Roll);
                    roll1 = Math.Max(roll1, poses[k].Roll);
                    double p = HandModel.PitchOf(poses[k]);
                    pitch0 = Math.Min(pitch0, p);
                    pitch1 = Math.Max(pitch1, p);
                    if (float.IsNaN(grips[k])) continue;
                    grip0 = Math.Min(grip0, grips[k]);
                    grip1 = Math.Max(grip1, grips[k]);
                }

                int k30 = Math.Min(k1, (int)Math.Ceiling((b - 0.3 * (b - a) - start) / tick - 1e-9));
                double tail = 0;
                for (int k = k30; k <= k1; k++) tail += speed[k];
                tail /= k1 - k30 + 1;
                double stop = double.NaN;
                int kStopMax = Math.Min(n - 1, k1 + (int)Math.Round(1.0 / tick));
                for (int k = kp; k <= kStopMax && double.IsNaN(stop); k++) if (speed[k] < 0.2 * peak) stop = (k - kp) * tick;
                int peaks = 0;
                for (int k = Math.Max(1, k0); k <= Math.Min(n - 2, k1); k++) if (speed[k] > speed[k - 1] && speed[k] >= speed[k + 1] && speed[k] > 0.3 * peak) peaks++;

                string next = "none";
                bool pullBack = false;
                if (i + 1 < moves.Count)
                {
                    PlannedMove nm = moves[i + 1];
                    next = ScriptMoveCodec.Name(nm.Kind);
                    if (nm.Kind == ScriptMoveKind.Reverse && Span(nm, start, end, tick, n, out int n0, out int n1, out _, out _))
                    {
                        int k70 = Math.Max(k0, Math.Min(k1, (int)Math.Floor((a + 0.7 * (b - a) - start) / tick + 1e-9)));
                        Vector3 went = s[k70] - s[k0], ahead = s[n1] - s[n0];
                        pullBack = Vector3.Dot(went, ahead) < 0f;
                    }
                }

                output.WriteLine(string.Join(",",
                    layout.ToString(CultureInfo.InvariantCulture), seed.ToString(CultureInfo.InvariantCulture), m.Index.ToString(CultureInfo.InvariantCulture),
                    ScriptMoveCodec.Name(m.Kind), m.ObjectId.ToString(CultureInfo.InvariantCulture), F(m.OffsetMeters * 1000.0), F(m.GapMeters * 1000.0),
                    double.IsNaN(m.Contact) ? "0" : "1", F(a - start), F(b - start), F(b - a), F(path), F(Vector3.Distance(s[k1], s[k0])), F(peak),
                    F(b > a ? path / (b - a) : 0.0), F(top), F(peak > 0 ? tail / peak : double.NaN), F(stop), peaks.ToString(CultureInfo.InvariantCulture),
                    F((lowest - table) * 1000.0), F((roll1 - roll0) * 180.0 / Math.PI), F((pitch1 - pitch0) * 180.0 / Math.PI),
                    F(grip1 >= grip0 ? grip1 - grip0 : double.NaN), next, next == "pause" ? "1" : "0", next == "reverse" ? "1" : "0", pullBack ? "1" : "0",
                    PartName(m.Part)));
                rows++;
            }

            return rows;
        }

        /// <summary>Move <paramref name="m"/>'s span clipped to the take (plan seconds <paramref name="a"/> to <paramref name="b"/>) and its ticks; false when none of it is in the take.</summary>
        private static bool Span(in PlannedMove m, double start, double end, double tick, int n, out int k0, out int k1, out double a, out double b)
        {
            a = Math.Max(m.Start, start);
            b = Math.Min(m.End, end);
            k0 = Math.Min(n - 1, Math.Max(0, (int)Math.Ceiling((a - start) / tick - 1e-9)));
            k1 = Math.Min(n - 1, Math.Max(k0, (int)Math.Floor((b - start) / tick + 1e-9)));
            return b >= a;
        }

        private static string PartName(MovePart p) => p switch
        {
            MovePart.BlindTransport => "blind-transport",
            MovePart.BlindStrokes => "blind-strokes",
            MovePart.BlindAdjust => "blind-adjust",
            MovePart.Staging => "staging",
            MovePart.Approach => "approach",
            MovePart.PullBack => "pull-back",
            MovePart.ForcedPullBack => "forced-pull-back",
            MovePart.Pause => "pause",
            MovePart.Recentre => "recentre",
            MovePart.Exit => "exit",
            _ => "other",
        };

        private static string F(double v) => double.IsNaN(v) || double.IsInfinity(v) ? "nan" : v.ToString("0.######", CultureInfo.InvariantCulture);
    }
}
