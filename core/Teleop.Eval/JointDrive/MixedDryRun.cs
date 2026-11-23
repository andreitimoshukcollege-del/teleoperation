using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Numerics;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// <c>joint-drive --script &lt;safety script&gt; --dry-run --scene &lt;file&gt; --layouts a-b</c> (docs/adr/0038 decision 1): the
    /// script planned from home on each of RobotHost's layouts <c>a</c> to <c>b</c> of the scene (<see cref="SceneLayout"/>, as a
    /// batch draws one per take), nothing sent, and what the plans do summed up: how many plan, how long they take to plan and
    /// last, and for the mixed script its moves (their kinds, the offsets aims were aimed at and the closest approaches as planned,
    /// where contacts come in the take and in their move, from which directions, at which heights); for every script the commanded
    /// motion per dataset frame (15 fps), the measure world-models compares with other datasets.
    /// </summary>
    internal static class MixedDryRun
    {
        /// <summary>The dataset's frame rate (robot/lerobot's default): the per-frame motion is measured at it.</summary>
        public const double FramesPerSecond = 15.0;

        internal static int Run(DriveScript script, WorldScene scene, in RobotArmProfile profile, in DriveEnvelope envelope, float speed, double seconds, ulong seed,
            int from, int to, SafetyOptions safetyOptions, MixedOptions mixedOptions, bool rows)
        {
            ArmCollisionModel model = profile.Collision!;
            ObjectShapes? shapes = WorldView.ShapesOf(scene);
            float table = (float)scene.Table.Top;
            var poses = new RigidTransform[scene.Objects.Length];
            var parked = new bool[scene.Objects.Length];
            var scales = new double[scene.Objects.Length];
            var planSeconds = new List<double>();
            var lengths = new List<double>();
            var offsets = new List<float>();
            var gaps = new List<float>();
            var passOffsets = new List<float>();
            var approach = new List<float>();
            var heights = new List<float>();
            var inTake = new List<double>();
            var inMove = new List<double>();
            var firstContact = new List<double>();
            var frameJoint = new List<double>();
            var frameWrist = new List<double>();
            var tickJoint = new List<double>();
            var jointSpeed = new List<double>[5];
            var jointAccel = new List<double>[5];
            for (int j = 0; j < 5; j++)
            {
                jointSpeed[j] = new List<double>();
                jointAccel[j] = new List<double>();
            }

            int movingFrames = 0, allFrames = 0;
            var kinds = new int[6];
            var failures = new List<string>();
            int planned = 0, contactTakes = 0;
            var watch = new Stopwatch();
            for (int layout = from; layout <= to; layout++)
            {
                SceneLayout.Draw(scene, (ulong)layout, poses, parked, scales);
                var objects = new List<SafetyObject>();
                for (int i = 0; i < scene.Objects.Length; i++)
                {
                    if (parked[i]) continue;
                    SceneObject o = scene.Objects[i];
                    objects.Add(WorldView.Bounds(o, scales[i], poses[i].X, poses[i].Y, poses[i].Z, poses[i].Qw, poses[i].Qx, poses[i].Qy, poses[i].Qz, shapes,
                        o.Movable && scene.InEdgeRegions(poses[i].X, poses[i].Y)));
                }

                ulong takeSeed = seed * 100000UL + (ulong)layout; // as a batch seeds its takes
                watch.Restart();
                if (!DrivePlan.TryBuild(script, profile, model.Home, speed, seconds, takeSeed, envelope, out DrivePlan? plan, out string? error, objects, safetyOptions, mixedOptions))
                {
                    if (failures.Count < 12) failures.Add($"layout {layout}: {error}");
                    if (rows) Console.WriteLine($"  layout {layout}: not planned ({error})");
                    continue;
                }

                planSeconds.Add(watch.Elapsed.TotalSeconds);
                planned++;
                double start = plan!.ScriptStart, length = plan.TotalSeconds - start;
                lengths.Add(length);
                bool touched = false;
                foreach (PlannedMove m in plan.Moves)
                {
                    kinds[(int)m.Kind]++;
                    if (m.ObjectId == 0) continue;
                    if (m.Kind == ScriptMoveKind.Aim) offsets.Add(m.OffsetMeters);
                    else passOffsets.Add(m.OffsetMeters);
                    if (!float.IsNaN(m.GapMeters)) gaps.Add(m.GapMeters);
                    approach.Add(m.ApproachDegrees);
                    if (double.IsNaN(m.Contact)) continue;
                    if (!touched) firstContact.Add(m.Contact - start);
                    touched = true;
                    inTake.Add((m.Contact - start) / length);
                    inMove.Add((m.Contact - m.Start) / Math.Max(1e-6, m.End - m.Start));
                    heights.Add(MixedTake.Lowest(profile, model, plan.Sample(m.Contact), plan.GripperAt(m.Contact)) - table);
                }

                if (touched) contactTakes++;
                int before = frameJoint.Count;
                Motion(profile, plan, frameJoint, frameWrist, tickJoint, jointSpeed, jointAccel, ref movingFrames);
                allFrames += frameJoint.Count - before;
                if (rows)
                {
                    Console.WriteLine(FormattableString.Invariant($"  layout {layout}: {length:0.0} s, {plan.Moves.Count} moves, planned in {planSeconds[planSeconds.Count - 1]:0.00} s; {plan.PlanningReport}"));
                    foreach (PlannedMove m in plan.Moves.Where(x => x.ObjectId != 0)) Console.WriteLine("    " + m);
                }
            }

            int total = to - from + 1;
            Console.WriteLine(FormattableString.Invariant($"[joint-drive] {SafetyScript.Category(script)} on layouts {from}-{to} of {scene.Name}: {planned} of {total} planned"));
            foreach (string f in failures) Console.WriteLine("  not planned: " + f);
            if (planned == 0) return 1;
            Console.WriteLine("  plan time s: " + Percentiles(planSeconds, "0.00"));
            Console.WriteLine("  take length s: " + Percentiles(lengths, "0.0"));
            if (script == DriveScript.Mixed)
            {
                Console.WriteLine(FormattableString.Invariant(
                    $"  moves a take: free {kinds[1] / (double)planned:0.0}, aim {kinds[2] / (double)planned:0.0}, pass {kinds[3] / (double)planned:0.0}, pause {kinds[4] / (double)planned:0.0}, reverse {kinds[5] / (double)planned:0.0}"));
                Console.WriteLine("  aims' offsets mm (aimed at), 10 mm bins from -40: " + Histogram(offsets.Select(o => o * 1000.0), -40, 10, 10));
                Console.WriteLine("  aims' offsets mm: " + Percentiles(offsets.Select(o => o * 1000.0).ToList(), "0"));
                Console.WriteLine("  passes' offsets mm (closest on the line), 10 mm bins from -40: " + Histogram(passOffsets.Select(o => o * 1000.0), -40, 10, 9));
                Console.WriteLine("  closest approach over the move as planned mm, 10 mm bins from -40: " + Histogram(gaps.Select(o => o * 1000.0), -40, 10, 9));
                Console.WriteLine(FormattableString.Invariant($"  planned contacts: {inTake.Count} in {contactTakes} of {planned} takes; the first {Percentiles(firstContact, "0.0")} s into the take"));
                Console.WriteLine("  where in the take (start, middle, end thirds): " + Histogram(inTake, 0, 1 / 3.0, 3));
                Console.WriteLine("  where in their move (start, middle, end thirds): " + Histogram(inMove, 0, 1 / 3.0, 3));
                Console.WriteLine("  approach about the object's bearing deg, 30 deg bins from -180: " + Histogram(approach.Select(a => (double)a), -180, 30, 12));
                Console.WriteLine("  contact height above the table mm (the arm's lowest point), 20 mm bins: " + Histogram(heights.Select(h => h * 1000.0), 0, 20, 12));
                Console.WriteLine("  contact height mm: " + Percentiles(heights.Select(h => h * 1000.0).ToList(), "0"));
            }

            Console.WriteLine("  commanded motion per frame (15 fps), largest arm-joint change rad: " + Percentiles(frameJoint, "0.0000"));
            Console.WriteLine("  commanded motion per frame (15 fps), wrist travel mm: " + Percentiles(frameWrist, "0.0"));
            Console.WriteLine("  commanded joint speed tick to tick (48 Hz), fastest joint rad/s: " + Percentiles(tickJoint, "0.00"));
            Console.WriteLine(FormattableString.Invariant($"  the command moves (a joint 2 servo units, 0.48 deg, or more in a frame) in {100.0 * movingFrames / Math.Max(1, allFrames):0}% of frames"));
            string[] names = { "yaw", "shoulder", "elbow", "wrist pitch", "roll" };
            for (int j = 0; j < 5; j++)
            {
                Console.WriteLine($"  {names[j]}, smoothed 25 ms: speed rad/s {Percentiles(jointSpeed[j], "0.00")}; acceleration rad/s2 {Percentiles(jointAccel[j], "0.0")}");
            }
            Console.WriteLine("[joint-drive] dry run: nothing was sent");
            return 0;
        }

        /// <summary>The plan's commanded motion from the take's start: per dataset frame (sample and hold of the 48 Hz commands), its largest arm-joint change and the wrist's travel; per tick, its fastest joint.</summary>
        internal static void Motion(in RobotArmProfile profile, DrivePlan plan, List<double> frameJoint, List<double> frameWrist, List<double> tickJoint,
            List<double>[] jointSpeed, List<double>[] jointAccel, ref int movingFrames)
        {
            double tick = HandModel.TickSeconds, frame = 1.0 / FramesPerSecond;
            double start = plan.ScriptStart;
            var commands = new List<ArmPose>();
            for (double t = start; t <= plan.TotalSeconds + 1e-9; t += tick) commands.Add(plan.Sample(t));
            for (int k = 1; k < commands.Count; k++) tickJoint.Add(commands[k].MaxJointDelta(commands[k - 1]) / tick);
            // Each joint at the command rate, smoothed with a 25 ms Gaussian, as workstream R measured people (metrics.py): speed and
            // acceleration by central differences.
            const double sigma = 0.025;
            int r = (int)Math.Ceiling(3 * sigma / tick);
            var w = new double[2 * r + 1];
            for (int i = -r; i <= r; i++) w[i + r] = Math.Exp(-0.5 * (i * tick / sigma) * (i * tick / sigma));
            for (int j = 0; j < 5; j++)
            {
                var q = new double[commands.Count];
                for (int k = 0; k < q.Length; k++)
                {
                    double sum = 0, ws = 0;
                    for (int i = -r; i <= r; i++)
                    {
                        int m = k + i;
                        if (m < 0 || m >= q.Length) continue;
                        sum += w[i + r] * DrivePlan.Joint(commands[m], j);
                        ws += w[i + r];
                    }

                    q[k] = sum / ws;
                }

                for (int k = 1; k + 1 < q.Length; k++)
                {
                    jointSpeed[j].Add(Math.Abs(q[k + 1] - q[k - 1]) / (2 * tick));
                    jointAccel[j].Add(Math.Abs(q[k + 1] - 2 * q[k] + q[k - 1]) / (tick * tick));
                }
            }

            ArmPose? previous = null;
            for (double f = 0; f <= plan.TotalSeconds - start + 1e-9; f += frame)
            {
                ArmPose p = commands[Math.Min(commands.Count - 1, (int)Math.Floor(f / tick + 1e-9))];
                if (previous.HasValue)
                {
                    ArmPose q = previous.Value;
                    double d = 0;
                    for (int j = 0; j < 4; j++) d = Math.Max(d, Math.Abs(DrivePlan.Joint(p, j) - DrivePlan.Joint(q, j)));
                    frameJoint.Add(d);
                    double any = d;
                    any = Math.Max(any, Math.Abs(DrivePlan.Joint(p, 4) - DrivePlan.Joint(q, 4)));
                    if (any >= 2 * 0.24 * Math.PI / 180) movingFrames++;
                    frameWrist.Add(Vector3.Distance(HandModel.Wrist(profile, p), HandModel.Wrist(profile, q)) * 1000.0);
                }

                previous = p;
            }
        }

        internal static string Percentiles(List<double> xs, string format)
        {
            if (xs.Count == 0) return "none";
            var s = xs.OrderBy(x => x).ToList();
            string P(double p) => s[Math.Min(s.Count - 1, (int)Math.Round(p / 100.0 * (s.Count - 1)))].ToString(format, CultureInfo.InvariantCulture);
            return $"p50 {P(50)}, p90 {P(90)}, p95 {P(95)}, p99 {P(99)}, p99.9 {P(99.9)}, max {s[s.Count - 1].ToString(format, CultureInfo.InvariantCulture)} (n {s.Count})";
        }

        internal static string Histogram(IEnumerable<double> xs, double from, double width, int bins)
        {
            var counts = new int[bins];
            int below = 0, above = 0;
            foreach (double x in xs)
            {
                int b = (int)Math.Floor((x - from) / width + 1e-9);
                if (b < 0) below++;
                else if (b >= bins) above++;
                else counts[b]++;
            }

            return (below > 0 ? $"(below {below}) " : "") + string.Join(" ", counts) + (above > 0 ? $" (above {above})" : "");
        }
    }
}
