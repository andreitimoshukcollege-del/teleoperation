using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
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
            int from, int to, SafetyOptions safetyOptions, MixedOptions mixedOptions, bool rows, string? featuresPath = null)
        {
            ArmCollisionModel model = profile.Collision!;
            // docs/adr/0038, "M, motion": every planned move's own motion, one CSV row a move (MoveFeatures).
            using StreamWriter? features = featuresPath != null && script == DriveScript.Mixed ? new StreamWriter(featuresPath, false) : null;
            features?.WriteLine(MoveFeatures.Header);
            int featureRows = 0;
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
            int pullBacks = 0, forcedPullBacks = 0, intoFixtures = 0; // docs/adr/0038 "M, motion": one pull-back law, and the fixtures' forced one
            int blindMoves = 0, blindTouching = 0, blindTakes = 0; // layout-blind moves that meet a movable object's box by the model
            // docs/adr/0038 decision 2: the edits of a group of two replays, as joint-drive draws them
            var editKinds = new int[5];
            int groups = 0, addGroups = 0, addsMoved = 0, addsUnparked = 0, addsMet = 0, noEdit = 0;
            var addFirst = new List<double>();
            var addHeights = new List<float>();
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
                    if (m.Part == MovePart.PullBack) pullBacks++;
                    if (m.Part == MovePart.ForcedPullBack) forcedPullBacks++;
                    if (m.ObjectId != 0 && m.OffsetMeters < 0.005f && objects.Exists(o => o.Id == m.ObjectId && !o.Movable)) intoFixtures++;
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
                if (script == DriveScript.Mixed)
                {
                    int t0 = blindTouching;
                    BlindContacts(profile, model, plan, objects, ref blindMoves, ref blindTouching);
                    if (blindTouching > t0) blindTakes++;
                    groups++;
                    ReplayEditDraw[] edits = GroupEdits(scene, profile, model, plan, objects, parked, scales, takeSeed, mixedOptions);
                    if (edits.Length < 2) noEdit++;
                    foreach (ReplayEditDraw e in edits)
                    {
                        editKinds[(int)e.Record.Edit]++;
                        if (e.Record.Edit != ReplayEdit.Add) continue;
                        addGroups++;
                        if (e.Op == WorldEditOp.Unpark) addsUnparked++;
                        else addsMoved++;
                        addHeights.Add((e.After!.Value.Max.Z - table) * 1000f);
                        double met = FirstMeeting(profile, model, plan, e.After!.Value);
                        if (double.IsNaN(met)) continue;
                        addsMet++;
                        addFirst.Add(met);
                    }
                }
                if (features != null) featureRows += MoveFeatures.Write(features, layout, takeSeed, profile, plan, table);
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
                Console.WriteLine(FormattableString.Invariant(
                    $"  pull-backs a take: {pullBacks / (double)planned:0.00} drawn, {forcedPullBacks / (double)planned:0.00} more forced out of a fixture (aims and passes into fixtures: {intoFixtures / (double)planned:0.00})"));
                Console.WriteLine(FormattableString.Invariant(
                    $"  layout-blind moves (free and reverse) meeting a movable object's box by the model: {blindTouching} of {blindMoves}, in {blindTakes} of {planned} takes"));
                Console.WriteLine(FormattableString.Invariant(
                    $"  replays, 2 a group as drawn (nothing refused offline): an add in {addGroups} of {groups} groups ({addsMoved} moving one of the layout's, {addsUnparked} a parked one); edits remove {editKinds[1]}, shift {editKinds[2]}, move {editKinds[3]}, add {editKinds[4]}; groups with fewer than 2 edits {noEdit}"));
                Console.WriteLine(FormattableString.Invariant(
                    $"  adds: the arm meets the added object's box by the model in {addsMet} of {addGroups}, first {Percentiles(addFirst, "0.0")} s into the take; the added object's top mm above the table: {Percentiles(addHeights.Select(h => (double)h).ToList(), "0")}"));
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
            if (features != null) Console.WriteLine($"  features: {featureRows} moves of {planned} takes written to {featuresPath}");
            Console.WriteLine("[joint-drive] dry run: nothing was sent");
            return 0;
        }

        /// <summary>
        /// The edits a group of two replays of <paramref name="plan"/> draws (docs/adr/0038 decision 2), as joint-drive draws them with nothing
        /// refused: the add looks where the plan's commands go, may stand the pool's parked obstacles on the table, and the objects the take
        /// moved are taken to be those its moves planned to touch. Fewer than two when a replay finds nothing new.
        /// </summary>
        internal static ReplayEditDraw[] GroupEdits(WorldScene scene, in RobotArmProfile profile, ArmCollisionModel model, DrivePlan plan, List<SafetyObject> objects,
            bool[] parked, double[] scales, ulong group, MixedOptions options)
        {
            var touched = new HashSet<byte>(plan.Moves.Where(m => m.ObjectId != 0 && !double.IsNaN(m.Contact)).Select(m => m.ObjectId));
            var parkedIds = new List<byte>();
            for (int i = 0; i < scene.Objects.Length; i++)
            {
                SceneObject o = scene.Objects[i];
                if (parked[i] && o.Movable && o.OperatorMovable && (!scene.Mixed || o.Role == SceneRole.Obstacle)) parkedIds.Add(o.Id);
            }

            var add = new ReplayAdd(ReplayAdd.Commanded(plan), plan.TotalSeconds - plan.ScriptStart, parkedIds, model, options);
            var used = new HashSet<string>();
            var edits = new List<ReplayEditDraw>();
            double ScaleOf(byte id)
            {
                int k = scene.IndexOf(id);
                return k >= 0 && scales[k] > 0 ? scales[k] : 1.0;
            }

            for (int r = 1; r <= 2; r++)
            {
                ReplayEditDraw e = ReplayEdits.Draw(group, r, 0, plan, objects, scene, touched, profile, used, ScaleOf, add);
                if (e.Pose.Id == 0) break;
                used.Add(ReplayEdits.Key(e.Record));
                edits.Add(e);
            }

            return edits.ToArray();
        }

        /// <summary>When the plan's arm first comes into <paramref name="o"/>'s box by the collision model (its margin off), seconds into the take; NaN when it never does.</summary>
        internal static double FirstMeeting(in RobotArmProfile profile, ArmCollisionModel model, DrivePlan plan, in SafetyObject o)
        {
            ArmCollisionModel check = SafetyScript.WithObjects(model, new[] { o }, 0f, 0f);
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds; t += HandModel.TickSeconds)
            {
                if (!ArmCollision.IsValid(profile, check, plan.Sample(t))) return t - plan.ScriptStart;
            }

            return double.NaN;
        }

        /// <summary>
        /// How many of the plan's layout-blind moves (free and reverse, not aims or passes) come into a movable object's box (the
        /// collision model's bodies, its margin off), checked every tick of the move: physics decides those, and the plan does not
        /// count them as contacts.
        /// </summary>
        internal static void BlindContacts(in RobotArmProfile profile, ArmCollisionModel model, DrivePlan plan, List<SafetyObject> objects, ref int moves, ref int touching)
        {
            var movable = objects.Where(o => o.Movable && !o.Scenery).ToArray();
            ArmCollisionModel check = SafetyScript.WithObjects(model, movable, 0f, 0f);
            foreach (PlannedMove m in plan.Moves)
            {
                if (m.Kind != ScriptMoveKind.Free && m.Kind != ScriptMoveKind.Reverse) continue;
                moves++;
                if (movable.Length == 0) continue;
                for (double t = Math.Max(m.Start, plan.ScriptStart); t <= Math.Min(m.End, plan.TotalSeconds); t += HandModel.TickSeconds)
                {
                    if (ArmCollision.IsValid(profile, check, plan.Sample(t))) continue;
                    touching++;
                    break;
                }
            }
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
