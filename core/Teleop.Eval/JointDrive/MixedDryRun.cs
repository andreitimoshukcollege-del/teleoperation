using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Numerics;
using Teleop.Core.Types;
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
    /// <summary>
    /// How a dry run plans its layouts (docs/adr/0038 "M, motion"): from home (the default), or each from where the last one left the arm
    /// (<see cref="Chain"/>, as the twin's takes start where the one before ended); and what an add looks at: the commanded way, or that way
    /// <see cref="LagSeconds"/> late (as a measured track trails its commands: about 0.2 s on the JetRover and its twin).
    /// </summary>
    internal sealed class DryRunOptions
    {
        public bool Chain { get; set; }
        public double LagSeconds { get; set; }
    }

    internal static class MixedDryRun
    {
        /// <summary>The dataset's frame rate (robot/lerobot's default): the per-frame motion is measured at it.</summary>
        public const double FramesPerSecond = 15.0;

        internal static int Run(DriveScript script, WorldScene scene, in RobotArmProfile profile, in DriveEnvelope envelope, float speed, double seconds, ulong seed,
            int from, int to, SafetyOptions safetyOptions, MixedOptions mixedOptions, bool rows, string? featuresPath = null, DryRunOptions? dry = null)
        {
            dry ??= new DryRunOptions();
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
            var kinds = new int[16];
            // docs/adr/0038 "M, motion" (accidents): each record's contacts by the model, whether it is an accident, and where the accidents came.
            int records = 0, contactRecords = 0, accidentRecords = 0, accidentContactRecords = 0, accidentOnsets = 0, plannedAccidentContacts = 0;
            int accidentsTouching = 0, accidentsNear = 0, accidentsFree = 0, accidentsQuietStart = 0, accidentsLate = 0, accidentTakes = 0;
            var accidentKinds = new Dictionary<string, int>();
            var accidentKindContacts = new Dictionary<string, int>();
            var wild = new Wild();
            var failures = new List<string>();
            int planned = 0, contactTakes = 0;
            int pullBacks = 0, forcedPullBacks = 0, intoFixtures = 0; // docs/adr/0038 "M, motion": one pull-back law, and the fixtures' forced one
            int blindMoves = 0, blindTouching = 0, blindTakes = 0; // layout-blind moves that meet a movable object's box by the model
            // docs/adr/0038 decision 2: the edits of a group of two replays, as joint-drive draws them
            var editKinds = new int[5];
            var wayBack = new int[3]; // groups with an edit; of them, no way back to the source's start as before; and with the tucked way
            int groups = 0, addGroups = 0, addsMoved = 0, addsUnparked = 0, addsMet = 0, noEdit = 0;
            var addFirst = new List<double>();
            var addHeights = new List<float>();
            // docs/adr/0038 "M, motion" (2026-11-23 review): where contacts fall. The planned ones (aims and passes) and every contact by the
            // model along the whole plan (the twin's links against each object's shape, else its box; blind moves included), each with the
            // tool point's height (the pads' midpoint, base frame z) and the object's kind.
            var plannedTool = new List<double>();
            var plannedKinds = new Dictionary<string, int>();
            var modelTool = new List<double>();
            var modelKinds = new Dictionary<string, int>();
            var modelParts = new Dictionary<string, int>();
            int modelTakes = 0, modelOnsets = 0;
            // docs/adr/0038 "M, motion" (2026-11-24 review): contacts as the twin's arm makes them, on the follower of the command (TakeAudit's,
            // fitted to the pilot's measured joints): each onset given to the accident whose span (and the lag after) holds it, else to the move
            // under way a delay before; the tool point's height, the touching link and the contact point's height.
            var lagged = new LagTally();
            var aimTool = new List<double>(); // the tool point where each aim ends (its goal as planned), base frame z
            var blindTool = new List<double>(); // and where each layout-blind move to a goal ends
            // docs/adr/0038 decision 2: whether the model expects each replay's edit to change what the take touches, by edit and by how near
            // the source's arm came to the edited object (contact, within 1 cm, within 5 cm, farther, or an object the layout left out).
            var editTally = new Dictionary<string, int[]>(); // key "kind/where": n, the object's contact changed, the take's any-contact changed
            var groupNotes = new List<string>();
            var aimKinds = new Dictionary<string, int>();
            var noAddWhy = new Dictionary<string, int>();
            ArmPose startPose = model.Home;
            var watch = new Stopwatch();
            for (int layout = from; layout <= to; layout++)
            {
                SceneLayout.Draw(scene, (ulong)layout, poses, parked, scales);
                var objects = new List<SafetyObject>();
                for (int i = 0; i < scene.Objects.Length; i++)
                {
                    if (parked[i]) continue;
                    SceneObject o = scene.Objects[i];
                    // As WorldView.TryObjects sees it on the twin: an edge object, or a mixed pool's distractor, is scenery.
                    objects.Add(WorldView.Bounds(o, scales[i], poses[i].X, poses[i].Y, poses[i].Z, poses[i].Qw, poses[i].Qx, poses[i].Qy, poses[i].Qz, shapes,
                        (o.Movable && scene.InEdgeRegions(poses[i].X, poses[i].Y)) || o.Role == SceneRole.Distractor));
                }

                ulong takeSeed = seed * 100000UL + (ulong)layout; // as a batch seeds its takes
                watch.Restart();
                if (!DrivePlan.TryBuild(script, profile, dry.Chain ? startPose : model.Home, speed, seconds, takeSeed, envelope, out DrivePlan? plan, out string? error, objects, safetyOptions,
                        mixedOptions))
                {
                    if (failures.Count < 12) failures.Add($"layout {layout}: {error}");
                    if (rows) Console.WriteLine($"  layout {layout}: not planned ({error})");
                    continue;
                }

                planSeconds.Add(watch.Elapsed.TotalSeconds);
                planned++;
                if (dry.Chain) startPose = plan!.Sample(plan.TotalSeconds);
                var arm = new ArmTouch(profile, model);
                double start = plan!.ScriptStart, length = plan.TotalSeconds - start;
                lengths.Add(length);
                bool touched = false;
                foreach (PlannedMove m in plan.Moves)
                {
                    kinds[(int)m.Kind]++;
                    if (ScriptMoveCodec.IsAccident(m.Kind))
                    {
                        // An accident's record: counted apart from the aims and passes it may overlie (it keeps their object and offset).
                        Count(accidentKinds, ScriptMoveCodec.Name(m.Kind));
                        if (m.ObjectId != 0 && !double.IsNaN(m.Contact)) plannedAccidentContacts++;
                        continue;
                    }

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
                    plannedTool.Add(arm.At(plan.Sample(m.Contact), plan.GripperAt(m.Contact)).Tool.Z);
                    int oi = objects.FindIndex(o => o.Id == m.ObjectId);
                    if (oi >= 0) Count(plannedKinds, KindOf(objects[oi], scene, table));
                }

                foreach (PlannedMove m in plan.Moves)
                {
                    if (m.ObjectId == 0 || ScriptMoveCodec.IsAccident(m.Kind)) continue;
                    int oi = objects.FindIndex(o => o.Id == m.ObjectId);
                    if (oi >= 0) Count(aimKinds, (m.Kind == ScriptMoveKind.Aim ? "aim " : "pass ") + KindOf(objects[oi], scene, table));
                }

                foreach (PlannedMove m in plan.Moves)
                {
                    // Where each aim and each layout-blind move to a goal ends: the height parity docs/adr/0038 "M, motion" asks for.
                    if ((m.Part != MovePart.Approach && m.Part != MovePart.BlindTransport) || ScriptMoveCodec.IsAccident(m.Kind)) continue;
                    double at = Math.Min(m.End, plan.TotalSeconds);
                    if (at < start) continue;
                    double z = arm.At(plan.Sample(at), plan.GripperAt(at)).Tool.Z;
                    if (m.Part == MovePart.Approach && m.Kind == ScriptMoveKind.Aim) aimTool.Add(z);
                    else if (m.Part == MovePart.BlindTransport) blindTool.Add(z);
                }

                List<TrackPoint> commanded = ReplayAdd.Commanded(plan);
                var byMove = new int[plan.Moves.Count];
                int onsets = ModelContacts(arm, plan, commanded, objects, scene, table, modelTool, modelKinds, modelParts, byMove);
                modelOnsets += onsets;
                if (onsets > 0) modelTakes++;
                if (script == DriveScript.Mixed)
                    Accidents(profile, model, plan, objects, byMove, ref records, ref contactRecords, ref accidentRecords, ref accidentContactRecords, ref accidentOnsets,
                        ref accidentsTouching, ref accidentsNear, ref accidentsFree, ref accidentsQuietStart, ref accidentsLate, ref accidentTakes, accidentKindContacts);

                if (script == DriveScript.Mixed) LagAwareContacts(arm, plan, Followed(plan), objects, lagged);
                if (touched) contactTakes++;
                if (script == DriveScript.Mixed)
                {
                    int t0 = blindTouching;
                    BlindContacts(profile, model, plan, objects, ref blindMoves, ref blindTouching);
                    if (blindTouching > t0) blindTakes++;
                    groups++;
                    List<TrackPoint> addTrack = dry.LagSeconds > 0 ? Lagged(plan, dry.LagSeconds) : commanded;
                    ReplayEditDraw[] edits = GroupEdits(scene, profile, model, plan, objects, parked, scales, takeSeed, mixedOptions, addTrack, groupNotes);
                    bool gotAdd = edits.Any(x => x.Record.Edit == ReplayEdit.Add);
                    foreach (string note in groupNotes)
                    {
                        // Why a group has no add, or a replay no edit: the reason's first words (the counts after them vary).
                        if (note.StartsWith("no add", StringComparison.Ordinal) && gotAdd) continue;
                        string key = note.StartsWith("no add: ", StringComparison.Ordinal) && note.Contains(" tried; ") ? "no add: no spot under the arm (" + Reason(note) + ")"
                            : note.StartsWith("nothing near the path", StringComparison.Ordinal) ? "a replay with nothing near the path to edit" : note;
                        Count(noAddWhy, key);
                    }
                    TallyEdits(profile, model, commanded, objects, edits, editTally);
                    // A replay first goes back to where the source began, from where it ended, on the source's layout (SetupMove): straight or a
                    // joint at a time, or (since round 7's twin check) by way of the arm tucked up.
                    if (edits.Length > 0)
                    {
                        ArmCollisionModel back = SafetyScript.WithObjects(model, objects, 0.01f);
                        ArmPose end = plan.Sample(plan.TotalSeconds), begin = plan.Sample(plan.ScriptStart);
                        wayBack[0]++;
                        if (!DrivePlan.TryApproach(profile, back, end, begin, out _, out _)) wayBack[1]++;
                        if (!DrivePlan.TryApproach(profile, back, end, begin, out _, out _, tucked: true)) wayBack[2]++;
                    }

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
                if (features != null) featureRows += MoveFeatures.Write(features, layout, takeSeed, profile, plan, table, byMove);
                int before = frameJoint.Count;
                wild.Spans.Clear();
                foreach (PlannedMove m in plan.Moves) if (ScriptMoveCodec.IsAccident(m.Kind)) wild.Spans.Add((m.Start, m.End));
                Motion(profile, plan, frameJoint, frameWrist, tickJoint, jointSpeed, jointAccel, ref movingFrames, wild);
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
                Console.WriteLine(FormattableString.Invariant(
                    $"  replays' way back to where the source began (from its end, on its layout, 1 cm clear): none in {wayBack[1]} of {wayBack[0]} groups straight or a joint at a time; {wayBack[2]} also by way of the arm tucked up"));
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
                Console.WriteLine("  planned contacts' tool point, cm from the base frame (the table at " + (table * 100).ToString("0.0", CultureInfo.InvariantCulture) + "): " +
                    Spread(plannedTool) + "; by kind: " + Kinds(plannedKinds));
                Console.WriteLine(FormattableString.Invariant($"  contacts by the model over every move (the twin's links, each object's shape or box): {modelOnsets} onsets in {modelTakes} of {planned} takes; by kind: {Kinds(modelKinds)}; by move: {Kinds(modelParts)}"));
                Console.WriteLine(FormattableString.Invariant(
                    $"  accidents: {accidentRecords} in {accidentTakes} of {planned} takes ({accidentRecords / (double)planned:0.00} a take); by kind: {Kinds(accidentKinds)}; with a contact by the model, by kind: {Kinds(accidentKindContacts)}"));
                Console.WriteLine(FormattableString.Invariant(
                    $"  accidents' contacts by the model: {accidentOnsets} of {modelOnsets} onsets ({100.0 * accidentOnsets / Math.Max(1, modelOnsets):0}%) made under an accident; planned (on an aim or a pass): {plannedAccidentContacts}"));
                Console.WriteLine(FormattableString.Invariant(
                    $"  where accidents came: touching an object by the model {accidentsTouching}, within 5 cm of one's box {accidentsNear}, farther (free space) {accidentsFree}; begun in the first 1.5 s {accidentsQuietStart}, in the last 2.5 s {accidentsLate}"));
                double tpr = accidentContactRecords / (double)Math.Max(1, contactRecords), fpr = (accidentRecords - accidentContactRecords) / (double)Math.Max(1, records - contactRecords);
                Console.WriteLine(FormattableString.Invariant(
                    $"  'the move is an accident' for 'the move makes a contact' (by the model, each onset to the move last begun): AUC {0.5 * (1 + tpr - fpr):0.000} (of {records} records, {contactRecords} with a contact; accidents {accidentContactRecords} of them, and {accidentRecords - accidentContactRecords} of the {records - contactRecords} without)"));
                lagged.Print();
                Console.WriteLine("  their tool point, cm from the base frame: " + Spread(modelTool) + "; 5 cm bins from -20: " + Histogram(modelTool.Select(z => z * 100.0), -20, 5, 8));
                Console.WriteLine("  tool point where moves end, cm from the base frame: aims " + Spread(aimTool) + "; layout-blind moves to a goal " + Spread(blindTool));
                Console.WriteLine("  aims' ends, 5 cm bins from -20: " + Histogram(aimTool.Select(z => z * 100.0), -20, 5, 8) + "; blind goals': " + Histogram(blindTool.Select(z => z * 100.0), -20, 5, 8));
                Console.WriteLine("  aims and passes by object kind: " + Kinds(aimKinds));
                Console.WriteLine("  why a group has no add, or a replay no edit: " + Kinds(noAddWhy));
                Console.WriteLine("  replays' edits the model expects to change contact (n: the edited object's own contact changed / the take's any contact changed):");
                foreach (KeyValuePair<string, int[]> kv in editTally.OrderBy(k => k.Key, StringComparer.Ordinal))
                {
                    Console.WriteLine(FormattableString.Invariant($"    {kv.Key}: {kv.Value[0]}: {kv.Value[1]} ({100.0 * kv.Value[1] / Math.Max(1, kv.Value[0]):0}%) / {kv.Value[2]} ({100.0 * kv.Value[2] / Math.Max(1, kv.Value[0]):0}%)"));
                }
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

            if (wild.FrameJoint[1].Count > 0)
            {
                // docs/adr/0038 "M, motion": the same in accidents' spans (each from its onset to the hand's recovery), and every frame outside them.
                int inFrames = wild.FrameJoint[1].Count, outFrames = wild.FrameJoint[0].Count;
                Console.WriteLine(FormattableString.Invariant($"  in accidents' spans: {inFrames} of {inFrames + outFrames} frames ({100.0 * inFrames / Math.Max(1, inFrames + outFrames):0.0}%)"));
                for (int w = 0; w < 2; w++)
                {
                    Console.WriteLine((w == 0 ? "  outside them" : "  in them") + ", per frame: largest arm-joint change rad " + Percentiles(wild.FrameJoint[w], "0.0000") + "; wrist travel mm " +
                        Percentiles(wild.FrameWrist[w], "0.0") + "; fastest joint tick to tick rad/s " + Percentiles(wild.TickJoint[w], "0.00") +
                        FormattableString.Invariant($"; the command moves in {100.0 * wild.Moving[w] / Math.Max(1, wild.FrameJoint[w].Count):0}% of frames"));
                    for (int j = 0; j < 5; j++)
                        Console.WriteLine($"    {names[j]}, smoothed 25 ms: speed rad/s {Percentiles(wild.Speed[w][j], "0.00")}; acceleration rad/s2 {Percentiles(wild.Accel[w][j], "0.0")}");
                }
            }
            if (features != null) Console.WriteLine($"  features: {featureRows} moves of {planned} takes written to {featuresPath}");
            if (dry.Chain || dry.LagSeconds > 0) Console.WriteLine(FormattableString.Invariant($"  planned {(dry.Chain ? "each from where the last left the arm" : "from home")}; adds looked at the commanded way {dry.LagSeconds:0.00} s late"));
            Console.WriteLine("[joint-drive] dry run: nothing was sent");
            return 0;
        }

        /// <summary>
        /// The edits a group of two replays of <paramref name="plan"/> draws (docs/adr/0038 decision 2), as joint-drive draws them with nothing
        /// refused: the add looks where the plan's commands go, may stand the pool's parked obstacles on the table, and the objects the take
        /// moved are taken to be those its moves planned to touch. Fewer than two when a replay finds nothing new.
        /// </summary>
        internal static ReplayEditDraw[] GroupEdits(WorldScene scene, in RobotArmProfile profile, ArmCollisionModel model, DrivePlan plan, List<SafetyObject> objects,
            bool[] parked, double[] scales, ulong group, MixedOptions options, List<TrackPoint>? track = null, List<string>? notes = null)
        {
            var touched = new HashSet<byte>(plan.Moves.Where(m => m.ObjectId != 0 && !double.IsNaN(m.Contact)).Select(m => m.ObjectId));
            var parkedIds = new List<byte>();
            for (int i = 0; i < scene.Objects.Length; i++)
            {
                SceneObject o = scene.Objects[i];
                if (parked[i] && o.Movable && o.OperatorMovable && (!scene.Mixed || o.Role == SceneRole.Obstacle)) parkedIds.Add(o.Id);
            }

            var add = new ReplayAdd(track ?? ReplayAdd.Commanded(plan), plan.TotalSeconds - plan.ScriptStart, parkedIds, model, options);
            var near = new PathProximity(profile, model, add.Track, objects, 2);
            var used = new HashSet<string>();
            var edits = new List<ReplayEditDraw>();
            notes?.Clear();
            double ScaleOf(byte id)
            {
                int k = scene.IndexOf(id);
                return k >= 0 && scales[k] > 0 ? scales[k] : 1.0;
            }

            for (int r = 1; r <= 2; r++)
            {
                ReplayEditDraw e = ReplayEdits.Draw(group, r, 0, plan, objects, scene, touched, profile, used, ScaleOf, add, near);
                if (e.AddNote != null) notes?.Add(e.AddNote);
                if (e.Pose.Id == 0)
                {
                    notes?.Add(e.Note ?? "no edit");
                    break;
                }

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
        /// <summary>The motion figures split by whether they fall in an accident's span (docs/adr/0038 "M, motion"): index 0 outside every span, 1 in one.</summary>
        internal sealed class Wild
        {
            public readonly List<(double From, double To)> Spans = new List<(double, double)>();
            public readonly List<double>[] FrameJoint = { new List<double>(), new List<double>() }, FrameWrist = { new List<double>(), new List<double>() };
            public readonly List<double>[] TickJoint = { new List<double>(), new List<double>() };
            public readonly List<double>[][] Speed = { Five(), Five() }, Accel = { Five(), Five() };
            public readonly int[] Moving = new int[2];

            private static List<double>[] Five() => new[] { new List<double>(), new List<double>(), new List<double>(), new List<double>(), new List<double>() };

            /// <summary>1 when plan time <paramref name="t"/> is in a span, else 0.</summary>
            public int At(double t)
            {
                foreach ((double from, double to) in Spans) if (t >= from && t <= to) return 1;
                return 0;
            }
        }

        /// <summary>
        /// docs/adr/0038 "M, motion": the plan's records, each with its contacts by the model (<paramref name="byMove"/>, each onset given to the move
        /// last begun), tallied by whether it is an accident; and each accident by where it came: touching an object by the model, within 5 cm of
        /// one's box, or farther (free space); begun in the take's first 1.5 s or its last 2.5 s.
        /// </summary>
        internal static void Accidents(in RobotArmProfile profile, ArmCollisionModel model, DrivePlan plan, List<SafetyObject> objects, int[] byMove,
            ref int records, ref int contactRecords, ref int accidentRecords, ref int accidentContactRecords, ref int accidentOnsets, ref int touching, ref int near,
            ref int free, ref int quietStart, ref int late, ref int takes, Dictionary<string, int> kindContacts)
        {
            SafetyObject[] inReach = objects.Where(o => !o.Scenery).ToArray();
            ArmCollisionModel within5 = SafetyScript.WithObjects(model, inReach, 0.05f);
            bool any = false;
            for (int k = 0; k < plan.Moves.Count; k++)
            {
                PlannedMove m = plan.Moves[k];
                bool accident = ScriptMoveCodec.IsAccident(m.Kind), contact = byMove[k] > 0;
                records++;
                if (contact) contactRecords++;
                if (!accident) continue;
                any = true;
                accidentRecords++;
                accidentOnsets += byMove[k];
                if (contact)
                {
                    accidentContactRecords++;
                    touching++;
                    Count(kindContacts, ScriptMoveCodec.Name(m.Kind));
                }
                else
                {
                    bool close = false;
                    for (double t = Math.Max(m.Start, plan.ScriptStart); t <= Math.Min(m.End, plan.TotalSeconds) && !close; t += HandModel.TickSeconds)
                        close = !ArmCollision.IsValid(profile, within5, plan.Sample(t));
                    if (close) near++;
                    else free++;
                }

                if (m.Start - plan.ScriptStart < 1.5) quietStart++;
                if (plan.TotalSeconds - m.Start < 2.5) late++;
            }

            if (any) takes++;
        }

        internal static void Motion(in RobotArmProfile profile, DrivePlan plan, List<double> frameJoint, List<double> frameWrist, List<double> tickJoint,
            List<double>[] jointSpeed, List<double>[] jointAccel, ref int movingFrames, Wild? wild = null)
        {
            double tick = HandModel.TickSeconds, frame = 1.0 / FramesPerSecond;
            double start = plan.ScriptStart;
            var commands = new List<ArmPose>();
            for (double t = start; t <= plan.TotalSeconds + 1e-9; t += tick) commands.Add(plan.Sample(t));
            for (int k = 1; k < commands.Count; k++)
            {
                tickJoint.Add(commands[k].MaxJointDelta(commands[k - 1]) / tick);
                wild?.TickJoint[wild.At(start + k * tick)].Add(tickJoint[tickJoint.Count - 1]);
            }

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
                    if (wild == null) continue;
                    int inside = wild.At(start + k * tick);
                    wild.Speed[inside][j].Add(jointSpeed[j][jointSpeed[j].Count - 1]);
                    wild.Accel[inside][j].Add(jointAccel[j][jointAccel[j].Count - 1]);
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
                    if (wild != null)
                    {
                        int inside = wild.At(start + f);
                        wild.FrameJoint[inside].Add(d);
                        wild.FrameWrist[inside].Add(frameWrist[frameWrist.Count - 1]);
                        if (any >= 2 * 0.24 * Math.PI / 180) wild.Moving[inside]++;
                    }
                }

                previous = p;
            }
        }

        /// <summary>
        /// An object's kind for a report (docs/adr/0038 "M, motion"): its role when it is a stand, a plank's support or a plank; tall and thin
        /// (a rod, a dowel, a marker or a post standing: at least 12 cm tall and three times its narrower side); raised (on a stand); else other.
        /// </summary>
        internal static string KindOf(in SafetyObject o, WorldScene scene, float table)
        {
            int k = scene.IndexOf(o.Id);
            SceneRole role = k >= 0 ? scene.Objects[k].Role : SceneRole.None;
            if (role == SceneRole.Stand) return "stand";
            if (role == SceneRole.Support) return "support";
            if (role == SceneRole.Plank) return "plank";
            float height = o.Max.Z - o.Min.Z, narrow = 2f * MathF.Min(o.HalfExtents.X, o.HalfExtents.Y);
            if (height >= 0.12f && height >= 3f * narrow && MathF.Abs(o.HalfExtents.Z * 2f - height) < 0.01f) return "rod";
            return o.Min.Z > table + 0.01f ? "raised" : "other";
        }

        /// <summary>
        /// Every contact by the model along <paramref name="track"/> (every tick of the plan): each object in reach that is not scenery, against
        /// the twin's links (<see cref="ArmTouch.Touches"/>); one onset a stretch, with the tool point's height, the object's kind and the
        /// part of the plan that made it. How many onsets.
        /// </summary>
        internal static int ModelContacts(ArmTouch arm, DrivePlan plan, List<TrackPoint> track, List<SafetyObject> objects, WorldScene scene, float table,
            List<double> tool, Dictionary<string, int> kinds, Dictionary<string, int> parts, int[]? byMove = null)
        {
            var reach = new List<int>();
            for (int i = 0; i < objects.Count; i++)
            {
                SafetyObject o = objects[i];
                if (!o.Scenery && new Vector2(o.Center.X, o.Center.Y).Length() - MathF.Max(o.HalfExtents.X, o.HalfExtents.Y) < 0.45f) reach.Add(i);
            }

            var touching = new bool[objects.Count];
            int onsets = 0;
            IReadOnlyList<PlannedMove> moves = plan.Moves;
            foreach (TrackPoint p in track)
            {
                arm.At(p.Pose, p.Gripper);
                foreach (int i in reach)
                {
                    bool now = arm.Touches(objects[i]);
                    if (now && !touching[i])
                    {
                        onsets++;
                        tool.Add(arm.Tool.Z);
                        Count(kinds, KindOf(objects[i], scene, table));
                        // The move under way: the last to begin by then (they overlap as a hand's do), and whether it was aimed at this object.
                        double t = plan.ScriptStart + p.At;
                        PlannedMove? under = null;
                        int underIndex = -1;
                        for (int k = 0; k < moves.Count; k++)
                        {
                            if (moves[k].Start > t) continue;
                            under = moves[k];
                            underIndex = k;
                        }

                        if (byMove != null && underIndex >= 0) byMove[underIndex]++;
                        string part = under == null ? "none"
                            : ScriptMoveCodec.IsAccident(under.Value.Kind) ? "accident"
                            : under.Value.ObjectId == objects[i].Id ? "aimed" : under.Value.ObjectId != 0 ? "aimed elsewhere"
                            : under.Value.Part == MovePart.BlindTransport || under.Value.Part == MovePart.BlindStrokes || under.Value.Part == MovePart.BlindAdjust ? "blind"
                            : under.Value.Part == MovePart.Staging ? "staging" : under.Value.Part == MovePart.PullBack || under.Value.Part == MovePart.ForcedPullBack ? "pull-back" : "other";
                        Count(parts, part);
                    }

                    touching[i] = now;
                }
            }

            return onsets;
        }

        /// <summary>
        /// For each of a group's edits, whether the model expects it to change what the source's commands touch: the edited object's own contact,
        /// and whether the take touches anything at all; tallied by edit and by how near the source's arm came to the object.
        /// </summary>
        internal static void TallyEdits(in RobotArmProfile profile, ArmCollisionModel model, List<TrackPoint> track, List<SafetyObject> objects, ReplayEditDraw[] edits,
            Dictionary<string, int[]> tally)
        {
            if (edits.Length == 0) return;
            var near = new PathProximity(profile, model, track, objects, 2);
            bool anyBefore = false;
            for (int j = 0; j < objects.Count; j++) anyBefore |= near.Contact(j);
            foreach (ReplayEditDraw e in edits)
            {
                int i = objects.FindIndex(o => o.Id == e.Record.ObjectId);
                bool before = i >= 0 && near.Contact(i);
                bool after = e.After.HasValue && PathProximity.GapOver(near.Arm, track, e.After.Value, 2, out _) <= 0f;
                bool othersTouch = false;
                for (int j = 0; j < objects.Count; j++) if (j != i) othersTouch |= near.Contact(j);
                bool anyAfter = othersTouch || after;
                string where = i < 0 ? "new" : near.Contact(i) ? "contact" : near.Touched(i) ? "within 1 cm" : near.Near(i) ? "within 5 cm" : "farther";
                string key = ReplayCodec.Describe(e.Record).Split(':')[0] + "/" + where;
                if (!tally.TryGetValue(key, out int[]? row)) tally[key] = row = new int[3];
                row[0]++;
                if (before != after) row[1]++;
                if (anyBefore != anyAfter) row[2]++;
            }
        }

        /// <summary>The lag-aware contacts' tallies (<see cref="LagAwareContacts"/>).</summary>
        internal sealed class LagTally
        {
            public int Takes, Planned, Onsets, AccidentOnsets, Records, ContactRecords, Accidents, AccidentContacts, FreeAccidents, Placed, PlacedContacts;
            public int Events, AccidentEvents; // the onsets as contact events (ContactEvents), as the twin's physics makes them
            public int Like, AccidentLike; // and physics-like: a tippy movable object gone after its first contact event
            public readonly List<double> LikeTool = new List<double>();
            public readonly Dictionary<string, int[]> LikeByKind = new Dictionary<string, int[]>();
            public readonly List<(int Accidents, List<double> Tool)> LikeTakes = new List<(int, List<double>)>(); // each take's physics-like events

            /// <summary>
            /// What a twin pilot of <paramref name="takes"/> source takes would show (docs/adr/0038 "M, motion", round 7 on the twin): takes drawn
            /// with replacement from the dry run's, 2000 times (seeded): the physics-like accident share's p10 / p50 / p90, and how often the
            /// largest 5 cm band of the tool point holds more than 40%.
            /// </summary>
            public string Pilot(int takes)
            {
                if (LikeTakes.Count == 0) return "none";
                var rng = new SeededRng(0x5EED_0007UL + (ulong)takes);
                var shares = new List<double>();
                int over = 0;
                for (int b = 0; b < 2000; b++)
                {
                    int acc = 0;
                    var bins = new int[64];
                    int n = 0;
                    for (int k = 0; k < takes; k++)
                    {
                        (int a, List<double> zs) = LikeTakes[(int)(rng.NextDouble() * LikeTakes.Count) % LikeTakes.Count];
                        acc += a;
                        foreach (double z in zs) bins[Math.Clamp((int)Math.Floor((z * 100.0 + 20.0) / 5.0), 0, 63)]++;
                        n += zs.Count;
                    }

                    if (n == 0) continue;
                    shares.Add(acc / (double)n);
                    if (bins.Max() > 0.4 * n) over++;
                }

                shares.Sort();
                string P(double q) => (100 * shares[Math.Min(shares.Count - 1, (int)Math.Round(q * (shares.Count - 1)))]).ToString("0", CultureInfo.InvariantCulture);
                return FormattableString.Invariant($"{takes} source takes: accident share p10 {P(0.1)}%, p50 {P(0.5)}%, p90 {P(0.9)}%; a band over 40% in {100.0 * over / Math.Max(1, shares.Count):0}%");
            }
            public readonly List<double> EventTool = new List<double>();
            public readonly Dictionary<string, int[]> EventsByKind = new Dictionary<string, int[]>(); // accident kind: events begun in its spans
            public readonly Dictionary<string, int[]> PlacedByKind = new Dictionary<string, int[]>();
            public readonly List<double> Tool = new List<double>(), Point = new List<double>();
            public readonly Dictionary<string, int> Links = new Dictionary<string, int>();
            public readonly Dictionary<string, int> Parts = new Dictionary<string, int>();
            public readonly Dictionary<string, List<double>> PointByLink = new Dictionary<string, List<double>>();

            public void Print()
            {
                double tpr = AccidentContacts / (double)Math.Max(1, ContactRecords), fpr = (Accidents - AccidentContacts) / (double)Math.Max(1, Records - ContactRecords);
                Console.WriteLine(FormattableString.Invariant(
                    $"  lag-aware contacts (the follower: {TakeAudit.FollowDelay:0.00} s late, tau {TakeAudit.FollowTau:0.00} s, {TakeAudit.FollowSpeed:0.0} rad/s): {Onsets} onsets in {Takes} of {Planned} takes; in an accident's span (+{LagSeconds:0.0} s) {AccidentOnsets} ({100.0 * AccidentOnsets / Math.Max(1, Onsets):0}%); by move: {Kinds(Parts)}"));
                Console.WriteLine(FormattableString.Invariant(
                    $"    accidents {Accidents}, with a lag-aware contact {AccidentContacts} ({100.0 * AccidentContacts / Math.Max(1, Accidents):0}%), farther than 10 cm from every object's box (free space) {FreeAccidents}; 'the record is an accident' for 'it makes a contact': AUC {0.5 * (1 + tpr - fpr):0.000} (of {Records} records, {ContactRecords} with a contact)"));
                Console.WriteLine(FormattableString.Invariant($"    of the accidents, placed near an object {Placed}, with a contact {PlacedContacts}; by kind (placed, with a contact): ") +
                                  string.Join(", ", PlacedByKind.OrderBy(k => k.Key, StringComparer.Ordinal).Select(k => $"{k.Key} {k.Value[0]}/{k.Value[1]}")));
                Console.WriteLine("    tool point cm, 5 cm bins from -20: " + Histogram(Tool.Select(z => z * 100.0), -20, 5, 8) + " (largest band " + Largest(Tool) + "); " + Spread(Tool));
                Console.WriteLine(FormattableString.Invariant(
                    $"    as contact events (a re-touch within {ContactEvents.MergeSeconds:0.0} s of the object's release the same contact): {Events}, in an accident's span {AccidentEvents} ({100.0 * AccidentEvents / Math.Max(1, Events):0}%); tool point 5 cm bins from -20: ") +
                    Histogram(EventTool.Select(z => z * 100.0), -20, 5, 8) + " (largest band " + Largest(EventTool) + "); " + Spread(EventTool) + "; by accident kind: " +
                    string.Join(", ", EventsByKind.OrderBy(k => k.Key, StringComparer.Ordinal).Select(k => $"{k.Key} {k.Value[0]}")));
                Console.WriteLine(FormattableString.Invariant(
                    $"    physics-like (the events, a tippy movable object knocked over at its first and lying after): {Like}, in an accident's span {AccidentLike} ({100.0 * AccidentLike / Math.Max(1, Like):0}%); tool point 5 cm bins from -20: ") +
                    Histogram(LikeTool.Select(z => z * 100.0), -20, 5, 8) + " (largest band " + Largest(LikeTool) + "); " + Spread(LikeTool) + "; by accident kind: " +
                    string.Join(", ", LikeByKind.OrderBy(k => k.Key, StringComparer.Ordinal).Select(k => $"{k.Key} {k.Value[0]}")));
                Console.WriteLine("    a twin pilot's spread, physics-like (takes resampled): " + Pilot(8) + "; " + Pilot(20) + "; " + Pilot(60));
                Console.WriteLine("    contact point cm, 5 cm bins from -20: " + Histogram(Point.Select(z => z * 100.0), -20, 5, 8) + " (largest band " + Largest(Point) + ")");
                Console.WriteLine("    touching link: " + Kinds(Links) + "; contact point by link, p50 cm: " +
                                  string.Join(", ", PointByLink.OrderBy(k => k.Key, StringComparer.Ordinal).Select(k => k.Key + " " + (k.Value.OrderBy(x => x).ElementAt(k.Value.Count / 2) * 100).ToString("0.0", CultureInfo.InvariantCulture))));
            }

            private static string Largest(List<double> zs)
            {
                if (zs.Count == 0) return "none";
                var bins = new int[64];
                foreach (double z in zs) bins[Math.Clamp((int)Math.Floor((z * 100.0 + 20.0) / 5.0), 0, 63)]++;
                return (100.0 * bins.Max() / zs.Count).ToString("0", CultureInfo.InvariantCulture) + "%";
            }
        }

        /// <summary>
        /// Contact events (docs/adr/0038 "M, motion", round 7 on the twin): a touch of an object that begins within <see cref="MergeSeconds"/>
        /// of the end of its last touch of that object continues that contact. Physics breaks a sustained push into several touches as the
        /// object gives and the arm presses on, and the model, whose arm passes through an object, counts one going in and another coming
        /// back; neither is a new contact.
        /// </summary>
        internal sealed class ContactEvents
        {
            public const double MergeSeconds = 0.5;
            private readonly Dictionary<int, double> _end = new Dictionary<int, double>();

            /// <summary>Object <paramref name="id"/> touched from <paramref name="t"/>: true when that begins a new contact.</summary>
            public bool Begin(int id, double t) => !_end.TryGetValue(id, out double e) || t - e > MergeSeconds;

            /// <summary>Object <paramref name="id"/> no longer touched from <paramref name="t"/>.</summary>
            public void Stop(int id, double t) => _end[id] = t;
        }

        /// <summary>
        /// Contacts as the twin's physics makes them (docs/adr/0038 "M, motion", round 7 on the twin): contact events (<see cref="ContactEvents"/>),
        /// and a tippy movable object knocked over at its first: from then on it lies on the table where it fell (<see cref="Fallen"/>), and only
        /// that is met. On the twin's pilots 28 of the 32 tippy objects touched tipped or slid away at the first touch; the model's later
        /// contacts with them as they stood (40 events) were 10 in physics, low, with the object lying.
        /// </summary>
        internal sealed class PhysicsLike
        {
            private readonly SafetyObject[] _objects;
            private readonly bool[] _fallen, _touching;
            private readonly ContactEvents _events = new ContactEvents();

            public PhysicsLike(IReadOnlyList<SafetyObject> objects)
            {
                _objects = objects.ToArray();
                _fallen = new bool[_objects.Length];
                _touching = new bool[_objects.Length];
            }

            /// <summary>
            /// Object <paramref name="i"/> at time <paramref name="t"/>, the arm already placed (<see cref="ArmTouch.At"/>) and its touch of the
            /// object as it stood given (<paramref name="touchingAsItStood"/>, <paramref name="link"/>): true when a new contact begins.
            /// </summary>
            public bool Step(ArmTouch arm, int i, double t, bool touchingAsItStood, ref ArmLink link)
            {
                bool now = touchingAsItStood;
                if (_fallen[i]) now = arm.Touching(_objects[i], out link, out _);
                if (!now && _touching[i]) _events.Stop(_objects[i].Id, t);
                bool fresh = now && !_touching[i] && _events.Begin(_objects[i].Id, t);
                _touching[i] = now;
                if (fresh && !_fallen[i] && _objects[i].Movable && _objects[i].Tippy)
                {
                    _fallen[i] = true;
                    _objects[i] = Fallen(_objects[i]);
                    _events.Stop(_objects[i].Id, t); // touching it again as it falls is the same contact
                    _touching[i] = false;
                }

                return fresh;
            }
        }

        /// <summary>
        /// A tippy object knocked over (<see cref="PhysicsLike"/>): its box lying on what it stood on, its height out along its bearing from the
        /// base (an arm pushes outward more often than in), its narrowest side up, pivoted about its far bottom edge.
        /// </summary>
        internal static SafetyObject Fallen(in SafetyObject o)
        {
            float h = (o.Max.Z - o.Min.Z) / 2f;
            Vector3 he = o.HalfExtents;
            float narrow = MathF.Min(he.X, MathF.Min(he.Y, he.Z)), wide = he.X + he.Y + he.Z - MathF.Max(he.X, MathF.Max(he.Y, he.Z)) - narrow;
            var radial = new Vector2(o.Center.X, o.Center.Y);
            float r = radial.Length();
            radial = r > 1e-4f ? radial / r : Vector2.UnitX;
            Vector2 c = new Vector2(o.Center.X, o.Center.Y) + radial * MathF.Max(0f, h - narrow);
            float yaw = MathF.Atan2(radial.Y, radial.X);
            return new SafetyObject(o.Id, new Vector3(c.X, c.Y, o.Min.Z + narrow), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, yaw), new Vector3(h, MathF.Max(narrow, wide), narrow),
                o.Movable, o.Mass, false, null, o.Scenery, o.Role);
        }

        /// <summary>How long after an accident's recorded end a contact is still its own: the arm's lag behind the command (docs/adr/0038 "M, motion").</summary>
        public const double LagSeconds = 0.2;

        /// <summary>The plan's commands as the twin's arm follows them (<see cref="TakeAudit"/>'s follower), every tick from the take's start.</summary>
        internal static List<TrackPoint> Followed(DrivePlan plan)
        {
            var track = new List<TrackPoint>();
            double tau = Math.Max(1e-3, TakeAudit.FollowTau), dt = HandModel.TickSeconds, a = 1 - Math.Exp(-dt / tau), max = TakeAudit.FollowSpeed * dt;
            ArmPose first = plan.Sample(plan.ScriptStart);
            float[] q = { first.BaseYaw, first.Proximal, first.Distal, first.Wrist, first.Roll };
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds + 1e-9; t += dt)
            {
                double at = Math.Max(plan.ScriptStart, t - TakeAudit.FollowDelay);
                ArmPose c = plan.Sample(at);
                float[] target = { c.BaseYaw, c.Proximal, c.Distal, c.Wrist, c.Roll };
                for (int j = 0; j < 5; j++) q[j] += (float)Math.Clamp((target[j] - q[j]) * a, -max, max);
                float g = plan.GripperAt(at);
                track.Add(new TrackPoint(t - plan.ScriptStart, TakeAudit.Sagged(q[0], q[1], q[2], q[3], q[4]), float.IsNaN(g) ? 0f : g));
            }

            return track;
        }

        /// <summary>
        /// docs/adr/0038 "M, motion": every contact by the model along <paramref name="track"/> (the follower of the plan's commands), as the
        /// twin's audit counts them on a recorded take (<see cref="TakeAudit"/>): one onset a stretch an object is touched; given to the accident
        /// whose span and <see cref="LagSeconds"/> after holds it, else to the move under way <see cref="TakeAudit.FollowDelay"/> before.
        /// </summary>
        internal static void LagAwareContacts(ArmTouch arm, DrivePlan plan, List<TrackPoint> track, List<SafetyObject> objects, LagTally tally)
        {
            var reach = new List<int>();
            for (int i = 0; i < objects.Count; i++)
            {
                SafetyObject o = objects[i];
                if (!o.Scenery && new Vector2(o.Center.X, o.Center.Y).Length() - MathF.Max(o.HalfExtents.X, o.HalfExtents.Y) < 0.45f) reach.Add(i);
            }

            IReadOnlyList<PlannedMove> moves = plan.Moves;
            var contact = new bool[moves.Count];
            var touching = new bool[objects.Count];
            int onsets = 0;
            var events = new ContactEvents();
            var like = new PhysicsLike(objects);
            int takeLikeAccidents = 0;
            var takeLikeTool = new List<double>();
            foreach (TrackPoint p in track)
            {
                arm.At(p.Pose, p.Gripper);
                foreach (int i in reach)
                {
                    bool now = arm.Touching(objects[i], out ArmLink link, out Vector3 point);
                    if (!now && touching[i]) events.Stop(objects[i].Id, p.At);
                    ArmLink likeLink = link;
                    if (like.Step(arm, i, p.At, now, ref likeLink))
                    {
                        double tl = plan.ScriptStart + p.At;
                        tally.Like++;
                        tally.LikeTool.Add(arm.Tool.Z);
                        takeLikeTool.Add(arm.Tool.Z);
                        for (int k = 0; k < moves.Count; k++)
                        {
                            if (!ScriptMoveCodec.IsAccident(moves[k].Kind) || tl < moves[k].Start || tl > moves[k].End + LagSeconds) continue;
                            tally.AccidentLike++;
                            takeLikeAccidents++;
                            string kn = ScriptMoveCodec.Name(moves[k].Kind) + (moves[k].Part == MovePart.NearAccident ? " placed" : "");
                            if (!tally.LikeByKind.TryGetValue(kn, out int[]? lk)) tally.LikeByKind[kn] = lk = new int[1];
                            lk[0]++;
                            break;
                        }
                    }

                    if (now && !touching[i])
                    {
                        onsets++;
                        double t = plan.ScriptStart + p.At;

                        if (events.Begin(objects[i].Id, p.At))
                        {
                            tally.Events++;
                            tally.EventTool.Add(arm.Tool.Z);
                            for (int k = 0; k < moves.Count; k++)
                            {
                                if (!ScriptMoveCodec.IsAccident(moves[k].Kind) || t < moves[k].Start || t > moves[k].End + LagSeconds) continue;
                                tally.AccidentEvents++;
                                string kn = ScriptMoveCodec.Name(moves[k].Kind) + (moves[k].Part == MovePart.NearAccident ? " placed" : "");
                                if (!tally.EventsByKind.TryGetValue(kn, out int[]? ek)) tally.EventsByKind[kn] = ek = new int[1];
                                ek[0]++;
                                break;
                            }
                        }

                        int owner = -1;
                        for (int k = 0; k < moves.Count && owner < 0; k++)
                            if (ScriptMoveCodec.IsAccident(moves[k].Kind) && t >= moves[k].Start && t <= moves[k].End + LagSeconds) owner = k;
                        bool accident = owner >= 0;
                        if (!accident)
                            for (int k = 0; k < moves.Count; k++)
                                if (!ScriptMoveCodec.IsAccident(moves[k].Kind) && moves[k].Start <= t - TakeAudit.FollowDelay) owner = k;
                        if (owner >= 0) contact[owner] = true;
                        if (accident) tally.AccidentOnsets++;
                        string part = accident ? "accident" : owner < 0 ? "none" : moves[owner].ObjectId == objects[i].Id ? "aimed" : moves[owner].ObjectId != 0 ? "aimed elsewhere" : "other";
                        Count(tally.Parts, part);
                        tally.Tool.Add(arm.Tool.Z);
                        tally.Point.Add(point.Z);
                        string name = link.ToString().ToLowerInvariant();
                        Count(tally.Links, name);
                        if (!tally.PointByLink.TryGetValue(name, out List<double>? zs)) tally.PointByLink[name] = zs = new List<double>();
                        zs.Add(point.Z);
                    }

                    touching[i] = now;
                }
            }

            tally.LikeTakes.Add((takeLikeAccidents, takeLikeTool));
            tally.Planned++;
            tally.Onsets += onsets;
            if (onsets > 0) tally.Takes++;
            // Accidents in free space: farther than 10 cm from every object's box over the span, by the model (the commanded way).
            ArmCollisionModel? within10 = null;
            for (int k = 0; k < moves.Count; k++)
            {
                tally.Records++;
                if (contact[k]) tally.ContactRecords++;
                if (!ScriptMoveCodec.IsAccident(moves[k].Kind)) continue;
                tally.Accidents++;
                if (moves[k].Part == MovePart.NearAccident)
                {
                    tally.Placed++;
                    if (contact[k]) tally.PlacedContacts++;
                    string kn = ScriptMoveCodec.Name(moves[k].Kind);
                    if (!tally.PlacedByKind.TryGetValue(kn, out int[]? pk)) tally.PlacedByKind[kn] = pk = new int[2];
                    pk[0]++;
                    if (contact[k]) pk[1]++;
                }

                if (contact[k])
                {
                    tally.AccidentContacts++;
                    continue;
                }

                within10 ??= SafetyScript.WithObjects(arm.Model, objects.Where(o => !o.Scenery).ToArray(), 0.10f);
                bool close = false;
                for (double t = Math.Max(moves[k].Start, plan.ScriptStart); t <= Math.Min(moves[k].End, plan.TotalSeconds) && !close; t += HandModel.TickSeconds)
                    close = !ArmCollision.IsValid(arm.Profile, within10, plan.Sample(t));
                if (!close) tally.FreeAccidents++;
            }
        }

        /// <summary>The plan's commanded way <paramref name="lag"/> seconds late, every tick from the take's start: roughly where a measured track has the arm.</summary>
        internal static List<TrackPoint> Lagged(DrivePlan plan, double lag)
        {
            var track = new List<TrackPoint>();
            for (double t = plan.ScriptStart; t <= plan.TotalSeconds + 1e-9; t += HandModel.TickSeconds)
            {
                double at = Math.Max(plan.ScriptStart, t - lag);
                float g = plan.GripperAt(at);
                track.Add(new TrackPoint(t - plan.ScriptStart, plan.Sample(at), float.IsNaN(g) ? 0f : g));
            }

            return track;
        }

        private static void Count(Dictionary<string, int> counts, string key) => counts[key] = counts.TryGetValue(key, out int n) ? n + 1 : 1;

        /// <summary>Which stage turned an add's spots down most, from its note's tally (too high, the bearing, the scene's rules, the quiet start and end).</summary>
        private static string Reason(string note)
        {
            string[] names = { "too high for the object", "beyond the reach bearing", "off the table, out of reach or on another object", "met the quiet start or end" };
            string[] keys = { "for the object ", "reach bearing ", "on another object ", "quiet start or end " };
            int best = -1;
            long most = 0;
            for (int k = 0; k < keys.Length; k++)
            {
                int at = note.IndexOf(keys[k], StringComparison.Ordinal);
                if (at < 0) continue;
                string digits = new string(note.Substring(at + keys[k].Length).TakeWhile(char.IsDigit).ToArray());
                if (long.TryParse(digits, out long n) && (k == 3 ? n > 0 : n > most) && (k != 3 || best < 0 || n > 0))
                {
                    if (k == 3 && n > 0) return names[3]; // a spot was found and the quiet windows refused it
                    most = n;
                    best = k;
                }
            }

            return best < 0 ? "none" : "mostly " + names[best];
        }

        private static string Kinds(Dictionary<string, int> counts)
        {
            int total = counts.Values.Sum();
            return total == 0 ? "none" : string.Join(", ", counts.OrderByDescending(k => k.Value).ThenBy(k => k.Key, StringComparer.Ordinal)
                .Select(k => FormattableString.Invariant($"{k.Key} {k.Value} ({100.0 * k.Value / total:0}%)")));
        }

        /// <summary>p10 / p50 / p90 in cm, and the share above -8 cm (base frame z, m in).</summary>
        private static string Spread(List<double> zs)
        {
            if (zs.Count == 0) return "none";
            var s = zs.OrderBy(z => z).ToList();
            string P(double p) => (s[Math.Min(s.Count - 1, (int)Math.Round(p / 100.0 * (s.Count - 1)))] * 100).ToString("0.0", CultureInfo.InvariantCulture);
            int above = zs.Count(z => z > -0.08);
            return FormattableString.Invariant($"p10 {P(10)}, p50 {P(50)}, p90 {P(90)}; above -8 cm {above} of {zs.Count} ({100.0 * above / zs.Count:0}%)");
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
