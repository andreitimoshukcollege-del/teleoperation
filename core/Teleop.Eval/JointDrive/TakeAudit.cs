using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// docs/adr/0038 "M, motion" (accidents on the twin): a recorded mixed take read back against the collision model. From its
    /// <c>robothost.jlog</c>: the planned moves (<c>MV</c>), the commands (<c>JC</c>, with RobotHost's clamps), the measured joints (<c>FR</c>),
    /// where every object stood (<c>WO</c>, <c>WZ</c>) and what physics touched (<c>WC</c>). For each accident, how close the commanded and
    /// the measured arm came to an object by the model in its span, and whether a physics contact began in it; for each physics contact, the
    /// move under way, the gap the command had there, the tool point's height and the touching link. Measures only; plans nothing.
    /// </summary>
    internal static class TakeAudit
    {
        private const double Ns = 1e9;
        private static readonly string[] LinkNames = { "base", "turntable", "proximal", "distal", "wrist", "gripper", "left_finger", "right_finger" };

        private readonly struct Sample
        {
            public readonly double T;
            public readonly ArmPose Pose;
            public readonly float Gripper;
            public readonly bool Clamped;

            public Sample(double t, ArmPose pose, float gripper, bool clamped)
            {
                T = t;
                Pose = pose;
                Gripper = gripper;
                Clamped = clamped;
            }
        }

        private sealed class Move
        {
            public double T, Start, End;
            public string Kind = "";
            public int Object;
        }

        internal static int Run(WorldScene scene, in RobotArmProfile profile, IReadOnlyList<string> takes, double lag, bool rows)
        {
            ArmCollisionModel model = profile.Collision!;
            ObjectShapes? shapes = WorldView.ShapesOf(scene);
            int accidents = 0, accidentsNearCmd = 0, accidentsTouchCmd = 0, accidentsTouchCmdLate = 0, accidentsTouchMeas = 0, accidentsPhysics = 0, accidentsClamped = 0;
            int onsets = 0, onsetsInAccident = 0, onsetsCmdTouch = 0;
            var lags = new List<double>();
            var onsetTool = new List<double>();
            var onsetLink = new Dictionary<string, int>();
            var kindRows = new Dictionary<string, int[]>(); // kind: n, cmd within 2 cm, cmd touch, meas touch, physics
            var cmdGap = new List<double>();
            var measGap = new List<double>();
            var agree = new int[4, 4]; // predictor: commanded in, commanded in for the lag, the follower in, the measured arm in; TP FP FN TN against physics
            var followErr = new List<double>();
            var sourceOnsets = new int[2];
            var causes = new[] { new HashSet<Move?>(), new HashSet<Move?>() };
            // The dry run's contacts on the recorded commands (docs/adr/0038 "M, motion", round 7 on the twin): the follower of the take's own
            // commands against the objects where they stood as it began, counted as the dry run counts them (MixedDryRun.LagAwareContacts), and
            // matched onset by onset to physics: the same object within MatchSeconds.
            var byModel = new ContactTally();
            var phys = new ContactTally();
            var byModelEvents = new ContactTally();
            var byMovingEvents = new ContactTally();
            var toolBias = new List<double>();
            var objectFates = new Dictionary<string, int[]>(); // fixed, tipped, moved, stayed: objects, later physics events, later model events
            var jointBias = new[] { new List<double>(), new List<double>(), new List<double>(), new List<double>(), new List<double>() };
            var physEvents = new ContactTally();
            var match = new int[2, 3]; // [other, accident] x [both, only the model, only physics]
            int physicsMerged = 0;
            foreach (string take in takes)
            {
                string log = Path.Combine(take, "robothost.jlog");
                if (!File.Exists(log)) continue;
                var moves = new List<Move>();
                var cmd = new List<Sample>();
                var meas = new List<Sample>();
                var poses = new List<(double T, Dictionary<byte, double[]> At)>();
                var scale = new Dictionary<byte, double>();
                var physics = new List<(double T, int Object, int[] Links)>();
                var touching = new Dictionary<int, int[]>();
                var ends = new List<(double T, int Object)>();
                string group = "";
                foreach (string line in File.ReadLines(log))
                {
                    string[] p = line.Split('|');
                    switch (p[0])
                    {
                        case "RP":
                            group = p.Length > 3 ? p[2] + " " + p[3] + (p.Length > 4 ? " " + p[4] : "") : "";
                            break;
                        case "MV":
                        {
                            double t = long.Parse(p[1], CultureInfo.InvariantCulture) / Ns;
                            moves.Add(new Move { T = t, Kind = p[4], Object = int.Parse(p[5], CultureInfo.InvariantCulture), Start = t + D(p[8]), End = t + D(p[11]) });
                            break;
                        }
                        case "JC":
                            if (p[5] != "A" && p[5] != "C") break;
                            if (TryJoints(p[6], 1, out ArmPose c, out float g)) cmd.Add(new Sample(long.Parse(p[2], CultureInfo.InvariantCulture) / Ns, c, g, p[5] == "C"));
                            break;
                        case "FR":
                            if (TryJoints(p[7], 2, out ArmPose m, out float mg)) meas.Add(new Sample(long.Parse(p[4], CultureInfo.InvariantCulture) / Ns, m, mg, false));
                            break;
                        case "WZ":
                            foreach (string e in p[3].Split(';', StringSplitOptions.RemoveEmptyEntries))
                            {
                                string[] kv = e.Split(':');
                                scale[byte.Parse(kv[0], CultureInfo.InvariantCulture)] = D(kv[1]);
                            }

                            break;
                        case "WO":
                        {
                            var at = new Dictionary<byte, double[]>();
                            foreach (string e in p[4].Split(';', StringSplitOptions.RemoveEmptyEntries))
                            {
                                string[] v = e.Split(':');
                                at[byte.Parse(v[0], CultureInfo.InvariantCulture)] = v.Skip(1).Select(D).ToArray();
                            }

                            poses.Add((long.Parse(p[2], CultureInfo.InvariantCulture) / Ns, at));
                            break;
                        }
                        case "WC":
                        {
                            double t = long.Parse(p[2], CultureInfo.InvariantCulture) / Ns;
                            var now = new Dictionary<int, List<int>>();
                            if (p.Length > 3)
                            {
                                foreach (string e in p[3].Split(';', StringSplitOptions.RemoveEmptyEntries))
                                {
                                    string[] v = e.Split(':');
                                    int o = int.Parse(v[2], CultureInfo.InvariantCulture);
                                    if (!now.TryGetValue(o, out List<int>? l)) now[o] = l = new List<int>();
                                    l.Add(int.Parse(v[1], CultureInfo.InvariantCulture));
                                }
                            }

                            foreach (KeyValuePair<int, List<int>> kv in now)
                                if (!touching.ContainsKey(kv.Key)) physics.Add((t, kv.Key, kv.Value.ToArray()));
                            foreach (int gone in touching.Keys)
                                if (!now.ContainsKey(gone)) ends.Add((t, gone));
                            touching = now.ToDictionary(k => k.Key, k => k.Value.ToArray());
                            break;
                        }
                    }
                }

                if (moves.Count == 0 || cmd.Count == 0) continue;
                double t0 = moves.Min(m => m.Start);
                double takeLag = Lag(cmd, meas);
                List<Sample> follow = Follow(cmd, FollowDelay, FollowTau, FollowSpeed);
                followErr.Add(Rms(follow, meas));
                lags.Add(takeLag);
                if (rows) Console.WriteLine(FormattableString.Invariant($"  {Path.GetFileName(take)} ({group}): lag {takeLag:0.00} s, {cmd.Count(c => c.Clamped)} clamped of {cmd.Count}"));

                List<SafetyObject> ObjectsAt(double t)
                {
                    var objects = new List<SafetyObject>();
                    if (poses.Count == 0) return objects;
                    int k = 0;
                    while (k + 1 < poses.Count && poses[k + 1].T <= t) k++;
                    foreach (KeyValuePair<byte, double[]> kv in poses[k].At)
                    {
                        int i = scene.IndexOf(kv.Key);
                        double[] v = kv.Value;
                        if (i < 0 || v.Length < 7 || v[2] < -1.0) continue; // parked on the shelf
                        SceneObject o = scene.Objects[i];
                        bool scenery = (o.Movable && scene.InEdgeRegions(v[0], v[1])) || o.Role == SceneRole.Distractor;
                        objects.Add(WorldView.Bounds(o, scale.TryGetValue(kv.Key, out double s) ? s : 1.0, v[0], v[1], v[2], v[3], v[4], v[5], v[6], shapes, scenery));
                    }

                    return objects;
                }

                var arm = new ArmTouch(profile, model);
                float MinGap(Sample s, List<SafetyObject> objects, out int id)
                {
                    arm.At(s.Pose, s.Gripper);
                    float best = float.PositiveInfinity;
                    id = 0;
                    foreach (SafetyObject o in objects)
                    {
                        if (o.Scenery) continue;
                        float g = arm.Gap(o, 0.10f);
                        if (g < best)
                        {
                            best = g;
                            id = o.Id;
                        }
                    }

                    return best;
                }

                foreach (Move a in moves)
                {
                    if (!IsAccident(a.Kind)) continue;
                    accidents++;
                    List<SafetyObject> objects = ObjectsAt(a.Start);
                    float gc = float.PositiveInfinity, gm = float.PositiveInfinity;
                    int clamped = 0;
                    double touchAt = double.NaN, lastTouch = double.NaN;
                    foreach (Sample s in cmd)
                    {
                        if (s.T < a.Start || s.T > a.End) continue;
                        if (s.Clamped) clamped++;
                        float g = MinGap(s, objects, out _);
                        if (g <= 0f)
                        {
                            if (double.IsNaN(touchAt)) touchAt = s.T;
                            lastTouch = s.T;
                        }

                        gc = MathF.Min(gc, g);
                    }

                    foreach (Sample s in meas)
                    {
                        if (s.T < a.Start || s.T > a.End + lag) continue;
                        gm = MathF.Min(gm, MinGap(s, objects, out _));
                    }

                    float gf = float.PositiveInfinity;
                    foreach (Sample s in follow)
                    {
                        if (s.T < a.Start || s.T > a.End + lag) continue;
                        gf = MathF.Min(gf, MinGap(s, objects, out _));
                    }

                    bool hit = physics.Any(c => c.Object != 0 && c.T >= a.Start && c.T <= a.End + lag);
                    // A command into an object for at least the arm's lag: the measured arm, trailing it, gets there too.
                    bool longTouch = !double.IsNaN(touchAt) && lastTouch - touchAt >= lag - 1e-6;
                    if (gc <= 0.02f) accidentsNearCmd++;
                    if (gc <= 0f) accidentsTouchCmd++;
                    if (longTouch) accidentsTouchCmdLate++;
                    Tally(agree, 0, gc <= 0f, hit);
                    Tally(agree, 1, longTouch, hit);
                    Tally(agree, 2, gf <= 0f, hit);
                    Tally(agree, 3, gm <= 0f, hit);
                    if (gm <= 0f) accidentsTouchMeas++;
                    if (hit) accidentsPhysics++;
                    if (clamped > 0) accidentsClamped++;
                    cmdGap.Add(float.IsInfinity(gc) ? 0.1 : gc);
                    measGap.Add(float.IsInfinity(gm) ? 0.1 : gm);
                    if (!kindRows.TryGetValue(a.Kind, out int[]? row)) kindRows[a.Kind] = row = new int[6];
                    row[0]++;
                    if (gc <= 0.02f) row[1]++;
                    if (gc <= 0f) row[2]++;
                    if (longTouch) row[3]++;
                    if (gm <= 0f) row[4]++;
                    if (hit) row[5]++;
                    if (rows)
                        Console.WriteLine(FormattableString.Invariant(
                            $"    {a.Kind,-9} {a.Start - t0,5:0.00}-{a.End - t0,5:0.00} s{(a.Object != 0 ? $" on aim {a.Object}" : "")}: closest by the model, commanded {Mm(gc)}, measured {Mm(gm)}; in the command {(double.IsNaN(touchAt) ? 0 : lastTouch - touchAt):0.00} s; clamped {clamped}; physics {(hit ? "contact" : "none")}"));
                }

                // The dry run's view of this take: the follower's onsets on the objects as they stood when it began.
                List<SafetyObject> atStart = ObjectsAt(t0);
                var reach = atStart.Where(o => !o.Scenery && new System.Numerics.Vector2(o.Center.X, o.Center.Y).Length() - MathF.Max(o.HalfExtents.X, o.HalfExtents.Y) < 0.45f).ToList();
                var modelOnsets = new List<(double T, int Object, bool Accident)>();
                var inside = new bool[reach.Count];
                var modelEvents = new MixedDryRun.ContactEvents();
                var modelEventTimes = new List<(double T, int Object)>();
                foreach (Sample s in follow)
                {
                    arm.At(s.Pose, s.Gripper);
                    for (int i = 0; i < reach.Count; i++)
                    {
                        bool now = arm.Touching(reach[i], out ArmLink link, out System.Numerics.Vector3 point);
                        if (now && !inside[i])
                        {
                            bool acc = moves.Any(m => IsAccident(m.Kind) && s.T >= m.Start && s.T <= m.End + lag);
                            modelOnsets.Add((s.T, reach[i].Id, acc));
                            string ln = link == ArmLink.Gripper ? "gripper" : link == ArmLink.Wrist ? "wrist" : link.ToString().ToLowerInvariant();
                            byModel.Add(acc, arm.Tool.Z, ln);
                            if (modelEvents.Begin(reach[i].Id, s.T))
                            {
                                byModelEvents.Add(acc, arm.Tool.Z, ln);
                                modelEventTimes.Add((s.T, reach[i].Id));
                            }
                            if (rows) Console.WriteLine(FormattableString.Invariant($"    model contact {reach[i].Id} at {s.T - t0:0.00} s by {link.ToString().ToLowerInvariant()}{(acc ? ", in an accident" : "")}; tool z {arm.Tool.Z * 100:0.0} cm"));
                        }

                        if (!now && inside[i]) modelEvents.Stop(reach[i].Id, s.T);
                        inside[i] = now;
                    }
                }

                // The same follower against the objects where physics had them at each moment (WO): what the model would say had it known
                // which objects the take moved; and where the measured arm's tool point is against the follower's, at each frame.
                var movingEvents = new MixedDryRun.ContactEvents();
                var movingInside = new Dictionary<int, bool>();
                int fi = 0;
                foreach (Sample s in follow)
                {
                    List<SafetyObject> now = ObjectsAt(s.T);
                    arm.At(s.Pose, s.Gripper);
                    foreach (SafetyObject o in now)
                    {
                        if (o.Scenery || new System.Numerics.Vector2(o.Center.X, o.Center.Y).Length() - MathF.Max(o.HalfExtents.X, o.HalfExtents.Y) >= 0.45f) continue;
                        bool was = movingInside.TryGetValue(o.Id, out bool w) && w;
                        bool touch = arm.Touching(o, out ArmLink link, out _);
                        if (!touch && was) movingEvents.Stop(o.Id, s.T);
                        if (touch && !was && movingEvents.Begin(o.Id, s.T))
                            byMovingEvents.Add(moves.Any(m => IsAccident(m.Kind) && s.T >= m.Start && s.T <= m.End + lag), arm.Tool.Z, link == ArmLink.Gripper ? "gripper" : link.ToString().ToLowerInvariant());
                        movingInside[o.Id] = touch;
                    }
                }

                foreach (Sample m in meas)
                {
                    while (fi + 1 < follow.Count && follow[fi + 1].T <= m.T) fi++;
                    if (follow[fi].T > m.T || m.T < t0) continue;
                    float zf = arm.At(follow[fi].Pose, follow[fi].Gripper).Tool.Z;
                    float zm = arm.At(m.Pose, follow[fi].Gripper).Tool.Z;
                    toolBias.Add((zm - zf) * 1000.0);
                    for (int j = 0; j < 5; j++) jointBias[j].Add(DrivePlan.Joint(m.Pose, j) - DrivePlan.Joint(follow[fi].Pose, j));
                }

                var physicsOnsets = new List<(double T, int Object, bool Accident)>();
                double lastEnd = double.NegativeInfinity;
                int lastObject = -1;
                foreach ((double t, int o, int[] links) in physics)
                {
                    if (o == 0) continue;
                    bool acc = moves.Any(m => IsAccident(m.Kind) && t >= m.Start && t <= m.End + lag);
                    physicsOnsets.Add((t, o, acc));
                    if (o == lastObject && t - lastEnd < MergeSeconds) physicsMerged++;
                    lastObject = o;
                    lastEnd = t;
                    Sample near = meas.Count == 0 ? cmd[0] : meas.OrderBy(x => Math.Abs(x.T - t)).First();
                    arm.At(near.Pose, near.Gripper);
                    phys.Add(acc, arm.Tool.Z, links.Length > 0 && links[0] >= 0 && links[0] < LinkNames.Length ? LinkNames[links[0]] : "link");
                }

                // Physics as contact events: a re-touch of an object within MergeSeconds of its last stretch's end is the same contact.
                var physicsEvents = new MixedDryRun.ContactEvents();
                var transitions = physics.Where(c => c.Object != 0).Select(c => (c.T, c.Object, On: true, c.Links)).Concat(ends.Where(e => e.Object != 0).Select(e => (e.T, e.Object, On: false, Links: Array.Empty<int>())))
                    .OrderBy(x => x.T).ThenBy(x => x.On ? 1 : 0).ToList();
                foreach ((double t, int o, bool on, int[] links) in transitions)
                {
                    if (!on)
                    {
                        physicsEvents.Stop(o, t);
                        continue;
                    }

                    if (!physicsEvents.Begin(o, t)) continue;
                    bool acc = moves.Any(m => IsAccident(m.Kind) && t >= m.Start && t <= m.End + lag);
                    Sample near = meas.Count == 0 ? cmd[0] : meas.OrderBy(x => Math.Abs(x.T - t)).First();
                    arm.At(near.Pose, near.Gripper);
                    physEvents.Add(acc, arm.Tool.Z, links.Length > 0 && links[0] >= 0 && links[0] < LinkNames.Length ? LinkNames[links[0]] : "link");
                }

                // Each object physics touched: how far it moved and tipped after its first touch, and how many contact events the model
                // and physics give it after that one.
                foreach (int o in physicsOnsets.Select(x => x.Object).Distinct())
                {
                    double first = physicsOnsets.First(x => x.Object == o).T;
                    int k0 = poses.FindIndex(x => x.T >= first);
                    if (k0 < 0 || !poses[0].At.TryGetValue((byte)o, out double[]? p0)) continue;
                    double moved = 0, tilt = 0;
                    for (int k = k0; k < poses.Count; k++)
                    {
                        if (!poses[k].At.TryGetValue((byte)o, out double[]? v) || v.Length < 7) continue;
                        moved = Math.Max(moved, Math.Sqrt((v[0] - p0[0]) * (v[0] - p0[0]) + (v[1] - p0[1]) * (v[1] - p0[1])));
                        double dot = Math.Abs(v[3] * p0[3] + v[4] * p0[4] + v[5] * p0[5] + v[6] * p0[6]);
                        tilt = Math.Max(tilt, 2 * Math.Acos(Math.Min(1, dot)) * 180 / Math.PI);
                    }

                    int si = scene.IndexOf((byte)o);
                    string name = si >= 0 ? scene.Objects[si].Name : o.ToString(CultureInfo.InvariantCulture);
                    bool fixedOne = si >= 0 && !scene.Objects[si].Movable;
                    string state = fixedOne ? "fixed" : tilt > 30 ? "tipped" : moved > 0.03 ? "moved" : "stayed";
                    int physicsLater = 0, modelLater = 0;
                    var pe = new MixedDryRun.ContactEvents();
                    foreach ((double t, int ob, bool on, int[] _) in transitions)
                    {
                        if (ob != o) continue;
                        if (!on) pe.Stop(ob, t);
                        else if (pe.Begin(ob, t) && t > first + MixedDryRun.ContactEvents.MergeSeconds) physicsLater++;
                    }

                    modelLater = modelEventTimes.Count(x => x.Object == o && x.T > first + MixedDryRun.ContactEvents.MergeSeconds);
                    if (!objectFates.TryGetValue(state, out int[]? f)) objectFates[state] = f = new int[3];
                    f[0]++;
                    f[1] += physicsLater;
                    f[2] += modelLater;
                    if (rows) Console.WriteLine(FormattableString.Invariant($"    object {o} {name}: {state} (moved {moved * 100:0.0} cm, tilted {tilt:0} deg after its first touch); later contact events: physics {physicsLater}, the model {modelLater}"));
                }

                foreach ((double t, int o, bool acc) in modelOnsets)
                    match[acc ? 1 : 0, physicsOnsets.Any(x => x.Object == o && Math.Abs(x.T - t) <= MatchSeconds) ? 0 : 1]++;
                foreach ((double t, int o, bool acc) in physicsOnsets)
                    if (!modelOnsets.Any(x => x.Object == o && Math.Abs(x.T - t) <= MatchSeconds)) match[acc ? 1 : 0, 2]++;

                foreach ((double t, int o, int[] links) in physics)
                {
                    if (o == 0) continue;
                    onsets++;
                    Move? under = moves.Where(m => !IsAccident(m.Kind) && m.Start <= t).OrderBy(m => m.Start).LastOrDefault();
                    Move? acc = moves.FirstOrDefault(m => IsAccident(m.Kind) && t >= m.Start && t <= m.End + lag);
                    if (acc != null) onsetsInAccident++;
                    Move? acc0 = moves.FirstOrDefault(m => IsAccident(m.Kind) && t >= m.Start && t <= m.End);
                    if (group.Contains(" source", StringComparison.Ordinal))
                    {
                        // The moves that made a contact: an accident whose span holds its onset (as recorded, and with the lag after), else the move under way.
                        causes[0].Add(acc0 ?? under);
                        causes[1].Add(acc ?? under);
                    }
                    if (group.Contains(" source", StringComparison.Ordinal))
                    {
                        sourceOnsets[0]++;
                        if (acc != null) sourceOnsets[1]++;
                    }
                    List<SafetyObject> objects = ObjectsAt(t);
                    // The command's closest to the object over the lag before the contact.
                    float g = float.PositiveInfinity;
                    foreach (Sample s in cmd)
                    {
                        if (s.T < t - lag - 0.1 || s.T > t) continue;
                        arm.At(s.Pose, s.Gripper);
                        foreach (SafetyObject x in objects) if (x.Id == o) g = MathF.Min(g, arm.Gap(x, 0.10f));
                    }

                    if (g <= 0f) onsetsCmdTouch++;
                    Sample at = meas.Count == 0 ? cmd[0] : meas.OrderBy(s => Math.Abs(s.T - t)).First();
                    arm.At(at.Pose, at.Gripper);
                    onsetTool.Add(arm.Tool.Z);
                    foreach (int l in links.Distinct()) Count(onsetLink, l >= 0 && l < LinkNames.Length ? LinkNames[l] : "link" + l);
                    if (rows)
                        Console.WriteLine(FormattableString.Invariant(
                            $"    contact {o} at {t - t0:0.00} s by {string.Join(",", links.Select(l => l < LinkNames.Length ? LinkNames[l] : l.ToString(CultureInfo.InvariantCulture)))}: under {under?.Kind ?? "none"}{(acc != null ? ", in a " + acc.Kind : "")}; the command's closest over the lag before {Mm(g)}; tool z {arm.Tool.Z * 100:0.0} cm"));
                }
            }

            Console.WriteLine(FormattableString.Invariant($"[joint-drive] take audit: {takes.Count} takes, lag (command to measured) {MixedDryRun.Percentiles(lags, "0.00")} s"));
            Console.WriteLine(FormattableString.Invariant(
                $"  accidents {accidents}: the command within 2 cm of an object by the model {accidentsNearCmd}, into one {accidentsTouchCmd}, into one for the lag ({lag:0.00} s) or longer {accidentsTouchCmdLate}; the measured arm into one {accidentsTouchMeas}; a physics contact begun in the span (+{lag:0.00} s) {accidentsPhysics}; clamped by RobotHost {accidentsClamped}"));
            foreach (KeyValuePair<string, int[]> kv in kindRows.OrderBy(k => k.Key, StringComparer.Ordinal))
                Console.WriteLine(FormattableString.Invariant($"    {kv.Key}: {kv.Value[0]}, within 2 cm {kv.Value[1]}, commanded in {kv.Value[2]}, for the lag {kv.Value[3]}, measured in {kv.Value[4]}, physics {kv.Value[5]}"));
            Console.WriteLine("  accidents' closest, commanded mm: " + MixedDryRun.Percentiles(cmdGap.Select(x => x * 1000).ToList(), "0") + "; measured mm: " +
                              MixedDryRun.Percentiles(measGap.Select(x => x * 1000).ToList(), "0"));
            Console.WriteLine(FormattableString.Invariant(
                $"  physics contacts with objects {onsets}: begun in an accident's span {onsetsInAccident} ({100.0 * onsetsInAccident / Math.Max(1, onsets):0}%); the command into the object within the lag before {onsetsCmdTouch}"));
            Console.WriteLine(FormattableString.Invariant($"    of them in source takes (not replays) {sourceOnsets[0]}: in an accident's span {sourceOnsets[1]}"));
            for (int k = 0; k < 2; k++)
                Console.WriteLine(FormattableString.Invariant(
                    $"    moves of the source takes that made a contact, an accident's span {(k == 0 ? "as recorded" : $"and {lag:0.00} s after")}: {causes[k].Count}, accidents {causes[k].Count(m => m != null && IsAccident(m.Kind))}"));
            string[] predictors = { "the command in an object", "the command in one for the lag", $"the follower (delay {FollowDelay:0.00} s, tau {FollowTau:0.00} s, {FollowSpeed:0.00} rad/s) in one", "the measured arm in one" };
            Console.WriteLine("  follower against the measured joints, rms largest-joint error rad: " + MixedDryRun.Percentiles(followErr, "0.000"));
            for (int k = 0; k < 4; k++)
                Console.WriteLine(FormattableString.Invariant($"  accidents, {predictors[k]} against a physics contact: both {agree[k, 0]}, only the model {agree[k, 1]}, only physics {agree[k, 2]}, neither {agree[k, 3]}"));
            Console.WriteLine("  their tool point (measured), cm from the base frame: 5 cm bins from -20: " + MixedDryRun.Histogram(onsetTool.Select(z => z * 100.0), -20, 5, 8) +
                              "; p10/p50/p90 " + MixedDryRun.Percentiles(onsetTool.Select(z => z * 100.0).ToList(), "0.0"));
            Console.WriteLine("  touching links: " + string.Join(", ", onsetLink.OrderByDescending(k => k.Value).Select(k => k.Key + " " + k.Value)));
            Console.WriteLine(FormattableString.Invariant($"  the dry run's contacts on the recorded commands (the follower, the objects as each take began): {byModel.Describe()}"));
            Console.WriteLine(FormattableString.Invariant($"  physics, counted the same way: {phys.Describe()}; re-touches of the same object within {MergeSeconds:0.0} s of its last onset {physicsMerged}"));
            Console.WriteLine(FormattableString.Invariant($"  as contact events (a re-touch within {MixedDryRun.ContactEvents.MergeSeconds:0.0} s of the object's last release is the same contact): the model {byModelEvents.Describe()}"));
            Console.WriteLine(FormattableString.Invariant($"    physics {physEvents.Describe()}"));
            Console.WriteLine(FormattableString.Invariant($"    the model with the objects where physics had them at each moment: {byMovingEvents.Describe()}"));
            Console.WriteLine("  objects physics touched, by what became of them after the first touch (objects: later contact events by physics / by the model): " +
                              string.Join(", ", objectFates.OrderBy(k => k.Key, StringComparer.Ordinal).Select(k => FormattableString.Invariant($"{k.Key} {k.Value[0]}: {k.Value[1]} / {k.Value[2]}"))));
            Console.WriteLine("  the measured tool point's height less the follower's, mm: " + MixedDryRun.Percentiles(toolBias, "0.0") + FormattableString.Invariant($"; mean {(toolBias.Count == 0 ? 0 : toolBias.Average()):0.0}") +
                              "; joints' mean measured less followed, rad: " + string.Join(", ", jointBias.Select(b => (b.Count == 0 ? 0 : b.Average()).ToString("0.000", CultureInfo.InvariantCulture))));
            Console.WriteLine(FormattableString.Invariant(
                $"  onsets matched (the same object within {MatchSeconds:0.0} s): outside accidents both {match[0, 0]}, only the model {match[0, 1]}, only physics {match[0, 2]}; in an accident's span both {match[1, 0]}, only the model {match[1, 1]}, only physics {match[1, 2]}"));
            return 0;
        }

        /// <summary>How near in time a model onset and a physics onset of the same object count as one contact, s.</summary>
        internal const double MatchSeconds = 0.4;

        /// <summary>A physics onset of the object last touched this soon after that one's onset counts as a re-touch, s.</summary>
        internal const double MergeSeconds = 0.5;

        /// <summary>Contact onsets: how many, how many in an accident's span (and the lag after), their tool point's height, the touching link.</summary>
        private sealed class ContactTally
        {
            private readonly List<double> _tool = new List<double>();
            private readonly Dictionary<string, int> _links = new Dictionary<string, int>();
            private int _accidents;

            public void Add(bool accident, double toolZ, string link)
            {
                if (accident) _accidents++;
                _tool.Add(toolZ);
                Count(_links, link);
            }

            public string Describe()
            {
                int n = _tool.Count;
                var bins = new int[64];
                foreach (double z in _tool) bins[Math.Clamp((int)Math.Floor((z * 100.0 + 20.0) / 5.0), 0, 63)]++;
                return FormattableString.Invariant($"{n} onsets, in an accident's span {_accidents} ({100.0 * _accidents / Math.Max(1, n):0}%); tool point 5 cm bins from -20: ") +
                       MixedDryRun.Histogram(_tool.Select(z => z * 100.0), -20, 5, 8) + FormattableString.Invariant($" (largest band {(n == 0 ? 0 : 100.0 * bins.Max() / n):0}%); links ") +
                       string.Join(", ", _links.OrderByDescending(k => k.Value).ThenBy(k => k.Key, StringComparer.Ordinal).Select(k => k.Key + " " + k.Value));
            }
        }

        /// <summary>The follower: the command <see cref="FollowDelay"/> late, each joint at most <see cref="FollowSpeed"/>, and lagging it with time constant <see cref="FollowTau"/>.</summary>
        internal static double FollowDelay = 0.10, FollowTau = 0.08, FollowSpeed = 3.7;

        private static void Tally(int[,] t, int k, bool model, bool physics) => t[k, model ? (physics ? 0 : 1) : (physics ? 2 : 3)]++;

        private static List<Sample> Follow(List<Sample> cmd, double delay, double tau, double speed)
        {
            var output = new List<Sample>(cmd.Count);
            var q = new float[5];
            var target = new float[5];
            int k = 0;
            double last = cmd[0].T;
            Fill(cmd[0].Pose, q);
            foreach (Sample s in cmd)
            {
                double dt = s.T - last;
                last = s.T;
                while (k + 1 < cmd.Count && cmd[k + 1].T <= s.T - delay) k++;
                Fill(cmd[k].Pose, target);
                double a = dt <= 0 ? 0 : 1 - Math.Exp(-dt / Math.Max(1e-3, tau));
                for (int j = 0; j < 5; j++)
                {
                    double step = (target[j] - q[j]) * a, max = speed * dt;
                    q[j] += (float)Math.Clamp(step, -max, max);
                }

                output.Add(new Sample(s.T, new ArmPose(q[0], q[1], q[2], q[3], q[4]), cmd[k].Gripper, false));
            }

            return output;
        }

        private static void Fill(in ArmPose p, float[] q)
        {
            q[0] = p.BaseYaw;
            q[1] = p.Proximal;
            q[2] = p.Distal;
            q[3] = p.Wrist;
            q[4] = p.Roll;
        }

        private static double Rms(List<Sample> track, List<Sample> meas)
        {
            double err = 0;
            int n = 0, k = 0;
            foreach (Sample m in meas)
            {
                while (k + 1 < track.Count && track[k + 1].T <= m.T) k++;
                if (track[k].T > m.T) continue;
                double d = m.Pose.MaxJointDelta(track[k].Pose);
                err += d * d;
                n++;
            }

            return n == 0 ? double.NaN : Math.Sqrt(err / n);
        }

        private static bool IsAccident(string kind) => kind == "jerk" || kind == "overshoot" || kind == "wrong-way" || kind == "slip" || kind == "rush";

        private static double D(string s) => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);

        private static string Mm(float g) => float.IsPositiveInfinity(g) ? ">100 mm" : (g * 1000).ToString("0", CultureInfo.InvariantCulture) + " mm";

        private static void Count(Dictionary<string, int> counts, string key) => counts[key] = counts.TryGetValue(key, out int n) ? n + 1 : 1;

        /// <summary>A command's joints (<c>motor:angle:speed</c>, <paramref name="field"/> 1) or a frame's (<c>motor:flags:value</c>, 2): motors 1-5 the arm, 10 the gripper.</summary>
        private static bool TryJoints(string text, int field, out ArmPose pose, out float gripper)
        {
            var q = new float[5];
            int seen = 0;
            gripper = float.NaN;
            foreach (string e in text.Split(';', StringSplitOptions.RemoveEmptyEntries))
            {
                string[] v = e.Split(':');
                if (v.Length < 2) continue;
                int motor = int.Parse(v[0], CultureInfo.InvariantCulture);
                if (v.Length <= field) continue;
                float value = float.Parse(v[field], NumberStyles.Float, CultureInfo.InvariantCulture);
                if (motor >= 1 && motor <= 5)
                {
                    q[motor - 1] = value;
                    seen |= 1 << (motor - 1);
                }
                else if (motor == 10) gripper = value;
            }

            pose = new ArmPose(q[0], q[1], q[2], q[3], q[4]);
            return seen == 31;
        }

        /// <summary>The delay that best lines the measured joints up with the commanded ones (least mean squared difference, 10 ms steps).</summary>
        private static double Lag(List<Sample> cmd, List<Sample> meas)
        {
            if (meas.Count < 10) return double.NaN;
            double best = 0, bestErr = double.MaxValue;
            for (double d = 0; d <= 0.6 + 1e-9; d += 0.01)
            {
                double err = 0;
                int n = 0, k = 0;
                foreach (Sample m in meas)
                {
                    double t = m.T - d;
                    while (k + 1 < cmd.Count && cmd[k + 1].T <= t) k++;
                    if (cmd[k].T > t) continue;
                    err += m.Pose.MaxJointDelta(cmd[k].Pose) * (double)m.Pose.MaxJointDelta(cmd[k].Pose);
                    n++;
                }

                if (n > 0 && err / n < bestErr)
                {
                    bestErr = err / n;
                    best = d;
                }
            }

            return best;
        }
    }
}
