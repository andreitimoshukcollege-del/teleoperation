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

                foreach ((double t, int o, int[] links) in physics)
                {
                    if (o == 0) continue;
                    onsets++;
                    Move? under = moves.Where(m => !IsAccident(m.Kind) && m.Start <= t).OrderBy(m => m.Start).LastOrDefault();
                    Move? acc = moves.FirstOrDefault(m => IsAccident(m.Kind) && t >= m.Start && t <= m.End + lag);
                    if (acc != null) onsetsInAccident++;
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
            string[] predictors = { "the command in an object", "the command in one for the lag", $"the follower (delay {FollowDelay:0.00} s, tau {FollowTau:0.00} s, {FollowSpeed:0.00} rad/s) in one", "the measured arm in one" };
            Console.WriteLine("  follower against the measured joints, rms largest-joint error rad: " + MixedDryRun.Percentiles(followErr, "0.000"));
            for (int k = 0; k < 4; k++)
                Console.WriteLine(FormattableString.Invariant($"  accidents, {predictors[k]} against a physics contact: both {agree[k, 0]}, only the model {agree[k, 1]}, only physics {agree[k, 2]}, neither {agree[k, 3]}"));
            Console.WriteLine("  their tool point (measured), cm from the base frame: 5 cm bins from -20: " + MixedDryRun.Histogram(onsetTool.Select(z => z * 100.0), -20, 5, 8) +
                              "; p10/p50/p90 " + MixedDryRun.Percentiles(onsetTool.Select(z => z * 100.0).ToList(), "0.0"));
            Console.WriteLine("  touching links: " + string.Join(", ", onsetLink.OrderByDescending(k => k.Value).Select(k => k.Key + " " + k.Value)));
            return 0;
        }

        /// <summary>The follower: the command <see cref="FollowDelay"/> late, each joint at most <see cref="FollowSpeed"/>, and lagging it with time constant <see cref="FollowTau"/>.</summary>
        internal static double FollowDelay = 0.0, FollowTau = 0.15, FollowSpeed = 3.7;

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
