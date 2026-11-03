using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.Json;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.IkStress
{
    /// <summary>
    /// <c>chain-ik-stress</c> (<c>just fr3-ik-stress</c>): long synthetic teleoperation sessions of the FR3 Duo through the headset's
    /// mapping and IK (<see cref="IkStressRunner"/>), headless. Prints, per seed and in total, how often each arm was off its
    /// target, how much of that the target was reachable, the episodes where a reachable target stayed unreached (stuck) or
    /// the gripper pointed the wrong way, and the worst of them with the joints. Exits 1 when any episode happened.
    /// </summary>
    public static class IkStressCommand
    {
        private const string Usage = "usage: chain-ik-stress [--profile RobotProfiles/fr3-duo.json] [--seconds 300] [--seeds 1-5] [--show 8] [--episodes-json <path>]";

        public static int Run(string[] args)
        {
            string profilePath = "RobotProfiles/fr3-duo.json";
            string? json = null;
            double seconds = 300;
            int show = 8;
            (int, double, double)? trace = null;
            var teleopOptions = new Teleop.RobotArm.Operator.SerialChainTeleopOptions();
            var seeds = new List<ulong> { 1, 2, 3, 4, 5 };
            try
            {
                for (int i = 1; i < args.Length; i++)
                {
                    string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                    switch (args[i])
                    {
                        case "--profile": profilePath = Next(); break;
                        case "--seconds": seconds = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--show": show = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--episodes-json": json = Next(); break;
                        case "--set":
                            // A teleop setting to try: Name=Value (RecoverOnlyWhenStill, RecoverAfterSeconds, RecoverSpeedShare, RecoverStarts, PostureSpeed, RecoverImprovement).
                            string[] kv = Next().Split('=');
                            switch (kv[0])
                            {
                                case "RecoverOnlyWhenStill": teleopOptions.RecoverOnlyWhenStill = bool.Parse(kv[1]); break;
                                case "RecoverAfterSeconds": teleopOptions.RecoverAfterSeconds = double.Parse(kv[1], CultureInfo.InvariantCulture); break;
                                case "RecoverSpeedShare": teleopOptions.RecoverSpeedShare = double.Parse(kv[1], CultureInfo.InvariantCulture); break;
                                case "RecoverStarts": teleopOptions.RecoverStarts = int.Parse(kv[1], CultureInfo.InvariantCulture); break;
                                case "PostureSpeed": teleopOptions.PostureSpeed = double.Parse(kv[1], CultureInfo.InvariantCulture); break;
                                case "RecoverImprovement": teleopOptions.RecoverImprovement = double.Parse(kv[1], CultureInfo.InvariantCulture); break;
                                default: throw new ArgumentException($"--set: unknown setting '{kv[0]}'");
                            }

                            break;
                        case "--trace":
                            string[] tr = Next().Split(':');
                            trace = (int.Parse(tr[0], CultureInfo.InvariantCulture), double.Parse(tr[1], CultureInfo.InvariantCulture), double.Parse(tr[2], CultureInfo.InvariantCulture));
                            break;
                        case "--seeds":
                            string[] r = Next().Split('-');
                            ulong a = ulong.Parse(r[0], CultureInfo.InvariantCulture), b = r.Length > 1 ? ulong.Parse(r[1], CultureInfo.InvariantCulture) : a;
                            seeds = new List<ulong>();
                            for (ulong s = a; s <= b; s++) seeds.Add(s);
                            break;
                        default: throw new ArgumentException($"unknown argument '{args[i]}'");
                    }
                }
            }
            catch (Exception e) when (e is ArgumentException || e is FormatException || e is OverflowException)
            {
                Console.Error.WriteLine($"chain-ik-stress: {e.Message}");
                Console.Error.WriteLine(Usage);
                return 64;
            }

            SerialChainProfile profile = ImportUrdf.SerialChainProfileJson.Load(profilePath);
            string collisionPath = Path.ChangeExtension(profilePath, null) + ".collision.json";
            SerialChainCollisionModel? collision = File.Exists(collisionPath)
                ? JsonSerializer.Deserialize<SerialChainCollisionDocument>(File.ReadAllText(collisionPath), new JsonSerializerOptions { IncludeFields = true })?.ToModel()
                : null;
            Console.WriteLine($"[ik-stress] {profile.Name}: {seconds:0} s per seed at 90 Hz, seeds {string.Join(",", seeds)}; collision model {(collision != null ? collisionPath : "none")}");
            var all = new List<IkStressResult>();
            foreach (ulong seed in seeds)
            {
                var started = DateTime.UtcNow;
                IkStressResult r = IkStressRunner.Run(profile, collision, seed, new IkStressOptions { Seconds = seconds, Trace = trace, Teleop = teleopOptions });
                all.Add(r);
                Console.WriteLine($"[ik-stress] seed {seed}: {Summary(r)}; {(DateTime.UtcNow - started).TotalSeconds:0} s");
            }

            var total = new IkStressResult();
            foreach (IkStressResult r in all)
            {
                total.Frames += r.Frames;
                total.EngagedFrames += r.EngagedFrames;
                total.OffFrames += r.OffFrames;
                total.OffReachableFrames += r.OffReachableFrames;
                total.OracleCalls += r.OracleCalls;
                total.SwingFrames += r.SwingFrames;
                total.MaxSwing = Math.Max(total.MaxSwing, r.MaxSwing);
                total.Episodes.AddRange(r.Episodes);
                total.PositionMm.AddRange(r.PositionMm);
                total.AngleDeg.AddRange(r.AngleDeg);
                foreach (var kv in r.StatusFrames) total.StatusFrames[kv.Key] = (total.StatusFrames.TryGetValue(kv.Key, out int c) ? c : 0) + kv.Value;
            }

            Console.WriteLine($"[ik-stress] total: {Summary(total)}");
            Console.WriteLine($"[ik-stress] status frames: {string.Join(", ", total.StatusFrames.OrderBy(k => k.Key).Select(k => $"{k.Key} {k.Value}"))}");
            foreach (IkStressResult r in all)
            {
                total.OffRuns.AddRange(r.OffRuns);
                total.SettleSeconds.AddRange(r.SettleSeconds);
                total.SettleMissed += r.SettleMissed;
                total.Searches += r.Searches;
                total.SearchesNoBetter += r.SearchesNoBetter;
                total.Transits += r.Transits;
                total.TransitsBlocked += r.TransitsBlocked;
                total.StepMs.AddRange(r.StepMs);
                total.RecoveringFrames += r.RecoveringFrames;
                total.OffReachableMoving += r.OffReachableMoving;
                foreach (var kv in r.OffReachableStatus) total.OffReachableStatus[kv.Key] = (total.OffReachableStatus.TryGetValue(kv.Key, out int c) ? c : 0) + kv.Value;
            }

            Console.WriteLine($"[ik-stress] off a reachable target: {string.Join(", ", total.OffReachableStatus.OrderByDescending(k => k.Value).Select(k => $"{k.Key} {k.Value}"))}; " +
                              $"target moving in {total.OffReachableMoving}; recovering frames {total.RecoveringFrames}");
            Console.WriteLine($"[ik-stress] settling on a reachable target after the hand stopped: {total.SettleSeconds.Count} holds, p50 {Percentile(total.SettleSeconds, 50):0.00} s, " +
                              $"p95 {Percentile(total.SettleSeconds, 95):0.00} s, p99 {Percentile(total.SettleSeconds, 99):0.00} s, max {(total.SettleSeconds.Count > 0 ? total.SettleSeconds.Max() : 0):0.00} s; " +
                              $"never reached before the hand moved on: {total.SettleMissed}");
            Console.WriteLine($"[ik-stress] recovery: {total.Searches} searches, {total.SearchesNoBetter} found nothing better, {total.Transits} moves, {total.TransitsBlocked} cut by a collision");
            Console.WriteLine($"[ik-stress] one step of both arms on this machine: p50 {Percentile(total.StepMs, 50):0.000} ms, p99 {Percentile(total.StepMs, 99):0.000} ms, max {(total.StepMs.Count > 0 ? total.StepMs.Max() : 0):0.000} ms");
            Console.WriteLine($"[ik-stress] off runs (ended): {total.OffRuns.Count}, p50 {Percentile(total.OffRuns, 50):0.00} s, p90 {Percentile(total.OffRuns, 90):0.00} s, p99 {Percentile(total.OffRuns, 99):0.00} s, max {(total.OffRuns.Count > 0 ? total.OffRuns.Max() : 0):0.00} s");
            var slow = total.Episodes.Where(e => e.Kind == "slow").Select(e => e.Seconds).ToList();
            Console.WriteLine($"[ik-stress] slow (a recovery got there): {slow.Count}, the hand stopped to the arm there p50 {Percentile(slow, 50):0.00} s, max {(slow.Count > 0 ? slow.Max() : 0):0.00} s");
            foreach (var group in total.Failures.GroupBy(e => e.Grip.Split('+')[0]).OrderByDescending(g => g.Sum(e => e.Seconds)))
            {
                Console.WriteLine($"[ik-stress]   grip {group.Key,-20} {group.Count(),3} episode(s), {group.Sum(e => e.Seconds),6:0.0} s");
            }

            foreach (IkEpisode e in total.Failures.OrderByDescending(e => e.Seconds).Take(show))
            {
                Console.WriteLine($"[ik-stress] {e.Kind} seed {e.Seed} arm {e.Arm} {e.Start:0.0}-{e.End:0.0} s ({e.Seconds:0.0} s), worst {e.WorstPositionMm:0} mm / {e.WorstAngleDeg:0}°, " +
                                  $"grip {e.Grip}, status {e.Status}{(e.Recovering ? ", a recovery under way" : "")}{(e.Arrived ? ", got there" : ", the hand moved on first")}");
                Console.WriteLine($"             target {Pose(e.Target)}  tool {Pose(e.Tool)}");
                Console.WriteLine($"             joints {Joints(e.Joints)}");
                Console.WriteLine($"             a reach {Joints(e.Reached)}");
            }

            if (json != null)
            {
                File.WriteAllText(json, JsonSerializer.Serialize(total.Episodes.Select(e => new
                {
                    e.Kind, e.Arm, e.Start, e.End, e.WorstPositionMm, e.WorstAngleDeg, e.Grip, e.Status,
                    Target = new[] { e.Target.X, e.Target.Y, e.Target.Z, e.Target.Qw, e.Target.Qx, e.Target.Qy, e.Target.Qz },
                    Tool = new[] { e.Tool.X, e.Tool.Y, e.Tool.Z, e.Tool.Qw, e.Tool.Qx, e.Tool.Qy, e.Tool.Qz },
                    e.Joints, e.Reached,
                }), new JsonSerializerOptions { WriteIndented = true }));
            }

            bool clean = !total.Failures.Any();
            Console.WriteLine(clean ? "[ik-stress] PASS: no stuck or wrong-way episode" : $"[ik-stress] FAIL: {total.Stuck} stuck, {total.WrongWay} wrong-way episode(s)");
            return clean ? 0 : 1;
        }

        private static string Summary(IkStressResult r)
        {
            double p95 = Percentile(r.PositionMm, 95), a95 = Percentile(r.AngleDeg, 95);
            return $"{r.Stuck} stuck, {r.WrongWay} wrong-way ({r.Failures.Sum(e => e.Seconds):0.0} s), {r.Slow} slow; off target {Share(r.OffFrames, r.EngagedFrames)} of engaged frames, " +
                   $"{Share(r.OffReachableFrames, r.EngagedFrames)} with the target reachable; error p95 {p95:0.0} mm / {a95:0.0}°; swings >0.08 rad {r.SwingFrames} (max {r.MaxSwing:0.000}); oracle {r.OracleCalls}";
        }

        private static string Share(int part, int whole) => whole == 0 ? "-" : (100.0 * part / whole).ToString("0.0", CultureInfo.InvariantCulture) + "%";

        private static double Percentile(List<double> v, double p)
        {
            if (v.Count == 0) return double.NaN;
            var s = v.OrderBy(x => x).ToList();
            return s[(int)Math.Min(s.Count - 1, Math.Ceiling(p / 100.0 * s.Count) - 1)];
        }

        private static string Pose(in RigidTransform t)
        {
            t.Rotate(0, 0, 1, out double ax, out double ay, out double az);
            return FormattableString.Invariant($"({t.X:0.000}, {t.Y:0.000}, {t.Z:0.000}) approach ({ax:0.00}, {ay:0.00}, {az:0.00})");
        }

        private static string Joints(double[] q) => q.Length == 0 ? "-" : string.Join(" ", q.Select(v => v.ToString("0.00", CultureInfo.InvariantCulture)));
    }
}
