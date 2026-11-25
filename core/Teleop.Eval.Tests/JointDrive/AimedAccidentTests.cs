using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Numerics;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Xunit;
using Xunit.Abstractions;
using static Teleop.Eval.Tests.JointDrive.Takes;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>
    /// docs/adr/0038 "M, motion", the 2026-11-24 review of the pilot: accidents placed near an object are aimed at it and go in far and long
    /// enough for the twin's arm, which trails the command by about 0.2 s, to get there (the dry run's follower of the command); others stay
    /// in free space. Link aims meet high objects with the wrist or the elbow's links; blind goals spread over the height bands; the take
    /// audit reads a recorded take's accidents and contacts.
    /// </summary>
    public class AimedAccidentTests
    {
        private readonly ITestOutputHelper _output;

        public AimedAccidentTests(ITestOutputHelper output) => _output = output;

        /// <summary>Of the plan's accidents of the kinds that are aimed and placed near objects: how many, and how many the follower of the command touches an object in (span and 0.2 s after); and how many accidents keep 10 cm from every object.</summary>
        private static (int Aimed, int Hit, int Free, int All) Tally(DrivePlan plan, SafetyObject[] objects)
        {
            var arm = new ArmTouch(Profile, Model);
            List<TrackPoint> follower = MixedDryRun.Followed(plan);
            SafetyObject[] inReach = objects.Where(o => !o.Scenery).ToArray();
            ArmCollisionModel within10 = SafetyScript.WithObjects(Model, inReach, 0.10f);
            int aimed = 0, hit = 0, free = 0, all = 0;
            foreach (PlannedMove m in plan.Moves.Where(x => ScriptMoveCodec.IsAccident(x.Kind)))
            {
                all++;
                bool clear = true;
                for (double t = Math.Max(m.Start, plan.ScriptStart); t <= Math.Min(m.End, plan.TotalSeconds) && clear; t += Tick) clear = ArmCollision.IsValid(Profile, within10, plan.Sample(t));
                if (clear) free++;
                if (m.Part != MovePart.NearAccident || m.Kind == ScriptMoveKind.WrongWay) continue;
                aimed++;
                bool touched = false;
                foreach (TrackPoint p in follower)
                {
                    double t = plan.ScriptStart + p.At;
                    if (t < m.Start || t > m.End + MixedDryRun.LagSeconds) continue;
                    arm.At(p.Pose, p.Gripper);
                    if (inReach.Any(o => arm.Touches(o)))
                    {
                        touched = true;
                        break;
                    }
                }

                if (touched) hit++;
            }

            return (aimed, hit, free, all);
        }

        [Fact]
        public void AccidentsPlacedNearObjects_AreAimedIn_FarAndLongEnoughForTheLaggingArm_AndOthersStayInFreeSpace()
        {
            int aimed = 0, hit = 0, free = 0, all = 0, aimedOff = 0, hitOff = 0;
            for (ulong layout = 1; layout <= 24; layout++)
            {
                SafetyObject[] objects = MixedTakeTests.MixedLayout(layout);
                if (MixedTakeTests.TryMixed(objects, 7000 + layout, out DrivePlan? plan, out _))
                {
                    (int a, int h, int f, int n) = Tally(plan!, objects);
                    (aimed, hit, free, all) = (aimed + a, hit + h, free + f, all + n);
                }

                if (MixedTakeTests.TryMixed(objects, 7000 + layout, out DrivePlan? before, out _, new MixedOptions { AimAccidents = false }))
                {
                    (int a, int h, _, _) = Tally(before!, objects);
                    (aimedOff, hitOff) = (aimedOff + a, hitOff + h);
                }
            }

            // Drawn as before (1-6 cm off the surface, the accident's own way) most of these reached an object too: aiming makes it the
            // rule, and holds the hand there for the reaction (docs/adr/0038 "M, motion": the share comes mostly from placing more).
            string report = $"aimed {hit} of {aimed} in an object by the follower; drawn as before {hitOff} of {aimedOff}; accidents 10 cm from every object {free} of {all}";
            _output.WriteLine(report);
            Assert.True(aimed >= 10, report);
            Assert.True(hit >= 0.6 * aimed, report); // most aimed accidents reach their object on the arm that trails the command
            Assert.True(free >= 0.2 * all, report); // accidents in free space stay: accident motion alone is no contact
        }

        [Fact]
        public void TheOptions_TurnTheAimingAndTheLinkAimsOff_AndAreChecked()
        {
            var o = new MixedOptions();
            Assert.True(o.AimAccidents);
            Assert.True(o.SharedHeights);
            Assert.Equal(0.2, o.LinkAimShare);
            Assert.Equal(0.8, o.AccidentNearShare);
            Assert.True(o.SpendTouched);
            o.LinkAimShare = 1.5;
            Assert.Throws<ArgumentException>(() => o.Validate());
            foreach (ScriptMoveKind k in new[] { ScriptMoveKind.Jerk, ScriptMoveKind.Overshoot, ScriptMoveKind.Slip, ScriptMoveKind.Rush })
            {
                (double min, double max) = MixedTake.AimedReach(k);
                Assert.True(min > MixedTake.AimedDepthMax / 2 && max <= 0.12 + 1e-9 && min < max, ScriptMoveCodec.Name(k)); // a kind's own size, within about 10 cm
            }
        }

        [Fact]
        public void LinkAims_MeetHighObjectsWithTheWristOrTheElbowsLinks()
        {
            var options = new MixedOptions { LinkAimShare = 1.0 };
            var arm = new ArmTouch(Profile, Model);
            var byLink = new Dictionary<ArmLink, int>();
            int aims = 0;
            for (ulong layout = 1; layout <= 30; layout++)
            {
                SafetyObject[] objects = MixedTakeTests.MixedLayout(layout);
                if (!MixedTakeTests.TryMixed(objects, 7100 + layout, out DrivePlan? plan, out _, options)) continue;
                foreach (PlannedMove m in plan!.Moves.Where(x => x.Kind == ScriptMoveKind.Aim && !double.IsNaN(x.Contact)))
                {
                    SafetyObject o = objects.Single(x => x.Id == m.ObjectId);
                    aims++;
                    // The first tick the plan's commands touch the aim's object: by which link.
                    for (double t = m.Start; t <= m.End; t += Tick)
                    {
                        arm.At(plan.Sample(t), plan.GripperAt(t));
                        if (!arm.Touching(o, out ArmLink link, out _)) continue;
                        byLink[link] = byLink.TryGetValue(link, out int n) ? n + 1 : 1;
                        break;
                    }
                }
            }

            string report = $"{aims} aims touching their object; first by: {string.Join(", ", byLink.Select(kv => $"{kv.Key} {kv.Value}"))}";
            _output.WriteLine(report);
            int others = byLink.Where(kv => kv.Key != ArmLink.Gripper).Sum(kv => kv.Value);
            Assert.True(others >= 5, report); // the wrist or the elbow's links, not only the pads
        }

        [Fact]
        public void BlindGoals_SpreadOverTheHeightBands()
        {
            float[] bands = MixedOptions.DefaultToolBands();
            Assert.Equal(15, bands.Length);
            var arm = new ArmTouch(Profile, Model);
            var counts = new int[bands.Length / 3];
            int n = 0;
            for (ulong layout = 1; layout <= 16; layout++)
            {
                SafetyObject[] objects = MixedTakeTests.MixedLayout(layout);
                if (!MixedTakeTests.TryMixed(objects, 7200 + layout, out DrivePlan? plan, out _)) continue;
                foreach (PlannedMove m in plan!.Moves.Where(x => x.Part == MovePart.BlindTransport && !ScriptMoveCodec.IsAccident(x.Kind)))
                {
                    double at = Math.Min(m.End, plan.TotalSeconds);
                    float z = arm.At(plan.Sample(at), plan.GripperAt(at)).Tool.Z;
                    for (int b = 0; b < counts.Length; b++)
                    {
                        if (z < bands[3 * b] - 0.02f || z >= bands[3 * b + 1] + 0.02f) continue; // the hand's own error about the goal
                        counts[b]++;
                        break;
                    }

                    n++;
                }
            }

            string report = $"{n} blind goals; by band from the table up: {string.Join(" ", counts)}";
            _output.WriteLine(report);
            Assert.True(n >= 50, report);
            Assert.True(counts.Count(c => c >= 0.08 * n) >= 4, report); // spread over the bands, not one
            Assert.True(counts.Max() <= 0.45 * n, report);
        }

        [Fact]
        public void TheTakeAudit_GivesAContactBegunInAnAccidentsSpanToTheAccident()
        {
            string dir = Path.Combine(Path.GetTempPath(), "take-audit-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
            try
            {
                const long t0 = 1_000_000_000_000L;
                string Ns(double s) => (t0 + (long)(s * 1e9)).ToString(CultureInfo.InvariantCulture);
                var lines = new List<string>
                {
                    $"RP|{Ns(0)}|1|source|none",
                    $"WO|{Ns(0)}|{Ns(0)}|1|31:0.25:0:-0.145:1:0:0:0",
                    $"MV|{Ns(0.5)}|1|3|free|0|NaN|NaN|0|NaN|NaN|2",
                    $"MV|{Ns(1.0)}|2|3|jerk|0|NaN|NaN|0|NaN|NaN|0.5",
                    $"MV|{Ns(3.0)}|3|3|free|0|NaN|NaN|0|NaN|NaN|1",
                };
                for (int k = 0; k <= 200; k++)
                {
                    double t = k * 0.02;
                    string joints = FormattableString.Invariant($"1:0:3.8;2:{0.5 + 0.1 * Math.Sin(t)}:3.8;3:-1:3.8;4:0.3:3.8;10:0:3.8;5:0:3.8");
                    lines.Add($"JC|{Ns(t)}|{Ns(t)}|{k}|0|A|{joints}");
                    string measured = FormattableString.Invariant($"1:1:0;2:1:{0.5 + 0.1 * Math.Sin(t - 0.18)};3:1:-1;4:1:0.3;10:1:0;5:1:0");
                    lines.Add($"FR|{Ns(t)}|{k}|{Ns(t)}|{Ns(t)}|0|0|{measured}");
                }

                lines.Add($"WC|{Ns(1.6)}|{Ns(1.6)}|0:5:31:2.5"); // within the jerk's span and 0.2 s after
                lines.Add($"WC|{Ns(1.8)}|{Ns(1.8)}|");
                lines.Add($"WC|{Ns(3.5)}|{Ns(3.5)}|0:7:31:1.0"); // under the free move
                lines.Add($"WC|{Ns(3.6)}|{Ns(3.6)}|");
                File.WriteAllLines(Path.Combine(dir, "robothost.jlog"), lines);
                var scene = MixedTakeTests.MixedScene.Value.Scene;
                TextWriter console = Console.Out;
                var captured = new StringWriter();
                Console.SetOut(captured);
                int code;
                try
                {
                    code = TakeAudit.Run(scene, Profile, new[] { dir }, 0.2, rows: true);
                }
                finally
                {
                    Console.SetOut(console);
                }

                string text = captured.ToString();
                _output.WriteLine(text);
                Assert.Equal(0, code);
                Assert.Contains("physics contacts with objects 2: begun in an accident's span 1 (50%)", text);
                Assert.Contains("accidents 1:", text);
                Assert.Contains("gripper 1", text);
                Assert.Contains("right_finger 1", text);
                Assert.Contains("an accident's span and 0.20 s after: 2, accidents 1", text); // the contact 0.1 s after the jerk's recorded end is its own
                Assert.Contains("an accident's span as recorded: 2, accidents 0", text);
                Assert.Matches(@"lag \(command to measured\) p50 0\.1[6-9]", text); // the measured joints 0.18 s behind, on a 10 ms grid
            }
            finally
            {
                foreach (string f in Directory.GetFiles(dir)) File.Delete(f);
                Directory.Delete(dir);
            }
        }
    }
}
