using System;
using System.Collections.Generic;
using System.Linq;
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
    /// The mixed script's accidents (docs/adr/0038 "M, motion", world-models' request 3): jerks, overshoots, wrong way starts, slips and
    /// rushes, each its own <c>MV</c> record over the move it came in; drawn without regard to the layout, so they come in free space too;
    /// their ticks within the arm's own limits (the servos' top speed, the joint limits) and every other tick within people's caps.
    /// </summary>
    public class AccidentTests
    {
        private readonly ITestOutputHelper _output;

        public AccidentTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void Accidents_AreTaggedOverTheirMove_KeepToTheArmsLimits_AndComeInFreeSpaceToo()
        {
            var options = new MixedOptions();
            MotionCaps people = options.Caps!, arm = AccidentModel.CapsFor(people, 3.8f);
            var kinds = new Dictionary<ScriptMoveKind, int>();
            int planned = 0, accidents = 0, onObjects = 0, free = 0, fastTicks = 0;
            int m3 = MotionCaps.AccelerationTicks;
            for (ulong layout = 1; layout <= 16; layout++)
            {
                SafetyObject[] objects = MixedTakeTests.MixedLayout(layout);
                if (!MixedTakeTests.TryMixed(objects, 5000 + layout, out DrivePlan? plan, out _, options)) continue;
                planned++;
                var spans = new List<(double From, double To)>();
                foreach (PlannedMove m in plan!.Moves.Where(m => ScriptMoveCodec.IsAccident(m.Kind)))
                {
                    accidents++;
                    kinds[m.Kind] = kinds.TryGetValue(m.Kind, out int n) ? n + 1 : 1;
                    PlannedMove over = plan.Moves.Single(x => x.Index == m.Overlies);
                    Assert.True(over.Start <= m.Start && m.Start < m.End, $"layout {layout}: {m} over {over}");
                    Assert.Equal(over.ObjectId, m.ObjectId);
                    ScriptMove record = m.Record(m.Start, plan.Moves.Count);
                    Assert.Equal(m.Kind, record.Kind);
                    Assert.Equal(0f, record.StartSeconds);
                    if (m.ObjectId != 0) onObjects++;
                    // In free space: no object (scenery aside) within 5 cm of the arm's model over the whole accident.
                    ArmCollisionModel near = SafetyScript.WithObjects(Model, objects.Where(o => !o.Scenery).ToArray(), 0.05f);
                    bool clear = true;
                    for (double t = Math.Max(m.Start, plan.ScriptStart); t <= Math.Min(m.End, plan.TotalSeconds) && clear; t += Tick) clear = ArmCollision.IsValid(Profile, near, plan.Sample(t));
                    if (clear) free++;
                    spans.Add((m.Start, m.End + AccidentModel.CapsTail));
                }

                // Every tick within the joint limits; within the arm's own limits in an accident, and people's caps everywhere else.
                var ticks = new List<ArmPose>();
                for (double t = plan.ScriptStart; t <= plan.TotalSeconds + 1e-9; t += Tick) ticks.Add(plan.Sample(t));
                for (int k = 1; k < ticks.Count; k++)
                {
                    Assert.True(Envelope.Contains(ticks[k]), $"layout {layout}: tick {k} outside the joint limits");
                    double at = plan.ScriptStart + k * Tick;
                    bool wild = spans.Any(s => at >= s.From - 2 * m3 * Tick && at <= s.To);
                    MotionCaps caps = wild ? arm : people;
                    for (int j = 0; j < DriveEnvelope.JointCount; j++)
                    {
                        double q0 = DrivePlan.Joint(ticks[k], j);
                        if (k < m3) continue;
                        double v = Math.Abs(q0 - DrivePlan.Joint(ticks[k - m3], j)) / (m3 * Tick);
                        Assert.True(v <= caps.JointSpeed[j] * 1.02 + 0.02, $"layout {layout}: {DriveEnvelope.JointName(j)} at {v:0.00} rad/s over {m3} ticks, tick {k}{(wild ? " (an accident)" : "")}");
                        Assert.True(v <= AccidentModel.ServoTopSpeed * 1.02 + 0.02, $"layout {layout}: {DriveEnvelope.JointName(j)} faster than the servos at {v:0.00} rad/s");
                        if (wild && v > people.JointSpeed[j]) fastTicks++;
                        if (k < 2 * m3) continue;
                        double a = Math.Abs(q0 - 2 * DrivePlan.Joint(ticks[k - m3], j) + DrivePlan.Joint(ticks[k - 2 * m3], j)) / (m3 * m3 * Tick * Tick);
                        Assert.True(a <= caps.JointAcceleration[j] * 1.02 + 0.5, $"layout {layout}: {DriveEnvelope.JointName(j)} at {a:0.0} rad/s2, tick {k}{(wild ? " (an accident)" : "")}");
                    }
                }
            }

            string report = $"{planned} planned, {accidents} accidents ({string.Join(", ", kinds.Select(kv => $"{ScriptMoveCodec.Name(kv.Key)} {kv.Value}"))}); {onObjects} on aims or passes, {free} in free space; {fastTicks} joint-ticks faster than people's caps";
            _output.WriteLine(report);
            Assert.True(planned >= 14, report);
            Assert.True(accidents >= planned, report); // about two a take by default
            foreach (ScriptMoveKind k in MixedOptions.AccidentKinds) Assert.True(kinds.ContainsKey(k), $"no {ScriptMoveCodec.Name(k)}: {report}");
            Assert.True(free >= 3 && onObjects >= 3, report); // in free space and on aims alike
        }

        [Fact]
        public void TheAccidentsOptions_ChooseKindsAndShares_AndNoneLeavesNoAccident()
        {
            SafetyObject[] objects = MixedTakeTests.MixedLayout(7);
            var none = new MixedOptions { AccidentShare = 0, AccidentNearShare = 0 };
            DrivePlan quiet = MixedTakeTests.AnyMixed(objects, 61, none), again = MixedTakeTests.AnyMixed(objects, 61, none);
            Assert.DoesNotContain(quiet.Moves, m => ScriptMoveCodec.IsAccident(m.Kind));
            Assert.Equal(quiet.Moves.Select(m => m.ToString()), again.Moves.Select(m => m.ToString()));
            var jerks = new MixedOptions { AccidentShare = 0.5, AccidentNearShare = 0.5, AccidentWeights = new double[] { 1, 0, 0, 0, 0 } };
            DrivePlan a = MixedTakeTests.AnyMixed(objects, 61, jerks), b = MixedTakeTests.AnyMixed(objects, 61, jerks);
            Assert.Equal(a.Moves.Select(m => m.ToString()), b.Moves.Select(m => m.ToString())); // deterministic with them
            List<PlannedMove> made = a.Moves.Where(m => ScriptMoveCodec.IsAccident(m.Kind)).ToList();
            Assert.NotEmpty(made);
            Assert.All(made, m => Assert.Equal(ScriptMoveKind.Jerk, m.Kind));
            Assert.Contains("accidents made of drawn: jerk", a.PlanningReport);
        }

        [Fact]
        public void TheAccidentsCaps_AreTheServosAtTheCommandedSpeed_NeverAboveTheirTopSpeed()
        {
            MotionCaps people = MixedOptions.DefaultCaps();
            MotionCaps fast = AccidentModel.CapsFor(people, 3.8f), slow = AccidentModel.CapsFor(people, 2.5f), uncapped = AccidentModel.CapsFor(null, 6f);
            for (int j = 0; j < DriveEnvelope.JointCount; j++)
            {
                Assert.Equal(MathF.Min(AccidentModel.ServoTopSpeed, MathF.Max(people.JointSpeed[j], 3.8f)), fast.JointSpeed[j]);
                Assert.Equal(MathF.Min(AccidentModel.ServoTopSpeed, MathF.Max(people.JointSpeed[j], 2.5f)), slow.JointSpeed[j]);
                Assert.Equal(AccidentModel.ServoTopSpeed, uncapped.JointSpeed[j]);
                Assert.True(fast.JointSpeed[j] >= people.JointSpeed[j] - 0.2f && fast.JointSpeed[j] <= AccidentModel.ServoTopSpeed);
                Assert.True(fast.JointAcceleration[j] >= people.JointAcceleration[j]);
            }

            // 917 servo units a second, the real servo's measured top speed (docs/adr/0038 "R, arm response"), at 238.73 units a radian.
            Assert.Equal(917.0 / 238.732407, AccidentModel.ServoTopSpeed, 2);
        }
    }
}
