using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Recording;
using static Teleop.RobotHost.Tests.Plant.ServoPathCollisionTests;

namespace Teleop.RobotHost.Tests.Plant
{
    /// <summary>
    /// The clamp (docs/adr/0020 amendment 2026-11-24): a joint command the collision check refuses is replaced by the farthest
    /// point toward it, on the joint-space line from the measured pose, that passes the same check; held only when none does.
    /// It never sends what the check refuses, the gripper passes through, the gate judges the clamped command, and refuse mode
    /// is the old behaviour.
    /// </summary>
    public sealed class HostCheckClampTests
    {
        private static readonly GenericArmPlantConfig Config = GenericArmPlantConfig.Default;
        private static readonly RobotArmProfile JetRover = Config.Profile;
        private static readonly ArmCollisionModel Model = JetRover.Collision!;
        private const float Deg = MathF.PI / 180f;

        // ---- the line search alone ------------------------------------------------------------

        private struct Stretches : IAlphaCheck
        {
            public float[] Pass; // pairs [from, to], inclusive
            public int Asked;

            public bool Passes(float alpha)
            {
                Asked++;
                for (int i = 0; i < Pass.Length; i += 2)
                {
                    if (alpha >= Pass[i] && alpha <= Pass[i + 1]) return true;
                }

                return false;
            }
        }

        [Fact]
        public void AMonotoneLine_IsBisectedToItsBoundary()
        {
            var check = new Stretches { Pass = new[] { 0f, 0.37f } };
            LineClampResult r = LineClamp.Search(ref check);
            Assert.Equal(LineClampKind.Bisection, r.Kind);
            Assert.True(r.Alpha <= 0.37f && r.Alpha + 1f / 256 > 0.37f, $"alpha {r.Alpha}");
            Assert.Equal(check.Asked, r.Checks);
            // The start, eight rounds, and the grid points beyond the failing bound (10/16 .. 15/16): all fail.
            Assert.Equal(1 + 8 + 10, r.Checks);
        }

        [Fact]
        public void ANonMonotoneLine_TakesTheLargestPassingGridPoint()
        {
            // Passes near the start, fails, then passes again from 0.7 to 0.8: bisection alone would stop near 0.2.
            var check = new Stretches { Pass = new[] { 0f, 0.2f, 0.7f, 0.8f } };
            LineClampResult r = LineClamp.Search(ref check);
            Assert.Equal(LineClampKind.Grid, r.Kind);
            Assert.Equal(12f / 16, r.Alpha);
            Assert.True(r.Checks <= LineClamp.MaxChecks);
        }

        [Fact]
        public void AFailingStart_OrNothingButTheStart_PassesNothing()
        {
            var failingStart = new Stretches { Pass = new[] { 0.3f, 0.6f } };
            LineClampResult r = LineClamp.Search(ref failingStart);
            Assert.Equal(LineClampKind.Nothing, r.Kind);
            Assert.Equal(1, r.Checks);

            var onlyStart = new Stretches { Pass = new[] { 0f, 0f } };
            r = LineClamp.Search(ref onlyStart);
            Assert.Equal(LineClampKind.Nothing, r.Kind);
            Assert.Equal(0f, r.Alpha);
            Assert.Equal(LineClamp.MaxChecks, r.Checks);
        }

        // ---- the plant -------------------------------------------------------------------------

        private static byte GripperMotor => Motor(JointRole.GripperMain);

        /// <summary>The plant's own belief (the measured pose as it converts the relay's pulses), by role.</summary>
        private static ArmPose Belief(GenericArmPlant plant)
        {
            var samples = new JointSample[JetRover.JointCount];
            plant.ReadJointState(samples);
            float A(JointRole role) => samples[JetRover.TryGetJointIndex(role)].Value;
            return new ArmPose(A(JointRole.BaseYaw), A(JointRole.Proximal), A(JointRole.Distal), A(JointRole.Wrist), A(JointRole.GripperRotate));
        }

        private static ArmPose IntoRanges(ArmPose p) => new ArmPose(Model.BaseYaw.Clamp(p.BaseYaw), Model.Proximal.Clamp(p.Proximal),
            Model.Distal.Clamp(p.Distal), Model.Wrist.Clamp(p.Wrist), Model.Roll.Clamp(p.Roll));

        /// <summary>Pairs of clear poses (measured, commanded) whose command the check refuses from the measured pose.</summary>
        internal static IEnumerable<(ArmPose At, ArmPose To)> RefusedPairs(int seed)
        {
            var rng = new Random(seed);
            for (int attempt = 0; attempt < 200000; attempt++)
            {
                ArmPose a = RandomValidPose(rng), b = RandomValidPose(rng);
                if (a.MaxJointDelta(b) < 20f * Deg) continue;
                if (PlantAt(a, new FakeRelayClient()).CommandJointAngles(Targets(b), captureTicks: 10) != JointCommandOutcome.Refused) continue;
                yield return (a, b);
            }
        }

        /// <summary>Commands <paramref name="targets"/> from <paramref name="at"/> in clamp mode; what the gate (which lets everything through) was shown.</summary>
        internal static (JointCommandOutcome Outcome, JointTarget[]? Sent, GenericArmPlant Plant, FakeRelayClient Relay) Clamp(ArmPose at, JointTarget[] targets, bool hold = false)
        {
            var relay = new FakeRelayClient();
            GenericArmPlant plant = PlantAt(at, relay, HostCheckOnFail.Clamp);
            JointTarget[]? shown = null;
            plant.CommandGate = (t, _) =>
            {
                shown = t.ToArray();
                return hold;
            };
            JointCommandOutcome outcome = plant.CommandJointAngles(targets, captureTicks: 10);
            return (outcome, shown, plant, relay);
        }

        private static bool PassesTheCheckFrom(ArmPose at, JointTarget[] targets) =>
            PlantAt(at, new FakeRelayClient()).CommandJointAngles(targets, captureTicks: 10) == JointCommandOutcome.Accepted;

        [Fact]
        public void AClampedCommand_LandsOnTheBoundary_WithinTheBisectionsTolerance_AndPassesTheCheck()
        {
            int bisected = 0;
            foreach ((ArmPose a, ArmPose b) in RefusedPairs(20261124))
            {
                var (outcome, sent, plant, relay) = Clamp(a, Targets(b));
                if (outcome != JointCommandOutcome.Clamped || plant.LastClamp.Kind != LineClampKind.Bisection) continue;

                float alpha = plant.LastClamp.Alpha;
                Assert.InRange(alpha, 1f / 256, 255f / 256);
                Assert.Equal(alpha * 256, MathF.Round(alpha * 256)); // a bisection point
                Assert.Single(relay.SentCommands);
                Assert.Equal(1, plant.ClampedJointCommands);
                Assert.Equal(0, plant.RefusedJointCommands);

                // What was sent is the point alpha along the line from the measured pose (into the model's ranges) to the command,
                // it passes the very check, and 1/256 further does not.
                ArmPose start = IntoRanges(Belief(plant));
                JointTarget[] expected = Targets(ArmPose.Lerp(start, b, alpha));
                Assert.Equal(expected, sent);
                Assert.True(PassesTheCheckFrom(a, sent!));
                Assert.False(PassesTheCheckFrom(a, Targets(ArmPose.Lerp(start, b, alpha + 1f / 256))));
                if (++bisected == 10) return;
            }

            Assert.Fail($"only {bisected} bisected clamps were found to test with");
        }

        [Fact]
        public void TheClamp_NeverSendsWhatTheCheckRefuses()
        {
            // The asserted invariant, over many refused commands: every clamped command passes the same check from the same
            // measured pose, the re-check never fires, and a command that fails is never sent as it was.
            int clamped = 0, held = 0;
            foreach ((ArmPose a, ArmPose b) in RefusedPairs(7))
            {
                var (outcome, sent, plant, relay) = Clamp(a, Targets(b));
                Assert.Equal(0, plant.ClampInvariantViolations);
                Assert.NotEqual(JointCommandOutcome.Accepted, outcome);
                if (outcome == JointCommandOutcome.Clamped)
                {
                    clamped++;
                    Assert.True(PassesTheCheckFrom(a, sent!), "a clamped command failed the check");
                    Assert.NotEqual(Targets(b), sent);
                    Assert.Single(relay.SentCommands);
                }
                else
                {
                    held++;
                    Assert.Equal(JointCommandOutcome.Refused, outcome);
                    Assert.Empty(relay.SentCommands);
                }

                if (clamped + held == 300) break;
            }

            Assert.True(clamped >= 250, $"{clamped} clamped, {held} held of 300");
        }

        [Fact]
        public void ANonMonotoneLine_InTheRealModel_TakesTheGrid()
        {
            foreach ((ArmPose a, ArmPose b) in RefusedPairs(3).Take(4000))
            {
                var (outcome, sent, plant, _) = Clamp(a, Targets(b));
                if (outcome != JointCommandOutcome.Clamped || plant.LastClamp.Kind != LineClampKind.Grid) continue;

                float alpha = plant.LastClamp.Alpha;
                Assert.Equal(alpha * 16, MathF.Round(alpha * 16)); // a grid point
                Assert.Equal(1, plant.GridClamps);
                Assert.True(PassesTheCheckFrom(a, sent!));
                // Below it, somewhere, the line fails: that is what made it non-monotone.
                ArmPose start = IntoRanges(Belief(plant));
                bool failsBelow = Enumerable.Range(1, 255).Select(k => k / 256f).Where(x => x < alpha)
                    .Any(x => !PassesTheCheckFrom(a, Targets(ArmPose.Lerp(start, b, x))));
                Assert.True(failsBelow);
                return;
            }

            Assert.Fail("no non-monotone line was found in 4000 refused commands");
        }

        [Fact]
        public void WhenNothingOnTheLinePasses_TheArmHolds_AsBefore()
        {
            // Measured in a pose the model calls colliding (inside the ranges), commanded to another colliding pose: the line's
            // start fails, so nothing is sent.
            var rng = new Random(11);
            float U(AngleRange r) => r.Min + (float)rng.NextDouble() * (r.Max - r.Min);
            ArmPose RandomColliding()
            {
                while (true)
                {
                    var p = new ArmPose(U(Model.BaseYaw), U(Model.Proximal), U(Model.Distal), U(Model.Wrist), U(Model.Roll));
                    if (!ArmCollision.IsValid(JetRover, Model, p)) return p;
                }
            }

            for (int attempt = 0; attempt < 1000; attempt++)
            {
                ArmPose a = RandomColliding(), b = RandomColliding();
                var relay = new FakeRelayClient();
                GenericArmPlant plant = PlantAt(a, relay, HostCheckOnFail.Clamp);
                if (ArmCollision.IsValid(JetRover, Model, IntoRanges(Belief(plant)))) continue; // the round trip through pulses cleared it
                Assert.Equal(JointCommandOutcome.Refused, plant.CommandJointAngles(Targets(b), captureTicks: 10));
                Assert.Empty(relay.SentCommands);
                Assert.Equal(1, plant.RefusedJointCommands);
                Assert.Equal(0, plant.ClampedJointCommands);
                Assert.Equal(LineClampKind.Nothing, plant.LastClamp.Kind);
                Assert.Equal(1, plant.LastClamp.Checks);
                return;
            }

            Assert.Fail("no colliding pair was found");
        }

        [Fact]
        public void TheGripper_PassesThroughUnchanged_AndEverySpeedToo()
        {
            (ArmPose a, ArmPose b) = RefusedPairs(5).First(p => Clamp(p.At, Targets(p.To)).Outcome == JointCommandOutcome.Clamped);
            JointTarget[] command = Targets(b).Select(t => new JointTarget(t.MotorId, t.Angle, 1.25f))
                .Append(new JointTarget(GripperMotor, 0.7f, 0.5f)).ToArray();
            var (outcome, sent, _, _) = Clamp(a, command);
            Assert.Equal(JointCommandOutcome.Clamped, outcome);
            JointTarget gripper = sent!.Single(t => t.MotorId == GripperMotor);
            Assert.Equal(0.7f, gripper.Angle);
            Assert.Equal(0.5f, gripper.Speed);
            Assert.All(sent!.Where(t => t.MotorId != GripperMotor), t => Assert.Equal(1.25f, t.Speed));
            Assert.Equal(command.Select(t => t.MotorId), sent!.Select(t => t.MotorId));
        }

        [Fact]
        public void TheGate_JudgesTheClampedCommand_AndAHoldSendsNothing()
        {
            (ArmPose a, ArmPose b) = RefusedPairs(9).First(p => Clamp(p.At, Targets(p.To)).Outcome == JointCommandOutcome.Clamped);

            var (outcome, shown, plant, relay) = Clamp(a, Targets(b));
            Assert.Equal(JointCommandOutcome.Clamped, outcome);
            Assert.NotNull(shown);
            Assert.NotEqual(Targets(b), shown);
            Assert.True(PassesTheCheckFrom(a, shown!));
            Assert.Single(relay.SentCommands);

            var (heldOutcome, heldShown, heldPlant, heldRelay) = Clamp(a, Targets(b), hold: true);
            Assert.Equal(JointCommandOutcome.Held, heldOutcome);
            Assert.Equal(shown, heldShown);
            Assert.Empty(heldRelay.SentCommands);
            Assert.Equal(1, heldPlant.HeldJointCommands);
            Assert.Equal(0, heldPlant.ClampedJointCommands);
            Assert.NotEqual(LineClampKind.Nothing, heldPlant.LastClamp.Kind); // the clamp ran, then the gate held what it made
        }

        [Fact]
        public void RefuseMode_AndThePark_RefuseAsBefore()
        {
            (ArmPose a, ArmPose b) = RefusedPairs(13).First(p => Clamp(p.At, Targets(p.To)).Outcome == JointCommandOutcome.Clamped);

            var relay = new FakeRelayClient();
            GenericArmPlant refusing = PlantAt(a, relay, HostCheckOnFail.Refuse);
            Assert.Equal(JointCommandOutcome.Refused, refusing.CommandJointAngles(Targets(b), captureTicks: 10));
            Assert.Empty(relay.SentCommands);
            Assert.Equal(0, refusing.LastClamp.Checks); // no search at all

            // RobotHost's own park never clamps: its path was checked before it began, so a refusal there is a bug, and holds.
            var parkRelay = new FakeRelayClient();
            GenericArmPlant parking = PlantAt(a, parkRelay, HostCheckOnFail.Clamp);
            Assert.Equal(JointCommandOutcome.Refused, parking.CommandJointAngles(Targets(b), captureTicks: 10, askGate: false, mayClamp: false));
            Assert.Empty(parkRelay.SentCommands);
        }

        [Fact]
        public void ThePlant_ClampsByDefault()
        {
            Assert.Equal(HostCheckOnFail.Clamp, new GenericArmPlant(Config, new FakeRelayClient()).OnHostCheckFail);
        }

        // ---- the offline replay -----------------------------------------------------------------

        private static string RiLine(long t, ArmPose at)
        {
            string E(JointRole role, float angle)
            {
                JointHardwareSpec j = JetRover.Joints[JetRover.TryGetJointIndex(role)];
                float pulse = Config.ZeroPulse + (angle + j.ZeroOffsetRadians) * Config.PulsePerRadian;
                return j.MotorId.ToString(CultureInfo.InvariantCulture) + ":1:" + pulse.ToString("R", CultureInfo.InvariantCulture);
            }

            return "RI|" + t + "|" + string.Join(";", E(JointRole.BaseYaw, at.BaseYaw), E(JointRole.Proximal, at.Proximal),
                E(JointRole.Distal, at.Distal), E(JointRole.Wrist, at.Wrist), E(JointRole.GripperRotate, at.Roll));
        }

        [Fact]
        public void TheReplay_ReproducesARecordedRefusal_InRefuseMode_AndClampsItInClampMode()
        {
            (ArmPose a, ArmPose b) = RefusedPairs(17).First(p => Clamp(p.At, Targets(p.To)).Outcome == JointCommandOutcome.Clamped);
            string[] lines =
            {
                "JSES|1|robothost|1000000000|CLOCK_MONOTONIC|x",
                RiLine(100, a),
                SessionLines.JointCommand(110, 110, 1, 50, JointCommandOutcome.Accepted, Targets(a)),
                RiLine(120, a),
                SessionLines.JointCommand(130, 130, 2, 60, JointCommandOutcome.Refused, Targets(b)),
                SessionLines.JointCommand(131, 131, 3, 60, JointCommandOutcome.Stale, Targets(b)),
                SessionLines.JointCommand(132, 132, 4, 61, JointCommandOutcome.Parked, Targets(b)),
            };

            var refuse = new CommandReplay(Config, HostCheckOnFail.Refuse);
            foreach (string line in lines) refuse.Line(line);
            Assert.Equal(3, refuse.Commands);
            Assert.Equal(0, refuse.Mismatches);
            Assert.Equal(new[] { "A>A", "R>R", "S>S" }, refuse.Outcomes.Keys);

            var clamp = new CommandReplay(Config, HostCheckOnFail.Clamp);
            foreach (string line in lines) clamp.Line(line);
            Assert.Equal(new[] { "A>A", "R>C", "S>S" }, clamp.Outcomes.Keys);
            Assert.Single(clamp.Alphas);
            Assert.Equal(0, clamp.Plant.ClampInvariantViolations);
        }
    }
}
