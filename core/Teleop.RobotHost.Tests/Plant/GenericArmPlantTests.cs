using System.Linq;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Plant
{
    public class GenericArmPlantTests
    {
        private static readonly GenericArmPlantConfig Config = GenericArmPlantConfig.Default;
        private static readonly RobotArmProfile JetRover = Config.Profile;

        // JetRoverMeasuredDefault's own motor ids -- see RobotArmProfile.JetRoverMeasuredDefault.
        private const byte BaseMotorId = 1, ProximalMotorId = 2, DistalMotorId = 3, WristMotorId = 4, GripperMotorId = 5;

        private static CommandFrame Frame(long captureTicks, Vector3 position, Quaternion? rotation = null, float gripper = 0f) =>
            new CommandFrame(
                sequence: 1,
                ackSequence: 0,
                captureTicks: captureTicks,
                pose: new Pose(position, rotation ?? Quaternion.Identity),
                linearVelocity: Vector3.Zero,
                angularVelocity: Vector3.Zero,
                gripper: gripper);

        // A reachable target roughly in front of and level with the shoulder, well inside the
        // arm's working envelope for these link lengths.
        private static readonly Vector3 ReachableTarget = new Vector3(0.15f, 0f, 0.08f);

        private static float Pulse(JointTarget[] sent, byte motorId) => sent.First(t => t.MotorId == motorId).Angle;

        private static JointTarget[] JointAnglesFor(
            RobotArmProfile profile, float baseYaw, float proximalPitch, float distalPitch, float gripper) =>
            profile.Joints.Select(j => new JointTarget(
                j.MotorId,
                j.Role switch
                {
                    JointRole.BaseYaw => baseYaw,
                    JointRole.Proximal => proximalPitch,
                    JointRole.Distal => distalPitch,
                    JointRole.Wrist => 0f,
                    JointRole.GripperMain => gripper,
                    _ => 0f,
                },
                speed: 0f)).ToArray();

        [Fact]
        public void Command_SendsANonZeroPulseTargetTowardTheIkTarget()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            plant.Command(Frame(captureTicks: 10, position: ReachableTarget));

            Assert.Single(relay.SentCommands);
            JointTarget[] sent = relay.SentCommands[0];
            // Starting from the zero-pulse (arm-centered) belief, any reachable off-center target
            // should produce a real move on at least one joint -- the sent value is the resulting
            // absolute pulse (docs/adr/0010/0011), so "moved" means "differs from ZeroPulse."
            bool anyJointMoved = sent.Any(t => t.Angle != Config.ZeroPulse);
            Assert.True(anyJointMoved);
        }

        [Fact]
        public void Command_RepeatingTheSameTarget_GraduallyConvergesToAStableAbsoluteTarget()
        {
            // Regression test for a real bug: an earlier version updated this plant's belief to
            // the full, unclamped IK target after every Command() call, even when the amount
            // actually applied had been clamped to a smaller step -- silently stalling out any
            // move whose single-step delta exceeded MaxDirectionMagnitude forever. The belief must
            // instead accumulate only the clamped, actually-applied delta each call, so repeating
            // the same target keeps closing the remaining distance until the sent absolute pulse
            // stops changing between calls (converged).
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            for (int i = 0; i < 20; i++)
            {
                plant.Command(Frame(captureTicks: 10 + i, position: ReachableTarget));
            }

            JointTarget[] secondToLast = relay.SentCommands[relay.SentCommands.Count - 2];
            JointTarget[] last = relay.SentCommands[relay.SentCommands.Count - 1];
            foreach (byte motorId in new[] { BaseMotorId, ProximalMotorId, DistalMotorId, WristMotorId })
            {
                Assert.True(
                    MathF.Abs(Pulse(last, motorId) - Pulse(secondToLast, motorId)) < 0.5f,
                    $"motor {motorId} still moving: {Pulse(last, motorId)} vs {Pulse(secondToLast, motorId)}");
            }
        }

        [Fact]
        public void Command_ClampsEachJointsPerCallMovementToConfiguredMaximum()
        {
            // GenericArmPlantConfig.Default's own MaxDirectionMagnitude (20, as of 2026-08-22) is
            // deliberately tuned to span this hardware's entire 0-1000 pulse range in one call --
            // see that field's own doc -- so no physically legal target can actually exercise the
            // clamp against Config itself anymore. This test verifies the CLAMP MECHANISM, not
            // today's production tuning, so it deliberately uses a small dedicated magnitude via
            // ConfigWithMaxDirectionMagnitude instead of Config -- keeping this test meaningful
            // regardless of how the deployed default gets tuned in the future.
            var configWithSmallClamp = ConfigWithMaxDirectionMagnitude(2f);
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(configWithSmallClamp, relay);

            // Full extension straight up is about as far from the zero-pulse start as this arm
            // can be commanded in one step -- expect at least one axis to hit the clamp.
            var extremeTarget = new Vector3(
                0f, 0f, JetRover.BaseHeight + JetRover.ProximalLinkLength + JetRover.DistalLinkLength - 0.001f);

            plant.Command(Frame(captureTicks: 10, position: extremeTarget));

            float maxStep = configWithSmallClamp.MaxDirectionMagnitude * configWithSmallClamp.StepSizePulses;
            JointTarget[] sent = relay.SentCommands[0];
            bool anyJointHitTheClamp = false;
            foreach (byte motorId in new[] { BaseMotorId, ProximalMotorId, DistalMotorId, WristMotorId })
            {
                float step = MathF.Abs(Pulse(sent, motorId) - configWithSmallClamp.ZeroPulse);
                Assert.True(step <= maxStep + 1e-3f);
                anyJointHitTheClamp |= MathF.Abs(step - maxStep) < 1e-3f;
            }

            Assert.True(anyJointHitTheClamp, "expected at least one joint to actually hit the clamp -- otherwise this test isn't testing what it claims to");
        }

        private static GenericArmPlantConfig ConfigWithMaxDirectionMagnitude(float magnitude) =>
            new GenericArmPlantConfig(
                JetRover, Config.PulsePerRadian, Config.PulsesPerSecond, Config.StepSizePulses,
                magnitude, Config.ZeroPulse, Config.MinPulse, Config.MaxPulse,
                Config.GripperOpenPulse, Config.GripperClosedPulse);

        private static GenericArmPlantConfig ConfigWithProximalFloor(float floorPulse)
        {
            float floorAngleRadians = (floorPulse - Config.ZeroPulse) / Config.PulsePerRadian;
            var joints = JetRover.Joints.Select(j => j.Role == JointRole.Proximal
                ? new JointHardwareSpec(j.MotorId, j.Role, j.WristIndex, minAngleRadians: floorAngleRadians, maxAngleRadians: j.MaxAngleRadians)
                : j).ToArray();
            var profile = new RobotArmProfile(
                JetRover.Name, JetRover.HasRotatingBase, JetRover.BaseHeight, JetRover.ProximalLinkLength,
                JetRover.DistalLinkLength, JetRover.WristJointCount, JetRover.HasGripper, JetRover.GripperCanRotate, joints);
            return new GenericArmPlantConfig(
                profile, Config.PulsePerRadian, Config.PulsesPerSecond, Config.StepSizePulses,
                Config.MaxDirectionMagnitude, Config.ZeroPulse, Config.MinPulse, Config.MaxPulse,
                Config.GripperOpenPulse, Config.GripperClosedPulse);
        }

        [Fact]
        public void Command_ProximalJointNeverGoesBelowConfiguredFloor()
        {
            // Regression test for two real incidents on the JetRover hardware this generalizes
            // from (2026-08-08). First: the lower (proximal) arm collided with the robot's own
            // base plate at a real physical target -- a per-joint floor exists specifically to
            // make that unreachable in software, independent of what the IK target asks for.
            // Second: this limit was originally implemented as an upper bound, which does nothing
            // to stop the target from going *below* it -- exactly the dangerous direction, since
            // lower pulse values drive this joint toward the plate. This target's unclamped IK
            // puts the proximal joint well below center; the configured floor of 400 must win
            // regardless of how many repeated commands try to keep pushing below it.
            var configWithFloor = ConfigWithProximalFloor(400f);
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(configWithFloor, relay);

            // Straight down: pushes the unclamped proximal-joint IK target well below the
            // configured floor -- the mirror image of the "straight up" target used to test the
            // opposite clamp in Command_ClampsEachJointsPerCallMovementToConfiguredMaximum.
            var straightDownTarget = new Vector3(
                0f, 0f, -(JetRover.BaseHeight + JetRover.ProximalLinkLength + JetRover.DistalLinkLength - 0.001f));

            for (int i = 0; i < 20; i++)
            {
                plant.Command(Frame(captureTicks: 10 + i, position: straightDownTarget));
            }

            JointTarget[] secondToLast = relay.SentCommands[relay.SentCommands.Count - 2];
            JointTarget[] last = relay.SentCommands[relay.SentCommands.Count - 1];
            Assert.True(
                Pulse(last, ProximalMotorId) >= 400f - 1e-3f,
                $"proximal joint pulse {Pulse(last, ProximalMotorId)} fell below the configured floor of 400");
            Assert.True(
                MathF.Abs(Pulse(last, ProximalMotorId) - Pulse(secondToLast, ProximalMotorId)) < 0.5f,
                "proximal joint still moving after 20 calls, should have converged at the floor");
        }

        private static GenericArmPlantConfig ConfigWithProximalZeroOffset(float offsetRadians)
        {
            var joints = JetRover.Joints.Select(j => j.Role == JointRole.Proximal
                ? new JointHardwareSpec(j.MotorId, j.Role, j.WristIndex, j.MinAngleRadians, j.MaxAngleRadians, offsetRadians)
                : j).ToArray();
            var profile = new RobotArmProfile(
                JetRover.Name, JetRover.HasRotatingBase, JetRover.BaseHeight, JetRover.ProximalLinkLength,
                JetRover.DistalLinkLength, JetRover.WristJointCount, JetRover.HasGripper, JetRover.GripperCanRotate, joints);
            return new GenericArmPlantConfig(
                profile, Config.PulsePerRadian, Config.PulsesPerSecond, Config.StepSizePulses,
                Config.MaxDirectionMagnitude, Config.ZeroPulse, Config.MinPulse, Config.MaxPulse,
                Config.GripperOpenPulse, Config.GripperClosedPulse);
        }

        [Fact]
        public void CommandJointAngles_AddsZeroOffsetBeforeConvertingToPulse()
        {
            // Regression test for a real hardware calibration finding (2026-08-17): the proximal
            // joint's servo horn can't be mounted at an exact zero, so a mounting correction is
            // added to the commanded kinematic angle before it becomes a pulse target. Kept small
            // enough (well under MaxDirectionMagnitude * StepSizePulses = 250 pulses) that this
            // lands in one call, unclamped, so the sent pulse is exactly checkable.
            const float offset = 0.2f;
            const float commandedAngle = 0.05f;
            GenericArmPlantConfig config = ConfigWithProximalZeroOffset(offset);
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(config, relay);

            JointTarget[] targets = JointAnglesFor(config.Profile, baseYaw: 0f, proximalPitch: commandedAngle, distalPitch: 0f, gripper: 0f);
            plant.CommandJointAngles(targets, captureTicks: 10);

            float expectedPulse = config.ZeroPulse + (commandedAngle + offset) * config.PulsePerRadian;
            Assert.Equal(expectedPulse, Pulse(relay.SentCommands[0], ProximalMotorId), 2);
        }

        [Fact]
        public void State_RoundTripsZeroOffsetBackToTheOriginalCommandedAngle()
        {
            // ApplyJointTargets adds ZeroOffsetRadians before converting to pulse (the mounting
            // correction above); State's GetBeliefAngle must subtract it back out when converting
            // the tracked pulse back to a kinematic angle, or State would report a position shifted
            // by the mounting correction even though the arm is actually where it was commanded --
            // a bug this test would have caught before it ever reached real hardware.
            const float offset = 0.2f;
            const float commandedAngle = 0.05f;
            GenericArmPlantConfig config = ConfigWithProximalZeroOffset(offset);
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(config, relay);

            JointTarget[] targets = JointAnglesFor(config.Profile, baseYaw: 0f, proximalPitch: commandedAngle, distalPitch: 0f, gripper: 0f);
            plant.CommandJointAngles(targets, captureTicks: 10);

            Vector3 expectedPosition = ArmKinematics.Forward(config.Profile, baseYaw: 0f, proximalPitch: commandedAngle, distalPitch: 0f);
            Vector3 actualPosition = plant.State.Value.Position;
            Assert.True(
                Vector3.Distance(expectedPosition, actualPosition) < 1e-3f,
                $"expected {expectedPosition}, got {actualPosition} -- ZeroOffsetRadians not reversed correctly in State");
        }

        [Fact]
        public void Command_DenormalizesGripperToConfiguredPulseRange()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            plant.Command(Frame(captureTicks: 10, position: ReachableTarget, gripper: 0f));
            plant.Command(Frame(captureTicks: 20, position: ReachableTarget, gripper: 1f));
            plant.Command(Frame(captureTicks: 30, position: ReachableTarget, gripper: 0.5f));

            Assert.Equal(Config.GripperOpenPulse, Pulse(relay.SentCommands[0], GripperMotorId), 3);
            Assert.Equal(Config.GripperClosedPulse, Pulse(relay.SentCommands[1], GripperMotorId), 3);
            Assert.Equal((Config.GripperOpenPulse + Config.GripperClosedPulse) / 2f, Pulse(relay.SentCommands[2], GripperMotorId), 3);
        }

        [Fact]
        public void Command_RejectsStaleOrDuplicateFramesWhole()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            plant.Command(Frame(captureTicks: 100, position: ReachableTarget));
            int afterFirst = relay.SentCommands.Count;

            plant.Command(Frame(captureTicks: 100, position: new Vector3(0.2f, 0f, 0.1f))); // duplicate stamp
            plant.Command(Frame(captureTicks: 50, position: new Vector3(0.2f, 0f, 0.1f)));  // stale

            Assert.Equal(afterFirst, relay.SentCommands.Count);
        }

        [Fact]
        public void Step_UpdatesStateFromFullValidFeedback()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            relay.EnqueueFeedback(
                new JointFeedbackEntry(BaseMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(ProximalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(DistalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(WristMotorId, true, Config.ZeroPulse));

            plant.Step(nowTicks: 1000);

            Assert.True(plant.IsFullySensed);
            // All joints at ZeroPulse (angle 0) should forward-kinematic to the arm's own resting
            // reach: base yaw 0, proximal/distal pitch 0 -> straight out along +X.
            Pose pose = plant.State.Value;
            Assert.Equal(1000, plant.State.CaptureTicks);
            Assert.True(pose.Position.X > 0f, $"expected a positive X reach at rest, got {pose.Position}");
        }

        [Fact]
        public void Command_FirstUseSeedsBeliefFromSensedData_NotZeroPulse()
        {
            // Regression test for a real incident: Step() polls the relay for feedback from
            // startup regardless of whether any command has arrived, so by the time the first
            // real Command() lands, a joint is often already sensed away from ZeroPulse.
            // Computing the first step from an unseeded ZeroPulse belief instead of the real
            // sensed position sizes that step for the wrong starting point.
            //
            // When a move is *not* clamped, the resulting absolute target converges to exactly
            // the same value regardless of the starting belief (the starting point algebraically
            // cancels out of `belief + clamp(target - belief)`), so seeding only produces an
            // observably different *sent* value when the move needs clamping -- this test
            // therefore uses the same extreme target as
            // Command_ClampsEachJointsPerCallMovementToConfiguredMaximum, not ReachableTarget, and
            // (for the same reason given there) a dedicated small MaxDirectionMagnitude rather
            // than Config's own -- Config.MaxDirectionMagnitude (20, as of 2026-08-22) is tuned to
            // span this hardware's full pulse range in one call, so no physically legal target can
            // exercise the clamp against Config itself anymore.
            var extremeTarget = new Vector3(
                0f, 0f, JetRover.BaseHeight + JetRover.ProximalLinkLength + JetRover.DistalLinkLength - 0.001f);
            var clampConfig = ConfigWithMaxDirectionMagnitude(2f);

            var seededRelay = new FakeRelayClient();
            var seededPlant = new GenericArmPlant(clampConfig, seededRelay);
            float sensedProximalPulse = 330f;
            seededRelay.EnqueueFeedback(
                new JointFeedbackEntry(BaseMotorId, true, 340f),
                new JointFeedbackEntry(ProximalMotorId, true, sensedProximalPulse),
                new JointFeedbackEntry(DistalMotorId, false, 0f),
                new JointFeedbackEntry(WristMotorId, true, 340f));
            seededPlant.Step(nowTicks: 1000);

            var unseededRelay = new FakeRelayClient();
            var unseededPlant = new GenericArmPlant(clampConfig, unseededRelay); // never Step()'d -- belief stays ZeroPulse

            seededPlant.Command(Frame(captureTicks: 2000, position: extremeTarget));
            unseededPlant.Command(Frame(captureTicks: 2000, position: extremeTarget));

            float seededSentProximal = Pulse(seededRelay.SentCommands[0], ProximalMotorId);
            float unseededSentProximal = Pulse(unseededRelay.SentCommands[0], ProximalMotorId);

            // Hand-derive the expected resulting absolute pulse directly against the real sensed
            // starting point, exactly as production code does, rather than hardcoding a
            // precomputed magic number. Confirm the move actually clamps -- otherwise this test
            // would silently stop testing what it claims to.
            Span<float> wristPitches = stackalloc float[JetRover.WristJointCount];
            ArmKinematics.TryInverse(JetRover, extremeTarget, 0f, out _, out float proximalPitch, out _, wristPitches, out _);
            float targetProximalPulse = clampConfig.ZeroPulse + proximalPitch * clampConfig.PulsePerRadian;
            float expectedDirection = System.Math.Clamp(
                (targetProximalPulse - sensedProximalPulse) / clampConfig.StepSizePulses,
                -clampConfig.MaxDirectionMagnitude, clampConfig.MaxDirectionMagnitude);
            Assert.Equal(clampConfig.MaxDirectionMagnitude, MathF.Abs(expectedDirection), 3);

            Assert.NotEqual(unseededSentProximal, seededSentProximal);

            float expectedProximalPulse = sensedProximalPulse + expectedDirection * clampConfig.StepSizePulses;
            Assert.Equal(expectedProximalPulse, seededSentProximal, 3);
        }

        [Fact]
        public void State_FallsBackToLastCommandedTarget_ForAJointThatHasNeverBeenSensed()
        {
            // Real, observed hardware limitation this generalizes from: JetRover's distal-arm
            // servo never responds to position-read requests at all (writes work fine, only
            // reads never succeed). A joint stuck at IsFullySensed=false forever must not report
            // State as if it were still at its power-on default while every other joint shows
            // real progress -- falling back to this plant's own last-commanded target is a
            // materially better estimate.
            //
            // No Command() call in this test, so every joint's target belief is still exactly
            // ZeroPulse (angle 0) -- distal's fallback and its "never left the default" starting
            // point are the same value here.
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            // Base sensed exactly at center; proximal sensed away from center; distal (this
            // profile's known-bad servo) never gets sensed.
            float sensedProximalPulse = 400f;
            relay.EnqueueFeedback(
                new JointFeedbackEntry(BaseMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(ProximalMotorId, true, sensedProximalPulse),
                new JointFeedbackEntry(DistalMotorId, false, 0f),
                new JointFeedbackEntry(WristMotorId, true, Config.ZeroPulse));
            plant.Step(nowTicks: 1000);

            Assert.False(plant.IsFullySensed);
            // Reverses GenericArmPlant's own "add ZeroOffsetRadians before converting to pulse"
            // step -- State must subtract it back out for a round trip to stay consistent (see
            // GenericArmPlant.GetBeliefAngle's own doc).
            float proximalZeroOffset = JetRover.Joints.First(j => j.Role == JointRole.Proximal).ZeroOffsetRadians;
            float proximalPitchFromSensedPulse = (sensedProximalPulse - Config.ZeroPulse) / Config.PulsePerRadian - proximalZeroOffset;
            Vector3 expectedPosition = ArmKinematics.Forward(
                JetRover, baseYaw: 0f, proximalPitch: proximalPitchFromSensedPulse, distalPitch: 0f);
            Vector3 reportedPosition = plant.State.Value.Position;
            Assert.True(
                Vector3.Distance(expectedPosition, reportedPosition) < 1e-3f,
                $"expected fallback position near {expectedPosition}, got {reportedPosition}");
        }

        [Fact]
        public void Step_PartialFeedback_LeavesIsFullySensedFalse()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            relay.EnqueueFeedback(
                new JointFeedbackEntry(BaseMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(ProximalMotorId, false, 0f),
                new JointFeedbackEntry(DistalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(WristMotorId, true, Config.ZeroPulse));

            plant.Step(nowTicks: 1000);

            Assert.False(plant.IsFullySensed);
        }

        [Fact]
        public void Step_AtOrBeforeCurrentStateTime_IsNoOp()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            relay.EnqueueFeedback(
                new JointFeedbackEntry(BaseMotorId, true, 510f),
                new JointFeedbackEntry(ProximalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(DistalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(WristMotorId, true, Config.ZeroPulse));
            plant.Step(nowTicks: 1000);
            bool sensedAfterFirstStep = plant.IsFullySensed;

            relay.EnqueueFeedback(
                new JointFeedbackEntry(BaseMotorId, true, 999f),
                new JointFeedbackEntry(ProximalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(DistalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(WristMotorId, true, Config.ZeroPulse));
            plant.Step(nowTicks: 1000); // not after the current state time -- must not consume feedback

            Assert.True(sensedAfterFirstStep);
            Assert.Equal(1000, plant.State.CaptureTicks);
        }

        [Fact]
        public void Reset_ClearsBookkeeping_AcceptsNextFrameAtAnyCaptureTicks()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            plant.Command(Frame(captureTicks: 1000, position: ReachableTarget));
            relay.EnqueueFeedback(
                new JointFeedbackEntry(BaseMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(ProximalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(DistalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(WristMotorId, true, Config.ZeroPulse));
            plant.Step(nowTicks: 1000);

            plant.Reset();

            Assert.False(plant.IsFullySensed);
            Assert.Equal(0, plant.State.CaptureTicks);

            // A reused instance (sweeps reuse instances across trials) must accept the next
            // trial's first command whatever its stamp, even one lower than before Reset, and
            // must compute its delta from the reset (zero-pulse) target, not the pre-reset one.
            int beforeCount = relay.SentCommands.Count;
            plant.Command(Frame(captureTicks: 5, position: ReachableTarget));
            Assert.Equal(beforeCount + 1, relay.SentCommands.Count);
        }

        [Fact]
        public void Reset_DoesNotSendAnyRelayCommand()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            plant.Command(Frame(captureTicks: 1000, position: ReachableTarget));
            int beforeReset = relay.SentCommands.Count;

            plant.Reset();

            Assert.Equal(beforeReset, relay.SentCommands.Count); // Reset itself sent nothing
        }

        // CommandJointAngles (docs/adr/0009) shares ApplyJointTargets's tail with Command -- these
        // tests establish parity with the equivalent Command(CommandFrame) tests above, not a full
        // second copy of every scenario.

        [Fact]
        public void CommandJointAngles_SendsANonZeroPulseTargetTowardTheGivenAngles()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            plant.CommandJointAngles(
                JointAnglesFor(JetRover, baseYaw: 0.3f, proximalPitch: 0.2f, distalPitch: -0.1f, gripper: 0f),
                captureTicks: 10);

            Assert.Single(relay.SentCommands);
            JointTarget[] sent = relay.SentCommands[0];
            Assert.Contains(sent, t => t.Angle != Config.ZeroPulse);
        }

        [Fact]
        public void CommandJointAngles_RejectsStaleOrDuplicateCaptureTicks()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            plant.CommandJointAngles(JointAnglesFor(JetRover, 0.3f, 0.2f, -0.1f, 0f), captureTicks: 100);
            int afterFirst = relay.SentCommands.Count;

            plant.CommandJointAngles(JointAnglesFor(JetRover, 0.5f, 0.4f, -0.3f, 0f), captureTicks: 100); // duplicate stamp
            plant.CommandJointAngles(JointAnglesFor(JetRover, 0.5f, 0.4f, -0.3f, 0f), captureTicks: 50);  // stale

            Assert.Equal(afterFirst, relay.SentCommands.Count);
        }

        [Fact]
        public void CommandJointAngles_ProximalJointNeverGoesBelowConfiguredFloor()
        {
            // Mirrors Command_ProximalJointNeverGoesBelowConfiguredFloor, but supplies the
            // proximal pitch directly instead of deriving it from a Cartesian target via IK --
            // CommandJointAngles skips TryInverse entirely, so this exercises ApplyJointTargets's
            // shared clamp/belief tail on its own.
            var configWithFloor = ConfigWithProximalFloor(400f);
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(configWithFloor, relay);

            // proximalPitch=-1.6 rad computes to an unclamped pulse well below the 400 floor
            // (ZeroPulse=500, PulsePerRadian~238.73 => ~500-383=117).
            for (int i = 0; i < 20; i++)
            {
                plant.CommandJointAngles(
                    JointAnglesFor(configWithFloor.Profile, baseYaw: 0f, proximalPitch: -1.6f, distalPitch: 0f, gripper: 0f),
                    captureTicks: 10 + i);
            }

            float lastProximalPulse = Pulse(relay.SentCommands[relay.SentCommands.Count - 1], ProximalMotorId);
            Assert.True(
                lastProximalPulse >= 400f - 1e-3f,
                $"proximal joint pulse {lastProximalPulse} fell below the configured floor of 400");
        }

        [Fact]
        public void CommandJointAngles_AndCommand_TrackStalenessIndependently()
        {
            // Regression test for a real bug found via real-hardware testing (2026-08-12):
            // CommandJointAngles and Command originally shared one _lastAcceptedCaptureTicks
            // tracker, so a real caller stamping both channels with the identical CaptureTicks in
            // the same tick would have roughly half its joint commands silently rejected as
            // "stale" against its own Cartesian command from the same instant. Each entry point
            // now tracks its own staleness independently.
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            plant.Command(Frame(captureTicks: 100, position: ReachableTarget));
            int afterCartesianCommand = relay.SentCommands.Count;

            // Same CaptureTicks as the Cartesian command above -- would have been rejected as
            // stale/duplicate under the old shared-tracker behavior.
            plant.CommandJointAngles(JointAnglesFor(JetRover, 0.3f, 0.2f, -0.1f, 0f), captureTicks: 100);

            Assert.True(
                relay.SentCommands.Count > afterCartesianCommand,
                "CommandJointAngles was rejected due to the Cartesian command's identical CaptureTicks -- the two channels must track staleness independently.");
        }

        [Fact]
        public void UnknownMotorId_InCommandJointAngles_IsIgnoredNotThrown()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);

            var targetsWithAForeignId = new[]
            {
                new JointTarget(BaseMotorId, 0.3f, 0f),
                new JointTarget(motorId: 200, angle: 1f, speed: 0f), // no such joint on this profile
            };

            plant.CommandJointAngles(targetsWithAForeignId, captureTicks: 10);

            Assert.Single(relay.SentCommands);
            Assert.DoesNotContain(relay.SentCommands[0], t => t.MotorId == 200);
        }

        // A second, structurally different profile (no rotating base, different link lengths, no
        // wrist joint, no gripper) proves the generalization actually generalizes, not just that
        // JetRover's own numbers still work.

        private const byte SimpleProximalMotorId = 10, SimpleDistalMotorId = 11;

        private static readonly RobotArmProfile SimpleProfile = new RobotArmProfile(
            name: "fixed-base-no-wrist-no-gripper", hasRotatingBase: false, baseHeight: 0.02f,
            proximalLinkLength: 0.05f, distalLinkLength: 0.09f, wristJointCount: 0,
            hasGripper: false, gripperCanRotate: false,
            joints: new[]
            {
                new JointHardwareSpec(SimpleProximalMotorId, JointRole.Proximal),
                new JointHardwareSpec(SimpleDistalMotorId, JointRole.Distal),
            });

        private static GenericArmPlantConfig SimpleConfig => new GenericArmPlantConfig(
            SimpleProfile, Config.PulsePerRadian, Config.PulsesPerSecond, Config.StepSizePulses,
            Config.MaxDirectionMagnitude, Config.ZeroPulse, Config.MinPulse, Config.MaxPulse,
            Config.GripperOpenPulse, Config.GripperClosedPulse);

        [Fact]
        public void SimpleProfile_CommandProducesExactlyTwoJointTargets_NoBaseYawNoGripper()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(SimpleConfig, relay);

            plant.Command(Frame(captureTicks: 10, position: new Vector3(0.10f, 0f, 0.06f)));

            Assert.Single(relay.SentCommands);
            JointTarget[] sent = relay.SentCommands[0];
            Assert.Equal(2, sent.Length);
            Assert.Contains(sent, t => t.MotorId == SimpleProximalMotorId);
            Assert.Contains(sent, t => t.MotorId == SimpleDistalMotorId);
        }

        [Fact]
        public void SimpleProfile_IsFullySensed_TrueAfterBothJointsSensed_NoGripperToWaitFor()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(SimpleConfig, relay);
            relay.EnqueueFeedback(
                new JointFeedbackEntry(SimpleProximalMotorId, true, Config.ZeroPulse),
                new JointFeedbackEntry(SimpleDistalMotorId, true, Config.ZeroPulse));

            plant.Step(nowTicks: 1000);

            Assert.True(plant.IsFullySensed);
        }

        [Fact]
        public void SimpleProfile_RejectsStaleOrDuplicateFramesWhole()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(SimpleConfig, relay);

            plant.Command(Frame(captureTicks: 100, position: new Vector3(0.10f, 0f, 0.06f)));
            int afterFirst = relay.SentCommands.Count;

            plant.Command(Frame(captureTicks: 50, position: new Vector3(0.08f, 0f, 0.05f))); // stale

            Assert.Equal(afterFirst, relay.SentCommands.Count);
        }
    }
}
