using System.Numerics;
using Teleop.Core.Types;
using Teleop.JetRover.Kinematics;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Plant
{
    public class JetRoverPlantTests
    {
        private static readonly JetRoverPlantConfig Config = JetRoverPlantConfig.Default;

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

        [Fact]
        public void Command_SendsANonZeroPulseTargetTowardTheIkTarget()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            plant.Command(Frame(captureTicks: 10, position: ReachableTarget));

            Assert.Single(relay.SentCommands);
            LocalArmCommand sent = relay.SentCommands[0];
            // Starting from the zero-pulse (arm-centered) belief, any reachable off-center
            // target should produce a real move on at least one joint -- the sent value is now
            // the resulting absolute pulse (docs/adr/0010), so "moved" means "differs from the
            // starting ZeroPulse belief," not "direction is nonzero."
            bool anyJointMoved =
                sent.BasePulse != Config.ZeroPulse || sent.LowerPulse != Config.ZeroPulse ||
                sent.MiddlePulse != Config.ZeroPulse || sent.UpperPulse != Config.ZeroPulse;
            Assert.True(anyJointMoved);
        }

        [Fact]
        public void Command_RepeatingTheSameTarget_GraduallyConvergesToAStableAbsoluteTarget()
        {
            // Regression test for a real bug: an earlier version updated this plant's belief to
            // the full, unclamped IK target after every Command() call, even when the amount
            // actually applied had been clamped to a smaller step -- silently stalling out any
            // move whose single-step delta exceeded MaxDirectionMagnitude forever, since a
            // repeated CommandFrame toward the same target would then compute a zero delta
            // against a belief that was never actually reached. The belief must instead
            // accumulate only the clamped, actually-applied delta each call, so repeating the
            // same target keeps closing the remaining distance over successive calls until the
            // sent absolute pulse stops changing between calls (converged).
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            for (int i = 0; i < 20; i++)
            {
                plant.Command(Frame(captureTicks: 10 + i, position: ReachableTarget));
            }

            LocalArmCommand secondToLast = relay.SentCommands[relay.SentCommands.Count - 2];
            LocalArmCommand last = relay.SentCommands[relay.SentCommands.Count - 1];
            Assert.True(MathF.Abs(last.BasePulse - secondToLast.BasePulse) < 0.5f, $"base still moving: {last.BasePulse} vs {secondToLast.BasePulse}");
            Assert.True(MathF.Abs(last.LowerPulse - secondToLast.LowerPulse) < 0.5f, $"lower still moving: {last.LowerPulse} vs {secondToLast.LowerPulse}");
            Assert.True(MathF.Abs(last.MiddlePulse - secondToLast.MiddlePulse) < 0.5f, $"middle still moving: {last.MiddlePulse} vs {secondToLast.MiddlePulse}");
            Assert.True(MathF.Abs(last.UpperPulse - secondToLast.UpperPulse) < 0.5f, $"upper still moving: {last.UpperPulse} vs {secondToLast.UpperPulse}");
        }

        [Fact]
        public void Command_ClampsEachJointsPerCallMovementToConfiguredMaximum()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            // Full extension straight up is about as far from the zero-pulse start as this arm
            // can be commanded in one step -- expect at least one axis to hit the clamp.
            var extremeTarget = new Vector3(0f, 0f, Config.Links.Base + Config.Links.Lower + Config.Links.Middle - 0.001f);

            plant.Command(Frame(captureTicks: 10, position: extremeTarget));

            // Only one call, starting from ZeroPulse, so the sent absolute pulse can differ from
            // ZeroPulse by no more than one clamped step (docs/adr/0010 -- the sent value is now
            // the resulting belief, not the direction, but the per-call movement it can represent
            // is still bounded by MaxDirectionMagnitude * StepSizePulses).
            float maxStep = Config.MaxDirectionMagnitude * Config.StepSizePulses;
            LocalArmCommand sent = relay.SentCommands[0];
            Assert.True(MathF.Abs(sent.BasePulse - Config.ZeroPulse) <= maxStep + 1e-3f);
            Assert.True(MathF.Abs(sent.LowerPulse - Config.ZeroPulse) <= maxStep + 1e-3f);
            Assert.True(MathF.Abs(sent.MiddlePulse - Config.ZeroPulse) <= maxStep + 1e-3f);
            Assert.True(MathF.Abs(sent.UpperPulse - Config.ZeroPulse) <= maxStep + 1e-3f);
        }

        [Fact]
        public void Command_LowerArmNeverGoesBelowConfiguredLowerArmMinPulse()
        {
            // Regression test for two real incidents (2026-08-08). First: the lower arm
            // collided with the robot's own base plate at a real physical target --
            // LowerArmMinPulse exists specifically to make that unreachable in software,
            // independent of what the IK target asks for. Second: this limit was originally
            // implemented as an upper bound (a maximum on the pulse value), which does nothing
            // to stop the target from going *below* it -- exactly the dangerous direction on
            // this hardware, since lower pulse values drive the lower arm toward the plate. That
            // bug was invisible throughout calibration because every test used the same target,
            // whose unclamped pulse happened to always be above the limit; a target whose
            // unclamped pulse is naturally low (like this one) exposes it immediately. This
            // target's unclamped IK puts the lower arm well below center; the configured floor
            // of 400 must win regardless of how many repeated commands try to keep pushing below
            // it.
            var configWithLowerLimit = new JetRoverPlantConfig(
                links: Config.Links, pulsePerRadian: Config.PulsePerRadian,
                pulsePerDegreeAssumed180: Config.PulsePerDegreeAssumed180, stepSizePulses: Config.StepSizePulses,
                maxDirectionMagnitude: Config.MaxDirectionMagnitude, zeroPulse: Config.ZeroPulse,
                minPulse: Config.MinPulse, maxPulse: Config.MaxPulse,
                gripperOpenDegrees: Config.GripperOpenDegrees, gripperClosedDegrees: Config.GripperClosedDegrees,
                lowerArmMinPulse: 400);
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(configWithLowerLimit, relay);

            // Straight down: pushes the unclamped lower-arm IK target well below the configured
            // floor -- the mirror image of the "straight up" target used to test the opposite
            // (upper) clamp in Command_ClampsEachJointsPerCallMovementToConfiguredMaximum.
            var straightDownTarget = new Vector3(
                0f, 0f, -(Config.Links.Base + Config.Links.Lower + Config.Links.Middle - 0.001f));

            for (int i = 0; i < 20; i++)
            {
                plant.Command(Frame(captureTicks: 10 + i, position: straightDownTarget));
            }

            // The sent value is now the resulting absolute pulse directly (docs/adr/0010), so no
            // need to accumulate a delta history to know where the belief ended up.
            LocalArmCommand secondToLast = relay.SentCommands[relay.SentCommands.Count - 2];
            LocalArmCommand last = relay.SentCommands[relay.SentCommands.Count - 1];
            Assert.True(
                last.LowerPulse >= configWithLowerLimit.LowerArmMinPulse - 1e-3f,
                $"lower arm pulse {last.LowerPulse} fell below the configured floor of {configWithLowerLimit.LowerArmMinPulse}");
            Assert.True(
                MathF.Abs(last.LowerPulse - secondToLast.LowerPulse) < 0.5f,
                $"lower arm still moving after 20 calls, should have converged at the floor: {last.LowerPulse} vs {secondToLast.LowerPulse}");
        }

        [Fact]
        public void Command_DenormalizesGripperToConfiguredDegreeRange()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            plant.Command(Frame(captureTicks: 10, position: ReachableTarget, gripper: 0f));
            plant.Command(Frame(captureTicks: 20, position: ReachableTarget, gripper: 1f));
            plant.Command(Frame(captureTicks: 30, position: ReachableTarget, gripper: 0.5f));

            Assert.Equal(Config.GripperOpenDegrees, relay.SentCommands[0].GripperDegrees, 3);
            Assert.Equal(Config.GripperClosedDegrees, relay.SentCommands[1].GripperDegrees, 3);
            Assert.Equal((Config.GripperOpenDegrees + Config.GripperClosedDegrees) / 2f, relay.SentCommands[2].GripperDegrees, 3);
        }

        [Fact]
        public void Command_RejectsStaleOrDuplicateFramesWhole()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

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
            var plant = new JetRoverPlant(Config, relay);
            relay.EnqueueFeedback(new LocalFeedback(
                @base: new JointFeedback(true, 0), lower: new JointFeedback(true, 0),
                middle: new JointFeedback(true, 0), upper: new JointFeedback(true, 0)));

            plant.Step(nowTicks: 1000);

            Assert.True(plant.IsFullySensed);
            // All-zero degrees (== zero pulse offset, arm centered) should forward-kinematic to
            // the arm's own resting reach: base yaw 0, lower/middle pitch 0 -> straight out along +X.
            Pose pose = plant.State.Value;
            Assert.Equal(1000, plant.State.CaptureTicks);
            Assert.True(pose.Position.X > 0f, $"expected a positive X reach at rest, got {pose.Position}");
        }

        [Fact]
        public void Command_FirstUseSeedsBeliefFromSensedData_NotZeroPulse()
        {
            // Regression test for a real incident (2026-08-08): Step() polls the relay for
            // feedback from startup regardless of whether any command has arrived, so by the
            // time the first real Command() lands, a joint is often already sensed away from
            // ZeroPulse (e.g. after a Teleop.RobotHost restart mid-session, or a manual physical
            // correction). Computing the first step from an unseeded ZeroPulse belief instead of
            // the real sensed position sizes that step for the wrong starting point, which the
            // real servo then applies on top of its own true position -- overshooting by the
            // belief/reality gap in either direction.
            //
            // Note (docs/adr/0010): when a move is *not* clamped, the resulting absolute target
            // converges to exactly the same value regardless of the starting belief (the starting
            // point algebraically cancels out of `belief + clamp(target - belief)`), so seeding
            // only produces an observably different *sent* value when the move needs clamping --
            // this test therefore uses the same extreme, clamp-triggering target as
            // Command_ClampsEachJointsPerCallMovementToConfiguredMaximum, not ReachableTarget, so
            // the seeded and unseeded beliefs actually diverge in what gets sent.
            var extremeTarget = new Vector3(0f, 0f, Config.Links.Base + Config.Links.Lower + Config.Links.Middle - 0.001f);

            var seededRelay = new FakeRelayClient();
            var seededPlant = new JetRoverPlant(Config, seededRelay);
            seededRelay.EnqueueFeedback(new LocalFeedback(
                @base: new JointFeedback(true, 90), lower: new JointFeedback(true, 70),
                middle: new JointFeedback(false, 0), upper: new JointFeedback(true, 90)));
            seededPlant.Step(nowTicks: 1000);

            var unseededRelay = new FakeRelayClient();
            var unseededPlant = new JetRoverPlant(Config, unseededRelay); // never Step()'d -- belief stays ZeroPulse

            seededPlant.Command(Frame(captureTicks: 2000, position: extremeTarget));
            unseededPlant.Command(Frame(captureTicks: 2000, position: extremeTarget));

            LocalArmCommand seededSent = seededRelay.SentCommands[0];
            LocalArmCommand unseededSent = unseededRelay.SentCommands[0];

            // Hand-derive the expected resulting absolute pulse directly against the real sensed
            // starting point, exactly as production code does, rather than hardcoding a
            // precomputed magic number. Confirm the move actually clamps -- otherwise this test
            // would silently stop testing what it claims to (see the note above).
            FourDofArmKinematics.TryInverse(
                Config.Links, extremeTarget, out _, out float lowerPitch, out _, out _);
            float targetPulseLower = Config.ZeroPulse + lowerPitch * Config.PulsePerRadian;
            float sensedPulseLower = 70 * Config.PulsePerDegreeAssumed180;
            float expectedLowerDirection = System.Math.Clamp(
                (targetPulseLower - sensedPulseLower) / Config.StepSizePulses,
                -Config.MaxDirectionMagnitude, Config.MaxDirectionMagnitude);
            Assert.Equal(Config.MaxDirectionMagnitude, MathF.Abs(expectedLowerDirection), 3);

            Assert.NotEqual(unseededSent.LowerPulse, seededSent.LowerPulse);

            float expectedLowerPulse = sensedPulseLower + expectedLowerDirection * Config.StepSizePulses;
            Assert.Equal(expectedLowerPulse, seededSent.LowerPulse, 3);
        }

        [Fact]
        public void State_FallsBackToLastCommandedTarget_ForAJointThatHasNeverBeenSensed()
        {
            // Real, observed hardware limitation: this arm's middle-arm servo never responds to
            // position-read requests at all (confirmed independently of ROS -- writes to it work
            // fine, only reads never succeed). A joint stuck at IsFullySensed=false forever must
            // not report State as if it were still at its power-on default while every other
            // joint shows real progress -- falling back to this plant's own last-commanded
            // target is a materially better estimate, even though it's still an estimate, not a
            // measurement (IsFullySensed is what tells a caller whether to trust it).
            //
            // No Command() call in this test, so every joint's target belief is still exactly
            // ZeroPulse (angle 0) -- middle's fallback and its "never left the default" starting
            // point are the same value here, which is why the assertion below only needs to
            // compare against a plain angle-zero forward kinematics, not re-derive a target.
            //
            // Feedback degrees are in the ROS SDK's own direct proportional 0-1000<->0-180
            // mapping (pulseToDeg), not an offset-from-center one -- degrees=90 is pulse 500
            // (this plant's kinematic zero), not degrees=0 (which is pulse 0, an extreme).
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            // Base sensed exactly at center (degrees=90 -> pulse 500 -> baseYaw=0); lower sensed
            // away from center; middle (this hardware's known-bad servo) never gets sensed.
            relay.EnqueueFeedback(new LocalFeedback(
                @base: new JointFeedback(true, 90), lower: new JointFeedback(true, 70),
                middle: new JointFeedback(false, 0), upper: new JointFeedback(true, 90)));
            plant.Step(nowTicks: 1000);

            Assert.False(plant.IsFullySensed);
            float lowerPulseFromSensedDegrees = 70 * Config.PulsePerDegreeAssumed180;
            float lowerPitchFromSensedDegrees = (lowerPulseFromSensedDegrees - Config.ZeroPulse) / Config.PulsePerRadian;
            Vector3 expectedPosition = FourDofArmKinematics.Forward(
                Config.Links, baseYaw: 0f, lowerPitch: lowerPitchFromSensedDegrees, middlePitch: 0f);
            Vector3 reportedPosition = plant.State.Value.Position;
            Assert.True(
                Vector3.Distance(expectedPosition, reportedPosition) < 1e-3f,
                $"expected fallback position near {expectedPosition}, got {reportedPosition}");
        }

        [Fact]
        public void Step_PartialFeedback_LeavesIsFullySensedFalse()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);
            relay.EnqueueFeedback(new LocalFeedback(
                @base: new JointFeedback(true, 0), lower: new JointFeedback(false, 0),
                middle: new JointFeedback(true, 0), upper: new JointFeedback(true, 0)));

            plant.Step(nowTicks: 1000);

            Assert.False(plant.IsFullySensed);
        }

        [Fact]
        public void Step_AtOrBeforeCurrentStateTime_IsNoOp()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);
            relay.EnqueueFeedback(new LocalFeedback(
                new JointFeedback(true, 10), new JointFeedback(true, 0), new JointFeedback(true, 0), new JointFeedback(true, 0)));
            plant.Step(nowTicks: 1000);
            bool sensedAfterFirstStep = plant.IsFullySensed;

            relay.EnqueueFeedback(new LocalFeedback(
                new JointFeedback(true, 999), new JointFeedback(true, 0), new JointFeedback(true, 0), new JointFeedback(true, 0)));
            plant.Step(nowTicks: 1000); // not after the current state time -- must not consume feedback

            Assert.True(sensedAfterFirstStep);
            Assert.Equal(1000, plant.State.CaptureTicks);
        }

        [Fact]
        public void Reset_ClearsBookkeeping_AcceptsNextFrameAtAnyCaptureTicks()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);
            plant.Command(Frame(captureTicks: 1000, position: ReachableTarget));
            relay.EnqueueFeedback(new LocalFeedback(
                new JointFeedback(true, 0), new JointFeedback(true, 0), new JointFeedback(true, 0), new JointFeedback(true, 0)));
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
            var plant = new JetRoverPlant(Config, relay);
            plant.Command(Frame(captureTicks: 1000, position: ReachableTarget));
            int beforeReset = relay.SentCommands.Count;

            plant.Reset();

            Assert.Equal(beforeReset, relay.SentCommands.Count); // Reset itself sent nothing
        }

        // CommandJointAngles (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md) shares
        // ApplyJointTargets's tail with Command -- these tests establish parity with the
        // equivalent Command(CommandFrame) tests above, not a full second copy of every scenario.

        [Fact]
        public void CommandJointAngles_SendsANonZeroPulseTargetTowardTheGivenAngles()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            plant.CommandJointAngles(
                baseYaw: 0.3f, lowerPitch: 0.2f, middlePitch: -0.1f, upperPitch: 0f, gripper: 0f, captureTicks: 10);

            Assert.Single(relay.SentCommands);
            LocalArmCommand sent = relay.SentCommands[0];
            bool anyJointMoved =
                sent.BasePulse != Config.ZeroPulse || sent.LowerPulse != Config.ZeroPulse ||
                sent.MiddlePulse != Config.ZeroPulse || sent.UpperPulse != Config.ZeroPulse;
            Assert.True(anyJointMoved);
        }

        [Fact]
        public void CommandJointAngles_RejectsStaleOrDuplicateCaptureTicks()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            plant.CommandJointAngles(0.3f, 0.2f, -0.1f, 0f, 0f, captureTicks: 100);
            int afterFirst = relay.SentCommands.Count;

            plant.CommandJointAngles(0.5f, 0.4f, -0.3f, 0f, 0f, captureTicks: 100); // duplicate stamp
            plant.CommandJointAngles(0.5f, 0.4f, -0.3f, 0f, 0f, captureTicks: 50);  // stale

            Assert.Equal(afterFirst, relay.SentCommands.Count);
        }

        [Fact]
        public void CommandJointAngles_LowerArmNeverGoesBelowConfiguredLowerArmMinPulse()
        {
            // Mirrors Command_LowerArmNeverGoesBelowConfiguredLowerArmMinPulse, but supplies the
            // lower-arm pitch directly instead of deriving it from a Cartesian target via IK --
            // CommandJointAngles skips TryInverse entirely, so this exercises ApplyJointTargets's
            // shared clamp/belief tail on its own.
            var configWithLowerLimit = new JetRoverPlantConfig(
                links: Config.Links, pulsePerRadian: Config.PulsePerRadian,
                pulsePerDegreeAssumed180: Config.PulsePerDegreeAssumed180, stepSizePulses: Config.StepSizePulses,
                maxDirectionMagnitude: Config.MaxDirectionMagnitude, zeroPulse: Config.ZeroPulse,
                minPulse: Config.MinPulse, maxPulse: Config.MaxPulse,
                gripperOpenDegrees: Config.GripperOpenDegrees, gripperClosedDegrees: Config.GripperClosedDegrees,
                lowerArmMinPulse: 400);
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(configWithLowerLimit, relay);

            // lowerPitch=-1.6 rad computes to an unclamped pulse well below the 400 floor
            // (ZeroPulse=500, PulsePerRadian~238.73 => ~500-383=117).
            for (int i = 0; i < 20; i++)
            {
                plant.CommandJointAngles(
                    baseYaw: 0f, lowerPitch: -1.6f, middlePitch: 0f, upperPitch: 0f, gripper: 0f, captureTicks: 10 + i);
            }

            // The sent value is now the resulting absolute pulse directly (docs/adr/0010).
            float lastLowerPulse = relay.SentCommands[relay.SentCommands.Count - 1].LowerPulse;
            Assert.True(
                lastLowerPulse >= configWithLowerLimit.LowerArmMinPulse - 1e-3f,
                $"lower arm pulse {lastLowerPulse} fell below the configured floor of {configWithLowerLimit.LowerArmMinPulse}");
        }

        [Fact]
        public void CommandJointAngles_AndCommand_TrackStalenessIndependently()
        {
            // Regression test for a real bug found via real-hardware testing (2026-08-12):
            // CommandJointAngles and Command originally shared one _lastAcceptedCaptureTicks
            // tracker, so a real caller stamping both channels with the identical CaptureTicks in
            // the same tick (JetRoverOperatorBridge does exactly this) would have roughly half its
            // joint commands silently rejected as "stale" against its own Cartesian command from
            // the same instant, even with no other caller involved at all. Each entry point now
            // tracks its own staleness independently, so a Command call must never cause a later
            // CommandJointAngles call (or vice versa) to be rejected, even at an identical or
            // earlier CaptureTicks.
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            plant.Command(Frame(captureTicks: 100, position: ReachableTarget));
            int afterCartesianCommand = relay.SentCommands.Count;

            // Same CaptureTicks as the Cartesian command above -- would have been rejected as
            // stale/duplicate under the old shared-tracker behavior.
            plant.CommandJointAngles(0.3f, 0.2f, -0.1f, 0f, 0f, captureTicks: 100);

            Assert.True(
                relay.SentCommands.Count > afterCartesianCommand,
                "CommandJointAngles was rejected due to the Cartesian command's identical CaptureTicks -- the two channels must track staleness independently.");
        }
    }
}
