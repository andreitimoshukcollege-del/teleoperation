using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotHost.Kinematics;
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
        public void Command_SendsANonZeroDirectionTowardTheIkTarget()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            plant.Command(Frame(captureTicks: 10, position: ReachableTarget));

            Assert.Single(relay.SentCommands);
            LocalArmCommand sent = relay.SentCommands[0];
            // Starting from the zero-pulse (arm-centered) belief, any reachable off-center
            // target should produce a real move on at least one joint.
            bool anyJointMoved =
                sent.BaseDirection != 0f || sent.LowerDirection != 0f || sent.MiddleDirection != 0f || sent.UpperDirection != 0f;
            Assert.True(anyJointMoved);
        }

        [Fact]
        public void Command_RepeatingTheSameTarget_GraduallyConvergesToZeroDelta()
        {
            // Regression test for a real bug: an earlier version updated this plant's belief to
            // the full, unclamped IK target after every Command() call, even when the direction
            // actually sent had been clamped to a smaller value -- silently stalling out any move
            // whose single-step delta exceeded MaxDirectionMagnitude forever, since a repeated
            // CommandFrame toward the same target would then compute a zero delta against a
            // belief that was never actually reached. The belief must instead accumulate only the
            // clamped, actually-applied delta each call, so repeating the same target keeps
            // closing the remaining distance over successive calls.
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            for (int i = 0; i < 20; i++)
            {
                plant.Command(Frame(captureTicks: 10 + i, position: ReachableTarget));
            }

            LocalArmCommand last = relay.SentCommands[relay.SentCommands.Count - 1];
            Assert.True(MathF.Abs(last.BaseDirection) < 0.01f, $"base still moving: {last.BaseDirection}");
            Assert.True(MathF.Abs(last.LowerDirection) < 0.01f, $"lower still moving: {last.LowerDirection}");
            Assert.True(MathF.Abs(last.MiddleDirection) < 0.01f, $"middle still moving: {last.MiddleDirection}");
            Assert.True(MathF.Abs(last.UpperDirection) < 0.01f, $"upper still moving: {last.UpperDirection}");
        }

        [Fact]
        public void Command_ClampsEachDirectionToConfiguredMaximum()
        {
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(Config, relay);

            // Full extension straight up is about as far from the zero-pulse start as this arm
            // can be commanded in one step -- expect at least one axis to hit the clamp.
            var extremeTarget = new Vector3(0f, 0f, Config.Links.Base + Config.Links.Lower + Config.Links.Middle - 0.001f);

            plant.Command(Frame(captureTicks: 10, position: extremeTarget));

            LocalArmCommand sent = relay.SentCommands[0];
            Assert.True(MathF.Abs(sent.BaseDirection) <= Config.MaxDirectionMagnitude + 1e-4f);
            Assert.True(MathF.Abs(sent.LowerDirection) <= Config.MaxDirectionMagnitude + 1e-4f);
            Assert.True(MathF.Abs(sent.MiddleDirection) <= Config.MaxDirectionMagnitude + 1e-4f);
            Assert.True(MathF.Abs(sent.UpperDirection) <= Config.MaxDirectionMagnitude + 1e-4f);
        }

        [Fact]
        public void Command_LowerArmNeverExceedsConfiguredLowerArmMaxPulse()
        {
            // Regression test for a real incident (2026-08-08): the lower arm collided with the
            // robot's own base plate at a real physical target -- LowerArmMaxPulse exists
            // specifically to make that unreachable in software, independent of what the IK
            // target asks for. This target's unclamped IK would put the lower arm at pulse
            // ~748 (well above center); the configured cap of 600 must win regardless of how
            // many repeated commands try to keep pushing past it.
            var configWithLowerLimit = new JetRoverPlantConfig(
                links: Config.Links, pulsePerRadian: Config.PulsePerRadian,
                pulsePerDegreeAssumed180: Config.PulsePerDegreeAssumed180, stepSizePulses: Config.StepSizePulses,
                maxDirectionMagnitude: Config.MaxDirectionMagnitude, zeroPulse: Config.ZeroPulse,
                minPulse: Config.MinPulse, maxPulse: Config.MaxPulse,
                gripperOpenDegrees: Config.GripperOpenDegrees, gripperClosedDegrees: Config.GripperClosedDegrees,
                lowerArmMaxPulse: 600);
            var relay = new FakeRelayClient();
            var plant = new JetRoverPlant(configWithLowerLimit, relay);

            // Straight up: pushes the unclamped lower-arm IK target well past the configured cap.
            var straightUpTarget = new Vector3(
                0f, 0f, Config.Links.Base + Config.Links.Lower + Config.Links.Middle - 0.001f);

            float cumulativeLowerPulseDelta = 0f;
            for (int i = 0; i < 20; i++)
            {
                plant.Command(Frame(captureTicks: 10 + i, position: straightUpTarget));
                cumulativeLowerPulseDelta += relay.SentCommands[relay.SentCommands.Count - 1].LowerDirection
                    * configWithLowerLimit.StepSizePulses;
            }

            float impliedLowerPulse = configWithLowerLimit.ZeroPulse + cumulativeLowerPulseDelta;
            Assert.True(
                impliedLowerPulse <= configWithLowerLimit.LowerArmMaxPulse + 1e-3f,
                $"lower arm pulse {impliedLowerPulse} exceeded the configured cap of {configWithLowerLimit.LowerArmMaxPulse}");

            LocalArmCommand last = relay.SentCommands[relay.SentCommands.Count - 1];
            Assert.True(
                MathF.Abs(last.LowerDirection) < 0.01f,
                $"lower arm still moving after 20 calls, should have converged at the cap: {last.LowerDirection}");
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
            // correction). Computing the first *relative* delta from an unseeded ZeroPulse belief
            // instead of the real sensed position sends a step sized for the wrong starting
            // point, which the real servo then applies on top of its own true position --
            // overshooting by the belief/reality gap in either direction. Verified here by
            // comparing two otherwise-identical plants, one that observed sensed feedback before
            // its first Command() and one that didn't: they must diverge for lower (whose sensed
            // value differs from ZeroPulse here), and the seeded one must match hand-derived IK
            // computed directly against the real sensed starting point, not ZeroPulse.
            var seededRelay = new FakeRelayClient();
            var seededPlant = new JetRoverPlant(Config, seededRelay);
            seededRelay.EnqueueFeedback(new LocalFeedback(
                @base: new JointFeedback(true, 90), lower: new JointFeedback(true, 70),
                middle: new JointFeedback(false, 0), upper: new JointFeedback(true, 90)));
            seededPlant.Step(nowTicks: 1000);

            var unseededRelay = new FakeRelayClient();
            var unseededPlant = new JetRoverPlant(Config, unseededRelay); // never Step()'d -- belief stays ZeroPulse

            seededPlant.Command(Frame(captureTicks: 2000, position: ReachableTarget));
            unseededPlant.Command(Frame(captureTicks: 2000, position: ReachableTarget));

            LocalArmCommand seededSent = seededRelay.SentCommands[0];
            LocalArmCommand unseededSent = unseededRelay.SentCommands[0];

            Assert.NotEqual(unseededSent.LowerDirection, seededSent.LowerDirection);

            // Hand-derive the expected direction directly against the real sensed starting
            // point, exactly as production code does, rather than hardcoding a precomputed
            // magic number.
            FourDofArmKinematics.TryInverse(
                Config.Links, ReachableTarget, out _, out float lowerPitch, out _);
            float targetPulseLower = Config.ZeroPulse + lowerPitch * Config.PulsePerRadian;
            float sensedPulseLower = 70 * Config.PulsePerDegreeAssumed180;
            float expectedLowerDirection = System.Math.Clamp(
                (targetPulseLower - sensedPulseLower) / Config.StepSizePulses,
                -Config.MaxDirectionMagnitude, Config.MaxDirectionMagnitude);

            Assert.Equal(expectedLowerDirection, seededSent.LowerDirection, 3);
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
    }
}
