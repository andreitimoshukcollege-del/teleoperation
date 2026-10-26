using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Power;

namespace Teleop.RobotHost.Tests.Power
{
    public class ArmPowerControllerTests
    {
        private const long Tps = 1_000_000_000;
        private const float Deg = MathF.PI / 180f;

        /// <summary>
        /// The rest pose the user placed the arm in the second time on 2026-10-26 and measured with just
        /// joint-probe (motors 1-5, degrees, the profile's zero offsets applied), with the wrist pulled in
        /// from -114.0 to -112.0, inside its range: the justfile's deployed default.
        /// </summary>
        private static readonly double[] MeasuredRestDeg = { -1.20, 91.16, -78.48, -112.0, 5.76 };

        /// <summary>That placing as read, wrist -114.0: 1.3 degrees past the wrist's measured range.</summary>
        private static readonly double[] PlacedRestDeg = { -1.20, 91.16, -78.48, -114.0, 5.76 };

        /// <summary>The first placing, 2026-10-26: the gripper lies on the body (Body.7) by the model.</summary>
        private static readonly double[] BodyContactRestDeg = { -0.72, 92.12, -93.12, -95.28, 5.76 };

        /// <summary>A rest pose the model calls clear, with a clear straight path from home: most tests park here.</summary>
        private static readonly double[] ClearRestDeg = { 0.0, 80.0, -100.0, -40.0, 0.0 };

        /// <summary>The model's home pose, as the arm joints in profile order (motors 1, 2, 3, 4, 5).</summary>
        private static float[] Home()
        {
            ArmPose h = JetRoverCollisionModel.Instance.Home;
            return new[] { h.BaseYaw, h.Proximal, h.Distal, h.Wrist, h.Roll };
        }

        private static long T(double seconds) => (long)Math.Round(seconds * Tps);

        /// <summary>A simulated arm: each joint moves toward its last target at the commanded speed, and is read every step.</summary>
        private sealed class SimArm : IArmPowerPort
        {
            public readonly float[] Angles;
            public readonly long[] Sensed = Enumerable.Repeat(long.MinValue, 5).ToArray();
            public float[] Target;
            public float Speed;
            public bool Silent, Stuck;
            public readonly List<(bool On, long T)> Torque = new List<(bool, long)>();
            public readonly List<(ArmPowerState State, ArmPowerReason Reason, uint Echo, float Remaining, long T)> Statuses =
                new List<(ArmPowerState, ArmPowerReason, uint, float, long)>();
            public readonly List<(ArmPowerState From, ArmPowerState To, ArmPowerReason Reason, string Detail)> Changes =
                new List<(ArmPowerState, ArmPowerState, ArmPowerReason, string)>();
            public readonly List<JointTarget[]> Commands = new List<JointTarget[]>();
            public float FastestStepRad;
            private long _last = long.MinValue;

            public SimArm(float[] start)
            {
                Angles = (float[])start.Clone();
                Target = (float[])start.Clone();
            }

            public void Advance(long now)
            {
                float dt = _last == long.MinValue ? 0f : (now - _last) / (float)Tps;
                _last = now;
                if (!Stuck)
                {
                    for (int i = 0; i < Angles.Length; i++)
                    {
                        float step = Math.Clamp(Target[i] - Angles[i], -Speed * dt, Speed * dt);
                        Angles[i] += step;
                    }
                }

                if (!Silent) for (int i = 0; i < Sensed.Length; i++) Sensed[i] = now;
            }

            public void ReadArm(Span<float> angles, Span<long> sensedTicks)
            {
                Angles.CopyTo(angles);
                Sensed.CopyTo(sensedTicks);
            }

            public JointCommandOutcome CommandArm(ReadOnlySpan<JointTarget> targets, long now)
            {
                JointTarget[] copy = targets.ToArray();
                if (Commands.Count > 0)
                {
                    JointTarget[] previous = Commands[^1];
                    for (int i = 0; i < copy.Length; i++) FastestStepRad = MathF.Max(FastestStepRad, MathF.Abs(copy[i].Angle - previous[i].Angle));
                }

                Commands.Add(copy);
                for (int i = 0; i < copy.Length; i++) Target[i] = copy[i].Angle;
                Speed = copy[0].Speed;
                return JointCommandOutcome.Accepted;
            }

            public void SetTorque(bool on, long now) => Torque.Add((on, now));

            public void SendStatus(ArmPowerState state, ArmPowerReason reason, uint echoSequence, long echoOperatorTicks, float remainingDeg, long now) =>
                Statuses.Add((state, reason, echoSequence, remainingDeg, now));

            public void StateChanged(long now, ArmPowerState from, ArmPowerState to, ArmPowerReason reason, string detail) =>
                Changes.Add((from, to, reason, detail));
        }

        private sealed class Rig
        {
            public readonly SimArm Arm;
            public readonly ArmPowerController Power;
            public long Now;

            public Rig(float[]? start = null, double[]? restDeg = null, ArmPowerOptions? options = null, RobotArmProfile? profile = null)
            {
                Arm = new SimArm(start ?? Home());
                options ??= new ArmPowerOptions();
                if (options.RestPoseDegrees == null) options.RestPoseDegrees = restDeg ?? ClearRestDeg;
                Power = new ArmPowerController(profile ?? RobotArmProfile.JetRoverMeasuredDefault, options, Tps, Arm);
                Run(0.1);
            }

            /// <summary>The main loop, every 5 ms.</summary>
            public void Run(double seconds)
            {
                long end = Now + T(seconds);
                while (Now < end)
                {
                    Now += T(0.005);
                    Arm.Advance(Now);
                    Power.Tick(Now);
                }
            }

            public void Request(ArmPowerRequest request, uint sequence) => Power.OnRequest(request, sequence, 1000 + sequence, Now);
        }

        [Fact]
        public void TheMeasuredRestPose_IsInsideTheJointLimits_AndClearByTheModel()
        {
            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            ArmCollisionModel model = profile.Collision!;
            ArmPose rest = Pose(MeasuredRestDeg);
            Assert.True(model.BaseYaw.Contains(rest.BaseYaw) && model.Proximal.Contains(rest.Proximal) && model.Distal.Contains(rest.Distal) &&
                model.Wrist.Contains(rest.Wrist) && model.Roll.Contains(rest.Roll));
            Assert.False(ArmCollision.Check(profile, model, rest).IsHit);
        }

        [Fact]
        public void TheMeasuredRestPose_ParksFromHome_WithTheStrictCheck()
        {
            var rig = new Rig(restDeg: MeasuredRestDeg);
            Assert.Null(rig.Power.RestPoseNote);
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.Equal(ArmPowerState.Parking, rig.Power.State);
            rig.Run(30);
            Assert.Equal(ArmPowerState.Relaxed, rig.Power.State);
            // The furthest joint is the wrist, 142 degrees: about 5 s at 0.5 rad/s, plus the settle.
            Assert.InRange(rig.Arm.Torque.Single().T / (double)Tps, 4.5, 7.0);
            for (int i = 0; i < 5; i++) Assert.InRange(rig.Arm.Angles[i] - (float)MeasuredRestDeg[i] * Deg, -3f * Deg, 3f * Deg);
        }

        [Fact]
        public void ThePoseAsPlaced_IsRefused_TheWristIsOutsideItsRange()
        {
            var rig = new Rig(restDeg: PlacedRestDeg);
            Assert.Contains("joint wrist outside its range", rig.Power.RestPoseNote);
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.Equal(ArmPowerState.Holding, rig.Power.State);
            Assert.Equal(ArmPowerReason.PathCollides, rig.Arm.Statuses[^1].Reason);
            Assert.Empty(rig.Arm.Commands);
        }

        private static ArmPose Pose(double[] d) =>
            new ArmPose((float)d[0] * Deg, (float)d[1] * Deg, (float)d[2] * Deg, (float)d[3] * Deg, (float)d[4] * Deg);

        [Fact]
        public void ABodyContactRestPose_IsInsideTheJointLimits_ButTheModelHasTheGripperOnTheBody()
        {
            // Asked for: inside the joint limits and collision-free. It is inside the limits. It is not
            // collision-free by the model: the gripper lies on the body (Body.7) within the model's 1 cm
            // margin, which is what supports it there. Nothing else touches.
            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            ArmCollisionModel model = profile.Collision!;
            var rest = new ArmPose((float)BodyContactRestDeg[0] * Deg, (float)BodyContactRestDeg[1] * Deg, (float)BodyContactRestDeg[2] * Deg,
                (float)BodyContactRestDeg[3] * Deg, (float)BodyContactRestDeg[4] * Deg);
            Assert.True(model.BaseYaw.Contains(rest.BaseYaw) && model.Proximal.Contains(rest.Proximal) && model.Distal.Contains(rest.Distal) &&
                model.Wrist.Contains(rest.Wrist) && model.Roll.Contains(rest.Roll));
            CollisionHit hit = ArmCollision.Check(profile, model, rest);
            Assert.Equal(CollisionHitKind.BodyBox, hit.Kind);
            Assert.Equal(ArmLink.Gripper, model.Bodies[hit.A].Link);
            Assert.Equal("gripper.1 against Body.7", hit.Describe(model));
        }

        [Fact]
        public void ABodyContactRestPose_IsRefused_ByTheStrictCheck()
        {
            var rig = new Rig(restDeg: BodyContactRestDeg);
            Assert.Contains("every relax will be refused", rig.Power.RestPoseNote);
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.Equal(ArmPowerState.Holding, rig.Power.State);
            Assert.Equal(ArmPowerReason.PathCollides, rig.Arm.Statuses[^1].Reason);
            Assert.Contains("gripper.1 against Body.7", rig.Arm.Changes[^1].Detail);
            Assert.Empty(rig.Arm.Commands);
        }

        [Fact]
        public void ABodyContactRestPose_ParksFromHome_WhenItsOwnContactIsAllowedNearTheEnd()
        {
            // --park-contact-deg 10: within 10 degrees of the rest pose the gripper may touch what it rests
            // on (Body.7); every sample further away is still strict.
            var rig = new Rig(restDeg: BodyContactRestDeg, options: new ArmPowerOptions { RestContactDegrees = 10 });
            Assert.Contains("Gripper on Body.7", rig.Power.RestPoseNote);
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.Equal(ArmPowerState.Parking, rig.Power.State);
            rig.Run(30);
            Assert.Equal(ArmPowerState.Relaxed, rig.Power.State);
            // The furthest joint is the wrist, 125 degrees: about 4.4 s at 0.5 rad/s, plus the settle.
            Assert.InRange(rig.Arm.Torque.Single().T / (double)Tps, 4.0, 6.0);
            for (int i = 0; i < 5; i++) Assert.InRange(rig.Arm.Angles[i] - (float)BodyContactRestDeg[i] * Deg, -3f * Deg, 3f * Deg);
        }

        [Fact]
        public void TheContactAllowance_AllowsOnlyTheRestPosesOwnSupport()
        {
            // A rest pose that collides with another link or the floor is not explained by a support.
            var rig = new Rig(restDeg: new[] { 0.0, -25.0, -110.0, 0.0, 0.0 }, options: new ArmPowerOptions { RestContactDegrees = 45 });
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.Equal(ArmPowerReason.PathCollides, rig.Arm.Statuses[^1].Reason);
        }

        [Fact]
        public void Relax_ParksSlowly_WaitsForTheReadings_ThenSwitchesTorqueOff()
        {
            var rig = new Rig();
            rig.Request(ArmPowerRequest.Relax, 7);
            Assert.Equal(ArmPowerState.Parking, rig.Power.State);
            Assert.Equal((ArmPowerState.Parking, 7u), (rig.Arm.Statuses[^1].State, rig.Arm.Statuses[^1].Echo));
            Assert.Empty(rig.Arm.Torque);

            rig.Run(30);
            Assert.Equal(ArmPowerState.Relaxed, rig.Power.State);
            Assert.Equal(new[] { false }, rig.Arm.Torque.Select(t => t.On));
            // Only the arm joints, at the park speed, the gripper never.
            Assert.All(rig.Arm.Commands, c => Assert.Equal(new byte[] { 1, 2, 3, 4, 5 }, c.Select(j => j.MotorId)));
            Assert.All(rig.Arm.Commands, c => Assert.All(c, j => Assert.Equal(0.5f, j.Speed)));
            // Waypoints 50 ms apart at 0.5 rad/s: no joint's target jumps more than 0.025 rad (plus a loop step).
            Assert.InRange(rig.Arm.FastestStepRad, 0f, 0.5f * 0.06f);
            for (int i = 0; i < 5; i++) Assert.InRange(rig.Arm.Angles[i] - (float)ClearRestDeg[i] * Deg, -3f * Deg, 3f * Deg);

            // The furthest joints move 70 degrees: about 2.4 s at 0.5 rad/s, plus the settle.
            long torqueOff = rig.Arm.Torque[0].T;
            Assert.InRange(torqueOff / (double)Tps, 2.4, 3.6);
            // Nothing was commanded while it settled (0.3 s before the torque went off).
            Assert.Equal(ArmPowerState.Relaxed, rig.Arm.Statuses[^1].State);
        }

        [Fact]
        public void OperatorCommands_AreIgnored_WhileParkingAndRelaxed_AndPassAgainAfterHold()
        {
            var rig = new Rig();
            Assert.True(rig.Power.AcceptOperatorCommand(rig.Now));
            rig.Arm.Torque.Clear();
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.False(rig.Power.AcceptOperatorCommand(rig.Now));
            rig.Run(30);
            Assert.False(rig.Power.AcceptOperatorCommand(rig.Now));
            Assert.Equal(2, rig.Power.IgnoredCommands);

            rig.Request(ArmPowerRequest.Hold, 2);
            Assert.Equal(ArmPowerState.Holding, rig.Power.State);
            Assert.Equal(new[] { false, true }, rig.Arm.Torque.Select(t => t.On));
            Assert.Equal((ArmPowerState.Holding, ArmPowerReason.None, 2u), (rig.Arm.Statuses[^1].State, rig.Arm.Statuses[^1].Reason, rig.Arm.Statuses[^1].Echo));
            Assert.True(rig.Power.AcceptOperatorCommand(rig.Now));
            Assert.Equal(2, rig.Arm.Torque.Count); // torque was already on: nothing more
        }

        [Fact]
        public void TheFirstOperatorCommandAfterStartUp_SwitchesTorqueOn_Once()
        {
            // A restarted RobotHost cannot know the arm is not relaxed.
            var rig = new Rig();
            Assert.True(rig.Power.AcceptOperatorCommand(rig.Now));
            Assert.True(rig.Power.AcceptOperatorCommand(rig.Now));
            Assert.Equal(new[] { true }, rig.Arm.Torque.Select(t => t.On));
        }

        [Fact]
        public void ARelaxWhileRelaxed_IsAnsweredRelaxed_AndARelaxWhileParking_Parking()
        {
            var rig = new Rig();
            rig.Request(ArmPowerRequest.Relax, 1);
            rig.Run(1);
            int commands = rig.Arm.Commands.Count;
            rig.Request(ArmPowerRequest.Relax, 2);
            Assert.Equal(ArmPowerState.Parking, rig.Arm.Statuses[^1].State);
            rig.Run(30);
            Assert.True(rig.Arm.Commands.Count > commands); // the park went on, not restarted
            rig.Request(ArmPowerRequest.Relax, 3);
            Assert.Equal((ArmPowerState.Relaxed, 3u), (rig.Arm.Statuses[^1].State, rig.Arm.Statuses[^1].Echo));
            Assert.Single(rig.Arm.Torque);
        }

        [Fact]
        public void AlreadyAtTheRestPose_TheTorqueGoesOffAtOnce()
        {
            float[] rest = ClearRestDeg.Select(d => (float)d * Deg + 1f * Deg).ToArray();
            var rig = new Rig(start: rest);
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.Equal(ArmPowerState.Relaxed, rig.Power.State);
            Assert.Empty(rig.Arm.Commands);
            Assert.Equal(new[] { false }, rig.Arm.Torque.Select(t => t.On));
        }

        [Fact]
        public void WithoutARestPose_EveryRelaxIsRefused()
        {
            var arm = new SimArm(Home());
            var power = new ArmPowerController(RobotArmProfile.JetRoverMeasuredDefault, new ArmPowerOptions(), Tps, arm);
            arm.Advance(T(1));
            power.OnRequest(ArmPowerRequest.Relax, 4, 0, T(1));
            Assert.Equal(ArmPowerState.Holding, power.State);
            Assert.Equal((ArmPowerState.Holding, ArmPowerReason.NoRestPose, 4u), (arm.Statuses[^1].State, arm.Statuses[^1].Reason, arm.Statuses[^1].Echo));
            Assert.True(float.IsNaN(arm.Statuses[^1].Remaining));
            Assert.Empty(arm.Torque);
            Assert.Empty(arm.Commands);
        }

        [Fact]
        public void APathThatCollides_IsRefused_AndNothingMoves()
        {
            // A rest pose through the deck: the elbow folded down with the shoulder low.
            var rig = new Rig(restDeg: new[] { 0.0, -25.0, -110.0, 0.0, 0.0 });
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.Equal(ArmPowerState.Holding, rig.Power.State);
            Assert.Equal(ArmPowerReason.PathCollides, rig.Arm.Statuses[^1].Reason);
            Assert.Empty(rig.Arm.Commands);
            Assert.Empty(rig.Arm.Torque);
            Assert.Contains("of the way", rig.Arm.Changes[^1].Detail);
        }

        [Fact]
        public void NoFreshReading_RefusesWithBoardNotAnswering()
        {
            var rig = new Rig();
            rig.Arm.Silent = true;
            rig.Run(1);
            rig.Request(ArmPowerRequest.Relax, 1);
            Assert.Equal(ArmPowerReason.BoardNotAnswering, rig.Arm.Statuses[^1].Reason);
            Assert.Empty(rig.Arm.Commands);
        }

        [Fact]
        public void TheBoardGoingSilentWhileParking_EndsInHolding()
        {
            var rig = new Rig();
            rig.Request(ArmPowerRequest.Relax, 1);
            rig.Run(1);
            rig.Arm.Silent = true;
            rig.Run(3);
            Assert.Equal(ArmPowerState.Holding, rig.Power.State);
            Assert.Equal(ArmPowerReason.BoardNotAnswering, rig.Power.LastReason);
            Assert.Empty(rig.Arm.Torque);
        }

        [Fact]
        public void AParkThatNeverArrives_TimesOut_HoldingWithTorqueOn()
        {
            var rig = new Rig(options: new ArmPowerOptions { ParkTimeoutSeconds = 8 });
            rig.Arm.Stuck = true;
            rig.Request(ArmPowerRequest.Relax, 1);
            rig.Run(7.9);
            Assert.Equal(ArmPowerState.Parking, rig.Power.State);
            rig.Run(0.2);
            Assert.Equal(ArmPowerState.Holding, rig.Power.State);
            Assert.Equal(ArmPowerReason.ParkTimedOut, rig.Arm.Statuses[^1].Reason);
            Assert.Empty(rig.Arm.Torque);
            Assert.True(rig.Power.AcceptOperatorCommand(rig.Now));
        }

        [Fact]
        public void AHoldWhileParking_StopsThePark()
        {
            var rig = new Rig();
            rig.Request(ArmPowerRequest.Relax, 1);
            rig.Run(1);
            int commands = rig.Arm.Commands.Count;
            rig.Request(ArmPowerRequest.Hold, 2);
            Assert.Equal(ArmPowerState.Holding, rig.Power.State);
            Assert.Equal(ArmPowerReason.HoldRequested, rig.Arm.Statuses[^1].Reason);
            rig.Run(10);
            Assert.Equal(commands, rig.Arm.Commands.Count);
            Assert.Empty(rig.Arm.Torque.Where(t => !t.On));
        }

        [Fact]
        public void WhileNotHolding_TheStatusRepeatsOnceASecond_AndAHeartbeatIsAnswered()
        {
            var rig = new Rig();
            rig.Request(ArmPowerRequest.Relax, 1);
            rig.Run(30);
            int before = rig.Arm.Statuses.Count;
            rig.Run(5);
            Assert.InRange(rig.Arm.Statuses.Count - before, 4, 6);
            Assert.All(rig.Arm.Statuses.Skip(before), s => Assert.Equal(ArmPowerState.Relaxed, s.State));
            Assert.InRange(rig.Arm.Statuses[^1].Remaining, 0f, 3f);

            rig.Request(ArmPowerRequest.None, 9);
            Assert.Equal((ArmPowerState.Relaxed, ArmPowerReason.None, 9u), (rig.Arm.Statuses[^1].State, rig.Arm.Statuses[^1].Reason, rig.Arm.Statuses[^1].Echo));
            Assert.Equal(ArmPowerState.Relaxed, rig.Power.State);

            // Holding: no repeats, but a heartbeat is still answered.
            rig.Request(ArmPowerRequest.Hold, 10);
            before = rig.Arm.Statuses.Count;
            rig.Run(5);
            Assert.Equal(before, rig.Arm.Statuses.Count);
            rig.Request(ArmPowerRequest.None, 11);
            Assert.Equal(ArmPowerState.Holding, rig.Arm.Statuses[^1].State);
        }

        [Fact]
        public void AnUnknownRequest_IsAnsweredWithTheStateAndAReason()
        {
            var rig = new Rig();
            rig.Request((ArmPowerRequest)9, 3);
            Assert.Equal((ArmPowerState.Holding, ArmPowerReason.UnknownRequest), (rig.Arm.Statuses[^1].State, rig.Arm.Statuses[^1].Reason));
        }

        [Fact]
        public void ARestPoseOfTheWrongLength_IsRefusedAtStart()
        {
            var options = new ArmPowerOptions { RestPoseDegrees = new[] { 0.0, 90.0, -90.0, -90.0 } };
            var e = Assert.Throws<ArgumentException>(() =>
                new ArmPowerController(RobotArmProfile.JetRoverMeasuredDefault, options, Tps, new SimArm(Home())));
            Assert.Contains("motors 1, 2, 3, 4, 5", e.Message);
        }

        [Fact]
        public void TheRestPoseParser()
        {
            Assert.True(ArmPowerOptions.TryParseRestPose("-1.20,91.16, -78.48,-112.0,5.76", out double[]? d, out _));
            Assert.Equal(MeasuredRestDeg, d);
            Assert.True(ArmPowerOptions.TryParseRestPose("", out d, out _));
            Assert.Null(d);
            Assert.True(ArmPowerOptions.TryParseRestPose("  ", out d, out _));
            Assert.Null(d);
            Assert.False(ArmPowerOptions.TryParseRestPose("1,x,3", out _, out string? error));
            Assert.Contains("'x'", error);
        }
    }
}
