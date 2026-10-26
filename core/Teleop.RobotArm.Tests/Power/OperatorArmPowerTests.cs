using System.Collections.Generic;
using Teleop.RobotArm.Power;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Power
{
    public class OperatorArmPowerTests
    {
        private const long Tps = 1000;

        private sealed class Rig
        {
            public readonly OperatorArmPower Power = new OperatorArmPower(Tps);
            public readonly List<(long T, ArmPowerRequest Request, uint Seq)> Sent = new List<(long, ArmPowerRequest, uint)>();
            public long Now;

            /// <summary>The bridge's Update, every 10 ms: whatever message is due goes out.</summary>
            public void Run(long ms)
            {
                long end = Now + ms;
                while (Now < end)
                {
                    Now += 10;
                    if (Power.TryTakeMessage(Now, out ArmPowerRequest r, out uint s)) Sent.Add((Now, r, s));
                }
            }

            public uint LastSeq => Sent.Count == 0 ? 0 : Sent[Sent.Count - 1].Seq;

            public void Status(ArmPowerState state, ArmPowerReason reason = ArmPowerReason.None) =>
                Power.OnStatus(state, reason, LastSeq, float.NaN, Now);

            public int Count(ArmPowerRequest r) => Sent.FindAll(m => m.Request == r).Count;
        }

        [Fact]
        public void AtLaunch_TheModeIsOff_OnlyHeartbeatsGo_AndNothingMayCommand()
        {
            var rig = new Rig();
            Assert.Equal(OperatorMotionMode.Off, rig.Power.Mode);
            rig.Run(3000);
            Assert.All(rig.Sent, m => Assert.Equal(ArmPowerRequest.None, m.Request));
            Assert.InRange(rig.Sent.Count, 3, 4); // once a second
            Assert.False(rig.Power.MayCommand(rig.Now));
            Assert.Equal(OperatorArmState.Unknown, rig.Power.RobotState(rig.Now));
        }

        [Fact]
        public void Off_RepeatsRelaxUntilTheStatusSaysParking()
        {
            var rig = new Rig();
            rig.Status(ArmPowerState.Holding);
            rig.Power.SetMode(OperatorMotionMode.Off, rig.Now);
            rig.Run(1000);
            Assert.InRange(rig.Count(ArmPowerRequest.Relax), 3, 5); // about 3 a second
            rig.Status(ArmPowerState.Parking);
            int relaxes = rig.Count(ArmPowerRequest.Relax);
            rig.Run(3000);
            Assert.Equal(relaxes, rig.Count(ArmPowerRequest.Relax));
            Assert.False(rig.Power.RequestPending);
            Assert.False(rig.Power.MayCommand(rig.Now));
        }

        [Fact]
        public void Off_StopsAskingWhenTheRelaxIsRefused_AndKeepsTheReason()
        {
            var rig = new Rig();
            rig.Status(ArmPowerState.Holding);
            rig.Power.SetMode(OperatorMotionMode.Off, rig.Now);
            rig.Run(100);
            rig.Status(ArmPowerState.Holding, ArmPowerReason.PathCollides);
            int relaxes = rig.Count(ArmPowerRequest.Relax);
            rig.Run(2000);
            Assert.Equal(relaxes, rig.Count(ArmPowerRequest.Relax));
            Assert.Equal(ArmPowerReason.PathCollides, rig.Power.Reason);
        }

        [Fact]
        public void AStatusAnsweringAnEarlierMessage_DoesNotConfirmTheRelax()
        {
            var rig = new Rig();
            rig.Run(100); // a heartbeat, seq 1
            rig.Power.SetMode(OperatorMotionMode.Off, rig.Now);
            rig.Power.OnStatus(ArmPowerState.Holding, ArmPowerReason.NoRestPose, 1, float.NaN, rig.Now); // answers the heartbeat
            Assert.True(rig.Power.RequestPending);
        }

        [Fact]
        public void On_WhileRelaxed_SendsHoldFirst_AndCommandsOnlyOnceHolding()
        {
            var rig = new Rig();
            rig.Status(ArmPowerState.Relaxed);
            rig.Power.SetMode(OperatorMotionMode.On, rig.Now);
            Assert.False(rig.Power.MayCommand(rig.Now));
            rig.Run(1000);
            Assert.InRange(rig.Count(ArmPowerRequest.Hold), 3, 5);
            Assert.False(rig.Power.MayCommand(rig.Now));

            // A late status from before the hold does not count.
            rig.Power.OnStatus(ArmPowerState.Holding, ArmPowerReason.None, 0, float.NaN, rig.Now);
            Assert.False(rig.Power.MayCommand(rig.Now));

            rig.Status(ArmPowerState.Holding);
            Assert.True(rig.Power.MayCommand(rig.Now));
            Assert.True(rig.Power.TakeResumed());
            Assert.False(rig.Power.TakeResumed());
            int holds = rig.Count(ArmPowerRequest.Hold);
            rig.Run(2000);
            Assert.Equal(holds, rig.Count(ArmPowerRequest.Hold));
        }

        [Fact]
        public void On_WhileHoldingOrWithAnOlderRobotHost_CommandsAtOnce()
        {
            var rig = new Rig();
            rig.Power.SetMode(OperatorMotionMode.On, rig.Now);
            Assert.True(rig.Power.MayCommand(rig.Now)); // unknown: as before this feature
            rig.Status(ArmPowerState.Holding);
            Assert.True(rig.Power.MayCommand(rig.Now));
            rig.Run(2000);
            Assert.Equal(0, rig.Count(ArmPowerRequest.Hold));
            Assert.False(rig.Power.TakeResumed());
        }

        [Fact]
        public void On_ARelaxedStatusArrivingLater_StopsCommandsAndSendsHold()
        {
            // Motion switched on in the first second after launch, before the robot's status said relaxed.
            var rig = new Rig();
            rig.Power.SetMode(OperatorMotionMode.On, rig.Now);
            rig.Run(100);
            rig.Status(ArmPowerState.Relaxed);
            Assert.False(rig.Power.MayCommand(rig.Now));
            rig.Run(100);
            Assert.True(rig.Count(ArmPowerRequest.Hold) >= 1);
        }

        [Fact]
        public void Paused_NeverPowersTheServos_AndNeverCommands()
        {
            var rig = new Rig();
            rig.Status(ArmPowerState.Relaxed);
            rig.Power.SetMode(OperatorMotionMode.Paused, rig.Now);
            rig.Run(3000);
            Assert.Equal(0, rig.Count(ArmPowerRequest.Hold));
            Assert.Equal(0, rig.Count(ArmPowerRequest.Relax));
            Assert.False(rig.Power.MayCommand(rig.Now));
        }

        [Fact]
        public void WithoutPermissionToCommand_OnlyHeartbeatsGo()
        {
            var rig = new Rig();
            rig.Power.RequestsAllowed = false;
            rig.Power.SetMode(OperatorMotionMode.Off, rig.Now);
            rig.Run(2000);
            Assert.All(rig.Sent, m => Assert.Equal(ArmPowerRequest.None, m.Request));
        }

        [Fact]
        public void TheStateGoesUnknownWhenStatusesStop()
        {
            var rig = new Rig();
            rig.Status(ArmPowerState.Relaxed);
            Assert.Equal(OperatorArmState.Relaxed, rig.Power.RobotState(rig.Now));
            rig.Run(3100);
            Assert.Equal(OperatorArmState.Unknown, rig.Power.RobotState(rig.Now));
        }

        [Fact]
        public void EveryMessage_HasTheNextSequence_AndAModeChangeGoesAtOnce()
        {
            var rig = new Rig();
            rig.Run(500);
            rig.Power.SetMode(OperatorMotionMode.Paused, rig.Now);
            rig.Run(10);
            Assert.Equal(rig.Now, rig.Sent[rig.Sent.Count - 1].T);
            for (int i = 0; i < rig.Sent.Count; i++) Assert.Equal((uint)(i + 1), rig.Sent[i].Seq);
        }
    }
}
