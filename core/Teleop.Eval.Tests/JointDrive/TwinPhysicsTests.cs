using System;
using System.Collections.Generic;
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
    /// docs/adr/0038 "M, motion", round 7 on the twin: the dry run counts contacts as the twin's physics makes them (contact events, a tippy
    /// object knocked over at its first contact and lying after, the arm sagging under its weight); the planner aims nothing more at a tippy
    /// object it has already touched; a replay finds its way back to where its source began by way of the arm tucked up.
    /// </summary>
    public class TwinPhysicsTests
    {
        private readonly ITestOutputHelper _output;

        public TwinPhysicsTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void ContactEvents_AreOneContact_WhenTheSameObjectIsTouchedAgainWithinHalfASecond()
        {
            var e = new MixedDryRun.ContactEvents();
            Assert.True(e.Begin(5, 1.0));
            e.Stop(5, 1.2);
            Assert.False(e.Begin(5, 1.6)); // 0.4 s after it let go: the same push
            e.Stop(5, 1.7);
            Assert.True(e.Begin(5, 2.3)); // 0.6 s after: a new contact
            Assert.True(e.Begin(6, 2.3)); // another object is its own
        }

        [Fact]
        public void PhysicsLike_KnocksATippyObjectOverAtItsFirstContact_AndMeetsItOnlyWhereItLies()
        {
            // A dowel standing 30 cm out (2 cm across, 20 cm tall) and a squat tin beside it, the table 19.6 cm below the base frame.
            var dowel = new SafetyObject(1, new Vector3(0.30f, 0f, -0.096f), Quaternion.Identity, new Vector3(0.01f, 0.01f, 0.10f), movable: true, mass: 0.04, tippy: true);
            var tin = new SafetyObject(2, new Vector3(0.30f, 0.15f, -0.166f), Quaternion.Identity, new Vector3(0.04f, 0.04f, 0.03f), movable: true, mass: 0.3, tippy: false);
            SafetyObject fallen = MixedDryRun.Fallen(dowel);
            Assert.Equal(dowel.Id, fallen.Id);
            Assert.False(fallen.Tippy);
            Assert.Equal(dowel.Min.Z, fallen.Min.Z, 3); // on what it stood on
            Assert.True(fallen.Max.Z - fallen.Min.Z <= 0.021f, "lying, its narrow side up");
            Assert.True(fallen.Center.X > dowel.Center.X + 0.05f, "fallen outward, away from the base");

            var like = new MixedDryRun.PhysicsLike(new[] { dowel, tin });
            var arm = new ArmTouch(Profile, Model).At(Model.Home, 0f); // high over the base: touching neither as it lies
            ArmLink link = ArmLink.Gripper;
            Assert.True(like.Step(arm, 0, 1.0, true, ref link)); // the first touch: a contact, and over it goes
            Assert.False(like.Step(arm, 0, 3.0, true, ref link)); // where it stood: nothing there now
            Assert.True(like.Step(arm, 1, 1.0, true, ref link));
            Assert.False(like.Step(arm, 1, 1.1, false, ref link));
            Assert.True(like.Step(arm, 1, 3.0, true, ref link)); // a steady object is met again
        }

        [Fact]
        public void TheFollower_SagsAsTheTwinsArmDoes()
        {
            Assert.Equal(new[] { 0, -0.019, -0.0135, -0.0045, 0 }, TakeAudit.FollowSag);
            ArmPose p = TakeAudit.Sagged(0.1f, 1.0f, -1.5f, 0.3f, 0.2f);
            Assert.Equal(1.0f - 0.019f, p.Proximal, 5);
            Assert.Equal(-1.5f - 0.0135f, p.Distal, 5);
            Assert.Equal(0.1f, p.BaseYaw, 5);
            float zFollowed = new ArmTouch(Profile, Model).At(new ArmPose(0f, 0.6f, -1.2f, -0.4f, 0f), 0f).Tool.Z;
            float zSagged = new ArmTouch(Profile, Model).At(TakeAudit.Sagged(0f, 0.6f, -1.2f, -0.4f, 0f), 0f).Tool.Z;
            Assert.InRange(zFollowed - zSagged, 0.003f, 0.02f); // the tool point hangs lower, about a centimetre
        }

        [Fact]
        public void APlan_AimsNothingMoreAtATippyObjectItHasTouched()
        {
            int spentAimsOn = 0, spentAimsOff = 0, placedOn = 0, placedOff = 0, touched = 0;
            for (ulong layout = 1; layout <= 30; layout++)
            {
                SafetyObject[] objects = MixedTakeTests.MixedLayout(layout);
                foreach (bool spend in new[] { true, false })
                {
                    if (!MixedTakeTests.TryMixed(objects, 7300 + layout, out DrivePlan? plan, out _, new MixedOptions { SpendTouched = spend })) continue;
                    // When the commands first touch each tippy movable object by the model.
                    var arm = new ArmTouch(Profile, Model);
                    var first = new Dictionary<byte, double>();
                    for (double t = plan!.ScriptStart; t <= plan.TotalSeconds; t += Tick)
                    {
                        arm.At(plan.Sample(t), plan.GripperAt(t));
                        foreach (SafetyObject o in objects)
                            if (o.Movable && o.Tippy && !o.Scenery && !first.ContainsKey(o.Id) && arm.Touches(o)) first[o.Id] = t;
                    }

                    if (spend) touched += first.Count;
                    // Aims and passes begun after their object was touched; accidents placed near objects aimed at one touched before they began.
                    int late = plan.Moves.Count(m => (m.Kind == ScriptMoveKind.Aim || m.Kind == ScriptMoveKind.Pass) && first.TryGetValue(m.ObjectId, out double t0) && m.Start > t0 + 0.5);
                    int placed = 0;
                    foreach (PlannedMove m in plan.Moves.Where(x => x.Part == MovePart.NearAccident && ScriptMoveCodec.IsAccident(x.Kind)))
                    {
                        for (double t = m.Start; t <= m.End && t <= plan.TotalSeconds; t += Tick)
                        {
                            arm.At(plan.Sample(t), plan.GripperAt(t));
                            if (!objects.Any(o => first.TryGetValue(o.Id, out double t0) && t0 < m.Start - 0.5 && arm.Touches(o))) continue;
                            placed++;
                            break;
                        }
                    }

                    if (spend) (spentAimsOn, placedOn) = (spentAimsOn + late, placedOn + placed);
                    else (spentAimsOff, placedOff) = (spentAimsOff + late, placedOff + placed);
                }
            }

            string report = $"tippy objects touched {touched}; aims and passes at one already touched: spent {spentAimsOn}, not spent {spentAimsOff}; accidents placed near objects meeting one touched before: {placedOn} against {placedOff}";
            _output.WriteLine(report);
            Assert.True(touched >= 10, report);
            Assert.Equal(0, spentAimsOn);
            Assert.True(placedOn <= placedOff / 2, report);
        }

        [Fact]
        public void TheWayBack_ByWayOfTheArmTuckedUp_IsNeverWorse_AndEveryLegIsClear()
        {
            int groups = 0, plain = 0, tucked = 0;
            for (ulong layout = 1; layout <= 40; layout++)
            {
                SafetyObject[] objects = MixedTakeTests.MixedLayout(layout);
                if (!MixedTakeTests.TryMixed(objects, 7400 + layout, out DrivePlan? plan, out _)) continue;
                ArmCollisionModel back = SafetyScript.WithObjects(Model, objects, 0.01f);
                ArmPose end = plan!.Sample(plan.TotalSeconds), begin = plan.Sample(plan.ScriptStart);
                groups++;
                bool a = DrivePlan.TryApproach(Profile, back, end, begin, out List<(ArmPose To, bool Step)> viaPlain, out _);
                bool b = DrivePlan.TryApproach(Profile, back, end, begin, out List<(ArmPose To, bool Step)> viaTucked, out _, tucked: true);
                if (a) plain++;
                if (b) tucked++;
                Assert.True(!a || b, $"layout {layout}: the tucked way refused what the plain way found");
                if (a) Assert.Equal(viaPlain, viaTucked); // where the plain way works it is taken unchanged
                if (!b) continue;
                ArmPose from = end;
                foreach ((ArmPose to, bool _) in viaTucked)
                {
                    Assert.True(DrivePlan.PathValid(Profile, back, null, from, to), $"layout {layout}: a leg of the way back meets an object or the arm");
                    from = to;
                }

                Assert.True(from.MaxJointDelta(begin) < 1e-4f);
            }

            _output.WriteLine($"{groups} plans: a way back {plain} straight or a joint at a time, {tucked} with the tucked way");
            Assert.True(groups >= 30);
        }

        [Fact]
        public void TheWayBack_GoesRoundAWallTheArmCannotSweepPastLow()
        {
            // The arm low on one side of a wall that stands between it and its target, low on the other side: no straight or joint-at-a-time
            // way passes it low, and the tucked arm turns over it.
            var wall = new SafetyObject(9, new Vector3(0.22f, 0.0f, -0.120f), Quaternion.Identity, new Vector3(0.10f, 0.012f, 0.076f), movable: false);
            ArmCollisionModel back = SafetyScript.WithObjects(Model, new[] { wall }, 0.01f);
            var from = new ArmPose(0.6f, 0.25f, -1.2f, -0.6f, 0f);
            var to = new ArmPose(-0.6f, 0.25f, -1.2f, -0.6f, 0f);
            Assert.True(ArmCollision.IsValid(Profile, back, from) && ArmCollision.IsValid(Profile, back, to), "both ends clear of the wall");
            Assert.False(DrivePlan.TryApproach(Profile, back, from, to, out _, out _));
            Assert.True(DrivePlan.TryApproach(Profile, back, from, to, out List<(ArmPose To, bool Step)> via, out string? error, tucked: true), error);
            ArmPose at = from;
            foreach ((ArmPose next, bool _) in via)
            {
                Assert.True(DrivePlan.PathValid(Profile, back, null, at, next));
                at = next;
            }
        }
    }
}
