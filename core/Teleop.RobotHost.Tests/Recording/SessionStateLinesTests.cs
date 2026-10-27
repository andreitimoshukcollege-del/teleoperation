using System.Collections.Generic;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Recording;

namespace Teleop.RobotHost.Tests.Recording
{
    /// <summary>
    /// Every session's file names its own subscriber, sender and uplink (docs/adr/0023 decision 4),
    /// once each, and again only on a change: the trainer's check allows one SS and one JS per session.
    /// </summary>
    public class SessionStateLinesTests
    {
        [Fact]
        public void EachStateIsWrittenOncePerSession_AndAgainOnlyOnAChange()
        {
            var lines = new List<string>();
            var state = new SessionStateLines(lines.Add);
            state.Subscriber(1, "10.0.0.5:6002");
            state.Subscriber(2, "10.0.0.5:6002");
            state.Sender(3, "10.0.0.5:6002");
            state.Sender(4, "10.0.0.5:6002");
            state.Uplink(5, "none", 0); // the format's starting state: nothing to say
            state.Uplink(6, "50ms-5j", 1);
            state.Uplink(7, "50ms-5j", 1);
            state.Uplink(8, "50ms-5j", 2);
            state.Uplink(9, "none", 2);
            state.Subscriber(10, null);
            Assert.Equal(new[] { "SS|1|10.0.0.5:6002", "JS|3|10.0.0.5:6002", "UP|6|50ms-5j|1", "UP|8|50ms-5j|2", "UP|9|none|2" }, lines);

            lines.Clear();
            state.Reset(); // a new session: its file says nothing yet
            state.Subscriber(20, "10.0.0.5:6002");
            state.Sender(20, "10.0.0.5:6002");
            state.Uplink(20, "none", 2);
            Assert.Equal(new[] { "SS|20|10.0.0.5:6002", "JS|20|10.0.0.5:6002" }, lines);
        }

        [Fact]
        public void TheCourse_IsWrittenWhenItsLabelChanges_NotForEveryScore()
        {
            var lines = new List<string>();
            var state = new SessionStateLines(lines.Add);
            state.Training(1, default); // stopped: the starting state, nothing to say
            var announced = new TrainingState(TrainingTask.Reach, TrainingPhase.Idle, TrainingPurpose.Train, false, 77, 1, 2.5f, 0f);
            state.Training(2, announced);
            state.Training(3, announced);
            var active = new TrainingState(TrainingTask.Reach, TrainingPhase.Active, TrainingPurpose.Train, false, 77, 1, 2.5f, 0f);
            state.Training(4, active);
            state.Training(5, new TrainingState(TrainingTask.Reach, TrainingPhase.Active, TrainingPurpose.Train, false, 77, 1, 2.5f, 3f));
            state.Training(6, new TrainingState(TrainingTask.Reach, TrainingPhase.Done, TrainingPurpose.Train, false, 77, 1, 2.5f, 8f));
            state.Training(7, new TrainingState(TrainingTask.Trace, TrainingPhase.Active, TrainingPurpose.Test, true, 77, 2, float.NaN, 12.5f));
            Assert.Equal(new[]
            {
                "TK|2|77|1|reach|idle|train|2.5|0|0",
                "TK|4|77|1|reach|active|train|2.5|0|0",
                "TK|6|77|1|reach|done|train|2.5|8|0",
                "TK|7|77|2|trace|active|test|varied|12.5|1",
            }, lines);

            lines.Clear();
            state.Reset(); // a new session repeats the round that is going
            state.Training(20, new TrainingState(TrainingTask.Trace, TrainingPhase.Active, TrainingPurpose.Test, true, 77, 2, float.NaN, 40f));
            Assert.Equal(new[] { "TK|20|77|2|trace|active|test|varied|40|1" }, lines);
        }
    }
}
