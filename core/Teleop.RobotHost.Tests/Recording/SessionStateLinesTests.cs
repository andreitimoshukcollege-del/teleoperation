using System.Collections.Generic;
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
    }
}
