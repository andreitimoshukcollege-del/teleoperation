using System.Collections.Generic;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Plant
{
    /// <summary>
    /// A test double for <see cref="IRelayClient"/> -- lets <see cref="JetRoverPlant"/>'s
    /// staleness/gap-policy/direction logic be tested with no ROS, no Jetson, and no motors,
    /// matching this project's "an algorithm that cannot be evaluated headlessly does not count"
    /// ethos extended to host-adjacent code with real algorithmic content.
    /// </summary>
    internal sealed class FakeRelayClient : IRelayClient
    {
        public List<LocalArmCommand> SentCommands { get; } = new List<LocalArmCommand>();

        private readonly Queue<LocalFeedback> _pendingFeedback = new Queue<LocalFeedback>();

        public void Send(in LocalArmCommand command) => SentCommands.Add(command);

        public void EnqueueFeedback(LocalFeedback feedback) => _pendingFeedback.Enqueue(feedback);

        public bool TryReceiveFeedback(out LocalFeedback feedback)
        {
            if (_pendingFeedback.Count == 0)
            {
                feedback = default;
                return false;
            }

            feedback = _pendingFeedback.Dequeue();
            return true;
        }
    }
}
