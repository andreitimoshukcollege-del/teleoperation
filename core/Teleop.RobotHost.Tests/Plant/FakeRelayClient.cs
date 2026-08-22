using System.Collections.Generic;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Plant
{
    /// <summary>
    /// A test double for <see cref="IRelayClient"/> -- lets <see cref="GenericArmPlant"/>'s
    /// staleness/gap-policy/direction logic be tested with no ROS, no Jetson, and no motors,
    /// matching this project's "an algorithm that cannot be evaluated headlessly does not count"
    /// ethos extended to host-adjacent code with real algorithmic content. Each captured command
    /// is copied into its own array (a <c>Span</c> passed to <see cref="Send"/> is only valid for
    /// the call itself) so tests can inspect every call's joint targets after the fact.
    /// </summary>
    internal sealed class FakeRelayClient : IRelayClient
    {
        public List<JointTarget[]> SentCommands { get; } = new List<JointTarget[]>();

        private readonly Queue<JointFeedbackEntry[]> _pendingFeedback = new Queue<JointFeedbackEntry[]>();

        public void Send(ReadOnlySpan<JointTarget> targets) => SentCommands.Add(targets.ToArray());

        public void EnqueueFeedback(params JointFeedbackEntry[] entries) => _pendingFeedback.Enqueue(entries);

        public bool TryReceiveFeedback(System.Span<JointFeedbackEntry> entriesBuffer, out int entryCount)
        {
            if (_pendingFeedback.Count == 0)
            {
                entryCount = 0;
                return false;
            }

            JointFeedbackEntry[] entries = _pendingFeedback.Dequeue();
            entries.CopyTo(entriesBuffer);
            entryCount = entries.Length;
            return true;
        }
    }
}
