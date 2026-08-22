using System;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Relay
{
    /// <summary>
    /// The seam between <c>GenericArmPlant</c> and whatever actually talks to the robot's ROS 2
    /// node (a Unix domain socket to a relay node, in the real implementation). Deliberately a
    /// plain interface local to this project, not a <c>Teleop.Core.Contracts</c> interface --
    /// Core must never know ROS, a relay process, or a local socket exists; this is purely a
    /// host-side seam so <c>GenericArmPlant</c>'s staleness/gap-policy logic can be unit tested
    /// against a fake, with no ROS, no Jetson, and no motors involved.
    ///
    /// No sequence numbers, staleness rejection, or coast/hold logic at this layer -- that is
    /// entirely <c>GenericArmPlant</c>'s and <c>RobotEndpoint</c>'s job, upstream of this seam.
    /// This interface only ever needs to carry "here are the current numbers," one entry per motor
    /// id (docs/adr/0011-generic-robot-arm-profiles.md) instead of the old fixed 4-joint shape.
    /// </summary>
    public interface IRelayClient
    {
        /// <summary>
        /// Sends the current per-joint targets. Fire-and-forget, no acknowledgement expected at
        /// this hop -- matches the "here are the current numbers" contract above.
        /// </summary>
        void Send(ReadOnlySpan<JointTarget> targets);

        /// <summary>
        /// Non-blocking; returns false when no feedback has arrived since the last call (the
        /// common case, not an error). <paramref name="entriesBuffer"/> is caller-supplied
        /// (typically <c>stackalloc</c>), keeping this allocation-free.
        /// </summary>
        bool TryReceiveFeedback(Span<JointFeedbackEntry> entriesBuffer, out int entryCount);
    }
}
