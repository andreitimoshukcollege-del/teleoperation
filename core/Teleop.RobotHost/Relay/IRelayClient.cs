namespace Teleop.RobotHost.Relay
{
    /// <summary>
    /// The seam between <c>JetRoverPlant</c> and whatever actually talks to the JetRover's ROS 2
    /// node (a Unix domain socket to a relay node, in the real implementation). Deliberately a
    /// plain interface local to this project, not a <c>Teleop.Core.Contracts</c> interface --
    /// Core must never know ROS, a relay process, or a local socket exists; this is purely a
    /// host-side seam so <c>JetRoverPlant</c>'s staleness/gap-policy logic can be unit tested
    /// against a fake, with no ROS, no Jetson, and no motors involved.
    ///
    /// No sequence numbers, staleness rejection, or coast/hold logic at this layer -- that is
    /// entirely <c>JetRoverPlant</c>'s and <c>RobotEndpoint</c>'s job, upstream of this seam.
    /// This interface only ever needs to carry "here are the current numbers."
    /// </summary>
    public interface IRelayClient
    {
        /// <summary>
        /// Sends the current arm command. Fire-and-forget, no acknowledgement expected at this
        /// hop -- matches the "here are the current numbers" contract above.
        /// </summary>
        void Send(in LocalArmCommand command);

        /// <summary>
        /// Non-blocking; returns false when no feedback has arrived since the last call, which
        /// is the common case and not an error.
        /// </summary>
        bool TryReceiveFeedback(out LocalFeedback feedback);
    }
}
