using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// The robot being commanded. One interface over three very different things: a Core
    /// rigid-body approximation (headless, deterministic, what sweeps run against), Unity
    /// physics (implemented in <c>Bridge/</c>), and real hardware over ROS 2. An algorithm that
    /// only works against one of them is not a result.
    ///
    /// Poses are ROS convention, metres and radians (see <see cref="Pose"/>). The plant owns
    /// the mapping from a normalized command to hardware units — joint limits, gripper travel,
    /// actuator dynamics — so that nothing above it needs to know which robot is attached.
    ///
    /// Time is always a parameter. A plant must never read a clock, not even the Unity one:
    /// that is the difference between a sweep that reproduces and one that does not.
    /// </summary>
    public interface IRobotPlant
    {
        /// <summary>
        /// Accept a command. Applies it as the current setpoint; it does not advance the
        /// simulation — <see cref="Step"/> does. Commands may arrive out of order or duplicated
        /// after transport, so an implementation compares
        /// <see cref="CommandFrame.CaptureTicks"/> against the setpoint it holds and ignores a
        /// stale or repeated frame rather than jerking backwards to it.
        ///
        /// Between commands the plant keeps its last setpoint; how it behaves through a gap —
        /// hold, coast on the commanded velocity, or ramp to a stop — is implementation policy
        /// and must be documented, because it is the behaviour packet loss actually exposes to
        /// the operator.
        /// </summary>
        void Command(in CommandFrame command);

        /// <summary>
        /// Advance the plant to <paramref name="nowTicks"/>. Called every step whether or not a
        /// command arrived. Advancing to a time at or before the current state time is a no-op,
        /// so a duplicate step cannot double-integrate.
        ///
        /// Implementations backed by an external simulator that runs on its own schedule (Unity
        /// physics) treat this as a synchronization point rather than an integration step, and
        /// say so in their documentation — the timing of that host loop is why results from a
        /// Unity plant are not interchangeable with results from the Core plant.
        /// Allocation-free.
        /// </summary>
        void Step(long nowTicks);

        /// <summary>
        /// Current plant state, stamped with the time it is valid at — which is the last
        /// <see cref="Step"/> time for a simulated plant, and the sensor capture time for real
        /// hardware, never the time it was read. Downstream latency accounting depends on that
        /// distinction.
        /// </summary>
        Stamped<Pose> State { get; }

        /// <summary>
        /// Returns the plant to its as-constructed state: initial pose, zero velocity, no
        /// setpoint, state time back to its initial value. Sweeps reuse instances across
        /// trials, and a trial that starts where the last one ended is a silent confound.
        /// </summary>
        void Reset();
    }
}
