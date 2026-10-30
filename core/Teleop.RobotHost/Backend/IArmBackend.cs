using System;
using Teleop.Core.Types;

namespace Teleop.RobotHost.Backend
{
    /// <summary>
    /// The seam between <see cref="Plant.SerialChainPlant"/> and whatever moves a serial-chain arm
    /// (docs/adr/0027 decision 3). Local to RobotHost, like <c>IRelayClient</c> is for the JetRover: Core never
    /// learns that a robot, a socket or a real-time process exists.
    ///
    /// <para>Two implementations are planned. <see cref="Fr3SimBackend"/> exists now: an in-process FR3 that
    /// shapes with Core's <c>JointTrajectoryShaper</c>. A client of a separate real-time arm server comes later,
    /// with its own ADR, when the robot arrives; it owns libfranka's 1 kHz callbacks and runs a golden-pinned
    /// copy of the same shaper.</para>
    /// </summary>
    public interface IArmBackend
    {
        /// <summary>
        /// The targets now in force, one per motor: radians for an arm joint, the 0..1 opening fraction for a
        /// gripper. Absolute and fire-and-forget; the backend shapes toward the newest. Ignored while disabled.
        /// </summary>
        void Send(ReadOnlySpan<JointSample> targets, long hostTicks);

        /// <summary>
        /// The newest measured state, one sample per motor, and when it was measured, on the host's clock.
        /// Returns the count written; 0 before the first measurement.
        /// </summary>
        int ReadState(long hostTicks, Span<JointSample> output, out long sensedTicks);

        /// <summary>Whether targets move the arm. Disabled, the arm holds where it is.</summary>
        bool Enabled { get; }

        void SetEnabled(bool enabled, long hostTicks);
    }
}
