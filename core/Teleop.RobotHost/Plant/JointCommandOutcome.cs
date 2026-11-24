namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// What <see cref="GenericArmPlant.CommandJointAngles"/> did with a joint command. Recorded with
    /// every command (docs/adr/0021): a refused or stale command is part of what the arm did not do.
    /// </summary>
    public enum JointCommandOutcome
    {
        /// <summary>Taken: the targets went to the relay.</summary>
        Accepted,

        /// <summary>Dropped: its capture stamp was not newer than the last one taken.</summary>
        Stale,

        /// <summary>
        /// Dropped, and the arm holds: the pose, or the servo path to it, would collide (docs/adr/0020) and, on the JetRover with
        /// <see cref="HostCheckOnFail.Clamp"/> (the default since 2026-11-24), no point on the line toward it passes either; or the
        /// command was malformed (a value not finite, or a motor named twice). The plants count the causes apart.
        /// </summary>
        Refused,

        /// <summary>Dropped: the arm was parking or relaxed (docs/adr/0023 decision 5); nothing reached the plant.</summary>
        Parked,

        /// <summary>
        /// Dropped, and the arm holds: the optional command gate held it (docs/adr/0038 N, off by default). Only a command the
        /// plant would otherwise have accepted reaches the gate.
        /// </summary>
        Held,

        /// <summary>
        /// The command failed the collision check (docs/adr/0020), and the farthest point toward it on the joint-space line from
        /// the measured pose that passes the same check went to the relay in its place (<see cref="HostCheckOnFail.Clamp"/>,
        /// docs/adr/0020 amendment 2026-11-24). Recorded as <c>JC</c> <c>C</c>, with a <c>JK</c> giving how far along the line.
        /// </summary>
        Clamped,
    }

    /// <summary>
    /// What <see cref="GenericArmPlant"/> does with a joint command that fails its collision check (docs/adr/0020 amendment
    /// 2026-11-24). Recorded at the start of every session as <c>HK</c>; a session without one refused.
    /// </summary>
    public enum HostCheckOnFail
    {
        /// <summary>Send the farthest point toward the command that passes the check; hold only when none does. The default.</summary>
        Clamp,

        /// <summary>Drop the command whole and hold, as before 2026-11-24 (<c>--host-check-onfail refuse</c>).</summary>
        Refuse,
    }
}
