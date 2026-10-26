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

        /// <summary>Dropped: the pose would collide (docs/adr/0020); the arm holds.</summary>
        Refused,

        /// <summary>Dropped: the arm was parking or relaxed (docs/adr/0023 decision 5); nothing reached the plant.</summary>
        Parked,
    }
}
