namespace Teleop.RobotArm.Collision
{
    /// <summary>
    /// The collision model each named profile gets (docs/adr/0020-collision-aware-ik.md). A plain
    /// table, no reflection (IL2CPP strips what nothing references). Every profile loader attaches
    /// the model from here by the profile's name, so a JSON profile never carries geometry.
    /// </summary>
    public static class ArmCollisionModels
    {
        /// <summary>The model for a profile name, or null when the robot has none.</summary>
        public static ArmCollisionModel? ForProfile(string? profileName) => profileName switch
        {
            "jetrover" => JetRoverCollisionModel.Instance,
            _ => null,
        };
    }
}
