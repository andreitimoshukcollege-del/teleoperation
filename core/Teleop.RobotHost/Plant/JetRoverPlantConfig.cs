namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// <see cref="JetRoverPlant"/>'s tunable knobs. Unlike <c>RigidBodyPlant</c> (Core's plant,
    /// deliberately config-free because it has no real research knobs), this plant drives real
    /// motors and has at least one genuinely safety-relevant parameter -- a config struct is the
    /// right call here, not ceremony.
    ///
    /// Phase 1 scope only: <see cref="PositionXToDirectionScale"/> and
    /// <see cref="MaxDirectionMagnitude"/> exist solely to support the temporary
    /// <c>CommandFrame.Pose</c>-to-relative-servo-nudge stand-in documented on
    /// <see cref="JetRoverPlant.Command"/>, and go away once real inverse kinematics replaces it
    /// (docs/adr/0007-jetrover-plant-and-robot-host.md).
    /// </summary>
    public readonly struct JetRoverPlantConfig
    {
        /// <summary>
        /// Phase-1-only stand-in scale factor: multiplies <c>CommandFrame.Pose.Position.X</c>
        /// (metres) into the base servo's "direction" units (see <c>RelayProtocol</c>'s doc for
        /// what that unit means on the ROS side). Meaningless once real IK lands.
        /// </summary>
        public readonly float PositionXToDirectionScale;

        /// <summary>
        /// Hard clamp on the magnitude of any single direction value sent to the relay,
        /// independent of whatever <see cref="PositionXToDirectionScale"/> computes -- a
        /// safety backstop against a bug in that computation (or in whatever produces
        /// <c>CommandFrame.Pose</c> upstream) commanding an oversized single step.
        /// </summary>
        public readonly float MaxDirectionMagnitude;

        public JetRoverPlantConfig(float positionXToDirectionScale, float maxDirectionMagnitude)
        {
            PositionXToDirectionScale = positionXToDirectionScale;
            MaxDirectionMagnitude = maxDirectionMagnitude;
        }

        /// <summary>
        /// Conservative defaults for the Phase 1 smoke test: a 1:1 scale, and a magnitude clamp
        /// matching the direction values already manually verified safe against the real
        /// hardware (docs/adr/0007-jetrover-plant-and-robot-host.md's Phase 0/1 hardware tests).
        /// </summary>
        public static JetRoverPlantConfig Default => new JetRoverPlantConfig(
            positionXToDirectionScale: 1f,
            maxDirectionMagnitude: 5f);
    }
}
