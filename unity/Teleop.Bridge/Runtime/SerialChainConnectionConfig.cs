using System;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>
    /// The FR3 Duo operator's settings (docs/adr/0029), loaded by <see cref="ConfigLoader"/> from
    /// <c>persistentDataPath/fr3_connection.json</c>, else <c>Resources/fr3_connection.json</c>. Plain data, like
    /// <see cref="JetRoverArmConfig"/>: every value is passed to <c>Teleop.RobotArm</c>'s operator, which decides what
    /// it means. The robot profile and its collision model are not here: they are generated files
    /// (<c>Resources/fr3_duo_profile.json</c>, <c>fr3_duo_collision.json</c>), copies a test holds to
    /// <c>core/RobotProfiles/</c>.
    /// </summary>
    [Serializable]
    public sealed class SerialChainConnectionConfig
    {
        /// <summary>The serial-chain RobotHost (<c>just fr3-sim-host</c>): an address or a host name. 127.0.0.1 is the PC itself, which is right for Direct Preview.</summary>
        public string RemoteHost = "127.0.0.1";

        public int JointRemotePort = 6002;
        public int JointLocalPort = 6012;
        public float CommandRateHz = 48f;
        public double MaxHorizonMs = 300.0;

        /// <summary>
        /// <c>absolute</c> (the default, docs/adr/0029 amendment): the robot anchored to your body, so where your hand is and
        /// which way your palm faces is where the gripper goes and faces. <c>relative</c>: the clutch, which moves the
        /// gripper by how far your hand moves from where you grabbed.
        /// </summary>
        public string HandMapping = "absolute";

        /// <summary>Hand motion → tool motion; 1 is one to one.</summary>
        public double PositionScale = 1.0;

        /// <summary>The absolute mapping's neutral hand, from the headset at recentre (m): forward, to each hand's side, down. There the arms are at their neutral tools.</summary>
        public double NeutralForward = 0.30;
        public double NeutralSide = 0.20;
        public double NeutralDown = 0.40;

        /// <summary>Frames a grab takes to slide the arm to where the hand maps (absolute mapping): about a second at 90 Hz.</summary>
        public int ApproachSteps = 90;

        /// <summary>The most a joint's command moves per frame (radians); a larger IK step is shortened.</summary>
        public double MaxStepRadians = 0.1;

        /// <summary>
        /// <c>toggle</c> (the default, docs/adr/0029 amendment): fold the middle, ring and little fingers twice quickly to
        /// lock that arm onto the hand, and twice again to unlock it. <c>hold</c>: the arm follows only while the curl is held.
        /// </summary>
        public string GrabMode = "toggle";

        /// <summary>The toggle's clicks: a curl held between these two lengths (s), the second starting within the longer of the first's end.</summary>
        public double MinPressSeconds = 0.04;
        public double DoubleClickSeconds = 0.6;

        /// <summary>The grab (middle, ring and little fingers curled) holds below this curl ratio and lets go above the release one.</summary>
        public float GrabEngageBelowCurl = 1.25f;
        public float GrabReleaseAboveCurl = 1.55f;

        /// <summary>The thumb-index gap that is the gripper fully open, and fully closed (m).</summary>
        public float PinchOpenMeters = 0.08f;
        public float PinchClosedMeters = 0.015f;

        /// <summary>False: nothing is ever sent but keepalives; the twin follows the robot and the markers your hands.</summary>
        public bool AllowMotion = true;

        /// <summary>The console's network sliders never go past these (ms of delay plus jitter, % loss), as the JetRover's.</summary>
        public float DisturbanceMaxDelayMs = 2000f;
        public float DisturbanceMaxLossPercent = 50f;

        public int ClockSyncHistoryCapacity = 32;
        public float ClockSyncSmoothingAlpha = 0.2f;
        public double MaxAcceptableRttMs = 2000.0;
        public double OutlierRttMultiple = 3.0;
        public int MinSamplesBeforeTrusted = 3;

        /// <summary>The predictors the twin can draw with; <see cref="JointPredictorActive"/> names the one drawing at launch.</summary>
        public JointPredictorPresetData[] JointPredictorPresets = new JointPredictorPresetData[0];
        public string JointPredictorActive = "franka-replay";
    }
}
