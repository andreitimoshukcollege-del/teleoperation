using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>
    /// What the operator console sees of a joint link, whichever robot it is (docs/adr/0029 amendment): the predictor
    /// bank's presets, each one's live numbers, and the added delay. <see cref="JetRoverJointStateLink"/> is one; the
    /// FR3 Duo's is <see cref="SerialChainOperatorBridge"/>'s view of its <c>SerialChainJointLink</c>.
    /// </summary>
    public interface IOperatorJointLink
    {
        int PredictorCount { get; }
        int ActivePredictorIndex { get; }
        string PredictorLabel(int index);
        string PredictorDescription(int index);
        bool IsPredictorEnabled(int index);
        bool IsPredictorWarmingUp(int index);
        bool TrySelectPredictor(int index);
        bool TrySetPredictorEnabled(int index, bool enabled);
        bool TryGetPredictorStats(int index, out JetRoverPredictorStats stats);

        /// <summary>Frames arrive but the robot's readings stopped advancing (the JetRover's servo board); never for a robot without one.</summary>
        bool BoardSilent { get; }

        float SecondsSinceFreshReading { get; }

        /// <summary>Holds the robot's reports back this long (ms) before they become the measured arm; 0 adds nothing.</summary>
        double AddedDelayMs { get; set; }
    }

    /// <summary>The twin's three arms, shown or hidden by the bench's switches (docs/adr/0018): predicted, commanded, measured.</summary>
    public interface ITwinVisibility
    {
        bool ShowPredicted { get; set; }
        bool ShowCommanded { get; set; }
        bool ShowMeasured { get; set; }
    }

    /// <summary>
    /// What the lab's controls drive, whichever robot is in the scene (docs/adr/0029 amendment): the motion modes the
    /// power and pause buttons set, the network disturbance and predictor console, the link's status for the lamps,
    /// and turning the robot off when the headset comes off. <see cref="JetRoverOperatorBridge"/> and
    /// <see cref="SerialChainOperatorBridge"/> are the two; the console, the sliders, the buttons and the status
    /// lights take this type, so one scene's controls are the other's.
    ///
    /// <para>A JetRover's arm power (parking, relaxed, the hold it waits for) has no meaning for a robot without one,
    /// so those members default to "unknown" here.</para>
    /// </summary>
    public abstract class OperatorBridgeBase : MonoBehaviour
    {
        /// <summary>
        /// Whether this bridge has ever heard back from the robot, and if so, whether that contact is still fresh. A
        /// read of recency, not a socket state: UDP has no connection.
        /// </summary>
        public enum ConnectionStatus
        {
            /// <summary>No reply has been received since this bridge started.</summary>
            NeverConnected,

            /// <summary>A reply arrived within the bridge's staleness limit (about a second).</summary>
            Connected,

            /// <summary>Replies came, but none within the staleness limit.</summary>
            Stale,
        }

        public abstract ConnectionStatus Status { get; }

        /// <summary>This bridge's clock (the operator's), in its ticks.</summary>
        public abstract long NowTicks { get; }

        /// <summary>On: commands go. Paused: none, the arm holds. Off: none, and the robot is turned off as far as it can be.</summary>
        public abstract JetRoverMotionMode MotionMode { get; set; }

        /// <summary>The config lets this bridge command the robot at all; false locks the buttons.</summary>
        public abstract bool HardwareMotionAllowed { get; }

        /// <summary>Why the robot was last turned off by something other than its button; empty otherwise.</summary>
        public abstract string MotionOffReason { get; }

        /// <summary>Turns the robot off without the button (the headset taken off): at once, and as reliably as the robot allows.</summary>
        public abstract void TurnOffNow(string reason);

        /// <summary>On, but commands held back until the robot says it holds (the JetRover's torque); false for a robot without arm power.</summary>
        public virtual bool WaitingForHold => false;

        public virtual JetRoverArmPower ArmPower => JetRoverArmPower.Unknown;
        public virtual float ArmPowerRemainingDeg => 0f;
        public virtual string ArmPowerReasonText => "";

        /// <summary>The joint link the console shows; null until <c>Awake</c>.</summary>
        public abstract IOperatorJointLink Link { get; }

        /// <summary>The network disturbance on the joint socket; null until <c>Awake</c>.</summary>
        public abstract JointLinkDisturbance JointDisturbance { get; }

        /// <summary>Disturbs the joint socket both ways: a one-way delay, a uniform jitter, a loss; all zero is off.</summary>
        public abstract void ApplyCustomDisturbance(float delayMs, float jitterMs, float lossPercent);

        /// <summary>The unit of the console's gripper column.</summary>
        public virtual string GripperErrorUnit => "servo degrees";

        /// <summary>The joint speed every command asks for (the JetRover's servos); a robot that shapes its own motion ignores it.</summary>
        public virtual float SpeedRadPerSec
        {
            get => 0f;
            set { }
        }

        /// <summary>The JetRover's training course, while it sets the speed and clears the disturbance; null for other robots.</summary>
        public virtual JetRoverTrainingCourse Training => null;
    }
}
