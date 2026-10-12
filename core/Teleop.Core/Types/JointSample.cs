// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// One joint's value in a joint-space state or command (docs/adr/0018): radians for arm joints
    /// and the wrist roll, a 0..1 fraction for a gripper, keyed by the motor that drives it.
    /// <see cref="Sensed"/> is true when the value was read from the servo, false when it is a
    /// commanded belief (an open-loop gripper, or a servo that gave no feedback).
    /// </summary>
    public readonly struct JointSample
    {
        public readonly byte MotorId;
        public readonly float Value;
        public readonly bool Sensed;

        public JointSample(byte motorId, float value, bool sensed)
        {
            MotorId = motorId;
            Value = value;
            Sensed = sensed;
        }

        public override string ToString() => $"#{MotorId}={Value:0.####}{(Sensed ? "" : "*")}";
    }
}
