using System.Numerics;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// One operator command, as it crosses the wire. This is the unit an
    /// <c>ICommandCodec</c> encodes and an <c>IRobotPlant</c> consumes.
    ///
    /// Poses are in the canonical Core convention: ROS, right-handed, Z-up, X-forward, metres
    /// and radians (see <see cref="Pose"/>). Conversion to Unity's convention happens in
    /// exactly one place, unity/.../Bridge/CoordConversion.cs.
    ///
    /// The velocity fields carry commanded *intent*, not a measurement. They exist because
    /// intent is what survives a lost packet: a plant that knows where the operator was heading
    /// can follow through a gap, and the trajectory codec extrapolates from them. A codec that
    /// does not transmit them must document that it drops them rather than sending zeros.
    /// </summary>
    public readonly struct CommandFrame
    {
        /// <summary>
        /// Monotonically increasing per sender, wrapping at <c>uint.MaxValue</c>. Used for loss
        /// and reordering accounting and as the key a delta codec deltas against. Compare with
        /// wrap-safe arithmetic, never with a plain <c>&lt;</c>.
        /// </summary>
        public readonly uint Sequence;

        /// <summary>
        /// Highest <see cref="Sequence"/> this sender has received from its peer;
        /// <see cref="Sequence"/> values are per-direction, so this is the acknowledgement
        /// channel a delta codec needs to know which frame the far end can still delta
        /// against. Zero means nothing has been received yet.
        /// </summary>
        public readonly uint AckSequence;

        /// <summary>
        /// <c>t_capture</c> from docs/metrics.md: when the input device was sampled, in ticks
        /// on the sender's <c>ITimeAuthority</c> timebase. Every latency figure is a difference
        /// from this stamp, so it must be the sample time, not the send time.
        /// </summary>
        public readonly long CaptureTicks;

        /// <summary>Commanded end-effector pose.</summary>
        public readonly Pose Pose;

        /// <summary>Commanded linear velocity at capture, metres/second.</summary>
        public readonly Vector3 LinearVelocity;

        /// <summary>
        /// Commanded angular velocity at capture, radians/second, as an axis-angle rate vector
        /// (direction is the rotation axis, magnitude is the rate).
        /// </summary>
        public readonly Vector3 AngularVelocity;

        /// <summary>
        /// Gripper command, 0 = fully open, 1 = fully closed. Normalized here so that no part
        /// of Core knows a particular gripper's travel; the mapping to hardware units belongs
        /// to the plant.
        /// </summary>
        public readonly float Gripper;

        public CommandFrame(
            uint sequence,
            uint ackSequence,
            long captureTicks,
            Pose pose,
            Vector3 linearVelocity,
            Vector3 angularVelocity,
            float gripper)
        {
            Sequence = sequence;
            AckSequence = ackSequence;
            CaptureTicks = captureTicks;
            Pose = pose;
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
            Gripper = gripper;
        }

        public override string ToString() =>
            $"CommandFrame(seq={Sequence}, ack={AckSequence}, t={CaptureTicks}, {Pose}, " +
            $"v=({LinearVelocity.X:F3},{LinearVelocity.Y:F3},{LinearVelocity.Z:F3}), " +
            $"w=({AngularVelocity.X:F3},{AngularVelocity.Y:F3},{AngularVelocity.Z:F3}), " +
            $"grip={Gripper:F3})";
    }
}
