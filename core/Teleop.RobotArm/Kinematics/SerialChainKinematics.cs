using System;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Kinematics
{
    /// <summary>
    /// Forward kinematics for one arm of a <see cref="SerialChainProfile"/> (docs/adr/0026 decision 4).
    /// Each joint's frame is <c>parent · Origin · motion(Axis, value)</c>, where the motion is a rotation
    /// for a revolute joint and a translation for a prismatic one, starting from the arm's
    /// <see cref="ChainArm.Mount"/>. Everything comes out in the robot's root frame.
    ///
    /// <para>Deterministic and allocation-free: results go into caller-supplied spans. It does not
    /// check limits. Inverse kinematics and collision are later decisions (decision 5).</para>
    /// </summary>
    public static class SerialChainKinematics
    {
        /// <summary>How many frames <see cref="ComputeFrames"/> writes for <paramref name="arm"/>: one per joint, then the tool.</summary>
        public static int FrameCount(ChainArm arm) => arm.Joints.Length + 1;

        /// <summary>
        /// Writes each joint's frame (index <c>i</c> is joint <c>i</c>'s moving frame, the frame of the
        /// link it drives) and then the tool centre point (index <c>Joints.Length</c>), in the robot's
        /// root frame.
        /// </summary>
        /// <param name="jointValues">One value per joint in <see cref="ChainArm.Joints"/> order: radians, or metres for a prismatic joint.</param>
        public static void ComputeFrames(ChainArm arm, ReadOnlySpan<double> jointValues, Span<RigidTransform> frames)
        {
            ChainJoint[] joints = arm.Joints;
            if (jointValues.Length != joints.Length)
            {
                throw new ArgumentException($"arm '{arm.Name}' has {joints.Length} joints; {jointValues.Length} values given", nameof(jointValues));
            }

            if (frames.Length < joints.Length + 1)
            {
                throw new ArgumentException($"frames must hold {joints.Length + 1} entries", nameof(frames));
            }

            RigidTransform current = arm.Mount;
            for (int i = 0; i < joints.Length; i++)
            {
                current = current.Compose(joints[i].Origin).Compose(Motion(joints[i], jointValues[i]));
                frames[i] = current;
            }

            frames[joints.Length] = current.Compose(arm.Tool);
        }

        /// <summary>The tool centre point in the robot's root frame.</summary>
        public static RigidTransform Tool(ChainArm arm, ReadOnlySpan<double> jointValues)
        {
            ChainJoint[] joints = arm.Joints;
            if (jointValues.Length != joints.Length)
            {
                throw new ArgumentException($"arm '{arm.Name}' has {joints.Length} joints; {jointValues.Length} values given", nameof(jointValues));
            }

            RigidTransform current = arm.Mount;
            for (int i = 0; i < joints.Length; i++)
            {
                current = current.Compose(joints[i].Origin).Compose(Motion(joints[i], jointValues[i]));
            }

            return current.Compose(arm.Tool);
        }

        private static RigidTransform Motion(in ChainJoint joint, double value) =>
            joint.Type == ChainJointType.Revolute
                ? RigidTransform.Rotation(joint.AxisX, joint.AxisY, joint.AxisZ, value)
                : RigidTransform.Translation(joint.AxisX * value, joint.AxisY * value, joint.AxisZ * value);
    }
}
