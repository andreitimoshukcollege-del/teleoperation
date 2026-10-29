using System;
using Teleop.Core.Types;
using UnityEngine;
using CoreQuat = System.Numerics.Quaternion;
using CoreVec = System.Numerics.Vector3;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>One moving joint of a <see cref="SerialChainArmRig"/>, as the editor's builder wrote it from the URDF.</summary>
    [Serializable]
    public sealed class SerialChainRigJoint
    {
        public string name = "";

        /// <summary>The motor id it answers to; a gripper's two fingers share one.</summary>
        public int motorId;

        public Transform transform;

        /// <summary>A finger: it slides along <see cref="axis"/>. Otherwise it turns about it.</summary>
        public bool prismatic;

        /// <summary>The joint's axis in its own frame, in Core's (URDF's) coordinates, unconverted.</summary>
        public Vector3 axis = new Vector3(0f, 0f, 1f);

        /// <summary>Metres per unit of the motor's value: a finger's travel, because the wire carries an opening fraction. 1 for a revolute joint.</summary>
        public float scale = 1f;

        /// <summary>The joint's URDF origin, converted: its pose at zero.</summary>
        public Vector3 restLocalPosition;
        public Quaternion restLocalRotation = Quaternion.identity;
    }

    /// <summary>
    /// A serial-chain robot's twin (docs/adr/0029 decision 6): the transform hierarchy the editor's builder made from
    /// the robot's URDF, the same file Core's profile was generated from, posed from joint values. Each joint is its
    /// URDF origin and then its motion, so the twin's frames are the profile's
    /// (<c>SerialChainKinematics.ComputeFrames</c>) converted, which <c>just unity-check-xr</c> checks at sampled poses.
    /// The conversion is <see cref="CoordConversion"/>'s, the only one there is.
    ///
    /// <para>Allocation-free: a joint whose motor is missing from the samples keeps its pose.</para>
    /// </summary>
    public sealed class SerialChainArmRig : MonoBehaviour
    {
        [SerializeField] private SerialChainRigJoint[] joints = new SerialChainRigJoint[0];

        public SerialChainRigJoint[] Joints => joints;

        /// <summary>Editor only: the builder sets the joints.</summary>
        public void Configure(SerialChainRigJoint[] rigJoints) => joints = rigJoints ?? new SerialChainRigJoint[0];

        /// <summary>Poses every joint whose motor is in <paramref name="samples"/>.</summary>
        public void Apply(ReadOnlySpan<JointSample> samples)
        {
            for (int k = 0; k < joints.Length; k++)
            {
                SerialChainRigJoint j = joints[k];
                if (j.transform == null) continue;
                for (int i = 0; i < samples.Length; i++)
                {
                    if (samples[i].MotorId != j.motorId) continue;
                    Set(j, samples[i].Value);
                    break;
                }
            }
        }

        /// <summary>Poses one joint by its index in <see cref="Joints"/> (the scene check's sampled poses).</summary>
        public void SetJoint(int index, float value) => Set(joints[index], value);

        private static void Set(SerialChainRigJoint j, float value)
        {
            if (float.IsNaN(value)) return;
            var axis = new CoreVec(j.axis.x, j.axis.y, j.axis.z);
            if (j.prismatic)
            {
                j.transform.localPosition = j.restLocalPosition + j.restLocalRotation * (axis * (value * j.scale)).ToUnity();
            }
            else
            {
                j.transform.localRotation = j.restLocalRotation * CoreQuat.CreateFromAxisAngle(axis, value * j.scale).ToUnity();
            }
        }
    }
}
