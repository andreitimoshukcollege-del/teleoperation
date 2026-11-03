using Teleop.RobotArm.World;
using UnityEngine;
using CoreQuat = System.Numerics.Quaternion;
using CoreVec = System.Numerics.Vector3;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>
    /// The world twin (docs/adr/0031 amendment 2): each object of the scene file, posed in <c>onBeforeRender</c> where the robot last
    /// reported it, after the joint link's playout, so objects and arms cross the headset's disturbance together. The objects are
    /// children of this transform, which sits at the robot's root frame; <c>XrFr3WorldSetup</c> builds them from the same file
    /// (visuals only: no collider, no rigidbody). Each stays hidden until its first pose, and all of them while RobotHost runs
    /// another scene file. A thin adapter: the poses are <see cref="WorldState"/>'s, the conversion is <see cref="CoordConversion"/>'s.
    /// </summary>
    public sealed class SerialChainWorldTwin : MonoBehaviour
    {
        [SerializeField] private SerialChainOperatorBridge bridge;
        [Tooltip("The objects' transforms, children of this one, in the scene file's order.")]
        [SerializeField] private Transform[] objects = new Transform[0];
        [Tooltip("Each object's scene id, parallel to objects.")]
        [SerializeField] private int[] ids = new int[0];
        [Tooltip("A see-through copy of each object, parallel to objects: shown where EDIT OBJECTS is dragging it (phase P5).")]
        [SerializeField] private Transform[] ghosts = new Transform[0];

        private bool _warnedMismatch;

        public int ObjectCount => objects.Length;

        private void OnEnable() => Application.onBeforeRender += Pose;

        private void OnDisable() => Application.onBeforeRender -= Pose;

        private void Start()
        {
            foreach (Transform t in objects)
            {
                if (t != null) t.gameObject.SetActive(false);
            }

            foreach (Transform t in ghosts)
            {
                if (t != null) t.gameObject.SetActive(false);
            }
        }

        private void Pose()
        {
            WorldState world = bridge != null ? bridge.World : null;
            if (world == null) return;
            bool mismatch = world.SceneMismatch;
            if (mismatch && !_warnedMismatch)
            {
                Debug.LogWarning("[world] RobotHost runs another scene file than the headset's: the world twin stays hidden (push the same core/Scenes file)");
                _warnedMismatch = true;
            }

            for (int i = 0; i < objects.Length && i < ids.Length; i++)
            {
                Transform t = objects[i];
                if (t == null) continue;
                bool posed = world.TryGet((byte)ids[i], out ObjectPose p, out _) && !mismatch;
                if (t.gameObject.activeSelf != posed) t.gameObject.SetActive(posed);
                if (!posed) continue;
                t.localPosition = new CoreVec(p.X, p.Y, p.Z).ToUnity();
                t.localRotation = CoreQuat.Normalize(new CoreQuat(p.Qx, p.Qy, p.Qz, p.Qw)).ToUnity();
            }

            // EDIT OBJECTS: the dragged object's ghost where the hand puts it. The solid one moves only when the world frames
            // show the move, so the operator sees it cross the network, or be refused.
            byte dragged = bridge.EditDraggedId;
            for (int i = 0; i < ghosts.Length && i < ids.Length; i++)
            {
                Transform g = ghosts[i];
                if (g == null) continue;
                bool show = dragged != 0 && ids[i] == dragged && !mismatch;
                if (g.gameObject.activeSelf != show) g.gameObject.SetActive(show);
                if (!show) continue;
                Teleop.RobotArm.Types.RigidTransform at = bridge.EditPreview;
                g.localPosition = new CoreVec((float)at.X, (float)at.Y, (float)at.Z).ToUnity();
                g.localRotation = CoreQuat.Normalize(new CoreQuat((float)at.Qx, (float)at.Qy, (float)at.Qz, (float)at.Qw)).ToUnity();
            }
        }
    }
}
