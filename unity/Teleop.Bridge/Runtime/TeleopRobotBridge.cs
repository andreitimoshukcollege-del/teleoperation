using Teleop.Core.Pipeline;
using Teleop.Core.Plant;
using Teleop.Core.Transport;
using UnityEngine;
using CorePose = Teleop.Core.Types.Pose;

namespace Teleop.Bridge
{
    /// <summary>
    /// The robot side of the Phase 4 loopback baseline. Reuses Core's own
    /// <see cref="RigidBodyPlant"/> directly rather than a Unity-physics-backed plant -- the
    /// zero-mitigation baseline needs the same headless kinematic dead-reckoner sweeps run
    /// against, per <c>Plant/CLAUDE.md</c>'s own reasoning; a <c>UnityRobotPlant</c> with real
    /// physics is a different, later research question about a second plant implementation, not
    /// a requirement for measuring this baseline.
    ///
    /// Reads <see cref="TeleopOperatorBridge.UplinkTransport"/>/<see cref="TeleopOperatorBridge.DownlinkTransport"/>
    /// in <see cref="Start"/> rather than <c>Awake</c>: Unity guarantees every active object's
    /// <c>Awake</c> runs before any <c>Start</c> in the same frame, so this is guaranteed to see
    /// the transports <see cref="TeleopOperatorBridge"/> constructs in its own <c>Awake</c>. No
    /// statics, no service locator -- just Unity's own ordering guarantee plus a serialized
    /// reference, per <c>core/Teleop.Core/CLAUDE.md</c>'s "no statics with mutable state" style
    /// rule (which exists specifically because Unity wipes statics on domain reload).
    ///
    /// <c>FixedUpdate</c>: drain the uplink into <see cref="RobotEndpoint"/> (which steps the
    /// plant and replies), matching Teleop/CLAUDE.md's "FixedUpdate: digital-twin physics only"
    /// row, then mirror <see cref="RigidBodyPlant.State"/> onto <see cref="groundTruthTarget"/> so
    /// the operator's estimate (the "ghost") and the plant's actual state are visually
    /// side-by-side in the scene.
    /// </summary>
    public sealed class TeleopRobotBridge : MonoBehaviour
    {
        [SerializeField] private TeleopOperatorBridge operatorBridge;
        [SerializeField] private Transform groundTruthTarget;

        private UnityMonotonicClock _clock;
        private RigidBodyPlant _plant;
        private RobotEndpoint _robotEndpoint;

        private void Start()
        {
            _clock = new UnityMonotonicClock();
            _plant = new RigidBodyPlant(CorePose.Identity, _clock.TicksPerSecond);

            _robotEndpoint = new RobotEndpoint(
                _plant,
                new RawPoseCodec(),
                new RobotStateFrameCodec(),
                operatorBridge.UplinkTransport,
                operatorBridge.DownlinkTransport,
                _clock,
                maxDatagramsPerStep: 64);
        }

        private void FixedUpdate()
        {
            _robotEndpoint.Step(_clock.NowTicks);

            if (groundTruthTarget != null)
            {
                _plant.State.Value.ApplyTo(groundTruthTarget);
            }
        }
    }
}
