using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// The real-hardware <see cref="IRobotPlant"/> for the Hiwonder JetRover's arm and gripper
    /// (docs/adr/0007-jetrover-plant-and-robot-host.md). Lives here, in
    /// <c>Teleop.RobotHost</c> -- deliberately not in Unity's <c>Bridge/</c>, since routing a
    /// real robot's commands through Unity would carry them outside Core's own
    /// <c>ITransport</c>/<c>ClockSync</c> instrumentation. Talks to the JetRover's ROS 2 node
    /// only indirectly, through an injected <see cref="IRelayClient"/> -- this class has no idea
    /// ROS exists.
    ///
    /// <b>Gap policy: hold, not <c>RigidBodyPlant</c>'s "coast indefinitely."</b> Between
    /// commands, this plant does nothing further -- no repeated relay traffic, no timeout, no
    /// ramp. That is safe specifically because the JetRover's bus servos already hold their
    /// last commanded position with no repeated command required; "coast indefinitely" is the
    /// right choice for a kinematic abstraction nobody can get hurt by, and the wrong one for a
    /// machine with real motors (see the ADR for the full reasoning).
    ///
    /// <b>Phase 1 scope: no inverse kinematics yet.</b> <see cref="Command"/> maps
    /// <c>CommandFrame.Pose.Position.X</c> directly into the base servo's relative "direction"
    /// units via <see cref="JetRoverPlantConfig.PositionXToDirectionScale"/> -- a deliberately
    /// temporary stand-in to prove the transport-to-hardware pipe end-to-end, not a real
    /// interpretation of a commanded pose. It is only safe for this phase's one-shot smoke test:
    /// because the relay's direction unit is <i>relative</i> to the servo's current position
    /// (matching the existing ROS node's own <c>setPos</c>), sending a nudge on every accepted
    /// frame from a continuous, high-frequency operator stream would compound indefinitely
    /// rather than converge anywhere. Real inverse kinematics (the phase that replaces this
    /// stand-in) is also where "absolute target position" bookkeeping belongs, closing that gap.
    /// Gripper and the lower/middle/upper joints are not wired up at all yet -- Phase 1 exercises
    /// only the base servo, matching what has already been manually verified against the real
    /// hardware.
    ///
    /// <b><see cref="State"/> only ever reports what the hardware actually confirmed</b> -- the
    /// base servo's last known angle, read back from the JetRover's own feedback topic via the
    /// relay. It is never dead-reckoned from a commanded value the way <c>RigidBodyPlant</c>
    /// integrates velocity: if the last feedback read failed (a real, observed occurrence -- the
    /// board's serial read can time out), <see cref="IsBaseDegreesSensed"/> is false and
    /// <see cref="State"/>'s position reports <see cref="float.NaN"/> rather than a stale or
    /// fabricated value. This is not yet a real <see cref="Pose"/> in any meaningful sense (no
    /// forward kinematics exist yet, and only one of four joints is tracked at all) -- it is
    /// reported through <see cref="Pose.Position"/>'s X component purely so the Phase 1 pipe's
    /// round trip is externally observable end-to-end, and must not be read as a real pose by
    /// anything downstream.
    ///
    /// Not thread-safe, by contract. Time is always a parameter, never a clock read, same as
    /// every other <see cref="IRobotPlant"/> implementation.
    /// </summary>
    public sealed class JetRoverPlant : IRobotPlant
    {
        private readonly JetRoverPlantConfig _config;
        private readonly IRelayClient _relay;

        private long _lastAcceptedCaptureTicks;
        private long _stateTicks;
        private bool _baseDegreesSensed;
        private int _lastKnownBaseDegrees;

        public JetRoverPlant(JetRoverPlantConfig config, IRelayClient relay)
        {
            _config = config;
            _relay = relay ?? throw new ArgumentNullException(nameof(relay));
            _lastAcceptedCaptureTicks = long.MinValue;
            _stateTicks = 0;
            _baseDegreesSensed = false;
            _lastKnownBaseDegrees = 0;
        }

        /// <summary>
        /// True once at least one feedback read has confirmed the base servo's real position.
        /// <see cref="IRobotPlant.State"/> has no room to distinguish "sensed" from "unknown" --
        /// this is a plant-specific accessor beyond the interface for exactly that, mirroring
        /// <c>RigidBodyPlant.Gripper</c>'s precedent of a plant-specific accessor where the
        /// shared interface type has no room for a plant-specific concern.
        /// </summary>
        public bool IsBaseDegreesSensed => _baseDegreesSensed;

        /// <summary>
        /// Accepts <paramref name="command"/> as the new setpoint and immediately forwards a
        /// relay nudge -- see the class doc for why this plant sends at command time rather than
        /// deferring to <see cref="Step"/> the way <c>RigidBodyPlant</c> does: the relay's
        /// direction unit is a relative step, not an absolute setpoint, so there is nothing for
        /// <see cref="Step"/> to integrate toward. Stale or duplicate frames (by
        /// <see cref="CommandFrame.CaptureTicks"/>) are rejected whole, never partially applied,
        /// per <see cref="IRobotPlant.Command"/>. Allocation-free.
        /// </summary>
        public void Command(in CommandFrame command)
        {
            if (command.CaptureTicks <= _lastAcceptedCaptureTicks)
            {
                return;
            }

            _lastAcceptedCaptureTicks = command.CaptureTicks;

            float direction = command.Pose.Position.X * _config.PositionXToDirectionScale;
            direction = Math.Clamp(direction, -_config.MaxDirectionMagnitude, _config.MaxDirectionMagnitude);

            _relay.Send(new LocalArmCommand(direction));
        }

        /// <summary>
        /// Advances <see cref="State"/>'s stamp to <paramref name="nowTicks"/> and polls the
        /// relay once for fresh feedback. A step at or before the current state time is a no-op.
        /// There is nothing else to do between commands -- see the class doc's gap-policy note.
        /// Allocation-free.
        /// </summary>
        public void Step(long nowTicks)
        {
            if (nowTicks <= _stateTicks)
            {
                return;
            }

            _stateTicks = nowTicks;

            if (_relay.TryReceiveFeedback(out LocalFeedback feedback) && feedback.BaseDegreesValid)
            {
                _lastKnownBaseDegrees = feedback.BaseDegrees;
                _baseDegreesSensed = true;
            }
        }

        /// <inheritdoc/>
        public Stamped<Pose> State
        {
            get
            {
                float x = _baseDegreesSensed ? _lastKnownBaseDegrees : float.NaN;
                return new Stamped<Pose>(_stateTicks, new Pose(new Vector3(x, 0f, 0f), Quaternion.Identity));
            }
        }

        /// <summary>
        /// Deliberately does <b>not</b> command any hardware motion, unlike
        /// <see cref="IRobotPlant.Reset"/>'s literal doc ("returns the plant to its
        /// as-constructed state"), which describes a costless teleport a kinematic plant can
        /// honor and a physical arm cannot -- see
        /// docs/adr/0007-jetrover-plant-and-robot-host.md. Clears only this plant's own
        /// bookkeeping: the staleness baseline (back to <see cref="long.MinValue"/>, not zero,
        /// same reasoning as <c>RigidBodyPlant.Reset</c>), the state timestamp, and the sensed
        /// feedback flag.
        /// </summary>
        public void Reset()
        {
            _lastAcceptedCaptureTicks = long.MinValue;
            _stateTicks = 0;
            _baseDegreesSensed = false;
            _lastKnownBaseDegrees = 0;
        }
    }
}
