using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Plant
{
    /// <summary>
    /// The Core-side <see cref="IRobotPlant"/>: a kinematic dead-reckoner that sweeps and replay
    /// run against. Headless, deterministic, allocation-free, and a pure function of
    /// (commands, step times).
    ///
    /// <b>Kinematic, not physical, and deliberately so.</b> There is no mass, no force, no
    /// actuator lag, no collision. <see cref="Command"/> snaps the pose one-to-one onto the
    /// commanded setpoint and <see cref="Step"/> dead-reckons forward on the last commanded
    /// velocity. Nothing here filters, damps, or eases. That is the point: Phase 4 (docs/setup.md)
    /// is the explicit "zero mitigation" baseline, and a spring/PD/critically-damped tracker is
    /// itself a smoothing behaviour — it would suppress exactly the correction cost the baseline
    /// exists to measure, and every mitigation later compared against that baseline would be
    /// scored against a plant that was already quietly mitigating.
    ///
    /// <b>Gap policy: coast indefinitely on the last commanded velocity.</b>
    /// <see cref="IRobotPlant.Command"/> requires a plant to document how it behaves through a
    /// gap, and names three options — hold, coast, ramp to a stop. This one coasts, with no
    /// timeout and no ramp. It is the simplest of the three (no additional parameter, no hidden
    /// time constant to sweep, no second behaviour regime to explain in a result), and it is the
    /// one that actually exercises <see cref="CommandFrame.LinearVelocity"/> and
    /// <see cref="CommandFrame.AngularVelocity"/> for their stated purpose: intent is what
    /// survives a lost packet. A plant that held position through a gap would make those fields
    /// dead weight and would understate what packet loss looks like to an operator.
    ///
    /// <b>No config struct, deliberately.</b> Same reasoning as
    /// <see cref="Teleop.Core.Transport.LoopbackTransport"/>: this class has no research knobs.
    /// Its three constructor parameters are initial conditions and a timebase, not parameters
    /// anyone would sweep, and no result would ever be reported "at ticksPerSecond 10000". A
    /// <c>RigidBodyPlantConfig</c> would be ceremony implying a tunable family of behaviours where
    /// there is exactly one fixed behaviour. This is a considered choice, not an oversight.
    ///
    /// <b>Two tick domains, never compared.</b> <see cref="CommandFrame.CaptureTicks"/> is on the
    /// <i>operator's</i> <c>ITimeAuthority</c>; <c>nowTicks</c> passed to <see cref="Step"/> is on
    /// whichever timebase the host drives the plant with. This class keeps a separate field for
    /// each and never compares one against the other, even though both are <c>long</c> ticks.
    /// Relating the two domains is <c>Time/ClockSync.cs</c>'s job, not the plant's.
    ///
    /// Not thread-safe, by contract. Time is a parameter, never a clock read.
    /// </summary>
    public sealed class RigidBodyPlant : IRobotPlant
    {
        /// <summary>
        /// Below this angular speed (radians/second) the rotation integration is skipped entirely
        /// rather than normalizing a zero axis. Not a research knob and not a dead band on
        /// operator input: it exists only because the rotation axis is undefined for a zero rate
        /// vector. Small enough that any rate a real operator or codec produces integrates
        /// normally — at this rate a full revolution takes longer than the age of the universe.
        /// </summary>
        private const float AngularRateEpsilon = 1e-12f;

        private readonly Pose _initialPose;
        private readonly long _ticksPerSecond;
        private readonly long _initialStateTicks;

        /// <summary>The plant's actual position, metres, ROS convention.</summary>
        private Vector3 _position;

        /// <summary>The plant's actual orientation, ROS convention.</summary>
        private Quaternion _rotation;

        /// <summary>Last commanded linear velocity, metres/second. Coasted on through gaps.</summary>
        private Vector3 _linearVelocity;

        /// <summary>
        /// Last commanded angular velocity as an axis-angle rate vector (direction is the axis,
        /// magnitude is the rate in radians/second), matching
        /// <see cref="CommandFrame.AngularVelocity"/>'s convention exactly. Coasted on through gaps.
        /// </summary>
        private Vector3 _angularVelocity;

        /// <summary>Last commanded gripper, 0 = fully open, 1 = fully closed.</summary>
        private float _gripper;

        /// <summary>
        /// The tick <see cref="State"/> is valid at, on the host's <c>Step</c> timebase. Advanced
        /// only by <see cref="Step"/>.
        /// </summary>
        private long _stateTicks;

        /// <summary>
        /// Highest <see cref="CommandFrame.CaptureTicks"/> accepted so far, on the <i>operator's</i>
        /// timebase. The staleness baseline, and a distinct quantity from <see cref="_stateTicks"/>
        /// — the two are never compared against each other. Starts at <see cref="long.MinValue"/>
        /// so that the very first command is accepted whatever its stamp, including zero.
        /// </summary>
        private long _lastAcceptedCaptureTicks;

        /// <param name="initialPose">
        /// Pose the plant starts at and that <see cref="Reset"/> returns it to.
        /// </param>
        /// <param name="ticksPerSecond">
        /// Ticks per second of the timebase <see cref="Step"/> is driven on; converts a tick delta
        /// into the seconds the integration needs.
        /// </param>
        /// <param name="initialStateTicks">
        /// Tick the initial pose is valid at, and the value <see cref="Reset"/> restores
        /// <see cref="State"/>'s stamp to. Defaults to zero.
        /// </param>
        public RigidBodyPlant(Pose initialPose, long ticksPerSecond, long initialStateTicks = 0)
        {
            if (ticksPerSecond <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(ticksPerSecond), ticksPerSecond, "Ticks per second must be positive.");
            }

            _initialPose = initialPose;
            _ticksPerSecond = ticksPerSecond;
            _initialStateTicks = initialStateTicks;

            _position = initialPose.Position;
            _rotation = initialPose.Rotation;
            _linearVelocity = Vector3.Zero;
            _angularVelocity = Vector3.Zero;
            _gripper = 0f;
            _stateTicks = initialStateTicks;
            _lastAcceptedCaptureTicks = long.MinValue;
        }

        /// <inheritdoc/>
        public Stamped<Pose> State => new Stamped<Pose>(_stateTicks, new Pose(_position, _rotation));

        /// <summary>
        /// Current gripper command, 0 = fully open, 1 = fully closed.
        /// <see cref="IRobotPlant.State"/> is typed <see cref="Stamped{T}"/> of <see cref="Pose"/>
        /// and has no room for a gripper, so this is a plant-specific accessor rather than a
        /// widening of the contract: the alternative — adding a gripper to every plant's state
        /// type — would push a manipulator-specific concern into an interface that also fronts
        /// Unity physics and real hardware. Exists so gripper passthrough is locally testable.
        /// </summary>
        public float Gripper => _gripper;

        /// <summary>
        /// Ticks per second of the <see cref="Step"/> timebase this plant integrates against.
        /// </summary>
        public long TicksPerSecond => _ticksPerSecond;

        /// <summary>
        /// Applies <paramref name="command"/> as the current setpoint: pose snaps one-to-one (no
        /// filtering — see the type doc on why), and the velocity and gripper fields are stored
        /// for the coast that follows.
        ///
        /// A frame whose <see cref="CommandFrame.CaptureTicks"/> is at or below the last accepted
        /// one is stale or a duplicate and is ignored <b>entirely</b> — not partially applied —
        /// rather than jerking the plant backwards to it, per
        /// <see cref="IRobotPlant.Command"/>. Equal stamps are rejected too: a repeat of the frame
        /// already held carries no new information, and treating it as new would let a duplicated
        /// datagram restart a coast the plant had already integrated past.
        ///
        /// Does not advance the simulation — <see cref="Step"/> does — so <see cref="State"/>'s
        /// stamp is unchanged by this call even though its pose may not be. Allocation-free.
        /// </summary>
        public void Command(in CommandFrame command)
        {
            if (command.CaptureTicks <= _lastAcceptedCaptureTicks)
            {
                return;
            }

            _lastAcceptedCaptureTicks = command.CaptureTicks;
            _position = command.Pose.Position;
            _rotation = command.Pose.Rotation;
            _linearVelocity = command.LinearVelocity;
            _angularVelocity = command.AngularVelocity;
            _gripper = command.Gripper;
        }

        /// <summary>
        /// Dead-reckons forward to <paramref name="nowTicks"/> by semi-implicit Euler on the last
        /// commanded velocities. A step at or before the current state time is a no-op, so a
        /// duplicate or out-of-order step cannot double-integrate or move time backwards.
        ///
        /// Rotation integrates in the <b>world</b> frame: the delta built from the axis-angle rate
        /// vector is pre-multiplied onto the current orientation, so the commanded axis is fixed
        /// in the world, not carried around by the body. The result is renormalized every step
        /// because thousands of quaternion products otherwise drift off the unit sphere.
        /// Allocation-free.
        /// </summary>
        public void Step(long nowTicks)
        {
            if (nowTicks <= _stateTicks)
            {
                return;
            }

            // A tick *delta* between two steps, not an absolute tick count, so float carries it
            // exactly at any session length a step schedule realistically produces.
            float dt = (float)(nowTicks - _stateTicks) / _ticksPerSecond;

            _position += _linearVelocity * dt;

            float angularSpeed = _angularVelocity.Length();
            if (angularSpeed > AngularRateEpsilon)
            {
                // Divide by the length already computed rather than calling Vector3.Normalize,
                // which would repeat the square root.
                Vector3 axis = _angularVelocity / angularSpeed;
                Quaternion delta = Quaternion.CreateFromAxisAngle(axis, angularSpeed * dt);
                _rotation = Quaternion.Normalize(delta * _rotation);
            }

            _stateTicks = nowTicks;
        }

        /// <summary>
        /// Returns the plant to its as-constructed state: the constructor's initial pose and state
        /// tick, zero velocity, open gripper, and — the one that is easy to miss — the staleness
        /// baseline back to <see cref="long.MinValue"/> rather than zero. Sweeps reuse instances
        /// across trials, and a plant that remembered the previous trial's highest
        /// <see cref="CommandFrame.CaptureTicks"/> would silently ignore the whole opening stretch
        /// of the next trial, which looks like a stuck robot rather than like a bug.
        /// </summary>
        public void Reset()
        {
            _position = _initialPose.Position;
            _rotation = _initialPose.Rotation;
            _linearVelocity = Vector3.Zero;
            _angularVelocity = Vector3.Zero;
            _gripper = 0f;
            _stateTicks = _initialStateTicks;
            _lastAcceptedCaptureTicks = long.MinValue;
        }
    }
}
