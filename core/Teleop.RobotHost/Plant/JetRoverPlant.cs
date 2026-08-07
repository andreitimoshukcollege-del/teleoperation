using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;
using Teleop.RobotHost.Kinematics;
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
    /// last commanded position with no repeated command required.
    ///
    /// <b>Real inverse kinematics (<see cref="FourDofArmKinematics"/>), replacing Phase 1's
    /// stand-in.</b> <see cref="Command"/> runs IK against <c>CommandFrame.Pose</c> to get a
    /// target angle for each of the four position-affecting joints, converts each target into a
    /// pulse value, and sends the *delta* from this plant's own tracked target (not from sensed
    /// feedback -- see "optimistic target tracking" below) as the relay's relative "direction"
    /// unit, since the underlying ROS topics are relative-step, not absolute-setpoint, and
    /// changing that is out of scope here. <see cref="FourDofArmKinematics"/>'s own doc covers
    /// what this model does and doesn't represent (roll/yaw dropped, target point is the wrist
    /// not the gripper fingertip, ruler-measured link lengths).
    ///
    /// <b>Optimistic target tracking.</b> This plant tracks two separate beliefs per joint: the
    /// pulse value it last *commanded* (updated in <see cref="Command"/> by the amount actually
    /// sent -- see the note on clamping below, not the unclamped IK target) and the pulse value
    /// last *sensed* from real feedback (updated only in <see cref="Step"/>, when a feedback read
    /// actually succeeds). A new command's delta is computed against the commanded belief, not
    /// the sensed one -- feedback lags physical motion by at least one relay round trip, so
    /// composing consecutive commands against stale sensed data would under- or over-shoot.
    ///
    /// A single command's required delta can exceed <see cref="JetRoverPlantConfig.MaxDirectionMagnitude"/>;
    /// when it does, only the clamped amount is actually asked of the hardware, and the belief
    /// update must track that same clamped amount -- crediting the full unclamped target here
    /// was a real bug found during Phase 2's own hardware testing (docs/adr/0007-jetrover-plant-and-robot-host.md):
    /// it made this plant believe a large move had already landed in one step when it hadn't,
    /// silently stalling the remaining distance forever, since a repeated identical
    /// <see cref="CommandFrame"/> would then compute a zero delta against a belief that was never
    /// true. Accumulating only the applied delta lets repeated/continuous commands toward the
    /// same real-world target keep closing the gap over successive calls instead.
    ///
    /// <b><see cref="State"/> is forward kinematics, preferring sensed angles but falling back to
    /// the commanded target for any joint that has never been sensed</b> (feedback invalid or
    /// none received yet) -- a real, observed case on this exact hardware: the middle-arm servo
    /// never responds to position-read requests at all, confirmed independently of ROS (writes to
    /// it work fine; only reads never succeed). Reporting that joint's contribution as though it
    /// were still at its power-on default while every other joint shows real progress would be a
    /// worse estimate than using the last thing this plant actually told it to do.
    /// <see cref="IsFullySensed"/> is what tells a caller whether to trust <see cref="State"/> as
    /// a measurement rather than partly an estimate -- it is always computed, never withheld,
    /// purely so a partial-sensing pipe issue is visible in the output rather than the whole
    /// state silently disappearing.
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

        // Optimistic targets -- what this plant last commanded, assumed to have been honored.
        private float _targetPulseBase;
        private float _targetPulseLower;
        private float _targetPulseMiddle;
        private float _targetPulseUpper;

        // Sensed beliefs -- only ever updated from real feedback in Step().
        private bool _baseSensed, _lowerSensed, _middleSensed, _upperSensed;
        private float _sensedPulseBase, _sensedPulseLower, _sensedPulseMiddle, _sensedPulseUpper;

        public JetRoverPlant(JetRoverPlantConfig config, IRelayClient relay)
        {
            _config = config;
            _relay = relay ?? throw new ArgumentNullException(nameof(relay));
            ResetInternal();
        }

        /// <summary>True once every one of the four position-affecting joints has been sensed at least once.</summary>
        public bool IsFullySensed => _baseSensed && _lowerSensed && _middleSensed && _upperSensed;

        /// <summary>
        /// Runs inverse kinematics against <paramref name="command"/>'s pose, converts each
        /// joint's target angle into a relay direction delta from this plant's own optimistic
        /// target belief, denormalizes the gripper, and sends the result. Stale or duplicate
        /// frames (by <see cref="CommandFrame.CaptureTicks"/>) are rejected whole, never
        /// partially applied, per <see cref="IRobotPlant.Command"/>.
        /// </summary>
        public void Command(in CommandFrame command)
        {
            if (command.CaptureTicks <= _lastAcceptedCaptureTicks)
            {
                return;
            }

            _lastAcceptedCaptureTicks = command.CaptureTicks;

            FourDofArmKinematics.TryInverse(
                _config.Links, command.Pose.Position,
                out float baseYaw, out float lowerPitch, out float middlePitch);
            float desiredPitch = FourDofArmKinematics.ExtractPitchRadians(command.Pose.Rotation);
            float upperPitch = FourDofArmKinematics.InverseUpperPitch(lowerPitch, middlePitch, desiredPitch);

            float targetPulseBase = ClampPulse(_config.ZeroPulse + baseYaw * _config.PulsePerRadian);
            float targetPulseLower = ClampPulse(_config.ZeroPulse + lowerPitch * _config.PulsePerRadian);
            float targetPulseMiddle = ClampPulse(_config.ZeroPulse + middlePitch * _config.PulsePerRadian);
            float targetPulseUpper = ClampPulse(_config.ZeroPulse + upperPitch * _config.PulsePerRadian);

            // Direction is clamped to MaxDirectionMagnitude below -- when a single command's
            // required delta exceeds that clamp, only the CLAMPED amount is actually asked of
            // the hardware. The belief update must track that same clamped amount, not the
            // unclamped IK target: crediting the full target here would make this plant think a
            // large move already landed in one step when it didn't, silently stalling out the
            // remaining distance forever (repeating the same CommandFrame would keep computing a
            // zero delta against a belief that was never true). Accumulating the actually-applied
            // delta instead lets repeated/continuous commands toward the same real-world target
            // keep closing the gap over successive calls, the same way the relay's own
            // fire-and-forget "here is the current setpoint" semantics already assume.
            float baseDirection = ToDirection(targetPulseBase - _targetPulseBase);
            float lowerDirection = ToDirection(targetPulseLower - _targetPulseLower);
            float middleDirection = ToDirection(targetPulseMiddle - _targetPulseMiddle);
            float upperDirection = ToDirection(targetPulseUpper - _targetPulseUpper);

            _targetPulseBase += baseDirection * _config.StepSizePulses;
            _targetPulseLower += lowerDirection * _config.StepSizePulses;
            _targetPulseMiddle += middleDirection * _config.StepSizePulses;
            _targetPulseUpper += upperDirection * _config.StepSizePulses;

            float gripperFraction = Math.Clamp(command.Gripper, 0f, 1f);
            float gripperDegrees = _config.GripperOpenDegrees
                + gripperFraction * (_config.GripperClosedDegrees - _config.GripperOpenDegrees);

            _relay.Send(new LocalArmCommand(baseDirection, lowerDirection, middleDirection, upperDirection, gripperDegrees));
        }

        /// <summary>
        /// Advances <see cref="State"/>'s stamp to <paramref name="nowTicks"/> and polls the
        /// relay once for fresh feedback, updating whichever joints' sensed beliefs the feedback
        /// reports as valid. A step at or before the current state time is a no-op. Allocation-free.
        /// </summary>
        public void Step(long nowTicks)
        {
            if (nowTicks <= _stateTicks)
            {
                return;
            }

            _stateTicks = nowTicks;

            if (!_relay.TryReceiveFeedback(out LocalFeedback feedback))
            {
                return;
            }

            if (feedback.Base.Valid)
            {
                _sensedPulseBase = DegreesToPulse(feedback.Base.Degrees);
                _baseSensed = true;
            }

            if (feedback.Lower.Valid)
            {
                _sensedPulseLower = DegreesToPulse(feedback.Lower.Degrees);
                _lowerSensed = true;
            }

            if (feedback.Middle.Valid)
            {
                _sensedPulseMiddle = DegreesToPulse(feedback.Middle.Degrees);
                _middleSensed = true;
            }

            if (feedback.Upper.Valid)
            {
                _sensedPulseUpper = DegreesToPulse(feedback.Upper.Degrees);
                _upperSensed = true;
            }
        }

        /// <inheritdoc/>
        public Stamped<Pose> State
        {
            get
            {
                // A joint that has never been sensed (real, observed occurrence -- this
                // hardware's middle-arm servo never responds to position-read requests at all,
                // confirmed independently of ROS; writes to it work fine) falls back to this
                // plant's own last-commanded target rather than a fixed default. That is still
                // an estimate, not a measurement -- IsFullySensed is what tells a caller whether
                // to trust this -- but it is a far better estimate than pretending the joint
                // never left its power-on default while every other joint reports real progress.
                float baseYaw = PulseToRadians(_baseSensed ? _sensedPulseBase : _targetPulseBase);
                float lowerPitch = PulseToRadians(_lowerSensed ? _sensedPulseLower : _targetPulseLower);
                float middlePitch = PulseToRadians(_middleSensed ? _sensedPulseMiddle : _targetPulseMiddle);
                Vector3 position = FourDofArmKinematics.Forward(_config.Links, baseYaw, lowerPitch, middlePitch);
                return new Stamped<Pose>(_stateTicks, new Pose(position, Quaternion.Identity));
            }
        }

        /// <summary>
        /// Deliberately does <b>not</b> command any hardware motion, unlike
        /// <see cref="IRobotPlant.Reset"/>'s literal doc ("returns the plant to its
        /// as-constructed state"), which describes a costless teleport a kinematic plant can
        /// honor and a physical arm cannot -- see
        /// docs/adr/0007-jetrover-plant-and-robot-host.md. Clears only this plant's own
        /// bookkeeping.
        /// </summary>
        public void Reset() => ResetInternal();

        private void ResetInternal()
        {
            _lastAcceptedCaptureTicks = long.MinValue;
            _stateTicks = 0;
            _targetPulseBase = _config.ZeroPulse;
            _targetPulseLower = _config.ZeroPulse;
            _targetPulseMiddle = _config.ZeroPulse;
            _targetPulseUpper = _config.ZeroPulse;
            _baseSensed = _lowerSensed = _middleSensed = _upperSensed = false;
            _sensedPulseBase = _sensedPulseLower = _sensedPulseMiddle = _sensedPulseUpper = _config.ZeroPulse;
        }

        private float ClampPulse(float pulse) => Math.Clamp(pulse, _config.MinPulse, _config.MaxPulse);

        private float ToDirection(float pulseDelta) =>
            Math.Clamp(pulseDelta / _config.StepSizePulses, -_config.MaxDirectionMagnitude, _config.MaxDirectionMagnitude);

        private float PulseToRadians(float pulse) => (pulse - _config.ZeroPulse) / _config.PulsePerRadian;

        /// <summary>
        /// Reverses the ROS SDK's own <c>pulseToDeg</c> conversion (180-degree assumption) to
        /// recover the pulse value a feedback reading's degrees came from -- see
        /// <see cref="JetRoverPlantConfig.PulsePerDegreeAssumed180"/>'s doc for why this must not
        /// use the corrected 240-degree range. Not perfectly exact: <c>pulseToDeg</c> truncates
        /// to an integer degree before publishing, so up to ~1 degree (a few pulse units) of
        /// quantization is already baked into <paramref name="degrees"/> before this ever sees it.
        /// </summary>
        private float DegreesToPulse(int degrees) => degrees * _config.PulsePerDegreeAssumed180;
    }
}
