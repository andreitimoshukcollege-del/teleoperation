using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// The real-hardware <see cref="IRobotPlant"/> for a robot arm described by a
    /// <see cref="RobotArmProfile"/> (docs/adr/0011-generic-robot-arm-profiles.md, generalizing
    /// docs/adr/0007's JetRover-only <c>JetRoverPlant</c>). Lives here, in
    /// <c>Teleop.RobotHost</c> -- deliberately not in Unity's <c>Bridge/</c>, since routing a
    /// real robot's commands through Unity would carry them outside Core's own
    /// <c>ITransport</c>/<c>ClockSync</c> instrumentation. Talks to the robot's ROS 2 node only
    /// indirectly, through an injected <see cref="IRelayClient"/> -- this class has no idea ROS
    /// exists.
    ///
    /// <b>Per-joint state is array-indexed, not named fields.</b> Every joint (base yaw, the two
    /// position-solving links, however many wrist joints, the gripper) gets one slot in each of
    /// <see cref="_targetPulse"/>/<see cref="_targetSeeded"/>/<see cref="_sensed"/>/
    /// <see cref="_sensedPulse"/>, indexed the same way <see cref="RobotArmProfile.Joints"/> is --
    /// this is the one structural change from <c>JetRoverPlant</c>; every rule below (gap policy,
    /// belief seeding, per-call clamping) is unchanged, just generalized from 4 named fields to
    /// N array slots.
    ///
    /// <b>Gap policy: hold, not <c>RigidBodyPlant</c>'s "coast indefinitely."</b> Between
    /// commands, this plant does nothing further -- no repeated relay traffic, no timeout, no
    /// ramp. That is safe specifically because these bus servos already hold their last commanded
    /// position with no repeated command required.
    ///
    /// <b>Real inverse kinematics (<see cref="ArmKinematics"/>).</b> <see cref="Command"/> runs IK
    /// against <c>CommandFrame.Pose</c> to get a target angle for each position-affecting joint,
    /// maps it through <see cref="ArmKinematics.MapAnglesToJointTargets"/>, then converts each
    /// target into a pulse value. The relay wire itself carries an *absolute* pulse target per
    /// joint (docs/adr/0010, generalized by docs/adr/0011 to a motor-id-keyed array) -- internally,
    /// this plant still advances its own tracked target by a clamped step each call (see
    /// "optimistic target tracking" below) rather than jumping straight to the IK output, but what
    /// crosses the wire is the resulting absolute belief, not the step used to reach it. The
    /// gripper is the one exception: open-loop, sent as an absolute target every call with no
    /// per-cycle stepping, same as before.
    ///
    /// <b>Optimistic target tracking.</b> This plant tracks two separate beliefs per joint: the
    /// pulse value it last *commanded* (updated in <see cref="Command"/>/<see cref="CommandJointAngles"/>
    /// by the amount actually sent, not the unclamped IK target) and the pulse value last *sensed*
    /// from real feedback (updated only in <see cref="Step"/>, when a feedback read actually
    /// succeeds). A new command's delta is computed against the commanded belief, not the sensed
    /// one -- feedback lags physical motion by at least one relay round trip, so composing
    /// consecutive commands against stale sensed data would under- or over-shoot.
    ///
    /// The commanded belief is seeded from sensed data exactly once, the first time each joint's
    /// belief is used, rather than always starting at <see cref="GenericArmPlantConfig.ZeroPulse"/>
    /// -- <see cref="Step"/> polls the relay from startup regardless of whether any command has
    /// arrived, so by the time the first real command lands the real sensed position is usually
    /// already known and does not necessarily match <c>ZeroPulse</c> (e.g. after a restart
    /// mid-session). Only the very first use per joint seeds; every use after that stays purely
    /// optimistic, per the no-stale-sensed-data rule above.
    ///
    /// A single command's required delta can exceed <see cref="GenericArmPlantConfig.MaxDirectionMagnitude"/>;
    /// when it does, only the clamped amount is actually asked of the hardware, and the belief
    /// update must track that same clamped amount -- crediting the full unclamped target would
    /// make this plant believe a large move had already landed in one step when it hadn't,
    /// silently stalling the remaining distance forever, since a repeated identical
    /// <see cref="CommandFrame"/> would then compute a zero delta against a belief that was never
    /// true. Accumulating only the applied delta lets repeated/continuous commands toward the
    /// same real-world target keep closing the gap over successive calls instead.
    ///
    /// <b><see cref="State"/> is forward kinematics, preferring sensed angles but falling back to
    /// the commanded target for any joint that has never been sensed</b> (feedback invalid or none
    /// received yet) -- a real, observed case on the JetRover hardware this generalizes from: one
    /// servo never responds to position-read requests at all (writes work fine; only reads never
    /// succeed). Reporting that joint's contribution as though it were still at its power-on
    /// default while every other joint shows real progress would be a worse estimate than using
    /// the last thing this plant actually told it to do. <see cref="IsFullySensed"/> is what tells
    /// a caller whether to trust <see cref="State"/> as a measurement rather than partly an
    /// estimate -- it excludes the gripper (which never has feedback wired up, by design, both
    /// before and after this generalization) and is always computed, never withheld.
    ///
    /// Not thread-safe, by contract. Time is always a parameter, never a clock read, same as
    /// every other <see cref="IRobotPlant"/> implementation.
    /// </summary>
    public sealed class GenericArmPlant : IRobotPlant
    {
        private readonly GenericArmPlantConfig _config;
        private readonly IRelayClient _relay;
        private readonly RobotArmProfile _profile;

        // Separate staleness trackers for Command and CommandJointAngles -- these are two
        // genuinely independent command streams (Cartesian-target callers like move-arm/
        // clocksync-check vs. pre-computed-angle callers like the operator VR feature), and a
        // real caller can legitimately stamp both with the identical CaptureTicks in the same
        // tick. A single shared tracker treated that as a stale duplicate and silently dropped
        // whichever command lost the race -- found via real-hardware testing (2026-08-12).
        private long _lastAcceptedCartesianCaptureTicks;
        private long _lastAcceptedJointCaptureTicks;

        // The Cartesian path's IK (docs/adr/0020): collision-aware when the profile has a model,
        // seeded from this plant's belief the first time it is used.
        private ArmSolver _solver;
        private bool _solverSeeded;

        // When the joint path last delivered a command (this plant's clock), for the joint path's
        // precedence over the Cartesian one (see JointPrecedenceTicks).
        private long _lastJointCommandStateTicks;
        private bool _hadJointCommand;

        // The servo speed actually sent to the relay (pulses/s): the joint path's latest requested
        // speed, never above GenericArmPlantConfig.PulsesPerSecond (docs/adr/0018). Both paths use it.
        private float _pulsesPerSecond;
        private long _stateTicks;

        // Per-joint state, indexed the same way RobotArmProfile.Joints is.
        private readonly float[] _targetPulse;
        private readonly bool[] _targetSeeded;
        private readonly bool[] _sensed;
        private readonly float[] _sensedPulse;

        // Resolved once at construction from each joint's optional per-joint override
        // (falling back to the config's global MinPulse/MaxPulse), or from the gripper's own
        // open/closed range for the gripper's open/close role -- see the constructor.
        private readonly float[] _minPulse;
        private readonly float[] _maxPulse;

        public GenericArmPlant(GenericArmPlantConfig config, IRelayClient relay)
        {
            _config = config;
            _relay = relay ?? throw new ArgumentNullException(nameof(relay));
            _profile = config.Profile;

            string? validationError = _profile.Validate();
            if (validationError != null)
            {
                throw new ArgumentException(
                    $"GenericArmPlant: profile '{_profile.Name}' is invalid: {validationError}", nameof(config));
            }

            int n = _profile.JointCount;
            _targetPulse = new float[n];
            _targetSeeded = new bool[n];
            _sensed = new bool[n];
            _sensedPulse = new float[n];
            _minPulse = new float[n];
            _maxPulse = new float[n];

            for (int i = 0; i < n; i++)
            {
                JointHardwareSpec joint = _profile.Joints[i];
                if (joint.Role == JointRole.GripperMain)
                {
                    _minPulse[i] = MathF.Min(_config.GripperOpenPulse, _config.GripperClosedPulse);
                    _maxPulse[i] = MathF.Max(_config.GripperOpenPulse, _config.GripperClosedPulse);
                }
                else
                {
                    _minPulse[i] = joint.MinAngleRadians.HasValue
                        ? _config.ZeroPulse + joint.MinAngleRadians.Value * _config.PulsePerRadian
                        : _config.MinPulse;
                    _maxPulse[i] = joint.MaxAngleRadians.HasValue
                        ? _config.ZeroPulse + joint.MaxAngleRadians.Value * _config.PulsePerRadian
                        : _config.MaxPulse;
                }
            }

            ResetInternal();
        }

        /// <summary>True once every position-affecting joint (every joint except the gripper, which never has feedback wired up) has been sensed at least once.</summary>
        public bool IsFullySensed
        {
            get
            {
                for (int i = 0; i < _profile.JointCount; i++)
                {
                    JointRole role = _profile.Joints[i].Role;
                    // The wrist roll stays out too, as it always was: whether the relay reports
                    // feedback for it has never been checked on the hardware, and requiring it
                    // could block a robot that worked before.
                    if (role == JointRole.GripperMain || role == JointRole.GripperRotate)
                    {
                        continue;
                    }

                    if (!_sensed[i])
                    {
                        return false;
                    }
                }

                return true;
            }
        }

        /// <summary>
        /// A new sender on the Cartesian path: the staleness check starts over. CaptureTicks are the
        /// sender's own clock, so another machine's, or the same machine's after a reboot, are not
        /// comparable with the last sender's. Without this, an operator whose clock reads lower than
        /// the previous one had every command dropped as stale until RobotHost restarted (found on
        /// hardware 2026-10-13: the Galaxy XR rebooted mid-session and the arm froze). Reordered
        /// datagrams from one sender are still dropped. Moves nothing.
        /// </summary>
        public void RestartCartesianCommandOrdering() => _lastAcceptedCartesianCaptureTicks = long.MinValue;

        /// <summary>A new sender on the joint path: as <see cref="RestartCartesianCommandOrdering"/>.</summary>
        public void RestartJointCommandOrdering() => _lastAcceptedJointCaptureTicks = long.MinValue;

        /// <summary>
        /// Runs inverse kinematics against <paramref name="command"/>'s pose, then applies the
        /// result exactly as <see cref="CommandJointAngles"/> does. Stale or duplicate frames (by
        /// <see cref="CommandFrame.CaptureTicks"/>) are rejected whole, never partially applied,
        /// per <see cref="IRobotPlant.Command"/>.
        /// </summary>
        public void Command(in CommandFrame command)
        {
            if (command.CaptureTicks <= _lastAcceptedCartesianCaptureTicks)
            {
                return;
            }

            _lastAcceptedCartesianCaptureTicks = command.CaptureTicks;

            // The headset sends both paths: its own IK's joint angles, and the Cartesian target for
            // timing. Each side's solver remembers its own history (docs/adr/0020), so the two can
            // pick different poses, and the arm would alternate between them. While joint commands
            // arrive, they alone move the arm.
            if (JointPrecedenceTicks > 0 && _hadJointCommand && _stateTicks - _lastJointCommandStateTicks < JointPrecedenceTicks)
            {
                IgnoredCartesianCommands++;
                return;
            }

            SeedBeliefsFromSensedIfNeeded();

            float desiredPitch = ArmKinematics.ExtractPitchRadians(command.Pose.Rotation);
            float desiredRoll = ArmKinematics.ExtractRollRadians(command.Pose.Rotation);
            Span<float> wristPitches = stackalloc float[_profile.WristJointCount];
            float baseYaw, proximalPitch, distalPitch;
            if (_profile.Collision != null && _profile.WristJointCount == 1)
            {
                // The same solver the headset uses: it never asks for a pose that hits the robot,
                // and stops at the body instead (docs/adr/0020).
                if (!_solverSeeded)
                {
                    _solver.Reset(_profile, BeliefPose());
                    _solverSeeded = true;
                }

                ArmSolution solution = _solver.Solve(_profile, command.Pose.Position, desiredPitch, desiredRoll);
                baseYaw = solution.Pose.BaseYaw;
                proximalPitch = solution.Pose.Proximal;
                distalPitch = solution.Pose.Distal;
                wristPitches[0] = solution.Pose.Wrist;
                desiredRoll = solution.Pose.Roll;
            }
            else
            {
                ArmKinematics.TryInverse(
                    _profile, command.Pose.Position, desiredPitch,
                    out baseYaw, out proximalPitch, out distalPitch, wristPitches, out _);
            }

            Span<JointTarget> ikTargets = stackalloc JointTarget[_profile.JointCount];
            int count = ArmKinematics.MapAnglesToJointTargets(
                _profile, baseYaw, proximalPitch, distalPitch, wristPitches, command.Gripper, desiredRoll, ikTargets);

            ApplyJointTargets(ikTargets.Slice(0, count));
        }

        /// <summary>
        /// Applies already-computed joint targets directly, skipping
        /// <see cref="ArmKinematics.TryInverse"/> entirely -- for a caller that ran inverse
        /// kinematics itself (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md, motivated
        /// by the Jetson's weak CPU under an interactive command rate far higher than
        /// <c>move-arm</c>'s occasional use). <paramref name="targets"/>'s <c>Angle</c> is radians
        /// for arm joints and a 0-1 fraction for gripper joints, matching
        /// <see cref="ArmKinematics.MapAnglesToJointTargets"/>'s own output shape -- a target for a
        /// motor id this profile doesn't have is ignored, not thrown, so a stale or foreign sender
        /// can't crash this plant. Shares every other rule <see cref="Command"/> has: stale/
        /// duplicate <paramref name="captureTicks"/> rejected whole, same one-time seed-from-sensed
        /// behavior, same clamp/belief-tracking tail.
        /// </summary>
        public JointCommandOutcome CommandJointAngles(ReadOnlySpan<JointTarget> targets, long captureTicks)
        {
            if (captureTicks <= _lastAcceptedJointCaptureTicks)
            {
                return JointCommandOutcome.Stale;
            }

            _lastAcceptedJointCaptureTicks = captureTicks;
            _lastJointCommandStateTicks = _stateTicks;
            _hadJointCommand = true;
            SeedBeliefsFromSensedIfNeeded();

            // The backstop (docs/adr/0020): the same collision model the headset's IK uses. A pose
            // that would hit the robot is refused whole and the arm holds. The headset never sends
            // one, so this only ever catches a bug or another tool.
            if (_profile.Collision != null)
            {
                CollisionHit hit = ArmCollision.Check(_profile, _profile.Collision, CommandedPose(targets));
                if (hit.IsHit)
                {
                    RefusedJointCommands++;
                    LastRefusal = hit;
                    return JointCommandOutcome.Refused;
                }
            }

            _pulsesPerSecond = RequestedPulsesPerSecond(targets);
            ApplyJointTargets(targets);
            return JointCommandOutcome.Accepted;
        }

        /// <summary>
        /// How long after a joint command a Cartesian command is not applied (this plant's clock
        /// ticks; 0, the default, applies both). Program sets half a second. Cartesian commands still
        /// get their state replies, which is what the headset sends them for (ClockSync, latency).
        /// </summary>
        public long JointPrecedenceTicks { get; set; }

        /// <summary>Cartesian commands not applied because joint commands had precedence, since construction.</summary>
        public int IgnoredCartesianCommands { get; private set; }

        /// <summary>Joint commands refused because the pose would collide (docs/adr/0020), since construction.</summary>
        public int RefusedJointCommands { get; private set; }

        /// <summary>Why the most recent refused joint command was refused.</summary>
        public CollisionHit LastRefusal { get; private set; }

        /// <summary>A readable reason for <see cref="LastRefusal"/>, for the log. Allocates.</summary>
        public string DescribeLastRefusal() =>
            _profile.Collision != null ? LastRefusal.Describe(_profile.Collision) : "no collision model";

        /// <summary>The pose a joint command asks for: its angles, and this plant's belief for any joint it leaves out.</summary>
        private ArmPose CommandedPose(ReadOnlySpan<JointTarget> targets)
        {
            ArmPose belief = BeliefPose();
            float yaw = belief.BaseYaw, proximal = belief.Proximal, distal = belief.Distal, wrist = belief.Wrist, roll = belief.Roll;
            foreach (JointTarget target in targets)
            {
                int index = FindJointIndexByMotorId(target.MotorId);
                if (index < 0) continue;
                JointHardwareSpec joint = _profile.Joints[index];
                switch (joint.Role)
                {
                    case JointRole.BaseYaw: yaw = target.Angle; break;
                    case JointRole.Proximal: proximal = target.Angle; break;
                    case JointRole.Distal: distal = target.Angle; break;
                    case JointRole.Wrist when joint.WristIndex == 0: wrist = target.Angle; break;
                    case JointRole.GripperRotate: roll = target.Angle; break;
                }
            }

            return new ArmPose(yaw, proximal, distal, wrist, roll);
        }

        private ArmPose BeliefPose() => new ArmPose(
            GetBeliefAngle(JointRole.BaseYaw), GetBeliefAngle(JointRole.Proximal), GetBeliefAngle(JointRole.Distal),
            GetBeliefAngle(JointRole.Wrist), GetBeliefAngle(JointRole.GripperRotate));

        /// <summary>
        /// One-time seed from real sensed feedback, the first time each joint's belief is ever
        /// used -- see this class's own doc for why. Shared by both <see cref="Command"/> and
        /// <see cref="CommandJointAngles"/> so this rule lives in exactly one place.
        /// </summary>
        private void SeedBeliefsFromSensedIfNeeded()
        {
            for (int i = 0; i < _profile.JointCount; i++)
            {
                if (_targetSeeded[i])
                {
                    continue;
                }

                if (_sensed[i])
                {
                    _targetPulse[i] = _sensedPulse[i];
                }

                _targetSeeded[i] = true;
            }
        }

        /// <summary>
        /// Converts each incoming target into a pulse value, clamps to that joint's resolved
        /// range, advances this plant's own optimistic target belief by no more than
        /// <see cref="GenericArmPlantConfig.MaxDirectionMagnitude"/> per call (the gripper's
        /// open/close role excepted -- open-loop, sent directly with no stepping, same as before), and sends the
        /// resulting absolute pulse targets -- the shared tail of both <see cref="Command"/> and
        /// <see cref="CommandJointAngles"/>, so there is exactly one place this logic lives
        /// regardless of which entry point supplied the angles.
        /// </summary>
        private void ApplyJointTargets(ReadOnlySpan<JointTarget> incomingTargets)
        {
            Span<JointTarget> outgoing = stackalloc JointTarget[_profile.JointCount];
            int outgoingCount = 0;

            foreach (JointTarget incoming in incomingTargets)
            {
                int index = FindJointIndexByMotorId(incoming.MotorId);
                if (index < 0)
                {
                    continue;
                }

                JointHardwareSpec joint = _profile.Joints[index];

                // Only the open/close actuator is a fraction. GripperRotate (the wrist roll) is an
                // angle and goes through the arm-joint path below, stepped like any other joint, so
                // a sudden twist does not slam the servo (docs/adr/0017).
                if (joint.Role == JointRole.GripperMain)
                {
                    float fraction = Math.Clamp(incoming.Angle, 0f, 1f);
                    float gripperPulse = Math.Clamp(
                        _config.GripperOpenPulse + fraction * (_config.GripperClosedPulse - _config.GripperOpenPulse),
                        _minPulse[index], _maxPulse[index]);
                    _targetPulse[index] = gripperPulse;
                }
                else
                {
                    // ZeroOffsetRadians corrects for a real mechanical mounting offset (the servo
                    // horn only mounts at discrete spline positions) -- applied here, to the
                    // commanded kinematic angle, not to MinAngleRadians/MaxAngleRadians below,
                    // since those express a real pulse-space physical limit that doesn't shift
                    // just because this mapping gets a mounting correction. See
                    // JointHardwareSpec.ZeroOffsetRadians's own doc.
                    float correctedAngle = incoming.Angle + joint.ZeroOffsetRadians;
                    float rawTargetPulse = _config.ZeroPulse + correctedAngle * _config.PulsePerRadian;
                    float clampedTargetPulse = Math.Clamp(rawTargetPulse, _minPulse[index], _maxPulse[index]);
                    // Direction is clamped to MaxDirectionMagnitude below -- when a single
                    // command's required delta exceeds that clamp, only the CLAMPED amount is
                    // actually applied to this plant's own belief (see class doc).
                    float direction = ToDirection(clampedTargetPulse - _targetPulse[index]);
                    _targetPulse[index] += direction * _config.StepSizePulses;
                }

                outgoing[outgoingCount++] = new JointTarget(joint.MotorId, _targetPulse[index], _pulsesPerSecond);
            }

            _relay.Send(outgoing.Slice(0, outgoingCount));
        }

        /// <summary>
        /// Advances <see cref="State"/>'s stamp to <paramref name="nowTicks"/> and polls the relay
        /// once for fresh feedback, updating whichever joints' sensed beliefs the feedback reports
        /// as valid (matched by motor id -- an entry for a motor id this profile doesn't have is
        /// ignored). A step at or before the current state time is a no-op. Allocation-free.
        /// </summary>
        public void Step(long nowTicks)
        {
            if (nowTicks <= _stateTicks)
            {
                return;
            }

            _stateTicks = nowTicks;

            Span<JointFeedbackEntry> feedbackBuffer = stackalloc JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage];
            if (!_relay.TryReceiveFeedback(feedbackBuffer, out int feedbackCount))
            {
                return;
            }

            for (int f = 0; f < feedbackCount; f++)
            {
                JointFeedbackEntry entry = feedbackBuffer[f];
                if (!entry.Valid)
                {
                    continue;
                }

                int index = FindJointIndexByMotorId(entry.MotorId);
                if (index < 0)
                {
                    continue;
                }

                _sensedPulse[index] = entry.Pulse;
                _sensed[index] = true;
                LastFeedbackTicks = nowTicks;
            }
        }

        /// <inheritdoc/>
        public Stamped<Pose> State
        {
            get
            {
                float baseYaw = GetBeliefAngle(JointRole.BaseYaw);
                float proximalPitch = GetBeliefAngle(JointRole.Proximal);
                float distalPitch = GetBeliefAngle(JointRole.Distal);
                Vector3 position = ArmKinematics.Forward(_profile, baseYaw, proximalPitch, distalPitch);
                return new Stamped<Pose>(_stateTicks, new Pose(position, Quaternion.Identity));
            }
        }

        /// <summary>
        /// Deliberately does <b>not</b> command any hardware motion, unlike
        /// <see cref="IRobotPlant.Reset"/>'s literal doc ("returns the plant to its
        /// as-constructed state"), which describes a costless teleport a kinematic plant can
        /// honor and a physical arm cannot -- see docs/adr/0007-jetrover-plant-and-robot-host.md.
        /// Clears only this plant's own bookkeeping.
        /// </summary>
        public void Reset() => ResetInternal();

        private void ResetInternal()
        {
            _lastAcceptedCartesianCaptureTicks = long.MinValue;
            _lastAcceptedJointCaptureTicks = long.MinValue;
            _stateTicks = 0;
            _hadJointCommand = false;

            for (int i = 0; i < _profile.JointCount; i++)
            {
                _targetPulse[i] = _config.ZeroPulse;
                _targetSeeded[i] = false;
                _sensed[i] = false;
                _sensedPulse[i] = _config.ZeroPulse;
            }

            LastFeedbackTicks = 0;
            _pulsesPerSecond = _config.PulsesPerSecond;
            _solverSeeded = false;
        }

        /// <summary>
        /// This joint's belief, converted back to a kinematic-model angle -- the inverse of
        /// <see cref="ApplyJointTargets"/>'s "add <see cref="JointHardwareSpec.ZeroOffsetRadians"/>
        /// before converting to pulse" step, so a round trip through a mounting-corrected joint
        /// (command angle -&gt; pulse -&gt; sensed pulse -&gt; reported angle) stays consistent; without
        /// subtracting the offset back out here, <see cref="State"/> would report a position
        /// shifted by the mounting correction even though the arm is actually where it was
        /// commanded. A role this profile might not have (e.g. no rotating base) reports angle 0,
        /// consistent with <see cref="ArmKinematics.Forward"/>'s own handling of a missing
        /// base-yaw joint.
        /// </summary>
        /// <summary>The servo speed currently sent with every move, pulses/s.</summary>
        public float CurrentPulsesPerSecond => _pulsesPerSecond;

        /// <summary>
        /// The speed a joint command asks for: <see cref="JointTarget.Speed"/> on its arm joints is a
        /// joint speed in rad/s (docs/adr/0018), 0 for "the robot's default". The fastest request
        /// wins, and it is never above <see cref="GenericArmPlantConfig.PulsesPerSecond"/>, which is
        /// what the robot was deployed to allow.
        /// </summary>
        private float RequestedPulsesPerSecond(ReadOnlySpan<JointTarget> targets)
        {
            float requested = 0f;
            foreach (JointTarget target in targets)
            {
                int index = FindJointIndexByMotorId(target.MotorId);
                if (index >= 0 && _profile.Joints[index].Role != JointRole.GripperMain && target.Speed > requested)
                {
                    requested = target.Speed;
                }
            }

            if (!(requested > 0f))
            {
                return _config.PulsesPerSecond;
            }

            return Math.Min(requested * _config.PulsePerRadian, _config.PulsesPerSecond);
        }

        /// <summary>
        /// When the relay last reported a valid joint (robot clock), the time
        /// <see cref="ReadJointState"/>'s sensed values were read. 0 until any feedback arrives.
        /// </summary>
        public long LastFeedbackTicks { get; private set; }

        /// <summary>
        /// Every joint as this plant knows it, for the joint-state downlink (docs/adr/0018), in the
        /// profile's order. Arm joints and the wrist roll are radians, with the mounting
        /// <see cref="JointHardwareSpec.ZeroOffsetRadians"/> removed, as <see cref="State"/> does.
        /// They are sensed where the relay reported the joint, and the commanded belief otherwise. The
        /// gripper is a 0..1 fraction, likewise sensed when the relay reads it back. Returns the
        /// count written. Allocation-free.
        /// </summary>
        public int ReadJointState(Span<JointSample> output)
        {
            int count = Math.Min(_profile.JointCount, output.Length);
            for (int i = 0; i < count; i++)
            {
                JointHardwareSpec joint = _profile.Joints[i];
                if (joint.Role == JointRole.GripperMain)
                {
                    // Measured when the relay reads the gripper servo back (FEEDBACK_MOTOR_IDS on the
                    // Jetson), otherwise the commanded fraction. A sensed value short of the command
                    // means the fingers stopped on something.
                    float span = _config.GripperClosedPulse - _config.GripperOpenPulse;
                    float pulse = _sensed[i] ? _sensedPulse[i] : _targetPulse[i];
                    float fraction = span != 0f ? (pulse - _config.GripperOpenPulse) / span : 0f;
                    output[i] = new JointSample(joint.MotorId, Math.Clamp(fraction, 0f, 1f), _sensed[i]);
                }
                else
                {
                    float pulse = _sensed[i] ? _sensedPulse[i] : _targetPulse[i];
                    float angle = (pulse - _config.ZeroPulse) / _config.PulsePerRadian - joint.ZeroOffsetRadians;
                    output[i] = new JointSample(joint.MotorId, angle, _sensed[i]);
                }
            }

            return count;
        }

        private float GetBeliefAngle(JointRole role)
        {
            int index = _profile.TryGetJointIndex(role);
            if (index < 0)
            {
                return 0f;
            }

            float pulse = _sensed[index] ? _sensedPulse[index] : _targetPulse[index];
            return (pulse - _config.ZeroPulse) / _config.PulsePerRadian - _profile.Joints[index].ZeroOffsetRadians;
        }

        private int FindJointIndexByMotorId(byte motorId)
        {
            for (int i = 0; i < _profile.Joints.Length; i++)
            {
                if (_profile.Joints[i].MotorId == motorId)
                {
                    return i;
                }
            }

            return -1;
        }

        private float ToDirection(float pulseDelta) =>
            Math.Clamp(pulseDelta / _config.StepSizePulses, -_config.MaxDirectionMagnitude, _config.MaxDirectionMagnitude);
    }
}
