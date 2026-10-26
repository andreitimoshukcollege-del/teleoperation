using System;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;

namespace Teleop.RobotHost.Power
{
    /// <summary>
    /// What <see cref="ArmPowerController"/> reads and drives. Program implements it over the plant,
    /// the relay and the joint socket; the tests implement it with a simulated arm.
    /// </summary>
    public interface IArmPowerPort
    {
        /// <summary>
        /// The arm joints (every joint but the gripper, the profile's order): the latest angle, radians,
        /// and when it was last sensed (robot ticks; <see cref="long.MinValue"/> if never).
        /// </summary>
        void ReadArm(Span<float> angles, Span<long> sensedTicks);

        /// <summary>A park waypoint (radians, rad/s), through the path joint commands take.</summary>
        JointCommandOutcome CommandArm(ReadOnlySpan<JointTarget> targets, long now);

        /// <summary>Torque on or off for every servo, through the relay.</summary>
        void SetTorque(bool on, long now);

        /// <summary>A status to the headset.</summary>
        void SendStatus(ArmPowerState state, ArmPowerReason reason, uint echoSequence, long echoOperatorTicks, float remainingDeg, long now);

        /// <summary>The state changed (or a relax was refused, from and to both holding). <paramref name="detail"/> is for the log.</summary>
        void StateChanged(long now, ArmPowerState from, ArmPowerState to, ArmPowerReason reason, string detail);
    }

    /// <summary>
    /// "Park, then relax" (docs/adr/0023 decision 5): holding → parking → relaxed → holding.
    /// <list type="bullet">
    /// <item><b>Relax</b>, while holding: refused without a rest pose, without a collision model, or
    /// without a fresh reading of every arm joint. Already within the tolerance of the rest pose: the
    /// torque goes off at once. Otherwise the straight joint-space path from the measured pose to the
    /// rest pose is sampled against the collision model RobotHost enforces (docs/adr/0020), every
    /// sample from the start to the end; one collision refuses it. Then the arm parks: waypoints along
    /// that path, the furthest joint at the park speed, through the joint-command path. When every arm
    /// joint's fresh reading is within the tolerance, nothing more is sent for the settle time; still
    /// there, the torque goes off (every servo, the gripper too): relaxed. The park times out back to
    /// holding, refused, and ends the same way when the board stops answering.</item>
    /// <item><b>Hold</b>: the torque goes on, then holding, and operator commands pass again. While
    /// parking it stops the park where the arm is.</item>
    /// <item><b>Operator commands</b> (joint and Cartesian) pass only while holding; otherwise they are
    /// ignored and counted. Until RobotHost has switched the torque itself it does not know it (a
    /// restart may find the arm relaxed), so the first one it passes switches the torque on first.</item>
    /// <item><b>Status:</b> an answer to every request, one on every change, and one a second while
    /// not holding.</item>
    /// </list>
    /// Pure: time is a parameter and every effect goes through <see cref="IArmPowerPort"/>. Main loop only.
    /// </summary>
    public sealed class ArmPowerController
    {
        private enum Torque { Unknown, On, Off }

        private const float Deg = MathF.PI / 180f;

        private readonly RobotArmProfile _profile;
        private readonly ArmPowerOptions _options;
        private readonly IArmPowerPort _port;
        private readonly long _tps;
        private readonly int[] _armIndexes;
        private readonly byte[] _armMotorIds;
        private readonly float[]? _rest;
        private readonly float[] _angles;
        private readonly long[] _sensed;
        private readonly float[] _parkStart;
        private readonly JointTarget[] _waypoint;
        private readonly ArmCollisionModel? _contactModel;

        private Torque _torque = Torque.Unknown;
        private uint _echoSequence;
        private long _echoOperatorTicks;
        private long _lastStatusTicks = long.MinValue;

        private long _parkStartTicks;
        private float _parkSeconds;
        private bool _reachedEnd;
        private long _lastWaypointTicks;
        private long _settleSinceTicks = long.MinValue;

        public ArmPowerController(in RobotArmProfile profile, ArmPowerOptions options, long ticksPerSecond, IArmPowerPort port)
        {
            _profile = profile;
            _options = options ?? throw new ArgumentNullException(nameof(options));
            _port = port ?? throw new ArgumentNullException(nameof(port));
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _tps = ticksPerSecond;
            string? invalid = options.Validate();
            if (invalid != null) throw new ArgumentException(invalid, nameof(options));

            int n = 0;
            foreach (JointHardwareSpec j in profile.Joints) if (j.Role != JointRole.GripperMain) n++;
            _armIndexes = new int[n];
            _armMotorIds = new byte[n];
            n = 0;
            for (int i = 0; i < profile.Joints.Length; i++)
            {
                if (profile.Joints[i].Role == JointRole.GripperMain) continue;
                _armIndexes[n] = i;
                _armMotorIds[n] = profile.Joints[i].MotorId;
                n++;
            }

            if (options.RestPoseDegrees != null)
            {
                if (options.RestPoseDegrees.Length != n)
                {
                    throw new ArgumentException(
                        $"--rest-pose-deg has {options.RestPoseDegrees.Length} values; profile '{profile.Name}' has {n} arm joints " +
                        $"(motors {string.Join(", ", _armMotorIds)}, in that order)", nameof(options));
                }

                _rest = new float[n];
                for (int i = 0; i < n; i++) _rest[i] = (float)options.RestPoseDegrees[i] * Deg;
            }

            if (_rest != null && profile.Collision != null)
            {
                ArmPose rest = Pose(_rest);
                CollisionHit restHit = ArmCollision.Check(profile, profile.Collision, rest);
                if (restHit.IsHit)
                {
                    if (options.RestContactDegrees > 0)
                    {
                        _contactModel = ContactModel(profile, profile.Collision, rest, out string contacts);
                        RestPoseNote = _contactModel != null
                            ? $"the rest pose touches the body by the model ({contacts}); allowed within {options.RestContactDegrees:0.#} deg of it"
                            : $"the rest pose collides by the model ({contacts}), not only by resting on the body: every relax will be refused";
                    }
                    else
                    {
                        RestPoseNote = $"the rest pose itself collides by the model ({restHit.Describe(profile.Collision)}): with " +
                            "--park-contact-deg 0 every relax will be refused";
                    }
                }
            }

            _angles = new float[n];
            _sensed = new long[n];
            _parkStart = new float[n];
            _waypoint = new JointTarget[n];
        }

        public ArmPowerState State { get; private set; } = ArmPowerState.Holding;

        /// <summary>Why the last relax was refused or ended back in holding; <see cref="ArmPowerReason.None"/> otherwise.</summary>
        public ArmPowerReason LastReason { get; private set; }

        /// <summary>Operator commands ignored because the arm was parking or relaxed, since construction.</summary>
        public int IgnoredCommands { get; private set; }

        /// <summary>The arm joints' motor ids, the order <c>--rest-pose-deg</c> lists them in.</summary>
        public ReadOnlySpan<byte> ArmMotorIds => _armMotorIds;

        public bool HasRestPose => _rest != null;

        /// <summary>What the collision model says about the rest pose itself, for the start-up log; null when it is clear.</summary>
        public string? RestPoseNote { get; }

        /// <summary>
        /// The model with the rest pose's own contacts allowed (<see cref="ArmPowerOptions.RestContactDegrees"/>):
        /// every (body, fixed box) pair the rest pose touches, widened to every body on the same link, is
        /// ignored; nothing else changes. Null when the rest pose's hit is not with a fixed box (the floor,
        /// another link, a joint range), which no support explains.
        /// </summary>
        private static ArmCollisionModel? ContactModel(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose rest, out string contacts)
        {
            var pairs = new System.Collections.Generic.List<(int, int)>();
            var boxes = new System.Collections.Generic.List<(int, int)>();
            int n = model.Bodies.Length, m = model.Boxes.Length;
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++) if (model.IgnoresBodyPair(i, j)) pairs.Add((i, j));
                for (int k = 0; k < m; k++) if (model.IgnoresBodyBox(i, k)) boxes.Add((i, k));
            }

            var found = new System.Collections.Generic.List<string>();
            ArmCollisionModel current = model;
            for (int round = 0; round <= m; round++)
            {
                CollisionHit hit = ArmCollision.Check(profile, current, rest);
                if (!hit.IsHit)
                {
                    contacts = string.Join(", ", found);
                    return current;
                }

                if (hit.Kind != CollisionHitKind.BodyBox)
                {
                    found.Add(hit.Describe(model));
                    contacts = string.Join(", ", found);
                    return null;
                }

                ArmLink link = model.Bodies[hit.A].Link;
                for (int b = 0; b < n; b++) if (model.Bodies[b].Link == link) boxes.Add((b, hit.B));
                found.Add($"{link} on {model.Boxes[hit.B].Name}");
                current = new ArmCollisionModel(model.Name + " (rest contacts allowed)", model.Bodies, model.Boxes, model.FloorZ, model.Margin,
                    model.BaseYaw, model.Proximal, model.Distal, model.Wrist, model.Roll, model.Home, pairs.ToArray(), boxes.ToArray(), model.Source);
            }

            contacts = string.Join(", ", found);
            return null;
        }

        /// <summary>
        /// An operator message: a heartbeat (<see cref="ArmPowerRequest.None"/>) or a relax or hold
        /// request. Answered with a status, always.
        /// </summary>
        public void OnRequest(ArmPowerRequest request, uint sequence, long operatorTicks, long now)
        {
            _echoSequence = sequence;
            _echoOperatorTicks = operatorTicks;
            ArmPowerReason answerReason = ArmPowerReason.None;
            switch (request)
            {
                case ArmPowerRequest.None:
                    // A heartbeat: answered with the state, nothing else.
                    break;
                case ArmPowerRequest.Relax:
                    if (State == ArmPowerState.Holding) answerReason = StartRelax(now);
                    break;
                case ArmPowerRequest.Hold:
                    if (State == ArmPowerState.Parking)
                    {
                        Transition(now, ArmPowerState.Holding, ArmPowerReason.HoldRequested, "stopped where it is");
                        answerReason = ArmPowerReason.HoldRequested;
                    }
                    else if (State == ArmPowerState.Relaxed)
                    {
                        TorqueOn(now);
                        Transition(now, ArmPowerState.Holding, ArmPowerReason.None, "torque on");
                    }
                    else if (_torque != Torque.On)
                    {
                        TorqueOn(now);
                    }

                    break;
                default:
                    answerReason = ArmPowerReason.UnknownRequest;
                    break;
            }

            SendStatus(now, answerReason);
        }

        /// <summary>
        /// An operator command (joint or Cartesian) is about to be applied. False: ignore it (parking or
        /// relaxed; counted). True: apply it; the first after start-up switches the torque on first.
        /// </summary>
        public bool AcceptOperatorCommand(long now)
        {
            if (State != ArmPowerState.Holding)
            {
                IgnoredCommands++;
                return false;
            }

            if (_torque != Torque.On) TorqueOn(now);
            return true;
        }

        /// <summary>Advances a park and repeats the status. Call every loop.</summary>
        public void Tick(long now)
        {
            if (State == ArmPowerState.Parking) TickPark(now);
            if (State != ArmPowerState.Holding && (_lastStatusTicks == long.MinValue || now - _lastStatusTicks >= Ticks(_options.StatusRepeatSeconds)))
            {
                SendStatus(now, ArmPowerReason.None);
            }
        }

        private ArmPowerReason StartRelax(long now)
        {
            if (_rest == null) return Refuse(now, ArmPowerReason.NoRestPose, "start RobotHost with --rest-pose-deg");
            ArmCollisionModel? model = _profile.Collision;
            if (model == null) return Refuse(now, ArmPowerReason.NoCollisionModel, $"profile '{_profile.Name}'");

            _port.ReadArm(_angles, _sensed);
            int stale = FirstStale(now, long.MinValue);
            if (stale >= 0) return Refuse(now, ArmPowerReason.BoardNotAnswering, $"motor {_armMotorIds[stale]} has no fresh reading");

            if (MaxDistance(_angles) <= _options.ToleranceDegrees * Deg)
            {
                // Already resting there: nothing to move.
                Transition(now, ArmPowerState.Parking, ArmPowerReason.None, "already at the rest pose");
                TorqueOff(now);
                Transition(now, ArmPowerState.Relaxed, ArmPowerReason.None, "torque off");
                return ArmPowerReason.None;
            }

            ArmPose from = Pose(_angles), to = Pose(_rest);
            float jump = from.MaxJointDelta(to);
            int steps = Math.Max(1, (int)MathF.Ceiling(jump / ((float)_options.PathStepDegrees * Deg)));
            float window = (float)_options.RestContactDegrees * Deg;
            for (int s = 0; s <= steps; s++)
            {
                ArmPose sample = ArmPose.Lerp(from, to, s / (float)steps);
                // Near the rest pose, and only when asked, the rest pose's own supports may be touched.
                ArmCollisionModel against = _contactModel != null && sample.MaxJointDelta(to) <= window ? _contactModel : model;
                CollisionHit hit = ArmCollision.Check(_profile, against, sample);
                if (hit.IsHit)
                {
                    return Refuse(now, ArmPowerReason.PathCollides,
                        $"at {100f * s / steps:0}% of the way: {hit.Describe(model)}");
                }
            }

            Array.Copy(_angles, _parkStart, _angles.Length);
            _parkStartTicks = now;
            _parkSeconds = jump / (float)_options.ParkSpeedRadPerSec;
            _reachedEnd = false;
            _lastWaypointTicks = long.MinValue;
            _settleSinceTicks = long.MinValue;
            Transition(now, ArmPowerState.Parking, ArmPowerReason.None,
                $"{jump / Deg:0.#} deg at {_options.ParkSpeedRadPerSec:0.##} rad/s, about {_parkSeconds:0.#} s");
            SendWaypoint(now);
            return ArmPowerReason.None;
        }

        private void TickPark(long now)
        {
            if (now - _parkStartTicks > Ticks(_options.ParkTimeoutSeconds))
            {
                Transition(now, ArmPowerState.Holding, ArmPowerReason.ParkTimedOut,
                    $"{_options.ParkTimeoutSeconds:0.#} s; still {MaxDistance(_angles) / Deg:0.#} deg away; holds where it is");
                SendStatus(now, ArmPowerReason.ParkTimedOut);
                return;
            }

            _port.ReadArm(_angles, _sensed);
            long newest = long.MinValue;
            foreach (long t in _sensed) if (t > newest) newest = t;
            if (now - Math.Max(newest, _parkStartTicks) > Ticks(_options.BoardSilentSeconds))
            {
                Transition(now, ArmPowerState.Holding, ArmPowerReason.BoardNotAnswering, "no reading while parking; holds where it is");
                SendStatus(now, ArmPowerReason.BoardNotAnswering);
                return;
            }

            // At the rest pose by fresh readings taken since the park began: send nothing more, let it settle.
            bool there = _reachedEnd && FirstStale(now, _parkStartTicks) < 0 && MaxDistance(_angles) <= _options.ToleranceDegrees * Deg;
            if (there)
            {
                if (_settleSinceTicks == long.MinValue) _settleSinceTicks = now;
                if (now - _settleSinceTicks >= Ticks(_options.SettleSeconds))
                {
                    TorqueOff(now);
                    Transition(now, ArmPowerState.Relaxed, ArmPowerReason.None, "torque off");
                    SendStatus(now, ArmPowerReason.None);
                }

                return;
            }

            _settleSinceTicks = long.MinValue;
            long period = _reachedEnd ? Ticks(0.5) : Ticks(_options.WaypointSeconds);
            if (now - _lastWaypointTicks >= period) SendWaypoint(now);
        }

        private void SendWaypoint(long now)
        {
            float s = _parkSeconds <= 0f ? 1f : Math.Min(1f, (now - _parkStartTicks) / (float)_tps / _parkSeconds);
            for (int i = 0; i < _waypoint.Length; i++)
            {
                float angle = _parkStart[i] + (_rest![i] - _parkStart[i]) * s;
                _waypoint[i] = new JointTarget(_armMotorIds[i], angle, (float)_options.ParkSpeedRadPerSec);
            }

            _lastWaypointTicks = now;
            if (s >= 1f) _reachedEnd = true;
            if (_port.CommandArm(_waypoint, now) == JointCommandOutcome.Refused)
            {
                // The path was checked; the plant's own check disagreeing is a bug, and the arm holds.
                Transition(now, ArmPowerState.Holding, ArmPowerReason.PathCollides, "the plant refused a waypoint; holds where it is");
                SendStatus(now, ArmPowerReason.PathCollides);
            }
        }

        private ArmPowerReason Refuse(long now, ArmPowerReason reason, string detail)
        {
            Transition(now, ArmPowerState.Holding, reason, detail);
            return reason;
        }

        private void Transition(long now, ArmPowerState to, ArmPowerReason reason, string detail)
        {
            ArmPowerState from = State;
            State = to;
            LastReason = reason;
            _port.StateChanged(now, from, to, reason, detail);
        }

        private void TorqueOn(long now)
        {
            _torque = Torque.On;
            _port.SetTorque(true, now);
        }

        private void TorqueOff(long now)
        {
            _torque = Torque.Off;
            _port.SetTorque(false, now);
        }

        private void SendStatus(long now, ArmPowerReason reason)
        {
            _lastStatusTicks = now;
            float remaining = float.NaN;
            if (_rest != null)
            {
                _port.ReadArm(_angles, _sensed);
                if (FirstStale(now, long.MinValue) < 0) remaining = MaxDistance(_angles) / Deg;
            }

            _port.SendStatus(State, reason, _echoSequence, _echoOperatorTicks, remaining, now);
        }

        /// <summary>The first arm joint without a fresh reading taken at or after <paramref name="since"/>; -1 when all have one.</summary>
        private int FirstStale(long now, long since)
        {
            long fresh = Ticks(_options.FreshSeconds);
            for (int i = 0; i < _sensed.Length; i++)
            {
                if (_sensed[i] == long.MinValue || _sensed[i] < since || now - _sensed[i] > fresh) return i;
            }

            return -1;
        }

        private float MaxDistance(float[] angles)
        {
            float max = 0f;
            for (int i = 0; i < angles.Length; i++) max = MathF.Max(max, MathF.Abs(angles[i] - _rest![i]));
            return max;
        }

        private ArmPose Pose(float[] arm)
        {
            float yaw = 0f, proximal = 0f, distal = 0f, wrist = 0f, roll = 0f;
            for (int i = 0; i < _armIndexes.Length; i++)
            {
                JointHardwareSpec joint = _profile.Joints[_armIndexes[i]];
                switch (joint.Role)
                {
                    case JointRole.BaseYaw: yaw = arm[i]; break;
                    case JointRole.Proximal: proximal = arm[i]; break;
                    case JointRole.Distal: distal = arm[i]; break;
                    case JointRole.Wrist when joint.WristIndex == 0: wrist = arm[i]; break;
                    case JointRole.GripperRotate: roll = arm[i]; break;
                }
            }

            return new ArmPose(yaw, proximal, distal, wrist, roll);
        }

        private long Ticks(double seconds) => (long)(seconds * _tps);
    }
}
