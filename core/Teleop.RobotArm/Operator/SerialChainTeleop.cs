using System;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>How a <see cref="SerialChainTeleop.Step"/> moves an arm toward its target (docs/adr/0029 amendment 2).</summary>
    public enum ChainStepMode
    {
        /// <summary>
        /// The gripper moves along the shortest path to the target (a straight line and the smallest rotation), at most
        /// <see cref="SerialChainTeleopOptions.MaxLinearSpeed"/> and <see cref="SerialChainTeleopOptions.MaxAngularSpeed"/>,
        /// and only that small step is solved, from where the arm is: the joints change no more than the path needs, and
        /// the arm keeps its configuration. The default.
        /// </summary>
        CartesianPath,

        /// <summary>
        /// The first way: solve all the way to the target every step, then move the joints straight toward that solution,
        /// shortened to <see cref="SerialChainTeleopOptions.MaxStepRadians"/>. A far target can be solved on another branch,
        /// and the straight joint-space line to it swings the arm. Kept to compare against (SerialChainTeleopTrackingTests).
        /// </summary>
        FullSolve,
    }

    /// <summary>The two-arm operator's settings (docs/adr/0029). Defaults are first guesses, tuned on the headset.</summary>
    public sealed class SerialChainTeleopOptions
    {
        /// <summary>How each step moves the arm toward its target.</summary>
        public ChainStepMode StepMode { get; set; } = ChainStepMode.CartesianPath;

        /// <summary><see cref="ChainStepMode.CartesianPath"/>'s cap on the gripper's speed along its path (m/s).</summary>
        public double MaxLinearSpeed { get; set; } = 1.0;

        /// <summary><see cref="ChainStepMode.CartesianPath"/>'s cap on the gripper's turning rate (rad/s).</summary>
        public double MaxAngularSpeed { get; set; } = 3.0;

        /// <summary>
        /// Start bending the arm away from what it is closing on this far before the collision margin (m), with the spare
        /// motion of a redundant arm, so the gripper stays on its path (<see cref="ChainStepMode.CartesianPath"/>); 0 never.
        /// </summary>
        public double AvoidDistance { get; set; } = 0.04;

        /// <summary>How much of the shortfall in clearance one step tries to win back by bending away (0..1).</summary>
        public double AvoidGain { get; set; } = 0.5;

        /// <summary>
        /// Steer a redundant arm's spare motion off its joint limits, at most this fast (rad/s) for a joint on a limit and
        /// starting <see cref="LimitZone"/> before it (<see cref="ChainStepMode.CartesianPath"/>); 0 never. Without it the
        /// arm drifts, as the hand wanders, until a joint (most often the forearm's) sits on its limit: the gripper then
        /// stops short of poses another elbow position reaches, and the other joints swing to make up for the stuck one.
        /// </summary>
        public double LimitReliefSpeed { get; set; } = 2.0;

        /// <summary>How far before each soft limit the relief starts (rad); at most a quarter of the joint's range.</summary>
        public double LimitZone { get; set; } = 0.6;

        /// <summary>
        /// The share of each joint's velocity limit (the profile's) one step may use: a step that would move a joint faster
        /// is shortened, every joint of the arm together, so the gripper keeps its path and only slows. 0 leaves only
        /// <see cref="MaxStepRadians"/>.
        /// </summary>
        public double JointSpeedShare { get; set; } = 1.0;

        /// <summary>
        /// How far from its target the gripper may stay before the arm reports <see cref="ChainArmStatus.StoppedShort"/>
        /// (m, and rad): the solver's own tolerances are a tenth of a millimetre, far below what the operator sees, and an arm
        /// that close to its target is following it (<see cref="ChainStepMode.CartesianPath"/>).
        /// </summary>
        public double ReachTolerance { get; set; } = 0.01;

        /// <summary>The same for the gripper's orientation (rad): 5°.</summary>
        public double ReachAngleTolerance { get; set; } = 5.0 * Math.PI / 180.0;

        /// <summary>The step's length when <see cref="SerialChainTeleop.Step()"/> is called without one (s): the headset's 90 Hz.</summary>
        public double DefaultStepSeconds { get; set; } = 1.0 / 90.0;

        /// <summary>Hand motion → tool motion (<see cref="PoseClutch.PositionScale"/>). One to one by default: the FR3 reaches about as far as an arm.</summary>
        public double PositionScale { get; set; } = 1.0;

        /// <summary>
        /// The most any joint's command may change in one <see cref="SerialChainTeleop.Step"/> (radians). A larger solution
        /// (the solver finding another branch, a hand reappearing elsewhere) is shortened to this, so every step's pose is
        /// near the last checked one and the robot never travels a long unchecked path between two checked poses.
        /// </summary>
        public double MaxStepRadians { get; set; } = 0.1;

        /// <summary>
        /// How many steps an <see cref="SerialChainTeleop.EngageAt"/> takes to slide the target from where the tool is to where the
        /// hand maps (90: a second at the headset's frame rate). The arm never lunges at the hand, and the solver, warm-started
        /// from the last command, follows a continuous path instead of jumping across the workspace.
        /// </summary>
        public int ApproachSteps { get; set; } = 90;

        /// <summary>The solver's settings; its defaults when null.</summary>
        public SerialChainSolverOptions? Solver { get; set; }

        public string? Validate()
        {
            if (!(PositionScale > 0) || double.IsInfinity(PositionScale)) return $"PositionScale must be positive and finite ({PositionScale})";
            if (!(MaxStepRadians > 0) || double.IsInfinity(MaxStepRadians)) return $"MaxStepRadians must be positive and finite ({MaxStepRadians})";
            if (ApproachSteps < 1) return $"ApproachSteps must be at least 1 ({ApproachSteps})";
            if (!(MaxLinearSpeed > 0) || !(MaxAngularSpeed > 0) || double.IsInfinity(MaxLinearSpeed + MaxAngularSpeed)) return "the gripper's speed caps must be positive and finite";
            if (!(AvoidDistance >= 0) || !(AvoidGain >= 0) || AvoidGain > 1) return $"AvoidDistance must be at least 0 and AvoidGain in [0, 1] ({AvoidDistance}, {AvoidGain})";
            if (!(DefaultStepSeconds > 0) || double.IsInfinity(DefaultStepSeconds)) return $"DefaultStepSeconds must be positive ({DefaultStepSeconds})";
            if (!(LimitReliefSpeed >= 0) || !(LimitZone > 0) || double.IsInfinity(LimitReliefSpeed + LimitZone)) return $"LimitReliefSpeed must be at least 0 and LimitZone positive ({LimitReliefSpeed}, {LimitZone})";
            if (!(JointSpeedShare >= 0) || JointSpeedShare > 1) return $"JointSpeedShare must be in [0, 1] ({JointSpeedShare})";
            if (!(ReachTolerance > 0) || !(ReachAngleTolerance > 0) || double.IsInfinity(ReachTolerance + ReachAngleTolerance)) return "the reach tolerances must be positive and finite";
            return null;
        }
    }

    /// <summary>Where one arm's clutch stands after a <see cref="SerialChainTeleop.Step"/>.</summary>
    public enum ChainArmStatus
    {
        /// <summary>Not clutched: the arm holds its last command.</summary>
        Idle,

        /// <summary>Clutched, and the command reaches the hand's target (or is on its way within the step limit).</summary>
        Following,

        /// <summary>
        /// Clutched, but the target is out of reach: the command is the closest pose inside the joint limits. With
        /// <see cref="ChainStepMode.CartesianPath"/>, only when the arm is stuck and further than the reach tolerances from it.
        /// </summary>
        StoppedShort,

        /// <summary>Clutched, but the pose toward the target would collide (self, the other arm, or the body): the command holds.</summary>
        Blocked,
    }

    /// <summary>
    /// The two-hand operator of a serial-chain robot (docs/adr/0029): one <see cref="PoseClutch"/> per arm, solved by
    /// <see cref="SerialChainSolver"/> from the arm's current command, and checked as a whole robot against the
    /// collision model before it is committed. It produces the joint command RobotHost receives: every arm joint and
    /// every gripper, in profile order, radians and opening fractions.
    ///
    /// <para><b>Safe by construction</b>, as the JetRover's clutch is (docs/adr/0017, 0020):</para>
    /// <list type="bullet">
    /// <item>Nothing is commanded before the robot's measured pose has been taken (<see cref="TryTakePose"/>): the first
    /// command is where the robot already is.</item>
    /// <item>A released arm holds its command. Re-engaging anywhere never jumps it (the clutch is relative).</item>
    /// <item>Every committed pose is inside the soft limits and, with a collision model, clear of the body and of the
    /// other arm's command; a step that would collide is cut to its largest clear part, or not taken and the arm holds
    /// where it was (<see cref="ChainArmStatus.Blocked"/>). RobotHost refuses the same poses on its side (docs/adr/0026 amendment).</item>
    /// <item>No joint's command moves more than <see cref="SerialChainTeleopOptions.MaxStepRadians"/> per step, nor faster than
    /// its share of the joint's velocity limit (<see cref="SerialChainTeleopOptions.JointSpeedShare"/>).</item>
    /// </list>
    ///
    /// <para>Plain C#: no clock, no I/O, and nothing allocated after construction. The host owns when to step and when
    /// to send.</para>
    /// </summary>
    public sealed class SerialChainTeleop
    {
        private readonly SerialChainProfile _profile;
        private readonly SerialChainCollisionModel? _collision;
        private readonly SerialChainTeleopOptions _options;
        private readonly SerialChainSolver[] _solvers;
        private readonly PoseClutch[] _clutches;
        private readonly ChainArmStatus[] _status;
        private readonly int[] _offset;
        private readonly double[] _q;
        private readonly double[] _candidate;
        private readonly double[] _solution;
        private readonly double[] _opening;
        private readonly bool[] _absolute;
        private readonly double[] _approach;
        private readonly RigidTransform[] _approachFrom;
        private readonly RigidTransform[] _handTarget;
        private readonly double[] _stepped;
        private readonly double[] _gradient;
        private readonly double[] _nullStep;

        public SerialChainTeleop(SerialChainProfile profile, SerialChainCollisionModel? collision = null, SerialChainTeleopOptions? options = null)
        {
            _profile = profile ?? throw new ArgumentNullException(nameof(profile));
            _options = options ?? new SerialChainTeleopOptions();
            if (_options.Validate() is string bad) throw new ArgumentException(bad, nameof(options));
            if (profile.Validate() is string badProfile) throw new ArgumentException(badProfile, nameof(profile));
            if (collision != null && collision.Profile != profile.Name)
            {
                throw new ArgumentException($"the collision model is for '{collision.Profile}', not '{profile.Name}'", nameof(collision));
            }

            _collision = collision;
            int arms = profile.Arms.Length;
            _solvers = new SerialChainSolver[arms];
            _clutches = new PoseClutch[arms];
            _status = new ChainArmStatus[arms];
            _offset = new int[arms];
            _opening = new double[arms];
            _absolute = new bool[arms];
            _approach = new double[arms];
            _approachFrom = new RigidTransform[arms];
            _handTarget = new RigidTransform[arms];
            int joints = 0, widest = 0;
            for (int a = 0; a < arms; a++)
            {
                ChainArm arm = profile.Arms[a];
                _solvers[a] = new SerialChainSolver(arm, _options.Solver);
                _clutches[a] = new PoseClutch(_options.PositionScale, RigidTransform.Identity);
                _offset[a] = joints;
                joints += arm.Joints.Length;
                widest = Math.Max(widest, arm.Joints.Length);
                _opening[a] = 1.0;
            }

            _q = new double[joints];
            _candidate = new double[joints];
            _solution = new double[widest];
            _stepped = new double[joints];
            _gradient = new double[widest];
            _nullStep = new double[widest];
            MotorCount = joints;
            for (int a = 0; a < arms; a++)
            {
                if (profile.Arms[a].HasGripper) MotorCount++;
            }
        }

        public SerialChainProfile Profile => _profile;

        public int ArmCount => _profile.Arms.Length;

        /// <summary>How many targets <see cref="WriteTargets"/> writes: every arm joint and every gripper.</summary>
        public int MotorCount { get; }

        /// <summary>Whether the robot's pose has been taken; until then nothing is commanded and no clutch engages.</summary>
        public bool HasPose { get; private set; }

        /// <summary>The arm whose profile name is <paramref name="name"/> (<c>left</c>, <c>right</c>), or -1.</summary>
        public int ArmIndex(string name)
        {
            for (int a = 0; a < _profile.Arms.Length; a++)
            {
                if (string.Equals(_profile.Arms[a].Name, name, StringComparison.Ordinal)) return a;
            }

            return -1;
        }

        /// <summary>
        /// Takes the robot's measured pose as the command: every arm joint must be in <paramref name="measured"/>; a
        /// gripper missing from it is taken as open. Releases every clutch, so nothing follows a hand from a stale anchor.
        /// Returns false, and changes nothing, when a joint is missing or not a number.
        /// </summary>
        public bool TryTakePose(ReadOnlySpan<JointSample> measured)
        {
            for (int a = 0; a < _profile.Arms.Length; a++)
            {
                ChainJoint[] joints = _profile.Arms[a].Joints;
                for (int j = 0; j < joints.Length; j++)
                {
                    if (!TryFind(measured, joints[j].MotorId, out float v) || float.IsNaN(v) || float.IsInfinity(v)) return false;
                }
            }

            for (int a = 0; a < _profile.Arms.Length; a++)
            {
                ChainArm arm = _profile.Arms[a];
                for (int j = 0; j < arm.Joints.Length; j++)
                {
                    TryFind(measured, arm.Joints[j].MotorId, out float v);
                    ChainJointLimits l = arm.Joints[j].Limits;
                    // Inside the soft range, so the first command is one RobotHost accepts.
                    _q[_offset[a] + j] = Math.Min(l.SoftUpper, Math.Max(l.SoftLower, v));
                }

                if (arm.HasGripper && TryFind(measured, arm.Gripper.MotorId, out float g) && !float.IsNaN(g))
                {
                    _opening[a] = Math.Min(1.0, Math.Max(0.0, g));
                }

                _absolute[a] = false;
                _clutches[a].Release();
                _clutches[a].Hold(Tool(a));
                _status[a] = ChainArmStatus.Idle;
            }

            HasPose = true;
            return true;
        }

        /// <summary>Starts arm <paramref name="arm"/> following <paramref name="hand"/> (root frame) from where its command is. False before a pose.</summary>
        public bool Engage(int arm, in RigidTransform hand)
        {
            if (!HasPose) return false;
            _clutches[arm].Engage(hand, Tool(arm));
            _status[arm] = ChainArmStatus.Following;
            return true;
        }

        /// <summary>Moves arm <paramref name="arm"/>'s target with the hand; nothing while released. <see cref="Step"/> solves it.</summary>
        public void Follow(int arm, in RigidTransform hand) => _clutches[arm].Follow(hand);

        /// <summary>
        /// The absolute mapping (docs/adr/0029 amendment): starts arm <paramref name="arm"/> toward <paramref name="target"/>
        /// (root frame), where the hand maps to, rather than from where the arm is. The arm still gets there no faster
        /// than <see cref="SerialChainTeleopOptions.MaxStepRadians"/> per step, through checked poses. False before a pose.
        /// </summary>
        public bool EngageAt(int arm, in RigidTransform target)
        {
            if (!HasPose) return false;
            RigidTransform tool = Tool(arm);
            _clutches[arm].Engage(RigidTransform.Identity, tool);
            _absolute[arm] = true;
            _approach[arm] = 0;
            _approachFrom[arm] = tool;
            _handTarget[arm] = target;
            _status[arm] = ChainArmStatus.Following;
            return true;
        }

        /// <summary>Moves an <see cref="EngageAt"/> arm's target to <paramref name="target"/>; nothing while released.</summary>
        public void Track(int arm, in RigidTransform target)
        {
            if (_clutches[arm].IsEngaged && _absolute[arm]) _handTarget[arm] = target;
        }

        /// <summary>
        /// While arm <paramref name="arm"/> is released, shows where it would go: its target (the marker) becomes
        /// <paramref name="target"/>, and nothing moves. The absolute mapping's preview.
        /// </summary>
        public void Preview(int arm, in RigidTransform target) => _clutches[arm].Hold(target);

        /// <summary>Stops arm <paramref name="arm"/> following; it holds its command, and its target moves to where the arm is.</summary>
        public void Release(int arm)
        {
            _absolute[arm] = false;
            _clutches[arm].Release();
            _clutches[arm].Hold(Tool(arm));
            _status[arm] = ChainArmStatus.Idle;
        }

        public bool IsEngaged(int arm) => _clutches[arm].IsEngaged;

        public ChainArmStatus Status(int arm) => _status[arm];

        /// <summary>Where arm <paramref name="arm"/>'s clutch asks its tool to be (root frame).</summary>
        public RigidTransform Target(int arm) => _clutches[arm].Target;

        /// <summary>Sets arm <paramref name="arm"/>'s gripper opening (1 open, 0 closed); NaN leaves it.</summary>
        public void SetOpening(int arm, double opening)
        {
            if (double.IsNaN(opening)) return;
            _opening[arm] = Math.Min(1.0, Math.Max(0.0, opening));
        }

        public double Opening(int arm) => _opening[arm];

        /// <summary>Arm <paramref name="arm"/>'s commanded joints, radians.</summary>
        public ReadOnlySpan<double> Commanded(int arm) => _q.AsSpan(_offset[arm], _profile.Arms[arm].Joints.Length);

        /// <summary>The tool of arm <paramref name="arm"/>'s command (root frame).</summary>
        public RigidTransform Tool(int arm) => SerialChainKinematics.Tool(_profile.Arms[arm], Commanded(arm));

        /// <summary>One step of <see cref="SerialChainTeleopOptions.DefaultStepSeconds"/>.</summary>
        public void Step() => Step(_options.DefaultStepSeconds);

        /// <summary>
        /// Moves every clutched arm toward its target by one step of <paramref name="seconds"/> and commits it if it is clear
        /// (<see cref="SerialChainTeleopOptions.StepMode"/>). A step that would collide is cut to the largest clear part of it
        /// (a half, then a quarter), or not taken: <see cref="ChainArmStatus.Blocked"/>. A step longer than
        /// <see cref="LongestStep"/> (a hitch in the host's frames) counts as that long: it never becomes a jump.
        /// </summary>
        public void Step(double seconds)
        {
            if (!HasPose) return;
            if (!(seconds > 0) || double.IsInfinity(seconds)) seconds = _options.DefaultStepSeconds;
            seconds = Math.Min(seconds, LongestStep);
            bool path = _options.StepMode == ChainStepMode.CartesianPath;
            for (int a = 0; a < _profile.Arms.Length; a++)
            {
                if (!_clutches[a].IsEngaged)
                {
                    _status[a] = ChainArmStatus.Idle;
                    continue;
                }

                int n = _profile.Arms[a].Joints.Length, o = _offset[a];
                Span<double> solution = _solution.AsSpan(0, n);
                RigidTransform goal, step = default;
                bool stuck = false;
                ChainSolveResult r;
                if (!path)
                {
                    if (_absolute[a])
                    {
                        // Slide toward the hand over ApproachSteps, then follow it.
                        if (_approach[a] < 1) _approach[a] = Math.Min(1.0, _approach[a] + 1.0 / _options.ApproachSteps);
                        _clutches[a].Track(_approach[a] < 1 ? RigidTransform.Interpolate(_approachFrom[a], _handTarget[a], _approach[a]) : _handTarget[a]);
                    }

                    goal = _clutches[a].Target;
                    r = _solvers[a].Solve(goal, Commanded(a), solution);
                }
                else
                {
                    // The speed caps make the approach to a hand: no ramp.
                    if (_absolute[a]) _clutches[a].Track(_handTarget[a]);
                    RigidTransform tool = Tool(a);
                    goal = _clutches[a].Target;
                    step = Advance(tool, goal, _options.MaxLinearSpeed * seconds, _options.MaxAngularSpeed * seconds);
                    // The spare motion first, then the step solved from there: the solve puts the tool back on its path.
                    Array.Copy(_q, _candidate, _q.Length);
                    Posture(a, seconds);
                    r = _solvers[a].Solve(step, _candidate.AsSpan(o, n), solution);
                    // Stuck: the solve did not reach half of its small step (a limit, or a singularity). Less is following, if slowly.
                    stuck = r.PositionError > Math.Max(0.1 * _options.ReachTolerance, 0.5 * Distance(tool, step))
                        || r.OrientationError > Math.Max(0.1 * _options.ReachAngleTolerance, 0.5 * AngleBetween(tool, step));
                }

                Array.Copy(_q, _candidate, _q.Length);
                for (int j = 0; j < n; j++) _candidate[o + j] = solution[j];
                double shorten = Shorten(a, seconds);
                for (int j = 0; j < n; j++)
                {
                    double from = _q[o + j];
                    _candidate[o + j] = from + shorten * (_candidate[o + j] - from);
                }

                bool collided = false;
                if (_collision != null && SerialChainCollision.Check(_profile, _collision, _candidate).IsHit)
                {
                    collided = true;
                    if (!LargestClearPart(a))
                    {
                        _status[a] = ChainArmStatus.Blocked;
                        continue;
                    }
                }

                Array.Copy(_candidate, o, _q, o, n);
                if (collided) _status[a] = ChainArmStatus.Blocked;
                else if (!path) _status[a] = r.Converged || shorten < 1.0 ? ChainArmStatus.Following : ChainArmStatus.StoppedShort;
                else _status[a] = stuck && OffTarget(a, goal) ? ChainArmStatus.StoppedShort : ChainArmStatus.Following;
            }
        }

        /// <summary>The longest step <see cref="Step(double)"/> takes (s): a host that stalls for longer gets a slower arm, not a jump.</summary>
        public const double LongestStep = 0.05;

        /// <summary>
        /// The pose a step toward <paramref name="to"/> from <paramref name="from"/> reaches: along the straight line, at most
        /// <paramref name="maxDistance"/>, and along the shorter arc, at most <paramref name="maxAngle"/>; each part on its own.
        /// </summary>
        public static RigidTransform Advance(in RigidTransform from, in RigidTransform to, double maxDistance, double maxAngle)
        {
            double dx = to.X - from.X, dy = to.Y - from.Y, dz = to.Z - from.Z;
            double d = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            double move = d > maxDistance ? maxDistance / d : 1.0;
            double angle = AngleBetween(from, to);
            double turn = angle > maxAngle ? maxAngle / angle : 1.0;
            RigidTransform r = RigidTransform.Interpolate(from, to, turn);
            return new RigidTransform(from.X + move * dx, from.Y + move * dy, from.Z + move * dz, r.Qw, r.Qx, r.Qy, r.Qz);
        }

        /// <summary>The angle between two poses' orientations (rad, 0..π).</summary>
        public static double AngleBetween(in RigidTransform a, in RigidTransform b)
        {
            double dot = Math.Abs(a.Qw * b.Qw + a.Qx * b.Qx + a.Qy * b.Qy + a.Qz * b.Qz);
            return 2 * Math.Acos(Math.Min(1.0, dot));
        }

        /// <summary>
        /// Moves arm <paramref name="a"/>'s candidate (its command, before the step is solved) in its null space, so the tool
        /// stays put to first order: off its joints' limits (<see cref="SerialChainTeleopOptions.LimitReliefSpeed"/>), and away
        /// from what it is closing on, along the gradient of its clearance (<see cref="SerialChainTeleopOptions.AvoidDistance"/>).
        /// </summary>
        private void Posture(int a, double seconds)
        {
            ChainArm arm = _profile.Arms[a];
            int n = arm.Joints.Length, o = _offset[a];
            bool any = false;
            for (int j = 0; j < n; j++)
            {
                _gradient[j] = 0;
                if (!(_options.LimitReliefSpeed > 0)) continue;
                // Inside the zone before a limit, pushed back: harder the deeper, at most LimitReliefSpeed on the limit.
                ChainJointLimits l = arm.Joints[j].Limits;
                double zone = Math.Min(_options.LimitZone, 0.25 * (l.SoftUpper - l.SoftLower));
                double q = _candidate[o + j];
                double depth = q > l.SoftUpper - zone ? (l.SoftUpper - zone - q) / zone
                    : q < l.SoftLower + zone ? (l.SoftLower + zone - q) / zone
                    : 0;
                if (depth == 0) continue;
                _gradient[j] = _options.LimitReliefSpeed * seconds * Math.Max(-1.0, Math.Min(1.0, depth));
                any = true;
            }

            if (_collision != null && _options.AvoidDistance > 0)
            {
                double want = _collision.Margin + _options.AvoidDistance;
                double clearance = SerialChainCollision.Clearance(_profile, _collision, _candidate, a);
                if (clearance < want)
                {
                    const double eps = 1e-3;
                    double norm = 0;
                    for (int j = 0; j < n; j++)
                    {
                        double keep = _candidate[o + j];
                        _candidate[o + j] = keep + eps;
                        _nullStep[j] = (SerialChainCollision.Clearance(_profile, _collision, _candidate, a) - clearance) / eps;
                        _candidate[o + j] = keep;
                        norm += _nullStep[j] * _nullStep[j];
                    }

                    if (norm > 1e-18)
                    {
                        // Harder the closer it is: to first order, a step wins back AvoidGain of the shortfall (where the null space allows).
                        double push = _options.AvoidGain * (want - clearance) / norm;
                        for (int j = 0; j < n; j++) _gradient[j] += push * _nullStep[j];
                        any = true;
                    }
                }
            }

            if (!any) return;
            _solvers[a].ProjectToNullSpace(_candidate.AsSpan(o, n), _gradient.AsSpan(0, n), _nullStep.AsSpan(0, n));
            double cap = 0.5 * _options.MaxStepRadians;
            for (int j = 0; j < n; j++)
            {
                double d = Math.Max(-cap, Math.Min(cap, _nullStep[j]));
                ChainJointLimits l = arm.Joints[j].Limits;
                _candidate[o + j] = Math.Max(l.SoftLower, Math.Min(l.SoftUpper, _candidate[o + j] + d));
            }
        }

        /// <summary>
        /// How much of arm <paramref name="a"/>'s candidate step to take (0..1): all of it, unless a joint would move more than
        /// <see cref="SerialChainTeleopOptions.MaxStepRadians"/>, or faster than its share of its velocity limit.
        /// </summary>
        private double Shorten(int a, double seconds)
        {
            ChainJoint[] joints = _profile.Arms[a].Joints;
            int o = _offset[a];
            double shorten = 1.0;
            for (int j = 0; j < joints.Length; j++)
            {
                double move = Math.Abs(_candidate[o + j] - _q[o + j]);
                double cap = _options.MaxStepRadians;
                if (_options.JointSpeedShare > 0) cap = Math.Min(cap, _options.JointSpeedShare * joints[j].Limits.Velocity * seconds);
                if (move > cap) shorten = Math.Min(shorten, cap / move);
            }

            return shorten;
        }

        /// <summary>Whether arm <paramref name="a"/>'s tool is further from <paramref name="goal"/> than the operator would call there.</summary>
        private bool OffTarget(int a, in RigidTransform goal)
        {
            RigidTransform tool = Tool(a);
            return Distance(tool, goal) > _options.ReachTolerance || AngleBetween(tool, goal) > _options.ReachAngleTolerance;
        }

        private static double Distance(in RigidTransform a, in RigidTransform b) =>
            Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

        /// <summary>Cuts arm <paramref name="a"/>'s colliding candidate to the largest clear part of its step (a half, then a quarter); false when none is.</summary>
        private bool LargestClearPart(int a)
        {
            int n = _profile.Arms[a].Joints.Length, o = _offset[a];
            Array.Copy(_candidate, o, _stepped, o, n);
            for (double part = 0.5; part >= 0.25; part *= 0.5)
            {
                for (int j = 0; j < n; j++) _candidate[o + j] = _q[o + j] + part * (_stepped[o + j] - _q[o + j]);
                if (!SerialChainCollision.Check(_profile, _collision!, _candidate).IsHit) return true;
            }

            Array.Copy(_q, o, _candidate, o, n);
            return false;
        }

        /// <summary>
        /// Writes the command, every arm joint and then each gripper, arm by arm in profile order, with speed 0 (the
        /// robot's own limits shape it). Returns how many were written: 0 before a pose.
        /// </summary>
        public int WriteTargets(Span<JointTarget> targets)
        {
            if (!HasPose) return 0;
            if (targets.Length < MotorCount) throw new ArgumentException($"targets must hold {MotorCount} entries", nameof(targets));
            int k = 0;
            for (int a = 0; a < _profile.Arms.Length; a++)
            {
                ChainArm arm = _profile.Arms[a];
                for (int j = 0; j < arm.Joints.Length; j++)
                {
                    targets[k++] = new JointTarget(arm.Joints[j].MotorId, (float)_q[_offset[a] + j], 0f);
                }

                if (arm.HasGripper) targets[k++] = new JointTarget(arm.Gripper.MotorId, (float)_opening[a], 0f);
            }

            return k;
        }

        private static bool TryFind(ReadOnlySpan<JointSample> samples, byte motorId, out float value)
        {
            for (int i = 0; i < samples.Length; i++)
            {
                if (samples[i].MotorId == motorId)
                {
                    value = samples[i].Value;
                    return true;
                }
            }

            value = float.NaN;
            return false;
        }
    }
}
