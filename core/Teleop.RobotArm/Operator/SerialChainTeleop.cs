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

        /// <summary>
        /// Recovery (<see cref="ChainStepMode.CartesianPath"/>): an arm off its target whose small steps have made no progress
        /// (or are blocked) for this long (s) searches for a better pose of the whole arm, and, when one is clearly better,
        /// moves there in joint space through checked poses, then follows again. 0 never. Without it an arm that crept into a
        /// corner of its joint space (stretching after an unreachable hand, its forearm or base on a limit) never came back,
        /// though the hand returned within reach (the user, 2026-11-03: it "gets stuck and confused in some positions").
        /// </summary>
        public double RecoverAfterSeconds { get; set; } = 0.3;

        /// <summary>
        /// Whether a recovery waits for the hand to be held still (true), or also starts while it moves when the arm's steps are
        /// stuck or blocked (false). A search started while the hand moves moves the arm only to a pose that reaches the target:
        /// one merely nearer left the moving hand's path and arrived after the hand had gone.
        /// </summary>
        public bool RecoverOnlyWhenStill { get; set; } = true;

        /// <summary>How many starts a recovery search solves from (the arm's command, its home pose, then seeded random poses).</summary>
        public int RecoverStarts { get; set; } = 32;

        /// <summary>How many of those one <see cref="SerialChainTeleop.Step"/> solves: the search is spread over frames.</summary>
        public int RecoverStartsPerStep { get; set; } = 4;

        /// <summary>
        /// A found pose is moved to when it puts the tool within half the reach tolerances of the target, or this much nearer it
        /// (m, an angle weighed at <see cref="OrientationMeters"/> per radian) than the arm is: the arm's own steps finish from
        /// there. Searches run only while the hand is held still, so neither leaves a moving hand's path.
        /// </summary>
        public double RecoverImprovement { get; set; } = 0.01;

        /// <summary>
        /// The share of each joint's velocity limit a recovery move uses: below following's, so it never lunges, yet quick, since
        /// the operator is holding still and waiting (a corner can mean 4.5 rad of one joint).
        /// </summary>
        public double RecoverSpeedShare { get; set; } = 0.75;

        /// <summary>
        /// A redundant arm's spare motion is drawn toward its home pose (the robot's pose when it was first taken), at most
        /// this fast (rad/s), so the hand's wandering does not walk the elbow and the base toward their limits; 0 never.
        /// </summary>
        public double PostureSpeed { get; set; } = 0.15;

        /// <summary>A radian of the gripper's orientation error weighs this many metres of position error, when poses are compared.</summary>
        public double OrientationMeters { get; set; } = 0.3;

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
            if (!(RecoverSpeedShare > 0) || RecoverSpeedShare > 1) return $"RecoverSpeedShare must be in (0, 1] ({RecoverSpeedShare})";
            if (!(RecoverAfterSeconds >= 0) || RecoverStarts < 2 || RecoverStartsPerStep < 1 || !(RecoverImprovement >= 0) || !(PostureSpeed >= 0) || !(OrientationMeters >= 0) ||
                double.IsInfinity(RecoverAfterSeconds + RecoverImprovement + PostureSpeed + OrientationMeters)) return "the recovery and posture settings must be finite, at least 0, with at least 2 starts";
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
    /// other arm's command; a step that would collide slides along what it would hit, or is cut to its largest clear part,
    /// or is not taken and the arm holds where it was (<see cref="ChainArmStatus.Blocked"/>). RobotHost refuses the same poses on its side (docs/adr/0026 amendment).</item>
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
        private readonly double[] _probe;
        private readonly double[] _slideGradient;
        private readonly double[] _unslid;
        private readonly SerialChainSolver[] _searchSolvers;
        private readonly double[] _home;
        private readonly bool[] _homeTaken;
        private readonly bool[] _transit;
        private readonly bool[] _searching;
        private readonly double[] _transitGoal;
        private readonly RigidTransform[] _transitFor;
        private readonly double[] _searchBest;
        private readonly double[] _searchBestScore;
        private readonly double[] _searchBestError;
        private readonly double[] _searchBestPosition;
        private readonly bool[] _searchStill;
        private readonly double[] _searchBestAngle;
        private readonly int[] _searchCount;
        private readonly double[] _watchError;
        private readonly double[] _watchSeconds;
        private readonly double[] _cooldown;
        private readonly SeededRng[] _rng;
        private readonly double[] _seed;
        private readonly double[] _found;
        private readonly RigidTransform[] _watchGoal;
        private readonly int[] _searchBestRoute;
        private readonly int[] _routeStage;
        private readonly double[] _escape;
        private readonly bool[] _escapeValid;
        private readonly double[] _path;

        /// <summary>The recovery's counts since construction, for diagnostics: searches, searches that found nothing better, moves started, moves cut by a collision.</summary>
        public int Searches { get; private set; }
        public int SearchesNoBetter { get; private set; }
        public int Transits { get; private set; }
        public int TransitsBlocked { get; private set; }

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
            _probe = new double[joints];
            _slideGradient = new double[widest];
            _unslid = new double[widest];
            // The recovery's solves: a longer budget, and no pull toward where they start (they start anywhere).
            SerialChainSolverOptions o0 = _options.Solver ?? new SerialChainSolverOptions();
            var searchOptions = new SerialChainSolverOptions
            {
                Iterations = Math.Max(o0.Iterations, 150), Damping = o0.Damping, OrientationWeight = o0.OrientationWeight,
                PositionTolerance = o0.PositionTolerance, OrientationTolerance = o0.OrientationTolerance, RestGain = 0, LimitGain = o0.LimitGain,
                MaxStep = o0.MaxStep, LimitMargin = o0.LimitMargin,
            };
            _searchSolvers = new SerialChainSolver[arms];
            _rng = new SeededRng[arms];
            for (int a = 0; a < arms; a++)
            {
                _searchSolvers[a] = new SerialChainSolver(profile.Arms[a], searchOptions);
                _rng[a] = new SeededRng(0x7E1E0Bu + (ulong)a);
            }

            _home = new double[joints];
            _homeTaken = new bool[arms];
            _transit = new bool[arms];
            _searching = new bool[arms];
            _transitGoal = new double[joints];
            _transitFor = new RigidTransform[arms];
            _searchBest = new double[joints];
            _searchBestScore = new double[arms];
            _searchBestError = new double[arms];
            _searchBestPosition = new double[arms];
            _searchStill = new bool[arms];
            _searchBestAngle = new double[arms];
            _searchCount = new int[arms];
            _watchError = new double[arms];
            _watchSeconds = new double[arms];
            _cooldown = new double[arms];
            _seed = new double[widest];
            _found = new double[widest];
            _watchGoal = new RigidTransform[arms];
            _searchBestRoute = new int[arms];
            _routeStage = new int[arms];
            _escape = new double[joints];
            _escapeValid = new bool[arms];
            _path = new double[joints];
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
                if (!_homeTaken[a])
                {
                    Array.Copy(_q, _offset[a], _home, _offset[a], arm.Joints.Length);
                    _homeTaken[a] = true;
                }

                ResetRecovery(a);
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
            ResetRecovery(arm);
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
            ResetRecovery(arm);
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
            ResetRecovery(arm);
        }

        /// <summary>Whether arm <paramref name="arm"/> is moving through joint space to a pose its recovery found (<see cref="SerialChainTeleopOptions.RecoverAfterSeconds"/>).</summary>
        public bool IsRecovering(int arm) => _transit[arm];

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
                    goal = _clutches[a].Target;
                    if (_transit[a])
                    {
                        Transit(a, goal, seconds);
                        continue;
                    }

                    RigidTransform tool = Tool(a);
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

                bool collided = false, slid = false;
                if (_collision != null && SerialChainCollision.Check(_profile, _collision, _candidate).IsHit)
                {
                    collided = true;
                    slid = path && Slide(a, step, seconds);
                    if (!slid && !LargestClearPart(a))
                    {
                        _status[a] = ChainArmStatus.Blocked;
                        if (path) Watch(a, goal, seconds, trouble: true); // a blocked arm looks for another way too
                        continue;
                    }
                }

                Array.Copy(_candidate, o, _q, o, n);
                // Sliding along an obstacle is blocked only while it keeps the tool off its target.
                if (collided && !(slid && !OffTarget(a, goal))) _status[a] = ChainArmStatus.Blocked;
                else if (!path) _status[a] = r.Converged || shorten < 1.0 ? ChainArmStatus.Following : ChainArmStatus.StoppedShort;
                else _status[a] = stuck && OffTarget(a, goal) ? ChainArmStatus.StoppedShort : ChainArmStatus.Following;
                if (path) Watch(a, goal, seconds, stuck || collided);
            }
        }

        private void ResetRecovery(int a)
        {
            _transit[a] = _searching[a] = false;
            _watchSeconds[a] = _cooldown[a] = 0;
            _watchError[a] = double.PositiveInfinity;
        }

        /// <summary>How far a pose is from <paramref name="goal"/>, in metres, an angle weighed at <see cref="SerialChainTeleopOptions.OrientationMeters"/> per radian.</summary>
        private double Miss(in RigidTransform tool, in RigidTransform goal) => Distance(tool, goal) + _options.OrientationMeters * AngleBetween(tool, goal);

        /// <summary>
        /// The recovery's watch, after each step of arm <paramref name="a"/>: off its target, with its steps in
        /// <paramref name="trouble"/> (stuck, or colliding) and no progress for <see cref="SerialChainTeleopOptions.RecoverAfterSeconds"/>,
        /// it starts a search; a search under way solves a few more starts.
        /// </summary>
        private void Watch(int a, in RigidTransform goal, double seconds, bool trouble)
        {
            if (!(_options.RecoverAfterSeconds > 0)) return;
            double miss = Miss(Tool(a), goal);
            if (!OffTarget(a, goal))
            {
                _searching[a] = false;
                _watchError[a] = miss;
                _watchSeconds[a] = 0;
                return;
            }

            if (_searching[a])
            {
                Search(a, goal);
                return;
            }

            if (_cooldown[a] > 0) _cooldown[a] -= seconds;
            // A hand held still makes any lack of progress trouble, even when each small step succeeds: an arm whose posture terms
            // undo them (a wrist on its limit, turning the long way) jitters in place reporting Following.
            bool still = Distance(goal, _watchGoal[a]) <= 0.01 && AngleBetween(goal, _watchGoal[a]) <= 0.05;
            if (!still) _watchGoal[a] = goal;
            // Progress is the gap closing by 30% in a watch period: an arm on a limit creeping the long way round (2° a tenth of a
            // second, against 3 rad/s free) is stuck, as the operator sees it.
            double progress = Math.Max(0.003, 0.3 * _watchError[a]);
            // Only while the hand is held still: a moving hand is followed along its path, and the branch the arm is on usually
            // catches up; reconfiguring then left the path and arrived after the hand had gone (wandering hands: off target 5% -> 10%).
            if ((still || (trouble && !_options.RecoverOnlyWhenStill)) && miss > _watchError[a] - progress)
            {
                _watchSeconds[a] += seconds;
            }
            else
            {
                _watchError[a] = miss;
                _watchSeconds[a] = 0;
            }

            if (_watchSeconds[a] >= _options.RecoverAfterSeconds && _cooldown[a] <= 0)
            {
                _searching[a] = true;
                _searchStill[a] = still;
                Searches++;
                _searchCount[a] = 0;
                _searchBestScore[a] = double.PositiveInfinity;
                _searchBestError[a] = double.PositiveInfinity;
                _watchSeconds[a] = 0;
                _watchError[a] = miss;
                Search(a, goal);
            }
        }

        /// <summary>
        /// Solves a few of the recovery's starts for <paramref name="goal"/>: the arm's command, its home pose, then seeded
        /// random poses inside the soft limits. A solution must be clear of the body and the other arm. The best reaches the
        /// target most nearly, then is nearest the command, then nearest home. When every start is solved, an arm that the best
        /// would bring clearly nearer its target moves there (<see cref="Transit"/>); otherwise it waits a second to search again.
        /// </summary>
        private void Search(int a, in RigidTransform goal)
        {
            ChainArm arm = _profile.Arms[a];
            int n = arm.Joints.Length, o = _offset[a];
            Span<double> seed = _seed.AsSpan(0, n), found = _found.AsSpan(0, n);
            if (_searchCount[a] == 0) Escape(a);
            for (int k = 0; k < _options.RecoverStartsPerStep && _searchCount[a] < _options.RecoverStarts; k++, _searchCount[a]++)
            {
                int c = _searchCount[a];
                for (int j = 0; j < n; j++)
                {
                    ChainJointLimits l = arm.Joints[j].Limits;
                    seed[j] = c == 0 ? _q[o + j] : c == 1 ? _home[o + j] : l.SoftLower + (l.SoftUpper - l.SoftLower) * _rng[a].NextDouble();
                }

                ChainSolveResult r = _searchSolvers[a].Solve(goal, seed, found);
                Array.Copy(_q, _probe, _q.Length);
                for (int j = 0; j < n; j++) _probe[o + j] = found[j];
                if (_collision != null && SerialChainCollision.Check(_profile, _collision, _probe).IsHit) continue;
                double error = r.PositionError + _options.OrientationMeters * r.OrientationError;
                double travel = 0, fromHome = 0;
                for (int j = 0; j < n; j++)
                {
                    // How long the move takes: the slowest joint's, at the recovery's share of its velocity limit.
                    double v = _options.RecoverSpeedShare * arm.Joints[j].Limits.Velocity;
                    travel = Math.Max(travel, Math.Abs(found[j] - _q[o + j]) / (v > 1e-9 ? v : 1.0));
                    fromHome += (found[j] - _home[o + j]) * (found[j] - _home[o + j]);
                }

                // Reaching the target first, then the quickest move (a second weighs a centimetre), then nearness to home.
                double score = error + 0.01 * travel + 0.001 * Math.Sqrt(fromHome / n);
                if (score >= _searchBestScore[a]) continue;
                // Only a pose the arm can get to, every sampled pose on the way clear: straight there; or by way of home; or, for an
                // arm pressed against something so that every straight line starts into it, first away from it, then home.
                int route = 0;
                if (!PathClear(a, _q.AsSpan(o, n), found))
                {
                    if (PathClear(a, _q.AsSpan(o, n), _home.AsSpan(o, n)) && PathClear(a, _home.AsSpan(o, n), found)) route = 1;
                    else if (_escapeValid[a] && PathClear(a, _escape.AsSpan(o, n), _home.AsSpan(o, n)) && PathClear(a, _home.AsSpan(o, n), found)) route = 2;
                    else continue;
                    score += 0.002 * route; // the longer way
                    if (score >= _searchBestScore[a]) continue;
                }

                _searchBestScore[a] = score;
                _searchBestError[a] = error;
                _searchBestPosition[a] = r.PositionError;
                _searchBestAngle[a] = r.OrientationError;
                _searchBestRoute[a] = route;
                for (int j = 0; j < n; j++) _searchBest[o + j] = found[j];
            }

            if (_searchCount[a] < _options.RecoverStarts) return;
            _searching[a] = false;
            // Better: it reaches the target, which the arm is off.
            double now = Miss(Tool(a), goal);
            bool reaches = _searchBestPosition[a] <= 0.5 * _options.ReachTolerance && _searchBestAngle[a] <= 0.5 * _options.ReachAngleTolerance;
            if ((reaches && now > _searchBestError[a] + 0.003) || (_searchStill[a] && _searchBestError[a] < now - _options.RecoverImprovement))
            {
                _routeStage[a] = _searchBestRoute[a];
                Array.Copy(_routeStage[a] == 2 ? _escape : _routeStage[a] == 1 ? _home : _searchBest, o, _transitGoal, o, n);
                _transitFor[a] = goal;
                _transit[a] = true;
                Transits++;
            }
            else
            {
                // Nothing better this time; the next search draws other random starts (near the body most solutions collide).
                _cooldown[a] = 0.3;
                SearchesNoBetter++;
            }
        }

        /// <summary>
        /// One step of arm <paramref name="a"/>'s move to the pose its recovery found: straight in joint space, every joint as
        /// fast as <see cref="Shorten"/> allows, each pose checked. A pose that would collide ends the move (the arm holds, and
        /// searches again later); so does arriving, or the target moving far from the one the pose was found for.
        /// </summary>
        private void Transit(int a, in RigidTransform goal, double seconds)
        {
            int n = _profile.Arms[a].Joints.Length, o = _offset[a];
            if (Distance(goal, _transitFor[a]) > 0.15 || AngleBetween(goal, _transitFor[a]) > 0.5)
            {
                // The hand went elsewhere meanwhile: search again for where it is now.
                _transit[a] = false;
                _searching[a] = true;
                _searchStill[a] = false;
                _searchCount[a] = 0;
                _searchBestScore[a] = _searchBestError[a] = double.PositiveInfinity;
                Search(a, goal);
                _status[a] = ChainArmStatus.Following;
                return;
            }

            Array.Copy(_q, _candidate, _q.Length);
            Array.Copy(_transitGoal, o, _candidate, o, n);
            double shorten = Shorten(a, seconds * _options.RecoverSpeedShare);
            double left = 0;
            for (int j = 0; j < n; j++)
            {
                double from = _q[o + j];
                _candidate[o + j] = from + shorten * (_transitGoal[o + j] - from);
                left = Math.Max(left, Math.Abs(_transitGoal[o + j] - _candidate[o + j]));
            }

            if (_collision != null && SerialChainCollision.Check(_profile, _collision, _candidate).IsHit)
            {
                // Cut by a collision (the other arm moved into the way since the path was checked): search again soon, from here.
                _transit[a] = false;
                _cooldown[a] = 0.2;
                TransitsBlocked++;
                _status[a] = ChainArmStatus.Blocked;
                return;
            }

            Array.Copy(_candidate, o, _q, o, n);
            _status[a] = ChainArmStatus.Following;
            if (left < 1e-4)
            {
                if (_routeStage[a] > 0)
                {
                    // A waypoint reached: from the escape on to home, from home on to the pose itself.
                    _routeStage[a]--;
                    Array.Copy(_routeStage[a] == 1 ? _home : _searchBest, o, _transitGoal, o, n);
                    return;
                }

                _transit[a] = false;
                _watchSeconds[a] = 0;
                _watchError[a] = Miss(Tool(a), goal);
            }
        }

        /// <summary>
        /// The escape for arm <paramref name="a"/>'s search: a pose a quarter of a radian away (the largest joint's move) that
        /// opens at least 5 mm more clearance to the body and the other arm, reached along a clear straight line. Tried in turn:
        /// up the gradient of the clearance, then each joint either way, then seeded random directions; the first that works.
        /// The gradient alone fails an arm touching two things at once: moving off one presses it into the other.
        /// </summary>
        private void Escape(int a)
        {
            _escapeValid[a] = false;
            if (_collision == null) return;
            ChainArm arm = _profile.Arms[a];
            int n = arm.Joints.Length, o = _offset[a];
            Array.Copy(_q, _probe, _q.Length);
            double clearance = SerialChainCollision.Clearance(_profile, _collision, _probe, a);
            const double eps = 1e-4, reach = 0.25;
            for (int j = 0; j < n; j++)
            {
                _probe[o + j] = _q[o + j] + eps;
                _slideGradient[j] = (SerialChainCollision.Clearance(_profile, _collision, _probe, a) - clearance) / eps;
                _probe[o + j] = _q[o + j];
            }

            int candidates = 1 + 2 * n + 8;
            for (int c = 0; c < candidates; c++)
            {
                double largest = 0;
                for (int j = 0; j < n; j++)
                {
                    double d = c == 0 ? _slideGradient[j]
                        : c <= 2 * n ? ((c - 1) / 2 == j ? ((c - 1) % 2 == 0 ? 1.0 : -1.0) : 0.0)
                        : 2 * _rng[a].NextDouble() - 1;
                    _found[j] = d;
                    largest = Math.Max(largest, Math.Abs(d));
                }

                if (!(largest > 1e-9)) continue;
                for (int j = 0; j < n; j++)
                {
                    ChainJointLimits l = arm.Joints[j].Limits;
                    _escape[o + j] = Math.Max(l.SoftLower, Math.Min(l.SoftUpper, _q[o + j] + reach * _found[j] / largest));
                }

                Array.Copy(_q, _probe, _q.Length);
                Array.Copy(_escape, o, _probe, o, n);
                if (SerialChainCollision.Clearance(_profile, _collision, _probe, a) < clearance + 0.005) continue;
                if (PathClear(a, _q.AsSpan(o, n), _escape.AsSpan(o, n)))
                {
                    _escapeValid[a] = true;
                    return;
                }
            }
        }

        /// <summary>Whether arm <paramref name="a"/> can move straight in joint space from <paramref name="from"/> to <paramref name="to"/>: every pose a twentieth of a radian apart clear, the other arm where it is.</summary>
        private bool PathClear(int a, ReadOnlySpan<double> from, ReadOnlySpan<double> to)
        {
            if (_collision == null) return true;
            int n = _profile.Arms[a].Joints.Length, o = _offset[a];
            double far = 0;
            for (int j = 0; j < n; j++) far = Math.Max(far, Math.Abs(to[j] - from[j]));
            int samples = Math.Max(1, (int)Math.Ceiling(far / 0.05));
            Array.Copy(_q, _path, _q.Length);
            for (int k = 1; k <= samples; k++)
            {
                double u = (double)k / samples;
                for (int j = 0; j < n; j++) _path[o + j] = from[j] + u * (to[j] - from[j]);
                if (SerialChainCollision.Check(_profile, _collision, _path).IsHit) return false;
            }

            return true;
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

            if (_options.PostureSpeed > 0 && _homeTaken[a])
            {
                // Toward home, at most PostureSpeed: the null space keeps the tool where it is.
                for (int j = 0; j < n; j++)
                {
                    double d = (_home[o + j] - _candidate[o + j]) / 0.5;
                    _gradient[j] += _options.PostureSpeed * seconds * Math.Max(-1.0, Math.Min(1.0, d));
                }

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

        /// <summary>
        /// Replaces arm <paramref name="a"/>'s colliding candidate with the step toward <paramref name="step"/> that comes no
        /// closer to what the arm would hit (and backs out to <see cref="SlideClearance"/> when inside it), so the arm slides
        /// along the obstacle instead of the whole step being refused. The step is the solver's
        /// <see cref="SerialChainSolver.ConstrainedStep"/>: the elbow's spare motion takes the constraint first, then the
        /// least error in the tool's pose. Without it an arm at the collision margin was held in every direction that came a
        /// hair closer, and the operator felt it stick there (the 2026-11-01 Isaac session: the right upper arm against the
        /// head). True when the slide is clear and still helps: the tool moves toward <paramref name="step"/> (sideways along
        /// the obstacle too, never away) and turns no further from it. Otherwise the candidate is put back as it was, for the
        /// cut to its largest clear part.
        /// </summary>
        private bool Slide(int a, in RigidTransform step, double seconds)
        {
            ChainArm arm = _profile.Arms[a];
            int n = arm.Joints.Length, o = _offset[a];
            Array.Copy(_candidate, o, _unslid, 0, n);
            Array.Copy(_q, _probe, _q.Length);
            double clearance = SerialChainCollision.Clearance(_profile, _collision!, _probe, a);
            const double eps = 1e-4;
            for (int j = 0; j < n; j++)
            {
                _probe[o + j] = _q[o + j] + eps;
                _slideGradient[j] = (SerialChainCollision.Clearance(_profile, _collision!, _probe, a) - clearance) / eps;
                _probe[o + j] = _q[o + j];
            }

            double change = Math.Max(0, _collision!.Margin + SlideClearance - clearance);
            if (_solvers[a].ConstrainedStep(Commanded(a), step, _slideGradient.AsSpan(0, n), change, _nullStep.AsSpan(0, n)))
            {
                for (int j = 0; j < n; j++) _candidate[o + j] = _q[o + j] + _nullStep[j];
                double shorten = Shorten(a, seconds);
                for (int j = 0; j < n; j++)
                {
                    ChainJointLimits l = arm.Joints[j].Limits;
                    double q = _q[o + j] + shorten * _nullStep[j];
                    _candidate[o + j] = Math.Max(l.SoftLower, Math.Min(l.SoftUpper, q));
                }

                if (!SerialChainCollision.Check(_profile, _collision!, _candidate).IsHit)
                {
                    RigidTransform from = Tool(a), to = SerialChainKinematics.Tool(arm, _candidate.AsSpan(o, n));
                    double toward = (to.X - from.X) * (step.X - from.X) + (to.Y - from.Y) * (step.Y - from.Y) + (to.Z - from.Z) * (step.Z - from.Z);
                    bool moves = Distance(from, step) < 1e-4 || toward > 0;
                    bool turns = AngleBetween(to, step) <= AngleBetween(from, step) + 0.01;
                    if (moves && turns) return true;
                }
            }

            Array.Copy(_unslid, 0, _candidate, o, n);
            return false;
        }

        /// <summary>The gap above the collision margin a slide keeps (m): room for what the first-order cut misses.</summary>
        private const double SlideClearance = 0.002;

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
