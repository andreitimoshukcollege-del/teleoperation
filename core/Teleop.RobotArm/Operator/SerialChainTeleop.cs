using System;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>The two-arm operator's settings (docs/adr/0029). Defaults are first guesses, tuned on the headset.</summary>
    public sealed class SerialChainTeleopOptions
    {
        /// <summary>Hand motion → tool motion (<see cref="PoseClutch.PositionScale"/>). One to one by default: the FR3 reaches about as far as an arm.</summary>
        public double PositionScale { get; set; } = 1.0;

        /// <summary>
        /// The most any joint's command may change in one <see cref="SerialChainTeleop.Step"/> (radians). A larger solution
        /// (the solver finding another branch, a hand reappearing elsewhere) is shortened to this, so every step's pose is
        /// near the last checked one and the robot never travels a long unchecked path between two checked poses.
        /// </summary>
        public double MaxStepRadians { get; set; } = 0.1;

        /// <summary>The solver's settings; its defaults when null.</summary>
        public SerialChainSolverOptions? Solver { get; set; }

        public string? Validate()
        {
            if (!(PositionScale > 0) || double.IsInfinity(PositionScale)) return $"PositionScale must be positive and finite ({PositionScale})";
            if (!(MaxStepRadians > 0) || double.IsInfinity(MaxStepRadians)) return $"MaxStepRadians must be positive and finite ({MaxStepRadians})";
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

        /// <summary>Clutched, but the target is out of reach: the command is the closest pose inside the joint limits.</summary>
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
    /// other arm's command; a step that would collide is not taken, and the arm holds where it was
    /// (<see cref="ChainArmStatus.Blocked"/>). RobotHost refuses the same poses on its side (docs/adr/0026 amendment).</item>
    /// <item>No joint's command moves more than <see cref="SerialChainTeleopOptions.MaxStepRadians"/> per step.</item>
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

        /// <summary>Stops arm <paramref name="arm"/> following; it holds its command, and its target moves to where the arm is.</summary>
        public void Release(int arm)
        {
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

        /// <summary>Solves every clutched arm toward its target and commits each step that is clear.</summary>
        public void Step()
        {
            if (!HasPose) return;
            for (int a = 0; a < _profile.Arms.Length; a++)
            {
                if (!_clutches[a].IsEngaged)
                {
                    _status[a] = ChainArmStatus.Idle;
                    continue;
                }

                int n = _profile.Arms[a].Joints.Length;
                Span<double> solution = _solution.AsSpan(0, n);
                ChainSolveResult r = _solvers[a].Solve(_clutches[a].Target, Commanded(a), solution);

                double largest = 0;
                for (int j = 0; j < n; j++) largest = Math.Max(largest, Math.Abs(solution[j] - _q[_offset[a] + j]));
                double shorten = largest > _options.MaxStepRadians ? _options.MaxStepRadians / largest : 1.0;

                Array.Copy(_q, _candidate, _q.Length);
                for (int j = 0; j < n; j++)
                {
                    double from = _q[_offset[a] + j];
                    _candidate[_offset[a] + j] = from + shorten * (solution[j] - from);
                }

                if (_collision != null && SerialChainCollision.Check(_profile, _collision, _candidate).IsHit)
                {
                    _status[a] = ChainArmStatus.Blocked;
                    continue;
                }

                Array.Copy(_candidate, _offset[a], _q, _offset[a], n);
                _status[a] = r.Converged || shorten < 1.0 ? ChainArmStatus.Following : ChainArmStatus.StoppedShort;
            }
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
