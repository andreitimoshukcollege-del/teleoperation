using System;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Kinematics
{
    /// <summary>The solver's tuning (docs/adr/0026 amendment). The defaults suit a 7-joint FR3 arm at teleoperation rates.</summary>
    public sealed class SerialChainSolverOptions
    {
        /// <summary>Iterations per solve: a fixed budget, so a solve's cost is bounded and deterministic.</summary>
        public int Iterations { get; set; } = 40;

        /// <summary>The damping of the least-squares step (metres): bigger is slower but steadier near singularities.</summary>
        public double Damping { get; set; } = 0.02;

        /// <summary>How much a radian of orientation error weighs against a metre of position error.</summary>
        public double OrientationWeight { get; set; } = 0.3;

        public double PositionTolerance { get; set; } = 1e-4;
        public double OrientationTolerance { get; set; } = 1e-3;

        /// <summary>
        /// The null-space pull toward where the arm was (per iteration), and away from its soft limits. Weak on purpose: at 0.1
        /// it held the arm back from far targets (62 of 200 random ones from the ready pose converged, against 195 at 0.02).
        /// </summary>
        public double RestGain { get; set; } = 0.02;
        public double LimitGain { get; set; } = 0.01;

        /// <summary>The largest change of one joint in one iteration (radians, or metres for a prismatic joint).</summary>
        public double MaxStep { get; set; } = 0.2;

        /// <summary>Solutions are kept this far inside the soft limits.</summary>
        public double LimitMargin { get; set; } = 0.02;
    }

    /// <summary>What a solve reached.</summary>
    public readonly struct ChainSolveResult
    {
        public readonly bool Converged;
        public readonly double PositionError;
        public readonly double OrientationError;
        public readonly int Iterations;

        public ChainSolveResult(bool converged, double positionError, double orientationError, int iterations)
        {
            Converged = converged;
            PositionError = positionError;
            OrientationError = orientationError;
            Iterations = iterations;
        }
    }

    /// <summary>
    /// Inverse kinematics for one arm of a <see cref="SerialChainProfile"/> (docs/adr/0026 amendment): the joints that put the
    /// tool centre point at a target pose, starting from the arm's current joints.
    /// <list type="number">
    /// <item><b>The step:</b> damped least squares on the 6-D error (position, and orientation as a rotation vector weighted by
    /// <see cref="SerialChainSolverOptions.OrientationWeight"/>), with the geometric Jacobian.</item>
    /// <item><b>The null space:</b> a redundant arm's free motion is pulled toward the joints the solve started from, and away
    /// from the soft limits. A 7-joint arm then stays near where it is rather than flipping its elbow.</item>
    /// <item><b>The bounds:</b> each step is clamped to <see cref="SerialChainSolverOptions.MaxStep"/> per joint, and every
    /// iterate is kept inside the soft limits.</item>
    /// </list>
    /// A fixed number of iterations at most, in double precision. Deterministic and allocation-free: everything lives in
    /// buffers made once. It does not check collisions: the caller checks the whole robot's pose
    /// (<c>SerialChainCollision</c>), since the other arm matters too.
    /// </summary>
    public sealed class SerialChainSolver
    {
        public const int MaxJoints = 12;

        private readonly ChainArm _arm;
        private readonly SerialChainSolverOptions _o;
        private readonly int _n;
        private readonly RigidTransform[] _frames;
        private readonly double[] _q, _rest, _j, _a, _e, _dq, _v, _w, _lo, _hi, _h, _hf, _hg;
        private readonly bool[] _locked;

        public SerialChainSolver(ChainArm arm, SerialChainSolverOptions? options = null)
        {
            _arm = arm ?? throw new ArgumentNullException(nameof(arm));
            _o = options ?? new SerialChainSolverOptions();
            _n = arm.Joints.Length;
            if (_n > MaxJoints) throw new ArgumentException($"at most {MaxJoints} joints", nameof(arm));
            if (_o.Iterations <= 0 || !(_o.Damping > 0) || !(_o.MaxStep > 0)) throw new ArgumentException("iterations, damping and the step must be positive", nameof(options));
            _frames = new RigidTransform[_n + 1];
            _q = new double[_n];
            _rest = new double[_n];
            _j = new double[6 * _n];
            _a = new double[36];
            _e = new double[6];
            _dq = new double[_n];
            _v = new double[_n];
            _w = new double[6];
            _lo = new double[_n];
            _hi = new double[_n];
            _h = new double[_n * _n];
            _hf = new double[_n];
            _hg = new double[_n];
            _locked = new bool[_n];
            for (int i = 0; i < _n; i++)
            {
                ChainJointLimits l = arm.Joints[i].Limits;
                _lo[i] = l.SoftLower + _o.LimitMargin;
                _hi[i] = l.SoftUpper - _o.LimitMargin;
                if (_lo[i] > _hi[i]) _lo[i] = _hi[i] = 0.5 * (l.SoftLower + l.SoftUpper);
            }
        }

        public ChainArm Arm => _arm;

        /// <summary>
        /// Solves for <paramref name="target"/> (the TCP's pose in the robot's root frame) from <paramref name="current"/>, into
        /// <paramref name="solution"/>. It always writes a pose inside the soft limits: the target's if it converged, otherwise the
        /// closest it got.
        /// </summary>
        public ChainSolveResult Solve(in RigidTransform target, ReadOnlySpan<double> current, Span<double> solution)
        {
            if (current.Length != _n || solution.Length < _n) throw new ArgumentException($"the arm has {_n} joints");
            for (int i = 0; i < _n; i++)
            {
                double q = current[i];
                _rest[i] = q;
                _q[i] = q < _lo[i] ? _lo[i] : q > _hi[i] ? _hi[i] : q;
            }

            double pos = double.PositiveInfinity, rot = double.PositiveInfinity;
            int it = 0;
            for (; it <= _o.Iterations; it++)
            {
                SerialChainKinematics.ComputeFrames(_arm, _q, _frames);
                Error(target, _frames[_n], out pos, out rot);
                if (pos <= _o.PositionTolerance && rot <= _o.OrientationTolerance) break;
                if (it == _o.Iterations) break;
                Jacobian();
                StepDls();
                for (int i = 0; i < _n; i++)
                {
                    double d = _dq[i];
                    if (d > _o.MaxStep) d = _o.MaxStep;
                    if (d < -_o.MaxStep) d = -_o.MaxStep;
                    double q = _q[i] + d;
                    _q[i] = q < _lo[i] ? _lo[i] : q > _hi[i] ? _hi[i] : q;
                }
            }

            for (int i = 0; i < _n; i++) solution[i] = _q[i];
            bool converged = pos <= _o.PositionTolerance && rot <= _o.OrientationTolerance;
            return new ChainSolveResult(converged, pos, rot, it);
        }

        /// <summary>
        /// The part of <paramref name="v"/> (a joint motion) that does not move the tool at <paramref name="q"/>, to first order:
        /// <c>(I − J⁺ J) v</c>, with the solver's damped pseudo-inverse. A redundant arm's spare motion: what the teleop uses to
        /// bend an elbow away from an obstacle while the gripper stays on its path. Allocation-free.
        /// </summary>
        public void ProjectToNullSpace(ReadOnlySpan<double> q, ReadOnlySpan<double> v, Span<double> result)
        {
            if (q.Length != _n || v.Length < _n || result.Length < _n) throw new ArgumentException($"the arm has {_n} joints");
            for (int i = 0; i < _n; i++) _q[i] = q[i];
            SerialChainKinematics.ComputeFrames(_arm, _q, _frames);
            Jacobian();
            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++)
                {
                    double sum = 0;
                    for (int i = 0; i < _n; i++) sum += _j[r * _n + i] * _j[c * _n + i];
                    _a[r * 6 + c] = sum + (r == c ? _o.Damping * _o.Damping : 0);
                }
            }

            Cholesky(_a);
            for (int r = 0; r < 6; r++)
            {
                double sum = 0;
                for (int i = 0; i < _n; i++) sum += _j[r * _n + i] * v[i];
                _w[r] = sum;
            }

            CholeskySolve(_a, _w);
            for (int i = 0; i < _n; i++)
            {
                double sum = 0;
                for (int r = 0; r < 6; r++) sum += _j[r * _n + i] * _w[r];
                result[i] = v[i] - sum;
            }
        }

        /// <summary>
        /// One damped least-squares step from <paramref name="q"/> toward <paramref name="target"/> that changes
        /// <c>g · q</c> by exactly <paramref name="change"/> (to first order), into <paramref name="step"/>: the motion that
        /// moves the tool most nearly as asked under one linear constraint, such as "no closer to what the arm is touching".
        /// The correction is taken in the step's own metric (<c>JᵀJ + λ²I</c>), so it is spent first where it costs the tool
        /// least: the arm's spare motion, then the smallest error in the tool's pose. A joint on its bound (the soft limit
        /// less the margin) that the step would push past it is held there, and the step taken again by the others, so an
        /// arm pinned at a limit can still slide. False when the free joints cannot change <c>g · q</c> at all.
        /// Allocation-free.
        /// </summary>
        public bool ConstrainedStep(ReadOnlySpan<double> q, in RigidTransform target, ReadOnlySpan<double> g, double change, Span<double> step)
        {
            if (q.Length != _n || g.Length < _n || step.Length < _n) throw new ArgumentException($"the arm has {_n} joints");
            for (int i = 0; i < _n; i++)
            {
                _q[i] = q[i];
                _locked[i] = false;
            }

            SerialChainKinematics.ComputeFrames(_arm, _q, _frames);
            Error(target, _frames[_n], out _, out _);
            Jacobian();
            // A joint on a bound that the step would push past it is held there, and the step is taken again by the others.
            for (int pass = 0; pass <= _n; pass++)
            {
                // H = JᵀJ + λ²I over the free joints (a held joint's row and column are the identity), f = Jᵀe, both held at 0.
                for (int r = 0; r < _n; r++)
                {
                    double f = 0;
                    if (!_locked[r]) for (int k = 0; k < 6; k++) f += _j[k * _n + r] * _e[k];
                    _hf[r] = f;
                    _hg[r] = _locked[r] ? 0 : g[r];
                    for (int c = 0; c < _n; c++)
                    {
                        double sum = 0;
                        if (!_locked[r] && !_locked[c]) for (int k = 0; k < 6; k++) sum += _j[k * _n + r] * _j[k * _n + c];
                        _h[r * _n + c] = sum + (r == c ? (_locked[r] ? 1.0 : _o.Damping * _o.Damping) : 0);
                    }
                }

                CholeskyN(_h, _n);
                CholeskySolveN(_h, _n, _hf);   // the unconstrained step
                CholeskySolveN(_h, _n, _hg);   // H⁻¹ g
                double gHg = 0, gStep = 0;
                for (int i = 0; i < _n; i++)
                {
                    if (_locked[i]) continue;
                    gHg += g[i] * _hg[i];
                    gStep += g[i] * _hf[i];
                }

                if (!(gHg > 1e-300)) return false;
                double mu = (change - gStep) / gHg;
                bool held = false;
                for (int i = 0; i < _n; i++)
                {
                    step[i] = _locked[i] ? 0 : _hf[i] + mu * _hg[i];
                    if (!_locked[i] && ((_q[i] <= _lo[i] + 1e-9 && step[i] < 0) || (_q[i] >= _hi[i] - 1e-9 && step[i] > 0)))
                    {
                        _locked[i] = true;
                        held = true;
                    }
                }

                if (!held) break;
            }

            for (int i = 0; i < _n; i++) if (_locked[i]) step[i] = 0;
            return true;
        }

        /// <summary>In place, the lower triangle of a symmetric positive-definite n×n.</summary>
        private static void CholeskyN(double[] a, int n)
        {
            for (int j = 0; j < n; j++)
            {
                double d = a[j * n + j];
                for (int k = 0; k < j; k++) d -= a[j * n + k] * a[j * n + k];
                d = Math.Sqrt(d > 1e-300 ? d : 1e-300);
                a[j * n + j] = d;
                for (int i = j + 1; i < n; i++)
                {
                    double s = a[i * n + j];
                    for (int k = 0; k < j; k++) s -= a[i * n + k] * a[j * n + k];
                    a[i * n + j] = s / d;
                }
            }
        }

        private static void CholeskySolveN(double[] l, int n, double[] b)
        {
            for (int i = 0; i < n; i++)
            {
                double s = b[i];
                for (int k = 0; k < i; k++) s -= l[i * n + k] * b[k];
                b[i] = s / l[i * n + i];
            }

            for (int i = n - 1; i >= 0; i--)
            {
                double s = b[i];
                for (int k = i + 1; k < n; k++) s -= l[k * n + i] * b[k];
                b[i] = s / l[i * n + i];
            }
        }

        /// <summary>The weighted 6-D error into <c>_e</c>: target position minus the TCP's, then the rotation vector taking the TCP's orientation to the target's.</summary>
        private void Error(in RigidTransform target, in RigidTransform tcp, out double pos, out double rot)
        {
            _e[0] = target.X - tcp.X;
            _e[1] = target.Y - tcp.Y;
            _e[2] = target.Z - tcp.Z;
            pos = Math.Sqrt(_e[0] * _e[0] + _e[1] * _e[1] + _e[2] * _e[2]);
            // q_err = q_target * conj(q_tcp), on the short way round.
            double w = target.Qw * tcp.Qw + target.Qx * tcp.Qx + target.Qy * tcp.Qy + target.Qz * tcp.Qz;
            double x = -target.Qw * tcp.Qx + target.Qx * tcp.Qw - target.Qy * tcp.Qz + target.Qz * tcp.Qy;
            double y = -target.Qw * tcp.Qy + target.Qx * tcp.Qz + target.Qy * tcp.Qw - target.Qz * tcp.Qx;
            double z = -target.Qw * tcp.Qz - target.Qx * tcp.Qy + target.Qy * tcp.Qx + target.Qz * tcp.Qw;
            if (w < 0)
            {
                w = -w;
                x = -x;
                y = -y;
                z = -z;
            }

            double s = Math.Sqrt(x * x + y * y + z * z);
            double angle = 2 * Math.Atan2(s, w);
            double k = s > 1e-12 ? angle / s : 2.0;
            rot = angle;
            _e[3] = _o.OrientationWeight * x * k;
            _e[4] = _o.OrientationWeight * y * k;
            _e[5] = _o.OrientationWeight * z * k;
        }

        /// <summary>The geometric Jacobian into <c>_j</c> (6 rows × n, row-major), orientation rows weighted as the error is.</summary>
        private void Jacobian()
        {
            RigidTransform tcp = _frames[_n];
            for (int i = 0; i < _n; i++)
            {
                ChainJoint joint = _arm.Joints[i];
                _frames[i].Rotate(joint.AxisX, joint.AxisY, joint.AxisZ, out double zx, out double zy, out double zz);
                if (joint.Type == ChainJointType.Revolute)
                {
                    double rx = tcp.X - _frames[i].X, ry = tcp.Y - _frames[i].Y, rz = tcp.Z - _frames[i].Z;
                    _j[0 * _n + i] = zy * rz - zz * ry;
                    _j[1 * _n + i] = zz * rx - zx * rz;
                    _j[2 * _n + i] = zx * ry - zy * rx;
                    _j[3 * _n + i] = _o.OrientationWeight * zx;
                    _j[4 * _n + i] = _o.OrientationWeight * zy;
                    _j[5 * _n + i] = _o.OrientationWeight * zz;
                }
                else
                {
                    _j[0 * _n + i] = zx;
                    _j[1 * _n + i] = zy;
                    _j[2 * _n + i] = zz;
                    _j[3 * _n + i] = _j[4 * _n + i] = _j[5 * _n + i] = 0;
                }
            }
        }

        /// <summary>
        /// dq = J⁺ e + (I − J⁺ J) v, with J⁺ = Jᵀ (J Jᵀ + λ² I)⁻¹ and v the null-space pull (toward the rest pose, away from the
        /// limits), into <c>_dq</c>.
        /// </summary>
        private void StepDls()
        {
            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++)
                {
                    double sum = 0;
                    for (int i = 0; i < _n; i++) sum += _j[r * _n + i] * _j[c * _n + i];
                    _a[r * 6 + c] = sum + (r == c ? _o.Damping * _o.Damping : 0);
                }
            }

            Cholesky(_a);
            // The task: dq = Jᵀ A⁻¹ e.
            for (int r = 0; r < 6; r++) _w[r] = _e[r];
            CholeskySolve(_a, _w);
            for (int i = 0; i < _n; i++)
            {
                double sum = 0;
                for (int r = 0; r < 6; r++) sum += _j[r * _n + i] * _w[r];
                _dq[i] = sum;
            }

            // The null space: v − J⁺ (J v).
            for (int i = 0; i < _n; i++)
            {
                double mid = 0.5 * (_lo[i] + _hi[i]), half = 0.5 * (_hi[i] - _lo[i]);
                double away = half > 1e-9 ? (mid - _q[i]) / (half * half) * 0.01 : 0;
                _v[i] = _o.RestGain * (_rest[i] - _q[i]) + _o.LimitGain * away;
            }

            for (int r = 0; r < 6; r++)
            {
                double sum = 0;
                for (int i = 0; i < _n; i++) sum += _j[r * _n + i] * _v[i];
                _w[r] = sum;
            }

            CholeskySolve(_a, _w);
            for (int i = 0; i < _n; i++)
            {
                double sum = 0;
                for (int r = 0; r < 6; r++) sum += _j[r * _n + i] * _w[r];
                _dq[i] += _v[i] - sum;
            }
        }

        /// <summary>In place, the lower triangle of a symmetric positive-definite 6×6 (the damping keeps it so).</summary>
        private static void Cholesky(double[] a)
        {
            for (int j = 0; j < 6; j++)
            {
                double d = a[j * 6 + j];
                for (int k = 0; k < j; k++) d -= a[j * 6 + k] * a[j * 6 + k];
                d = Math.Sqrt(d > 1e-300 ? d : 1e-300);
                a[j * 6 + j] = d;
                for (int i = j + 1; i < 6; i++)
                {
                    double s = a[i * 6 + j];
                    for (int k = 0; k < j; k++) s -= a[i * 6 + k] * a[j * 6 + k];
                    a[i * 6 + j] = s / d;
                }
            }
        }

        private static void CholeskySolve(double[] l, double[] b)
        {
            for (int i = 0; i < 6; i++)
            {
                double s = b[i];
                for (int k = 0; k < i; k++) s -= l[i * 6 + k] * b[k];
                b[i] = s / l[i * 6 + i];
            }

            for (int i = 5; i >= 0; i--)
            {
                double s = b[i];
                for (int k = i + 1; k < 6; k++) s -= l[k * 6 + i] * b[k];
                b[i] = s / l[i * 6 + i];
            }
        }
    }
}
