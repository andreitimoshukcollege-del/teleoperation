using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// One motor's shaped state: where the shaper has it, and for a finger, the move it is in.
    /// A plain mutable struct: the shaper steps it in place, a thousand times a second, without allocating.
    /// </summary>
    public struct ShapedJointState
    {
        /// <summary>Position (radians, or a finger's 0..1 fraction), velocity and acceleration.</summary>
        public double Q;
        public double V;
        public double A;

        /// <summary>A finger only: whether a move is under way, its target, and the target of the last move started.</summary>
        public bool Moving;
        public double To;
        public double LastTo;

        /// <summary>At rest at <paramref name="q"/>, with no move under way.</summary>
        public static ShapedJointState AtRest(double q) => new ShapedJointState { Q = q, V = 0, A = 0, Moving = false, To = q, LastTo = q };
    }

    /// <summary>
    /// The trajectory shaper (docs/adr/0027 decision 1). It turns a stream of absolute targets into a trajectory
    /// a Franka arm accepts: one step every <see cref="TrajectoryShaperConfig.StepSeconds"/> (1 ms), never past
    /// the configured shares of the joint's velocity, acceleration and jerk limits, inside its position-based
    /// velocity limits and its soft limits. It is the single copy that the robot's backend drives the arm with
    /// and that <c>franka-replay</c> predicts it with. The trainer's copy and a future real-time arm server's
    /// are pinned to it by golden files (<c>gen-trainer-golden</c>).
    ///
    /// <para><b>An arm joint</b>, per step, in double precision, in this order. It is in the style of
    /// libfranka's <c>limitRate</c>, with a target to steer toward:</para>
    /// <list type="number">
    /// <item>The target is clamped to [soft lower + margin, soft upper − margin].</item>
    /// <item>Velocity bounds: v_max · velocityScale each way, and toward each hard limit at most
    /// velocityScale · (offset + sqrt(2 · deceleration · max(0, distance to it))) when the joint has a
    /// position-based limit.</item>
    /// <item>A reference velocity toward the target, e = target − q:
    /// sign(e) · min(sqrt(2 · a_b · |e|), |e| / settleSeconds), with a_b = a_max · accelerationScale ·
    /// brakingScale, or velocityScale² · the position-based deceleration if that is less. The goal is never
    /// past a limit, so with that a_b the reference is under the position-based bound for any goal. Far away it is the speed from which a stop at a_b ends at the target. Near the target it
    /// is a first-order approach, which settles smoothly at a 1 ms step. The two meet where they are equal,
    /// so the reference is continuous. It is then clamped to the velocity bounds.</item>
    /// <item>The acceleration that would reach it in one step, clamped to ±a_max · accelerationScale, then to
    /// within jerk_max · jerkScale · step of the previous acceleration, then (libfranka's <c>limitRate</c>) to
    /// (jerk / acceleration) · the velocity headroom to each bound, so a velocity bound is met with the
    /// acceleration already ramped down rather than cut off.</item>
    /// <item>v += a · step, clamped to the velocity bounds. The bounds win: a is recomputed from the clamped
    /// velocity, so only a joint pressing on its position-based limit can exceed the acceleration share.
    /// Then q += v · step.</item>
    /// </list>
    /// <para><b>A finger</b> (a Franka Hand, whose moves block until they finish) is a newest-wins gate. When
    /// no move is under way and the target, clamped to [0, 1], is more than the deadband from the last move's
    /// target, a move starts toward it. A move runs at the finger speed and ends exactly on its target. Targets
    /// that arrive during a move are not queued; whatever is in force when it ends is what the gate sees next.</para>
    ///
    /// <para><b>Gap policy</b> (decision 2) is the caller's half: the shaper always steers to the target it is
    /// given. With no new command, the caller keeps passing the last one, and the arm finishes its move under
    /// the limits and stays there.</para>
    /// </summary>
    public static class JointTrajectoryShaper
    {
        /// <summary>Advances <paramref name="state"/> one step toward <paramref name="target"/>.</summary>
        public static void Step(ref ShapedJointState state, double target, in ShapedJointLimits limits, TrajectoryShaperConfig config)
        {
            if (limits.IsFinger)
            {
                StepFinger(ref state, target, limits, config.StepSeconds);
            }
            else
            {
                StepJoint(ref state, target, limits, config);
            }
        }

        private static void StepJoint(ref ShapedJointState s, double target, in ShapedJointLimits l, TrajectoryShaperConfig c)
        {
            double dt = c.StepSeconds;

            double lo = l.SoftLower + c.PositionMargin, hi = l.SoftUpper - c.PositionMargin;
            if (lo > hi) lo = hi = 0.5 * (l.SoftLower + l.SoftUpper);
            double goal = target < lo ? lo : target > hi ? hi : target;

            double vMax = l.Velocity * c.VelocityScale;
            double aMax = l.Acceleration * c.AccelerationScale;
            double jStep = l.Jerk * c.JerkScale * dt;

            double vUp = vMax, vDown = vMax;
            if (l.PositionBasedDeceleration > 0)
            {
                double up = c.VelocityScale * (l.PositionBasedVelocityOffset + Math.Sqrt(2.0 * l.PositionBasedDeceleration * Math.Max(0.0, l.Upper - s.Q)));
                double down = c.VelocityScale * (l.PositionBasedVelocityOffset + Math.Sqrt(2.0 * l.PositionBasedDeceleration * Math.Max(0.0, s.Q - l.Lower)));
                if (up < vUp) vUp = up;
                if (down < vDown) vDown = down;
            }

            // A stop is planned no harder than velocityScale^2 times the position-based deceleration. The goal is
            // never past a limit, so the reference then stays under the position-based bound for any goal, and
            // the arm follows a curve it can follow instead of running into a bound that shrinks as it moves.
            double brakingAcceleration = aMax * c.BrakingScale;
            if (l.PositionBasedDeceleration > 0)
            {
                double positionBased = c.VelocityScale * c.VelocityScale * l.PositionBasedDeceleration;
                if (positionBased < brakingAcceleration) brakingAcceleration = positionBased;
            }

            double e = goal - s.Q;
            double distance = Math.Abs(e);
            double braking = Math.Sqrt(2.0 * brakingAcceleration * distance);
            double settling = distance / c.SettleSeconds;
            double vRef = (e < 0 ? -1.0 : e > 0 ? 1.0 : 0.0) * (braking < settling ? braking : settling);
            if (vRef > vUp) vRef = vUp;
            if (vRef < -vDown) vRef = -vDown;

            double a = (vRef - s.V) / dt;
            if (a > aMax) a = aMax;
            if (a < -aMax) a = -aMax;
            if (a > s.A + jStep) a = s.A + jStep;
            if (a < s.A - jStep) a = s.A - jStep;

            // libfranka's limitRate: the acceleration toward a velocity bound shrinks in proportion to the
            // headroom left, so the velocity meets its bound with the acceleration ramped down at the jerk
            // limit, never cut off in one step.
            double headroomGain = l.Jerk * c.JerkScale / aMax;
            double safeMax = headroomGain * (vUp - s.V), safeMin = headroomGain * (-vDown - s.V);
            if (safeMax > aMax) safeMax = aMax;
            if (safeMin < -aMax) safeMin = -aMax;
            if (a > safeMax) a = safeMax;
            if (a < safeMin) a = safeMin;

            double v = s.V + a * dt;
            if (v > vUp) v = vUp;
            if (v < -vDown) v = -vDown;

            s.A = (v - s.V) / dt;
            s.V = v;
            s.Q += v * dt;
        }

        private static void StepFinger(ref ShapedJointState s, double target, in ShapedJointLimits l, double dt)
        {
            double goal = target < 0 ? 0 : target > 1 ? 1 : target;
            if (!s.Moving && Math.Abs(goal - s.LastTo) > l.FingerDeadband)
            {
                s.Moving = true;
                s.To = goal;
                s.LastTo = goal;
            }

            if (!s.Moving)
            {
                s.V = 0;
                s.A = 0;
                return;
            }

            double step = l.FingerSpeed * dt;
            double remaining = s.To - s.Q;
            if (Math.Abs(remaining) <= step)
            {
                s.Q = s.To;
                s.V = 0;
                s.Moving = false;
            }
            else
            {
                s.V = remaining > 0 ? l.FingerSpeed : -l.FingerSpeed;
                s.Q += remaining > 0 ? step : -step;
            }

            s.A = 0;
        }
    }
}
