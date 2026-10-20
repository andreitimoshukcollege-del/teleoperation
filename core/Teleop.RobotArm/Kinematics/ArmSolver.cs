using System;
using System.Numerics;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Kinematics
{
    /// <summary>How an <see cref="ArmSolver.Solve"/> result relates to what was asked.</summary>
    public enum ArmSolveStatus : byte
    {
        /// <summary>The wrist is at the target with the asked pitch.</summary>
        Exact = 0,

        /// <summary>The wrist is at the target, but the pitch was tilted to keep clear of the robot.</summary>
        PitchAdjusted = 1,

        /// <summary>The target is blocked: the arm stopped at the last collision-free point toward it.</summary>
        Blocked = 2,

        /// <summary>
        /// The target is reachable but further than one solve may move a joint: this pose is a step
        /// along the shortest joint path to it, and the next solves carry on.
        /// </summary>
        Approaching = 3,
    }

    /// <summary>The joint angles a solve chose, where they put the wrist, and how that relates to the request.</summary>
    public readonly struct ArmSolution
    {
        public readonly ArmPose Pose;

        /// <summary>The wrist position the pose reaches, arm base frame (metres).</summary>
        public readonly Vector3 Position;

        /// <summary>The wrist's absolute pitch the pose reaches, radians.</summary>
        public readonly float Pitch;

        public readonly ArmSolveStatus Status;

        /// <summary>The target was outside the arm's reach and was pulled onto its edge first.</summary>
        public readonly bool ReachClamped;

        /// <summary>Which closed-form branch the pose came from (see <see cref="ArmSolver"/>). Stable between nearby targets.</summary>
        public readonly int Branch;

        public ArmSolution(in ArmPose pose, Vector3 position, float pitch, ArmSolveStatus status, bool reachClamped, int branch)
        {
            Pose = pose;
            Position = position;
            Pitch = pitch;
            Status = status;
            ReachClamped = reachClamped;
            Branch = branch;
        }

        /// <summary>Anything short of exactly what was asked: the reach warning's meaning.</summary>
        public bool IsCompromised => Status == ArmSolveStatus.Blocked || ReachClamped;
    }

    /// <summary>
    /// Inverse kinematics that never asks for a pose the robot cannot take
    /// (docs/adr/0020-collision-aware-ik.md). Every candidate is checked against the profile's
    /// <see cref="ArmCollisionModel"/>, the robot's own body and joint ranges, and the search is
    /// deterministic:
    /// <list type="number">
    /// <item>The target is pulled into reach (<see cref="ArmKinematics.ProjectToWorkspace"/>).</item>
    /// <item>The closed-form solutions are tried: four branches (elbow either way, and the base
    /// either facing the target or turned half round reaching over the top), each with the pitch
    /// tilted by 0, ±5°, … ±<see cref="MaxPitchTilt"/>. The previous branch goes first with every
    /// tilt before any other branch is tried, so a small target move never flips the arm.</item>
    /// <item>No joint moves more than <see cref="MaxJumpPerSolve"/> in one solve. A candidate
    /// beyond a few degrees away (a branch switch) must also have a clear straight joint-space path
    /// to it, since that is roughly how the servos travel.</item>
    /// <item>If nothing reaches the target, the target is bisected along the segment from the
    /// last pose reached, and the arm stops at the furthest point it can reach. Repeated solves
    /// carry it on from there, so a target further than one step away is reached over several.</item>
    /// </list>
    /// A profile without a collision model gets the plain closed-form solve, as before.
    ///
    /// <para>Stateful (it remembers the last pose for continuity) and allocation-free. One instance
    /// per arm being driven; <see cref="Reset"/> it from the robot's measured pose when known.</para>
    /// </summary>
    public struct ArmSolver
    {
        /// <summary>The largest pitch tilt tried, radians (90°).</summary>
        public const float MaxPitchTilt = MathF.PI / 2f;

        /// <summary>The tilt step, radians (5°).</summary>
        public const float PitchStep = 5f * MathF.PI / 180f;

        /// <summary>Joint jumps above this (radians, 8°) need a clear joint-space path.</summary>
        public const float PathCheckThreshold = 8f * MathF.PI / 180f;

        /// <summary>Joint-space path samples are at most this far apart (radians, 4°).</summary>
        public const float PathStep = 4f * MathF.PI / 180f;

        /// <summary>
        /// The most any joint may move in one solve (radians, 30°). A candidate further away is not
        /// taken: the arm moves toward it over several solves instead, or stops. This is what keeps a
        /// branch switch (the base half round to reach over the top) from becoming one huge swing
        /// when the target crosses the base's ±120° limit. Over the top is still reachable, by
        /// moving there continuously, where that switch is a small step.
        /// </summary>
        public const float MaxJumpPerSolve = 30f * MathF.PI / 180f;

        /// <summary>
        /// The furthest a target's pose may be from the current one (any joint, radians, 90°) for the
        /// solver to travel there on its own. Beyond it, reaching the target would mean swinging the
        /// base half round or flipping the elbow, which nobody asked for: the arm stops instead.
        /// Over the top from near vertical stays within it.
        /// </summary>
        public const float MaxApproach = 90f * MathF.PI / 180f;

        /// <summary>
        /// The most the wrist roll may turn in one solve (radians, 30°): it moves toward what is
        /// asked at up to this rate, and never stops the arm by itself. Before (2026-10-21) the roll
        /// counted against <see cref="MaxJumpPerSolve"/> and <see cref="MaxApproach"/> like the arm
        /// joints, so one frame of a jumpy roll reading (a hand pointing straight down) reported a
        /// reachable target as blocked and froze the arm. Kept equal to MaxJumpPerSolve, so a roll
        /// step on its own always fits within the other joints' caps.
        /// </summary>
        public const float MaxRollPerSolve = MaxJumpPerSolve;

        private const int BisectionSteps = 8;

        /// <summary>
        /// The most collision checks one <see cref="Solve"/> may run. Past it, a candidate counts as
        /// blocked, so the arm holds its best valid answer so far. A count rather than a time: the
        /// same inputs give the same answer on every machine, which replay depends on.
        /// </summary>
        public const int MaxChecksPerSolve = 600;

        private int _checks;
        private bool _lastValid;
        private bool _lastMadeProgress;
        private const int BranchCount = 4;

        private ArmPose _last;
        private Vector3 _lastPosition;
        private int _lastBranch;
        private bool _hasLast;

        // The last request and its answer: a target held still (a hand pushing against the robot)
        // is answered without searching again.
        private Vector3 _lastTarget;
        private float _lastDesiredPitch, _lastRoll;
        private ArmSolution _lastSolution;
        private bool _hasLastSolution;

        /// <summary>Whether a pose has been seeded or solved yet.</summary>
        public bool HasPose => _hasLast;

        /// <summary>The last pose this solver returned (or was reset to).</summary>
        public ArmPose LastPose => _last;

        /// <summary>
        /// Starts from <paramref name="pose"/> (the robot's measured pose, or the model's
        /// <see cref="ArmCollisionModel.Home"/>). A blocked target is approached from here.
        /// </summary>
        public void Reset(in RobotArmProfile profile, in ArmPose pose)
        {
            _last = pose;
            _lastPosition = ArmKinematics.Forward(profile, pose.BaseYaw, pose.Proximal, pose.Distal);
            _lastBranch = BranchOf(pose);
            _hasLast = true;
            _hasLastSolution = false;
        }

        /// <summary>
        /// Solves for <paramref name="target"/> (the wrist, arm base frame) with the wrist's absolute
        /// <paramref name="desiredPitch"/> and <paramref name="roll"/>. Always returns a pose the
        /// model allows when the profile has one; see the type's doc for the search.
        /// </summary>
        public ArmSolution Solve(in RobotArmProfile profile, Vector3 target, float desiredPitch, float roll)
        {
            if (profile.WristJointCount != 1 && profile.Collision != null)
            {
                throw new InvalidOperationException("ArmSolver's collision search supports exactly one wrist pitch joint.");
            }

            Vector3 projected = ArmKinematics.ProjectToWorkspace(profile, target, float.NegativeInfinity);
            bool reachClamped = Vector3.DistanceSquared(projected, target) > 1e-10f;
            ArmCollisionModel? model = profile.Collision;

            if (model == null)
            {
                ArmSolution plain = Plain(profile, projected, desiredPitch, roll, reachClamped);
                Remember(plain);
                return plain;
            }

            if (!_hasLast)
            {
                Reset(profile, model.Home);
            }

            // A held target is answered without searching again, unless the arm is still on its way
            // there: a blocked answer that moved the arm moves it further on the next call.
            if (_hasLastSolution && target == _lastTarget && desiredPitch == _lastDesiredPitch && roll == _lastRoll &&
                !_lastMadeProgress)
            {
                return _lastSolution;
            }

            _lastTarget = target;
            _lastDesiredPitch = desiredPitch;
            _lastRoll = roll;
            _checks = 0;
            // A path can only be required to stay clear when it starts clear. The robot may well
            // start in a pose the model calls colliding (folded at rest); then the next pose is
            // judged on its own.
            _lastValid = ArmCollision.IsValid(profile, model, _last);
            // The roll turns toward the request at a bounded rate rather than in one jump; every
            // candidate below uses this solve's step of it.
            float requestedRoll = model.Roll.Clamp(roll);
            float clampedRoll = model.Roll.Clamp(_last.Roll + Math.Clamp(requestedRoll - _last.Roll, -MaxRollPerSolve, MaxRollPerSolve));
            bool rollCatchingUp = clampedRoll != requestedRoll;

            // 1. The valid pose at the target that is closest to where the arm is: the shortest
            //    joint path there.
            if (ClosestAt(profile, model, projected, desiredPitch, clampedRoll, out ArmPose goal, out int branch, out float tilt) &&
                (!_lastValid || _last.MaxJointDelta(goal) <= MaxApproach))
            {
                float jump = _last.MaxJointDelta(goal);
                if (!_lastValid || jump <= MaxJumpPerSolve)
                {
                    if (PathClear(profile, model, _last, goal))
                    {
                        var exact = new ArmSolution(goal, projected, desiredPitch + tilt,
                            tilt == 0f ? ArmSolveStatus.Exact : ArmSolveStatus.PitchAdjusted, reachClamped, branch);
                        Remember(exact);
                        // Still turning toward the asked roll: solve again next time, not from the cache.
                        _lastMadeProgress = rollCatchingUp;
                        return exact;
                    }
                }
                else
                {
                    // 2. Too far for one solve: a step of at most the cap along the straight joint
                    //    path to it. The next solves carry on, so the arm gets there over a few
                    //    frames without ever swinging round, and without the dead ends a straight
                    //    line in space can hit (one passing over the shoulder, inside the reach the
                    //    elbow's range allows).
                    ArmPose step = ArmPose.Lerp(_last, goal, MaxJumpPerSolve / jump);
                    if (Valid(profile, model, step) && PathClear(profile, model, _last, step))
                    {
                        Vector3 at = ArmKinematics.Forward(profile, step.BaseYaw, step.Proximal, step.Distal);
                        var approaching = new ArmSolution(step, at, PitchOf(step), ArmSolveStatus.Approaching, reachClamped, branch);
                        Remember(approaching);
                        _lastMadeProgress = true;
                        return approaching;
                    }
                }
            }

            // 3. Nothing at the target can be reached from here: stop as close as possible.

            // Blocked: the furthest point toward the target, from the last pose reached.
            Vector3 from = _lastPosition;
            ArmPose bestPose = _last;
            int bestBranch = _lastBranch;
            float bestPitch = PitchOf(_last);
            Vector3 bestPosition = from;
            if (!_lastValid)
            {
                // The remembered pose is no longer allowed (the roll changed, or it was a bad
                // seed): fall back to Home rather than hold an invalid pose.
                bestPose = model.Home;
                bestBranch = BranchOf(bestPose);
                bestPitch = PitchOf(bestPose);
                bestPosition = ArmKinematics.Forward(profile, bestPose.BaseYaw, bestPose.Proximal, bestPose.Distal);
                from = bestPosition;
            }

            float lo = 0f, hi = 1f;
            for (int step = 0; step < BisectionSteps; step++)
            {
                float mid = 0.5f * (lo + hi);
                Vector3 point = Vector3.Lerp(from, projected, mid);
                // Points on the way may switch branch too: the cap keeps any switch a small step, so
                // the arm can go on over the top (straight overhead, the base-facing and the
                // leaning-back branches meet). The current branch is still tried first.
                if (TryReach(profile, model, point, desiredPitch, clampedRoll, BranchCount, out ArmPose midPose, out int midBranch, out float midTilt))
                {
                    lo = mid;
                    bestPose = midPose;
                    bestBranch = midBranch;
                    bestPitch = desiredPitch + midTilt;
                    bestPosition = point;
                }
                else
                {
                    hi = mid;
                }
            }

            var blocked = new ArmSolution(bestPose, bestPosition, bestPitch, ArmSolveStatus.Blocked, reachClamped, bestBranch);
            Remember(blocked);
            _lastMadeProgress = lo > 0f || rollCatchingUp;
            return blocked;
        }

        /// <summary>
        /// The valid pose at <paramref name="point"/> closest to the last one: the least joint
        /// motion, counting each radian of pitch tilt as one radian of motion, so an exact pitch
        /// wins unless keeping it would cost a bigger swing. The current branch untilted is taken at
        /// once when it is within a few degrees, the everyday case of a hand moving smoothly.
        /// </summary>
        private bool ClosestAt(in RobotArmProfile profile, ArmCollisionModel model, Vector3 point, float desiredPitch, float roll,
            out ArmPose best, out int bestBranch, out float bestTilt)
        {
            best = default;
            bestBranch = 0;
            bestTilt = 0f;
            if (!WithinElbowReach(profile, model, point))
            {
                return false;
            }

            if (Closed(profile, point, desiredPitch, roll, _lastBranch, out ArmPose quick) &&
                _last.MaxJointDelta(quick) <= PathCheckThreshold && Valid(profile, model, quick))
            {
                best = quick;
                bestBranch = _lastBranch;
                return true;
            }

            float bestCost = float.MaxValue;
            for (int candidate = 0; candidate < BranchCount; candidate++)
            {
                for (int t = 0; ; t++)
                {
                    float offset = TiltAt(t);
                    if (MathF.Abs(offset) > MaxPitchTilt + 1e-6f)
                    {
                        break;
                    }

                    if (!Closed(profile, point, desiredPitch + offset, roll, candidate, out ArmPose pose))
                    {
                        continue;
                    }

                    float cost = _last.MaxJointDelta(pose) + MathF.Abs(offset);
                    if (cost >= bestCost || !Valid(profile, model, pose))
                    {
                        continue;
                    }

                    bestCost = cost;
                    best = pose;
                    bestBranch = candidate;
                    bestTilt = offset;
                }
            }

            return bestCost < float.MaxValue;
        }

        private bool TryReach(in RobotArmProfile profile, ArmCollisionModel model, Vector3 point, float desiredPitch, float roll,
            int branches, out ArmPose pose, out int branch, out float tilt)
        {
            pose = default;
            branch = 0;
            tilt = 0f;
            if (!WithinElbowReach(profile, model, point))
            {
                return false;
            }

            // The previous branch with every tilt first, then the others in a fixed order.
            for (int order = 0; order < branches; order++)
            {
                int candidate = order == 0 ? _lastBranch : (order <= _lastBranch ? order - 1 : order);
                for (int t = 0; ; t++)
                {
                    float offset = TiltAt(t);
                    if (MathF.Abs(offset) > MaxPitchTilt + 1e-6f)
                    {
                        break;
                    }

                    if (Closed(profile, point, desiredPitch + offset, roll, candidate, out pose) &&
                        (!_lastValid || _last.MaxJointDelta(pose) <= MaxJumpPerSolve) &&
                        Valid(profile, model, pose) &&
                        PathClear(profile, model, _last, pose))
                    {
                        branch = candidate;
                        tilt = offset;
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Whether any branch can put the wrist at <paramref name="point"/> with the elbow inside its
        /// range: the elbow's limit sets a minimum distance from the shoulder (13 cm on the JetRover,
        /// two 13 cm links and ±120°). Cheap, and it spares the full search for points nearer.
        /// </summary>
        private static bool WithinElbowReach(in RobotArmProfile profile, ArmCollisionModel model, Vector3 point)
        {
            float maxBend = MathF.Max(MathF.Abs(model.Distal.Min), MathF.Abs(model.Distal.Max));
            if (maxBend >= MathF.PI)
            {
                return true;
            }

            float l1 = profile.ProximalLinkLength, l2 = profile.DistalLinkLength;
            float minReachSquared = l1 * l1 + l2 * l2 + 2f * l1 * l2 * MathF.Cos(maxBend);
            Vector3 fromShoulder = point - new Vector3(0f, 0f, profile.BaseHeight);
            // A hair of slack, so a point exactly on the limit is still tried.
            return fromShoulder.LengthSquared() >= minReachSquared - 1e-6f;
        }

        /// <summary>0, +5°, −5°, +10°, −10°, …</summary>
        private static float TiltAt(int index)
        {
            if (index == 0)
            {
                return 0f;
            }

            int magnitude = (index + 1) / 2;
            return (index % 2 == 1 ? 1f : -1f) * magnitude * PitchStep;
        }

        /// <summary>A collision check that counts against <see cref="MaxChecksPerSolve"/>; past it, everything is blocked.</summary>
        private bool Valid(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose pose)
        {
            if (_checks >= MaxChecksPerSolve)
            {
                return false;
            }

            _checks++;
            return ArmCollision.IsValid(profile, model, pose);
        }

        private bool PathClear(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose from, in ArmPose to)
        {
            if (!_lastValid)
            {
                return true;
            }

            float jump = from.MaxJointDelta(to);
            if (jump <= PathCheckThreshold)
            {
                return true;
            }

            int steps = (int)MathF.Ceiling(jump / PathStep);
            for (int s = 1; s < steps; s++)
            {
                if (!Valid(profile, model, ArmPose.Lerp(from, to, s / (float)steps)))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// One closed-form branch. Bit 0 picks the elbow (0: as <see cref="ArmKinematics.TryInverse"/>,
        /// 1: mirrored); bit 1 turns the base half round and reaches back over the top. False when
        /// the branch has no solution within the yaw range.
        /// </summary>
        private bool Closed(in RobotArmProfile profile, Vector3 point, float pitch, float roll, int branch, out ArmPose pose)
        {
            pose = default;
            bool flip = (branch & 2) != 0;
            bool mirror = (branch & 1) != 0;
            if (flip && !profile.HasRotatingBase)
            {
                return false;
            }

            float horizontal = MathF.Sqrt(point.X * point.X + point.Y * point.Y);
            float yaw;
            if (!profile.HasRotatingBase)
            {
                yaw = 0f;
                horizontal = point.X;
            }
            else if (horizontal > 0.01f)
            {
                yaw = MathF.Atan2(point.Y, point.X);
            }
            else
            {
                // Straight overhead the yaw is free: keep the last one.
                yaw = _last.BaseYaw;
            }

            float r = horizontal;
            if (flip)
            {
                yaw = WrapAngle(yaw + MathF.PI);
                r = -horizontal;
            }

            float dz = point.Z - profile.BaseHeight;
            float l1 = profile.ProximalLinkLength, l2 = profile.DistalLinkLength;
            float reach = Math.Clamp(MathF.Sqrt(r * r + dz * dz), MathF.Abs(l1 - l2) + 1e-4f, l1 + l2 - 1e-4f);
            float gamma = MathF.Acos(Math.Clamp((l1 * l1 + l2 * l2 - reach * reach) / (2f * l1 * l2), -1f, 1f));
            float alpha = MathF.Acos(Math.Clamp((l1 * l1 + reach * reach - l2 * l2) / (2f * l1 * reach), -1f, 1f));
            float phi = MathF.Atan2(dz, r);

            float proximal = mirror ? phi + alpha : phi - alpha;
            float distal = mirror ? -(MathF.PI - gamma) : MathF.PI - gamma;
            float wrist = WrapAngle(pitch - (proximal + distal));
            pose = new ArmPose(yaw, WrapAngle(proximal), distal, wrist, roll);
            return true;
        }

        private static ArmSolution Plain(in RobotArmProfile profile, Vector3 target, float pitch, float roll, bool reachClamped)
        {
            Span<float> wrists = stackalloc float[Math.Max(1, profile.WristJointCount)];
            ArmKinematics.TryInverse(profile, target, pitch, out float yaw, out float proximal, out float distal, wrists, out _);
            var pose = new ArmPose(yaw, proximal, distal, profile.WristJointCount > 0 ? wrists[0] : 0f, roll);
            return new ArmSolution(pose, target, pitch, ArmSolveStatus.Exact, reachClamped, 0);
        }

        private void Remember(in ArmSolution solution)
        {
            _lastSolution = solution;
            _hasLastSolution = true;
            _last = solution.Pose;
            _lastPosition = solution.Position;
            _lastBranch = solution.Branch;
            _hasLast = true;
        }

        private static float PitchOf(in ArmPose pose) => pose.Proximal + pose.Distal + pose.Wrist;

        /// <summary>The branch a pose belongs to: elbow mirrored when the distal bend is negative, flipped when the proximal link leans back past vertical.</summary>
        private static int BranchOf(in ArmPose pose) =>
            (pose.Distal < 0f ? 1 : 0) | (MathF.Abs(pose.Proximal) > MathF.PI / 2f ? 2 : 0);

        private static float WrapAngle(float radians)
        {
            const float twoPi = 2f * MathF.PI;
            radians %= twoPi;
            if (radians > MathF.PI) radians -= twoPi;
            else if (radians < -MathF.PI) radians += twoPi;
            return radians;
        }
    }
}
