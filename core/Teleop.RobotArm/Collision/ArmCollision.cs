using System;
using System.Numerics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Collision
{
    /// <summary>What stopped a pose: nothing, a joint out of its range, or two shapes too close.</summary>
    public enum CollisionHitKind : byte
    {
        None = 0,
        JointLimit = 1,
        BodyBody = 2,
        BodyBox = 3,
        Floor = 4,
    }

    /// <summary>
    /// Why <see cref="ArmCollision.Check"/> rejected a pose. <see cref="A"/> is a body index (or,
    /// for <see cref="CollisionHitKind.JointLimit"/>, the joint: 0 yaw, 1 proximal, 2 distal, 3 wrist,
    /// 4 roll); <see cref="B"/> the other body or the box.
    /// </summary>
    public readonly struct CollisionHit
    {
        public readonly CollisionHitKind Kind;
        public readonly int A;
        public readonly int B;

        public CollisionHit(CollisionHitKind kind, int a, int b)
        {
            Kind = kind;
            A = a;
            B = b;
        }

        public bool IsHit => Kind != CollisionHitKind.None;

        /// <summary>A readable description, for logs. Allocates: never call it per frame.</summary>
        public string Describe(ArmCollisionModel model) => Kind switch
        {
            CollisionHitKind.None => "no collision",
            CollisionHitKind.JointLimit => $"joint {JointName(A)} outside its range",
            CollisionHitKind.BodyBody => $"{model.Bodies[A].Name} against {model.Bodies[B].Name}",
            CollisionHitKind.BodyBox => $"{model.Bodies[A].Name} against {model.Boxes[B].Name}",
            CollisionHitKind.Floor => $"{model.Bodies[A].Name} against the floor",
            _ => Kind.ToString(),
        };

        private static string JointName(int joint) => joint switch
        {
            0 => "base yaw",
            1 => "proximal",
            2 => "distal",
            3 => "wrist",
            4 => "roll",
            _ => joint.ToString(),
        };
    }

    /// <summary>A rigid frame: <c>world = Origin + Rotation · local</c>.</summary>
    public readonly struct LinkFrame
    {
        public readonly Vector3 Origin;
        public readonly Quaternion Rotation;

        public LinkFrame(Vector3 origin, Quaternion rotation)
        {
            Origin = origin;
            Rotation = rotation;
        }

        public Vector3 ToBase(Vector3 local) => Origin + Vector3.Transform(local, Rotation);
    }

    /// <summary>
    /// Self-collision and joint-range checks against an <see cref="ArmCollisionModel"/>
    /// (docs/adr/0020-collision-aware-ik.md). Deterministic and allocation-free: everything per call
    /// lives on the stack. Used by <c>ArmSolver</c> for every IK candidate and by the robot for
    /// every command it receives, so the two sides use one definition of "collides".
    /// </summary>
    public static class ArmCollision
    {
        /// <summary>
        /// The link frames for a pose, in <see cref="ArmLink"/> order, in the arm base frame (Core's
        /// convention: origin at the yaw pivot, X forward, Z up). They are the same chain
        /// <c>ArmKinematics.Forward</c> uses: yaw about Z, pitches about −Y (positive raises +X
        /// toward +Z), roll about the wrist's X (positive raises +Y).
        /// </summary>
        public static void ComputeFrames(in RobotArmProfile profile, in ArmPose pose, Span<LinkFrame> frames)
        {
            if (frames.Length < 5)
            {
                throw new ArgumentException("frames must hold 5 entries", nameof(frames));
            }

            Quaternion yaw = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, profile.HasRotatingBase ? pose.BaseYaw : 0f);
            frames[0] = new LinkFrame(Vector3.Zero, yaw);

            Quaternion proximal = yaw * Pitch(pose.Proximal);
            var shoulder = new Vector3(0f, 0f, profile.BaseHeight);
            frames[1] = new LinkFrame(shoulder, proximal);

            Quaternion distal = proximal * Pitch(pose.Distal);
            Vector3 elbow = shoulder + Vector3.Transform(new Vector3(profile.ProximalLinkLength, 0f, 0f), proximal);
            frames[2] = new LinkFrame(elbow, distal);

            Quaternion wrist = distal * Pitch(pose.Wrist);
            Vector3 wristPoint = elbow + Vector3.Transform(new Vector3(profile.DistalLinkLength, 0f, 0f), distal);
            frames[3] = new LinkFrame(wristPoint, wrist);

            frames[4] = new LinkFrame(wristPoint, wrist * Quaternion.CreateFromAxisAngle(Vector3.UnitX, pose.Roll));
        }

        /// <summary>
        /// Whether <paramref name="pose"/> is allowed: every joint inside the model's range, and no
        /// checked pair of shapes within <see cref="ArmCollisionModel.Margin"/> of each other.
        /// </summary>
        public static bool IsValid(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose pose)
            => !Check(profile, model, pose).IsHit;

        /// <summary>As <see cref="IsValid"/>, and says what blocked the pose (the first hit found).</summary>
        public static CollisionHit Check(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose pose)
        {
            if (!model.BaseYaw.Contains(pose.BaseYaw)) return new CollisionHit(CollisionHitKind.JointLimit, 0, -1);
            if (!model.Proximal.Contains(pose.Proximal)) return new CollisionHit(CollisionHitKind.JointLimit, 1, -1);
            if (!model.Distal.Contains(pose.Distal)) return new CollisionHit(CollisionHitKind.JointLimit, 2, -1);
            if (!model.Wrist.Contains(pose.Wrist)) return new CollisionHit(CollisionHitKind.JointLimit, 3, -1);
            if (!model.Roll.Contains(pose.Roll)) return new CollisionHit(CollisionHitKind.JointLimit, 4, -1);

            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ComputeFrames(profile, pose, frames);

            CollisionBody[] bodies = model.Bodies;
            int n = bodies.Length;
            // Each body in the base frame: centre, three unit axes, half extents grown by the margin,
            // and its axis-aligned bounds for the broad phase.
            Span<Vector3> centre = stackalloc Vector3[ArmCollisionModel.MaxBodies];
            Span<Vector3> ax = stackalloc Vector3[ArmCollisionModel.MaxBodies];
            Span<Vector3> ay = stackalloc Vector3[ArmCollisionModel.MaxBodies];
            Span<Vector3> az = stackalloc Vector3[ArmCollisionModel.MaxBodies];
            Span<Vector3> half = stackalloc Vector3[ArmCollisionModel.MaxBodies];
            Span<Vector3> lo = stackalloc Vector3[ArmCollisionModel.MaxBodies];
            Span<Vector3> hi = stackalloc Vector3[ArmCollisionModel.MaxBodies];
            float margin = model.Margin;
            for (int i = 0; i < n; i++)
            {
                LinkFrame frame = frames[(int)bodies[i].Link];
                Quaternion rotation = frame.Rotation * bodies[i].Rotation;
                centre[i] = frame.ToBase(bodies[i].Center);
                ax[i] = Vector3.Transform(Vector3.UnitX, rotation);
                ay[i] = Vector3.Transform(Vector3.UnitY, rotation);
                az[i] = Vector3.Transform(Vector3.UnitZ, rotation);
                half[i] = bodies[i].HalfExtents;
                Vector3 reach = Vector3.Abs(ax[i]) * half[i].X + Vector3.Abs(ay[i]) * half[i].Y + Vector3.Abs(az[i]) * half[i].Z;
                lo[i] = centre[i] - reach;
                hi[i] = centre[i] + reach;

                if (bodies[i].Link != ArmLink.Turntable && lo[i].Z - margin < model.FloorZ)
                {
                    return new CollisionHit(CollisionHitKind.Floor, i, -1);
                }
            }

            CollisionBox[] boxes = model.Boxes;
            var pad = new Vector3(margin);
            for (int i = 0; i < n; i++)
            {
                for (int k = 0; k < boxes.Length; k++)
                {
                    if (model.IgnoresBodyBox(i, k) || !Overlaps(lo[i] - pad, hi[i] + pad, boxes[k].Min, boxes[k].Max))
                    {
                        continue;
                    }

                    // The fixed box grown by the margin, against the body.
                    Vector3 boxCentre = 0.5f * (boxes[k].Min + boxes[k].Max);
                    Vector3 boxHalf = 0.5f * (boxes[k].Max - boxes[k].Min) + pad;
                    if (BoxesOverlap(centre[i], ax[i], ay[i], az[i], half[i], boxCentre, Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ, boxHalf))
                    {
                        return new CollisionHit(CollisionHitKind.BodyBox, i, k);
                    }
                }

                for (int j = i + 1; j < n; j++)
                {
                    if (model.IgnoresBodyPair(i, j) || !Overlaps(lo[i] - pad, hi[i] + pad, lo[j], hi[j]))
                    {
                        continue;
                    }

                    if (BoxesOverlap(centre[i], ax[i], ay[i], az[i], half[i] + pad, centre[j], ax[j], ay[j], az[j], half[j]))
                    {
                        return new CollisionHit(CollisionHitKind.BodyBody, i, j);
                    }
                }
            }

            return default;
        }

        /// <summary>
        /// Whether two oriented boxes intersect: the separating-axis test over the 15 candidate axes
        /// (Ericson, Real-Time Collision Detection, 4.4.1). Exact. Growing one box's half extents by
        /// the margin first makes it conservative: the true clearance of a pair it calls free is at
        /// least the margin.
        /// </summary>
        public static bool BoxesOverlap(
            Vector3 ca, Vector3 ax, Vector3 ay, Vector3 az, Vector3 ha,
            Vector3 cb, Vector3 bx, Vector3 by, Vector3 bz, Vector3 hb)
        {
            const float epsilon = 1e-6f;
            Vector3 t = cb - ca;
            // R[i][j] = a_i . b_j, and its absolute value with an epsilon against parallel edges.
            float r00 = Vector3.Dot(ax, bx), r01 = Vector3.Dot(ax, by), r02 = Vector3.Dot(ax, bz);
            float r10 = Vector3.Dot(ay, bx), r11 = Vector3.Dot(ay, by), r12 = Vector3.Dot(ay, bz);
            float r20 = Vector3.Dot(az, bx), r21 = Vector3.Dot(az, by), r22 = Vector3.Dot(az, bz);
            float a00 = MathF.Abs(r00) + epsilon, a01 = MathF.Abs(r01) + epsilon, a02 = MathF.Abs(r02) + epsilon;
            float a10 = MathF.Abs(r10) + epsilon, a11 = MathF.Abs(r11) + epsilon, a12 = MathF.Abs(r12) + epsilon;
            float a20 = MathF.Abs(r20) + epsilon, a21 = MathF.Abs(r21) + epsilon, a22 = MathF.Abs(r22) + epsilon;
            float t0 = Vector3.Dot(t, ax), t1 = Vector3.Dot(t, ay), t2 = Vector3.Dot(t, az);

            // A's axes.
            if (MathF.Abs(t0) > ha.X + hb.X * a00 + hb.Y * a01 + hb.Z * a02) return false;
            if (MathF.Abs(t1) > ha.Y + hb.X * a10 + hb.Y * a11 + hb.Z * a12) return false;
            if (MathF.Abs(t2) > ha.Z + hb.X * a20 + hb.Y * a21 + hb.Z * a22) return false;
            // B's axes.
            if (MathF.Abs(t0 * r00 + t1 * r10 + t2 * r20) > ha.X * a00 + ha.Y * a10 + ha.Z * a20 + hb.X) return false;
            if (MathF.Abs(t0 * r01 + t1 * r11 + t2 * r21) > ha.X * a01 + ha.Y * a11 + ha.Z * a21 + hb.Y) return false;
            if (MathF.Abs(t0 * r02 + t1 * r12 + t2 * r22) > ha.X * a02 + ha.Y * a12 + ha.Z * a22 + hb.Z) return false;
            // The nine edge cross products.
            if (MathF.Abs(t2 * r10 - t1 * r20) > ha.Y * a20 + ha.Z * a10 + hb.Y * a02 + hb.Z * a01) return false;
            if (MathF.Abs(t2 * r11 - t1 * r21) > ha.Y * a21 + ha.Z * a11 + hb.X * a02 + hb.Z * a00) return false;
            if (MathF.Abs(t2 * r12 - t1 * r22) > ha.Y * a22 + ha.Z * a12 + hb.X * a01 + hb.Y * a00) return false;
            if (MathF.Abs(t0 * r20 - t2 * r00) > ha.X * a20 + ha.Z * a00 + hb.Y * a12 + hb.Z * a11) return false;
            if (MathF.Abs(t0 * r21 - t2 * r01) > ha.X * a21 + ha.Z * a01 + hb.X * a12 + hb.Z * a10) return false;
            if (MathF.Abs(t0 * r22 - t2 * r02) > ha.X * a22 + ha.Z * a02 + hb.X * a11 + hb.Y * a10) return false;
            if (MathF.Abs(t1 * r00 - t0 * r10) > ha.X * a10 + ha.Y * a00 + hb.Y * a22 + hb.Z * a21) return false;
            if (MathF.Abs(t1 * r01 - t0 * r11) > ha.X * a11 + ha.Y * a01 + hb.X * a22 + hb.Z * a20) return false;
            if (MathF.Abs(t1 * r02 - t0 * r12) > ha.X * a12 + ha.Y * a02 + hb.X * a21 + hb.Y * a20) return false;
            return true;
        }

        public static float PointBoxDistanceSquared(Vector3 point, Vector3 boxMin, Vector3 boxMax)
        {
            Vector3 nearest = Vector3.Clamp(point, boxMin, boxMax);
            return Vector3.DistanceSquared(point, nearest);
        }

        private static bool Overlaps(Vector3 aMin, Vector3 aMax, Vector3 bMin, Vector3 bMax) =>
            aMin.X <= bMax.X && aMax.X >= bMin.X &&
            aMin.Y <= bMax.Y && aMax.Y >= bMin.Y &&
            aMin.Z <= bMax.Z && aMax.Z >= bMin.Z;

        /// <summary>Raises +X toward +Z by <paramref name="radians"/>: a turn about −Y.</summary>
        private static Quaternion Pitch(float radians) => Quaternion.CreateFromAxisAngle(Vector3.UnitY, -radians);
    }
}
