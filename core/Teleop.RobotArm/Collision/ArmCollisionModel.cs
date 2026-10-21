using System;
using System.Numerics;

namespace Teleop.RobotArm.Collision
{
    /// <summary>
    /// The rigid frames a moving collision shape can ride on, base to tip
    /// (docs/adr/0020-collision-aware-ik.md). Every frame is Core's convention, X along the link,
    /// Z up at rest, and they are built by <see cref="ArmCollision.ComputeFrames"/>:
    /// <list type="bullet">
    /// <item><see cref="Turntable"/>: yaw about Z at the arm base origin.</item>
    /// <item><see cref="Proximal"/>: the shoulder, <c>BaseHeight</c> above the origin, pitched by the proximal joint.</item>
    /// <item><see cref="Distal"/>: the elbow, <c>ProximalLinkLength</c> along the proximal link.</item>
    /// <item><see cref="Wrist"/>: the wrist pitch joint, <c>DistalLinkLength</c> along the distal link.</item>
    /// <item><see cref="Gripper"/>: the wrist frame turned by the roll about its X axis.</item>
    /// </list>
    /// </summary>
    public enum ArmLink : byte
    {
        Turntable = 0,
        Proximal = 1,
        Distal = 2,
        Wrist = 3,
        Gripper = 4,
    }

    /// <summary>
    /// A box riding on one link: centred at <see cref="Center"/> in that link's frame (metres),
    /// turned by <see cref="Rotation"/> relative to it, <see cref="HalfExtents"/> along its own axes.
    /// Oriented boxes, not capsules, because the arm's links are flat brackets: a round cross-section
    /// sized to a link's width overhangs its thickness by centimetres.
    /// </summary>
    public readonly struct CollisionBody
    {
        public readonly string Name;
        public readonly ArmLink Link;
        public readonly Vector3 Center;
        public readonly Quaternion Rotation;
        public readonly Vector3 HalfExtents;

        public CollisionBody(string name, ArmLink link, Vector3 center, Quaternion rotation, Vector3 halfExtents)
        {
            Name = name;
            Link = link;
            Center = center;
            Rotation = rotation;
            HalfExtents = halfExtents;
        }
    }

    /// <summary>A fixed part of the robot's body: an axis-aligned box in the arm base frame (metres).</summary>
    public readonly struct CollisionBox
    {
        public readonly string Name;
        public readonly Vector3 Min;
        public readonly Vector3 Max;

        public CollisionBox(string name, Vector3 min, Vector3 max)
        {
            Name = name;
            Min = min;
            Max = max;
        }
    }

    /// <summary>A joint's allowed kinematic angles, radians, inclusive. Kinematic means the IK's own angle, without the servo's mounting offset.</summary>
    public readonly struct AngleRange
    {
        public readonly float Min;
        public readonly float Max;

        public AngleRange(float min, float max)
        {
            Min = min;
            Max = max;
        }

        public bool Contains(float radians) => radians >= Min && radians <= Max;

        public float Clamp(float radians) => radians < Min ? Min : radians > Max ? Max : radians;
    }

    /// <summary>
    /// What a robot arm can collide with, and how far each joint may turn
    /// (docs/adr/0020-collision-aware-ik.md). Generated from the robot's CAD
    /// (<c>just robot-collision-model</c>), never hand-tuned. Read by <see cref="ArmCollision"/>
    /// and <c>Kinematics/ArmSolver</c>, on the headset and on the robot alike.
    ///
    /// <para>Which pairs are tested:</para>
    /// <list type="bullet">
    /// <item><b>Adjacent links are never box-tested</b> (and the gripper counts as adjacent to the
    /// distal link and the wrist, since it hangs off the wrist joint). Links meeting at a joint nest
    /// millimetres apart by design, finer than boxes and a margin can resolve. Their contact is
    /// instead each joint's range (<see cref="Proximal"/>, <see cref="Distal"/>, <see cref="Wrist"/>),
    /// which the generator sweeps against the CAD's own surfaces.</item>
    /// <item>The turntable is never tested against the body it turns on.</item>
    /// <item>The explicit <c>ignoredBodyPairs</c> and <c>ignoredBodyBoxes</c> are skipped too: pairs
    /// the generator found in contact in virtually every pose, and the proximal link's piece at the
    /// shoulder against the body.</item>
    /// <item>Everything else is checked: a link against any non-adjacent link, the body and the
    /// floor.</item>
    /// </list>
    /// </summary>
    public sealed class ArmCollisionModel
    {
        private readonly bool[] _ignoreBodyPair;
        private readonly bool[] _ignoreBodyBox;

        public ArmCollisionModel(
            string name,
            CollisionBody[] bodies,
            CollisionBox[] boxes,
            float floorZ,
            float margin,
            AngleRange baseYaw,
            AngleRange proximal,
            AngleRange distal,
            AngleRange wrist,
            AngleRange roll,
            ArmPose home,
            (int, int)[] ignoredBodyPairs,
            (int, int)[] ignoredBodyBoxes,
            string source)
        {
            Name = name ?? throw new ArgumentNullException(nameof(name));
            Bodies = bodies ?? throw new ArgumentNullException(nameof(bodies));
            Boxes = boxes ?? throw new ArgumentNullException(nameof(boxes));
            if (Bodies.Length > MaxBodies)
            {
                throw new ArgumentException($"At most {MaxBodies} bodies, got {Bodies.Length}.", nameof(bodies));
            }

            if (!(margin >= 0f))
            {
                throw new ArgumentOutOfRangeException(nameof(margin), margin, "margin must be >= 0");
            }

            FloorZ = floorZ;
            Margin = margin;
            BaseYaw = baseYaw;
            Proximal = proximal;
            Distal = distal;
            Wrist = wrist;
            Roll = roll;
            Home = home;
            Source = source ?? "";

            int n = Bodies.Length, m = Boxes.Length;
            _ignoreBodyPair = new bool[n * n];
            _ignoreBodyBox = new bool[n * m];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    _ignoreBodyPair[i * n + j] = Math.Abs(ChainIndex(Bodies[i].Link) - ChainIndex(Bodies[j].Link)) <= 1;
                }

                for (int k = 0; k < m; k++)
                {
                    _ignoreBodyBox[i * m + k] = Bodies[i].Link == ArmLink.Turntable;
                }
            }

            foreach ((int a, int b) in ignoredBodyPairs ?? Array.Empty<(int, int)>())
            {
                CheckIndex(a, n, nameof(ignoredBodyPairs));
                CheckIndex(b, n, nameof(ignoredBodyPairs));
                _ignoreBodyPair[a * n + b] = true;
                _ignoreBodyPair[b * n + a] = true;
            }

            foreach ((int c, int box) in ignoredBodyBoxes ?? Array.Empty<(int, int)>())
            {
                CheckIndex(c, n, nameof(ignoredBodyBoxes));
                CheckIndex(box, m, nameof(ignoredBodyBoxes));
                _ignoreBodyBox[c * m + box] = true;
            }
        }

        /// <summary>The most bodies a model may have: <see cref="ArmCollision"/> works in a fixed stack buffer of this size.</summary>
        public const int MaxBodies = 32;

        public string Name { get; }
        public CollisionBody[] Bodies { get; }
        public CollisionBox[] Boxes { get; }

        /// <summary>Nothing may come within <see cref="Margin"/> of this height (arm base frame, metres): the surface the robot stands on.</summary>
        public float FloorZ { get; }

        /// <summary>Clearance every test keeps, metres.</summary>
        public float Margin { get; }

        public AngleRange BaseYaw { get; }
        public AngleRange Proximal { get; }
        public AngleRange Distal { get; }
        public AngleRange Wrist { get; }
        public AngleRange Roll { get; }

        /// <summary>A pose the model knows to be valid: where a solver starts when it has nothing better.</summary>
        public ArmPose Home { get; }

        /// <summary>Where the model came from (the generator's stamp), for logs and reviews.</summary>
        public string Source { get; }

        public bool IgnoresBodyPair(int a, int b) => _ignoreBodyPair[a * Bodies.Length + b];

        public bool IgnoresBodyBox(int body, int box) => _ignoreBodyBox[body * Boxes.Length + box];

        /// <summary>
        /// Position along the chain of joints, for adjacency: the gripper hangs off the same wrist
        /// joint as the wrist pitch, so both are one step from the distal link.
        /// </summary>
        private static int ChainIndex(ArmLink link) => link == ArmLink.Gripper ? (int)ArmLink.Wrist : (int)link;

        private static void CheckIndex(int index, int count, string what)
        {
            if (index < 0 || index >= count)
            {
                throw new ArgumentOutOfRangeException(what, index, $"index must be in [0, {count})");
            }
        }
    }

    /// <summary>The arm's joint angles that matter for collision, radians, kinematic (no servo offsets).</summary>
    public readonly struct ArmPose
    {
        public readonly float BaseYaw;
        public readonly float Proximal;
        public readonly float Distal;
        public readonly float Wrist;
        public readonly float Roll;

        public ArmPose(float baseYaw, float proximal, float distal, float wrist, float roll)
        {
            BaseYaw = baseYaw;
            Proximal = proximal;
            Distal = distal;
            Wrist = wrist;
            Roll = roll;
        }

        /// <summary>The largest single-joint difference to <paramref name="other"/>, radians.</summary>
        public float MaxJointDelta(in ArmPose other) =>
            MathF.Max(MathF.Max(MathF.Abs(BaseYaw - other.BaseYaw), MathF.Abs(Proximal - other.Proximal)),
                MathF.Max(MathF.Max(MathF.Abs(Distal - other.Distal), MathF.Abs(Wrist - other.Wrist)), MathF.Abs(Roll - other.Roll)));

        public static ArmPose Lerp(in ArmPose a, in ArmPose b, float t) => new ArmPose(
            a.BaseYaw + (b.BaseYaw - a.BaseYaw) * t,
            a.Proximal + (b.Proximal - a.Proximal) * t,
            a.Distal + (b.Distal - a.Distal) * t,
            a.Wrist + (b.Wrist - a.Wrist) * t,
            a.Roll + (b.Roll - a.Roll) * t);
    }
}
