using System.Numerics;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// Shared extrapolation math: the quaternion exponential/logarithm maps, world-frame rotation
    /// integration, and magnitude clamping. Every predictor that dead-reckons or smooths needs
    /// exactly these four operations, and they live here for the same reason
    /// <see cref="PoseMath"/> does -- written once so that two implementations cannot silently
    /// disagree about what "the rotation between these two orientations" means. Two predictors
    /// that each rolled their own log map would usually agree and would differ in the corners
    /// (double cover, near-identity, non-unit input), which is precisely where a prediction-error
    /// plot becomes uninterpretable.
    ///
    /// <b>Rotation-vector convention</b>, matching <see cref="CommandFrame.AngularVelocity"/>
    /// exactly: a <see cref="Vector3"/> whose direction is the rotation axis and whose magnitude
    /// is the rotation angle in radians. Divided by a time in seconds it is an angular rate in
    /// radians/second; that is the only form of angular velocity anything in Core uses.
    ///
    /// <b>World frame, not body frame.</b> <see cref="IntegrateWorld"/> pre-multiplies the delta
    /// onto the current orientation and <see cref="RelativeRotationVector"/> is its exact inverse,
    /// so the axis is fixed in the world rather than carried around by the body. This matches
    /// <c>Plant/RigidBodyPlant.Step</c>'s integration; a predictor that used the body frame here
    /// would disagree with the plant it is predicting, and the disagreement would look exactly
    /// like prediction error.
    ///
    /// Units are metres and radians (ROS convention, matching <see cref="Pose"/>). Every method is
    /// static, pure, and allocation-free.
    /// </summary>
    public static class MotionMath
    {
        /// <summary>
        /// Below this the rotation is treated as exactly none: a rotation vector shorter than this
        /// has an undefined axis, and a quaternion whose squared length is below this cannot be
        /// normalized without producing NaN. Not a research knob and not a dead band on operator
        /// motion -- it exists only because the axis of a zero rotation does not exist. Chosen to
        /// match <c>Plant/RigidBodyPlant</c>'s <c>AngularRateEpsilon</c>, which guards the same
        /// degeneracy on the same quantities: it is far below any rotation a real operator, codec,
        /// or trace produces.
        /// </summary>
        public const float RotationEpsilon = 1e-12f;

        /// <summary>
        /// <paramref name="value"/> shortened to at most <paramref name="maxMagnitude"/>, keeping
        /// its direction; returned unchanged when already within the bound. Used to apply
        /// <see cref="PredictorConfig.MaxLinearSpeed"/> and
        /// <see cref="PredictorConfig.MaxAngularSpeed"/> to an estimated rate <i>before</i> it is
        /// extrapolated over a horizon, so a two-sample rate spike cannot throw a prediction an
        /// implausible distance. Callers must pass a non-negative bound; a negative one would
        /// scale the vector through zero and flip it.
        /// </summary>
        public static Vector3 ClampMagnitude(in Vector3 value, float maxMagnitude)
        {
            float length = value.Length();
            if (length <= maxMagnitude)
            {
                return value;
            }

            return value * (maxMagnitude / length);
        }

        /// <summary>
        /// Logarithm map: the rotation vector (axis * angle, radians) equivalent to
        /// <paramref name="rotation"/>.
        ///
        /// Takes the <b>shortest arc</b>: <c>q</c> and <c>-q</c> are the same rotation, and
        /// without the sign fix the same orientation would map to either a small angle or its
        /// <c>2*pi</c> complement depending on which representative arrived. Uses
        /// <c>atan2(|v|, w)</c> rather than <c>acos(w)</c> because atan2 stays well conditioned as
        /// the angle approaches zero, which is the regime almost every inter-sample rotation is
        /// in. Returns <see cref="Vector3.Zero"/> for a rotation at or below
        /// <see cref="RotationEpsilon"/> and for an unnormalizable (zero-length) quaternion, so
        /// that a <c>default(Quaternion)</c> leaking in produces zero rather than NaN.
        /// </summary>
        public static Vector3 ToRotationVector(in Quaternion rotation)
        {
            if (rotation.LengthSquared() <= RotationEpsilon)
            {
                return Vector3.Zero;
            }

            Quaternion q = Quaternion.Normalize(rotation);
            if (q.W < 0f)
            {
                q = new Quaternion(-q.X, -q.Y, -q.Z, -q.W);
            }

            var axisPart = new Vector3(q.X, q.Y, q.Z);
            float axisPartLength = axisPart.Length();
            if (axisPartLength <= RotationEpsilon)
            {
                return Vector3.Zero;
            }

            float angle = 2f * System.MathF.Atan2(axisPartLength, q.W);
            return axisPart * (angle / axisPartLength);
        }

        /// <summary>
        /// Exponential map: the unit quaternion for a rotation vector (axis * angle, radians).
        /// Exactly inverts <see cref="ToRotationVector"/> for angles in <c>[0, pi]</c>. Returns
        /// <see cref="Quaternion.Identity"/> below <see cref="RotationEpsilon"/>, where the axis is
        /// undefined.
        /// </summary>
        public static Quaternion FromRotationVector(in Vector3 rotationVector)
        {
            float angle = rotationVector.Length();
            if (angle <= RotationEpsilon)
            {
                return Quaternion.Identity;
            }

            return Quaternion.CreateFromAxisAngle(rotationVector / angle, angle);
        }

        /// <summary>
        /// <paramref name="rotation"/> advanced by <paramref name="rotationVector"/> in the
        /// <b>world</b> frame (delta pre-multiplied), renormalized because a long chain of
        /// quaternion products drifts off the unit sphere.
        ///
        /// A rotation vector at or below <see cref="RotationEpsilon"/> returns
        /// <paramref name="rotation"/> <b>bit-identically</b> rather than multiplying by identity
        /// and renormalizing. That matters: a zero-horizon or zero-rate prediction must reproduce
        /// its input exactly, or a determinism test comparing "predict at the observation's own
        /// stamp" against the observation would fail on renormalization noise alone.
        /// </summary>
        public static Quaternion IntegrateWorld(in Quaternion rotation, in Vector3 rotationVector)
        {
            float angle = rotationVector.Length();
            if (angle <= RotationEpsilon)
            {
                return rotation;
            }

            Quaternion delta = Quaternion.CreateFromAxisAngle(rotationVector / angle, angle);
            return Quaternion.Normalize(delta * rotation);
        }

        /// <summary>
        /// Rotation vector taking <paramref name="from"/> to <paramref name="to"/> in the world
        /// frame, i.e. the <c>r</c> for which <c>IntegrateWorld(from, r) == to</c>. Divided by the
        /// elapsed seconds this is the angular rate between two samples.
        ///
        /// Inverts the unit quaternion by conjugation, which is valid only for unit input; both
        /// arguments are normalized first so a slightly-drifted orientation off the wire does not
        /// quietly scale the result.
        /// </summary>
        public static Vector3 RelativeRotationVector(in Quaternion from, in Quaternion to)
        {
            if (from.LengthSquared() <= RotationEpsilon || to.LengthSquared() <= RotationEpsilon)
            {
                return Vector3.Zero;
            }

            Quaternion delta = Quaternion.Normalize(to) * Quaternion.Conjugate(Quaternion.Normalize(from));
            return ToRotationVector(delta);
        }
    }
}
