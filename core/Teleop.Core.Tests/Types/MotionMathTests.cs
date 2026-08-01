using System.Numerics;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Types;

public class MotionMathTests
{
    [Fact]
    public void ClampMagnitude_WithinBound_ReturnsUnchanged()
    {
        var v = new Vector3(1, 0, 0);
        Assert.Equal(v, MotionMath.ClampMagnitude(v, 5f));
    }

    [Fact]
    public void ClampMagnitude_ExceedsBound_ScalesToMaxKeepingDirection()
    {
        var v = new Vector3(10, 0, 0);
        Vector3 clamped = MotionMath.ClampMagnitude(v, 2f);

        Assert.Equal(2f, clamped.Length(), 4);
        Assert.Equal(1f, Vector3.Normalize(clamped).X, 4);
    }

    [Fact]
    public void ToRotationVector_Identity_IsZero()
    {
        Assert.Equal(Vector3.Zero, MotionMath.ToRotationVector(Quaternion.Identity));
    }

    [Fact]
    public void ToRotationVector_BelowEpsilon_IsZero()
    {
        Assert.Equal(Vector3.Zero, MotionMath.ToRotationVector(default));
    }

    [Fact]
    public void ToRotationVector_DoubleCover_NegatedQuaternionGivesSameVector()
    {
        var q = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 1.0f);
        var negatedQ = new Quaternion(-q.X, -q.Y, -q.Z, -q.W);

        Vector3 fromQ = MotionMath.ToRotationVector(q);
        Vector3 fromNegatedQ = MotionMath.ToRotationVector(negatedQ);

        Assert.Equal(fromQ.X, fromNegatedQ.X, 4);
        Assert.Equal(fromQ.Y, fromNegatedQ.Y, 4);
        Assert.Equal(fromQ.Z, fromNegatedQ.Z, 4);
    }

    [Fact]
    public void ToRotationVector_KnownRotation_MatchesExpectedAxisAndAngle()
    {
        var q = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, System.MathF.PI / 2);

        Vector3 v = MotionMath.ToRotationVector(q);

        Assert.Equal(System.MathF.PI / 2, v.Length(), 3);
        Assert.Equal(1f, Vector3.Normalize(v).Z, 3);
    }

    [Fact]
    public void FromRotationVector_Zero_IsIdentity()
    {
        Assert.Equal(Quaternion.Identity, MotionMath.FromRotationVector(Vector3.Zero));
    }

    [Fact]
    public void FromRotationVector_ToRotationVector_RoundTrips()
    {
        var original = new Vector3(0.3f, -0.2f, 0.8f); // magnitude < pi, within the invertible range

        Quaternion q = MotionMath.FromRotationVector(original);
        Vector3 roundTripped = MotionMath.ToRotationVector(q);

        Assert.Equal(original.X, roundTripped.X, 3);
        Assert.Equal(original.Y, roundTripped.Y, 3);
        Assert.Equal(original.Z, roundTripped.Z, 3);
    }

    [Fact]
    public void IntegrateWorld_ZeroRotationVector_ReturnsInputBitIdentically()
    {
        var rotation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.4f);

        Quaternion result = MotionMath.IntegrateWorld(rotation, Vector3.Zero);

        Assert.Equal(rotation.X, result.X);
        Assert.Equal(rotation.Y, result.Y);
        Assert.Equal(rotation.Z, result.Z);
        Assert.Equal(rotation.W, result.W);
    }

    [Fact]
    public void IntegrateWorld_KnownRotationVector_MatchesDirectAxisAngleConstruction()
    {
        var rotationVector = new Vector3(0, 0, System.MathF.PI / 2);

        Quaternion result = MotionMath.IntegrateWorld(Quaternion.Identity, rotationVector);
        Quaternion expected = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, System.MathF.PI / 2);

        Assert.Equal(expected.X, result.X, 4);
        Assert.Equal(expected.Y, result.Y, 4);
        Assert.Equal(expected.Z, result.Z, 4);
        Assert.Equal(expected.W, result.W, 4);
    }

    [Fact]
    public void IntegrateWorld_ManyRepeatedSteps_KeepsQuaternionNormalized()
    {
        var rotation = Quaternion.Identity;
        var rotationVector = new Vector3(0.001f, 0.0007f, -0.0012f);

        for (int i = 0; i < 10_000; i++)
        {
            rotation = MotionMath.IntegrateWorld(rotation, rotationVector);
        }

        float length = rotation.Length();
        Assert.Equal(1.0, length, 6);
    }

    [Fact]
    public void RelativeRotationVector_SameRotation_IsZero()
    {
        var q = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.5f);
        Assert.Equal(Vector3.Zero, MotionMath.RelativeRotationVector(q, q));
    }

    [Fact]
    public void RelativeRotationVector_IsExactInverseOfIntegrateWorld()
    {
        var from = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.3f);
        var appliedVector = new Vector3(0.1f, 0.2f, -0.15f);
        Quaternion to = MotionMath.IntegrateWorld(from, appliedVector);

        Vector3 recovered = MotionMath.RelativeRotationVector(from, to);

        Assert.Equal(appliedVector.X, recovered.X, 3);
        Assert.Equal(appliedVector.Y, recovered.Y, 3);
        Assert.Equal(appliedVector.Z, recovered.Z, 3);
    }

    [Fact]
    public void RelativeRotationVector_BelowEpsilonInput_IsZero()
    {
        Assert.Equal(Vector3.Zero, MotionMath.RelativeRotationVector(default, Quaternion.Identity));
        Assert.Equal(Vector3.Zero, MotionMath.RelativeRotationVector(Quaternion.Identity, default));
    }

    [Fact]
    public void ClampMagnitude_Allocates_Zero_Bytes()
    {
        var v = new Vector3(10, 0, 0);
        AllocationAssert.Zero(() => MotionMath.ClampMagnitude(v, 2f));
    }

    [Fact]
    public void ToRotationVector_Allocates_Zero_Bytes()
    {
        var q = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 1.0f);
        AllocationAssert.Zero(() => MotionMath.ToRotationVector(q));
    }

    [Fact]
    public void FromRotationVector_Allocates_Zero_Bytes()
    {
        var v = new Vector3(0.1f, 0.2f, 0.3f);
        AllocationAssert.Zero(() => MotionMath.FromRotationVector(v));
    }

    [Fact]
    public void IntegrateWorld_Allocates_Zero_Bytes()
    {
        var rotation = Quaternion.Identity;
        var v = new Vector3(0.1f, 0f, 0f);
        AllocationAssert.Zero(() => MotionMath.IntegrateWorld(rotation, v));
    }

    [Fact]
    public void RelativeRotationVector_Allocates_Zero_Bytes()
    {
        var from = Quaternion.Identity;
        var to = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.2f);
        AllocationAssert.Zero(() => MotionMath.RelativeRotationVector(from, to));
    }
}
