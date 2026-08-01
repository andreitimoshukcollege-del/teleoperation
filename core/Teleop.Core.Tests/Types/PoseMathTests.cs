using System.Numerics;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Types;

public class PoseMathTests
{
    [Fact]
    public void PositionErrorMeters_SamePosition_IsZero()
    {
        var a = new Pose(new Vector3(1, 2, 3), Quaternion.Identity);
        var b = new Pose(new Vector3(1, 2, 3), Quaternion.Identity);

        Assert.Equal(0f, PoseMath.PositionErrorMeters(a, b));
    }

    [Fact]
    public void PositionErrorMeters_ComputesEuclideanDistance()
    {
        var a = new Pose(new Vector3(0, 0, 0), Quaternion.Identity);
        var b = new Pose(new Vector3(3, 4, 0), Quaternion.Identity);

        Assert.Equal(5f, PoseMath.PositionErrorMeters(a, b), 4);
    }

    [Fact]
    public void OrientationErrorRadians_SameRotation_IsZero()
    {
        var q = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.7f);
        var a = new Pose(Vector3.Zero, q);
        var b = new Pose(Vector3.Zero, q);

        Assert.Equal(0f, PoseMath.OrientationErrorRadians(a, b), 5);
    }

    [Fact]
    public void OrientationErrorRadians_DoubleCover_NegatedQuaternionIsZeroError()
    {
        var q = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 1.2f);
        var negatedQ = new Quaternion(-q.X, -q.Y, -q.Z, -q.W); // represents the identical rotation
        var a = new Pose(Vector3.Zero, q);
        var b = new Pose(Vector3.Zero, negatedQ);

        Assert.Equal(0f, PoseMath.OrientationErrorRadians(a, b), 4);
    }

    [Fact]
    public void OrientationErrorRadians_KnownRotation_MatchesExpectedAngle()
    {
        var a = new Pose(Vector3.Zero, Quaternion.Identity);
        var b = new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitX, System.MathF.PI / 2));

        Assert.Equal(System.MathF.PI / 2, PoseMath.OrientationErrorRadians(a, b), 4);
    }

    [Fact]
    public void OrientationErrorRadians_OppositeRotations_IsPi()
    {
        var a = new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0f));
        var b = new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitY, System.MathF.PI));

        Assert.Equal(System.MathF.PI, PoseMath.OrientationErrorRadians(a, b), 3);
    }

    [Fact]
    public void PositionErrorMeters_Allocates_Zero_Bytes()
    {
        var a = new Pose(new Vector3(1, 2, 3), Quaternion.Identity);
        var b = new Pose(new Vector3(4, 5, 6), Quaternion.Identity);
        AllocationAssert.Zero(() => PoseMath.PositionErrorMeters(a, b));
    }

    [Fact]
    public void OrientationErrorRadians_Allocates_Zero_Bytes()
    {
        var a = new Pose(Vector3.Zero, Quaternion.Identity);
        var b = new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.5f));
        AllocationAssert.Zero(() => PoseMath.OrientationErrorRadians(a, b));
    }
}
