using System.Numerics;
using Teleop.Core.Plant;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Plant;

public class RigidBodyPlantTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    private static CommandFrame Frame(
        long captureTicks,
        Vector3? position = null,
        Quaternion? rotation = null,
        Vector3? linearVelocity = null,
        Vector3? angularVelocity = null,
        float gripper = 0f,
        uint sequence = 1) =>
        new CommandFrame(
            sequence,
            ackSequence: 0,
            captureTicks,
            new Pose(position ?? Vector3.Zero, rotation ?? Quaternion.Identity),
            linearVelocity ?? Vector3.Zero,
            angularVelocity ?? Vector3.Zero,
            gripper);

    private static RigidBodyPlant NewPlant(long initialStateTicks = 0) =>
        new RigidBodyPlant(Pose.Identity, TicksPerSecond, initialStateTicks);

    private static void AssertSameState(Stamped<Pose> expected, Stamped<Pose> actual)
    {
        Assert.Equal(expected.CaptureTicks, actual.CaptureTicks);
        Assert.Equal(expected.Value.Position, actual.Value.Position);
        Assert.Equal(expected.Value.Rotation, actual.Value.Rotation);
    }

    [Fact]
    public void Constructor_StartsAtInitialPoseAndInitialStateTick()
    {
        var initial = new Pose(new Vector3(1f, 2f, 3f), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.5f));
        var plant = new RigidBodyPlant(initial, TicksPerSecond, initialStateTicks: 42);

        Assert.Equal(42, plant.State.CaptureTicks);
        Assert.Equal(initial.Position, plant.State.Value.Position);
        Assert.Equal(initial.Rotation, plant.State.Value.Rotation);
        Assert.Equal(0f, plant.Gripper);
        Assert.Equal(TicksPerSecond, plant.TicksPerSecond);
    }

    [Fact]
    public void Constructor_RejectsNonPositiveTicksPerSecond()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new RigidBodyPlant(Pose.Identity, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new RigidBodyPlant(Pose.Identity, -1));
    }

    // 1. Determinism.
    [Fact]
    public void IdenticalCallSequences_ProduceBitIdenticalStateThroughout()
    {
        var a = NewPlant();
        var b = NewPlant();

        for (int i = 1; i <= 200; i++)
        {
            // A mix of accepted commands, stale commands, advancing steps and no-op steps, so the
            // sequence exercises every branch rather than only the happy path.
            var frame = Frame(
                captureTicks: i % 7 == 0 ? i - 5 : i,
                position: new Vector3(i * 0.01f, i * -0.02f, i * 0.03f),
                rotation: Quaternion.CreateFromAxisAngle(Vector3.UnitY, i * 0.017f),
                linearVelocity: new Vector3(0.1f, -0.2f, 0.35f),
                angularVelocity: new Vector3(0.3f, 0.7f, -0.11f),
                gripper: (i % 11) / 10f,
                sequence: (uint)i);

            a.Command(frame);
            b.Command(frame);

            long stepTicks = i % 5 == 0 ? i - 2 : i;
            a.Step(stepTicks);
            b.Step(stepTicks);

            AssertSameState(a.State, b.State);
            Assert.Equal(a.Gripper, b.Gripper);
        }
    }

    // 2. Stale / duplicate CaptureTicks is rejected whole.
    [Fact]
    public void Command_WithStaleOrDuplicateCaptureTicks_ChangesNothingAtAll()
    {
        var plant = NewPlant();

        var accepted = Frame(
            captureTicks: 100,
            position: new Vector3(1f, 1f, 1f),
            rotation: Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.25f),
            linearVelocity: new Vector3(2f, 0f, 0f),
            angularVelocity: new Vector3(0f, 0f, 1f),
            gripper: 0.5f);
        plant.Command(accepted);
        plant.Step(TicksPerSecond / 10); // 100 ms, so the setpoint has been integrated past.

        Stamped<Pose> before = plant.State;
        float gripperBefore = plant.Gripper;

        var stale = Frame(
            captureTicks: 99,
            position: new Vector3(-50f, -50f, -50f),
            rotation: Quaternion.CreateFromAxisAngle(Vector3.UnitX, 3f),
            linearVelocity: new Vector3(-9f, -9f, -9f),
            angularVelocity: new Vector3(-9f, 0f, 0f),
            gripper: 1f);
        var duplicate = Frame(
            captureTicks: 100,
            position: new Vector3(-50f, -50f, -50f),
            rotation: Quaternion.CreateFromAxisAngle(Vector3.UnitX, 3f),
            linearVelocity: new Vector3(-9f, -9f, -9f),
            angularVelocity: new Vector3(-9f, 0f, 0f),
            gripper: 1f);

        plant.Command(stale);
        plant.Command(duplicate);

        // No partial application: pose, gripper and the state stamp are all untouched...
        AssertSameState(before, plant.State);
        Assert.Equal(gripperBefore, plant.Gripper);

        // ...and the *velocity* setpoint is untouched too, which is only visible by stepping:
        // the plant must keep coasting on the accepted command's +2 m/s X, not the stale -9.
        plant.Step(plant.State.CaptureTicks + TicksPerSecond / 10);
        Assert.Equal(before.Value.Position.X + 0.2f, plant.State.Value.Position.X, 5);
    }

    // 3. Command alone does not advance simulation time.
    [Fact]
    public void Command_DoesNotAdvanceSimulationTime()
    {
        var plant = NewPlant(initialStateTicks: 7);
        long before = plant.State.CaptureTicks;

        plant.Command(Frame(
            captureTicks: 5_000,
            position: new Vector3(4f, 5f, 6f),
            linearVelocity: new Vector3(1f, 0f, 0f)));

        // The pose snapped, but the stamp did not move: only Step advances the simulation.
        Assert.Equal(new Vector3(4f, 5f, 6f), plant.State.Value.Position);
        Assert.Equal(before, plant.State.CaptureTicks);
        Assert.Equal(7, plant.State.CaptureTicks);
    }

    // 4. Step at or before the current state time is a no-op.
    [Fact]
    public void Step_AtOrBeforeCurrentStateTime_DoesNotDoubleIntegrateOrRewind()
    {
        var plant = NewPlant();
        plant.Command(Frame(captureTicks: 1, linearVelocity: new Vector3(1f, 0f, 0f)));

        plant.Step(1000); // 1 s at 1 m/s.
        Stamped<Pose> afterFirst = plant.State;
        Assert.Equal(1f, afterFirst.Value.Position.X, 5);

        plant.Step(1000); // Duplicate step: must not integrate again.
        AssertSameState(afterFirst, plant.State);

        plant.Step(999); // Out-of-order step: must not rewind time or move the pose.
        AssertSameState(afterFirst, plant.State);

        plant.Step(0);
        AssertSameState(afterFirst, plant.State);

        // And a genuinely later step still works, measured from 1000 not from 999.
        plant.Step(1500);
        Assert.Equal(1500, plant.State.CaptureTicks);
        Assert.Equal(1.5f, plant.State.Value.Position.X, 5);
    }

    // 5. Coasting through a gap on the last commanded velocity.
    [Fact]
    public void Step_WithNoFurtherCommands_CoastsLinearlyOnLastCommandedVelocity()
    {
        var plant = NewPlant();
        var velocity = new Vector3(0.5f, -1.25f, 2f);
        plant.Command(Frame(
            captureTicks: 10,
            position: new Vector3(1f, 2f, 3f),
            linearVelocity: velocity));

        var expected = new Vector3(1f, 2f, 3f);

        // A 500 ms gap with no commands at all, stepped at 100 ms.
        for (int i = 1; i <= 5; i++)
        {
            plant.Step(i * 100);
            expected += velocity * 0.1f;

            Assert.Equal(i * 100, plant.State.CaptureTicks);
            Assert.Equal(expected.X, plant.State.Value.Position.X, 4);
            Assert.Equal(expected.Y, plant.State.Value.Position.Y, 4);
            Assert.Equal(expected.Z, plant.State.Value.Position.Z, 4);
        }

        // Coast is indefinite: no timeout, no ramp to a stop. Still moving at full rate after 5 s.
        plant.Step(5500);
        Assert.Equal(1f + velocity.X * 5.5f, plant.State.Value.Position.X, 3);
    }

    // 6. Reset restores as-constructed state, staleness baseline included.
    [Fact]
    public void Reset_RestoresAsConstructedState_IncludingTheStalenessBaseline()
    {
        var initial = new Pose(new Vector3(1f, 2f, 3f), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.5f));
        var plant = new RigidBodyPlant(initial, TicksPerSecond, initialStateTicks: 20);

        plant.Command(Frame(
            captureTicks: 9_000,
            position: new Vector3(-4f, -5f, -6f),
            rotation: Quaternion.CreateFromAxisAngle(Vector3.UnitX, 1.1f),
            linearVelocity: new Vector3(3f, 3f, 3f),
            angularVelocity: new Vector3(0f, 2f, 0f),
            gripper: 0.75f));
        plant.Step(2_000);

        plant.Reset();

        Assert.Equal(20, plant.State.CaptureTicks);
        Assert.Equal(initial.Position, plant.State.Value.Position);
        Assert.Equal(initial.Rotation, plant.State.Value.Rotation);
        Assert.Equal(0f, plant.Gripper);

        // Velocities cleared: stepping after a reset with no command must not coast.
        plant.Step(1_000);
        Assert.Equal(initial.Position, plant.State.Value.Position);
        Assert.Equal(initial.Rotation, plant.State.Value.Rotation);

        // The staleness baseline itself was cleared, not merely the visible pose: a fresh trial's
        // very first command must be accepted even at CaptureTicks 0, far below the 9000 the
        // previous trial reached.
        plant.Reset();
        plant.Command(Frame(captureTicks: 0, position: new Vector3(8f, 8f, 8f), gripper: 0.25f));
        Assert.Equal(new Vector3(8f, 8f, 8f), plant.State.Value.Position);
        Assert.Equal(0.25f, plant.Gripper);
    }

    [Fact]
    public void Reset_MakesTheInstanceBehaveExactlyLikeAFreshOne()
    {
        var initial = new Pose(new Vector3(0.5f, 0f, -0.5f), Quaternion.Identity);
        var reused = new RigidBodyPlant(initial, TicksPerSecond, initialStateTicks: 3);
        var fresh = new RigidBodyPlant(initial, TicksPerSecond, initialStateTicks: 3);

        // Dirty the reused instance with a whole trial's worth of history first.
        for (int i = 1; i <= 50; i++)
        {
            reused.Command(Frame(
                captureTicks: i * 100,
                position: new Vector3(i, i, i),
                linearVelocity: new Vector3(i, 0f, 0f),
                angularVelocity: new Vector3(0f, 0f, i * 0.1f),
                gripper: 1f,
                sequence: (uint)i));
            reused.Step(i * 100);
        }

        reused.Reset();

        for (int i = 1; i <= 20; i++)
        {
            var frame = Frame(
                captureTicks: i,
                position: new Vector3(i * 0.1f, 0f, 0f),
                linearVelocity: new Vector3(0.25f, 0f, 0f),
                angularVelocity: new Vector3(0f, 0f, 0.4f),
                gripper: 0.5f,
                sequence: (uint)i);
            reused.Command(frame);
            fresh.Command(frame);
            reused.Step(i * 10);
            fresh.Step(i * 10);

            AssertSameState(fresh.State, reused.State);
            Assert.Equal(fresh.Gripper, reused.Gripper);
        }
    }

    // 7. Rotation integration.
    [Fact]
    public void Step_IntegratesConstantAngularRateToTheExpectedRotation()
    {
        var plant = NewPlant();

        // 90 degrees/second about +Z, expressed in radians/second as the axis-angle rate vector
        // CommandFrame.AngularVelocity documents.
        float quarterTurnPerSecond = MathF.PI / 2f;
        plant.Command(Frame(captureTicks: 1, angularVelocity: new Vector3(0f, 0f, quarterTurnPerSecond)));

        // Ten 100 ms steps: one second total, so exactly a quarter turn about Z.
        for (int i = 1; i <= 10; i++)
        {
            plant.Step(i * 100);
        }

        Quaternion expected = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, quarterTurnPerSecond);
        Quaternion actual = plant.State.Value.Rotation;

        const float tolerance = 1e-5f;
        Assert.Equal(expected.X, actual.X, tolerance);
        Assert.Equal(expected.Y, actual.Y, tolerance);
        Assert.Equal(expected.Z, actual.Z, tolerance);
        Assert.Equal(expected.W, actual.W, tolerance);
    }

    [Fact]
    public void Step_OverManyStepsKeepsTheQuaternionNormalized()
    {
        var plant = NewPlant();
        plant.Command(Frame(
            captureTicks: 1,
            // A deliberately awkward, non-axis-aligned rate so the products do not stay in a plane.
            angularVelocity: new Vector3(0.37f, -1.9f, 2.4f)));

        for (int i = 1; i <= 10_000; i++)
        {
            plant.Step(i);
        }

        float length = plant.State.Value.Rotation.Length();
        Assert.Equal(1f, length, 6);
    }

    [Fact]
    public void Step_WithZeroAngularVelocity_LeavesRotationExactlyUnchanged()
    {
        var rotation = Quaternion.Normalize(new Quaternion(0.1f, 0.2f, 0.3f, 0.9f));
        var plant = NewPlant();
        plant.Command(Frame(
            captureTicks: 1,
            rotation: rotation,
            linearVelocity: new Vector3(1f, 0f, 0f),
            angularVelocity: Vector3.Zero));

        for (int i = 1; i <= 100; i++)
        {
            plant.Step(i * 10);
        }

        // Bit-identical, not merely close: a zero rate must not be renormalized into drift.
        Assert.Equal(rotation, plant.State.Value.Rotation);
    }

    // 8. Gripper passthrough, gated by the same staleness check.
    [Fact]
    public void Gripper_FollowsAcceptedCommandsAndIgnoresStaleOnes()
    {
        var plant = NewPlant();

        plant.Command(Frame(captureTicks: 10, gripper: 0.4f));
        Assert.Equal(0.4f, plant.Gripper);

        plant.Command(Frame(captureTicks: 20, gripper: 1f));
        Assert.Equal(1f, plant.Gripper);

        plant.Command(Frame(captureTicks: 20, gripper: 0f)); // duplicate stamp
        Assert.Equal(1f, plant.Gripper);

        plant.Command(Frame(captureTicks: 15, gripper: 0f)); // stale stamp
        Assert.Equal(1f, plant.Gripper);

        plant.Command(Frame(captureTicks: 21, gripper: 0f)); // fresh again
        Assert.Equal(0f, plant.Gripper);
    }

    // 9. Allocation-free hot path.
    [Fact]
    public void Command_Allocates_Zero_Bytes()
    {
        var plant = NewPlant();
        long captureTicks = 0;

        AllocationAssert.Zero(() =>
        {
            captureTicks++;
            var frame = new CommandFrame(
                sequence: 1,
                ackSequence: 0,
                captureTicks,
                new Pose(new Vector3(1f, 2f, 3f), Quaternion.Identity),
                new Vector3(0.1f, 0.2f, 0.3f),
                new Vector3(0f, 0f, 0.5f),
                0.5f);
            plant.Command(frame);
        });
    }

    [Fact]
    public void Command_WhenRejectedAsStale_Allocates_Zero_Bytes()
    {
        var plant = NewPlant();
        plant.Command(Frame(captureTicks: long.MaxValue));
        var stale = Frame(captureTicks: 1, position: new Vector3(1f, 1f, 1f));

        AllocationAssert.Zero(() => plant.Command(stale));
    }

    [Fact]
    public void Step_Allocates_Zero_Bytes()
    {
        var plant = NewPlant();
        plant.Command(Frame(
            captureTicks: 1,
            linearVelocity: new Vector3(0.1f, 0.2f, 0.3f),
            angularVelocity: new Vector3(0.4f, 0f, 0.9f)));
        long ticks = 0;

        AllocationAssert.Zero(() =>
        {
            ticks++;
            plant.Step(ticks);
        });
    }

    [Fact]
    public void Step_WhenNoOp_Allocates_Zero_Bytes()
    {
        var plant = NewPlant();
        plant.Step(1_000);

        AllocationAssert.Zero(() => plant.Step(500));
    }
}
