using System;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>Dragging the scene's objects with a pointing ray while the robot is off (docs/adr/0031 amendment 5).</summary>
    public sealed class PointerDragTests
    {
        // The shipped scene: post-left (1) at (0.50, 0.30), post-right (2) at (0.50, -0.30), wall (3) at (0.62, 0), can (4) at
        // (0.40, 0); the table's top at z = 0; the placement region x 0.28-0.85, y +-0.65. The operator's eye about (-0.5, 0, 0.6).
        private const double Ex = -0.5, Ey = 0.0, Ez = 0.6;

        private static WorldState Posed(WorldScene scene)
        {
            var world = new WorldState(scene, new byte[32]);
            foreach (SceneObject o in scene.Objects) world.Apply(ObjectPose.Of(o.Id, 0, o.Nominal), 1);
            return world;
        }

        /// <summary>A ray from the eye at the point (x, y, z).</summary>
        private static void At(PointerDrag d, int hand, bool pinch, double x, double y, double z, WorldState world, bool tracked = true) =>
            d.Update(hand, tracked, pinch, Ex, Ey, Ez, x - Ex, y - Ey, z - Ez, world);

        [Fact]
        public void APinchOnAnObject_DragsItAlongTheTable_AndReleasingAsksOnce()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            WorldState world = Posed(scene);
            var d = new PointerDrag(scene);
            At(d, 1, true, 0.40, 0.0, 0.10, world); // the can's centre
            Assert.True(d.IsDragging);
            Assert.Equal((byte)4, d.DraggedId);
            At(d, 1, true, 0.40, 0.10, 0.10, world);
            At(d, 1, true, 0.40, 0.20, 0.10, world);
            RigidTransform preview = d.Preview;
            Assert.Equal(0.20, preview.Y, 2); // on the level through its centre, the ray moved 0.2 m across
            Assert.Equal(0.40, preview.X, 2);
            Assert.Equal(0.10, preview.Z, 6); // stood on the table
            Assert.False(d.TryTakeRequest(out _));
            At(d, 1, false, 0.40, 0.20, 0.10, world);
            Assert.False(d.IsDragging);
            Assert.True(d.TryTakeRequest(out ObjectPose move));
            Assert.Equal(((byte)4, 0.20f), (move.Id, (float)Math.Round(move.Y, 2)));
            Assert.False(d.TryTakeRequest(out _));
        }

        [Fact]
        public void TheNearestObjectAlongTheRay_IsTaken_AndMissingAsksNothing()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            WorldState world = Posed(scene);
            var d = new PointerDrag(scene);
            // From the eye through the can at (0.40, 0) and on to the wall at (0.62, 0): the can is nearer.
            Assert.Equal(3, d.Hit(Ex, Ey, Ez, 0.40 - Ex, 0 - Ey, 0.10 - Ez, world, out double near));
            Assert.True(near > 0.8 && near < 1.2);
            Assert.Equal(-1, d.Hit(Ex, Ey, Ez, 0.40 - Ex, 0.5 - Ey, 0.0 - Ez, world, out _)); // the bare table
            At(d, 0, true, 0.30, 0.6, 0.0, world);
            Assert.False(d.IsDragging);
            At(d, 0, false, 0.30, 0.6, 0.0, world);
            Assert.False(d.TryTakeRequest(out _));
        }

        [Fact]
        public void TheDrag_StaysOnTheTable_AndAClickWithoutMovingAsksNothing()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            WorldState world = Posed(scene);
            var d = new PointerDrag(scene);
            At(d, 0, true, 0.50, 0.30, 0.25, world);
            Assert.Equal((byte)1, d.DraggedId);
            At(d, 0, true, 0.50, 3.0, 0.25, world); // far off the side
            Assert.True(d.Preview.Y <= scene.RegionYMax - scene.Objects[0].FootprintRadius + 1e-9);
            At(d, 0, true, 0.50, 0.30, 0.25, world); // back where it was
            At(d, 0, false, 0.50, 0.30, 0.25, world);
            Assert.False(d.TryTakeRequest(out _));
        }

        [Fact]
        public void AFallenObject_IsStoodUpByAClick_EvenOffTheTable()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            WorldState world = Posed(scene);
            // The can on the floor beyond the table's far edge, lying on its side.
            RigidTransform lying = new RigidTransform(1.05, 0.1, -0.81, 1, 0, 0, 0).Compose(RigidTransform.Rotation(1, 0, 0, Math.PI / 2));
            world.Apply(ObjectPose.Of(4, 0, lying), 2);
            var d = new PointerDrag(scene);
            At(d, 0, true, 1.05, 0.1, -0.81, world);
            Assert.Equal((byte)4, d.DraggedId);
            At(d, 0, false, 1.05, 0.1, -0.81, world);
            Assert.True(d.TryTakeRequest(out ObjectPose move), "a click on a fallen object asks to stand it up");
            Assert.True(scene.FitsOnTable(3, move.X, move.Y), $"stood at ({move.X}, {move.Y})");
            Assert.Equal(0.10, move.Z, 5);
        }

        [Fact]
        public void OneObjectAtATime_AndLostTrackingCancels()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            WorldState world = Posed(scene);
            var d = new PointerDrag(scene);
            At(d, 0, true, 0.50, 0.30, 0.25, world);
            At(d, 1, true, 0.50, -0.30, 0.25, world); // the other hand cannot take another meanwhile
            Assert.Equal(0, d.Owner);
            At(d, 0, true, 0.50, 0.10, 0.25, world, tracked: false);
            Assert.False(d.IsDragging);
            At(d, 0, false, 0.50, 0.10, 0.25, world);
            Assert.False(d.TryTakeRequest(out _));
        }

        [Fact]
        public void Dragging_AllocatesNothing()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            WorldState world = Posed(scene);
            var d = new PointerDrag(scene);
            At(d, 0, true, 0.40, 0.0, 0.10, world);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 2000; i++)
            {
                At(d, 0, true, 0.40, 0.001 * (i % 100), 0.10, world);
                At(d, 1, false, 0.7, 0.4, 0.0, world);
            }

            Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
        }
    }
}
