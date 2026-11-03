using System;
using System.IO;
using System.Text.Json;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>Moving the scene's objects by hand (phase P5): grab, drag, turn, clamp, release once; never two at once.</summary>
    public sealed class ObjectEditGestureTests
    {
        // The shipped scene: post-left (1) at (0.50, 0.30), post-right (2) at (0.50, -0.30), wall (3) at (0.62, 0),
        // can (4) at (0.40, 0), 0.20 tall; the table's top at z = 0; the placement region x 0.28-0.85, y +-0.65.
        private static readonly RigidTransform Forward = RigidTransform.Rotation(0, 1, 0, Math.PI / 2); // the tool's z along +x

        private static RigidTransform At(double x, double y, double z, double yaw = 0) =>
            new RigidTransform(x, y, z, 1, 0, 0, 0).Compose(RigidTransform.Rotation(0, 0, 1, yaw)).Compose(Forward);

        private static WorldState Posed(WorldScene scene, params byte[] skip)
        {
            var world = new WorldState(scene, new byte[32]);
            foreach (SceneObject o in scene.Objects)
            {
                if (Array.IndexOf(skip, o.Id) < 0) world.Apply(ObjectPose.Of(o.Id, ObjectPose.FlagKinematic, o.Nominal), 1);
            }

            return world;
        }

        private static double Yaw(in ObjectPose p) => Math.Atan2(2.0 * (p.Qw * p.Qz + p.Qx * p.Qy), 1.0 - 2.0 * (p.Qy * p.Qy + p.Qz * p.Qz));

        [Fact]
        public void AHeldCurl_GrabsTheNearestMovableObject_AndReleasingAsksOnceForWhereItWasDragged()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var g = new ObjectEditGesture(scene);
            WorldState world = Posed(scene);
            g.Update(0, true, true, At(0.41, 0.02, 0.22), world);
            Assert.True(g.IsDragging);
            Assert.Equal(4, g.DraggedId);
            Assert.Equal(0, g.Owner);

            g.Update(0, true, true, At(0.46, 0.05, 0.22), world);
            Assert.Equal(0.45, g.Preview.X, 6); // the world's poses are float32
            Assert.Equal(0.03, g.Preview.Y, 6);
            Assert.Equal(0.10, g.Preview.Z, 9); // stood on the table whatever the hand's height
            Assert.False(g.TryTakeRequest(out _)); // nothing until the hand opens

            g.Update(0, true, false, At(0.46, 0.05, 0.22), world);
            Assert.False(g.IsDragging);
            Assert.True(g.TryTakeRequest(out ObjectPose request));
            Assert.Equal(4, request.Id);
            Assert.Equal(0.45, request.X, 6);
            Assert.Equal(0.03, request.Y, 6);
            Assert.True(request.IsValid());
            Assert.False(g.TryTakeRequest(out _)); // once
        }

        [Fact]
        public void TurningTheHand_TurnsTheObject()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var g = new ObjectEditGesture(scene);
            WorldState world = Posed(scene);
            g.Update(1, true, true, At(0.50, 0.30, 0.35), world);
            Assert.Equal(1, g.DraggedId);
            g.Update(1, true, true, At(0.50, 0.30, 0.35, 0.5), world);
            g.Update(1, true, false, At(0.50, 0.30, 0.35, 0.5), world);
            Assert.True(g.TryTakeRequest(out ObjectPose request));
            Assert.Equal(0.5, Yaw(request), 6);
            Assert.Equal(0.50, request.X, 6);
        }

        [Fact]
        public void ADrag_StaysOnTheTableAndInTheRegion()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var g = new ObjectEditGesture(scene);
            WorldState world = Posed(scene);
            g.Update(0, true, true, At(0.40, 0.0, 0.22), world);
            g.Update(0, true, true, At(1.80, -2.0, 0.22), world);
            double r = scene.Objects[3].FootprintRadius;
            Assert.Equal(0.85 - r, g.Preview.X, 9);
            Assert.Equal(-0.65 + r, g.Preview.Y, 9);
            Assert.True(scene.FitsOnTable(3, g.Preview.X, g.Preview.Y));
        }

        [Fact]
        public void NothingOutOfReach_IsGrabbed_NorAnObjectTheWorldHasNotPosed()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var g = new ObjectEditGesture(scene);
            g.Update(0, true, true, At(0.40, 0.0, 0.60), Posed(scene)); // far above everything
            Assert.False(g.IsDragging);
            g.Update(0, true, false, At(0.40, 0.0, 0.60), Posed(scene));
            g.Update(0, true, true, At(0.40, 0.0, 0.22), Posed(scene, 4)); // the can, not posed yet
            Assert.False(g.IsDragging);
        }

        [Fact]
        public void AFixedObject_IsNeverGrabbed()
        {
            string text = File.ReadAllText(SceneDocumentTests.ScenePath());
            int at = text.IndexOf("\"name\": \"can\"", StringComparison.Ordinal);
            int movable = text.IndexOf("\"operatorMovable\": true", at, StringComparison.Ordinal);
            text = text.Substring(0, movable) + "\"operatorMovable\": false" + text.Substring(movable + "\"operatorMovable\": true".Length);
            WorldScene scene = JsonSerializer.Deserialize<SceneDocument>(text, new JsonSerializerOptions { IncludeFields = true })!.ToScene();
            var g = new ObjectEditGesture(scene);
            g.Update(0, true, true, At(0.40, 0.0, 0.22), Posed(scene));
            Assert.False(g.IsDragging && g.DraggedId == 4);
        }

        [Fact]
        public void ATinyDrag_AsksNothing_AndLosingTheHandCancels()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var g = new ObjectEditGesture(scene);
            WorldState world = Posed(scene);
            g.Update(0, true, true, At(0.40, 0.0, 0.22), world);
            g.Update(0, true, false, At(0.402, 0.0, 0.22), world);
            Assert.False(g.TryTakeRequest(out _));

            g.Update(0, true, true, At(0.40, 0.0, 0.22), world);
            g.Update(0, true, true, At(0.50, 0.1, 0.22), world);
            g.Update(0, false, false, At(0.50, 0.1, 0.22), world); // tracking lost
            Assert.False(g.IsDragging);
            Assert.False(g.TryTakeRequest(out _));
        }

        [Fact]
        public void OneObjectAtATime_ByOneHand()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var g = new ObjectEditGesture(scene);
            WorldState world = Posed(scene);
            g.Update(0, true, true, At(0.40, 0.0, 0.22), world);
            g.Update(1, true, true, At(0.50, -0.30, 0.35), world);
            Assert.Equal(0, g.Owner);
            Assert.Equal(4, g.DraggedId);
            g.Update(1, true, false, At(0.50, -0.30, 0.35), world); // the other hand opening changes nothing
            Assert.True(g.IsDragging);
            Assert.False(g.TryTakeRequest(out _));
            // A curl already held when the grab ends does not grab: only a new curl does.
            g.Update(0, true, false, At(0.40, 0.0, 0.22), world);
            g.Update(1, true, true, At(0.50, -0.30, 0.35), world);
            Assert.Equal(2, g.DraggedId);
        }

        [Fact]
        public void Reset_ForgetsTheDragAndTheRequest()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var g = new ObjectEditGesture(scene);
            WorldState world = Posed(scene);
            g.Update(0, true, true, At(0.40, 0.0, 0.22), world);
            g.Update(0, true, true, At(0.50, 0.1, 0.22), world);
            g.Update(0, true, false, At(0.50, 0.1, 0.22), world);
            g.Reset();
            Assert.False(g.IsDragging);
            Assert.False(g.TryTakeRequest(out _));
        }

        [Fact]
        public void Dragging_AllocatesNothing()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var g = new ObjectEditGesture(scene);
            WorldState world = Posed(scene);
            RigidTransform a = At(0.40, 0.0, 0.22), b = At(0.45, 0.05, 0.22, 0.3);
            for (int i = 0; i < 100; i++) g.Update(0, true, i % 10 != 9, i % 2 == 0 ? a : b, world);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 5000; i++)
            {
                g.Update(0, true, i % 10 != 9, i % 2 == 0 ? a : b, world);
                g.TryTakeRequest(out _);
            }

            Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
        }
    }
}
