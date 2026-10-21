using System;
using System.Numerics;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Collision
{
    /// <summary>
    /// A small hand-made model shaped like the JetRover (docs/adr/0020), so the collision and
    /// solver tests do not depend on the CAD extraction. In the arm base frame (X forward, Z up,
    /// origin at the yaw pivot):
    /// <list type="bullet">
    /// <item>a chassis deck whose top is 1 cm below the pivot, reaching 16 cm forward;</item>
    /// <item>a screen standing up behind the arm;</item>
    /// <item>the floor 19.6 cm down;</item>
    /// <item>one box per link, and a fat gripper.</item>
    /// </list>
    /// </summary>
    internal static class TestArm
    {
        public static readonly float Deg = MathF.PI / 180f;
        public static readonly AngleRange Servo = new AngleRange(-120f * Deg, 120f * Deg);

        public static readonly CollisionBox Deck = new CollisionBox("deck", new Vector3(-0.20f, -0.12f, -0.20f), new Vector3(0.16f, 0.12f, -0.01f));
        public static readonly CollisionBox Screen = new CollisionBox("screen", new Vector3(-0.18f, -0.10f, -0.01f), new Vector3(-0.12f, 0.10f, 0.15f));

        public static ArmCollisionModel Model(float margin = 0.01f) => new ArmCollisionModel(
            name: "test",
            bodies: new[]
            {
                new CollisionBody("turntable", ArmLink.Turntable, new Vector3(0f, 0f, 0.0175f), Quaternion.Identity, new Vector3(0.03f, 0.03f, 0.0125f)),
                new CollisionBody("proximal", ArmLink.Proximal, new Vector3(0.065f, 0f, 0f), Quaternion.Identity, new Vector3(0.045f, 0.015f, 0.015f)),
                new CollisionBody("distal", ArmLink.Distal, new Vector3(0.065f, 0f, 0f), Quaternion.Identity, new Vector3(0.045f, 0.015f, 0.015f)),
                new CollisionBody("gripper", ArmLink.Gripper, new Vector3(0.05f, 0f, 0f), Quaternion.Identity, new Vector3(0.04f, 0.02f, 0.02f)),
            },
            boxes: new[] { Deck, Screen },
            floorZ: -0.196f,
            margin: margin,
            baseYaw: Servo,
            proximal: Servo,
            distal: Servo,
            wrist: Servo,
            roll: new AngleRange(-90f * Deg, 90f * Deg),
            home: new ArmPose(0f, 60f * Deg, -90f * Deg, 0f, 0f),
            // The pieces at each shared joint, which touch at every angle.
            ignoredBodyPairs: new[] { (0, 1), (1, 2), (2, 3) },
            ignoredBodyBoxes: null!,
            source: "TestArm");

        public static RobotArmProfile Profile(ArmCollisionModel? model) =>
            RobotArmProfile.JetRoverMeasuredDefault.WithCollision(model);
    }
}
