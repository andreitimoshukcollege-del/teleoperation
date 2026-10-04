using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// The JetRover's virtual twin, as our own geometry: no Hiwonder files are used or committed (the
    /// repository is public). Proportions come from what is published or measured:
    /// <list type="bullet">
    /// <item>Hiwonder's specs: mecanum M1 is 324 × 260 mm; HTD-35H servos are 51.1 × 20.14 × 40 mm.</item>
    /// <item>The repo's measured arm: shoulder 3.5 cm above the yaw axis, two 13 cm links.</item>
    /// <item>Hiwonder's description: a two-deck black chassis with green accents, a lidar, a 7-inch
    /// screen, and a DaBai DCW camera on the arm.</item>
    /// </list>
    ///
    /// <para><b>The chassis</b> never moves, but it is lit like a dynamic object (light probes plus the
    /// real-time key light) rather than lightmapped. It is mostly black, and black only reads as a
    /// shape through its highlights, which lightmaps do not give. It is its own root at the arm's base,
    /// is not under any pivot, and is never driven.</para>
    ///
    /// <para><b>The arm's visuals</b> ride on the existing pivots, as dynamic children named
    /// <c>TwinVisual</c>, each modelled in its pivot's frame. That frame is Y up, links along +Z, pitch
    /// about X: the frame <c>JetRoverArmRig.ApplyAngles</c> rotates, so the twin shows exactly what
    /// is commanded. The pivots themselves and the rig's behaviour are not changed. Nothing here has
    /// a collider.</para>
    /// </summary>
    internal static class JetRoverTwin
    {
        public const string VisualName = "TwinVisual";

        private const float WheelY = -0.12f;     // wheel centre relative to the arm base
        private const float WheelX = 0.1225f;
        private const float FrontWheelZ = 0.045f, RearWheelZ = -0.145f;
        private const float DeckTop = -0.025f;   // the upper deck's top, below the turntable

        public static Renderer Build(Transform parent, LabPalette p, Transform yaw, Transform lower, Transform middle, Transform upper, Vector3 shoulderOffset)
        {
            Transform chassis = LabParts.Group("JetRoverChassis", parent).transform;
            chassis.position = LabLayout.ArmBase;
            BuildChassis(chassis, p);
            return BuildArm(p, yaw, lower, middle, upper, shoulderOffset);
        }

        // ---- chassis ------------------------------------------------------------------------

        private static void BuildChassis(Transform root, LabPalette p)
        {
            var black = new LabMesh();
            var green = new LabMesh();
            var rubber = new LabMesh();
            var alu = new LabMesh();
            var brass = new LabMesh();
            var servo = new LabMesh();
            var pcb = new LabMesh();
            var heatsink = new LabMesh();
            var gloss = new LabMesh();

            // Lower chassis: motor housing and battery bay, with green side panels and bumpers.
            black.Chamfer(new Vector3(0f, -0.105f, -0.05f), new Vector3(0.17f, 0.065f, 0.29f), 0.006f);
            foreach (int s in new[] { -1, 1 })
            {
                green.Chamfer(new Vector3(s * 0.0865f, -0.105f, -0.05f), new Vector3(0.004f, 0.045f, 0.24f), 0.0015f);
            }

            green.Chamfer(new Vector3(0f, -0.118f, 0.1f), new Vector3(0.16f, 0.03f, 0.012f), 0.004f);
            green.Chamfer(new Vector3(0f, -0.118f, -0.2f), new Vector3(0.16f, 0.03f, 0.012f), 0.004f);

            // Wheels: hub plates, a silver cap, and nine barrel rollers at 45° to the axle. The roller
            // angle alternates between diagonally opposite wheels, as on any mecanum base.
            foreach (int sx in new[] { -1, 1 })
            {
                foreach (float wz in new[] { FrontWheelZ, RearWheelZ })
                {
                    Vector3 c = new Vector3(sx * WheelX, WheelY, wz);
                    black.Cylinder(c + Vector3.right * 0.0175f, Vector3.right, 0.036f, 0.005f, 28, 0.001f);
                    black.Cylinder(c - Vector3.right * 0.0175f, Vector3.right, 0.036f, 0.005f, 28, 0.001f);
                    alu.Cylinder(c, Vector3.right, 0.013f, 0.05f, 20, 0.002f);
                    float hand = sx * (wz > 0f ? 1f : -1f);
                    const float rollerRadius = 0.0115f, rollerLength = 0.042f;
                    float ring = LabLayout.WheelRadius - rollerRadius;
                    for (int i = 0; i < 9; i++)
                    {
                        float theta = i / 9f * Mathf.PI * 2f;
                        Vector3 centre = c + new Vector3(0f, ring * Mathf.Cos(theta), ring * Mathf.Sin(theta));
                        Vector3 tangent = new Vector3(0f, -Mathf.Sin(theta), Mathf.Cos(theta));
                        Vector3 axis = (Vector3.right * hand + tangent).normalized;
                        float l = rollerLength / 2f;
                        rubber.Lathe(centre, axis, new[]
                        {
                            new Vector2(0f, -l), new Vector2(0.006f, -l), new Vector2(0.0102f, -l * 0.5f),
                            new Vector2(rollerRadius, 0f), new Vector2(0.0102f, l * 0.5f), new Vector2(0.006f, l), new Vector2(0f, l),
                        }, 10);
                    }

                    // Motor and gearbox inboard of the wheel.
                    servo.Cylinder(new Vector3(sx * 0.062f, WheelY, wz), Vector3.right, 0.0165f, 0.06f, 18, 0.002f);
                    brass.Cylinder(new Vector3(sx * 0.093f, WheelY, wz), Vector3.right, 0.0175f, 0.012f, 18, 0.001f);
                }
            }

            // Standoffs, upper deck.
            foreach (int sx in new[] { -1, 1 })
            {
                foreach (float z in new[] { -0.17f, -0.05f, 0.07f })
                {
                    brass.Cylinder(new Vector3(sx * 0.078f, -0.0505f, z), Vector3.up, 0.0035f, 0.047f, 8);
                }
            }

            black.Chamfer(new Vector3(0f, DeckTop - 0.0025f, -0.05f), new Vector3(0.2f, 0.005f, 0.3f), 0.002f);
            green.Box(new Vector3(0f, DeckTop - 0.0055f, -0.05f), new Vector3(0.204f, 0.0015f, 0.304f));

            // Between the decks: the Jetson (board, finned heatsink, fan), the controller board, the battery.
            pcb.Box(new Vector3(0f, -0.07f, -0.135f), new Vector3(0.1f, 0.0016f, 0.08f));
            heatsink.Box(new Vector3(0f, -0.064f, -0.135f), new Vector3(0.06f, 0.008f, 0.06f));
            for (int i = 0; i < 9; i++)
            {
                heatsink.Box(new Vector3(-0.028f + i * 0.007f, -0.052f, -0.135f), new Vector3(0.002f, 0.016f, 0.06f));
            }

            servo.Cylinder(new Vector3(0f, -0.042f, -0.135f), Vector3.up, 0.02f, 0.006f, 20);
            pcb.Box(new Vector3(0.035f, -0.07f, 0.03f), new Vector3(0.07f, 0.0016f, 0.06f));
            servo.Chamfer(new Vector3(-0.03f, -0.06f, 0.03f), new Vector3(0.06f, 0.028f, 0.11f), 0.004f);

            // On the deck: the base servo housing under the turntable, the lidar behind it.
            servo.Chamfer(new Vector3(0f, DeckTop + 0.0125f, 0f), new Vector3(0.055f, 0.025f, 0.045f), 0.003f);
            Vector3 lidar = new Vector3(0f, DeckTop, -0.165f);
            servo.Cylinder(lidar + Vector3.up * 0.011f, Vector3.up, 0.036f, 0.022f, 28, 0.002f);
            gloss.Cylinder(lidar + Vector3.up * 0.028f, Vector3.up, 0.0335f, 0.012f, 28);
            servo.Cylinder(lidar + Vector3.up * 0.04f, Vector3.up, 0.035f, 0.012f, 28, 0.003f);

            // The 7-inch screen on a bracket at the back, tilted up toward the operator behind the robot.
            Vector3 screenCentre = new Vector3(0f, 0.035f, -0.215f);
            Vector3 toEye = (LabLayout.OperatorEye - (LabLayout.ArmBase + screenCentre)).normalized;
            Quaternion face = Quaternion.LookRotation(-toEye, Vector3.up);
            black.Chamfer(screenCentre, new Vector3(0.18f, 0.115f, 0.012f), 0.004f, face);
            black.Chamfer(new Vector3(0f, (DeckTop + screenCentre.y) / 2f, -0.2f), new Vector3(0.04f, screenCentre.y - DeckTop, 0.006f), 0.002f);
            var lcd = new LabMesh();
            lcd.Screen(screenCentre + face * new Vector3(0f, 0f, -0.0065f), toEye, Vector3.up, new Vector2(0.155f, 0.088f));

            LabParts.Part("Twin/Black", root, black, p.AnodizedBlack, PartKind.Dynamic);
            LabParts.Part("Twin/Green", root, green, p.RobotGreen, PartKind.Dynamic);
            LabParts.Part("Twin/Rollers", root, rubber, p.Rubber, PartKind.Dynamic);
            LabParts.Part("Twin/Aluminium", root, alu, p.Aluminium, PartKind.Dynamic);
            LabParts.Part("Twin/Brass", root, brass, p.Brass, PartKind.Dynamic);
            LabParts.Part("Twin/Servos", root, servo, p.ServoBlack, PartKind.Dynamic);
            LabParts.Part("Twin/Boards", root, pcb, p.Pcb, PartKind.Dynamic);
            LabParts.Part("Twin/Heatsink", root, heatsink, p.Heatsink, PartKind.Dynamic);
            LabParts.Part("Twin/LidarWindow", root, gloss, p.LidarGloss, PartKind.Dynamic);
            LabParts.Part("Twin/Lcd", root, lcd, p.RobotLcd, PartKind.Emitter);
        }

        // ---- arm ----------------------------------------------------------------------------

        private static Renderer BuildArm(LabPalette p, Transform yaw, Transform lower, Transform middle, Transform upper, Vector3 shoulder)
        {
            // Base (rotates in yaw): turntable, U-bracket, and the shoulder servo's body.
            Transform baseVisual = Visual(yaw);
            var bBlack = new LabMesh();
            var bServo = new LabMesh();
            var bAlu = new LabMesh();
            bAlu.Cylinder(new Vector3(0f, 0.004f, 0f), Vector3.up, 0.042f, 0.008f, 32, 0.0015f);
            bBlack.Chamfer(new Vector3(shoulder.x, 0.011f, 0f), new Vector3(0.062f, 0.006f, 0.05f), 0.002f);
            foreach (int s in new[] { -1, 1 })
            {
                bBlack.Chamfer(new Vector3(shoulder.x + s * 0.031f, 0.031f, 0f), new Vector3(0.004f, 0.046f, 0.046f), 0.0015f);
            }

            bServo.Chamfer(new Vector3(shoulder.x, shoulder.y - 0.004f, 0f), new Vector3(0.040f, 0.0511f, 0.0201f), 0.003f);
            LabParts.Part("Arm/Base/Bracket", baseVisual, bBlack, p.AnodizedBlack, PartKind.Dynamic);
            LabParts.Part("Arm/Base/ShoulderServo", baseVisual, bServo, p.ServoBlack, PartKind.Dynamic);
            LabParts.Part("Arm/Base/Turntable", baseVisual, bAlu, p.Aluminium, PartKind.Dynamic);

            Link(p, lower, "Lower");
            Link(p, middle, "Middle");
            return Wrist(p, upper);
        }

        /// <summary>One 13 cm link: horn at the pivot, two side plates, a web, and the next joint's servo body.</summary>
        private static void Link(LabPalette p, Transform pivot, string name)
        {
            Transform visual = Visual(pivot);
            var plates = new LabMesh();
            var servo = new LabMesh();
            var horn = new LabMesh();
            horn.Cylinder(Vector3.zero, Vector3.right, 0.012f, 0.054f, 20, 0.002f);
            foreach (int s in new[] { -1, 1 })
            {
                plates.Chamfer(new Vector3(s * 0.0255f, 0f, 0.065f), new Vector3(0.003f, 0.03f, 0.155f), 0.0012f);
            }

            plates.Chamfer(new Vector3(0f, -0.012f, 0.06f), new Vector3(0.048f, 0.004f, 0.05f), 0.0012f);
            servo.Chamfer(new Vector3(0f, 0f, 0.13f - 0.0175f), new Vector3(0.040f, 0.0201f, 0.0511f), 0.003f);
            LabParts.Part($"Arm/{name}/Plates", visual, plates, p.AnodizedBlack, PartKind.Dynamic);
            LabParts.Part($"Arm/{name}/Servo", visual, servo, p.ServoBlack, PartKind.Dynamic);
            LabParts.Part($"Arm/{name}/Horn", visual, horn, p.Aluminium, PartKind.Dynamic);
        }

        /// <summary>Wrist bracket, wrist-roll servo, gripper (open), DaBai camera, and the reach LED.</summary>
        private static Renderer Wrist(LabPalette p, Transform pivot)
        {
            Transform visual = Visual(pivot);
            var black = new LabMesh();
            var servo = new LabMesh();
            var alu = new LabMesh();
            var pads = new LabMesh();
            var glass = new LabMesh();

            alu.Cylinder(Vector3.zero, Vector3.right, 0.012f, 0.054f, 20, 0.002f);
            foreach (int s in new[] { -1, 1 })
            {
                black.Chamfer(new Vector3(s * 0.0255f, 0f, 0.022f), new Vector3(0.003f, 0.03f, 0.05f), 0.0012f);
            }

            black.Chamfer(new Vector3(0f, 0f, 0.046f), new Vector3(0.054f, 0.03f, 0.004f), 0.0012f);
            servo.Chamfer(new Vector3(0f, 0f, 0.064f), new Vector3(0.032f, 0.032f, 0.03f), 0.004f);   // wrist roll
            black.Chamfer(new Vector3(0f, 0f, 0.088f), new Vector3(0.058f, 0.02f, 0.018f), 0.003f);  // gripper palm
            servo.Chamfer(new Vector3(0.012f, 0.019f, 0.086f), new Vector3(0.026f, 0.018f, 0.03f), 0.003f); // gripper servo
            foreach (int s in new[] { -1, 1 })
            {
                alu.Chamfer(new Vector3(s * 0.022f, 0f, 0.117f), new Vector3(0.005f, 0.014f, 0.046f), 0.0015f);
                pads.Chamfer(new Vector3(s * 0.0185f, 0f, 0.13f), new Vector3(0.0025f, 0.012f, 0.016f), 0.0008f);
            }

            // The DaBai DCW on a short bracket above the wrist, looking along the gripper.
            Vector3 cam = new Vector3(0f, 0.036f, 0.07f);
            black.Box(new Vector3(0f, 0.022f, 0.07f), new Vector3(0.02f, 0.012f, 0.01f));
            servo.Chamfer(cam, new Vector3(0.09f, 0.024f, 0.026f), 0.006f);
            foreach (float x in new[] { -0.028f, 0.028f, 0f })
            {
                float r = x == 0f ? 0.004f : 0.0065f;
                glass.Cylinder(cam + new Vector3(x, 0f, 0.0135f), Vector3.forward, r, 0.002f, 16);
                alu.Torus(cam + new Vector3(x, 0f, 0.0136f), Vector3.forward, r + 0.0008f, 0.0009f, 16, 6);
            }

            LabParts.Part("Arm/Wrist/Brackets", visual, black, p.AnodizedBlack, PartKind.Dynamic);
            LabParts.Part("Arm/Wrist/Servos", visual, servo, p.ServoBlack, PartKind.Dynamic);
            LabParts.Part("Arm/Wrist/Metal", visual, alu, p.Aluminium, PartKind.Dynamic);
            LabParts.Part("Arm/Wrist/Pads", visual, pads, p.Rubber, PartKind.Dynamic);
            LabParts.Part("Arm/Wrist/CameraGlass", visual, glass, p.Glass, PartKind.Emitter);

            // The reach warning: its own renderer, because JetRoverArmRig recolours a whole renderer.
            var led = new LabMesh();
            led.Torus(new Vector3(0f, 0f, 0.064f), Vector3.forward, 0.0215f, 0.0024f, 28, 8);
            return LabParts.Part("Arm/Wrist/ReachLed", visual, led, p.WristLed, PartKind.Emitter);
        }

        private static Transform Visual(Transform pivot)
        {
            var go = new GameObject(VisualName);
            go.transform.SetParent(pivot, false);
            return go.transform;
        }
    }
}
