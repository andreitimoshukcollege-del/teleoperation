using System.Collections.Generic;
using TMPro;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Every dimension the lab is laid out by, in metres, in one place. The operator stands at the
    /// origin facing +z. The robot's arm base is where TeleopVR's <c>JetRoverControl</c> puts it, and
    /// the bench and the robot's chassis are sized down from there.
    /// </summary>
    internal static class LabLayout
    {
        public static readonly Vector3 ArmBase = new Vector3(-0.014f, 1.045f, 0.328f);
        public static readonly Vector3 OperatorEye = new Vector3(0f, 1.6f, 0f);

        public const float MinX = -5f, MaxX = 5f, MinZ = -3f, MaxZ = 5f, Height = 3.2f, WallThickness = 0.15f;

        // The robot's wheels stand on the bench: arm base -> wheel centre -> wheel radius.
        public const float WheelRadius = 0.0485f;
        public const float WheelCentreBelowArmBase = 0.12f;
        public static float BenchTop => ArmBase.y - WheelCentreBelowArmBase - WheelRadius;

        public const float BenchHalfWidth = 0.8f, BenchNearZ = 0.1f, BenchDepth = 0.8f;
        public static float BenchCentreZ => BenchNearZ + BenchDepth / 2f;

        // The taped workcell on the floor, and the status ring just inside it.
        public const float CellMinX = -1.2f, CellMaxX = 1.2f, CellMinZ = -0.45f, CellMaxZ = 1.45f;

        // The big camera display stands just outside the workcell, to the right, facing the operator.
        public static readonly Vector3 DisplayFoot = new Vector3(1.45f, 0f, 0.95f);
        public const float ScreenCentreHeight = 1.42f;

        // Window strip on the left wall.
        public const float WindowMinZ = -1.6f, WindowMaxZ = 3.6f, WindowBottom = 0.95f, WindowTop = 2.45f;

        // The taped mobile-robot test area on the open floor beyond the workcell, and the point the
        // motion-capture cameras around it aim at.
        public const float ArenaMinX = -1.9f, ArenaMaxX = 1.9f, ArenaMinZ = 2.15f, ArenaMaxZ = 4.15f;
        public static readonly Vector3 CaptureVolumeCentre = new Vector3(0f, 0.4f, 2.6f);
    }

    /// <summary>The room, its furniture and props: everything that never moves.</summary>
    internal static class LabRoom
    {
        public sealed class Result
        {
            public Renderer StatusRing;
            public Renderer[] RackLeds;
        }

        public static Result Build(Transform parent, LabPalette p)
        {
            Transform room = LabParts.Group("Room", parent).transform;
            BuildShell(room, p);
            BuildSignWall(room, p);
            BuildFixtures(room, p);
            BuildCeilingServices(room, p);
            BuildBench(room, p);
            Renderer ring = BuildWorkcell(room, p);
            Renderer[] leds = BuildRacks(room, p);
            BuildDesks(room, p);
            BuildShelving(room, p);
            BuildDoorWall(room, p);
            BuildDecor(room, p);
            BuildArena(room, p);
            BuildMotionCapture(room, p);
            return new Result { StatusRing = ring, RackLeds = leds };
        }

        // ---- shell --------------------------------------------------------------------------

        private static void BuildShell(Transform room, LabPalette p)
        {
            const float minX = LabLayout.MinX, maxX = LabLayout.MaxX, minZ = LabLayout.MinZ, maxZ = LabLayout.MaxZ, h = LabLayout.Height, t = LabLayout.WallThickness;
            float cx = (minX + maxX) / 2f, cz = (minZ + maxZ) / 2f, sx = maxX - minX, sz = maxZ - minZ;

            LabParts.Part("Room/Floor", room, new LabMesh().Box(new Vector3(cx, -0.01f, cz), new Vector3(sx, 0.02f, sz)), p.Floor, PartKind.Static, 0.6f);
            LabParts.Part("Room/Ceiling", room, new LabMesh().Box(new Vector3(cx, h + 0.05f, cz), new Vector3(sx + 2 * t, 0.1f, sz + 2 * t)), p.Ceiling, PartKind.Static, 0.35f);

            var walls = new LabMesh();
            walls.Box(new Vector3(cx, h / 2f, maxZ + t / 2f), new Vector3(sx + 2 * t, h, t));
            walls.Box(new Vector3(cx, h / 2f, minZ - t / 2f), new Vector3(sx + 2 * t, h, t));
            walls.Box(new Vector3(maxX + t / 2f, h / 2f, cz), new Vector3(t, h, sz));
            // Left wall, around the window opening.
            float wz0 = LabLayout.WindowMinZ, wz1 = LabLayout.WindowMaxZ, wy0 = LabLayout.WindowBottom, wy1 = LabLayout.WindowTop;
            walls.Box(new Vector3(minX - t / 2f, wy0 / 2f, cz), new Vector3(t, wy0, sz));
            walls.Box(new Vector3(minX - t / 2f, (wy1 + h) / 2f, cz), new Vector3(t, h - wy1, sz));
            walls.Box(new Vector3(minX - t / 2f, (wy0 + wy1) / 2f, (minZ + wz0) / 2f), new Vector3(t, wy1 - wy0, wz0 - minZ));
            walls.Box(new Vector3(minX - t / 2f, (wy0 + wy1) / 2f, (wz1 + maxZ) / 2f), new Vector3(t, wy1 - wy0, maxZ - wz1));
            LabParts.Part("Room/Walls", room, walls, p.Wall, PartKind.Static, 0.7f);

            // Skirting, with a cyan LED line on top of it, and a cove line where wall meets ceiling.
            var skirting = new LabMesh();
            var cove = new LabMesh();
            void Run(Vector3 a, Vector3 b, Vector3 inward)
            {
                Vector3 mid = (a + b) / 2f, along = b - a;
                Vector3 size = new Vector3(Mathf.Abs(along.x) + 0.001f, 0.12f, Mathf.Abs(along.z) + 0.001f) + new Vector3(Mathf.Abs(inward.x), 0, Mathf.Abs(inward.z)) * 0.025f;
                skirting.Box(mid + Vector3.up * 0.06f + inward * 0.0125f, size);
                cove.Box(mid + Vector3.up * 0.126f + inward * 0.02f, new Vector3(Mathf.Abs(along.x) + 0.001f, 0.012f, Mathf.Abs(along.z) + 0.001f) + new Vector3(Mathf.Abs(inward.x), 0, Mathf.Abs(inward.z)) * 0.012f);
                cove.Box(mid + Vector3.up * (h - 0.06f) + inward * 0.015f, new Vector3(Mathf.Abs(along.x) + 0.001f, 0.02f, Mathf.Abs(along.z) + 0.001f) + new Vector3(Mathf.Abs(inward.x), 0, Mathf.Abs(inward.z)) * 0.02f);
            }

            Run(new Vector3(minX, 0, maxZ), new Vector3(maxX, 0, maxZ), Vector3.back);
            Run(new Vector3(minX, 0, minZ), new Vector3(maxX, 0, minZ), Vector3.forward);
            Run(new Vector3(maxX, 0, minZ), new Vector3(maxX, 0, maxZ), Vector3.left);
            Run(new Vector3(minX, 0, minZ), new Vector3(minX, 0, maxZ), Vector3.right);
            LabParts.Part("Room/Skirting", room, skirting, p.WallDark, PartKind.Static, 0.5f);
            LabParts.Part("Room/CoveLights", room, cove, p.CoveCyan, PartKind.Glow);

            // Window: a frame with mullions, and the night skyline a little behind the wall.
            var frame = new LabMesh();
            float fx = minX + 0.01f;
            frame.Box(new Vector3(fx, wy0, (wz0 + wz1) / 2f), new Vector3(0.06f, 0.05f, wz1 - wz0 + 0.05f));
            frame.Box(new Vector3(fx, wy1, (wz0 + wz1) / 2f), new Vector3(0.06f, 0.05f, wz1 - wz0 + 0.05f));
            for (int i = 0; i <= 5; i++)
            {
                float z = Mathf.Lerp(wz0, wz1, i / 5f);
                frame.Box(new Vector3(fx, (wy0 + wy1) / 2f, z), new Vector3(0.06f, wy1 - wy0, 0.05f));
            }

            frame.Box(new Vector3(minX - t / 2f, (wy0 + wy1) / 2f, wz0 - 0.005f), new Vector3(t, wy1 - wy0, 0.01f));
            frame.Box(new Vector3(minX - t / 2f, (wy0 + wy1) / 2f, wz1 + 0.005f), new Vector3(t, wy1 - wy0, 0.01f));
            frame.Box(new Vector3(minX - 0.3f, (wy0 + wy1) / 2f, (wz0 + wz1) / 2f) + Vector3.up * ((wy1 - wy0) / 2f + 0.005f), new Vector3(0.45f, 0.01f, wz1 - wz0));
            frame.Box(new Vector3(minX - 0.3f, (wy0 + wy1) / 2f, (wz0 + wz1) / 2f) - Vector3.up * ((wy1 - wy0) / 2f + 0.005f), new Vector3(0.45f, 0.01f, wz1 - wz0));
            LabParts.Part("Room/WindowFrame", room, frame, p.WindowFrame, PartKind.Static, 1f);

            var sky = new LabMesh();
            sky.Screen(new Vector3(minX - 0.5f, (wy0 + wy1) / 2f, (wz0 + wz1) / 2f), Vector3.right, Vector3.up, new Vector2(wz1 - wz0 + 1.2f, (wy1 - wy0) + 0.6f));
            LabParts.Part("Room/Skyline", room, sky, p.Skyline, PartKind.Glow);
        }

        // ---- sign wall ----------------------------------------------------------------------

        private static void BuildSignWall(Transform room, LabPalette p)
        {
            float z = LabLayout.MaxZ;
            var slats = new LabMesh();
            slats.Box(new Vector3(-3.3f, 1.6f, z - 0.03f), new Vector3(2.8f, 2.3f, 0.06f));
            slats.Box(new Vector3(3.3f, 1.6f, z - 0.03f), new Vector3(2.8f, 2.3f, 0.06f));
            LabParts.Part("Room/SlatPanels", room, slats, p.Slats, PartKind.Static, 1f);

            LabParts.Part("Room/SignPanel", room, new LabMesh().Chamfer(new Vector3(0f, 2.15f, z - 0.04f), new Vector3(3.2f, 1.2f, 0.08f), 0.02f), p.WallDark, PartKind.Static, 1f);
            var glow = new LabMesh();
            glow.Box(new Vector3(0f, 1.53f, z - 0.085f), new Vector3(3.0f, 0.012f, 0.01f));
            glow.Box(new Vector3(0f, 2.77f, z - 0.085f), new Vector3(3.0f, 0.012f, 0.01f));
            LabParts.Part("Room/SignGlow", room, glow, p.CoveCyan, PartKind.Glow);

            Quaternion facing = Quaternion.identity; // TMP faces -z, toward the room
            LabParts.Label("SignTitle", room, new Vector3(0f, 2.25f, z - 0.09f), facing, "TELEOPERATION LAB", 3.2f,
                new Color(0.55f, 0.92f, 1f), new Vector2(3f, 0.4f), TextAlignmentOptions.Center, FontStyles.Bold);
            LabParts.Label("SignSubtitle", room, new Vector3(0f, 1.86f, z - 0.09f), facing, "remote manipulation  ·  latency research  ·  JetRover", 1.1f,
                new Color(0.55f, 0.6f, 0.68f), new Vector2(3f, 0.15f));
        }

        // ---- light fixtures (the lights themselves are LabLighting's) -----------------------

        public static readonly Vector3[] FixturePositions =
        {
            new Vector3(-2.4f, LabLayout.Height - 0.45f, 0.4f), new Vector3(2.4f, LabLayout.Height - 0.45f, 0.4f),
            new Vector3(-2.4f, LabLayout.Height - 0.45f, 3.2f), new Vector3(0f, LabLayout.Height - 0.45f, 3.2f), new Vector3(2.4f, LabLayout.Height - 0.45f, 3.2f),
            new Vector3(0f, LabLayout.Height - 0.45f, -1.8f),
        };

        public static Vector3 TaskFixture => new Vector3(LabLayout.ArmBase.x, 2.35f, LabLayout.BenchCentreZ);

        private static void BuildFixtures(Transform room, LabPalette p)
        {
            var housing = new LabMesh();
            var diffuser = new LabMesh();
            void Fixture(Vector3 c, float length)
            {
                housing.Chamfer(c, new Vector3(length, 0.06f, 0.16f), 0.012f);
                diffuser.Box(c + Vector3.down * 0.031f, new Vector3(length - 0.04f, 0.004f, 0.12f));
                for (int s = -1; s <= 1; s += 2)
                {
                    float top = LabLayout.Height - (c.y + 0.03f);
                    housing.Cylinder(c + new Vector3(s * (length / 2f - 0.15f), 0.03f + top / 2f, 0f), Vector3.up, 0.003f, top, 6);
                }
            }

            foreach (Vector3 c in FixturePositions) Fixture(c, 1.5f);
            Fixture(TaskFixture, 1.2f);
            LabParts.Part("Room/FixtureHousings", room, housing, p.FixtureHousing, PartKind.Static, 1f);
            LabParts.Part("Room/FixtureDiffusers", room, diffuser, p.FixtureDiffuser, PartKind.Glow);
        }

        private static void BuildCeilingServices(Transform room, LabPalette p)
        {
            var trays = new LabMesh();
            foreach (float x in new[] { -3.6f, 3.6f })
            {
                float y = LabLayout.Height - 0.25f, len = LabLayout.MaxZ - LabLayout.MinZ - 0.4f, cz = (LabLayout.MinZ + LabLayout.MaxZ) / 2f;
                trays.Box(new Vector3(x, y, cz), new Vector3(0.3f, 0.01f, len));
                trays.Box(new Vector3(x - 0.15f, y + 0.04f, cz), new Vector3(0.01f, 0.08f, len));
                trays.Box(new Vector3(x + 0.15f, y + 0.04f, cz), new Vector3(0.01f, 0.08f, len));
                for (float z = LabLayout.MinZ + 0.6f; z < LabLayout.MaxZ - 0.4f; z += 1.5f)
                {
                    trays.Cylinder(new Vector3(x, (y + LabLayout.Height) / 2f, z), Vector3.up, 0.005f, LabLayout.Height - y, 6);
                }
            }

            trays.Cylinder(new Vector3(0f, LabLayout.Height - 0.32f, LabLayout.MaxZ - 0.55f), Vector3.right, 0.17f, LabLayout.MaxX - LabLayout.MinX - 0.3f, 24, 0.02f);
            LabParts.Part("Room/CeilingServices", room, trays, p.SteelDark, PartKind.Static, 0.5f);
        }

        // ---- the robot's bench and the taped workcell ---------------------------------------

        private static void BuildBench(Transform room, LabPalette p)
        {
            float top = LabLayout.BenchTop, hw = LabLayout.BenchHalfWidth, cz = LabLayout.BenchCentreZ, d = LabLayout.BenchDepth;
            float cx = LabLayout.ArmBase.x;
            LabParts.Part("Bench/Top", room, new LabMesh().Chamfer(new Vector3(cx, top - 0.02f, cz), new Vector3(2 * hw, 0.04f, d), 0.008f), p.BenchTop, PartKind.Static, 3f);
            var mat = new LabMesh { UvScale = 1f };
            mat.Box(new Vector3(cx + 0.05f, top + 0.0015f, cz + 0.02f), new Vector3(1.25f, 0.003f, 0.62f));
            LabParts.Part("Bench/EsdMat", room, mat, p.BenchMat, PartKind.Static, 3f);

            var frame = new LabMesh();
            float legH = top - 0.04f;
            foreach (int sx in new[] { -1, 1 })
            {
                foreach (int sz in new[] { -1, 1 })
                {
                    frame.Chamfer(new Vector3(cx + sx * (hw - 0.06f), legH / 2f, cz + sz * (d / 2f - 0.06f)), new Vector3(0.045f, legH, 0.045f), 0.004f);
                }

                frame.Chamfer(new Vector3(cx + sx * (hw - 0.06f), 0.16f, cz), new Vector3(0.04f, 0.04f, d - 0.12f), 0.004f);
                frame.Chamfer(new Vector3(cx + sx * (hw - 0.06f), legH - 0.03f, cz), new Vector3(0.04f, 0.04f, d - 0.12f), 0.004f);
            }

            frame.Chamfer(new Vector3(cx, 0.16f, cz + d / 2f - 0.06f), new Vector3(2 * hw - 0.12f, 0.04f, 0.04f), 0.004f);
            frame.Chamfer(new Vector3(cx, legH - 0.03f, cz + d / 2f - 0.06f), new Vector3(2 * hw - 0.12f, 0.04f, 0.04f), 0.004f);
            LabParts.Part("Bench/Frame", room, frame, p.Aluminium, PartKind.Static, 1.5f);
            LabParts.Part("Bench/Shelf", room, new LabMesh().Chamfer(new Vector3(cx, 0.2f, cz), new Vector3(2 * hw - 0.16f, 0.025f, d - 0.16f), 0.005f), p.SteelDark, PartKind.Static, 1f);

            // A toolbox and a crate on the lower shelf, an under-bench glow on the far edge.
            LabParts.Part("Bench/Toolbox", room, new LabMesh().Chamfer(new Vector3(cx + 0.4f, 0.31f, cz + 0.05f), new Vector3(0.48f, 0.2f, 0.24f), 0.012f), p.SafetyRed, PartKind.Static, 1f);
            LabParts.Part("Bench/Crate", room, new LabMesh().Chamfer(new Vector3(cx - 0.42f, 0.32f, cz), new Vector3(0.4f, 0.22f, 0.32f), 0.01f), p.BinGrey, PartKind.Static, 1f);
            LabParts.Part("Bench/Underglow", room, new LabMesh().Box(new Vector3(cx, top - 0.045f, cz + d / 2f - 0.01f), new Vector3(2 * hw - 0.1f, 0.006f, 0.006f)), p.CoveCyan, PartKind.Glow);
        }

        private static Renderer BuildWorkcell(Transform room, LabPalette p)
        {
            float x0 = LabLayout.CellMinX, x1 = LabLayout.CellMaxX, z0 = LabLayout.CellMinZ, z1 = LabLayout.CellMaxZ;
            const float tape = 0.08f, y = 0.0015f;
            var hazard = new LabMesh { UvScale = 1f };
            hazard.Box(new Vector3((x0 + x1) / 2f, y, z0), new Vector3(x1 - x0 + tape, 0.003f, tape));
            hazard.Box(new Vector3((x0 + x1) / 2f, y, z1), new Vector3(x1 - x0 + tape, 0.003f, tape));
            hazard.Box(new Vector3(x0, y, (z0 + z1) / 2f), new Vector3(tape, 0.003f, z1 - z0 - tape));
            hazard.Box(new Vector3(x1, y, (z0 + z1) / 2f), new Vector3(tape, 0.003f, z1 - z0 - tape));
            LabParts.Part("Workcell/Hazard", room, hazard, p.Hazard, PartKind.Static, 2f);

            // The status ring: a thin bright line inside the tape, recoloured at runtime by LinkStatusLamp.
            const float inset = 0.12f, w = 0.03f, yr = 0.0035f;
            float a0 = x0 + inset, a1 = x1 - inset, b0 = z0 + inset, b1 = z1 - inset;
            var ring = new LabMesh();
            ring.Box(new Vector3((a0 + a1) / 2f, yr, b0), new Vector3(a1 - a0 + w, 0.004f, w));
            ring.Box(new Vector3((a0 + a1) / 2f, yr, b1), new Vector3(a1 - a0 + w, 0.004f, w));
            ring.Box(new Vector3(a0, yr, (b0 + b1) / 2f), new Vector3(w, 0.004f, b1 - b0 - w));
            ring.Box(new Vector3(a1, yr, (b0 + b1) / 2f), new Vector3(w, 0.004f, b1 - b0 - w));
            return LabParts.Part("Workcell/StatusRing", room, ring, p.StatusRing, PartKind.Emitter);
        }

        // ---- server racks -------------------------------------------------------------------

        private static Renderer[] BuildRacks(Transform room, LabPalette p)
        {
            var leds = new List<Renderer>();
            Transform group = LabParts.Group("Racks", room).transform;
            var body = new LabMesh();
            var front = new LabMesh { UvScale = 1f };
            int ledIndex = 0;
            foreach (float x in new[] { 3.45f, 4.15f })
            {
                Vector3 c = new Vector3(x, 1.0f, LabLayout.MaxZ - 0.62f);
                body.Chamfer(c, new Vector3(0.62f, 2.0f, 1.05f), 0.01f);
                body.Chamfer(c + new Vector3(0f, 1.02f, 0f), new Vector3(0.64f, 0.04f, 1.07f), 0.006f);
                front.Screen(c + new Vector3(0f, 0f, -0.53f), Vector3.back, Vector3.up, new Vector2(0.5f, 1.8f));
                var rng = new System.Random(17 + (int)(x * 10));
                for (int i = 0; i < 10; i++)
                {
                    var led = new LabMesh();
                    float ly = 0.25f + (float)rng.NextDouble() * 1.5f, lx = -0.2f + (float)rng.NextDouble() * 0.4f;
                    led.Box(c + new Vector3(lx, ly - 1.0f, -0.537f), new Vector3(0.008f, 0.008f, 0.004f));
                    leds.Add(LabParts.Part($"Racks/Led{ledIndex++}", group, led, p.Led, PartKind.Emitter));
                }
            }

            LabParts.Part("Racks/Bodies", group, body, p.RackBody, PartKind.Static, 1f);
            LabParts.Part("Racks/Fronts", group, front, p.RackFront, PartKind.Static, 1f);
            return leds.ToArray();
        }

        // ---- desks along the window --------------------------------------------------------

        private static void BuildDesks(Transform room, LabPalette p)
        {
            Transform group = LabParts.Group("Desks", room).transform;
            var tops = new LabMesh();
            var legs = new LabMesh();
            var plastic = new LabMesh();
            var fabric = new LabMesh();
            var code = new LabMesh();
            var graph = new LabMesh();
            float x = LabLayout.MinX + 1.25f;
            int n = 0;
            foreach (float z in new[] { 0.0f, 2.0f })
            {
                // Desk against the window wall; the operator of this desk faces -x, the screens face +x (the room).
                tops.Chamfer(new Vector3(x, 0.74f, z), new Vector3(0.75f, 0.035f, 1.6f), 0.006f);
                foreach (int sz in new[] { -1, 1 })
                {
                    legs.Chamfer(new Vector3(x, 0.36f, z + sz * 0.74f), new Vector3(0.65f, 0.72f, 0.05f), 0.006f);
                }

                foreach (int side in new[] { -1, 1 })
                {
                    Vector3 screenCentre = new Vector3(x - 0.2f, 1.08f, z + side * 0.33f);
                    // Screens face +x, into the room, angled in a little toward the chair.
                    Quaternion yaw = Quaternion.Euler(0f, -90f + side * 10f, 0f);
                    plastic.Chamfer(screenCentre, new Vector3(0.62f, 0.38f, 0.03f), 0.006f, yaw);
                    plastic.Box(new Vector3(x - 0.22f, 0.86f, z + side * 0.33f), new Vector3(0.05f, 0.22f, 0.05f));
                    plastic.Chamfer(new Vector3(x - 0.22f, 0.762f, z + side * 0.33f), new Vector3(0.22f, 0.012f, 0.16f), 0.004f);
                    (side < 0 ? code : graph).Screen(screenCentre + yaw * new Vector3(0f, 0f, -0.016f), yaw * Vector3.back, Vector3.up, new Vector2(0.59f, 0.35f));
                }

                plastic.Chamfer(new Vector3(x + 0.12f, 0.764f, z), new Vector3(0.16f, 0.015f, 0.44f), 0.003f);
                plastic.Chamfer(new Vector3(x - 0.05f, 0.25f, z + 0.55f), new Vector3(0.45f, 0.45f, 0.2f), 0.01f);

                // Chair, pushed in.
                Vector3 seat = new Vector3(x + 0.62f, 0.47f, z + 0.1f);
                fabric.Chamfer(seat, new Vector3(0.48f, 0.07f, 0.48f), 0.02f);
                fabric.Chamfer(seat + new Vector3(0.24f, 0.3f, 0f), new Vector3(0.06f, 0.5f, 0.44f), 0.02f);
                legs.Cylinder(seat + Vector3.down * 0.23f, Vector3.up, 0.025f, 0.4f, 10);
                for (int k = 0; k < 5; k++)
                {
                    Quaternion r = Quaternion.Euler(0f, k * 72f, 0f);
                    legs.Box(seat + Vector3.down * 0.42f + r * new Vector3(0.15f, 0f, 0f), new Vector3(0.3f, 0.025f, 0.04f), r);
                }

                n++;
            }

            LabParts.Part("Desks/Tops", group, tops, p.BenchTop, PartKind.Static, 1.5f);
            LabParts.Part("Desks/Legs", group, legs, p.SteelDark, PartKind.Static, 1f);
            LabParts.Part("Desks/Hardware", group, plastic, p.Plastic, PartKind.Static, 1f);
            LabParts.Part("Desks/Chairs", group, fabric, p.ChairFabric, PartKind.Static, 1f);
            LabParts.Part("Desks/CodeScreens", group, code, p.CodeScreen, PartKind.Glow);
            LabParts.Part("Desks/GraphScreens", group, graph, p.GraphScreen, PartKind.Glow);
        }

        // ---- shelving, door, pegboard -------------------------------------------------------

        private static void BuildShelving(Transform room, LabPalette p)
        {
            Transform group = LabParts.Group("Shelving", room).transform;
            var steel = new LabMesh();
            var blue = new LabMesh();
            var grey = new LabMesh();
            var orange = new LabMesh();
            float x = LabLayout.MaxX - 0.28f, z0 = 0.4f, z1 = 2.6f;
            foreach (float z in new[] { z0, z1 })
            {
                foreach (float dx in new[] { -0.2f, 0.2f })
                {
                    steel.Box(new Vector3(x + dx, 1.0f, z), new Vector3(0.03f, 2.0f, 0.03f));
                }
            }

            var rng = new System.Random(5);
            for (int level = 0; level < 5; level++)
            {
                float y = 0.12f + level * 0.44f;
                steel.Box(new Vector3(x, y, (z0 + z1) / 2f), new Vector3(0.44f, 0.02f, z1 - z0 + 0.03f));
                if (level == 4) break;
                for (float z = z0 + 0.22f; z < z1 - 0.15f; z += 0.42f)
                {
                    float hgt = 0.16f + (float)rng.NextDouble() * 0.14f;
                    var target = rng.NextDouble() < 0.45 ? blue : rng.NextDouble() < 0.6 ? grey : orange;
                    target.Chamfer(new Vector3(x, y + 0.01f + hgt / 2f, z), new Vector3(0.36f, hgt, 0.34f), 0.008f);
                }
            }

            LabParts.Part("Shelving/Frame", group, steel, p.SteelDark, PartKind.Static, 1f);
            LabParts.Part("Shelving/BinsBlue", group, blue, p.BinBlue, PartKind.Static, 1f);
            LabParts.Part("Shelving/BinsGrey", group, grey, p.BinGrey, PartKind.Static, 1f);
            LabParts.Part("Shelving/BinsOrange", group, orange, p.BinOrange, PartKind.Static, 1f);
        }

        private static void BuildDoorWall(Transform room, LabPalette p)
        {
            Transform group = LabParts.Group("DoorWall", room).transform;
            float x = LabLayout.MaxX;
            LabParts.Part("DoorWall/Door", group, new LabMesh().Chamfer(new Vector3(x - 0.03f, 1.05f, -2.0f), new Vector3(0.05f, 2.1f, 0.95f), 0.01f), p.Door, PartKind.Static, 1f);
            var trim = new LabMesh();
            trim.Box(new Vector3(x - 0.06f, 1.0f, -2.38f), new Vector3(0.02f, 0.02f, 0.12f));
            trim.Box(new Vector3(x - 0.02f, 2.13f, -2.0f), new Vector3(0.06f, 0.06f, 1.07f));
            trim.Box(new Vector3(x - 0.02f, 1.05f, -2.5f), new Vector3(0.06f, 2.16f, 0.06f));
            trim.Box(new Vector3(x - 0.02f, 1.05f, -1.5f), new Vector3(0.06f, 2.16f, 0.06f));
            LabParts.Part("DoorWall/Trim", group, trim, p.Aluminium, PartKind.Static, 1f);
            LabParts.Part("DoorWall/ExitSign", group, new LabMesh().Chamfer(new Vector3(x - 0.06f, 2.42f, -2.0f), new Vector3(0.06f, 0.16f, 0.4f), 0.008f), p.ExitSign, PartKind.Glow);
            LabParts.Label("ExitLabel", group, new Vector3(x - 0.095f, 2.42f, -2.0f), Quaternion.Euler(0f, 90f, 0f), "EXIT", 1.2f,
                new Color(0.95f, 1f, 0.95f), new Vector2(0.38f, 0.14f), TextAlignmentOptions.Center, FontStyles.Bold);
            LabParts.Part("DoorWall/Extinguisher", group, new LabMesh().Cylinder(new Vector3(x - 0.14f, 0.32f, -1.1f), Vector3.up, 0.08f, 0.55f, 16, 0.03f), p.SafetyRed, PartKind.Static, 1f);

            // Pegboard tool wall between the door and the shelving.
            float pz0 = -1.0f, pz1 = 0.2f;
            var board = new LabMesh { UvScale = 1f };
            board.Box(new Vector3(x - 0.02f, 1.5f, (pz0 + pz1) / 2f), new Vector3(0.02f, 1.1f, pz1 - pz0));
            LabParts.Part("DoorWall/Pegboard", group, board, p.Pegboard, PartKind.Static, 1.5f);
            var tools = new LabMesh();
            var rng = new System.Random(9);
            for (int i = 0; i < 9; i++)
            {
                float z = pz0 + 0.12f + i * 0.12f, y = 1.35f + (float)rng.NextDouble() * 0.45f;
                if (i % 3 == 0) tools.Cylinder(new Vector3(x - 0.05f, y, z), Vector3.up, 0.012f, 0.22f, 8);
                else tools.Chamfer(new Vector3(x - 0.045f, y, z), new Vector3(0.015f, 0.18f + (float)rng.NextDouble() * 0.1f, 0.04f), 0.004f);
            }

            LabParts.Part("DoorWall/Tools", group, tools, p.SteelDark, PartKind.Static, 1f);
        }

        // ---- decor: an industrial arm and a plant -------------------------------------------

        private static void BuildDecor(Transform room, LabPalette p)
        {
            Transform group = LabParts.Group("Decor", room).transform;
            Vector3 b = new Vector3(-3.3f, 0f, 4.0f);
            var orange = new LabMesh();
            var dark = new LabMesh();
            dark.Cylinder(b + Vector3.up * 0.3f, Vector3.up, 0.32f, 0.6f, 32, 0.02f);
            orange.Cylinder(b + Vector3.up * 0.7f, Vector3.up, 0.22f, 0.2f, 32, 0.02f);
            Vector3 shoulder = b + Vector3.up * 0.95f;
            orange.Cylinder(shoulder, Vector3.right, 0.14f, 0.32f, 28, 0.02f);
            Quaternion lowerTilt = Quaternion.Euler(-25f, 0f, 0f);
            Vector3 elbow = shoulder + lowerTilt * new Vector3(0f, 0.75f, 0f);
            orange.Chamfer((shoulder + elbow) / 2f, new Vector3(0.18f, 0.78f, 0.2f), 0.03f, lowerTilt);
            dark.Cylinder(elbow, Vector3.right, 0.11f, 0.26f, 24, 0.015f);
            Quaternion upperTilt = Quaternion.Euler(70f, 0f, 0f);
            Vector3 wrist = elbow + upperTilt * new Vector3(0f, 0.65f, 0f);
            orange.Chamfer((elbow + wrist) / 2f, new Vector3(0.14f, 0.66f, 0.15f), 0.025f, upperTilt);
            dark.Cylinder(wrist, upperTilt * Vector3.up, 0.07f, 0.12f, 20, 0.01f);
            dark.Cylinder(wrist + upperTilt * new Vector3(0f, 0.1f, 0f), upperTilt * Vector3.up, 0.04f, 0.1f, 16);
            LabParts.Part("Decor/IndustrialArm", group, orange, p.OrangeArm, PartKind.Static, 1f);
            LabParts.Part("Decor/IndustrialArmDark", group, dark, p.SteelDark, PartKind.Static, 1f);

            Vector3 pot = new Vector3(LabLayout.MinX + 0.45f, 0f, LabLayout.MaxZ - 0.45f);
            LabParts.Part("Decor/Pot", group, new LabMesh().Lathe(pot, Vector3.up,
                new[] { new Vector2(0f, 0f), new Vector2(0.17f, 0f), new Vector2(0.22f, 0.42f), new Vector2(0.2f, 0.42f), new Vector2(0f, 0.38f) }, 24), p.Pot, PartKind.Static, 1f);
            var leaves = new LabMesh();
            var rng = new System.Random(3);
            for (int i = 0; i < 14; i++)
            {
                Quaternion r = Quaternion.Euler(-20f - (float)rng.NextDouble() * 45f, i * 26f, 0f);
                float len = 0.45f + (float)rng.NextDouble() * 0.4f;
                leaves.Chamfer(pot + Vector3.up * 0.4f + r * new Vector3(0f, len / 2f, 0f), new Vector3(0.09f, len, 0.012f), 0.004f, r);
            }

            LabParts.Part("Decor/Plant", group, leaves, p.Plant, PartKind.Static, 1f);
        }

        // ---- the mobile-robot test area and its motion capture ------------------------------

        /// <summary>
        /// The open floor between the workcell and the sign wall, taped out as a mobile-robot test area:
        /// an outline, a dashed centre line, a printed tag in each corner and a slalom of cones.
        /// </summary>
        private static void BuildArena(Transform room, LabPalette p)
        {
            Transform group = LabParts.Group("Arena", room).transform;
            float x0 = LabLayout.ArenaMinX, x1 = LabLayout.ArenaMaxX, z0 = LabLayout.ArenaMinZ, z1 = LabLayout.ArenaMaxZ;
            float cx = (x0 + x1) / 2f, cz = (z0 + z1) / 2f;
            const float tape = 0.05f, y = 0.0015f;
            var lines = new LabMesh();
            lines.Box(new Vector3(cx, y, z0), new Vector3(x1 - x0 + tape, 0.003f, tape));
            lines.Box(new Vector3(cx, y, z1), new Vector3(x1 - x0 + tape, 0.003f, tape));
            lines.Box(new Vector3(x0, y, cz), new Vector3(tape, 0.003f, z1 - z0 - tape));
            lines.Box(new Vector3(x1, y, cz), new Vector3(tape, 0.003f, z1 - z0 - tape));
            for (float x = x0 + 0.2f; x + 0.3f < x1; x += 0.35f)
            {
                lines.Box(new Vector3(x + 0.1f, y, cz), new Vector3(0.2f, 0.003f, 0.03f));
            }

            LabParts.Part("Arena/Tape", group, lines, p.Tape, PartKind.Static, 1f);

            // A printed tag in each corner, each a different cell of the atlas, top edge away from the operator.
            var tags = new LabMesh();
            int tag = 0;
            foreach (float tz in new[] { z0 + 0.32f, z1 - 0.32f })
            {
                foreach (float tx in new[] { x0 + 0.32f, x1 - 0.32f })
                {
                    tags.Screen(new Vector3(tx, 0.002f, tz), Vector3.up, Vector3.forward, new Vector2(0.3f, 0.3f),
                        new Rect(tag % 2 * 0.5f, tag / 2 * 0.5f, 0.5f, 0.5f));
                    tag++;
                }
            }

            LabParts.Part("Arena/Fiducials", group, tags, p.Fiducial, PartKind.Static, 2f);

            // A slalom of three cones, each with a reflective collar standing a hair proud of it.
            const float coneBase = 0.115f, coneTop = 0.022f, coneHeight = 0.3f, plinth = 0.025f;
            float CollarRadius(float along) => Mathf.Lerp(coneBase, coneTop, along / coneHeight) + 0.0015f;
            var cones = new LabMesh();
            var collars = new LabMesh();
            foreach (Vector2 c in new[] { new Vector2(cx - 1.0f, cz - 0.35f), new Vector2(cx, cz + 0.35f), new Vector2(cx + 1.0f, cz - 0.35f) })
            {
                Vector3 b = new Vector3(c.x, 0f, c.y);
                cones.Chamfer(b + Vector3.up * (plinth / 2f), new Vector3(0.25f, plinth, 0.25f), 0.006f);
                cones.Lathe(b + Vector3.up * plinth, Vector3.up, new[]
                {
                    new Vector2(0f, 0f), new Vector2(coneBase, 0f), new Vector2(coneTop, coneHeight), new Vector2(0f, coneHeight),
                }, 24);
                collars.Lathe(b + Vector3.up * plinth, Vector3.up, new[]
                {
                    new Vector2(CollarRadius(0.11f), 0.11f), new Vector2(CollarRadius(0.18f), 0.18f),
                }, 24);
            }

            LabParts.Part("Arena/Cones", group, cones, p.ConeOrange, PartKind.Static, 1f);
            LabParts.Part("Arena/ConeCollars", group, collars, p.Tape, PartKind.Static, 1f);

            // Painted on the floor in the tape's white at the near edge, lying flat and readable from the operator's position.
            LabParts.Label("ArenaLabel", group, new Vector3(cx, 0.003f, z0 + 0.18f), Quaternion.Euler(90f, 0f, 0f), "MOBILE TEST AREA", 1.6f,
                new Color(0.72f, 0.74f, 0.77f), new Vector2(1.6f, 0.2f), TextAlignmentOptions.Center, FontStyles.Bold);
        }

        /// <summary>
        /// Six motion-capture cameras on the walls and cable trays, aimed at the test area: the optical
        /// tracking a robotics lab takes ground truth from. Each has the faint red ring of its strobe.
        /// </summary>
        private static void BuildMotionCapture(Transform room, LabPalette p)
        {
            Transform group = LabParts.Group("MotionCapture", room).transform;
            var bodies = new LabMesh();
            var mounts = new LabMesh();
            var lenses = new LabMesh();
            var rings = new LabMesh();
            const float wallY = 2.92f;
            float trayY = LabLayout.Height - 0.25f;
            // Each camera, and where its mount meets the building.
            var cameras = new (Vector3 at, Vector3 anchor)[]
            {
                (new Vector3(-2.6f, wallY, LabLayout.MaxZ - 0.2f), new Vector3(-2.6f, wallY, LabLayout.MaxZ)),
                (new Vector3(2.6f, wallY, LabLayout.MaxZ - 0.2f), new Vector3(2.6f, wallY, LabLayout.MaxZ)),
                (new Vector3(-3.6f, trayY - 0.2f, 1.2f), new Vector3(-3.6f, trayY, 1.2f)),
                (new Vector3(3.6f, trayY - 0.2f, 1.2f), new Vector3(3.6f, trayY, 1.2f)),
                (new Vector3(-2.2f, wallY, LabLayout.MinZ + 0.2f), new Vector3(-2.2f, wallY, LabLayout.MinZ)),
                (new Vector3(2.2f, wallY, LabLayout.MinZ + 0.2f), new Vector3(2.2f, wallY, LabLayout.MinZ)),
            };
            foreach ((Vector3 at, Vector3 anchor) in cameras)
            {
                Vector3 look = (LabLayout.CaptureVolumeCentre - at).normalized;
                Quaternion aim = Quaternion.LookRotation(look, Vector3.up);
                bodies.Chamfer(at, new Vector3(0.09f, 0.075f, 0.12f), 0.01f, aim);
                Vector3 front = at + look * 0.06f;
                lenses.Cylinder(front + look * 0.004f, look, 0.019f, 0.008f, 20);
                rings.Torus(front + look * 0.002f, look, 0.031f, 0.0045f, 24, 6);
                // A rod from the wall or tray to a clamp behind the body.
                Vector3 clamp = at - look * 0.07f;
                mounts.Cylinder((anchor + clamp) / 2f, clamp - anchor, 0.008f, Vector3.Distance(anchor, clamp), 8);
                mounts.Cylinder(clamp, Vector3.up, 0.014f, 0.03f, 10, 0.003f);
            }

            LabParts.Part("MotionCapture/Bodies", group, bodies, p.Plastic, PartKind.Static, 1f);
            LabParts.Part("MotionCapture/Mounts", group, mounts, p.SteelDark, PartKind.Static, 1f);
            LabParts.Part("MotionCapture/Lenses", group, lenses, p.Glass, PartKind.Glow);
            LabParts.Part("MotionCapture/StrobeRings", group, rings, p.MocapRing, PartKind.Glow);
        }
    }
}
