using System.IO;
using Teleop.Bridge;
using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Editor
{
    /// <summary>
    /// <b>Teleop → Build Lab Scene</b>: writes <c>Assets/Scenes/JetRoverLab.unity</c>, a lab room
    /// with the JetRover's virtual twin on a bench and the robot's camera feed on a monitor beside it.
    ///
    /// <para><b>Why a builder and not a scene file.</b> Unity's own serializer should be the only thing
    /// writing scene YAML (unity/TeleopVR/Assets/Teleop/CLAUDE.md), so this script does what a human
    /// would do in the editor, through the editor's API, and the scene it saves is ordinary Unity
    /// output.</para>
    ///
    /// <para><b>It only ever creates; it never overwrites.</b> Once built, the scene is edited by hand
    /// in the editor, and those edits exist nowhere else: this script does not read them back. So it
    /// refuses to run while <c>JetRoverLab.unity</c> exists, and it leaves any material asset that is
    /// already in <c>Assets/Teleop/Lab/</c> exactly as it is. To start a fresh lab, rename or delete
    /// the scene first.</para>
    ///
    /// <para><b>What is reused, untouched.</b> The lab starts as a copy of
    /// <c>JetRoverControl.unity</c>, so the XR rig, the arm rig and its pivots, the drag target,
    /// <see cref="JetRoverOperatorBridge"/> and the HUD keep the wiring that already works there. None
    /// of those objects is moved, reparented or edited: the room is built around the arm's base where
    /// that scene already puts it. Everything added is either set dressing with no behaviour, or one
    /// <see cref="CameraFeedBridge"/> on the operator object, whose screen is a quad on the monitor.</para>
    ///
    /// <para><b>Quest budget.</b> Primitives only, colliders removed (nothing here should catch an XR
    /// ray meant for the drag target), the room static-batched, one directional light as in the source
    /// scene, Trilight ambient, no baked lighting. The room shell casts no shadows, or the ceiling
    /// would shadow the whole room from the directional light.</para>
    /// </summary>
    public static class LabSceneBuilder
    {
        private const string SourceScenePath = "Assets/Scenes/JetRoverControl.unity";
        private const string LabScenePath = "Assets/Scenes/JetRoverLab.unity";
        private const string AssetRoot = "Assets/Teleop/Lab";
        private const string MaterialFolder = AssetRoot + "/Materials";
        private const string TextureFolder = AssetRoot + "/Textures";

        // The camera stream is 640x480 by default (Teleop.CameraHost serve); CameraFeedBridge resizes the
        // screen to the real aspect on the first frame, and the bezel is a child of the screen so it follows.
        private const float ScreenHeight = 0.36f;
        private const float ScreenAspect = 640f / 480f;

        // Where the operator's head is, roughly, for turning the monitor to face it. Yaw only, so the
        // height does not matter.
        private static readonly Vector3 OperatorHead = new Vector3(0f, 1.5f, 0f);

        private sealed class Palette
        {
            public Material Floor, Wall, Ceiling, LightPanel, BenchTop, Aluminium, Chassis, Rubber, Accent,
                Lidar, Plastic, Screen, SafetyTape, Whiteboard, ShelfSteel, Cardboard, BinBlue, BinGrey,
                Extinguisher, Door;
        }

        [MenuItem("Teleop/Build Lab Scene")]
        public static void Build()
        {
            if (!Application.isBatchMode && !EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                return;
            }

            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(SourceScenePath) == null)
            {
                Debug.LogError($"[lab] {SourceScenePath} not found; the lab scene is built from a copy of it.");
                return;
            }

            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(LabScenePath) != null)
            {
                string message =
                    $"{LabScenePath} already exists and is edited by hand in the editor now; the builder never " +
                    "overwrites it, because those edits exist nowhere else. To start a fresh lab, rename or " +
                    "delete the scene first.";
                Debug.LogWarning($"[lab] {message}");
                if (!Application.isBatchMode)
                {
                    EditorUtility.DisplayDialog("Lab scene already exists", message, "OK");
                }

                return;
            }

            if (!AssetDatabase.CopyAsset(SourceScenePath, LabScenePath))
            {
                Debug.LogError($"[lab] Could not copy {SourceScenePath} to {LabScenePath}.");
                return;
            }

            Scene scene = EditorSceneManager.OpenScene(LabScenePath, OpenSceneMode.Single);

            JetRoverArmRig rig = Object.FindObjectOfType<JetRoverArmRig>();
            JetRoverOperatorBridge operatorBridge = Object.FindObjectOfType<JetRoverOperatorBridge>();
            Transform baseAnchor = rig != null
                ? new SerializedObject(rig).FindProperty("baseAnchor").objectReferenceValue as Transform
                : null;
            if (rig == null || operatorBridge == null || baseAnchor == null)
            {
                Debug.LogError(
                    $"[lab] {SourceScenePath} must contain a JetRoverArmRig with its Base Anchor assigned and a " +
                    "JetRoverOperatorBridge; the lab is placed around the arm's base and the camera feed goes on the operator.");
                return;
            }

            Palette palette = CreateMaterials();
            Vector3 armBase = baseAnchor.position;

            var lab = new GameObject("Lab").transform;
            float benchTop = BuildTwinChassis(lab, palette, armBase);
            BuildBench(lab, palette, armBase, benchTop);
            BuildRoom(lab, palette);
            BuildProps(lab, palette);
            MarkStatic(lab.gameObject);

            // Not static: CameraFeedBridge resizes the screen on the first frame.
            Renderer screen = BuildMonitor(palette, armBase, benchTop);
            WireCameraFeed(operatorBridge.gameObject, screen);

            ConfigureLighting();

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            Debug.Log($"[lab] Built {LabScenePath}: twin on the bench at {armBase}, camera monitor to its right.");
        }

        // ---- the twin's base -------------------------------------------------------------------------

        /// <summary>
        /// A chassis under the arm's base, so the arm rig reads as the JetRover rather than a floating
        /// arm. Purely visual and never driven: it is not a child of any pivot, and the arm rig keeps
        /// its own transforms. Returns the height of the bench top the wheels stand on.
        /// </summary>
        private static float BuildTwinChassis(Transform lab, Palette p, Vector3 armBase)
        {
            var root = new GameObject("JetRoverChassis").transform;
            root.SetParent(lab, false);

            const float bodyWidth = 0.20f, bodyHeight = 0.075f, bodyLength = 0.28f;
            const float wheelRadius = 0.045f, wheelWidth = 0.04f;

            float bodyTop = armBase.y;
            float bodyCentreY = bodyTop - bodyHeight / 2f;
            float bodyCentreZ = armBase.z - 0.03f; // the arm sits a little forward of centre
            var bodyCentre = new Vector3(armBase.x, bodyCentreY, bodyCentreZ);

            Box("Body", root, bodyCentre, new Vector3(bodyWidth, bodyHeight, bodyLength), p.Chassis);
            Box("TopDeck", root, new Vector3(armBase.x, bodyTop + 0.002f, bodyCentreZ),
                new Vector3(bodyWidth + 0.01f, 0.008f, bodyLength + 0.01f), p.Accent);
            Box("FrontBumper", root, new Vector3(armBase.x, bodyCentreY - 0.01f, bodyCentreZ + bodyLength / 2f + 0.01f),
                new Vector3(bodyWidth * 0.9f, 0.03f, 0.02f), p.Rubber);
            Cylinder("Turntable", root, new Vector3(armBase.x, bodyTop + 0.008f, armBase.z), 0.075f, 0.016f, p.Aluminium);
            Cylinder("Lidar", root, new Vector3(armBase.x, bodyTop + 0.02f, bodyCentreZ - 0.095f), 0.07f, 0.04f, p.Lidar);

            float wheelCentreY = bodyCentreY - bodyHeight / 2f + 0.015f;
            for (int side = -1; side <= 1; side += 2)
            {
                for (int end = -1; end <= 1; end += 2)
                {
                    var centre = new Vector3(
                        armBase.x + side * (bodyWidth / 2f + wheelWidth / 2f + 0.004f),
                        wheelCentreY,
                        bodyCentreZ + end * (bodyLength / 2f - wheelRadius - 0.01f));
                    Transform wheel = Cylinder($"Wheel{(side < 0 ? "L" : "R")}{(end < 0 ? "Rear" : "Front")}",
                        root, centre, wheelRadius * 2f, wheelWidth, p.Rubber);
                    wheel.rotation = Quaternion.Euler(0f, 0f, 90f); // axle along x
                    Cylinder("Hub", root, centre + new Vector3(side * 0.003f, 0f, 0f), wheelRadius, wheelWidth + 0.004f, p.Aluminium)
                        .rotation = wheel.rotation;
                }
            }

            return wheelCentreY - wheelRadius;
        }

        // ---- bench, room, props ----------------------------------------------------------------------

        private static void BuildBench(Transform lab, Palette p, Vector3 armBase, float top)
        {
            var root = new GameObject("Bench").transform;
            root.SetParent(lab, false);

            // The near edge sits just in front of the chassis, so the operator stands at the bench with
            // the robot an arm's length away, where JetRoverControl already put it.
            const float width = 1.9f, depth = 0.8f, thickness = 0.04f;
            float nearEdge = armBase.z - 0.19f;
            var topCentre = new Vector3(armBase.x + 0.26f, top - thickness / 2f, nearEdge + depth / 2f);

            Box("Top", root, topCentre, new Vector3(width, thickness, depth), p.BenchTop);
            float legHeight = top - thickness;
            for (int sx = -1; sx <= 1; sx += 2)
            {
                for (int sz = -1; sz <= 1; sz += 2)
                {
                    Box("Leg", root,
                        new Vector3(topCentre.x + sx * (width / 2f - 0.06f), legHeight / 2f, topCentre.z + sz * (depth / 2f - 0.06f)),
                        new Vector3(0.05f, legHeight, 0.05f), p.Aluminium);
                }
            }

            Box("LowerShelf", root, new Vector3(topCentre.x, 0.18f, topCentre.z), new Vector3(width - 0.1f, 0.025f, depth - 0.1f), p.Aluminium);
            Box("Toolbox", root, new Vector3(topCentre.x + 0.55f, 0.18f + 0.0125f + 0.09f, topCentre.z), new Vector3(0.45f, 0.18f, 0.22f), p.Extinguisher);

            // A taped-off workcell around the bench, as round a real robot cell.
            const float margin = 0.45f, tape = 0.05f;
            float halfX = width / 2f + margin, halfZ = depth / 2f + margin;
            Box("TapeFront", root, new Vector3(topCentre.x, 0.002f, topCentre.z - halfZ), new Vector3(2f * halfX + tape, 0.002f, tape), p.SafetyTape);
            Box("TapeBack", root, new Vector3(topCentre.x, 0.002f, topCentre.z + halfZ), new Vector3(2f * halfX + tape, 0.002f, tape), p.SafetyTape);
            Box("TapeLeft", root, new Vector3(topCentre.x - halfX, 0.002f, topCentre.z), new Vector3(tape, 0.002f, 2f * halfZ), p.SafetyTape);
            Box("TapeRight", root, new Vector3(topCentre.x + halfX, 0.002f, topCentre.z), new Vector3(tape, 0.002f, 2f * halfZ), p.SafetyTape);
        }

        private static void BuildRoom(Transform lab, Palette p)
        {
            var root = new GameObject("Room").transform;
            root.SetParent(lab, false);

            // 8 m x 8 m x 3 m, with more of it in front of the operator than behind.
            const float minX = -4f, maxX = 4f, minZ = -3f, maxZ = 5f, height = 3f, wall = 0.1f;
            float cx = (minX + maxX) / 2f, cz = (minZ + maxZ) / 2f, sx = maxX - minX, sz = maxZ - minZ;

            Shell(Box("Floor", root, new Vector3(cx, -0.01f, cz), new Vector3(sx, 0.02f, sz), p.Floor));
            Shell(Box("Ceiling", root, new Vector3(cx, height + wall / 2f, cz), new Vector3(sx, wall, sz), p.Ceiling));
            Shell(Box("WallBack", root, new Vector3(cx, height / 2f, maxZ + wall / 2f), new Vector3(sx, height, wall), p.Wall));
            Shell(Box("WallFront", root, new Vector3(cx, height / 2f, minZ - wall / 2f), new Vector3(sx, height, wall), p.Wall));
            Shell(Box("WallLeft", root, new Vector3(minX - wall / 2f, height / 2f, cz), new Vector3(wall, height, sz), p.Wall));
            Shell(Box("WallRight", root, new Vector3(maxX + wall / 2f, height / 2f, cz), new Vector3(wall, height, sz), p.Wall));

            // Floor tile seams every metre, and a skirting line, so depth and scale read in the headset.
            for (float x = minX + 1f; x < maxX; x += 1f)
            {
                Shell(Box("SeamX", root, new Vector3(x, 0.0015f, cz), new Vector3(0.008f, 0.001f, sz), p.ShelfSteel));
            }

            for (float z = minZ + 1f; z < maxZ; z += 1f)
            {
                Shell(Box("SeamZ", root, new Vector3(cx, 0.0015f, z), new Vector3(sx, 0.001f, 0.008f), p.ShelfSteel));
            }

            // Ceiling light panels: emissive only, no light sources (the directional light does the work).
            for (int ix = -1; ix <= 1; ix++)
            {
                for (int iz = 0; iz <= 2; iz++)
                {
                    Shell(Box("LightPanel", root, new Vector3(ix * 2.2f, height - 0.005f, -0.5f + iz * 2.2f), new Vector3(1.2f, 0.01f, 0.6f), p.LightPanel));
                }
            }

            // Door on the right wall.
            Shell(Box("Door", root, new Vector3(maxX - 0.02f, 1.05f, -1.6f), new Vector3(0.04f, 2.1f, 0.95f), p.Door));
            Shell(Box("DoorHandle", root, new Vector3(maxX - 0.05f, 1.0f, -1.95f), new Vector3(0.03f, 0.03f, 0.14f), p.Aluminium));
        }

        private static void BuildProps(Transform lab, Palette p)
        {
            var root = new GameObject("Props").transform;
            root.SetParent(lab, false);

            // Whiteboard on the back wall, with the lab's name.
            Box("WhiteboardFrame", root, new Vector3(0.2f, 1.55f, 4.975f), new Vector3(2.5f, 1.3f, 0.03f), p.Aluminium);
            Box("Whiteboard", root, new Vector3(0.2f, 1.55f, 4.955f), new Vector3(2.4f, 1.2f, 0.02f), p.Whiteboard);
            Label("WhiteboardTitle", root, new Vector3(0.2f, 1.95f, 4.94f), Quaternion.identity,
                "JetRover Teleoperation Lab", 1.4f, new Color(0.13f, 0.2f, 0.45f), new Vector2(2.3f, 0.25f));
            Label("WhiteboardNote", root, new Vector3(0.2f, 1.55f, 4.94f), Quaternion.identity,
                "camera latency = capture -> send -> network -> decode -> render", 0.65f, new Color(0.15f, 0.15f, 0.15f), new Vector2(2.3f, 0.2f));

            // Shelving along the left wall, with parts bins.
            const float shelfX = -3.7f, shelfZ = 1.8f, shelfWidth = 2.0f, shelfDepth = 0.45f;
            for (int i = 0; i < 4; i++)
            {
                float y = 0.15f + i * 0.5f;
                Box("Shelf", root, new Vector3(shelfX, y, shelfZ), new Vector3(shelfDepth, 0.025f, shelfWidth), p.ShelfSteel);
                for (int b = 0; b < 4; b++)
                {
                    Material bin = (b + i) % 3 == 0 ? p.BinBlue : (b + i) % 3 == 1 ? p.BinGrey : p.Cardboard;
                    float h = 0.18f + 0.06f * ((b * 7 + i * 3) % 3);
                    Box("Bin", root, new Vector3(shelfX, y + 0.0125f + h / 2f, shelfZ - 0.7f + b * 0.47f), new Vector3(0.38f, h, 0.36f), bin);
                }
            }

            for (int sz = -1; sz <= 1; sz += 2)
            {
                for (int sx = -1; sx <= 1; sx += 2)
                {
                    Box("Upright", root, new Vector3(shelfX + sx * (shelfDepth / 2f - 0.015f), 0.95f, shelfZ + sz * (shelfWidth / 2f - 0.015f)),
                        new Vector3(0.03f, 1.9f, 0.03f), p.ShelfSteel);
                }
            }

            // A second desk on the right with a workstation, facing the right wall.
            var deskTop = new Vector3(3.3f, 0.74f, 2.4f);
            Box("DeskTop", root, deskTop, new Vector3(0.8f, 0.03f, 1.6f), p.BenchTop);
            for (int sx = -1; sx <= 1; sx += 2)
            {
                for (int sz = -1; sz <= 1; sz += 2)
                {
                    Box("DeskLeg", root, new Vector3(deskTop.x + sx * 0.35f, 0.3625f, deskTop.z + sz * 0.75f), new Vector3(0.04f, 0.725f, 0.04f), p.Aluminium);
                }
            }

            Box("WorkstationMonitor", root, deskTop + new Vector3(0.25f, 0.3f, 0f), new Vector3(0.03f, 0.34f, 0.58f), p.Plastic);
            Box("WorkstationStand", root, deskTop + new Vector3(0.28f, 0.08f, 0f), new Vector3(0.12f, 0.13f, 0.06f), p.Plastic);
            Box("Keyboard", root, deskTop + new Vector3(-0.1f, 0.025f, 0f), new Vector3(0.15f, 0.02f, 0.44f), p.Plastic);
            Box("Tower", root, new Vector3(deskTop.x + 0.1f, 0.23f, deskTop.z + 0.55f), new Vector3(0.45f, 0.45f, 0.2f), p.Plastic);

            // Fire extinguisher by the door.
            Cylinder("Extinguisher", root, new Vector3(3.85f, 0.3f, -0.8f), 0.15f, 0.55f, p.Extinguisher);
        }

        // ---- the camera monitor ----------------------------------------------------------------------

        /// <summary>
        /// A monitor on the bench to the right of the robot, turned to face the operator. World-locked,
        /// never parented to the camera (ADR 0014 §7). Returns the screen quad's renderer.
        /// </summary>
        private static Renderer BuildMonitor(Palette p, Vector3 armBase, float benchTop)
        {
            const float standHeight = 0.12f, bezel = 0.02f, depth = 0.03f;
            float width = ScreenHeight * ScreenAspect;

            var root = new GameObject("CameraMonitor").transform;
            root.position = new Vector3(armBase.x + 0.64f, benchTop, armBase.z + 0.3f);
            Vector3 away = root.position - OperatorHead;
            away.y = 0f;
            root.rotation = Quaternion.LookRotation(away.normalized, Vector3.up);

            Transform stand = Box("StandBase", root, Vector3.zero, new Vector3(0.22f, 0.012f, 0.16f), p.Plastic);
            stand.localPosition = new Vector3(0f, 0.006f, 0.03f);
            Transform neck = Box("StandNeck", root, Vector3.zero, new Vector3(0.05f, standHeight + ScreenHeight / 2f, 0.025f), p.Plastic);
            neck.localPosition = new Vector3(0f, (standHeight + ScreenHeight / 2f) / 2f, depth + 0.02f);

            // The screen: a Quad, which faces -z, i.e. toward the operator once the root is turned away.
            GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
            quad.name = "CameraScreen";
            RemoveCollider(quad);
            quad.transform.SetParent(root, false);
            quad.transform.localPosition = new Vector3(0f, standHeight + ScreenHeight / 2f, 0f);
            quad.transform.localScale = new Vector3(width, ScreenHeight, 1f);
            var screen = quad.GetComponent<Renderer>();
            screen.sharedMaterial = p.Screen;
            screen.shadowCastingMode = ShadowCastingMode.Off;
            screen.receiveShadows = false;

            // The bezel is a child of the screen, in the screen's units, so it follows the aspect
            // CameraFeedBridge sets on the first frame. Behind the quad by enough to avoid z-fighting.
            Transform frame = Box("Bezel", quad.transform, Vector3.zero, Vector3.one, p.Plastic);
            frame.localPosition = new Vector3(0f, 0f, depth / 2f + 0.003f);
            frame.localScale = new Vector3(1f + 2f * bezel / width, 1f + 2f * bezel / ScreenHeight, depth);

            Label("ScreenTitle", root, new Vector3(0f, standHeight + ScreenHeight + bezel + 0.03f, -0.005f), Quaternion.identity,
                "JetRover camera", 0.32f, new Color(0.9f, 0.92f, 0.95f), new Vector2(width, 0.05f), local: true);

            return screen;
        }

        private static void WireCameraFeed(GameObject host, Renderer screen)
        {
            CameraFeedBridge feed = host.GetComponent<CameraFeedBridge>();
            if (feed == null)
            {
                feed = host.AddComponent<CameraFeedBridge>();
            }

            var so = new SerializedObject(feed);
            so.FindProperty("panel").objectReferenceValue = screen;
            so.FindProperty("panelHeightMeters").floatValue = ScreenHeight;
            // Left unassigned on purpose: with an arm rig, CameraFeedBridge moves the screen beside the
            // arm's base at start, which would pull it out of the monitor built around it here.
            so.FindProperty("armRig").objectReferenceValue = null;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        // ---- lighting and materials ------------------------------------------------------------------

        private static void ConfigureLighting()
        {
            // Indoors: no sky. Trilight ambient fills what the one directional light does not reach.
            RenderSettings.skybox = null;
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = new Color(0.78f, 0.8f, 0.84f);
            RenderSettings.ambientEquatorColor = new Color(0.56f, 0.57f, 0.6f);
            RenderSettings.ambientGroundColor = new Color(0.3f, 0.3f, 0.32f);
            RenderSettings.reflectionIntensity = 0.4f;
            RenderSettings.fog = false;

            foreach (Light light in Object.FindObjectsOfType<Light>())
            {
                if (light.type == LightType.Directional)
                {
                    light.transform.rotation = Quaternion.Euler(62f, -25f, 0f);
                    light.intensity = 0.9f;
                    light.color = new Color(1f, 0.97f, 0.93f);
                }
            }

            foreach (Camera camera in Object.FindObjectsOfType<Camera>())
            {
                camera.backgroundColor = new Color(0.1f, 0.1f, 0.12f);
            }
        }

        private static Palette CreateMaterials()
        {
            EnsureFolder(AssetRoot);
            EnsureFolder(MaterialFolder);
            EnsureFolder(TextureFolder);

            var p = new Palette
            {
                Floor = Standard("LabFloor", new Color(0.62f, 0.64f, 0.66f), 0.35f),
                Wall = Standard("LabWall", new Color(0.88f, 0.88f, 0.86f), 0.05f),
                Ceiling = Standard("LabCeiling", new Color(0.93f, 0.93f, 0.93f), 0.0f),
                LightPanel = Standard("LabLightPanel", Color.white, 0.0f, emission: new Color(1.2f, 1.2f, 1.15f)),
                BenchTop = Standard("BenchTop", new Color(0.16f, 0.17f, 0.19f), 0.45f),
                Aluminium = Standard("Aluminium", new Color(0.74f, 0.75f, 0.77f), 0.55f, metallic: 0.8f),
                Chassis = Standard("TwinChassis", new Color(0.12f, 0.12f, 0.13f), 0.35f, metallic: 0.3f),
                Rubber = Standard("TwinRubber", new Color(0.05f, 0.05f, 0.05f), 0.1f),
                Accent = Standard("TwinAccent", new Color(0.12f, 0.42f, 0.85f), 0.5f),
                Lidar = Standard("TwinLidar", new Color(0.2f, 0.2f, 0.22f), 0.7f, metallic: 0.5f),
                Plastic = Standard("MonitorPlastic", new Color(0.07f, 0.07f, 0.08f), 0.4f),
                SafetyTape = Standard("SafetyTape", new Color(0.98f, 0.8f, 0.05f), 0.3f),
                Whiteboard = Standard("Whiteboard", new Color(0.97f, 0.97f, 0.97f), 0.6f),
                ShelfSteel = Standard("ShelfSteel", new Color(0.35f, 0.37f, 0.4f), 0.4f, metallic: 0.6f),
                Cardboard = Standard("Cardboard", new Color(0.66f, 0.5f, 0.33f), 0.05f),
                BinBlue = Standard("BinBlue", new Color(0.13f, 0.3f, 0.65f), 0.3f),
                BinGrey = Standard("BinGrey", new Color(0.45f, 0.46f, 0.48f), 0.3f),
                Extinguisher = Standard("SafetyRed", new Color(0.75f, 0.08f, 0.06f), 0.55f),
                Door = Standard("Door", new Color(0.4f, 0.3f, 0.22f), 0.3f),
            };

            // The screen is unlit: the camera image should look like the camera, not like a lit surface.
            p.Screen = LoadOrCreate("CameraScreen", Shader.Find("Unlit/Texture"), out bool created);
            if (created)
            {
                p.Screen.mainTexture = NoSignalTexture();
                EditorUtility.SetDirty(p.Screen);
            }

            AssetDatabase.SaveAssets();
            return p;
        }

        /// <summary>A Standard material, configured only when it is first created; an existing one is left as edited.</summary>
        private static Material Standard(string name, Color color, float smoothness, float metallic = 0f, Color? emission = null)
        {
            Material m = LoadOrCreate(name, Shader.Find("Standard"), out bool created);
            if (!created)
            {
                return m;
            }

            m.color = color;
            m.SetFloat("_Glossiness", smoothness);
            m.SetFloat("_Metallic", metallic);
            if (emission.HasValue)
            {
                m.EnableKeyword("_EMISSION");
                m.SetColor("_EmissionColor", emission.Value);
                m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.None;
            }
            else
            {
                m.DisableKeyword("_EMISSION");
            }

            EditorUtility.SetDirty(m);
            return m;
        }

        private static Material LoadOrCreate(string name, Shader shader, out bool created)
        {
            string path = $"{MaterialFolder}/{name}.mat";
            Material m = AssetDatabase.LoadAssetAtPath<Material>(path);
            created = m == null;
            if (created)
            {
                m = new Material(shader);
                AssetDatabase.CreateAsset(m, path);
            }

            return m;
        }

        /// <summary>
        /// Colour bars, shown on the screen until the first camera frame replaces them, so "no feed yet"
        /// is visibly different from "feed is black". Written once as a PNG asset.
        /// </summary>
        private static Texture2D NoSignalTexture()
        {
            string path = $"{TextureFolder}/NoSignal.png";
            var existing = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
            if (existing != null)
            {
                return existing;
            }

            const int w = 160, h = 120;
            Color[] bars =
            {
                new Color(0.75f, 0.75f, 0.75f), new Color(0.75f, 0.75f, 0f), new Color(0f, 0.75f, 0.75f),
                new Color(0f, 0.75f, 0f), new Color(0.75f, 0f, 0.75f), new Color(0.75f, 0f, 0f), new Color(0f, 0f, 0.75f),
            };
            var texture = new Texture2D(w, h, TextureFormat.RGB24, false);
            var pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    // Bars over the top two thirds (texture rows run bottom-up), a dark band below.
                    pixels[y * w + x] = y >= h / 3 ? bars[x * bars.Length / w] : new Color(0.08f, 0.08f, 0.1f);
                }
            }

            texture.SetPixels(pixels);
            texture.Apply();
            File.WriteAllBytes(path, texture.EncodeToPNG());
            Object.DestroyImmediate(texture);

            AssetDatabase.ImportAsset(path);
            if (AssetImporter.GetAtPath(path) is TextureImporter importer)
            {
                importer.mipmapEnabled = false;
                importer.wrapMode = TextureWrapMode.Clamp;
                importer.filterMode = FilterMode.Point;
                importer.npotScale = TextureImporterNPOTScale.None;
                importer.SaveAndReimport();
            }

            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        // ---- primitives ------------------------------------------------------------------------------

        private static Transform Box(string name, Transform parent, Vector3 worldCentre, Vector3 size, Material material)
        {
            return Primitive(PrimitiveType.Cube, name, parent, worldCentre, size, material);
        }

        /// <summary>A cylinder standing on y, <paramref name="diameter"/> across and <paramref name="height"/> tall.</summary>
        private static Transform Cylinder(string name, Transform parent, Vector3 worldCentre, float diameter, float height, Material material)
        {
            // Unity's cylinder is 1 across and 2 tall.
            return Primitive(PrimitiveType.Cylinder, name, parent, worldCentre, new Vector3(diameter, height / 2f, diameter), material);
        }

        private static Transform Primitive(PrimitiveType type, string name, Transform parent, Vector3 worldCentre, Vector3 scale, Material material)
        {
            GameObject go = GameObject.CreatePrimitive(type);
            go.name = name;
            RemoveCollider(go);
            go.transform.SetParent(parent, false);
            go.transform.position = worldCentre;
            go.transform.localScale = scale;
            go.GetComponent<Renderer>().sharedMaterial = material;
            return go.transform;
        }

        private static void Label(string name, Transform parent, Vector3 position, Quaternion rotation, string text,
            float fontSize, Color color, Vector2 size, bool local = false)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var tmp = go.AddComponent<TextMeshPro>();
            if (tmp.font == null)
            {
                // TMP Essentials not imported: skip the label rather than leave a broken one.
                Object.DestroyImmediate(go);
                return;
            }

            if (local)
            {
                go.transform.localPosition = position;
                go.transform.localRotation = rotation;
            }
            else
            {
                go.transform.position = position;
                go.transform.rotation = rotation;
            }

            tmp.text = text;
            tmp.fontSize = fontSize; // a 3D TextMeshPro's line is about fontSize / 10 metres
            tmp.color = color;
            tmp.alignment = TextAlignmentOptions.Center;
            tmp.enableWordWrapping = false;
            tmp.rectTransform.sizeDelta = size;
            go.GetComponent<MeshRenderer>().shadowCastingMode = ShadowCastingMode.Off;
        }

        private static void RemoveCollider(GameObject go)
        {
            Collider collider = go.GetComponent<Collider>();
            if (collider != null)
            {
                Object.DestroyImmediate(collider);
            }
        }

        /// <summary>The room's shell: lit, but casting no shadows, or the ceiling would shadow the room.</summary>
        private static void Shell(Transform t)
        {
            Renderer r = t.GetComponent<Renderer>();
            r.shadowCastingMode = ShadowCastingMode.Off;
        }

        private static void MarkStatic(GameObject root)
        {
            foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
            {
                if (t.GetComponent<TMP_Text>() != null)
                {
                    continue; // TextMeshPro rebuilds its own mesh; static batching would bake a stale one
                }

                GameObjectUtility.SetStaticEditorFlags(t.gameObject, StaticEditorFlags.BatchingStatic);
            }
        }

        private static void EnsureFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path))
            {
                return;
            }

            string parent = Path.GetDirectoryName(path).Replace('\\', '/');
            EnsureFolder(parent);
            AssetDatabase.CreateFolder(parent, Path.GetFileName(path));
        }
    }
}
