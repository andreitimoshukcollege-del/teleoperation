using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// The lab's lighting: baked, because a headset that renders two very high-resolution eyes cannot
    /// afford a room of real-time lights.
    /// <list type="bullet">
    /// <item><b>Baked:</b> rectangle area lights under every ceiling fixture and over the bench (soft
    /// pools, a dark periphery); a cyan wash from the cove lines; a cold glow from the window. Bounced
    /// light and ambient occlusion are baked in too.</item>
    /// <item><b>Mixed, Subtractive:</b> the one directional light. It casts the moving arm's real-time
    /// shadow onto the baked bench, which tells the operator how high the gripper is.</item>
    /// <item><b>Light probes:</b> a dense grid where the arm moves, a sparse one in the room. The arm and
    /// the target are not in the lightmaps, so this is how they get the room's light.</item>
    /// <item><b>One box-projected reflection probe:</b> what the glossy floor and the metal reflect.</item>
    /// </list>
    /// </summary>
    internal static class LabLighting
    {
        public const string SettingsPath = LabPaths.Generated + "/JetRoverLabXR.lighting";

        public static void Build(Transform parent, Scene scene, Light keyLight)
        {
            Transform root = LabParts.Group("Lighting", parent).transform;

            if (keyLight != null)
            {
                keyLight.lightmapBakeType = LightmapBakeType.Mixed;
                keyLight.intensity = 1.1f;
                keyLight.color = new Color(1f, 0.97f, 0.93f);
                keyLight.shadows = LightShadows.Soft;
                keyLight.shadowStrength = 0.9f;
                keyLight.transform.rotation = Quaternion.Euler(66f, -32f, 0f);
                EditorUtility.SetDirty(keyLight);
            }

            // The pendants light down onto the floor and, more faintly, up onto the ceiling, so the
            // ceiling reads as a surface instead of a void (and the black robot has something to reflect).
            var warmWhite = new Color(1f, 0.95f, 0.88f);
            foreach (Vector3 fixture in LabRoom.FixturePositions)
            {
                Area(root, "FixtureLight", fixture + Vector3.down * 0.04f, Quaternion.Euler(90f, 0f, 0f), new Vector2(1.4f, 0.12f), warmWhite, 70f);
                Area(root, "FixtureUplight", fixture + Vector3.up * 0.04f, Quaternion.Euler(-90f, 0f, 0f), new Vector2(1.4f, 0.12f), warmWhite, 8f);
            }

            Area(root, "TaskLight", LabRoom.TaskFixture + Vector3.down * 0.04f, Quaternion.Euler(90f, 0f, 0f), new Vector2(1.1f, 0.12f), new Color(1f, 0.97f, 0.92f), 80f);
            Area(root, "TaskUplight", LabRoom.TaskFixture + Vector3.up * 0.04f, Quaternion.Euler(-90f, 0f, 0f), new Vector2(1.1f, 0.12f), warmWhite, 10f);

            // Cyan cove wash down the walls, and up from the skirting.
            var cyan = new Color(0.15f, 0.8f, 1f);
            float h = LabLayout.Height, sx = LabLayout.MaxX - LabLayout.MinX, sz = LabLayout.MaxZ - LabLayout.MinZ;
            float cx = (LabLayout.MinX + LabLayout.MaxX) / 2f, cz = (LabLayout.MinZ + LabLayout.MaxZ) / 2f;
            Area(root, "CoveBack", new Vector3(cx, h - 0.08f, LabLayout.MaxZ - 0.08f), Quaternion.Euler(90f, 0f, 0f), new Vector2(sx - 0.2f, 0.05f), cyan, 8f);
            Area(root, "CoveRight", new Vector3(LabLayout.MaxX - 0.08f, h - 0.08f, cz), Quaternion.Euler(90f, 90f, 0f), new Vector2(sz - 0.2f, 0.05f), cyan, 8f);
            Area(root, "CoveFront", new Vector3(cx, h - 0.08f, LabLayout.MinZ + 0.08f), Quaternion.Euler(90f, 0f, 0f), new Vector2(sx - 0.2f, 0.05f), cyan, 6f);
            Area(root, "SkirtingBack", new Vector3(cx, 0.14f, LabLayout.MaxZ - 0.06f), Quaternion.Euler(-90f, 0f, 0f), new Vector2(sx - 0.2f, 0.04f), cyan, 5f);
            Area(root, "SkirtingRight", new Vector3(LabLayout.MaxX - 0.06f, 0.14f, cz), Quaternion.Euler(-90f, 90f, 0f), new Vector2(sz - 0.2f, 0.04f), cyan, 5f);
            Area(root, "SignGlow", new Vector3(0f, 2.15f, LabLayout.MaxZ - 0.12f), Quaternion.Euler(0f, 180f, 0f), new Vector2(3f, 1.1f), new Color(0.2f, 0.75f, 1f), 3f);

            // The window: the city's glow, cold and dim.
            Area(root, "WindowGlow", new Vector3(LabLayout.MinX + 0.05f, (LabLayout.WindowBottom + LabLayout.WindowTop) / 2f, (LabLayout.WindowMinZ + LabLayout.WindowMaxZ) / 2f),
                Quaternion.Euler(0f, 90f, 0f), new Vector2(LabLayout.WindowMaxZ - LabLayout.WindowMinZ, LabLayout.WindowTop - LabLayout.WindowBottom), new Color(0.4f, 0.42f, 0.75f), 2.5f);

            BuildProbes(root);
            BuildReflectionProbe(root);
            ApplySceneSettings(scene);
        }

        /// <summary>
        /// A baked rectangle light. The intensities above look large, and they have to be: the
        /// lightmapper treats area lights physically, so a 1.4 × 0.12 m fixture 2.75 m above the floor
        /// delivers only about intensity × 0.02 of illuminance, and a dark floor divides that by π again.
        /// They were tuned against the preview captures (`just unity-lab-xr`), not derived.
        /// </summary>
        private static void Area(Transform root, string name, Vector3 position, Quaternion rotation, Vector2 size, Color color, float intensity)
        {
            var go = new GameObject(name);
            go.transform.SetParent(root, false);
            go.transform.SetPositionAndRotation(position, rotation);
            var light = go.AddComponent<Light>();
            light.type = LightType.Rectangle;
            light.lightmapBakeType = LightmapBakeType.Baked;
            light.areaSize = size;
            light.color = color;
            light.intensity = intensity;
            light.shadows = LightShadows.Soft;
        }

        private static void BuildProbes(Transform root)
        {
            var positions = new List<Vector3>();
            Vector3 b = LabLayout.ArmBase;

            // Where the arm moves: 7 × 7 × 4 around the base, 15 cm apart.
            for (int ix = -3; ix <= 3; ix++)
                for (int iz = -3; iz <= 3; iz++)
                    foreach (float dy in new[] { 0.06f, 0.2f, 0.34f, 0.48f })
                        positions.Add(b + new Vector3(ix * 0.15f, dy, iz * 0.15f));

            // Just above the bench, around the chassis, for reaches down toward the mat.
            float benchY = LabLayout.BenchTop + 0.05f;
            for (int ix = -3; ix <= 3; ix++)
                for (int iz = -2; iz <= 3; iz++)
                {
                    var q = new Vector3(b.x + ix * 0.2f, benchY, LabLayout.BenchNearZ + 0.05f + iz * 0.15f);
                    if (Mathf.Abs(q.x - b.x) < 0.16f && q.z > b.z - 0.23f && q.z < b.z + 0.13f) continue; // inside the chassis
                    positions.Add(q);
                }

            // The room, every ~1.4 m at three heights, skipping solid furniture.
            for (float x = LabLayout.MinX + 0.7f; x < LabLayout.MaxX; x += 1.4f)
                for (float z = LabLayout.MinZ + 0.6f; z < LabLayout.MaxZ; z += 1.4f)
                    foreach (float y in new[] { 0.4f, 1.4f, 2.5f })
                    {
                        var q = new Vector3(x, y, z);
                        if (InsideFurniture(q)) continue;
                        positions.Add(q);
                    }

            var go = new GameObject("LightProbes");
            go.transform.SetParent(root, false);
            var group = go.AddComponent<LightProbeGroup>();
            group.probePositions = positions.ToArray();
            group.dering = true;
        }

        private static bool InsideFurniture(Vector3 q)
        {
            bool In(Vector3 c, Vector3 size) => Mathf.Abs(q.x - c.x) < size.x / 2f && Mathf.Abs(q.y - c.y) < size.y / 2f && Mathf.Abs(q.z - c.z) < size.z / 2f;
            float top = LabLayout.BenchTop;
            return In(new Vector3(LabLayout.ArmBase.x, top / 2f, LabLayout.BenchCentreZ), new Vector3(1.7f, top + 0.05f, 0.9f))
                || In(new Vector3(3.8f, 1.0f, LabLayout.MaxZ - 0.62f), new Vector3(1.4f, 2.1f, 1.15f))
                || In(new Vector3(LabLayout.MaxX - 0.28f, 1.0f, 1.5f), new Vector3(0.5f, 2.1f, 2.3f))
                || In(new Vector3(LabLayout.MinX + 1.25f, 0.4f, 1.0f), new Vector3(0.8f, 0.85f, 3.8f));
        }

        private static void BuildReflectionProbe(Transform root)
        {
            var go = new GameObject("ReflectionProbe");
            go.transform.SetParent(root, false);
            float cx = (LabLayout.MinX + LabLayout.MaxX) / 2f, cz = (LabLayout.MinZ + LabLayout.MaxZ) / 2f;
            go.transform.position = new Vector3(0.3f, 1.4f, 1.2f);
            var probe = go.AddComponent<ReflectionProbe>();
            probe.mode = ReflectionProbeMode.Baked;
            probe.boxProjection = true;
            probe.size = new Vector3(LabLayout.MaxX - LabLayout.MinX, LabLayout.Height, LabLayout.MaxZ - LabLayout.MinZ);
            probe.center = new Vector3(cx, LabLayout.Height / 2f, cz) - go.transform.position;
            probe.resolution = 256;
            probe.hdr = true;
            probe.importance = 1;
            probe.clearFlags = ReflectionProbeClearFlags.SolidColor;
            probe.backgroundColor = Color.black;
        }

        private static void ApplySceneSettings(Scene scene)
        {
            RenderSettings.skybox = null;
            RenderSettings.ambientMode = AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.05f, 0.055f, 0.07f);
            RenderSettings.subtractiveShadowColor = new Color(0.36f, 0.38f, 0.44f);
            RenderSettings.fog = false;
            RenderSettings.reflectionIntensity = 1f;

            LightingSettings settings = AssetDatabase.LoadAssetAtPath<LightingSettings>(SettingsPath);
            if (settings == null)
            {
                settings = new LightingSettings { name = "JetRoverLabXR" };
                AssetDatabase.CreateAsset(settings, SettingsPath);
            }

            settings.bakedGI = true;
            settings.realtimeGI = false;
            settings.mixedBakeMode = MixedLightingMode.Subtractive;
            settings.lightmapper = LightingSettings.Lightmapper.ProgressiveGPU;
            settings.directionalityMode = LightmapsMode.NonDirectional;
            settings.lightmapResolution = 28f;
            settings.lightmapPadding = 4;
            settings.lightmapMaxSize = 2048;
            settings.lightmapCompression = LightmapCompression.HighQuality;
            settings.ao = true;
            settings.aoMaxDistance = 0.5f;
            settings.aoExponentIndirect = 1f;
            settings.aoExponentDirect = 0f;
            settings.maxBounces = 2;
            settings.directSampleCount = 32;
            settings.indirectSampleCount = 256;
            settings.environmentSampleCount = 64;
            EditorUtility.SetDirty(settings);
            Lightmapping.SetLightingSettingsForScene(scene, settings);
        }

        /// <summary>Synchronous bake; the scene must be saved first and again afterwards.</summary>
        public static bool Bake()
        {
            float started = Time.realtimeSinceStartup;
            bool ok = Lightmapping.Bake();
            Debug.Log($"[lab] bake {(ok ? "done" : "FAILED")} in {Time.realtimeSinceStartup - started:0} s; {LightmapSettings.lightmaps.Length} lightmap(s)");
            return ok;
        }
    }
}
