using UnityEditor;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// The lab's palette as URP materials, owned by the generator: each rebuild rewrites them in
    /// place, so their asset IDs (and every renderer's reference to them) stay stable.
    ///
    /// <para>Three shaders, chosen for cost.</para>
    /// <list type="bullet">
    /// <item><b>Lit</b> where reflections or metal matter: the glossy floor, the robot, the bezel.</item>
    /// <item><b>Simple Lit</b> for the big matte surfaces (walls, ceiling, furniture). Galaxy XR's
    /// displays are very high resolution, so every full-screen pixel counts.</item>
    /// <item><b>Unlit</b> for anything that emits its own light: screens, LEDs, the status ring.</item>
    /// </list>
    /// </summary>
    internal static class LabMaterials
    {
        private const string LitShader = "Universal Render Pipeline/Lit";
        private const string SimpleLitShader = "Universal Render Pipeline/Simple Lit";
        private const string UnlitShader = "Universal Render Pipeline/Unlit";

        public static Material Lit(string name, Color color, float smoothness, float metallic = 0f,
            Texture2D texture = null, float tiling = 1f, Color? emission = null, bool emissionLightsScene = false)
        {
            Material m = Load(name, LitShader);
            m.SetColor("_BaseColor", color);
            m.SetFloat("_Smoothness", smoothness);
            m.SetFloat("_Metallic", metallic);
            SetTexture(m, texture, tiling);
            SetEmission(m, emission, emissionLightsScene, texture);
            EditorUtility.SetDirty(m);
            return m;
        }

        public static Material Matte(string name, Color color, float smoothness = 0.15f, Texture2D texture = null,
            float tiling = 1f, Color? emission = null, bool emissionLightsScene = false)
        {
            Material m = Load(name, SimpleLitShader);
            m.SetColor("_BaseColor", color);
            m.SetFloat("_Smoothness", smoothness);
            m.SetColor("_SpecColor", new Color(0.18f, 0.18f, 0.18f) * (0.3f + smoothness));
            m.EnableKeyword("_SPECULAR_COLOR");
            SetTexture(m, texture, tiling);
            SetEmission(m, emission, emissionLightsScene, texture);
            EditorUtility.SetDirty(m);
            return m;
        }

        public static Material Unlit(string name, Color color, Texture2D texture = null, float tiling = 1f)
        {
            Material m = Load(name, UnlitShader);
            m.SetColor("_BaseColor", color);
            SetTexture(m, texture, tiling);
            // Unlit emitters never light the bake: their colour may change at runtime (status ring,
            // LEDs), and a baked bounce would keep showing the old colour.
            m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.EmissiveIsBlack;
            EditorUtility.SetDirty(m);
            return m;
        }

        private static void SetTexture(Material m, Texture2D texture, float tiling)
        {
            m.SetTexture("_BaseMap", texture);
            m.SetTextureScale("_BaseMap", new Vector2(tiling, tiling));
        }

        private static void SetEmission(Material m, Color? emission, bool lightsScene, Texture2D map)
        {
            if (emission.HasValue)
            {
                m.EnableKeyword("_EMISSION");
                m.SetColor("_EmissionColor", emission.Value);
                m.SetTexture("_EmissionMap", map);
                m.globalIlluminationFlags = lightsScene ? MaterialGlobalIlluminationFlags.BakedEmissive : MaterialGlobalIlluminationFlags.EmissiveIsBlack;
            }
            else
            {
                m.DisableKeyword("_EMISSION");
                m.SetColor("_EmissionColor", Color.black);
                m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.EmissiveIsBlack;
            }
        }

        private static Material Load(string name, string shaderName)
        {
            LabPaths.EnsureFolder(LabPaths.Materials);
            string path = $"{LabPaths.Materials}/{name}.mat";
            Shader shader = Shader.Find(shaderName)
                ?? throw new System.InvalidOperationException($"Shader '{shaderName}' not found; is URP installed?");
            Material m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                m = new Material(shader) { name = name };
                AssetDatabase.CreateAsset(m, path);
            }
            else if (m.shader != shader)
            {
                m.shader = shader;
            }

            return m;
        }
    }

    /// <summary>The palette itself: every colour and finish in one place, named by what it is.</summary>
    internal sealed class LabPalette
    {
        // Room
        public Material Floor, Wall, WallDark, Ceiling, Slats, CoveCyan, FixtureHousing, FixtureDiffuser, Hazard, StatusRing,
            Skyline, WindowFrame, SignBacklight, Pegboard, Door, ExitSign;
        // Furniture and props
        public Material BenchTop, BenchMat, Aluminium, SteelDark, Plastic, ChairFabric, BinBlue, BinGrey, BinOrange,
            RackBody, RackFront, Led, CodeScreen, GraphScreen, ScreenOff, SafetyRed, Plant, Pot, OrangeArm;
        // Robot
        public Material AnodizedBlack, ServoBlack, RobotGreen, Rubber, Brass, Pcb, Heatsink, LidarGloss, Glass, LensRing,
            RobotLcd, WristLed;
        // Display
        public Material Bezel, StripGlass, LiveDot, Underglow;
        // Test area and motion capture
        public Material Tape, Fiducial, ConeOrange, MocapRing;

        public static LabPalette Create()
        {
            var p = new LabPalette
            {
                Floor = LabMaterials.Lit("Floor", new Color(0.95f, 0.97f, 1f), 0.88f, 0f, LabTextures.FloorEpoxy(), 0.5f),
                Wall = LabMaterials.Matte("Wall", new Color(0.21f, 0.22f, 0.24f), 0.1f),
                WallDark = LabMaterials.Matte("WallDark", new Color(0.085f, 0.09f, 0.1f), 0.15f),
                Ceiling = LabMaterials.Matte("Ceiling", new Color(0.17f, 0.175f, 0.185f), 0.05f),
                Slats = LabMaterials.Matte("Slats", Color.white, 0.25f, LabTextures.Slats(), 1f),
                // HDR colours: the screen clamps them to what they look like, but the reflection probe keeps
                // the full value, so the floor reflects them as bright streaks instead of faint smudges.
                CoveCyan = LabMaterials.Unlit("CoveCyan", new Color(0.12f, 2.5f, 3f)),
                FixtureHousing = LabMaterials.Lit("FixtureHousing", new Color(0.08f, 0.085f, 0.09f), 0.5f, 0.6f),
                FixtureDiffuser = LabMaterials.Unlit("FixtureDiffuser", new Color(3f, 3.05f, 3.15f)),
                Hazard = LabMaterials.Matte("Hazard", Color.white, 0.3f, LabTextures.Hazard(), 6f),
                StatusRing = LabMaterials.Unlit("StatusRing", new Color(0.55f, 0.06f, 0.08f)),
                Skyline = LabMaterials.Unlit("Skyline", Color.white, LabTextures.Skyline()),
                WindowFrame = LabMaterials.Lit("WindowFrame", new Color(0.05f, 0.05f, 0.055f), 0.5f, 0.7f),
                SignBacklight = LabMaterials.Unlit("SignBacklight", new Color(0.05f, 0.35f, 0.5f)),
                Pegboard = LabMaterials.Matte("Pegboard", Color.white, 0.1f, LabTextures.Pegboard(), 2f),
                Door = LabMaterials.Matte("Door", new Color(0.18f, 0.19f, 0.21f), 0.3f),
                ExitSign = LabMaterials.Unlit("ExitSign", new Color(0.15f, 0.95f, 0.35f)),

                BenchTop = LabMaterials.Lit("BenchTop", new Color(0.09f, 0.095f, 0.105f), 0.55f),
                BenchMat = LabMaterials.Matte("BenchMat", Color.white, 0.2f, LabTextures.EsdMat(), 2f),
                Aluminium = LabMaterials.Lit("Aluminium", new Color(0.72f, 0.74f, 0.77f), 0.62f, 1f),
                SteelDark = LabMaterials.Lit("SteelDark", new Color(0.3f, 0.31f, 0.33f), 0.45f, 0.8f),
                Plastic = LabMaterials.Lit("Plastic", new Color(0.04f, 0.042f, 0.046f), 0.45f),
                ChairFabric = LabMaterials.Matte("ChairFabric", new Color(0.09f, 0.1f, 0.12f), 0.05f),
                BinBlue = LabMaterials.Matte("BinBlue", new Color(0.1f, 0.3f, 0.65f), 0.35f),
                BinGrey = LabMaterials.Matte("BinGrey", new Color(0.32f, 0.33f, 0.35f), 0.35f),
                BinOrange = LabMaterials.Matte("BinOrange", new Color(0.85f, 0.38f, 0.06f), 0.35f),
                RackBody = LabMaterials.Lit("RackBody", new Color(0.03f, 0.032f, 0.036f), 0.4f, 0.5f),
                RackFront = LabMaterials.Lit("RackFront", Color.white, 0.4f, 0.3f, LabTextures.RackFront(), 1f, new Color(1f, 1f, 1f)),
                Led = LabMaterials.Unlit("Led", new Color(0.2f, 1f, 0.45f)),
                CodeScreen = LabMaterials.Unlit("CodeScreen", new Color(0.85f, 0.85f, 0.85f), LabTextures.CodeScreen()),
                GraphScreen = LabMaterials.Unlit("GraphScreen", new Color(0.85f, 0.85f, 0.85f), LabTextures.GraphScreen()),
                ScreenOff = LabMaterials.Lit("ScreenOff", new Color(0.01f, 0.01f, 0.012f), 0.9f),
                SafetyRed = LabMaterials.Lit("SafetyRed", new Color(0.7f, 0.05f, 0.04f), 0.6f),
                Plant = LabMaterials.Matte("Plant", new Color(0.1f, 0.32f, 0.12f), 0.2f),
                Pot = LabMaterials.Matte("Pot", new Color(0.75f, 0.74f, 0.7f), 0.3f),
                OrangeArm = LabMaterials.Lit("OrangeArm", new Color(0.95f, 0.42f, 0.05f), 0.55f),

                AnodizedBlack = LabMaterials.Lit("AnodizedBlack", new Color(0.07f, 0.072f, 0.078f), 0.7f, 0.7f),
                ServoBlack = LabMaterials.Lit("ServoBlack", new Color(0.035f, 0.035f, 0.04f), 0.5f),
                RobotGreen = LabMaterials.Lit("RobotGreen", new Color(0.12f, 0.62f, 0.28f), 0.55f, 0.3f),
                Rubber = LabMaterials.Lit("Rubber", new Color(0.035f, 0.035f, 0.038f), 0.35f),
                Brass = LabMaterials.Lit("Brass", new Color(0.78f, 0.62f, 0.3f), 0.6f, 1f),
                Pcb = LabMaterials.Lit("Pcb", new Color(0.04f, 0.22f, 0.12f), 0.5f),
                Heatsink = LabMaterials.Lit("Heatsink", new Color(0.05f, 0.05f, 0.055f), 0.45f, 0.8f),
                LidarGloss = LabMaterials.Lit("LidarGloss", new Color(0.015f, 0.015f, 0.02f), 0.92f),
                Glass = LabMaterials.Unlit("Glass", new Color(0.02f, 0.03f, 0.05f)),
                LensRing = LabMaterials.Lit("LensRing", new Color(0.6f, 0.62f, 0.65f), 0.7f, 1f),
                RobotLcd = LabMaterials.Unlit("RobotLcd", Color.white, LabTextures.RobotLcd()),
                WristLed = LabMaterials.Unlit("WristLed", Color.white),

                Bezel = LabMaterials.Lit("Bezel", new Color(0.025f, 0.026f, 0.03f), 0.7f, 0.4f),
                StripGlass = LabMaterials.Unlit("StripGlass", new Color(0.012f, 0.014f, 0.02f)),
                LiveDot = LabMaterials.Unlit("LiveDot", new Color(0.3f, 0.32f, 0.36f)),
                Underglow = LabMaterials.Unlit("Underglow", new Color(0.1f, 0.85f, 1f)),

                Tape = LabMaterials.Matte("Tape", new Color(0.82f, 0.83f, 0.85f), 0.3f),
                Fiducial = LabMaterials.Matte("Fiducial", Color.white, 0.2f, LabTextures.Fiducials(), 1f),
                ConeOrange = LabMaterials.Lit("ConeOrange", new Color(0.95f, 0.3f, 0.03f), 0.45f),
                MocapRing = LabMaterials.Unlit("MocapRing", new Color(0.75f, 0.06f, 0.05f)),
            };
            AssetDatabase.SaveAssets();
            return p;
        }
    }
}
