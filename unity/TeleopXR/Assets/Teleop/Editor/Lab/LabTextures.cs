using System;
using System.IO;
using UnityEditor;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>Folders the lab writes into, and a folder helper.</summary>
    internal static class LabPaths
    {
        public const string Root = "Assets/Teleop/Lab";
        public const string Generated = Root + "/Generated";
        public const string Textures = Generated + "/Textures";
        public const string Materials = Generated + "/Materials";

        public static void EnsureFolder(string path) => XrProjectSetup.EnsureFolder(path);
    }

    /// <summary>
    /// The lab's textures, drawn in code from a fixed seed and saved as PNGs. Every rebuild produces
    /// identical files, so they only change in git when the drawing code changes. Each one exists to
    /// make a surface read as a real material from a metre away. None carries information.
    /// </summary>
    internal static class LabTextures
    {
        public static Texture2D FloorEpoxy() => Get("FloorEpoxy", 1024, 1024, true, (x, y, w, h) =>
        {
            // One texture is 2 m x 2 m: dark epoxy, soft mottling, fine speckle, a seam every metre.
            float n = Noise(x * 0.012f, y * 0.012f, 11) * 0.6f + Noise(x * 0.05f, y * 0.05f, 12) * 0.3f + Hash(x, y, 13) * 0.1f;
            float v = 0.11f + n * 0.05f;
            Color c = new Color(v, v * 1.06f, v * 1.18f);
            int sx = x % 512, sy = y % 512;
            if (sx < 2 || sy < 2) c *= 0.55f;
            else if (sx < 4 || sy < 4) c *= 1.25f;
            return c;
        });

        // Square, so the diagonal stays at 45° whatever the strip's direction; four bands per tile.
        public static Texture2D Hazard() => Get("Hazard", 256, 256, true, (x, y, w, h) =>
            ((x + y) / 64) % 2 == 0 ? new Color(0.95f, 0.72f, 0.04f) : new Color(0.04f, 0.04f, 0.045f));

        public static Texture2D EsdMat() => Get("EsdMat", 512, 512, true, (x, y, w, h) =>
        {
            // 0.5 m x 0.5 m of blue-grey anti-static mat: a faint 1 cm grid, a stronger line every 5 cm.
            Color c = new Color(0.20f, 0.26f, 0.32f) * (0.95f + Noise(x * 0.03f, y * 0.03f, 21) * 0.1f);
            float cm = 512f / 50f;
            bool fine = (x % Mathf.RoundToInt(cm)) == 0 || (y % Mathf.RoundToInt(cm)) == 0;
            bool major = (x % Mathf.RoundToInt(cm * 5)) < 2 || (y % Mathf.RoundToInt(cm * 5)) < 2;
            return major ? c * 1.35f : fine ? c * 1.12f : c;
        });

        public static Texture2D Slats() => Get("Slats", 512, 512, true, (x, y, w, h) =>
        {
            // 1 m of vertical walnut slats on black felt: 16 slats, grain along their length.
            int slat = x * 16 / w;
            int within = x - slat * w / 16;
            if (within < 6) return new Color(0.012f, 0.012f, 0.014f);
            float grain = Noise(x * 0.6f, y * 0.01f, 31 + slat) * 0.5f + Noise(x * 0.08f, y * 0.004f, 41 + slat) * 0.5f;
            float tone = 0.85f + Hash(slat, 0, 51) * 0.25f;
            return new Color(0.34f, 0.21f, 0.12f) * tone * (0.75f + grain * 0.5f);
        });

        public static Texture2D CodeScreen() => Get("CodeScreen", 512, 320, false, (x, y, w, h) =>
        {
            // An editor full of code: coloured tokens on a dark background, with indentation.
            var bg = new Color(0.02f, 0.025f, 0.04f);
            int line = (h - 1 - y) / 9, row = (h - 1 - y) % 9;
            if (x < 22) return (row >= 2 && row <= 5 && x > 6 && x < 18 && Hash(line, 1, 61) > 0.4f) ? new Color(0.25f, 0.27f, 0.32f) : new Color(0.03f, 0.035f, 0.05f);
            if (row < 2 || row > 6) return bg;
            int indent = (int)(Hash(line, 2, 62) * 4) * 14;
            int cursor = 28 + indent;
            if (x < cursor) return bg;
            int tokens = 2 + (int)(Hash(line, 3, 63) * 6);
            if (Hash(line, 4, 64) < 0.12f) return bg; // blank line
            for (int t = 0; t < tokens; t++)
            {
                int len = 10 + (int)(Hash(line, 10 + t, 65) * 46);
                if (x >= cursor && x < cursor + len)
                {
                    float k = Hash(line, 20 + t, 66);
                    return k < 0.25f ? new Color(0.35f, 0.78f, 0.95f) : k < 0.45f ? new Color(0.85f, 0.45f, 0.85f)
                        : k < 0.65f ? new Color(0.55f, 0.85f, 0.45f) : k < 0.8f ? new Color(0.95f, 0.7f, 0.35f) : new Color(0.75f, 0.77f, 0.8f);
                }

                cursor += len + 7;
            }

            return bg;
        });

        public static Texture2D GraphScreen() => Get("GraphScreen", 512, 320, false, (x, y, w, h) =>
        {
            // A telemetry dashboard: grid, three traces, a bar chart.
            var c = new Color(0.02f, 0.03f, 0.045f);
            if (x % 64 == 0 || y % 40 == 0) c = new Color(0.06f, 0.08f, 0.11f);
            float fx = x / (float)w;
            float t1 = 0.62f + 0.18f * Mathf.Sin(fx * 18f) * Mathf.Sin(fx * 3.1f);
            float t2 = 0.40f + 0.12f * Mathf.Sin(fx * 31f + 1f) + 0.05f * Noise(x * 0.2f, 0, 71);
            float t3 = 0.22f + 0.10f * Mathf.Sin(fx * 9f + 2f);
            float fy = y / (float)h;
            if (Mathf.Abs(fy - t1) < 0.006f) return new Color(0.2f, 0.85f, 1f);
            if (Mathf.Abs(fy - t2) < 0.006f) return new Color(1f, 0.6f, 0.2f);
            if (Mathf.Abs(fy - t3) < 0.006f) return new Color(0.4f, 0.95f, 0.5f);
            if (fy > 0.82f && x % 24 < 16 && fy < 0.84f + Hash(x / 24, 0, 72) * 0.12f) return new Color(0.35f, 0.55f, 0.95f);
            return c;
        });

        public static Texture2D Skyline() => Get("Skyline", 2048, 512, false, (x, y, w, h) =>
        {
            // A city at night: deep blue sky to a violet glow at the horizon, towers with lit windows.
            float fy = y / (float)h;
            Color sky = Color.Lerp(new Color(0.16f, 0.09f, 0.22f), new Color(0.015f, 0.02f, 0.06f), Mathf.Pow(fy, 0.6f));
            if (fy > 0.55f && Hash(x, y, 81) > 0.9993f) sky += new Color(0.6f, 0.6f, 0.7f);
            int column = x / 38;
            float towerTop = 0.18f + Hash(column, 0, 82) * 0.55f;
            int towerX = x % 38;
            bool gap = towerX < 3 + (int)(Hash(column, 1, 83) * 5);
            if (gap || fy > towerTop)
            {
                // A lower building behind, in a second rhythm.
                int back = (x + 17) / 53;
                float backTop = 0.12f + Hash(back, 2, 84) * 0.25f;
                return fy > backTop ? sky : new Color(0.03f, 0.03f, 0.05f);
            }

            if (towerTop - fy < 0.006f && towerX > 15 && towerX < 19 && Hash(column, 3, 85) > 0.6f) return new Color(1f, 0.1f, 0.08f);
            bool window = (towerX % 6) > 2 && (y % 9) > 4;
            if (window && Hash(x / 6, y / 9, 86 + column) > 0.55f)
            {
                float warm = Hash(x / 6, y / 9, 87);
                return warm > 0.3f ? new Color(1f, 0.82f, 0.5f) * (0.6f + warm * 0.4f) : new Color(0.7f, 0.85f, 1f) * 0.8f;
            }

            return new Color(0.025f, 0.025f, 0.04f);
        });

        public static Texture2D RackFront() => Get("RackFront", 256, 512, false, (x, y, w, h) =>
        {
            // 1U panels with vents, drive bays and status LEDs. Used as the emission map too, so only
            // the LEDs and bay lights glow.
            int u = y / 20, within = y % 20;
            if (within < 1) return new Color(0.01f, 0.01f, 0.012f);
            float kind = Hash(u, 0, 91);
            if (x > 12 && x < 22 && within > 8 && within < 12)
            {
                return Hash(u, 1, 92) > 0.3f ? new Color(0.2f, 1f, 0.4f) : new Color(1f, 0.6f, 0.1f);
            }

            if (kind < 0.4f && x > 40 && x < 230 && within > 5 && within < 16 && (x % 24) < 20)
            {
                return (x % 24 > 16 && within > 9 && within < 12 && Hash(x / 24, u, 93) > 0.4f) ? new Color(0.25f, 0.6f, 1f) : new Color(0.08f, 0.085f, 0.095f);
            }

            if (kind >= 0.4f && x > 40 && x < 230 && (x % 4) < 2 && (within % 4) < 2) return new Color(0.005f, 0.005f, 0.006f);
            return new Color(0.045f, 0.047f, 0.052f);
        });

        public static Texture2D RobotLcd() => Get("RobotLcd", 256, 160, false, (x, y, w, h) =>
        {
            // The JetRover's 7-inch screen: a status bar, a battery gauge, a terminal.
            var bg = new Color(0.03f, 0.05f, 0.07f);
            if (y > h - 18) return x > w - 46 && x < w - 10 && y > h - 14 && y < h - 5 ? (x < w - 20 ? new Color(0.3f, 0.9f, 0.4f) : new Color(0.15f, 0.2f, 0.2f)) : new Color(0.08f, 0.12f, 0.16f);
            int line = (h - 26 - y) / 10;
            if (y < h - 26 && (h - 26 - y) % 10 > 3 && (h - 26 - y) % 10 < 8 && x > 10 && x < 10 + 40 + Hash(line, 0, 101) * 180)
            {
                return Hash(line, 1, 102) > 0.7f ? new Color(0.3f, 0.95f, 0.5f) : new Color(0.7f, 0.75f, 0.8f);
            }

            return bg;
        });

        public static Texture2D Pegboard() => Get("Pegboard", 512, 512, true, (x, y, w, h) =>
        {
            // 0.5 m of white pegboard with a hole every 2.5 cm.
            int pitch = 512 / 20;
            int dx = x % pitch - pitch / 2, dy = y % pitch - pitch / 2;
            if (dx * dx + dy * dy < 12) return new Color(0.04f, 0.04f, 0.045f);
            return new Color(0.78f, 0.79f, 0.8f) * (0.95f + Noise(x * 0.02f, y * 0.02f, 111) * 0.08f);
        });

        public static Texture2D Fiducials() => Get("Fiducials", 512, 512, false, (x, y, w, h) =>
        {
            // Four printed square tags in a 2 x 2 atlas, laid out like AprilTags: a white margin, a black
            // border, a 6 x 6 grid of bits. The bits are arbitrary, not a real tag family's codes.
            var paper = new Color(0.86f, 0.86f, 0.84f);
            var ink = new Color(0.03f, 0.03f, 0.035f);
            int lx = x % 256, ly = y % 256;
            if (lx < 8 || ly < 8 || lx >= 248 || ly >= 248) return paper;
            int cx = (lx - 8) / 24, cy = (ly - 8) / 24; // 10 x 10 cells of 24 px
            if (cx == 0 || cy == 0 || cx == 9 || cy == 9) return paper;
            if (cx == 1 || cy == 1 || cx == 8 || cy == 8) return ink;
            int tag = x / 256 + 2 * (y / 256);
            return Hash(cx, cy, 121 + tag) < 0.5f ? ink : paper;
        });

        // ---- machinery ---------------------------------------------------------------------

        private static Texture2D Get(string name, int width, int height, bool repeat, Func<int, int, int, int, Color> pixel)
        {
            LabPaths.EnsureFolder(LabPaths.Textures);
            string path = $"{LabPaths.Textures}/{name}.png";
            var pixels = new Color32[width * height];
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    Color c = pixel(x, y, width, height);
                    pixels[y * width + x] = new Color(Mathf.Clamp01(c.r), Mathf.Clamp01(c.g), Mathf.Clamp01(c.b), 1f);
                }
            }

            var texture = new Texture2D(width, height, TextureFormat.RGBA32, false);
            texture.SetPixels32(pixels);
            texture.Apply();
            byte[] png = texture.EncodeToPNG();
            UnityEngine.Object.DestroyImmediate(texture);

            // Rewrite only when the bytes differ, so an unchanged texture never shows up in git.
            if (!File.Exists(path) || !BytesEqual(File.ReadAllBytes(path), png))
            {
                File.WriteAllBytes(path, png);
                AssetDatabase.ImportAsset(path);
            }

            if (AssetImporter.GetAtPath(path) is TextureImporter importer)
            {
                bool changed = importer.wrapMode != (repeat ? TextureWrapMode.Repeat : TextureWrapMode.Clamp)
                    || !importer.mipmapEnabled || importer.anisoLevel != 4 || importer.npotScale != TextureImporterNPOTScale.None;
                if (changed)
                {
                    importer.wrapMode = repeat ? TextureWrapMode.Repeat : TextureWrapMode.Clamp;
                    importer.mipmapEnabled = true;
                    importer.anisoLevel = 4;
                    importer.filterMode = FilterMode.Trilinear;
                    importer.npotScale = TextureImporterNPOTScale.None;
                    importer.SaveAndReimport();
                }
            }

            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        private static bool BytesEqual(byte[] a, byte[] b)
        {
            if (a.Length != b.Length) return false;
            for (int i = 0; i < a.Length; i++)
            {
                if (a[i] != b[i]) return false;
            }

            return true;
        }

        /// <summary>A stable pseudo-random value in [0, 1) for an integer lattice point.</summary>
        internal static float Hash(int x, int y, int seed)
        {
            unchecked
            {
                uint h = (uint)(x * 374761393 + y * 668265263 + seed * 2147483647);
                h = (h ^ (h >> 13)) * 1274126177u;
                h ^= h >> 16;
                return (h & 0xFFFFFF) / 16777216f;
            }
        }

        /// <summary>Smooth value noise in [0, 1).</summary>
        internal static float Noise(float x, float y, int seed)
        {
            int ix = Mathf.FloorToInt(x), iy = Mathf.FloorToInt(y);
            float fx = x - ix, fy = y - iy;
            fx = fx * fx * (3f - 2f * fx);
            fy = fy * fy * (3f - 2f * fy);
            float a = Hash(ix, iy, seed), b = Hash(ix + 1, iy, seed), c = Hash(ix, iy + 1, seed), d = Hash(ix + 1, iy + 1, seed);
            return Mathf.Lerp(Mathf.Lerp(a, b, fx), Mathf.Lerp(c, d, fx), fy);
        }
    }
}
