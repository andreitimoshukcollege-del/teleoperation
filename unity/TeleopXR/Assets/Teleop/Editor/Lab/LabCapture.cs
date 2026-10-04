using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Teleop.Bridge;
using TMPro;
using UnityEditor;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Renders the lab from fixed viewpoints into PNGs plus one contact sheet, headless or from the
    /// menu (<b>Teleop → XR → Capture Lab Previews</b>). This is how a change to the lab is reviewed
    /// without a headset.
    ///
    /// <para>The arm is posed through the public <c>JetRoverArmRig.ApplyAngles</c> for the shots, one of
    /// them with the reach warning on, and is put back afterwards. Nothing is saved. Output goes outside
    /// the repository, to the system temp folder.</para>
    /// </summary>
    internal static class LabCapture
    {
        private readonly struct View
        {
            public readonly string Name;
            public readonly Vector3 From, To;
            public readonly float VerticalFov;
            public readonly bool Clamped;

            public View(string name, Vector3 from, Vector3 to, float fov, bool clamped = false)
            {
                Name = name; From = from; To = to; VerticalFov = fov; Clamped = clamped;
            }
        }

        private static IEnumerable<View> Views()
        {
            Vector3 arm = LabLayout.ArmBase;
            Vector3 screen = LabLayout.DisplayFoot + Vector3.up * LabLayout.ScreenCentreHeight;
            yield return new View("1_operator", new Vector3(0f, 1.62f, -0.05f), new Vector3(0.25f, 1.12f, 0.9f), 76f);
            yield return new View("2_operator_room", new Vector3(0f, 1.65f, -0.3f), new Vector3(0f, 1.35f, 4f), 88f);
            yield return new View("3_robot_close", new Vector3(0.42f, 1.24f, 0.02f), arm + new Vector3(0f, 0.02f, 0.06f), 48f);
            yield return new View("4_robot_side", new Vector3(-0.62f, 1.16f, 0.38f), arm + new Vector3(0f, -0.03f, 0.02f), 46f);
            yield return new View("5_reach_warning", new Vector3(0.3f, 1.35f, 0.75f), arm + new Vector3(0f, 0.1f, 0.12f), 50f, true);
            yield return new View("6_display", new Vector3(0.25f, 1.58f, 0.15f), screen, 58f);
            yield return new View("7_room_corner", new Vector3(4.3f, 2.5f, -2.5f), new Vector3(-0.8f, 0.9f, 2.2f), 78f);
            yield return new View("8_room_window", new Vector3(3.4f, 1.7f, 4.3f), new Vector3(-3.5f, 1.2f, 0.4f), 78f);
            yield return new View("9_floor_low", new Vector3(-0.9f, 0.32f, -0.9f), new Vector3(1.2f, 0.7f, 3.5f), 70f);
        }

        public static string Capture(string outDir)
        {
            Directory.CreateDirectory(outDir);
            ShaderUtil.allowAsyncCompilation = false;
            foreach (TMP_Text text in UnityEngine.Object.FindObjectsByType<TMP_Text>(FindObjectsInactive.Exclude))
            {
                text.ForceMeshUpdate(true, true);
            }

            JetRoverArmRig rig = UnityEngine.Object.FindAnyObjectByType<JetRoverArmRig>();
            Transform[] pivots = rig == null ? new Transform[0] : PivotsOf(rig);
            Quaternion[] saved = pivots.Select(t => t.localRotation).ToArray();

            var cameraObject = new GameObject("LabCaptureCamera") { hideFlags = HideFlags.HideAndDontSave };
            var cam = cameraObject.AddComponent<Camera>();
            cam.enabled = false;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.03f, 0.03f, 0.04f);
            cam.nearClipPlane = 0.02f;
            cam.farClipPlane = 40f;
            cam.allowHDR = false;
            cam.allowMSAA = true;

            const int fullW = 1600, fullH = 900, thumbW = 640, thumbH = 360;
            List<View> views = Views().ToList();
            int columns = 3, rows = (views.Count + columns - 1) / columns;
            var sheet = new Texture2D(thumbW * columns, thumbH * rows, TextureFormat.RGB24, false);
            var report = new StringBuilder();
            try
            {
                for (int i = 0; i < views.Count; i++)
                {
                    View view = views[i];
                    if (rig != null)
                    {
                        // A reaching pose: base turned a little, shoulder up, elbow folded down, wrist level.
                        rig.ApplyAngles(0.35f, 0.75f, -1.25f, 0.3f, view.Clamped);
                    }

                    cam.transform.position = view.From;
                    cam.transform.rotation = Quaternion.LookRotation(view.To - view.From, Vector3.up);
                    cam.fieldOfView = view.VerticalFov;

                    Texture2D full = Render(cam, fullW, fullH);
                    File.WriteAllBytes(Path.Combine(outDir, view.Name + ".png"), full.EncodeToPNG());
                    UnityEngine.Object.DestroyImmediate(full);

                    Texture2D thumb = Render(cam, thumbW, thumbH);
                    int col = i % columns, row = rows - 1 - i / columns;
                    sheet.SetPixels(col * thumbW, row * thumbH, thumbW, thumbH, thumb.GetPixels());
                    UnityEngine.Object.DestroyImmediate(thumb);
                    report.AppendLine(view.Name);
                }

                sheet.Apply();
                string sheetPath = Path.Combine(outDir, "00_contact.png");
                File.WriteAllBytes(sheetPath, sheet.EncodeToPNG());
                File.WriteAllText(Path.Combine(outDir, "report.txt"), Stats() + report);
                Debug.Log($"[lab] captured {views.Count} views to {outDir}");
                return sheetPath;
            }
            finally
            {
                for (int i = 0; i < pivots.Length; i++) pivots[i].localRotation = saved[i];
                if (rig != null)
                {
                    var so = new SerializedObject(rig);
                    if (so.FindProperty("reachWarningRenderer").objectReferenceValue is Renderer led) led.SetPropertyBlock(null);
                }

                UnityEngine.Object.DestroyImmediate(cameraObject);
                UnityEngine.Object.DestroyImmediate(sheet);
            }
        }

        private static Texture2D Render(Camera cam, int width, int height)
        {
            var msaa = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 4 };
            var resolved = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            cam.targetTexture = msaa;
            cam.Render();
            cam.Render(); // the first render can still be compiling shader variants
            Graphics.Blit(msaa, resolved);
            RenderTexture previous = RenderTexture.active;
            RenderTexture.active = resolved;
            var texture = new Texture2D(width, height, TextureFormat.RGB24, false);
            texture.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
            texture.Apply();
            RenderTexture.active = previous;
            cam.targetTexture = null;
            msaa.Release();
            resolved.Release();
            UnityEngine.Object.DestroyImmediate(msaa);
            UnityEngine.Object.DestroyImmediate(resolved);
            return texture;
        }

        private static Transform[] PivotsOf(JetRoverArmRig rig)
        {
            var so = new SerializedObject(rig);
            return new[] { "baseYawPivot", "lowerPitchPivot", "middlePitchPivot", "upperPitchPivot" }
                .Select(f => so.FindProperty(f).objectReferenceValue as Transform).Where(t => t != null).ToArray();
        }

        /// <summary>What the scene costs: triangles, renderers, materials, lightmaps.</summary>
        public static string Stats()
        {
            Renderer[] renderers = UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsInactive.Exclude).Where(r => r.enabled).ToArray();
            long triangles = 0;
            foreach (Renderer r in renderers)
            {
                if (r is MeshRenderer && r.TryGetComponent(out MeshFilter f) && f.sharedMesh != null)
                {
                    for (int s = 0; s < f.sharedMesh.subMeshCount; s++) triangles += f.sharedMesh.GetIndexCount(s) / 3;
                }
            }

            int materials = renderers.SelectMany(r => r.sharedMaterials).Where(m => m != null).Distinct().Count();
            int lightmaps = LightmapSettings.lightmaps.Length;
            string sizes = string.Join(", ", LightmapSettings.lightmaps.Select(l => l.lightmapColor != null ? $"{l.lightmapColor.width}x{l.lightmapColor.height}" : "?"));
            int lights = UnityEngine.Object.FindObjectsByType<Light>(FindObjectsInactive.Exclude).Count(l => l.lightmapBakeType != LightmapBakeType.Baked);
            return $"triangles: {triangles}\nrenderers: {renderers.Length}\nmaterials: {materials}\nlightmaps: {lightmaps} ({sizes})\nnon-baked lights: {lights}\n";
        }
    }
}
