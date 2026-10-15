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
            /// <summary>For a gripper close-up: the gripper fraction, else negative.</summary>
            public readonly float Gripper;

            public View(string name, Vector3 from, Vector3 to, float fov, bool clamped = false, float gripper = -1f)
            {
                Name = name; From = from; To = to; VerticalFov = fov; Clamped = clamped; Gripper = gripper;
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
            yield return new View("10_motion_button", new Vector3(-0.2f, 1.45f, -0.1f), new Vector3(-0.514f, 0.92f, 0.22f), 50f);
            yield return new View("9_floor_low", new Vector3(-0.9f, 0.32f, -0.9f), new Vector3(1.2f, 0.7f, 3.5f), 70f);
            // Square onto the commanded arm's gripper, along the jaws' hinge axis; placed in Capture.
            yield return new View("11_gripper_open", Vector3.zero, Vector3.zero, 30f, gripper: 0f);
            yield return new View("12_gripper_closed", Vector3.zero, Vector3.zero, 30f, gripper: 1f);
        }

        /// <summary>
        /// The shown arm's gripper: between its two jaw hinges, and the hinge axis, toward the room's +x
        /// side. Previews show only the commanded arm; the other two start inactive.
        /// </summary>
        private static bool GripperShot(out Vector3 centre, out Vector3 axis, out Vector3 fingers)
        {
            centre = axis = fingers = Vector3.zero;
            Transform[] all = UnityEngine.Object.FindObjectsByType<Transform>(FindObjectsInactive.Exclude);
            Transform left = all.FirstOrDefault(t => t.name == JetRoverCad.JawLeft);
            Transform right = all.FirstOrDefault(t => t.name == JetRoverCad.JawRight);
            if (left == null || right == null) return false;
            Transform leftOuter = all.FirstOrDefault(t => t.name == JetRoverCad.JawLeftOuter);
            Transform rightOuter = all.FirstOrDefault(t => t.name == JetRoverCad.JawRightOuter);
            if (leftOuter == null || rightOuter == null) return false;
            centre = 0.5f * (left.position + right.position);
            fingers = (0.5f * (leftOuter.position + rightOuter.position) - centre).normalized;
            // Seen from above, or from the operator's side when the hinge axis lies flat.
            axis = Vector3.Dot(left.right, new Vector3(0.3f, 1f, -0.3f)) >= 0f ? left.right : -left.right;
            return true;
        }

        public static string Capture(string outDir)
        {
            Directory.CreateDirectory(outDir);
            ShaderUtil.allowAsyncCompilation = false;
            foreach (TMP_Text text in UnityEngine.Object.FindObjectsByType<TMP_Text>(FindObjectsInactive.Exclude))
            {
                text.ForceMeshUpdate(true, true);
            }

            JetRoverArmRig rig = XrControlSetup.CommandedRig();
            Transform[] pivots = rig == null ? new Transform[0] : PivotsOf(rig);
            Quaternion[] saved = pivots.Select(t => t.localRotation).ToArray();
            TwinGripper[] grippers = UnityEngine.Object.FindObjectsByType<TwinGripper>(FindObjectsInactive.Exclude);

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

                    // The reach-warning view also shows the gripper closed and rolled.
                    float gripper = view.Gripper >= 0f ? view.Gripper : view.Clamped ? 1f : 0f;
                    foreach (TwinGripper g in grippers) g.Show(gripper, view.Clamped ? 0.6f : 0f);

                    Vector3 from = view.From, to = view.To, up = Vector3.up;
                    if (view.Gripper >= 0f && GripperShot(out Vector3 centre, out Vector3 axis, out Vector3 fingers))
                    {
                        // Fingers up in the frame.
                        to = centre + fingers * 0.02f;
                        from = to + axis * 0.32f;
                        up = fingers;
                    }

                    cam.transform.position = from;
                    cam.transform.rotation = Quaternion.LookRotation(to - from, up);
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
                foreach (TwinGripper g in grippers) g.Show(0f, 0f);
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
