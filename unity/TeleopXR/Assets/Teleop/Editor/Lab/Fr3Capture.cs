using System;
using System.IO;
using System.Linq;
using Teleop.Bridge;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Renders <c>Fr3DuoXR</c> from fixed viewpoints into PNGs (<c>just unity-preview-fr3-xr</c>), the twin posed at
    /// Franka's ready pose with the hands half open, so the twin's meshes can be reviewed without a headset. The FR3
    /// scene check proves the twin's frames; this shows its shapes. Nothing is saved; output goes to the system temp
    /// folder, <c>%TEMP%\teleop-fr3\&lt;time&gt;\</c>.
    /// </summary>
    internal static class Fr3Capture
    {
        private static readonly float[] LeftReady = { 0f, -Mathf.PI / 4, 0f, -3 * Mathf.PI / 4, 0f, Mathf.PI / 2, Mathf.PI / 4 };

        private static readonly (string Name, Vector3 From, Vector3 To)[] Views =
        {
            ("01-operator", new Vector3(0f, 1.62f, 0f), new Vector3(0f, 1.0f, 0.95f)),
            ("02-front", new Vector3(0f, 1.3f, 2.4f), new Vector3(0f, 1.05f, 0.8f)),
            ("03-left", new Vector3(-1.6f, 1.25f, 0.9f), new Vector3(0f, 1.05f, 0.85f)),
            ("04-above", new Vector3(0f, 2.8f, 0.9f), new Vector3(0f, 0.8f, 0.95f)),
        };

        /// <summary>Batch entry point (with graphics): <c>-executeMethod Teleop.XR.Editor.Fr3Capture.Run</c>.</summary>
        public static void Run()
        {
            try
            {
                Debug.Log($"[fr3-scene] previews: {Capture()}");
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[fr3-scene] preview FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        public static string Capture()
        {
            EditorSceneManager.OpenScene(XrFr3SceneBuilder.ScenePath, OpenSceneMode.Single);
            var twin = Object.FindAnyObjectByType<SerialChainArmRig>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("no SerialChainArmRig in the FR3 scene");
            for (int i = 0; i < twin.Joints.Length; i++)
            {
                SerialChainRigJoint j = twin.Joints[i];
                int k = j.motorId % 10 - 1; // 1-7 left, 11-17 right; 8 and 18 the hands
                bool right = j.motorId > 10;
                if (j.prismatic) twin.SetJoint(i, 0.5f);
                else if (k >= 0 && k < 7) twin.SetJoint(i, right && k % 2 == 0 ? -LeftReady[k] : LeftReady[k]);
            }

            string outDir = Path.Combine(Path.GetTempPath(), "teleop-fr3", DateTime.Now.ToString("yyyyMMdd-HHmmss"));
            Directory.CreateDirectory(outDir);
            var cameraObject = new GameObject("PreviewCamera");
            var cam = cameraObject.AddComponent<Camera>();
            cam.fieldOfView = 60f;
            cam.nearClipPlane = 0.02f;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.07f, 0.075f, 0.09f);
            try
            {
                foreach ((string name, Vector3 from, Vector3 to) in Views)
                {
                    cam.transform.position = from;
                    cam.transform.LookAt(to);
                    Texture2D image = LabCapture.Render(cam, 1280, 960);
                    File.WriteAllBytes(Path.Combine(outDir, name + ".png"), image.EncodeToPNG());
                    Object.DestroyImmediate(image);
                }
            }
            finally
            {
                Object.DestroyImmediate(cameraObject);
            }

            // Index counts, not the triangle arrays: the visual meshes are imported without Read/Write.
            long triangles = twin.GetComponentsInChildren<MeshFilter>(true).Where(f => f.sharedMesh != null)
                .Sum(f => Enumerable.Range(0, f.sharedMesh.subMeshCount).Sum(m => (long)f.sharedMesh.GetIndexCount(m)) / 3);
            File.WriteAllText(Path.Combine(outDir, "report.txt"), $"FR3 Duo twin: {twin.Joints.Length} moving joints, {triangles} mesh triangles (boxes included)\n");
            return outDir;
        }
    }
}
