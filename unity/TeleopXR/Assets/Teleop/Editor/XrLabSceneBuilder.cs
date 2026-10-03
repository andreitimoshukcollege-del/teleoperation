using System;
using System.IO;
using System.Linq;
using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Attachment;
using UnityEngine.XR.Interaction.Toolkit.Inputs;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// <b>Teleop → XR → Build Lab Scene</b> (or <c>just unity-scene-xr</c>): writes
    /// <c>Assets/Scenes/JetRoverLabXR.unity</c>, the Galaxy XR operator scene (docs/adr/0016 §5),
    /// through the Editor API, so Unity's serializer is the only thing writing scene YAML.
    ///
    /// <para><b>What it contains:</b></para>
    /// <list type="bullet">
    /// <item>XRI's hands rig, with the controllers taken out of its modality manager so the scene is
    /// hands only.</item>
    /// <item>The rig's One Euro hand-smoothing post-processor switched off: it filters every hand joint,
    /// including the pinch pose that becomes the robot's target, which ADR 0016 rules out.</item>
    /// <item>The same arm rig, pivots and values as TeleopVR's <c>JetRoverControl</c>.</item>
    /// <item>The same <c>DragTarget</c> start pose, now grabbed by pinch.</item>
    /// <item><c>JetRoverOperatorBridge</c> and <c>CameraFeedBridge</c> wired to them.</item>
    /// <item>A world-locked camera panel and a status line.</item>
    /// <item>A plain floor and bench. The lab visuals come later.</item>
    /// </list>
    ///
    /// <para><b>The target's grab settings are safety settings, not taste.</b></para>
    /// <list type="bullet">
    /// <item><b>Far attach mode Far:</b> a far pinch must not pull the sphere to the hand, because the
    /// robot would jump with it.</item>
    /// <item><b>Dynamic attach:</b> grabbing must not snap the sphere's centre to the fingers.</item>
    /// <item><b>Instantaneous movement, no smoothing, no throw:</b> the target follows the pinch exactly
    /// and stops where it is released.</item>
    /// </list>
    /// <c>XrSceneCheck</c> fails the build if any of them changes.
    ///
    /// <para>Create-only, like TeleopVR's lab: once built, the scene is edited by hand in the editor,
    /// so this refuses to overwrite it unless run headless with <c>-teleopRebuildScene</c>.</para>
    /// </summary>
    public static class XrLabSceneBuilder
    {
        public const string ScenePath = "Assets/Scenes/JetRoverLabXR.unity";
        private const string MaterialFolder = "Assets/Teleop/Materials";
        private const string TextureFolder = "Assets/Teleop/Textures";
        private const string RebuildFlag = "-teleopRebuildScene";

        // Copied from TeleopVR's JetRoverControl.unity, so the IK sees the same arm in both projects.
        private static readonly Vector3 ArmBase = new Vector3(-0.014f, 1.045f, 0.328f);
        private static readonly Vector3 ShoulderOffset = new Vector3(-0.014f, 0.035f, 0f);
        private const float LinkLength = 0.13f;
        private const float LinkThickness = 0.03f;
        private static readonly Vector3 DragTargetStart = new Vector3(0.169f, 1.293f, 0.492f);
        private const float DragTargetDiameter = 0.05f;

        // Camera panel: 640x480 stream, 0.36 m tall; CameraFeedBridge places it beside the arm's base.
        private const float PanelHeight = 0.36f;
        private const float PanelAspect = 640f / 480f;

        [MenuItem("Teleop/XR/Build Lab Scene")]
        public static void BuildFromMenu()
        {
            if (File.Exists(ScenePath))
            {
                EditorUtility.DisplayDialog("Scene already exists",
                    $"{ScenePath} exists and is edited by hand now; the builder never overwrites it. " +
                    "Rename or delete it to build a fresh one.", "OK");
                return;
            }

            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
            {
                return;
            }

            Build();
        }

        /// <summary>Batch entry point: <c>-executeMethod Teleop.XR.Editor.XrLabSceneBuilder.Run</c>.</summary>
        public static void Run()
        {
            try
            {
                if (File.Exists(ScenePath))
                {
                    if (!Environment.GetCommandLineArgs().Contains(RebuildFlag))
                    {
                        Debug.Log($"[xr-scene] {ScenePath} exists; left as is (pass {RebuildFlag} to replace it).");
                        EditorApplication.Exit(0);
                        return;
                    }

                    AssetDatabase.DeleteAsset(ScenePath);
                }

                Build();
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[xr-scene] FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        private static void Build()
        {
            GameObject rigPrefab = FindHandsRigPrefab();
            XrProjectSetup.EnsureFolder("Assets/Scenes");
            XrProjectSetup.EnsureFolder(MaterialFolder);
            XrProjectSetup.EnsureFolder(TextureFolder);

            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            Material linkMat = LitMaterial("ArmLink", new Color(0.82f, 0.83f, 0.85f), 0.45f);
            Material targetMat = LitMaterial("DragTarget", new Color(0.1f, 0.75f, 0.95f), 0.6f);
            Material floorMat = LitMaterial("Floor", new Color(0.18f, 0.19f, 0.21f), 0.3f);
            Material benchMat = LitMaterial("Bench", new Color(0.12f, 0.13f, 0.15f), 0.4f);
            Material chassisMat = LitMaterial("Chassis", new Color(0.08f, 0.08f, 0.09f), 0.35f);
            Material panelMat = PanelMaterial();

            BuildLighting();
            GameObject rig = BuildHandsRig(rigPrefab);
            new GameObject("XR Interaction Manager").AddComponent<XRInteractionManager>();

            JetRoverArmRig armRig = BuildArmRig(linkMat, out Transform baseAnchor);
            XRGrabInteractable target = BuildDragTarget(targetMat);
            Renderer panel = BuildCameraPanel(panelMat);
            BuildRoom(floorMat, benchMat, chassisMat);

            var operatorObject = new GameObject("JetRoverOperator");
            var bridge = operatorObject.AddComponent<JetRoverOperatorBridge>();
            SetReference(bridge, "dragTarget", target.transform);
            SetReference(bridge, "armRig", armRig);

            var feed = operatorObject.AddComponent<CameraFeedBridge>();
            var feedSo = new SerializedObject(feed);
            feedSo.FindProperty("panel").objectReferenceValue = panel;
            feedSo.FindProperty("panelHeightMeters").floatValue = PanelHeight;
            feedSo.FindProperty("armRig").objectReferenceValue = armRig; // places the panel beside the arm's base at start
            feedSo.ApplyModifiedPropertiesWithoutUndo();

            operatorObject.AddComponent<HandTrackingSession>();
            BuildStatusLine(bridge);

            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            AssetDatabase.SaveAssets();
            Debug.Log($"[xr-scene] built {ScenePath} (rig: {AssetDatabase.GetAssetPath(rigPrefab)}); it is the only scene in Build Settings");
        }

        // ---- the rig ----------------------------------------------------------------------------

        private static GameObject FindHandsRigPrefab()
        {
            string path = AssetDatabase.FindAssets("t:Prefab XR Origin Hands", new[] { "Assets/Samples" })
                .Select(AssetDatabase.GUIDToAssetPath)
                .FirstOrDefault(p => Path.GetFileNameWithoutExtension(p) == "XR Origin Hands (XR Rig)");
            if (path == null)
            {
                throw new InvalidOperationException(
                    "XRI's 'XR Origin Hands (XR Rig)' prefab is not in Assets/Samples. Run `just unity-setup-xr` (twice on a fresh clone) first.");
            }

            return AssetDatabase.LoadAssetAtPath<GameObject>(path);
        }

        private static GameObject BuildHandsRig(GameObject prefab)
        {
            var rig = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            rig.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

            // Floor-level tracking, so eye height is the operator's real height and the bench sits where it should.
            XROrigin origin = rig.GetComponent<XROrigin>() ?? rig.GetComponentInChildren<XROrigin>(true);
            if (origin != null)
            {
                origin.RequestedTrackingOriginMode = XROrigin.TrackingOriginMode.Floor;
                EditorUtility.SetDirty(origin);
                if (origin.Camera != null)
                {
                    origin.Camera.clearFlags = CameraClearFlags.SolidColor;
                    origin.Camera.backgroundColor = new Color(0.07f, 0.075f, 0.09f);
                    origin.Camera.nearClipPlane = 0.02f;
                    EditorUtility.SetDirty(origin.Camera);
                }
            }

            // Hands only: without controller references the modality manager never switches to them.
            XRInputModalityManager modality = rig.GetComponentInChildren<XRInputModalityManager>(true);
            if (modality != null)
            {
                GameObject left = modality.leftController, right = modality.rightController;
                modality.leftController = null;
                modality.rightController = null;
                if (left != null) left.SetActive(false);
                if (right != null) right.SetActive(false);
                EditorUtility.SetDirty(modality);
            }

            // No smoothing of the operator's hand (ADR 0016 §5).
            foreach (MonoBehaviour behaviour in rig.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour != null && behaviour.GetType().Name == XrSceneCheck.HandSmoothingTypeName)
                {
                    behaviour.gameObject.SetActive(false);
                    EditorUtility.SetDirty(behaviour.gameObject);
                }
            }

            return rig;
        }

        // ---- the arm and its target -------------------------------------------------------------

        private static JetRoverArmRig BuildArmRig(Material linkMat, out Transform baseAnchor)
        {
            baseAnchor = new GameObject("BaseAnchor").transform;
            baseAnchor.position = ArmBase;

            var yaw = new GameObject("BaseYawPivot").transform;
            yaw.position = ArmBase;
            var rig = yaw.gameObject.AddComponent<JetRoverArmRig>();

            Transform shoulder = Child("ShoulderOffset", yaw, ShoulderOffset);
            Transform lower = Child("LowerPitchPivot", shoulder, Vector3.zero);
            Link("LowerSegment", lower, linkMat);
            Transform middle = Child("MiddlePitchPivot", lower, new Vector3(0f, 0f, LinkLength));
            Link("MiddleSegment", middle, linkMat);
            Transform upper = Child("UpperPitchPivot", middle, new Vector3(0f, 0f, LinkLength));
            Renderer upperLink = Link("UpperSegment", upper, linkMat);

            var so = new SerializedObject(rig);
            so.FindProperty("baseAnchor").objectReferenceValue = baseAnchor;
            so.FindProperty("baseYawPivot").objectReferenceValue = yaw;
            so.FindProperty("lowerPitchPivot").objectReferenceValue = lower;
            so.FindProperty("middlePitchPivot").objectReferenceValue = middle;
            so.FindProperty("upperPitchPivot").objectReferenceValue = upper;
            so.FindProperty("reachWarningRenderer").objectReferenceValue = upperLink; // _BaseColor turns red when clamped
            so.ApplyModifiedPropertiesWithoutUndo();
            return rig;
        }

        private static XRGrabInteractable BuildDragTarget(Material mat)
        {
            GameObject target = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            target.name = "DragTarget";
            target.transform.position = DragTargetStart;
            target.transform.localScale = Vector3.one * DragTargetDiameter;
            target.GetComponent<Renderer>().sharedMaterial = mat;

            var body = target.AddComponent<Rigidbody>();
            body.isKinematic = true;
            body.useGravity = false;

            var grab = target.AddComponent<XRGrabInteractable>();
            grab.movementType = XRBaseInteractable.MovementType.Instantaneous;
            grab.trackPosition = true;
            grab.trackRotation = true; // the wrist's pitch follows the hand's
            grab.trackScale = false;
            grab.throwOnDetach = false;
            grab.smoothPosition = false;
            grab.smoothRotation = false;
            grab.useDynamicAttach = true;
            grab.farAttachMode = InteractableFarAttachMode.Far;

            var highlight = target.AddComponent<GrabHighlight>();
            SetReference(highlight, "target", target.GetComponent<Renderer>());
            return grab;
        }

        // ---- camera panel, status, room -------------------------------------------------------

        private static Renderer BuildCameraPanel(Material mat)
        {
            GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
            quad.name = "CameraPanel";
            Object.DestroyImmediate(quad.GetComponent<Collider>());
            quad.transform.position = ArmBase + new Vector3(0.35f, 0.25f, 0f); // CameraFeedBridge's default offset
            quad.transform.localScale = new Vector3(PanelHeight * PanelAspect, PanelHeight, 1f);
            var renderer = quad.GetComponent<Renderer>();
            renderer.sharedMaterial = mat;
            renderer.shadowCastingMode = ShadowCastingMode.Off;
            return renderer;
        }

        private static void BuildStatusLine(JetRoverOperatorBridge bridge)
        {
            var go = new GameObject("ConnectionStatus");
            var text = go.AddComponent<TextMeshPro>();
            go.transform.position = ArmBase + new Vector3(-0.35f, 0.32f, 0.1f);
            text.text = "Robot: not connected";
            text.fontSize = 0.35f; // a 3D TextMeshPro line is about fontSize / 10 metres
            text.alignment = TextAlignmentOptions.Center;
            text.textWrappingMode = TextWrappingModes.NoWrap;
            text.rectTransform.sizeDelta = new Vector2(0.5f, 0.06f);

            var hud = go.AddComponent<JetRoverConnectionHud>();
            SetReference(hud, "operatorBridge", bridge);
            SetReference(hud, "label", text);
        }

        private static void BuildRoom(Material floorMat, Material benchMat, Material chassisMat)
        {
            var room = new GameObject("Room").transform;

            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.name = "Floor";
            floor.transform.SetParent(room, false);
            floor.transform.localScale = new Vector3(1.2f, 1f, 1.2f);
            floor.GetComponent<Renderer>().sharedMaterial = floorMat;
            Object.DestroyImmediate(floor.GetComponent<Collider>());

            // Placeholder chassis under the arm's base and a bench under it; the lab visuals replace both.
            const float chassisHeight = 0.075f, benchTop = 0.94f;
            Box("Chassis", room, new Vector3(ArmBase.x, ArmBase.y - chassisHeight / 2f, ArmBase.z - 0.03f),
                new Vector3(0.20f, chassisHeight, 0.28f), chassisMat);
            Box("Bench", room, new Vector3(ArmBase.x + 0.26f, benchTop / 2f, ArmBase.z + 0.21f),
                new Vector3(1.9f, benchTop, 0.8f), benchMat);
        }

        private static void BuildLighting()
        {
            var lightObject = new GameObject("Directional Light");
            var light = lightObject.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.1f;
            light.color = new Color(1f, 0.97f, 0.93f);
            light.shadows = LightShadows.Soft;
            lightObject.transform.rotation = Quaternion.Euler(62f, -25f, 0f);

            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = new Color(0.55f, 0.58f, 0.64f);
            RenderSettings.ambientEquatorColor = new Color(0.36f, 0.37f, 0.40f);
            RenderSettings.ambientGroundColor = new Color(0.18f, 0.18f, 0.2f);
            RenderSettings.skybox = null;
            RenderSettings.sun = light;
        }

        // ---- helpers ----------------------------------------------------------------------------

        private static Transform Child(string name, Transform parent, Vector3 localPosition)
        {
            var t = new GameObject(name).transform;
            t.SetParent(parent, false);
            t.localPosition = localPosition;
            return t;
        }

        private static Renderer Link(string name, Transform pivot, Material mat)
        {
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = name;
            Object.DestroyImmediate(cube.GetComponent<Collider>()); // nothing on the arm may push the target's rigidbody
            cube.transform.SetParent(pivot, false);
            cube.transform.localPosition = new Vector3(0f, 0f, LinkLength / 2f);
            cube.transform.localScale = new Vector3(LinkThickness, LinkThickness, LinkLength);
            var renderer = cube.GetComponent<Renderer>();
            renderer.sharedMaterial = mat;
            return renderer;
        }

        private static void Box(string name, Transform parent, Vector3 centre, Vector3 size, Material mat)
        {
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = name;
            Object.DestroyImmediate(cube.GetComponent<Collider>());
            cube.transform.SetParent(parent, false);
            cube.transform.position = centre;
            cube.transform.localScale = size;
            cube.GetComponent<Renderer>().sharedMaterial = mat;
        }

        private static void SetReference(Object component, string field, Object value)
        {
            var so = new SerializedObject(component);
            SerializedProperty property = so.FindProperty(field)
                ?? throw new InvalidOperationException($"{component.GetType().Name} has no serialized field '{field}'.");
            property.objectReferenceValue = value;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        private static Material LitMaterial(string name, Color color, float smoothness)
        {
            Material m = LoadOrCreateMaterial(name, "Universal Render Pipeline/Lit");
            m.SetColor("_BaseColor", color);
            m.SetFloat("_Smoothness", smoothness);
            EditorUtility.SetDirty(m);
            return m;
        }

        /// <summary>A material asset (never Shader.Find at runtime: IL2CPP would strip the shader).</summary>
        private static Material PanelMaterial()
        {
            Material m = LoadOrCreateMaterial("CameraPanel", "Universal Render Pipeline/Unlit");
            m.SetColor("_BaseColor", Color.white); // the camera image is multiplied by this
            m.mainTexture = NoSignalTexture();     // colour bars until CameraFeedBridge's first frame
            EditorUtility.SetDirty(m);
            return m;
        }

        private static Material LoadOrCreateMaterial(string name, string shaderName)
        {
            string path = $"{MaterialFolder}/{name}.mat";
            Material m = AssetDatabase.LoadAssetAtPath<Material>(path);
            Shader shader = Shader.Find(shaderName) ?? throw new InvalidOperationException($"Shader '{shaderName}' not found; is URP installed?");
            if (m == null)
            {
                m = new Material(shader);
                AssetDatabase.CreateAsset(m, path);
            }
            else
            {
                m.shader = shader;
            }

            return m;
        }

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
    }
}
