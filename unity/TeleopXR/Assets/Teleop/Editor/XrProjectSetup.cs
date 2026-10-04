using System;
using System.Collections.Generic;
using System.Linq;
using Google.XR.Extensions;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.PackageManager.UI;
using UnityEditor.XR.Management;
using UnityEditor.XR.Management.Metadata;
using UnityEditor.XR.OpenXR.Features;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.XR.Hands.OpenXR;
using UnityEngine.XR.Management;
using UnityEngine.XR.OpenXR;
using UnityEngine.XR.OpenXR.Features;
using UnityEngine.XR.OpenXR.Features.Android;
using UnityEngine.XR.OpenXR.Features.Interactions;
using PackageInfo = UnityEditor.PackageManager.PackageInfo;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// The Galaxy XR project's configuration, as code (docs/adr/0016 §4): a setting that only
    /// exists as a click in an Inspector cannot be reviewed, reproduced or rebuilt.
    /// <b>Teleop → XR → Configure Project</b>, or <c>just unity-setup-xr</c> headless.
    /// Idempotent: a second run changes nothing.
    ///
    /// <para>What it sets, and why each value:</para>
    /// <list type="bullet">
    /// <item><b>Samples:</b> XRI's Starter Assets and Hands Interaction Demo (the hands rig and the
    /// Android XR hand visuals), and XR Hands' HandVisualizer, which the demo needs. Importing a
    /// sample adds scripts, so a scene that uses them can only be built by a later run.</item>
    /// <item><b>URP:</b> HDR off and no post-processing (Google's Android XR guidance), 4× MSAA, a
    /// 4 m shadow distance (the scene is a bench), no depth or opaque copy textures.</item>
    /// <item><b>Android player:</b> Vulkan only, GameActivity, resizeable activity (both Android XR
    /// packages' validation rules), IL2CPP ARM64, .NET Standard 2.1 and Low stripping (as
    /// TeleopVR), Internet permission forced on (custom sockets; unity/CLAUDE.md), linear colour.</item>
    /// <item><b>OpenXR on Android:</b> Android XR Support, the Hand Tracking Subsystem and the Hand
    /// Interaction Profile, and nothing else. Every Android XR AR feature (planes, meshing,
    /// passthrough, faces) is left off: none is used, and each costs runtime work and permissions.</item>
    /// <item><b>OpenXR on Windows</b> (Direct Preview: Play mode streamed to the headset): the same,
    /// plus Android XR's session features and Google's Android XR Streaming feature.</item>
    /// <item><b>The Windows editor, for Direct Preview:</b> Vulkan only, multi-pass, 24-bit depth and
    /// the legacy foveation API. The streaming feature's Project Validation fails without the first
    /// three, and Google's setup guide asks for all four. They apply to Windows only; the Android
    /// build keeps single-pass instanced and URP foveation. The editor changes graphics API only
    /// when it restarts.</item>
    /// </list>
    /// </summary>
    public static class XrProjectSetup
    {
        private const string SettingsFolder = "Assets/Settings";
        private const string UrpAssetPath = SettingsFolder + "/TeleopXR_URP.asset";
        private const string UrpRendererPath = SettingsFolder + "/TeleopXR_URP_Renderer.asset";
        private const string BuiltinRendererPath = "Assets/UniversalRenderer.asset"; // where URP's LoadBuiltinRendererData writes
        private const string XrSettingsFolder = "Assets/XR";
        private const string OpenXRLoaderType = "UnityEngine.XR.OpenXR.OpenXRLoader";

        public const string ApplicationId = "com.teleop.jetroverxr";
        public const string ProductName = "TeleopXR";

        // Galaxy XR runs Android 14 (API 34). 29 sits above the Android XR package's own minimum
        // (26 on Unity 6000.5+, AndroidXRProjectValidationRules.cs) without excluding anything we need.
        private const AndroidSdkVersions MinSdk = AndroidSdkVersions.AndroidApiLevel29;

        private static readonly (string Package, string Sample)[] Samples =
        {
            ("com.unity.xr.interaction.toolkit", "Starter Assets"),
            ("com.unity.xr.hands", "HandVisualizer"),
            ("com.unity.xr.interaction.toolkit", "Hands Interaction Demo"),
        };

        private static readonly string[] AndroidFeatures =
        {
            AndroidXRSupportFeature.featureId,
            HandTracking.featureId2,
            HandInteractionProfile.featureId,
        };

        private static readonly string[] StandaloneFeatures =
        {
            AndroidXRSupportFeature.featureId,
            ARSessionFeature.featureId,
            XRSessionFeature.FeatureId,
            XRStreamingFeature.FeatureId,
            HandTracking.featureId2,
            HandInteractionProfile.featureId,
        };

        [MenuItem("Teleop/XR/Configure Project")]
        public static void ConfigureFromMenu()
        {
            int imported = ConfigureAll();
            if (imported > 0)
            {
                Debug.Log($"[xr-setup] imported {imported} sample(s); let Unity finish compiling, then build the scene.");
            }
        }

        /// <summary>Batch entry point: <c>-executeMethod Teleop.XR.Editor.XrProjectSetup.Run</c>.</summary>
        public static void Run()
        {
            try
            {
                int imported = ConfigureAll();
                if (imported > 0)
                {
                    Debug.Log($"[xr-setup] imported {imported} sample(s); they compile after this run, so build the scene in a later one.");
                }

                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[xr-setup] FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        private static int ConfigureAll()
        {
            int imported = ImportSamples() + ImportTextMeshProEssentials();
            ConfigureUrp();
            ConfigurePlayer();
            ConfigureXr(BuildTargetGroup.Android, AndroidFeatures);
            ConfigureXr(BuildTargetGroup.Standalone, StandaloneFeatures);
            OpenXRSettings.GetSettingsForBuildTargetGroup(BuildTargetGroup.Android).renderMode = OpenXRSettings.RenderMode.SinglePassInstanced;
            ConfigureDirectPreview();
            AssetDatabase.SaveAssets();
            Debug.Log("[xr-setup] PASS: samples, URP, Android player settings and OpenXR (Android + Windows/Direct Preview) configured");
            return imported;
        }

        /// <summary>
        /// The Windows editor settings Direct Preview needs. Google's <c>XRStreamingFeature</c>
        /// validation rules require an explicit Windows graphics API list, multi-pass and 24-bit depth.
        /// Google's guide asks for Vulkan only and the legacy foveation API.
        /// </summary>
        private static void ConfigureDirectPreview()
        {
            var vulkan = new[] { GraphicsDeviceType.Vulkan };
            if (PlayerSettings.GetUseDefaultGraphicsAPIs(BuildTarget.StandaloneWindows64)
                || !PlayerSettings.GetGraphicsAPIs(BuildTarget.StandaloneWindows64).SequenceEqual(vulkan))
            {
                PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.StandaloneWindows64, false);
                PlayerSettings.SetGraphicsAPIs(BuildTarget.StandaloneWindows64, vulkan);
            }

            OpenXRSettings editor = OpenXRSettings.GetSettingsForBuildTargetGroup(BuildTargetGroup.Standalone);
            editor.renderMode = OpenXRSettings.RenderMode.MultiPass;
            editor.depthSubmissionMode = OpenXRSettings.DepthSubmissionMode.Depth24Bit;
            editor.foveatedRenderingApi = OpenXRSettings.BackendFovationApi.Legacy;
            EditorUtility.SetDirty(editor);
        }

        private static int ImportSamples()
        {
            int imported = 0;
            foreach ((string package, string sampleName) in Samples)
            {
                PackageInfo info = PackageInfo.FindForPackageName(package)
                    ?? throw new InvalidOperationException($"{package} is not installed; check Packages/manifest.json.");
                Sample sample = Sample.FindByPackage(package, info.version).FirstOrDefault(s => s.displayName == sampleName);
                if (string.IsNullOrEmpty(sample.displayName))
                {
                    throw new InvalidOperationException($"{package} {info.version} has no sample named '{sampleName}'.");
                }

                if (sample.isImported)
                {
                    continue;
                }

                if (!sample.Import(Sample.ImportOptions.OverridePreviousImports))
                {
                    throw new InvalidOperationException($"Importing sample '{sampleName}' from {package} failed.");
                }

                Debug.Log($"[xr-setup] imported sample '{sampleName}' from {package} {info.version}");
                imported++;
            }

            if (imported > 0)
            {
                AssetDatabase.Refresh();
            }

            return imported;
        }

        /// <summary>
        /// TextMesh Pro's default font and settings live in <c>Assets/TextMesh Pro</c>, committed like
        /// TeleopVR's. On Unity 6 they come from com.unity.ugui's "TMP Essential Resources" package, which
        /// the editor offers to import the first time a TMP object appears. Batch mode never offers, and
        /// imports packages asynchronously after <c>-quit</c> has already exited, so this only checks.
        /// Without them every label in the lab renders as nothing.
        /// </summary>
        private static int ImportTextMeshProEssentials()
        {
            const string settings = "Assets/TextMesh Pro/Resources/TMP Settings.asset";
            if (AssetDatabase.LoadMainAssetAtPath(settings) == null)
            {
                throw new InvalidOperationException(
                    $"{settings} is missing. Restore Assets/TextMesh Pro from git, or in the editor run " +
                    "Window > TextMeshPro > Import TMP Essential Resources.");
            }

            return 0;
        }

        private static void ConfigureUrp()
        {
            EnsureFolder(SettingsFolder);
            var asset = AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>(UrpAssetPath);
            if (asset == null)
            {
                asset = UniversalRenderPipelineAsset.Create();
                AssetDatabase.CreateAsset(asset, UrpAssetPath);

                // URP's own way to build a correctly initialised renderer; it always writes to one
                // fixed path, so move the result next to the pipeline asset.
                asset.LoadBuiltinRendererData();
                string moveError = AssetDatabase.MoveAsset(BuiltinRendererPath, UrpRendererPath);
                if (!string.IsNullOrEmpty(moveError))
                {
                    throw new InvalidOperationException($"Could not move the URP renderer to {UrpRendererPath}: {moveError}");
                }
            }

            var renderer = AssetDatabase.LoadAssetAtPath<UniversalRendererData>(UrpRendererPath)
                ?? throw new InvalidOperationException($"{UrpRendererPath} is missing; delete {UrpAssetPath} and rerun.");
            renderer.postProcessData = null;
            EditorUtility.SetDirty(renderer);

            asset.supportsHDR = false;
            asset.msaaSampleCount = 4;
            asset.shadowDistance = 4f;
            asset.supportsCameraDepthTexture = false;
            asset.supportsCameraOpaqueTexture = false;
            EditorUtility.SetDirty(asset);

            GraphicsSettings.defaultRenderPipeline = asset;
        }

        private static void ConfigurePlayer()
        {
            PlayerSettings.companyName = "Teleop";
            PlayerSettings.productName = ProductName;
            PlayerSettings.colorSpace = ColorSpace.Linear;

            NamedBuildTarget android = NamedBuildTarget.Android;
            PlayerSettings.SetApplicationIdentifier(android, ApplicationId);
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.Android, false);
            PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, new[] { GraphicsDeviceType.Vulkan });
            PlayerSettings.Android.applicationEntry = AndroidApplicationEntry.GameActivity;
            PlayerSettings.Android.resizeableActivity = true;
            PlayerSettings.Android.minSdkVersion = MinSdk;
            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
            PlayerSettings.Android.forceInternetPermission = true;
            PlayerSettings.SetScriptingBackend(android, ScriptingImplementation.IL2CPP);
            PlayerSettings.SetApiCompatibilityLevel(android, ApiCompatibilityLevel.NET_Standard);
            PlayerSettings.SetManagedStrippingLevel(android, ManagedStrippingLevel.Low);
            PlayerSettings.SetApiCompatibilityLevel(NamedBuildTarget.Standalone, ApiCompatibilityLevel.NET_Standard);
        }

        private static void ConfigureXr(BuildTargetGroup group, IEnumerable<string> featureIds)
        {
            XRGeneralSettingsPerBuildTarget perTarget = GetOrCreateXrSettings();
            if (!perTarget.HasManagerSettingsForBuildTarget(group))
            {
                perTarget.CreateDefaultManagerSettingsForBuildTarget(group);
            }

            XRGeneralSettings general = perTarget.SettingsForBuildTarget(group);
            general.InitManagerOnStart = true;
            if (!general.Manager.activeLoaders.Any(l => l != null && l.GetType().FullName == OpenXRLoaderType))
            {
                if (!XRPackageMetadataStore.AssignLoader(general.Manager, OpenXRLoaderType, group))
                {
                    throw new InvalidOperationException($"Could not assign the OpenXR loader for {group}.");
                }
            }

            EditorUtility.SetDirty(general);
            EditorUtility.SetDirty(general.Manager);

            foreach (string id in featureIds)
            {
                OpenXRFeature feature = FeatureHelpers.GetFeatureWithIdForBuildTarget(group, id)
                    ?? throw new InvalidOperationException($"OpenXR feature '{id}' does not exist for {group}; is its package installed?");
                if (!feature.enabled)
                {
                    feature.enabled = true;
                    EditorUtility.SetDirty(feature);
                }
            }
        }

        private static XRGeneralSettingsPerBuildTarget GetOrCreateXrSettings()
        {
            if (EditorBuildSettings.TryGetConfigObject(XRGeneralSettings.settingsKey, out XRGeneralSettingsPerBuildTarget existing) && existing != null)
            {
                return existing;
            }

            EnsureFolder(XrSettingsFolder);
            var created = ScriptableObject.CreateInstance<XRGeneralSettingsPerBuildTarget>();
            AssetDatabase.CreateAsset(created, XrSettingsFolder + "/XRGeneralSettingsPerBuildTarget.asset");
            EditorBuildSettings.AddConfigObject(XRGeneralSettings.settingsKey, created, true);
            return created;
        }

        internal static void EnsureFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path))
            {
                return;
            }

            int slash = path.LastIndexOf('/');
            string parent = path.Substring(0, slash);
            EnsureFolder(parent);
            AssetDatabase.CreateFolder(parent, path.Substring(slash + 1));
        }
    }
}
