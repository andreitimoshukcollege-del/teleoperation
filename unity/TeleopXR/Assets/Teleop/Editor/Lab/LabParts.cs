using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>How a part takes part in lighting.</summary>
    internal enum PartKind
    {
        /// <summary>Never moves: lightmapped, static-batched, seen by the reflection probe, casts baked shadows.</summary>
        Static,

        /// <summary>Moves, or might (the arm, the display): lit by light probes, casts real-time shadows.</summary>
        Dynamic,

        /// <summary>Gives off light whose colour changes at runtime (the status ring, LEDs, the camera screen): unlit, no shadows, not in the bake.</summary>
        Emitter,

        /// <summary>
        /// Gives off fixed light and never moves (cove lines, fixture diffusers, the skyline): like
        /// <see cref="Emitter"/>, but seen by the baked reflection probe, so the glossy floor reflects
        /// it. A runtime colour must never be baked into that reflection, which is why the status ring
        /// and the LEDs are not this.
        /// </summary>
        Glow,
    }

    /// <summary>Turns a <see cref="LabMesh"/> and a material into a scene object, consistently.</summary>
    internal static class LabParts
    {
        public static GameObject Group(string name, Transform parent)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            return go;
        }

        /// <summary>
        /// A renderer for <paramref name="shape"/>, stored in the mesh library as
        /// <paramref name="meshName"/>, which must be unique across the lab. The shape's coordinates
        /// are in <paramref name="parent"/>'s space.
        /// </summary>
        public static MeshRenderer Part(string meshName, Transform parent, LabMesh shape, Material material,
            PartKind kind, float lightmapScale = 1f)
        {
            string objectName = meshName.Substring(meshName.LastIndexOf('/') + 1);
            var go = new GameObject(objectName);
            go.transform.SetParent(parent, false);
            go.AddComponent<MeshFilter>().sharedMesh = LabMeshLibrary.Store(meshName, shape, kind == PartKind.Static);
            var renderer = go.AddComponent<MeshRenderer>();
            renderer.sharedMaterial = material;
            Configure(renderer, kind, lightmapScale);
            return renderer;
        }

        public static void Configure(Renderer renderer, PartKind kind, float lightmapScale = 1f)
        {
            GameObject go = renderer.gameObject;
            switch (kind)
            {
                case PartKind.Static:
                    GameObjectUtility.SetStaticEditorFlags(go,
                        StaticEditorFlags.ContributeGI | StaticEditorFlags.BatchingStatic | StaticEditorFlags.ReflectionProbeStatic);
                    renderer.shadowCastingMode = ShadowCastingMode.On;
                    if (renderer is MeshRenderer mr)
                    {
                        mr.receiveGI = ReceiveGI.Lightmaps;
                        mr.scaleInLightmap = lightmapScale;
                    }

                    break;
                case PartKind.Dynamic:
                    GameObjectUtility.SetStaticEditorFlags(go, 0);
                    renderer.shadowCastingMode = ShadowCastingMode.On;
                    renderer.lightProbeUsage = LightProbeUsage.BlendProbes;
                    renderer.reflectionProbeUsage = ReflectionProbeUsage.BlendProbes;
                    break;
                case PartKind.Emitter:
                case PartKind.Glow:
                    GameObjectUtility.SetStaticEditorFlags(go, kind == PartKind.Glow
                        ? StaticEditorFlags.BatchingStatic | StaticEditorFlags.ReflectionProbeStatic
                        : 0);
                    renderer.shadowCastingMode = ShadowCastingMode.Off;
                    renderer.receiveShadows = false;
                    renderer.lightProbeUsage = LightProbeUsage.Off;
                    renderer.reflectionProbeUsage = ReflectionProbeUsage.Off;
                    break;
            }
        }

        /// <summary>A 3D TextMeshPro label; a line is about <paramref name="fontSize"/> / 10 metres tall.</summary>
        public static TextMeshPro Label(string name, Transform parent, Vector3 localPosition, Quaternion localRotation,
            string text, float fontSize, Color color, Vector2 size, TextAlignmentOptions alignment = TextAlignmentOptions.Center,
            FontStyles style = FontStyles.Normal)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPosition;
            go.transform.localRotation = localRotation;
            var tmp = go.AddComponent<TextMeshPro>();
            tmp.text = text;
            tmp.fontSize = fontSize;
            tmp.color = color;
            tmp.fontStyle = style;
            tmp.alignment = alignment;
            tmp.textWrappingMode = TextWrappingModes.NoWrap;
            tmp.rectTransform.sizeDelta = size;
            go.GetComponent<MeshRenderer>().shadowCastingMode = ShadowCastingMode.Off;
            return tmp;
        }
    }
}
