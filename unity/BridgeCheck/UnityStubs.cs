// Hand-written stand-ins for the Unity API surface (and for the Bridge adapters that are not
// themselves compiled here), so the real Bridge sources can be compiled headlessly against the
// real Core assemblies. See README.md in this folder before changing anything here.
//
// THE ONE RULE: a stub exists to make a Bridge file's Core usage compilable. It must never make a
// Bridge file compile in a way Unity would not. So:
//
//   - Copy signatures from the real thing (UnityEngine's documented API, or the real Bridge file
//     named in the comment), never from what happens to make the error go away. A stub whose
//     signature has drifted turns this gate into a source of false confidence, which is worse
//     than not having it -- root CLAUDE.md invariant 10's argument, applied to this file.
//   - Bodies are irrelevant and should stay trivial. Nothing here runs; only the signatures are
//     ever type-checked.
//   - Add only members that a compiled Bridge file actually touches. A speculative stub is an
//     unverified claim about Unity's API sitting in the repo forever.
//
// When `just bridge-check` fails on a missing member here rather than on a real disagreement with
// Core, that is the expected maintenance cost of the gate, not a defect. Add the member and move
// on. What it must never do is fail silently.
using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

namespace UnityEngine
{
    public class Object
    {
        public static void Destroy(Object obj) { }
    }

    public class Transform : Object
    {
        public Vector3 position { get; set; }
        public Vector3 localScale { get; set; }
        public Quaternion rotation { get; set; }
        public Vector3 InverseTransformPoint(Vector3 p) => default;
    }
    public struct Vector3
    {
        public float x, y, z;
        public Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public static Vector3 operator +(Vector3 a, Vector3 b) => default;
    }
    public struct Quaternion
    {
        public static Quaternion Inverse(Quaternion q) => default;
        public static Quaternion operator *(Quaternion a, Quaternion b) => default;
    }

    public class Component : Object
    {
        public Transform transform => null;
        public T GetComponent<T>() where T : class => null;
    }

    // CameraFeedBridge: texture decode and the panel it is drawn on.
    public class Texture : Object
    {
        public virtual int width { get; set; }
        public virtual int height { get; set; }
    }

    public enum TextureFormat { RGB24 = 3 }

    public sealed class Texture2D : Texture
    {
        public Texture2D(int width, int height, TextureFormat textureFormat, bool mipChain) { }
    }

    // UnityEngine.ImageConversion (ImageConversionModule): LoadImage is an extension method in 2022.3.
    public static class ImageConversion
    {
        public static bool LoadImage(this Texture2D tex, byte[] data, bool markNonReadable) => false;
        public static bool LoadImage(this Texture2D tex, byte[] data) => false;
    }

    public class Material : Object { public Texture mainTexture { get; set; } }
    public class Renderer : Component { public Material material { get; set; } }
    public class Behaviour : Component { public bool enabled { get; set; } }
    public class MonoBehaviour : Behaviour { }

    [AttributeUsage(AttributeTargets.Field)] public sealed class SerializeField : Attribute { }
    [AttributeUsage(AttributeTargets.Field)] public sealed class HeaderAttribute : Attribute { public HeaderAttribute(string h) { } }
    [AttributeUsage(AttributeTargets.Field)] public sealed class TooltipAttribute : Attribute { public TooltipAttribute(string t) { } }
    [AttributeUsage(AttributeTargets.Field)] public sealed class MinAttribute : Attribute { public MinAttribute(float m) { } }
    [AttributeUsage(AttributeTargets.Field)] public sealed class RangeAttribute : Attribute { public RangeAttribute(float a, float b) { } }

    public static class Mathf { public static bool Approximately(float a, float b) => a == b; }
    public static class Time { public static float unscaledTime => 0f; }

    public static class Debug
    {
        public static void Log(object message) { }
        public static void LogError(object message) { }
        public static void LogError(object message, Object context) { }
    }

    public static class Application
    {
        public static string persistentDataPath => string.Empty;
        public static event Action onBeforeRender { add { } remove { } }
    }

    public class TextAsset : Object { public string text => string.Empty; }

    public static class Resources
    {
        public static T Load<T>(string path) where T : Object => null;
    }
}

namespace Teleop.Bridge
{
    // Signatures copied from the real files, which are NOT compiled here.
    public sealed class UnityMonotonicClock : ITimeAuthority
    {
        public long TicksPerSecond => 10_000_000L;
        public long NowTicks => 0L;
    }

    public sealed class DisplayCalibrationConfig { }

    public static class ConfigLoader
    {
        public static DisplayCalibrationConfig Load() => null;
        public static T Load<T>(string resourceName, string overrideFileName, T fallback) where T : class => fallback;
    }

    public sealed class XrDisplayTimeProvider
    {
        public long GetDisplayOffsetTicks(ITimeAuthority clock, DisplayCalibrationConfig config) => 0L;
    }

    public sealed class UnityMetricSink : IMetricSink, IDisposable
    {
        public UnityMetricSink(int hudCapacity, string tlogPath, long ticksPerSecond, ulong sessionId) { }
        public void Record(string name, double value, long ticks) { }
        public void WriteLatencyTrace(LatencyTrace trace) { }
        public void Dispose() { }
    }

    public static class CoordConversion
    {
        public static System.Numerics.Vector3 ToCore(this UnityEngine.Vector3 v) => default;
        public static System.Numerics.Quaternion ToCore(this UnityEngine.Quaternion q) => default;
        public static Pose ToCorePose(this UnityEngine.Transform t) => Pose.Identity;
        public static void ApplyTo(this Pose p, UnityEngine.Transform t) { }
    }
}

namespace Teleop.Bridge
{
    // JetRoverArmRig: only the members JetRoverOperatorBridge touches.
    public sealed class JetRoverArmRig : UnityEngine.MonoBehaviour
    {
        public UnityEngine.Transform BaseAnchor => null;
        public void ApplyAngles(float baseYaw, float proximalPitch, float distalPitch, float upperPitch, bool wasClamped) { }
    }

    public sealed class UdpTransport : Teleop.Core.Contracts.ITransport
    {
        public UdpTransport(int localPort, System.Net.IPEndPoint remoteEndPoint, int maxPayloadBytes, Teleop.Core.Contracts.ITimeAuthority clock) { }
        public int MaxPayloadBytes => 128;
        public bool Send(System.ReadOnlySpan<byte> payload, long nowTicks) => true;
        public bool TryReceive(long nowTicks, System.Span<byte> destination, out int byteCount, out long arrivalTicks) { byteCount = 0; arrivalTicks = 0; return false; }
        public void Reset() { }
        public void Dispose() { }
    }
}
