using System.Diagnostics;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Set dressing: a handful of small lamps (server-rack activity LEDs) that flicker on and off, so
    /// the room reads as a working lab rather than a photograph. Each lamp keeps its own schedule
    /// from a fixed seed, so the pattern is the same every run. Times with <see cref="Stopwatch"/>
    /// and recolours through a property block; no allocation per frame. Carries no information:
    /// nothing in the scene depends on it.
    /// </summary>
    public sealed class BlinkingLights : MonoBehaviour
    {
        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        private static readonly int ColorId = Shader.PropertyToID("_Color");

        [SerializeField] private Renderer[] lamps = new Renderer[0];
        [SerializeField] private Color[] onColors = { new Color(0.2f, 1f, 0.45f), new Color(0.25f, 0.6f, 1f), new Color(1f, 0.7f, 0.15f) };
        [SerializeField] private int seed = 7;

        private MaterialPropertyBlock _block;
        private long[] _nextToggle;
        private bool[] _on;
        private System.Random _random;

        private void Start()
        {
            _block = new MaterialPropertyBlock();
            _random = new System.Random(seed);
            _nextToggle = new long[lamps.Length];
            _on = new bool[lamps.Length];
            long now = Stopwatch.GetTimestamp();
            for (int i = 0; i < lamps.Length; i++)
            {
                _on[i] = _random.NextDouble() < 0.5;
                _nextToggle[i] = now + NextInterval();
                Apply(i);
            }
        }

        private void Update()
        {
            long now = Stopwatch.GetTimestamp();
            for (int i = 0; i < lamps.Length; i++)
            {
                if (now < _nextToggle[i]) continue;
                _on[i] = !_on[i];
                _nextToggle[i] = now + NextInterval();
                Apply(i);
            }
        }

        private long NextInterval() => (long)((0.05 + _random.NextDouble() * 0.6) * Stopwatch.Frequency);

        private void Apply(int i)
        {
            Renderer lamp = lamps[i];
            if (lamp == null) return;
            Color on = onColors.Length > 0 ? onColors[i % onColors.Length] : Color.green;
            Color color = _on[i] ? on : on * 0.08f;
            lamp.GetPropertyBlock(_block);
            _block.SetColor(BaseColorId, color);
            _block.SetColor(ColorId, color);
            lamp.SetPropertyBlock(_block);
        }
    }
}
