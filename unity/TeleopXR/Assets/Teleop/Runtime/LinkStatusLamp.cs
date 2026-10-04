using System.Diagnostics;
using Teleop.Bridge;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Colours a set of renderers by the robot connection: the lab's floor ring around the workcell,
    /// and any other lamp that should say "is the robot listening" from across the room.
    /// <list type="bullet">
    /// <item>Connected: steady cyan.</item>
    /// <item>Stale (it was connected, and replies stopped): amber, blinking at 1 Hz.</item>
    /// <item>Never connected: dim red.</item>
    /// </list>
    /// Display only: reads <see cref="JetRoverOperatorBridge.Status"/>; recolours through a property
    /// block, both <c>_BaseColor</c> and <c>_Color</c>; times with <see cref="Stopwatch"/>; no
    /// allocation per frame.
    /// </summary>
    public sealed class LinkStatusLamp : MonoBehaviour
    {
        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        private static readonly int ColorId = Shader.PropertyToID("_Color");

        [SerializeField] private JetRoverOperatorBridge robot;
        [SerializeField] private Renderer[] lamps = new Renderer[0];
        [SerializeField] private Color connectedColor = new Color(0.1f, 0.85f, 1f);
        [SerializeField] private Color staleColor = new Color(1f, 0.6f, 0.08f);
        [SerializeField] private Color disconnectedColor = new Color(0.55f, 0.06f, 0.08f);

        private MaterialPropertyBlock _block;
        private Color _shown = new Color(-1f, -1f, -1f, -1f);

        private void Start()
        {
            if (robot == null) robot = FindAnyObjectByType<JetRoverOperatorBridge>();
            _block = new MaterialPropertyBlock();
            if (robot == null)
            {
                Apply(disconnectedColor);
                enabled = false;
            }
        }

        private void Update()
        {
            JetRoverOperatorBridge.ConnectionStatus status = robot.Status;
            Color color = status == JetRoverOperatorBridge.ConnectionStatus.Connected ? connectedColor
                : status == JetRoverOperatorBridge.ConnectionStatus.Stale
                    ? (Stopwatch.GetTimestamp() / (Stopwatch.Frequency / 2) % 2 == 0 ? staleColor : staleColor * 0.25f)
                    : disconnectedColor;
            if (color != _shown)
            {
                Apply(color);
            }
        }

        private void Apply(Color color)
        {
            _shown = color;
            foreach (Renderer lamp in lamps)
            {
                if (lamp == null) continue;
                lamp.GetPropertyBlock(_block);
                _block.SetColor(BaseColorId, color);
                _block.SetColor(ColorId, color);
                lamp.SetPropertyBlock(_block);
            }
        }
    }
}
