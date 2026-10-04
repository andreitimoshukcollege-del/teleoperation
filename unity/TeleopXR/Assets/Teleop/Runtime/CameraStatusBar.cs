using System.Diagnostics;
using Teleop.Bridge;
using TMPro;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// The status strip under the lab's camera display: is the feed live, how fast is it arriving,
    /// what does a decode cost, and is the robot connected.
    ///
    /// <para><b>Display only.</b> It reads <see cref="CameraFeedBridge"/>'s public counters and
    /// <see cref="JetRoverOperatorBridge.Status"/> four times a second and writes text and two lamp
    /// colours. It never touches the stream or the robot.</para>
    ///
    /// <para><b>The numbers are counts, not metrics.</b></para>
    /// <list type="bullet">
    /// <item>fps is frames shown over the last whole second or more.</item>
    /// <item>decode is <c>LastDecodeMilliseconds</c>, the <c>LoadImage</c> time.</item>
    /// </list>
    /// <para>No smoothing coefficient, nothing recorded. Camera latency belongs to Bridge's
    /// <c>camera_*</c> metrics (docs/metrics.md §9), not to a display. It reads only Bridge types:
    /// this assembly never references Core (unity/CLAUDE.md).</para>
    ///
    /// <para>Timing uses <see cref="Stopwatch"/>, never <c>Time.time</c> (unity/CLAUDE.md). After
    /// <c>Start</c> nothing allocates: TMP's numeric <c>SetText</c> overloads are allocation-free in
    /// players, and the fixed strings are set only when the state changes.</para>
    /// </summary>
    public sealed class CameraStatusBar : MonoBehaviour
    {
        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        private static readonly int ColorId = Shader.PropertyToID("_Color");

        [SerializeField] private CameraFeedBridge feed;
        [SerializeField] private JetRoverOperatorBridge robot;
        [SerializeField] private Renderer screen;
        [SerializeField] private TMP_Text feedLabel;
        [SerializeField] private TMP_Text numbersLabel;
        [SerializeField] private TMP_Text linkLabel;
        [SerializeField] private Renderer liveDot;

        [SerializeField] private Color liveColor = new Color(1f, 0.18f, 0.2f);
        [SerializeField] private Color idleColor = new Color(0.3f, 0.32f, 0.36f);

        private const double RefreshSeconds = 0.25;
        private const double FrozenAfterSeconds = 1.0;

        private MaterialPropertyBlock _block;
        private long _nextRefresh;
        private long _windowStartTicks;
        private long _windowStartFrames;
        private long _lastFrames;
        private long _lastNewFrameTicks;
        private float _fps;
        private int _feedState = -1;
        private int _linkState = -1;
        private bool _dotOn;

        private void Start()
        {
            if (feed == null) feed = FindAnyObjectByType<CameraFeedBridge>();
            if (robot == null) robot = FindAnyObjectByType<JetRoverOperatorBridge>();
            if (feed == null || robot == null)
            {
                UnityEngine.Debug.LogWarning("CameraStatusBar: no CameraFeedBridge or JetRoverOperatorBridge in the scene; status strip disabled.", this);
                enabled = false;
                return;
            }

            _block = new MaterialPropertyBlock();
            long now = Stopwatch.GetTimestamp();
            _windowStartTicks = now;
            _lastNewFrameTicks = now;
        }

        private void Update()
        {
            long now = Stopwatch.GetTimestamp();
            if (now < _nextRefresh)
            {
                return;
            }

            _nextRefresh = now + (long)(RefreshSeconds * Stopwatch.Frequency);

            long frames = feed.FramesShown;
            if (frames != _lastFrames)
            {
                _lastFrames = frames;
                _lastNewFrameTicks = now;
            }

            double windowSeconds = (now - _windowStartTicks) / (double)Stopwatch.Frequency;
            if (windowSeconds >= 1.0)
            {
                _fps = (float)((frames - _windowStartFrames) / windowSeconds);
                _windowStartFrames = frames;
                _windowStartTicks = now;
            }

            // 0 no signal yet, 1 live, 2 frozen (frames came, then stopped), 3 feed component off.
            int feedState = !feed.isActiveAndEnabled ? 3
                : frames == 0 ? 0
                : (now - _lastNewFrameTicks) / (double)Stopwatch.Frequency > FrozenAfterSeconds ? 2
                : 1;

            if (feedState != _feedState)
            {
                _feedState = feedState;
                if (feedLabel != null)
                {
                    feedLabel.text = feedState == 1 ? "LIVE  ·  JETROVER CAM"
                        : feedState == 2 ? "FROZEN  ·  no new frames"
                        : feedState == 3 ? "FEED OFF"
                        : "NO SIGNAL  ·  waiting for the robot camera";
                }
            }

            if (numbersLabel != null)
            {
                Texture texture = screen != null ? screen.sharedMaterial.mainTexture : null;
                int width = feedState == 0 || texture == null ? 0 : texture.width;
                int height = feedState == 0 || texture == null ? 0 : texture.height;
                numbersLabel.SetText("{0:0} fps   decode {1:1} ms   {2:0}x{3:0}",
                    feedState == 1 ? _fps : 0f, (float)feed.LastDecodeMilliseconds, width, height);
            }

            JetRoverOperatorBridge.ConnectionStatus status = robot.Status;
            int linkState = (int)status;
            if (linkState != _linkState && linkLabel != null)
            {
                _linkState = linkState;
                linkLabel.text = status == JetRoverOperatorBridge.ConnectionStatus.Connected ? "ROBOT CONNECTED"
                    : status == JetRoverOperatorBridge.ConnectionStatus.Stale ? "ROBOT CONNECTION LOST"
                    : "ROBOT NOT CONNECTED";
            }

            // The LIVE dot blinks while frames arrive, like a recording light, and is grey otherwise.
            _dotOn = feedState == 1 && !_dotOn;
            if (liveDot != null)
            {
                Color color = feedState == 1 ? (_dotOn ? liveColor : liveColor * 0.35f) : idleColor;
                liveDot.GetPropertyBlock(_block);
                _block.SetColor(BaseColorId, color);
                _block.SetColor(ColorId, color);
                liveDot.SetPropertyBlock(_block);
            }
        }
    }
}
