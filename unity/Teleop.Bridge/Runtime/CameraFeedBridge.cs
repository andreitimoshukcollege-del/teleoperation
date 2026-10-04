using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Teleop.Core.Camera;
using UnityEngine;
using Debug = UnityEngine.Debug;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Bridge
{
    /// <summary>
    /// Shows the JetRover's camera on a world-locked panel beside the arm proxy
    /// (docs/adr/0014-camera-frame-downlink.md §6, §7, resolved question 2).
    ///
    /// <para><b>Threads.</b> A background thread owns the socket. It sends the keepalive once a second,
    /// stamps each datagram's arrival at dequeue (docs/metrics.md §1 warns what stamping on the main
    /// thread does), and feeds Core's <see cref="CameraFrameReassembler"/>. Each completed frame is
    /// handed to the main thread, newest wins. The main thread decodes it with
    /// <c>Texture2D.LoadImage</c>, which is Unity's only built-in JPEG decoder and runs on the main
    /// thread; its cost is measured and logged (<see cref="LastDecodeMilliseconds"/>), because at 90 Hz
    /// on the Quest it is the stage most likely to matter.</para>
    ///
    /// <para><b>Scene setup (human, in the editor).</b> Add a Quad with an unlit material (built-in
    /// pipeline: <c>Unlit/Texture</c>) and assign its Renderer to <see cref="panel"/>. Assign the arm
    /// rig to place the panel beside the arm's base at start, or leave it unassigned to keep the
    /// panel where the scene puts it. The panel is never parented to the camera: video that moves
    /// with the head while lagging the world is a sensory conflict (ADR 0014 §7).</para>
    ///
    /// <para><b>Not here yet.</b> No <c>camera_*</c> metrics: those need the pose path's ClockSync to
    /// put capture and arrival on one clock (ADR 0014 §5, §9) and are defined in docs/metrics.md by the
    /// PR that emits them. This component shows the feed and reports arrival-side counts only.</para>
    /// </summary>
    public sealed class CameraFeedBridge : MonoBehaviour
    {
        private const int MaxFrameBytes = 256 * 1024;
        private const float LogIntervalSeconds = 10f;

        [Header("Panel: a Quad with an unlit material, world-locked (ADR 0014 §7)")]
        [SerializeField] private Renderer panel;
        [Tooltip("Height of the panel in metres; width follows the camera's aspect ratio.")]
        [SerializeField] private float panelHeightMeters = 0.24f;

        [Header("Placement beside the arm (optional)")]
        [SerializeField] private JetRoverArmRig armRig;
        [Tooltip("World-space offset from the arm's base anchor, applied once at start.")]
        [SerializeField] private Vector3 offsetFromArmBase = new Vector3(0.35f, 0.25f, 0f);

        [Header("Network (robot address comes from jetrover_connection, like JetRoverOperatorBridge)")]
        [SerializeField] private int robotPort = 6003;
        [SerializeField] private int localPort = 6004;
        [Tooltip("0 asks for every frame the camera produces; lower it if decoding costs too much frame time.")]
        [SerializeField] private int maxFramesPerSecond = 0;
        [Tooltip("Consecutive late chunks before Core's reassembler resyncs, which recovers from a sender restart (Teleop.Core/Camera/CLAUDE.md). 0 never resyncs.")]
        [SerializeField] private int lateChunksBeforeResync = 40;

        private readonly object _gate = new object();
        private Socket _socket;
        private Thread _thread;
        private volatile bool _running;
        private IPEndPoint _robot;

        // Written by the network thread under _gate, read by the main thread.
        private byte[] _pendingJpeg;
        private CameraFrameInfo _pendingInfo;
        private CameraReassemblerDiagnostics _lastDiagnostics;
        private long _framesReceived;
        private long _framesSkippedBeforeDecode;

        private Texture2D _texture;
        private long _framesShown;
        private long _nextLogTicks;

        /// <summary>Frames decoded and shown since enable.</summary>
        public long FramesShown => _framesShown;

        /// <summary>Main-thread cost of the most recent <c>LoadImage</c>, in milliseconds.</summary>
        public double LastDecodeMilliseconds { get; private set; }

        /// <summary>The most recent frame shown; its stamp is in the robot's clock domain.</summary>
        public CameraFrameInfo LastFrame { get; private set; }

        private void Start()
        {
            if (panel == null)
            {
                Debug.LogError("CameraFeedBridge needs a panel Renderer (a Quad with an unlit material).", this);
                enabled = false;
                return;
            }

            if (armRig != null && armRig.BaseAnchor != null)
            {
                panel.transform.position = armRig.BaseAnchor.position + offsetFromArmBase;
            }

            // Not assigned to the panel until the first frame decodes, so whatever the scene shows
            // there (a "no signal" card, say) stays up until there is a picture to replace it.
            _texture = new Texture2D(2, 2, TextureFormat.RGB24, false);
        }

        private void OnEnable()
        {
            JetRoverArmConfig config = ConfigLoader.Load("jetrover_connection", "jetrover_connection.json", new JetRoverArmConfig());
            _robot = new IPEndPoint(IPAddress.Parse(config.RemoteHost), robotPort);

            _socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            _socket.ReceiveBufferSize = 4 << 20;
            _socket.ReceiveTimeout = 100;
            _socket.Bind(new IPEndPoint(IPAddress.Any, localPort));

            _running = true;
            _thread = new Thread(ReceiveLoop) { IsBackground = true, Name = "CameraFeed receive" };
            _thread.Start();
            _nextLogTicks = Stopwatch.GetTimestamp() + (long)(LogIntervalSeconds * Stopwatch.Frequency);
        }

        private void OnDisable()
        {
            _running = false;
            Socket socket = _socket;
            _socket = null;
            if (socket != null)
            {
                socket.Close();
            }

            if (_thread != null)
            {
                _thread.Join(500);
                _thread = null;
            }
        }

        private void OnDestroy()
        {
            if (_texture != null)
            {
                Destroy(_texture);
            }
        }

        private void Update()
        {
            byte[] jpeg;
            CameraFrameInfo info;
            lock (_gate)
            {
                jpeg = _pendingJpeg;
                info = _pendingInfo;
                _pendingJpeg = null;
            }

            if (jpeg != null && _texture != null)
            {
                long started = Stopwatch.GetTimestamp();
                bool decoded = _texture.LoadImage(jpeg, false);
                LastDecodeMilliseconds = (Stopwatch.GetTimestamp() - started) * 1000.0 / Stopwatch.Frequency;

                if (decoded)
                {
                    if (_framesShown == 0)
                    {
                        panel.material.mainTexture = _texture;
                        if (info.Frame.Height > 0)
                        {
                            float aspect = info.Frame.Width / (float)info.Frame.Height;
                            panel.transform.localScale = new Vector3(panelHeightMeters * aspect, panelHeightMeters, 1f);
                        }
                    }

                    _framesShown++;
                    LastFrame = info;
                }
            }

            long now = Stopwatch.GetTimestamp();
            if (now >= _nextLogTicks)
            {
                _nextLogTicks = now + (long)(LogIntervalSeconds * Stopwatch.Frequency);
                LogSummary();
            }
        }

        /// <summary>Background thread: keepalive, receive, reassemble, hand over the newest frame.</summary>
        private void ReceiveLoop()
        {
            // A fresh reassembler per subscription, so a sender that restarted while this viewer was
            // disabled is not judged against old frame ids; one that restarts mid-stream is caught by
            // Core's resync rule.
            var reassembler = new CameraFrameReassembler(
                slotCount: 3, maxFrameBytes: MaxFrameBytes, lateChunksBeforeResync: Math.Max(0, lateChunksBeforeResync));
            var subscribeCodec = new CameraSubscribeCodec();
            var keepalive = new byte[CameraSubscribeCodec.EncodedSize];
            var datagram = new byte[CameraChunkCodec.MaxDatagramBytes];
            long nextKeepalive = 0;

            while (_running)
            {
                Socket socket = _socket;
                if (socket == null)
                {
                    break;
                }

                long now = Stopwatch.GetTimestamp();
                if (now >= nextKeepalive)
                {
                    subscribeCodec.TryEncode((ushort)Math.Max(0, Math.Min(ushort.MaxValue, maxFramesPerSecond)), now, keepalive, out int keepaliveBytes);
                    try
                    {
                        socket.SendTo(keepalive, 0, keepaliveBytes, SocketFlags.None, _robot);
                    }
                    catch (SocketException)
                    {
                        // Unreachable for now (no network yet, robot down): keep trying every second.
                    }
                    catch (ObjectDisposedException)
                    {
                        break;
                    }

                    nextKeepalive = now + Stopwatch.Frequency;
                }

                int received;
                try
                {
                    received = socket.Receive(datagram);
                }
                catch (SocketException)
                {
                    continue; // the 100 ms receive timeout, or a transient error
                }
                catch (ObjectDisposedException)
                {
                    break;
                }

                long arrival = Stopwatch.GetTimestamp(); // at dequeue, per docs/metrics.md §1
                CameraChunkOutcome outcome = reassembler.Accept(new ReadOnlySpan<byte>(datagram, 0, received), arrival);
                if (outcome != CameraChunkOutcome.CompletedFrame ||
                    !reassembler.TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> jpeg))
                {
                    continue;
                }

                // LoadImage takes a whole array, so each frame needs its own (about 43 KB at 30 fps).
                byte[] copy = jpeg.ToArray();
                lock (_gate)
                {
                    if (_pendingJpeg != null)
                    {
                        _framesSkippedBeforeDecode++; // the main thread had not shown the previous one yet
                    }

                    _pendingJpeg = copy;
                    _pendingInfo = info;
                    _framesReceived++;
                    _lastDiagnostics = reassembler.Diagnostics;
                }
            }
        }

        private void LogSummary()
        {
            long received, skipped;
            CameraReassemblerDiagnostics d;
            lock (_gate)
            {
                received = _framesReceived;
                skipped = _framesSkippedBeforeDecode;
                d = _lastDiagnostics;
            }

            Debug.Log(
                $"[camera] {_robot}: received {received} frames, shown {_framesShown}, skipped before decode {skipped}, " +
                $"last decode {LastDecodeMilliseconds:0.0} ms; reassembler completed {d.FramesCompleted}, " +
                $"incomplete {d.DroppedIncomplete}, superseded {d.DroppedSuperseded}, late {d.LateChunks}, resyncs {d.Resyncs}");
        }
    }
}
