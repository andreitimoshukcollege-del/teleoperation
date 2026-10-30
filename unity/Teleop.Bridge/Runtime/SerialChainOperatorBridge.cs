using System;
using System.Collections.Generic;
using System.Net;
using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using UnityEngine;
using CoreQuat = System.Numerics.Quaternion;
using CoreVec = System.Numerics.Vector3;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>Where one arm's clutch stands, Unity-free so TeleopXR's display code can read it (it never touches RobotArm).</summary>
    public enum DuoArmState
    {
        Idle,
        Following,
        StoppedShort,
        Blocked,
    }

    /// <summary>
    /// The FR3 Duo's operator (docs/adr/0029): each hand drives its own arm. A thin adapter. Every decision is
    /// <c>Teleop.RobotArm</c>'s: <see cref="GrabGesture"/> (a curled hand holds, the thumb-index gap is the gripper),
    /// <see cref="SerialChainTeleop"/> (clutch, IK, collision, the command) and <see cref="SerialChainJointLink"/>
    /// (keepalive, ClockSync, the predictor bank). This converts the hands into the robot's root frame, owns the
    /// socket, sends the command at <see cref="SerialChainConnectionConfig.CommandRateHz"/>, and poses the twin and
    /// the target markers.
    /// <list type="bullet">
    /// <item><b>Update</b> (after TeleopXR's hand driver has called <see cref="SetHand"/>): drain the link, take the
    /// robot's pose once, step the clutches, send.</item>
    /// <item><b>onBeforeRender</b>: the twin from the active predictor's estimate for now, else the newest frame.</item>
    /// </list>
    /// <b>Motion is off at launch</b> (<see cref="MotionEnabled"/>): nothing but keepalives goes until the operator
    /// switches it on, and switching it on first takes the robot's measured pose as the command and lets go of both
    /// clutches, so the arms never jump to wherever the hands were while it was off.
    /// </summary>
    [DefaultExecutionOrder(-40)]
    public sealed class SerialChainOperatorBridge : MonoBehaviour
    {
        private const string ConfigResource = "fr3_connection";
        private const string ProfileResource = "fr3_duo_profile";
        private const string CollisionResource = "fr3_duo_collision";

        [Tooltip("The robot's root frame in the scene: the URDF's base link, where the twin is built.")]
        [SerializeField] private Transform robotRoot;
        [SerializeField] private SerialChainArmRig twin;
        [Tooltip("One marker per arm (left, right): where its clutch asks the tool to be.")]
        [SerializeField] private Transform[] targetMarkers = new Transform[0];
        [SerializeField] private bool motionEnabledAtStart;

        private static readonly Color IdleColor = new Color(0.55f, 0.6f, 0.7f);
        private static readonly Color FollowingColor = new Color(0.2f, 0.75f, 1f);
        private static readonly Color ShortColor = new Color(1f, 0.6f, 0.1f);
        private static readonly Color BlockedColor = new Color(1f, 0.2f, 0.2f);

        private SerialChainConnectionConfig _config;
        private SerialChainProfile _profile;
        private UnityMonotonicClock _clock;
        private UdpTransport _udp;
        private SerialChainJointLink _link;
        private SerialChainTeleop _teleop;
        private GrabGestureConfig _grabConfig;
        private BodyAnchoredMapping _mapping;
        private GrabGesture[] _grab;
        private GrabToggleConfig _toggleConfig;
        private GrabToggle[] _toggle;
        private JointTarget[] _targets;
        private readonly byte[] _buffer = new byte[JointDatagram.MaxBytes];
        private long _sendIntervalTicks, _lastSendTicks = long.MinValue;
        private uint _sequence;
        private int _left = -1, _right = -1;
        private bool _motion;
        private MaterialPropertyBlock _markerColor;
        private Renderer[] _markerRenderers;
        private int[] _shownMarkerState;

        public bool IsReady => _link != null;

        /// <summary>Whether commands go to the robot. On takes the robot's measured pose first, and fails (stays off) before one has arrived.</summary>
        public bool MotionEnabled
        {
            get => _motion;
            set
            {
                if (value == _motion) return;
                if (!value || _link == null)
                {
                    _motion = false;
                    if (_link != null) _link.MotionOn = false;
                    return;
                }

                if (!_config.AllowMotion)
                {
                    Debug.LogWarning("[fr3] motion stays off: AllowMotion is false in fr3_connection");
                    return;
                }

                if (!_teleop.TryTakePose(_link.Measured))
                {
                    Debug.LogWarning("[fr3] motion stays off: no complete joint-state frame from the robot yet");
                    return;
                }

                ResetGrabs();
                // Where you stand when you switch on is where the robot is anchored to you.
                Recentre();
                _motion = true;
                _link.MotionOn = true;
            }
        }

        public bool MotionAllowed => _config != null && _config.AllowMotion;

        /// <summary>Whether the hands map absolutely (the robot anchored to your body), rather than through the relative clutch.</summary>
        public bool IsAbsolute => _mapping != null;

        /// <summary>
        /// Anchors the robot to your body as you stand now (docs/adr/0029 amendment): the headset's position and heading
        /// become the body frame, and your neutral hands the arms' neutral tools. Also done when motion is switched on.
        /// </summary>
        public void Recentre()
        {
            if (_mapping == null) return;
            Camera eye = Camera.main;
            if (eye == null) return;
            Transform head = eye.transform;
            _mapping.Recentre(ToWorld(head.position, head.rotation));
            Debug.Log("[fr3] recentred: the robot is anchored to where you stand and face");
        }

        public bool IsSynced => _link != null && _link.IsSynced;
        public long FramesReceived => _link != null ? _link.FramesReceived : 0;
        public float RoundTripMilliseconds => _link != null ? _link.RoundTripMilliseconds : 0f;
        public float SecondsSinceLastFrame => _link != null ? (float)_link.SecondsSinceLastFrame(_clock.NowTicks) : float.PositiveInfinity;
        public string ActivePredictor => _link != null ? _link.Bank.ActiveLabel : "";

        /// <summary>Whether a double curl locks the arm on (true) or the curl must be held (false).</summary>
        public bool UsesToggle => _toggleConfig != null;

        /// <summary>Whether this hand's arm is locked on by a double curl (toggle mode); it may still be holding, if the hand is lost.</summary>
        public bool IsLocked(bool leftHand)
        {
            int arm = leftHand ? _left : _right;
            return _toggle != null && arm >= 0 && _toggle[arm].IsLocked;
        }

        public DuoArmState ArmState(bool leftHand)
        {
            int arm = leftHand ? _left : _right;
            if (_teleop == null || arm < 0) return DuoArmState.Idle;
            return (DuoArmState)(int)_teleop.Status(arm);
        }

        /// <summary>The arm's commanded gripper opening, 0..1.</summary>
        public float Opening(bool leftHand)
        {
            int arm = leftHand ? _left : _right;
            return _teleop == null || arm < 0 ? 1f : (float)_teleop.Opening(arm);
        }

        /// <summary>The grab's curl ratio from three fingers' tip-to-wrist and knuckle-to-wrist distances (<see cref="GrabGesture.CurlRatio"/>).</summary>
        public static float CurlRatio(float middleTip, float middleKnuckle, float ringTip, float ringKnuckle, float littleTip, float littleKnuckle) =>
            GrabGesture.CurlRatio(middleTip, middleKnuckle, ringTip, ringKnuckle, littleTip, littleKnuckle);

        /// <summary>
        /// One hand's frame (world space): whether it is tracked, its palm's pose, its curl ratio and its thumb-index
        /// gap. A held grab engages that hand's arm, or moves it; letting go, or losing tracking, holds it.
        /// </summary>
        public void SetHand(bool leftHand, bool tracked, Vector3 palmPosition, Quaternion palmRotation, float curlRatio, float pinchMeters)
        {
            int arm = leftHand ? _left : _right;
            if (_teleop == null || arm < 0) return;
            bool held = _grab[arm].Update(_grabConfig, tracked, curlRatio);
            if (_toggleConfig != null)
            {
                // A double curl locks the arm on and another unlocks it; while locked it follows the hand whether or not
                // tracking reads the hand as curled, and holds while the hand is lost.
                bool locked = _toggle[arm].Update(_toggleConfig, tracked, held, _clock.NowTicks / (double)_clock.TicksPerSecond);
                held = locked && tracked;
            }

            if (_mapping != null)
            {
                SetHandAbsolute(arm, held, tracked, palmPosition, palmRotation, pinchMeters);
                return;
            }

            if (!held)
            {
                if (_teleop.IsEngaged(arm)) _teleop.Release(arm);
                return;
            }

            RigidTransform hand = ToRoot(palmPosition, palmRotation);
            if (!_teleop.IsEngaged(arm))
            {
                if (!_teleop.Engage(arm, hand)) return;
            }
            else
            {
                _teleop.Follow(arm, hand);
            }

            _teleop.SetOpening(arm, GrabGesture.Opening(_grabConfig, pinchMeters));
        }

        /// <summary>
        /// The absolute mapping: where the hand is (relative to your body) is where the gripper goes, and the palm's
        /// orientation is the gripper's. While the hand is open nothing moves and the marker stays on the arm; a grab
        /// slides the arm to where the hand maps and then follows the hand.
        /// </summary>
        private void SetHandAbsolute(int arm, bool held, bool tracked, Vector3 palmPosition, Quaternion palmRotation, float pinchMeters)
        {
            if (!tracked || !_mapping.IsCentred)
            {
                if (_teleop.IsEngaged(arm)) _teleop.Release(arm);
                return;
            }

            if (!held)
            {
                // An open hand moves nothing, its marker included: the marker stays on the arm until a grab (the user's call).
                if (_teleop.IsEngaged(arm)) _teleop.Release(arm);
                return;
            }

            RigidTransform target = _mapping.Map(arm, ToWorld(palmPosition, palmRotation));

            if (!_teleop.IsEngaged(arm))
            {
                if (!_teleop.EngageAt(arm, target)) return;
            }
            else
            {
                _teleop.Track(arm, target);
            }

            _teleop.SetOpening(arm, GrabGesture.Opening(_grabConfig, pinchMeters));
        }

        private void Awake()
        {
            _config = ConfigLoader.Load(ConfigResource, ConfigResource + ".json", new SerialChainConnectionConfig());
            _profile = LoadResource<SerialChainProfileDocument>(ProfileResource).ToProfile();
            SerialChainCollisionModel collision = LoadResource<SerialChainCollisionDocument>(CollisionResource).ToModel();
            _clock = new UnityMonotonicClock();
            _sendIntervalTicks = (long)(_clock.TicksPerSecond / Math.Max(1f, _config.CommandRateHz));

            _grabConfig = new GrabGestureConfig
            {
                EngageBelowCurl = _config.GrabEngageBelowCurl, ReleaseAboveCurl = _config.GrabReleaseAboveCurl,
                PinchOpenMeters = _config.PinchOpenMeters, PinchClosedMeters = _config.PinchClosedMeters,
            };
            if (_grabConfig.Validate() is string badGrab) throw new InvalidOperationException("fr3_connection: " + badGrab);
            _teleop = new SerialChainTeleop(_profile, collision,
                new SerialChainTeleopOptions { PositionScale = _config.PositionScale, MaxStepRadians = _config.MaxStepRadians, ApproachSteps = Math.Max(1, _config.ApproachSteps) });
            if (_config.HandMapping == "absolute")
            {
                _mapping = new BodyAnchoredMapping(_profile, BodyAnchoredMapping.NeutralTools(_profile), new BodyMappingOptions
                {
                    PositionScale = _config.PositionScale, NeutralForward = _config.NeutralForward, NeutralSide = _config.NeutralSide, NeutralDown = _config.NeutralDown,
                });
            }
            else if (_config.HandMapping != "relative")
            {
                throw new InvalidOperationException($"fr3_connection: HandMapping '{_config.HandMapping}' is neither 'absolute' nor 'relative'");
            }
            _left = _teleop.ArmIndex("left");
            _right = _teleop.ArmIndex("right");
            _grab = new GrabGesture[_teleop.ArmCount];
            _toggle = new GrabToggle[_teleop.ArmCount];
            if (_config.GrabMode == "toggle")
            {
                _toggleConfig = new GrabToggleConfig { MinPressSeconds = _config.MinPressSeconds, DoubleClickSeconds = _config.DoubleClickSeconds };
                if (_toggleConfig.Validate() is string badToggle) throw new InvalidOperationException("fr3_connection: " + badToggle);
            }
            else if (_config.GrabMode != "hold")
            {
                throw new InvalidOperationException($"fr3_connection: GrabMode '{_config.GrabMode}' is neither 'toggle' nor 'hold'");
            }
            _targets = new JointTarget[_teleop.MotorCount];

            _udp = new UdpTransport(_config.JointLocalPort, new IPEndPoint(Resolve(_config.RemoteHost), _config.JointRemotePort),
                JointDatagram.MaxBytes, _clock);
            var clockSync = new ClockSyncConfig(
                historyCapacity: _config.ClockSyncHistoryCapacity, smoothingAlpha: _config.ClockSyncSmoothingAlpha,
                maxAcceptableRttTicks: (long)(_config.MaxAcceptableRttMs / 1000.0 * _clock.TicksPerSecond),
                outlierRttMultiple: _config.OutlierRttMultiple, minSamplesBeforeTrusted: _config.MinSamplesBeforeTrusted);
            List<JointPredictorPreset> presets = Presets(out int active);
            _link = new SerialChainJointLink(_udp, _clock, _profile, clockSync, presets, active);

            _markerColor = new MaterialPropertyBlock();
            _markerRenderers = new Renderer[targetMarkers.Length];
            _shownMarkerState = new int[targetMarkers.Length];
            for (int i = 0; i < targetMarkers.Length; i++)
            {
                _markerRenderers[i] = targetMarkers[i] != null ? targetMarkers[i].GetComponentInChildren<Renderer>() : null;
                _shownMarkerState[i] = -1;
            }

            Debug.Log($"[fr3] {_profile.Name}: {_teleop.ArmCount} arms, {_teleop.MotorCount} motors; joint port " +
                      $"{_config.RemoteHost}:{_config.JointRemotePort} from local {_config.JointLocalPort}; twin drawn by '{_link.Bank.ActiveLabel}'; " +
                      $"hands {_config.HandMapping}, grab {_config.GrabMode}");
            if (motionEnabledAtStart) Debug.LogWarning("[fr3] motionEnabledAtStart is set; motion goes on with the first full frame");
        }

        private void OnEnable() => Application.onBeforeRender += HandleBeforeRender;

        private void OnDisable() => Application.onBeforeRender -= HandleBeforeRender;

        private void Update()
        {
            if (_link == null) return;
            long now = _clock.NowTicks;
            _link.Tick(now);
            if (!_teleop.HasPose && _link.FramesReceived > 0) _teleop.TryTakePose(_link.Measured);
            if (motionEnabledAtStart && !_motion && _teleop.HasPose) MotionEnabled = true;
            // Anchored at launch too, once the headset is tracking, so the markers show where a grab would go.
            if (_mapping != null && !_mapping.IsCentred && Camera.main != null && Camera.main.transform.position != Vector3.zero) Recentre();

            _teleop.Step();
            if (_motion && _teleop.HasPose && (_lastSendTicks == long.MinValue || now - _lastSendTicks >= _sendIntervalTicks))
            {
                int n = _teleop.WriteTargets(_targets);
                if (JointCommandCodec.TryEncode(++_sequence, now, _targets.AsSpan(0, n), _buffer, out int written))
                {
                    _udp.Send(_buffer.AsSpan(0, written), now);
                    _link.OnCommandSent(now, _targets.AsSpan(0, n));
                }

                _lastSendTicks = now;
            }

            ShowMarkers();
        }

        private void HandleBeforeRender()
        {
            if (_link == null || twin == null) return;
            ReadOnlySpan<JointSample> pose = _link.Predicted(_clock.NowTicks);
            if (pose.IsEmpty) pose = _link.Measured;
            if (!pose.IsEmpty) twin.Apply(pose);
        }

        private void OnApplicationPause(bool paused)
        {
            // Android pausing the app (the headset taken off, another app) stops the robot: nobody is operating it.
            if (paused) MotionEnabled = false;
        }

        private void OnDestroy() => _udp?.Dispose();

        private void ShowMarkers()
        {
            for (int i = 0; i < targetMarkers.Length && i < _teleop.ArmCount; i++)
            {
                Transform marker = targetMarkers[i];
                if (marker == null || robotRoot == null) continue;
                RigidTransform t = _teleop.HasPose ? _teleop.Target(i) : RigidTransform.Identity;
                marker.gameObject.SetActive(_teleop.HasPose);
                marker.SetPositionAndRotation(
                    robotRoot.TransformPoint(new CoreVec((float)t.X, (float)t.Y, (float)t.Z).ToUnity()),
                    robotRoot.rotation * new CoreQuat((float)t.Qx, (float)t.Qy, (float)t.Qz, (float)t.Qw).ToUnity());

                int state = (int)_teleop.Status(i);
                Renderer r = _markerRenderers[i];
                if (r == null || state == _shownMarkerState[i]) continue;
                _shownMarkerState[i] = state;
                Color c = state == 1 ? FollowingColor : state == 2 ? ShortColor : state == 3 ? BlockedColor : IdleColor;
                // Both pipelines' colour names (unity/CLAUDE.md): URP reads _BaseColor, built-in _Color.
                _markerColor.SetColor("_BaseColor", c);
                _markerColor.SetColor("_Color", c);
                r.SetPropertyBlock(_markerColor);
            }
        }

        /// <summary>A world-space pose in Core's convention, still in the world (the absolute mapping's frame).</summary>
        private static RigidTransform ToWorld(Vector3 position, Quaternion rotation)
        {
            CoreVec p = position.ToCore();
            CoreQuat q = rotation.ToCore();
            return new RigidTransform(p.X, p.Y, p.Z, q.W, q.X, q.Y, q.Z);
        }

        /// <summary>A world-space hand pose in the robot's root frame, Core's convention.</summary>
        private RigidTransform ToRoot(Vector3 position, Quaternion rotation)
        {
            Transform root = robotRoot != null ? robotRoot : transform;
            CoreVec p = root.InverseTransformPoint(position).ToCore();
            CoreQuat q = (Quaternion.Inverse(root.rotation) * rotation).ToCore();
            return new RigidTransform(p.X, p.Y, p.Z, q.W, q.X, q.Y, q.Z);
        }

        private void ResetGrabs()
        {
            for (int a = 0; a < _grab.Length; a++)
            {
                _grab[a].Reset();
                _toggle[a].Reset();
            }
        }

        private List<JointPredictorPreset> Presets(out int active)
        {
            JointPredictorConfig config = SerialChainJointLink.PredictorConfig(_profile, _clock.TicksPerSecond, _config.MaxHorizonMs, 2.5f);
            var presets = new List<JointPredictorPreset>();
            active = 0;
            foreach (JointPredictorPresetData p in _config.JointPredictorPresets ?? new JointPredictorPresetData[0])
            {
                if (p == null || !p.Enabled) continue;
                JointPredictorConfig c = SerialChainJointLink.PredictorConfig(_profile, _clock.TicksPerSecond, _config.MaxHorizonMs, p.ModelSpeedRadPerSec);
                if (p.Predictor == "learned")
                {
                    // A model that does not load leaves its preset out, saying why; it never takes the others down.
                    if (!LearnedModels.TryLoad(p.Model, out IInferenceBackend model, out string error))
                    {
                        Debug.LogError($"[fr3] preset '{p.Label}': {error}; left out");
                        continue;
                    }

                    c = c.WithModel(model);
                }

                if (p.Label == _config.JointPredictorActive) active = presets.Count;
                presets.Add(new JointPredictorPreset(p.Label, p.Predictor, c));
            }

            if (presets.Count == 0) presets.Add(new JointPredictorPreset("franka-replay", "franka-replay", config));
            return presets;
        }

        private static T LoadResource<T>(string name) where T : class
        {
            TextAsset asset = Resources.Load<TextAsset>(name);
            if (asset == null) throw new InvalidOperationException($"SerialChainOperatorBridge: Resources/{name}.json is missing");
            return JsonUtility.FromJson<T>(asset.text) ?? throw new InvalidOperationException($"SerialChainOperatorBridge: Resources/{name}.json did not parse");
        }

        private static IPAddress Resolve(string host)
        {
            if (IPAddress.TryParse(host, out IPAddress address)) return address;
            // Once, at start: a name (the PC running just fr3-sim-host) rather than an address.
            foreach (IPAddress a in Dns.GetHostAddresses(host))
            {
                if (a.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork) return a;
            }

            throw new InvalidOperationException($"SerialChainOperatorBridge: '{host}' has no IPv4 address");
        }
    }
}
