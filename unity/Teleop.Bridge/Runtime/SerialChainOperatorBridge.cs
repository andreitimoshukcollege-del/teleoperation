using System;
using System.Collections.Generic;
using System.Net;
using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Prediction;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;
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
    /// <item><b>onBeforeRender</b>: the twin's three arms (docs/adr/0018, as the JetRover's): <b>predicted</b>, the
    /// active predictor's estimate for now (the newest frame before the clocks sync); <b>commanded</b>, where the
    /// clutches tell the arms to go; <b>measured</b>, the newest frame as received. The bench's switches show or hide
    /// each (<see cref="ITwinVisibility"/>).</item>
    /// </list>
    /// The lab's controls drive it as they drive the JetRover's bridge (<see cref="OperatorBridgeBase"/>, docs/adr/0029
    /// amendment): the console's predictors and network disturbance, the added delay, power and pause, and the headset
    /// guard. <b>Motion is off at launch</b>: nothing but keepalives goes until the operator switches it on, and
    /// switching it on first takes the robot's measured pose as the command and lets go of both clutches, so the arms
    /// never jump to wherever the hands were while it was off or paused. Off and paused differ only in their labels: a
    /// serial-chain host has no torque to keep or drop (docs/adr/0027 decision 5), so both stop commanding and the
    /// robot's own shaper brings it to rest at its last target.
    /// </summary>
    [DefaultExecutionOrder(-40)]
    public sealed class SerialChainOperatorBridge : OperatorBridgeBase, ITwinVisibility
    {
        private const string ConfigResource = "fr3_connection";
        private const string ProfileResource = "fr3_duo_profile";
        private const string CollisionResource = "fr3_duo_collision";
        private const string SceneResource = "fr3_scene";

        [Tooltip("The robot's root frame in the scene: the URDF's base link, where the twin is built.")]
        [SerializeField] private Transform robotRoot;
        [Tooltip("The predicted arms, solid: the active predictor's estimate for now.")]
        [SerializeField] private SerialChainArmRig twin;
        [Tooltip("Optional: the commanded arms, a ghost: where the clutches tell the arms to go.")]
        [SerializeField] private SerialChainArmRig commandedTwin;
        [Tooltip("Optional: the measured arms, faint: the robot's newest frame as received.")]
        [SerializeField] private SerialChainArmRig measuredTwin;
        [SerializeField] private bool showPredicted = true;
        [SerializeField] private bool showCommanded = true;
        [SerializeField] private bool showMeasured = true;
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
        private JointLinkDisturbance _transport;
        private ulong _disturbanceSeed;
        private (float Lag, float Jitter, float Loss) _disturbance;
        private SerialChainJointLink _link;
        private WorldState _world;
        private ObjectEditGesture _edit;
        private PointerDrag _pointer;
        private bool _editObjects, _hasPendingMove;
        private ObjectPose _pendingMove;
        private LinkView _view;
        private JointSample[] _commanded;
        private SerialChainTeleop _teleop;
        private GrabGestureConfig _grabConfig;
        private BodyAnchoredMapping _mapping;
        private HandFilterConfig _handFilterConfig;
        private HandPoseFilter[] _handFilter;
        private GrabGesture[] _grab;
        private GrabToggleConfig _toggleConfig;
        private GrabToggle[] _toggle;
        private JointTarget[] _targets;
        private readonly byte[] _buffer = new byte[JointDatagram.MaxBytes];
        private long _sendIntervalTicks, _lastSendTicks = long.MinValue, _lastStepTicks = long.MinValue;
        private uint _sequence;
        private int _left = -1, _right = -1;
        private JetRoverMotionMode _mode = JetRoverMotionMode.Off;
        private string _motionOffReason = "";
        private MaterialPropertyBlock _markerColor;
        private Renderer[] _markerRenderers;
        private int[] _shownMarkerState;

        public bool IsReady => _link != null;

        /// <summary>The two-way switch: true is <see cref="JetRoverMotionMode.On"/>, false <see cref="JetRoverMotionMode.Off"/>; reads true only in On.</summary>
        public bool MotionEnabled
        {
            get => _mode == JetRoverMotionMode.On;
            set => MotionMode = value ? JetRoverMotionMode.On : JetRoverMotionMode.Off;
        }

        /// <summary>
        /// On: commands go, from the robot's measured pose (taken first; refused, staying as it was, before a complete
        /// frame has arrived), with both clutches let go and the robot anchored to where you stand. Paused and Off: none.
        /// </summary>
        public override JetRoverMotionMode MotionMode
        {
            get => _mode;
            set
            {
                if (value == _mode) return;
                if (value == JetRoverMotionMode.On)
                {
                    if (_link == null) return;
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
                    EditObjects = false; // never both: a curl would be a grab of the arm and of an object
                    // Where you stand when you switch on is where the robot is anchored to you.
                    Recentre();
                }
                else
                {
                    // Nothing more is sent; the clutches let go, so the next start begins from the robot's pose.
                    ResetGrabs();
                }

                _mode = value;
                _motionOffReason = "";
                if (_link != null) _link.MotionOn = value == JetRoverMotionMode.On;
            }
        }

        public override bool HardwareMotionAllowed => MotionAllowed;

        public override string MotionOffReason => _motionOffReason;

        /// <summary>ROBOT OFF without the button (the headset taken off): commanding stops at once.</summary>
        public override void TurnOffNow(string reason)
        {
            if (_mode == JetRoverMotionMode.Off) return;
            MotionMode = JetRoverMotionMode.Off;
            _motionOffReason = reason ?? "";
            Debug.Log($"[fr3] robot motion -> Off: {_motionOffReason}");
        }

        public override ConnectionStatus Status =>
            _link == null || _link.FramesReceived == 0 ? ConnectionStatus.NeverConnected
            : _link.SecondsSinceLastFrame(_clock.NowTicks) < 1.0 ? ConnectionStatus.Connected
            : ConnectionStatus.Stale;

        public override long NowTicks => _clock != null ? _clock.NowTicks : 0;

        public override IOperatorJointLink Link => _view;

        public override JointLinkDisturbance JointDisturbance => _transport;

        public override string GripperErrorUnit => "% of the opening";

        /// <summary>
        /// Disturbs the joint socket in both directions with a one-way delay, a uniform jitter and a random loss, clamped
        /// to the config's caps; all zero switches it off. The same router and impairments as the JetRover's
        /// (<see cref="JointLinkDisturbance"/>, Core's <c>NetworkImpairmentSettings</c>), and every change is logged
        /// with its seed and reported to the recording.
        /// </summary>
        public override void ApplyCustomDisturbance(float delayMs, float jitterMs, float lossPercent)
        {
            if (_transport == null) return;
            delayMs = Math.Min(Math.Max(delayMs, 0f), _config.DisturbanceMaxDelayMs);
            jitterMs = Math.Min(Math.Max(jitterMs, 0f), Math.Max(0f, _config.DisturbanceMaxDelayMs - delayMs));
            lossPercent = Math.Min(Math.Max(lossPercent, 0f), _config.DisturbanceMaxLossPercent);
            _disturbance = (delayMs, jitterMs, lossPercent);
            if (delayMs <= 0f && jitterMs <= 0f && lossPercent <= 0f)
            {
                _transport.Clear();
                Debug.Log("[network] disturbance off");
                return;
            }

            var settings = new NetworkImpairmentSettings
            {
                EnableDelay = delayMs > 0f, BaseDelayMs = delayMs,
                EnableJitter = jitterMs > 0f, JitterMs = jitterMs,
                EnableLoss = lossPercent > 0f, LossPercent = lossPercent,
            };
            string description = $"{delayMs:0} ms ± {jitterMs:0} ms, {lossPercent:0.#}% loss both ways";
            _disturbanceSeed += 2;
            _transport.Set(settings.CreateImpairments(_clock.TicksPerSecond, null), settings.CreateImpairments(_clock.TicksPerSecond, null), _disturbanceSeed, description);
            Debug.Log($"[network] disturbance: {description}, seed {_disturbanceSeed}, at {_clock.NowTicks} ticks");
        }

        public bool ShowPredicted { get => showPredicted; set => showPredicted = value; }
        public bool ShowCommanded { get => showCommanded; set => showCommanded = value; }
        public bool ShowMeasured { get => showMeasured; set => showMeasured = value; }

        public bool MotionAllowed => _config != null && _config.AllowMotion;

        /// <summary>Whether the hands map absolutely (the robot anchored to your body), rather than through the relative clutch.</summary>
        public bool IsAbsolute => _mapping != null;

        /// <summary>The world as the headset sees it (docs/adr/0031 amendment 2); null without a scene file.</summary>
        public WorldState World => _world;

        /// <summary>Whether a scene file was loaded, so the world can be shown at all.</summary>
        public bool HasWorld => _world != null;

        /// <summary>Whether RobotHost runs another scene file than this headset's (SCENE MISMATCH).</summary>
        public bool WorldSceneMismatch => _world != null && _world.SceneMismatch;

        /// <summary>Whether NEW LAYOUT can be asked for now: a world is running, motion is off, and no edit is waiting.</summary>
        public bool CanRequestNewLayout => _link != null && _world != null && _world.HasStatus && _world.Status.Running &&
                                           !_world.SceneMismatch && _mode != JetRoverMotionMode.On && !_link.WorldEditPending;

        /// <summary>
        /// Whether EDIT OBJECTS can be on (phase P5): a world is running with this headset's scene file, the robot is off, and the
        /// hands map absolutely (the cursor is where a hand would put the gripper).
        /// </summary>
        public bool CanEditObjects => _edit != null && _world.HasStatus && _world.Status.Running && !_world.SceneMismatch &&
                                      _mode != JetRoverMotionMode.On && _mapping != null && _mapping.IsCentred;

        /// <summary>
        /// EDIT OBJECTS: a held curl grabs the nearest movable object and drags it; opening the hand asks RobotHost to move it there
        /// (<see cref="ObjectEditGesture"/>). The arms let go and do not follow the hands meanwhile. Only while <see cref="CanEditObjects"/>.
        /// </summary>
        public bool EditObjects
        {
            get => _editObjects;
            set
            {
                bool on = value && CanEditObjects;
                if (on == _editObjects) return;
                _editObjects = on;
                ResetGrabs();
                _edit?.Reset();
                if (on && _teleop != null)
                {
                    if (_left >= 0 && _teleop.IsEngaged(_left)) _teleop.Release(_left);
                    if (_right >= 0 && _teleop.IsEngaged(_right)) _teleop.Release(_right);
                }

                Debug.Log($"[world] EDIT OBJECTS {(on ? "on" : "off")}");
            }
        }

        /// <summary>Whether an object is being dragged, by EDIT OBJECTS or a pointer (the twin shows its ghost at <see cref="EditPreview"/>).</summary>
        public bool EditDragging => (_editObjects && _edit != null && _edit.IsDragging) || PointerDragging;

        public byte EditDraggedId => _editObjects && _edit != null && _edit.IsDragging ? _edit.DraggedId : PointerDragging ? _pointer.DraggedId : (byte)0;

        /// <summary>Where the dragged object would go, in the robot's base frame.</summary>
        public RigidTransform EditPreview => PointerDragging ? _pointer.Preview : _edit != null ? _edit.Preview : RigidTransform.Identity;

        // --- pointing (docs/adr/0031 amendment 5) ------------------------------------------------------------------------

        /// <summary>
        /// Whether a pointing ray may drag objects now: a running world with this headset's scene file, and the robot off (the
        /// hands drive nothing). Unlike EDIT OBJECTS, the mapping does not matter: the ray is where the hand points.
        /// </summary>
        public bool CanPointerDrag => _pointer != null && _world.HasStatus && _world.Status.Running && !_world.SceneMismatch &&
                                      _mode != JetRoverMotionMode.On && !_editObjects;

        /// <summary>Whether a pointer drags an object now.</summary>
        public bool PointerDragging => _pointer != null && _pointer.IsDragging;

        /// <summary>
        /// How far along a world-space ray the first object it hits is, in metres (the ray's direction need not be unit);
        /// negative when it hits none, or pointing cannot drag now.
        /// </summary>
        public float PointerHit(Vector3 origin, Vector3 direction)
        {
            if (!CanPointerDrag) return -1f;
            RootRay(origin, direction, out CoreVec o, out CoreVec d);
            return _pointer.Hit(o.X, o.Y, o.Z, d.X, d.Y, d.Z, _world, out double distance) >= 0 ? (float)distance : -1f; // the root is not scaled
        }

        /// <summary>
        /// One hand's pointing ray this frame (world space) and whether it pinches: a pinch that starts on an object drags it,
        /// and letting go asks RobotHost to put it there (<see cref="PointerDrag"/>). Call for both hands every frame; with
        /// <paramref name="tracked"/> false (or when pointing cannot drag) a drag is dropped and nothing asked.
        /// </summary>
        public void UpdatePointer(int hand, bool tracked, bool pinched, Vector3 origin, Vector3 direction)
        {
            if (_pointer == null) return;
            bool can = CanPointerDrag;
            RootRay(origin, direction, out CoreVec o, out CoreVec d);
            bool was = _pointer.IsDragging;
            _pointer.Update(hand, tracked && can, pinched && can, o.X, o.Y, o.Z, d.X, d.Y, d.Z, _world);
            if (!was && _pointer.IsDragging) Debug.Log($"[world] pointer grabbed object {_pointer.DraggedId}");
        }

        /// <summary>Whether a move is waiting for RobotHost's answer.</summary>
        public bool EditPending => _hasPendingMove || (_link != null && _link.WorldEditPending);

        /// <summary>RobotHost's answer to the last edit, in words; empty before any.</summary>
        public string LastEditOutcome
        {
            get
            {
                if (_world == null || !_world.HasStatus || _world.Status.EditSequence == 0) return "";
                switch (_world.Status.EditOutcome)
                {
                    case 0: return "done";
                    case 1: return "refused: an arm is in the way";
                    case 2: return "refused: another object is there";
                    case 3: return "refused: off the table";
                    case 5: return "refused: that one is fixed";
                    case 6: return "refused: an arm is touching it";
                    case 7: return "refused: the robot is on";
                    case 9: return "no answer from the world";
                    default: return "refused";
                }
            }
        }

        /// <summary>Asks RobotHost for the next seeded layout (only while motion is off); false when it cannot be asked now.</summary>
        public bool RequestNewLayout()
        {
            if (!CanRequestNewLayout) return false;
            bool sent = _link.RequestWorldEdit(WorldEditOp.NewLayout, default, _clock.NowTicks);
            if (sent) Debug.Log("[world] NEW LAYOUT requested");
            return sent;
        }

        // --- scenario recording (docs/adr/0033) ------------------------------------------------------------------------

        private int _selectedScenario;

        /// <summary>RobotHost's recording status and scenario list; null before the link exists.</summary>
        public RecordingState Recording => _link != null ? _link.Recording : null;

        /// <summary>Whether RobotHost records scenarios at all (it answered with a status within the last 5 s).</summary>
        public bool RecordingAvailable => _link != null && _link.Recording.HasStatus && _link.Recording.Status.State != RecordState.Unavailable &&
                                          _link.SecondsSinceLastFrame(_clock.NowTicks) < 5.0;

        public bool IsRecording => RecordingAvailable && _link.Recording.Status.State == RecordState.Recording;

        /// <summary>Whether a record request is waiting for RobotHost's answer.</summary>
        public bool RecordPending => _link != null && _link.RecordRequestPending;

        /// <summary>The scenario the RECORD button records next: an index into the list, kept within it.</summary>
        public int SelectedScenario
        {
            get
            {
                int n = _link != null ? _link.Recording.Count : 0;
                return n == 0 ? 0 : Math.Max(0, Math.Min(n - 1, _selectedScenario));
            }
        }

        /// <summary>How many scenarios RobotHost's list holds (0 before it arrives).</summary>
        public int ScenarioCount => _link != null ? _link.Recording.Count : 0;

        /// <summary>The take being recorded: its scenario, its number, and how long it has run.</summary>
        public string TakeScenarioName => _link != null ? _link.Recording.Name(_link.Recording.Status.Scenario) : "";
        public int TakeNumber => _link != null ? _link.Recording.Status.Take : 0;
        public int TakeSeconds => _link != null ? (int)(_link.Recording.Status.ElapsedMs / 1000) : 0;

        /// <summary>Free disk where the takes go, GB, as RobotHost last said.</summary>
        public int RecordingFreeGigabytes => _link != null ? _link.Recording.Status.FreeGigabytes : 0;

        /// <summary>The selected scenario's name, or empty before the list arrives.</summary>
        public string SelectedScenarioName => _link != null ? _link.Recording.Name(SelectedScenario) : "";

        /// <summary>Moves the selection by <paramref name="step"/> through the list, wrapping; not while a take records.</summary>
        public void StepScenario(int step)
        {
            int n = _link != null ? _link.Recording.Count : 0;
            if (n == 0 || IsRecording) return;
            _selectedScenario = ((SelectedScenario + step) % n + n) % n;
        }

        /// <summary>Starts a take of the selected scenario, or stops the one going; false when it cannot be asked now.</summary>
        public bool ToggleRecording()
        {
            if (!RecordingAvailable || RecordPending) return false;
            bool stop = IsRecording;
            bool sent = _link.RequestRecord(stop ? RecordOp.Stop : RecordOp.Start, SelectedScenario, _clock.NowTicks);
            if (sent) Debug.Log(stop ? "[record] STOP asked" : $"[record] RECORD '{SelectedScenarioName}' asked");
            return sent;
        }

        /// <summary>Sets the last take aside (RobotHost moves it into the folder's _discarded); false when it cannot be asked now.</summary>
        public bool DiscardLastTake()
        {
            if (!RecordingAvailable || RecordPending || IsRecording) return false;
            bool sent = _link.RequestRecord(RecordOp.DiscardLast, SelectedScenario, _clock.NowTicks);
            if (sent) Debug.Log("[record] DISCARD LAST asked");
            return sent;
        }

        /// <summary>RobotHost's answer to the last record request, in words; empty before any.</summary>
        public string LastRecordOutcome
        {
            get
            {
                if (_link == null || !_link.Recording.HasStatus || _link.Recording.Status.Sequence == 0) return "";
                switch (_link.Recording.Status.Outcome)
                {
                    case RecordOutcome.Started: return "recording";
                    case RecordOutcome.Saved: return "take saved";
                    case RecordOutcome.Discarded: return "last take set aside";
                    case RecordOutcome.Busy: return "busy: try again";
                    case RecordOutcome.UnknownScenario: return "the list changed: pick again";
                    case RecordOutcome.LowDisk: return "the disk is nearly full";
                    case RecordOutcome.Failed: return "could not record: see RobotHost's log";
                    case RecordOutcome.NothingToDiscard: return "no take to set aside";
                    case RecordOutcome.StoppedByItself: return "the take stopped by itself";
                    default: return "";
                }
            }
        }

        /// <summary>
        /// Anchors the robot to your body as you stand now (docs/adr/0029 amendments 1 and 4): the headset's position, with
        /// the robot's heading, becomes the body frame, and your neutral hands the arms' neutral tools. Also done when
        /// motion is switched on. The heading is the robot's, not the headset's: RECENTRE and Power are buttons you look at
        /// to press, and taking the direction you looked as forward turned the gripper sideways from a hand pointing
        /// forward (2026-11-01).
        /// </summary>
        public void Recentre()
        {
            if (_mapping == null) return;
            Camera eye = Camera.main;
            if (eye == null) return;
            Transform head = eye.transform;
            Transform facing = robotRoot != null ? robotRoot : head;
            _mapping.Recentre(ToWorld(head.position, Quaternion.Euler(0f, facing.rotation.eulerAngles.y, 0f)));
            Debug.Log($"[fr3] recentred: the robot is anchored where you stand, facing its way (you faced {Mathf.DeltaAngle(facing.rotation.eulerAngles.y, head.rotation.eulerAngles.y):0} deg off it)");
        }

        public bool IsSynced => _link != null && _link.IsSynced;
        public bool HasState => _link != null && !_link.Measured.IsEmpty;
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
            // The hand filter (docs/adr/0029 amendment 7) sees every tracked frame, held or not, so it has settled when a
            // grab takes it; a lost hand starts it again. Absolute hands are filtered in the world, relative ones in the root.
            RigidTransform palm = default;
            if (!tracked) _handFilter[arm].Reset();
            else palm = Filtered(arm, _mapping != null ? ToWorld(palmPosition, palmRotation) : ToRoot(palmPosition, palmRotation));
            bool held = _grab[arm].Update(_grabConfig, tracked, curlRatio);
            if (_editObjects)
            {
                // EDIT OBJECTS: the curl held grabs an object, never the arm (the double-curl lock is not read meanwhile).
                if (_teleop.IsEngaged(arm)) _teleop.Release(arm);
                _mapping?.ClearAnchor(arm); // the EDIT OBJECTS cursor is where the hand maps by itself
                bool mapped = tracked && _mapping != null && _mapping.IsCentred;
                _edit.Update(leftHand ? 0 : 1, mapped, held, mapped ? _mapping.Map(arm, palm) : RigidTransform.Identity, _world);
                return;
            }

            if (_toggleConfig != null)
            {
                // A double curl locks the arm on and another unlocks it; while locked it follows the hand whether or not
                // tracking reads the hand as curled, and holds while the hand is lost.
                bool locked = _toggle[arm].Update(_toggleConfig, tracked, held, _clock.NowTicks / (double)_clock.TicksPerSecond);
                held = locked && tracked;
            }

            if (_mapping != null)
            {
                SetHandAbsolute(arm, held, tracked, palm, pinchMeters);
                return;
            }

            if (!held)
            {
                if (_teleop.IsEngaged(arm)) _teleop.Release(arm);
                return;
            }

            if (!_teleop.IsEngaged(arm))
            {
                if (!_teleop.Engage(arm, palm)) return;
            }
            else
            {
                _teleop.Follow(arm, palm);
            }

            _teleop.SetOpening(arm, GrabGesture.Opening(_grabConfig, pinchMeters));
        }

        /// <summary>
        /// The absolute mapping: where the hand is (relative to your body) is where the gripper goes, and the palm's
        /// orientation is the gripper's. While the hand is open nothing moves and the marker stays on the arm; a grab
        /// slides the arm to where the hand maps and then follows the hand.
        /// </summary>
        private void SetHandAbsolute(int arm, bool held, bool tracked, in RigidTransform palm, float pinchMeters)
        {
            if (!tracked || !_mapping.IsCentred)
            {
                if (_teleop.IsEngaged(arm)) _teleop.Release(arm);
                _mapping.ClearAnchor(arm);
                return;
            }

            if (!held)
            {
                // An open hand moves nothing, its marker included: the marker stays on the arm until a grab (the user's call).
                if (_teleop.IsEngaged(arm)) _teleop.Release(arm);
                _mapping.ClearAnchor(arm);
                return;
            }

            // Position relative (HandPosition "relative", docs/adr/0029 amendment 9): the lock anchors the hand to where the tool
            // is, so the arm stays put and then moves with the hand; the gripper turns to face the palm's way over the slide.
            if (!_teleop.IsEngaged(arm) && _teleop.HasPose) _mapping.Anchor(arm, palm, _teleop.Tool(arm));
            RigidTransform target = _mapping.Map(arm, palm);

            if (!_teleop.IsEngaged(arm))
            {
                if (!_teleop.EngageAt(arm, target))
                {
                    _mapping.ClearAnchor(arm);
                    return;
                }
            }
            else
            {
                _teleop.Track(arm, target);
            }

            _teleop.SetOpening(arm, GrabGesture.Opening(_grabConfig, pinchMeters));
        }

        /// <summary>The palm as the hand filter passes it on (unchanged with <c>HandFilter</c> off).</summary>
        private RigidTransform Filtered(int arm, in RigidTransform palm) => _handFilterConfig == null
            ? palm
            : _handFilter[arm].Filter(_handFilterConfig, palm, _clock.NowTicks / (double)_clock.TicksPerSecond);

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
                    RelativePosition = _config.HandPosition == "relative",
                });
                if (_config.HandPosition != "relative" && _config.HandPosition != "absolute")
                {
                    throw new InvalidOperationException($"fr3_connection: HandPosition '{_config.HandPosition}' is neither 'relative' nor 'absolute'");
                }
            }
            else if (_config.HandMapping != "relative")
            {
                throw new InvalidOperationException($"fr3_connection: HandMapping '{_config.HandMapping}' is neither 'absolute' nor 'relative'");
            }
            if (_config.HandFilter == "one-euro")
            {
                _handFilterConfig = new HandFilterConfig
                {
                    MinCutoffHz = _config.HandFilterMinCutoffHz, PositionBeta = _config.HandFilterPositionBeta,
                    RotationBeta = _config.HandFilterRotationBeta, DerivativeCutoffHz = _config.HandFilterDerivativeCutoffHz,
                };
                if (_handFilterConfig.Validate() is string badFilter) throw new InvalidOperationException("fr3_connection: " + badFilter);
            }
            else if (_config.HandFilter != "off")
            {
                throw new InvalidOperationException($"fr3_connection: HandFilter '{_config.HandFilter}' is neither 'one-euro' nor 'off'");
            }
            _left = _teleop.ArmIndex("left");
            _right = _teleop.ArmIndex("right");
            _handFilter = new HandPoseFilter[_teleop.ArmCount];
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
            // Off until the console switches one on; commands, keepalives and frames all pass through it.
            _transport = new JointLinkDisturbance(_udp);
            _disturbanceSeed = unchecked((ulong)DateTime.UtcNow.Ticks) & ~1UL;
            var clockSync = new ClockSyncConfig(
                historyCapacity: _config.ClockSyncHistoryCapacity, smoothingAlpha: _config.ClockSyncSmoothingAlpha,
                maxAcceptableRttTicks: (long)(_config.MaxAcceptableRttMs / 1000.0 * _clock.TicksPerSecond),
                outlierRttMultiple: _config.OutlierRttMultiple, minSamplesBeforeTrusted: _config.MinSamplesBeforeTrusted);
            List<JointPredictorPreset> presets = Presets(out int active, out string[] descriptions);
            _world = LoadWorld(_profile);
            _link = new SerialChainJointLink(_transport, _clock, _profile, clockSync, presets, active, OperatorClientKind.Headset, _world);
            _edit = _world != null ? new ObjectEditGesture(_world.Scene) : null;
            _pointer = _world != null ? new PointerDrag(_world.Scene) : null;
            _view = new LinkView(_link, descriptions, _clock);
            _commanded = new JointSample[_teleop.MotorCount];

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
                      $"hands {_config.HandMapping}, grab {_config.GrabMode}; hand filter " + (_handFilterConfig == null ? "off (raw hands)" :
                      $"one-euro: {_handFilterConfig.MinCutoffHz} Hz still, +{_handFilterConfig.PositionBeta} Hz per m/s, " +
                      $"+{_handFilterConfig.RotationBeta} Hz per rad/s, speed at {_handFilterConfig.DerivativeCutoffHz} Hz"));
            if (motionEnabledAtStart) Debug.LogWarning("[fr3] motionEnabledAtStart is set; motion goes on with the first full frame");
        }

        private void OnEnable() => Application.onBeforeRender += HandleBeforeRender;

        private void OnDisable() => Application.onBeforeRender -= HandleBeforeRender;

        private void Update()
        {
            if (_link == null) return;
            long now = _clock.NowTicks;
            _link.Tick(now);
            if (_edit != null)
            {
                if (_editObjects && !CanEditObjects) EditObjects = false; // the world stopped, or the scene changed
                if (_edit.TryTakeRequest(out ObjectPose move) || (_pointer != null && _pointer.TryTakeRequest(out move)))
                {
                    _pendingMove = move;
                    _hasPendingMove = true;
                }

                // Sent once nothing else is waiting; the link re-sends it until RobotHost echoes it.
                if (_hasPendingMove && !_link.WorldEditPending && _link.RequestWorldEdit(WorldEditOp.Move, _pendingMove, now))
                {
                    _hasPendingMove = false;
                    Debug.Log($"[world] move {_pendingMove.Id} to ({_pendingMove.X:0.000}, {_pendingMove.Y:0.000}) requested");
                }
            }

            if (!_teleop.HasPose && _link.FramesReceived > 0) _teleop.TryTakePose(_link.Measured);
            if (motionEnabledAtStart && _mode != JetRoverMotionMode.On && _teleop.HasPose) MotionEnabled = true;
            // Anchored at launch too, once the headset is tracking, so the markers show where a grab would go.
            if (_mapping != null && !_mapping.IsCentred && Camera.main != null && Camera.main.transform.position != Vector3.zero) Recentre();

            // A step as long as the frame, so the gripper's speed caps hold at any frame rate (Direct Preview runs below 90 Hz).
            double frame = _lastStepTicks == long.MinValue ? 0 : (now - _lastStepTicks) / (double)_clock.TicksPerSecond;
            _lastStepTicks = now;
            // The body frame walks with you (docs/adr/0029 amendment 5): standing it stays, walking it comes along.
            if (_mapping != null && _mapping.IsCentred && Camera.main != null) _mapping.Follow(ToWorld(Camera.main.transform.position, Quaternion.identity), frame);
            _teleop.Step(frame);
            if (_mode == JetRoverMotionMode.On && _teleop.HasPose && (_lastSendTicks == long.MinValue || now - _lastSendTicks >= _sendIntervalTicks))
            {
                int n = _teleop.WriteTargets(_targets);
                if (JointCommandCodec.TryEncode(++_sequence, now, _targets.AsSpan(0, n), _buffer, out int written))
                {
                    _transport.Send(_buffer.AsSpan(0, written), now);
                    _link.OnCommandSent(now, _targets.AsSpan(0, n));
                }

                _lastSendTicks = now;
            }

            // What the headset does to its link, its view and the hands, for the robot's recording (docs/adr/0023
            // amendment, docs/adr/0029 amendment 7: a min cutoff of 0 is raw hands).
            HandFilterConfig f = _handFilterConfig;
            _link.SetHeadsetConditions(new HeadsetConditions(_disturbance.Lag, _disturbance.Jitter, _disturbance.Loss,
                (float)_link.AddedDelayMs, _transport.IsActive ? _transport.Seed : 0UL,
                f != null ? (float)f.MinCutoffHz : 0f, f != null ? (float)f.PositionBeta : 0f,
                f != null ? (float)f.RotationBeta : 0f, f != null ? (float)f.DerivativeCutoffHz : 0f));
            ShowMarkers();
        }

        private void HandleBeforeRender()
        {
            if (_link == null) return;

            // Predicted, solid: the estimate for now, or the newest frame before the clocks sync. Stays where it was
            // (and shown) before any frame, so the robot is never missing from the scene.
            ReadOnlySpan<JointSample> pose = _link.IsSynced ? _link.Predicted(_clock.NowTicks) : ReadOnlySpan<JointSample>.Empty;
            if (pose.IsEmpty) pose = _link.Measured;
            Drive(twin, showPredicted, pose);

            // Measured, faint: the newest frame as received, once there is one.
            Drive(measuredTwin, showMeasured && !_link.Measured.IsEmpty, _link.Measured);

            // Commanded, a ghost: the clutches' command, once the robot's pose has been taken.
            if (commandedTwin != null)
            {
                int n = _teleop.HasPose ? _teleop.WriteTargets(_targets) : 0;
                for (int i = 0; i < n; i++) _commanded[i] = new JointSample(_targets[i].MotorId, _targets[i].Angle, false);
                Drive(commandedTwin, showCommanded && n > 0, _commanded.AsSpan(0, n));
            }
        }

        private static void Drive(SerialChainArmRig rig, bool visible, ReadOnlySpan<JointSample> pose)
        {
            if (rig == null) return;
            if (rig.gameObject.activeSelf != visible) rig.gameObject.SetActive(visible);
            if (visible && !pose.IsEmpty) rig.Apply(pose);
        }

        private void OnApplicationPause(bool paused)
        {
            // Android pausing the app (the headset taken off, another app) stops the robot: nobody is operating it. The
            // scene's HeadsetRemovalGuard does this too (and for user presence); this is the bridge's own second line.
            if (paused) TurnOffNow("the app was paused");
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

        /// <summary>A world-space ray in the robot's root frame, Core's convention, its direction a unit vector.</summary>
        private void RootRay(Vector3 origin, Vector3 direction, out CoreVec o, out CoreVec d)
        {
            Transform root = robotRoot != null ? robotRoot : transform;
            o = root.InverseTransformPoint(origin).ToCore();
            CoreVec far = root.InverseTransformPoint(origin + direction).ToCore();
            d = CoreVec.Normalize(far - o);
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

        private List<JointPredictorPreset> Presets(out int active, out string[] descriptions)
        {
            JointPredictorConfig config = SerialChainJointLink.PredictorConfig(_profile, _clock.TicksPerSecond, _config.MaxHorizonMs, 2.5f);
            var presets = new List<JointPredictorPreset>();
            var text = new List<string>();
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
                text.Add(p.Description ?? "");
            }

            if (presets.Count == 0)
            {
                presets.Add(new JointPredictorPreset("franka-replay", "franka-replay", config));
                text.Add("");
            }

            descriptions = text.ToArray();
            return presets;
        }

        /// <summary>The console's view of the link: the bank's presets, the scorer's numbers, the added delay.</summary>
        private sealed class LinkView : IOperatorJointLink
        {
            private readonly SerialChainJointLink _link;
            private readonly string[] _descriptions;
            private readonly UnityMonotonicClock _clock;

            public LinkView(SerialChainJointLink link, string[] descriptions, UnityMonotonicClock clock)
            {
                _link = link;
                _descriptions = descriptions;
                _clock = clock;
            }

            public int PredictorCount => _link.Bank.Count;
            public int ActivePredictorIndex => _link.Bank.ActiveIndex;
            public string PredictorLabel(int index) => _link.Bank.LabelAt(index);
            public string PredictorDescription(int index) => index < _descriptions.Length ? _descriptions[index] : "";
            public bool IsPredictorEnabled(int index) => _link.Bank.IsEnabled(index);
            public bool IsPredictorWarmingUp(int index) => _link.Bank.IsEnabled(index) && !_link.Bank.HasState(index);
            public bool TrySelectPredictor(int index) => _link.Bank.TrySetActive(index);
            public bool TrySetPredictorEnabled(int index, bool enabled) => _link.Bank.TrySetEnabled(index, enabled);

            public bool TryGetPredictorStats(int index, out JetRoverPredictorStats stats)
            {
                stats = default;
                if (!_link.Scorer.TryGetStats(index, out LiveJointStats s)) return false;
                stats = new JetRoverPredictorStats(s.HasData, s.Readings, s.ErrorP50Deg, s.ErrorP95Deg, s.CorrectionP95Deg,
                    s.JitterP95Deg, s.GripperErrorP95ServoDeg);
                return true;
            }

            // The simulated FR3 has no servo board to fall silent.
            public bool BoardSilent => false;
            public float SecondsSinceFreshReading => (float)_link.SecondsSinceFreshReading(_clock.NowTicks);

            public double AddedDelayMs
            {
                get => _link.AddedDelayMs;
                set => _link.AddedDelayMs = value;
            }
        }

        /// <summary>
        /// The scene file (docs/adr/0031 amendment 2): <c>persistentDataPath/fr3_scene.json</c> when pushed, else the copy in Resources.
        /// Its sha256 is compared with the one RobotHost reports. Null (no world) when neither loads.
        /// </summary>
        private static WorldState LoadWorld(SerialChainProfile profile)
        {
            try
            {
                string pushed = System.IO.Path.Combine(Application.persistentDataPath, SceneResource + ".json");
                byte[] bytes = System.IO.File.Exists(pushed) ? System.IO.File.ReadAllBytes(pushed) : Resources.Load<TextAsset>(SceneResource)?.bytes;
                if (bytes == null) return null;
                WorldScene scene = JsonUtility.FromJson<SceneDocument>(System.Text.Encoding.UTF8.GetString(bytes)).ToScene();
                if (scene.RobotProfile != profile.Name) throw new FormatException($"the scene is for '{scene.RobotProfile}', not '{profile.Name}'");
                using (var sha = System.Security.Cryptography.SHA256.Create())
                {
                    Debug.Log($"[world] scene '{scene.Name}': {scene.Objects.Length} objects{(System.IO.File.Exists(pushed) ? " (pushed)" : "")}");
                    return new WorldState(scene, sha.ComputeHash(bytes));
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[world] the scene file does not load: {e.Message}; no world twin");
                return null;
            }
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
