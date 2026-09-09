using Teleop.Core.Types;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// The network-disturbance panel: per-axis checkboxes (lag, jitter, loss, reorder) that take
    /// effect immediately, in Play mode, without restarting the session.
    ///
    /// Attach to the same GameObject as <see cref="TeleopOperatorBridge"/>, or anywhere with a
    /// reference to it. Everything is driven from <see cref="NetworkImpairmentSettings"/>, which is
    /// serialized, so the Inspector renders it as checkboxes and sliders with no custom editor --
    /// and the same fields are reachable from a world-space UI for the in-headset case, via
    /// <see cref="SetDelayEnabled"/> and friends.
    ///
    /// <b>What this actually impairs.</b> Both directions of the Phase 4 loopback -- uplink
    /// (operator command to robot) and downlink (robot state back to operator) -- because
    /// <see cref="TeleopOperatorBridge"/> owns both and both are in-process. That is the honest
    /// place for this feature. The real-robot path (<see cref="JetRoverOperatorBridge"/>) is
    /// deliberately *not* wired here: <c>EmulatedTransport</c> impairs on the receiving side, so
    /// impairing what the real JetRover receives would mean wrapping <c>Teleop.RobotHost</c>'s own
    /// transport on the Jetson -- a robot-side change. Wiring this to that bridge would produce a
    /// checkbox that visibly changes the HUD's latency numbers while the physical arm keeps moving
    /// exactly as before, which is worse than no checkbox. That limitation is already recorded in
    /// <see cref="JetRoverOperatorBridge"/>'s own type doc.
    ///
    /// <b>A session whose conditions changed mid-run is not a result.</b> Every change is logged
    /// with the tick it happened at, because the impairment state is not otherwise recoverable from
    /// the recorded <c>.tlog</c>. If you are recording something citable, set the conditions before
    /// pressing Play and leave them alone; this control exists for feeling out the parameter space
    /// and for demos, not for producing numbers. Numbers come from <c>Teleop.Eval</c> sweeps, whose
    /// profiles are frozen and named for exactly this reason.
    /// </summary>
    public sealed class NetworkImpairmentController : MonoBehaviour
    {
        [SerializeField] private TeleopOperatorBridge operatorBridge;

        [Tooltip("Per-axis impairment. Changes apply immediately, including during Play mode.")]
        [SerializeField] private NetworkImpairmentSettings settings = new NetworkImpairmentSettings();

        [Header("Direction")]
        [Tooltip("Impair operator -> robot commands. Uncheck to isolate downlink-only impairment.")]
        [SerializeField] private bool applyToUplink = true;

        [Tooltip("Impair robot -> operator state replies. Uncheck to isolate uplink-only impairment.")]
        [SerializeField] private bool applyToDownlink = true;

        /// <summary>
        /// Fixed rather than time-derived on purpose: the same seed replays the same drop and jitter
        /// sequence, so "50ms vs 200ms" is a comparison of one variable instead of two. Change it to
        /// resample the impairment without changing its statistics.
        ///
        /// Typed <see cref="long"/> rather than the <see cref="ulong"/> the RNG wants purely because
        /// <see cref="long"/> has an unambiguous Inspector drawer; it is cast unchecked at use.
        /// </summary>
        [Header("Determinism")]
        [SerializeField] private long randomSeed = 0x5EED_1234L;

        /// <summary>
        /// Distinguishes the two directions' RNG streams. Without this the uplink and downlink
        /// emulators, built from one seed, would drop the same datagram indices in lockstep -- an
        /// artifact no real link has. See <see cref="SwappableTransport.Install"/>.
        /// </summary>
        private const ulong DownlinkSeedOffset = 0x9E37_79B9_7F4A_7C15UL;

        private SwappableTransport _uplink;
        private SwappableTransport _downlink;
        private UnityMonotonicClock _clock;

        /// <summary>Last applied state, kept to detect an Inspector edit without rebuilding every frame.</summary>
        private readonly NetworkImpairmentSettings _applied = new NetworkImpairmentSettings();
        private bool _appliedUplink;
        private bool _appliedDownlink;
        private bool _hasApplied;

        /// <summary>The live settings object. Mutate it and call <see cref="Apply"/>, or use the setters below.</summary>
        public NetworkImpairmentSettings Settings => settings;

        /// <summary>One-line summary of what is currently switched on, for a HUD. "none" when clean.</summary>
        public string Describe() => settings.Describe();

        private void Start()
        {
            if (operatorBridge == null)
            {
                operatorBridge = GetComponent<TeleopOperatorBridge>();
            }

            if (operatorBridge == null)
            {
                Debug.LogError(
                    "NetworkImpairmentController: no TeleopOperatorBridge assigned and none on this " +
                    "GameObject. Network impairment is disabled for this session.", this);
                enabled = false;
                return;
            }

            _clock = new UnityMonotonicClock();
            _uplink = operatorBridge.UplinkTransport as SwappableTransport;
            _downlink = operatorBridge.DownlinkTransport as SwappableTransport;

            if (_uplink == null || _downlink == null)
            {
                Debug.LogError(
                    "NetworkImpairmentController: TeleopOperatorBridge's transports are not " +
                    "SwappableTransport, so impairment cannot be changed at runtime. This means the " +
                    "bridge was built without the swappable indirection -- check its Awake().", this);
                enabled = false;
                return;
            }

            Apply();
        }

        /// <summary>
        /// How long a numeric edit must stop changing before it is applied. Exists because a
        /// rebuild is not free: each one discards the old emulator's in-flight datagrams
        /// (<see cref="SwappableTransport"/>) and writes a log line, so applying every frame of a
        /// slider drag would shred the link and flood the Console while the operator is still
        /// choosing a value. Checkbox changes bypass this entirely -- a toggle is a discrete
        /// decision the operator expects to land instantly, and it cannot be dragged.
        /// </summary>
        private const float NumericDebounceSeconds = 0.2f;

        private float _numericChangeUnappliedSince = -1f;

        /// <summary>
        /// Picks up Inspector edits. Cheap: a field comparison per frame, and a transport rebuild
        /// only when something actually changed. Runs in <c>Update</c> rather than
        /// <c>OnValidate</c> because <c>OnValidate</c> does not fire for a slider dragged in Play
        /// mode on all Unity versions, and because the rebuild needs a live clock.
        /// </summary>
        private void Update()
        {
            if (!_hasApplied
                || ToggleStateChanged()
                || applyToUplink != _appliedUplink
                || applyToDownlink != _appliedDownlink)
            {
                Apply();
                return;
            }

            if (settings.ValueEquals(_applied))
            {
                // Nothing outstanding; clear any debounce a reverted edit left behind.
                _numericChangeUnappliedSince = -1f;
                return;
            }

            // Only numeric fields differ -- wait for the value to settle before rebuilding.
            if (_numericChangeUnappliedSince < 0f)
            {
                _numericChangeUnappliedSince = Time.unscaledTime;
                return;
            }

            if (Time.unscaledTime - _numericChangeUnappliedSince >= NumericDebounceSeconds)
            {
                Apply();
            }
        }

        /// <summary>True when any checkbox differs from what is installed, ignoring numeric fields.</summary>
        private bool ToggleStateChanged()
        {
            return settings.EnableDelay != _applied.EnableDelay
                || settings.EnableJitter != _applied.EnableJitter
                || settings.EnableLoss != _applied.EnableLoss
                || settings.EnableBurstLoss != _applied.EnableBurstLoss
                || settings.EnableReorder != _applied.EnableReorder;
        }

        /// <summary>
        /// Rebuilds both directions' impairment stages from the current settings and logs what
        /// changed. Safe to call at any time; call it after mutating <see cref="Settings"/> from
        /// code or from a UI toggle.
        /// </summary>
        public void Apply()
        {
            if (_uplink == null || _downlink == null)
            {
                return;
            }

            bool impair = settings.AnyEnabled;
            NetworkProfile profile = settings.ToProfile(_clock.TicksPerSecond);

            ulong seed = unchecked((ulong)randomSeed);
            int discarded = 0;
            discarded += ApplyTo(_uplink, impair && applyToUplink, profile, seed);
            discarded += ApplyTo(_downlink, impair && applyToDownlink, profile, unchecked(seed + DownlinkSeedOffset));

            settings.CopyTo(_applied);
            _numericChangeUnappliedSince = -1f;
            _appliedUplink = applyToUplink;
            _appliedDownlink = applyToDownlink;
            _hasApplied = true;

            // Not the hot path -- this runs only when an operator changes something, so a log line
            // here does not violate Teleop/CLAUDE.md's no-Debug.Log-in-the-hot-path rule. It is the
            // only durable record that conditions changed partway through a recording.
            string directions = applyToUplink
                ? (applyToDownlink ? "uplink+downlink" : "uplink only")
                : (applyToDownlink ? "downlink only" : "neither direction");

            Debug.Log(
                $"NetworkImpairment @ tick {_clock.NowTicks}: {settings.Describe()} [{directions}]" +
                (discarded > 0 ? $" -- {discarded} in-flight datagram(s) discarded by the swap" : string.Empty));
        }

        private static int ApplyTo(SwappableTransport transport, bool impair, NetworkProfile profile, ulong seed)
        {
            if (impair)
            {
                transport.Install(profile, seed);
            }
            else
            {
                transport.Remove();
            }

            return transport.LastSwapDiscardedCount;
        }

        // --- Setters for a world-space UI toggle's OnValueChanged, which can only bind to a
        // --- single-argument method. Each applies immediately.

        public void SetDelayEnabled(bool value)
        {
            settings.EnableDelay = value;
            Apply();
        }

        public void SetJitterEnabled(bool value)
        {
            settings.EnableJitter = value;
            Apply();
        }

        public void SetLossEnabled(bool value)
        {
            settings.EnableLoss = value;
            Apply();
        }

        public void SetBurstLossEnabled(bool value)
        {
            settings.EnableBurstLoss = value;
            Apply();
        }

        public void SetReorderEnabled(bool value)
        {
            settings.EnableReorder = value;
            Apply();
        }

        public void SetBaseDelayMs(float value)
        {
            settings.BaseDelayMs = value;
            Apply();
        }

        public void SetJitterMs(float value)
        {
            settings.JitterMs = value;
            Apply();
        }

        public void SetLossPercent(float value)
        {
            settings.LossPercent = value;
            Apply();
        }

        /// <summary>Turns every axis off in one call -- the "back to a clean link" button.</summary>
        public void ClearAll()
        {
            settings.EnableDelay = false;
            settings.EnableJitter = false;
            settings.EnableLoss = false;
            settings.EnableBurstLoss = false;
            settings.EnableReorder = false;
            Apply();
        }
    }
}
