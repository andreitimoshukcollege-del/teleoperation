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

        /// <summary>
        /// Picking a preset other than <see cref="NetworkProfilePreset.Custom"/> loads that frozen
        /// profile's exact values into <see cref="settings"/> once, then leaves them alone. Editing
        /// any axis afterwards is expected and supported — it just means the settings are no longer
        /// that profile, and <see cref="Describe"/> starts saying "(modified)" so a recording never
        /// claims a parity it does not have.
        /// </summary>
        [Header("Preset -- loads the frozen suite's values, then you edit freely")]
        [SerializeField] private NetworkProfilePreset preset = NetworkProfilePreset.Custom;

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

        /// <summary>The preset last loaded, so a change in the dropdown is distinguishable from an axis edit.</summary>
        private NetworkProfilePreset _loadedPreset = NetworkProfilePreset.Custom;

        /// <summary>
        /// The preset's values as loaded, before the operator touched anything. Comparing
        /// <see cref="settings"/> against this is what makes "(modified)" in
        /// <see cref="Describe"/> honest rather than a guess.
        /// </summary>
        private readonly NetworkImpairmentSettings _presetBaseline = new NetworkImpairmentSettings();
        private bool _hasPresetBaseline;

        /// <summary>Samples of the loaded delay trace, already in this clock's tick domain. Null unless in trace mode.</summary>
        private long[] _traceTicks;
        private string _loadedTraceName;

        /// <summary>The live settings object. Mutate it and call <see cref="Apply"/>, or use the setters below.</summary>
        public NetworkImpairmentSettings Settings => settings;

        /// <summary>
        /// One-line summary of what is currently switched on, for a HUD. "none" when clean.
        ///
        /// When a preset is loaded and untouched, this names it, so a session can be described by
        /// the same name a sweep would use. The moment any axis differs from what the preset
        /// loaded, it becomes "&lt;name&gt; (modified)" followed by the actual axis values: the
        /// settings genuinely are no longer that profile, and a summary that kept claiming the name
        /// would make a recording look comparable to a sweep it is not comparable to.
        /// </summary>
        public string Describe()
        {
            string axes = settings.Describe();

            if (_loadedPreset == NetworkProfilePreset.Custom || !_hasPresetBaseline)
            {
                return axes;
            }

            string name = NetworkProfilePresets.CatalogName(_loadedPreset);
            return settings.ValueEquals(_presetBaseline)
                ? $"{name} [{axes}]"
                : $"{name} (modified) [{axes}]";
        }

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
            // A dropdown change is a load, not an edit: it overwrites the axes, so it must happen
            // before the change detection below sees the new values and treats them as a manual
            // edit that dirties the preset.
            if (preset != _loadedPreset)
            {
                LoadPreset();
                Apply();
                return;
            }

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

        /// <summary>
        /// Loads <see cref="preset"/> into <see cref="settings"/> and records the baseline that
        /// "(modified)" is measured against.
        ///
        /// Selecting <see cref="NetworkProfilePreset.Custom"/> deliberately loads nothing and
        /// clears nothing: it means "these are hand-configured", so wiping the operator's values on
        /// the way back to Custom would destroy work rather than reveal anything.
        /// </summary>
        private void LoadPreset()
        {
            _loadedPreset = preset;

            if (preset == NetworkProfilePreset.Custom)
            {
                _hasPresetBaseline = false;
                settings.DelayTrace.Enabled = false;
                return;
            }

            if (NetworkProfilePresets.IsTraceDriven(preset))
            {
                // The trace supplies delay, so the parametric delay axes are switched off rather
                // than left on and silently ignored -- ToProfile would zero them anyway, and an
                // Inspector showing a ticked "Enable Delay" that does nothing is a lie.
                settings.DelayTrace.Enabled = true;
                settings.DelayTrace.TraceName = NetworkProfilePresets.CatalogName(preset);
                settings.Delay.Enabled = false;
                settings.Jitter.Enabled = false;
            }
            else
            {
                settings.DelayTrace.Enabled = false;
                if (!NetworkProfilePresets.TryApply(preset, _clock.TicksPerSecond, settings, out string error))
                {
                    Debug.LogError($"NetworkImpairmentController: could not load preset '{preset}': {error}", this);
                    _hasPresetBaseline = false;
                    return;
                }
            }

            settings.CopyTo(_presetBaseline);
            _hasPresetBaseline = true;
        }

        /// <summary>
        /// True when any checkbox differs from what is installed, ignoring numeric fields -- a
        /// toggle applies instantly while a dragged slider debounces. Delegated to the aggregate so
        /// a newly added axis is covered without editing this file.
        ///
        /// The trace *name* counts as a toggle rather than a numeric edit: it selects a different
        /// file, so debouncing it would leave the old trace running for no benefit.
        /// </summary>
        private bool ToggleStateChanged()
        {
            return !settings.EnabledStateEquals(_applied)
                || settings.DelayTrace.TraceName != _applied.DelayTrace.TraceName;
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

            if (!EnsureTraceLoaded())
            {
                // The trace is the delay source; without it, applying loss and reorder alone would
                // be a different experiment than the one asked for, silently. Refusing is the
                // honest outcome, and the loader has already logged why.
                settings.DelayTrace.Enabled = false;
                impair = settings.AnyEnabled;
            }

            ulong seed = unchecked((ulong)randomSeed);
            int discarded = 0;

            // Built separately per direction, never shared: a Core impairment owns mutable model
            // state and one RNG substream and rejects a second bind.
            discarded += ApplyTo(_uplink, impair && applyToUplink, seed);
            discarded += ApplyTo(_downlink, impair && applyToDownlink, unchecked(seed + DownlinkSeedOffset));

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

        private int ApplyTo(SwappableTransport transport, bool impair, ulong seed)
        {
            if (impair)
            {
                transport.Install(
                    settings.CreateImpairments(_clock.TicksPerSecond, _traceTicks), seed);
            }
            else
            {
                transport.Remove();
            }

            return transport.LastSwapDiscardedCount;
        }

        /// <summary>
        /// Loads the configured trace if trace mode is on and the file is not already loaded.
        /// Returns false only when trace mode is wanted but the trace could not be read.
        ///
        /// Cached by name so dragging an unrelated slider does not re-read and re-rescale a
        /// 2000-sample file every 0.2s. Both directions share the same sample array, matching the
        /// sweeps: <c>SweepCommand</c> hands the same <c>namedProfile.TraceTicks</c> to its uplink
        /// and downlink, decorrelating them through the seed rather than through separate traces.
        /// </summary>
        private bool EnsureTraceLoaded()
        {
            if (!settings.UsesDelayTrace)
            {
                _traceTicks = null;
                _loadedTraceName = null;
                return true;
            }

            if (_traceTicks != null && _loadedTraceName == settings.DelayTrace.TraceName)
            {
                return true;
            }

            _traceTicks = DelayTraceLoader.TryLoad(settings.DelayTrace.TraceName, _clock.TicksPerSecond, out string error);
            if (_traceTicks == null)
            {
                _loadedTraceName = null;
                Debug.LogError(
                    $"NetworkImpairmentController: delay trace unavailable, so trace mode is off. {error}", this);
                return false;
            }

            _loadedTraceName = settings.DelayTrace.TraceName;
            return true;
        }

        // --- Setters for a world-space UI toggle's OnValueChanged, which can only bind to a
        // --- single-argument method. Each applies immediately.

        public void SetDelayEnabled(bool value)
        {
            settings.Delay.Enabled = value;
            Apply();
        }

        public void SetJitterEnabled(bool value)
        {
            settings.Jitter.Enabled = value;
            Apply();
        }

        public void SetLossEnabled(bool value)
        {
            settings.Loss.Enabled = value;
            Apply();
        }

        public void SetBurstLossEnabled(bool value)
        {
            settings.Loss.Bursty = value;
            Apply();
        }

        public void SetReorderEnabled(bool value)
        {
            settings.Reorder.Enabled = value;
            Apply();
        }

        public void SetBaseDelayMs(float value)
        {
            settings.Delay.BaseDelayMs = value;
            Apply();
        }

        public void SetJitterMs(float value)
        {
            settings.Jitter.JitterMs = value;
            Apply();
        }

        public void SetLossPercent(float value)
        {
            settings.Loss.LossPercent = value;
            Apply();
        }

        /// <summary>Turns every axis off in one call -- the "back to a clean link" button.</summary>
        public void ClearAll()
        {
            NetworkImpairment[] axes = settings.AllAxes;
            for (int i = 0; i < axes.Length; i++)
            {
                axes[i].Enabled = false;
            }

            Apply();
        }
    }
}
