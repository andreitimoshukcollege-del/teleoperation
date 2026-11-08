using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// The FR3 Duo's buttons (docs/adr/0029 decision 7 and amendment): <b>Power</b>, the motion switch, and <b>Recentre</b>,
    /// which anchors the robot to where you stand and face now (the absolute mapping). Each is a button you press down with a fingertip
    /// (<see cref="FingertipPress"/>, so no ray, pinch or grab elsewhere can press it). It sets
    /// <see cref="SerialChainOperatorBridge.MotionEnabled"/>: off sends nothing, and on first takes the robot's measured
    /// pose as the command and lets go of both arms, so nothing jumps to where the hands were. The scene starts off.
    /// The serial-chain host has no arm power to park or relax (docs/adr/0027 decision 5): off only stops commanding,
    /// and the robot's own shaper brings it to rest at its last target.
    /// </summary>
    public sealed class Fr3MotionButton : MonoBehaviour
    {
        // New roles go at the end: the scene stores the index.
        public enum Role { Power, Recentre, NewLayout, EditObjects, Record, ScenarioPrevious, ScenarioNext, DiscardLast }

        [SerializeField] private Role role = Role.Power;
        [Tooltip("Either robot's bridge. RECORD, its scenario buttons and NEW LAYOUT work with both (docs/adr/0034 J6); power, RECENTRE and EDIT OBJECTS are the FR3 Duo's.")]
        [SerializeField] private OperatorBridgeBase bridge;
        [SerializeField] private XROrigin origin;
        [Tooltip("The cap that is pressed, sitting level; its top face is the button's surface. Recoloured with the state.")]
        [SerializeField] private Renderer cap;
        [SerializeField] private TMP_Text label;
        [SerializeField] private float radiusMeters = 0.03f;
        [SerializeField] private float pressDepthMeters = 0.012f;
        [SerializeField] private float rearmDistanceMeters = 0.07f;
        [SerializeField] private float cooldownSeconds = 0.6f;
        [SerializeField] private Color offColor = new Color(0.9f, 0.12f, 0.1f);
        [SerializeField] private Color onColor = new Color(0.15f, 0.95f, 0.35f);
        [SerializeField] private Color lockedColor = new Color(0.35f, 0.36f, 0.4f);

        private FingertipPress _press;
        private MaterialPropertyBlock _block;
        private int _shown = -1;
        private float _flashUntil;

        public Role ButtonRole => role;

        private OperatorSideControls Side => bridge.SideControls;
        private SerialChainOperatorBridge Fr3 => bridge as SerialChainOperatorBridge;
        private bool RobotOn => bridge.MotionMode == JetRoverMotionMode.On;
        private bool SharedRole => role == Role.Record || role == Role.ScenarioPrevious || role == Role.ScenarioNext || role == Role.DiscardLast || role == Role.NewLayout;

        private void Update()
        {
            if (bridge == null || cap == null || Side == null || (Fr3 == null ? !SharedRole : !Fr3.IsReady))
            {
                return;
            }

            _press ??= new FingertipPress(radiusMeters, pressDepthMeters, rearmDistanceMeters, cooldownSeconds);
            bool pressed = _press.Update(cap, origin);
            if (role != Role.NewLayout && SharedRole)
            {
                // Takes of named scenarios (docs/adr/0033), with the robot on or off.
                if (pressed)
                {
                    bool asked = role == Role.Record ? Side.ToggleRecording()
                        : role == Role.DiscardLast ? Side.DiscardLastTake()
                        : StepScenario(role == Role.ScenarioNext ? 1 : -1);
                    if (asked) _flashUntil = Time.unscaledTime + 0.4f;
                }

                ShowRecording();
                return;
            }

            if (role == Role.NewLayout)
            {
                // A new seeded layout of the scene's objects (docs/adr/0031 amendment 2), only while motion is off.
                if (pressed && Side.RequestNewLayout()) _flashUntil = Time.unscaledTime + 0.6f;
                ShowNewLayout();
                return;
            }

            if (role == Role.EditObjects)
            {
                // Moving the scene's objects by hand (phase P5), only while the robot is off.
                if (pressed && (Fr3.EditObjects || Fr3.CanEditObjects)) Fr3.EditObjects = !Fr3.EditObjects;
                ShowEditObjects();
                return;
            }

            if (role == Role.Recentre)
            {
                if (pressed && Fr3.IsAbsolute)
                {
                    Fr3.Recentre();
                    _flashUntil = Time.unscaledTime + 0.6f;
                }

                ShowRecentre();
                return;
            }

            if (pressed && Fr3.MotionAllowed)
            {
                bool before = Fr3.MotionEnabled;
                Fr3.MotionEnabled = !before;
                Debug.Log($"[fr3] robot motion {(before ? "on" : "off")} -> {(Fr3.MotionEnabled ? "on" : "off")} by the button");
            }

            Show();
        }

        private bool StepScenario(int step)
        {
            if (!Side.RecordingAvailable || Side.IsRecording) return false;
            Side.StepScenario(step);
            return true;
        }

        private string _shownText = "";

        private void ShowRecording()
        {
            bool available = Side.RecordingAvailable, recording = Side.IsRecording, flash = Time.unscaledTime < _flashUntil;
            string scenario = Side.SelectedScenarioName;
            int count = Side.ScenarioCount;
            string outcome = Side.LastRecordOutcome;
            Color color;
            string text;
            if (!available)
            {
                color = lockedColor;
                text = role == Role.Record ? "RECORD\n<size=60%>RobotHost records no scenarios</size>" : role == Role.DiscardLast ? "DISCARD LAST" : role == Role.ScenarioNext ? "NEXT >" : "< PREV";
            }
            else if (role == Role.Record)
            {
                if (recording)
                {
                    int seconds = Side.TakeSeconds;
                    color = offColor;
                    text = $"STOP  <color=#ff3030>\u25CF</color> {seconds / 60:00}:{seconds % 60:00}\n<size=60%>{Side.TakeScenarioName} take {Side.TakeNumber} \u00B7 press to save</size>";
                }
                else
                {
                    color = flash || Side.RecordPending ? onColor : new Color(0.85f, 0.15f, 0.15f);
                    string tail = Side.RecordPending ? "asking RobotHost..." : outcome.Length > 0 ? outcome : $"{Side.RecordingFreeGigabytes} GB free";
                    text = $"RECORD\n<size=60%>{scenario} \u00B7 {tail}</size>";
                }
            }
            else if (role == Role.DiscardLast)
            {
                color = recording ? lockedColor : flash ? onColor : new Color(0.55f, 0.56f, 0.6f);
                text = recording ? "DISCARD LAST\n<size=60%>stop the take first</size>" : "DISCARD LAST\n<size=60%>sets the last take aside</size>";
            }
            else
            {
                color = recording || count < 2 ? lockedColor : flash ? onColor : new Color(0.25f, 0.55f, 1f);
                int at = Side.SelectedScenario + 1;
                text = role == Role.ScenarioNext ? $"NEXT >\n<size=60%>{scenario} ({at}/{count})</size>" : "< PREV";
            }

            int state = color.GetHashCode();
            if (state == _shown && text == _shownText) return;
            _shown = state;
            _shownText = text;
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null) label.SetText(text);
        }

        private void ShowNewLayout()
        {
            int state = !Side.HasWorld ? 4 : Side.WorldSceneMismatch ? 5 : Time.unscaledTime < _flashUntil ? 1 : Side.CanRequestNewLayout ? 0 : RobotOn ? 2 : 3;
            if (state == _shown)
            {
                return;
            }

            _shown = state;
            Color color = state == 1 ? onColor : state == 0 ? new Color(0.95f, 0.65f, 0.15f) : lockedColor;
            string text = state switch
            {
                0 => "NEW LAYOUT\n<size=60%>press to move the objects</size>",
                1 => "NEW LAYOUT\n<size=60%>asked</size>",
                2 => "NEW LAYOUT\n<size=60%>robot off to use</size>",
                4 => "NEW LAYOUT\n<size=60%>no scene file</size>",
                5 => "SCENE MISMATCH\n<size=60%>RobotHost runs another scene</size>",
                _ => "NEW LAYOUT\n<size=60%>waiting for the world</size>",
            };
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null) label.SetText(text);
        }

        private string _shownOutcome = "";

        private void ShowEditObjects()
        {
            string outcome = Side.LastEditOutcome;
            int state = !Side.HasWorld ? 6 : Side.WorldSceneMismatch ? 7
                : Fr3.EditObjects ? (Side.EditDragging ? 2 : Side.EditPending ? 3 : 1)
                : Fr3.CanEditObjects ? 0 : Fr3.MotionEnabled ? 4 : 5;
            if (state == _shown && ReferenceEquals(outcome, _shownOutcome))
            {
                return;
            }

            _shown = state;
            _shownOutcome = outcome;
            Color color = state == 1 || state == 2 || state == 3 ? onColor : state == 0 ? new Color(0.55f, 0.35f, 0.9f) : lockedColor;
            string text = state switch
            {
                0 => "EDIT OBJECTS\n<size=60%>press, then curl a hand by one</size>",
                1 => outcome.Length > 0 ? "EDITING\n<size=60%>" + outcome + " · press to stop</size>" : "EDITING\n<size=60%>curl by an object to grab it</size>",
                2 => "EDITING\n<size=60%>open the hand to put it down</size>",
                3 => "EDITING\n<size=60%>asking RobotHost...</size>",
                4 => "EDIT OBJECTS\n<size=60%>robot off to use</size>",
                6 => "EDIT OBJECTS\n<size=60%>no scene file</size>",
                7 => "SCENE MISMATCH\n<size=60%>RobotHost runs another scene</size>",
                _ => "EDIT OBJECTS\n<size=60%>waiting for the world</size>",
            };
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null) label.SetText(text);
        }

        private void ShowRecentre()
        {
            int state = !Fr3.IsAbsolute ? 2 : Time.unscaledTime < _flashUntil ? 1 : 0;
            if (state == _shown)
            {
                return;
            }

            _shown = state;
            Color color = state == 1 ? onColor : state == 0 ? new Color(0.25f, 0.55f, 1f) : lockedColor;
            string text = state == 2 ? "RECENTRE\n<size=60%>relative mapping: not used</size>" : "RECENTRE\n<size=60%>stand comfortably, press</size>";
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null) label.SetText(text);
        }

        private void OnApplicationFocus(bool focused)
        {
            // Builds only: the editor loses focus whenever another window is clicked. (Pausing is the bridge's.)
            if (role == Role.Power && !focused && !Application.isEditor && Fr3 != null) Fr3.MotionEnabled = false;
        }

        private void Show()
        {
            int state = !Fr3.MotionAllowed ? 2 : Fr3.MotionEnabled ? 1 : Fr3.FramesReceived == 0 ? 3 : 0;
            if (state == _shown)
            {
                return;
            }

            _shown = state;
            Color color = state == 1 ? onColor : state == 0 ? offColor : lockedColor;
            string text = state == 1 ? "ROBOT ON\n<size=60%>press to stop commanding</size>"
                : state == 2 ? "MOTION LOCKED\n<size=60%>config: AllowMotion off</size>"
                : state == 3 ? "ROBOT OFF\n<size=60%>waiting for the robot's state</size>"
                : "ROBOT OFF\n<size=60%>press to start</size>";
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null) label.SetText(text);
        }
    }
}
