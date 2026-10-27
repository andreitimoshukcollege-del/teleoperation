using System.Globalization;
using System.Text;
using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// The training course on the bench (docs/adr/0024): its START and TRAIN/TEST buttons, its panel,
    /// and the targets, curve and bead it draws in the robot's workspace. Display and buttons only;
    /// the course itself is Bridge's <see cref="JetRoverTrainingCourse"/>, driven by the bridge every
    /// frame with the commanded wrist. The buttons read fingertips (<see cref="FingertipPress"/>), like
    /// the robot's, so no ray or far pinch presses them. Coverage (seconds driven per speed) is kept
    /// between sessions in <see cref="PlayerPrefs"/>.
    /// </summary>
    public sealed class TrainingCourseView : MonoBehaviour
    {
        private const string CoverageKey = "teleop.course.coverage.v1";

        [SerializeField] private JetRoverOperatorBridge bridge;
        [SerializeField] private XROrigin origin;
        [SerializeField] private Renderer startCap;
        [SerializeField] private TMP_Text startLabel;
        [SerializeField] private Renderer purposeCap;
        [SerializeField] private TMP_Text purposeLabel;
        [SerializeField] private TMP_Text panel;
        [Tooltip("One per reach target (eight); the shuttle uses the first two.")]
        [SerializeField] private Renderer[] targets;
        [Tooltip("Above the active target: what it asks of the gripper.")]
        [SerializeField] private TMP_Text goalLabel;
        [SerializeField] private Renderer bead;
        [SerializeField] private LineRenderer path;
        [SerializeField] private Color targetColor = new Color(1f, 0.2f, 0.85f);
        [SerializeField] private Color heldColor = Color.white;
        [SerializeField] private Color runColor = new Color(0.15f, 0.95f, 0.35f);
        [SerializeField] private Color stopColor = new Color(0.9f, 0.12f, 0.1f);
        [SerializeField] private Color trainColor = new Color(0.82f, 0.86f, 0.9f);
        [SerializeField] private Color testColor = new Color(1f, 0.62f, 0.15f);

        private readonly FingertipPress _startPress = new FingertipPress(0.025f, 0.012f, 0.07f, 0.6f);
        private readonly FingertipPress _purposePress = new FingertipPress(0.02f, 0.012f, 0.07f, 0.6f);
        private readonly StringBuilder _text = new StringBuilder(512);
        private MaterialPropertyBlock _block;
        private Vector3[] _pathPoints;
        private int _pathRound = -1;
        private JetRoverCoursePhase _lastPhase = JetRoverCoursePhase.Stopped;
        private float _nextPanel;
        private bool _loaded;

        private void Update()
        {
            JetRoverTrainingCourse course = bridge != null ? bridge.Training : null;
            if (course == null)
            {
                Hide();
                if (panel != null && Time.unscaledTime >= _nextPanel)
                {
                    _nextPanel = Time.unscaledTime + 1f;
                    panel.text = "TRAINING COURSE\nnot available: the bridge has no arm rig or no collision model";
                }

                return;
            }

            if (!_loaded)
            {
                _loaded = true;
                course.LoadCoverage(LoadCoverage());
            }

            if (_startPress.Update(startCap, origin))
            {
                if (course.Running)
                {
                    course.Stop();
                    SaveCoverage(course);
                }
                else
                {
                    course.Start();
                }
            }

            if (_purposePress.Update(purposeCap, origin))
            {
                course.Test = !course.Test;
                Debug.Log($"[course] the next round's data: {(course.Test ? "test (held out)" : "train")}");
            }

            JetRoverCoursePhase phase = course.Phase;
            if (phase == JetRoverCoursePhase.Done && _lastPhase != JetRoverCoursePhase.Done)
            {
                SaveCoverage(course);
                Debug.Log($"[course] round {course.Round} {course.Task} done: score {course.Score:0.#}");
            }

            _lastPhase = phase;
            Paint(startCap, course.Running ? stopColor : runColor);
            if (startLabel != null) startLabel.text = course.Running ? "STOP\n<size=60%>course</size>" : "START\n<size=60%>course</size>";
            Paint(purposeCap, course.Test ? testColor : trainColor);
            if (purposeLabel != null) purposeLabel.text = course.Test ? "TEST\n<size=60%>held out</size>" : "TRAIN\n<size=60%>data</size>";

            Draw(course, phase);
            if (panel != null && Time.unscaledTime >= _nextPanel)
            {
                _nextPanel = Time.unscaledTime + 0.25f;
                panel.text = PanelText(course, phase);
            }
        }

        private void OnApplicationPause(bool paused)
        {
            if (paused && bridge != null && bridge.Training != null) SaveCoverage(bridge.Training);
        }

        private void OnDestroy()
        {
            if (bridge != null && bridge.Training != null && _loaded) SaveCoverage(bridge.Training);
        }

        private void Draw(JetRoverTrainingCourse course, JetRoverCoursePhase phase)
        {
            bool shown = phase != JetRoverCoursePhase.Stopped;
            JetRoverCourseTask task = course.Task;
            int count = shown && task != JetRoverCourseTask.Trace ? course.TargetCount : 0;
            int active = course.ActiveTarget;
            for (int i = 0; targets != null && i < targets.Length; i++)
            {
                Renderer r = targets[i];
                if (r == null) continue;
                // Reach shows the target to go for, and the next one faintly; the shuttle shows both.
                bool visible = i < count && (task == JetRoverCourseTask.Shuttle || i == active || i == active + 1);
                r.gameObject.SetActive(visible);
                if (!visible) continue;
                r.transform.position = course.TargetWorld(i);
                bool current = i == active;
                Color c = current ? Color.Lerp(targetColor, heldColor, task == JetRoverCourseTask.Reach ? course.HoldProgress : 0f) : targetColor * 0.35f;
                Paint(r, c);
                r.transform.localScale = Vector3.one * (current ? 0.035f : 0.022f);
            }

            if (goalLabel != null)
            {
                JetRoverGripperGoal goal = task == JetRoverCourseTask.Reach && active < count ? course.Goal(active) : JetRoverGripperGoal.Any;
                goalLabel.gameObject.SetActive(goal != JetRoverGripperGoal.Any);
                if (goal != JetRoverGripperGoal.Any)
                {
                    goalLabel.text = goal == JetRoverGripperGoal.Open ? "OPEN" : "CLOSE";
                    goalLabel.transform.position = course.TargetWorld(active) + Vector3.up * 0.045f;
                    Camera eye = Camera.main;
                    if (eye != null) goalLabel.transform.rotation = Quaternion.LookRotation(goalLabel.transform.position - eye.transform.position);
                }
            }

            bool trace = shown && task == JetRoverCourseTask.Trace && course.PathCount > 0;
            if (path != null)
            {
                path.gameObject.SetActive(trace);
                if (trace && _pathRound != (int)course.Round)
                {
                    _pathRound = (int)course.Round;
                    int n = course.PathCount;
                    if (_pathPoints == null || _pathPoints.Length != n) _pathPoints = new Vector3[n];
                    for (int i = 0; i < n; i++) _pathPoints[i] = course.PathWorld(i);
                    path.positionCount = n;
                    path.loop = true;
                    path.SetPositions(_pathPoints);
                }
            }

            if (bead != null)
            {
                bead.gameObject.SetActive(trace);
                if (trace) bead.transform.position = course.BeadWorld;
            }
        }

        private string PanelText(JetRoverTrainingCourse course, JetRoverCoursePhase phase)
        {
            StringBuilder b = _text;
            b.Clear();
            b.Append("<b>TRAINING COURSE</b>  ").Append(course.Test ? "<color=#FF9E26>TEST DATA (held out)</color>" : "train data").Append('\n');
            if (phase == JetRoverCoursePhase.Stopped)
            {
                b.Append("Press START. Targets appear around the arm:\n");
                b.Append("drive the marker to them with the clutch.\n");
                b.Append("The course sets the arm speed for each round.\n");
            }
            else
            {
                b.Append("round ").Append(course.Round).Append(" · ").Append(TaskName(course.Task)).Append(" · ");
                if (course.Varied) b.Append("varied, now ");
                b.Append(course.SpeedRadPerSec.ToString("0.0", CultureInfo.InvariantCulture)).Append(" rad/s\n");
                int left = Mathf.CeilToInt((float)course.SecondsLeft);
                switch (phase)
                {
                    case JetRoverCoursePhase.Announcing:
                        b.Append("starts in ").Append(left).Append(" s · ").Append(Instructions(course.Task)).Append('\n');
                        break;
                    case JetRoverCoursePhase.Suspended:
                        b.Append("<color=#FF9E26>paused: press ROBOT ON to carry on</color> · ").Append(left).Append(" s left\n");
                        break;
                    case JetRoverCoursePhase.Done:
                        b.Append("<b>done: ").Append(ScoreText(course)).Append("</b>\n");
                        break;
                    default:
                        b.Append(ScoreText(course)).Append(" · ").Append(left).Append(" s left\n");
                        break;
                }
            }

            if (course.Disturbed && phase != JetRoverCoursePhase.Stopped)
            {
                b.Append("<color=#FF9E26>NETWORK DISTURBED: not real-network data</color>\n");
            }

            b.Append("<size=80%>driven, min (").Append(course.Test ? "test" : "train").Append("):");
            for (int i = 0; i < JetRoverTrainingCourse.LevelCount; i++)
            {
                float level = JetRoverTrainingCourse.Level(i);
                b.Append("  ").Append(float.IsNaN(level) ? "varied" : level.ToString("0.0", CultureInfo.InvariantCulture)).Append(' ')
                    .Append((course.CoverageSeconds(course.Test, i) / 60f).ToString("0.0", CultureInfo.InvariantCulture));
            }

            b.Append("</size>");
            return b.ToString();
        }

        private static string TaskName(JetRoverCourseTask task) => task switch
        {
            JetRoverCourseTask.Reach => "REACH",
            JetRoverCourseTask.Trace => "TRACE",
            JetRoverCourseTask.Shuttle => "SHUTTLE",
            _ => "-",
        };

        private static string Instructions(JetRoverCourseTask task) => task switch
        {
            JetRoverCourseTask.Reach => "hold the marker on each target (some ask for the gripper)",
            JetRoverCourseTask.Trace => "keep the marker on the moving bead",
            JetRoverCourseTask.Shuttle => "touch the two targets in turn, fast",
            _ => "",
        };

        private static string ScoreText(JetRoverTrainingCourse course) => course.Task switch
        {
            JetRoverCourseTask.Reach => $"targets {course.Score:0}/{course.TargetCount}",
            JetRoverCourseTask.Trace => $"on the bead {course.Score:0}%",
            _ => $"touches {course.Score:0}",
        };

        private void Hide()
        {
            if (targets != null) foreach (Renderer r in targets) if (r != null) r.gameObject.SetActive(false);
            if (goalLabel != null) goalLabel.gameObject.SetActive(false);
            if (bead != null) bead.gameObject.SetActive(false);
            if (path != null) path.gameObject.SetActive(false);
        }

        private void Paint(Renderer r, Color color)
        {
            if (r == null) return;
            _block ??= new MaterialPropertyBlock();
            r.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            r.SetPropertyBlock(_block);
        }

        private static float[] LoadCoverage()
        {
            string saved = PlayerPrefs.GetString(CoverageKey, "");
            if (saved.Length == 0) return null;
            string[] parts = saved.Split(',');
            var counts = new float[parts.Length];
            for (int i = 0; i < parts.Length; i++)
            {
                if (!float.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out counts[i])) return null;
            }

            return counts;
        }

        private static void SaveCoverage(JetRoverTrainingCourse course)
        {
            float[] counts = course.SaveCoverage();
            var parts = new string[counts.Length];
            for (int i = 0; i < counts.Length; i++) parts[i] = counts[i].ToString("R", CultureInfo.InvariantCulture);
            PlayerPrefs.SetString(CoverageKey, string.Join(",", parts));
            PlayerPrefs.Save();
        }
    }
}
