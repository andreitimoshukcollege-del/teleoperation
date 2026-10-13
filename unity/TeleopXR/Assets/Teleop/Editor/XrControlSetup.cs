using System;
using System.Linq;
using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Puts clutch control and the off-hand gripper into the scene (docs/adr/0017). Idempotent, and
    /// touches only what it owns:
    /// <list type="bullet">
    /// <item>the drag target loses its grab interactable, rigidbody, collider and old highlight script, and
    /// shrinks to a marker. It stays the bridge's input; the clutch moves it now;</item>
    /// <item><see cref="HandClutchDriver"/> is added next to <see cref="JetRoverOperatorBridge"/> and
    /// wired to the arm rig, the XR Origin and the display's caption line;</item>
    /// <item><see cref="TwinGripper"/> is added on the wrist's twin visual and wired to its fingers and
    /// its roll group;</item>
    /// <item>the robot's start/stop button (<see cref="MotionToggleButton"/>) is built on the bench's
    /// front-left corner, and the bridge is set to start with motion <b>off</b>;</item>
    /// <item>the twin's predicted and last-measured arms, the commanded arm's ghost appearance, and
    /// three switches that show or hide each arm (docs/adr/0018).</item>
    /// </list>
    /// <see cref="LabDressing.Apply"/> calls it after every rebuild, because a rebuild replaces the twin.
    /// <b>Teleop → XR → Set Up Clutch Control</b>, or headless:
    /// <c>just _unity-batch TeleopXR Teleop.XR.Editor.XrControlSetup.Run</c>.
    /// </summary>
    public static class XrControlSetup
    {
        public const float MarkerDiameter = 0.02f;

        [MenuItem("Teleop/XR/Set Up Clutch Control")]
        public static void RunFromMenu()
        {
            Apply(SceneManager.GetActiveScene());
            EditorSceneManager.SaveOpenScenes();
        }

        public static void Run()
        {
            try
            {
                Scene scene = EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
                Apply(scene);
                EditorSceneManager.SaveScene(scene);
                Debug.Log("[xr-scene] clutch control set up");
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[xr-scene] FAILED: {e}");
                EditorApplication.Exit(1);
            }
        }

        /// <summary>
        /// The arm the clutch commands: the bridge's own <c>armRig</c>. The scene also holds the
        /// predicted and measured twin arms, which are rigs too, so "any rig" is not good enough.
        /// </summary>
        public static JetRoverArmRig CommandedRig()
        {
            var bridge = Object.FindAnyObjectByType<JetRoverOperatorBridge>(FindObjectsInactive.Include);
            if (bridge != null && new SerializedObject(bridge).FindProperty("armRig").objectReferenceValue is JetRoverArmRig rig)
            {
                return rig;
            }

            return Object.FindAnyObjectByType<JetRoverArmRig>(FindObjectsInactive.Include);
        }

        public static void Apply(Scene scene)
        {
            var bridge = Object.FindAnyObjectByType<JetRoverOperatorBridge>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no JetRoverOperatorBridge");
            var rig = CommandedRig()
                ?? throw new InvalidOperationException("the scene has no JetRoverArmRig");
            var origin = Object.FindAnyObjectByType<XROrigin>(FindObjectsInactive.Include)
                ?? throw new InvalidOperationException("the scene has no XR Origin");
            var target = new SerializedObject(bridge).FindProperty("dragTarget").objectReferenceValue as Transform
                ?? throw new InvalidOperationException("JetRoverOperatorBridge.dragTarget is not assigned");

            StripGrab(target);

            HandClutchDriver driver = bridge.GetComponent<HandClutchDriver>() ?? bridge.gameObject.AddComponent<HandClutchDriver>();
            var driverSo = new SerializedObject(driver);
            driverSo.FindProperty("bridge").objectReferenceValue = bridge;
            driverSo.FindProperty("armRig").objectReferenceValue = rig;
            driverSo.FindProperty("origin").objectReferenceValue = origin;
            TMP_Text caption = Object.FindObjectsByType<TMP_Text>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .FirstOrDefault(t => t.name == "DisplayCaption");
            driverSo.FindProperty("statusLabel").objectReferenceValue = caption;
            driverSo.ApplyModifiedPropertiesWithoutUndo();

            // Off until the button is pressed: nothing reaches the robot at launch.
            var bridgeSo = new SerializedObject(bridge);
            bridgeSo.FindProperty("motionEnabledAtStart").boolValue = false;
            bridgeSo.ApplyModifiedPropertiesWithoutUndo();
            BuildMotionButton(scene, bridge, origin);

            var upper = new SerializedObject(rig).FindProperty("upperPitchPivot").objectReferenceValue as Transform;
            Transform visual = upper != null ? upper.Find(JetRoverTwin.VisualName) : null;
            if (visual != null)
            {
                TwinGripper twin = visual.GetComponent<TwinGripper>() ?? visual.gameObject.AddComponent<TwinGripper>();
                var twinSo = new SerializedObject(twin);
                twinSo.FindProperty("bridge").objectReferenceValue = bridge;
                Transform roll = visual.Find(JetRoverTwin.RollName);
                twinSo.FindProperty("rollPivot").objectReferenceValue = roll;
                twinSo.FindProperty("leftFinger").objectReferenceValue = roll != null ? roll.Find(JetRoverTwin.FingerName + "L") : null;
                twinSo.FindProperty("rightFinger").objectReferenceValue = roll != null ? roll.Find(JetRoverTwin.FingerName + "R") : null;
                twinSo.FindProperty("travelMeters").floatValue = JetRoverTwin.FingerTravel;
                twinSo.ApplyModifiedPropertiesWithoutUndo();
            }

            BuildTwinArms(scene, bridge, rig, origin);
            BuildSpeedSlider(scene, bridge, origin);
            BuildDisplayHandle(scene, driver);
            EditorSceneManager.MarkSceneDirty(scene);
        }

        public const string SpeedSliderName = "SpeedSlider", DisplayHandleName = "MoveHandle";

        /// <summary>
        /// The arm-speed slider (docs/adr/0018): a track on the bench's left, running away from the
        /// operator (near = slow, far = fast), with a knob and a readout. Rebuilt on every run.
        /// </summary>
        private static void BuildSpeedSlider(Scene scene, JetRoverOperatorBridge bridge, XROrigin origin)
        {
            foreach (GameObject root in scene.GetRootGameObjects().Where(r => r.name == SpeedSliderName || r.name == DelaySliderName).ToList())
            {
                Object.DestroyImmediate(root);
            }

            BuildSlider(SpeedSliderName, 0f, SpeedSlider.Setting.ArmSpeed, 0.5f, 6f, 3.8f, "ARM SPEED", bridge, origin);
            // Beside it: an added delay on the robot's reports (0-400 ms), to see the prediction lead on a fast LAN.
            BuildSlider(DelaySliderName, -0.07f, SpeedSlider.Setting.AddedDelay, 0f, 400f, 0f, "ADDED DELAY", bridge, origin);
        }

        public const string DelaySliderName = "DelaySlider";

        private static void BuildSlider(string name, float xOffset, SpeedSlider.Setting setting, float min, float max, float start,
            string caption, JetRoverOperatorBridge bridge, XROrigin origin)
        {
            float top = LabLayout.BenchTop;
            const float length = 0.26f, height = 0.012f;
            var slider = new GameObject(name).transform;
            slider.position = new Vector3(LabLayout.ArmBase.x - 0.70f + xOffset, top, LabLayout.BenchNearZ + 0.05f);

            GameObject track = GameObject.CreatePrimitive(PrimitiveType.Cube);
            track.name = "Track";
            track.transform.SetParent(slider, false);
            track.transform.localPosition = new Vector3(0f, height / 2f, length / 2f);
            track.transform.localScale = new Vector3(0.022f, height, length);
            track.GetComponent<Renderer>().sharedMaterial = LabMaterials.Lit("MotionButtonPlinth", new Color(0.06f, 0.062f, 0.068f), 0.6f, 0.5f);

            var slow = new GameObject("NearEnd").transform;
            slow.SetParent(slider, false);
            slow.localPosition = new Vector3(0f, height, 0.01f);
            var fast = new GameObject("FarEnd").transform;
            fast.SetParent(slider, false);
            fast.localPosition = new Vector3(0f, height, length - 0.01f);

            GameObject knob = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            knob.name = "Knob";
            knob.transform.SetParent(slider, false);
            knob.transform.localScale = new Vector3(0.03f, 0.006f, 0.03f);
            knob.transform.localPosition = new Vector3(0f, height, 0.01f);
            // Neutral white: blue, green and amber belong to the twin's arms (TwinColors).
            Color knobColor = new Color(0.88f, 0.9f, 0.93f);
            knob.GetComponent<Renderer>().sharedMaterial = LabMaterials.Unlit(setting == SpeedSlider.Setting.ArmSpeed ? "SpeedKnob" : "DelayKnob", knobColor);
            foreach (Collider collider in slider.GetComponentsInChildren<Collider>()) Object.DestroyImmediate(collider);

            // Labels stand at the slider's near end, tilted toward the operator; the two sliders' labels alternate depth so they never overlap.
            float labelZ = setting == SpeedSlider.Setting.ArmSpeed ? -0.03f : -0.055f;
            TextMeshPro label = LabParts.Label("Label", slider, new Vector3(0f, 0.006f, labelZ), Quaternion.Euler(60f, 0f, 0f),
                caption, 0.1f, knobColor, new Vector2(0.2f, 0.02f), TextAlignmentOptions.Center, FontStyles.Bold);

            var component = slider.gameObject.AddComponent<SpeedSlider>();
            var so = new SerializedObject(component);
            so.FindProperty("setting").enumValueIndex = (int)setting;
            so.FindProperty("bridge").objectReferenceValue = bridge;
            so.FindProperty("origin").objectReferenceValue = origin;
            so.FindProperty("trackStart").objectReferenceValue = slow;
            so.FindProperty("trackEnd").objectReferenceValue = fast;
            so.FindProperty("knob").objectReferenceValue = knob.transform;
            so.FindProperty("label").objectReferenceValue = label;
            so.FindProperty("minRadPerSec").floatValue = min;
            so.FindProperty("maxRadPerSec").floatValue = max;
            so.FindProperty("startRadPerSec").floatValue = start;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        /// <summary>
        /// A bar on top of the camera display, the <see cref="DisplayMover"/> that moves the display
        /// when the clutch hand points at it and pinches (robot motion off only), and the pointer line. Rebuilt on every run; the display itself is the
        /// lab's (LabDisplay).
        /// </summary>
        private static void BuildDisplayHandle(Scene scene, HandClutchDriver driver)
        {
            Transform display = scene.GetRootGameObjects()
                .SelectMany(r => r.GetComponentsInChildren<Transform>(true))
                .FirstOrDefault(t => t.name == "CameraDisplay");
            if (display == null)
            {
                Debug.LogWarning("[xr-scene] no CameraDisplay in the scene; the display cannot be moved (rebuild the lab)");
                return;
            }

            Transform old = display.Find(DisplayHandleName);
            if (old != null) Object.DestroyImmediate(old.gameObject);

            Bounds bounds = default;
            bool any = false;
            foreach (Renderer r in display.GetComponentsInChildren<Renderer>(true))
            {
                if (!any) { bounds = r.bounds; any = true; } else bounds.Encapsulate(r.bounds);
            }

            GameObject handle = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            handle.name = DisplayHandleName;
            handle.transform.SetParent(display, false);
            handle.transform.position = new Vector3(bounds.center.x, bounds.max.y + 0.05f, bounds.center.z);
            handle.transform.localRotation = Quaternion.Euler(0f, 0f, 90f); // along the display's width
            handle.transform.localScale = new Vector3(0.025f, 0.16f, 0.025f);
            handle.GetComponent<Renderer>().sharedMaterial = LabMaterials.Unlit("DisplayHandle", new Color(0.35f, 0.85f, 1f));
            Object.DestroyImmediate(handle.GetComponent<Collider>());

            DisplayMover mover = display.GetComponent<DisplayMover>() ?? display.gameObject.AddComponent<DisplayMover>();
            var so = new SerializedObject(mover);
            so.FindProperty("handle").objectReferenceValue = handle.GetComponent<Renderer>();
            so.FindProperty("grabMargin").floatValue = 0.05f;
            so.ApplyModifiedPropertiesWithoutUndo();

            // The pointer line, drawn from the hand to the display while the ray is on it.
            Transform oldPointer = driver.transform.Find("DisplayPointer");
            if (oldPointer != null) Object.DestroyImmediate(oldPointer.gameObject);
            var pointerObject = new GameObject("DisplayPointer");
            pointerObject.transform.SetParent(driver.transform, false);
            var line = pointerObject.AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.positionCount = 2;
            line.startWidth = 0.004f;
            line.endWidth = 0.002f;
            line.sharedMaterial = LabMaterials.Transparent("DisplayPointer", new Color(1f, 0.85f, 0.25f, 0.8f));
            line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.enabled = false;

            var driverSo = new SerializedObject(driver);
            driverSo.FindProperty("displayMover").objectReferenceValue = mover;
            driverSo.FindProperty("pointer").objectReferenceValue = line;
            driverSo.ApplyModifiedPropertiesWithoutUndo();
        }

        public const string PredictedArmName = "PredictedArm", MeasuredArmName = "MeasuredArm", TwinSwitchesName = "TwinSwitches";

        /// <summary>
        /// The predicted and last-measured arms (docs/adr/0018): copies of the commanded arm's
        /// hierarchy, made after its twin visuals are built, so each is a full JetRover arm on the
        /// same base anchor. Their gripper display is fed by <see cref="TwinArmsDriver"/>, not the
        /// bridge. The commanded arm turns into a ghost and the measured one turns faint at runtime
        /// (<see cref="ArmAppearance"/>). Three fingertip switches beside the motion button show or
        /// hide each arm. Everything here is rebuilt on every run.
        /// </summary>
        private static void BuildTwinArms(Scene scene, JetRoverOperatorBridge bridge, JetRoverArmRig rig, XROrigin origin)
        {
            foreach (GameObject root in scene.GetRootGameObjects()
                         .Where(r => r.name == PredictedArmName || r.name == MeasuredArmName || r.name == TwinSwitchesName).ToList())
            {
                Object.DestroyImmediate(root);
            }

            // The commanded arm keeps its reach-warning LED red-capable; everything else turns ghostly.
            var rigSo = new SerializedObject(rig);
            var reachLed = rigSo.FindProperty("reachWarningRenderer").objectReferenceValue as Renderer;
            Color ghost = TwinColors.Commanded;
            ghost.a = 0.25f;
            SetAppearance(rig.gameObject, LabMaterials.Transparent("TwinGhost", ghost), reachLed);
            // The clutch's "you are driving" cue on the commanded arm's wrist LED is its colour too.
            var commandedRigSo = new SerializedObject(rig);
            commandedRigSo.FindProperty("highlightColor").colorValue = TwinColors.Commanded;
            commandedRigSo.ApplyModifiedPropertiesWithoutUndo();
            TintTarget(bridge);

            JetRoverArmRig predicted = CloneArm(rig, PredictedArmName, out TwinGripper predictedGripper);
            Object.DestroyImmediate(predicted.GetComponent<ArmAppearance>());
            JetRoverArmRig measured = CloneArm(rig, MeasuredArmName, out TwinGripper measuredGripper);
            Color faint = TwinColors.Measured;
            faint.a = 0.3f;
            SetAppearance(measured.gameObject, LabMaterials.Transparent("TwinMeasured", faint), null);
            // Each arm's wrist LED rests in its own colour, so even the solid predicted arm is marked.
            SetLedColor(predicted, TwinColors.Predicted);
            SetLedColor(measured, TwinColors.Measured);

            ArmDriverFor(bridge, out TwinArmsDriver driver);
            var so = new SerializedObject(driver);
            so.FindProperty("bridge").objectReferenceValue = bridge;
            so.FindProperty("predictedRig").objectReferenceValue = predicted;
            so.FindProperty("predictedGripper").objectReferenceValue = predictedGripper;
            so.FindProperty("measuredRig").objectReferenceValue = measured;
            so.FindProperty("measuredGripper").objectReferenceValue = measuredGripper;
            GameObject[] visuals = rig.GetComponentsInChildren<Transform>(true)
                .Where(t => t.name == JetRoverTwin.VisualName).Select(t => t.gameObject).ToArray();
            SerializedProperty array = so.FindProperty("commandedVisuals");
            array.arraySize = visuals.Length;
            for (int i = 0; i < visuals.Length; i++) array.GetArrayElementAtIndex(i).objectReferenceValue = visuals[i];

            // The switches: a row to the right of the motion button, with the stream's status above.
            float top = LabLayout.BenchTop;
            var switches = new GameObject(TwinSwitchesName).transform;
            switches.position = new Vector3(LabLayout.ArmBase.x - 0.5f + 0.13f, top, LabLayout.BenchNearZ + 0.12f);
            Material capMaterial = LabMaterials.Unlit("TwinSwitchCap", Color.white);
            Material plinthMaterial = LabMaterials.Lit("MotionButtonPlinth", new Color(0.06f, 0.062f, 0.068f), 0.6f, 0.5f);
            var arms = new[] { (TwinVisibilityButton.Arm.Predicted, "PREDICTED"), (TwinVisibilityButton.Arm.Commanded, "COMMANDED"), (TwinVisibilityButton.Arm.Measured, "MEASURED") };
            Color[] armColors = { TwinColors.Predicted, TwinColors.Commanded, TwinColors.Measured };
            for (int i = 0; i < arms.Length; i++)
            {
                var button = new GameObject(arms[i].Item2).transform;
                button.SetParent(switches, false);
                button.localPosition = new Vector3(i * 0.09f, 0f, 0f);
                GameObject plinth = GameObject.CreatePrimitive(PrimitiveType.Cube);
                plinth.name = "Plinth";
                plinth.transform.SetParent(button, false);
                plinth.transform.localPosition = new Vector3(0f, 0.01f, 0f);
                plinth.transform.localScale = new Vector3(0.06f, 0.02f, 0.06f);
                plinth.GetComponent<Renderer>().sharedMaterial = plinthMaterial;
                GameObject cap = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                cap.name = "Cap";
                cap.transform.SetParent(button, false);
                cap.transform.localPosition = new Vector3(0f, 0.025f, 0f);
                cap.transform.localScale = new Vector3(0.036f, 0.005f, 0.036f);
                cap.GetComponent<Renderer>().sharedMaterial = capMaterial;
                foreach (Collider collider in button.GetComponentsInChildren<Collider>()) Object.DestroyImmediate(collider);
                // In front of the button, tilted up toward the operator, so neighbouring labels never overlap.
                TextMeshPro label = LabParts.Label("Label", button, new Vector3(0f, 0.006f, -0.042f), Quaternion.Euler(60f, 0f, 0f),
                    arms[i].Item2, 0.11f, armColors[i], new Vector2(0.085f, 0.02f), TextAlignmentOptions.Center, FontStyles.Bold);

                var toggle = button.gameObject.AddComponent<TwinVisibilityButton>();
                var toggleSo = new SerializedObject(toggle);
                toggleSo.FindProperty("arms").objectReferenceValue = driver;
                toggleSo.FindProperty("arm").enumValueIndex = (int)arms[i].Item1;
                toggleSo.FindProperty("origin").objectReferenceValue = origin;
                toggleSo.FindProperty("cap").objectReferenceValue = cap.GetComponent<Renderer>();
                toggleSo.FindProperty("label").objectReferenceValue = label;
                toggleSo.FindProperty("onColor").colorValue = armColors[i];
                toggleSo.FindProperty("offColor").colorValue = armColors[i] * 0.18f;
                toggleSo.ApplyModifiedPropertiesWithoutUndo();
            }

            TextMeshPro status = LabParts.Label("Status", switches, new Vector3(0.09f, 0.075f, 0.05f), Quaternion.Euler(15f, 0f, 0f),
                "NO ROBOT STATE", 0.13f, new Color(0.7f, 0.85f, 1f), new Vector2(0.45f, 0.03f), TextAlignmentOptions.Center);
            so.FindProperty("statusLabel").objectReferenceValue = status;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        /// <summary>The twin's colour code (docs/adr/0018): each arm, its switch, and the target share one colour.</summary>
        internal static class TwinColors
        {
            /// <summary>Where you told it to go: the target marker, the ghost arm, the clutch cue.</summary>
            public static readonly Color Commanded = new Color(0.25f, 0.55f, 1f);

            /// <summary>Where the robot most likely is now: the solid arm, in the robot's own green.</summary>
            public static readonly Color Predicted = new Color(0.2f, 0.9f, 0.4f);

            /// <summary>The robot's last report, as received: the faint arm.</summary>
            public static readonly Color Measured = new Color(1f, 0.72f, 0.3f);
        }

        private static void SetLedColor(JetRoverArmRig rig, Color color)
        {
            var so = new SerializedObject(rig);
            so.FindProperty("normalColor").colorValue = color;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        /// <summary>The target marker takes the commanded colour; its material is the scene builder's asset.</summary>
        private static void TintTarget(JetRoverOperatorBridge bridge)
        {
            var target = new SerializedObject(bridge).FindProperty("dragTarget").objectReferenceValue as Transform;
            Renderer r = target != null ? target.GetComponent<Renderer>() : null;
            if (r == null || r.sharedMaterial == null)
            {
                return;
            }

            r.sharedMaterial.SetColor("_BaseColor", TwinColors.Commanded);
            r.sharedMaterial.SetColor("_Color", TwinColors.Commanded);
            EditorUtility.SetDirty(r.sharedMaterial);
        }

        private static JetRoverArmRig CloneArm(JetRoverArmRig rig, string name, out TwinGripper gripper)
        {
            GameObject copy = Object.Instantiate(rig.gameObject);
            copy.name = name;
            copy.transform.SetPositionAndRotation(rig.transform.position, rig.transform.rotation);
            gripper = copy.GetComponentInChildren<TwinGripper>(true);
            if (gripper != null)
            {
                var so = new SerializedObject(gripper);
                so.FindProperty("bridge").objectReferenceValue = null; // fed by TwinArmsDriver
                so.ApplyModifiedPropertiesWithoutUndo();
            }

            copy.SetActive(false); // TwinArmsDriver shows it once the robot's state arrives
            return copy.GetComponent<JetRoverArmRig>();
        }

        private static void SetAppearance(GameObject arm, Material material, Renderer keep)
        {
            ArmAppearance appearance = arm.GetComponent<ArmAppearance>() ?? arm.AddComponent<ArmAppearance>();
            var so = new SerializedObject(appearance);
            so.FindProperty("material").objectReferenceValue = material;
            SerializedProperty keepArray = so.FindProperty("keep");
            keepArray.arraySize = keep != null ? 1 : 0;
            if (keep != null) keepArray.GetArrayElementAtIndex(0).objectReferenceValue = keep;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        private static void ArmDriverFor(JetRoverOperatorBridge bridge, out TwinArmsDriver driver)
        {
            driver = bridge.GetComponent<TwinArmsDriver>() ?? bridge.gameObject.AddComponent<TwinArmsDriver>();
        }

        public const string MotionButtonName = "MotionButton";

        /// <summary>
        /// A red/green push button on the bench's front-left corner, out of the arm's reach and to the
        /// left of where the right hand clutches, with a label behind it. Rebuilt on every run.
        /// </summary>
        private static void BuildMotionButton(Scene scene, JetRoverOperatorBridge bridge, XROrigin origin)
        {
            foreach (GameObject root in scene.GetRootGameObjects().Where(r => r.name == MotionButtonName).ToList())
            {
                Object.DestroyImmediate(root);
            }

            float top = LabLayout.BenchTop;
            var button = new GameObject(MotionButtonName).transform;
            button.position = new Vector3(LabLayout.ArmBase.x - 0.5f, top, LabLayout.BenchNearZ + 0.12f);

            GameObject plinth = GameObject.CreatePrimitive(PrimitiveType.Cube);
            plinth.name = "Plinth";
            plinth.transform.SetParent(button, false);
            plinth.transform.localPosition = new Vector3(0f, 0.015f, 0f);
            plinth.transform.localScale = new Vector3(0.11f, 0.03f, 0.11f);
            plinth.GetComponent<Renderer>().sharedMaterial = LabMaterials.Lit("MotionButtonPlinth", new Color(0.06f, 0.062f, 0.068f), 0.6f, 0.5f);

            GameObject cap = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cap.name = "Cap";
            cap.transform.SetParent(button, false);
            cap.transform.localPosition = new Vector3(0f, 0.038f, 0f);
            cap.transform.localScale = new Vector3(0.06f, 0.008f, 0.06f); // a cylinder primitive is 2 units tall
            cap.GetComponent<Renderer>().sharedMaterial = LabMaterials.Unlit("MotionButtonCap", new Color(0.9f, 0.12f, 0.1f)); // red = off; recoloured at runtime

            foreach (Collider collider in button.GetComponentsInChildren<Collider>())
            {
                Object.DestroyImmediate(collider); // no XRI interactor may press it; the fingertip check does
            }

            TextMeshPro label = LabParts.Label("Label", button, new Vector3(0f, 0.11f, 0.075f), Quaternion.Euler(15f, 0f, 0f),
                "ROBOT OFF", 0.45f, Color.white, new Vector2(0.3f, 0.1f), TextAlignmentOptions.Center, FontStyles.Bold);

            var toggle = button.gameObject.AddComponent<MotionToggleButton>();
            var so = new SerializedObject(toggle);
            so.FindProperty("bridge").objectReferenceValue = bridge;
            so.FindProperty("origin").objectReferenceValue = origin;
            so.FindProperty("cap").objectReferenceValue = cap.GetComponent<Renderer>();
            so.FindProperty("label").objectReferenceValue = label;
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        /// <summary>Removes everything that made the target grabbable, dependents first.</summary>
        private static void StripGrab(Transform target)
        {
            // The old GrabHighlight script is gone, so an older scene shows it as a missing script.
            GameObjectUtility.RemoveMonoBehavioursWithMissingScript(target.gameObject);

            // The interactable, then its grab transformers (XRI adds them alongside), then what it required.
            foreach (MonoBehaviour behaviour in target.GetComponents<MonoBehaviour>()
                         .Where(b => b != null && b.GetType().Namespace != null
                                     && b.GetType().Namespace.StartsWith("UnityEngine.XR.Interaction.Toolkit", StringComparison.Ordinal))
                         .OrderBy(b => b is UnityEngine.XR.Interaction.Toolkit.Interactables.XRBaseInteractable ? 0 : 1)
                         .ToList())
            {
                Object.DestroyImmediate(behaviour);
            }

            foreach (Collider collider in target.GetComponents<Collider>())
            {
                Object.DestroyImmediate(collider);
            }

            if (target.TryGetComponent(out Rigidbody body))
            {
                Object.DestroyImmediate(body);
            }

            target.localScale = Vector3.one * MarkerDiameter;
            EditorUtility.SetDirty(target.gameObject);
        }
    }
}
