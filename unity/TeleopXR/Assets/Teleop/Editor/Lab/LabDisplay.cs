using Teleop.Bridge;
using TMPro;
using UnityEditor;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// The camera display: a 1.2 × 0.9 m screen (the stream is 4:3) in a bezel on a rolling stand, just
    /// outside the workcell to the operator's right and turned to face them. A two-row status strip sits
    /// under the image.
    ///
    /// <para><b>Placement.</b> The screen's centre is about 1.4 m up and about 1.6 m from the operator's
    /// eye. That is about 40° across: big enough to read the robot's view without turning to it, and
    /// close to the arm's height, as ADR 0014 asks.</para>
    ///
    /// <para><b>The screen is a unit quad</b> scaled to 1.2 × 0.9 under a unit-scale parent, because
    /// <c>CameraFeedBridge</c> sets the panel's scale from the stream's aspect on the first frame. It
    /// uses the existing <c>CameraPanel</c> material asset (URP Unlit), which shows colour bars until
    /// then.</para>
    /// </summary>
    internal static class LabDisplay
    {
        public const float ScreenWidth = 1.2f, ScreenHeight = 0.9f;

        public sealed class Result
        {
            public Renderer Screen;
            public CameraStatusBar StatusBar;
        }

        public static Result Build(Transform parent, LabPalette p, Material screenMaterial, CameraFeedBridge feed, JetRoverOperatorBridge robot)
        {
            Transform root = LabParts.Group("CameraDisplay", parent).transform;
            root.position = LabLayout.DisplayFoot;
            Vector3 away = LabLayout.DisplayFoot - LabLayout.OperatorEye;
            away.y = 0f;
            root.rotation = Quaternion.LookRotation(away.normalized, Vector3.up);

            // Stand: H-base on casters, a column, a VESA box behind the panel.
            var stand = new LabMesh();
            const float standZ = 0.07f;
            stand.Chamfer(new Vector3(0f, 0.07f, standZ), new Vector3(0.7f, 0.04f, 0.08f), 0.008f);
            foreach (int s in new[] { -1, 1 })
            {
                stand.Chamfer(new Vector3(s * 0.33f, 0.07f, standZ), new Vector3(0.08f, 0.04f, 0.62f), 0.008f);
                foreach (int t in new[] { -1, 1 })
                {
                    stand.Cylinder(new Vector3(s * 0.33f, 0.03f, standZ + t * 0.27f), Vector3.right, 0.03f, 0.025f, 16, 0.004f);
                }
            }

            stand.Chamfer(new Vector3(0f, (0.09f + LabLayout.ScreenCentreHeight) / 2f, standZ), new Vector3(0.08f, LabLayout.ScreenCentreHeight - 0.09f, 0.05f), 0.01f);
            stand.Chamfer(new Vector3(0f, LabLayout.ScreenCentreHeight - 0.05f, 0.05f), new Vector3(0.32f, 0.26f, 0.05f), 0.01f);
            LabParts.Part("Display/Stand", root, stand, p.SteelDark, PartKind.Static, 1f);

            Transform panel = LabParts.Group("Panel", root).transform;
            panel.localPosition = new Vector3(0f, LabLayout.ScreenCentreHeight, 0f);

            // Bezel around the screen and the status strip below it.
            const float stripHeight = 0.1f, margin = 0.035f;
            float top = ScreenHeight / 2f + margin, bottom = -ScreenHeight / 2f - 0.012f - stripHeight - margin;
            var bezel = new LabMesh();
            bezel.Chamfer(new Vector3(0f, (top + bottom) / 2f, 0f), new Vector3(ScreenWidth + 2 * margin, top - bottom, 0.045f), 0.012f);
            LabParts.Part("Display/Bezel", panel, bezel, p.Bezel, PartKind.Static, 2f);

            // The screen: a unit quad, scaled. Facing -z, toward the operator.
            var quad = new LabMesh();
            quad.Screen(Vector3.zero, Vector3.back, Vector3.up, Vector2.one);
            var screenGo = new GameObject("Screen");
            screenGo.transform.SetParent(panel, false);
            screenGo.transform.localPosition = new Vector3(0f, 0f, -0.0235f);
            screenGo.transform.localScale = new Vector3(ScreenWidth, ScreenHeight, 1f);
            screenGo.AddComponent<MeshFilter>().sharedMesh = LabMeshLibrary.Store("Display/Screen", quad, false);
            var screen = screenGo.AddComponent<MeshRenderer>();
            screen.sharedMaterial = screenMaterial;
            LabParts.Configure(screen, PartKind.Emitter);

            float stripY = -ScreenHeight / 2f - 0.012f - stripHeight / 2f;
            var strip = new LabMesh();
            strip.Screen(new Vector3(0f, stripY, -0.0234f), Vector3.back, Vector3.up, new Vector2(ScreenWidth, stripHeight));
            LabParts.Part("Display/Strip", panel, strip, p.StripGlass, PartKind.Emitter);

            var underglow = new LabMesh();
            underglow.Box(new Vector3(0f, bottom - 0.004f, -0.02f), new Vector3(ScreenWidth, 0.006f, 0.006f));
            LabParts.Part("Display/Underglow", panel, underglow, p.Underglow, PartKind.Emitter);

            var dot = new LabMesh();
            dot.Cylinder(new Vector3(-ScreenWidth / 2f + 0.03f, stripY + 0.024f, -0.0245f), Vector3.forward, 0.009f, 0.002f, 16);
            Renderer liveDot = LabParts.Part("Display/LiveDot", panel, dot, p.LiveDot, PartKind.Emitter);

            float textZ = -0.026f, left = -ScreenWidth / 2f + 0.055f;
            var white = new Color(0.86f, 0.9f, 0.95f);
            TextMeshPro feedLabel = LabParts.Label("FeedLabel", panel, new Vector3(left + 0.3f, stripY + 0.024f, textZ), Quaternion.identity,
                "NO SIGNAL  ·  waiting for the robot camera", 0.26f, white, new Vector2(0.6f, 0.04f), TextAlignmentOptions.Left, FontStyles.Bold);
            TextMeshPro linkLabel = LabParts.Label("LinkLabel", panel, new Vector3(ScreenWidth / 2f - 0.33f, stripY + 0.024f, textZ), Quaternion.identity,
                "ROBOT NOT CONNECTED", 0.26f, new Color(0.55f, 0.92f, 1f), new Vector2(0.6f, 0.04f), TextAlignmentOptions.Right, FontStyles.Bold);
            TextMeshPro numbersLabel = LabParts.Label("NumbersLabel", panel, new Vector3(left + 0.3f, stripY - 0.022f, textZ), Quaternion.identity,
                "0 fps   decode 0.0 ms   0x0", 0.22f, new Color(0.6f, 0.66f, 0.74f), new Vector2(0.6f, 0.04f), TextAlignmentOptions.Left);
            LabParts.Label("DisplayCaption", panel, new Vector3(ScreenWidth / 2f - 0.33f, stripY - 0.022f, textZ), Quaternion.identity,
                "robot camera  ·  Orbbec DaBai DCW", 0.2f, new Color(0.45f, 0.5f, 0.58f), new Vector2(0.6f, 0.04f), TextAlignmentOptions.Right);

            var bar = root.gameObject.AddComponent<CameraStatusBar>();
            var so = new SerializedObject(bar);
            so.FindProperty("feed").objectReferenceValue = feed;
            so.FindProperty("robot").objectReferenceValue = robot;
            so.FindProperty("screen").objectReferenceValue = screen;
            so.FindProperty("feedLabel").objectReferenceValue = feedLabel;
            so.FindProperty("numbersLabel").objectReferenceValue = numbersLabel;
            so.FindProperty("linkLabel").objectReferenceValue = linkLabel;
            so.FindProperty("liveDot").objectReferenceValue = liveDot;
            so.ApplyModifiedPropertiesWithoutUndo();
            return new Result { Screen = screen, StatusBar = bar };
        }
    }
}
