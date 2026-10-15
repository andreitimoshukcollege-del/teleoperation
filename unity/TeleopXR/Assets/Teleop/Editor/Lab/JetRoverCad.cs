using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// The JetRover twin from the robot's own CAD (<see cref="Folder"/>: Body, Base, LowerArm,
    /// CenterArm, UpperArm, Gripper; one FBX per URDF link, exported from Blender by the user), fitted
    /// onto the arm rig. It replaces <see cref="JetRoverTwin"/>'s procedural geometry when the files
    /// are present, and builds the same structure: a <c>JetRoverChassis</c> root, a
    /// <see cref="JetRoverTwin.VisualName"/> group under each pivot, a <see cref="JetRoverTwin.RollName"/>
    /// group for the wrist roll, and the wrist LED.
    ///
    /// <para><b>How the files are read</b> (measured with <see cref="RobotModelInspector"/>):</para>
    /// <list type="bullet">
    /// <item>Each file's root carries the part's place in one assembly, with the arm standing straight
    /// up and the robot facing −z. Positions are in units 4.4435× a metre: the wheels come out 0.43 m
    /// across, the real ones 0.097 m.</item>
    /// <item>Base.fbx (the turntable) was exported without the 270° X rotation every other part has;
    /// <see cref="BaseFix"/> restores it. It then sits on the arm's axis 0.034 m under the shoulder,
    /// where the repo's measured arm puts it (0.035).</item>
    /// <item>The links are 0.1304 and 0.1291 m joint to joint, which matches the measured 0.13 m arm. Each
    /// part is snapped onto its own pivot, so the IK's 0.13 m is what moves.</item>
    /// </list>
    /// Materials are replaced from the lab palette by the imported material's name, so nothing depends on
    /// how the FBX materials imported.
    /// </summary>
    internal static class JetRoverCad
    {
        public const string Folder = RobotModelInspector.Folder;
        private const float UnitsPerMetre = 4.4435f;

        /// <summary>The 270° about X that Base.fbx lacks.</summary>
        private static readonly Quaternion BaseFix = Quaternion.Euler(270f, 0f, 0f);

        public static bool Available => new[] { "Body", "Base", "LowerArm", "CenterArm", "UpperArm", "Gripper" }
            .All(n => File.Exists(Path.Combine(Folder, n + ".fbx")));

        /// <summary>Wheel bottom below the arm's yaw pivot, from the CAD (m): the bench is set from it.</summary>
        public const float WheelBottomBelowYawPivot = 0.1962f;

        /// <summary>The wrist pitch pivot to the wrist roll joint, along the last link, from the CAD (m).</summary>
        public const float WristToRoll = 0.0531f;

        public static Renderer Build(Transform parent, LabPalette p, Transform yaw, Transform lower, Transform middle, Transform upper)
        {
            // The CAD robot faces -z; the rig's forward is +z.
            Quaternion facing = Quaternion.Euler(0f, 180f, 0f);
            // The CAD arm stands straight up; the rig's rest pose points it straight forward.
            Quaternion tipForward = Quaternion.Euler(90f, 0f, 0f) * facing;

            GameObject baseAsset = Load("Base");
            Vector3 yawPointCad = BaseFix * baseAsset.transform.localPosition;
            Vector3 shoulderPointCad = Load("LowerArm").transform.localPosition;

            // The chassis and the turntable, about the yaw pivot.
            Transform chassis = LabParts.Group("JetRoverChassis", parent).transform;
            chassis.position = yaw.position;
            Place("Body", chassis, yaw.position, yawPointCad, facing, false, p);
            Transform baseVisual = Visual(yaw);
            Place("Base", baseVisual, yaw.position, yawPointCad, facing, true, p);

            // The arm, tipped forward about the shoulder, each part snapped to its pivot.
            Transform lowerVisual = Visual(lower);
            Snap(Place("LowerArm", lowerVisual, lower.position, shoulderPointCad, tipForward, false, p), lower);
            Transform middleVisual = Visual(middle);
            Snap(Place("CenterArm", middleVisual, lower.position, shoulderPointCad, tipForward, false, p), middle);
            Transform upperVisual = Visual(upper);
            Snap(Place("UpperArm", upperVisual, lower.position, shoulderPointCad, tipForward, false, p), upper);

            // The gripper rolls about the last link's axis: under the roll group, which TwinGripper turns.
            Transform roll = LabParts.Group(JetRoverTwin.RollName, upperVisual).transform;
            GameObject gripper = Place("Gripper", roll, lower.position, shoulderPointCad, tipForward, false, p);
            // Onto the link's axis, WristToRoll along it: the CAD puts it there to within 0.2 mm.
            Vector3 local = upper.InverseTransformPoint(gripper.transform.position);
            gripper.transform.position = upper.TransformPoint(new Vector3(0f, 0f, local.z));

            // The reach warning: a ring around the roll joint, recoloured whole by JetRoverArmRig.
            var led = new LabMesh();
            led.Torus(new Vector3(0f, 0f, WristToRoll - 0.004f), Vector3.forward, 0.026f, 0.0024f, 28, 8);
            return LabParts.Part("Arm/Wrist/ReachLed", upperVisual, led, p.WristLed, PartKind.Emitter);
        }

        /// <summary>
        /// One part at real scale. Its root goes where the assembly puts it, relative to
        /// <paramref name="referenceCad"/>, which lands on <paramref name="anchor"/>, rotated by
        /// <paramref name="rotation"/>.
        /// </summary>
        private static GameObject Place(string name, Transform parent, Vector3 anchor, Vector3 referenceCad, Quaternion rotation,
            bool isBase, LabPalette p)
        {
            GameObject asset = Load(name);
            var part = (GameObject)PrefabUtility.InstantiatePrefab(asset);
            part.name = "Cad" + name;
            Vector3 position = asset.transform.localPosition;
            Quaternion orientation = asset.transform.localRotation;
            if (isBase)
            {
                position = BaseFix * position;
                orientation = BaseFix * orientation;
            }

            part.transform.SetParent(parent, true);
            part.transform.position = anchor + rotation * ((position - referenceCad) / UnitsPerMetre);
            part.transform.rotation = rotation * orientation;
            part.transform.localScale = asset.transform.localScale / UnitsPerMetre;

            foreach (Renderer r in part.GetComponentsInChildren<Renderer>(true))
            {
                r.sharedMaterials = r.sharedMaterials.Select(m => Palette(m, p)).ToArray();
                LabParts.Configure(r, PartKind.Dynamic);
            }

            return part;
        }

        /// <summary>
        /// The gripper's jaw links. Gripper.fbx keeps the URDF's joints: each link's origin is its hinge,
        /// and its local X the hinge axis. The inner links (<c>l_link</c>, <c>r_link</c>) turn on the
        /// gear centres, 13.9 mm either side of the middle, and each finger (<c>l_out_link</c>) hangs on
        /// the far end of its link. As exported the jaws are open, the inner links 40.6° out.
        /// </summary>
        public const string JawLeft = "l_link", JawLeftOuter = "l_out_link", JawRight = "r_link", JawRightOuter = "r_out_link";

        /// <summary>Moves a part so its own origin (its joint) sits exactly on the pivot it hangs from.</summary>
        private static void Snap(GameObject part, Transform pivot) => part.transform.position = pivot.position;

        private static Material Palette(Material imported, LabPalette p)
        {
            string n = imported != null ? imported.name : "";
            if (n.StartsWith("Green")) return p.RobotGreen;
            if (n.StartsWith("Black")) return p.AnodizedBlack;
            return p.LensRing; // the depth camera's housing
        }

        private static GameObject Load(string name) =>
            AssetDatabase.LoadAssetAtPath<GameObject>($"{Folder}/{name}.fbx")
            ?? throw new System.InvalidOperationException($"{Folder}/{name}.fbx did not import");

        private static Transform Visual(Transform pivot)
        {
            var go = new GameObject(JetRoverTwin.VisualName);
            go.transform.SetParent(pivot, false);
            return go.transform;
        }
    }
}
