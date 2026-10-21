using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using Teleop.Bridge;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.XR;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using NVector3 = System.Numerics.Vector3;
using Object = UnityEngine.Object;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR.Editor
{
    /// <summary>
    /// Builds the arm's collision model from the robot's CAD (docs/adr/0020-collision-aware-ik.md)
    /// and writes it to <c>core/Teleop.RobotArm/Collision/JetRoverCollisionModel.g.cs</c>.
    /// <c>just robot-collision-model</c>. Reads the lab scene; saves nothing in it.
    ///
    /// <para>Steps:</para>
    /// <list type="number">
    /// <item><b>Sample.</b> The commanded arm goes to its rest pose with the gripper open. Every CAD
    /// part's surface is sampled every <see cref="Spacing"/>, in its own link's frame, converted to
    /// Core's axes. The chassis is sampled in the arm base frame.</item>
    /// <item><b>Fit.</b> Each link gets 1 to 6 oriented boxes (k-means clusters; around each, the
    /// smaller of a box on its principal axes and one square to the link) and each body part 1 to
    /// 20 axis-aligned boxes, the count chosen by covered volume plus a small cost per
    /// shape (smaller for boxes, which are cheap to test). Every shape is grown by <see cref="Pad"/>, the most a point on a triangle can be from
    /// its nearest sample.</item>
    /// <item><b>Skip list.</b> Over 3,000 random poses in the servo ranges, a pair that overlaps in at
    /// least 98% of them is joint-local (it touches at every angle) and is skipped. The proximal and
    /// distal links get at least 3 bodies and the wrist 2, so that piece is never a whole link.
    /// Each link's joint-local piece (its body nearest the joint it turns on) is skipped against
    /// its parent link, and the proximal link's against the body too: those sit millimetres apart by
    /// design, which the margin cannot resolve. Every wrist-against-gripper pair is skipped too: the
    /// gripper turns on its own roll servo inside the wrist bracket, built to turn freely. Every other pair, adjacent links included,
    /// is checked.</item>
    /// <item><b>Validate, after rounding to the generated file's precision:</b>
    ///   <list type="bullet">
    ///   <item>every surface sample lies inside its own part's shapes, so any real contact between two
    ///   parts lies inside both parts' shapes and is caught for every checked pair;</item>
    ///   <item>Core's link frames (<see cref="ArmCollision.ComputeFrames"/>) put the samples where
    ///   Unity's rig does, within 1 mm, at 200 random poses: the sign conventions agree;</item>
    ///   <item>the home pose is valid.</item>
    ///   </list>
    /// Any failure exits non-zero and writes nothing.</item>
    /// <item><b>Write</b> the model, stamped with the FBX hashes, a report, and preview images (CAD and
    /// shapes from the same views), to <c>%TEMP%\teleop-collision\</c>.</item>
    /// </list>
    /// </summary>
    public static class RobotCollisionExtractor
    {
        private const float Spacing = 0.0025f;
        private const float Pad = 0.0025f;
        private const float Margin = 0.01f;
        private const float BodyCost = 1e-5f;    // m^3 (10 cm^3) per extra body
        private const float BoxCost = 2e-6f;     // m^3 (2 cm^3) per extra box: boxes are cheap to test
        private const int ClusterSample = 60000; // clustering runs on at most this many points
        private const string OutputPath = "../../core/Teleop.RobotArm/Collision/JetRoverCollisionModel.g.cs";
        private static readonly float Deg = Mathf.Deg2Rad;

        /// <summary>An oriented box in a link's frame, Core axes: centre, three unit axes (right-handed), half extents.</summary>
        private sealed class Obb
        {
            public string Name;
            public ArmLink Link;
            public Vector3 Center;
            public Vector3[] Axes;
            public Vector3 Half;
        }

        private sealed class PartSamples
        {
            public string Name;
            public ArmLink Link;
            public bool IsStatic;
            public readonly List<Vector3> Points = new List<Vector3>(); // Core axes
        }

        public static void Run()
        {
            try
            {
                string report = Extract();
                Debug.Log("[collision] " + report);
                EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError("[collision] FAILED: " + e);
                EditorApplication.Exit(1);
            }
        }

        private static string Extract()
        {
            EditorSceneManager.OpenScene(XrLabSceneBuilder.ScenePath, OpenSceneMode.Single);
            JetRoverArmRig rig = XrControlSetup.CommandedRig() ?? throw new InvalidOperationException("no commanded arm rig in the scene");
            var rigSo = new SerializedObject(rig);
            Transform Pivot(string field) => rigSo.FindProperty(field).objectReferenceValue as Transform
                ?? throw new InvalidOperationException($"the rig's {field} is not set");
            Transform baseAnchor = Pivot("baseAnchor"), yaw = Pivot("baseYawPivot"), lower = Pivot("lowerPitchPivot"),
                middle = Pivot("middlePitchPivot"), upper = Pivot("upperPitchPivot");
            TwinGripper gripper = rig.GetComponentsInChildren<TwinGripper>(true).FirstOrDefault()
                ?? Object.FindObjectsByType<TwinGripper>(FindObjectsInactive.Include).FirstOrDefault(g => g.transform.IsChildOf(rig.transform) || g.gameObject == rig.gameObject)
                ?? throw new InvalidOperationException("no TwinGripper on the commanded arm");
            Transform roll = upper.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == JetRoverTwin.RollName)
                ?? throw new InvalidOperationException("no TwinRoll group on the upper link");
            Transform chassis = Object.FindObjectsByType<Transform>(FindObjectsInactive.Include).FirstOrDefault(t => t.name == "JetRoverChassis")
                ?? throw new InvalidOperationException("no JetRoverChassis in the scene: rebuild the lab with the CAD (just unity-lab-xr)");

            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            rig.ApplyAngles(0f, 0f, 0f, 0f, false);
            gripper.Show(0f, 0f);

            // 1. Sample.
            var parts = new List<PartSamples>();
            SampleLink(parts, Visual(yaw), ArmLink.Turntable, yaw, null);
            SampleLink(parts, Visual(lower), ArmLink.Proximal, lower, null);
            SampleLink(parts, Visual(middle), ArmLink.Distal, middle, null);
            SampleLink(parts, Visual(upper), ArmLink.Wrist, upper, roll);
            SampleLink(parts, roll, ArmLink.Gripper, roll, null);
            foreach (Transform part in chassis.Cast<Transform>())
            {
                var samples = new PartSamples { Name = Clean(part.name), IsStatic = true };
                foreach (MeshFilter mf in part.GetComponentsInChildren<MeshFilter>(true)) Sample(mf, baseAnchor, samples.Points);
                if (samples.Points.Count > 0) parts.Add(samples);
            }

            // 2. Fit.
            var bodies = new List<Obb>();
            var boxes = new List<(string name, Vector3 min, Vector3 max)>();
            var fitLog = new StringBuilder();
            foreach (PartSamples part in parts)
            {
                if (part.IsStatic)
                {
                    List<List<Vector3>> clusters = BestClusters(part.Points, 1, 20, BoxVolume, BoxCost);
                    for (int i = 0; i < clusters.Count; i++)
                    {
                        (Vector3 min, Vector3 max) = Box(clusters[i]);
                        boxes.Add(($"{part.Name}{(clusters.Count > 1 ? "." + i : "")}", min - Vector3.one * Pad, max + Vector3.one * Pad));
                    }

                    fitLog.AppendLine($"  {part.Name}: {part.Points.Count} samples, {clusters.Count} box(es)");
                }
                else
                {
                    // Links get at least a few bodies, so the piece at a joint (which touches its
                    // neighbour at every angle and is skipped) is never the whole link.
                    int minK = part.Link == ArmLink.Proximal || part.Link == ArmLink.Distal ? 3 : part.Link == ArmLink.Wrist ? 2 : 1;
                    List<List<Vector3>> clusters = BestClusters(part.Points, minK, 6, ObbVolume, BodyCost);
                    for (int i = 0; i < clusters.Count; i++)
                    {
                        Obb obb = FitObb(clusters[i]);
                        obb.Name = $"{part.Name}{(clusters.Count > 1 ? "." + i : "")}";
                        obb.Link = part.Link;
                        obb.Half += Vector3.one * Pad;
                        bodies.Add(obb);
                    }

                    fitLog.AppendLine($"  {part.Name} ({part.Link}): {part.Points.Count} samples, {clusters.Count} body(ies)");
                }
            }

            if (bodies.Count > ArmCollisionModel.MaxBodies)
            {
                throw new InvalidOperationException($"{bodies.Count} bodies is over the model's limit of {ArmCollisionModel.MaxBodies}");
            }

            // Round outward to the file's precision, so the generated model covers at least what was
            // fitted: centres to 0.1 mm, half extents up by 0.2 mm, the axes through a quaternion
            // written to 6 decimals and read back.
            foreach (Obb obb in bodies)
            {
                obb.Center = Round(obb.Center);
                System.Numerics.Quaternion q = RoundQuaternion(ToQuaternion(obb.Axes));
                obb.Axes = AxesOf(q);
                obb.Half = new Vector3(Up(obb.Half.x) + 0.0002f, Up(obb.Half.y) + 0.0002f, Up(obb.Half.z) + 0.0002f);
            }

            for (int i = 0; i < boxes.Count; i++)
            {
                var b = boxes[i];
                boxes[i] = (b.name, Floor(b.min), Ceil(b.max));
            }

            float floorZ = Down(parts.Where(p => p.IsStatic).SelectMany(p => p.Points).Min(p => p.z));
            float servo = 120f * Deg;
            AngleRange Range(JointRole role)
            {
                int index = profile.TryGetJointIndex(role);
                float offset = index >= 0 ? profile.Joints[index].ZeroOffsetRadians : 0f;
                return new AngleRange(Down(-servo - offset), Up(servo - offset));
            }

            AngleRange yawRange = Range(JointRole.BaseYaw), rollRange = Range(JointRole.GripperRotate);
            // Adjacent links' contact is each joint's range, swept against the CAD's own surfaces.
            PartSamples Of(ArmLink link) => parts.First(p => !p.IsStatic && p.Link == link);
            var sweepLog = new StringBuilder("joint ranges, swept against the surfaces (servo range where nothing touches):");
            AngleRange proximalRange = Sweep("proximal", Of(ArmLink.Turntable).Points, Of(ArmLink.Proximal).Points,
                new Vector3(0f, 0f, profile.BaseHeight), Range(JointRole.Proximal), sweepLog);
            AngleRange distalRange = Sweep("distal", Of(ArmLink.Proximal).Points, Of(ArmLink.Distal).Points,
                new Vector3(profile.ProximalLinkLength, 0f, 0f), Range(JointRole.Distal), sweepLog);
            // The wrist carries the gripper, at any roll: sweep with it at five.
            var wristAndGripper = new List<Vector3>(Of(ArmLink.Wrist).Points);
            foreach (float rollDeg in new[] { -90f, -45f, 0f, 45f, 90f })
            {
                float c = Mathf.Cos(rollDeg * Deg), sn = Mathf.Sin(rollDeg * Deg);
                List<Vector3> g = Of(ArmLink.Gripper).Points;
                for (int i = 0; i < g.Count; i += 4) wristAndGripper.Add(new Vector3(g[i].x, g[i].y * c - g[i].z * sn, g[i].y * sn + g[i].z * c));
            }

            AngleRange wristRange = Sweep("wrist", Of(ArmLink.Distal).Points, wristAndGripper,
                new Vector3(profile.DistalLinkLength, 0f, 0f), Range(JointRole.Wrist), sweepLog);

            // Where the robot was measured going further than the CAD allows, the measurement wins.
            proximalRange = Measured("proximal", proximalRange, Range(JointRole.Proximal), sweepLog);
            distalRange = Measured("distal", distalRange, Range(JointRole.Distal), sweepLog);
            wristRange = Measured("wrist", wristRange, Range(JointRole.Wrist), sweepLog);

            // 3. Skip list: pairs overlapping in nearly every pose.
            var draft = Build(bodies, boxes, floorZ, yawRange, proximalRange, distalRange, wristRange, rollRange, default,
                Array.Empty<(int, int)>(), Array.Empty<(int, int)>(), "draft");
            (List<(int, int)> skipPairs, List<(int, int)> skipBoxes, string skipLog) = SkipLists(profile.WithCollision(draft), draft);

            // 4. Validate the frames and the coverage first: they do not depend on the home pose.
            ArmCollisionModel unhomed = Build(bodies, boxes, floorZ, yawRange, proximalRange, distalRange, wristRange, rollRange, default,
                skipPairs.ToArray(), skipBoxes.ToArray(), "unhomed");
            string frames = ValidateFrames(profile.WithCollision(unhomed), rig, gripper, baseAnchor, yaw, lower, middle, upper, roll, parts);
            rig.ApplyAngles(0f, 0f, 0f, 0f, false);
            gripper.Show(0f, 0f);
            string coverage = ValidateCoverage(parts, unhomed);

            // Home: the first valid of a few natural rest poses.
            var homes = new[]
            {
                new ArmPose(0f, 60f * Deg, -90f * Deg, 30f * Deg, 0f),
                new ArmPose(0f, 75f * Deg, -110f * Deg, 35f * Deg, 0f),
                new ArmPose(0f, 45f * Deg, -60f * Deg, 15f * Deg, 0f),
                new ArmPose(0f, 90f * Deg, -90f * Deg, 0f, 0f),
                new ArmPose(0f, 30f * Deg, -30f * Deg, 0f, 0f),
                new ArmPose(0f, 0f, 0f, 0f, 0f),
            };
            ArmCollisionModel model = null;
            ArmPose home = default;
            var homeLog = new StringBuilder();
            foreach (ArmPose candidate in homes)
            {
                ArmCollisionModel m = Build(bodies, boxes, floorZ, yawRange, proximalRange, distalRange, wristRange, rollRange, candidate,
                    skipPairs.ToArray(), skipBoxes.ToArray(), "candidate");
                CollisionHit hit = ArmCollision.Check(profile.WithCollision(m), m, candidate);
                homeLog.AppendLine($"  home prox {candidate.Proximal / Deg:0} distal {candidate.Distal / Deg:0} wrist {candidate.Wrist / Deg:0}: {hit.Describe(m)}");
                if (!hit.IsHit)
                {
                    model = m;
                    home = candidate;
                    break;
                }
            }

            if (model == null)
            {
                throw new InvalidOperationException("none of the home candidates is valid: the fit is too fat, or the frames are wrong\n" +
                    homeLog + fitLog + sweepLog + "\n" + skipLog + "\n" + frames + "\n" + BodyTable(bodies) + BoxTable(boxes));
            }

            // 5. Write.
            string source = SourceStamp();
            File.WriteAllText(OutputPath, Generate(bodies, boxes, floorZ, yawRange, proximalRange, distalRange, wristRange, rollRange,
                home, skipPairs, skipBoxes, source));
            string outDir = Path.Combine(Path.GetTempPath(), "teleop-collision");
            Directory.CreateDirectory(outDir);
            RenderPreviews(outDir, bodies, boxes, baseAnchor, yaw, lower, middle, upper, roll);
            var report = new StringBuilder();
            report.AppendLine($"wrote {Path.GetFullPath(OutputPath)}: {bodies.Count} bodies, {boxes.Count} boxes, floor {floorZ:0.0000} m, margin {Margin} m");
            report.Append(fitLog);
            report.AppendLine(sweepLog.ToString());
            report.AppendLine(skipLog);
            report.AppendLine(coverage);
            report.AppendLine(frames);
            report.Append(homeLog);
            report.Append(BodyTable(bodies)).Append(BoxTable(boxes));
            report.AppendLine($"home: yaw {home.BaseYaw / Deg:0} prox {home.Proximal / Deg:0} distal {home.Distal / Deg:0} wrist {home.Wrist / Deg:0} deg");
            report.AppendLine($"previews in {outDir}");
            report.AppendLine(source);
            File.WriteAllText(Path.Combine(outDir, "report.txt"), report.ToString());
            return report.ToString();
        }

        // ---- sampling --------------------------------------------------------------------------

        private static Transform Visual(Transform pivot) =>
            pivot.Cast<Transform>().FirstOrDefault(t => t.name == JetRoverTwin.VisualName)
            ?? throw new InvalidOperationException($"{pivot.name} has no {JetRoverTwin.VisualName}");

        private static void SampleLink(List<PartSamples> parts, Transform root, ArmLink link, Transform frame, Transform exclude)
        {
            var samples = new PartSamples { Name = link.ToString().ToLowerInvariant(), Link = link };
            foreach (MeshFilter mf in root.GetComponentsInChildren<MeshFilter>(true))
            {
                if (exclude != null && mf.transform.IsChildOf(exclude)) continue;
                if (mf.name.Contains("ReachLed") || mf.GetComponent<Renderer>() is Renderer r && !r.enabled) continue;
                Sample(mf, frame, samples.Points);
            }

            if (samples.Points.Count == 0)
            {
                throw new InvalidOperationException($"no CAD geometry found for {link}");
            }

            parts.Add(samples);
        }

        /// <summary>Samples a mesh's surface every <see cref="Spacing"/> into <paramref name="frame"/>'s local space, in Core's axes.</summary>
        private static void Sample(MeshFilter mf, Transform frame, List<Vector3> output)
        {
            Mesh mesh = mf.sharedMesh;
            if (mesh == null) return;
            Vector3[] v = mesh.vertices;
            int[] tri = mesh.triangles;
            Matrix4x4 toFrame = frame.worldToLocalMatrix * mf.transform.localToWorldMatrix;
            var local = new Vector3[v.Length];
            for (int i = 0; i < v.Length; i++)
            {
                local[i] = toFrame.MultiplyPoint3x4(v[i]);
                output.Add(ToCoreAxes(local[i]));
            }

            for (int t = 0; t < tri.Length; t += 3)
            {
                Vector3 a = local[tri[t]], b = local[tri[t + 1]], c = local[tri[t + 2]];
                float longest = Mathf.Max((b - a).magnitude, Mathf.Max((c - b).magnitude, (a - c).magnitude));
                int n = Mathf.CeilToInt(longest / Spacing);
                if (n <= 1) continue;
                // A barycentric grid: every point of the triangle is within one grid cell of a sample.
                for (int i = 0; i <= n; i++)
                {
                    for (int j = 0; j <= n - i; j++)
                    {
                        float u = i / (float)n, w = j / (float)n;
                        output.Add(ToCoreAxes(a + (b - a) * u + (c - a) * w));
                    }
                }
            }
        }

        /// <summary>Unity local (x right, y up, z forward) to Core (x forward, y left, z up): what CoordConversion does.</summary>
        private static Vector3 ToCoreAxes(Vector3 v) => new Vector3(v.z, -v.x, v.y);

        // ---- fitting ---------------------------------------------------------------------------

        /// <summary>
        /// The clustering with the smallest covered volume plus <paramref name="cost"/> per shape. The
        /// k-means runs on an even subsample of at most <see cref="ClusterSample"/> points; every
        /// point is then assigned to its nearest centre, so the shapes cover all of them.
        /// </summary>
        private static List<List<Vector3>> BestClusters(List<Vector3> points, int minK, int maxK, Func<List<Vector3>, float> volume, float cost)
        {
            int stride = Math.Max(1, points.Count / ClusterSample);
            var sample = new List<Vector3>(points.Count / stride + 1);
            for (int i = 0; i < points.Count; i += stride) sample.Add(points[i]);

            List<Vector3> bestCentres = null;
            float bestScore = float.MaxValue;
            for (int k = minK; k <= maxK; k++)
            {
                List<Vector3> centres = KMeans(sample, k);
                List<List<Vector3>> clusters = Assign(sample, centres);
                float score = clusters.Sum(volume) + cost * clusters.Count;
                if (score < bestScore)
                {
                    bestScore = score;
                    bestCentres = centres;
                }
            }

            return Assign(points, bestCentres);
        }

        private static List<List<Vector3>> Assign(List<Vector3> points, List<Vector3> centres)
        {
            var clusters = centres.Select(_ => new List<Vector3>()).ToList();
            foreach (Vector3 p in points)
            {
                int best = 0;
                float bestD = float.MaxValue;
                for (int c = 0; c < centres.Count; c++)
                {
                    float d = (p - centres[c]).sqrMagnitude;
                    if (d < bestD) { bestD = d; best = c; }
                }

                clusters[best].Add(p);
            }

            return clusters.Where(c => c.Count > 0).ToList();
        }

        /// <summary>Deterministic k-means: farthest-point seeds from the point nearest the centroid, 30 Lloyd rounds. Returns the centres.</summary>
        private static List<Vector3> KMeans(List<Vector3> points, int k)
        {
            Vector3 centroid = Average(points);
            var centres = new List<Vector3> { points.OrderBy(p => (p - centroid).sqrMagnitude).First() };
            var nearest = points.Select(p => (p - centres[0]).sqrMagnitude).ToArray();
            while (centres.Count < k)
            {
                int far = 0;
                for (int i = 1; i < points.Count; i++) if (nearest[i] > nearest[far]) far = i;
                centres.Add(points[far]);
                for (int i = 0; i < points.Count; i++) nearest[i] = Mathf.Min(nearest[i], (points[i] - points[far]).sqrMagnitude);
            }

            var assign = new int[points.Count];
            for (int round = 0; round < 30; round++)
            {
                for (int i = 0; i < points.Count; i++)
                {
                    int bestC = 0;
                    float bestD = float.MaxValue;
                    for (int c = 0; c < k; c++)
                    {
                        float d = (points[i] - centres[c]).sqrMagnitude;
                        if (d < bestD) { bestD = d; bestC = c; }
                    }

                    assign[i] = bestC;
                }

                for (int c = 0; c < k; c++)
                {
                    Vector3 sum = Vector3.zero;
                    int count = 0;
                    for (int i = 0; i < points.Count; i++) if (assign[i] == c) { sum += points[i]; count++; }
                    if (count > 0) centres[c] = sum / count;
                }
            }

            return centres;
        }

        /// <summary>
        /// The smaller of two boxes around the points: one along their principal axes, one along the
        /// link's own axes (the brackets are mostly square to their link). Every point is inside by
        /// construction: the extents are the points' extremes along each axis.
        /// </summary>
        private static Obb FitObb(List<Vector3> points)
        {
            Obb pca = ObbAlong(points, PrincipalAxes(points, Average(points)));
            Obb aligned = ObbAlong(points, new[] { Vector3.right, Vector3.up, Vector3.forward });
            return Volume(pca) <= Volume(aligned) ? pca : aligned;
        }

        private static Obb ObbAlong(List<Vector3> points, Vector3[] axes)
        {
            Vector3 lo = Vector3.one * float.MaxValue, hi = Vector3.one * float.MinValue;
            foreach (Vector3 p in points)
            {
                var t = new Vector3(Vector3.Dot(p, axes[0]), Vector3.Dot(p, axes[1]), Vector3.Dot(p, axes[2]));
                lo = Vector3.Min(lo, t);
                hi = Vector3.Max(hi, t);
            }

            Vector3 mid = 0.5f * (lo + hi);
            return new Obb
            {
                Center = axes[0] * mid.x + axes[1] * mid.y + axes[2] * mid.z,
                Axes = axes,
                Half = 0.5f * (hi - lo),
            };
        }

        private static float Volume(Obb obb) => 8f * obb.Half.x * obb.Half.y * obb.Half.z;

        private static float ObbVolume(List<Vector3> points)
        {
            Obb obb = FitObb(points);
            Vector3 h = obb.Half + Vector3.one * Pad;
            return 8f * h.x * h.y * h.z;
        }

        /// <summary>The covariance's eigenvectors (Jacobi rotations), as a right-handed set.</summary>
        private static Vector3[] PrincipalAxes(List<Vector3> points, Vector3 centre)
        {
            double[,] a = new double[3, 3];
            foreach (Vector3 p in points)
            {
                Vector3 d = p - centre;
                a[0, 0] += d.x * d.x; a[0, 1] += d.x * d.y; a[0, 2] += d.x * d.z;
                a[1, 1] += d.y * d.y; a[1, 2] += d.y * d.z; a[2, 2] += d.z * d.z;
            }

            a[1, 0] = a[0, 1]; a[2, 0] = a[0, 2]; a[2, 1] = a[1, 2];
            double[,] v = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
            for (int sweep = 0; sweep < 50; sweep++)
            {
                for (int p = 0; p < 2; p++)
                {
                    for (int q = p + 1; q < 3; q++)
                    {
                        if (Math.Abs(a[p, q]) < 1e-18) continue;
                        double theta = 0.5 * Math.Atan2(2 * a[p, q], a[q, q] - a[p, p]);
                        double c = Math.Cos(theta), s = Math.Sin(theta);
                        for (int k = 0; k < 3; k++)
                        {
                            double akp = a[k, p], akq = a[k, q];
                            a[k, p] = c * akp - s * akq;
                            a[k, q] = s * akp + c * akq;
                        }

                        for (int k = 0; k < 3; k++)
                        {
                            double apk = a[p, k], aqk = a[q, k];
                            a[p, k] = c * apk - s * aqk;
                            a[q, k] = s * apk + c * aqk;
                        }

                        for (int k = 0; k < 3; k++)
                        {
                            double vkp = v[k, p], vkq = v[k, q];
                            v[k, p] = c * vkp - s * vkq;
                            v[k, q] = s * vkp + c * vkq;
                        }
                    }
                }
            }

            Vector3 e0 = new Vector3((float)v[0, 0], (float)v[1, 0], (float)v[2, 0]).normalized;
            Vector3 e1 = new Vector3((float)v[0, 1], (float)v[1, 1], (float)v[2, 1]);
            e1 = (e1 - Vector3.Dot(e1, e0) * e0).normalized;
            Vector3 e2 = Vector3.Cross(e0, e1).normalized;
            return new[] { e0, e1, e2 };
        }

        /// <summary>The rotation whose X, Y, Z axes are <paramref name="axes"/> (a right-handed orthonormal set), as Core's quaternion.</summary>
        private static System.Numerics.Quaternion ToQuaternion(Vector3[] axes)
        {
            var m = new System.Numerics.Matrix4x4(
                axes[0].x, axes[0].y, axes[0].z, 0f,
                axes[1].x, axes[1].y, axes[1].z, 0f,
                axes[2].x, axes[2].y, axes[2].z, 0f,
                0f, 0f, 0f, 1f);
            return System.Numerics.Quaternion.Normalize(System.Numerics.Quaternion.CreateFromRotationMatrix(m));
        }

        private static System.Numerics.Quaternion RoundQuaternion(System.Numerics.Quaternion q) =>
            System.Numerics.Quaternion.Normalize(new System.Numerics.Quaternion(R6(q.X), R6(q.Y), R6(q.Z), R6(q.W)));

        private static float R6(float f) => (float)Math.Round(f, 6);

        private static Vector3[] AxesOf(System.Numerics.Quaternion q) => new[]
        {
            U(System.Numerics.Vector3.Transform(System.Numerics.Vector3.UnitX, q)),
            U(System.Numerics.Vector3.Transform(System.Numerics.Vector3.UnitY, q)),
            U(System.Numerics.Vector3.Transform(System.Numerics.Vector3.UnitZ, q)),
        };

        private static Vector3 U(System.Numerics.Vector3 v) => new Vector3(v.X, v.Y, v.Z);

        private static (Vector3 min, Vector3 max) Box(List<Vector3> points)
        {
            Vector3 min = points[0], max = points[0];
            foreach (Vector3 p in points)
            {
                min = Vector3.Min(min, p);
                max = Vector3.Max(max, p);
            }

            return (min, max);
        }

        private static float BoxVolume(List<Vector3> points)
        {
            (Vector3 min, Vector3 max) = Box(points);
            Vector3 s = max - min + Vector3.one * 2f * Pad;
            return s.x * s.y * s.z;
        }

        private static Vector3 Average(List<Vector3> points)
        {
            Vector3 sum = Vector3.zero;
            foreach (Vector3 p in points) sum += p;
            return sum / points.Count;
        }

        // ---- joint ranges ----------------------------------------------------------------------

        /// <summary>
        /// Physical stops measured on the robot by hand, in the IK's angles (`just
        /// robot-measure-joint-limits`: torque off, the joint moved until it touched). Each replaces
        /// the CAD sweep's end on that side, less <see cref="MeasuredMarginDegrees"/>, within the
        /// servo range. Only real measurements go here, with when they were taken: the CAD's answer
        /// stands wherever nobody has measured.
        /// </summary>
        private static readonly (string joint, bool lowerEnd, float stopDegrees, string measured)[] MeasuredStops =
        {
            // The wrist folded down against the forearm: -115.2 and -114.7 deg in two runs (the nearer
            // is used), where the CAD's sweep stopped at -106. Folded up and back: +119.0, where the CAD
            // stopped at +118. Measured 2026-10-21. The shoulder stopped near -14 deg in both runs, but
            // that was the gripper meeting the deck (a contact the boxes model), not the lower arm and
            // the turntable, so the CAD's -30 stands until that is measured with the gripper clear.
            ("wrist", true, -114.7f, "2026-10-21"),
            ("wrist", false, 119.0f, "2026-10-21"),
        };

        private const float MeasuredMarginDegrees = 2f;

        private static AngleRange Measured(string joint, AngleRange swept, AngleRange servo, StringBuilder log)
        {
            float lo = swept.Min, hi = swept.Max;
            foreach (var stop in MeasuredStops.Where(m => m.joint == joint))
            {
                float limit = (stop.stopDegrees + (stop.lowerEnd ? MeasuredMarginDegrees : -MeasuredMarginDegrees)) * Deg;
                if (stop.lowerEnd) lo = Mathf.Max(servo.Min, limit);
                else hi = Mathf.Min(servo.Max, limit);
                log.Append($"\n  {joint}: {(stop.lowerEnd ? "lower" : "upper")} end {(stop.lowerEnd ? lo : hi) / Deg:0.0} deg from the robot " +
                    $"(measured stop {stop.stopDegrees:0.0} deg on {stop.measured}, less {MeasuredMarginDegrees:0} deg), replacing the CAD's " +
                    $"{(stop.lowerEnd ? swept.Min : swept.Max) / Deg:0.0}");
            }

            return new AngleRange(lo, hi);
        }

        private const float ContactDistance = 0.0035f; // surfaces this close are touching (samples are 2.5 mm apart)
        private const float AxisExclusion = 0.015f;    // around the joint axis, the horn and bracket touch by design
        private const float RangeMargin = 2f;          // degrees backed off from first contact

        /// <summary>
        /// A joint's range: turns the child's surface samples about the joint (its pitch axis, Core Y,
        /// through <paramref name="jointOrigin"/> in the parent's frame), 1 degree at a time outward
        /// from rest, until they come within <see cref="ContactDistance"/> of the parent's, then backs
        /// off <see cref="RangeMargin"/>. Samples near the joint axis are left out on both sides: the
        /// servo horn and its bracket touch there by construction. Clamped to the servo's range.
        /// </summary>
        private static AngleRange Sweep(string joint, List<Vector3> parentPoints, List<Vector3> childPoints, Vector3 jointOrigin,
            AngleRange servo, StringBuilder log)
        {
            float exclusion = AxisExclusion;
            for (int attempt = 0; attempt < 4; attempt++, exclusion += 0.005f)
            {
                var grid = new Dictionary<long, List<Vector3>>();
                foreach (Vector3 p in parentPoints)
                {
                    Vector3 d = p - jointOrigin;
                    if (d.x * d.x + d.z * d.z < exclusion * exclusion) continue;
                    long key = Cell(p);
                    if (!grid.TryGetValue(key, out List<Vector3> list)) grid[key] = list = new List<Vector3>();
                    list.Add(p);
                }

                var child = new List<Vector3>();
                int stride = Math.Max(1, childPoints.Count / 40000);
                for (int i = 0; i < childPoints.Count; i += stride)
                {
                    Vector3 c = childPoints[i];
                    if (c.x * c.x + c.z * c.z >= exclusion * exclusion) child.Add(c);
                }

                if (Touches(grid, child, jointOrigin, 0f, out _, out _))
                {
                    continue; // touching at rest even outside the exclusion: widen it and retry
                }

                float hi = SweepOneWay(grid, child, jointOrigin, servo.Max, +1, out string hiWhy);
                float lo = SweepOneWay(grid, child, jointOrigin, servo.Min, -1, out string loWhy);
                log.Append($"\n  {joint}: [{lo / Deg:0}, {hi / Deg:0}] deg (servo [{servo.Min / Deg:0}, {servo.Max / Deg:0}]; " +
                    $"down {loWhy}, up {hiWhy}; axis exclusion {exclusion * 1000f:0} mm)");
                return new AngleRange(Down(lo), Up(hi) - 0.0002f);
            }

            throw new InvalidOperationException($"{joint}: the links touch at rest even outside a {exclusion * 1000f:0} mm exclusion round the joint axis");
        }

        private static float SweepOneWay(Dictionary<long, List<Vector3>> grid, List<Vector3> child, Vector3 origin, float limit, int direction, out string why)
        {
            int steps = Mathf.FloorToInt(Mathf.Abs(limit) / Deg);
            for (int deg = 1; deg <= steps; deg++)
            {
                if (Touches(grid, child, origin, direction * deg * Deg, out Vector3 at, out Vector3 against))
                {
                    why = $"contact at {direction * deg} deg: child sample {at.x * 1000f:0},{at.y * 1000f:0},{at.z * 1000f:0} mm (its own frame) " +
                        $"near parent sample {against.x * 1000f:0},{against.y * 1000f:0},{against.z * 1000f:0} mm";
                    return direction * Mathf.Max(0f, deg - 1 - RangeMargin) * Deg;
                }
            }

            why = "free to the servo limit";
            return limit;
        }

        private static bool Touches(Dictionary<long, List<Vector3>> grid, List<Vector3> child, Vector3 origin, float angle,
            out Vector3 at, out Vector3 against)
        {
            at = against = default;
            float c = Mathf.Cos(angle), s = Mathf.Sin(angle);
            const float d2 = ContactDistance * ContactDistance;
            foreach (Vector3 p in child)
            {
                // Pitch: raises +X toward +Z by the angle, about Y.
                var q = new Vector3(origin.x + p.x * c - p.z * s, origin.y + p.y, origin.z + p.x * s + p.z * c);
                int cx = Mathf.FloorToInt(q.x / CellSize), cy = Mathf.FloorToInt(q.y / CellSize), cz = Mathf.FloorToInt(q.z / CellSize);
                for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                for (int k = -1; k <= 1; k++)
                {
                    if (!grid.TryGetValue(Key(cx + i, cy + j, cz + k), out List<Vector3> list)) continue;
                    foreach (Vector3 r in list)
                    {
                        if ((r - q).sqrMagnitude < d2)
                        {
                            at = p;
                            against = r;
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        private const float CellSize = 0.004f;

        private static long Cell(Vector3 p) => Key(Mathf.FloorToInt(p.x / CellSize), Mathf.FloorToInt(p.y / CellSize), Mathf.FloorToInt(p.z / CellSize));

        private static long Key(int x, int y, int z) =>
            ((long)(x + (1 << 20)) << 42) | ((long)(y + (1 << 20)) << 21) | (long)(z + (1 << 20));

        // ---- the skip list ---------------------------------------------------------------------

        private static (List<(int, int)>, List<(int, int)>, string) SkipLists(in RobotArmProfile profile, ArmCollisionModel model)
        {
            const int poses = 3000;
            var random = new System.Random(20261019);
            int n = model.Bodies.Length, m = model.Boxes.Length;
            var pairHits = new int[n, n];
            var boxHits = new int[n, m];
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            var c = new NVector3[n];
            var ax = new NVector3[n];
            var ay = new NVector3[n];
            var az = new NVector3[n];
            var pad = new NVector3(model.Margin);
            for (int s = 0; s < poses; s++)
            {
                var pose = new ArmPose(Uniform(random, model.BaseYaw), Uniform(random, model.Proximal), Uniform(random, model.Distal),
                    Uniform(random, model.Wrist), Uniform(random, model.Roll));
                ArmCollision.ComputeFrames(profile, pose, frames);
                for (int i = 0; i < n; i++)
                {
                    LinkFrame frame = frames[(int)model.Bodies[i].Link];
                    System.Numerics.Quaternion q = frame.Rotation * model.Bodies[i].Rotation;
                    c[i] = frame.ToBase(model.Bodies[i].Center);
                    ax[i] = NVector3.Transform(NVector3.UnitX, q);
                    ay[i] = NVector3.Transform(NVector3.UnitY, q);
                    az[i] = NVector3.Transform(NVector3.UnitZ, q);
                }

                for (int i = 0; i < n; i++)
                {
                    NVector3 hi = model.Bodies[i].HalfExtents + pad;
                    for (int j = i + 1; j < n; j++)
                    {
                        if (model.Bodies[i].Link == model.Bodies[j].Link) continue;
                        if (ArmCollision.BoxesOverlap(c[i], ax[i], ay[i], az[i], hi, c[j], ax[j], ay[j], az[j], model.Bodies[j].HalfExtents)) pairHits[i, j]++;
                    }

                    if (model.Bodies[i].Link == ArmLink.Turntable) continue;
                    for (int k = 0; k < m; k++)
                    {
                        NVector3 bc = 0.5f * (model.Boxes[k].Min + model.Boxes[k].Max), bh = 0.5f * (model.Boxes[k].Max - model.Boxes[k].Min);
                        if (ArmCollision.BoxesOverlap(c[i], ax[i], ay[i], az[i], hi, bc, NVector3.UnitX, NVector3.UnitY, NVector3.UnitZ, bh)) boxHits[i, k]++;
                    }
                }
            }

            // The proximal link's piece at the shoulder sits millimetres from the housing it turns
            // in, by design, finer than the margin resolves: it is skipped against the body. The
            // proximal link's outer pieces are what reach the plate, and they stay checked.
            int shoulderPiece = -1;
            for (int i = 0; i < n; i++)
            {
                if (model.Bodies[i].Link == ArmLink.Proximal &&
                    (shoulderPiece < 0 || model.Bodies[i].Center.Length() < model.Bodies[shoulderPiece].Center.Length())) shoulderPiece = i;
            }

            var skipPairs = new List<(int, int)>();
            var skipBoxes = new List<(int, int)>();
            var log = new StringBuilder("skipped (overlap in >= 98% of poses), and pairs that overlap often but are checked (adjacent links are never box-tested):");
            if (shoulderPiece >= 0) log.Append($"\n  skip {model.Bodies[shoulderPiece].Name} / the body (the shoulder piece)");
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    if (model.IgnoresBodyPair(i, j)) continue;
                    float f = pairHits[i, j] / (float)poses;
                    if (f >= 0.98f) { skipPairs.Add((i, j)); log.Append($"\n  skip {model.Bodies[i].Name} / {model.Bodies[j].Name}"); }
                    else if (f >= 0.3f) log.Append($"\n  CHECKED BUT OFTEN TOUCHING ({f:P0}): {model.Bodies[i].Name} / {model.Bodies[j].Name}");
                }

                for (int k = 0; k < m; k++)
                {
                    if (i == shoulderPiece)
                    {
                        skipBoxes.Add((i, k));
                        continue;
                    }

                    float f = boxHits[i, k] / (float)poses;
                    if (f >= 0.98f) { skipBoxes.Add((i, k)); log.Append($"\n  skip {model.Bodies[i].Name} / {model.Boxes[k].Name}"); }
                    else if (f >= 0.3f) log.Append($"\n  CHECKED BUT OFTEN TOUCHING ({f:P0}): {model.Bodies[i].Name} / {model.Boxes[k].Name}");
                }
            }

            return (skipPairs, skipBoxes, log.ToString());
        }

        private static float Uniform(System.Random random, AngleRange range) => range.Min + (float)random.NextDouble() * (range.Max - range.Min);

        // ---- validation ------------------------------------------------------------------------

        private static string ValidateCoverage(List<PartSamples> parts, ArmCollisionModel model)
        {
            int checkedPoints = 0;
            foreach (PartSamples part in parts)
            {
                foreach (Vector3 p in part.Points)
                {
                    var q = new NVector3(p.x, p.y, p.z);
                    bool covered = part.IsStatic
                        ? model.Boxes.Any(b => ArmCollision.PointBoxDistanceSquared(q, b.Min, b.Max) <= 0f)
                        : model.Bodies.Any(b => b.Link == part.Link && InsideBody(q, b));
                    if (!covered)
                    {
                        throw new InvalidOperationException($"coverage: a surface sample of {part.Name} at {q} is outside its shapes");
                    }

                    checkedPoints++;
                }
            }

            return $"coverage: all {checkedPoints} surface samples lie inside their part's shapes";
        }

        private static bool InsideBody(NVector3 point, CollisionBody body)
        {
            NVector3 d = point - body.Center;
            NVector3 x = NVector3.Transform(NVector3.UnitX, body.Rotation), y = NVector3.Transform(NVector3.UnitY, body.Rotation),
                z = NVector3.Transform(NVector3.UnitZ, body.Rotation);
            return Math.Abs(NVector3.Dot(d, x)) <= body.HalfExtents.X && Math.Abs(NVector3.Dot(d, y)) <= body.HalfExtents.Y &&
                Math.Abs(NVector3.Dot(d, z)) <= body.HalfExtents.Z;
        }

        private static string ValidateFrames(in RobotArmProfile profile, JetRoverArmRig rig, TwinGripper gripper, Transform baseAnchor,
            Transform yaw, Transform lower, Transform middle, Transform upper, Transform roll, List<PartSamples> parts)
        {
            var random = new System.Random(5);
            var frameOf = new Dictionary<ArmLink, Transform>
            {
                { ArmLink.Turntable, yaw }, { ArmLink.Proximal, lower }, { ArmLink.Distal, middle }, { ArmLink.Wrist, upper }, { ArmLink.Gripper, roll },
            };
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            float worst = 0f;
            ArmCollisionModel model = profile.Collision;
            for (int s = 0; s < 200; s++)
            {
                var pose = new ArmPose(Uniform(random, model.BaseYaw), Uniform(random, model.Proximal), Uniform(random, model.Distal),
                    Uniform(random, model.Wrist), Uniform(random, model.Roll));
                rig.ApplyAngles(pose.BaseYaw, pose.Proximal, pose.Distal, pose.Wrist, false);
                gripper.Show(0f, pose.Roll);
                ArmCollision.ComputeFrames(profile, pose, frames);
                foreach (PartSamples part in parts.Where(p => !p.IsStatic))
                {
                    Transform unityFrame = frameOf[part.Link];
                    for (int i = 0; i < part.Points.Count; i += Math.Max(1, part.Points.Count / 50))
                    {
                        Vector3 c = part.Points[i];
                        // Each sample was taken in its link's own Unity frame (the gripper's in the roll
                        // group's), so the rig carries it to the world exactly as it carries the part.
                        Vector3 world = unityFrame.TransformPoint(new Vector3(-c.y, c.z, c.x));
                        Vector3 expected = ToCoreAxes(baseAnchor.InverseTransformPoint(world));
                        NVector3 core = frames[(int)part.Link].ToBase(new NVector3(c.x, c.y, c.z));
                        worst = Mathf.Max(worst, (new Vector3(core.X, core.Y, core.Z) - expected).magnitude);
                    }
                }
            }

            if (worst > 0.001f)
            {
                throw new InvalidOperationException($"frames: Core's link frames disagree with the Unity rig by {worst * 1000f:0.0} mm (a sign or axis convention differs)");
            }

            return $"frames: Core's link frames match the Unity rig within {worst * 1000f:0.000} mm over 200 random poses";
        }

        // ---- output ----------------------------------------------------------------------------

        private static ArmCollisionModel Build(List<Obb> bodies,
            List<(string name, Vector3 min, Vector3 max)> boxes, float floorZ, AngleRange yaw, AngleRange proximal, AngleRange distal,
            AngleRange wrist, AngleRange roll, ArmPose home, (int, int)[] skipPairs, (int, int)[] skipBoxes, string source) =>
            new ArmCollisionModel("jetrover",
                bodies.Select(b => new CollisionBody(b.Name, b.Link, N(b.Center), ToQuaternion(b.Axes), N(b.Half))).ToArray(),
                boxes.Select(b => new CollisionBox(b.name, N(b.min), N(b.max))).ToArray(),
                floorZ, Margin, yaw, proximal, distal, wrist, roll, home, skipPairs, skipBoxes, source);

        private static string Generate(List<Obb> bodies,
            List<(string name, Vector3 min, Vector3 max)> boxes, float floorZ, AngleRange yaw, AngleRange proximal, AngleRange distal,
            AngleRange wrist, AngleRange roll, ArmPose home, List<(int, int)> skipPairs, List<(int, int)> skipBoxes, string source)
        {
            var s = new StringBuilder();
            s.AppendLine("// <auto-generated>");
            s.AppendLine("// Generated by unity/TeleopXR/Assets/Teleop/Editor/Lab/RobotCollisionExtractor.cs from the robot's CAD");
            s.AppendLine("// (just robot-collision-model). Do not edit by hand: change the CAD or the generator and rerun it.");
            foreach (string line in source.Split('\n')) s.AppendLine("// " + line.TrimEnd());
            s.AppendLine("// </auto-generated>");
            s.AppendLine("using System.Numerics;");
            s.AppendLine();
            s.AppendLine("namespace Teleop.RobotArm.Collision");
            s.AppendLine("{");
            s.AppendLine("    /// <summary>The JetRover's collision model, from its CAD (docs/adr/0020-collision-aware-ik.md).</summary>");
            s.AppendLine("    public static class JetRoverCollisionModel");
            s.AppendLine("    {");
            s.AppendLine("        public static readonly ArmCollisionModel Instance = new ArmCollisionModel(");
            s.AppendLine("            name: \"jetrover\",");
            s.AppendLine("            bodies: new[]");
            s.AppendLine("            {");
            foreach (Obb b in bodies)
            {
                System.Numerics.Quaternion q = ToQuaternion(b.Axes);
                s.AppendLine($"                new CollisionBody(\"{b.Name}\", ArmLink.{b.Link}, {V(b.Center)}, " +
                    $"new Quaternion({Q(q.X)}, {Q(q.Y)}, {Q(q.Z)}, {Q(q.W)}), {V(b.Half)}),");
            }

            s.AppendLine("            },");
            s.AppendLine("            boxes: new[]");
            s.AppendLine("            {");
            foreach (var b in boxes)
            {
                s.AppendLine($"                new CollisionBox(\"{b.name}\", {V(b.min)}, {V(b.max)}),");
            }

            s.AppendLine("            },");
            s.AppendLine($"            floorZ: {F(floorZ)},");
            s.AppendLine($"            margin: {F(Margin)},");
            s.AppendLine($"            baseYaw: new AngleRange({F(yaw.Min)}, {F(yaw.Max)}),");
            s.AppendLine($"            proximal: new AngleRange({F(proximal.Min)}, {F(proximal.Max)}),");
            s.AppendLine($"            distal: new AngleRange({F(distal.Min)}, {F(distal.Max)}),");
            s.AppendLine($"            wrist: new AngleRange({F(wrist.Min)}, {F(wrist.Max)}),");
            s.AppendLine($"            roll: new AngleRange({F(roll.Min)}, {F(roll.Max)}),");
            s.AppendLine($"            home: new ArmPose({F(home.BaseYaw)}, {F(home.Proximal)}, {F(home.Distal)}, {F(home.Wrist)}, {F(home.Roll)}),");
            s.AppendLine($"            ignoredBodyPairs: new (int, int)[] {{ {string.Join(", ", skipPairs.Select(p => $"({p.Item1}, {p.Item2})"))} }},");
            s.AppendLine($"            ignoredBodyBoxes: new (int, int)[] {{ {string.Join(", ", skipBoxes.Select(p => $"({p.Item1}, {p.Item2})"))} }},");
            s.AppendLine($"            source: \"{source.Replace("\n", " | ").Replace("\"", "'")}\");");
            s.AppendLine("    }");
            s.AppendLine("}");
            return s.ToString();
        }

        private static string SourceStamp()
        {
            var s = new StringBuilder($"generated {DateTime.UtcNow:yyyy-MM-dd} from:");
            using (SHA256 sha = SHA256.Create())
            {
                foreach (string path in Directory.GetFiles(JetRoverCad.Folder, "*.fbx").OrderBy(p => p))
                {
                    string hash = BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(path))).Replace("-", "").Substring(0, 16).ToLowerInvariant();
                    s.Append($"\n{Path.GetFileName(path)} sha256:{hash}");
                }
            }

            return s.ToString();
        }

        private static string BodyTable(List<Obb> bodies) =>
            "bodies (link frame, Core axes, m):\n" + string.Concat(bodies.Select((b, i) =>
                $"  [{i}] {b.Name} {b.Link}: centre {b.Center.x:0.000},{b.Center.y:0.000},{b.Center.z:0.000} " +
                $"half {b.Half.x:0.000},{b.Half.y:0.000},{b.Half.z:0.000} x-axis {b.Axes[0].x:0.00},{b.Axes[0].y:0.00},{b.Axes[0].z:0.00}\n"));

        private static string BoxTable(List<(string name, Vector3 min, Vector3 max)> boxes) =>
            "boxes (arm base frame, Core axes, m):\n" + string.Concat(boxes.Select((b, i) =>
                $"  [{i}] {b.name}: min {b.min.x:0.000},{b.min.y:0.000},{b.min.z:0.000} max {b.max.x:0.000},{b.max.y:0.000},{b.max.z:0.000}\n"));

        private static string V(Vector3 v) => $"new Vector3({F(v.x)}, {F(v.y)}, {F(v.z)})";
        private static string Q(float f) => f.ToString("0.000000", CultureInfo.InvariantCulture) + "f";
        private static string F(float f) => f.ToString("0.0000", CultureInfo.InvariantCulture) + "f";
        private static NVector3 N(Vector3 v) => new NVector3(v.x, v.y, v.z);
        private static float Up(float f) => Mathf.Ceil(f * 10000f) / 10000f;
        private static float Down(float f) => Mathf.Floor(f * 10000f) / 10000f;
        private static Vector3 Round(Vector3 v) => new Vector3(Mathf.Round(v.x * 10000f), Mathf.Round(v.y * 10000f), Mathf.Round(v.z * 10000f)) / 10000f;
        private static Vector3 Floor(Vector3 v) => new Vector3(Down(v.x), Down(v.y), Down(v.z));
        private static Vector3 Ceil(Vector3 v) => new Vector3(Up(v.x), Up(v.y), Up(v.z));
        private static string Clean(string name) => name.Replace("Cad", "").Replace("(Clone)", "").Trim();

        // ---- previews --------------------------------------------------------------------------

        private static void RenderPreviews(string outDir, List<Obb> bodies,
            List<(string name, Vector3 min, Vector3 max)> boxes, Transform baseAnchor, Transform yaw, Transform lower, Transform middle,
            Transform upper, Transform roll)
        {
            var frameOf = new Dictionary<ArmLink, Transform>
            {
                { ArmLink.Turntable, yaw }, { ArmLink.Proximal, lower }, { ArmLink.Distal, middle }, { ArmLink.Wrist, upper }, { ArmLink.Gripper, roll },
            };
            Color[] colours = { new Color(0.6f, 0.6f, 0.6f), new Color(0.9f, 0.3f, 0.3f), new Color(0.3f, 0.8f, 0.3f), new Color(0.3f, 0.5f, 1f), new Color(1f, 0.8f, 0.2f) };
            var shapes = new GameObject("CollisionPreview") { hideFlags = HideFlags.HideAndDontSave };
            Shader unlit = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
            Material Mat(Color c)
            {
                var m = new Material(unlit) { color = c };
                if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
                return m;
            }

            foreach (Obb b in bodies)
            {
                Transform frame = frameOf[b.Link];
                GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.transform.SetParent(shapes.transform, true);
                go.transform.position = frame.TransformPoint(new Vector3(-b.Center.y, b.Center.z, b.Center.x));
                go.transform.rotation = frame.rotation * ToQuaternion(b.Axes).ToUnity();
                Vector3 h = b.Half;
                go.transform.localScale = new Vector3(2f * h.y, 2f * h.z, 2f * h.x); // Core x, y, z are Unity z, x, y
                go.GetComponent<Renderer>().sharedMaterial = Mat(colours[(int)b.Link]);
            }

            foreach (var b in boxes)
            {
                GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.transform.SetParent(shapes.transform, true);
                Vector3 min = new Vector3(-b.max.y, b.min.z, b.min.x), max = new Vector3(-b.min.y, b.max.z, b.max.x);
                go.transform.position = baseAnchor.TransformPoint(0.5f * (min + max));
                go.transform.rotation = baseAnchor.rotation;
                go.transform.localScale = max - min;
                go.GetComponent<Renderer>().sharedMaterial = Mat(new Color(0.55f, 0.35f, 0.75f));
            }

            ShaderUtil.allowAsyncCompilation = false;
            var cam = new GameObject("cam") { hideFlags = HideFlags.HideAndDontSave }.AddComponent<Camera>();
            cam.enabled = false;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.85f, 0.87f, 0.9f);
            cam.fieldOfView = 40f;
            cam.allowHDR = false;
            cam.allowMSAA = true;
            cam.nearClipPlane = 0.02f;
            cam.farClipPlane = 10f;
            Vector3 centre = baseAnchor.position + Vector3.up * 0.02f + baseAnchor.forward * 0.08f;
            var views = new (string name, Vector3 dir)[] { ("side", -baseAnchor.right), ("front", baseAnchor.forward), ("top", Vector3.up + 0.01f * baseAnchor.forward), ("quarter", (-baseAnchor.right + baseAnchor.forward + Vector3.up).normalized) };
            foreach (var v in views)
            {
                cam.transform.position = centre + v.dir.normalized * 0.9f;
                cam.transform.LookAt(centre, Mathf.Abs(Vector3.Dot(v.dir.normalized, Vector3.up)) > 0.9f ? baseAnchor.forward : Vector3.up);
                Debug.Log($"[collision] preview {v.name}: camera {cam.transform.position} looking at {centre}");
                foreach (bool showShapes in new[] { false, true })
                {
                    shapes.SetActive(showShapes);
                    Texture2D tex = LabCapture.Render(cam, 900, 700);
                    File.WriteAllBytes(Path.Combine(outDir, $"{v.name}_{(showShapes ? "shapes" : "cad")}.png"), tex.EncodeToPNG());
                    Object.DestroyImmediate(tex);
                }
            }

            Object.DestroyImmediate(cam.gameObject);
            Object.DestroyImmediate(shapes);
        }
    }
}
