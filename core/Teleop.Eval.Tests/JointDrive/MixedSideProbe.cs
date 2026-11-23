using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Training;
using Teleop.RobotArm.Types;
using Xunit;
using static Teleop.Eval.Tests.JointDrive.Takes;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>TEMPORARY diagnostic: where the pads reach beside the robot (bearing 60-100 degrees), and what refuses the rest.</summary>
    public class MixedSideProbe
    {
        [Fact]
        public void Probe()
        {
            float table = -0.1962f;
            var lines = new List<string>();
            foreach (float bearingDeg in new[] { 30f, 60f, 75f, 89f, 100f })
            {
                foreach (float pitchDeg in new[] { -88f, -70f, -55f, -40f })
                {
                    float pitch = pitchDeg * MathF.PI / 180f;
                    Vector3 t = MixedTake.ToolInGripper;
                    float lh = t.X * MathF.Cos(pitch) - t.Z * MathF.Sin(pitch), lv = t.X * MathF.Sin(pitch) + t.Z * MathF.Cos(pitch);
                    var row = new List<string>();
                    foreach (float hcm in new[] { 3f, 5f, 8f })
                    {
                        var cells = new List<char>();
                        foreach (float r in new[] { 0.18f, 0.21f, 0.24f, 0.27f, 0.30f })
                        {
                            float b = bearingDeg * MathF.PI / 180f;
                            var tool = new Vector3(r * MathF.Cos(b), r * MathF.Sin(b), table + hcm / 100f);
                            var rad = Vector2.Normalize(new Vector2(tool.X, tool.Y));
                            var w = new Vector3(tool.X - lh * rad.X, tool.Y - lh * rad.Y, tool.Z - lv);
                            var solver = new ArmSolver();
                            solver.Reset(Profile, Model.Home);
                            char c = 'x';
                            for (int i = 0; i < TrainingTargetSampler.MaxSolvesPerCheck; i++)
                            {
                                ArmSolution s = solver.Solve(Profile, w, pitch, 0f);
                                if (s.Status == ArmSolveStatus.Approaching) continue;
                                bool reach = (s.Status == ArmSolveStatus.Exact || s.Status == ArmSolveStatus.PitchAdjusted) && !s.ReachClamped && Vector3.Distance(s.Position, w) <= 0.003f;
                                if (s.Status == ArmSolveStatus.Blocked)
                                {
                                    // What the exact pose would hit: B the robot's body boxes, S the arm itself or the floor.
                                    CollisionHit hit = ArmCollision.Check(Profile, Model, s.Pose);
                                    c = hit.IsHit ? (hit.Describe(Model).Contains("body") || hit.Describe(Model).Contains("chassis") || hit.Describe(Model).Contains("base") ? 'B' : 'S') : 'b';
                                }
                                else if (!reach) c = 'i';
                                else if (!Envelope.Contains(s.Pose)) c = 'e';
                                else if (!ArmCollision.IsValid(Profile, Model, s.Pose)) c = 'm';
                                else c = 'O';
                                break;
                            }

                            cells.Add(c);
                        }

                        row.Add($"h{hcm:0}cm " + new string(cells.ToArray()));
                    }

                    lines.Add($"bearing {bearingDeg,3:0} pitch {pitchDeg,3:0}: " + string.Join(" | ", row));
                }
            }

            Assert.True(false, string.Join("\n", lines));
        }
    }
}
