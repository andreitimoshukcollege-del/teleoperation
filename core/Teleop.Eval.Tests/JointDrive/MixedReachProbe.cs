using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Training;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Xunit;
using Xunit.Abstractions;
using static Teleop.Eval.Tests.JointDrive.Takes;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>TEMPORARY diagnostic: where the pads can be, by height above the table, radius and pitch.</summary>
    public class MixedReachProbe
    {
        private readonly ITestOutputHelper _output;
        public MixedReachProbe(ITestOutputHelper output) => _output = output;

        [Fact]
        public void Probe()
        {
            float table = -0.1962f;
            var lines = new List<string>();
            foreach (float pitchDeg in new[] { -88f, -75f, -60f, -45f, -30f, -15f })
            {
                float pitch = pitchDeg * MathF.PI / 180f;
                Vector3 t = (ShapeGap.LeftPadAtZero.Center + ShapeGap.RightPadAtZero.Center) / 2f;
                float lh = t.X * MathF.Cos(pitch) - t.Z * MathF.Sin(pitch), lv = t.X * MathF.Sin(pitch) + t.Z * MathF.Cos(pitch);
                var row = new List<string>();
                foreach (float hcm in new[] { 2.5f, 4f, 6f, 9f, 13f, 18f, 25f })
                {
                    var cells = new List<string>();
                    foreach (float r in new[] { 0.12f, 0.15f, 0.18f, 0.21f, 0.24f, 0.27f, 0.30f, 0.33f })
                    {
                        var tool = new Vector3(r, 0.05f, table + hcm / 100f);
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
                            if (!reach) c = 'i';
                            else if (!Envelope.Contains(s.Pose)) c = 'e';
                            else if (!ArmCollision.IsValid(Profile, Model, s.Pose)) c = 'm';
                            else c = s.Status == ArmSolveStatus.PitchAdjusted ? 'p' : 'O';
                            break;
                        }

                        cells.Add(c.ToString());
                    }

                    row.Add($"h{hcm,4:0.#}cm " + string.Join("", cells));
                }

                lines.Add($"pitch {pitchDeg,4:0}: " + string.Join(" | ", row));
            }

            foreach (string l in lines) _output.WriteLine(l);
            Assert.True(false, string.Join("\n", lines));
        }
    }
}
