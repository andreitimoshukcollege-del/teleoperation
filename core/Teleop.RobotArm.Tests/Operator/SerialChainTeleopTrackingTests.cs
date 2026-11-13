using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Tests.Kinematics;
using Teleop.RobotArm.Types;
using Xunit.Abstractions;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>
    /// How well the two-hand operator follows a moving operator (docs/adr/0029 amendment 2): the shortest-path step mode
    /// against the first one (<see cref="ChainStepMode.FullSolve"/>), on the same synthetic hands. The user reported the IK
    /// "glitchy": refusing some positions and making huge rotations to reach others. These numbers are that, measured.
    ///
    /// <para><b>The hands</b> wander smoothly (sums of three sines per axis, 0.07-0.4 Hz, seeded) around the neutral pose in
    /// the body frame: about ±15 cm, the palm ±35° in yaw, ±30° in pitch and ±70° in roll, through the absolute mapping.
    /// Both arms are locked on at the start, from Franka's ready pose; 30 s at 90 Hz per seed, the first second dropped.</para>
    /// <para><b>The numbers,</b> per arm and frame: blocked and out-of-reach frames; off-target frames (the tool more than
    /// 1 cm or 5° from its target, whatever the arm reports) and false alarms (the arm reports a refusal within both); the
    /// tool's distance and angle to its target; the largest joint move in a frame (the swings); how often the reported
    /// state changes (per arm-second: a refusal that flickers on and off); and the tool's path against the target's (above
    /// 1, the tool wanders).</para>
    /// </summary>
    public class SerialChainTeleopTrackingTests
    {
        private readonly ITestOutputHelper _out;

        public SerialChainTeleopTrackingTests(ITestOutputHelper output) => _out = output;

        internal sealed class Score
        {
            public int Frames, Blocked, Short, SwingFrames, OffTarget, FalseAlarms, Changes;
            public readonly List<double> PositionMm = new List<double>(), AngleDeg = new List<double>(), Swing = new List<double>();
            public double ToolPath, TargetPath;

            public double BlockedShare => (double)Blocked / Frames;
            public double ShortShare => (double)Short / Frames;
            public double OffTargetShare => (double)OffTarget / Frames;
            public double PathRatio => ToolPath / TargetPath;
        }

        /// <summary>One smooth coordinate: three sines of the given total amplitude, seeded.</summary>
        private sealed class Wander
        {
            private readonly double[] _a = new double[3], _f = new double[3], _p = new double[3];

            public Wander(ref SeededRng rng, double amplitude)
            {
                for (int i = 0; i < 3; i++)
                {
                    _a[i] = amplitude / 3.0 * (0.6 + 0.8 * rng.NextDouble());
                    _f[i] = 0.07 + 0.33 * rng.NextDouble();
                    _p[i] = 2 * Math.PI * rng.NextDouble();
                }
            }

            public double At(double t) => _a[0] * Math.Sin(2 * Math.PI * _f[0] * t + _p[0]) + _a[1] * Math.Sin(2 * Math.PI * _f[1] * t + _p[1]) + _a[2] * Math.Sin(2 * Math.PI * _f[2] * t + _p[2]);
        }

        internal static Score Run(SerialChainTeleopOptions options, ulong seed, double seconds = 30.0)
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            SerialChainCollisionModel collision = SerialChainSolverTests.Collision();
            var teleop = new SerialChainTeleop(profile, collision, options);
            var mapping = new BodyAnchoredMapping(profile, BodyAnchoredMapping.NeutralTools(profile));
            mapping.Recentre(RigidTransform.Identity);
            Assert.True(teleop.TryTakePose(ReadyMeasured(profile)));

            var rng = new SeededRng(seed);
            var axes = new Wander[2, 6];
            double[] amplitude = { 0.15, 0.12, 0.15, 0.6, 0.5, 1.2 }; // forward, left, up (m); yaw, pitch, roll (rad)
            for (int a = 0; a < 2; a++)
                for (int k = 0; k < 6; k++)
                    axes[a, k] = new Wander(ref rng, amplitude[k]);

            RigidTransform Hand(int a, double t)
            {
                double side = profile.Arms[a].Mount.Y >= 0 ? 0.20 : -0.20;
                RigidTransform r = RigidTransform.Rotation(0, 0, 1, axes[a, 3].At(t))
                    .Compose(RigidTransform.Rotation(0, 1, 0, axes[a, 4].At(t)))
                    .Compose(RigidTransform.Rotation(1, 0, 0, axes[a, 5].At(t)));
                return new RigidTransform(0.30 + axes[a, 0].At(t), side + axes[a, 1].At(t), -0.40 + axes[a, 2].At(t), r.Qw, r.Qx, r.Qy, r.Qz);
            }

            for (int a = 0; a < 2; a++) Assert.True(teleop.EngageAt(a, mapping.Map(a, Hand(a, 0))));
            var score = new Score();
            const double dt = 1.0 / 90.0;
            var before = new double[2][];
            var lastTool = new RigidTransform[2];
            var lastTarget = new RigidTransform[2];
            var lastStatus = new ChainArmStatus[2];
            for (int step = 0; step * dt < seconds; step++)
            {
                double t = step * dt;
                for (int a = 0; a < 2; a++)
                {
                    teleop.Track(a, mapping.Map(a, Hand(a, t)));
                    before[a] = teleop.Commanded(a).ToArray();
                }

                teleop.Step(dt);
                for (int a = 0; a < 2; a++)
                {
                    RigidTransform tool = teleop.Tool(a), target = teleop.Target(a);
                    if (t >= 1.0)
                    {
                        score.Frames++;
                        if (teleop.Status(a) == ChainArmStatus.Blocked) score.Blocked++;
                        if (teleop.Status(a) == ChainArmStatus.StoppedShort) score.Short++;
                        double mm = 1000 * Distance(tool, target);
                        double deg = 57.29578 * SerialChainTeleop.AngleBetween(tool, target);
                        score.PositionMm.Add(mm);
                        score.AngleDeg.Add(deg);
                        bool off = mm > 10 || deg > 5;
                        if (off) score.OffTarget++;
                        if (!off && teleop.Status(a) != ChainArmStatus.Following) score.FalseAlarms++;
                        if (t > 1.0 && teleop.Status(a) != lastStatus[a]) score.Changes++;
                        double[] after = teleop.Commanded(a).ToArray();
                        double swing = 0;
                        for (int j = 0; j < after.Length; j++) swing = Math.Max(swing, Math.Abs(after[j] - before[a][j]));
                        score.Swing.Add(swing);
                        if (swing > 0.05) score.SwingFrames++;
                        score.ToolPath += Distance(tool, lastTool[a]);
                        score.TargetPath += Distance(target, lastTarget[a]);
                    }

                    lastTool[a] = tool;
                    lastTarget[a] = target;
                    lastStatus[a] = teleop.Status(a);
                }
            }

            return score;
        }

        [Fact]
        public void TheShortestPathStep_FollowsBetterThanTheFullSolve_OnTheSameHands()
        {
            var before = new List<Score>();
            var now = new List<Score>();
            foreach (ulong seed in new ulong[] { 1, 2, 3, 4, 5 })
            {
                // The first way as it shipped: no joint speed limit either.
                before.Add(Run(new SerialChainTeleopOptions { StepMode = ChainStepMode.FullSolve, JointSpeedShare = 0 }, seed));
                now.Add(Run(new SerialChainTeleopOptions(), seed));
            }

            Score f = Merge(before), p = Merge(now);
            _out.WriteLine("                     blocked  out-of-reach  off target  false alarm   err p50/p95 (mm)   angle p50/p95 (deg)   swing p99/max (rad/frame)  frames >0.05 rad  changes/s  tool/target path");
            Line("FullSolve (before)", f);
            Line("CartesianPath (now)", p);

            // What the user felt: refusals, and swings. The new step must cut both, follow closer, and never wander more.
            Assert.True(p.BlockedShare + p.ShortShare < f.BlockedShare + f.ShortShare, "no fewer refusals than before");
            Assert.True(p.FalseAlarms < f.FalseAlarms, "no fewer false alarms than before");
            Assert.True(p.OffTargetShare <= f.OffTargetShare, "off target more often than before");
            Assert.True(Percentile(p.PositionMm, 95) < Percentile(f.PositionMm, 95), "no closer in position than before");
            Assert.True(Percentile(p.AngleDeg, 95) < Percentile(f.AngleDeg, 95), "no closer in angle than before");
            Assert.True(Percentile(p.Swing, 99) < Percentile(f.Swing, 99), "swings no smaller than before");
            Assert.True(p.Swing.Max() < f.Swing.Max(), "the largest swing is no smaller than before");
            Assert.True(p.SwingFrames < f.SwingFrames, "no fewer swing frames than before");
            Assert.True(p.PathRatio < 1.05, $"the tool travels {p.PathRatio:0.00} times its target's path");
        }

        private void Line(string name, Score s) =>
            _out.WriteLine($"{name,-20} {100 * s.BlockedShare,6:0.0}% {100 * s.ShortShare,9:0.0}%  {100 * s.OffTargetShare,9:0.0}%  {100.0 * s.FalseAlarms / s.Frames,9:0.0}%" +
                           $"      {Percentile(s.PositionMm, 50),6:0.0} / {Percentile(s.PositionMm, 95),6:0.0}" +
                           $"       {Percentile(s.AngleDeg, 50),6:0.0} / {Percentile(s.AngleDeg, 95),6:0.0}          {Percentile(s.Swing, 99),6:0.000} / {s.Swing.Max(),6:0.000}" +
                           $"          {s.SwingFrames,6}         {s.Changes / (s.Frames / 90.0),6:0.00}        {s.PathRatio,5:0.00}");

        private static Score Merge(List<Score> scores)
        {
            var m = new Score();
            foreach (Score s in scores)
            {
                m.Frames += s.Frames;
                m.Blocked += s.Blocked;
                m.Short += s.Short;
                m.SwingFrames += s.SwingFrames;
                m.OffTarget += s.OffTarget;
                m.FalseAlarms += s.FalseAlarms;
                m.Changes += s.Changes;
                m.PositionMm.AddRange(s.PositionMm);
                m.AngleDeg.AddRange(s.AngleDeg);
                m.Swing.AddRange(s.Swing);
                m.ToolPath += s.ToolPath;
                m.TargetPath += s.TargetPath;
            }

            return m;
        }

        internal static double Percentile(List<double> values, double p)
        {
            var sorted = values.OrderBy(v => v).ToList();
            return sorted[(int)Math.Min(sorted.Count - 1, Math.Ceiling(p / 100.0 * sorted.Count) - 1)];
        }

        private static double Distance(in RigidTransform a, in RigidTransform b) =>
            Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

        private static JointSample[] ReadyMeasured(SerialChainProfile profile)
        {
            var samples = new List<JointSample>();
            foreach (ChainArm arm in profile.Arms)
            {
                bool mirrored = arm.Mount.Y < 0;
                for (int j = 0; j < arm.Joints.Length; j++)
                {
                    double q = mirrored && j % 2 == 0 ? -BodyAnchoredMapping.FrankaReady[j] : BodyAnchoredMapping.FrankaReady[j];
                    samples.Add(new JointSample(arm.Joints[j].MotorId, (float)q, true));
                }

                if (arm.HasGripper) samples.Add(new JointSample(arm.Gripper.MotorId, 1f, true));
            }

            return samples.ToArray();
        }
    }
}
