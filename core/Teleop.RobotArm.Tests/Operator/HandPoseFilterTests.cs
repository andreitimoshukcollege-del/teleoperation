using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Tests.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>The hand filter (docs/adr/0029 amendment 7): a still hand's tracking jitter goes, a moving hand barely lags.</summary>
    public class HandPoseFilterTests
    {
        private const double Frame = 1 / 90.0;
        private static readonly HandFilterConfig Config = new HandFilterConfig();

        /// <summary>
        /// A still hand as tracking reports it: up to 1.5 mm off on each axis and up to 0.01 rad about two axes, new every
        /// frame. With it the arm's joints moved 12 rad in 3 s of a hand held still (2026-11-01's jitter, reproduced).
        /// </summary>
        private static RigidTransform Noisy(in RigidTransform hand, ref SeededRng rng)
        {
            RigidTransform r = hand.Compose(RigidTransform.Rotation(0, 0, 1, 0.01 * N(ref rng))).Compose(RigidTransform.Rotation(1, 0, 0, 0.01 * N(ref rng)));
            return new RigidTransform(hand.X + 0.0015 * N(ref rng), hand.Y + 0.0015 * N(ref rng), hand.Z + 0.0015 * N(ref rng), r.Qw, r.Qx, r.Qy, r.Qz);
        }

        /// <summary>Uniform in -1..1. <see cref="SeededRng"/> is a struct: passed by value, it would repeat one draw.</summary>
        private static double N(ref SeededRng rng) => (rng.NextDouble() - 0.5) * 2;

        private static double Distance(in RigidTransform a, in RigidTransform b) =>
            Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

        private static readonly RigidTransform Palm = RigidTransform.FromXyzRpy(0.3, 0.2, -0.4, 0.3, -0.6, 0.4);

        [Fact]
        public void AStillHand_KeepsUnderAThirdOfItsTrackingNoise()
        {
            var filter = new HandPoseFilter();
            var rng = new SeededRng(3);
            RigidTransform still = Palm;
            double rawSq = 0, filteredSq = 0, rawAngleSq = 0, filteredAngleSq = 0;
            for (int k = 0; k < 360; k++)
            {
                RigidTransform hand = Noisy(still, ref rng);
                RigidTransform f = filter.Filter(Config, hand, k * Frame);
                if (k < 90) continue; // the first second settles
                rawSq += Math.Pow(Distance(hand, still), 2);
                filteredSq += Math.Pow(Distance(f, still), 2);
                rawAngleSq += Math.Pow(SerialChainTeleop.AngleBetween(hand, still), 2);
                filteredAngleSq += Math.Pow(SerialChainTeleop.AngleBetween(f, still), 2);
            }

            Assert.InRange(Math.Sqrt(filteredSq / rawSq), 0, 0.3);
            Assert.InRange(Math.Sqrt(filteredAngleSq / rawAngleSq), 0, 0.3);
        }

        [Theory]
        [InlineData(0.02)]
        [InlineData(0.1)]
        [InlineData(0.5)]
        [InlineData(1.0)]
        public void AMovingHand_LagsAtMostTwoAndAHalfMillimetres(double speed)
        {
            // A steady motion, noise on: at speed the cutoff is high, so the lag is near 1/(2π·beta), 2 mm, at any speed.
            var filter = new HandPoseFilter();
            var rng = new SeededRng(5);
            double behind = 0;
            int n = 0;
            for (int k = 0; k < 200; k++)
            {
                double t = k * Frame;
                var hand = new RigidTransform(Palm.X + speed * t, Palm.Y, Palm.Z, Palm.Qw, Palm.Qx, Palm.Qy, Palm.Qz);
                RigidTransform f = filter.Filter(Config, Noisy(hand, ref rng), t);
                if (t < 1.5) continue;
                behind += hand.X - f.X;
                n++;
            }

            Assert.InRange(behind / n, 0, 0.0025);
        }

        [Theory]
        [InlineData(0.2)]
        [InlineData(1.0)]
        [InlineData(3.0)]
        public void ATurningPalm_LagsAtMostThreeQuartersOfADegree(double rate)
        {
            var filter = new HandPoseFilter();
            double behind = 0;
            int n = 0;
            for (int k = 0; k < 200; k++)
            {
                double t = k * Frame;
                RigidTransform hand = Palm.Compose(RigidTransform.Rotation(0, 0, 1, rate * t));
                RigidTransform f = filter.Filter(Config, hand, t);
                if (t < 1.5) continue;
                behind += SerialChainTeleop.AngleBetween(f, hand);
                n++;
            }

            Assert.InRange(behind / n, 0, 0.75 * Math.PI / 180);
        }

        [Fact]
        public void AReachFromStill_LagsAtMostSixAndAHalfMillimetres_AndUnderFiveATenthOfASecondIn()
        {
            // The One Euro's known cost: the speed estimate (low-passed at 1 Hz) needs a moment to see a motion start, and
            // until it does the still hand's low cutoff holds the target back. At 0.3 m/s: 5.9 mm at most, 4.5 mm at 0.1 s.
            var filter = new HandPoseFilter();
            double at100 = double.NaN, most = 0;
            for (int k = 0; k < 120; k++)
            {
                double t = k * Frame, x = t < 1 ? 0 : 0.3 * (t - 1);
                RigidTransform f = filter.Filter(Config, new RigidTransform(x, 0, 0, 1, 0, 0, 0), t);
                most = Math.Max(most, x - f.X);
                if (Math.Abs(t - 1.1) < Frame / 2) at100 = x - f.X;
            }

            Assert.InRange(most, 0, 0.0065);
            Assert.InRange(at100, 0, 0.005);
        }

        [Fact]
        public void TheFirstSample_AndTheFirstAfterAGap_PassThrough()
        {
            var filter = new HandPoseFilter();
            AssertSame(Palm, filter.Filter(Config, Palm, 10.0));
            // A 2 mm twitch a frame later is smoothed: the filter has started.
            var twitch = new RigidTransform(Palm.X + 0.002, Palm.Y, Palm.Z, Palm.Qw, Palm.Qx, Palm.Qy, Palm.Qz);
            Assert.InRange(filter.Filter(Config, twitch, 10.0 + Frame).X - Palm.X, 0, 0.001);

            RigidTransform far = new RigidTransform(-0.5, 0.4, 0.1, 0, 1, 0, 0);
            AssertSame(far, filter.Filter(Config, far, 10.0 + Frame + Config.MaxGapSeconds * 1.5));
        }

        [Fact]
        public void ARepeatedOrEarlierTime_ChangesNothing()
        {
            var a = new HandPoseFilter();
            var b = new HandPoseFilter();
            var rng = new SeededRng(7);
            RigidTransform last = default;
            for (int k = 0; k < 50; k++)
            {
                RigidTransform hand = Noisy(new RigidTransform(Palm.X + 0.002 * k, Palm.Y, Palm.Z, Palm.Qw, Palm.Qx, Palm.Qy, Palm.Qz), ref rng);
                last = a.Filter(Config, hand, k * Frame);
                b.Filter(Config, hand, k * Frame);
            }

            var elsewhere = new RigidTransform(1, 1, 1, 0, 0, 1, 0);
            AssertSame(last, a.Filter(Config, elsewhere, 49 * Frame));
            AssertSame(last, a.Filter(Config, elsewhere, 20 * Frame));
            AssertSame(b.Filter(Config, Palm, 50 * Frame), a.Filter(Config, Palm, 50 * Frame));
        }

        [Fact]
        public void ANonFiniteSample_PassesThrough_AndStartsAgain()
        {
            var filter = new HandPoseFilter();
            for (int k = 0; k < 20; k++) filter.Filter(Config, Palm, k * Frame);
            var broken = new RigidTransform(double.NaN, 0, 0, 1, 0, 0, 0);
            Assert.True(double.IsNaN(filter.Filter(Config, broken, 20 * Frame).X));
            var next = new RigidTransform(0.1, 0.1, 0.1, 1, 0, 0, 0);
            AssertSame(next, filter.Filter(Config, next, 21 * Frame));
        }

        [Fact]
        public void AFlippedQuaternion_IsTheSameRotation_NotASpin()
        {
            // Unity may hand over q or -q for one orientation; the filter must see no turn between them.
            var filter = new HandPoseFilter();
            var flipped = new RigidTransform(Palm.X, Palm.Y, Palm.Z, -Palm.Qw, -Palm.Qx, -Palm.Qy, -Palm.Qz);
            for (int k = 0; k < 90; k++)
            {
                RigidTransform f = filter.Filter(Config, k % 2 == 0 ? Palm : flipped, k * Frame);
                Assert.InRange(SerialChainTeleop.AngleBetween(f, Palm), 0, 1e-6);
            }

            Assert.InRange(filter.TurnRate, 0, 1e-6);
        }

        [Fact]
        public void Reset_RestoresTheFilterAsConstructed()
        {
            var used = new HandPoseFilter();
            var rng = new SeededRng(11);
            for (int k = 0; k < 100; k++) used.Filter(Config, Noisy(Palm, ref rng), k * Frame);
            used.Reset();
            var fresh = new HandPoseFilter();
            var again = new SeededRng(12);
            var copy = new SeededRng(12);
            for (int k = 0; k < 100; k++)
            {
                double t = 5 + k * Frame;
                AssertSame(fresh.Filter(Config, Noisy(Palm, ref again), t), used.Filter(Config, Noisy(Palm, ref copy), t));
            }
        }

        [Fact]
        public void IdenticalInput_GivesIdenticalOutput()
        {
            var a = new HandPoseFilter();
            var b = new HandPoseFilter();
            var ra = new SeededRng(13);
            var rb = new SeededRng(13);
            for (int k = 0; k < 300; k++)
            {
                RigidTransform moving = Palm.Compose(RigidTransform.Rotation(0, 1, 0, 0.5 * Math.Sin(k * Frame)));
                AssertSame(a.Filter(Config, Noisy(moving, ref ra), k * Frame), b.Filter(Config, Noisy(moving, ref rb), k * Frame));
            }
        }

        [Fact]
        public void Filtering_AllocatesNothing()
        {
            var filter = new HandPoseFilter();
            double t = 0;
            for (int i = 0; i < 500; i++) // warm up: the first calls JIT, which counts as allocation
            {
                t += Frame;
                filter.Filter(Config, Palm.Compose(RigidTransform.Rotation(0, 0, 1, t)), t);
            }

            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 2000; i++)
            {
                t += Frame;
                filter.Filter(Config, Palm.Compose(RigidTransform.Rotation(0, 0, 1, t)), t);
            }

            Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
        }

        [Fact]
        public void TheArm_UnderAStillNoisyHand_MovesAFifthAsMuchFiltered()
        {
            // End to end through the IK: the jitter the operator saw was the joints chasing a still hand's noise.
            double raw = JointPathUnderAStillHand(null), filtered = JointPathUnderAStillHand(Config);
            Assert.InRange(filtered, 0, 0.2 * raw);
        }

        [Fact]
        public void Validate_RejectsCutoffsAndBetasThatCannotFilter()
        {
            Assert.Null(new HandFilterConfig().Validate());
            Assert.NotNull(new HandFilterConfig { MinCutoffHz = 0 }.Validate());
            Assert.NotNull(new HandFilterConfig { DerivativeCutoffHz = double.NaN }.Validate());
            Assert.NotNull(new HandFilterConfig { PositionBeta = -1 }.Validate());
            Assert.NotNull(new HandFilterConfig { RotationBeta = double.PositiveInfinity }.Validate());
            Assert.NotNull(new HandFilterConfig { MaxGapSeconds = 0 }.Validate());
        }

        /// <summary>The right arm's joint motion (rad) over 3 s of a still hand, after a second to settle.</summary>
        private static double JointPathUnderAStillHand(HandFilterConfig? config)
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            SerialChainCollisionModel collision = SerialChainSolverTests.Collision();
            double[][] arms =
            {
                new[] { -0.0882, -0.7412, -0.2330, -2.6143, 2.7212, 2.1025, 0.5537 },
                new[] { -0.0578, -0.7210, -0.1975, -2.4718, 1.5825, 4.3749, -2.0 },
            };
            var samples = new List<JointSample>();
            for (int a = 0; a < 2; a++)
            {
                for (int j = 0; j < 7; j++) samples.Add(new JointSample(profile.Arms[a].Joints[j].MotorId, (float)arms[a][j], true));
                samples.Add(new JointSample(profile.Arms[a].Gripper.MotorId, 1f, true));
            }

            var teleop = new SerialChainTeleop(profile, collision);
            Assert.True(teleop.TryTakePose(samples.ToArray()));
            RigidTransform still = teleop.Tool(1);
            Assert.True(teleop.EngageAt(1, still));
            var filter = new HandPoseFilter();
            var rng = new SeededRng(3);
            double[] previous = teleop.Commanded(1).ToArray();
            double path = 0;
            for (int k = 0; k < 360; k++)
            {
                RigidTransform hand = Noisy(still, ref rng);
                teleop.Track(1, config == null ? hand : filter.Filter(config, hand, k * Frame));
                teleop.Step(Frame);
                double[] q = teleop.Commanded(1).ToArray();
                if (k >= 90)
                {
                    for (int j = 0; j < q.Length; j++) path += Math.Abs(q[j] - previous[j]);
                }

                previous = q;
            }

            return path;
        }

        private static void AssertSame(in RigidTransform expected, in RigidTransform actual)
        {
            Assert.Equal(expected.X, actual.X, 12);
            Assert.Equal(expected.Y, actual.Y, 12);
            Assert.Equal(expected.Z, actual.Z, 12);
            double dot = Math.Abs(expected.Qw * actual.Qw + expected.Qx * actual.Qx + expected.Qy * actual.Qy + expected.Qz * actual.Qz);
            double n = Math.Sqrt(expected.Qw * expected.Qw + expected.Qx * expected.Qx + expected.Qy * expected.Qy + expected.Qz * expected.Qz);
            Assert.Equal(1.0, dot / n, 12);
        }
    }
}
