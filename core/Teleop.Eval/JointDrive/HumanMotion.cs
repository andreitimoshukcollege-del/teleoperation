using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// How a person drives the JetRover through the headset's clutch, as numbers (docs/adr/0036 amendment 2). Measured
    /// 2026-11-15 on 31 human sessions (12 on the robot, 19 on the Isaac twin; 55 min, 117k commands): a stroke is a chain of
    /// overlapping submovements (1.6 speed peaks a second of motion, the speed between them at ~22% of the peaks, never a bell
    /// that comes to rest), the clutch is let go while the hand still moves (75% of holds) and taken again on the move (46%),
    /// paths wander (chord/path 0.37), the hand's jitter grows with its speed, and the roll and gripper are in use. The
    /// <c>joint-drive</c> scripts that came before (minimum-jerk strokes to rest, then the next) were outside the human band on
    /// 37 of 45 measures. Speeds are the wrist's (the clutch's target), which moves at half the hand's.
    /// </summary>
    public static class HandModel
    {
        /// <summary>joint-drive's command rate: one target a tick.</summary>
        public const double TickSeconds = 1.0 / 48.0;

        /// <summary>The clutch's floor, as the headset keeps the target above it (<see cref="ClutchMapping"/>).</summary>
        public const float MinHeightMeters = CourseScript.MinHeightMeters;

        // A submovement's duration: DurationScale * amplitude^DurationExponent * lognormal(DurationSigma), clamped (the human
        // main sequence: nearly isochronous, T ~ A^0.20; peak speed ~ A^0.83).
        public const double DurationScale = 0.70, DurationExponent = 0.22, DurationSigma = 0.45, MinDuration = 0.18, MaxDuration = 1.3;

        /// <summary>The next submovement starts this fraction of the last one's duration after it began (uniform between the two): overlapping, so the speed rarely comes to rest.</summary>
        public const double OverlapLow = 0.40, OverlapHigh = 0.80;

        /// <summary>A submovement's time base is u^skew: its speed peaks before the middle, as a hand's does (single-peak bouts peak at 0.34 of their length).</summary>
        public const double SkewLow = 0.50, SkewHigh = 0.75;

        /// <summary>No submovement peaks faster than this, m/s (people's fastest: ~1 m/s at the wrist).</summary>
        public const double MaxPeakSpeed = 0.90;

        /// <summary>A tick that moves the wrist faster than this is refused, m/s: a jump, not a drag.</summary>
        public const float MaxWristSpeed = 1.5f;

        // A goal-directed move: the goal's distance (lognormal, metres), the primary's endpoint error along and across the
        // way (fractions of the distance), and corrections aimed at what is left until within the tolerance.
        public const double TransportMedian = 0.10, TransportSigma = 0.85, TransportMax = 0.30;
        public const double ErrAlongMu = -0.10, ErrAlongSigma = 0.16, ErrPerpSigma = 0.12;
        public const double CorrAlongMu = 0.0, CorrAlongSigma = 0.22, CorrPerpSigma = 0.18, Tolerance = 0.004, CorrectionSlow = 1.2;
        public const int MaxCorrections = 3;

        /// <summary>A move curves through a point off its line with this probability (two overlapping halves), by this sd of its length.</summary>
        public const double ViaProbability = 0.45, ViaSigma = 0.14;

        /// <summary>A straight move is made of two overlapping pushes (a dip in its speed) with this probability.</summary>
        public const double SplitProbability = 0.30;

        // Back and forth: 1 + Poisson(mean) strokes of a lognormal length, each bowed to one side so a reversal goes round a
        // curve (a straight reversal brings the speed to rest: people's reversals keep 22% of their speed).
        public const double ReciprocateMean = 3.0, ReciprocateMedian = 0.06, ReciprocateSigma = 0.70, ReciprocateRatioSigma = 0.35;
        public const double ReciprocateTurnSigmaDegrees = 15.0, ReciprocateBowLow = 0.10, ReciprocateBow = 0.35, ReciprocateLoop = 0.85;

        // Fine positioning: up to AdjustMax small moves of a lognormal length.
        public const int AdjustMax = 3;
        public const double AdjustMedian = 0.012, AdjustSigma = 0.6;

        // What the operator does next: a move to a goal, back and forth, or small moves.
        public const double TransportProbability = 0.50, ReciprocateProbability = 0.32;

        // Between those: carry straight on (overlapping), keep the clutch but hold the hand still, or let go of the clutch.
        // Continuous by the user's choice (2026-11-16, docs/adr/0036 amendment 3): people carry on 63% of the time and let go
        // in 62% of the rest (the fit of amendment 2); the takes looked stop-and-go, so the hand carries on 92% of the time,
        // a pause is a brief dwell (at most DwellMax), and the clutch is never let go mid-take.
        public const double ContinueProbability = 0.92, ReleaseProbability = 0.0;
        public const double DwellMedian = 0.18, DwellSigma = 0.6, DwellMax = 0.5;

        /// <summary>A hold (the clutch let go): lognormal, seconds, at most HoldMax. People: median 0.72 s, p95 5 s; continuous
        /// takes (amendment 3) hold no longer than a dwell.</summary>
        public const double HoldMedian = 0.25, HoldSigma = 0.6, HoldMax = 0.5;

        /// <summary>The pinch opens while the hand still moves, this far into its last submovement (75% of people's holds start at speed).</summary>
        public const double CutOffProbability = 0.95, CutOffLow = 0.35, CutOffHigh = 0.70;

        /// <summary>The hand is already moving when the pinch closes again (46% of people's holds end at speed): its next move began this long before.</summary>
        public const double EngageMovingProbability = 0.42, EngageLeadLow = 0.04, EngageLeadHigh = 0.22;

        /// <summary>After a hold, a short re-pinch and another hold, with this probability; the re-pinch's length, seconds.</summary>
        public const double BlipProbability = 0.0, BlipLow = 0.05, BlipHigh = 0.25;

        /// <summary>A move interrupted by a short release while the hand goes on, with this probability.</summary>
        public const double MidReleaseProbability = 0.0, MidReleaseLow = 0.06, MidReleaseHigh = 0.20;

        // The hand's pitch: each move turns it by Laplace(PitchLaplaceDegrees), pulled back toward the centre; now and then
        // re-aimed. It follows over this fraction of the move.
        public const double PitchLaplaceDegrees = 34.0, OrientProbability = 0.30, OrientSigmaDegrees = 30.0;
        public const double PitchLowDegrees = -85.0, PitchHighDegrees = 40.0, PitchCentreDegrees = -24.0, PitchPull = 0.40, PitchTime = 0.7;

        // The wrist's roll: people turn it 41% of the time, within about +-40 degrees.
        public const double RollProbability = 0.95, RollLaplaceDegrees = 10.0, RollOrientProbability = 0.10, RollOrientSigmaDegrees = 25.0;
        public const double RollMaxDegrees = 50.0, RollPull = 0.15;

        // The gripper (the other hand's pinch): ~14 large changes a minute while the clutch is held, mostly while the arm
        // moves; open (0) a third of the time.
        public const double GripRatePerSecond = 0.36, GripDurationLow = 0.25, GripDurationHigh = 0.70, GripOpenProbability = 0.35;

        // The hand as tracked: a drifting position (an Ornstein-Uhlenbeck velocity, pulled back), and a jitter whose size
        // grows with the hand's speed (people's 4-15 Hz velocity ~ 80 v^0.78 mm/s, 1.45 mm/s still).
        public const double DriftVelocityTau = 0.10, DriftVelocitySigma = 0.0045, DriftPositionTau = 0.6;
        public const double JitterAr = 0.35, JitterFloorMeters = 0.00002, JitterGain = 0.0015, JitterExponent = 0.78;
        public const double PitchNoiseRadians = 0.30 * Math.PI / 180.0, PitchNoiseTau = 0.08;

        // Where free goals are drawn (people's wrist: radius 77-222 mm, height 67-247 mm, azimuth -47..36 degrees, p10-p90),
        // and how far past the reach shell the hand may push (the clutch keeps the target on the shell; people spend 15% of
        // their time there).
        public const float GoalRadiusLow = 0.07f, GoalRadiusHigh = 0.245f, GoalHeightLow = -0.02f, GoalHeightHigh = 0.27f;
        public const float GoalYawDegrees = 70f, ReachOvershootMeters = 0.025f;

        /// <summary>Half the free goals head for a point drawn where people work (radius, height and azimuth: mean, sd), so the wrist spends its time there.</summary>
        public const double WorkspacePull = 0.65, WorkRadius = 0.125, WorkRadiusSigma = 0.045, WorkHeight = 0.185, WorkHeightSigma = 0.06;
        public const double WorkAzimuthDegrees = -6.0, WorkAzimuthSigmaDegrees = 28.0;

        /// <summary>Where the wrist is at <paramref name="pose"/> (the IK's target point), arm base frame.</summary>
        public static Vector3 Wrist(in RobotArmProfile profile, in ArmPose pose) => ArmKinematics.Forward(profile, pose.BaseYaw, pose.Proximal, pose.Distal);

        /// <summary>The wrist's absolute pitch at <paramref name="pose"/>, as the IK counts it (proximal + distal + wrist).</summary>
        public static float PitchOf(in ArmPose pose) => pose.Proximal + pose.Distal + pose.Wrist;

        /// <summary>The minimum-jerk shape: the fraction of a move covered at the fraction <paramref name="u"/> of its time.</summary>
        public static double MinimumJerk(double u)
        {
            u = Math.Clamp(u, 0.0, 1.0);
            return u * u * u * (10 - u * (15 - 6 * u));
        }

        /// <summary>The minimum-jerk shape's rate: d<see cref="MinimumJerk"/>/du.</summary>
        public static double MinimumJerkRate(double u) => u <= 0 || u >= 1 ? 0 : 30 * u * u * (1 - u) * (1 - u);
    }

    /// <summary>The random draws the hand model needs, all from the plan's seeded generator.</summary>
    internal static class Draw
    {
        public static double Uniform(ref SeededRng rng, double low, double high) => low + (high - low) * rng.NextDouble();

        public static bool Chance(ref SeededRng rng, double p) => rng.NextDouble() < p;

        /// <summary>A standard normal draw (Box-Muller, one of the pair).</summary>
        public static double Normal(ref SeededRng rng)
        {
            double u1 = 1.0 - rng.NextDouble(), u2 = rng.NextDouble();
            return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }

        public static double Normal(ref SeededRng rng, double mean, double sigma) => mean + sigma * Normal(ref rng);

        public static double LogNormal(ref SeededRng rng, double median, double sigma) => median * Math.Exp(sigma * Normal(ref rng));

        /// <summary>
        /// How far a free move's goal is from the hand, m (the mixed script's layout-blind goals near the hand; docs/adr/0038 "M, motion":
        /// an aim's approach and a pass's lead-in draw the same): lognormal about <see cref="HandModel.TransportMedian"/>, within 1 cm and
        /// <see cref="HandModel.TransportMax"/>.
        /// </summary>
        public static float ReachDistance(ref SeededRng rng) => (float)Math.Clamp(LogNormal(ref rng, HandModel.TransportMedian, HandModel.TransportSigma), 0.01, HandModel.TransportMax);

        /// <summary>One small adjustment's distance, m, as the hand model's adjustments draw it: lognormal about <see cref="HandModel.AdjustMedian"/>, within 2 mm and 4 cm.</summary>
        public static float AdjustDistance(ref SeededRng rng) => (float)Math.Clamp(LogNormal(ref rng, HandModel.AdjustMedian, HandModel.AdjustSigma), 0.002, 0.04);

        public static double Laplace(ref SeededRng rng, double scale)
        {
            double u = rng.NextDouble() - 0.5;
            return -scale * Math.Sign(u) * Math.Log(Math.Max(1e-12, 1.0 - 2.0 * Math.Abs(u)));
        }

        public static int Poisson(ref SeededRng rng, double mean)
        {
            double limit = Math.Exp(-mean), p = 1.0;
            int k = 0;
            do
            {
                k++;
                p *= rng.NextDouble();
            }
            while (p > limit && k < 64);
            return k - 1;
        }

        public static Vector3 Gaussian3(ref SeededRng rng) => new Vector3((float)Normal(ref rng), (float)Normal(ref rng), (float)Normal(ref rng));

        /// <summary>A height drawn from bands of (low, high, share) triples (<see cref="BlindReach.ToolBands"/>): a band by its share, then uniformly within it.</summary>
        public static float Band(ref SeededRng rng, float[] bands)
        {
            float total = 0f;
            for (int i = 2; i < bands.Length; i += 3) total += bands[i];
            double u = rng.NextDouble() * total;
            int k = 0;
            for (int i = 0; i < bands.Length; i += 3)
            {
                k = i;
                u -= bands[i + 2];
                if (u < 0) break;
            }

            return (float)Uniform(ref rng, bands[k], bands[k + 1]);
        }

        /// <summary>A random direction, its vertical part scaled by <paramref name="zScale"/> before normalizing (people move less up and down).</summary>
        public static Vector3 Direction(ref SeededRng rng, float zScale)
        {
            for (int i = 0; i < 16; i++)
            {
                Vector3 g = Gaussian3(ref rng) * new Vector3(1f, 1f, zScale);
                float n = g.Length();
                if (n > 1e-4f) return g / n;
            }

            return Vector3.UnitX;
        }

        /// <summary>A normal vector across <paramref name="u"/> (a unit vector) with the given sd on each axis.</summary>
        public static Vector3 Across(ref SeededRng rng, Vector3 u, double sigma)
        {
            Vector3 w = Gaussian3(ref rng) * (float)sigma;
            return w - Vector3.Dot(w, u) * u;
        }
    }

    /// <summary>
    /// Caps on the commanded motion (docs/adr/0038 decision 1, the mixed script's speed caps; joint-drive's --max-joint-speed,
    /// --max-joint-accel, --max-wrist-speed, --max-peak-speed): each joint's commanded speed tick to tick within its
    /// <see cref="JointSpeed"/>, and its acceleration within its <see cref="JointAcceleration"/>, measured over two ticks (about the
    /// 25 ms smoothing the human figures were measured with); the wrist within <see cref="MaxWristSpeed"/>, and every submovement
    /// planned to peak no faster than <see cref="MaxPeakSpeed"/>. A part of the plan that breaks one is drawn again. Joints in the
    /// model's order: yaw, proximal (shoulder), distal (elbow), wrist pitch, roll. Infinite: no cap.
    /// </summary>
    public sealed class MotionCaps
    {
        /// <summary>Each joint's commanded speed, rad/s.</summary>
        public float[] JointSpeed { get; } = { float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity };

        /// <summary>Each joint's commanded acceleration, rad/s², over two ticks.</summary>
        public float[] JointAcceleration { get; } = { float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity };

        /// <summary>The wrist's commanded travel, tick to tick, m/s (the hand model refuses more than <see cref="HandModel.MaxWristSpeed"/> anyway).</summary>
        public float MaxWristSpeed { get; set; } = HandModel.MaxWristSpeed;

        /// <summary>A submovement's planned peak speed at the wrist, m/s: its duration stretches to it (the hand model's own is <see cref="HandModel.MaxPeakSpeed"/>).</summary>
        public double MaxPeakSpeed { get; set; } = HandModel.MaxPeakSpeed;

        /// <summary>Every joint's speed cap at once, rad/s.</summary>
        public MotionCaps AllSpeeds(float radPerSecond)
        {
            for (int j = 0; j < JointSpeed.Length; j++) JointSpeed[j] = radPerSecond;
            return this;
        }

        /// <summary>Every joint's acceleration cap at once, rad/s².</summary>
        public MotionCaps AllAccelerations(float radPerSecond2)
        {
            for (int j = 0; j < JointAcceleration.Length; j++) JointAcceleration[j] = radPerSecond2;
            return this;
        }

        /// <summary>The ticks the acceleration is measured over (about the 25 ms smoothing people's figures were measured with): its second difference spans twice this.</summary>
        public const int AccelerationTicks = 3;

        /// <summary>
        /// Whether the tick to <paramref name="p0"/> keeps to the joint caps, after <paramref name="history"/> (the poses before it,
        /// newest last; at least <see cref="AccelerationTicks"/> times two), <paramref name="dt"/> apart: its speed and its
        /// acceleration over <see cref="AccelerationTicks"/> ticks (the second difference over as many each side), as people's were
        /// measured smoothed: the commands' own tick to tick jitter is not the arm's motion.
        /// </summary>
        public bool Allows(ReadOnlySpan<ArmPose> history, in ArmPose p0, double dt)
        {
            int n = history.Length, m = AccelerationTicks;
            for (int j = 0; j < DriveEnvelope.JointCount; j++)
            {
                // The speed over the same ticks as the acceleration (one tick before there are that many).
                int back = Math.Min(m, n);
                float q0 = DrivePlan.Joint(p0, j), qb = DrivePlan.Joint(history[n - back], j);
                if (MathF.Abs(q0 - qb) > JointSpeed[j] * (float)(back * dt)) return false;
                if (float.IsPositiveInfinity(JointAcceleration[j]) || n < 2 * m) continue;
                float a = (q0 - 2f * DrivePlan.Joint(history[n - m], j) + DrivePlan.Joint(history[n - 2 * m], j)) / (float)(m * m * dt * dt);
                if (MathF.Abs(a) > JointAcceleration[j]) return false;
            }

            return true;
        }

        /// <summary>Throws when a cap is not positive.</summary>
        public void Validate()
        {
            for (int j = 0; j < DriveEnvelope.JointCount; j++)
            {
                if (!(JointSpeed[j] > 0) || !(JointAcceleration[j] > 0)) throw new ArgumentException("the joint caps must be positive");
            }

            if (!(MaxWristSpeed > 0) || !(MaxPeakSpeed > 0)) throw new ArgumentException("the speed caps must be positive");
        }
    }

    /// <summary>
    /// How a hand moves where the mixed script asks it to move more like the people measured (docs/adr/0038 decision 1, workstream
    /// R's figures: the scripts turned the wrist pitch twice as fast as people at p90, and the elbow 1.4 times): each move's pitch
    /// and roll change scaled by <see cref="PitchScale"/> and <see cref="RollScale"/>, followed over <see cref="PitchTime"/> of each
    /// submovement (the hand model: 0.7), and every submovement stretched by <see cref="Slow"/>. Null on the operator: the hand
    /// model's own (the four safety scripts).
    /// </summary>
    public sealed class HandStyle
    {
        public double PitchScale { get; set; } = 1.0;
        public double RollScale { get; set; } = 1.0;
        public double PitchTime { get; set; } = HandModel.PitchTime;
        public double Slow { get; set; } = 1.0;
    }

    /// <summary>
    /// Where the mixed script's layout-blind goals are drawn (docs/adr/0038 decision 1): the wrist anywhere in reach, not only where
    /// people work, and a share of the goals low with the gripper pointing down, so the hand reaches near the table as often as
    /// an object-directed move does. Heights are the wrist's (the gripper's pads hang 13-15 cm below it pointing down).
    /// </summary>
    public sealed class BlindReach
    {
        public float RadiusLow { get; set; } = 0.08f;
        public float RadiusHigh { get; set; } = 0.25f;
        public float HeightLow { get; set; } = -0.03f;
        public float HeightHigh { get; set; } = 0.26f;
        public float YawDegrees { get; set; } = 100f; // the obstacles stand round the base to +-120 degrees; the envelope's yaw ends at +-105

        /// <summary>The share of goals drawn low: the wrist at most <see cref="LowHeightHigh"/>, the gripper pitched <see cref="LowPitchLowDegrees"/> to <see cref="LowPitchHighDegrees"/>.</summary>
        public double LowShare { get; set; } = 0.4;
        public float LowHeightHigh { get; set; } = 0.06f;
        public double LowPitchLowDegrees { get; set; } = -85;
        public double LowPitchHighDegrees { get; set; } = -45;

        /// <summary>How far inside the arm's reach (from the shoulder) a low goal stays, m; a goal that is not low may be past it by <see cref="HandModel.ReachOvershootMeters"/>.</summary>
        public float LowReachInset { get; set; } = 0.03f;

        /// <summary>The share of goals drawn as free movement draws them, a lognormal distance from where the hand is; the rest anywhere in the region.</summary>
        public double NearShare { get; set; } = 0.5;

        /// <summary>
        /// A near goal's direction from the hand, its part out from the base and its vertical part scaled by these before it is
        /// normalized (1: as free movement draws it): below 1, the hand moves more round the base, as people do (workstream R: the
        /// scripted hand moved the shoulder and elbow 1.2-1.4 times as fast as people at p90, the yaw 0.8 times).
        /// </summary>
        public float RadialScale { get; set; } = 1f;

        /// <summary>See <see cref="RadialScale"/>.</summary>
        public float VerticalScale { get; set; } = 1f;

        /// <summary>
        /// docs/adr/0038 "M, motion" (the 2026-11-23 review: every layout-blind goal low made how low a path goes say whether it hits): when
        /// set, every goal's tool point (the pads' midpoint) is at a height drawn from these bands, base frame z, as triples (low, high,
        /// share), and the wrist is pitched for that height as an aim at an object there is (<see cref="MixedTake.PitchAt"/>); its place across
        /// is drawn as before (near the hand, or anywhere in the region), and <see cref="LowShare"/>, <see cref="HeightLow"/> and
        /// <see cref="HeightHigh"/> are not used. Null: the wrist's height drawn as before.
        /// </summary>
        public float[]? ToolBands { get; set; }

        /// <summary>Throws when <see cref="ToolBands"/> is not triples of low &lt; high and shares at least 0 that add up to more than 0.</summary>
        public void Validate()
        {
            if (ToolBands == null) return;
            if (ToolBands.Length == 0 || ToolBands.Length % 3 != 0) throw new ArgumentException("tool bands are triples: low, high, share");
            float total = 0f;
            for (int i = 0; i < ToolBands.Length; i += 3)
            {
                if (!(ToolBands[i] < ToolBands[i + 1]) || !(ToolBands[i + 2] >= 0) || ToolBands[i] < -0.25f || ToolBands[i + 1] > 0.25f)
                    throw new ArgumentException($"a tool band must be -0.25 <= low < high <= 0.25 m with a share at least 0, not {ToolBands[i]}..{ToolBands[i + 1]} ({ToolBands[i + 2]})");
                total += ToolBands[i + 2];
            }

            if (!(total > 0)) throw new ArgumentException("the tool bands' shares add up to 0");
        }

        /// <summary>A copy without the lowest of its <see cref="ToolBands"/> (or, without bands, with no low goals): the quiet windows' fallback where low goals all come too near an object.</summary>
        public BlindReach WithoutLow()
        {
            var copy = new BlindReach
            {
                RadiusLow = RadiusLow, RadiusHigh = RadiusHigh, HeightLow = HeightLow, HeightHigh = HeightHigh, YawDegrees = YawDegrees, LowShare = 0, NearShare = NearShare,
                RadialScale = RadialScale, VerticalScale = VerticalScale, LowHeightHigh = LowHeightHigh, LowPitchLowDegrees = LowPitchLowDegrees,
                LowPitchHighDegrees = LowPitchHighDegrees, LowReachInset = LowReachInset,
            };
            if (ToolBands != null && ToolBands.Length > 3)
            {
                int lowest = 0;
                for (int i = 3; i < ToolBands.Length; i += 3) if (ToolBands[i] < ToolBands[lowest]) lowest = i;
                var rest = new List<float>();
                for (int i = 0; i < ToolBands.Length; i += 3) if (i != lowest) rest.AddRange(new[] { ToolBands[i], ToolBands[i + 1], ToolBands[i + 2] });
                copy.ToolBands = rest.ToArray();
            }

            return copy;
        }
    }

    /// <summary>One submovement: a minimum-jerk displacement of the hand (and of its pitch and roll), from <see cref="Start"/> over <see cref="Duration"/>.</summary>
    public readonly struct Submovement
    {
        public readonly double Start, Duration;
        public readonly Vector3 Displacement;
        public readonly float Pitch, Roll;
        public readonly float Skew;

        public Submovement(double start, double duration, Vector3 displacement, float pitch, float roll, float skew)
        {
            Start = start;
            Duration = duration;
            Displacement = displacement;
            Pitch = pitch;
            Roll = roll;
            Skew = skew;
        }

        public double End => Start + Duration;
    }

    /// <summary>The clutch let go at <see cref="Start"/> and taken again at <see cref="Engage"/>: the target holds in between.</summary>
    public readonly struct ClutchRelease
    {
        public readonly double Start, Engage;

        public ClutchRelease(double start, double engage)
        {
            Start = start;
            Engage = engage;
        }
    }

    /// <summary>The gripper moving from <see cref="From"/> to <see cref="To"/> (fractions, 0 open), minimum jerk over its duration.</summary>
    public readonly struct GripMove
    {
        public readonly double Start, Duration;
        public readonly float From, To;

        public GripMove(double start, double duration, float from, float to)
        {
            Start = start;
            Duration = duration;
            From = from;
            To = to;
        }

        public double End => Start + Duration;
    }

    /// <summary>What the headset would send at one tick: the clutch's target (arm base frame), the wrist's pitch and roll, the gripper, and whether the clutch is let go.</summary>
    public readonly struct HandCommand
    {
        public readonly Vector3 Target;
        public readonly float Pitch, Roll, Gripper;
        public readonly bool Held;

        public HandCommand(Vector3 target, float pitch, float roll, float gripper, bool held)
        {
            Target = target;
            Pitch = pitch;
            Roll = roll;
            Gripper = gripper;
            Held = held;
        }
    }

    /// <summary>The hand's plan: where it starts, its submovements, the clutch's releases (in time order, never overlapping) and the gripper's moves (likewise).</summary>
    public sealed class HandTimeline
    {
        public HandTimeline(Vector3 start, float pitch, float roll, float gripper)
        {
            Start = start;
            StartPitch = pitch;
            StartRoll = roll;
            StartGripper = gripper;
        }

        public Vector3 Start { get; }
        public float StartPitch { get; }
        public float StartRoll { get; }
        public float StartGripper { get; }
        public List<Submovement> Moves { get; } = new List<Submovement>();
        public List<ClutchRelease> Releases { get; } = new List<ClutchRelease>();
        public List<GripMove> Grips { get; } = new List<GripMove>();

        /// <summary>The share of each submovement over which the pitch and roll follow (<see cref="HandModel.PitchTime"/> unless a <see cref="HandStyle"/> says otherwise).</summary>
        public double PitchTime { get; set; } = HandModel.PitchTime;

        public HandTimeline Copy()
        {
            var c = new HandTimeline(Start, StartPitch, StartRoll, StartGripper) { PitchTime = PitchTime };
            c.Moves.AddRange(Moves);
            c.Releases.AddRange(Releases);
            c.Grips.AddRange(Grips);
            return c;
        }

        /// <summary>When the last submovement ends.</summary>
        public double MotionEnd
        {
            get
            {
                double e = 0;
                foreach (Submovement s in Moves) e = Math.Max(e, s.End);
                return e;
            }
        }

        /// <summary>When everything planned has happened: motion, holds and gripper moves.</summary>
        public double LastEnd
        {
            get
            {
                double e = MotionEnd;
                foreach (ClutchRelease r in Releases) e = Math.Max(e, r.Engage);
                foreach (GripMove g in Grips) e = Math.Max(e, g.End);
                return e;
            }
        }

        /// <summary>When the clutch was last taken again (0 if never let go).</summary>
        public double LastEngage => Releases.Count == 0 ? 0 : Releases[Releases.Count - 1].Engage;

        /// <summary>Where the hand ends once every submovement has run (its pitch and roll too).</summary>
        public void Endpoint(out Vector3 position, out float pitch, out float roll)
        {
            position = Start;
            pitch = StartPitch;
            roll = StartRoll;
            foreach (Submovement s in Moves)
            {
                position += s.Displacement;
                pitch += s.Pitch;
                roll += s.Roll;
            }
        }

        /// <summary>The intended hand at <paramref name="t"/>: position, velocity, pitch and roll.</summary>
        public void Hand(double t, out Vector3 position, out Vector3 velocity, out float pitch, out float roll)
        {
            position = Start;
            velocity = default;
            pitch = StartPitch;
            roll = StartRoll;
            foreach (Submovement s in Moves)
            {
                if (t <= s.Start) continue;
                double u = (t - s.Start) / s.Duration;
                if (u >= 1)
                {
                    position += s.Displacement;
                    pitch += s.Pitch;
                    roll += s.Roll;
                    continue;
                }

                double w = Math.Pow(u, s.Skew);
                position += (float)HandModel.MinimumJerk(w) * s.Displacement;
                velocity += (float)(HandModel.MinimumJerkRate(w) * s.Skew * Math.Pow(u, s.Skew - 1) / s.Duration) * s.Displacement;
                double up = Math.Min(1.0, (t - s.Start) / (s.Duration * PitchTime));
                float a = (float)HandModel.MinimumJerk(up);
                pitch += a * s.Pitch;
                roll += a * s.Roll;
            }
        }

        /// <summary>The gripper fraction at <paramref name="t"/>.</summary>
        public float Gripper(double t)
        {
            float g = StartGripper;
            foreach (GripMove m in Grips)
            {
                if (t <= m.Start) break;
                g = m.From + (m.To - m.From) * (float)HandModel.MinimumJerk((t - m.Start) / m.Duration);
            }

            return g;
        }

        /// <summary>The gripper fraction once every move has run.</summary>
        public float FinalGripper => Grips.Count == 0 ? StartGripper : Grips[Grips.Count - 1].To;

        /// <summary>Whether the clutch is let go at <paramref name="t"/>.</summary>
        public bool Released(double t)
        {
            foreach (ClutchRelease r in Releases)
            {
                if (t >= r.Start && t < r.Engage) return true;
            }

            return false;
        }
    }

    /// <summary>The hand-tracking noise, one value a tick, from its own seeded generator: the same whatever is planned on top of it.</summary>
    internal sealed class HandNoise
    {
        private SeededRng _rng;
        private readonly List<Vector3> _drift = new List<Vector3>(), _jitter = new List<Vector3>();
        private readonly List<float> _pitch = new List<float>();
        private Vector3 _v, _d, _j;
        private float _q;
        private readonly float _av, _sv, _ax, _sj, _ap, _sp;

        public HandNoise(ulong seed)
        {
            _rng = new SeededRng(seed == 0 ? 1 : seed);
            double dt = HandModel.TickSeconds;
            _av = (float)Math.Exp(-dt / HandModel.DriftVelocityTau);
            _sv = (float)(HandModel.DriftVelocitySigma * Math.Sqrt(1 - _av * _av));
            _ax = (float)Math.Exp(-dt / HandModel.DriftPositionTau);
            _sj = (float)Math.Sqrt(1 - HandModel.JitterAr * HandModel.JitterAr);
            _ap = (float)Math.Exp(-dt / HandModel.PitchNoiseTau);
            _sp = (float)(HandModel.PitchNoiseRadians * Math.Sqrt(1 - _ap * _ap));
        }

        private void Extend(int k)
        {
            while (_drift.Count <= k)
            {
                _v = _av * _v + _sv * Draw.Gaussian3(ref _rng);
                _d = _ax * _d + _v * (float)HandModel.TickSeconds;
                _j = (float)HandModel.JitterAr * _j + _sj * Draw.Gaussian3(ref _rng);
                _q = _ap * _q + _sp * (float)Draw.Normal(ref _rng);
                _drift.Add(_d);
                _jitter.Add(_j);
                _pitch.Add(_q);
            }
        }

        public Vector3 Drift(int k)
        {
            Extend(k);
            return _drift[k];
        }

        public Vector3 Jitter(int k)
        {
            Extend(k);
            return _jitter[k];
        }

        public float Pitch(int k)
        {
            Extend(k);
            return _pitch[k];
        }
    }

    /// <summary>What a scripted event does at its end: stop short and hold, touch and pull back, or swipe through and lift away.</summary>
    public enum HandEvent
    {
        Stop,
        Touch,
        Through,
    }

    /// <summary>
    /// An operator's state, to try more than one way on from it (<see cref="HumanOperator.Save"/>): its plan, and every pose and
    /// command solved so far, so it can be gone back to whatever was tried from it, or from another state, since (2026-11-21: it
    /// kept only how many there were, which was right only while nothing went back past it; a take that goes on after a knock
    /// goes back to one of several points of its runway, docs/adr/0036 amendment 5).
    /// </summary>
    public sealed class HandSnapshot
    {
        internal HandSnapshot(HandTimeline timeline, double free, ArmPose[] poses, HandCommand[] commands, SeededRng rng, bool grips, bool rolls)
        {
            Timeline = timeline;
            Free = free;
            Poses = poses;
            Commands = commands;
            Rng = rng;
            Grips = grips;
            Rolls = rolls;
        }

        internal HandTimeline Timeline { get; }
        internal double Free { get; }
        internal ArmPose[] Poses { get; }
        internal HandCommand[] Commands { get; }
        internal int Ticks => Poses.Length;
        internal SeededRng Rng { get; }
        internal bool Grips { get; }
        internal bool Rolls { get; }
    }

    /// <summary>
    /// A person at the headset, as joint-drive's safety scripts drive (docs/adr/0036 amendment 2): the hand's intended path is
    /// a sum of overlapping minimum-jerk submovements (Flash and Hogan), grouped as an operator groups them (moves to a goal
    /// with an endpoint error and corrections, back and forth, small adjustments), with the hand-tracking noise on top; the
    /// clutch maps it onto the wrist's target as <see cref="ClutchMapping"/> does (relative to where both were at engage, kept
    /// in reach and above the floor), holds it while let go, and re-anchors on engage, so nothing jumps. Every tick is solved
    /// by the headset's IK (<see cref="ArmSolver"/>, from the pose before it) and checked against the envelope and a
    /// collision model; a part that fails is drawn again. Deterministic from its seed.
    /// </summary>
    public sealed class HumanOperator
    {
        private const double Tick = HandModel.TickSeconds;
        private readonly RobotArmProfile _profile;
        private readonly DriveEnvelope _envelope;
        private readonly ArmPose _start;
        private readonly HandNoise _noise;
        private readonly List<ArmPose> _poses = new List<ArmPose>();
        private readonly List<HandCommand> _commands = new List<HandCommand>();
        private HandTimeline _timeline;
        private SeededRng _rng;
        private double _free;

        public HumanOperator(in RobotArmProfile profile, in DriveEnvelope envelope, in ArmPose start, ulong seed)
        {
            _profile = profile;
            _envelope = envelope;
            _start = start;
            _rng = new SeededRng(seed == 0 ? 1 : seed);
            _noise = new HandNoise(_rng.NextUInt64());
            float gripper = DrawGripLevel(float.NaN);
            _timeline = new HandTimeline(HandModel.Wrist(profile, start), HandModel.PitchOf(start), start.Roll, gripper);
            _free = Draw.Uniform(ref _rng, 0.15, 0.45);
        }

        /// <summary>Whether the gripper may move (not during an approach to an object, whose contact it would change).</summary>
        public bool Grips { get; set; } = true;

        /// <summary>Whether the roll may wander (not during an approach to an object).</summary>
        public bool Rolls { get; set; } = true;

        /// <summary>The poses solved so far, one a tick from the operator's start.</summary>
        public IReadOnlyList<ArmPose> Poses => _poses;

        /// <summary>The commands those poses were solved from.</summary>
        public IReadOnlyList<HandCommand> Commands => _commands;

        /// <summary>When the next part may start, seconds from the operator's start.</summary>
        public double Free => _free;

        public HandTimeline Timeline => _timeline;

        public HandSnapshot Save() => new HandSnapshot(_timeline.Copy(), _free, _poses.ToArray(), _commands.ToArray(), _rng, Grips, Rolls);

        /// <summary>Back to <paramref name="s"/>; with <paramref name="reseed"/>, every draw from here on comes from that seed instead (the same draws whatever came before).</summary>
        public void Restore(HandSnapshot s, ulong? reseed = null)
        {
            _timeline = s.Timeline.Copy();
            _free = s.Free;
            _poses.Clear();
            _poses.AddRange(s.Poses);
            _commands.Clear();
            _commands.AddRange(s.Commands);
            _rng = reseed.HasValue ? new SeededRng(reseed.Value == 0 ? 1 : reseed.Value) : s.Rng;
            Grips = s.Grips;
            Rolls = s.Rolls;
        }

        /// <summary>
        /// Free movement until <paramref name="end"/> seconds: goals, back and forth, adjustments and pauses, every tick checked
        /// against <paramref name="check"/>. With <paramref name="partStarts"/>, each part's start is added to it; with
        /// <paramref name="snapshots"/>, the operator's state after each part (a point the plan can go on from another way).
        /// </summary>
        public bool FreeUntil(double end, ArmCollisionModel check, List<double>? partStarts = null, List<HandSnapshot>? snapshots = null)
        {
            while (_free < end)
            {
                double start = _free;
                if (!StepFree(check)) return false;
                partStarts?.Add(start);
                snapshots?.Add(Save());
            }

            return true;
        }

        /// <summary>
        /// The operator is already moving when the take begins (docs/adr/0036 amendment 5): its first part started a moment
        /// before (<see cref="HandModel.EngageLeadLow"/>-<see cref="HandModel.EngageLeadHigh"/> s, as a pinch taken on the
        /// move), so the first commands already move the arm, where a person's reaction time (0.15-0.45 s) left it still.
        /// Call it before anything is planned.
        /// </summary>
        public void EngageOnTheMove()
        {
            if (_poses.Count > 0 || _timeline.Moves.Count > 0) throw new InvalidOperationException("the operator has already moved");
            _free = -Draw.Uniform(ref _rng, HandModel.EngageLeadLow, HandModel.EngageLeadHigh);
        }

        /// <summary>
        /// Places free goals are not drawn near (horizontally, arm base frame), within <see cref="AvoidRadius"/>: after a knock,
        /// where the object stood and where it went (docs/adr/0036 amendment 5). Null: anywhere.
        /// </summary>
        public IReadOnlyList<Vector2>? AvoidGoals { get; set; }

        /// <summary>How far from each of <see cref="AvoidGoals"/> a free goal is drawn, m.</summary>
        public float AvoidRadius { get; set; } = 0.10f;

        /// <summary>
        /// What the commands may do tick to tick (docs/adr/0038 decision 1, the mixed script's speed caps): every solved tick within
        /// them, and every submovement planned no faster than <see cref="MotionCaps.MaxPeakSpeed"/>. Null (the four safety scripts):
        /// the hand model's own limits only, every draw as before.
        /// </summary>
        public MotionCaps? Caps { get; set; }

        /// <summary>How the hand moves when the mixed script asks for people's own pace (<see cref="HandStyle"/>); null: the hand model's (every draw as before).</summary>
        public HandStyle? Style
        {
            get => _style;
            set
            {
                _style = value;
                _timeline.PitchTime = value?.PitchTime ?? HandModel.PitchTime;
            }
        }

        private HandStyle? _style;

        /// <summary>
        /// docs/adr/0038 decision 1: a tick before <see cref="GuardBefore"/> or from <see cref="GuardFrom"/> on (seconds on the hand's
        /// timeline) must also pass this model, the take's quiet start and end, whatever its part is checked against otherwise.
        /// Null: no guard (the four safety scripts).
        /// </summary>
        public ArmCollisionModel? Guard { get; set; }

        /// <summary>See <see cref="Guard"/>.</summary>
        public double GuardBefore { get; set; } = double.NegativeInfinity;

        /// <summary>See <see cref="Guard"/>.</summary>
        public double GuardFrom { get; set; } = double.PositiveInfinity;

        /// <summary>Where the target comes to rest once everything planned so far has run, arm base frame.</summary>
        public Vector3 TargetEnd => TargetEndpoint(_timeline);

        /// <summary>The gripper fraction once everything planned so far has run.</summary>
        public float GripperEnd => _timeline.FinalGripper;

        /// <summary>How many ticks refused a part, by what: the IK (blocked, out of reach), the envelope or the check, a jump, the guard, the caps, a wall. For reports only.</summary>
        public int[] Refusals { get; } = new int[6];

        /// <summary>
        /// docs/adr/0038 (the twin check of 2026-11-23): while a layout-blind part is planned, its ticks from <see cref="WallsFrom"/>
        /// on (seconds on the hand's timeline) must also pass this model, the fixed objects as walls: a person stops at a wall, where
        /// a blind move pushed the arm into a shelf for seconds. Null: none.
        /// </summary>
        public ArmCollisionModel? Walls { get; set; }

        /// <summary>See <see cref="Walls"/>.</summary>
        public double WallsFrom { get; set; } = double.PositiveInfinity;

        /// <summary>
        /// A move to <paramref name="goal"/> (the wrist's target, arm base frame) with the wrist at <paramref name="pitch"/> and
        /// <paramref name="roll"/>, ending within <paramref name="tolerance"/>, every tick checked against <paramref name="check"/>
        /// (redrawn on failure). With <paramref name="overTheTop"/>, every other try goes by a point above the goal first. With
        /// <paramref name="pauseAfter"/>, the operator then carries on, keeps still, or lets go (never mid-move: the goal must be
        /// reached).
        /// </summary>
        public bool MoveTo(Vector3 goal, float pitch, float? roll, ArmCollisionModel check, double tolerance, bool overTheTop = false, bool pauseAfter = false,
            int attempts = 12, float? gripperGoal = null)
        {
            for (int attempt = 0; attempt < attempts; attempt++)
            {
                HandTimeline tl = _timeline.Copy();
                double t0 = _free, next;
                int moves = tl.Moves.Count;
                if (overTheTop && attempt % 2 == 1)
                {
                    Vector3 above = goal + new Vector3(0, 0, (float)Draw.Uniform(ref _rng, 0.04, 0.08));
                    above.Z = MathF.Min(above.Z, 0.20f);
                    double mid = Transport(tl, above, t0, tolerance: 0.012, pitchGoal: pitch, rollGoal: roll, maxCorrections: 1);
                    next = Transport(tl, goal, mid, tolerance: tolerance, pitchGoal: pitch, rollGoal: roll);
                }
                else
                {
                    next = Transport(tl, goal, t0, tolerance: tolerance, pitchGoal: pitch, rollGoal: roll);
                }

                if (gripperGoal.HasValue && MathF.Abs(tl.FinalGripper - gripperGoal.Value) > 1e-3f)
                {
                    // Set early in the move, and done before it ends (an approach is planned for the gripper as it will be).
                    double busy = tl.Grips.Count == 0 ? t0 : Math.Max(t0, tl.Grips[tl.Grips.Count - 1].End);
                    double duration = Draw.Uniform(ref _rng, HandModel.GripDurationLow, HandModel.GripDurationHigh);
                    double start = Math.Min(busy + Draw.Uniform(ref _rng, 0.0, 0.3), Math.Max(busy, tl.MotionEnd - duration));
                    tl.Grips.Add(new GripMove(start, duration, tl.FinalGripper, gripperGoal.Value));
                }

                if (pauseAfter)
                {
                    double u = _rng.NextDouble();
                    next = Pause(tl, next, u < 0.85 ? PauseKind.Continue : PauseKind.Dwell, cutOff: true, firstMove: moves); // continuous (amendment 3)
                }

                if (Accept(tl, t0, check))
                {
                    _free = next;
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// The end of a scripted episode from where the operator is (staged in front of an object): the final approach toward
        /// <paramref name="aim"/> (stop short and hold, touch and pull back, or swipe through and lift away), then a move back to
        /// <paramref name="home"/>. Every tick is solved and checked against the robot's own model and the envelope only; what
        /// it does to the objects is the caller's to judge (<paramref name="finalStart"/> and <paramref name="eventEnd"/>, seconds,
        /// bound the event).
        /// </summary>
        public bool Event(HandEvent kind, IReadOnlyList<Vector3> path, float pitch, Vector3 home, float homePitch, out double finalStart, out double eventEnd)
        {
            finalStart = _free;
            eventEnd = _free;
            if (path.Count < 2) return false;
            HandTimeline tl = _timeline.Copy();
            Vector3 way = path[path.Count - 1] - path[path.Count - 2];
            Vector3 u = way.Length() > 1e-5f ? Vector3.Normalize(way) : Vector3.UnitX;
            bool rolls = Rolls;
            Rolls = false; // the approach keeps the roll it has
            double next;
            if (kind == HandEvent.Through)
            {
                // A fast swipe along the way and past the object, overshooting as fast moves do; then lifted away.
                next = Along(tl, path, _free, slow: 0.6, maxPeak: 0.6, alongMu: 0.04, errorScale: 0.6, pitchGoal: pitch, rollGoal: null,
                    tolerance: 0.012, maxCorrections: 0);
                Vector3 end = TargetEndpoint(tl);
                Vector3 lift = end + new Vector3(0, 0, (float)Draw.Uniform(ref _rng, 0.05, 0.08)) - u * (float)Draw.Uniform(ref _rng, 0.0, 0.03);
                Rolls = rolls;
                next = Transport(tl, lift, next, tolerance: 0.008);
                eventEnd = tl.MotionEnd;
            }
            else
            {
                // Slowing into the last few centimetres along the way, short of the goal, then small corrections the rest of it
                // (for a touch, a gentle push), as a hand placing something does.
                bool stop = kind == HandEvent.Stop;
                next = Along(tl, path, _free, slow: stop ? 1.3 : 1.4, maxPeak: null, alongMu: stop ? -0.12 : -0.25, errorScale: 0.5, pitchGoal: pitch,
                    rollGoal: null, tolerance: 0.0015, maxCorrections: 3);
                if (kind == HandEvent.Stop)
                {
                    next = tl.MotionEnd + Draw.Uniform(ref _rng, 0.3, 0.5); // stopped short: a brief stillness, then away (amendment 3)
                }
                else
                {
                    next = tl.MotionEnd + Draw.Uniform(ref _rng, 0.10, 0.35); // a moment in contact, then pulled back
                }

                eventEnd = next;
                Vector3 end = TargetEndpoint(tl);
                Vector3 back = end - u * (float)Draw.Uniform(ref _rng, 0.04, 0.08) + new Vector3(0, 0, (float)Draw.Uniform(ref _rng, 0.02, 0.05));
                Rolls = rolls;
                next = Transport(tl, back, next, slow: kind == HandEvent.Touch ? 0.85 : 1.0, tolerance: 0.008);
            }

            // Then on, not home (amendment 3): a free move toward a point in reach, during which the take ends.
            double clear = tl.MotionEnd;
            Vector3 onward = TargetEndpoint(tl) + new Vector3((float)Draw.Uniform(ref _rng, -0.06, 0.02), (float)Draw.Uniform(ref _rng, -0.10, 0.10),
                (float)Draw.Uniform(ref _rng, 0.03, 0.08));
            double onwardStart = next;
            next = Transport(tl, onward, next, tolerance: 0.008);
            double onwardEnd = tl.MotionEnd;
            if (Grips) MaybeGrip(tl, clear, tl.MotionEnd); // back to using the gripper once clear of the object
            if (!Accept(tl, finalStart, _profile.Collision!, 0.3)) return false;
            _free = next;
            OnwardStart = onwardStart;
            OnwardEnd = onwardEnd;
            return true;
        }

        /// <summary>The last <see cref="Event"/>'s move onward, after the event, seconds on the hand's timeline: where the take may end (docs/adr/0036 amendment 3).</summary>
        public double OnwardStart { get; private set; } = double.NaN;

        /// <summary>See <see cref="OnwardStart"/>.</summary>
        public double OnwardEnd { get; private set; } = double.NaN;

        /// <summary>
        /// Appends the poses solved so far to <paramref name="legs"/> from <paramref name="t"/>, one linear leg a tick (a run of
        /// the same pose and gripper, the clutch let go, as one hold), carrying the gripper; moves <paramref name="current"/>
        /// and <paramref name="t"/> to the end.
        /// </summary>
        public void AppendLegs(List<DriveLeg> legs, ref double t, ref ArmPose current)
        {
            int end = Math.Min(_poses.Count, (int)Math.Ceiling((_timeline.LastEnd + 0.2) / Tick) + 1);
            ArmPose previous = current;
            bool holding = false;
            for (int k = 0; k < end; k++)
            {
                ArmPose p = _poses[k];
                float g = _commands[k].Gripper;
                bool still = k > 0 && Same(p, previous) && g == _commands[k - 1].Gripper;
                if (still && holding)
                {
                    DriveLeg h = legs[legs.Count - 1];
                    legs[legs.Count - 1] = new DriveLeg(h.Start, 0, h.Hold + Tick, h.From, h.To, h.Phase, h.Speed, h.GripperFrom, h.GripperTo);
                }
                else if (still)
                {
                    legs.Add(new DriveLeg(t, 0, Tick, p, p, LegPhase.Script, float.NaN, g, g));
                }
                else
                {
                    legs.Add(new DriveLeg(t, Tick, 0, previous, p, LegPhase.Script, float.NaN, g, g, linear: true));
                }

                holding = still;
                t += Tick;
                previous = p;
            }

            current = previous;
        }

        /// <summary>Whether two poses are the same, joint for joint, to the bit.</summary>
        public static bool Same(in ArmPose a, in ArmPose b) =>
            a.BaseYaw == b.BaseYaw && a.Proximal == b.Proximal && a.Distal == b.Distal && a.Wrist == b.Wrist && a.Roll == b.Roll;

        // ---------------------------------------------------------------------------------------------------------------
        // The mixed script's moves (docs/adr/0038 decision 1): each one part of the hand's plan, made of the same overlapping
        // submovements, corrections and jitter as free movement, at a slowness the caller draws for every move alike. Each says
        // when it began and when its motion ends, seconds on the hand's timeline; the next part begins before that, as a hand's do.

        /// <summary>
        /// A layout-blind part at slowness <paramref name="slow"/>: a move to a goal drawn anywhere in <paramref name="reach"/> (with
        /// <paramref name="transportShare"/>), back-and-forth strokes (<paramref name="reverseShare"/>), or small adjustments, every
        /// tick checked against <paramref name="check"/> (and the guard). Its kind: <see cref="Teleop.RobotArm.Wire.ScriptMoveKind.Free"/>
        /// or <see cref="Teleop.RobotArm.Wire.ScriptMoveKind.Reverse"/>.
        /// </summary>
        public bool MixedFree(ArmCollisionModel check, BlindReach reach, double slow, double transportShare, double reverseShare,
            out Teleop.RobotArm.Wire.ScriptMoveKind kind, out double start, out double end, ArmCollisionModel? walls = null)
        {
            Walls = walls;
            try
            {
                return MixedFreeParts(check, reach, slow, transportShare, reverseShare, out kind, out start, out end);
            }
            finally
            {
                Walls = null;
                WallsFrom = double.PositiveInfinity;
            }
        }

        private bool MixedFreeParts(ArmCollisionModel check, BlindReach reach, double slow, double transportShare, double reverseShare,
            out Teleop.RobotArm.Wire.ScriptMoveKind kind, out double start, out double end)
        {
            kind = Teleop.RobotArm.Wire.ScriptMoveKind.Free;
            start = end = _free;
            double stretch = 1.0, u = _rng.NextDouble();
            for (int attempt = 0; attempt < 30; attempt++)
            {
                // The kind of part drawn is kept for most of the tries (a goal out of reach draws another goal), so the parts come in the
                // shares asked for (docs/adr/0038 "M, motion": when a failed goal drew the kind again, small adjustments, which seldom
                // fail, were 46% of the parts that are not strokes, not 27%); a cornered hand then tries any kind.
                if (attempt >= KeepPartTries && stretch == 1.0) u = _rng.NextDouble();
                SeededRng drawn = _rng;
                int capped = Refusals[4];
                HandTimeline tl = _timeline.Copy();
                double t0 = _free, next;
                WallsFrom = t0 + 0.3; // where the last move left the hand, against a wall, it has a moment to leave
                slow *= stretch;
                Teleop.RobotArm.Wire.ScriptMoveKind k = Teleop.RobotArm.Wire.ScriptMoveKind.Free;
                if (u < transportShare)
                {
                    if (!BlindGoal(tl, reach, out Vector3 goal, out float? pitch)) continue;
                    next = Transport(tl, goal, t0, slow: slow, pitchGoal: pitch);
                }
                else if (u < transportShare + reverseShare)
                {
                    next = Reciprocate(tl, t0, slow);
                    k = Teleop.RobotArm.Wire.ScriptMoveKind.Reverse;
                }
                else
                {
                    next = Adjust(tl, t0, slow);
                }

                double moving = tl.MotionEnd;
                if (Grips) MaybeGrip(tl, t0, Math.Max(t0 + 0.3, moving));
                slow /= stretch;
                if (!Accept(tl, t0, check))
                {
                    stretch = Stretch(drawn, capped, stretch);
                    continue;
                }

                _free = next;
                kind = k;
                LastFreePart = k == Teleop.RobotArm.Wire.ScriptMoveKind.Reverse ? MovePart.BlindStrokes : u < transportShare ? MovePart.BlindTransport : MovePart.BlindAdjust;
                start = t0;
                end = moving;
                return true;
            }

            return false;
        }

        /// <summary>How many of a layout-blind part's tries keep the kind of part first drawn.</summary>
        public const int KeepPartTries = 20;

        /// <summary>What the last <see cref="MixedFree"/> made: a move to a goal, strokes or adjustments. For reports only.</summary>
        public MovePart LastFreePart { get; private set; }

        /// <summary>
        /// After a part was refused: when the caps refused it (and it is not yet 2.5 times as slow), the same draws again a third
        /// slower, so the caps stretch a fast move instead of trading it for a slow one; otherwise new draws at the drawn pace.
        /// The factor for the next try.
        /// </summary>
        private double Stretch(in SeededRng drawn, int cappedBefore, double stretch)
        {
            if (Refusals[4] > cappedBefore && stretch < 2.5)
            {
                _rng = drawn;
                return stretch * 1.3;
            }

            return 1.0;
        }

        /// <summary>
        /// A move to <paramref name="goal"/> (the wrist's target) at slowness <paramref name="slow"/>, the wrist turning to
        /// <paramref name="pitch"/> and <paramref name="roll"/> when given, ending within <paramref name="tolerance"/>; every tick checked
        /// against <paramref name="check"/> (and the guard), redrawn on failure. The gripper is used on the way as in free movement.
        /// </summary>
        public bool MixedTo(Vector3 goal, float? pitch, float? roll, double slow, double tolerance, ArmCollisionModel check, out double start, out double end,
            int attempts = 12, bool overTheTop = false, ArmCollisionModel? walls = null)
        {
            Walls = walls;
            WallsFrom = _free + 0.3;
            try
            {
                return MixedToParts(goal, pitch, roll, slow, tolerance, check, out start, out end, attempts, overTheTop);
            }
            finally
            {
                Walls = null;
                WallsFrom = double.PositiveInfinity;
            }
        }

        private bool MixedToParts(Vector3 goal, float? pitch, float? roll, double slow, double tolerance, ArmCollisionModel check, out double start, out double end,
            int attempts, bool overTheTop)
        {
            start = end = _free;
            double stretch = 1.0;
            for (int attempt = 0; attempt < attempts; attempt++)
            {
                SeededRng drawn = _rng;
                int capped = Refusals[4];
                HandTimeline tl = _timeline.Copy();
                double t0 = _free, next;
                slow *= stretch;
                if (overTheTop && attempt % 2 == 1)
                {
                    // By a point above the goal first, as a hand reaching round something does.
                    Vector3 above = goal + new Vector3(0, 0, (float)Draw.Uniform(ref _rng, 0.04, 0.08));
                    above.Z = MathF.Min(above.Z, 0.20f);
                    double mid = Transport(tl, above, t0, slow: slow, tolerance: 0.012, pitchGoal: pitch, rollGoal: roll, maxCorrections: 1);
                    next = Transport(tl, goal, mid, slow: slow, tolerance: tolerance, pitchGoal: pitch, rollGoal: roll);
                }
                else
                {
                    next = Transport(tl, goal, t0, slow: slow, tolerance: tolerance, pitchGoal: pitch, rollGoal: roll);
                }

                double moving = tl.MotionEnd;
                if (Grips) MaybeGrip(tl, t0, Math.Max(t0 + 0.3, moving));
                slow /= stretch;
                if (!Accept(tl, t0, check))
                {
                    stretch = Stretch(drawn, capped, stretch);
                    continue;
                }

                _free = next;
                start = t0;
                end = moving;
                return true;
            }

            return false;
        }

        /// <summary>The hand kept still, the clutch held, for <paramref name="seconds"/> once its motion has ended: a pause, from <paramref name="start"/> to <paramref name="end"/>.</summary>
        public void MixedPause(double seconds, out double start, out double end)
        {
            start = Math.Max(_free, _timeline.MotionEnd);
            end = start + Math.Max(0.0, seconds);
            _free = end;
        }

        /// <summary>A goal for a layout-blind move (<see cref="BlindReach"/>), the wrist's pitch for it when it is a low one; false when none of the draws is in reach.</summary>
        private bool BlindGoal(HandTimeline tl, BlindReach reach, out Vector3 goal, out float? pitch)
        {
            if (reach.ToolBands != null) return BandGoal(tl, reach, out goal, out pitch);
            Vector3 from = TargetEndpoint(tl);
            float maxReach = _profile.ProximalLinkLength + _profile.DistalLinkLength;
            var shoulder = new Vector3(0, 0, _profile.BaseHeight);
            const double deg = Math.PI / 180.0;
            for (int i = 0; i < 40; i++)
            {
                bool low = Draw.Chance(ref _rng, reach.LowShare);
                float zLow = reach.HeightLow, zHigh = low ? reach.LowHeightHigh : reach.HeightHigh;
                Vector3 g;
                if (Draw.Chance(ref _rng, reach.NearShare))
                {
                    float a = Draw.ReachDistance(ref _rng);
                    Vector3 direction = Draw.Direction(ref _rng, 0.7f);
                    if (reach.RadialScale != 1f || reach.VerticalScale != 1f)
                    {
                        var radial = new Vector2(from.X, from.Y);
                        radial = radial.Length() > 1e-4f ? Vector2.Normalize(radial) : Vector2.UnitX;
                        var r3 = new Vector3(radial.X, radial.Y, 0f);
                        var t3 = new Vector3(-radial.Y, radial.X, 0f);
                        Vector3 scaled = Vector3.Dot(direction, r3) * reach.RadialScale * r3 + Vector3.Dot(direction, t3) * t3 + direction.Z * reach.VerticalScale * Vector3.UnitZ;
                        if (scaled.Length() > 1e-4f) direction = Vector3.Normalize(scaled);
                    }

                    g = from + a * direction;
                    if (low) g.Z = (float)Draw.Uniform(ref _rng, zLow, zHigh);
                }
                else
                {
                    double r = Draw.Uniform(ref _rng, reach.RadiusLow, reach.RadiusHigh), az = Draw.Uniform(ref _rng, -reach.YawDegrees, reach.YawDegrees) * deg;
                    g = new Vector3((float)(r * Math.Cos(az)), (float)(r * Math.Sin(az)), (float)Draw.Uniform(ref _rng, zLow, zHigh));
                }

                float radius = MathF.Sqrt(g.X * g.X + g.Y * g.Y), yaw = MathF.Atan2(g.Y, g.X) * 180f / MathF.PI;
                if (radius < reach.RadiusLow || radius > reach.RadiusHigh || g.Z < zLow || g.Z > zHigh || MathF.Abs(yaw) > reach.YawDegrees) continue;
                // A low goal stays inside the reach (docs/adr/0038 "M, motion": low goals past it left the arm stretched out near the table,
                // where every move broke the speed caps); others may push past it, as the clutch lets a hand.
                if (Vector3.Distance(g, shoulder) > maxReach + (low ? -reach.LowReachInset : HandModel.ReachOvershootMeters)) continue;
                if (AvoidGoals != null && Near(new Vector2(g.X, g.Y), AvoidGoals, AvoidRadius)) continue;
                goal = g;
                pitch = low ? (float)(Draw.Uniform(ref _rng, reach.LowPitchLowDegrees, reach.LowPitchHighDegrees) * deg) : (float?)null;
                return true;
            }

            goal = from;
            pitch = null;
            return false;
        }

        /// <summary>
        /// A layout-blind goal by its tool point's height (<see cref="BlindReach.ToolBands"/>, docs/adr/0038 "M, motion"): the height drawn from
        /// the bands, the wrist pitched for it as an aim there is (<see cref="MixedTake.PitchAt"/>, the same jitter), its place across drawn as
        /// <see cref="BlindGoal"/> draws one, and kept inside the arm's reach by <see cref="BlindReach.LowReachInset"/>. False when none of the
        /// draws is in reach.
        /// </summary>
        private bool BandGoal(HandTimeline tl, BlindReach reach, out Vector3 goal, out float? pitch)
        {
            Vector3 from = TargetEndpoint(tl);
            float maxReach = _profile.ProximalLinkLength + _profile.DistalLinkLength, table = _profile.Collision?.FloorZ ?? -0.1962f;
            var shoulder = new Vector3(0, 0, _profile.BaseHeight);
            Vector3 tool = MixedTake.ToolInGripper;
            const double deg = Math.PI / 180.0;
            float[] bands = reach.ToolBands!;
            for (int i = 0; i < 40; i++)
            {
                float z = Draw.Band(ref _rng, bands);
                Vector3 g;
                if (Draw.Chance(ref _rng, reach.NearShare))
                {
                    float a = Draw.ReachDistance(ref _rng);
                    Vector3 direction = Draw.Direction(ref _rng, 0.7f);
                    var radial = new Vector2(from.X, from.Y);
                    radial = radial.Length() > 1e-4f ? Vector2.Normalize(radial) : Vector2.UnitX;
                    var r3 = new Vector3(radial.X, radial.Y, 0f);
                    var t3 = new Vector3(-radial.Y, radial.X, 0f);
                    Vector3 scaled = Vector3.Dot(direction, r3) * reach.RadialScale * r3 + Vector3.Dot(direction, t3) * t3 + direction.Z * reach.VerticalScale * Vector3.UnitZ;
                    if (scaled.Length() > 1e-4f) direction = Vector3.Normalize(scaled);
                    g = from + a * direction;
                }
                else
                {
                    double r = Draw.Uniform(ref _rng, reach.RadiusLow, reach.RadiusHigh), az = Draw.Uniform(ref _rng, -reach.YawDegrees, reach.YawDegrees) * deg;
                    g = new Vector3((float)(r * Math.Cos(az)), (float)(r * Math.Sin(az)), 0f);
                }

                float radius = MathF.Sqrt(g.X * g.X + g.Y * g.Y), yaw = MathF.Atan2(g.Y, g.X) * 180f / MathF.PI;
                if (radius < reach.RadiusLow || radius > reach.RadiusHigh || MathF.Abs(yaw) > reach.YawDegrees) continue;
                // The pads reach out from the wrist by their pitch's share of 15 cm: the pitch is the tool's radius's, so found in two steps.
                float p = MixedTake.PitchAt(radius + 0.10f, z, table);
                p = MixedTake.PitchAt(radius + tool.X * MathF.Cos(p) - tool.Z * MathF.Sin(p), z, table) + (float)(Draw.Uniform(ref _rng, -8, 8) * deg);
                g.Z = z - (tool.X * MathF.Sin(p) + tool.Z * MathF.Cos(p));
                if (g.Z < -0.06f || g.Z > 0.28f) continue;
                if (Vector3.Distance(g, shoulder) > maxReach - reach.LowReachInset) continue;
                if (AvoidGoals != null && Near(new Vector2(g.X, g.Y), AvoidGoals, AvoidRadius)) continue;
                goal = g;
                pitch = p;
                return true;
            }

            goal = from;
            pitch = null;
            return false;
        }

        // ---------------------------------------------------------------------------------------------------------------
        // Checking: render the commands, solve them, accept.

        private bool StepFree(ArmCollisionModel check)
        {
            for (int attempt = 0; attempt < 30; attempt++)
            {
                HandTimeline tl = _timeline.Copy();
                double t0 = _free;
                int moves = tl.Moves.Count;
                double u = _rng.NextDouble(), next;
                if (u < HandModel.TransportProbability)
                {
                    if (!SampleGoal(tl, out Vector3 goal)) continue;
                    next = Transport(tl, goal, t0);
                }
                else if (u < HandModel.TransportProbability + HandModel.ReciprocateProbability)
                {
                    next = Reciprocate(tl, t0);
                }
                else
                {
                    next = Adjust(tl, t0);
                }

                if (tl.Moves.Count > moves && Draw.Chance(ref _rng, HandModel.MidReleaseProbability))
                {
                    // The pinch opens for a moment while the hand moves: the target freezes, and the hand's motion meanwhile is lost.
                    double a = double.MaxValue, b = 0;
                    for (int i = moves; i < tl.Moves.Count; i++)
                    {
                        a = Math.Min(a, tl.Moves[i].Start);
                        b = Math.Max(b, tl.Moves[i].End);
                    }

                    double r = Draw.Uniform(ref _rng, a + 0.1 * (b - a), a + 0.8 * (b - a));
                    AddRelease(tl, r, r + Draw.Uniform(ref _rng, HandModel.MidReleaseLow, HandModel.MidReleaseHigh));
                }

                double moving = tl.MotionEnd;
                next = Pause(tl, next, null, cutOff: true, firstMove: moves);
                if (Grips) MaybeGrip(tl, t0, Math.Max(t0 + 0.3, moving));
                if (Accept(tl, t0, check))
                {
                    _free = next;
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Renders <paramref name="tl"/> from the tick before <paramref name="from"/> to its end (and <paramref name="tail"/>
        /// seconds more), solves each tick from the pose accepted before it and checks it; on success the timeline and its poses
        /// replace what was there from that tick on.
        /// </summary>
        private bool Accept(HandTimeline tl, double from, ArmCollisionModel check, double tail = 0.4)
        {
            int k0 = Math.Min(_poses.Count, Math.Max(0, (int)Math.Floor(from / Tick)));
            int k1 = (int)Math.Ceiling((tl.LastEnd + tail) / Tick);
            if (k1 < k0) k1 = k0;
            var commands = new List<HandCommand>(k1 - k0 + 1);
            Render(tl, k0, k1, commands);
            var poses = new List<ArmPose>(commands.Count);
            if (!Follow(k0 == 0 ? _start : _poses[k0 - 1], commands, check, poses, k0)) return false;
            _poses.RemoveRange(k0, _poses.Count - k0);
            _commands.RemoveRange(k0, _commands.Count - k0);
            _poses.AddRange(poses);
            _commands.AddRange(commands);
            _timeline = tl;
            return true;
        }

        /// <summary>
        /// The headset's IK along <paramref name="commands"/> (tick <paramref name="firstTick"/> on), one solve a tick from
        /// <paramref name="from"/> (<paramref name="beforeFrom"/> the tick before it): false when a tick is blocked, out of reach, a
        /// jump, outside the envelope or fails <paramref name="check"/>, or, when they are set, the <see cref="Guard"/> in its windows
        /// or the <see cref="Caps"/>.
        /// </summary>
        private bool Follow(ArmPose from, List<HandCommand> commands, ArmCollisionModel check, List<ArmPose> poses, int firstTick)
        {
            var solver = new ArmSolver();
            solver.Reset(_profile, from);
            Vector3 last = HandModel.Wrist(_profile, from);
            float step = HandModel.MaxWristSpeed * (float)Tick;
            MotionCaps? caps = Caps;
            ArmCollisionModel? guard = Guard, walls = Walls;
            const int kept = 2 * MotionCaps.AccelerationTicks;
            Span<ArmPose> history = stackalloc ArmPose[kept]; // the ticks before this one, oldest first (the start pose before the first)
            if (caps != null)
            {
                step = MathF.Min(step, caps.MaxWristSpeed * (float)Tick);
                for (int i = 0; i < kept; i++)
                {
                    int k = firstTick - kept + i;
                    history[i] = k < 0 ? _start : k < _poses.Count ? _poses[k] : from;
                }
            }

            int tick = firstTick;
            foreach (HandCommand c in commands)
            {
                ArmSolution s = solver.Solve(_profile, c.Target, c.Pitch, c.Roll);
                if (s.Status == ArmSolveStatus.Blocked || s.Status == ArmSolveStatus.Approaching || s.ReachClamped) return Refuse(0);
                if (!_envelope.Contains(s.Pose) || !ArmCollision.IsValid(_profile, check, s.Pose)) return Refuse(1);
                if (Vector3.Distance(s.Position, last) > step) return Refuse(2);
                if (guard != null && (tick * Tick < GuardBefore || tick * Tick >= GuardFrom) && !ArmCollision.IsValid(_profile, guard, s.Pose)) return Refuse(3);
                if (walls != null && tick * Tick >= WallsFrom && !ArmCollision.IsValid(_profile, walls, s.Pose)) return Refuse(5);
                if (caps != null)
                {
                    if (!caps.Allows(history, s.Pose, Tick)) return Refuse(4);
                    for (int i = 0; i < kept - 1; i++) history[i] = history[i + 1];
                    history[kept - 1] = s.Pose;
                }

                last = s.Position;
                poses.Add(s.Pose);
                tick++;
            }

            return true;
        }

        private bool Refuse(int why)
        {
            Refusals[why]++;
            return false;
        }

        // The clutch's pieces: engaged (the target follows the hand relative to where both were at the piece's first tick) or
        // held (the target stays where it was).
        private readonly struct Piece
        {
            public readonly int From;
            public readonly bool Held;
            public readonly Vector3 Anchor, RefHand;
            public readonly float AnchorPitch, AnchorRoll, RefPitch, RefRoll;

            public Piece(int from, bool held, Vector3 anchor, float anchorPitch, float anchorRoll, Vector3 refHand, float refPitch, float refRoll)
            {
                From = from;
                Held = held;
                Anchor = anchor;
                AnchorPitch = anchorPitch;
                AnchorRoll = anchorRoll;
                RefHand = refHand;
                RefPitch = refPitch;
                RefRoll = refRoll;
            }
        }

        /// <summary>The hand as tracked at tick <paramref name="k"/>: the intended path, its drift and its speed-sized jitter.</summary>
        private void Raw(HandTimeline tl, int k, out Vector3 position, out float pitch, out float roll)
        {
            tl.Hand(k * Tick, out Vector3 h, out Vector3 v, out float hp, out float hr);
            double sigma = HandModel.JitterFloorMeters + HandModel.JitterGain * Math.Pow(v.Length(), HandModel.JitterExponent);
            position = h + _noise.Drift(k) + (float)sigma * _noise.Jitter(k);
            pitch = hp + _noise.Pitch(k);
            roll = hr;
        }

        private List<Piece> Pieces(HandTimeline tl)
        {
            var pieces = new List<Piece>(1 + 2 * tl.Releases.Count);
            Raw(tl, 0, out Vector3 h0, out float p0, out float r0);
            Vector3 anchor = ArmKinematics.ProjectToWorkspace(_profile, tl.Start, HandModel.MinHeightMeters);
            pieces.Add(new Piece(0, false, anchor, tl.StartPitch, tl.StartRoll, h0, p0, r0));
            foreach (ClutchRelease r in tl.Releases)
            {
                int kr = Math.Max(1, (int)Math.Ceiling(r.Start / Tick - 1e-9));
                int ke = Math.Max(kr + 1, (int)Math.Ceiling(r.Engage / Tick - 1e-9));
                Piece last = pieces[pieces.Count - 1];
                if (kr <= last.From) continue;
                Command(tl, last, kr - 1, out Vector3 held, out float hp, out float hr);
                pieces.Add(new Piece(kr, true, held, hp, hr, default, 0f, 0f));
                Raw(tl, ke, out Vector3 he, out float pe, out float re);
                pieces.Add(new Piece(ke, false, held, hp, hr, he, pe, re));
            }

            return pieces;
        }

        private void Command(HandTimeline tl, in Piece piece, int k, out Vector3 target, out float pitch, out float roll)
        {
            if (piece.Held)
            {
                target = piece.Anchor;
                pitch = piece.AnchorPitch;
                roll = piece.AnchorRoll;
                return;
            }

            Raw(tl, k, out Vector3 h, out float hp, out float hr);
            target = ArmKinematics.ProjectToWorkspace(_profile, piece.Anchor + (h - piece.RefHand), HandModel.MinHeightMeters);
            pitch = piece.AnchorPitch + (hp - piece.RefPitch);
            roll = piece.AnchorRoll + (hr - piece.RefRoll);
        }

        private void Render(HandTimeline tl, int k0, int k1, List<HandCommand> output)
        {
            List<Piece> pieces = Pieces(tl);
            int pi = 0;
            for (int k = k0; k <= k1; k++)
            {
                while (pi + 1 < pieces.Count && pieces[pi + 1].From <= k) pi++;
                Command(tl, pieces[pi], k, out Vector3 target, out float pitch, out float roll);
                output.Add(new HandCommand(target, pitch, roll, tl.Gripper(k * Tick), pieces[pi].Held));
            }
        }

        /// <summary>Where the target comes to rest once everything planned has run: the clutch's last anchor plus the hand's motion since.</summary>
        private Vector3 TargetEndpoint(HandTimeline tl)
        {
            List<Piece> pieces = Pieces(tl);
            Piece last = pieces[pieces.Count - 1];
            tl.Endpoint(out Vector3 e, out _, out _);
            return ArmKinematics.ProjectToWorkspace(_profile, last.Anchor + (e - last.RefHand), HandModel.MinHeightMeters);
        }

        private void TargetAngles(HandTimeline tl, out float pitch, out float roll)
        {
            List<Piece> pieces = Pieces(tl);
            Piece last = pieces[pieces.Count - 1];
            tl.Endpoint(out _, out float ep, out float er);
            pitch = last.AnchorPitch + (ep - last.RefPitch);
            roll = last.AnchorRoll + (er - last.RefRoll);
        }

        // ---------------------------------------------------------------------------------------------------------------
        // The operator's moves. Each appends submovements (and releases) to a copy of the timeline and returns when the next
        // may start.

        private enum PauseKind
        {
            Continue,
            Dwell,
            Release,
        }

        private Submovement Move(double start, double duration, Vector3 displacement, float pitch, float roll) =>
            new Submovement(start, duration, displacement, pitch, roll, (float)Draw.Uniform(ref _rng, HandModel.SkewLow, HandModel.SkewHigh));

        private double Duration(double amplitude, double slow = 1.0, double? maxPeak = null)
        {
            if (_style != null) slow *= _style.Slow;
            double d = HandModel.DurationScale * Math.Pow(Math.Max(amplitude, 1e-4), HandModel.DurationExponent)
                       * Math.Exp(HandModel.DurationSigma * Draw.Normal(ref _rng)) * slow;
            d = Math.Clamp(d, HandModel.MinDuration, HandModel.MaxDuration * Math.Max(1.0, slow));
            double peak = Math.Min(maxPeak ?? Caps?.MaxPeakSpeed ?? HandModel.MaxPeakSpeed, HandModel.MaxPeakSpeed);
            return Math.Max(d, 1.875 * amplitude / peak);
        }

        private double Overlap(double start, double duration) => start + Draw.Uniform(ref _rng, HandModel.OverlapLow, HandModel.OverlapHigh) * duration;

        private float PitchStep(float current, float? goal)
        {
            if (goal.HasValue) return goal.Value - current;
            const double deg = Math.PI / 180.0;
            double want;
            if (Draw.Chance(ref _rng, HandModel.OrientProbability))
            {
                want = HandModel.PitchCentreDegrees * deg + Draw.Normal(ref _rng, 0, HandModel.OrientSigmaDegrees * deg);
            }
            else
            {
                want = current + Draw.Laplace(ref _rng, HandModel.PitchLaplaceDegrees * deg) - HandModel.PitchPull * (current - HandModel.PitchCentreDegrees * deg);
            }

            want = Math.Clamp(want, HandModel.PitchLowDegrees * deg, HandModel.PitchHighDegrees * deg);
            return (float)((want - current) * (_style?.PitchScale ?? 1.0));
        }

        private float RollStep(float current, float? goal)
        {
            if (goal.HasValue) return goal.Value - current;
            if (!Rolls || !Draw.Chance(ref _rng, HandModel.RollProbability)) return 0f;
            const double deg = Math.PI / 180.0;
            double want = Draw.Chance(ref _rng, HandModel.RollOrientProbability)
                ? Draw.Normal(ref _rng, 0, HandModel.RollOrientSigmaDegrees * deg)
                : current + Draw.Laplace(ref _rng, HandModel.RollLaplaceDegrees * deg) - HandModel.RollPull * current;
            want = Math.Clamp(want, -HandModel.RollMaxDegrees * deg, HandModel.RollMaxDegrees * deg);
            return (float)((want - current) * (_style?.RollScale ?? 1.0));
        }

        /// <summary>
        /// A goal-directed move to <paramref name="goal"/> (the wrist's target): a primary submovement with an endpoint error
        /// (now and then curved through a point off its line, or in two pushes), then overlapping corrections aimed at what is
        /// left until within <paramref name="tolerance"/>. Returns when the next submovement may start.
        /// </summary>
        private double Transport(HandTimeline tl, Vector3 goal, double start, double slow = 1.0, double tolerance = HandModel.Tolerance, double errorScale = 1.0,
            float? pitchGoal = null, float? rollGoal = null, double? alongMu = null, int? maxCorrections = null, bool? via = null, double? maxPeak = null)
        {
            // The goal is the target's; the hand moves by the same displacement from where its own plan ends.
            Vector3 targetEnd = TargetEndpoint(tl);
            tl.Endpoint(out Vector3 e, out float pitchNow, out float rollNow);
            TargetAngles(tl, out float targetPitch, out float targetRoll);
            Vector3 handGoal = e + (goal - targetEnd);
            Vector3 vec = handGoal - e;
            float a = vec.Length();
            float dPitch = PitchStep(targetPitch, pitchGoal), dRoll = RollStep(targetRoll, rollGoal);
            if (a < 1e-4f && MathF.Abs(dPitch) < 1e-3f && MathF.Abs(dRoll) < 1e-3f) return start;
            Vector3 u = a > 1e-6f ? vec / a : Vector3.UnitX;
            double mu = alongMu ?? HandModel.ErrAlongMu;
            bool curved = via ?? Draw.Chance(ref _rng, HandModel.ViaProbability);
            bool split = !curved && via == null && Draw.Chance(ref _rng, HandModel.SplitProbability);
            double t = start, tLast, dLast;
            if (curved && a > 0.03f)
            {
                Vector3 end = handGoal + vec * (float)Draw.Normal(ref _rng, mu, HandModel.ErrAlongSigma * errorScale)
                              + Draw.Across(ref _rng, u, HandModel.ErrPerpSigma * a * errorScale);
                Vector3 bow = Draw.Across(ref _rng, u, HandModel.ViaSigma * a);
                (tLast, dLast) = Bowed(tl, t, end - e, bow, dPitch, dRoll, slow, maxPeak);
            }
            else if (split && a > 0.03f)
            {
                Vector3 dp = vec * (float)(1.0 + Draw.Normal(ref _rng, mu, HandModel.ErrAlongSigma * errorScale))
                             + Draw.Across(ref _rng, u, HandModel.ErrPerpSigma * a * errorScale);
                float f = (float)Draw.Uniform(ref _rng, 0.45, 0.7);
                Vector3 d1 = f * dp + Draw.Across(ref _rng, u, 0.08 * a), d2 = dp - d1;
                double dur1 = Duration(d1.Length(), slow, maxPeak);
                tl.Moves.Add(Move(t, dur1, d1, f * dPitch, f * dRoll));
                double t2 = t + Draw.Uniform(ref _rng, 0.55, 0.9) * dur1, dur2 = Duration(d2.Length(), slow, maxPeak);
                tl.Moves.Add(Move(t2, dur2, d2, (1 - f) * dPitch, (1 - f) * dRoll));
                (tLast, dLast) = (t2, dur2);
            }
            else
            {
                Vector3 dp = vec * (float)(1.0 + Draw.Normal(ref _rng, mu, HandModel.ErrAlongSigma * errorScale))
                             + (a > 1e-4f ? Draw.Across(ref _rng, u, HandModel.ErrPerpSigma * a * errorScale) : Vector3.Zero);
                double d = Duration(MathF.Max(dp.Length(), 0.004f), slow, maxPeak);
                tl.Moves.Add(Move(t, d, dp, dPitch, dRoll));
                (tLast, dLast) = (t, d);
            }

            float pitchGoalNow = pitchNow + dPitch, rollGoalNow = rollNow + dRoll;
            int corrections = maxCorrections ?? HandModel.MaxCorrections;
            for (int i = 0; i < corrections; i++)
            {
                tl.Endpoint(out e, out float pitchThen, out float rollThen);
                Vector3 r = handGoal - e;
                float ar = r.Length();
                if (ar < tolerance) break;
                Vector3 ur = r / ar;
                Vector3 dp = r * (float)(1.0 + Draw.Normal(ref _rng, HandModel.CorrAlongMu, HandModel.CorrAlongSigma * errorScale))
                             + Draw.Across(ref _rng, ur, HandModel.CorrPerpSigma * ar * errorScale);
                t = Overlap(tLast, dLast);
                double d = Duration(dp.Length(), slow * HandModel.CorrectionSlow, maxPeak);
                tl.Moves.Add(Move(t, d, dp, (pitchGoalNow - pitchThen) * 0.7f, (rollGoalNow - rollThen) * 0.7f));
                (tLast, dLast) = (t, d);
            }

            return Overlap(tLast, dLast);
        }

        /// <summary>The longest straight piece of a stroke along a curved way: short enough that its chord stays within a few millimetres of the way (a 4 cm chord of the 20 cm arc round the base: 1 mm).</summary>
        public const float AlongSegmentMeters = 0.04f;

        /// <summary>
        /// One stroke along <paramref name="path"/> (the target's way, arm base frame): overlapping submovements from via point to
        /// via point, so a curved way is followed as a hand follows it (a straight move would cut inside the arc round the base:
        /// 2.4 cm on a 20 cm swipe), ending with an endpoint error of the whole stroke's length (<paramref name="alongMu"/>: short
        /// below 0, past above), then corrections at what is left. Returns when the next submovement may start.
        /// </summary>
        private double Along(HandTimeline tl, IReadOnlyList<Vector3> path, double start, double slow, double? maxPeak, double alongMu, double errorScale,
            float? pitchGoal, float? rollGoal, double tolerance, int maxCorrections)
        {
            Vector3 targetEnd = TargetEndpoint(tl);
            tl.Endpoint(out Vector3 e, out float pitchNow, out float rollNow);
            TargetAngles(tl, out float targetPitch, out float targetRoll);
            Vector3 offset = e - targetEnd;
            var way = new List<Vector3> { targetEnd };
            foreach (Vector3 p in path)
            {
                if (Vector3.Distance(p, way[way.Count - 1]) > 1e-4f) way.Add(p);
            }

            if (way.Count < 2) return start;
            var cumulative = new double[way.Count];
            for (int i = 1; i < way.Count; i++) cumulative[i] = cumulative[i - 1] + Vector3.Distance(way[i - 1], way[i]);
            double length = cumulative[way.Count - 1];
            int segments = Math.Max(1, (int)Math.Ceiling(length / AlongSegmentMeters));
            var via = new Vector3[segments + 1];
            for (int j = 0, i = 0; j <= segments; j++)
            {
                double s = length * j / segments;
                while (i < way.Count - 2 && cumulative[i + 1] < s) i++;
                double seg = cumulative[i + 1] - cumulative[i];
                via[j] = Vector3.Lerp(way[i], way[i + 1], seg > 0 ? (float)Math.Clamp((s - cumulative[i]) / seg, 0, 1) : 0f);
            }

            float dPitch = PitchStep(targetPitch, pitchGoal), dRoll = RollStep(targetRoll, rollGoal);
            Vector3 last = Vector3.Normalize(via[segments] - via[segments - 1] + 1e-6f * Vector3.UnitX);
            Vector3 error = last * (float)(length * Draw.Normal(ref _rng, alongMu, HandModel.ErrAlongSigma * errorScale * 0.5))
                            + Draw.Across(ref _rng, last, HandModel.ErrPerpSigma * length * errorScale * 0.3);
            Vector3 handGoal = via[segments] + offset;
            double t = start, tLast = start, dLast = 0;
            Vector3 at = e;
            for (int i = 1; i <= segments; i++)
            {
                Vector3 next = via[i] + offset + (i == segments ? error : Vector3.Zero);
                Vector3 dp = next - at;
                double d = Duration(dp.Length(), slow, maxPeak);
                if (i > 1) t = tLast + Draw.Uniform(ref _rng, 0.35, 0.6) * dLast;
                tl.Moves.Add(Move(t, d, dp, i == 1 ? dPitch : 0f, i == 1 ? dRoll : 0f));
                (tLast, dLast) = (t, d);
                at = next;
            }

            float pitchGoalNow = pitchNow + dPitch, rollGoalNow = rollNow + dRoll;
            for (int i = 0; i < maxCorrections; i++)
            {
                tl.Endpoint(out Vector3 now, out float pitchThen, out float rollThen);
                Vector3 r = handGoal - now;
                float ar = r.Length();
                if (ar < tolerance) break;
                Vector3 ur = r / ar;
                Vector3 dp = r * (float)(1.0 + Draw.Normal(ref _rng, HandModel.CorrAlongMu, HandModel.CorrAlongSigma * errorScale))
                             + Draw.Across(ref _rng, ur, HandModel.CorrPerpSigma * ar * errorScale);
                t = Overlap(tLast, dLast);
                double d = Duration(dp.Length(), slow * HandModel.CorrectionSlow, maxPeak);
                tl.Moves.Add(Move(t, d, dp, (pitchGoalNow - pitchThen) * 0.7f, (rollGoalNow - rollThen) * 0.7f));
                (tLast, dLast) = (t, d);
            }

            return Overlap(tLast, dLast);
        }

        /// <summary>One stroke curved by <paramref name="bow"/> (across it): two overlapping submovements, half the way plus and minus the bow, so it has one speed peak, or a shallow dip, and a curved path.</summary>
        private (double Start, double Duration) Bowed(HandTimeline tl, double t, Vector3 dp, Vector3 bow, float dPitch, float dRoll, double slow = 1.0, double? maxPeak = null)
        {
            if (bow.Length() < 1e-4f)
            {
                double d = Duration(dp.Length(), slow, maxPeak);
                tl.Moves.Add(Move(t, d, dp, dPitch, dRoll));
                return (t, d);
            }

            Vector3 d1 = 0.5f * dp + bow, d2 = 0.5f * dp - bow;
            double dur1 = Duration(d1.Length(), slow, maxPeak);
            tl.Moves.Add(Move(t, dur1, d1, 0.5f * dPitch, 0.5f * dRoll));
            double t2 = t + Draw.Uniform(ref _rng, 0.3, 0.6) * dur1, dur2 = Duration(d2.Length(), slow, maxPeak);
            tl.Moves.Add(Move(t2, dur2, d2, 0.5f * dPitch, 0.5f * dRoll));
            return (t2, dur2);
        }

        /// <summary>Back-and-forth strokes along an axis, each starting before the last ends, bowed to one side: retracing one arc, or (mostly) going round a loop, so the reversals keep moving. <paramref name="slow"/> stretches every stroke (the mixed script's drawn speed).</summary>
        private double Reciprocate(HandTimeline tl, double start, double slow = 1.0)
        {
            int n = 1 + Draw.Poisson(ref _rng, HandModel.ReciprocateMean);
            double amplitude = Math.Clamp(Draw.LogNormal(ref _rng, HandModel.ReciprocateMedian, HandModel.ReciprocateSigma), 0.006, 0.12);
            Vector3 axis = Draw.Direction(ref _rng, 0.8f);
            Vector3 side = Vector3.Normalize(Draw.Across(ref _rng, axis, 1.0) + 1e-4f * Vector3.UnitZ);
            float bow = (float)Draw.Uniform(ref _rng, HandModel.ReciprocateBowLow, HandModel.ReciprocateBow);
            bool loop = Draw.Chance(ref _rng, HandModel.ReciprocateLoop);
            tl.Endpoint(out Vector3 e0, out _, out _);
            double t = start, tLast = start, dLast = 0;
            float sign = 1f;
            for (int k = 0; k < n; k++)
            {
                Vector3 ax = Vector3.Normalize(axis + Draw.Across(ref _rng, axis, HandModel.ReciprocateTurnSigmaDegrees * Math.PI / 180.0));
                Vector3 dp = sign * (float)amplitude * ax;
                if (k == n - 1 && n > 1)
                {
                    tl.Endpoint(out Vector3 e, out _, out _);
                    dp = 0.6f * (e0 - e) + 0.4f * dp; // the last stroke tends back toward where the strokes began
                }

                TargetAngles(tl, out float pitch, out float roll); // the target's, which releases set apart from the hand's
                Vector3 across = (loop ? sign : 1f) * side;
                (tLast, dLast) = Bowed(tl, t, dp, bow * dp.Length() * across, 0.4f * PitchStep(pitch, null), 0.5f * RollStep(roll, null), slow);
                t = Overlap(tLast, dLast);
                sign = -sign;
                amplitude = Math.Clamp(amplitude * Math.Exp(Draw.Normal(ref _rng, 0, HandModel.ReciprocateRatioSigma)), 0.004, 0.12);
            }

            return Overlap(tLast, dLast);
        }

        /// <summary>A few small moves in random directions: fine positioning. <paramref name="slow"/> stretches every move (the mixed script's drawn speed).</summary>
        private double Adjust(HandTimeline tl, double start, double slow = 1.0)
        {
            int n = 1 + (int)(_rng.NextDouble() * HandModel.AdjustMax);
            double t = start, tLast = start, dLast = 0;
            for (int i = 0; i < n; i++)
            {
                double a = Math.Clamp(Draw.LogNormal(ref _rng, HandModel.AdjustMedian, HandModel.AdjustSigma), 0.002, 0.04);
                Vector3 dp = (float)a * Draw.Direction(ref _rng, 1f);
                double d = Duration(a, 1.15 * slow);
                TargetAngles(tl, out float pitch, out float roll);
                tl.Moves.Add(Move(t, d, dp, 0.5f * PitchStep(pitch, null), 0.5f * RollStep(roll, null)));
                (tLast, dLast) = (t, d);
                t += Draw.Uniform(ref _rng, 0.5, 1.0) * d;
            }

            return Overlap(tLast, dLast);
        }

        /// <summary>
        /// Between moves: carry on (the next starts at <paramref name="next"/>, overlapping), keep the clutch but hold the hand
        /// still, or let go. A release mostly comes while the hand still moves (<paramref name="cutOff"/>: the rest of its last
        /// submovement is lost), and the clutch is often taken again on the move. Returns when the next move may start.
        /// </summary>
        private double Pause(HandTimeline tl, double next, PauseKind? kind, bool cutOff, double holdAtLeast = 0, int firstMove = 0)
        {
            PauseKind k = kind ?? (Draw.Chance(ref _rng, HandModel.ContinueProbability) ? PauseKind.Continue
                : Draw.Chance(ref _rng, HandModel.ReleaseProbability) ? PauseKind.Release : PauseKind.Dwell);
            if (k == PauseKind.Continue) return next;
            double motionEnd = tl.MotionEnd;
            if (k == PauseKind.Dwell) return Math.Max(next, motionEnd + Math.Min(HandModel.DwellMax, Draw.LogNormal(ref _rng, HandModel.DwellMedian, HandModel.DwellSigma)));

            // A release at speed comes during the move's last substantial submovement (not a trailing correction): people's
            // hands are still at 44% of their recent peak in the last 100 ms before the pinch opens.
            Submovement? cut = null;
            double top = 0;
            for (int i = firstMove; i < tl.Moves.Count; i++) top = Math.Max(top, tl.Moves[i].Displacement.Length() / tl.Moves[i].Duration);
            for (int i = firstMove; i < tl.Moves.Count; i++)
            {
                Submovement s = tl.Moves[i];
                if (s.Duration > 0.15 && s.Displacement.Length() / s.Duration >= 0.5 * top && (cut == null || s.End > cut.Value.End)) cut = s;
            }

            double release;
            if (cutOff && cut.HasValue && Draw.Chance(ref _rng, HandModel.CutOffProbability))
            {
                release = cut.Value.Start + cut.Value.Duration * Draw.Uniform(ref _rng, HandModel.CutOffLow, HandModel.CutOffHigh);
            }
            else
            {
                release = motionEnd + Draw.Uniform(ref _rng, 0.0, 0.12);
            }

            double hold = Math.Max(holdAtLeast, Math.Min(HandModel.HoldMax, Draw.LogNormal(ref _rng, HandModel.HoldMedian, HandModel.HoldSigma)));
            if (!AddRelease(tl, release, release + hold)) return Math.Max(next, motionEnd + 0.1);
            double engage = release + hold;
            while (Draw.Chance(ref _rng, HandModel.BlipProbability))
            {
                double again = engage + Draw.Uniform(ref _rng, HandModel.BlipLow, HandModel.BlipHigh);
                double hold2 = Math.Min(HandModel.HoldMax, Draw.LogNormal(ref _rng, HandModel.HoldMedian, HandModel.HoldSigma));
                if (!AddRelease(tl, again, again + hold2)) break;
                engage = again + hold2;
            }

            if (Draw.Chance(ref _rng, HandModel.EngageMovingProbability))
            {
                // The hand already moving when the pinch closes: the next move began a little before.
                double lead = Math.Min(Draw.Uniform(ref _rng, HandModel.EngageLeadLow, HandModel.EngageLeadHigh), 0.8 * hold);
                return Math.Max(release + Tick, engage - lead);
            }

            return engage + Draw.Uniform(ref _rng, 0.0, 0.10);
        }

        /// <summary>Adds a release unless it would overlap the last one (or come within two ticks of its end).</summary>
        private static bool AddRelease(HandTimeline tl, double start, double engage)
        {
            if (tl.Releases.Count > 0 && start < tl.Releases[tl.Releases.Count - 1].Engage + 2 * Tick) return false;
            if (engage - start < 2 * Tick) return false;
            tl.Releases.Add(new ClutchRelease(start, engage));
            return true;
        }

        /// <summary>Now and then the gripper changes between <paramref name="from"/> and <paramref name="to"/>, while the clutch is held.</summary>
        private void MaybeGrip(HandTimeline tl, double from, double to)
        {
            double busy = tl.Grips.Count == 0 ? 0 : tl.Grips[tl.Grips.Count - 1].End;
            int n = Draw.Poisson(ref _rng, HandModel.GripRatePerSecond * Math.Max(0.0, to - from));
            for (int i = 0; i < n; i++)
            {
                double start = Draw.Uniform(ref _rng, Math.Max(from, busy), Math.Max(from, busy) + Math.Max(0.05, to - from));
                double duration = Draw.Uniform(ref _rng, HandModel.GripDurationLow, HandModel.GripDurationHigh);
                if (tl.Released(start) || tl.Released(start + duration)) continue;
                float level = tl.FinalGripper;
                tl.Grips.Add(new GripMove(start, duration, level, DrawGripLevel(level)));
                busy = start + duration;
            }
        }

        /// <summary>A new gripper level: open, part-way or closed, at least 0.3 from <paramref name="current"/> (a change people make on purpose).</summary>
        private float DrawGripLevel(float current)
        {
            for (int i = 0; i < 8; i++)
            {
                double u = _rng.NextDouble();
                float level = u < HandModel.GripOpenProbability ? 0f
                    : u < 0.65 ? (float)Draw.Uniform(ref _rng, 0.1, 0.75)
                    : (float)Draw.Uniform(ref _rng, 0.8, 1.0);
                if (float.IsNaN(current) || MathF.Abs(level - current) >= 0.3f) return level;
            }

            return current > 0.5f ? 0f : 1f;
        }

        /// <summary>A free goal for the target: a lognormal distance from where it will be, in the human workspace, reachable or a little past the shell (the clutch keeps it on the shell).</summary>
        private bool SampleGoal(HandTimeline tl, out Vector3 goal)
        {
            Vector3 from = TargetEndpoint(tl);
            float maxReach = _profile.ProximalLinkLength + _profile.DistalLinkLength;
            var shoulder = new Vector3(0, 0, _profile.BaseHeight);
            for (int i = 0; i < 12; i++)
            {
                float a = (float)Math.Clamp(Draw.LogNormal(ref _rng, HandModel.TransportMedian, HandModel.TransportSigma), 0.01, HandModel.TransportMax);
                for (int j = 0; j < 10; j++)
                {
                    Vector3 direction = Draw.Direction(ref _rng, 0.7f);
                    if (Draw.Chance(ref _rng, HandModel.WorkspacePull))
                    {
                        double wr = Math.Clamp(Draw.Normal(ref _rng, HandModel.WorkRadius, HandModel.WorkRadiusSigma), HandModel.GoalRadiusLow, HandModel.GoalRadiusHigh);
                        double wz = Math.Clamp(Draw.Normal(ref _rng, HandModel.WorkHeight, HandModel.WorkHeightSigma), HandModel.GoalHeightLow, HandModel.GoalHeightHigh);
                        double wa = Math.Clamp(Draw.Normal(ref _rng, HandModel.WorkAzimuthDegrees, HandModel.WorkAzimuthSigmaDegrees), -HandModel.GoalYawDegrees, HandModel.GoalYawDegrees) * Math.PI / 180.0;
                        Vector3 toward = new Vector3((float)(wr * Math.Cos(wa)), (float)(wr * Math.Sin(wa)), (float)wz) - from;
                        if (toward.Length() > 1e-3f) direction = Vector3.Normalize(toward);
                    }

                    Vector3 g = from + a * direction;
                    float r = MathF.Sqrt(g.X * g.X + g.Y * g.Y), yaw = MathF.Atan2(g.Y, g.X) * 180f / MathF.PI;
                    if (r < HandModel.GoalRadiusLow || r > HandModel.GoalRadiusHigh || g.Z < HandModel.GoalHeightLow || g.Z > HandModel.GoalHeightHigh
                        || MathF.Abs(yaw) > HandModel.GoalYawDegrees) continue;
                    if (Vector3.Distance(g, shoulder) > maxReach + HandModel.ReachOvershootMeters) continue;
                    if (AvoidGoals != null && Near(new Vector2(g.X, g.Y), AvoidGoals, AvoidRadius)) continue;
                    goal = g;
                    return true;
                }
            }

            goal = from;
            return false;
        }

        private static bool Near(Vector2 p, IReadOnlyList<Vector2> points, float radius)
        {
            for (int i = 0; i < points.Count; i++)
            {
                if (Vector2.Distance(p, points[i]) < radius) return true;
            }

            return false;
        }
    }
}
