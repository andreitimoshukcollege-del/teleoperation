using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using Teleop.Eval.ImportUrdf;
using Teleop.Eval.IkStress;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Types;
using Xunit.Abstractions;

namespace Teleop.Eval.Tests.IkStress
{
    /// <summary>
    /// The FR3 Duo's teleop under a long synthetic operator (<see cref="IkStressRunner"/>, <c>just fr3-ik-stress</c>): the user,
    /// 2026-11-03, "it still gets stuck and confused in some positions ... the robot should never get stuck or be pointing the
    /// wrong way". What these hold: once the hand stops on a target the arm can reach, the arm gets there, by its own steps or
    /// by moving to another pose of the arm (its recovery); it is never left off such a target with no recovery tried. Without
    /// the recovery, the same operator leaves it stuck many times.
    /// </summary>
    public class IkStressTests
    {
        private readonly ITestOutputHelper _out;

        public IkStressTests(ITestOutputHelper output) => _out = output;

        private static (SerialChainProfile, SerialChainCollisionModel) Fr3Duo()
        {
            string root = RepoRoot();
            SerialChainProfile profile = SerialChainProfileJson.Load(Path.Combine(root, "core", "RobotProfiles", "fr3-duo.json"));
            var doc = JsonSerializer.Deserialize<SerialChainCollisionDocument>(File.ReadAllText(Path.Combine(root, "core", "RobotProfiles", "fr3-duo.collision.json")),
                new JsonSerializerOptions { IncludeFields = true });
            return (profile, doc!.ToModel());
        }

        [Fact]
        public void AReachableTargetTheHandHoldsOn_IsReached_AndNeverLeftWithoutARecovery()
        {
            (SerialChainProfile profile, SerialChainCollisionModel collision) = Fr3Duo();
            // One 90 s session (just fr3-ik-stress runs the long ones): with the recovery, and without it.
            var options = new IkStressOptions { Seconds = 90, OracleStarts = 24 };
            var now = new[] { IkStressRunner.Run(profile, collision, 1, options) };
            var before = new[] { IkStressRunner.Run(profile, collision, 1,
                new IkStressOptions { Seconds = 90, OracleStarts = 24, Teleop = new SerialChainTeleopOptions { RecoverAfterSeconds = 0, PostureSpeed = 0 } }) };

            int abandoned = now.Sum(r => r.Episodes.Count(e => e.Kind != "slow" && !e.Recovering));
            int stuckBefore = before.Sum(r => r.Stuck + r.WrongWay);
            var settle = now.SelectMany(r => r.SettleSeconds).OrderBy(x => x).ToList();
            double p95 = settle[(int)Math.Min(settle.Count - 1, Math.Ceiling(0.95 * settle.Count) - 1)];
            _out.WriteLine($"now: {now.Sum(r => r.Stuck)} stuck, {now.Sum(r => r.WrongWay)} wrong-way, {now.Sum(r => r.Slow)} slow, {abandoned} without a recovery; " +
                           $"{settle.Count} holds, settling p95 {p95:0.00} s; recovery off: {stuckBefore} stuck or wrong-way");

            Assert.True(settle.Count > 30, $"only {settle.Count} holds on a reachable target: the session no longer tests settling");
            Assert.Equal(0, abandoned);
            Assert.True(p95 < 2.0, $"the arm reached a held target {p95:0.00} s after the hand stopped (p95)");
            Assert.True(stuckBefore >= now.Sum(r => r.Stuck + r.WrongWay) + 3, "the recovery no longer makes a difference: the session no longer finds traps");
        }

        [Fact]
        public void TheSession_IsDeterministic()
        {
            (SerialChainProfile profile, SerialChainCollisionModel collision) = Fr3Duo();
            var options = new IkStressOptions { Seconds = 20, OracleStarts = 16 };
            IkStressResult a = IkStressRunner.Run(profile, collision, 3, options), b = IkStressRunner.Run(profile, collision, 3, options);
            Assert.Equal(a.PositionMm, b.PositionMm);
            Assert.Equal(a.AngleDeg, b.AngleDeg);
            Assert.Equal(a.Searches, b.Searches);
        }

        private static string RepoRoot()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                if (Directory.Exists(Path.Combine(dir.FullName, "core", "RobotProfiles")) && Directory.Exists(Path.Combine(dir.FullName, "robot", "franka")))
                {
                    return dir.FullName;
                }
            }

            throw new InvalidOperationException($"no repo root above {AppContext.BaseDirectory}");
        }
    }
}
