using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

public class JointPredictorBankTests
{
    private const long Tps = 1000;
    private const byte Arm = 1, Gripper = 10;

    private static JointPredictorConfig Config() =>
        new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: 5f,
            instantMotorIds: new[] { Gripper }, maxHorizonTicks: 1000, ticksPerSecond: Tps);

    private static JointPredictorPreset Preset(string label, string key) => new JointPredictorPreset(label, key, Config());

    /// <summary>hold-last, servo-replay and relay-replay; servo-replay active.</summary>
    private static JointPredictorBank Bank(int active = 1) => JointPredictorBank.Create(new[]
    {
        Preset("hold", "hold-last"),
        Preset("servo", "servo-replay"),
        Preset("relay", "relay-replay"),
    }, active);

    private static JointSample[] State(float arm) => new[] { new JointSample(Arm, arm, true), new JointSample(Gripper, 0f, false) };

    private static JointSample[] Command(float arm) => new[] { new JointSample(Arm, arm, false), new JointSample(Gripper, 0f, false) };

    private static float Predict(JointPredictorBank bank, int member, long at)
    {
        var output = new JointSample[8];
        Assert.True(bank.TryPredictMember(member, at, output, out int count));
        for (int i = 0; i < count; i++)
        {
            if (output[i].MotorId == Arm) return output[i].Value;
        }

        throw new Xunit.Sdk.XunitException("arm not predicted");
    }

    /// <summary>A state of 0 at 0, then a command to 1 rad taking effect at 0: servo-replay moves, hold-last does not.</summary>
    private static void Feed(JointPredictorBank p)
    {
        p.ObserveState(0, State(0f));
        p.ObserveCommand(0, Command(1f), 0f);
    }

    [Fact]
    public void TheBank_AnswersWithTheActiveMember_AndFansEveryCallOut()
    {
        var bank = Bank(active: 1);
        Feed(bank);

        var output = new JointSample[8];
        Assert.True(bank.TryPredict(100, output, out int count));
        float viaBank = output[0].Value;
        Assert.Equal(Predict(bank, 1, 100), viaBank);    // the active member answers
        Assert.True(viaBank > 0.3f);                       // servo-replay has moved toward 1 rad by 100 ms
        Assert.Equal(0f, Predict(bank, 0, 100));          // hold-last got the state too, and holds it
        Assert.True(bank.HasState(0) && bank.HasState(1) && bank.HasState(2));
        Assert.Equal("servo", bank.ActiveLabel);
    }

    [Fact]
    public void Switching_IsInstant_AndKeepsHistory()
    {
        var bank = Bank(active: 1);
        Feed(bank);
        float servoBefore = Predict(bank, 1, 100);

        Assert.True(bank.TrySetActive(0));
        var output = new JointSample[8];
        Assert.True(bank.TryPredict(100, output, out _));
        Assert.Equal(0f, output[0].Value);                 // hold-last now draws, with its state

        Assert.True(bank.TrySetActive(1));
        Assert.True(bank.TryPredict(100, output, out _));
        Assert.Equal(servoBefore, output[0].Value);        // and switching back loses nothing
    }

    [Fact]
    public void ADisabledMember_GetsNothing_AndEnablingResetsIt()
    {
        var bank = Bank(active: 1);
        Assert.True(bank.TrySetEnabled(0, false));
        Feed(bank);
        Assert.False(bank.TryPredictMember(0, 100, new JointSample[8], out _));
        Assert.False(bank.TrySetActive(0));                // a disabled member cannot draw
        int generation = bank.Generation(0);

        Assert.True(bank.TrySetEnabled(0, true));
        Assert.Equal(generation + 1, bank.Generation(0));  // enabling reset it
        Assert.False(bank.HasState(0));                    // warming up
        Assert.False(bank.TryPredictMember(0, 100, new JointSample[8], out _));

        bank.ObserveState(200, State(0.5f));
        Assert.True(bank.HasState(0));
        Assert.Equal(0.5f, Predict(bank, 0, 250));
    }

    [Fact]
    public void TheActiveMember_CannotBeDisabled()
    {
        var bank = Bank(active: 2);
        Assert.False(bank.TrySetEnabled(2, false));
        Assert.True(bank.IsEnabled(2));
        Assert.False(bank.TrySetEnabled(7, false));
        Assert.False(bank.TrySetActive(-1));
    }

    [Fact]
    public void Create_RejectsAnUnknownKey_AndDuplicateLabels()
    {
        var unknown = Assert.Throws<ArgumentException>(() => JointPredictorBank.Create(new[] { Preset("x", "no-such-predictor") }, 0));
        Assert.Contains("relay-replay", unknown.Message);   // it names the keys that exist
        Assert.Throws<ArgumentException>(() => JointPredictorBank.Create(new[] { Preset("a", "hold-last"), Preset("a", "relay-replay") }, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => JointPredictorBank.Create(new[] { Preset("a", "hold-last") }, 1));
    }

    [Fact]
    public void TwoPresets_MayShareARegistryKey()
    {
        var bank = JointPredictorBank.Create(new[] { Preset("fit A", "relay-replay-servo"), Preset("fit B", "relay-replay-servo") }, 0);
        Assert.Equal(2, bank.Count);
        Assert.Equal(1, bank.IndexOf("fit B"));
        Assert.Equal(-1, bank.IndexOf("nope"));
    }

    [Fact]
    public void Reset_ReturnsEveryMemberToAsConstructed_AndKeepsTheSelection()
    {
        var bank = Bank(active: 2);
        Assert.True(bank.TrySetEnabled(0, false));
        Feed(bank);
        bank.Reset();

        Assert.Equal(2, bank.ActiveIndex);
        Assert.False(bank.IsEnabled(0));
        Assert.True(bank.IsEnabled(1));
        for (int i = 0; i < bank.Count; i++) Assert.False(bank.HasState(i));
        Assert.False(bank.TryPredict(100, new JointSample[8], out _));

        // As constructed: the same calls give the same answers as a fresh bank's.
        var fresh = Bank(active: 2);
        Assert.True(fresh.TrySetEnabled(0, false));
        Feed(bank);
        Feed(fresh);
        Assert.Equal(Predict(fresh, 2, 80), Predict(bank, 2, 80));
    }

    [Fact]
    public void FeedingPredictingAndSwitching_AllocateNothing()
    {
        var bank = Bank(active: 1);
        JointSample[] state = State(0f);
        var commands = new[] { Command(0f), Command(1f) };
        var output = new JointSample[8];
        long t = 0;
        AllocationAssert.Zero(() =>
        {
            t += 20;
            bank.ObserveCommand(t, commands[(int)(t / 20 % 2)], 0f);
            bank.ObserveState(t, state);
            bank.TryPredict(t + 50, output, out _);
            bank.TryPredictMember(0, t + 50, output, out _);
            bank.TrySetActive((int)(t / 20 % 3));
            bank.TrySetEnabled(0, (t / 20 % 5) != 0 || bank.ActiveIndex == 0);
        });
    }
}
