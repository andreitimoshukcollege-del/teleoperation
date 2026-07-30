using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Types;

public class SeededRngTests
{
    [Fact]
    public void SameSeed_ProducesIdenticalSequence()
    {
        var a = new SeededRng(12345UL);
        var b = new SeededRng(12345UL);

        for (int i = 0; i < 1000; i++)
        {
            Assert.Equal(a.NextUInt64(), b.NextUInt64());
        }
    }

    [Fact]
    public void DifferentSeeds_ProduceDifferentSequences()
    {
        var a = new SeededRng(1UL);
        var b = new SeededRng(2UL);

        bool anyDifferent = false;
        for (int i = 0; i < 16; i++)
        {
            if (a.NextUInt64() != b.NextUInt64())
            {
                anyDifferent = true;
                break;
            }
        }

        Assert.True(anyDifferent);
    }

    [Fact]
    public void NextDouble_IsWithinUnitInterval()
    {
        var rng = new SeededRng(999UL);

        for (int i = 0; i < 10_000; i++)
        {
            double value = rng.NextDouble();
            Assert.True(value >= 0.0 && value < 1.0, $"value {value} out of [0,1)");
        }
    }

    [Fact]
    public void Reset_ReproducesTheOriginalSequence()
    {
        var rng = new SeededRng(42UL);

        var beforeReset = new ulong[10];
        for (int i = 0; i < beforeReset.Length; i++)
        {
            beforeReset[i] = rng.NextUInt64();
        }

        rng.Reset();

        for (int i = 0; i < beforeReset.Length; i++)
        {
            Assert.Equal(beforeReset[i], rng.NextUInt64());
        }
    }

    [Fact]
    public void ZeroSeed_DoesNotProduceDegenerateAllZeroState()
    {
        var rng = new SeededRng(0UL);

        bool anyNonZero = false;
        for (int i = 0; i < 16; i++)
        {
            if (rng.NextUInt64() != 0UL)
            {
                anyNonZero = true;
                break;
            }
        }

        Assert.True(anyNonZero);
    }

    [Fact]
    public void NextUInt64_Allocates_Zero_Bytes()
    {
        var rng = new SeededRng(7UL);
        AllocationAssert.Zero(() => rng.NextUInt64());
    }

    [Fact]
    public void NextDouble_Allocates_Zero_Bytes()
    {
        var rng = new SeededRng(7UL);
        AllocationAssert.Zero(() => rng.NextDouble());
    }
}
