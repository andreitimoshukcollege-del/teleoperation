using Xunit;

namespace Teleop.Core.Tests.TestSupport;

/// <summary>
/// The gate that guards the gate. <see cref="AllocationAssert.Zero"/> retries a measurement window
/// to absorb a JIT-tiering race, and a retry loop that quietly always passed would be strictly
/// worse than the flake it replaced — the failure would be silent instead of noisy, and every
/// `Allocates_Zero_Bytes` test in the suite would become decorative.
///
/// So: one test that it still fails on a real allocation, and one that it passes on a genuinely
/// clean delegate. Neither is hypothetical — the first is the exact property the retry could
/// plausibly have destroyed.
/// </summary>
public class AllocationAssertTests
{
    [Fact]
    public void Zero_StillFails_WhenTheDelegateAllocatesEveryCall()
    {
        // 24 bytes minimum per object, every call -- four orders of magnitude above the
        // sub-byte-per-call residue the retry is meant to absorb. No window can come back clean,
        // so the retry budget is spent and the assertion fires.
        Assert.ThrowsAny<Xunit.Sdk.XunitException>(
            () => AllocationAssert.Zero(() => _ = new object(), iterations: 100));
    }

    [Fact]
    public void Zero_StillFails_WhenTheDelegateAllocatesOnlyOccasionally()
    {
        // The case a single-window harness cannot distinguish from the tiering race, and the
        // reason two *consecutive* clean windows are required rather than one: something that
        // allocates periodically keeps landing in windows and can never produce two in a row.
        int call = 0;
        Assert.ThrowsAny<Xunit.Sdk.XunitException>(
            () => AllocationAssert.Zero(
                () =>
                {
                    if (++call % 50 == 0)
                    {
                        _ = new object();
                    }
                },
                iterations: 100));
    }

    [Fact]
    public void Zero_Passes_ForAGenuinelyAllocationFreeDelegate()
    {
        int accumulator = 0;
        AllocationAssert.Zero(() => accumulator++, iterations: 100);
        Assert.True(accumulator > 0);
    }
}
