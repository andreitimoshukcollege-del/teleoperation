namespace Teleop.Core.Tests.TestSupport;

/// <summary>
/// Asserts a delegate allocates nothing on the managed heap. The first allocation-assertion
/// harness in this project — enforces the "no allocations in the per-frame hot path" invariant
/// (root CLAUDE.md invariant 8) for every hot-path method introduced from Phase 3 onward.
/// </summary>
public static class AllocationAssert
{
    /// <summary>
    /// Calls <paramref name="action"/> once, untimed, to absorb one-time costs that are not the
    /// hot path allocating (JIT compilation, a lazy static initializer, first-use collection
    /// growth) — then calls it <paramref name="iterations"/> more times and asserts the total
    /// bytes allocated on the current thread is exactly zero, not merely small. A truly
    /// allocation-free path allocates nothing; a nonzero-but-tiny delta is exactly the kind of
    /// regression (a boxed struct, a closure capture, a hidden string interpolation) this exists
    /// to catch rather than tolerate.
    /// </summary>
    public static void Zero(Action action, int iterations = 10_000)
    {
        action();

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < iterations; i++)
        {
            action();
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        long allocated = after - before;
        Assert.True(
            allocated == 0,
            $"Expected zero allocation over {iterations} iterations, but {allocated} bytes " +
            $"were allocated ({(double)allocated / iterations:F3} bytes/call).");
    }
}
