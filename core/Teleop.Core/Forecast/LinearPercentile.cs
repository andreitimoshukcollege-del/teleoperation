using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Forecast
{
    /// <summary>
    /// numpy's <c>np.percentile(values, 100 * q)</c> with its default <c>linear</c> method, as the trainer's forecasters
    /// call it: the sorted values' virtual index <c>q (n − 1)</c>, and numpy's two-sided lerp between its neighbours
    /// (<c>a + (b − a) t</c>, or <c>b − (b − a)(1 − t)</c> when <c>t ≥ 0.5</c>), so the result is numpy's to the last bit
    /// where the arithmetic allows. Sorts <paramref name="values"/> in place; allocation-free.
    /// </summary>
    public static class LinearPercentile
    {
        /// <summary>Sorts the first <paramref name="count"/> values; then <see cref="OfSorted"/> answers each quantile.</summary>
        public static void Sort(double[] values, int count) => Array.Sort(values, 0, count);

        public static double OfSorted(double[] sorted, int count, double quantile)
        {
            if (count <= 0) throw new ArgumentException("no values", nameof(count));
            double q = 100.0 * quantile / 100.0; // np.percentile(..., 100 * q) divides by 100 again
            double virtualIndex = q * (count - 1);
            double previous = Math.Floor(virtualIndex);
            int lo = (int)previous;
            if (lo < 0) lo = 0;
            if (lo > count - 1) lo = count - 1;
            int hi = Math.Min(lo + 1, count - 1);
            double t = virtualIndex - previous;
            double a = sorted[lo], b = sorted[hi];
            double diff = b - a;
            return t >= 0.5 ? b - diff * (1.0 - t) : a + diff * t;
        }
    }
}
