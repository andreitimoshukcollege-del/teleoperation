namespace Teleop.RobotHost.Plant
{
    /// <summary>Whether a point a fraction <c>alpha</c> of the way along a line passes a check (<see cref="LineClamp"/>).</summary>
    public interface IAlphaCheck
    {
        bool Passes(float alpha);
    }

    /// <summary>How <see cref="LineClamp.Search"/> found its answer.</summary>
    public enum LineClampKind
    {
        /// <summary>No point on the line passes: neither its start nor any point tried. The caller holds.</summary>
        Nothing,

        /// <summary>The bisection's boundary: the line looked monotone (passing, then failing).</summary>
        Bisection,

        /// <summary>
        /// The line is not monotone: a point on the grid beyond the bisection's failing bound passed. The answer is the largest
        /// passing grid point.
        /// </summary>
        Grid,
    }

    /// <summary>What <see cref="LineClamp.Search"/> found.</summary>
    public readonly struct LineClampResult
    {
        public readonly LineClampKind Kind;

        /// <summary>The farthest passing fraction found, in (0, 1); 0 when <see cref="Kind"/> is <see cref="LineClampKind.Nothing"/>.</summary>
        public readonly float Alpha;

        /// <summary>How many times the check was asked.</summary>
        public readonly int Checks;

        public LineClampResult(LineClampKind kind, float alpha, int checks)
        {
            Kind = kind;
            Alpha = alpha;
            Checks = checks;
        }
    }

    /// <summary>
    /// The farthest point toward a failing end that still passes a check, on a line from a start (alpha 0) to that end (alpha
    /// 1), for RobotHost's clamp (docs/adr/0020 amendment 2026-11-24): a joint command the collision check refuses is replaced
    /// by the farthest point toward it on the joint-space line from the arm's measured pose that passes the same check.
    /// <para>Deterministic and allocation-free: the check is a struct, so nothing is boxed or captured. At most
    /// 1 + <see cref="BisectionRounds"/> + (<see cref="GridSteps"/> - 1) checks, 24.</para>
    /// <list type="number">
    /// <item>The start is asked first. If it fails, nothing passes (<see cref="LineClampKind.Nothing"/>).</item>
    /// <item>Bisection, <see cref="BisectionRounds"/> rounds, between a passing bound (from 0) and a failing one (from 1, which
    /// the caller has already found failing): the boundary to 1/256 of the line, if the line passes up to one point and fails
    /// after it.</item>
    /// <item><b>Monotonicity probe.</b> Bisection alone cannot see a second passing stretch beyond its failing bound. So the
    /// grid points k/<see cref="GridSteps"/> beyond that bound are asked from the end down, stopping at the first that passes.
    /// One passing means the line is not monotone (pass, fail, pass again), and that grid point, the largest passing one on
    /// the grid, is the answer (<see cref="LineClampKind.Grid"/>). None passing, the bisection's bound stands.</item>
    /// </list>
    /// A failing point below the passing bound is not looked for: it cannot make the answer larger, and the answer is judged on
    /// its own, never by its neighbours.
    /// </summary>
    public static class LineClamp
    {
        public const int BisectionRounds = 8;
        public const int GridSteps = 16;

        /// <summary>The worst case of <see cref="LineClampResult.Checks"/>.</summary>
        public const int MaxChecks = 1 + BisectionRounds + GridSteps - 1;

        /// <summary>Searches the line; the caller has found its end (alpha 1) failing.</summary>
        public static LineClampResult Search<TCheck>(ref TCheck check) where TCheck : struct, IAlphaCheck
        {
            int checks = 1;
            if (!check.Passes(0f))
            {
                return new LineClampResult(LineClampKind.Nothing, 0f, checks);
            }

            float lo = 0f, hi = 1f;
            for (int round = 0; round < BisectionRounds; round++)
            {
                float mid = 0.5f * (lo + hi);
                checks++;
                if (check.Passes(mid)) lo = mid;
                else hi = mid;
            }

            for (int k = GridSteps - 1; k > 0; k--)
            {
                float alpha = (float)k / GridSteps;
                if (alpha <= hi) break;
                checks++;
                if (check.Passes(alpha))
                {
                    return new LineClampResult(LineClampKind.Grid, alpha, checks);
                }
            }

            return lo > 0f
                ? new LineClampResult(LineClampKind.Bisection, lo, checks)
                : new LineClampResult(LineClampKind.Nothing, 0f, checks);
        }
    }
}
