namespace Teleop.RobotHost.Time
{
    /// <summary>
    /// Whether a line RobotHost prints at most once per interval is due. <see cref="Never"/> means not printed yet. Until
    /// 2026-11-24 the two such lines (joint commands the plant refused, operator commands the arm power ignored) started from
    /// <c>long.MinValue</c> and asked <c>now - last &gt;= interval</c>, which overflows to a negative number, so neither line
    /// ever printed: a twin batch's 38 takes had 476 refused commands and its log not one line (docs/adr/0038 "M, motion").
    /// </summary>
    internal static class LogThrottle
    {
        public const long Never = long.MinValue;

        public static bool Due(long now, long last, long interval) => last == Never || now - last >= interval;
    }
}
