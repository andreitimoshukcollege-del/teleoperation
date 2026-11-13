using System.Text.Json;

namespace Teleop.RobotArm.Tests
{
    /// <summary>
    /// Compares a golden JSON file with what the code writes now: the same structure and strings, and numbers within a
    /// tolerance. Text equality fails across platforms, because sin and cos may differ in the last digit between Windows
    /// and Linux; the Python side compares with a tolerance too.
    /// </summary>
    internal static class GoldenFile
    {
        public static void AssertSame(string expectedJson, string actualJson, double tolerance = 1e-12)
        {
            using JsonDocument e = JsonDocument.Parse(expectedJson), a = JsonDocument.Parse(actualJson);
            Compare(e.RootElement, a.RootElement, "$", tolerance);
        }

        private static void Compare(JsonElement e, JsonElement a, string at, double tol)
        {
            Assert.True(e.ValueKind == a.ValueKind, $"{at}: {e.ValueKind} in the golden file, {a.ValueKind} now");
            switch (e.ValueKind)
            {
                case JsonValueKind.Object:
                    var ek = e.EnumerateObject().Select(p => p.Name).ToArray();
                    var ak = a.EnumerateObject().Select(p => p.Name).ToArray();
                    Assert.True(ek.SequenceEqual(ak), $"{at}: keys [{string.Join(",", ek)}] in the golden file, [{string.Join(",", ak)}] now");
                    foreach (string k in ek) Compare(e.GetProperty(k), a.GetProperty(k), at + "." + k, tol);
                    break;
                case JsonValueKind.Array:
                    Assert.True(e.GetArrayLength() == a.GetArrayLength(), $"{at}: {e.GetArrayLength()} items in the golden file, {a.GetArrayLength()} now");
                    int i = 0;
                    foreach ((JsonElement x, JsonElement y) in e.EnumerateArray().Zip(a.EnumerateArray())) Compare(x, y, $"{at}[{i++}]", tol);
                    break;
                case JsonValueKind.Number:
                    double ev = e.GetDouble(), av = a.GetDouble();
                    Assert.True(Math.Abs(ev - av) <= tol, $"{at}: {ev:R} in the golden file, {av:R} now");
                    break;
                default:
                    Assert.True(e.GetRawText() == a.GetRawText(), $"{at}: {e.GetRawText()} in the golden file, {a.GetRawText()} now");
                    break;
            }
        }
    }
}
