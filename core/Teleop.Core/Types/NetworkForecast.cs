using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// One network forecast, format version 1 (trainer/contract/FORECAST_FORMAT.md; docs/adr/0021 decision 11, 0035 N3):
    /// issued at <see cref="IssuedTicks"/> on the operator's clock, per direction (<see cref="Up"/>: commands to the robot,
    /// <see cref="Down"/>: frames from it) and per send-time horizon, the one-way delay's quantiles in ms and the probability
    /// of loss; per direction the floor (ms) and the probability of an outage within 1 s; and which of those the forecaster
    /// knows. An unknown field is marked invalid and left zero. Values are <c>float</c>, as the trainer stores them.
    /// Preallocated and reused: a forecaster writes into one with <see cref="Clear"/> first.
    /// </summary>
    public sealed class NetworkForecast
    {
        public const int Directions = 2;
        public const int Up = 0;
        public const int Down = 1;
        public const int HorizonCount = 9;
        public const int QuantileCount = 4;
        /// <summary>The horizon index of h = 0, and the quantile index of the median.</summary>
        public const int H0 = 4;
        public const int P50 = 1;

        private static readonly int[] HorizonsMsTable = { -1000, -500, -250, -100, 0, 100, 250, 500, 1000 };
        private static readonly double[] QuantilesTable = { 0.10, 0.50, 0.90, 0.99 };

        /// <summary>Send-time horizon <paramref name="h"/> in ms: −1000, −500, −250, −100, 0, +100, +250, +500, +1000.</summary>
        public static int HorizonMs(int h) => HorizonsMsTable[h];

        /// <summary>Quantile <paramref name="p"/>: 0.10, 0.50, 0.90, 0.99.</summary>
        public static double Quantile(int p) => QuantilesTable[p];

        /// <summary>Index into <see cref="QMs"/> of direction <paramref name="d"/>, horizon <paramref name="h"/>, quantile <paramref name="p"/>.</summary>
        public static int QIndex(int d, int h, int p) => (d * HorizonCount + h) * QuantileCount + p;

        /// <summary>Index into <see cref="Loss"/> of direction <paramref name="d"/>, horizon <paramref name="h"/>.</summary>
        public static int LossIndex(int d, int h) => d * HorizonCount + h;

        public readonly float[] QMs = new float[Directions * HorizonCount * QuantileCount];
        public readonly float[] Loss = new float[Directions * HorizonCount];
        public readonly float[] FloorMs = new float[Directions];
        public readonly float[] Outage = new float[Directions];
        public readonly bool[] ValidQ = new bool[Directions];
        public readonly bool[] ValidLoss = new bool[Directions];
        public readonly bool[] ValidOutage = new bool[Directions];

        public long IssuedTicks { get; private set; }

        /// <summary>Everything zero and invalid, issued at <paramref name="issuedTicks"/>.</summary>
        public void Clear(long issuedTicks)
        {
            IssuedTicks = issuedTicks;
            Array.Clear(QMs, 0, QMs.Length);
            Array.Clear(Loss, 0, Loss.Length);
            Array.Clear(FloorMs, 0, Directions);
            Array.Clear(Outage, 0, Directions);
            Array.Clear(ValidQ, 0, Directions);
            Array.Clear(ValidLoss, 0, Directions);
            Array.Clear(ValidOutage, 0, Directions);
        }

        public void CopyFrom(NetworkForecast other)
        {
            if (other == null) throw new ArgumentNullException(nameof(other));
            IssuedTicks = other.IssuedTicks;
            Array.Copy(other.QMs, QMs, QMs.Length);
            Array.Copy(other.Loss, Loss, Loss.Length);
            Array.Copy(other.FloorMs, FloorMs, Directions);
            Array.Copy(other.Outage, Outage, Directions);
            Array.Copy(other.ValidQ, ValidQ, Directions);
            Array.Copy(other.ValidLoss, ValidLoss, Directions);
            Array.Copy(other.ValidOutage, ValidOutage, Directions);
        }
    }
}
