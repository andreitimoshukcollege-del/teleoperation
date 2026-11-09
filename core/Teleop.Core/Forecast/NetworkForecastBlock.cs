using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Forecast
{
    /// <summary>
    /// A forecast as the learned model's network block (layouts v3 and v4; a C# copy of
    /// <c>trainer/armpred/features/layout.py</c>'s <c>encode_forecast</c>, pinned by the forecast golden): per direction and
    /// horizon, <c>asinh(quantile ms / 100)</c> for each quantile and P(loss); per direction <c>asinh(floor ms / 100)</c>;
    /// per direction P(outage); the six validity flags (quantiles, loss, outage, per direction). An invalid field stays zero
    /// and its flag says so. The window adds the forecast's age as the block's last value.
    /// </summary>
    public static class NetworkForecastBlock
    {
        public const double DelayScaleMs = 100.0;

        /// <summary>The values written: the network block less its age.</summary>
        public const int Length = NetworkForecast.Directions * NetworkForecast.HorizonCount * (NetworkForecast.QuantileCount + 1)
                                  + NetworkForecast.Directions * (1 + 1 + 3);

        public static void Encode(NetworkForecast f, Span<float> block)
        {
            if (f == null) throw new ArgumentNullException(nameof(f));
            if (block.Length < Length) throw new ArgumentException($"the block needs {Length} values", nameof(block));
            block.Slice(0, Length).Clear();
            int k = 0;
            for (int d = 0; d < NetworkForecast.Directions; d++)
            {
                bool vq = f.ValidQ[d], vl = f.ValidLoss[d];
                for (int h = 0; h < NetworkForecast.HorizonCount; h++)
                {
                    if (vq)
                    {
                        // layout.py: np.arcsinh(float32 q / 100.0) is float32 arithmetic; the division is the same here.
                        for (int p = 0; p < NetworkForecast.QuantileCount; p++)
                        {
                            float x = f.QMs[NetworkForecast.QIndex(d, h, p)] / (float)DelayScaleMs;
                            block[k + p] = (float)Math.Asinh(x);
                        }
                    }

                    k += NetworkForecast.QuantileCount;
                    if (vl) block[k] = f.Loss[NetworkForecast.LossIndex(d, h)];
                    k++;
                }
            }

            for (int d = 0; d < NetworkForecast.Directions; d++)
            {
                if (f.ValidQ[d]) block[k] = (float)Math.Asinh((double)f.FloorMs[d] / DelayScaleMs);
                k++;
            }

            for (int d = 0; d < NetworkForecast.Directions; d++)
            {
                if (f.ValidOutage[d]) block[k] = f.Outage[d];
                k++;
            }

            for (int d = 0; d < NetworkForecast.Directions; d++)
            {
                block[k] = f.ValidQ[d] ? 1f : 0f;
                block[k + 1] = f.ValidLoss[d] ? 1f : 0f;
                block[k + 2] = f.ValidOutage[d] ? 1f : 0f;
                k += 3;
            }
        }
    }
}
