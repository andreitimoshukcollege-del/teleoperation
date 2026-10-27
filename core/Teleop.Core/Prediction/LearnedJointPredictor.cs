using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// The learned joint predictor (<c>learned</c>; docs/adr/0021, docs/adr/0025): the trainer's model
    /// correcting a speed-blind <c>relay-replay</c>. Each prediction builds the layout-v3 window
    /// (<see cref="LearnedJointWindow"/>), runs <see cref="JointPredictorConfig.Model"/> on it, and
    /// answers <c>prior + correction</c> for the five arm joints and the gripper; any other joint takes the
    /// prior. The model is the host's (an <see cref="IInferenceBackend"/>, loaded once); the config's other
    /// values must be the ones the model card names for its prior (the headset's, with its fitted
    /// reading ages).
    ///
    /// <para>Falls back to the prior's own prediction, and counts it (trainer/contract/MODEL_FORMAT.md):
    /// no fresh reading of every arm joint yet; the newest reading more than 1 s old; the backend failing;
    /// an output that is not finite. Deterministic for a deterministic backend; allocation-free.</para>
    /// </summary>
    public sealed class LearnedJointPredictor : ICommandAwarePredictor
    {
        private readonly IInferenceBackend _model;
        private readonly LearnedJointWindow _window;
        private readonly float[] _x = new float[LearnedJointWindow.InputLength];
        private readonly float[] _y;
        private readonly double[] _prior = new double[LearnedJointWindow.JointCount];

        public LearnedJointPredictor(JointPredictorConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            _model = config.Model ?? throw new ArgumentException("the learned predictor needs a model (JointPredictorConfig.Model)", nameof(config));
            if (_model.InputLength != LearnedJointWindow.InputLength || _model.OutputLength != 2 * LearnedJointWindow.JointCount)
            {
                throw new ArgumentException(
                    $"model {_model.ModelId}: {_model.InputLength} in, {_model.OutputLength} out; layout v3 is {LearnedJointWindow.InputLength} in, {2 * LearnedJointWindow.JointCount} out",
                    nameof(config));
            }

            _y = new float[_model.OutputLength];
            _window = new LearnedJointWindow(config);
        }

        /// <summary>Predictions answered by the model.</summary>
        public long Inferences { get; private set; }

        /// <summary>Predictions answered by the prior instead: no fresh reading yet, stale, or the model failed.</summary>
        public long Fallbacks { get; private set; }

        public string ModelId => _model.ModelId;

        public PredictorDiagnostics Diagnostics => _window.Prior.Diagnostics;

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets, float speedRadPerSec) =>
            _window.ObserveCommand(effectiveTicks, targets); // speed-blind: the model's prior is given no speed

        public void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints) => _window.ObserveState(sensedTicks, joints);

        public bool TryPredict(long targetTicks, Span<JointSample> output, out int count)
        {
            if (!_window.Prior.TryPredict(targetTicks, output, out count))
            {
                return false;
            }

            if (!_window.TryBuild(targetTicks, _x, _prior, out bool valid) || !valid || !_model.TryRun(_x, _y) || !Finite(_y))
            {
                Fallbacks++;
                return true;
            }

            for (int i = 0; i < count; i++)
            {
                byte motor = output[i].MotorId;
                int row = LearnedJointWindow.RowOf(motor);
                if (row < 0) continue;
                double value = _prior[row] + _y[2 * row];
                output[i] = new JointSample(motor, (float)LearnedJointWindow.ToWire(motor, value), output[i].Sensed);
            }

            Inferences++;
            return true;
        }

        public void Reset()
        {
            _window.Reset();
            _model.Reset();
            Inferences = 0;
            Fallbacks = 0;
        }

        private static bool Finite(float[] y)
        {
            for (int i = 0; i < y.Length; i++) if (float.IsNaN(y[i]) || float.IsInfinity(y[i])) return false;
            return true;
        }
    }
}
