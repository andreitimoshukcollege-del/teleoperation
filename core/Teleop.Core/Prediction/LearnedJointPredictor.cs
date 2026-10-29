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
    /// <para><b>Layout v4</b> (a serial-chain robot such as the FR3 Duo; docs/adr/0028) is used when the config carries
    /// <see cref="JointPredictorConfig.ChainLayout"/>. The window is then <see cref="LearnedJointWindowV4"/>, on top of
    /// <c>franka-replay</c> (the config's <see cref="JointPredictorConfig.Shaper"/>), and every row of the layout is
    /// corrected. The model must take that window's length: a layout v3 model is refused, and so is the reverse.</para>
    ///
    /// <para>Falls back to the prior's own prediction, and counts it (trainer/contract/MODEL_FORMAT.md):
    /// no fresh reading of every arm joint yet; the newest reading more than 1 s old; the backend failing;
    /// an output that is not finite. Deterministic for a deterministic backend; allocation-free.</para>
    /// </summary>
    public sealed class LearnedJointPredictor : ICommandAwarePredictor
    {
        private readonly IInferenceBackend _model;
        private readonly LearnedJointWindow? _window;
        private readonly LearnedJointWindowV4? _window4;
        private readonly float[] _x;
        private readonly float[] _y;
        private readonly double[] _prior;

        public LearnedJointPredictor(JointPredictorConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            _model = config.Model ?? throw new ArgumentException("the learned predictor needs a model (JointPredictorConfig.Model)", nameof(config));
            int inputs, rows;
            string layout;
            if (config.ChainLayout != null)
            {
                _window4 = new LearnedJointWindowV4(config);
                inputs = _window4.InputLength;
                rows = _window4.Rows;
                layout = "v4";
            }
            else
            {
                _window = new LearnedJointWindow(config);
                inputs = LearnedJointWindow.InputLength;
                rows = LearnedJointWindow.JointCount;
                layout = "v3";
            }

            if (_model.InputLength != inputs || _model.OutputLength != 2 * rows)
            {
                throw new ArgumentException(
                    $"model {_model.ModelId}: {_model.InputLength} in, {_model.OutputLength} out; layout {layout} here is {inputs} in, {2 * rows} out",
                    nameof(config));
            }

            _x = new float[inputs];
            _y = new float[_model.OutputLength];
            _prior = new double[rows];
        }

        /// <summary>Predictions answered by the model.</summary>
        public long Inferences { get; private set; }

        /// <summary>Predictions answered by the prior instead: no fresh reading yet, stale, or the model failed.</summary>
        public long Fallbacks { get; private set; }

        public string ModelId => _model.ModelId;

        /// <summary>The window's layout: <c>v3</c> (the JetRover's) or <c>v4</c> (a serial-chain robot's).</summary>
        public string Layout => _window4 != null ? LearnedJointWindowV4.LayoutId : "v3";

        public PredictorDiagnostics Diagnostics => _window4 != null ? _window4.Prior.Diagnostics : _window!.Prior.Diagnostics;

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets, float speedRadPerSec)
        {
            // Speed-blind: the model's prior is given no speed.
            if (_window4 != null) _window4.ObserveCommand(effectiveTicks, targets);
            else _window!.ObserveCommand(effectiveTicks, targets);
        }

        public void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints)
        {
            if (_window4 != null) _window4.ObserveState(sensedTicks, joints);
            else _window!.ObserveState(sensedTicks, joints);
        }

        public bool TryPredict(long targetTicks, Span<JointSample> output, out int count)
        {
            bool predicted = _window4 != null
                ? _window4.Prior.TryPredict(targetTicks, output, out count)
                : _window!.Prior.TryPredict(targetTicks, output, out count);
            if (!predicted)
            {
                return false;
            }

            bool valid;
            bool built = _window4 != null ? _window4.TryBuild(targetTicks, _x, _prior, out valid) : _window!.TryBuild(targetTicks, _x, _prior, out valid);
            if (!built || !valid || !_model.TryRun(_x, _y) || !Finite(_y))
            {
                Fallbacks++;
                return true;
            }

            for (int i = 0; i < count; i++)
            {
                byte motor = output[i].MotorId;
                if (_window4 != null)
                {
                    int r = _window4.Layout.RowOf(motor);
                    if (r < 0) continue;
                    output[i] = new JointSample(motor, (float)_window4.ToWire(r, _prior[r] + _y[2 * r]), output[i].Sensed);
                    continue;
                }

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
            if (_window4 != null) _window4.Reset();
            else _window!.Reset();
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
