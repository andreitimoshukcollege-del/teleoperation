"""The model's input window (layout v1, contract/MODEL_FORMAT.md).

The C# predictor that runs a trained model rebuilds exactly this from its own calls, so the layout
is a contract: every scale here is a constant, and a change is a new layout id.
"""
