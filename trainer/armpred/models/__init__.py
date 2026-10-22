"""The learned correction: torch modules, and the flat ONNX graph the repo will run.

Every model maps one joint's window row plus the arm-wide row to two numbers: a bounded correction
to the prior (rad) and a Laplace log-scale (log rad). Weights are shared across the five joints, so
one model serves all of them and a joint is only its one-hot.
"""
