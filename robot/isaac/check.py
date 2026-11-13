"""Isaac Sim's first check on the GPU machine (docs/adr/0031, `just isaac-check`).

Launches Isaac headless, drops a 5 cm cube from 1 m onto a ground, steps PhysX at 1 kHz, and reports
the three things a real-time bridge depends on:

- Isaac starts on this GPU and driver;
- the cube lands and rests on the ground (its centre half its size above it);
- the physics runs at least as fast as the wall clock at that step.

Prints one JSON line, and exits 0 only when all three hold.
"""

import argparse
import json
import sys
import time

from isaacsim import SimulationApp


def main() -> int:
    parser = argparse.ArgumentParser(description="Isaac Sim's first check (docs/adr/0031)")
    parser.add_argument("--seconds", type=float, default=5.0, help="simulated time to step")
    parser.add_argument("--dt", type=float, default=0.001, help="the physics step, s")
    args = parser.parse_args()
    sys.argv = sys.argv[:1]  # Kit reads the command line too; it gets none of ours

    launch = time.perf_counter()
    app = SimulationApp({"headless": True})
    started = time.perf_counter() - launch
    try:
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        import numpy as np
        from importlib.metadata import version
        from isaacsim.core.experimental.prims import RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import UsdGeom, UsdPhysics

        stage = stage_utils.create_new_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.GetSizeAttr().Set(1.0)
        UsdGeom.XformCommonAPI(ground).SetTranslate((0.0, 0.0, -0.05))
        UsdGeom.XformCommonAPI(ground).SetScale((10.0, 10.0, 0.1))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        size = 0.05
        cube = UsdGeom.Cube.Define(stage, "/World/Cube")
        cube.GetSizeAttr().Set(size)
        UsdGeom.XformCommonAPI(cube).SetTranslate((0.0, 0.0, 1.0))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(0.1)

        SimulationManager.setup_simulation(dt=args.dt, device="cpu")
        app_utils.play()
        app.update()
        body = RigidPrim("/World/Cube")

        steps = int(round(args.seconds / args.dt))
        t0 = time.perf_counter()
        SimulationManager.step(steps=steps)
        wall = time.perf_counter() - t0

        positions, _ = body.get_world_poses()
        z = float(np.asarray(positions.numpy() if hasattr(positions, "numpy") else positions).reshape(-1)[2])
        rested = abs(z - size / 2) < 0.005
        factor = args.seconds / wall if wall > 0 else float("inf")
        print(json.dumps({
            "isaacsim": version("isaacsim"),
            "launch_s": round(started, 1),
            "physics_dt": args.dt,
            "steps": steps,
            "wall_s": round(wall, 3),
            "real_time_factor": round(factor, 1),
            "cube_z_m": round(z, 4),
            "cube_rested": rested,
            "pass": bool(rested and factor >= 1.0),
        }))
        return 0 if rested and factor >= 1.0 else 1
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
