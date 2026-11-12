"""The JetRover's arm in Isaac Sim, standing in for its ROS servo driver (docs/adr/0034 decision 1).

Run on the GPU machine with Isaac's Python (``just isaac-jetrover-host`` does it, J2):

    ~/isaacsim/venv/bin/python -m teleop_isaac.jetrover_bridge --socket /tmp/teleop-jetrover-relay.sock

It binds the Unix datagram socket the JetRover's RobotHost names with ``--relay-socket`` and speaks relay protocol v5, the ROS
relay's own module: targets in (pulse and pulses per second per motor), torque on or off, and feedback out to whoever sent the
last command (before any, to ``--client-socket``, RobotHost's own relay socket, as the real relay sends it). Every millisecond,
paced to the wall clock:

- each servo's own setpoint ramps toward its target over ``|delta| / pulses_per_second`` (``servo.TimedSetpoint``), and the
  joint's PhysX position drive follows it, no harder than the servo's stall torque (decision 3);
- the arm feels gravity, so it sags and slows under load; with torque off the servos' gearboxes hold it where it is and
  commands are ignored until torque comes back, as the real arm's readings show (docs/adr/0034 amendment 11; ``--relaxed
  fall`` instead lets it fall limp);
- the drives, torque limits and servo speed are J3's fit to the real arm (amendment 11; ``just jetrover-sim-fit``);
- PhysX steps once.

Every 20 ms a cycle reads motors 1 to 5 back to back in whole pulses, as the real driver does, and every 20 ms the newest
reading of each goes back, valid while under 0.2 s old; the gripper (10) is never valid (decision 4, ``servo.FeedbackEmulator``). The base is fixed (decision 5).

With ``--scene`` (docs/adr/0034 decision 7, J4) the scene file's surfaces and objects are in the stage too (``world.py``, as
the FR3's), served to RobotHost on a second socket (``--world-socket``) over the arm-server wire's world messages: a Welcome
with no motors and WORLD_INFO after each Hello, WORLD every 20 ms from PhysX, PLACE applied or refused. ``--tap`` copies the
arm's state (an arm-server STATE of joint values) and every WORLD to the renderer.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import socket
import sys
import time
import traceback
from typing import Dict, List, Optional

from . import driver_log
from . import scene as scene_module
from . import servo
from . import wire

HOME = (0.0, 1.0472, -1.5708, 0.5236, 0.0)  # JetRoverCollisionModel's home: yaw, proximal, distal, wrist, roll


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    a = argparse.ArgumentParser(description="The JetRover's arm in Isaac Sim, behind its own RobotHost (docs/adr/0034)")
    a.add_argument("--sim", default=servo.DEFAULT_SIM, help="the sidecar (robot/jetrover/urdf/jetrover.sim.json)")
    a.add_argument("--socket", default="/tmp/teleop-jetrover-relay.sock", help="the relay socket RobotHost's --relay-socket names")
    a.add_argument("--client-socket", default="", help="RobotHost's --local-relay-socket, where feedback goes every 50 ms whether or not "
                                                     "commands come, as the real relay sends it (default: --socket plus .robothost)")
    a.add_argument("--work", default=os.path.expanduser("~/isaacsim/work"), help="where the URDF's USD conversion is kept")
    # J3's fit to the real arm behind the emulated driver and relay (docs/adr/0034 amendment 13, a87cea9): held-out tracking
    # p95 3.12 deg (amendment 11's 21.213 / 4.2426 / 1.5 / 1.5625 without them: 4.32). The fit's 5 ms command delay is not
    # applied live: RobotHost's socket to this bridge is a hop of its own.
    a.add_argument("--stiffness", type=float, default=50.4533, help="joint drive stiffness, Nm/rad: a servo holding its setpoint")
    a.add_argument("--damping", type=float, default=3.2715, help="joint drive damping, Nm s/rad")
    a.add_argument("--effort-scale", type=float, default=0.54433, help="times each servo's stall torque: the drives' force limit")
    a.add_argument("--speed-scale", type=float, default=0.97249, help="times each command's pulses per second: how fast the real "
                                                                     "servos ramp, against what RobotHost asks")
    a.add_argument("--relaxed", choices=("hold", "fall"), default="hold", help="torque off: the gearboxes hold the arm (the real "
                                                                               "arm's readings), or the drives go limp and it falls")
    a.add_argument("--finger-stiffness", type=float, default=2000.0, help="finger drive stiffness, N/m")
    a.add_argument("--finger-damping", type=float, default=40.0, help="finger drive damping, N s/m")
    a.add_argument("--limp-damping", type=float, default=0.02, help="joint damping with torque off (Nm s/rad): a relaxed servo's gearbox")
    a.add_argument("--iterations", type=int, default=16, help="the articulation solver's position iterations")
    a.add_argument("--dt", type=float, default=0.001, help="the physics step, s")
    a.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0: until stopped)")
    a.add_argument("--driver-gate", choices=("on", "off"), default="on",
                   help="the real driver's write gate and single thread in front of the servos (docs/adr/0034 amendment 13); "
                        "off: every target starts its ramp at once, as before")
    a.add_argument("--relay-poll", choices=("on", "off"), default="on",
                   help="RobotHost's messages reach the driver at the ROS relay's 20 ms socket polls, the newest command only "
                        "(relay_node.py, docs/adr/0034 amendment 13); off: as they arrive")
    a.add_argument("--replay-cycles", choices=("recorded", "emulated"), default="recorded",
                   help="--replay: read at each session's own driver cycles (its driver.jlog), or run the driver's cycles from "
                        "the first reading (before amendment 13)")
    a.add_argument("--torque-off-at-start", action="store_true", help="start relaxed, as the robot does after ROBOT OFF")
    a.add_argument("--scene", default="", help="a scene file (core/Scenes/*.json) for the robot's bench: its surfaces and objects")
    a.add_argument("--world-socket", default="", help="where RobotHost's world client talks (default: --socket plus .world)")
    a.add_argument("--world-every", type=int, default=20, help="send the objects' poses every this many steps")
    a.add_argument("--tap", default="", help="the renderer's socket: a copy of the arm's state every 4 steps and of every WORLD")
    a.add_argument("--record-control", default="", help="RobotHost's recording control file (its .active): while it names a session, "
                                                        "write that session's driver.jlog, the reads and moves this stand-in driver did "
                                                        "(driver_log.py, docs/adr/0035 N4)")
    a.add_argument("--gui", action="store_true", help="open Isaac's window (needs a display)")
    a.add_argument("--replay", default="", help="sim against real (docs/adr/0034 J3): a sessions file from just jetrover-sim-fit; "
                                                "replay it headless and unpaced, score it, fit, and exit. No sockets")
    a.add_argument("--fit-seconds", type=float, default=90.0, help="--replay: seconds of each fit session per evaluation")
    a.add_argument("--fit-rounds", type=int, default=2, help="--replay: the coordinate search's rounds (0: score only)")
    a.add_argument("--out", default="", help="--replay: where the result goes (JSON)")
    return a.parse_args(argv)


def urdf_digest(urdf: str) -> str:
    """The sha256 of the URDF and of every mesh file it names, in order: what a conversion is made of. The URDF alone is not
    enough: re-exported meshes keep their names (docs/adr/0034 amendment 7, the jaws swapped and Isaac kept the old ones)."""
    import re

    h = hashlib.sha256()
    with open(urdf, "rb") as f:
        text = f.read()
    h.update(text)
    base = os.path.dirname(os.path.abspath(urdf))
    for name in re.findall(rb'filename="([^"]+)"', text):
        path = os.path.join(base, name.decode())
        h.update(name)
        if os.path.isfile(path):
            with open(path, "rb") as m:
                h.update(hashlib.sha256(m.read()).digest())
    return h.hexdigest()[:12]


def convert_urdf(urdf: str, work: str) -> str:
    """The URDF as Isaac's USD, converted once per content: the URDF and its meshes (:func:`urdf_digest` names the folder)."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    digest = urdf_digest(urdf)
    out = os.path.join(work, f"jetrover-{digest}")
    name = os.path.splitext(os.path.basename(urdf))[0]
    usd = os.path.join(out, name, f"{name}.usda")
    if not os.path.isfile(usd):
        os.makedirs(out, exist_ok=True)
        usd = URDFImporter(URDFImporterConfig(urdf_path=urdf, usd_path=out, fix_base=True,
                                              joint_drive_type="force", joint_target_type="position")).import_urdf()
    return usd


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    sys.argv = sys.argv[:1]  # Kit reads the command line too; it gets none of ours
    model = servo.SimModel.load(args.sim)
    rp = servo.relay_protocol()
    urdf = os.path.join(os.path.dirname(os.path.abspath(args.sim)), model.urdf)
    pulses = servo.home_pulses(model, HOME)
    setpoints: Dict[int, servo.TimedSetpoint] = {m: servo.TimedSetpoint(p) for m, p in pulses.items()}
    feedback = servo.FeedbackEmulator(model)
    scene = scene_module.load(args.scene) if args.scene else None
    if scene is not None and scene.robot_profile != "jetrover":
        raise SystemExit(f"error: the scene {args.scene} is for profile {scene.robot_profile!r}, not 'jetrover'")
    scene_sha = __import__("hashlib").sha256(open(args.scene, "rb").read()).digest() if scene is not None else b""

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not args.gui})
    try:
        import carb
        import numpy as np
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from importlib.metadata import version
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import PhysxSchema, UsdPhysics

        usd = convert_urdf(urdf, args.work)
        stage_utils.open_stage(usd)
        while stage_utils.is_stage_loading():
            app.update()
        stage = stage_utils.get_current_stage()
        robot = stage.GetDefaultPrim()
        robot.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
        for _ in range(3):
            app.update()
        roots = [str(x.GetPath()) for x in stage.Traverse() if x.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f"expected one articulation in {usd}, found {roots}")
        art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(roots[0]))
        art_api.CreateSolverPositionIterationCountAttr(args.iterations)
        art_api.CreateSolverVelocityIterationCountAttr(1)
        art_api.CreateEnabledSelfCollisionsAttr(False)
        # Gravity stays on for the arm (decision 3): the real servos hold it up, and sag under load.
        from .world import IsaacWorld

        world = IsaacWorld(scene, str(robot.GetPath())) if scene is not None else None
        if world is not None:
            world.build(stage)
            fixed = world.fixed_bodies(stage)
            print(f"[jetrover] the robot's fixed bodies ignore the surfaces: {', '.join(b.rsplit('/', 1)[-1] for b in fixed)}", flush=True)

        settings = carb.settings.get_settings()
        for key in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/updateParticlesToUsd", "/physics/updateResidualsToUsd"):
            settings.set(key, False)
        SimulationManager.setup_simulation(dt=args.dt, device="cpu")
        app_utils.play()
        app.update()

        art = Articulation(roots[0])
        names = list(art.dof_names)
        index = {n: i for i, n in enumerate(names)}
        missing = [m.joint for m in model.motors.values() if not m.is_gripper and m.joint not in index]
        missing += [f for m in model.motors.values() for f in m.fingers if f not in index]
        if missing:
            raise RuntimeError(f"Isaac's articulation lacks the sidecar's joints: {', '.join(missing)} (has {', '.join(names)})")
        n = len(names)
        finger = np.array([("finger" in nm or nm.startswith("gripper")) for nm in names])
        efforts = np.asarray(art.get_dof_max_efforts().numpy()).reshape(-1).copy()
        for m in model.motors.values():
            if not m.is_gripper:
                efforts[index[m.joint]] = m.stall_torque
        art.set_dof_max_efforts(efforts.reshape(1, -1))
        if world is not None:
            world.attach()

        target = np.zeros((1, n))

        def apply_setpoints() -> None:
            for mid, sp in setpoints.items():
                m = model.motors[mid]
                v = model.value(mid, sp.pulse)
                if m.is_gripper:
                    for f in m.fingers:
                        target[0, index[f]] = v
                else:
                    target[0, index[m.joint]] = v

        apply_setpoints()
        art.set_dof_positions(target)
        art.set_dof_position_targets(target)
        # One servo arm for the live loop and the replay (J3): what was fitted is what runs.
        from . import simfit
        arm = IsaacServoSim(args, model, art, setpoints, index, finger, efforts, target, apply_setpoints, np, SimulationManager)
        live = simfit.Params(stiffness=args.stiffness, damping=args.damping, effort_scale=args.effort_scale, speed_scale=args.speed_scale)
        arm.reset({mid: sp.pulse for mid, sp in setpoints.items()}, live)
        if args.torque_off_at_start:
            arm.torque(False)
        if args.replay:
            return replay_and_fit(args, model, arm, live)
        torque = arm.on

        description = (f"Isaac Sim {version('isaacsim')}, PhysX {1 / args.dt:.0f} Hz on CPU, servo drives {args.stiffness:g}/{args.damping:g} "
                       f"within {args.effort_scale:g} x {'/'.join(f'{m.stall_torque:g}' for m in model.motors.values() if not m.is_gripper)} Nm, "
                       f"speed x{args.speed_scale:g}, relaxed: {args.relaxed}, arm gravity on, "
                       f"timed-ramp setpoints, driver gate {args.driver_gate}, relay poll {args.relay_poll}, feedback motors {','.join(str(x) for x in feedback.order)} read back to back ({feedback.read_seconds * 1000:.1f} ms each) "
                       f"every {feedback.cycle_seconds * 1000:.0f} ms, sent every {model.feedback_seconds * 1000:.0f} ms")
        if os.path.exists(args.socket):
            os.unlink(args.socket)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.bind(args.socket)
        sock.setblocking(False)
        wsock = None
        world_path = args.world_socket or args.socket + ".world"
        if world is not None:
            movable = sum(1 for o in scene.objects if o.movable)
            world_description = (f"Isaac Sim {version('isaacsim')}, the JetRover's world: scene {scene.name} ({scene_sha.hex()[:12]}, "
                                 f"{len(scene.objects) - movable} fixed and {movable} movable objects)")[:wire.MAX_DESCRIPTION_BYTES]
            description += f", scene {scene.name} ({scene_sha.hex()[:12]}, {len(scene.objects) - movable} fixed and {movable} movable objects)"
            if os.path.exists(world_path):
                os.unlink(world_path)
            wsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            wsock.bind(world_path)
            wsock.setblocking(False)
        print(f"[jetrover] {description}; {n} DOFs ({', '.join(names)}); torque {'on' if torque else 'off'}; listening on {args.socket}"
              + (f", world on {world_path}" if wsock is not None else ""), flush=True)
        world_client = None
        sizes_due, last_sizes = True, 0  # SIZES at start, on change and once a second (docs/adr/0034 amendment 6)
        wseq = [0]

        def next_wseq() -> int:
            wseq[0] += 1
            return wseq[0]

        tapped = tap_dropped = 0

        def tap(message: bytes) -> None:
            nonlocal tapped, tap_dropped
            if not args.tap:
                return
            try:
                sock.sendto(message, args.tap)
                tapped += 1
            except OSError:
                tap_dropped += 1

        motor_order = sorted(model.motors)

        # The driver's half of a recorded session (docs/adr/0035 N4): what this stand-in driver read and moved.
        dlog = None
        if args.record_control:
            from . import isaac_session

            dlog = driver_log.DriverLog(isaac_session.IsaacSession(args.record_control, streams=[], writer="driver"),
                                        feedback.read_seconds, feedback.order)

        def switch_torque(on: bool) -> None:
            nonlocal torque
            if on != torque:
                torque = on
                arm.torque(on)
                print(f"[jetrover] torque {'on' if on else 'off'}", flush=True)

        def ungated(mid: int, goal: float, pps: float) -> None:
            # --driver-gate off: the bridge before amendment 13, each servo's ramp at the command's own speed.
            sp = setpoints.get(mid)
            if dlog is not None and sp is not None and (arm.on or not arm.hold_relaxed) and abs(goal - sp.target) >= 0.5:
                dlog.moved(time.monotonic_ns(), mid, goal, sp.pulse, pps * arm.speed_scale)
            arm.command(mid, goal, pps)

        # The driver between the relay and the servos (docs/adr/0034 amendment 13): its write gate and its single thread.
        driver = servo.DriverEmulator(model, feedback, servo_move=arm.write, servo_command=ungated, servo_torque=switch_torque,
                                      gate=args.driver_gate == "on", log=dlog.driver if dlog is not None else None,
                                      clock_ns=time.monotonic_ns)

        stopping = []
        signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
        signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
        client = args.client_socket or args.socket + ".robothost"
        commands = torques = rejected_total = sent = 0
        steps = late = 0
        step_ns = int(round(args.dt * 1e9))
        start_ns = next_tick = time.monotonic_ns()
        stop_at = start_ns + int(args.seconds * 1e9) if args.seconds > 0 else None
        # The ROS relay drains its socket every 20 ms and acts on the newest command (docs/adr/0034 amendment 13).
        poll_steps = max(1, int(round(servo.RelayEmulator.POLL_SECONDS / args.dt))) if args.relay_poll == "on" else 1
        datagrams = []
        report_at = start_ns + 5_000_000_000

        while not stopping and (stop_at is None or time.monotonic_ns() < stop_at):
            while True:
                try:
                    data, sender = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                datagrams.append(data)
                if sender:
                    client = sender  # whoever commands is where feedback goes
            if datagrams and steps % poll_steps == 0:
                batch, datagrams = datagrams, []
                actions, rejected = rp.sort_datagrams(batch)
                rejected_total += rejected
                for kind, payload in actions:
                    if kind == "torque":
                        torques += 1
                        driver.torque(steps * args.dt, bool(payload))
                    else:
                        commands += 1
                        driver.command(steps * args.dt, payload)

            while wsock is not None:
                try:
                    data, sender = wsock.recvfrom(wire.MAX_BYTES)
                except BlockingIOError:
                    break
                kind = wire.kind_of(data)
                if kind == wire.HELLO and wire.decode_hello(data) is not None:
                    hello = wire.decode_hello(data)
                    if hello.profile != "jetrover":
                        print(f"[jetrover] a world hello for profile {hello.profile!r}: not answered", flush=True)
                        continue
                    if sender != world_client:
                        print(f"[jetrover] RobotHost's world client at {sender}", flush=True)
                    world_client = sender
                    try:
                        wsock.sendto(wire.encode_welcome(next_wseq(), wire.SERVER_ISAAC, time.monotonic_ns(), [], world_description), world_client)
                        wsock.sendto(wire.encode_world_info(next_wseq(), scene_sha, [o.id for o in scene.objects]), world_client)
                        sizes_due = True
                    except OSError:
                        world_client = None
                elif kind == wire.PLACE and sender == world_client:
                    place = wire.decode_place(data)
                    if place is not None:
                        _, request, cause, object_id, pose = place
                        outcome, now_pose = world.place(cause, object_id, pose)
                        if outcome != wire.APPLIED or cause != wire.CAUSE_LAYOUT:
                            print(f"[jetrover] place {object_id} (cause {cause}): outcome {outcome}", flush=True)
                        try:
                            wsock.sendto(wire.encode_placed(next_wseq(), request, object_id, outcome, world.model.revision, now_pose), world_client)
                        except OSError:
                            pass
                elif kind == wire.RESIZE and sender == world_client:
                    rz = wire.decode_resize(data)
                    if rz is not None:
                        _, request, object_id, scale = rz
                        outcome, now_pose = world.resize(object_id, scale)
                        print(f"[jetrover] resize {object_id} to {scale:.3g}x: outcome {outcome}", flush=True)
                        sizes_due = sizes_due or outcome == wire.APPLIED
                        try:
                            wsock.sendto(wire.encode_placed(next_wseq(), request, object_id, outcome, world.model.revision, now_pose), world_client)
                        except OSError:
                            pass

            arm.step()
            steps += 1
            now_s = steps * args.dt

            if args.tap and steps % 4 == 0:
                # The renderer poses the arm from these: every motor's joint value (radians; the gripper's opening 0..1).
                q = np.asarray(art.get_dof_positions().numpy()).reshape(-1)
                report = []
                for m_id in motor_order:
                    m = model.motors[m_id]
                    if m.is_gripper:
                        v = model.finger_fraction(m_id, float(q[index[m.fingers[0]]]))
                    else:
                        v = float(q[index[m.joint]])
                    report.append((m_id, float(v), 0.0, float(v)))
                tap(wire.encode_state(next_wseq(), time.monotonic_ns(), steps * step_ns, wire.FLAG_ENABLED if torque else 0, report))
            if world is not None and (world_client is not None or args.tap) and (sizes_due or steps - last_sizes >= round(1.0 / args.dt)):
                sizes_due, last_sizes = False, steps
                message = world.model.sizes_message(next_wseq())
                if world_client is not None:
                    try:
                        wsock.sendto(message, world_client)
                    except OSError:
                        pass
                tap(message)
            if world is not None and (world_client is not None or args.tap) and steps % args.world_every == 0:
                objects = world.read()
                for message in world.model.world_messages(next_wseq, time.monotonic_ns(), objects):
                    if world_client is not None:
                        try:
                            wsock.sendto(message, world_client)
                        except OSError:
                            pass
                    tap(message)

            mid = driver.due_read(now_s)
            if mid is not None:
                q = np.asarray(art.get_dof_positions().numpy()).reshape(-1)
                reading = model.pulse(mid, q[index[model.motors[mid].joint]])
                driver.record(mid, reading, now_s)
                if dlog is not None:
                    dlog.read(time.monotonic_ns(), mid, reading)
            entries = feedback.due_feedback(now_s)
            if entries is not None:
                try:
                    sock.sendto(rp.encode_feedback(entries), client)
                    sent += 1
                except OSError:
                    pass  # RobotHost is not there (yet): the real relay's feedback is lost the same way

            next_tick += step_ns
            now = time.monotonic_ns()
            behind = now - next_tick
            if behind > step_ns:
                late += 1
                if behind > 50 * step_ns:
                    next_tick = now
            else:
                while next_tick - time.monotonic_ns() > 300_000:
                    time.sleep(0.0002)
                while time.monotonic_ns() < next_tick:
                    pass
            if now >= report_at:
                report_at = now + 5_000_000_000
                print(f"[jetrover] {steps} steps, {late} late; {commands} commands, {torques} torque switches, {rejected_total} rejected; "
                      f"{sent} feedback sent; torque {'on' if torque else 'off'}", flush=True)

        if dlog is not None:
            dlog.session.close()
        sock.close()
        os.unlink(args.socket)
        if wsock is not None:
            wsock.close()
            os.unlink(world_path)
        print(f"[jetrover] stopped after {steps} steps, {late} late; {commands} commands, {sent} feedback sent", flush=True)
        return 0
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        app.close()


class IsaacServoSim:
    """:class:`simfit.ServoSim` over the bridge's own articulation and setpoints: exactly what RobotHost drives, unpaced."""

    def __init__(self, args, model, art, setpoints, index, finger, efforts, target, apply_setpoints, np, sim_manager):
        self.dt = args.dt
        self.args, self.model, self.art, self.setpoints, self.index = args, model, art, setpoints, index
        self.finger, self.efforts, self.target, self.apply_setpoints, self.np, self.sm = finger, efforts, target, apply_setpoints, np, sim_manager
        self.on = True
        self.q = None
        self.kp_on = self.kd_on = None
        self.speed_scale = 1.0
        self.hold_relaxed = getattr(args, "relaxed", "hold") == "hold"

    def _q(self):
        if self.q is None:
            self.q = self.np.asarray(self.art.get_dof_positions().numpy()).reshape(-1)
        return self.q

    def reset(self, pulses, params) -> None:
        np = self.np
        n = len(self.finger)
        self.kp_on = np.where(self.finger, self.args.finger_stiffness, params.stiffness).reshape(1, -1)
        self.kd_on = np.where(self.finger, self.args.finger_damping, params.damping).reshape(1, -1)
        self.kp_off = np.zeros((1, n))
        self.kd_off = np.where(self.finger, self.args.finger_damping, self.args.limp_damping).reshape(1, -1)
        self.speed_scale = params.speed_scale
        e = self.efforts.copy()
        for m in self.model.motors.values():
            if not m.is_gripper:
                e[self.index[m.joint]] = m.stall_torque * params.effort_scale
        self.art.set_dof_max_efforts(e.reshape(1, -1))
        for mid, sp in self.setpoints.items():
            sp.hold(pulses[mid])
        self.apply_setpoints()
        self.art.set_dof_positions(self.target)
        self.art.set_dof_velocities(np.zeros_like(self.target))
        self.art.set_dof_position_targets(self.target)
        self.on = True
        self.art.set_dof_gains(self.kp_on, self.kd_on)
        self.q = None

    def write(self, motor, pulse, seconds) -> None:
        """A board move packet's servo (docs/adr/0034 amendment 13): from where its ramp stands to ``pulse`` over the packet's
        duration, shortened by the fitted speed scale; ignored while relaxed and held."""
        if motor in self.setpoints and (self.on or not self.hold_relaxed):
            self.setpoints[motor].move_over(pulse, seconds / self.speed_scale)

    def command(self, motor, pulse, pps) -> None:
        """A relay target, ramped at its pulses per second times the fitted speed scale; ignored while relaxed and held, as a
        servo with torque off ignores its position."""
        if motor in self.setpoints and (self.on or not self.hold_relaxed):
            self.setpoints[motor].move(pulse, pps * self.speed_scale)

    def torque(self, on) -> None:
        """Held (the default): torque off freezes each servo's setpoint where its ramp stands, and the arm stays where it is (the
        real gearboxes, amendment 11); commands are ignored; torque on goes on from there. Re-targeting at the measured place
        instead would drop the drives' preload against gravity and let the arm sag again. Falling: torque off makes the drives
        limp, and torque on holds where the arm lies then."""
        if on == self.on:
            return
        self.on = on
        if self.hold_relaxed:
            if not on:
                for sp in self.setpoints.values():
                    sp.hold(sp.pulse)
            return
        if on:
            q = self._q()
            for mid, sp in self.setpoints.items():
                m = self.model.motors[mid]
                sp.hold(self.model.pulse(mid, q[self.index[m.fingers[0] if m.is_gripper else m.joint]]))
        self.art.set_dof_gains(self.kp_on if on else self.kp_off, self.kd_on if on else self.kd_off)

    def step(self) -> None:
        for sp in self.setpoints.values():
            sp.step(self.dt)
        if self.on or self.hold_relaxed:
            self.apply_setpoints()
            self.art.set_dof_position_targets(self.target)
        self.sm.step(steps=1)
        self.q = None

    def read(self, motor) -> float:
        m = self.model.motors[motor]
        return self.model.pulse(motor, self._q()[self.index[m.fingers[0] if m.is_gripper else m.joint]])


def replay_and_fit(args, model, sim, default) -> int:
    """Sim against real (docs/adr/0034 decision 6, J3): score the default drives on every session, fit on the ``fit``
    sessions, score the fitted ones on every session, write the result. docs/metrics.md section 10."""
    from . import simfit

    with open(args.replay, encoding="utf-8") as f:
        doc = json.load(f)
    replayer = simfit.Replayer(model, gate=args.driver_gate == "on", cycles=args.replay_cycles, relay=args.relay_poll == "on")

    def score_all(p, label):
        out = {}
        for group in ("fit", "heldout"):
            pooled = simfit.Score()
            per = {}
            for s in doc[group]:
                t = time.monotonic()
                sc = replayer.run(s, sim, p)
                pooled.add(sc)
                per[s["name"]] = sc.summary()
                tr = per[s["name"]]["sim_tracking_error_deg"]["pooled1to4"]
                print(f"[sim-fit] {label} {group} {s['name']}: tracking p50 {tr['p50']} p95 {tr['p95']} deg over {tr['n']} readings "
                      f"({time.monotonic() - t:.0f} s)", flush=True)
            out[group] = {"pooled": pooled.summary(), "sessions": per}
        return out

    result = {"format": "jetrover-sim-fit", "version": 1, "split": {g: [s["name"] for s in doc[g]] for g in ("fit", "heldout")},
              "fitSeconds": args.fit_seconds, "driverGate": args.driver_gate, "replayCycles": args.replay_cycles,
              "relayPoll": args.relay_poll, "default": {"params": default.to_dict()}}
    result["default"]["scores"] = score_all(default, "default")

    def evaluate(p):
        total = simfit.Score()
        for s in doc["fit"]:
            total.add(replayer.run(s, sim, p, seconds=args.fit_seconds))
        return total.objective()

    if args.fit_rounds > 0:
        best, objective, history = simfit.fit(evaluate, default, rounds=args.fit_rounds, log=lambda s: print(s, flush=True))
        result["fitted"] = {"params": best.to_dict(), "objective": objective, "history": history, "scores": score_all(best, "fitted")}
    if args.out:
        simfit.write_result(args.out, result)
    for k in ("default", "fitted"):
        if k in result:
            h = result[k]["scores"]["heldout"]["pooled"]["sim_tracking_error_deg"]["pooled1to4"]
            print(f"[sim-fit] {k} {result[k]['params']}: held-out tracking p50 {h['p50']} p95 {h['p95']} deg", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
