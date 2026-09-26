from __future__ import annotations
import json, math, os, sys, tempfile, traceback
from pathlib import Path

WORKTREE = Path(r"C:\Users\suyog\.codex\worktrees\restock-r1-runtime\Simulation")
OUT = WORKTREE / "robot_spike" / "evidence" / "isaac_r1_runtime"
SCRATCH = Path(r"C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\isaac_r1_temp")
OUT.mkdir(parents=True, exist_ok=True)
SCRATCH.mkdir(parents=True, exist_ok=True)
os.environ["TMP"] = str(SCRATCH)
os.environ["TEMP"] = str(SCRATCH)
tempfile.tempdir = str(SCRATCH)
sys.path.insert(0, str(WORKTREE))

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "multi_gpu": False})
app_utils = None
result = {"candidate": "1fe5ac5c42466b3ea581b73585e919e0a8e32117", "task": "R1 free-base Isaac import, target response, contact, and reset smoke", "physics_dt_s": 1.0 / 200.0}
try:
    from robot_spike.production.model import load_production_spec
    from robot_spike.production.runtime import IsaacRobotLoader
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.objects import GroundPlane
    from isaacsim.core.experimental.prims import Articulation, RigidPrim, XformPrim
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Usd, UsdGeom, UsdPhysics
    import omni.usd

    spec = load_production_spec(WORKTREE / "robot_spike" / "production")
    loader = IsaacRobotLoader(spec)
    usd_path = loader.import_urdf(OUT / "imported")
    result["generated_usd"] = str(usd_path)
    result["generated_usd_bytes"] = usd_path.stat().st_size

    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    GroundPlane("/World/GroundPlane", positions=[0.0, 0.0, 0.0])
    stage_utils.add_reference_to_stage(usd_path=str(usd_path), path="/World/Robot", variants=[("Physics", "physx")])
    controller = loader.controller("/World/Robot")
    robot = controller.articulation
    result["dof_count"] = len(robot.dof_names)
    result["dof_names"] = list(robot.dof_names)
    result["exact_dof_set"] = set(robot.dof_names) == set(spec.canonical_dof_order)
    result["stage_meters_per_unit"] = float(UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage()))

    link_path_by_name = dict(zip(robot.link_names, robot.link_paths[0]))
    result["foot_link_paths"] = {name: link_path_by_name[name] for name in ("left_ankle_roll_link", "right_ankle_roll_link")}
    left_contact = RigidPrim(link_path_by_name["left_ankle_roll_link"], contact_filter_paths=["/World/GroundPlane"])
    right_contact = RigidPrim(link_path_by_name["right_ankle_roll_link"], contact_filter_paths=["/World/GroundPlane"])
    left_contact.set_enabled_contact_tracking([True])
    right_contact.set_enabled_contact_tracking([True])
    SimulationManager.set_physics_dt(result["physics_dt_s"])
    app_utils.play()
    app.update()
    controller.reset()
    SimulationManager.step()
    app.update()
    for _ in range(240):
        SimulationManager.step()
        if _ % 20 == 0:
            app.update()
    def plain(value):
        if hasattr(value, "numpy"):
            value = value.numpy()
        elif hasattr(value, "cpu"):
            value = value.cpu().numpy()
        if hasattr(value, "tolist"):
            value = value.tolist()
        return value
    def scalar_array(value):
        rows = plain(value)
        while isinstance(rows, list) and len(rows) == 1 and isinstance(rows[0], list):
            rows = rows[0]
        return rows
    def pose():
        p, q = robot.get_world_poses()
        return {"position": scalar_array(p), "orientation_wxyz": scalar_array(q)}
    def speeds():
        lv, av = robot.get_velocities()
        return {"linear_m_s": scalar_array(lv), "angular_rad_s": scalar_array(av)}
    def q_map():
        q = scalar_array(robot.get_dof_positions())
        return {name: float(q[i]) for i, name in enumerate(robot.dof_names)}
    def contact_force(view):
        try:
            return scalar_array(view.get_net_contact_forces(dt=result["physics_dt_s"]))
        except Exception as exc:
            result["contact_measurement_error"] = repr(exc)
            return None
    def norm3(v):
        return math.sqrt(sum(float(x) * float(x) for x in v))

    result["settled_root_pose"] = pose()
    result["settled_root_velocity"] = speeds()
    q0 = q_map()
    result["settled_left_foot_ground_contact_force_n"] = contact_force(left_contact)
    result["settled_right_foot_ground_contact_force_n"] = contact_force(right_contact)
    contact_values = [result["settled_left_foot_ground_contact_force_n"], result["settled_right_foot_ground_contact_force_n"]]
    result["foot_ground_contact_observed"] = any(value is not None and norm3(value) > 0.1 for value in contact_values)
    root_position = result["settled_root_pose"]["position"]
    root_quaternion = result["settled_root_pose"]["orientation_wxyz"]
    result["root_drift_from_reset_m"] = norm3([float(a) - float(b) for a, b in zip(root_position, spec.root_position_m)])
    result["root_tilt_from_upright_deg"] = 2.0 * math.degrees(math.acos(min(1.0, abs(float(root_quaternion[0])))))
    result["neutral_pose_stable"] = result["root_drift_from_reset_m"] < 0.05 and result["root_tilt_from_upright_deg"] < 5.0 and norm3(result["settled_root_velocity"]["linear_m_s"]) < 0.1

    target = 0.20
    spec.validate_targets({"right_shoulder_pitch_joint": target})
    controller.command_joint_positions({"right_shoulder_pitch_joint": target})
    for _ in range(100):
        SimulationManager.step()
        if _ % 20 == 0:
            app.update()
    q1 = q_map()
    result["commanded_joint"] = "right_shoulder_pitch_joint"
    result["command_target_rad"] = target
    result["commanded_joint_delta_rad"] = q1["right_shoulder_pitch_joint"] - q0["right_shoulder_pitch_joint"]
    result["drive_response_observed"] = abs(result["commanded_joint_delta_rad"]) > 0.01

    controller.reset()
    for _ in range(10):
        SimulationManager.step()
    qa, pa = q_map(), pose()
    controller.command_joint_positions({"right_shoulder_pitch_joint": target})
    for _ in range(25):
        SimulationManager.step()
    controller.reset()
    for _ in range(10):
        SimulationManager.step()
    qb, pb = q_map(), pose()
    result["repeat_reset_joint_max_error_rad"] = max(abs(qa[n] - qb[n]) for n in qa)
    result["repeat_reset_root_position_max_error_m"] = max(abs(float(a) - float(b)) for a, b in zip(pa["position"], pb["position"]))
    result["repeat_reset_orientation_max_error"] = max(abs(float(a) - float(b)) for a, b in zip(pa["orientation_wxyz"], pb["orientation_wxyz"]))
    result["deterministic_reset_observed"] = result["repeat_reset_joint_max_error_rad"] < 1e-3 and result["repeat_reset_root_position_max_error_m"] < 1e-3 and result["repeat_reset_orientation_max_error"] < 1e-3
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath("/World/Robot")
    colliders = rigid_bodies = 0
    for prim in Usd.PrimRange(robot_prim):
        colliders += int(prim.HasAPI(UsdPhysics.CollisionAPI))
        rigid_bodies += int(prim.HasAPI(UsdPhysics.RigidBodyAPI))
    result["usd_collision_prim_count"] = colliders
    result["usd_rigid_body_prim_count"] = rigid_bodies
    result["status"] = "completed"
except Exception as exc:
    result["status"] = "failed"
    result["exception"] = repr(exc)
    result["traceback"] = traceback.format_exc()
finally:
    report = OUT / "r1_runtime_smoke.json"
    report.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    try:
        if app_utils is not None:
            app_utils.stop()
    except Exception:
        pass
    app.close()









