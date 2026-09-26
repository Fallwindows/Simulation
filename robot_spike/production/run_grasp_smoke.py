"""Headless R4 physical-grasp smoke harness with durable evidence.

Importing this module is CPU safe.  ``run_isaac`` is the sole boundary that
starts SimulationApp and is intentionally exercised only by a later runtime
validation assignment.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Callable, Protocol

from robot_spike.production.arm_reach import ARM_DOF_NAMES, RightArmKinematics, ToolPose
from robot_spike.production.isaac_grasp import (
    Isaac61GraspFeedbackAdapter,
    IsaacArmApproachPort,
    IsaacArmLiftPort,
    IsaacContactBindings,
)
from robot_spike.production.model import canonical_text_sha256, load_production_spec
from robot_spike.production.physical_grasp import (
    HAND_DOF_NAMES,
    GraspLimits,
    GraspPhase,
    PhysicalGraspController,
    REQUIRED_CONTACT_LINKS,
    RobotContactBodyMap,
)
from robot_spike.production.runtime import ArticulationController, IsaacRobotLoader
from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.restocking_layout import build_restocking_layout


ACCEPTANCE_SPEC_SHA256 = "7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad"
SOURCE_URDF_CANONICAL_SHA256 = "25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea"
PRODUCTION_URDF_CANONICAL_SHA256 = "b605c9a54f4a8494d333fd7cc5a20de3b4c76ffc83e33bd8027e6bafb7a1f59c"
BASELINE_SCENARIO_SHA256 = "12d41a428dfe0e82da60584da9595b1c74090b33f375cc8be60b237941d8a5d0"
ROBOT_CONFIG_SHA256 = "b6b2a0a524b65e683a69324debd212f2e91b7e446f8141d2b11f6ab75b473fe7"
ISAAC_BUILD = "6.1.0-rc.26+release.49347.2d230af4.gl"
WAIST_ORIGIN_IN_ROOT_M = (-0.052, 0.0, 0.074755)
PREGRASP_ARM_JOINTS_RAD = dict(
    zip(ARM_DOF_NAMES, (-0.224, 0.893, 0.790, -1.558, 0.061, -0.463))
)
PREGRASP_MAXIMUM_DURATION_S = 3.0
PREGRASP_CONFIRMATION_SAMPLES = 3


class PhysicsStepper(Protocol):
    def step(self) -> None: ...


class AdvancingLiftPort(Protocol):
    def advance(self) -> bool: ...


class AdvancingApproachPort(Protocol):
    last_error: str | None

    def request_approach(self, target: ToolPose, maximum_duration_s: float) -> bool: ...

    def advance(self) -> bool: ...

    def target_reached(self) -> bool: ...

    def diagnostics(self) -> dict[str, object]: ...


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=repo, check=True, capture_output=True, text=True
    ).stdout.strip()


def identity_preflight_receipt(
    repo: Path,
    acceptance_spec: Path,
    expected_candidate_sha: str,
    expected_candidate_tree_sha: str,
    isaac_root: Path = Path("C:/isaacsim"),
) -> dict[str, object]:
    """Return a complete expected/observed identity receipt without raising."""

    production_root = repo / "robot_spike" / "production"
    source_path = repo / "robot_spike" / "asimov_orcahand_right.urdf"
    production_path = production_root / "asimov_orcahand_restocking.urdf"
    scenario_path = repo / "config" / "scenarios" / "baseline_straight.yaml"
    robot_config_path = production_root / "robot_config.json"
    isaac_version_path = isaac_root / "VERSION"
    expected = {
        "candidate_commit": expected_candidate_sha,
        "candidate_tree": expected_candidate_tree_sha,
        "acceptance_spec_sha256": ACCEPTANCE_SPEC_SHA256,
        "combined_source_urdf_canonical_sha256": SOURCE_URDF_CANONICAL_SHA256,
        "production_urdf_canonical_sha256": PRODUCTION_URDF_CANONICAL_SHA256,
        "scenario_sha256": BASELINE_SCENARIO_SHA256,
        "robot_config_sha256": ROBOT_CONFIG_SHA256,
        "isaac_build": ISAAC_BUILD,
    }
    observed: dict[str, object] = {
        "acceptance_spec_path": str(acceptance_spec.resolve()),
        "combined_source_urdf_path": str(source_path.resolve()),
        "production_urdf_path": str(production_path.resolve()),
        "scenario_path": str(scenario_path.resolve()),
        "robot_config_path": str(robot_config_path.resolve()),
        "isaac_root": str(isaac_root.resolve()),
        "isaac_version_path": str(isaac_version_path.resolve()),
    }
    errors: list[str] = []

    def observe(name: str, operation) -> None:
        try:
            observed[name] = operation()
        except Exception as exc:
            observed[name] = None
            errors.append(f"{name}: {exc}")

    observe("candidate_commit", lambda: _git(repo, "rev-parse", "HEAD"))
    observe("candidate_tree", lambda: _git(repo, "rev-parse", "HEAD^{tree}"))
    observe("worktree_clean", lambda: not bool(_git(repo, "status", "--porcelain")))
    observe("acceptance_spec_sha256", lambda: _sha256(acceptance_spec))
    observe(
        "combined_source_urdf_canonical_sha256",
        lambda: canonical_text_sha256(source_path),
    )
    observe(
        "production_urdf_canonical_sha256",
        lambda: canonical_text_sha256(production_path),
    )
    observe("production_urdf_byte_sha256", lambda: _sha256(production_path))
    observe("scenario_sha256", lambda: _sha256(scenario_path))
    observe("robot_config_sha256", lambda: _sha256(robot_config_path))
    observe(
        "isaac_build",
        lambda: isaac_version_path.read_text(encoding="utf-8").strip(),
    )

    mismatches = [
        name for name, value in expected.items() if observed.get(name) != value
    ]
    if observed.get("worktree_clean") is not True:
        mismatches.append("worktree_clean")
    mismatches.extend(errors)
    return {
        "status": "pass" if not mismatches else "fail",
        "expected": expected,
        "observed": observed,
        "mismatches": mismatches,
    }


def write_durable_json(path: Path, value: object) -> None:
    """Atomically replace a report and fsync both file and containing directory."""

    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, indent=2, sort_keys=True) + "\n"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    if os.name == "nt":
        # Windows does not expose a portable directory handle suitable for
        # fsync.  The flushed temporary plus atomic replace is the strongest
        # available local-filesystem boundary here.
        return
    try:
        directory_fd = os.open(path.parent, os.O_RDONLY)
    except (AttributeError, OSError):
        return
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


@dataclass(frozen=True)
class PickupResetPlan:
    root_position_m: tuple[float, float, float]
    root_orientation_wxyz: tuple[float, float, float, float]
    arm_target: ToolPose
    predicted_palm_position_world_m: tuple[float, float, float]
    product_position_world_m: tuple[float, float, float]

    def evidence(self) -> dict[str, object]:
        return {
            "root_position_m": self.root_position_m,
            "root_orientation_wxyz": self.root_orientation_wxyz,
            "arm_target_position_m_in_waist": self.arm_target.position_m,
            "arm_target_orientation_xyzw_in_waist": self.arm_target.orientation_xyzw,
            "predicted_palm_position_world_m": self.predicted_palm_position_world_m,
            "product_position_world_m": self.product_position_world_m,
            "source": "layout product pose plus validated URDF/R3 forward kinematics",
            "application": "sole explicit robot reset before smoke physics",
        }


def pickup_reset_plan(spec, layout, kinematics: RightArmKinematics) -> PickupResetPlan:
    """Place the stationary base outside the pickup board and aim the palm inward."""

    goal = spec.validate_targets(PREGRASP_ARM_JOINTS_RAD)
    target = kinematics.forward({name: goal[name] for name in ARM_DOF_NAMES})
    x, y, z, w = target.orientation_xyzw
    palm_approach_x = 2.0 * (x * z + y * w)
    palm_approach_y = 2.0 * (y * z - x * w)
    if math.hypot(palm_approach_x, palm_approach_y) <= 1.0e-9:
        raise RuntimeError("pregrasp palm approach axis has no horizontal projection")
    local_x = WAIST_ORIGIN_IN_ROOT_M[0] + target.position_m[0]
    local_y = WAIST_ORIGIN_IN_ROOT_M[1] + target.position_m[1]
    if math.hypot(local_x, local_y) <= 1.0e-9:
        raise RuntimeError("pregrasp arm target has no horizontal reach")
    # Put the pelvis on the open, negative-y side of the pickup board by
    # rotating the root-to-palm reach toward +Y. The palm approach axis is then
    # used to select the corresponding product face.
    yaw = math.pi / 2.0 - math.atan2(local_y, local_x)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotated_x = cosine * local_x - sine * local_y
    rotated_y = sine * local_x + cosine * local_y
    approach_x = cosine * palm_approach_x - sine * palm_approach_y
    approach_y = sine * palm_approach_x + cosine * palm_approach_y
    approach_norm = math.hypot(approach_x, approach_y)
    approach_x /= approach_norm
    approach_y /= approach_norm
    product = tuple(float(value) for value in layout.product.source_reset_pose.position_m)
    face_offset = min(layout.product.dimensions_m[:2]) / 2.0
    desired_palm_xy = (
        product[0] - face_offset * approach_x,
        product[1] - face_offset * approach_y,
    )
    root = (
        desired_palm_xy[0] - rotated_x,
        desired_palm_xy[1] - rotated_y,
        float(spec.root_position_m[2]),
    )
    orientation = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    predicted_palm = (
        desired_palm_xy[0],
        desired_palm_xy[1],
        root[2] + WAIST_ORIGIN_IN_ROOT_M[2] + target.position_m[2],
    )
    return PickupResetPlan(root, orientation, target, predicted_palm, product)


class TrackingArticulationController:
    """Delegate every command to the production controller and retain targets."""

    def __init__(self, controller: ArticulationController):
        self._controller = controller
        self.spec = controller.spec
        self._targets: dict[str, float] = {}

    def reset(self, **kwargs) -> None:
        self._controller.reset(**kwargs)
        self._targets = dict(self.spec.reset_joint_positions)

    def command_joint_positions(self, targets: dict[str, float]):
        commanded = tuple(self._controller.command_joint_positions(targets))
        if set(commanded) == set(targets) and len(commanded) == len(targets):
            self._targets.update({name: float(targets[name]) for name in commanded})
        return commanded

    def targets(self) -> dict[str, float]:
        return dict(self._targets)


@dataclass(frozen=True)
class SmokeResult:
    status: str
    terminal_phase: str
    failure: str | None
    physics_steps: int
    samples: tuple[dict[str, object], ...]


class GraspSmokeRunner:
    """Bounded orchestration that never writes robot or product state directly."""

    def __init__(
        self,
        physics: PhysicsStepper,
        controller: PhysicalGraspController,
        arm_approach: AdvancingApproachPort,
        arm_lift: AdvancingLiftPort,
        approach_observer: Callable[[], object],
        approach_target: ToolPose,
        diagnostics: Callable[[], dict[str, object]],
        *,
        maximum_physics_steps: int,
        status_path: Path,
        preflight: dict[str, object],
    ) -> None:
        if isinstance(maximum_physics_steps, bool) or maximum_physics_steps < 1:
            raise ValueError("maximum physics steps must be a positive integer")
        self.physics = physics
        self.controller = controller
        self.arm_approach = arm_approach
        self.arm_lift = arm_lift
        self.approach_observer = approach_observer
        self.approach_target = approach_target
        self.diagnostics = diagnostics
        self.maximum_physics_steps = maximum_physics_steps
        self.status_path = status_path
        self.preflight = dict(preflight)

    def _report(self, status: str, samples: list[dict[str, object]], error: str | None = None):
        result = {
            "schema_version": 1,
            "status": status,
            "preflight": self.preflight,
            "state_write_policy": {
                "explicit_robot_reset_before_runner": True,
                "post_reset_direct_state_writes": 0,
                "finger_and_arm_motion": "ArticulationController.command_joint_positions",
                "base_motion_after_reset": "none",
                "product_motion": "physics_only",
            },
            "samples": samples,
            "last_adapter_diagnostics": self.diagnostics(),
            "error": error,
        }
        write_durable_json(self.status_path, result)
        return result

    def run(self) -> SmokeResult:
        samples: list[dict[str, object]] = []
        self._report("starting", samples)
        if not self.arm_approach.request_approach(
            self.approach_target, PREGRASP_MAXIMUM_DURATION_S
        ):
            error = self.arm_approach.last_error or "arm approach request rejected"
            self._report("fail", samples, error)
            return SmokeResult("fail", "approach_failed", error, 0, ())
        self._report("running", samples)
        error: str | None = None
        grasp_started = False
        approach_confirmations = 0
        for step in range(self.maximum_physics_steps):
            if not grasp_started:
                if not self.arm_approach.advance():
                    error = self.arm_approach.last_error or "arm approach could not advance"
            elif self.controller.phase is GraspPhase.LIFTING and not self.arm_lift.advance():
                error = "arm lift port could not advance within its bounded deadline"
            self.physics.step()
            if not grasp_started:
                try:
                    self.approach_observer()
                    reached = self.arm_approach.target_reached()
                    if not reached and self.arm_approach.last_error is not None:
                        error = self.arm_approach.last_error
                except Exception as exc:
                    error = f"arm approach measurement failed: {exc}"
                    reached = False
                approach_confirmations = approach_confirmations + 1 if reached else 0
                sample = {
                    "step": step,
                    "sequence_phase": "approaching",
                    "phase": "idle",
                    "failure": error,
                    "approach_confirmations": approach_confirmations,
                    "approach": self.arm_approach.diagnostics(),
                    "contact_confirmations": 0,
                    "lift_confirmations": 0,
                    "adapter": self.diagnostics(),
                }
                samples.append(sample)
                self._report("running", samples, error)
                if error is not None:
                    break
                if approach_confirmations >= PREGRASP_CONFIRMATION_SAMPLES:
                    try:
                        self.controller.start()
                    except Exception as exc:
                        error = str(exc)
                        break
                    grasp_started = True
                continue
            status = self.controller.step()
            sample = {
                "step": step,
                "sequence_phase": "grasp",
                "phase": status.phase.value,
                "failure": status.failure.value if status.failure else None,
                "phase_observations": status.phase_observations,
                "contact_confirmations": status.contact_confirmations,
                "lift_confirmations": status.lift_confirmations,
                "adapter": self.diagnostics(),
            }
            samples.append(sample)
            self._report("running", samples, error)
            if status.phase in (GraspPhase.COMPLETE, GraspPhase.FAILED):
                break
        terminal = self.controller.status
        if not grasp_started and error is None:
            error = "arm approach did not reach its measured target before the step bound"
        elif terminal.phase not in (GraspPhase.COMPLETE, GraspPhase.FAILED):
            error = error or "smoke harness physics-step bound expired"
        passed = terminal.phase is GraspPhase.COMPLETE and error is None
        self._report("pass" if passed else "fail", samples, error)
        return SmokeResult(
            "pass" if passed else "fail",
            terminal.phase.value,
            terminal.failure.value if terminal.failure else error,
            len(samples),
            tuple(samples),
        )


def exact_contact_bindings(
    articulation_root_path: str,
    link_names,
    link_paths,
    product_body_path: str,
    product_collider_path: str,
    support_path: str,
) -> IsaacContactBindings:
    names = tuple(str(item) for item in link_names)
    paths = tuple(str(item) for item in link_paths)
    if len(names) != len(paths) or len(set(names)) != len(names):
        raise RuntimeError("articulation link name/path arrays are not one-to-one")
    by_name = dict(zip(names, paths))
    missing = sorted(set(REQUIRED_CONTACT_LINKS) - set(by_name))
    if missing:
        raise RuntimeError(f"imported articulation lacks required hand links: {missing}")
    selected = {name: by_name[name] for name in REQUIRED_CONTACT_LINKS}
    return IsaacContactBindings(
        RobotContactBodyMap(articulation_root_path, selected),
        product_body_path,
        product_collider_path,
        support_path,
    )


def diagnostic_fingertip_paths(bindings: IsaacContactBindings) -> dict[str, str]:
    """Derive the five fixed marker paths from their validated parent bodies."""

    parents = {
        "right_thumb_fingertip": "right_thumb_dp",
        "right_index_fingertip": "right_index_ip",
        "right_middle_fingertip": "right_middle_ip",
        "right_ring_fingertip": "right_ring_ip",
        "right_pinky_fingertip": "right_pinky_ip",
    }
    body_paths = bindings.robot_contacts.link_body_prim_paths
    return {
        marker: f"{body_paths[parent]}/{marker}"
        for marker, parent in parents.items()
    }


def runtime_preflight_evidence(
    identity_receipt: dict[str, object],
    bindings: IsaacContactBindings,
    *,
    scenario_path: Path,
    robot_config_path: Path,
    isaac_root: Path,
    physics_dt_s: float,
    robot_link_count: int,
    robot_dof_count: int,
    product_contact_sensor_path: str,
    pickup_reset: PickupResetPlan | None = None,
    diagnostic_pose_paths: dict[str, str] | None = None,
) -> dict[str, object]:
    """Bind runtime paths and setup to the already validated source identity."""

    if identity_receipt.get("status") != "pass":
        raise RuntimeError("runtime preflight requires a passing identity receipt")
    observed = identity_receipt["observed"]
    expected_paths = {
        "scenario_path": str(scenario_path.resolve()),
        "robot_config_path": str(robot_config_path.resolve()),
        "isaac_root": str(isaac_root.resolve()),
    }
    for name, path in expected_paths.items():
        if observed.get(name) != path:
            raise RuntimeError(f"runtime preflight {name} differs from identity receipt")
    robot_map = dict(sorted(bindings.robot_contacts.link_body_prim_paths.items()))
    return {
        "identity": identity_receipt,
        "installed_isaac": {
            "root": str(isaac_root.resolve()),
            "build": observed["isaac_build"],
        },
        "scenario": {
            "path": str(scenario_path.resolve()),
            "sha256": observed["scenario_sha256"],
        },
        "robot_config": {
            "path": str(robot_config_path.resolve()),
            "sha256": observed["robot_config_sha256"],
        },
        "physics_engine": "physx",
        "physics_dt_s": physics_dt_s,
        "robot_root": bindings.robot_contacts.robot_root_prim_path,
        "robot_link_count": robot_link_count,
        "robot_dof_count": robot_dof_count,
        "contact_bindings": {
            "robot_link_body_prim_paths": robot_map,
            "product_body_prim_path": bindings.product_body_prim_path,
            "product_collider_prim_path": bindings.product_collider_prim_path,
            "pickup_support_prim_path": bindings.pickup_support_prim_path,
            "complete_allowed_body_paths": sorted(bindings.allowed_body_paths),
        },
        "product_contact_sensor": product_contact_sensor_path,
        "sensor_min_threshold_n": 0.0,
        "sensor_radius": -1.0,
        "pickup_reset_plan": None if pickup_reset is None else pickup_reset.evidence(),
        "diagnostic_pose_paths": dict(sorted((diagnostic_pose_paths or {}).items())),
    }


def run_isaac(args: argparse.Namespace, repo: Path) -> SmokeResult:
    """Build and execute the real smoke scene; this function starts Isaac."""

    app = None
    timeline = None
    status_path = args.output / "grasp_smoke_status.json"
    identity_receipt: dict[str, object] = {
        "status": "fail",
        "expected": {
            "candidate_commit": args.expected_candidate_sha,
            "candidate_tree": args.expected_candidate_tree_sha,
            "acceptance_spec_sha256": ACCEPTANCE_SPEC_SHA256,
            "combined_source_urdf_canonical_sha256": SOURCE_URDF_CANONICAL_SHA256,
            "production_urdf_canonical_sha256": PRODUCTION_URDF_CANONICAL_SHA256,
            "scenario_sha256": BASELINE_SCENARIO_SHA256,
            "robot_config_sha256": ROBOT_CONFIG_SHA256,
            "isaac_build": ISAAC_BUILD,
        },
        "observed": {},
        "mismatches": ["identity collection did not complete"],
    }
    terminal_receipt: dict[str, object] = {
        "schema_version": 1,
        "status": "preflight_fail",
        "identity_preflight": identity_receipt,
    }
    try:
        identity_receipt = identity_preflight_receipt(
            repo,
            args.acceptance_spec,
            args.expected_candidate_sha,
            args.expected_candidate_tree_sha,
            args.isaac_root,
        )
        terminal_receipt = {
            "schema_version": 1,
            "status": "preflight_pass" if identity_receipt["status"] == "pass" else "preflight_fail",
            "identity_preflight": identity_receipt,
        }
        write_durable_json(status_path, terminal_receipt)
        if identity_receipt["status"] != "pass":
            raise RuntimeError(
                "identity preflight failed: "
                + ", ".join(str(item) for item in identity_receipt["mismatches"])
            )
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "fast_shutdown": False})
        import omni.timeline
        import omni.usd
        from isaacsim.core.experimental.prims import Articulation, RigidPrim, XformPrim
        from isaacsim.core.experimental.utils import app as app_utils
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.sensors.experimental.physics import Contact, ContactSensor
        from pxr import PhysicsSchemaTools, UsdGeom, UsdPhysics

        from simulator.environment.isaac_builder import IsaacAisleBuilder
        from simulator.environment.isaac_restocking_builder import IsaacRestockingBuilder

        spec = load_production_spec(repo / "robot_spike" / "production")
        scenario_path = repo / "config" / "scenarios" / "baseline_straight.yaml"
        robot_config_path = repo / "robot_spike" / "production" / "robot_config.json"
        scenario = load_scenario(scenario_path)
        aisle = build_aisle_layout(scenario.environment)
        layout = build_restocking_layout(aisle)
        usd_path = IsaacRobotLoader(spec).import_urdf(args.output / "derived_usd")
        omni.usd.get_context().open_stage(str(usd_path))
        app_utils.update_app(steps=8)
        stage = omni.usd.get_context().get_stage()
        if not math.isclose(float(UsdGeom.GetStageMetersPerUnit(stage)), 1.0, abs_tol=1e-12):
            raise RuntimeError("imported robot stage must use metres")
        default_prim = stage.GetDefaultPrim()
        variant = default_prim.GetVariantSets().GetVariantSet("Physics")
        if not default_prim.IsValid() or not variant.IsValid():
            raise RuntimeError("imported robot lacks its Physics variant")
        variant.SetVariantSelection("physx")
        IsaacAisleBuilder(stage).build(aisle)
        handles = IsaacRestockingBuilder(stage).build(layout)
        roots = [prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f"expected one complete production articulation, found {len(roots)}")
        root_path = str(roots[0].GetPath())
        articulation = Articulation(root_path)
        if len(articulation.link_paths) != 1:
            raise RuntimeError("expected one articulation instance")
        link_paths = tuple(str(path) for path in articulation.link_paths[0])
        bindings = exact_contact_bindings(
            root_path,
            articulation.link_names,
            link_paths,
            handles.product_rigid_body_prim_path,
            handles.product_collider_prim_path,
            handles.pickup_support_prim_path,
        )
        by_name = dict(zip(articulation.link_names, link_paths))
        palm = RigidPrim([by_name["right_palm"]], resolve_paths=False)
        product = RigidPrim([handles.product_rigid_body_prim_path], resolve_paths=False)
        fingertip_paths = diagnostic_fingertip_paths(bindings)
        missing_diagnostic_paths = [
            path for path in fingertip_paths.values() if not stage.GetPrimAtPath(path).IsValid()
        ]
        if missing_diagnostic_paths:
            raise RuntimeError(
                f"imported robot lacks diagnostic fingertip paths: {missing_diagnostic_paths}"
            )
        diagnostic_link_poses = {"right_palm": palm}
        diagnostic_link_poses.update(
            {
                name: XformPrim([path], resolve_paths=False)
                for name, path in fingertip_paths.items()
            }
        )
        sensor_path = handles.product_rigid_body_prim_path + "/grasp_contact_sensor"
        sensor = ContactSensor(Contact.create(sensor_path, min_threshold=0.0,
                                               max_threshold=100000.0, radius=-1.0))
        SimulationManager.set_physics_dt(1.0 / args.physics_hz)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        app_utils.update_app(steps=3)
        physics_dt = float(SimulationManager.get_physics_dt())
        kinematics = RightArmKinematics(spec)
        reset_plan = pickup_reset_plan(spec, layout, kinematics)
        joint_controller = TrackingArticulationController(
            ArticulationController(spec, articulation)
        )
        feedback = Isaac61GraspFeedbackAdapter(
            articulation, palm, product, sensor,
            SimulationManager.get_simulation_time,
            lambda handle: str(PhysicsSchemaTools.intToSdfPath(handle)),
            bindings,
            maximum_contact_age_s=2.5 * physics_dt,
            diagnostic_link_poses=diagnostic_link_poses,
            product_velocity=product,
            joint_target_source=joint_controller.targets,
        )
        approach = IsaacArmApproachPort(
            joint_controller,
            articulation,
            kinematics,
            SimulationManager.get_simulation_time,
            command_period_s=physics_dt,
        )
        lift = IsaacArmLiftPort(
            joint_controller, articulation, kinematics,
            SimulationManager.get_simulation_time,
            handles.product_rigid_body_prim_path,
            command_period_s=physics_dt,
        )
        grasp = PhysicalGraspController(
            joint_controller, layout, feedback=feedback, arm_lift=lift,
            robot_contacts=bindings.robot_contacts, limits=GraspLimits(),
        )
        class _Physics:
            def step(self):
                SimulationManager.step()

        preflight = runtime_preflight_evidence(
            identity_receipt,
            bindings,
            scenario_path=scenario_path,
            robot_config_path=robot_config_path,
            isaac_root=args.isaac_root,
            physics_dt_s=physics_dt,
            robot_link_count=len(link_paths),
            robot_dof_count=len(articulation.dof_names),
            product_contact_sensor_path=sensor_path,
            pickup_reset=reset_plan,
            diagnostic_pose_paths={
                "right_palm": by_name["right_palm"], **fingertip_paths
            },
        )
        terminal_receipt = {
            "schema_version": 1,
            "status": "runtime_preflight_pass",
            "preflight": preflight,
        }
        write_durable_json(status_path, terminal_receipt)
        joint_controller.reset(
            root_position_m=reset_plan.root_position_m,
            root_orientation_wxyz=reset_plan.root_orientation_wxyz,
        )
        return GraspSmokeRunner(
            _Physics(),
            grasp,
            approach,
            lift,
            feedback.read_observation,
            reset_plan.arm_target,
            feedback.diagnostics,
            maximum_physics_steps=args.maximum_steps,
            status_path=status_path,
            preflight=preflight,
        ).run()
    except Exception as exc:
        terminal_receipt.update({"status": "error", "runtime_error": str(exc)})
        write_durable_json(status_path, terminal_receipt)
        raise
    finally:
        if timeline is not None:
            timeline.stop()
        if app is not None:
            app.close()


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-candidate-sha", required=True)
    parser.add_argument("--expected-candidate-tree-sha", required=True)
    parser.add_argument(
        "--acceptance-spec",
        type=Path,
        default=Path.home() / "Downloads" / "ROBOT_BACKROOM_RESTOCKING_IMPLEMENTATION_SPEC.md",
    )
    parser.add_argument("--physics-hz", type=float, default=120.0)
    parser.add_argument("--maximum-steps", type=int, default=1800)
    parser.add_argument("--isaac-root", type=Path, default=Path("C:/isaacsim"))
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _arguments(argv)
    if not math.isfinite(args.physics_hz) or args.physics_hz <= 0.0 or args.maximum_steps < 1:
        raise SystemExit("physics-hz and maximum-steps must be positive")
    repo = Path(__file__).resolve().parents[2]
    return 0 if run_isaac(args, repo).status == "pass" else 1


def _terminate_process(
    exit_code: int, *, hard_exit: Callable[[int], object] = os._exit
) -> None:
    """Flush durable output, then set kit.exe's status for python.bat."""

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except Exception:
            pass
    hard_exit(int(exit_code))


if __name__ == "__main__":
    _terminate_process(main())
