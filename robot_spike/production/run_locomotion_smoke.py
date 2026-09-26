#!/usr/bin/env python3
"""One serialized Isaac Sim 6.1 locomotion smoke with fail-closed evidence.

This file is a future GPU entry point.  Importing it does not import Isaac or
start SimulationApp.  Run it only through the installed Isaac Python after an
orchestrator assigns the single heavy-process slot.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import traceback
from typing import Any, Callable

from robot_spike.production.isaac_feedback import (
    ASIMOV_SOLE_GEOMETRY,
    FeedbackUnavailableError,
    Isaac61LocomotionFeedbackAdapter,
)
from robot_spike.production.locomotion import (
    BipedLocomotionController,
    FootSide,
    GaitConfig,
    LEG_JOINTS,
    LocomotionFeedback,
    LocomotionState,
    PlanarPose,
    symmetric_crouch_targets,
)
from robot_spike.production.model import load_production_spec
from robot_spike.production.runtime import ArticulationController, IsaacRobotLoader


OWNER_SPEC_SHA256 = "7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad"
OWNER_SPEC_PATH = Path(
    "C:/Users/suyog/.codex/visualizations/2026/09/26/"
    "01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e/"
    "ROBOT_BACKROOM_RESTOCKING_IMPLEMENTATION_SPEC.md"
)
EVIDENCE_FILES = (
    "robot_spike/production/isaac_feedback.py",
    "robot_spike/production/locomotion.py",
    "robot_spike/production/model.py",
    "robot_spike/production/runtime.py",
    "robot_spike/production/robot_config.json",
    "robot_spike/production/production_manifest.json",
    "robot_spike/production/asimov_orcahand_restocking.urdf",
    "robot_spike/asimov_orcahand_right.urdf",
    "robot_spike/production/run_locomotion_smoke.py",
    "tests/test_robot_isaac_feedback.py",
    "review/robot_manipulation/LOCOMOTION_CONTROL.md",
)
ISAAC_API_SOURCE_FILES = (
    "VERSION",
    "python.bat",
    "exts/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py",
    "exts/isaacsim.core.experimental.prims/isaacsim/core/experimental/prims/impl/articulation.py",
    "exts/isaacsim.core.experimental.prims/isaacsim/core/experimental/prims/impl/rigid_prim.py",
    "exts/isaacsim.sensors.experimental.physics/isaacsim/sensors/experimental/physics/impl/contact_sensor.py",
    "exts/isaacsim.sensors.experimental.physics/isaacsim/sensors/experimental/physics/impl/contact.py",
    "exts/isaacsim.core.simulation_manager/isaacsim/core/simulation_manager/impl/simulation_manager.py",
    "extscache/omni.physics.tensors-110.3.2+110.3.0.wx64.r.cp312.u7f4/omni/physics/tensors/api.py",
)


def _arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-candidate-sha", required=True)
    parser.add_argument("--expected-candidate-tree-sha", required=True)
    parser.add_argument("--acceptance-spec", type=Path, default=OWNER_SPEC_PATH)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--forward-m", type=float, default=0.06)
    parser.add_argument("--physics-hz", type=int, default=120)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args(argv)


def _validate_arguments(args: argparse.Namespace) -> None:
    """Reject invalid run parameters before importing or starting Isaac."""

    if args.physics_hz <= 0 or args.repeats <= 0:
        raise ValueError("physics-hz and repeats must be positive")
    durations_and_distance = (args.forward_m, args.settle_s, args.timeout_s)
    if not all(math.isfinite(value) and value > 0.0 for value in durations_and_distance):
        raise ValueError("forward-m, settle-s, and timeout-s must be finite and positive")
    minimum_startup_s = 2.0 * GaitConfig().double_support_duration_s + 1.0 / args.physics_hz
    if args.settle_s < minimum_startup_s:
        raise ValueError(
            "settle-s must allow one acquisition step plus the configured "
            "double-support ramp and dwell"
        )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git(repo: Path, *args: str) -> str:
    completed = subprocess.run(
        ["git", *args], cwd=repo, check=True, capture_output=True, text=True
    )
    return completed.stdout.strip()


def source_identity(
    repo: Path,
    expected_candidate: str,
    expected_tree: str,
    *,
    require_clean: bool = True,
) -> dict[str, Any]:
    """Read the exact candidate and every directly consumed source input."""

    candidate = _git(repo, "rev-parse", "HEAD")
    tree = _git(repo, "rev-parse", "HEAD^{tree}")
    if require_clean:
        dirty = _git(repo, "status", "--porcelain=v1")
        if dirty:
            raise RuntimeError(f"refusing to run from a dirty worktree:\n{dirty}")
    if candidate != expected_candidate:
        raise RuntimeError(
            f"candidate mismatch: expected {expected_candidate}, checked out {candidate}"
        )
    if tree != expected_tree:
        raise RuntimeError(f"tree mismatch: expected {expected_tree}, checked out {tree}")
    hashes: dict[str, str] = {}
    for relative in EVIDENCE_FILES:
        path = repo / relative
        if not path.is_file():
            raise RuntimeError(f"required evidence input is missing: {relative}")
        hashes[relative] = _sha256(path)
    return {
        "candidate_sha": candidate,
        "candidate_tree_sha": tree,
        "source_files_sha256": hashes,
    }


def acceptance_spec_identity(
    path: Path, expected_sha256: str = OWNER_SPEC_SHA256
) -> dict[str, str]:
    """Hash the actual acceptance-spec bytes and reject any mismatch."""

    resolved = path.resolve()
    if not resolved.is_file():
        raise RuntimeError(f"acceptance specification is missing: {resolved}")
    observed = _sha256(resolved)
    if observed != expected_sha256:
        raise RuntimeError(
            "acceptance specification SHA-256 mismatch: "
            f"expected {expected_sha256}, observed {observed} at {resolved}"
        )
    return {
        "path": str(resolved),
        "sha256": observed,
        "expected_sha256": expected_sha256,
    }


def evidence_identity(
    repo: Path,
    expected_candidate: str,
    expected_tree: str,
    acceptance_spec: Path,
    *,
    require_clean: bool,
    isaac_root: Path = Path("C:/isaacsim"),
) -> dict[str, Any]:
    """Read the complete owner-spec, source/input, and installed-API vector."""

    return {
        "acceptance_spec": acceptance_spec_identity(acceptance_spec),
        "source": source_identity(
            repo,
            expected_candidate,
            expected_tree,
            require_clean=require_clean,
        ),
        "installed_isaac": installed_isaac_identity(isaac_root),
    }


def _identity_sha256(identity: dict[str, Any]) -> str:
    canonical = json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _identity_mismatches(expected: object, observed: object, path: str = "identity") -> list[str]:
    if isinstance(expected, dict) and isinstance(observed, dict):
        mismatches: list[str] = []
        for key in sorted(set(expected) | set(observed)):
            child = f"{path}.{key}"
            if key not in observed:
                mismatches.append(f"{child}: missing")
            elif key not in expected:
                mismatches.append(f"{child}: unexpected")
            else:
                mismatches.extend(_identity_mismatches(expected[key], observed[key], child))
        return mismatches
    if expected != observed:
        return [f"{path}: expected {expected!r}, observed {observed!r}"]
    return []


class EvidenceIdentityMismatchError(RuntimeError):
    """A repeat/final boundary no longer matches the initial identity vector."""

    def __init__(self, check: dict[str, Any]):
        self.check = check
        detail = check.get("error") or "; ".join(check.get("mismatches", ()))
        super().__init__(f"evidence identity check {check['phase']!r} failed: {detail}")


class EvidenceIdentityGuard:
    """Re-read and compare evidence identity at deterministic run boundaries."""

    def __init__(
        self,
        initial_identity: dict[str, Any],
        identity_reader: Callable[[], dict[str, Any]],
    ) -> None:
        self.initial_identity = initial_identity
        self.identity_reader = identity_reader
        self.checks: list[dict[str, Any]] = []

    def check(self, phase: str) -> dict[str, Any]:
        check: dict[str, Any] = {"phase": phase}
        try:
            observed = self.identity_reader()
        except Exception as exc:
            check.update(
                {
                    "status": "error",
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                }
            )
            self.checks.append(check)
            raise EvidenceIdentityMismatchError(check) from exc
        mismatches = _identity_mismatches(self.initial_identity, observed)
        check.update(
            {
                "status": "match" if not mismatches else "mismatch",
                "observed_identity_sha256": _identity_sha256(observed),
                "mismatches": mismatches,
            }
        )
        self.checks.append(check)
        if mismatches:
            raise EvidenceIdentityMismatchError(check)
        return check


def _record_identity_boundary(
    guard: EvidenceIdentityGuard,
    phase: str,
    result: dict[str, Any],
    status_path: Path,
    repeat_result: dict[str, Any] | None = None,
) -> None:
    """Persist a boundary check, including a mismatch, before continuing or failing."""

    try:
        check = guard.check(phase)
    except EvidenceIdentityMismatchError as exc:
        check = exc.check
        raise
    finally:
        if "check" in locals() and repeat_result is not None:
            repeat_result["identity_checks"].append(check)
        status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


class _RecordingFeedbackSource:
    def __init__(self, adapter: Isaac61LocomotionFeedbackAdapter):
        self.adapter = adapter
        self.last = None

    def read_feedback(self):
        self.last = self.adapter.read_feedback()
        return self.last


class StartupValidationError(FeedbackUnavailableError):
    """Raised when measured startup state cannot safely reach double support."""

    def __init__(self, phase: str, reason: str, report: dict[str, Any]):
        super().__init__(f"staged startup {phase} failed: {reason}")
        self.phase = phase
        self.report = report


def _interpolate_joint_targets(
    start: dict[str, float], end: dict[str, float], fraction: float
) -> dict[str, float]:
    """Linearly interpolate a name-bound target vector with closed endpoints."""

    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("joint target interpolation fraction must be in [0, 1]")
    if start.keys() != end.keys():
        raise ValueError("joint target interpolation endpoints must have identical names")
    return {
        name: start[name] + fraction * (end[name] - start[name])
        for name in start
    }


def _kinematic_safety_failures(
    diagnostics: dict[str, Any], config: GaitConfig
) -> tuple[list[str], dict[str, float]]:
    """Apply root stability gates even when contact/support construction failed."""

    try:
        root = diagnostics["kinematics"]["root"]
        roll, pitch, _yaw = (float(value) for value in root["roll_pitch_yaw_rad"])
        linear = tuple(float(value) for value in root["linear_velocity_body_mps"])
        angular = tuple(float(value) for value in root["angular_velocity_body_rps"])
    except (KeyError, TypeError, ValueError) as exc:
        return [f"measured root kinematics unavailable: {exc}"], {}
    values = (roll, pitch, *linear, *angular)
    if not all(math.isfinite(value) for value in values):
        return ["measured root kinematics contain a nonfinite value"], {}
    tilt = max(abs(roll), abs(pitch))
    linear_speed = math.sqrt(sum(value * value for value in linear))
    angular_speed = max(abs(value) for value in angular)
    failures: list[str] = []
    if tilt > config.maximum_root_tilt_rad:
        failures.append("root tilt exceeded configured limit")
    if linear_speed > config.maximum_root_linear_speed_mps:
        failures.append("root linear speed exceeded configured limit")
    if angular_speed > config.maximum_root_angular_speed_rps:
        failures.append("root angular speed exceeded configured limit")
    return failures, {
        "root_tilt_rad": tilt,
        "root_linear_speed_mps": linear_speed,
        "root_angular_speed_rps": angular_speed,
    }


def _startup_feedback_failures(
    feedback: LocomotionFeedback,
    diagnostics: dict[str, Any],
    targets: dict[str, float] | None,
    config: GaitConfig,
) -> tuple[list[str], dict[str, Any]]:
    """Check true double support, stability, sole flatness, and target tracking."""

    failures, metrics = _kinematic_safety_failures(diagnostics, config)
    if not (feedback.left_foot.in_contact and feedback.right_foot.in_contact):
        failures.append("both measured feet are not in contact")
    contacting_heights = [
        foot.pose.position_m[2]
        for foot in (feedback.left_foot, feedback.right_foot)
        if foot.in_contact
    ]
    if contacting_heights:
        root_clearance = feedback.root_height_m - max(contacting_heights)
        metrics["root_clearance_m"] = root_clearance
        if not (
            config.minimum_root_clearance_m
            <= root_clearance
            <= config.maximum_root_clearance_m
        ):
            failures.append("root clearance left the configured envelope")
    else:
        failures.append("root clearance is unavailable without measured contact")
    metrics["support_margin_m"] = feedback.support_margin_m
    if feedback.support_margin_m < config.minimum_support_margin_m:
        failures.append("COM projection left the configured support margin")

    sole_span = max(
        math.dist(first, second)
        for first in ASIMOV_SOLE_GEOMETRY.collision_sphere_centers_m
        for second in ASIMOV_SOLE_GEOMETRY.collision_sphere_centers_m
    )
    maximum_height_spread = sole_span * math.sin(config.maximum_root_tilt_rad)
    metrics["maximum_sole_sphere_height_spread_m"] = maximum_height_spread
    try:
        ankle_diagnostics = diagnostics["kinematics"]["ankles"]
        for side in ("left", "right"):
            points = ankle_diagnostics[side]["sphere_world_lowest_points_m"]
            heights = [float(point[2]) for point in points]
            if len(heights) != len(ASIMOV_SOLE_GEOMETRY.collision_sphere_centers_m):
                raise ValueError(f"{side} sole sphere count is invalid")
            spread = max(heights) - min(heights)
            metrics[f"{side}_sole_sphere_height_spread_m"] = spread
            if not math.isfinite(spread) or spread > maximum_height_spread:
                failures.append(f"{side} sole flatness exceeded configured tilt envelope")
    except (KeyError, TypeError, ValueError) as exc:
        failures.append(f"measured ankle/sole kinematics unavailable: {exc}")

    if targets is not None:
        try:
            target_error = max(
                abs(float(feedback.joint_position_rad[name]) - target)
                for name, target in targets.items()
            )
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"measured target tracking is unavailable: {exc}")
        else:
            metrics["maximum_joint_target_error_rad"] = target_error
            if not math.isfinite(target_error) or target_error > config.maximum_target_error_rad:
                failures.append("joint target tracking exceeded configured limit")
    return failures, metrics


def _staged_double_support_startup(
    *,
    feedback_source: _RecordingFeedbackSource,
    joint_controller: ArticulationController,
    step_physics: Callable[[], None],
    crouch_targets: dict[str, float],
    physics_dt: float,
    maximum_duration_s: float,
    config: GaitConfig = GaitConfig(),
    progress: Callable[[dict[str, Any]], None] | None = None,
) -> tuple[LocomotionFeedback, dict[str, Any]]:
    """Acquire double support, ramp drive targets, and prove a stable dwell.

    The caller performs the sole explicit reset.  This routine advances physics,
    reads measured feedback, and issues only name-bound articulation drive
    targets.  No pose, velocity, or joint-state setter is used.
    """

    if not math.isfinite(physics_dt) or physics_dt <= 0.0:
        raise ValueError("physics timestep must be finite and positive")
    if not math.isfinite(maximum_duration_s) or maximum_duration_s <= 0.0:
        raise ValueError("startup duration must be finite and positive")
    if set(crouch_targets) != set(LEG_JOINTS):
        raise ValueError("startup crouch targets must name every leg joint exactly once")
    total_steps = max(1, math.ceil(maximum_duration_s / physics_dt))
    ramp_steps = max(1, math.ceil(config.double_support_duration_s / physics_dt))
    dwell_steps = max(1, math.ceil(config.double_support_duration_s / physics_dt))
    acquisition_steps = min(
        max(1, math.ceil(config.double_support_timeout_s / physics_dt)),
        total_steps - ramp_steps - dwell_steps,
    )
    if acquisition_steps < 1:
        raise ValueError("startup duration does not leave a contact acquisition step")
    report: dict[str, Any] = {
        "status": "running",
        "physics_dt_s": physics_dt,
        "maximum_duration_s": maximum_duration_s,
        "step_budget": {
            "contact_acquisition": acquisition_steps,
            "target_ramp": ramp_steps,
            "verified_dwell": dwell_steps,
        },
        "gates": {
            "maximum_root_tilt_rad": config.maximum_root_tilt_rad,
            "maximum_root_linear_speed_mps": config.maximum_root_linear_speed_mps,
            "maximum_root_angular_speed_rps": config.maximum_root_angular_speed_rps,
            "maximum_joint_target_error_rad": config.maximum_target_error_rad,
            "minimum_root_clearance_m": config.minimum_root_clearance_m,
            "maximum_root_clearance_m": config.maximum_root_clearance_m,
            "minimum_support_margin_m": config.minimum_support_margin_m,
            "sole_flatness_source": "sole span times sine(maximum_root_tilt_rad)",
        },
        "events": [],
    }

    def publish() -> None:
        if progress is not None:
            progress(report)

    def abort(phase: str, reason: str) -> None:
        report["status"] = "error"
        report["failure_phase"] = phase
        report["failure_reason"] = reason
        publish()
        raise StartupValidationError(phase, reason, report)

    acquired: LocomotionFeedback | None = None
    for step_index in range(1, acquisition_steps + 1):
        step_physics()
        try:
            feedback = feedback_source.read_feedback()
        except FeedbackUnavailableError as exc:
            diagnostics = feedback_source.adapter.diagnostics()
            root_failures, root_metrics = _kinematic_safety_failures(diagnostics, config)
            report["events"].append(
                {
                    "stage": "contact_acquisition",
                    "step": step_index,
                    "status": "waiting_for_measured_double_support",
                    "feedback_error": str(exc),
                    "root_metrics": root_metrics,
                    "diagnostics": diagnostics,
                }
            )
            publish()
            if root_failures:
                abort("contact_acquisition", "; ".join(root_failures))
            continue
        diagnostics = feedback_source.adapter.diagnostics()
        if not (feedback.left_foot.in_contact and feedback.right_foot.in_contact):
            root_failures, root_metrics = _kinematic_safety_failures(diagnostics, config)
            report["events"].append(
                {
                    "stage": "contact_acquisition",
                    "step": step_index,
                    "status": "waiting_for_measured_double_support",
                    "root_metrics": root_metrics,
                    "diagnostics": diagnostics,
                }
            )
            publish()
            if root_failures:
                abort("contact_acquisition", "; ".join(root_failures))
            continue
        failures, metrics = _startup_feedback_failures(
            feedback, diagnostics, None, config
        )
        report["events"].append(
            {
                "stage": "contact_acquisition",
                "step": step_index,
                "status": "accepted" if not failures else "rejected",
                "timestamp_s": feedback.timestamp_s,
                "metrics": metrics,
                "diagnostics": diagnostics,
            }
        )
        publish()
        if failures:
            abort("contact_acquisition", "; ".join(failures))
        acquired = feedback
        report["contact_acquired_step"] = step_index
        break
    if acquired is None:
        abort("contact_acquisition", "bounded window ended without measured bilateral support")

    start_targets = {
        name: float(acquired.joint_position_rad[name]) for name in LEG_JOINTS
    }
    report["ramp_start_joint_position_rad"] = start_targets
    report["crouch_targets_rad"] = dict(crouch_targets)
    latest = acquired
    for step_index in range(1, ramp_steps + 1):
        targets = _interpolate_joint_targets(
            start_targets, crouch_targets, step_index / ramp_steps
        )
        joint_controller.command_joint_positions(targets)
        step_physics()
        try:
            latest = feedback_source.read_feedback()
        except FeedbackUnavailableError as exc:
            report["events"].append(
                {
                    "stage": "target_ramp",
                    "step": step_index,
                    "status": "rejected",
                    "feedback_error": str(exc),
                    "commanded_joint_targets_rad": targets,
                    "diagnostics": feedback_source.adapter.diagnostics(),
                }
            )
            abort("target_ramp", f"measured contact/support was lost: {exc}")
        diagnostics = feedback_source.adapter.diagnostics()
        failures, metrics = _startup_feedback_failures(
            latest, diagnostics, targets, config
        )
        report["events"].append(
            {
                "stage": "target_ramp",
                "step": step_index,
                "status": "accepted" if not failures else "rejected",
                "timestamp_s": latest.timestamp_s,
                "metrics": metrics,
                "commanded_joint_targets_rad": targets,
                "diagnostics": diagnostics if failures else None,
            }
        )
        publish()
        if failures:
            abort("target_ramp", "; ".join(failures))

    for step_index in range(1, dwell_steps + 1):
        step_physics()
        try:
            latest = feedback_source.read_feedback()
        except FeedbackUnavailableError as exc:
            report["events"].append(
                {
                    "stage": "verified_dwell",
                    "step": step_index,
                    "status": "rejected",
                    "feedback_error": str(exc),
                    "commanded_joint_targets_rad": dict(crouch_targets),
                    "diagnostics": feedback_source.adapter.diagnostics(),
                }
            )
            abort("verified_dwell", f"measured contact/support was lost: {exc}")
        diagnostics = feedback_source.adapter.diagnostics()
        failures, metrics = _startup_feedback_failures(
            latest, diagnostics, crouch_targets, config
        )
        report["events"].append(
            {
                "stage": "verified_dwell",
                "step": step_index,
                "status": "accepted" if not failures else "rejected",
                "timestamp_s": latest.timestamp_s,
                "metrics": metrics,
                "diagnostics": diagnostics if failures else None,
            }
        )
        publish()
        if failures:
            abort("verified_dwell", "; ".join(failures))

    report["status"] = "pass"
    report["final_timestamp_s"] = latest.timestamp_s
    publish()
    return latest, report


def _sample(step: int, command, feedback, adapter) -> dict[str, Any]:
    contacting_heights = [
        foot.pose.position_m[2]
        for foot in (feedback.left_foot, feedback.right_foot)
        if foot.in_contact
    ]
    return {
        "step": step,
        "timestamp_s": feedback.timestamp_s,
        "state": command.state.value,
        "phase": command.phase.value,
        "active_step_index": command.active_step_index,
        "planned_step_count": command.planned_step_count,
        "failure_reason": command.failure_reason,
        "root_pose": {
            "x_m": feedback.root_pose.x_m,
            "y_m": feedback.root_pose.y_m,
            "yaw_rad": feedback.root_pose.yaw_rad,
            "height_m": feedback.root_height_m,
            "roll_rad": feedback.root_tilt_roll_pitch_rad[0],
            "pitch_rad": feedback.root_tilt_roll_pitch_rad[1],
        },
        "root_linear_velocity_body_mps": list(feedback.root_linear_velocity_body_mps),
        "root_angular_velocity_body_rps": list(feedback.root_angular_velocity_body_rps),
        "left_sole_position_m": list(feedback.left_foot.pose.position_m),
        "right_sole_position_m": list(feedback.right_foot.pose.position_m),
        "left_contact": feedback.left_foot.in_contact,
        "right_contact": feedback.right_foot.in_contact,
        "contact_force_n": dict(adapter.last_contact_forces_n),
        "contact_time_s": dict(adapter.last_contact_times_s),
        "contact_point_count": dict(adapter.last_contact_point_counts),
        "contact_diagnostics": adapter.diagnostics()["contacts"],
        "support_diagnostics": adapter.diagnostics()["support"],
        "com_position_world_m": list(feedback.com_position_world_m),
        "support_center_world_m": list(feedback.support_center_world_m),
        "support_margin_m": feedback.support_margin_m,
        "root_clearance_m": (
            feedback.root_height_m - max(contacting_heights)
            if contacting_heights
            else None
        ),
        "joint_position_rad": dict(feedback.joint_position_rad),
        "joint_velocity_rad_s": dict(feedback.joint_velocity_rad_s),
        "commanded_joint_targets_rad": dict(command.joint_targets_rad),
    }


def _persist_feedback_failure(
    *,
    adapter: Isaac61LocomotionFeedbackAdapter,
    repeat_result: dict[str, Any],
    result: dict[str, Any],
    status_path: Path,
    phase: str,
    error: FeedbackUnavailableError,
) -> None:
    """Persist sensor and support evidence before a feedback error unwinds."""

    repeat_result["feedback_failure"] = {
        "phase": phase,
        "error_type": type(error).__name__,
        "error": str(error),
        "diagnostics": adapter.diagnostics(),
    }
    status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


def _persist_pre_shutdown_runtime_error(
    *,
    initial_identity: dict[str, Any],
    status_path: Path,
    error: BaseException,
    partial_result: dict[str, Any] | None,
) -> dict[str, Any]:
    """Write terminal error state before SimulationApp shutdown can intervene."""

    report = partial_result if partial_result is not None else {
        "schema_version": 1,
        "initial_evidence_identity": initial_identity,
        "initial_evidence_identity_sha256": _identity_sha256(initial_identity),
        "repeats": [],
    }
    report.update(
        {
            "status": "error",
            "phase": "runtime",
            "shutdown_returned": False,
            "error_type": type(error).__name__,
            "error": str(error),
            "traceback": traceback.format_exc(),
        }
    )
    status_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def _repeat_metrics(
    samples: list[dict[str, Any]], command, target: PlanarPose
) -> dict[str, Any]:
    if not samples:
        raise RuntimeError("repeat produced no measured samples")
    first = samples[0]
    last = samples[-1]
    start = first["root_pose"]
    end = last["root_pose"]
    left_states = [bool(sample["left_contact"]) for sample in samples]
    right_states = [bool(sample["right_contact"]) for sample in samples]
    final_position_error = math.hypot(
        end["x_m"] - target.x_m, end["y_m"] - target.y_m
    )
    final_yaw_error = abs(
        math.atan2(
            math.sin(end["yaw_rad"] - target.yaw_rad),
            math.cos(end["yaw_rad"] - target.yaw_rad),
        )
    )
    fault_reason = command.failure_reason or ""
    return {
        "status": "pass" if command.state is LocomotionState.DOCKED else "fail",
        "terminal_state": command.state.value,
        "failure_reason": command.failure_reason,
        "sample_count": len(samples),
        "start_time_s": first["timestamp_s"],
        "end_time_s": last["timestamp_s"],
        "elapsed_s": last["timestamp_s"] - first["timestamp_s"],
        "planned_step_count": command.planned_step_count,
        "completed_step_count": command.active_step_index,
        "root_start": start,
        "root_end": end,
        "root_xy_displacement_m": math.hypot(end["x_m"] - start["x_m"], end["y_m"] - start["y_m"]),
        "final_target_position_error_m": final_position_error,
        "final_target_yaw_error_rad": final_yaw_error,
        "minimum_root_height_m": min(sample["root_pose"]["height_m"] for sample in samples),
        "minimum_root_clearance_m": min(
            sample["root_clearance_m"]
            for sample in samples
            if sample["root_clearance_m"] is not None
        ),
        "maximum_abs_roll_rad": max(abs(sample["root_pose"]["roll_rad"]) for sample in samples),
        "maximum_abs_pitch_rad": max(abs(sample["root_pose"]["pitch_rad"]) for sample in samples),
        "minimum_support_margin_m": min(sample["support_margin_m"] for sample in samples),
        "maximum_root_linear_speed_mps": max(
            math.sqrt(sum(value * value for value in sample["root_linear_velocity_body_mps"]))
            for sample in samples
        ),
        "maximum_root_angular_speed_rps": max(
            math.sqrt(sum(value * value for value in sample["root_angular_velocity_body_rps"]))
            for sample in samples
        ),
        "left_observed_unloaded": any(not value for value in left_states),
        "right_observed_unloaded": any(not value for value in right_states),
        "left_contact_transition_count": sum(a != b for a, b in zip(left_states, left_states[1:])),
        "right_contact_transition_count": sum(a != b for a, b in zip(right_states, right_states[1:])),
        "maximum_contacting_sole_slip_m": {
            side: _maximum_contacting_sole_slip(samples, side)
            for side in ("left", "right")
        },
        "maximum_one_step_lag_joint_target_error_rad": _maximum_lagged_target_error(samples),
        "fall_safety_fault_detected": any(
            phrase in fault_reason
            for phrase in ("root clearance", "root tilt", "root angular speed")
        ),
    }


def installed_isaac_identity(isaac_root: Path = Path("C:/isaacsim")) -> dict[str, Any]:
    """Hash the exact installed source contracts used by the runtime adapter."""

    files: dict[str, str] = {}
    for relative in ISAAC_API_SOURCE_FILES:
        path = isaac_root / relative
        if not path.is_file():
            raise RuntimeError(f"required installed Isaac API source is missing: {path}")
        files[relative] = _sha256(path)
    version = (isaac_root / "VERSION").read_text(encoding="utf-8").strip()
    if not version.startswith("6.1.0"):
        raise RuntimeError(f"expected Isaac Sim 6.1.0, found {version!r}")
    return {
        "root": str(isaac_root.resolve()),
        "version": version,
        "api_source_files_sha256": files,
        "contact_raw_data_world_frame_documentation": (
            "https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/api/"
            "structisaacsim_1_1sensors_1_1experimental_1_1physics_1_1_contact_raw_data.html"
        ),
    }


def _maximum_contacting_sole_slip(samples: list[dict[str, Any]], side: str) -> float:
    maximum = 0.0
    for previous, current in zip(samples, samples[1:]):
        if previous[f"{side}_contact"] and current[f"{side}_contact"]:
            previous_position = previous[f"{side}_sole_position_m"]
            current_position = current[f"{side}_sole_position_m"]
            maximum = max(
                maximum,
                math.hypot(
                    current_position[0] - previous_position[0],
                    current_position[1] - previous_position[1],
                ),
            )
    return maximum


def _maximum_lagged_target_error(samples: list[dict[str, Any]]) -> float:
    maximum = 0.0
    for previous, current in zip(samples, samples[1:]):
        for name, target in previous["commanded_joint_targets_rad"].items():
            maximum = max(
                maximum,
                abs(float(current["joint_position_rad"][name]) - float(target)),
            )
    return maximum


def run_isaac(
    args: argparse.Namespace,
    repo: Path,
    initial_identity: dict[str, Any],
    output: Path,
) -> dict[str, Any]:
    # All Isaac imports remain below this boundary.  Calling this function is
    # the explicit heavy-job action; importing the harness is CPU-safe.
    from isaacsim import SimulationApp

    simulation_app = None
    timeline = None
    status_path = output / "locomotion_smoke_status.json"
    try:
        simulation_app = SimulationApp(
            {
                "headless": bool(args.headless),
                "renderer": "RaytracedLighting",
                # Isaac Sim 6.1 defaults this to True, and close() then terminates
                # through os._exit().  Graceful shutdown must return so runtime
                # exceptions and exit status reach _execute_smoke's durable report.
                "fast_shutdown": False,
            }
        )
        import omni.timeline
        import omni.usd
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.experimental.utils import app as app_utils
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.sensors.experimental.physics import Contact, ContactSensor
        from pxr import Gf, UsdGeom, UsdPhysics

        spec = load_production_spec(repo / "robot_spike" / "production")
        usd_path = IsaacRobotLoader(spec).import_urdf(output / "derived_usd")
        omni.usd.get_context().open_stage(str(usd_path))
        app_utils.update_app(steps=8)
        stage = omni.usd.get_context().get_stage()
        meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
        if not math.isclose(meters_per_unit, 1.0, rel_tol=0.0, abs_tol=1.0e-12):
            raise RuntimeError(
                f"imported stage units are {meters_per_unit} metres per unit; expected 1.0"
            )
        default_prim = stage.GetDefaultPrim()
        if not default_prim.IsValid():
            raise RuntimeError("imported USD has no default prim")
        physics_variant = default_prim.GetVariantSets().GetVariantSet("Physics")
        if not physics_variant.IsValid():
            raise RuntimeError("imported USD has no Physics variant")
        physics_variant.SetVariantSelection("physx")

        floor = UsdGeom.Cube.Define(stage, "/LocomotionSmoke/Ground")
        floor.GetSizeAttr().Set(1.0)
        floor_xform = UsdGeom.XformCommonAPI(floor)
        floor_xform.SetTranslate(Gf.Vec3d(args.forward_m / 2.0, 0.0, -0.025))
        floor_xform.SetScale(Gf.Vec3f(max(2.0, args.forward_m + 1.0), 2.0, 0.05))
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

        articulation_roots = [
            prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        if len(articulation_roots) != 1:
            raise RuntimeError(f"expected one articulation root, found {len(articulation_roots)}")
        articulation_root_path = str(articulation_roots[0].GetPath())
        articulation = Articulation(articulation_root_path)
        if len(articulation.link_paths) != 1:
            raise RuntimeError(f"expected one articulation instance, got {len(articulation.link_paths)}")
        link_paths = list(articulation.link_paths[0])
        if len(link_paths) != len(articulation.link_names):
            raise RuntimeError("articulation link path/name counts differ")
        for path in link_paths:
            if not stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError(f"articulation link is not a rigid body: {path}")
        link_poses = RigidPrim(link_paths, resolve_paths=False)

        link_path_by_name = dict(zip(articulation.link_names, link_paths))
        sole_translation = [list(ASIMOV_SOLE_GEOMETRY.reference_m)]
        sensors = {}
        for side in ("left", "right"):
            link_path = link_path_by_name[f"{side}_ankle_roll_link"]
            sensor_path = f"{link_path}/locomotion_contact_sensor"
            sensors[side] = ContactSensor(
                Contact.create(
                    sensor_path,
                    translations=sole_translation,
                    min_threshold=1.0,
                    max_threshold=100000.0,
                    radius=0.095,
                )
            )

        SimulationManager.set_physics_dt(1.0 / args.physics_hz)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        app_utils.update_app(steps=3)
        physics_dt = float(SimulationManager.get_physics_dt())
        if not math.isclose(physics_dt, 1.0 / args.physics_hz, rel_tol=0.0, abs_tol=1.0e-9):
            raise RuntimeError(f"physics dt mismatch: requested {1.0 / args.physics_hz}, got {physics_dt}")

        joint_controller = ArticulationController(spec, articulation)
        adapter = Isaac61LocomotionFeedbackAdapter(
            articulation,
            link_poses,
            sensors["left"],
            sensors["right"],
            SimulationManager.get_simulation_time,
            maximum_contact_age_s=2.5 * physics_dt,
        )
        guard = EvidenceIdentityGuard(
            initial_identity,
            lambda: evidence_identity(
                repo,
                args.expected_candidate_sha,
                args.expected_candidate_tree_sha,
                args.acceptance_spec,
                require_clean=False,
            ),
        )
        result: dict[str, Any] = {
            "schema_version": 1,
            "status": "running",
            "initial_evidence_identity": initial_identity,
            "initial_evidence_identity_sha256": _identity_sha256(initial_identity),
            "identity_checks": guard.checks,
            "runtime": {
                "physics_engine": "physx",
                "physics_hz": args.physics_hz,
                "physics_dt_s": physics_dt,
                "stage_meters_per_unit": meters_per_unit,
                "headless": bool(args.headless),
                "articulation_root_path": articulation_root_path,
                "dof_names": list(articulation.dof_names),
                "link_names": list(articulation.link_names),
                "contact_sensor_paths": {
                    side: f"{link_path_by_name[f'{side}_ankle_roll_link']}/locomotion_contact_sensor"
                    for side in ("left", "right")
                },
                "sole_geometry": {
                    "reference_m": list(ASIMOV_SOLE_GEOMETRY.reference_m),
                    "collision_sphere_centers_m": [
                        list(center)
                        for center in ASIMOV_SOLE_GEOMETRY.collision_sphere_centers_m
                    ],
                    "contact_sphere_radius_m": ASIMOV_SOLE_GEOMETRY.contact_sphere_radius_m,
                    "source": "production URDF ankle-roll collision spheres",
                },
                "support_inference": {
                    "enabled_only_when_raw_hull_is_insufficient": True,
                    "requires_positive_contact_force": True,
                    "sphere_support_direction_world": [0.0, 0.0, -1.0],
                    "plane_tolerance_m": adapter.support_plane_tolerance_m,
                    "raw_point_sphere_xy_tolerance_m": (
                        ASIMOV_SOLE_GEOMETRY.contact_sphere_radius_m
                        + adapter.support_plane_tolerance_m
                    ),
                    "inset_fraction": adapter.inferred_support_inset_fraction,
                },
            },
            "parameters": {
                "repeats": args.repeats,
                "forward_m": args.forward_m,
                "settle_s": args.settle_s,
                "timeout_s": args.timeout_s,
                "first_swing": FootSide.LEFT.value,
            },
            "state_write_policy": {
                "explicit_reset_per_repeat": True,
                "post_reset_root_writes": 0,
                "post_reset_joint_position_writes": 0,
                "normal_motion_command": "set_dof_position_targets via ArticulationController",
            },
            "repeats": [],
        }
        status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

        timeout_steps = max(1, math.ceil(args.timeout_s / physics_dt))
        for repeat_index in range(args.repeats):
            sample_path = output / f"repeat_{repeat_index:02d}_samples.jsonl"
            repeat_result: dict[str, Any] = {
                "repeat_index": repeat_index,
                "status": "running",
                "samples_path": sample_path.name,
                "identity_checks": [],
            }
            result["repeats"].append(repeat_result)
            status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            _record_identity_boundary(
                guard,
                f"repeat_{repeat_index}_start",
                result,
                status_path,
                repeat_result,
            )
            # This is the only state-writing operation in a repeat.  Every
            # later robot command is a drive target and physics advances state.
            joint_controller.reset()
            recording_source = _RecordingFeedbackSource(adapter)

            def persist_startup(startup_report: dict[str, Any]) -> None:
                repeat_result["startup"] = startup_report
                status_path.write_text(
                    json.dumps(result, indent=2) + "\n", encoding="utf-8"
                )

            try:
                initial, startup_report = _staged_double_support_startup(
                    feedback_source=recording_source,
                    joint_controller=joint_controller,
                    step_physics=SimulationManager.step,
                    crouch_targets=symmetric_crouch_targets(spec),
                    physics_dt=physics_dt,
                    maximum_duration_s=args.settle_s,
                    progress=persist_startup,
                )
                repeat_result["startup"] = startup_report
            except StartupValidationError as exc:
                repeat_result["startup"] = exc.report
                _persist_feedback_failure(
                    adapter=adapter,
                    repeat_result=repeat_result,
                    result=result,
                    status_path=status_path,
                    phase=f"staged_startup_{exc.phase}",
                    error=exc,
                )
                raise
            target = PlanarPose(
                initial.root_pose.x_m + args.forward_m,
                initial.root_pose.y_m,
                initial.root_pose.yaw_rad,
            )
            controller = BipedLocomotionController(spec, joint_controller, recording_source)
            controller.start_route([target], first_swing=FootSide.LEFT)
            samples: list[dict[str, Any]] = []
            command = None
            with sample_path.open("x", encoding="utf-8") as sample_stream:
                for step in range(timeout_steps):
                    try:
                        command = controller.update()
                    except FeedbackUnavailableError as exc:
                        _persist_feedback_failure(
                            adapter=adapter,
                            repeat_result=repeat_result,
                            result=result,
                            status_path=status_path,
                            phase=f"controller_feedback_step_{step}",
                            error=exc,
                        )
                        raise
                    feedback = recording_source.last
                    if feedback is None:
                        raise FeedbackUnavailableError("controller update produced no feedback sample")
                    sample = _sample(step, command, feedback, adapter)
                    samples.append(sample)
                    sample_stream.write(json.dumps(sample, sort_keys=True) + "\n")
                    sample_stream.flush()
                    if command.state in {LocomotionState.DOCKED, LocomotionState.FAULT}:
                        break
                    SimulationManager.step()
            if command is None:
                raise RuntimeError("locomotion loop did not execute")
            metrics = _repeat_metrics(samples, command, target)
            _record_identity_boundary(
                guard,
                f"repeat_{repeat_index}_end",
                result,
                status_path,
                repeat_result,
            )
            repeat_result.update(
                {
                    "status": metrics["status"],
                    "target": {"x_m": target.x_m, "y_m": target.y_m, "yaw_rad": target.yaw_rad},
                    "metrics": metrics,
                    "samples_sha256": _sha256(sample_path),
                }
            )
            result["status"] = "running" if metrics["status"] == "pass" else "fail"
            status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            if metrics["status"] != "pass":
                break

        result["repeatability"] = _repeatability(result["repeats"])
        all_repeats_passed = len(result["repeats"]) == args.repeats and all(
            item["metrics"]["status"] == "pass" for item in result["repeats"]
        )
        if all_repeats_passed:
            _record_identity_boundary(
                guard, "final_success", result, status_path
            )
        result["status"] = "pass" if all_repeats_passed else "fail"
        status_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        return result
    except BaseException as exc:
        _persist_pre_shutdown_runtime_error(
            initial_identity=initial_identity,
            status_path=status_path,
            error=exc,
            partial_result=locals().get("result"),
        )
        raise
    finally:
        if timeline is not None:
            try:
                timeline.stop()
            except Exception:
                pass
        if simulation_app is not None:
            simulation_app.close()


def _repeatability(repeats: list[dict[str, Any]]) -> dict[str, Any]:
    if len(repeats) < 2:
        return {"available": False, "reason": "fewer than two completed repeats"}
    first = repeats[0]["metrics"]
    comparisons = []
    for repeat in repeats[1:]:
        metrics = repeat["metrics"]
        comparisons.append(
            {
                "repeat_index": repeat["repeat_index"],
                "root_xy_displacement_delta_m": metrics["root_xy_displacement_m"] - first["root_xy_displacement_m"],
                "elapsed_delta_s": metrics["elapsed_s"] - first["elapsed_s"],
                "minimum_support_margin_delta_m": metrics["minimum_support_margin_m"] - first["minimum_support_margin_m"],
            }
        )
    return {"available": True, "reference_repeat_index": 0, "comparisons": comparisons}


def _execute_smoke(
    args: argparse.Namespace,
    repo: Path,
    *,
    identity_reader: Callable[..., dict[str, Any]] = evidence_identity,
    runtime_runner: Callable[..., dict[str, Any]] = run_isaac,
) -> int:
    """Run preflight and runtime while durably reporting every identity failure."""

    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"refusing to overwrite output: {output}")
    output.mkdir(parents=True)
    status_path = output / "locomotion_smoke_status.json"
    try:
        initial_identity = identity_reader(
            repo,
            args.expected_candidate_sha,
            args.expected_candidate_tree_sha,
            args.acceptance_spec,
            require_clean=True,
        )
    except BaseException as exc:
        failure = {
            "schema_version": 1,
            "status": "error",
            "phase": "preflight_identity",
            "runtime_invoked": False,
            "requested_identity": {
                "expected_candidate_sha": args.expected_candidate_sha,
                "expected_candidate_tree_sha": args.expected_candidate_tree_sha,
                "acceptance_spec_path": str(args.acceptance_spec),
                "expected_acceptance_spec_sha256": OWNER_SPEC_SHA256,
            },
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }
        try:
            status_path.write_text(
                json.dumps(failure, indent=2) + "\n", encoding="utf-8"
            )
        except Exception as report_exc:
            failure["status_report_error"] = (
                f"{type(report_exc).__name__}: {report_exc}"
            )
        print(json.dumps(failure, indent=2), file=sys.stderr, flush=True)
        return 2

    try:
        result = runtime_runner(args, repo, initial_identity, output)
        # The real runner writes incremental evidence before shutdown.  Write
        # its returned terminal result again after graceful close has returned,
        # proving that the caller regained control before reporting success.
        status_path.write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
    except BaseException as exc:
        partial_result = None
        if status_path.is_file():
            try:
                partial_result = json.loads(status_path.read_text(encoding="utf-8"))
            except Exception:
                partial_result = {"status": "unreadable_partial_status"}
        failure = {
            "schema_version": 1,
            "status": "error",
            "phase": "runtime",
            "runtime_invoked": True,
            "initial_evidence_identity": initial_identity,
            "initial_evidence_identity_sha256": _identity_sha256(initial_identity),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
            "partial_result": partial_result,
        }
        status_path.write_text(
            json.dumps(failure, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(failure, indent=2), file=sys.stderr, flush=True)
        return 2
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["status"] == "pass" else 1


def main(argv: list[str] | None = None) -> int:
    args = _arguments(argv)
    _validate_arguments(args)
    repo = Path(__file__).resolve().parents[2]
    return _execute_smoke(args, repo)


def _terminate_process(
    exit_code: int, *, hard_exit: Callable[[int], object] = os._exit
) -> None:
    """Flush reports, then set kit.exe's status for the python.bat launcher."""

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except Exception:
            pass
    hard_exit(int(exit_code))


if __name__ == "__main__":
    _terminate_process(main())
