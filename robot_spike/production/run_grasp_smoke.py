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
from typing import Callable, Protocol

from robot_spike.production.arm_reach import RightArmKinematics
from robot_spike.production.isaac_grasp import (
    Isaac61GraspFeedbackAdapter,
    IsaacArmLiftPort,
    IsaacContactBindings,
)
from robot_spike.production.model import load_production_spec
from robot_spike.production.physical_grasp import (
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


class PhysicsStepper(Protocol):
    def step(self) -> None: ...


class AdvancingLiftPort(Protocol):
    def advance(self) -> bool: ...


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


def candidate_identity(repo: Path, acceptance_spec: Path) -> dict[str, object]:
    manifest_path = repo / "robot_spike" / "production" / "production_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    return {
        "candidate_commit": _git(repo, "rev-parse", "HEAD"),
        "candidate_tree": _git(repo, "rev-parse", "HEAD^{tree}"),
        "worktree_clean": not bool(_git(repo, "status", "--porcelain")),
        "production_urdf_sha256": _sha256(
            repo / "robot_spike" / "production" / "asimov_orcahand_restocking.urdf"
        ),
        "production_urdf_canonical_sha256": manifest["production_urdf_canonical_sha256"],
        "combined_source_urdf_canonical_sha256": manifest["source_urdf_canonical_sha256"],
        "acceptance_spec_path": str(acceptance_spec.resolve()),
        "acceptance_spec_sha256": _sha256(acceptance_spec),
    }


def write_durable_json(path: Path, value: object) -> None:
    """Atomically replace a report and fsync both file and containing directory."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    payload = json.dumps(value, indent=2, sort_keys=True) + "\n"
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
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
        arm_lift: AdvancingLiftPort,
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
        self.arm_lift = arm_lift
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
        try:
            self.controller.start()
        except Exception as exc:
            self._report("error", samples, str(exc))
            raise
        self._report("running", samples)
        error: str | None = None
        for step in range(self.maximum_physics_steps):
            if self.controller.phase is GraspPhase.LIFTING and not self.arm_lift.advance():
                error = "arm lift port could not advance within its bounded deadline"
            self.physics.step()
            status = self.controller.step()
            sample = {
                "step": step,
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
        if terminal.phase not in (GraspPhase.COMPLETE, GraspPhase.FAILED):
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


def run_isaac(args: argparse.Namespace, repo: Path) -> SmokeResult:
    """Build and execute the real smoke scene; this function starts Isaac."""

    app = None
    timeline = None
    status_path = args.output / "grasp_smoke_status.json"
    try:
        initial_identity = candidate_identity(repo, args.acceptance_spec)
        mismatches = []
        if initial_identity["candidate_commit"] != args.expected_candidate_sha:
            mismatches.append("candidate commit")
        if initial_identity["candidate_tree"] != args.expected_candidate_tree_sha:
            mismatches.append("candidate tree")
        if initial_identity["acceptance_spec_sha256"] != ACCEPTANCE_SPEC_SHA256:
            mismatches.append("acceptance specification")
        if initial_identity["worktree_clean"] is not True:
            mismatches.append("worktree cleanliness")
        if mismatches:
            raise RuntimeError("identity preflight failed: " + ", ".join(mismatches))
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "fast_shutdown": False})
        import omni.timeline
        import omni.usd
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.experimental.utils import app as app_utils
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.sensors.experimental.physics import Contact, ContactSensor
        from pxr import PhysicsSchemaTools, UsdGeom, UsdPhysics

        from simulator.environment.isaac_builder import IsaacAisleBuilder
        from simulator.environment.isaac_restocking_builder import IsaacRestockingBuilder

        spec = load_production_spec(repo / "robot_spike" / "production")
        scenario = load_scenario(repo / "config" / "scenarios" / "baseline_straight.yaml")
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
        sensor_path = handles.product_rigid_body_prim_path + "/grasp_contact_sensor"
        sensor = ContactSensor(Contact.create(sensor_path, min_threshold=0.0,
                                               max_threshold=100000.0, radius=-1.0))
        SimulationManager.set_physics_dt(1.0 / args.physics_hz)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        app_utils.update_app(steps=3)
        physics_dt = float(SimulationManager.get_physics_dt())
        joint_controller = ArticulationController(spec, articulation)
        feedback = Isaac61GraspFeedbackAdapter(
            articulation, palm, product, sensor,
            SimulationManager.get_simulation_time,
            lambda handle: str(PhysicsSchemaTools.intToSdfPath(int(handle))),
            bindings,
            maximum_contact_age_s=2.5 * physics_dt,
        )
        lift = IsaacArmLiftPort(
            joint_controller, articulation, RightArmKinematics(spec),
            SimulationManager.get_simulation_time,
            handles.product_rigid_body_prim_path,
            command_period_s=physics_dt,
        )
        grasp = PhysicalGraspController(
            joint_controller, layout, feedback=feedback, arm_lift=lift,
            robot_contacts=bindings.robot_contacts, limits=GraspLimits(),
        )
        joint_controller.reset()

        class _Physics:
            def step(self):
                SimulationManager.step()

        preflight = {
            "identity": initial_identity,
            "isaac_version": "6.1",
            "physics_engine": "physx",
            "physics_dt_s": physics_dt,
            "robot_root": root_path,
            "robot_link_count": len(link_paths),
            "robot_dof_count": len(articulation.dof_names),
            "product_body": handles.product_rigid_body_prim_path,
            "product_collider": handles.product_collider_prim_path,
            "pickup_support": handles.pickup_support_prim_path,
            "product_contact_sensor": sensor_path,
            "sensor_min_threshold_n": 0.0,
            "sensor_radius": -1.0,
        }
        return GraspSmokeRunner(
            _Physics(), grasp, lift, feedback.diagnostics,
            maximum_physics_steps=args.maximum_steps,
            status_path=status_path,
            preflight=preflight,
        ).run()
    except Exception as exc:
        try:
            diagnostic = json.loads(status_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            diagnostic = {"schema_version": 1}
        diagnostic.update({"status": "error", "runtime_error": str(exc)})
        write_durable_json(status_path, diagnostic)
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
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _arguments(argv)
    if not math.isfinite(args.physics_hz) or args.physics_hz <= 0.0 or args.maximum_steps < 1:
        raise SystemExit("physics-hz and maximum-steps must be positive")
    repo = Path(__file__).resolve().parents[2]
    return 0 if run_isaac(args, repo).status == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
