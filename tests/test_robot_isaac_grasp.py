from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import tempfile
import unittest

from robot_spike.production.isaac_feedback import FeedbackUnavailableError
from robot_spike.production.isaac_grasp import (
    Isaac61GraspFeedbackAdapter,
    IsaacArmLiftPort,
    IsaacContactBindings,
)
from robot_spike.production.arm_reach import ARM_DOF_NAMES, ToolPose
from robot_spike.production.model import load_production_spec
from robot_spike.production.physical_grasp import (
    GraspLimits,
    GraspPhase,
    HAND_DOF_NAMES,
    LiftRequest,
    PhysicalGraspController,
    RobotContactBodyMap,
)
from robot_spike.production.run_grasp_smoke import (
    GraspSmokeRunner,
    exact_contact_bindings,
    runtime_preflight_evidence,
    run_isaac,
)
from robot_spike.production.runtime import ArticulationController
from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.restocking_layout import build_restocking_layout


ROOT = Path(__file__).resolve().parents[1]


class FakeArray:
    def __init__(self, value):
        self.value = value

    def numpy(self):
        return self

    def tolist(self):
        return self.value


class FakeArticulation:
    def __init__(self, spec):
        self.dof_names = tuple(reversed(spec.canonical_dof_order))
        self.link_names = tuple(spec.model.link_names)
        self.positions = {name: spec.reset_joint_positions[name] for name in self.dof_names}
        self.targets = []

    def get_dof_positions(self):
        return FakeArray([[self.positions[name] for name in self.dof_names]])

    def set_dof_position_targets(self, values, *, dof_indices):
        self.targets.append((values, dof_indices))
        for value, index in zip(values[0], dof_indices):
            self.positions[self.dof_names[index]] = float(value)


class FakePose:
    def __init__(self, position, orientation_xyzw):
        self.position = list(position)
        x, y, z, w = orientation_xyzw
        self.orientation_wxyz = [w, x, y, z]

    def get_world_poses(self):
        return FakeArray([self.position]), FakeArray([self.orientation_wxyz])


class Reading:
    def __init__(self, time=1.0, *, valid=True, in_contact=True, value=10.0):
        self.time = time
        self.is_valid = valid
        self.in_contact = in_contact
        self.value = value


class FakeSensor:
    def __init__(self, clock, records):
        self.clock = clock
        self.records = records
        self.valid = True
        self.raw_override = None

    def get_sensor_reading(self):
        return Reading(self.clock[0], valid=self.valid, in_contact=bool(self.records))

    def get_raw_data(self):
        if self.raw_override is not None:
            return self.raw_override
        result = []
        for body0, body1, impulse in self.records:
            result.append({
                "body0": body0,
                "body1": body1,
                "impulse": {"x": impulse, "y": 0.0, "z": 0.0},
                "normal": {"x": 1.0, "y": 0.0, "z": 0.0},
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "time": self.clock[0],
                "dt": 0.01,
            })
        return result


class AcceptingLift:
    def __init__(self, product_pose, palm_pose, sensor=None):
        self.requests = []
        self.product_pose = product_pose
        self.palm_pose = palm_pose
        self.sensor = sensor

    def request_lift(self, request):
        self.requests.append(request)
        return True

    def advance(self):
        self.product_pose.position[2] += 0.06
        self.palm_pose.position[2] += 0.06
        if self.sensor is not None:
            self.sensor.records = [record for record in self.sensor.records if 2 not in record[:2]]
        return True


class FakePhysics:
    def __init__(self, clock):
        self.clock = clock

    def step(self):
        self.clock[0] += 0.01


class FakeIK:
    def __init__(self, mapping):
        self.mapping = mapping

    def as_mapping(self):
        return dict(self.mapping)


class FakeKinematics:
    def forward(self, positions):
        return ToolPose((0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))

    def solve(self, target, start):
        return FakeIK({name: 0.01 for name in ARM_DOF_NAMES})


class FakePlanner:
    def maximum_step_for_joint(self, name):
        return 1.0


class RecordingController:
    def __init__(self):
        self.calls = []

    def command_joint_positions(self, targets):
        self.calls.append(dict(targets))
        return ARM_DOF_NAMES


class IsaacGraspAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = load_production_spec(ROOT / "robot_spike" / "production")
        scenario = load_scenario(ROOT / "config/scenarios/baseline_straight.yaml")
        cls.layout = build_restocking_layout(build_aisle_layout(scenario.environment))
        cls.paths = {link: f"/World/Robot/{link}" for link in (
            "right_thumb_ip", "right_thumb_dp", "right_index_ip", "right_middle_ip",
            "right_ring_ip", "right_pinky_ip", "right_palm",
        )}
        cls.robot_map = RobotContactBodyMap("/World/Robot", cls.paths)
        cls.bindings = IsaacContactBindings(
            cls.robot_map,
            cls.layout.product.rigid_body_prim_path,
            cls.layout.product.collider_prim_path,
            cls.layout.pickup_support.fixture.prim_path,
        )

    def make_adapter(self, records=None):
        clock = [1.0]
        articulation = FakeArticulation(self.spec)
        source = self.layout.product.source_reset_pose
        palm = FakePose((source.position_m[0], source.position_m[1] - 0.1, source.position_m[2]),
                        (0.0, 0.0, 0.0, 1.0))
        product = FakePose(source.position_m, source.orientation_xyzw)
        handles = {
            1: self.layout.product.rigid_body_prim_path,
            2: self.layout.pickup_support.fixture.prim_path,
            3: self.paths["right_thumb_dp"],
            4: self.paths["right_index_ip"],
        }
        sensor = FakeSensor(clock, records if records is not None else [(1, 2, 0.03), (1, 3, 0.01), (1, 4, 0.01)])
        adapter = Isaac61GraspFeedbackAdapter(
            articulation, palm, product, sensor, lambda: clock[0],
            lambda handle: handles[handle], self.bindings,
            maximum_contact_age_s=0.025,
        )
        return adapter, articulation, palm, product, sensor, clock, handles

    def test_raw_impulse_paths_and_poses_become_measured_observation(self):
        adapter, _, _, _, _, _, _ = self.make_adapter()
        observed = adapter.read_observation()
        self.assertEqual(observed.product_prim_path, self.layout.product.rigid_body_prim_path)
        contacts = {item.body1_prim_path: item for item in observed.contacts}
        self.assertAlmostEqual(contacts[self.paths["right_thumb_dp"]].normal_force_n, 1.0)
        self.assertEqual(contacts[self.paths["right_thumb_dp"]].robot_link_name, "right_thumb_dp")
        self.assertEqual(contacts[self.layout.pickup_support.fixture.prim_path].robot_link_name, None)
        self.assertEqual(observed.product_pose_world.orientation_xyzw,
                         self.layout.product.source_reset_pose.orientation_xyzw)

    def test_marker_or_unbound_body_path_fails_closed(self):
        adapter, _, _, _, _, _, handles = self.make_adapter([(1, 9, 0.01)])
        handles[9] = "/World/Robot/right_thumb_fingertip"
        with self.assertRaisesRegex(FeedbackUnavailableError, "unbound contact body"):
            adapter.read_observation()

    def test_collapsed_body_pair_fails_closed(self):
        adapter, _, _, _, _, _, _ = self.make_adapter([(1, 1, 0.01)])
        with self.assertRaisesRegex(FeedbackUnavailableError, "collapsed"):
            adapter.read_observation()

    def test_fractional_nonfinite_and_boolean_body_handles_fail_closed(self):
        for malformed in (1.9, math.nan, True, "1"):
            with self.subTest(handle=malformed):
                adapter, _, _, _, sensor, clock, _ = self.make_adapter()
                sensor.raw_override = [{
                    "body0": malformed,
                    "body1": 3,
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "normal": {"x": 1.0, "y": 0.0, "z": 0.0},
                    "impulse": {"x": 0.01, "y": 0.0, "z": 0.0},
                    "time": clock[0],
                    "dt": 0.01,
                }]
                with self.assertRaisesRegex(FeedbackUnavailableError, "handle must be an integer"):
                    adapter.read_observation()

    def test_missing_or_invalid_raw_geometry_fails_closed(self):
        cases = (
            ("missing position", None, {"x": 1.0, "y": 0.0, "z": 0.0}),
            ("missing normal", {"x": 0.0, "y": 0.0, "z": 0.0}, None),
            ("nonfinite position", {"x": math.inf, "y": 0.0, "z": 0.0},
             {"x": 1.0, "y": 0.0, "z": 0.0}),
            ("zero normal", {"x": 0.0, "y": 0.0, "z": 0.0},
             {"x": 0.0, "y": 0.0, "z": 0.0}),
            ("nonfinite normal", {"x": 0.0, "y": 0.0, "z": 0.0},
             {"x": 1.0, "y": math.nan, "z": 0.0}),
        )
        for label, position, normal in cases:
            with self.subTest(case=label):
                adapter, _, _, _, sensor, clock, _ = self.make_adapter()
                raw = {
                    "body0": 1,
                    "body1": 3,
                    "impulse": {"x": 0.01, "y": 0.0, "z": 0.0},
                    "time": clock[0],
                    "dt": 0.01,
                }
                if position is not None:
                    raw["position"] = position
                if normal is not None:
                    raw["normal"] = normal
                sensor.raw_override = [raw]
                with self.assertRaises(FeedbackUnavailableError):
                    adapter.read_observation()

    def test_stale_and_invalid_contact_data_fail_closed(self):
        adapter, _, _, _, sensor, clock, _ = self.make_adapter()
        sensor.valid = False
        with self.assertRaisesRegex(FeedbackUnavailableError, "invalid"):
            adapter.read_observation()
        adapter, _, _, _, sensor, clock, _ = self.make_adapter()
        original = sensor.get_sensor_reading
        sensor.get_sensor_reading = lambda: Reading(clock[0] - 1.0)  # type: ignore[method-assign]
        with self.assertRaisesRegex(FeedbackUnavailableError, "stale"):
            adapter.read_observation()
        sensor.get_sensor_reading = original  # type: ignore[method-assign]

    def test_binding_rejects_missing_or_marker_links(self):
        names = list(self.spec.model.link_names)
        paths = [f"/World/Robot/{name}" for name in names]
        binding = exact_contact_bindings(
            "/World/Robot", names, paths,
            self.layout.product.rigid_body_prim_path,
            self.layout.product.collider_prim_path,
            self.layout.pickup_support.fixture.prim_path,
        )
        self.assertEqual(binding.robot_contacts.link_body_prim_paths["right_thumb_dp"],
                         "/World/Robot/right_thumb_dp")
        names.remove("right_thumb_dp")
        paths = [f"/World/Robot/{name}" for name in names]
        with self.assertRaisesRegex(RuntimeError, "required hand links"):
            exact_contact_bindings("/World/Robot", names, paths, "/p", "/p/c", "/s")

    def test_arm_lift_port_binds_product_arm_names_and_inclusive_deadline(self):
        articulation = FakeArticulation(self.spec)
        control = RecordingController()
        clock = [1.0]
        port = IsaacArmLiftPort(
            control, articulation, FakeKinematics(), lambda: clock[0],
            self.layout.product.rigid_body_prim_path, command_period_s=0.01,
        )
        port.planner = FakePlanner()
        wrong = LiftRequest(0.075, 0.02, "/World/Restocking/Wrong")
        self.assertFalse(port.request_lift(wrong))
        request = LiftRequest(0.075, 0.02, self.layout.product.rigid_body_prim_path)
        self.assertTrue(port.request_lift(request))
        clock[0] = 1.02
        self.assertTrue(port.advance())
        self.assertEqual(tuple(control.calls[0]), ARM_DOF_NAMES)
        clock[0] = 1.020001
        self.assertFalse(port.advance())
        self.assertEqual(port.last_error, "arm lift deadline expired")

    def test_smoke_sequence_requires_contacts_and_measured_lift(self):
        adapter, articulation, palm, product, sensor, clock, _ = self.make_adapter()
        joint_controller = ArticulationController(self.spec, articulation)
        lift = AcceptingLift(product, palm, sensor)
        controller = PhysicalGraspController(
            joint_controller, self.layout, feedback=adapter, arm_lift=lift,
            robot_contacts=self.robot_map,
            limits=GraspLimits(contact_confirmation_samples=1, lift_confirmation_samples=1,
                               maximum_observations_per_phase=10),
        )
        with tempfile.TemporaryDirectory() as temporary:
            result = GraspSmokeRunner(
                FakePhysics(clock), controller, lift, adapter.diagnostics,
                maximum_physics_steps=8,
                status_path=Path(temporary) / "status.json",
                preflight={"test": True},
            ).run()
            report = json.loads((Path(temporary) / "status.json").read_text(encoding="utf-8"))
        self.assertEqual(result.status, "pass", result)
        self.assertEqual(result.terminal_phase, GraspPhase.COMPLETE.value)
        self.assertEqual(report["status"], "pass")
        commanded_sets = []
        for _, indices in articulation.targets:
            commanded_sets.append({articulation.dof_names[index] for index in indices})
        self.assertTrue(all(names == set(HAND_DOF_NAMES) for names in commanded_sets))
        self.assertGreaterEqual(len(lift.requests), 1)

    def test_smoke_persists_fail_closed_diagnostics(self):
        adapter, articulation, palm, product, sensor, clock, handles = self.make_adapter([(1, 9, 0.01)])
        handles[9] = "/World/Robot/right_index_fingertip"
        joint_controller = ArticulationController(self.spec, articulation)
        lift = AcceptingLift(product, palm)
        controller = PhysicalGraspController(
            joint_controller, self.layout, feedback=adapter, arm_lift=lift,
            robot_contacts=self.robot_map,
            limits=GraspLimits(maximum_observations_per_phase=2),
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "failed.json"
            result = GraspSmokeRunner(
                FakePhysics(clock), controller, lift, adapter.diagnostics,
                maximum_physics_steps=2, status_path=path, preflight={"test": True},
            ).run()
            report = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(result.status, "fail")
        self.assertEqual(result.failure, "feedback_unavailable")
        self.assertEqual(report["status"], "fail")
        self.assertEqual(report["state_write_policy"]["post_reset_direct_state_writes"], 0)

    def test_preflight_failure_receipt_binds_expected_and_observed_identities(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            args = argparse.Namespace(
                output=output,
                acceptance_spec=(
                    Path.home()
                    / "Downloads"
                    / "ROBOT_BACKROOM_RESTOCKING_IMPLEMENTATION_SPEC.md"
                ),
                expected_candidate_sha="0" * 40,
                expected_candidate_tree_sha="1" * 40,
                isaac_root=Path("C:/isaacsim"),
            )
            with self.assertRaisesRegex(RuntimeError, "identity preflight failed"):
                run_isaac(args, ROOT)
            report = json.loads(
                (output / "grasp_smoke_status.json").read_text(encoding="utf-8")
            )
        self.assertEqual(report["status"], "error")
        identity = report["identity_preflight"]
        self.assertEqual(identity["status"], "fail")
        self.assertEqual(identity["expected"]["candidate_commit"], "0" * 40)
        self.assertEqual(identity["expected"]["candidate_tree"], "1" * 40)
        self.assertEqual(
            identity["expected"]["acceptance_spec_sha256"],
            "7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad",
        )
        self.assertEqual(
            identity["observed"]["isaac_build"],
            "6.1.0-rc.26+release.49347.2d230af4.gl",
        )
        for name in (
            "candidate_commit",
            "candidate_tree",
            "acceptance_spec_sha256",
            "combined_source_urdf_canonical_sha256",
            "production_urdf_canonical_sha256",
            "scenario_sha256",
            "robot_config_sha256",
            "isaac_build",
        ):
            self.assertIn(name, identity["observed"])

    def test_prepopulated_output_cannot_mix_stale_failure_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            status_path = output / "grasp_smoke_status.json"
            status_path.write_text(
                json.dumps({"status": "pass", "stale_success": True}), encoding="utf-8"
            )
            (output / "grasp_smoke_status.json.tmp").write_text(
                json.dumps({"stale_temporary": True}), encoding="utf-8"
            )
            args = argparse.Namespace(
                output=output,
                acceptance_spec=(
                    Path.home()
                    / "Downloads"
                    / "ROBOT_BACKROOM_RESTOCKING_IMPLEMENTATION_SPEC.md"
                ),
                expected_candidate_sha="a" * 40,
                expected_candidate_tree_sha="b" * 40,
                isaac_root=Path("C:/isaacsim"),
            )
            with self.assertRaisesRegex(RuntimeError, "identity preflight failed"):
                run_isaac(args, ROOT)
            report = json.loads(status_path.read_text(encoding="utf-8"))
        self.assertNotIn("stale_success", report)
        self.assertNotIn("stale_temporary", report)
        self.assertEqual(report["status"], "error")
        self.assertEqual(
            report["identity_preflight"]["expected"]["candidate_commit"], "a" * 40
        )

    def test_runtime_preflight_binds_exact_build_scenario_config_and_allowlist(self):
        scenario_path = ROOT / "config" / "scenarios" / "baseline_straight.yaml"
        robot_config_path = ROOT / "robot_spike" / "production" / "robot_config.json"
        identity = {
            "status": "pass",
            "expected": {},
            "observed": {
                "isaac_build": "6.1.0-rc.26+release.49347.2d230af4.gl",
                "scenario_sha256": "12d41a428dfe0e82da60584da9595b1c74090b33f375cc8be60b237941d8a5d0",
                "robot_config_sha256": "b6b2a0a524b65e683a69324debd212f2e91b7e446f8141d2b11f6ab75b473fe7",
                "scenario_path": str(scenario_path.resolve()),
                "robot_config_path": str(robot_config_path.resolve()),
                "isaac_root": str(Path("C:/isaacsim").resolve()),
            },
            "mismatches": [],
        }
        preflight = runtime_preflight_evidence(
            identity,
            self.bindings,
            scenario_path=scenario_path,
            robot_config_path=robot_config_path,
            isaac_root=Path("C:/isaacsim"),
            physics_dt_s=1.0 / 120.0,
            robot_link_count=len(self.spec.model.link_names),
            robot_dof_count=len(self.spec.canonical_dof_order),
            product_contact_sensor_path=(
                self.layout.product.rigid_body_prim_path + "/grasp_contact_sensor"
            ),
        )
        self.assertEqual(
            preflight["installed_isaac"]["build"],
            "6.1.0-rc.26+release.49347.2d230af4.gl",
        )
        self.assertEqual(preflight["scenario"]["path"], str(scenario_path.resolve()))
        self.assertEqual(
            preflight["robot_config"]["path"], str(robot_config_path.resolve())
        )
        self.assertEqual(
            preflight["contact_bindings"]["robot_link_body_prim_paths"],
            dict(sorted(self.paths.items())),
        )
        self.assertEqual(
            preflight["contact_bindings"]["complete_allowed_body_paths"],
            sorted(self.bindings.allowed_body_paths),
        )


if __name__ == "__main__":
    unittest.main()
