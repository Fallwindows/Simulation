from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from robot_spike.production.model import (
    JointTargetError,
    ModelValidationError,
    UrdfModel,
    canonical_text_sha256,
    load_production_spec,
    validate_mesh_geometry,
)
from robot_spike.production.runtime import ArticulationController


ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_ROOT = ROOT / "robot_spike" / "production"


class FakeArticulation:
    def __init__(self, dof_names):
        self.dof_names = tuple(dof_names)
        self.calls = []

    def _record(self, name, args, kwargs):
        self.calls.append((name, args, kwargs))

    def set_dof_position_targets(self, *args, **kwargs):
        self._record("set_dof_position_targets", args, kwargs)

    def set_dof_positions(self, *args, **kwargs):
        self._record("set_dof_positions", args, kwargs)

    def set_dof_velocities(self, *args, **kwargs):
        self._record("set_dof_velocities", args, kwargs)

    def set_world_poses(self, *args, **kwargs):
        self._record("set_world_poses", args, kwargs)

    def set_velocities(self, *args, **kwargs):
        self._record("set_velocities", args, kwargs)


class ProductionRobotModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = load_production_spec(PRODUCTION_ROOT)

    def test_exact_source_identity_and_isolated_derivation(self):
        source = ROOT / "robot_spike" / "asimov_orcahand_right.urdf"
        upstream_manifest = ROOT / "robot_spike" / "upstream_manifest.json"
        manifest = json.loads((PRODUCTION_ROOT / "production_manifest.json").read_text())
        self.assertEqual(
            canonical_text_sha256(source),
            "25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea",
        )
        self.assertEqual(
            canonical_text_sha256(upstream_manifest),
            "6ddcd328db7c61de83e6ccea5bbf5a496b714bc9370acb937831f3a21f9a6b80",
        )
        self.assertEqual(manifest["derivation"]["inertial_changes"], 0)
        self.assertEqual(manifest["derivation"]["collision_geometry_changes"], 0)
        self.assertEqual(manifest["derivation"]["joint_changes"], 0)
        self.assertEqual(manifest["derivation"]["link_changes"], 0)

        source_robot = ET.parse(source).getroot()
        production_robot = ET.parse(self.spec.model.path).getroot()
        self.assertEqual(source_robot.attrib["name"], "asimov_1_orcahand_v1_right_spike")
        self.assertEqual(
            production_robot.attrib["name"], "asimov_1_orcahand_v1_right_restocking"
        )
        production_robot.attrib["name"] = source_robot.attrib["name"]
        source_meshes = source_robot.findall(".//mesh")
        production_meshes = production_robot.findall(".//mesh")
        self.assertEqual(len(source_meshes), len(production_meshes))
        for source_mesh, production_mesh in zip(source_meshes, production_meshes):
            self.assertEqual(
                production_mesh.attrib["filename"], f"../{source_mesh.attrib['filename']}"
            )
            production_mesh.attrib["filename"] = source_mesh.attrib["filename"]
        for element in (*source_robot.iter(), *production_robot.iter()):
            if element.text is not None and not element.text.strip():
                element.text = None
            if element.tail is not None and not element.tail.strip():
                element.tail = None
        self.assertEqual(ET.tostring(production_robot), ET.tostring(source_robot))

    def test_complete_asimov_and_orcahand_tree_is_preserved(self):
        model = self.spec.model
        self.assertEqual(model.root_link, "pelvis_link")
        self.assertEqual(
            (len(model.link_names), len(model.joint_names), len(model.dof_names)),
            (66, 65, 40),
        )
        self.assertIn("left_wrist_yaw_link", model.link_names)
        self.assertIn("right_wrist_yaw_link", model.link_names)
        for name in ("right_tower", "right_wrist_jointbody", "right_palm"):
            self.assertIn(name, model.link_names)
        for digit in ("thumb", "index", "middle", "ring", "pinky"):
            self.assertIn(f"right_{digit}_fingertip", model.link_names)
        self.assertEqual(set(self.spec.canonical_dof_order), set(model.dof_names))

    def test_all_meshes_resolve_and_have_finite_meter_scale_geometry(self):
        self.assertEqual(len(self.spec.model.mesh_references), 94)
        self.assertTrue(
            all(ref.scale == (1.0, 1.0, 1.0) for ref in self.spec.model.mesh_references)
        )
        extents = validate_mesh_geometry(self.spec.model)
        self.assertEqual(len(extents), 93)
        self.assertTrue(all(max(value) < 1.0 for value in extents.values()))
        collision_meshes = [
            reference for reference in self.spec.model.mesh_references
            if reference.role == "collision"
        ]
        self.assertEqual(len(collision_meshes), 34)

    def test_missing_mesh_and_bad_scale_fail_closed(self):
        tree = ET.parse(self.spec.model.path)
        first_mesh = tree.getroot().find(".//mesh")
        self.assertIsNotNone(first_mesh)
        with tempfile.TemporaryDirectory() as temporary:
            temp_root = Path(temporary)
            missing = temp_root / "missing.urdf"
            first_mesh.attrib["filename"] = "missing.stl"
            tree.write(missing, encoding="utf-8", xml_declaration=True)
            with self.assertRaisesRegex(ModelValidationError, "missing or empty mesh"):
                UrdfModel.load(missing, temp_root)

        tree = ET.parse(self.spec.model.path)
        first_mesh = tree.getroot().find(".//mesh")
        first_mesh.attrib["filename"] = str(self.spec.model.mesh_references[0].path)
        first_mesh.attrib["scale"] = "1 -1 1"
        with tempfile.TemporaryDirectory() as temporary:
            invalid = Path(temporary) / "scale.urdf"
            tree.write(invalid, encoding="utf-8", xml_declaration=True)
            drive_root = Path(invalid.anchor)
            with self.assertRaisesRegex(ModelValidationError, "mesh scale"):
                UrdfModel.load(invalid, drive_root)

    def test_invalid_joint_tree_and_nonfinite_limits_fail_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            temporary_root = Path(temporary)
            tree = ET.parse(self.spec.model.path)
            joints = tree.getroot().findall("joint")
            joints[1].find("child").attrib["link"] = joints[0].find("child").attrib["link"]
            invalid_tree = temporary_root / "multiple_parent.urdf"
            tree.write(invalid_tree, encoding="utf-8", xml_declaration=True)
            with self.assertRaisesRegex(ModelValidationError, "multiple parents"):
                UrdfModel.load(invalid_tree, temporary_root)

            tree = ET.parse(self.spec.model.path)
            first_limit = tree.getroot().find("joint/limit")
            first_limit.attrib["lower"] = "nan"
            invalid_limit = temporary_root / "nonfinite_limit.urdf"
            tree.write(invalid_limit, encoding="utf-8", xml_declaration=True)
            with self.assertRaisesRegex(ModelValidationError, "must be finite"):
                UrdfModel.load(invalid_limit, temporary_root)

    def test_nonfixed_biped_import_and_assumptions_are_explicit(self):
        self.assertIs(self.spec.importer["fix_base"], False)
        self.assertIs(self.spec.importer["collision_from_visuals"], False)
        self.assertIs(self.spec.importer["allow_self_collision"], False)
        self.assertEqual(len(self.spec.model.zero_mass_links), 5)
        self.assertEqual(len(self.spec.model.tiny_inertia_links), 17)
        self.assertEqual(len(self.spec.model.links_without_collision), 33)

    def test_mapping_is_by_name_and_independent_of_runtime_order(self):
        reversed_names = tuple(reversed(self.spec.canonical_dof_order))
        mapping = self.spec.bind_runtime_dofs(reversed_names)
        self.assertEqual(mapping[reversed_names[0]], 0)
        self.assertEqual(mapping[reversed_names[-1]], 39)
        indices, values, names = self.spec.command_plan(
            reversed_names,
            {"right_index_pip": 0.5, "left_hip_pitch_joint": -0.25},
        )
        self.assertEqual(names, ("left_hip_pitch_joint", "right_index_pip"))
        self.assertEqual(indices, (39, 4))
        self.assertEqual(values, (-0.25, 0.5))
        with self.assertRaisesRegex(ModelValidationError, "right_thumb_dip"):
            self.spec.bind_runtime_dofs(reversed_names[1:])

    def test_joint_targets_enforce_names_finiteness_and_limits(self):
        self.assertEqual(
            self.spec.validate_targets({"right_wrist": 0.8972}),
            {"right_wrist": 0.8972},
        )
        with self.assertRaisesRegex(JointTargetError, "unknown joint"):
            self.spec.validate_targets({"not_a_joint": 0.0})
        with self.assertRaisesRegex(JointTargetError, "must be finite"):
            self.spec.validate_targets({"right_wrist": float("nan")})
        with self.assertRaisesRegex(JointTargetError, "allowed range"):
            self.spec.validate_targets({"right_wrist": 0.8972001})

    def test_motion_uses_drive_targets_without_direct_state_writes(self):
        fake = FakeArticulation(tuple(reversed(self.spec.canonical_dof_order)))
        controller = ArticulationController(self.spec, fake)
        names = controller.command_joint_positions(
            {"right_index_pip": 0.5, "left_hip_pitch_joint": -0.25}
        )
        self.assertEqual(names, ("left_hip_pitch_joint", "right_index_pip"))
        self.assertEqual([call[0] for call in fake.calls], ["set_dof_position_targets"])
        _, args, kwargs = fake.calls[0]
        self.assertEqual(args, ([[-0.25, 0.5]],))
        self.assertEqual(kwargs["dof_indices"], [39, 4])

    def test_reset_restores_root_joints_and_all_velocities_deterministically(self):
        runtime_names = tuple(reversed(self.spec.canonical_dof_order))
        fake = FakeArticulation(runtime_names)
        controller = ArticulationController(self.spec, fake)
        controller.reset()
        self.assertEqual(
            [call[0] for call in fake.calls],
            [
                "set_world_poses",
                "set_velocities",
                "set_dof_positions",
                "set_dof_velocities",
                "set_dof_position_targets",
            ],
        )
        world = fake.calls[0][2]
        self.assertEqual(world["positions"], [[0.0, 0.0, 0.635]])
        self.assertEqual(world["orientations"], [[1.0, 0.0, 0.0, 0.0]])
        root_velocity = fake.calls[1][2]
        self.assertEqual(root_velocity["linear_velocities"], [[0.0, 0.0, 0.0]])
        self.assertEqual(root_velocity["angular_velocities"], [[0.0, 0.0, 0.0]])
        joint_positions = fake.calls[2][1][0][0]
        joint_velocities = fake.calls[3][1][0][0]
        drive_targets = fake.calls[4][1][0][0]
        self.assertEqual(len(joint_positions), 40)
        self.assertEqual(joint_positions, [0.0] * 40)
        self.assertEqual(joint_velocities, [0.0] * 40)
        self.assertEqual(drive_targets, joint_positions)


if __name__ == "__main__":
    unittest.main()
