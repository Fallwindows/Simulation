import re
import math
import hashlib
import json
import os
import shutil
import subprocess
import sys
import unittest
from collections import Counter
from dataclasses import replace
from pathlib import Path
from unittest import mock

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import (
    SHELF_THICKNESS_M,
    build_aisle_layout,
    shelf_level_counts_by_zone,
)
from simulator.environment.retail_catalog import load_retail_catalog
from tools.retail_assets.generate_packaging import (
    ASSET_SPECS,
    BOTTLE_LAYOUT_FIVE_TEXT_INSET_FRACTION,
    BOTTLE_LEFT_STRIP_FRACTION,
    ITEM_ART_DIRECTIONS,
    _asset_usda,
    generate_library,
)
import tools.retail_assets.generate_packaging as packaging_generator


def _authored_z_bounds(source, asset_key):
    """Return analytic local-space Z bounds for the asset's authored shapes."""
    bounds = []
    for match in re.finditer(
        r'def Cylinder "[^"]+"\s*\(.*?\)\s*\{([^}]+)\}', source, re.DOTALL
    ):
        body = match.group(1)
        height = float(re.search(r'double height = ([0-9.]+)', body).group(1))
        radius = float(re.search(r'double radius = ([0-9.]+)', body).group(1))
        axis_match = re.search(r'uniform token axis = "([XYZ])"', body)
        axis = axis_match.group(1) if axis_match else "Z"
        translate = re.search(r'xformOp:translate = \(([^)]+)\)', body)
        center_z = float(translate.group(1).split(",")[2]) if translate else 0.0
        half_z = height / 2.0 if axis == "Z" else radius
        bounds.extend((center_z - half_z, center_z + half_z))

    for match in re.finditer(r'point3f\[\] points = \[([^]]+)\]', source, re.DOTALL):
        points = [
            tuple(float(value.strip()) for value in point.split(","))
            for point in re.findall(r'\(([^()]+)\)', match.group(1))
        ]
        bounds.extend(point[2] for point in points)

    for match in re.finditer(
        r'def Xform "[^"]+"\s*\{\s*double3 xformOp:translate = \(([^)]+)\)'
        r'\s*double3 xformOp:scale = \(([^)]+)\)'
        r'\s*uniform token\[\] xformOpOrder = \[[^]]+\]\s*def Cube',
        source,
        re.DOTALL,
    ):
        center_z = float(match.group(1).split(",")[-1])
        half_z = float(match.group(2).split(",")[-1])
        bounds.extend((center_z - half_z, center_z + half_z))

    for match in re.finditer(
        r'def Xform "[^"]+"\s*\{\s*double3 xformOp:translate = \(([^)]+)\)'
        r'\s*double3 xformOp:scale = \(([^)]+)\)'
        r'\s*uniform token\[\] xformOpOrder = \[[^]]+\]\s*def Sphere',
        source,
        re.DOTALL,
    ):
        center_z = float(match.group(1).split(",")[-1])
        size_z = float(match.group(2).split(",")[-1])
        bounds.extend((center_z - size_z / 2.0, center_z + size_z / 2.0))

    if not bounds:
        raise AssertionError(f"No supported authored geometry found for {asset_key}")
    return min(bounds), max(bounds)


class RetailAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[1]
        cls.scenario = load_scenario(root / "config/scenarios/baseline_straight.yaml")
        cls.config = cls.scenario.environment
        cls.layout = build_aisle_layout(cls.config)
        cls.catalog = load_retail_catalog(cls.config.asset_manifest_path)
        cls.catalog_by_key = {asset.asset_key: asset for asset in cls.catalog.assets}
        cls.bay_count = int(cls.config.length_m // cls.config.bay_width_m)
        cls.shelves = {}
        for primitive in cls.layout.primitives:
            match = re.match(r"shelf_r(\d+)_b(\d+)_l(\d+)$", primitive.name)
            if match:
                row, bay, level = (int(value) for value in match.groups())
                cls.shelves[(row, bay, level)] = primitive

    def test_catalog_is_complete_and_portable(self):
        self.assertEqual(len(self.catalog.assets), 79)
        for asset in self.catalog.assets:
            self.assertTrue(asset.usd_path.is_file(), asset.asset_key)
            self.assertTrue(asset.texture_path.is_file(), asset.asset_key)
            self.assertEqual(asset.local_front_axis, "+Y")

    def test_front_panel_winding_matches_its_authored_normal(self):
        # Evaluate face direction and texture orientation independently from
        # authored normals so a consistently wrong winding/normal pair fails.
        for asset in self.catalog.assets:
            spec = next(spec for spec in ASSET_SPECS if spec.asset_key == asset.asset_key)
            sources = (
                asset.usd_path.read_text(encoding="utf-8"),
                _asset_usda(spec, f"{spec.asset_key}.png"),
            )
            for source_index, source in enumerate(sources):
                match = re.search(
                    r'def Mesh "FrontPanel".*?point3f\[\] points = \[([^\]]+)\]'
                    r'.*?int\[\] faceVertexIndices = \[([^\]]+)\]'
                    r'.*?normal3f\[\] normals = \[\(([^)]+)\)\]'
                    r'.*?texCoord2f\[\] primvars:st = \[([^\]]+)\]',
                    source,
                    re.DOTALL,
                )
                if match is None:
                    continue
                label = f"{asset.asset_key} source={source_index}"
                points = [tuple(float(v) for v in point) for point in re.findall(
                    r'(-?\d+\.\d+),\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)', match.group(1)
                )]
                indices = [int(value.strip()) for value in match.group(2).split(",")]
                normal = tuple(float(value.strip()) for value in match.group(3).split(","))
                uvs = [tuple(float(value.strip()) for value in uv.split(",")) for uv in re.findall(r'\(([^()]+)\)', match.group(4))]
                self.assertEqual(len(points), 4, label)
                self.assertEqual(len(indices), 4, label)
                self.assertEqual(len(uvs), 4, label)
                p0, p1, p2 = (points[indices[i]] for i in range(3))
                edge_a = tuple(p1[i] - p0[i] for i in range(3))
                edge_b = tuple(p2[i] - p0[i] for i in range(3))
                geometric_normal = (
                    edge_a[1] * edge_b[2] - edge_a[2] * edge_b[1],
                    edge_a[2] * edge_b[0] - edge_a[0] * edge_b[2],
                    edge_a[0] * edge_b[1] - edge_a[1] * edge_b[0],
                )
                self.assertGreater(geometric_normal[1], 0.0, label)
                self.assertEqual(normal, (0.0, 1.0, 0.0), label)
                # U increases with local X and V with local Z at every corner.
                for point, (u, v) in zip(points, uvs):
                    self.assertAlmostEqual(u, 1.0 if point[0] > 0.0 else 0.0, places=7, msg=label)
                    self.assertAlmostEqual(v, 1.0 if point[2] > 0.0 else 0.0, places=7, msg=label)

    def test_generated_usd_uses_supported_schema_properties_and_types(self):
        usd_by_key = {asset.asset_key: asset.usd_path for asset in self.catalog.assets}
        records_by_key = {asset.asset_key: asset for asset in self.catalog.assets}
        for spec in ASSET_SPECS:
            generated = _asset_usda(spec, f"{spec.asset_key}.png")
            on_disk = usd_by_key[spec.asset_key].read_text(encoding="utf-8")
            self._assert_schema_safe_asset_source(generated, spec.asset_key, spec.model_type, spec.dimensions_m)
            self._assert_schema_safe_asset_source(on_disk, spec.asset_key, spec.model_type, records_by_key[spec.asset_key].dimensions_m)

    def _assert_schema_safe_asset_source(self, source, asset_key, model_type, dimensions_m):
        self.assertNotRegex(source, r"\bradius2\s*=", asset_key)
        self.assertNotRegex(source, r"token\s+outputs:rgb\b", asset_key)
        self.assertNotRegex(source, r"normals:interpolation\b", asset_key)
        for prim_name in re.findall(r'\bdef\s+\w+\s+"([^"]+)"', source):
            self.assertRegex(prim_name, r"^[A-Za-z_][A-Za-z0-9_]*$", asset_key)
        if "outputs:rgb" in source:
            self.assertRegex(source, r"float3\s+outputs:rgb\b", asset_key)
        if 'def Mesh "FrontPanel"' in source:
            self.assertRegex(
                source,
                r'normal3f\[\]\s+normals\s*=\s*\[[^]]+\]\s*\(\s*interpolation\s*=\s*"uniform"',
                asset_key,
            )
        if model_type == "bottle":
            self.assertIn('def Mesh "Shoulder"', source, asset_key)
            self.assertNotIn('def Cone "Shoulder"', source, asset_key)
            points = re.search(r'def Mesh "Shoulder".*?point3f\[\] points = \[([^]]+)\]', source, re.DOTALL)
            self.assertIsNotNone(points, asset_key)
            point_values = [tuple(float(v.strip()) for v in point.split(",")) for point in re.findall(r'\(([^()]+)\)', points.group(1))]
            self.assertEqual(len(point_values), 80, asset_key)
            counts_match = re.search(r'def Mesh "Shoulder".*?int\[\] faceVertexCounts = \[([^]]+)\]', source, re.DOTALL)
            indices_match = re.search(r'def Mesh "Shoulder".*?int\[\] faceVertexIndices = \[([^]]+)\]', source, re.DOTALL)
            self.assertIsNotNone(counts_match, asset_key)
            self.assertIsNotNone(indices_match, asset_key)
            face_counts = [int(value.strip()) for value in counts_match.group(1).split(",")]
            face_indices = [int(value.strip()) for value in indices_match.group(1).split(",")]
            self.assertEqual(face_counts, [40, 40, *([4] * 40)], asset_key)
            self.assertEqual(sum(face_counts), len(face_indices), asset_key)
            self.assertTrue(all(0 <= value < len(point_values) for value in face_indices), asset_key)

            def face_normal(indices):
                a, b, c = (point_values[indices[index]] for index in range(3))
                u = tuple(b[axis] - a[axis] for axis in range(3))
                v = tuple(c[axis] - a[axis] for axis in range(3))
                return (
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                )

            bottom_normal = face_normal(face_indices[:40])
            top_normal = face_normal(face_indices[40:80])
            side_normal = face_normal(face_indices[80:84])
            radial_midpoint = tuple((point_values[0][axis] + point_values[1][axis]) / 2.0 for axis in range(2))
            self.assertLess(bottom_normal[2], 0.0, asset_key)
            self.assertGreater(top_normal[2], 0.0, asset_key)
            self.assertGreater(side_normal[0] * radial_midpoint[0] + side_normal[1] * radial_midpoint[1], 0.0, asset_key)
        if model_type == "fruit":
            stem = re.search(r'def Cylinder "Stem".*?double height = ([0-9.]+).*?xformOp:translate = \(0, 0, ([0-9.]+)\)', source, re.DOTALL)
            self.assertIsNotNone(stem, asset_key)
            self.assertLessEqual(float(stem.group(2)) + float(stem.group(1)) / 2.0, dimensions_m[2] / 2.0 + 1e-9, asset_key)

    def test_non_cereal_instances_use_distinct_shapes_and_sizes(self):
        placed_keys = {asset.asset_key for asset in self.layout.assets if asset.category != "cereal"}
        placed = [self.catalog_by_key[key] for key in placed_keys]
        used_model_types = {asset.model_type for asset in placed}
        self.assertTrue({"milk_jug", "pillow_bag", "short_can", "jam_jar", "banana_bunch"}.issubset(used_model_types))
        # These independent catalog bounds represent the geometry actually
        # referenced into this layout, rather than a count of category labels.
        heights = {asset.dimensions_m[2] for asset in placed if asset.model_type != "fruit"}
        horizontal_ratios = {
            round(asset.dimensions_m[2] / max(asset.dimensions_m[0], asset.dimensions_m[1]), 2)
            for asset in placed if asset.model_type != "fruit"
        }
        self.assertGreaterEqual(len(heights), 4)
        self.assertGreaterEqual(len(horizontal_ratios), 3)
        self.assertGreater(max(heights) - min(heights), 0.12)

        # Semantic SKU and referenced physical asset must agree.  This guards
        # the former cereal-ID/soda-geometry substitution.
        for asset in self.layout.assets:
            self.assertEqual(asset.semantic_id.split("/")[1], asset.asset_key)

    def test_dimension_aware_shelf_levels_are_dense_but_clear(self):
        counts = shelf_level_counts_by_zone(self.config, self.catalog)
        self.assertEqual(counts, {"hero": 5, "cereal": 5, "snacks": 6, "cans_jars": 5, "beverage": 5, "produce": 5})
        self.assertEqual(len(self.shelves), 205)
        for row in range(2):
            for bay in range(self.bay_count):
                levels = [key for key in self.shelves if key[:2] == (row, bay)]
                z_values = [self.shelves[key].center_m[2] for key in sorted(levels)]
                self.assertEqual(z_values, sorted(z_values))
                self.assertEqual(len(set(z_values)), len(z_values))

    def test_dense_count_and_real_depth_facings(self):
        self.assertGreaterEqual(len(self.layout.assets), 1900)
        self.assertLessEqual(len(self.layout.assets), 2300)
        self.assertGreaterEqual(len({asset.asset_key for asset in self.layout.assets}), 70)
        regular = [asset for asset in self.layout.assets if asset.category != "produce_crate"]
        self.assertTrue(any("/d1/" in asset.semantic_id for asset in regular))
        counts = Counter(asset.category for asset in self.layout.assets)
        for category in (
            "cereal", "pantry_box", "snack_bag", "bagged_goods", "bakery",
            "milk", "refrigerated", "frozen", "beverage", "juice", "cans",
            "jars", "condiments", "cleaning", "household", "fresh_produce",
            "produce_crate", "produce_fixture", "shelf_fixture",
        ):
            self.assertGreater(counts[category], 0, category)

    def test_new_physical_type_register_is_auditable_and_instantiated(self):
        root = Path(__file__).resolve().parents[1]
        register = json.loads((root / "assets/retail/asset_register.json").read_text(encoding="utf-8"))
        audit = register["audit"]
        self.assertEqual(audit["baseline_type_count"], 34)
        self.assertEqual(audit["new_type_count"], 45)
        self.assertEqual(audit["new_unique_geometry_signature_count"], 45)
        self.assertGreaterEqual(len(audit["new_assembly_profiles"]), 45)
        self.assertTrue({"pantry", "refrigerated", "beverage", "produce", "household", "fixtures"}.issubset(audit["new_departments"]))
        self.assertEqual(register["provenance"]["provider"], "project-authored deterministic procedural geometry")
        self.assertIn("license", register["provenance"])

        new_assets = [asset for asset in self.catalog.assets if asset.introduced_in == "G02-A"]
        self.assertEqual(len(new_assets), 45)
        self.assertEqual(len({asset.geometry_signature for asset in new_assets}), 45)
        self.assertEqual(len({asset.geometry_scope_sha256 for asset in new_assets}), 45)
        self.assertTrue(all(len(asset.assembly_parts) >= 3 for asset in new_assets))
        self.assertTrue(all(len(asset.material_classes) >= 2 for asset in new_assets))
        placed_new_keys = {
            asset.asset_key for asset in self.layout.assets
            if self.catalog_by_key[asset.asset_key].introduced_in == "G02-A"
        }
        self.assertEqual(placed_new_keys, {asset.asset_key for asset in new_assets})

    def test_checked_in_library_matches_deterministic_regeneration(self):
        try:
            packaging_generator._validate_texture_toolchain()
        except RuntimeError as error:
            self.skipTest(f"documented generation runtime required: {error}")
        root = Path(__file__).resolve().parents[1]
        scratch = root / "review" / f".test-retail-{os.getpid()}-regeneration"
        shutil.rmtree(scratch, ignore_errors=True)
        scratch.mkdir(parents=True)
        try:
            generated_manifest = generate_library(scratch)
            generated_root = generated_manifest.parent
            committed_root = root / "assets/retail"
            self.assertEqual(generated_manifest.read_bytes(), (committed_root / "manifest.json").read_bytes())
            self.assertEqual((generated_root / "asset_register.json").read_bytes(), (committed_root / "asset_register.json").read_bytes())
            for asset in self.catalog.assets:
                self.assertEqual(
                    hashlib.sha256((generated_root / "usd" / f"{asset.asset_key}.usda").read_bytes()).hexdigest(),
                    hashlib.sha256(asset.usd_path.read_bytes()).hexdigest(),
                )
                self.assertEqual(
                    hashlib.sha256((generated_root / "textures" / f"{asset.asset_key}.png").read_bytes()).hexdigest(),
                    hashlib.sha256(asset.texture_path.read_bytes()).hexdigest(),
                )
                for surface_path in (asset.normal_texture_path, asset.roughness_texture_path):
                    if surface_path is not None:
                        self.assertEqual(
                            hashlib.sha256((generated_root / "textures" / surface_path.name).read_bytes()).hexdigest(),
                            hashlib.sha256(surface_path.read_bytes()).hexdigest(),
                        )
        finally:
            shutil.rmtree(scratch, ignore_errors=True)

    def test_generation_is_cross_process_and_hash_seed_deterministic(self):
        try:
            packaging_generator._validate_texture_toolchain()
        except RuntimeError as error:
            self.skipTest(f"documented generation runtime required: {error}")
        root = Path(__file__).resolve().parents[1]
        scratch_roots = [
            root / "review" / f".test-retail-{os.getpid()}-process-{seed}"
            for seed in ("1", "8675309")
        ]
        for scratch in scratch_roots:
            shutil.rmtree(scratch, ignore_errors=True)
        try:
            for seed, scratch in zip(("1", "8675309"), scratch_roots):
                environment = dict(os.environ)
                environment["PYTHONHASHSEED"] = seed
                subprocess.check_call(
                    [
                        sys.executable,
                        "-c",
                        "from pathlib import Path; "
                        "from tools.retail_assets.generate_packaging import generate_library; "
                        f"generate_library(Path({str(scratch)!r}))",
                    ],
                    cwd=root,
                    env=environment,
                )
            generated_roots = [scratch / "assets/retail" for scratch in scratch_roots]
            first_files = sorted(
                path.relative_to(generated_roots[0])
                for path in generated_roots[0].rglob("*")
                if path.is_file()
            )
            second_files = sorted(
                path.relative_to(generated_roots[1])
                for path in generated_roots[1].rglob("*")
                if path.is_file()
            )
            self.assertEqual(first_files, second_files)
            for relative_path in first_files:
                self.assertEqual(
                    hashlib.sha256((generated_roots[0] / relative_path).read_bytes()).hexdigest(),
                    hashlib.sha256((generated_roots[1] / relative_path).read_bytes()).hexdigest(),
                    relative_path.as_posix(),
                )
        finally:
            for scratch in scratch_roots:
                shutil.rmtree(scratch, ignore_errors=True)

    def test_manifest_order_and_python_hash_seed_do_not_change_layout(self):
        root = Path(__file__).resolve().parents[1]
        source_manifest = json.loads((root / "assets/retail/manifest.json").read_text(encoding="utf-8"))
        scratch = root / "review" / f".test-retail-{os.getpid()}-ordering"
        shutil.rmtree(scratch, ignore_errors=True)
        scratch.mkdir(parents=True)
        try:
            reordered = dict(source_manifest)
            reordered_entries = []
            for entry in reversed(source_manifest["assets"]):
                copied = dict(entry)
                copied["usd_path"] = str((root / "assets/retail" / entry["usd_path"]).resolve())
                copied["texture_path"] = str((root / "assets/retail" / entry["texture_path"]).resolve())
                for field in ("normal_texture_path", "roughness_texture_path"):
                    if field in entry:
                        copied[field] = str((root / "assets/retail" / entry[field]).resolve())
                reordered_entries.append(copied)
            reordered["assets"] = reordered_entries
            reordered_path = scratch / "manifest.json"
            reordered_path.write_text(json.dumps(reordered), encoding="utf-8")
            alternate = build_aisle_layout(replace(self.config, asset_manifest_path=str(reordered_path)))
            self.assertEqual(alternate.primitives, self.layout.primitives)
            self.assertEqual(alternate.assets, self.layout.assets)
        finally:
            shutil.rmtree(scratch, ignore_errors=True)

        scenario_path = root / "config/scenarios/baseline_straight.yaml"
        script = (
            "import hashlib,json; from pathlib import Path; "
            "from simulator.config.loader import load_scenario; "
            "from simulator.environment.aisle_builder import build_aisle_layout; "
            f"layout=build_aisle_layout(load_scenario(Path({str(scenario_path)!r})).environment); "
            "payload=[a.__dict__ for a in layout.assets]; "
            "print(hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest())"
        )
        digests = []
        for seed in ("1", "8675309"):
            environment = dict(os.environ)
            environment["PYTHONHASHSEED"] = seed
            digests.append(subprocess.check_output(
                [sys.executable, "-c", script], cwd=root, env=environment, text=True
            ).strip())
        self.assertEqual(digests[0], digests[1])

    def test_hero_bays_exercise_new_silhouettes_and_group_skus(self):
        hero = [
            asset for asset in self.layout.assets
            if asset.name.startswith(("product_r0_b0_", "product_r0_b1_"))
        ]
        hero_records = [self.catalog_by_key[asset.asset_key] for asset in hero]
        self.assertGreaterEqual(len({record.asset_key for record in hero_records}), 20)
        self.assertGreaterEqual(len({record.assembly_profile for record in hero_records}), 12)
        self.assertGreaterEqual(sum(record.introduced_in == "G02-A" for record in {r.asset_key: r for r in hero_records}.values()), 12)
        self.assertTrue({"milk_jug", "pillow_bag", "window_box", "grip_bottle"}.issubset(
            {record.assembly_profile for record in hero_records}
        ))
        by_slot = {}
        for asset in hero:
            match = re.match(r"product_r0_b(\d+)_l(\d+)_d(\d+)_f(\d+)$", asset.name)
            self.assertIsNotNone(match)
            bay, level, depth, facing = (int(value) for value in match.groups())
            by_slot[(bay, level, depth, facing)] = asset.asset_key
        grouped_pairs = 0
        for bay, level, depth, facing in list(by_slot):
            if facing % 2 == 0 and (bay, level, depth, facing + 1) in by_slot:
                self.assertEqual(by_slot[(bay, level, depth, facing)], by_slot[(bay, level, depth, facing + 1)])
                grouped_pairs += 1
        self.assertGreaterEqual(grouped_pairs, 12)

    def test_rotated_bounds_have_support_clearance_and_no_facing_overlap(self):
        negative_aisle_edge = -math.inf
        positive_aisle_edge = math.inf
        facing_groups = {}
        depth_groups = {}
        for asset in self.layout.assets:
            record = self.catalog_by_key[asset.asset_key]
            yaw = math.radians(asset.rotation_rpy_deg[2])
            width = record.dimensions_m[0] * asset.scale_xyz[0]
            depth = record.dimensions_m[1] * asset.scale_xyz[1]
            half_x = abs(math.cos(yaw)) * width / 2.0 + abs(math.sin(yaw)) * depth / 2.0
            half_y = abs(math.sin(yaw)) * width / 2.0 + abs(math.cos(yaw)) * depth / 2.0
            if asset.position_m[1] < 0.0:
                negative_aisle_edge = max(negative_aisle_edge, asset.position_m[1] + half_y)
            else:
                positive_aisle_edge = min(positive_aisle_edge, asset.position_m[1] - half_y)

            match = re.match(r"product_r(\d+)_b(\d+)_l(\d+)_d(\d+)_f(\d+)$", asset.name)
            if match:
                row, bay, level, depth_index, facing = (int(value) for value in match.groups())
                facing_groups.setdefault((row, bay, level, depth_index), []).append((asset.position_m[0], half_x, asset.semantic_id))
                depth_groups.setdefault((row, bay, level, facing), []).append((asset.position_m[1], half_y, asset.semantic_id))

                shelf = self.shelves[(row, bay, level)]
                local_bottom, local_top = _authored_z_bounds(record.usd_path.read_text(encoding="utf-8"), asset.asset_key)
                actual_bottom = asset.position_m[2] + local_bottom * asset.scale_xyz[2]
                actual_top = asset.position_m[2] + local_top * asset.scale_xyz[2]
                shelf_top = shelf.center_m[2] + SHELF_THICKNESS_M / 2.0
                self.assertLessEqual(abs(actual_bottom - shelf_top), 0.001, asset.semantic_id)
                next_shelf = self.shelves.get((row, bay, level + 1))
                if next_shelf is not None:
                    clearance = next_shelf.center_m[2] - SHELF_THICKNESS_M / 2.0 - actual_top
                    self.assertGreaterEqual(clearance, self.config.shelf_clearance_m - 1e-5, asset.semantic_id)

        self.assertGreaterEqual(positive_aisle_edge - negative_aisle_edge, 1.375)
        for objects in facing_groups.values():
            objects.sort()
            for left, right in zip(objects, objects[1:]):
                self.assertGreaterEqual(right[0] - right[1] - (left[0] + left[1]), -0.001, (left[2], right[2]))
        for objects in depth_groups.values():
            objects.sort()
            for first, second in zip(objects, objects[1:]):
                self.assertGreaterEqual(second[0] - second[1] - (first[0] + first[1]), -0.001, (first[2], second[2]))

    def test_regular_products_do_not_intersect_produce_displays(self):
        def bounds(asset):
            record = self.catalog_by_key[asset.asset_key]
            yaw = math.radians(asset.rotation_rpy_deg[2])
            width = record.dimensions_m[0] * asset.scale_xyz[0]
            depth = record.dimensions_m[1] * asset.scale_xyz[1]
            half_x = abs(math.cos(yaw)) * width / 2.0 + abs(math.sin(yaw)) * depth / 2.0
            half_y = abs(math.sin(yaw)) * width / 2.0 + abs(math.cos(yaw)) * depth / 2.0
            local_bottom, local_top = _authored_z_bounds(record.usd_path.read_text(encoding="utf-8"), asset.asset_key)
            return (
                asset.position_m[0] - half_x, asset.position_m[0] + half_x,
                asset.position_m[1] - half_y, asset.position_m[1] + half_y,
                asset.position_m[2] + local_bottom * asset.scale_xyz[2],
                asset.position_m[2] + local_top * asset.scale_xyz[2],
            )

        products = [asset for asset in self.layout.assets if asset.name.startswith("product_")]
        produce = [
            asset for asset in self.layout.assets
            if asset.name.startswith("produce_bin_") or asset.name.startswith("fruit_")
        ]
        intersections = []
        for product in products:
            product_bounds = bounds(product)
            for display in produce:
                display_bounds = bounds(display)
                overlaps = all(
                    min(product_bounds[axis + 1], display_bounds[axis + 1])
                    - max(product_bounds[axis], display_bounds[axis]) > 0.001
                    for axis in (0, 2, 4)
                )
                if overlaps:
                    intersections.append((product.semantic_id, display.semantic_id))
        self.assertEqual(intersections, [])

    def test_texture_generation_fails_closed_without_pinned_prerequisites(self):
        root = Path(__file__).resolve().parents[1]
        destination = root / "review" / ".must-not-be-created"
        with mock.patch.object(packaging_generator, "PIL", None), \
                mock.patch.object(packaging_generator, "Image", None), \
                self.assertRaisesRegex(RuntimeError, "(?s)Pillow==12.3.0.*No assets were generated"):
            generate_library(destination)
        self.assertFalse(destination.exists())

    def test_new_texture_bindings_are_uv_authored_and_art_is_varied(self):
        try:
            from PIL import Image as PillowImage
        except ImportError as error:
            self.skipTest(f"Pillow image inspection unavailable: {error}")
        new_assets = [asset for asset in self.catalog.assets if asset.introduced_in == "G02-A"]
        sizes = set()
        texture_hashes = set()
        assets_without_surface_maps = set()

        for asset in new_assets:
            source = asset.usd_path.read_text(encoding="utf-8")
            geometry = source.split('def Scope "Looks"', 1)[0]
            front_bindings = geometry.count("</Asset/Looks/Front>")
            print_bindings = geometry.count("</Asset/Looks/Print>")
            uv_sets = geometry.count("primvars:st")
            self.assertEqual(front_bindings + print_bindings, uv_sets, asset.asset_key)
            normal_blocks = re.findall(r'def Shader "NormalTexture" \{(.*?)\n\s+\}', source, re.DOTALL)
            roughness_blocks = re.findall(r'def Shader "RoughnessTexture" \{(.*?)\n\s+\}', source, re.DOTALL)
            self.assertEqual(len(normal_blocks), source.count("inputs:normal.connect"), asset.asset_key)
            self.assertEqual(len(roughness_blocks), source.count("inputs:roughness.connect"), asset.asset_key)
            for block in normal_blocks:
                self.assertIn('token inputs:sourceColorSpace = "raw"', block, asset.asset_key)
                self.assertIn("float4 inputs:scale = (2, 2, 2, 1)", block, asset.asset_key)
                self.assertIn("float4 inputs:bias = (-1, -1, -1, 0)", block, asset.asset_key)
            for block in roughness_blocks:
                self.assertIn('token inputs:sourceColorSpace = "raw"', block, asset.asset_key)
            if normal_blocks:
                self.assertGreater(front_bindings, 0, asset.asset_key)
                self.assertIsNotNone(asset.normal_texture_path, asset.asset_key)
                self.assertIsNotNone(asset.roughness_texture_path, asset.asset_key)
                self.assertIn("inputs:normal.connect", source)
                self.assertIn("inputs:roughness.connect", source)
                self.assertIn("normal_map", asset.material_classes)
                self.assertIn("roughness_map", asset.material_classes)
            else:
                assets_without_surface_maps.add(asset.asset_key)
                self.assertIsNone(asset.normal_texture_path, asset.asset_key)
                self.assertIsNone(asset.roughness_texture_path, asset.asset_key)
                self.assertNotIn("normal_map", asset.material_classes)
                self.assertNotIn("roughness_map", asset.material_classes)
                self.assertFalse((asset.texture_path.parent / f"{asset.asset_key}_normal.png").exists())
                self.assertFalse((asset.texture_path.parent / f"{asset.asset_key}_roughness.png").exists())
            if asset.asset_key == "price_display":
                self.assertGreater(front_bindings, 0)
            with PillowImage.open(asset.texture_path) as image:
                sizes.add(image.size)
            texture_hashes.add(hashlib.sha256(asset.texture_path.read_bytes()).hexdigest())

        self.assertEqual(len(texture_hashes), 45)
        self.assertGreaterEqual(len(sizes), 8)
        self.assertEqual(len(ITEM_ART_DIRECTIONS), 45)
        self.assertEqual(len({direction[0] for direction in ITEM_ART_DIRECTIONS.values()}), 45)
        self.assertEqual({direction[2] for direction in ITEM_ART_DIRECTIONS.values()}, set(range(6)))
        self.assertEqual(ITEM_ART_DIRECTIONS["maple_syrup"][2], 5)
        self.assertGreater(BOTTLE_LAYOUT_FIVE_TEXT_INSET_FRACTION, BOTTLE_LEFT_STRIP_FRACTION)
        self.assertEqual(assets_without_surface_maps, {
            "banana_bunch", "pear", "broccoli", "carrot_bunch",
            "angled_produce_bin", "wicker_basket", "shelf_divider", "bottle_rack",
        })

    def test_register_parts_match_upgraded_organic_and_fixture_geometry(self):
        expectations = {
            "banana_bunch": (r'def (?:Xform|Mesh) "Finger[^\"]*"', 5),
            "broccoli": (r'def (?:Xform|Mesh) "Branch[^\"]*"', 5),
            "wicker_basket": (r'def (?:Xform|Mesh) "FrontWeave[^\"]*"', 8),
            "angled_produce_bin": (r'def (?:Xform|Mesh) "[^\"]*Rail"', 2),
        }
        for asset_key, (pattern, count) in expectations.items():
            record = self.catalog_by_key[asset_key]
            source = record.usd_path.read_text(encoding="utf-8")
            self.assertEqual(len(re.findall(pattern, source)), count, asset_key)
        self.assertLessEqual(self.catalog_by_key["banana_bunch"].usd_path.read_text(encoding="utf-8").count('def Sphere'), 1)
        self.assertNotIn('def Sphere', self.catalog_by_key["broccoli"].usd_path.read_text(encoding="utf-8"))
        self.assertNotIn('def Sphere', self.catalog_by_key["bread_loaf"].usd_path.read_text(encoding="utf-8"))
        wicker = self.catalog_by_key["wicker_basket"].usd_path.read_text(encoding="utf-8")
        self.assertEqual(len(re.findall(r'def Mesh "(?:Front|Back)Weave', wicker)), 16)

    def test_every_physical_object_has_one_unique_identity(self):
        ids = [asset.instance_id for asset in self.layout.assets]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertTrue(all(ids))
        self.assertEqual([asset.instance_id for asset in self.layout.assets], [asset.semantic_id for asset in self.layout.assets])
        self.assertEqual(self.layout, build_aisle_layout(self.config))

    def test_all_assets_resolve_and_products_are_shelf_anchored(self):
        for asset in self.layout.assets:
            self.assertIn(asset.asset_key, self.catalog_by_key)
            if asset.category in {"red_apples", "green_apples", "oranges", "lemons"}:
                record = self.catalog_by_key[asset.asset_key]
                effective_size = tuple(record.dimensions_m[axis] * asset.scale_xyz[axis] for axis in range(3))
                self.assertTrue(all(0.092 <= value <= 0.094 for value in effective_size))
                self.assertLess(max(effective_size) - min(effective_size), 1e-9)
            else:
                self.assertEqual(asset.scale_xyz, (1.0, 1.0, 1.0))
            self.assertGreater(abs(asset.position_m[1]), self.config.shelf_depth_m / 2.0)
        for asset in self.layout.assets:
            if asset.category == "produce_crate":
                continue
            product_match = re.match(r"product_r(\d+)_b(\d+)_l(\d+)_d\d+_f\d+$", asset.name)
            if product_match is None:
                continue
            row, bay, level = (int(value) for value in product_match.groups())
            shelf = self.shelves[(row, bay, level)]
            record = self.catalog_by_key[asset.asset_key]
            local_bottom, local_top = _authored_z_bounds(
                record.usd_path.read_text(encoding="utf-8"), asset.asset_key
            )
            declared_half_height = record.dimensions_m[2] / 2.0
            # USDA transform values are authored at 5 decimal places; allow
            # that small serialization tolerance at declared bounds.
            self.assertGreaterEqual(local_bottom, -declared_half_height - 1e-5, asset.asset_key)
            self.assertLessEqual(local_top, declared_half_height + 1e-5, asset.asset_key)
            actual_bottom = asset.position_m[2] + local_bottom * asset.scale_xyz[2]
            actual_top = asset.position_m[2] + local_top * asset.scale_xyz[2]
            shelf_top = shelf.center_m[2] + SHELF_THICKNESS_M / 2.0
            self.assertLessEqual(abs(actual_bottom - shelf_top), 0.001, asset.semantic_id)
            self.assertLessEqual(abs(asset.position_m[1] - shelf.center_m[1]) + record.dimensions_m[1] / 2.0, self.config.shelf_depth_m / 2.0 + 1e-9)
            next_shelf = self.shelves.get((row, bay, level + 1))
            if next_shelf is not None:
                next_shelf_bottom = next_shelf.center_m[2] - SHELF_THICKNESS_M / 2.0
                self.assertGreaterEqual(next_shelf_bottom - actual_top, self.config.shelf_clearance_m - 1e-9, asset.semantic_id)

    def test_produce_is_substantial_and_inside_four_bins(self):
        crates = [asset for asset in self.layout.assets if asset.category == "produce_crate"]
        fruits = [asset for asset in self.layout.assets if asset.category in {"red_apples", "green_apples", "oranges", "lemons"}]
        self.assertEqual(len(crates), 4)
        self.assertEqual(len(fruits), 96)
        self.assertEqual(Counter(asset.category for asset in fruits), Counter({
            "red_apples": 24,
            "green_apples": 24,
            "oranges": 24,
            "lemons": 24,
        }))
        crate_by_prefix = {"/".join(asset.semantic_id.split("/")[2:]): asset for asset in crates}
        parts_by_asset_key = {}
        for asset_key in ("produce_crate_green", "produce_crate_red"):
            source = self.catalog_by_key[asset_key].usd_path.read_text(encoding="utf-8")
            self.assertNotIn('def Xform "Fruit"', source)
            self.assertIn('def Xform "CornerPost00"', source)
            self.assertIn('def Xform "LongRail00"', source)
            base = re.search(r'def Xform "Base" \{.*?xformOp:scale = \(([^)]+)\)', source, re.DOTALL)
            self.assertIsNotNone(base)
            base_height = 2.0 * float(base.group(1).split(",")[-1])
            self.assertLess(base_height, 0.04)
            parts = []
            for name, translate, scale in re.findall(
                r'def Xform "([^"]+)" \{\s*double3 xformOp:translate = \(([^)]+)\)'
                r'\s*double3 xformOp:scale = \(([^)]+)\)',
                source,
            ):
                center = tuple(float(value.strip()) for value in translate.split(","))
                half = tuple(float(value.strip()) for value in scale.split(","))
                parts.append((name, center, half))
            self.assertEqual(len(parts), 13, asset_key)
            for _, center, half in parts:
                self.assertLessEqual(abs(center[0]) + half[0], 0.22 + 1e-9)
                self.assertLessEqual(abs(center[1]) + half[1], 0.16 + 1e-9)
                self.assertLessEqual(abs(center[2]) + half[2], 0.11 + 1e-9)
            top_parts = [name for name, center, half in parts if center[2] + half[2] > 0.102]
            self.assertEqual(len(top_parts), 8, asset_key)
            self.assertEqual(sum(name.startswith("CornerPost") for name in top_parts), 4)
            self.assertEqual(sum(name.startswith(("LongRail1", "EndRail1")) for name in top_parts), 4)
            parts_by_asset_key[asset_key] = parts

        checked_fruits = set()
        component_checks = 0
        for fruit in fruits:
            prefix = "/".join(fruit.semantic_id.split("/")[2:-1])
            crate = crate_by_prefix[prefix]
            fruit_record = self.catalog_by_key[fruit.asset_key]
            crate_record = self.catalog_by_key[crate.asset_key]
            fruit_half = tuple(fruit_record.dimensions_m[axis] * fruit.scale_xyz[axis] / 2.0 for axis in range(3))
            effective_size = tuple(2.0 * value for value in fruit_half)
            self.assertTrue(all(0.092 <= value <= 0.094 for value in effective_size))
            self.assertLess(max(effective_size) - min(effective_size), 1e-9)
            self.assertLessEqual(abs(fruit.position_m[0] - crate.position_m[0]) + fruit_half[0], crate_record.dimensions_m[0] / 2.0 + 1e-9)
            self.assertLessEqual(abs(fruit.position_m[1] - crate.position_m[1]) + fruit_half[1], crate_record.dimensions_m[1] / 2.0 + 1e-9)
            crate_bottom = crate.position_m[2] - crate_record.dimensions_m[2] / 2.0
            crate_top = crate.position_m[2] + crate_record.dimensions_m[2] / 2.0
            self.assertLessEqual(fruit.position_m[2] + fruit_half[2], crate_top + 1e-9)
            self.assertGreaterEqual(fruit.position_m[2] - fruit_half[2], crate_bottom + 0.025 - 1e-9)

        for crate in crates:
            prefix = "/".join(crate.semantic_id.split("/")[2:])
            group = [fruit for fruit in fruits if "/".join(fruit.semantic_id.split("/")[2:-1]) == prefix]
            lower = [fruit for fruit in group if int(fruit.semantic_id.rsplit("fruit", 1)[1]) < 12]
            upper = [fruit for fruit in group if int(fruit.semantic_id.rsplit("fruit", 1)[1]) >= 12]
            self.assertEqual((len(lower), len(upper)), (12, 12))
            crate_record = self.catalog_by_key[crate.asset_key]
            base_top = crate.position_m[2] - crate_record.dimensions_m[2] / 2.0 + 0.025
            for fruit in lower:
                record = self.catalog_by_key[fruit.asset_key]
                bottom = fruit.position_m[2] - record.dimensions_m[2] * fruit.scale_xyz[2] / 2.0
                self.assertAlmostEqual(bottom, base_top, places=9)
            for fruit in upper:
                record = self.catalog_by_key[fruit.asset_key]
                radius = sum(record.dimensions_m[axis] * fruit.scale_xyz[axis] / 2.0 for axis in range(3)) / 3.0
                neighbor_distances = []
                for support in lower:
                    support_record = self.catalog_by_key[support.asset_key]
                    support_radius = sum(support_record.dimensions_m[axis] * support.scale_xyz[axis] / 2.0 for axis in range(3)) / 3.0
                    delta = tuple(fruit.position_m[axis] - support.position_m[axis] for axis in range(3))
                    neighbor_distances.append((sum(value * value for value in delta) ** 0.5, radius + support_radius))
                distance, combined_radius = min(neighbor_distances, key=lambda candidate: candidate[0])
                self.assertLessEqual(distance, combined_radius + 0.005)

            # Verify all 276 fruit pairs in each bin using the scaled USD
            # sphere radius; allow at most 1 mm for contact/numeric precision.
            for index, first in enumerate(group):
                first_record = self.catalog_by_key[first.asset_key]
                first_radius = max(first_record.dimensions_m[axis] * first.scale_xyz[axis] for axis in range(3)) / 2.0
                for second in group[index + 1:]:
                    second_record = self.catalog_by_key[second.asset_key]
                    second_radius = max(second_record.dimensions_m[axis] * second.scale_xyz[axis] for axis in range(3)) / 2.0
                    delta = tuple(first.position_m[axis] - second.position_m[axis] for axis in range(3))
                    distance = sum(value * value for value in delta) ** 0.5
                    self.assertGreaterEqual(distance, first_radius + second_radius - 0.001)

            # Transform fruit into each crate's local frame, then test its
            # sphere against the actual authored base, posts, and rails. This
            # catches rail interpenetration that catalog bounds cannot detect.
            parts = parts_by_asset_key[crate.asset_key]
            crate_yaw = math.radians(crate.rotation_rpy_deg[2])
            crate_cos, crate_sin = math.cos(crate_yaw), math.sin(crate_yaw)
            self.assertEqual(len(group), 24, crate.semantic_id)
            for fruit in group:
                checked_fruits.add(fruit.semantic_id)
                record = self.catalog_by_key[fruit.asset_key]
                radius = max(record.dimensions_m[axis] * fruit.scale_xyz[axis] for axis in range(3)) / 2.0
                dx = fruit.position_m[0] - crate.position_m[0]
                dy = fruit.position_m[1] - crate.position_m[1]
                local_center = (
                    crate_cos * dx + crate_sin * dy,
                    -crate_sin * dx + crate_cos * dy,
                    fruit.position_m[2] - crate.position_m[2],
                )
                for part_name, center, half in parts:
                    component_checks += 1
                    closest = tuple(
                        min(max(local_center[axis], center[axis] - half[axis]), center[axis] + half[axis])
                        for axis in range(3)
                    )
                    distance = math.sqrt(sum((local_center[axis] - closest[axis]) ** 2 for axis in range(3)))
                    self.assertGreaterEqual(
                        distance, radius - 0.001,
                        f"{crate.asset_key} {part_name} intersects {fruit.semantic_id} by {radius - distance:.6f} m",
                    )
        self.assertEqual(len(checked_fruits), 96)
        self.assertEqual(component_checks, 1248)

    def test_products_face_the_aisle_without_corridor_intrusion(self):
        for asset in self.layout.assets:
            if asset.category == "produce_crate":
                continue
            yaw = asset.rotation_rpy_deg[2] % 360.0
            if asset.position_m[1] < 0.0:
                self.assertLess(min(yaw, 360.0 - yaw), 5.0)
            else:
                self.assertLess(abs(yaw - 180.0), 5.0)
            self.assertGreater(abs(asset.position_m[1]), 1.0)


if __name__ == "__main__":
    unittest.main()
