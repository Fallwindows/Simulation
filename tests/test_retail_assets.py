import re
import math
import unittest
from collections import Counter
from dataclasses import replace
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import (
    SHELF_THICKNESS_M,
    build_aisle_layout,
    shelf_level_counts_by_zone,
)
from simulator.environment.retail_catalog import load_retail_catalog
from tools.retail_assets.generate_packaging import ASSET_SPECS, _asset_usda


def _authored_z_bounds(source, asset_key):
    """Return analytic local-space Z bounds for the asset's authored shapes."""
    bounds = []
    for match in re.finditer(
        r'def Cylinder "[^"]+"\s*\(.*?\)\s*\{([^}]+)\}', source, re.DOTALL
    ):
        body = match.group(1)
        height = float(re.search(r'double height = ([0-9.]+)', body).group(1))
        translate = re.search(r'xformOp:translate = \(0, 0, (-?[0-9.]+)\)', body)
        center_z = float(translate.group(1)) if translate else 0.0
        bounds.extend((center_z - height / 2.0, center_z + height / 2.0))

    for match in re.finditer(r'point3f\[\] points = \[([^]]+)\]', source, re.DOTALL):
        points = [
            tuple(float(value.strip()) for value in point.split(","))
            for point in re.findall(r'\(([^()]+)\)', match.group(1))
        ]
        bounds.extend(point[2] for point in points)

    for match in re.finditer(
        r'def Xform "[^"]+"\s*\{\s*double3 xformOp:translate = \(([^)]+)\)'
        r'\s*double3 xformOp:scale = \(([^)]+)\).*?def Cube',
        source,
        re.DOTALL,
    ):
        center_z = float(match.group(1).split(",")[-1])
        half_z = float(match.group(2).split(",")[-1])
        bounds.extend((center_z - half_z, center_z + half_z))

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
        self.assertEqual(len(self.catalog.assets), 34)
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
        self.assertTrue({"carton", "bottle", "can", "jar", "fruit"}.issubset(used_model_types))
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

        # The near-row first bay includes a visibly distinct, already
        # cataloged bottle family at every shelf level. Slots, two-depth
        # occupancy, and physical IDs remain the same as the box-only layout.
        original_slot_keys = {
            0: "cereal_honey",
            1: "snack_wafer",
            2: "snack_wafer",
            3: "cereal_harvest",
            4: "snack_popcorn",
        }
        for level, original_key in original_slot_keys.items():
            facings = [
                asset for asset in self.layout.assets
                if asset.name.startswith(f"product_r0_b0_l{level}_") and asset.name.endswith("_f0")
            ]
            self.assertEqual({asset.asset_key for asset in facings}, {"soda_orbit"})
            self.assertEqual({self.catalog_by_key[asset.asset_key].model_type for asset in facings}, {"bottle"})
            self.assertEqual(
                {asset.semantic_id for asset in facings},
                {
                    f"retail/{original_key}/r0/b0/l{level}/d0/f0",
                    f"retail/{original_key}/r0/b0/l{level}/d1/f0",
                },
            )

    def test_dimension_aware_shelf_levels_are_dense_but_clear(self):
        counts = shelf_level_counts_by_zone(self.config, self.catalog)
        self.assertEqual(counts, {"cereal": 5, "snacks": 5, "cans_jars": 7, "beverage": 6, "produce": 6})
        self.assertEqual(len(self.shelves), 229)
        for row in range(2):
            for bay in range(self.bay_count):
                levels = [key for key in self.shelves if key[:2] == (row, bay)]
                z_values = [self.shelves[key].center_m[2] for key in sorted(levels)]
                self.assertEqual(z_values, sorted(z_values))
                self.assertEqual(len(set(z_values)), len(z_values))

    def test_dense_count_and_real_depth_facings(self):
        self.assertGreaterEqual(len(self.layout.assets), 1800)
        self.assertLessEqual(len(self.layout.assets), 2300)
        self.assertGreater(len(self.layout.assets), 4 * 433)
        self.assertGreaterEqual(len({asset.asset_key for asset in self.layout.assets}), 30)
        regular = [asset for asset in self.layout.assets if asset.category != "produce_crate"]
        self.assertTrue(any("/d1/" in asset.semantic_id for asset in regular))
        self.assertEqual(Counter(asset.category for asset in self.layout.assets), Counter({
                "cereal": 398,
                "snacks": 279,
                "cans": 177,
                "juice": 374,
                "soda": 356,
                "jars": 115,
                "water": 196,
            "red_apples": 24,
            "green_apples": 24,
            "oranges": 24,
            "lemons": 24,
            "produce_crate": 4,
        }))

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
