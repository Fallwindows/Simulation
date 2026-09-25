import struct
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from simulator.sensors.feature_selection import select_bbox_front_surface
from simulator.sensors.scan_projection import (
    CameraIntrinsics,
    ProjectedScan,
    decode_pointcloud2_cdr,
    project_lidar_scan,
    read_pointcloud2_sqlite,
    resolve_transform,
)


class _CdrWriter:
    def __init__(self) -> None:
        self.data = bytearray(b"\x00\x01\x00\x00")

    def align(self, alignment: int) -> None:
        self.data.extend(b"\x00" * ((-len(self.data)) % alignment))

    def pack(self, code: str, alignment: int, value) -> None:
        self.align(alignment)
        self.data.extend(struct.pack("<" + code, value))

    def u8(self, value: int) -> None:
        self.pack("B", 1, value)

    def i32(self, value: int) -> None:
        self.pack("i", 4, value)

    def u32(self, value: int) -> None:
        self.pack("I", 4, value)

    def string(self, value: str) -> None:
        encoded = value.encode("utf-8") + b"\x00"
        self.u32(len(encoded))
        self.data.extend(encoded)

    def octets(self, value: bytes) -> None:
        self.u32(len(value))
        self.data.extend(value)


def _pointcloud2_cdr(points: list[tuple[float, float, float]], stamp_ns: int = 200_000_000) -> bytes:
    writer = _CdrWriter()
    writer.i32(stamp_ns // 1_000_000_000)
    writer.u32(stamp_ns % 1_000_000_000)
    writer.string("lidar_link")
    writer.u32(1)
    writer.u32(len(points))
    writer.u32(3)
    for name, offset in (("x", 0), ("y", 4), ("z", 8)):
        writer.string(name)
        writer.u32(offset)
        writer.u8(7)
        writer.u32(1)
    writer.u8(0)
    writer.u32(12)
    writer.u32(12 * len(points))
    writer.octets(b"".join(struct.pack("<fff", *point) for point in points))
    writer.u8(1)
    return bytes(writer.data)


class ScanProjectionTests(unittest.TestCase):
    def test_cdr_decoder_preserves_raw_point_order_and_header(self):
        cloud = decode_pointcloud2_cdr(
            _pointcloud2_cdr([(1.0, 2.0, 3.0), (float("nan"), 4.0, 5.0), (6.0, 7.0, 8.0)])
        )
        self.assertEqual(cloud.stamp_ns, 200_000_000)
        self.assertEqual(cloud.frame_id, "lidar_link")
        self.assertEqual(cloud.width, 3)
        np.testing.assert_array_equal(cloud.raw_point_indices, [0, 1, 2])
        np.testing.assert_allclose(cloud.xyz_m[[0, 2]], [[1, 2, 3], [6, 7, 8]])
        self.assertTrue(np.isnan(cloud.xyz_m[1, 0]))

    def test_sqlite_reader_requires_one_exact_record_and_matching_header_stamp(self):
        class Cursor:
            def __init__(self, rows):
                self.rows = rows

            def fetchall(self):
                return self.rows

        class Connection:
            def __enter__(self):
                return self

            def __exit__(self, *_):
                return False

            def execute(self, query, parameters=()):
                if query.startswith("PRAGMA"):
                    return Cursor([])
                if "FROM topics" in query:
                    return Cursor([(3, "sensor_msgs/msg/PointCloud2", "cdr")])
                self.last_message_parameters = parameters
                return Cursor([(24, 200_000_000, _pointcloud2_cdr([(1.0, 2.0, 3.0)]))])

        connection = Connection()
        with patch("simulator.sensors.scan_projection.sqlite3.connect", return_value=connection) as connect:
            record = read_pointcloud2_sqlite(
                Path(__file__), topic="/points", timestamp_ns=200_000_000, expected_message_id=24
            )
        self.assertEqual(record.message_id, 24)
        self.assertEqual(record.cloud.raw_point_indices.tolist(), [0])
        self.assertEqual(connection.last_message_parameters, (3, 200_000_000))
        self.assertIn("mode=ro&immutable=1", connect.call_args.args[0])

        mismatched = Connection()
        mismatched.execute = lambda query, parameters=(): (
            Cursor([])
            if query.startswith("PRAGMA")
            else Cursor([(3, "sensor_msgs/msg/PointCloud2", "cdr")])
            if "FROM topics" in query
            else Cursor([(24, 200_000_000, _pointcloud2_cdr([(1.0, 2.0, 3.0)], stamp_ns=300_000_000))])
        )
        with patch("simulator.sensors.scan_projection.sqlite3.connect", return_value=mismatched):
            with self.assertRaisesRegex(ValueError, "header timestamp"):
                read_pointcloud2_sqlite(
                    Path(__file__), topic="/points", timestamp_ns=200_000_000, expected_message_id=24
                )

    def test_resolve_transform_matches_independent_matrix_chain(self):
        angle = np.deg2rad(90.0) / 2.0
        transforms = [
            {
                "parent": "rig",
                "child": "lidar",
                "translation_m": [1.0, 2.0, 3.0],
                "rotation_xyzw": [0.0, 0.0, float(np.sin(angle)), float(np.cos(angle))],
            },
            {
                "parent": "rig",
                "child": "optical",
                "translation_m": [-0.5, 0.25, 1.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        ]
        actual = resolve_transform(transforms, source_frame="lidar", target_frame="optical")
        expected = np.asarray(
            [[0.0, -1.0, 0.0, 1.5], [1.0, 0.0, 0.0, 1.75], [0.0, 0.0, 1.0, 2.0], [0, 0, 0, 1]],
            dtype=np.float64,
        )
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_projection_filters_finite_forward_and_image_without_losing_indices(self):
        # T_optical_from_lidar: optical x=-lidar y, y=lidar z, z=lidar x+1.
        transform = np.asarray(
            [[0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 1.0], [0, 0, 0, 1]],
            dtype=np.float64,
        )
        points = np.asarray(
            [
                [1.0, 0.0, 0.0],       # optical (0,0,2), image center
                [1.0, -0.5, 0.5],      # optical (.5,.5,2)
                [-2.0, 0.0, 0.0],      # behind
                [1.0, -10.0, 0.0],     # outside image
                [float("nan"), 0, 0], # invalid
            ]
        )
        projected = project_lidar_scan(
            points,
            np.arange(10, 15),
            optical_from_lidar=transform,
            intrinsics=CameraIntrinsics(100, 80, 40.0, 40.0, 49.5, 39.5),
            minimum_depth_m=0.1,
            maximum_depth_m=10.0,
        )
        np.testing.assert_array_equal(projected.stage_raw_indices["finite"], [10, 11, 12, 13])
        np.testing.assert_array_equal(projected.stage_raw_indices["forward"], [10, 11, 13])
        np.testing.assert_array_equal(projected.raw_point_indices, [10, 11])
        np.testing.assert_allclose(projected.u_px, [49.5, 59.5])
        np.testing.assert_allclose(projected.v_px, [39.5, 49.5])

    def test_front_surface_selection_is_bounded_and_keeps_raw_indices(self):
        count = 6
        projected = ProjectedScan(
            raw_xyz_m=np.column_stack((np.arange(count), np.zeros(count), np.ones(count))),
            optical_xyz_m=np.column_stack((np.zeros(count), np.zeros(count), [2.0, 2.02, 2.05, 2.08, 2.4, 3.0])),
            u_px=np.asarray([10, 11, 12, 13, 14, 90], dtype=float),
            v_px=np.asarray([10, 11, 12, 13, 14, 90], dtype=float),
            depth_m=np.asarray([2.0, 2.02, 2.05, 2.08, 2.4, 3.0]),
            raw_point_indices=np.asarray([100, 101, 102, 103, 104, 105]),
            stage_raw_indices={},
        )
        selected = select_bbox_front_surface(
            projected, (0, 0, 20, 20), front_surface_band_m=0.1, maximum_selected_points=2
        )
        np.testing.assert_array_equal(selected.stage_raw_indices["bbox"], [100, 101, 102, 103, 104])
        np.testing.assert_array_equal(selected.stage_raw_indices["front_surface"], [100, 101, 102, 103])
        np.testing.assert_array_equal(selected.raw_point_indices, [100, 103])
        self.assertEqual(selected.nearest_depth_m, 2.0)
        self.assertEqual(selected.front_surface_max_depth_m, 2.1)


if __name__ == "__main__":
    unittest.main()
