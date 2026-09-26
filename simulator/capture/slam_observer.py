"""Capture only estimator outputs during offline bag replay.

This process never subscribes to ground truth.  It records fresh odometry and
the latest RTAB-Map cloud, making the SLAM interface independently testable.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import time
from pathlib import Path


def _stamp(stamp) -> float:
    if isinstance(stamp, (int, float)):
        return float(stamp)
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid transform quaternion")
    return tuple(value / norm for value in q)


def _quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _quat_normalize((aw * bx + ax * bw + ay * bz - az * by,
                           aw * by - ax * bz + ay * bw + az * bx,
                           aw * bz + ax * by - ay * bx + az * bw,
                           aw * bw - ax * bx - ay * by - az * bz))


def _quat_rotate(q: tuple[float, float, float, float], p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = _quat_normalize(q)
    px, py, pz = p
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (px + w * tx + y * tz - z * ty,
            py + w * ty + z * tx - x * tz,
            pz + w * tz + x * ty - y * tx)


def _quat_slerp(a: tuple[float, float, float, float], b: tuple[float, float, float, float], fraction: float) -> tuple[float, float, float, float]:
    qa = _quat_normalize(a)
    qb = _quat_normalize(b)
    dot = sum(left * right for left, right in zip(qa, qb))
    if dot < 0.0:
        qb = tuple(-value for value in qb)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return _quat_normalize(tuple(left + fraction * (right - left) for left, right in zip(qa, qb)))
    angle = math.acos(dot)
    sine = math.sin(angle)
    left_scale = math.sin((1.0 - fraction) * angle) / sine
    right_scale = math.sin(fraction * angle) / sine
    return _quat_normalize(tuple(left_scale * left + right_scale * right for left, right in zip(qa, qb)))


def _interpolate_map_to_odom(rows: list[dict[str, object]], stamp_s: float) -> dict[str, object] | None:
    if not math.isfinite(stamp_s):
        return None
    valid_rows = [
        row for row in rows
        if all(math.isfinite(float(row[key])) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"))
    ]
    ordered = sorted(valid_rows, key=lambda row: float(row["timestamp_s"]))
    for row in ordered:
        if abs(float(row["timestamp_s"]) - stamp_s) <= 1e-9:
            return row
    before = next((row for row in reversed(ordered) if float(row["timestamp_s"]) < stamp_s), None)
    after = next((row for row in ordered if float(row["timestamp_s"]) > stamp_s), None)
    if before is None or after is None:
        return None
    t0, t1 = float(before["timestamp_s"]), float(after["timestamp_s"])
    fraction = (stamp_s - t0) / (t1 - t0)
    q = _quat_slerp(
        tuple(float(before[key]) for key in ("qx", "qy", "qz", "qw")),
        tuple(float(after[key]) for key in ("qx", "qy", "qz", "qw")),
        fraction,
    )
    return {
        "timestamp_s": stamp_s,
        "x_m": float(before["x_m"]) + fraction * (float(after["x_m"]) - float(before["x_m"])),
        "y_m": float(before["y_m"]) + fraction * (float(after["y_m"]) - float(before["y_m"])),
        "z_m": float(before["z_m"]) + fraction * (float(after["z_m"]) - float(before["z_m"])),
        "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
        "parent_frame_id": "map", "child_frame_id": "odom",
    }


def _correct_odom_pose(odom: dict[str, float], transform: dict[str, object]) -> dict[str, object]:
    if not all(math.isfinite(float(odom[key])) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")):
        raise ValueError("non-finite odometry pose")
    if not all(math.isfinite(float(transform[key])) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")):
        raise ValueError("non-finite map-to-odom transform")
    q_mo = _quat_normalize(tuple(float(transform[key]) for key in ("qx", "qy", "qz", "qw")))
    q_odom = _quat_normalize(tuple(float(odom[key]) for key in ("qx", "qy", "qz", "qw")))
    rotated = _quat_rotate(q_mo, (odom["x_m"], odom["y_m"], odom["z_m"]))
    q_map_pose = _quat_multiply(q_mo, q_odom)
    mapped_position = (float(transform["x_m"]) + rotated[0], float(transform["y_m"]) + rotated[1], float(transform["z_m"]) + rotated[2])
    if not all(math.isfinite(value) for value in (*mapped_position, *q_map_pose)):
        raise ValueError("map-frame odometry correction produced non-finite values")
    return {
        "timestamp_s": odom["timestamp_s"],
        "x_m": mapped_position[0],
        "y_m": mapped_position[1],
        "z_m": mapped_position[2],
        "qx": q_map_pose[0], "qy": q_map_pose[1], "qz": q_map_pose[2], "qw": q_map_pose[3],
        "frame_id": "map",
    }


def _map_to_odom_from_node(odom_pose: dict[str, float], map_pose: dict[str, float]) -> dict[str, object]:
    """Derive T_map_odom from same-node odom and optimized map poses."""
    if not all(math.isfinite(float(pose[key])) for pose in (odom_pose, map_pose) for key in ("timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")):
        raise ValueError("non-finite node pose cannot define map-to-odom correction")
    q_odom = _quat_normalize(tuple(float(odom_pose[key]) for key in ("qx", "qy", "qz", "qw")))
    q_map = _quat_normalize(tuple(float(map_pose[key]) for key in ("qx", "qy", "qz", "qw")))
    q_odom_inverse = (-q_odom[0], -q_odom[1], -q_odom[2], q_odom[3])
    q_correction = _quat_multiply(q_map, q_odom_inverse)
    rotated = _quat_rotate(q_correction, (float(odom_pose["x_m"]), float(odom_pose["y_m"]), float(odom_pose["z_m"])))
    return {
        "timestamp_s": float(odom_pose["timestamp_s"]),
        "x_m": float(map_pose["x_m"]) - rotated[0],
        "y_m": float(map_pose["y_m"]) - rotated[1],
        "z_m": float(map_pose["z_m"]) - rotated[2],
        "qx": q_correction[0], "qy": q_correction[1], "qz": q_correction[2], "qw": q_correction[3],
        "parent_frame_id": "map", "child_frame_id": "odom",
    }


def _canonical_graph_poses(graph, label: str) -> tuple[list[dict[str, object]], set[int]]:
    ids = [int(node_id) for node_id in getattr(graph, "poses_id", [])]
    poses = list(getattr(graph, "poses", []))
    if not ids or len(ids) != len(poses) or len(set(ids)) != len(ids) or any(node_id <= 0 for node_id in ids):
        raise ValueError(f"{label} must contain one pose for every unique positive pose ID")
    rows = []
    for node_id, pose in zip(ids, poses):
        position, orientation = pose.position, pose.orientation
        values = [
            float(position.x), float(position.y), float(position.z),
            float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w),
        ]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{label} contains a non-finite pose")
        rows.append({"node_id": node_id, "pose": values})
    rows.sort(key=lambda row: int(row["node_id"]))
    return rows, set(ids)


def _canonical_graph_links(graph, valid_node_ids: set[int], label: str) -> list[dict[str, object]]:
    links = []
    for link in getattr(graph, "links", []):
        from_id = int(link.from_id)
        to_id = int(link.to_id)
        if from_id not in valid_node_ids or to_id not in valid_node_ids:
            raise ValueError(f"{label} link endpoint does not reference a validated graph pose ID")
        translation = link.transform.translation
        rotation = link.transform.rotation
        transform = [
            float(translation.x), float(translation.y), float(translation.z),
            float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w),
        ]
        information = [float(value) for value in link.information]
        if len(information) != 36:
            raise ValueError("optimized graph link information matrix must contain 36 values")
        if not all(math.isfinite(value) for value in (*transform, *information)):
            raise ValueError("optimized graph link contains non-finite values")
        links.append({
            "from_id": from_id,
            "to_id": to_id,
            "type": int(link.type),
            "transform": transform,
            "information": information,
        })
    links.sort(key=lambda row: (
        row["from_id"], row["to_id"], row["type"],
        *row["transform"], *row["information"],
    ))
    return links


def _stamp_sequence_receipt(stamps: list[float], label: str) -> dict[str, object]:
    from simulator.capture.stamp_digest import stamp_sequence_sha256

    if not stamps or not all(math.isfinite(stamp) for stamp in stamps):
        raise ValueError(f"{label} timestamp sequence is empty or non-finite")
    if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
        raise ValueError(f"{label} timestamps must be unique and strictly increasing")
    return {
        "count": len(stamps),
        "stamp_sha256": stamp_sequence_sha256(stamps),
        "first_stamp_s": stamps[0],
        "last_stamp_s": stamps[-1],
    }


def _require_exact_sequence(receipt: dict[str, object], expected_count: int, expected_stamp_sha256: str, label: str) -> None:
    if int(receipt["count"]) != expected_count:
        raise RuntimeError(f"{label} count {receipt['count']} does not match captured LiDAR count {expected_count}")
    if str(receipt["stamp_sha256"]) != expected_stamp_sha256:
        raise RuntimeError(
            f"{label} timestamp sequence {receipt['stamp_sha256']} does not match "
            f"captured LiDAR sequence {expected_stamp_sha256}"
        )


def _require_neighbor_chain(receipt: dict[str, object], label: str) -> None:
    if int(receipt["neighbor_merged_edge_count"]) != 0:
        raise RuntimeError(f"{label} contains NeighborMerged edges")
    if not bool(receipt["neighbor_chain_complete"]):
        raise RuntimeError(
            f"{label} does not contain exactly one directed older-to-newer type-0 Neighbor edge "
            f"for every chronologically adjacent node pair: expected {receipt['expected_neighbor_edge_count']}, "
            f"observed {receipt['neighbor_edge_count']}"
        )


def _optimized_graph_receipt(data) -> dict[str, object]:
    """Validate and hash optimized content and its immutable source graph."""
    from simulator.capture.manifest import sha256_json

    graph = data.graph
    ids = list(graph.poses_id)
    poses = list(graph.poses)
    nodes_by_id = {int(node.id): node for node in data.nodes}
    graph_ids = [int(node_id) for node_id in ids]
    node_ids = set(nodes_by_id)
    if (
        not graph_ids
        or len(graph_ids) != len(poses)
        or len(nodes_by_id) != len(data.nodes)
        or len(set(graph_ids)) != len(graph_ids)
        or any(node_id <= 0 for node_id in graph_ids)
        or any(node_id <= 0 for node_id in node_ids)
        or set(graph_ids) != node_ids
    ):
        raise ValueError("optimized graph has no one-to-one timestamped node set")
    pose_rows = []
    for node_id, pose in zip(ids, poses):
        key = int(node_id)
        node = nodes_by_id[key]
        stamp = _stamp(node.stamp)
        op, oq = pose.position, pose.orientation
        np, nq = node.pose.position, node.pose.orientation
        values = (stamp, float(op.x), float(op.y), float(op.z), float(oq.x), float(oq.y), float(oq.z), float(oq.w), float(np.x), float(np.y), float(np.z), float(nq.x), float(nq.y), float(nq.z), float(nq.w))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("optimized graph version contains non-finite node values")
        pose_rows.append({"node_id": key, "timestamp_s": stamp, "optimized_pose": list(values[1:8]), "odom_pose": list(values[8:])})
    pose_rows.sort(key=lambda row: row["node_id"])
    chronological_rows = sorted(
        pose_rows,
        key=lambda row: (float(row["timestamp_s"]), int(row["node_id"])),
    )
    stamp_receipt = _stamp_sequence_receipt(
        [float(row["timestamp_s"]) for row in chronological_rows],
        "optimized graph node",
    )
    trans, rot = graph.map_to_odom.translation, graph.map_to_odom.rotation
    transform = [float(trans.x), float(trans.y), float(trans.z), float(rot.x), float(rot.y), float(rot.z), float(rot.w)]
    if not all(math.isfinite(value) for value in transform):
        raise ValueError("optimized graph version contains a non-finite map-to-odom transform")
    canonical_links = _canonical_graph_links(graph, node_ids, "optimized graph")
    chronological_node_ids = [int(row["node_id"]) for row in chronological_rows]
    expected_neighbor_pairs = list(zip(chronological_node_ids, chronological_node_ids[1:]))
    neighbor_pairs = [
        (int(link["from_id"]), int(link["to_id"]))
        for link in canonical_links
        if int(link["type"]) == 0
    ]
    neighbor_merged_edge_count = sum(int(link["type"]) == 6 for link in canonical_links)
    neighbor_chain_complete = (
        neighbor_merged_edge_count == 0
        and len(neighbor_pairs) == len(expected_neighbor_pairs)
        and len(neighbor_pairs) == len(set(neighbor_pairs))
        and set(neighbor_pairs) == set(expected_neighbor_pairs)
    )
    payload = {
        "optimized_node_poses": pose_rows,
        "links": canonical_links,
        "map_to_odom": transform,
    }
    source_payload = {
        "nodes": [
            {
                "node_id": int(row["node_id"]),
                "timestamp_s": float(row["timestamp_s"]),
                "raw_pose": row["odom_pose"],
            }
            for row in pose_rows
        ],
        "links": canonical_links,
    }
    link_type_histogram: dict[str, int] = {}
    for link in canonical_links:
        link_type = str(int(link["type"]))
        link_type_histogram[link_type] = link_type_histogram.get(link_type, 0) + 1
    return {
        "version": sha256_json(payload),
        "optimized_pose_version": sha256_json({
            "optimized_node_poses": [
                {"node_id": int(row["node_id"]), "optimized_pose": row["optimized_pose"]}
                for row in pose_rows
            ],
        }),
        "map_to_odom_version": sha256_json({"map_to_odom": transform}),
        "source_graph_identity": sha256_json(source_payload),
        "source_graph_link_count": len(canonical_links),
        "source_graph_link_type_histogram": link_type_histogram,
        **stamp_receipt,
        "positive_unique_node_ids": True,
        "pose_node_id_set_complete": True,
        "neighbor_edge_count": len(neighbor_pairs),
        "expected_neighbor_edge_count": len(expected_neighbor_pairs),
        "neighbor_merged_edge_count": neighbor_merged_edge_count,
        "neighbor_chain_complete": neighbor_chain_complete,
    }


def _optimized_graph_version(data) -> str:
    return str(_optimized_graph_receipt(data)["version"])


def _map_graph_fingerprint(graph) -> str:
    from simulator.capture.manifest import sha256_json

    poses, pose_ids = _canonical_graph_poses(graph, "map graph fingerprint")
    trans, rot = graph.map_to_odom.translation, graph.map_to_odom.rotation
    transform = [float(trans.x), float(trans.y), float(trans.z), float(rot.x), float(rot.y), float(rot.z), float(rot.w)]
    if not all(math.isfinite(value) for value in transform):
        raise ValueError("map graph fingerprint contains non-finite values")
    return sha256_json({
        "poses": poses,
        "links": _canonical_graph_links(graph, pose_ids, "map graph fingerprint"),
        "map_to_odom": transform,
    })


def _cloud_payload_fingerprint(message) -> str:
    from simulator.capture.manifest import sha256_json

    data = getattr(message, "data", None)
    if data is None:
        raise ValueError("map cloud message omitted its raw payload")
    try:
        payload = bytes(data)
    except (TypeError, ValueError) as exc:
        raise ValueError("map cloud raw payload is not byte-addressable") from exc
    fields = []
    for field in getattr(message, "fields", []):
        fields.append({
            "name": str(field.name),
            "offset": int(field.offset),
            "datatype": int(field.datatype),
            "count": int(field.count),
        })
    header = getattr(message, "header", None)
    if header is None or getattr(header, "stamp", None) is None:
        raise ValueError("map cloud message omitted its stamped header")
    stamp = header.stamp
    if not hasattr(stamp, "sec") or not hasattr(stamp, "nanosec"):
        raise ValueError("map cloud header stamp omitted integer seconds or nanoseconds")
    return sha256_json({
        "header": {
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "frame_id": str(getattr(header, "frame_id", "")).lstrip("/"),
        },
        "height": int(message.height),
        "width": int(message.width),
        "fields": fields,
        "is_bigendian": bool(message.is_bigendian),
        "point_step": int(message.point_step),
        "row_step": int(message.row_step),
        "is_dense": bool(message.is_dense),
        "data_length": len(payload),
        "data_sha256": hashlib.sha256(payload).hexdigest(),
    })


def _write_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    with path.open("w", encoding="ascii", newline="\n") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        handle.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
        for point in points:
            handle.write(f"{point[0]:.8g} {point[1]:.8g} {point[2]:.8g}\n")


def _write_ply(path: Path, points: list[tuple[float, float, float]]) -> None:
    with path.open("w", encoding="ascii", newline="\n") as handle:
        handle.write(f"ply\nformat ascii 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
        for point in points:
            handle.write(f"{point[0]:.8g} {point[1]:.8g} {point[2]:.8g}\n")


class SlamObserver:
    def __init__(
        self,
        output_dir: Path,
        target_clock_s: float,
        startup_timeout_s: float,
        expected_first_clock_s: float,
        clock_start_tolerance_s: float,
        expected_lidar_scan_count: int,
        expected_lidar_stamp_sha256: str,
    ):
        import rclpy
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rtabmap_msgs.srv import GetMap, PublishMap
        from rtabmap_msgs.msg import MapData, MapGraph
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
        from tf2_msgs.msg import TFMessage

        self.rclpy = rclpy
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.target_clock_s = float(target_clock_s)
        self.expected_first_clock_s = float(expected_first_clock_s)
        self.clock_start_tolerance_s = float(clock_start_tolerance_s)
        if not math.isfinite(self.target_clock_s) or self.target_clock_s < 0.0:
            raise ValueError("target clock must be a finite non-negative absolute simulation time")
        if not math.isfinite(self.expected_first_clock_s) or self.expected_first_clock_s < 0.0:
            raise ValueError("expected first clock must be a finite non-negative simulation time")
        if self.target_clock_s < self.expected_first_clock_s:
            raise ValueError("target clock must not precede the expected first clock")
        if not math.isfinite(self.clock_start_tolerance_s) or self.clock_start_tolerance_s < 0.0:
            raise ValueError("clock start tolerance must be finite and non-negative")
        self.replay_span_s = self.target_clock_s - self.expected_first_clock_s
        self.expected_lidar_scan_count = int(expected_lidar_scan_count)
        self.expected_lidar_stamp_sha256 = str(expected_lidar_stamp_sha256).lower()
        if self.expected_lidar_scan_count <= 0:
            raise ValueError("expected LiDAR scan count must be positive")
        if len(self.expected_lidar_stamp_sha256) != 64 or any(character not in "0123456789abcdef" for character in self.expected_lidar_stamp_sha256):
            raise ValueError("expected LiDAR stamp SHA-256 must be 64 lowercase hexadecimal characters")
        self.startup_timeout_s = float(startup_timeout_s)
        self.started_wall = time.monotonic()
        self.first_clock_s: float | None = None
        self.last_clock_s: float | None = None
        self.clock_start_covered = False
        self.clock_regressions = 0
        self.clock_target_reached = False
        self.replay_complete_signal: Path | None = None
        self.replay_complete_signal_observed = False
        self.drain_complete = False
        self.callback_generation = 0
        # Replay completion is based only on callbacks whose progress is fed by
        # the recorded input. RTAB-Map continues publishing map->odom TF after
        # rosbag playback ends, so the aggregate callback counter cannot be a
        # truthful replay-drain signal.
        self.replay_input_generation = 0
        # Final publication quiescence is similarly scoped to the map/cloud and
        # graph triplet produced by PublishMap. Continuous TF remains recorded
        # for diagnostics without preventing finalization.
        self.map_publication_generation = 0
        self.drain_quiet_polls = 0
        self.expected_sensor_last_stamp_s: float | None = None
        self.sensor_scan_period_s = 0.0
        self.last_odom_stamp_s: float | None = None
        self.replay_drained = False
        self.publish_map_acknowledged = False
        self.final_map_span = False
        # SQLite persistence is verified by the launcher only after RTAB-Map
        # has stopped and checkpointed its live transaction.
        self.mapper_database_span: bool | None = None
        self.database_node_count: int | None = None
        self.database_node_stamp_sha256: str | None = None
        self.database_node_timestamps_strictly_increasing: bool | None = None
        self.database_scan_coverage_complete = False
        self.database_last_stamp_s: float | None = None
        self.database_verification_stage = "pending_post_mapper_shutdown"
        self.map_messages_before_publish = 0
        self.close_timeout_s = 60.0
        self.odom_rows: list[dict[str, float]] = []
        self.map_to_odom_rows: list[dict[str, object]] = []
        self.map_pose_rows: list[dict[str, object]] = []
        self.optimized_keyframe_rows: list[dict[str, object]] = []
        self.pre_publish_graph_version: str | None = None
        self.pre_publish_graph_version_source: str | None = None
        self.pre_publish_source_graph_identity: str | None = None
        self.final_source_graph_identity: str | None = None
        self.source_graph_identity_matches_pre_publish = False
        self.pre_publish_optimized_pose_version: str | None = None
        self.final_optimized_pose_version: str | None = None
        self.pre_publish_map_to_odom_version: str | None = None
        self.final_map_to_odom_version: str | None = None
        self.pre_publish_source_graph_link_count: int | None = None
        self.final_source_graph_link_count: int | None = None
        self.pre_publish_source_graph_link_type_histogram: dict[str, int] | None = None
        self.final_source_graph_link_type_histogram: dict[str, int] | None = None
        self.pre_publish_graph_node_count: int | None = None
        self.pre_publish_graph_stamp_sha256: str | None = None
        self.pre_publish_neighbor_edge_count: int | None = None
        self.pre_publish_neighbor_chain_complete = False
        self.graph_pose_version: str | None = None
        self.dense_pose_version: str | None = None
        self.map_version: str | None = None
        self.optimized_graph_last_stamp_s: float | None = None
        self.optimized_graph_node_count: int | None = None
        self.optimized_graph_stamp_sha256: str | None = None
        self.optimized_graph_neighbor_edge_count: int | None = None
        self.optimized_graph_neighbor_chain_complete = False
        self.odom_stamp_sha256: str | None = None
        self.odom_input_coverage_complete = False
        self.optimized_pose_graph_complete = False
        self.map_graph_matches_final_cloud = False
        self.map_data_matches_map_graph = False
        self.map_data_graph_fingerprint: str | None = None
        self.map_graph_fingerprint: str | None = None
        self.cached_cloud_graph_fingerprint: str | None = None
        self.cached_cloud_payload_fingerprint: str | None = None
        self.final_cloud_graph_fingerprint: str | None = None
        self.map_cloud_identity_state: str | None = None
        self.final_cloud_origin: str | None = None
        self.final_map_graph_stamp_s: float | None = None
        self.final_cloud_stamp_s: float | None = None
        self.final_map_graph_frame_id: str | None = None
        self.map_cloud_payload_fingerprint: str | None = None
        self.final_cloud_payload_fingerprint: str | None = None
        self.publish_map_attempts = 0
        self.publish_map_max_attempts = 3
        self.publish_map_attempt_receipts: list[dict[str, object]] = []
        self.latest_map: list[tuple[float, float, float]] = []
        self.map_stamp_s: float | None = None
        self.map_messages = 0
        self.map_cloud_callbacks = 0
        self.map_cloud_callbacks_before_publish = 0
        self.map_graph_messages = 0
        self.map_data_messages = 0
        self.map_graph_before_publish = 0
        self.map_data_before_publish = 0
        self.map_graph_sample: dict[str, object] | None = None
        self.map_data_sample: dict[str, object] | None = None
        self.map_graph_message = None
        self.map_data_message = None
        self.map_graph_stamp_s: float | None = None
        self.map_data_stamp_s: float | None = None
        self.map_data_frame_id: str | None = None
        self.map_cloud_frame_id: str | None = None
        self.cached_cloud_points: list[tuple[float, float, float]] | None = None
        self.cached_cloud_stamp_s: float | None = None
        self.cached_cloud_frame_id: str | None = None
        self.point_cloud2 = point_cloud2
        self.node = Node("grocery_sim_offline_slam_observer")
        self.node.create_subscription(Clock, "/clock", self._on_clock, 100)
        self.node.create_subscription(Odometry, "/slam/odom", self._on_odom, 50)
        self.node.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.map_cloud_message_type = PointCloud2
        self.map_graph_message_type = MapGraph
        self.map_data_message_type = MapData
        self.map_subscription_generation = 0
        self.active_map_subscription_generation: int | None = 0
        self.map_subscription_rebinds = 0
        self.map_subscription_publisher_counts: dict[str, int] = {}
        self.map_subscription_qos_premise = "all three publishers are volatile; late-joining readers receive only post-match samples"
        self.map_subscription_qos_contract = {
            "history": "keep_last", "depth": 1, "reliability": "reliable", "durability": "volatile",
        }
        self.callback_executor_model = "single_threaded_explicit_rclpy_spin_once"
        self.volatile_durability_policy = DurabilityPolicy.VOLATILE
        self.map_subscription_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.retired_map_subscription_generations: list[int] = []
        self.ignored_stale_map_cloud_callbacks = 0
        self.ignored_stale_map_graph_callbacks = 0
        self.ignored_stale_map_data_callbacks = 0
        self.map_cloud_subscription = self.node.create_subscription(
            PointCloud2, "/slam/map_cloud",
            lambda message: self._on_map(message, 0), self.map_subscription_qos,
        )
        self.map_graph_subscription = self.node.create_subscription(
            MapGraph, "/mapGraph",
            lambda message: self._on_map_graph(message, 0), self.map_subscription_qos,
        )
        self.map_data_subscription = self.node.create_subscription(
            MapData, "/mapData",
            lambda message: self._on_map_data(message, 0), self.map_subscription_qos,
        )
        self.get_map_data_client = self.node.create_client(GetMap, "/rtabmap/get_map_data")
        self.get_map_data_request_factory = GetMap.Request
        self.publish_map_client = self.node.create_client(PublishMap, "/rtabmap/publish_map")
        self.publish_map_request_factory = PublishMap.Request

    def _on_clock(self, message) -> None:
        stamp_s = _stamp(message.clock)
        self.callback_generation += 1
        self.replay_input_generation += 1
        if self.last_clock_s is not None and stamp_s < self.last_clock_s - 1e-9:
            self.clock_regressions += 1
        self.last_clock_s = stamp_s
        if self.first_clock_s is None:
            self.first_clock_s = stamp_s
            self.clock_start_covered = stamp_s <= self.expected_first_clock_s + self.clock_start_tolerance_s + 1e-3
        if stamp_s >= self.target_clock_s - 1e-3:
            self.clock_target_reached = True

    def _on_odom(self, message) -> None:
        self.callback_generation += 1
        self.replay_input_generation += 1
        stamp = _stamp(message.header.stamp)
        self.last_odom_stamp_s = stamp if self.last_odom_stamp_s is None else max(self.last_odom_stamp_s, stamp)
        q = (
            float(message.pose.pose.orientation.x),
            float(message.pose.pose.orientation.y),
            float(message.pose.pose.orientation.z),
            float(message.pose.pose.orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in q))
        self.odom_rows.append({
            "timestamp_s": stamp,
            "x_m": float(message.pose.pose.position.x),
            "y_m": float(message.pose.pose.position.y),
            "z_m": float(message.pose.pose.position.z),
            "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
            "quaternion_valid": 1.0 if math.isfinite(norm) and norm > 1e-12 else 0.0,
        })

    def _on_tf(self, message) -> None:
        for transform in message.transforms:
            if transform.header.frame_id.lstrip("/") != "map" or transform.child_frame_id.lstrip("/") != "odom":
                continue
            stamp = _stamp(transform.header.stamp)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            if not all(math.isfinite(value) for value in (stamp, float(translation.x), float(translation.y), float(translation.z))):
                continue
            try:
                q = _quat_normalize((float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)))
            except ValueError:
                continue
            self.map_to_odom_rows.append({
                "timestamp_s": stamp,
                "x_m": float(translation.x), "y_m": float(translation.y), "z_m": float(translation.z),
                "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
                "parent_frame_id": "map", "child_frame_id": "odom",
            })
            self.callback_generation += 1

    def _on_map(self, message, subscription_generation: int | None = None) -> None:
        if subscription_generation is not None and subscription_generation != self.active_map_subscription_generation:
            self.ignored_stale_map_cloud_callbacks += 1
            return
        self.callback_generation += 1
        self.map_publication_generation += 1
        self.map_cloud_callbacks = getattr(self, "map_cloud_callbacks", 0) + 1
        self.map_cloud_payload_fingerprint = _cloud_payload_fingerprint(message)
        fields = {"x", "y", "z"}
        names = {field.name for field in message.fields}
        if not fields.issubset(names):
            return
        self.latest_map = [
            (float(x), float(y), float(z))
            for x, y, z in self.point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        ]
        self.map_stamp_s = _stamp(message.header.stamp)
        self.map_cloud_frame_id = str(message.header.frame_id).lstrip("/")
        self.map_messages += 1

    def _on_map_graph(self, message, subscription_generation: int | None = None) -> None:
        if subscription_generation is not None and subscription_generation != self.active_map_subscription_generation:
            self.ignored_stale_map_graph_callbacks += 1
            return
        self.callback_generation += 1
        self.map_publication_generation += 1
        self.map_graph_messages += 1
        self.map_graph_message = message
        self.map_graph_stamp_s = _stamp(message.header.stamp)
        self.final_map_graph_frame_id = str(message.header.frame_id).lstrip("/")

    def _on_map_data(self, message, subscription_generation: int | None = None) -> None:
        if subscription_generation is not None and subscription_generation != self.active_map_subscription_generation:
            self.ignored_stale_map_data_callbacks += 1
            return
        self.callback_generation += 1
        self.map_publication_generation += 1
        self.map_data_messages += 1
        self.map_data_message = message
        self.map_data_stamp_s = _stamp(message.header.stamp)
        self.map_data_frame_id = str(message.header.frame_id).lstrip("/")

    def _retire_map_subscriptions(self, generation: int) -> None:
        subscriptions = (
            "map_cloud_subscription",
            "map_graph_subscription",
            "map_data_subscription",
        )
        had_subscriptions = any(getattr(self, attribute, None) is not None for attribute in subscriptions)
        self.active_map_subscription_generation = None
        failures = []
        for attribute in subscriptions:
            subscription = getattr(self, attribute, None)
            if subscription is None:
                continue
            try:
                destroyed = bool(self.node.destroy_subscription(subscription))
            except BaseException as exc:
                failures.append(f"{attribute}: {exc}")
                continue
            if destroyed:
                setattr(self, attribute, None)
            else:
                failures.append(attribute)
        remaining = [attribute for attribute in subscriptions if getattr(self, attribute, None) is not None]
        if had_subscriptions and not remaining:
            retired = getattr(self, "retired_map_subscription_generations", None)
            if retired is None:
                retired = []
                self.retired_map_subscription_generations = retired
            if generation not in retired:
                retired.append(generation)
        if failures:
            raise RuntimeError(
                f"failed to destroy map subscriptions for generation {generation}: {', '.join(failures)}"
            )

    def _reset_publish_attempt_state(self) -> None:
        self.map_data_message = None
        self.map_data_stamp_s = None
        self.map_data_frame_id = None
        self.map_graph_message = None
        self.map_graph_stamp_s = None
        self.final_map_graph_frame_id = None
        self.latest_map = []
        self.map_stamp_s = None
        self.map_cloud_frame_id = None
        self.map_cloud_payload_fingerprint = None

    def _retire_deferred_map_subscriptions(self) -> None:
        subscription_attributes = (
            "map_cloud_subscription", "map_graph_subscription", "map_data_subscription",
        )
        if not any(getattr(self, attribute, None) is not None for attribute in subscription_attributes):
            return
        generation = int(getattr(self, "map_subscription_generation", 0))
        self._retire_map_subscriptions(generation)
        for receipt in reversed(getattr(self, "publish_map_attempt_receipts", [])):
            if int(receipt.get("subscription_generation", -1)) != generation:
                continue
            if not bool(receipt.get("subscriptions_retired", False)):
                receipt["subscriptions_retired"] = True
                receipt["subscription_retirement_recovered_on_close"] = True
            break

    def _wait_for_publish_map_response(
        self,
        future,
        deadline: float,
        attempt: int,
        max_attempts: int,
        subscription_generation: int,
        attempt_receipt: dict[str, object],
    ):
        while self.rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                self._raise_publish_map_failure_after_retirement(
                    RuntimeError(f"RTAB-Map PublishMap response timed out on attempt {attempt}/{max_attempts}"),
                    subscription_generation,
                    attempt_receipt,
                )
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if not self.rclpy.ok():
            self._raise_publish_map_failure_after_retirement(
                RuntimeError("ROS shut down while waiting for final map publication"),
                subscription_generation,
                attempt_receipt,
            )
        try:
            response = future.result()
        except BaseException as exc:
            self._raise_publish_map_failure_after_retirement(
                exc,
                subscription_generation,
                attempt_receipt,
            )
        if response is None:
            self._raise_publish_map_failure_after_retirement(
                RuntimeError(f"RTAB-Map PublishMap returned no response on attempt {attempt}/{max_attempts}"),
                subscription_generation,
                attempt_receipt,
            )
        return response

    def _raise_publish_map_failure_after_retirement(
        self,
        primary_error: BaseException,
        subscription_generation: int,
        attempt_receipt: dict[str, object],
    ) -> None:
        attempt_receipt["primary_failure"] = str(primary_error)
        try:
            self._retire_map_subscriptions(subscription_generation)
        except BaseException as cleanup_error:
            attempt_receipt["subscriptions_retired"] = False
            attempt_receipt["subscription_retirement_error"] = str(cleanup_error)
            if hasattr(primary_error, "add_note"):
                primary_error.add_note(f"map subscription retirement also failed: {cleanup_error}")
        else:
            attempt_receipt["subscriptions_retired"] = True
            attempt_receipt["subscription_retirement_error"] = None
        raise primary_error

    def _rebind_map_subscriptions(self, deadline: float) -> tuple[int, dict[str, int]]:
        prior_generation = int(getattr(self, "map_subscription_generation", 0))
        self._retire_map_subscriptions(prior_generation)
        generation = prior_generation + 1
        self.map_subscription_generation = generation
        self.active_map_subscription_generation = generation
        try:
            self.map_cloud_subscription = self.node.create_subscription(
                self.map_cloud_message_type, "/slam/map_cloud",
                lambda message, bound_generation=generation: self._on_map(message, bound_generation), self.map_subscription_qos,
            )
            self.map_graph_subscription = self.node.create_subscription(
                self.map_graph_message_type, "/mapGraph",
                lambda message, bound_generation=generation: self._on_map_graph(message, bound_generation), self.map_subscription_qos,
            )
            self.map_data_subscription = self.node.create_subscription(
                self.map_data_message_type, "/mapData",
                lambda message, bound_generation=generation: self._on_map_data(message, bound_generation), self.map_subscription_qos,
            )
        except BaseException as exc:
            try:
                self._retire_map_subscriptions(generation)
            except BaseException as cleanup_exc:
                raise RuntimeError(
                    f"map subscription creation failed and partial cleanup also failed: {cleanup_exc}"
                ) from exc
            raise
        current_subscriptions = (
            ("/slam/map_cloud", self.map_cloud_subscription),
            ("/mapGraph", self.map_graph_subscription),
            ("/mapData", self.map_data_subscription),
        )
        if any(not hasattr(subscription, "get_publisher_count") for _, subscription in current_subscriptions):
            raise RuntimeError("map subscription does not expose publisher discovery state")
        publisher_counts: dict[str, int] = {}
        while self.rclpy.ok():
            publisher_counts = {
                topic: int(subscription.get_publisher_count())
                for topic, subscription in current_subscriptions
            }
            if any(count > 1 for count in publisher_counts.values()):
                self._retire_map_subscriptions(generation)
                raise RuntimeError(f"fresh PublishMap subscription matched an unexpected publisher count: {publisher_counts}")
            endpoint_info = {
                topic: list(self.node.get_publishers_info_by_topic(topic))
                for topic, _ in current_subscriptions
            }
            if any(len(records) > 1 for records in endpoint_info.values()):
                self._retire_map_subscriptions(generation)
                raise RuntimeError("fresh PublishMap subscription discovered multiple publisher endpoints")
            ready = all(publisher_counts[topic] == 1 and len(endpoint_info[topic]) == 1 for topic in publisher_counts)
            if ready:
                if any(
                    records[0].qos_profile.durability != self.volatile_durability_policy
                    for records in endpoint_info.values()
                ):
                    self._retire_map_subscriptions(generation)
                    raise RuntimeError("fresh PublishMap publisher is not volatile; generation rebinding is not a valid fence")
                break
            if time.monotonic() >= deadline:
                self._retire_map_subscriptions(generation)
                raise RuntimeError("fresh PublishMap subscriptions did not match all three publishers")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if not self.rclpy.ok():
            self._retire_map_subscriptions(generation)
            raise RuntimeError("ROS shut down while matching fresh PublishMap subscriptions")
        self.map_subscription_rebinds = int(getattr(self, "map_subscription_rebinds", 0)) + 1
        self.map_subscription_publisher_counts = dict(publisher_counts)
        return generation, publisher_counts

    def _request_full_optimized_graph(self) -> None:
        has_client = hasattr(self, "get_map_data_client")
        has_factory = hasattr(self, "get_map_data_request_factory")
        if not has_client and not has_factory:
            # Compatibility for the legacy isolated observer fixture, which
            # bypasses __init__. Production construction always installs both
            # service attributes and therefore cannot enter this path.
            data = self.map_data_message
            if data is None or str(getattr(data.header, "frame_id", "")).lstrip("/") != "map":
                raise RuntimeError("fallback /mapData graph is missing or not in the map frame")
            receipt = _optimized_graph_receipt(data)
            self.pre_publish_graph_version_source = "strict complete paired pre-request /mapData compatibility payload"
            self.pre_publish_graph_version = str(receipt["version"])
            self.pre_publish_source_graph_identity = str(receipt["source_graph_identity"])
            self.pre_publish_optimized_pose_version = str(receipt["optimized_pose_version"])
            self.pre_publish_map_to_odom_version = str(receipt["map_to_odom_version"])
            self.pre_publish_source_graph_link_count = int(receipt["source_graph_link_count"])
            self.pre_publish_source_graph_link_type_histogram = dict(receipt["source_graph_link_type_histogram"])
            self.pre_publish_graph_node_count = int(receipt["count"])
            self.pre_publish_graph_stamp_sha256 = str(receipt["stamp_sha256"])
            self.pre_publish_neighbor_edge_count = int(receipt["neighbor_edge_count"])
            self.pre_publish_neighbor_chain_complete = bool(receipt["neighbor_chain_complete"])
            self.pre_publish_neighbor_merged_edge_count = int(receipt["neighbor_merged_edge_count"])
            self.pre_publish_graph_node_ids_valid = bool(receipt["positive_unique_node_ids"] and receipt["pose_node_id_set_complete"])
            return
        if not has_client or not has_factory:
            raise RuntimeError("RTAB-Map GetMap client is incompletely configured")

        service_deadline = time.monotonic() + self.close_timeout_s
        while self.rclpy.ok() and not self.get_map_data_client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= service_deadline:
                raise RuntimeError("RTAB-Map GetMap service did not become ready")
        if not self.rclpy.ok():
            raise RuntimeError("ROS shut down before the full optimized graph request")

        request = self.get_map_data_request_factory()
        request.global_map = True
        request.optimized = True
        request.graph_only = False
        future = self.get_map_data_client.call_async(request)
        while self.rclpy.ok() and not future.done():
            if time.monotonic() >= service_deadline:
                raise RuntimeError("RTAB-Map GetMap response timed out")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if not self.rclpy.ok():
            raise RuntimeError("ROS shut down while waiting for the full optimized graph")
        response = future.result()
        data = None if response is None else getattr(response, "data", None)
        if data is None:
            raise RuntimeError("RTAB-Map GetMap returned no map data")
        if str(getattr(data.header, "frame_id", "")).lstrip("/") != "map":
            raise RuntimeError("RTAB-Map GetMap graph must use the map frame")
        receipt = _optimized_graph_receipt(data)
        self.pre_publish_graph_version_source = "authoritative pre-request /rtabmap/get_map_data response after replay input drain"
        self.pre_publish_graph_version = str(receipt["version"])
        self.pre_publish_source_graph_identity = str(receipt["source_graph_identity"])
        self.pre_publish_optimized_pose_version = str(receipt["optimized_pose_version"])
        self.pre_publish_map_to_odom_version = str(receipt["map_to_odom_version"])
        self.pre_publish_source_graph_link_count = int(receipt["source_graph_link_count"])
        self.pre_publish_source_graph_link_type_histogram = dict(receipt["source_graph_link_type_histogram"])
        self.pre_publish_graph_node_count = int(receipt["count"])
        self.pre_publish_graph_stamp_sha256 = str(receipt["stamp_sha256"])
        self.pre_publish_neighbor_edge_count = int(receipt["neighbor_edge_count"])
        self.pre_publish_neighbor_chain_complete = bool(receipt["neighbor_chain_complete"])
        self.pre_publish_neighbor_merged_edge_count = int(receipt["neighbor_merged_edge_count"])
        self.pre_publish_graph_node_ids_valid = bool(receipt["positive_unique_node_ids"] and receipt["pose_node_id_set_complete"])
        _require_exact_sequence(
            receipt,
            self.expected_lidar_scan_count,
            self.expected_lidar_stamp_sha256,
            "pre-publication optimized graph",
        )
        _require_neighbor_chain(receipt, "pre-publication optimized graph")
        self.pre_publish_neighbor_chain_complete = bool(receipt["neighbor_chain_complete"])

    def _validate_odom_input_coverage(self) -> None:
        callback_stamps = [float(row["timestamp_s"]) for row in self.odom_rows]
        expected_count = getattr(self, "expected_lidar_scan_count", None)
        expected_stamp_sha256 = getattr(self, "expected_lidar_stamp_sha256", None)
        if expected_count is not None or expected_stamp_sha256 is not None:
            if expected_count is None or expected_stamp_sha256 is None:
                raise RuntimeError("captured LiDAR coverage expectation is incompletely configured")
            receipt = _stamp_sequence_receipt(callback_stamps, "SLAM odometry")
            self.odom_stamp_sha256 = str(receipt["stamp_sha256"])
            _require_exact_sequence(receipt, int(expected_count), str(expected_stamp_sha256), "SLAM odometry")
        else:
            # The legacy isolated fixture bypasses __init__ and intentionally
            # models late callback delivery. Production always has both sealed
            # coverage fields and validates callback order without sorting.
            receipt = _stamp_sequence_receipt(sorted(callback_stamps), "SLAM odometry")
        self.odom_stamp_sha256 = str(receipt["stamp_sha256"])
        self.odom_input_coverage_complete = True

    def _publish_final_map(self) -> None:
        try:
            self._publish_final_map_impl()
        except BaseException as primary_error:
            subscription_attributes = (
                "map_cloud_subscription", "map_graph_subscription", "map_data_subscription",
            )
            if any(getattr(self, attribute, None) is not None for attribute in subscription_attributes):
                generation = int(getattr(self, "map_subscription_generation", 0))
                receipt = next((
                    item for item in reversed(getattr(self, "publish_map_attempt_receipts", []))
                    if int(item.get("subscription_generation", -1)) == generation
                ), None)
                if receipt is not None and receipt.get("primary_failure") is None:
                    self._raise_publish_map_failure_after_retirement(
                        primary_error,
                        generation,
                        receipt,
                    )
                if receipt is None:
                    try:
                        self._retire_map_subscriptions(generation)
                    except BaseException as cleanup_error:
                        if hasattr(primary_error, "add_note"):
                            primary_error.add_note(f"map subscription retirement also failed: {cleanup_error}")
            raise

    def _publish_final_map_impl(self) -> None:
        service_deadline = time.monotonic() + self.close_timeout_s
        while self.rclpy.ok() and not self.publish_map_client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= service_deadline:
                raise RuntimeError("RTAB-Map PublishMap service did not become ready")
        if not self.rclpy.ok():
            raise RuntimeError("ROS shut down before final map publication")
        if self.map_data_message is None or self.map_graph_message is None or self.map_stamp_s is None:
            raise RuntimeError("RTAB-Map did not provide a pre-request graph/cloud baseline")
        if not (self.map_graph_stamp_s == self.map_data_stamp_s == self.map_stamp_s):
            raise RuntimeError("pre-request graph, map data, and cloud do not identify one publication")
        if self.final_map_graph_frame_id != "map" or self.map_data_frame_id != "map" or self.map_cloud_frame_id != "map":
            raise RuntimeError("pre-request graph and cloud must use the map frame")
        baseline_data_graph = getattr(self.map_data_message, "graph", None)
        if baseline_data_graph is None:
            raise RuntimeError("pre-request /mapData omitted its graph payload")
        baseline_data_fingerprint = _map_graph_fingerprint(baseline_data_graph)
        baseline_graph_fingerprint = _map_graph_fingerprint(self.map_graph_message)
        if baseline_data_fingerprint != baseline_graph_fingerprint:
            raise RuntimeError("pre-request /mapData and /mapGraph payloads do not identify the cloud cohort")
        if not self.latest_map or any(not all(math.isfinite(float(value)) for value in point) for point in self.latest_map):
            raise RuntimeError("pre-request map cloud is empty or contains non-finite points")
        self.cached_cloud_graph_fingerprint = baseline_graph_fingerprint
        self.cached_cloud_payload_fingerprint = self.map_cloud_payload_fingerprint
        self.cached_cloud_points = list(self.latest_map)
        self.cached_cloud_stamp_s = self.map_stamp_s
        self.cached_cloud_frame_id = self.map_cloud_frame_id
        # Incremental /mapData messages are not guaranteed to contain NodeData
        # for every optimized graph pose. Fetch the authoritative complete graph
        # before asking PublishMap to emit the matching graph/data/cloud triplet.
        self._request_full_optimized_graph()
        self.publish_map_attempts = 0
        self.publish_map_attempt_receipts = []
        max_attempts = int(getattr(self, "publish_map_max_attempts", 3))
        if max_attempts <= 0:
            raise RuntimeError("PublishMap retry count must be positive")
        overall_deadline = time.monotonic() + self.close_timeout_s

        for attempt in range(1, max_attempts + 1):
            now = time.monotonic()
            if now >= overall_deadline:
                break
            remaining_attempts = max_attempts - attempt + 1
            attempt_deadline = now + (overall_deadline - now) / remaining_attempts
            self.publish_map_attempts = attempt
            self.final_map_span = False
            subscription_generation, publisher_counts = self._rebind_map_subscriptions(attempt_deadline)
            self._reset_publish_attempt_state()
            # This process dispatches callbacks only through its direct single-threaded
            # spin_once calls. With no intervening spin, these counters are the exact
            # post-discovery/pre-request boundary for the new volatile subscriptions.
            self.map_messages_before_publish = self.map_messages
            self.map_cloud_callbacks_before_publish = self.map_cloud_callbacks
            self.map_graph_before_publish = self.map_graph_messages
            self.map_data_before_publish = self.map_data_messages
            attempt_receipt = {
                "attempt": attempt,
                "subscription_generation": subscription_generation,
                "subscriptions_matched": True,
                "matched_publisher_counts": dict(publisher_counts),
                "publisher_qos_premise": self.map_subscription_qos_premise,
                "subscription_qos": dict(self.map_subscription_qos_contract),
                "callback_executor_model": self.callback_executor_model,
                "subscriptions_retired": False,
                "subscription_retirement_error": None,
                "primary_failure": None,
                "acknowledged": False,
                "map_cloud_callbacks_before": self.map_cloud_callbacks_before_publish,
                "map_cloud_callbacks_after": self.map_cloud_callbacks,
                "map_messages_before": self.map_messages_before_publish,
                "map_messages_after": self.map_messages,
                "map_graph_messages_before": self.map_graph_before_publish,
                "map_graph_messages_after": self.map_graph_messages,
                "map_data_messages_before": self.map_data_before_publish,
                "map_data_messages_after": self.map_data_messages,
                "fresh_cloud": False,
                "fresh_map_graph": False,
                "fresh_map_data": False,
                "map_cloud_stamp_s": None,
                "map_graph_stamp_s": None,
                "map_data_stamp_s": None,
                "map_cloud_frame_id": None,
                "map_graph_frame_id": None,
                "map_data_frame_id": None,
                "graph_fingerprints": [],
                "cloud_payload_fingerprints": [],
                "source_graph_identity": None,
                "cloud_identity_state": None,
                "ignored_stale_callbacks_before": {
                    "map_cloud": self.ignored_stale_map_cloud_callbacks,
                    "map_graph": self.ignored_stale_map_graph_callbacks,
                    "map_data": self.ignored_stale_map_data_callbacks,
                },
                "ignored_stale_callbacks_after": {
                    "map_cloud": self.ignored_stale_map_cloud_callbacks,
                    "map_graph": self.ignored_stale_map_graph_callbacks,
                    "map_data": self.ignored_stale_map_data_callbacks,
                },
                "accepted": False,
            }
            self.publish_map_attempt_receipts.append(attempt_receipt)
            request = self.publish_map_request_factory()
            request.global_map = True
            request.optimized = True
            request.graph_only = False
            future = self.publish_map_client.call_async(request)
            response = self._wait_for_publish_map_response(
                future, attempt_deadline, attempt, max_attempts,
                subscription_generation, attempt_receipt,
            )
            # PublishMap.srv has an empty response. A completed, exception-free
            # future is its acknowledgement; the generated response has no success field.
            self.publish_map_acknowledged = True
            attempt_receipt["acknowledged"] = True

            attempt_graph_fingerprints: set[str] = set()
            attempt_cloud_payload_fingerprints: set[str] = set()
            quiet_since: float | None = None
            last_generation = self.map_publication_generation
            accepted = False
            while self.rclpy.ok():
                now = time.monotonic()
                fresh_cloud = (
                    self.map_cloud_callbacks > self.map_cloud_callbacks_before_publish
                    and self.map_messages > self.map_messages_before_publish
                )
                fresh_graph = self.map_graph_messages > self.map_graph_before_publish
                fresh_data = self.map_data_messages > self.map_data_before_publish
                attempt_receipt.update({
                    "map_cloud_callbacks_after": self.map_cloud_callbacks,
                    "map_messages_after": self.map_messages,
                    "map_graph_messages_after": self.map_graph_messages,
                    "map_data_messages_after": self.map_data_messages,
                    "fresh_cloud": fresh_cloud,
                    "fresh_map_graph": fresh_graph,
                    "fresh_map_data": fresh_data,
                    "map_cloud_stamp_s": self.map_stamp_s if fresh_cloud else None,
                    "map_graph_stamp_s": self.map_graph_stamp_s if fresh_graph else None,
                    "map_data_stamp_s": self.map_data_stamp_s if fresh_data else None,
                    "map_cloud_frame_id": self.map_cloud_frame_id if fresh_cloud else None,
                    "map_graph_frame_id": self.final_map_graph_frame_id if fresh_graph else None,
                    "map_data_frame_id": self.map_data_frame_id if fresh_data else None,
                    "ignored_stale_callbacks_after": {
                        "map_cloud": self.ignored_stale_map_cloud_callbacks,
                        "map_graph": self.ignored_stale_map_graph_callbacks,
                        "map_data": self.ignored_stale_map_data_callbacks,
                    },
                })
                data_fingerprint: str | None = None
                graph_fingerprint: str | None = None
                if fresh_graph:
                    graph_fingerprint = _map_graph_fingerprint(self.map_graph_message)
                    attempt_graph_fingerprints.add(graph_fingerprint)
                if fresh_data:
                    data_graph = getattr(self.map_data_message, "graph", None)
                    if data_graph is None:
                        raise RuntimeError("fresh post-request /mapData omitted its graph payload")
                    data_fingerprint = _map_graph_fingerprint(data_graph)
                    attempt_graph_fingerprints.add(data_fingerprint)
                if fresh_cloud:
                    cloud_payload_fingerprint = self.map_cloud_payload_fingerprint
                    if cloud_payload_fingerprint is None:
                        raise RuntimeError("fresh post-request cloud omitted its raw payload fingerprint")
                    attempt_cloud_payload_fingerprints.add(cloud_payload_fingerprint)
                attempt_receipt["graph_fingerprints"] = sorted(attempt_graph_fingerprints)
                attempt_receipt["cloud_payload_fingerprints"] = sorted(attempt_cloud_payload_fingerprints)

                fresh_graph_pair = fresh_graph and fresh_data
                cloud_cohort_ready = False
                cloud_span_stamp: float | None = None
                if fresh_graph_pair:
                    if not (
                        self.map_graph_stamp_s == self.map_data_stamp_s
                        and self.final_map_graph_frame_id == self.map_data_frame_id == "map"
                    ):
                        raise RuntimeError("fresh PublishMap /mapData and /mapGraph do not share one map-frame stamp")
                    if data_fingerprint != graph_fingerprint:
                        raise RuntimeError("fresh PublishMap /mapData and /mapGraph fingerprints differ")
                    source_receipt = _optimized_graph_receipt(self.map_data_message)
                    attempt_receipt["source_graph_identity"] = str(source_receipt["source_graph_identity"])
                    if str(source_receipt["source_graph_identity"]) != self.pre_publish_source_graph_identity:
                        raise RuntimeError("immutable RTAB-Map source graph changed between GetMap and PublishMap")
                    if fresh_cloud:
                        if not (
                            self.map_stamp_s == self.map_graph_stamp_s
                            and self.map_cloud_frame_id == "map"
                        ):
                            raise RuntimeError("fresh PublishMap cloud does not share the graph stamp and map frame")
                        cloud_cohort_ready = True
                        cloud_span_stamp = self.map_stamp_s
                        attempt_receipt["cloud_identity_state"] = "fresh_shared_publication"
                    elif (
                        self.cached_cloud_points is not None
                        and self.cached_cloud_stamp_s == self.map_graph_stamp_s
                        and self.cached_cloud_frame_id == "map"
                        and self.cached_cloud_graph_fingerprint == graph_fingerprint
                        and self.cached_cloud_payload_fingerprint is not None
                    ):
                        cloud_cohort_ready = True
                        cloud_span_stamp = self.cached_cloud_stamp_s
                        attempt_receipt["cloud_identity_state"] = "cached_exact_graph_reuse"
                    self.final_map_span = (
                        cloud_cohort_ready
                        and cloud_span_stamp is not None
                        and self.expected_sensor_last_stamp_s is not None
                        and cloud_span_stamp >= self.expected_sensor_last_stamp_s - self.sensor_scan_period_s - 1e-3
                    )
                if self.map_publication_generation != last_generation:
                    quiet_since = None
                    last_generation = self.map_publication_generation
                elif fresh_graph_pair and cloud_cohort_ready and self.final_map_span:
                    if quiet_since is None:
                        quiet_since = now
                    elif now - quiet_since >= 1.0:
                        accepted = True
                        attempt_receipt["accepted"] = True
                        break
                if now >= attempt_deadline:
                    break
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
            if not self.rclpy.ok():
                raise RuntimeError("ROS shut down while waiting for final map publication")

            self._retire_map_subscriptions(subscription_generation)
            attempt_receipt["subscriptions_retired"] = True
            if accepted:
                self.drain_complete = True
                self._capture_final_optimized_graph()
                return

        raise RuntimeError(
            f"final optimized map/graph/data triplet did not settle after "
            f"{self.publish_map_attempts}/{max_attempts} bounded PublishMap attempts"
        )

    def _capture_final_optimized_graph(self) -> None:
        data = self.map_data_message
        graph_message = self.map_graph_message
        if data is None or graph_message is None:
            raise RuntimeError("RTAB-Map did not publish post-request optimized graph and map data")
        if self.map_graph_stamp_s != self.map_data_stamp_s:
            raise RuntimeError("post-request /mapGraph and /mapData stamps do not identify one publication")
        if str(getattr(data.header, "frame_id", "")).lstrip("/") != "map" or str(getattr(graph_message.header, "frame_id", "")).lstrip("/") != "map":
            raise RuntimeError("post-request /mapGraph and /mapData must use the map frame")
        graph = getattr(data, "graph", None)
        if graph is None:
            raise RuntimeError("post-request /mapData omitted its graph payload")
        self.map_data_graph_fingerprint = _map_graph_fingerprint(graph)
        self.map_graph_fingerprint = _map_graph_fingerprint(graph_message)
        self.map_data_matches_map_graph = self.map_data_graph_fingerprint == self.map_graph_fingerprint
        if not self.map_data_matches_map_graph:
            raise RuntimeError("/mapGraph and /mapData graph payloads differ despite matching publication stamps")
        fresh_cloud = (
            self.map_cloud_callbacks > self.map_cloud_callbacks_before_publish
            and self.map_messages > self.map_messages_before_publish
        )
        if fresh_cloud:
            if (
                self.map_stamp_s != self.map_graph_stamp_s
                or self.map_cloud_frame_id != "map"
                or self.map_cloud_payload_fingerprint is None
            ):
                raise RuntimeError("fresh post-request cloud does not share the settled graph stamp and map frame")
            self.final_cloud_origin = "fresh_post_publish"
            self.map_cloud_identity_state = "fresh_shared_publication"
            self.final_cloud_payload_fingerprint = self.map_cloud_payload_fingerprint
        else:
            if (
                self.cached_cloud_points is None
                or self.cached_cloud_stamp_s != self.map_graph_stamp_s
                or self.cached_cloud_frame_id != "map"
                or self.cached_cloud_graph_fingerprint != self.map_graph_fingerprint
                or self.cached_cloud_payload_fingerprint is None
            ):
                raise RuntimeError("cached cloud does not exactly match the final graph fingerprint, stamp, and map frame")
            self.latest_map = list(self.cached_cloud_points)
            self.map_stamp_s = self.cached_cloud_stamp_s
            self.map_cloud_frame_id = self.cached_cloud_frame_id
            self.final_cloud_origin = "cached_pre_publish"
            self.map_cloud_identity_state = "cached_exact_graph_reuse"
            self.final_cloud_payload_fingerprint = self.cached_cloud_payload_fingerprint
        if not self.latest_map or self.map_stamp_s is None or not math.isfinite(float(self.map_stamp_s)) or any(not all(math.isfinite(float(value)) for value in point) for point in self.latest_map):
            raise RuntimeError("RTAB-Map final cloud is empty or contains non-finite points")
        self.final_cloud_graph_fingerprint = self.map_graph_fingerprint
        self.map_graph_matches_final_cloud = True
        graph_receipt = _optimized_graph_receipt(data)
        self.final_source_graph_identity = str(graph_receipt["source_graph_identity"])
        self.final_optimized_pose_version = str(graph_receipt["optimized_pose_version"])
        self.final_map_to_odom_version = str(graph_receipt["map_to_odom_version"])
        self.final_source_graph_link_count = int(graph_receipt["source_graph_link_count"])
        self.final_source_graph_link_type_histogram = dict(graph_receipt["source_graph_link_type_histogram"])
        self.source_graph_identity_matches_pre_publish = (
            self.pre_publish_source_graph_identity is not None
            and self.final_source_graph_identity == self.pre_publish_source_graph_identity
        )
        if not self.source_graph_identity_matches_pre_publish:
            raise RuntimeError("immutable RTAB-Map source graph changed between GetMap and PublishMap")
        self.optimized_graph_node_count = int(graph_receipt["count"])
        self.optimized_graph_stamp_sha256 = str(graph_receipt["stamp_sha256"])
        self.optimized_graph_neighbor_edge_count = int(graph_receipt["neighbor_edge_count"])
        self.optimized_graph_neighbor_chain_complete = bool(graph_receipt["neighbor_chain_complete"])
        self.optimized_graph_neighbor_merged_edge_count = int(graph_receipt["neighbor_merged_edge_count"])
        self.optimized_graph_node_ids_valid = bool(graph_receipt["positive_unique_node_ids"] and graph_receipt["pose_node_id_set_complete"])
        expected_count = getattr(self, "expected_lidar_scan_count", None)
        expected_stamp_sha256 = getattr(self, "expected_lidar_stamp_sha256", None)
        if expected_count is not None or expected_stamp_sha256 is not None:
            if expected_count is None or expected_stamp_sha256 is None:
                raise RuntimeError("captured LiDAR graph coverage expectation is incompletely configured")
            _require_exact_sequence(
                graph_receipt,
                int(expected_count),
                str(expected_stamp_sha256),
                "final published optimized graph",
            )
            _require_neighbor_chain(graph_receipt, "final published optimized graph")
        ids = [] if graph is None else list(getattr(graph, "poses_id", []))
        poses = [] if graph is None else list(getattr(graph, "poses", []))
        nodes = list(getattr(data, "nodes", []))
        if not ids or len(ids) != len(poses):
            raise RuntimeError("RTAB-Map published an empty or inconsistent optimized pose graph")
        stamp_by_id = {int(node.id): _stamp(node.stamp) for node in nodes}
        if len(stamp_by_id) != len(nodes):
            raise RuntimeError("RTAB-Map published duplicate node identities")
        graph_map_to_odom = getattr(graph, "map_to_odom", None)
        if graph_map_to_odom is None:
            raise RuntimeError("RTAB-Map graph omitted its map-to-odom transform")
        map_translation = graph_map_to_odom.translation
        map_rotation = graph_map_to_odom.rotation
        map_q = _quat_normalize((float(map_rotation.x), float(map_rotation.y), float(map_rotation.z), float(map_rotation.w)))
        map_to_odom_record = {
            "x_m": float(map_translation.x), "y_m": float(map_translation.y), "z_m": float(map_translation.z),
            "qx": map_q[0], "qy": map_q[1], "qz": map_q[2], "qw": map_q[3],
        }
        if not all(math.isfinite(float(value)) for value in map_to_odom_record.values()):
            raise RuntimeError("RTAB-Map graph returned a non-finite map-to-odom transform")
        optimized_rows: list[dict[str, object]] = []
        optimized_keyframes: list[dict[str, object]] = []
        correction_rows: list[dict[str, object]] = []
        nodes_by_id = {int(node.id): node for node in nodes}
        for node_id, pose in zip(ids, poses):
            key = int(node_id)
            if key not in stamp_by_id:
                raise RuntimeError(f"RTAB-Map optimized graph pose has no timestamped node: {key}")
            node = nodes_by_id[key]
            stamp = stamp_by_id[key]
            position = pose.position
            orientation = pose.orientation
            quaternion = _quat_normalize((float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)))
            values = (stamp, float(position.x), float(position.y), float(position.z), *quaternion)
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError(f"RTAB-Map optimized graph contains non-finite pose data for node {key}")
            row = {
                "timestamp_s": stamp,
                "x_m": float(position.x), "y_m": float(position.y), "z_m": float(position.z),
                "qx": quaternion[0], "qy": quaternion[1], "qz": quaternion[2], "qw": quaternion[3],
                "frame_id": "map",
            }
            optimized_rows.append(row)
            optimized_keyframes.append({"node_id": key, **row})
            node_pose = node.pose
            node_position = node_pose.position
            node_orientation = node_pose.orientation
            raw_node_pose = {
                "timestamp_s": stamp,
                "x_m": float(node_position.x), "y_m": float(node_position.y), "z_m": float(node_position.z),
                "qx": float(node_orientation.x), "qy": float(node_orientation.y), "qz": float(node_orientation.z), "qw": float(node_orientation.w),
            }
            correction = _map_to_odom_from_node(raw_node_pose, {**row, "timestamp_s": stamp})
            correction_rows.append(correction)
            optimized_keyframes[-1].update({
                "odom_x_m": raw_node_pose["x_m"], "odom_y_m": raw_node_pose["y_m"], "odom_z_m": raw_node_pose["z_m"],
                "odom_qx": raw_node_pose["qx"], "odom_qy": raw_node_pose["qy"], "odom_qz": raw_node_pose["qz"], "odom_qw": raw_node_pose["qw"],
                "correction_x_m": correction["x_m"], "correction_y_m": correction["y_m"], "correction_z_m": correction["z_m"],
                "correction_qx": correction["qx"], "correction_qy": correction["qy"], "correction_qz": correction["qz"], "correction_qw": correction["qw"],
            })
        optimized_rows.sort(key=lambda row: float(row["timestamp_s"]))
        optimized_keyframes.sort(key=lambda row: float(row["timestamp_s"]))
        correction_rows.sort(key=lambda row: float(row["timestamp_s"]))
        if len({float(row["timestamp_s"]) for row in optimized_rows}) != len(optimized_rows):
            raise RuntimeError("RTAB-Map optimized graph has duplicate node timestamps")
        dense_map_rows: list[dict[str, object]] = []
        ordered_odom_rows = sorted(self.odom_rows, key=lambda row: float(row["timestamp_s"]))
        odom_stamps = [float(row["timestamp_s"]) for row in ordered_odom_rows]
        if len(odom_stamps) != len(set(odom_stamps)):
            raise RuntimeError("raw odometry contains duplicate timestamps")
        for odom in ordered_odom_rows:
            correction = _interpolate_map_to_odom(correction_rows, float(odom["timestamp_s"]))
            if correction is None:
                raise RuntimeError("final optimized graph does not bracket every raw odometry timestamp")
            dense_map_rows.append(_correct_odom_pose(odom, correction))
        if not dense_map_rows:
            raise RuntimeError("final optimized graph cannot export an empty corrected odometry stream")
        self.map_pose_rows = dense_map_rows
        self.optimized_keyframe_rows = optimized_keyframes
        self.graph_pose_version = str(graph_receipt["version"])
        from simulator.capture.manifest import sha256_json
        self.dense_pose_version = sha256_json({
            "graph_pose_version": self.graph_pose_version,
            "raw_odom_poses": [{**row, "frame_id": "odom"} for row in ordered_odom_rows],
            "dense_map_poses": dense_map_rows,
            "correction_policy": "derive optimized_node_pose * inverse(raw_node_odom_pose); linear translation + quaternion slerp between node timestamps; no extrapolation",
        })
        self.map_version = sha256_json({
            "graph_pose_version": self.graph_pose_version,
            "dense_pose_version": self.dense_pose_version,
            "cloud_frame_id": self.map_cloud_frame_id,
            "cloud_points": self.latest_map,
        })
        self.final_map_graph_stamp_s = self.map_graph_stamp_s
        self.final_cloud_stamp_s = self.map_stamp_s
        self.final_map_graph_frame_id = str(getattr(graph_message.header, "frame_id", "")).lstrip("/")
        self.optimized_graph_last_stamp_s = float(graph_receipt["last_stamp_s"])
        self.optimized_pose_graph_complete = (
            self.expected_sensor_last_stamp_s is not None
            and self.optimized_graph_last_stamp_s >= self.expected_sensor_last_stamp_s - self.sensor_scan_period_s - 1e-3
        )
        if not self.optimized_pose_graph_complete:
            raise RuntimeError("RTAB-Map optimized graph does not cover the captured sensor input span")

    def spin_until_done(self) -> None:
        if self.replay_complete_signal is None:
            raise RuntimeError("offline SLAM observer requires a replay-complete signal path")
        signal_seen_wall: float | None = None
        quiet_since: float | None = None
        last_generation = self.replay_input_generation
        wall_deadline = self.started_wall + max(60.0, self.replay_span_s * 20.0 + 30.0)
        while self.rclpy.ok():
            now = time.monotonic()
            if self.replay_complete_signal.is_file():
                if signal_seen_wall is None:
                    signal_seen_wall = now
                    self.replay_complete_signal_observed = True
                if self.replay_input_generation == last_generation:
                    if quiet_since is None:
                        quiet_since = now
                    self.drain_quiet_polls += 1
                else:
                    quiet_since = None
                    self.drain_quiet_polls = 0
                    last_generation = self.replay_input_generation
                if now - signal_seen_wall >= 60.0:
                    raise RuntimeError("offline replay callbacks did not settle within the bounded wait")
                if quiet_since is not None and now - quiet_since >= 1.0:
                    if not self.clock_start_covered:
                        raise RuntimeError(
                            f"bag replay observer missed the beginning of /clock: first {self.first_clock_s}, "
                            f"expected no later than {self.expected_first_clock_s + self.clock_start_tolerance_s}"
                        )
                    if not self.clock_target_reached:
                        raise RuntimeError(
                            f"bag replay exited before /clock reached target {self.target_clock_s}; last clock was {self.last_clock_s}"
                        )
                    if self.clock_regressions:
                        raise RuntimeError(f"bag replay /clock moved backwards {self.clock_regressions} time(s)")
                    if (
                        self.expected_sensor_last_stamp_s is None
                        or self.last_odom_stamp_s is None
                        or self.last_odom_stamp_s < self.expected_sensor_last_stamp_s - self.sensor_scan_period_s - 1e-3
                    ):
                        raise RuntimeError(
                            f"SLAM odometry did not process the captured sensor span: "
                            f"last {self.last_odom_stamp_s}, expected {self.expected_sensor_last_stamp_s}"
                        )
                    self._validate_odom_input_coverage()
                    self.replay_drained = True
                    self._publish_final_map()
                    return
            if self.first_clock_s is None and now - self.started_wall >= self.startup_timeout_s:
                raise RuntimeError("offline SLAM observer did not observe /clock")
            if now >= wall_deadline:
                raise RuntimeError("offline SLAM replay did not complete within the bounded wall-time guard")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> dict[str, object]:
        self._retire_deferred_map_subscriptions()
        odom_fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "quaternion_valid", "frame_id"]
        raw_rows = [{**row, "frame_id": "odom"} for row in sorted(self.odom_rows, key=lambda row: float(row["timestamp_s"]))]
        with (self.output_dir / "slam_odom_poses.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=odom_fields)
            writer.writeheader()
            writer.writerows(raw_rows)

        transform_fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "parent_frame_id", "child_frame_id"]
        with (self.output_dir / "map_to_odom.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=transform_fields)
            writer.writeheader()
            writer.writerows(sorted(self.map_to_odom_rows, key=lambda row: float(row["timestamp_s"])))

        map_fields = ["timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "frame_id"]
        for filename in ("slam_map_poses.csv", "slam_poses.csv"):
            with (self.output_dir / filename).open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=map_fields)
                writer.writeheader()
                writer.writerows(sorted(self.map_pose_rows, key=lambda row: float(row["timestamp_s"])))

        keyframe_fields = [
            "node_id", "timestamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "frame_id",
            "odom_x_m", "odom_y_m", "odom_z_m", "odom_qx", "odom_qy", "odom_qz", "odom_qw",
            "correction_x_m", "correction_y_m", "correction_z_m", "correction_qx", "correction_qy", "correction_qz", "correction_qw",
        ]
        with (self.output_dir / "slam_map_keyframes.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=keyframe_fields)
            writer.writeheader()
            writer.writerows(getattr(self, "optimized_keyframe_rows", []))

        _write_pcd(self.output_dir / "slam_map.pcd", self.latest_map)
        _write_ply(self.output_dir / "slam_map.ply", self.latest_map)
        map_pose_correction_complete = bool(self.map_pose_rows) and self.optimized_pose_graph_complete
        files = []
        map_version = getattr(self, "map_version", None) if self.map_graph_matches_final_cloud else None
        metadata = {
            "slam_map_poses.csv": ("dense_corrected_trajectory", "map", True),
            "slam_map_keyframes.csv": ("optimized_graph_keyframes", "map", True),
            "slam_poses.csv": ("legacy_map_trajectory", "map", True),
            "slam_odom_poses.csv": ("raw_odometry_diagnostic", "odom", False),
            "map_to_odom.csv": ("incremental_tf_diagnostic", "map->odom", False),
            "slam_map.pcd": ("final_optimized_cloud", "map", True),
            "slam_map.ply": ("final_optimized_cloud", "map", True),
        }
        for name in metadata:
            path = self.output_dir / name
            role, frame_id, optimized = metadata[name]
            files.append({
                "path": name,
                "role": role,
                "size_bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "frame_id": frame_id,
                "optimized": optimized,
                "map_version": map_version if optimized else None,
                "dense_pose_version": getattr(self, "dense_pose_version", None) if name == "slam_map_poses.csv" else None,
                "schema_version": 2 if name == "slam_map_keyframes.csv" else 1,
                "correction_policy": "derive optimized_node_pose * inverse(raw_node_odom_pose); linear translation + quaternion slerp between node timestamps; no extrapolation" if name in ("slam_map_poses.csv", "slam_map_keyframes.csv") else None,
            })
        processed_sensor_span = (
            self.expected_sensor_last_stamp_s is not None
            and self.last_odom_stamp_s is not None
            and self.last_odom_stamp_s >= self.expected_sensor_last_stamp_s - self.sensor_scan_period_s - 1e-3
        )
        live_complete = bool(
            self.odom_rows and self.latest_map and self.replay_complete_signal_observed
            and self.clock_start_covered and self.clock_target_reached and self.replay_drained
            and self.publish_map_acknowledged and self.drain_complete and not self.clock_regressions
            and processed_sensor_span and self.final_map_span and map_pose_correction_complete
            and self.map_graph_matches_final_cloud
            and getattr(self, "map_data_matches_map_graph", False)
            and getattr(self, "source_graph_identity_matches_pre_publish", False)
            and getattr(self, "odom_input_coverage_complete", False)
        )
        result = {
            "status": "pending_database_validation" if live_complete else "incomplete",
            "odom_sample_count": len(self.odom_rows),
            "map_pose_frame_id": "map",
            "raw_odom_frame_id": "odom",
            "map_pose_sample_count": len(self.map_pose_rows),
            "map_pose_correction_complete": map_pose_correction_complete,
            "optimized_pose_graph_complete": self.optimized_pose_graph_complete,
            "map_graph_matches_final_cloud": self.map_graph_matches_final_cloud,
            "pre_publish_graph_version": getattr(self, "pre_publish_graph_version", None),
            "pre_publish_graph_version_source": getattr(self, "pre_publish_graph_version_source", None),
            "pre_publish_source_graph_identity": getattr(self, "pre_publish_source_graph_identity", None),
            "final_source_graph_identity": getattr(self, "final_source_graph_identity", None),
            "source_graph_identity_matches_pre_publish": getattr(self, "source_graph_identity_matches_pre_publish", False),
            "pre_publish_optimized_pose_version": getattr(self, "pre_publish_optimized_pose_version", None),
            "final_optimized_pose_version": getattr(self, "final_optimized_pose_version", None),
            "pre_publish_map_to_odom_version": getattr(self, "pre_publish_map_to_odom_version", None),
            "final_map_to_odom_version": getattr(self, "final_map_to_odom_version", None),
            "pre_publish_source_graph_link_count": getattr(self, "pre_publish_source_graph_link_count", None),
            "final_source_graph_link_count": getattr(self, "final_source_graph_link_count", None),
            "pre_publish_source_graph_link_type_histogram": getattr(self, "pre_publish_source_graph_link_type_histogram", None),
            "final_source_graph_link_type_histogram": getattr(self, "final_source_graph_link_type_histogram", None),
            "expected_lidar_scan_count": getattr(self, "expected_lidar_scan_count", None),
            "expected_lidar_stamp_sha256": getattr(self, "expected_lidar_stamp_sha256", None),
            "odom_stamp_sha256": getattr(self, "odom_stamp_sha256", None),
            "odom_input_coverage_complete": getattr(self, "odom_input_coverage_complete", False),
            "pre_publish_graph_node_count": getattr(self, "pre_publish_graph_node_count", None),
            "pre_publish_graph_stamp_sha256": getattr(self, "pre_publish_graph_stamp_sha256", None),
            "pre_publish_neighbor_edge_count": getattr(self, "pre_publish_neighbor_edge_count", None),
            "pre_publish_expected_neighbor_edge_count": None if getattr(self, "pre_publish_graph_node_count", None) is None else max(0, int(self.pre_publish_graph_node_count) - 1),
            "pre_publish_neighbor_merged_edge_count": getattr(self, "pre_publish_neighbor_merged_edge_count", None),
            "pre_publish_neighbor_chain_complete": getattr(self, "pre_publish_neighbor_chain_complete", False),
            "pre_publish_graph_node_ids_valid": getattr(self, "pre_publish_graph_node_ids_valid", False),
            "pose_source": "rtabmap_optimized_graph",
            "graph_pose_version": self.graph_pose_version,
            "optimized_graph_node_count": getattr(self, "optimized_graph_node_count", None),
            "optimized_graph_stamp_sha256": getattr(self, "optimized_graph_stamp_sha256", None),
            "optimized_graph_neighbor_edge_count": getattr(self, "optimized_graph_neighbor_edge_count", None),
            "optimized_graph_expected_neighbor_edge_count": None if getattr(self, "optimized_graph_node_count", None) is None else max(0, int(self.optimized_graph_node_count) - 1),
            "optimized_graph_neighbor_merged_edge_count": getattr(self, "optimized_graph_neighbor_merged_edge_count", None),
            "optimized_graph_neighbor_chain_complete": getattr(self, "optimized_graph_neighbor_chain_complete", False),
            "optimized_graph_node_ids_valid": getattr(self, "optimized_graph_node_ids_valid", False),
            "dense_pose_version": getattr(self, "dense_pose_version", None),
            "map_version": map_version,
            "map_data_graph_fingerprint": getattr(self, "map_data_graph_fingerprint", None),
            "map_graph_fingerprint": getattr(self, "map_graph_fingerprint", None),
            "map_data_matches_map_graph": getattr(self, "map_data_matches_map_graph", False),
            "cached_cloud_graph_fingerprint": getattr(self, "cached_cloud_graph_fingerprint", None),
            "cached_cloud_payload_fingerprint": getattr(self, "cached_cloud_payload_fingerprint", None),
            "final_cloud_graph_fingerprint": getattr(self, "final_cloud_graph_fingerprint", None),
            "final_cloud_payload_fingerprint": getattr(self, "final_cloud_payload_fingerprint", None),
            "map_cloud_identity_state": getattr(self, "map_cloud_identity_state", None),
            "final_cloud_origin": getattr(self, "final_cloud_origin", None),
            "final_map_graph_stamp_s": getattr(self, "final_map_graph_stamp_s", None),
            "final_cloud_stamp_s": getattr(self, "final_cloud_stamp_s", None),
            "final_map_graph_frame_id": getattr(self, "final_map_graph_frame_id", None),
            "final_cloud_frame_id": getattr(self, "map_cloud_frame_id", None),
            "optimized_graph_last_stamp_s": self.optimized_graph_last_stamp_s,
            "map_to_odom_sample_count": len(self.map_to_odom_rows),
            "files": files,
            "map_message_count": self.map_messages,
            "map_cloud_callback_count": getattr(self, "map_cloud_callbacks", self.map_messages),
            "map_cloud_callbacks_before_publish": getattr(self, "map_cloud_callbacks_before_publish", self.map_messages_before_publish),
            "map_graph_message_count": self.map_graph_messages,
            "map_graph_messages_before_publish": self.map_graph_before_publish,
            "map_data_message_count": self.map_data_messages,
            "map_data_messages_before_publish": self.map_data_before_publish,
            "map_point_count": len(self.latest_map),
            "first_clock_s": self.first_clock_s,
            "last_clock_s": self.last_clock_s,
            "target_clock_s": self.target_clock_s,
            "replay_span_s": self.replay_span_s,
            "expected_first_clock_s": self.expected_first_clock_s,
            "clock_start_tolerance_s": self.clock_start_tolerance_s,
            "clock_start_covered": self.clock_start_covered,
            "clock_target_reached": self.clock_target_reached,
            "clock_regressions": self.clock_regressions,
            "replay_complete_signal_observed": self.replay_complete_signal_observed,
            "drain_complete": self.drain_complete,
            "drain_quiet_polls": self.drain_quiet_polls,
            "callback_generation": self.callback_generation,
            "replay_input_generation": self.replay_input_generation,
            "replay_drain_basis": ["/clock", "/slam/odom"],
            "map_publication_generation": self.map_publication_generation,
            "post_publish_drain_basis": ["/slam/map_cloud", "/mapGraph", "/mapData"],
            "map_subscription_generation": getattr(self, "map_subscription_generation", 0),
            "active_map_subscription_generation": getattr(self, "active_map_subscription_generation", None),
            "map_subscription_rebinds": getattr(self, "map_subscription_rebinds", 0),
            "retired_map_subscription_generations": getattr(self, "retired_map_subscription_generations", []),
            "map_subscription_publisher_counts": getattr(self, "map_subscription_publisher_counts", {}),
            "map_subscription_qos_premise": getattr(self, "map_subscription_qos_premise", None),
            "map_subscription_qos_contract": getattr(self, "map_subscription_qos_contract", {}),
            "callback_executor_model": getattr(self, "callback_executor_model", None),
            "ignored_stale_map_cloud_callbacks": getattr(self, "ignored_stale_map_cloud_callbacks", 0),
            "ignored_stale_map_graph_callbacks": getattr(self, "ignored_stale_map_graph_callbacks", 0),
            "ignored_stale_map_data_callbacks": getattr(self, "ignored_stale_map_data_callbacks", 0),
            "replay_drained": self.replay_drained,
            "publish_map_acknowledged": self.publish_map_acknowledged,
            "publish_map_attempts": getattr(self, "publish_map_attempts", 0),
            "publish_map_max_attempts": getattr(self, "publish_map_max_attempts", 3),
            "publish_map_attempt_receipts": getattr(self, "publish_map_attempt_receipts", []),
            "map_messages_before_publish": self.map_messages_before_publish,
            "final_map_span": self.final_map_span,
            "mapper_database_span": self.mapper_database_span,
            "database_node_count": self.database_node_count,
            "database_node_stamp_sha256": getattr(self, "database_node_stamp_sha256", None),
            "database_node_timestamps_strictly_increasing": getattr(
                self, "database_node_timestamps_strictly_increasing", None
            ),
            "database_scan_coverage_complete": getattr(
                self, "database_scan_coverage_complete", False
            ),
            "database_last_stamp_s": self.database_last_stamp_s,
            "database_verification_stage": self.database_verification_stage,
            "last_map_stamp_s": self.map_stamp_s,
            "last_odom_stamp_s": self.last_odom_stamp_s,
            "expected_sensor_last_stamp_s": self.expected_sensor_last_stamp_s,
            "sensor_scan_period_s": self.sensor_scan_period_s,
            "processed_sensor_span": processed_sensor_span,
            "fresh_odom": bool(self.odom_rows),
            "fresh_map": bool(self.latest_map),
            "ground_truth_subscribed": False,
        }
        (self.output_dir / "slam_observer.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.node.destroy_node()
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--target-clock-seconds", required=True, type=float)
    parser.add_argument("--expected-first-clock-seconds", required=True, type=float)
    parser.add_argument("--clock-start-tolerance-seconds", required=True, type=float)
    parser.add_argument("--startup-timeout-seconds", type=float, default=180.0)
    parser.add_argument("--replay-complete-signal", required=True)
    parser.add_argument("--expected-sensor-last-stamp-seconds", required=True, type=float)
    parser.add_argument("--sensor-scan-period-seconds", required=True, type=float)
    parser.add_argument("--expected-lidar-scan-count", required=True, type=int)
    parser.add_argument("--expected-lidar-stamp-sha256", required=True)
    args = parser.parse_args()
    import rclpy

    rclpy.init()
    observer = SlamObserver(
        Path(args.output_dir),
        args.target_clock_seconds,
        args.startup_timeout_seconds,
        args.expected_first_clock_seconds,
        args.clock_start_tolerance_seconds,
        args.expected_lidar_scan_count,
        args.expected_lidar_stamp_sha256,
    )
    observer.replay_complete_signal = Path(args.replay_complete_signal)
    observer.expected_sensor_last_stamp_s = args.expected_sensor_last_stamp_seconds
    observer.sensor_scan_period_s = args.sensor_scan_period_seconds
    error: BaseException | None = None
    result: dict[str, object] | None = None
    try:
        observer.spin_until_done()
    except BaseException as exc:
        error = exc
    finally:
        try:
            result = observer.close()
        except BaseException as exc:
            if error is None:
                error = exc
            elif hasattr(error, "add_note"):
                error.add_note(f"Offline observer close also failed: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
    if result is not None:
        print(json.dumps(result, indent=2))
    if error is not None:
        raise error
    if result is None or result["status"] != "pending_database_validation":
        raise RuntimeError(f"offline SLAM did not complete replay, clock drain, odometry and map capture: {result}")


if __name__ == "__main__":
    main()
