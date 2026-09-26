#!/usr/bin/env python3
"""Validate structure, resources, kinematics, and collision meshes for the spike."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _values(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    return np.asarray([float(value) for value in text.split()] if text else default, dtype=float)


def _rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def _origin(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, :3] = _rpy(_values(element.attrib.get("rpy"), (0, 0, 0)))
    transform[:3, 3] = _values(element.attrib.get("xyz"), (0, 0, 0))
    return transform


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    cross = 1 - c
    rotation = np.array(
        (
            (c + x * x * cross, x * y * cross - z * s, x * z * cross + y * s),
            (y * x * cross + z * s, c + y * y * cross, y * z * cross - x * s),
            (z * x * cross - y * s, z * y * cross + x * s, c + z * z * cross),
        ),
        dtype=float,
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    return transform


def _fk(
    root_name: str,
    children: dict[str, list[ET.Element]],
    joint_positions: dict[str, float],
) -> dict[str, np.ndarray]:
    transforms = {root_name: np.eye(4)}
    pending = [root_name]
    while pending:
        parent = pending.pop()
        for joint in children.get(parent, []):
            child = joint.find("child").attrib["link"]
            transform = transforms[parent] @ _origin(joint.find("origin"))
            if joint.attrib["type"] in {"revolute", "continuous"}:
                axis_element = joint.find("axis")
                axis = _values(
                    axis_element.attrib.get("xyz") if axis_element is not None else None,
                    (1, 0, 0),
                )
                transform = transform @ _axis_angle(axis, joint_positions.get(joint.attrib["name"], 0.0))
            transforms[child] = transform
            pending.append(child)
    return transforms


def _pose_record(transform: np.ndarray) -> dict[str, object]:
    return {
        "position_m": transform[:3, 3].round(9).tolist(),
        "rotation_matrix": transform[:3, :3].round(9).tolist(),
    }


def validate(root: Path) -> dict[str, object]:
    urdf_path = root / "asimov_orcahand_right.urdf"
    manifest_path = root / "upstream_manifest.json"
    robot = ET.parse(urdf_path).getroot()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    links = robot.findall("link")
    joints = robot.findall("joint")
    link_names = [link.attrib["name"] for link in links]
    joint_names = [joint.attrib["name"] for joint in joints]
    assert len(link_names) == len(set(link_names)), "duplicate links"
    assert len(joint_names) == len(set(joint_names)), "duplicate joints"
    assert "world" not in link_names
    assert "world2right_tower_fixed_jointbody" not in link_names
    assert "world2right_tower_fixed" not in joint_names
    assert "world2right_tower_fixed_offset" not in joint_names

    by_child: dict[str, ET.Element] = {}
    children: dict[str, list[ET.Element]] = {}
    adjacent_pairs: set[frozenset[str]] = set()
    for joint in joints:
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        assert parent in link_names and child in link_names
        assert child not in by_child, f"multiple parents for {child}"
        by_child[child] = joint
        children.setdefault(parent, []).append(joint)
        adjacent_pairs.add(frozenset((parent, child)))
    roots = sorted(set(link_names) - set(by_child))
    assert roots == ["pelvis_link"], roots

    neutral = _fk(roots[0], children, {})
    assert set(neutral) == set(link_names), "disconnected kinematic graph"

    required_links = {
        "right_wrist_yaw_link",
        "right_tower",
        "right_wrist_jointbody",
        "right_palm",
        "right_thumb_fingertip",
        "right_index_fingertip",
        "right_middle_fingertip",
        "right_ring_fingertip",
        "right_pinky_fingertip",
    }
    assert required_links <= set(link_names)
    mount = next(joint for joint in joints if joint.attrib["name"] == "asimov_to_orcahand_mount")
    assert mount.attrib["type"] == "fixed"
    assert mount.find("parent").attrib["link"] == "right_wrist_yaw_link"
    assert mount.find("child").attrib["link"] == "right_tower"

    mesh_files: list[Path] = []
    for mesh in robot.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        assert not filename.startswith("package://"), filename
        path = (root / filename).resolve()
        assert path.is_relative_to(root.resolve()), filename
        assert path.is_file(), filename
        mesh_files.append(path)

    manifest_assets = {record["derived_path"]: record for record in manifest["copied_assets"]}
    for relative, record in manifest_assets.items():
        path = root / relative
        assert path.stat().st_size == record["size_bytes"], relative
        assert _sha256(path) == record["sha256"], relative
    assert _sha256(urdf_path) == manifest["derivation"]["output_urdf_sha256"]

    collision_meshes: list[dict[str, object]] = []
    collision_aabbs: list[tuple[str, str, np.ndarray, np.ndarray]] = []
    for link in links:
        link_name = link.attrib["name"]
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            local = _origin(collision.find("origin"))
            mesh_element = geometry.find("mesh")
            if mesh_element is not None:
                path = root / mesh_element.attrib["filename"]
                mesh = trimesh.load_mesh(path, process=False)
                assert len(mesh.vertices) > 0 and np.isfinite(mesh.vertices).all()
                points = trimesh.transform_points(mesh.vertices, neutral[link_name] @ local)
                bounds = np.vstack((points.min(axis=0), points.max(axis=0)))
                extents = mesh.extents
                assert float(extents.max()) < 0.5, f"scale explosion in {path}"
                collision_meshes.append(
                    {
                        "link": link_name,
                        "path": path.relative_to(root).as_posix(),
                        "vertices": int(len(mesh.vertices)),
                        "extents_m": np.asarray(extents).round(9).tolist(),
                    }
                )
            else:
                cylinder = geometry.find("cylinder")
                sphere = geometry.find("sphere")
                box = geometry.find("box")
                if cylinder is not None:
                    radius = float(cylinder.attrib["radius"])
                    half = float(cylinder.attrib["length"]) / 2.0
                    half_extents = (radius, radius, half)
                elif sphere is not None:
                    radius = float(sphere.attrib["radius"])
                    half_extents = (radius, radius, radius)
                elif box is not None:
                    half_extents = tuple(value / 2.0 for value in _values(box.attrib["size"], (0, 0, 0)))
                else:
                    raise AssertionError(f"unsupported collision geometry on {link_name}")
                corners = np.array(
                    [
                        [x, y, z]
                        for x in (-half_extents[0], half_extents[0])
                        for y in (-half_extents[1], half_extents[1])
                        for z in (-half_extents[2], half_extents[2])
                    ]
                )
                points = trimesh.transform_points(corners, neutral[link_name] @ local)
                bounds = np.vstack((points.min(axis=0), points.max(axis=0)))
            collision_aabbs.append((link_name, collision.attrib.get("name", ""), bounds[0], bounds[1]))

    potential_nonadjacent_overlaps: list[dict[str, object]] = []
    for index, first in enumerate(collision_aabbs):
        for second in collision_aabbs[index + 1 :]:
            if first[0] == second[0] or frozenset((first[0], second[0])) in adjacent_pairs:
                continue
            overlap = np.minimum(first[3], second[3]) - np.maximum(first[2], second[2])
            if np.all(overlap > 0):
                potential_nonadjacent_overlaps.append(
                    {
                        "first": f"{first[0]}/{first[1]}",
                        "second": f"{second[0]}/{second[1]}",
                        "aabb_overlap_m": overlap.round(9).tolist(),
                    }
                )

    moved_asimov = _fk(roots[0], children, {"right_wrist_yaw_joint": 0.25})
    moved_orca = _fk(
        roots[0], children, {"right_wrist_yaw_joint": 0.25, "right_wrist": 0.30}
    )
    moved_fingers = _fk(
        roots[0],
        children,
        {
            "right_wrist_yaw_joint": 0.25,
            "right_wrist": 0.30,
            "right_thumb_mcp": 0.25,
            "right_thumb_abd": -0.40,
            "right_thumb_pip": 0.60,
            "right_thumb_dip": 0.50,
            "right_index_mcp": 0.70,
            "right_index_pip": 0.80,
            "right_middle_mcp": 0.60,
            "right_middle_pip": 0.75,
            "right_ring_mcp": 0.55,
            "right_ring_pip": 0.70,
            "right_pinky_mcp": 0.50,
            "right_pinky_pip": 0.65,
        },
    )
    tower_rotation_delta = float(
        np.linalg.norm(moved_asimov["right_tower"][:3, :3] - neutral["right_tower"][:3, :3])
    )
    palm_rotation_delta = float(
        np.linalg.norm(moved_orca["right_palm"][:3, :3] - moved_asimov["right_palm"][:3, :3])
    )
    fingertip_displacements = {
        link: float(np.linalg.norm(moved_fingers[link][:3, 3] - moved_orca[link][:3, 3]))
        for link in sorted(required_links)
        if link.endswith("fingertip")
    }
    assert tower_rotation_delta > 0.1
    assert palm_rotation_delta > 0.1
    assert min(fingertip_displacements.values()) > 0.005

    return {
        "status": "pass",
        "urdf": urdf_path.name,
        "urdf_sha256": _sha256(urdf_path),
        "root_link": roots[0],
        "link_count": len(links),
        "joint_count": len(joints),
        "revolute_joint_count": sum(joint.attrib["type"] == "revolute" for joint in joints),
        "fixed_joint_count": sum(joint.attrib["type"] == "fixed" for joint in joints),
        "mesh_reference_count": len(mesh_files),
        "unique_mesh_file_count": len(set(mesh_files)),
        "collision_mesh_count": len(collision_meshes),
        "collision_meshes": collision_meshes,
        "potential_nonadjacent_collision_aabb_overlaps": potential_nonadjacent_overlaps,
        "kinematic_motion": {
            "right_wrist_yaw_rad": 0.25,
            "orca_wrist_rad": 0.30,
            "tower_rotation_matrix_delta_norm": tower_rotation_delta,
            "palm_rotation_matrix_delta_norm": palm_rotation_delta,
            "fingertip_displacement_m": fingertip_displacements,
            "neutral_tower_pose": _pose_record(neutral["right_tower"]),
            "moved_tower_pose": _pose_record(moved_asimov["right_tower"]),
            "neutral_palm_pose": _pose_record(neutral["right_palm"]),
            "moved_palm_pose": _pose_record(moved_orca["right_palm"]),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--evidence", type=Path, default=None)
    args = parser.parse_args()
    root = args.root.resolve()
    result = validate(root)
    evidence = args.evidence or (root / "evidence/cpu_model_validation.json")
    evidence.parent.mkdir(parents=True, exist_ok=True)
    evidence.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    summary = {key: value for key, value in result.items() if key != "collision_meshes"}
    print(json.dumps(summary, indent=2))
    print(f"wrote {evidence}")


if __name__ == "__main__":
    main()
