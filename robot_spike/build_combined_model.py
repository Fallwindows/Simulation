#!/usr/bin/env python3
"""Build the pinned Asimov 1 + OrcaHand V1 right-hand spike model.

The upstream checkouts are read-only inputs. This script copies only the
referenced simulation meshes and license notices into ``robot_spike`` and
authors a derived URDF with local relative mesh paths.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


ASIMOV_COMMIT = "ccf5326f0adffd65edf930cbbac052454cdd82be"
ASIMOV_TREE = "dccddbbd164db4e516ec655597cebd5e9b6d482e"
ORCA_COMMIT = "b9b349a21ee0238c62b6cf92ae7597027867adf8"
ORCA_TREE = "694a70bec9d960eecb05ae5b6746259b2e43d6f6"

# The Asimov wrist-yaw axis is (sin(130 deg), 0, cos(130 deg)). Aligning
# Orca's +Z forearm/tower direction to that axis continues the arm. The 60 mm
# translation clears the approximately 40 mm Asimov wrist body while the
# adapter collar bridges back to it.
MOUNT_XYZ = (0.0459626666, 0.0, -0.0385672566)
MOUNT_RPY = (0.0, 2.2689280276, 0.0)
ADAPTER_RADIUS_M = 0.022
ADAPTER_LENGTH_M = 0.070


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_value(repo: Path, *args: str) -> str:
    completed = subprocess.run(
        ["git", *args], cwd=repo, check=True, capture_output=True, text=True
    )
    return completed.stdout.strip()


def _fmt(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.10f}".rstrip("0").rstrip(".") for value in values)


def _copy_with_record(
    source: Path,
    destination: Path,
    upstream_path: str,
    derived_root: Path,
) -> dict[str, object]:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    return {
        "upstream_path": upstream_path.replace("\\", "/"),
        "derived_path": destination.relative_to(derived_root).as_posix(),
        "sha256": _sha256(destination),
        "size_bytes": destination.stat().st_size,
    }


def _mesh_paths(robot: ET.Element) -> list[str]:
    return sorted({mesh.attrib["filename"] for mesh in robot.findall(".//mesh")})


def _rewrite_asimov_meshes(robot: ET.Element) -> None:
    for mesh in robot.findall(".//mesh"):
        name = Path(mesh.attrib["filename"].replace("\\", "/")).name
        mesh.attrib["filename"] = f"assets/asimov/{name}"


def _rewrite_orca_meshes(element: ET.Element) -> None:
    prefix = "package://orcahand_description/v1/assets/urdf/right/"
    for mesh in element.findall(".//mesh"):
        source = mesh.attrib["filename"]
        if not source.startswith(prefix):
            raise ValueError(f"unexpected Orca mesh URI: {source}")
        mesh.attrib["filename"] = "assets/orcahand_v1/right/" + source[len(prefix) :]


def _make_adapter_geometry(tower: ET.Element) -> None:
    adapter_origin = (0.0, 0.0, -ADAPTER_LENGTH_M / 2.0)
    visual = ET.SubElement(tower, "visual", {"name": "asimov_orca_adapter_visual"})
    ET.SubElement(visual, "origin", {"xyz": _fmt(adapter_origin), "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "cylinder",
        {"radius": f"{ADAPTER_RADIUS_M:.3f}", "length": f"{ADAPTER_LENGTH_M:.3f}"},
    )
    material = ET.SubElement(visual, "material", {"name": "asimov_orca_adapter_metal"})
    ET.SubElement(material, "color", {"rgba": "0.12 0.16 0.20 1"})

    collision = ET.SubElement(tower, "collision", {"name": "asimov_orca_adapter_collision"})
    ET.SubElement(collision, "origin", {"xyz": _fmt(adapter_origin), "rpy": "0 0 0"})
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(
        collision_geometry,
        "cylinder",
        {"radius": f"{ADAPTER_RADIUS_M:.3f}", "length": f"{ADAPTER_LENGTH_M:.3f}"},
    )


def build(asimov_root: Path, orca_root: Path, output_root: Path) -> None:
    asimov_head = _git_value(asimov_root, "rev-parse", "HEAD")
    orca_head = _git_value(orca_root, "rev-parse", "HEAD")
    if asimov_head != ASIMOV_COMMIT:
        raise ValueError(f"Asimov checkout is {asimov_head}; expected {ASIMOV_COMMIT}")
    if orca_head != ORCA_COMMIT:
        raise ValueError(f"Orca checkout is {orca_head}; expected {ORCA_COMMIT}")
    asimov_tree = _git_value(asimov_root, "rev-parse", "HEAD^{tree}")
    orca_tree = _git_value(orca_root, "rev-parse", "HEAD^{tree}")
    if asimov_tree != ASIMOV_TREE:
        raise ValueError(f"Asimov tree is {asimov_tree}; expected {ASIMOV_TREE}")
    if orca_tree != ORCA_TREE:
        raise ValueError(f"Orca tree is {orca_tree}; expected {ORCA_TREE}")

    asimov_urdf = asimov_root / "sim-model/urdf/asimov_1.urdf"
    orca_urdf = orca_root / "v1/models/urdf/orcahand_right_extended.urdf"
    asimov_robot = ET.parse(asimov_urdf).getroot()
    orca_robot = ET.parse(orca_urdf).getroot()

    combined = copy.deepcopy(asimov_robot)
    combined.attrib["name"] = "asimov_1_orcahand_v1_right_spike"
    _rewrite_asimov_meshes(combined)

    existing_links = {link.attrib["name"] for link in combined.findall("link")}
    existing_joints = {joint.attrib["name"] for joint in combined.findall("joint")}
    removed_links = {"world", "world2right_tower_fixed_jointbody"}
    removed_joints = {"world2right_tower_fixed", "world2right_tower_fixed_offset"}

    orca_links: list[ET.Element] = []
    for source_link in orca_robot.findall("link"):
        name = source_link.attrib["name"]
        if name in removed_links:
            continue
        if name in existing_links:
            raise ValueError(f"duplicate link name across upstream models: {name}")
        link = copy.deepcopy(source_link)
        _rewrite_orca_meshes(link)
        if name == "right_tower":
            _make_adapter_geometry(link)
        orca_links.append(link)
        existing_links.add(name)

    orca_joints: list[ET.Element] = []
    for source_joint in orca_robot.findall("joint"):
        name = source_joint.attrib["name"]
        if name in removed_joints:
            continue
        if name in existing_joints:
            raise ValueError(f"duplicate joint name across upstream models: {name}")
        joint = copy.deepcopy(source_joint)
        orca_joints.append(joint)
        existing_joints.add(name)

    mount = ET.Element("joint", {"name": "asimov_to_orcahand_mount", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": "right_wrist_yaw_link"})
    ET.SubElement(mount, "child", {"link": "right_tower"})
    ET.SubElement(mount, "origin", {"xyz": _fmt(MOUNT_XYZ), "rpy": _fmt(MOUNT_RPY)})

    combined.append(
        ET.Comment(
            " Derived 2026-09-26: removed only Orca's world/fixed anchoring pair, "
            "attached the retained right_tower subtree to Asimov's retained right wrist. "
        )
    )
    combined.append(mount)
    for link in orca_links:
        combined.append(link)
    for joint in orca_joints:
        combined.append(joint)

    output_root.mkdir(parents=True, exist_ok=True)
    output_urdf = output_root / "asimov_orcahand_right.urdf"
    ET.indent(combined, space="  ")
    ET.ElementTree(combined).write(output_urdf, encoding="utf-8", xml_declaration=True)

    copied_assets: list[dict[str, object]] = []
    for source_uri in _mesh_paths(asimov_robot):
        source = (asimov_urdf.parent / source_uri).resolve()
        destination = output_root / "assets/asimov" / source.name
        copied_assets.append(
            _copy_with_record(
                source,
                destination,
                source.relative_to(asimov_root).as_posix(),
                output_root,
            )
        )

    orca_prefix = "package://orcahand_description/"
    for source_uri in _mesh_paths(orca_robot):
        if not source_uri.startswith(orca_prefix):
            raise ValueError(f"unexpected Orca mesh URI: {source_uri}")
        relative = source_uri[len(orca_prefix) :]
        source = orca_root / relative
        suffix = relative.removeprefix("v1/assets/urdf/right/")
        destination = output_root / "assets/orcahand_v1/right" / suffix
        copied_assets.append(_copy_with_record(source, destination, relative, output_root))

    copied_notices: list[dict[str, object]] = []
    for source_name, derived in (
        ("README.md", "licenses/asimov/UPSTREAM_README.md"),
        ("SOFTWARE-LICENSE.txt", "licenses/asimov/SOFTWARE-LICENSE.txt"),
        ("HARDWARE-LICENSE.txt", "licenses/asimov/HARDWARE-LICENSE.txt"),
    ):
        copied_notices.append(
            _copy_with_record(asimov_root / source_name, output_root / derived, source_name, output_root)
        )
    for source_name, derived in (
        ("README.md", "licenses/orcahand/UPSTREAM_README.md"),
        ("LICENSE", "licenses/orcahand/LICENSE"),
    ):
        copied_notices.append(
            _copy_with_record(orca_root / source_name, output_root / derived, source_name, output_root)
        )

    manifest = {
        "schema_version": 1,
        "generated_on": "2026-09-26",
        "purpose": "Local feasibility spike; not a production G02 asset or license determination.",
        "sources": {
            "asimov_1": {
                "repository": "https://github.com/menloresearch/asimov-1",
                "commit": ASIMOV_COMMIT,
                "tree": ASIMOV_TREE,
                "commit_timestamp": _git_value(asimov_root, "show", "-s", "--format=%cI", "HEAD"),
                "commit_subject": _git_value(asimov_root, "show", "-s", "--format=%s", "HEAD"),
                "source_urdf": "sim-model/urdf/asimov_1.urdf",
                "source_urdf_git_blob": _git_value(
                    asimov_root, "rev-parse", f"{ASIMOV_COMMIT}:sim-model/urdf/asimov_1.urdf"
                ),
                "source_urdf_sha256": _sha256(asimov_urdf),
                "mesh_root": "sim-model/assets/meshes/",
                "license_notices": ["SOFTWARE-LICENSE.txt", "HARDWARE-LICENSE.txt"],
                "license_scope_note": (
                    "The upstream README labels repository software GPL-2.0 and hardware "
                    "CERN-OHL-S-2.0, but the selected sim-model files have no path-level SPDX "
                    "markers. Both notices are preserved; redistribution needs separate review."
                ),
            },
            "orcahand_description": {
                "repository": "https://github.com/orcahand/orcahand_description",
                "commit": ORCA_COMMIT,
                "tree": ORCA_TREE,
                "commit_timestamp": _git_value(orca_root, "show", "-s", "--format=%cI", "HEAD"),
                "commit_subject": _git_value(orca_root, "show", "-s", "--format=%s", "HEAD"),
                "source_urdf": "v1/models/urdf/orcahand_right_extended.urdf",
                "source_urdf_git_blob": _git_value(
                    orca_root,
                    "rev-parse",
                    f"{ORCA_COMMIT}:v1/models/urdf/orcahand_right_extended.urdf",
                ),
                "source_urdf_sha256": _sha256(orca_urdf),
                "mesh_root": "v1/assets/urdf/right/",
                "license_notice": "LICENSE",
                "license": "MIT",
            },
        },
        "derivation": {
            "output_urdf": output_urdf.name,
            "output_urdf_sha256": _sha256(output_urdf),
            "mount_joint": "asimov_to_orcahand_mount",
            "mount_parent": "right_wrist_yaw_link",
            "mount_child": "right_tower",
            "mount_xyz_m": list(MOUNT_XYZ),
            "mount_rpy_rad": list(MOUNT_RPY),
            "adapter_radius_m": ADAPTER_RADIUS_M,
            "adapter_length_m": ADAPTER_LENGTH_M,
            "removed_orca_links": sorted(removed_links),
            "removed_orca_joints": sorted(removed_joints),
            "preserved_subtree_root": "right_tower",
        },
        "copied_assets": sorted(copied_assets, key=lambda record: str(record["derived_path"])),
        "copied_license_notices": copied_notices,
    }
    (output_root / "upstream_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )

    print(f"wrote {output_urdf}")
    print(f"copied {len(copied_assets)} referenced meshes")
    print(f"wrote {output_root / 'upstream_manifest.json'}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--asimov-root", type=Path, required=True)
    parser.add_argument("--orca-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    build(args.asimov_root.resolve(), args.orca_root.resolve(), args.output_root.resolve())


if __name__ == "__main__":
    main()
