#!/usr/bin/env python3
"""Deterministically derive the production URDF without editing spike inputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from robot_spike.production.model import UrdfModel, canonical_text_sha256
else:
    from .model import UrdfModel, canonical_text_sha256


EXPECTED_SOURCE_SHA256 = "25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea"
EXPECTED_MANIFEST_SHA256 = "6ddcd328db7c61de83e6ccea5bbf5a496b714bc9370acb937831f3a21f9a6b80"


def build(root: Path) -> dict[str, object]:
    root = root.resolve()
    source_root = root.parent
    source = source_root / "asimov_orcahand_right.urdf"
    source_manifest = source_root / "upstream_manifest.json"
    output = root / "asimov_orcahand_restocking.urdf"
    if canonical_text_sha256(source) != EXPECTED_SOURCE_SHA256:
        raise RuntimeError("approved combined URDF hash mismatch")
    if canonical_text_sha256(source_manifest) != EXPECTED_MANIFEST_SHA256:
        raise RuntimeError("approved upstream manifest hash mismatch")

    tree = ET.parse(source)
    robot = tree.getroot()
    source_name = robot.attrib["name"]
    robot.attrib["name"] = "asimov_1_orcahand_v1_right_restocking"
    rewritten = 0
    for mesh in robot.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        if filename.startswith("assets/"):
            mesh.attrib["filename"] = f"../{filename}"
            rewritten += 1
        else:
            raise RuntimeError(f"unexpected source mesh path {filename!r}")

    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    with output.open("ab") as stream:
        stream.write(b"\n")

    model = UrdfModel.load(output, source_root)
    manifest = {
        "schema_version": 1,
        "source_urdf": "../asimov_orcahand_right.urdf",
        "source_urdf_canonical_sha256": EXPECTED_SOURCE_SHA256,
        "source_upstream_manifest": "../upstream_manifest.json",
        "source_upstream_manifest_canonical_sha256": EXPECTED_MANIFEST_SHA256,
        "production_urdf": output.name,
        "production_urdf_canonical_sha256": canonical_text_sha256(output),
        "derivation": {
            "source_robot_name": source_name,
            "production_robot_name": model.robot_name,
            "mesh_reference_rewrites": rewritten,
            "mesh_policy": "reuse exact approved robot_spike/assets files in place",
            "inertial_changes": 0,
            "collision_geometry_changes": 0,
            "joint_changes": 0,
            "link_changes": 0,
        },
        "structure": {
            "root_link": model.root_link,
            "link_count": len(model.link_names),
            "joint_count": len(model.joint_names),
            "dof_count": len(model.dof_names),
            "mesh_reference_count": len(model.mesh_references),
            "unique_mesh_file_count": len({reference.path for reference in model.mesh_references}),
            "links_without_collision_geometry": list(model.links_without_collision),
            "zero_mass_links": list(model.zero_mass_links),
            "tiny_inertia_links": list(model.tiny_inertia_links),
        },
    }
    (root / "production_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    print(json.dumps(build(args.root), indent=2))


if __name__ == "__main__":
    main()
