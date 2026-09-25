from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "tools" / "generate_shopping_cart.py"
ASSET_DIR = ROOT / "assets" / "scene" / "store_context"


def _load_generator():
    spec = importlib.util.spec_from_file_location("generate_shopping_cart", GENERATOR)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _tree_bytes(directory: Path) -> dict[str, bytes]:
    return {path.relative_to(directory).as_posix(): path.read_bytes() for path in sorted(directory.rglob("*")) if path.is_file()}


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_text_bytes(data: bytes) -> bytes:
    return data.replace(b"\r\n", b"\n")


def test_generator_is_byte_reproducible_and_committed_outputs_are_current(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"
    for output in (first, second):
        result = subprocess.run(
            [sys.executable, str(GENERATOR), "--output-dir", str(output)],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        assert '"asset_id": "shopping_cart"' in result.stdout
    assert _tree_bytes(first) == _tree_bytes(second)
    assert _tree_bytes(first) == _tree_bytes(ASSET_DIR)


def test_manifest_hashes_every_generated_asset_and_generator() -> None:
    manifest = json.loads((ASSET_DIR / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["asset_id"] == "shopping_cart"
    assert manifest["generator"]["path"] == "tools/generate_shopping_cart.py"
    assert manifest["generator"]["hash_basis"] == "UTF-8 with LF line endings"
    assert manifest["generator"]["sha256"] == _sha256(_canonical_text_bytes(GENERATOR.read_bytes()))
    expected = {".gitattributes", "shopping_cart.usda", "shopping_cart.metadata.json", "shopping_cart_preview.svg"}
    assert {record["path"] for record in manifest["files"]} == expected
    for record in manifest["files"]:
        data = (ASSET_DIR / record["path"]).read_bytes()
        assert record["bytes"] == len(data)
        assert record["sha256"] == _sha256(data)
    assert [item["sha256"] for item in manifest["source_references"]] == [
        "1dd93c320ce31c79eaa1a40c7e3b96507656ae59ab28eadec7d94e089cfc77d7",
        "09154a4f078b9d625c4cb80bd3a9e46397c49159338cdfba98929eab2dd39297",
    ]


def test_dimensions_obb_placement_and_collision_contract_are_exact() -> None:
    metadata = json.loads((ASSET_DIR / "shopping_cart.metadata.json").read_text(encoding="utf-8"))
    assert metadata["units"] == "meters"
    assert metadata["dimensions_m"] == {"length_x": 1.03, "width_y": 0.6, "height_z": 1.03}
    assert metadata["oriented_bounding_box"] == {
        "axes": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "center_m": [-0.003, 0.0, 0.515],
        "half_extents_m": [0.515, 0.3, 0.515],
        "min_m": [-0.518, -0.3, 0.0],
        "max_m": [0.512, 0.3, 1.03],
    }
    assert metadata["placement"] == {
        "corridor_membership": "unassigned; scene owner must place outside M1 if used as context",
        "ground_plane_z_m": 0.0,
        "independently_placeable": True,
        "pivot_m": [0.0, 0.0, 0.0],
        "requires_human": False,
        "root_prim": "/ShoppingCart",
        "scene_placement": "unassigned",
    }
    assert metadata["footprint"] == {
        "aabb_xy_m": {"min": [-0.518, -0.3], "max": [0.512, 0.3]},
        "corners_xy_m": [[-0.518, -0.3], [0.512, -0.3], [0.512, 0.3], [-0.518, 0.3]],
    }
    assert metadata["casters"] == {
        "centers_m": [[-0.335, -0.258, 0.068], [-0.335, 0.258, 0.068], [0.375, -0.258, 0.068], [0.375, 0.258, 0.068]],
        "count": 4,
        "wheel_contact_z_m": 0.0,
        "wheel_radius_m": 0.068,
        "wheel_width_m": 0.038,
    }
    collision = metadata["collision"]
    assert collision["representation"] == "authored_box_proxies"
    assert len(collision["primitives"]) == 13
    assert {primitive["shape"] for primitive in collision["primitives"]} == {"box"}
    for primitive in collision["primitives"]:
        assert all(component > 0.0 for component in primitive["size_m"])
        for axis in range(3):
            assert primitive["center_m"][axis] - primitive["size_m"][axis] * 0.5 >= metadata["oriented_bounding_box"]["min_m"][axis] - 1e-9
            assert primitive["center_m"][axis] + primitive["size_m"][axis] * 0.5 <= metadata["oriented_bounding_box"]["max_m"][axis] + 1e-9


def test_geometry_stays_inside_declared_obb_and_is_full_scale() -> None:
    generator = _load_generator()
    tubes = generator.build_tubes()
    bounds_min = generator.ASSET_BOUNDS_MIN
    bounds_max = generator.ASSET_BOUNDS_MAX
    assert 0.9 <= generator.ASSET_DIMENSIONS[2] <= 1.1
    for tube in tubes:
        delta = [tube.end[axis] - tube.start[axis] for axis in range(3)]
        length = math.sqrt(sum(component * component for component in delta))
        axis_vector = [component / length for component in delta]
        for axis in range(3):
            radial_extent = tube.radius * math.sqrt(1.0 - axis_vector[axis] ** 2)
            actual_min = min(tube.start[axis], tube.end[axis]) - radial_extent
            actual_max = max(tube.start[axis], tube.end[axis]) + radial_extent
            assert actual_min >= bounds_min[axis] - 1e-9, tube.name
            assert actual_max <= bounds_max[axis] + 1e-9, tube.name


def test_basket_is_open_wire_lattice_with_real_interior() -> None:
    metadata = json.loads((ASSET_DIR / "shopping_cart.metadata.json").read_text(encoding="utf-8"))
    basket = metadata["basket"]
    assert basket["construction"] == "open_wire_lattice"
    assert basket["open_top"] is True
    assert basket["opaque_shell"] is False
    cavity = basket["usable_interior_aabb_m"]
    cavity_dimensions = [cavity["max"][axis] - cavity["min"][axis] for axis in range(3)]
    assert cavity_dimensions == [0.72, 0.47, 0.31999999999999995]
    role_counts = metadata["geometry"]["role_counts"]
    assert role_counts["basket_wire"] >= 50
    assert role_counts["basket_floor_wire"] >= 18
    usda = (ASSET_DIR / "shopping_cart.usda").read_text(encoding="utf-8")
    assert 'def Xform "ShoppingCart"' in usda
    assert 'def Xform "Visual"' in usda
    assert 'def Xform "Collision"' in usda
    assert "opaque basket" not in usda.lower()
    assert 'cart:role = "basket_wire"' in usda
    visual_section = usda.split('def Xform "Visual"', 1)[1].split('def Xform "Collision"', 1)[0]
    assert 'def Cube ' not in visual_section


def test_four_casters_handle_and_pbr_materials_are_authored() -> None:
    generator = _load_generator()
    tubes = generator.build_tubes()
    assert len([tube for tube in tubes if tube.role == "wheel"]) == 4
    assert len([tube for tube in tubes if tube.role == "caster_stem"]) == 4
    assert len([tube for tube in tubes if tube.role == "caster_fork"]) == 8
    assert len([tube for tube in tubes if tube.role == "handle"]) == 1
    usda = (ASSET_DIR / "shopping_cart.usda").read_text(encoding="utf-8")
    assert usda.count('uniform token info:id = "UsdPreviewSurface"') == 4
    assert usda.count("float inputs:metallic") == 4
    assert usda.count("float inputs:roughness") == 4
    assert 'rel material:binding = </ShoppingCart/Looks/ZincWire>' in usda
    assert 'rel material:binding = </ShoppingCart/Looks/Rubber>' in usda


def test_cpu_preview_is_self_contained_and_identifies_scale() -> None:
    preview = (ASSET_DIR / "shopping_cart_preview.svg").read_text(encoding="utf-8")
    assert preview.startswith('<svg xmlns="http://www.w3.org/2000/svg"')
    assert "1.03 × 0.60 × 1.03 m" in preview
    assert "open wire basket" in preview
    assert "http://" not in preview.removeprefix('<svg xmlns="http://www.w3.org/2000/svg"')
    assert "https://" not in preview


def _run_without_pytest() -> None:
    scratch = ROOT / ".shopping-cart-test-scratch"
    if scratch.exists():
        raise RuntimeError(f"refusing to reuse existing test scratch: {scratch}")
    scratch.mkdir()
    try:
        test_generator_is_byte_reproducible_and_committed_outputs_are_current(scratch)
    finally:
        shutil.rmtree(scratch)
    for name, value in sorted(globals().items()):
        if name.startswith("test_") and name != "test_generator_is_byte_reproducible_and_committed_outputs_are_current":
            value()
        if name.startswith("test_"):
            print(f"PASS {name}")


if __name__ == "__main__":
    _run_without_pytest()
