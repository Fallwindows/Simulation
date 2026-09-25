"""Dependency-light retail asset catalog validation."""

from __future__ import annotations

import json
import hashlib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RetailAsset:
    asset_key: str
    category: str
    usd_path: Path
    texture_path: Path
    normal_texture_path: Path | None
    roughness_texture_path: Path | None
    dimensions_m: tuple[float, float, float]
    local_front_axis: str
    model_type: str
    product_name: str
    department: str
    intended_support: str
    assembly_profile: str
    assembly_parts: tuple[str, ...]
    geometry_signature: str
    material_classes: tuple[str, ...]
    introduced_in: str
    geometry_scope_sha256: str
    appearance_scope_sha256: str


@dataclass(frozen=True)
class RetailAssetCatalog:
    manifest_path: Path
    assets: tuple[RetailAsset, ...]

    def by_category(self, category: str) -> tuple[RetailAsset, ...]:
        return tuple(sorted(
            (asset for asset in self.assets if asset.category == category),
            key=lambda asset: asset.asset_key,
        ))

    def by_key(self, asset_key: str) -> RetailAsset:
        for asset in self.assets:
            if asset.asset_key == asset_key:
                return asset
        raise KeyError(f"Retail asset key is not in the manifest: {asset_key}")


def load_retail_catalog(path: str | Path) -> RetailAssetCatalog:
    manifest_path = Path(path).resolve()
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    if int(data.get("schema_version", 0)) < 2:
        raise ValueError("Retail manifest schema_version must be at least 2")
    if data.get("standard_front_axis") != "+Y":
        raise ValueError("Retail manifest standard_front_axis must be +Y")
    entries = data.get("assets")
    if not isinstance(entries, list) or not entries:
        raise ValueError("Retail manifest must contain a non-empty assets list")
    assets: list[RetailAsset] = []
    keys: set[str] = set()
    for entry in entries:
        key = str(entry["asset_key"])
        if key in keys:
            raise ValueError(f"Duplicate retail asset key: {key}")
        keys.add(key)
        dimensions = tuple(float(v) for v in entry["dimensions_m"])
        if len(dimensions) != 3 or not all(v > 0 for v in dimensions):
            raise ValueError(f"Retail asset {key} has invalid dimensions")
        if entry.get("local_front_axis") != "+Y":
            raise ValueError(f"Retail asset {key} must use local +Y front axis")
        usd_path = manifest_path.parent / str(entry["usd_path"])
        texture_path = manifest_path.parent / str(entry["texture_path"])
        if not usd_path.is_file():
            raise FileNotFoundError(f"Retail asset {key} USD is missing: {usd_path}")
        if not texture_path.is_file():
            raise FileNotFoundError(f"Retail asset {key} texture is missing: {texture_path}")
        for artifact_path, hash_field in ((usd_path, "usd_sha256"), (texture_path, "texture_sha256")):
            expected_hash = str(entry.get(hash_field, ""))
            actual_hash = hashlib.sha256(artifact_path.read_bytes()).hexdigest()
            if expected_hash != actual_hash:
                raise ValueError(
                    f"Retail asset {key} {hash_field} mismatch: expected {expected_hash}, got {actual_hash}"
                )
        surface_maps: dict[str, Path | None] = {}
        for prefix in ("normal", "roughness"):
            path_value = entry.get(f"{prefix}_texture_path")
            hash_value = entry.get(f"{prefix}_texture_sha256")
            if (path_value is None) != (hash_value is None):
                raise ValueError(f"Retail asset {key} has incomplete {prefix} texture evidence")
            if path_value is None:
                surface_maps[prefix] = None
                continue
            surface_path = manifest_path.parent / str(path_value)
            if not surface_path.is_file():
                raise FileNotFoundError(f"Retail asset {key} {prefix} texture is missing: {surface_path}")
            actual_hash = hashlib.sha256(surface_path.read_bytes()).hexdigest()
            if actual_hash != str(hash_value):
                raise ValueError(
                    f"Retail asset {key} {prefix}_texture_sha256 mismatch: expected {hash_value}, got {actual_hash}"
                )
            surface_maps[prefix] = surface_path
        assembly_parts = tuple(str(value) for value in entry.get("assembly_parts", ()))
        material_classes = tuple(str(value) for value in entry.get("material_classes", ()))
        geometry_signature = str(entry.get("geometry_signature", ""))
        scoped_hashes = {
            "geometry_signature": geometry_signature,
            "geometry_scope_sha256": str(entry.get("geometry_scope_sha256", "")),
            "appearance_scope_sha256": str(entry.get("appearance_scope_sha256", "")),
        }
        for field, value in scoped_hashes.items():
            if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
                raise ValueError(f"Retail asset {key} has invalid {field}")
        if not assembly_parts or not material_classes:
            raise ValueError(f"Retail asset {key} lacks assembly/material evidence")
        assets.append(RetailAsset(
            asset_key=key,
            category=str(entry["category"]),
            usd_path=usd_path,
            texture_path=texture_path,
            normal_texture_path=surface_maps["normal"],
            roughness_texture_path=surface_maps["roughness"],
            dimensions_m=dimensions,
            local_front_axis="+Y",
            model_type=str(entry.get("model_type", "box")),
            product_name=str(entry.get("product_name", key)),
            department=str(entry.get("department", "unknown")),
            intended_support=str(entry.get("intended_support", "shelf")),
            assembly_profile=str(entry.get("assembly_profile", entry.get("model_type", "box"))),
            assembly_parts=assembly_parts,
            geometry_signature=geometry_signature,
            material_classes=material_classes,
            introduced_in=str(entry.get("introduced_in", "unknown")),
            geometry_scope_sha256=scoped_hashes["geometry_scope_sha256"],
            appearance_scope_sha256=scoped_hashes["appearance_scope_sha256"],
        ))
    return RetailAssetCatalog(manifest_path, tuple(assets))
