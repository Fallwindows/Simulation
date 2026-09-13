"""Dependency-light retail asset catalog validation."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RetailAsset:
    asset_key: str
    category: str
    usd_path: Path
    texture_path: Path
    dimensions_m: tuple[float, float, float]
    local_front_axis: str
    model_type: str
    product_name: str


@dataclass(frozen=True)
class RetailAssetCatalog:
    manifest_path: Path
    assets: tuple[RetailAsset, ...]

    def by_category(self, category: str) -> tuple[RetailAsset, ...]:
        return tuple(asset for asset in self.assets if asset.category == category)

    def by_key(self, asset_key: str) -> RetailAsset:
        for asset in self.assets:
            if asset.asset_key == asset_key:
                return asset
        raise KeyError(f"Retail asset key is not in the manifest: {asset_key}")


def load_retail_catalog(path: str | Path) -> RetailAssetCatalog:
    manifest_path = Path(path).resolve()
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
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
        assets.append(RetailAsset(
            asset_key=key,
            category=str(entry["category"]),
            usd_path=usd_path,
            texture_path=texture_path,
            dimensions_m=dimensions,
            local_front_axis="+Y",
            model_type=str(entry.get("model_type", "box")),
            product_name=str(entry.get("product_name", key)),
        ))
    return RetailAssetCatalog(manifest_path, tuple(assets))
