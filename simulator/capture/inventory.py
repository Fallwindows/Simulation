"""Ground-truth inventory/scene export for evaluation-only consumers."""

from __future__ import annotations

import csv
import json
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.environment.retail_catalog import load_retail_catalog


def inventory_rows(scenario_path: str | Path) -> list[dict[str, object]]:
    scenario = load_scenario(scenario_path)
    layout = build_aisle_layout(scenario.environment)
    catalog = load_retail_catalog(scenario.environment.asset_manifest_path)
    records = {asset.asset_key: asset for asset in catalog.assets}
    rows = []
    for asset in layout.assets:
        record = records[asset.asset_key]
        x, y, z = asset.position_m
        roll, pitch, yaw = asset.rotation_rpy_deg
        width, depth, height = record.dimensions_m
        rows.append({
            "semantic_id": asset.semantic_id,
            "asset_key": asset.asset_key,
            "category": asset.category,
            "x_m": x,
            "y_m": y,
            "z_m": z,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
            "width_m": width,
            "depth_m": depth,
            "height_m": height,
        })
    return rows


def export_inventory(scenario_path: str | Path, output_dir: str | Path) -> dict[str, object]:
    target = Path(output_dir)
    target.mkdir(parents=True, exist_ok=True)
    rows = inventory_rows(scenario_path)
    fields = list(rows[0]) if rows else ["semantic_id", "asset_key", "category", "x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg", "width_m", "depth_m", "height_m"]
    with (target / "inventory_ground_truth.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (target / "inventory_ground_truth.json").write_text(json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {"status": "complete", "asset_count": len(rows), "csv": "inventory_ground_truth.csv", "json": "inventory_ground_truth.json"}


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    print(json.dumps(export_inventory(args.scenario, args.output_dir), indent=2))


if __name__ == "__main__":
    main()
