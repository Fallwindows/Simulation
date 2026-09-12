"""Non-interactive scenario sanity runner.

This command exercises config loading, deterministic geometry, and rig motion
without pretending to replace Isaac Sim's sensor/rendering runtime.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulator.config.loader import load_scenario
from simulator.environment.aisle_builder import build_aisle_layout
from simulator.motion.trajectory import StraightTrajectory, WalkingTrajectory


def run(scenario_path: str | Path, steps: int) -> dict[str, object]:
    scenario = load_scenario(scenario_path)
    layout = build_aisle_layout(scenario.environment)
    trajectory_cls = WalkingTrajectory if scenario.trajectory.name.lower() == "walking" else StraightTrajectory
    trajectory = trajectory_cls(scenario.trajectory)
    samples = tuple(trajectory.sample(i / scenario.trajectory.sample_hz) for i in range(max(1, steps)))
    return {
        "scenario": scenario.name,
        "primitive_count": len(layout.primitives),
        "product_count": len(layout.products),
        "duration_s": scenario.trajectory.duration_s,
        "sample_count": len(samples),
        "first_pose": {"timestamp_s": samples[0].timestamp_s, "position_m": samples[0].position_m},
        "last_pose": {"timestamp_s": samples[-1].timestamp_s, "position_m": samples[-1].position_m},
        "runtime_note": "This is a deterministic preflight; Isaac sensor rendering remains runtime-specific.",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="config/scenarios/baseline_straight.yaml")
    parser.add_argument("--steps", type=int, default=10)
    args = parser.parse_args()
    print(json.dumps(run(args.scenario, args.steps), indent=2))


if __name__ == "__main__":
    main()
