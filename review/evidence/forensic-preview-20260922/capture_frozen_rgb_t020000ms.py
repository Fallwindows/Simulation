"""Capture one frozen RGB diagnostic frame with Isaac Sim's in-process renderer.

This deliberately reuses the repository's aisle and camera builders without
starting ROS, LiDAR, the normal runtime loop, or an external video recorder.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp


EVIDENCE_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[3]
SCENARIO_PATH = REPO_ROOT / "config/scenarios/walking_baseline.yaml"
OUTPUT_PATH = EVIDENCE_DIR / "before_rgb_t020000ms.png"
RECORD_PATH = EVIDENCE_DIR / "before_rgb_t020000ms.json"
EXPECTED_COMMIT = "d5e825c8f6dab77aa6a1007c9731c226b588dfcf"
EXPECTED_TREE = "a9d85319fb018a94de695d44d085633f19cdc008"
CAPTURE_TIME_S = 20.0
RENDERER = "RaytracedLighting"
RT_SUBFRAMES = 8
INPUT_PATHS = (
    "simulator/runtime/isaac_sim_runner.py",
    "simulator/config/loader.py",
    "simulator/environment/isaac_builder.py",
    "simulator/environment/aisle_builder.py",
    "simulator/environment/retail_catalog.py",
    "simulator/motion/trajectory.py",
    "simulator/sensors/transforms.py",
    "config/scenarios/walking_baseline.yaml",
    "config/environments/baseline_aisle.yaml",
    "config/sensors/baseline_rgb_lidar.yaml",
    "config/trajectories/walking.yaml",
    "assets/retail/manifest.json",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git(*args: str) -> str:
    completed = subprocess.run(
        ["git", "-C", str(REPO_ROOT), *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def _validate_inputs() -> tuple[str, str, dict[str, str]]:
    head = _git("rev-parse", "HEAD")
    tree = _git("rev-parse", "HEAD^{tree}")
    if head != EXPECTED_COMMIT or tree != EXPECTED_TREE:
        raise RuntimeError(f"unexpected source identity: commit={head} tree={tree}")
    dirty = _git("status", "--short", "--", *INPUT_PATHS)
    if dirty:
        raise RuntimeError(f"capture inputs differ from HEAD:\n{dirty}")
    hashes = {relative: _sha256(REPO_ROOT / relative) for relative in INPUT_PATHS}
    return head, tree, hashes


def main() -> None:
    if OUTPUT_PATH.exists() or RECORD_PATH.exists():
        raise FileExistsError("refusing to overwrite existing forensic evidence")

    head, tree, input_hashes = _validate_inputs()
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))

    app = SimulationApp({"renderer": RENDERER, "headless": True})
    rgb_annotator = None
    render_product = None
    try:
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom

        from simulator.config.loader import load_scenario
        from simulator.motion.trajectory import WalkingTrajectory
        from simulator.runtime.isaac_sim_runner import _build_sensor_rig, _build_world
        from simulator.sensors.transforms import rpy_deg_from_quaternion

        scenario = load_scenario(SCENARIO_PATH)
        omni.usd.get_context().new_stage()
        app.update()
        stage = omni.usd.get_context().get_stage()
        _layout, primitive_count = _build_world(stage, scenario)
        rig_prim, camera_path, _lidar_path = _build_sensor_rig(stage, scenario)

        sample = WalkingTrajectory(scenario.trajectory).sample(CAPTURE_TIME_S)
        rig_api = UsdGeom.XformCommonAPI(rig_prim)
        rig_api.SetTranslate(Gf.Vec3d(*sample.position_m))
        rig_api.SetRotate(
            Gf.Vec3f(*rpy_deg_from_quaternion(sample.orientation_xyzw)),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )

        camera_prim = stage.GetPrimAtPath(camera_path)
        if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
            raise RuntimeError(f"invalid USD camera at {camera_path}")

        deadline = time.monotonic() + 30.0
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            if time.monotonic() >= deadline:
                raise TimeoutError("USD references did not finish loading within 30 seconds")
            app.update()

        rep.orchestrator.set_capture_on_play(False)
        render_product = rep.create.render_product(
            camera_path,
            (scenario.camera.width_px, scenario.camera.height_px),
            name="ForensicBaselineRGB",
        )
        rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annotator.attach(render_product)

        rep.orchestrator.step(
            rt_subframes=RT_SUBFRAMES,
            delta_time=0.0,
            pause_timeline=False,
        )
        rgb_data = rgb_annotator.get_data()
        if getattr(rgb_data, "size", 0) == 0:
            raise RuntimeError("RGB annotator returned no pixels")
        rep.functional.write_image(path=str(OUTPUT_PATH), data=rgb_data)
        rep.orchestrator.wait_until_complete()

        with Image.open(OUTPUT_PATH) as image:
            image.load()
            width, height = image.size
            mode = image.mode
            extrema = [list(pair) for pair in image.getextrema()]
        expected_size = (scenario.camera.width_px, scenario.camera.height_px)
        if (width, height) != expected_size:
            raise RuntimeError(f"wrong image dimensions: {(width, height)} != {expected_size}")
        if not any(low < high for low, high in extrema[:3]):
            raise RuntimeError(f"RGB image is constant: extrema={extrema}")
        pngs = sorted(EVIDENCE_DIR.glob("before_rgb_t020000ms.png"))
        if pngs != [OUTPUT_PATH]:
            raise RuntimeError(f"expected exactly one PNG, found: {pngs}")

        script_hash = _sha256(Path(__file__).resolve())
        record = {
            "status": "complete",
            "purpose": "forensic comparable before frame",
            "source": {
                "commit": head,
                "tree": tree,
                "workspace": str(REPO_ROOT),
                "selected_inputs_match_head": True,
                "input_sha256": input_hashes,
                "script": str(Path(__file__).resolve()),
                "script_sha256": script_hash,
            },
            "runtime": {
                "launcher": "C:/isaacsim/python.bat",
                "isaac_version": (Path("C:/isaacsim/VERSION").read_text(encoding="utf-8").strip()),
                "renderer": RENDERER,
                "headless": True,
                "ros_enabled": False,
                "opencv_used": False,
                "capture_api": "omni.replicator.core rgb annotator + functional.write_image",
                "rt_subframes": RT_SUBFRAMES,
                "captured_frame_count": 1,
            },
            "scene": {
                "scenario": str(SCENARIO_PATH),
                "camera_path": camera_path,
                "capture_time_s": CAPTURE_TIME_S,
                "rig_position_m": list(sample.position_m),
                "rig_orientation_xyzw": list(sample.orientation_xyzw),
                "primitive_count": primitive_count,
            },
            "image": {
                "path": str(OUTPUT_PATH),
                "width": width,
                "height": height,
                "mode": mode,
                "channel_extrema": extrema,
                "size_bytes": OUTPUT_PATH.stat().st_size,
                "sha256": _sha256(OUTPUT_PATH),
            },
        }
        RECORD_PATH.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(record, indent=2), flush=True)
    finally:
        if rgb_annotator is not None:
            rgb_annotator.detach()
        if render_product is not None:
            render_product.destroy()
        app.close()


if __name__ == "__main__":
    main()
