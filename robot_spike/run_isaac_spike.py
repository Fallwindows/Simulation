#!/usr/bin/env python3
"""Import, inspect, articulate, and render the combined robot in Isaac Sim."""

from __future__ import annotations

import argparse
import json
import math
import os
import traceback
from pathlib import Path

from isaacsim import SimulationApp


parser = argparse.ArgumentParser()
parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
args, _unknown = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless, "renderer": "RaytracedLighting"})

import carb.settings
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.core.experimental.prims import Articulation
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade


def _enable(extension: str) -> None:
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate(extension, True)


def _prim_by_name(stage: Usd.Stage, name: str) -> Usd.Prim:
    candidates = [prim for prim in stage.Traverse() if prim.GetName() == name]
    if not candidates:
        raise RuntimeError(f"No prim named {name!r} in imported stage")
    xforms = [prim for prim in candidates if prim.IsA(UsdGeom.Xform)]
    return (xforms or candidates)[0]


def _world_matrix(stage: Usd.Stage, name: str) -> np.ndarray:
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(_prim_by_name(stage, name))
    return np.asarray(matrix, dtype=float).reshape(4, 4).T


def _translation(stage: Usd.Stage, name: str) -> np.ndarray:
    return _world_matrix(stage, name)[:3, 3]


def _stage_range(stage: Usd.Stage, prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]:
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    extent = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return np.asarray(extent.GetMin(), dtype=float), np.asarray(extent.GetMax(), dtype=float)


def _apply_orca_diagnostic_material(stage: Usd.Stage) -> dict[str, object]:
    """Make the Orca geometry legible without modifying the imported package."""
    imported_white = stage.GetPrimAtPath("/asimov_1_orcahand_v1_right_spike/Materials/white")
    imported_opacity = None
    if imported_white.IsValid():
        opacity = imported_white.GetAttribute("inputs:opacity")
        imported_opacity = opacity.Get() if opacity.IsValid() else None

    material = UsdShade.Material.Define(stage, "/SpikeDiagnostics/OrcaOpaque")
    shader = UsdShade.Shader.Define(stage, "/SpikeDiagnostics/OrcaOpaque/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.10, 0.32, 0.52))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.38)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.10)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    targets = [
        prim
        for prim in stage.Traverse()
        if prim.GetName().startswith("right_visual_")
    ]
    for prim in targets:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, bindingStrength=UsdShade.Tokens.strongerThanDescendants
        )
    return {
        "purpose": "render-only neutral opaque override for Orca diagnostic views",
        "source_urdf_material": "named white references with no color/alpha definition",
        "imported_white_opacity": imported_opacity,
        "bound_visual_prim_count": len(targets),
        "material_path": str(material.GetPath()),
    }


def _set_dofs(articulation: Articulation, values: dict[str, float]) -> None:
    missing = sorted(set(values) - set(articulation.dof_names))
    if missing:
        raise RuntimeError(f"Imported articulation is missing DOFs: {missing}")
    names = list(values)
    indices = articulation.get_dof_indices(names).numpy().tolist()
    articulation.set_dof_positions([values[name] for name in names], dof_indices=indices)


def _capture(
    output_dir: Path,
    name: str,
    position: np.ndarray,
    target: np.ndarray,
    resolution: tuple[int, int],
    subframes: int = 16,
) -> Path:
    view_dir = output_dir / "renders" / name
    if view_dir.exists():
        raise RuntimeError(f"Refusing to overwrite existing render directory: {view_dir}")
    view_dir.mkdir(parents=True)
    camera = rep.functional.create.camera(
        position=tuple(float(value) for value in position),
        look_at=tuple(float(value) for value in target),
        name=f"Camera_{name}",
    )
    render_product = rep.create.render_product(camera, resolution, name=f"Render_{name}")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(output_dir=str(view_dir), rgb=True)
    writer.attach(render_product)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=subframes, pause_timeline=False)
    rep.orchestrator.wait_until_complete()
    writer.detach()
    render_product.destroy()
    pngs = sorted(view_dir.rglob("*.png"))
    if len(pngs) != 1:
        raise RuntimeError(f"Expected one PNG for {name}, found {len(pngs)}: {pngs}")
    final = output_dir / "renders" / f"{name}.png"
    pngs[0].replace(final)
    return final


def _motion_capture(
    output_dir: Path,
    articulation_root_path: str,
    stage: Usd.Stage,
    position: np.ndarray,
    target: np.ndarray,
) -> dict[str, object]:
    frame_dir = output_dir / "motion_frames"
    if frame_dir.exists():
        raise RuntimeError(f"Refusing to overwrite existing motion directory: {frame_dir}")
    frame_dir.mkdir(parents=True)
    camera = rep.functional.create.camera(
        position=tuple(float(value) for value in position),
        look_at=tuple(float(value) for value in target),
        name="Camera_motion",
    )
    render_product = rep.create.render_product(camera, (640, 480), name="Render_motion")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(output_dir=str(frame_dir), rgb=True)

    # Creating the camera and render product mutates the stage and invalidates an
    # already initialized physics tensor view. Recreate the articulation only
    # after the motion capture graph exists and the timeline has restarted.
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.stop()
    for _ in range(2):
        simulation_app.update()
    timeline.play()
    for _ in range(5):
        simulation_app.update()
    articulation = Articulation(articulation_root_path)
    writer.attach(render_product)

    fixed_pose = {
        "right_shoulder_pitch_joint": 0.20,
        "right_shoulder_roll_joint": 0.60,
        "right_shoulder_yaw_joint": -0.15,
        "right_elbow_joint": -0.65,
        "left_shoulder_pitch_joint": 0.10,
        "left_shoulder_roll_joint": -0.25,
        "left_elbow_joint": 0.0,
    }
    records: list[dict[str, object]] = []
    relative_mounts: list[np.ndarray] = []
    frame_count = 24
    for frame in range(frame_count):
        phase = frame / (frame_count - 1)
        ease = phase * phase * (3.0 - 2.0 * phase)
        pose = dict(fixed_pose)
        pose.update(
            {
                "right_wrist_yaw_joint": -0.15 + 0.40 * ease,
                "right_wrist": -0.20 + 0.55 * ease,
                "right_thumb_mcp": 0.25 * ease,
                "right_thumb_abd": -0.40 * ease,
                "right_thumb_pip": 0.60 * ease,
                "right_thumb_dip": 0.50 * ease,
                "right_index_mcp": 0.70 * ease,
                "right_index_pip": 0.80 * ease,
                "right_middle_mcp": 0.60 * ease,
                "right_middle_pip": 0.75 * ease,
                "right_ring_mcp": 0.55 * ease,
                "right_ring_pip": 0.70 * ease,
                "right_pinky_mcp": 0.50 * ease,
                "right_pinky_pip": 0.65 * ease,
            }
        )
        _set_dofs(articulation, pose)
        simulation_app.update()
        rep.orchestrator.step(delta_time=1.0 / 12.0, rt_subframes=4, pause_timeline=False)
        wrist_matrix = _world_matrix(stage, "right_wrist_yaw_link")
        tower_matrix = _world_matrix(stage, "right_tower")
        relative_mounts.append(np.linalg.inv(wrist_matrix) @ tower_matrix)
        records.append(
            {
                "frame": frame,
                "phase": phase,
                "commanded_rad": {
                    key: pose[key]
                    for key in (
                        "right_wrist_yaw_joint",
                        "right_wrist",
                        "right_thumb_pip",
                        "right_index_mcp",
                        "right_index_pip",
                        "right_middle_mcp",
                        "right_middle_pip",
                    )
                },
                "right_wrist_yaw_position_m": _translation(stage, "right_wrist_yaw_link").round(9).tolist(),
                "right_tower_position_m": _translation(stage, "right_tower").round(9).tolist(),
                "right_palm_position_m": _translation(stage, "right_palm").round(9).tolist(),
                "right_index_fingertip_position_m": _translation(stage, "right_index_fingertip").round(9).tolist(),
            }
        )

    rep.orchestrator.wait_until_complete()
    writer.detach()
    pngs = sorted(frame_dir.rglob("*.png"))
    if len(pngs) != frame_count:
        raise RuntimeError(f"Expected {frame_count} motion PNGs, found {len(pngs)}")

    mount_drift = float(np.max(np.abs(relative_mounts[-1] - relative_mounts[0])))
    if mount_drift > 1.0e-5:
        raise RuntimeError(f"Fixed mount transform drifted by {mount_drift}")
    render_product.destroy()

    return {
        "frame_count": frame_count,
        "fps_for_encoding": 12,
        "fixed_mount_max_abs_transform_drift": mount_drift,
        "first": records[0],
        "last": records[-1],
        "frame_directory": frame_dir.relative_to(output_dir).as_posix(),
    }


def main() -> None:
    root = args.root.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"Refusing to overwrite existing output directory: {output}")
    output.mkdir(parents=True)

    _enable("omni.scene.optimizer.core")
    _enable("isaacsim.robot.schema")
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)
    rep.orchestrator.set_capture_on_play(False)

    urdf = root / "asimov_orcahand_right.urdf"
    usd_dir = root / "derived_usd"
    usd_dir.mkdir(exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(urdf),
        usd_path=str(usd_dir),
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_from_visuals=False,
        allow_self_collision=False,
        fix_base=True,
        joint_drive_type="force",
        joint_target_type="position",
        override_joint_stiffness=120.0,
        override_joint_damping=12.0,
        run_asset_transformer=True,
        run_multi_physics_conversion=True,
    )
    output_usd = Path(URDFImporter(config).import_urdf()).resolve()
    if not output_usd.is_file():
        raise RuntimeError(f"URDF importer returned missing USD: {output_usd}")
    omni.usd.get_context().open_stage(str(output_usd))
    for _ in range(8):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    default_prim = stage.GetDefaultPrim()
    if not default_prim.IsValid():
        raise RuntimeError("Imported USD has no default prim")
    physics_variant = default_prim.GetVariantSets().GetVariantSet("Physics")
    if not physics_variant.IsValid():
        raise RuntimeError("Imported USD has no Physics variant set")
    physics_variant.SetVariantSelection("physx")
    for _ in range(3):
        simulation_app.update()

    articulation_roots = [
        prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        raise RuntimeError(f"Expected one articulation root, found {len(articulation_roots)}")
    articulation_root = articulation_roots[0]

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(5):
        simulation_app.update()
    articulation = Articulation(str(articulation_root.GetPath()))

    presentation_pose = {
        "right_shoulder_pitch_joint": 0.20,
        "right_shoulder_roll_joint": 0.60,
        "right_shoulder_yaw_joint": -0.15,
        "right_elbow_joint": -0.65,
        "right_wrist_yaw_joint": -0.15,
        "right_wrist": -0.20,
        "left_shoulder_pitch_joint": 0.10,
        "left_shoulder_roll_joint": -0.25,
    }
    _set_dofs(articulation, presentation_pose)
    for _ in range(3):
        simulation_app.update()

    minimum, maximum = _stage_range(stage, default_prim)
    center = (minimum + maximum) / 2.0
    span = maximum - minimum
    scale = float(max(span))
    if not 0.7 < scale < 3.0:
        raise RuntimeError(f"Imported robot bounds indicate a scale problem: min={minimum}, max={maximum}")

    # Neutral inspection lighting plus a floor at the imported robot's lowest point.
    dome = UsdLux.DomeLight.Define(stage, "/SpikeLighting/Dome")
    dome.GetIntensityAttr().Set(700.0)
    dome.GetColorAttr().Set(Gf.Vec3f(0.85, 0.90, 1.0))
    key = UsdLux.DistantLight.Define(stage, "/SpikeLighting/Key")
    key.GetIntensityAttr().Set(1800.0)
    key.GetAngleAttr().Set(1.5)
    UsdGeom.XformCommonAPI(key).SetRotate(
        Gf.Vec3f(315.0, 25.0, 25.0), UsdGeom.XformCommonAPI.RotationOrderXYZ
    )
    fill = UsdLux.RectLight.Define(stage, "/SpikeLighting/Fill")
    fill.GetIntensityAttr().Set(4500.0)
    fill.GetWidthAttr().Set(2.0)
    fill.GetHeightAttr().Set(2.0)
    UsdGeom.XformCommonAPI(fill).SetTranslate(Gf.Vec3d(1.5, -1.5, 1.8))
    UsdGeom.XformCommonAPI(fill).SetRotate(
        Gf.Vec3f(45.0, 0.0, 45.0), UsdGeom.XformCommonAPI.RotationOrderXYZ
    )
    floor = UsdGeom.Cube.Define(stage, "/SpikeGround")
    floor.GetSizeAttr().Set(1.0)
    floor_api = UsdGeom.XformCommonAPI(floor)
    floor_api.SetTranslate(Gf.Vec3d(float(center[0]), float(center[1]), float(minimum[2] - 0.025)))
    floor_api.SetScale(Gf.Vec3f(4.0, 4.0, 0.05))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.13, 0.14, 0.16)])

    diagnostic_material = _apply_orca_diagnostic_material(stage)

    wrist_target = (_translation(stage, "right_wrist_yaw_link") + _translation(stage, "right_tower")) / 2.0
    hand_target = (
        _translation(stage, "right_tower")
        + _translation(stage, "right_palm")
        + _translation(stage, "right_middle_fingertip")
    ) / 3.0
    renders = [
        _capture(
            output,
            "full_body_3q",
            center + np.array((1.25 * scale, -1.65 * scale, 0.75 * scale)),
            center,
            (960, 720),
        ),
        _capture(
            output,
            "wrist_connection",
            wrist_target + np.array((0.78, -1.02, 0.46)),
            wrist_target,
            (960, 720),
        ),
        _capture(
            output,
            "orcahand_close",
            hand_target + np.array((0.72, -0.94, 0.42)),
            hand_target,
            (960, 720),
        ),
    ]

    motion = _motion_capture(
        output,
        str(articulation_root.GetPath()),
        stage,
        hand_target + np.array((0.72, -0.94, 0.42)),
        hand_target,
    )

    collision_prims = [
        str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    rigid_bodies = [
        str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    physx_api = PhysxSchema.PhysxArticulationAPI(articulation_root)
    self_collision_attr = physx_api.GetEnabledSelfCollisionsAttr()
    self_collision = self_collision_attr.Get() if self_collision_attr.IsValid() else None

    report = {
        "status": "pass",
        "isaac_version": (Path(os.environ.get("ISAAC_PATH", "C:/isaacsim")) / "VERSION").read_text().strip()
        if (Path(os.environ.get("ISAAC_PATH", "C:/isaacsim")) / "VERSION").is_file()
        else "unknown",
        "source_urdf": urdf.relative_to(root).as_posix(),
        "imported_usd": output_usd.relative_to(root).as_posix(),
        "default_prim": str(default_prim.GetPath()),
        "articulation_root": str(articulation_root.GetPath()),
        "dof_count": len(articulation.dof_names),
        "dof_names": list(articulation.dof_names),
        "rigid_body_prim_count": len(rigid_bodies),
        "collision_prim_count": len(collision_prims),
        "import_config_allow_self_collision": False,
        "self_collision_enabled": self_collision,
        "diagnostic_material": diagnostic_material,
        "bounds_m": {"min": minimum.tolist(), "max": maximum.tolist(), "span": span.tolist()},
        "renders": [str(path.relative_to(output)) for path in renders],
        "motion": motion,
    }
    (output / "isaac_import_articulation.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    timeline.stop()


try:
    main()
except BaseException:
    failure = traceback.format_exc()
    print(failure, flush=True)
    if args.output:
        args.output.resolve().mkdir(parents=True, exist_ok=True)
        (args.output.resolve() / "_FAILED.txt").write_text(failure, encoding="utf-8")
    raise
finally:
    simulation_app.close()
