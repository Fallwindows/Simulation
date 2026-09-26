# Restocking robot model changes

Date: 2026-09-26

Task: RST-001, R1 robot-model code milestone

Approved source: `robot_spike/asimov_orcahand_right.urdf`

Canonical-LF SHA-256: `25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea`

The source combined model and all upstream assets remain unchanged. The
production derivative is generated under `robot_spike/production/` and reuses
the exact approved meshes in place.

## Changes

| Original item | Observed problem | Production change | Reason | Value source | Classification |
|---|---|---|---|---|---|
| Robot name `asimov_1_orcahand_v1_right_spike` | A spike-specific identity is ambiguous in production outputs. | Derived URDF name is `asimov_1_orcahand_v1_right_restocking`. | Keep production imports distinguishable without editing the approved source. | Deterministic derivation policy in `build_production_model.py`. | Project naming decision; no physical change. |
| 94 mesh references rooted at `assets/...` | Those paths are relative to `robot_spike/`, while the production URDF is one directory deeper. | Each reference is rewritten to `../assets/...`. | Resolve the same approved files from the isolated production directory. | Exact source path plus one relative parent segment. | Path-derived; no mesh, scale, origin, or physical change. |
| Spike importer setting `fix_base=true` | A fixed root cannot represent the supplied biped's physical base motion. | Production import configuration sets `fix_base=false`. | R1 must expose a non-fixed base for later physical gait work. | Restocking specification sections 9 and 14; supplied model is a legged biped. | Simulation configuration decision; runtime behavior unverified. |
| Spike direct `set_dof_positions` motion helper | Direct state writes teleport joints and cannot demonstrate simulated motion. | `ArticulationController.command_joint_positions` uses `set_dof_position_targets`; direct pose/velocity setters are confined to explicit reset. | Physics must advance commanded articulation motion. | Installed Isaac Sim 6.1 API contract: position targets drive motion; position setters teleport state. | API behavior from installed runtime source; Isaac execution pending. |
| Source-neutral root at `z=0` | With a free root, neutral foot geometry extends to approximately `z=-0.63062938 m`. | Reset root position is `[0, 0, 0.635] m`, leaving about 4.4 mm initial clearance. | Provide a reproducible non-penetrating initial state for later settling tests. | CAD/mesh-derived neutral geometry bound from the approved model. | CAD-derived simulation initialization; not a measured hardware pose and not yet dynamically validated. |
| Runtime DOF ordering | Importer ordering is not a safe caller contract. | Commands bind the exact 40-DOF set by name and map the stable canonical order to actual runtime indices. | Prevent commands from reaching the wrong joint when importer order changes. | Canonical order recorded by the approved Isaac spike; runtime set equality is checked at bind time. | Evidence-derived software contract. |

## Preserved structures

- Both Asimov forearms and wrist-yaw links remain present.
- The OrcaHand tower, wrist, palm, thumb, and all four fingers remain present.
- Link count remains 66; joint count remains 65; revolute DOF count remains 40.
- All joint axes, origins, limits, inertials, collision elements, and mesh scales
  are unchanged from the approved combined source.
- The derived URDF has one root, `pelvis_link`, and all links are connected.

## Explicit assumptions and unresolved physical-model work

The approved combined URDF contains five fixed fingertip marker links with
zero mass/inertia and 17 movable OrcaHand joint-frame links with placeholder
`0.001 kg` / `1e-9 kg m²` inertials. These values are preserved so this code
milestone does not invent hardware measurements. They are classified and
fail-closed allowlisted in `robot_config.json`; any added or removed dummy value
causes production loading to fail.

Thirty-three links have no collision element. They are predominantly
intermediate joint frames, fixed fingertip markers, or multipart carrier links;
physical neighboring links retain the approved collision meshes/primitives.
`collision_from_visuals=false` prevents silent fabrication of contact geometry.
The exact list is recorded in `robot_config.json` and
`production_manifest.json`. Contact adequacy remains unverified until an Isaac
contact test runs.

Self-collision remains disabled because the spike reported 102 conservative
nonadjacent AABB candidates and did not establish a safe collision-filter set.
Environment collision geometry still imports. Enabling self-collision without
filter tuning could destabilize the free-base articulation, so that change is
deferred to a reviewed runtime milestone.

No gait, balance, reach, grasp, carrying, placement, or R1 runtime pass is
claimed by this record.
