# Robot manipulation status

Updated: 2026-09-26

Overall restocking goal: **in progress; R8 not complete**

Current task: **R2 locomotion source approved; feedback adapter under independent review**

## Current result

An isolated production derivative and CPU-safe runtime seam now exist under
`robot_spike/production/`. The code verifies the approved source hashes,
complete Asimov/OrcaHand structure, mesh resolution and scale, collision mesh
loading, finite joint limits, documented dummy inertias/contact gaps, and a
single connected joint tree. The importer configuration is free-rooted
(`fix_base=false`).

The controller binds DOFs by name and rejects missing, extra, duplicate,
unknown, nonfinite, and out-of-limit inputs. Normal joint motion uses Isaac
position targets. Explicit reset restores the configured root pose, root linear
and angular velocity, all 40 joint positions, all joint velocities, and the
drive targets in runtime DOF order.

## Verified on CPU

- Approved external restocking specification SHA-256:
  `7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad`.
- Approved upstream manifest canonical-LF SHA-256:
  `6ddcd328db7c61de83e6ccea5bbf5a496b714bc9370acb937831f3a21f9a6b80`.
- Approved combined URDF canonical-LF SHA-256:
  `25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea`.
- Production derivation is deterministic and records zero link, joint,
  inertial, collision-geometry, and scale changes.
- Focused CPU suite passes 10 tests, including negative missing-mesh,
  invalid-scale, malformed-tree, nonfinite-limit, invalid-target, reordered-DOF,
  motion-command, and deterministic-reset cases.

## R1 runtime result (2026-09-26)

A serialized headless Isaac Sim 6.1 run tested integrated code candidate
`1fe5ac5c42466b3ea581b73585e919e0a8e32117` / tree
`88c52e464ab2542ba5e3eb854b00064719f659c7`, with the `Physics=physx` variant,
a ground plane, and 200 Hz physics. The imported stage was 1.0 m/unit and
reported the exact expected 40 DOFs, 48 collision prims, and 61 rigid bodies.
A 0.20 rad right shoulder pitch target produced 0.176431 rad measured motion.
Both ankle-roll supports contacted the ground (143.888 N left, 160.528 N
right). Repeating reset after motion produced zero measured position/orientation
error after the same 10 simulation steps.

The neutral all-zero free-base configuration is not dynamically stable. After
240 steps (1.2 s), the root drifted 0.168067 m, tilted 18.8275 degrees, and had
0.4671 m/s linear and 0.7210 rad/s angular speed. Therefore R1 is partially
runtime validated (import, DOF mapping, drive response, foot contact, and reset),
but not a dynamic stability pass. The code-level candidate and prior approvals
remain unchanged; evidence and harness are in
`robot_spike/evidence/isaac_r1_runtime_smoke.json` and
`robot_spike/evidence/isaac_r1_runtime_smoke.py`.

## Open runtime gates

- Neutral-pose passive balance remains unverified/failed; closed-loop balance
  and gait are required for R2.
- Collision behavior and a safe self-collision filter set remain open.
- R3 arm reach, R4 physical grasp, R5 stationary pick-and-place, R6 route,
  R7 carrying drive, and the continuous full restocking run (R8) remain open.
## Routing observation

The task was configured for GPT-5.6 Sol with high reasoning effort. The worker
runtime did not expose an independently observable model or effort identity, so
observed model and observed effort are recorded as `unknown`.


## R2 source and R3 reach code review outcomes (2026-09-26)

The exact RST-003 locomotion source candidate db94dd139cd0e1e60a01557c25103a3c792ce9d6 / tree 29752b846a0ad72999a9c31fca251d4c32d78878 received independent approval after revisions. The reviewer confirmed the specified owner acceptance document SHA-256 7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad and the formal relevant robot input-manifest Git-blob SHA-256 8c4f698803a8e877eb714a3c1d0964d284480eb95f6fe055efec9dd2ddf2b542. The focused model/locomotion CPU suite passed 25/25. The early static review found and closed measured-step bounds, lateral-control, root-height safety, hip-yaw sign/touchdown-yaw, and invalid-feedback fault-latching issues. This approves source behavior only. Physical gait, root stability, foot tracking/contact, balance, support margin, drive tuning, route, stop/dock, and repeatability remain open pending R2 Isaac/PhysX validation.

RST-004 runtime feedback adapter and R2 smoke harness candidate 208642a5c363a7b91905926f86c553330855562d / tree c2b1e4190b6332df1dc85d96135a82da39b54420, based on the approved RST-003 candidate, passes 22 focused CPU tests and statically binds to Isaac Sim 6.1 measured articulation, contact, mass, COM, and simulation-time APIs. It has not run in Isaac, PhysX, or on the GPU; the first exact independent review returned REQUEST_CHANGES: the harness must hash the actual owner spec and production_manifest.json, and revalidate the source/runtime identity vector at each repeat and before final success. A revision is underway; no R2 runtime gate is claimed passed.

The RST-005 arm-reach source candidate 6b297645a889c376b44969b62e3e39f471584383 / tree 5f9dad7cdb94069656cd6672fc1f0d4688698c31 is independently approved with zero findings. Its focused arm-reach and production-model CPU tests passed 18/18. The review confirmed the corrected weighted-DLS merit function converges for the previously missed reachable near-target case. This is a CPU/source approval only; physical reach, support during arm motion, and collision clearance remain open.
