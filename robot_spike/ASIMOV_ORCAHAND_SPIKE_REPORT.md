# Asimov 1 + OrcaHand V1 right-hand feasibility report

Date: 2026-09-26

Scope: isolated local spike, separate from production G02 and restocking work

## Verdict

The combination is mechanically and technically feasible as a simulation
prototype. The pinned models form one rooted graph, import into Isaac Sim 6.1
as one 40-DOF articulation, retain the Asimov forearms and complete OrcaHand V1
extended-right tower/wrist/palm/fingers/thumb, and accept direct wrist and
finger motion commands.

It is not ready as a production robot asset. The doubled distal-arm structure
is bulky and reaches near the knee in the inspected pose. Collision meshes
load and import, but self-collision was disabled for this spike and the broad
CPU scan reports 102 conservative AABB candidates. The adapter, collision
filtering/proxies, inertials, drive gains, and proportions need engineering
before dynamic contact use.

## Exact upstream provenance

| Source | Revision | Selected path | SHA-256 |
|---|---|---|---|
| Asimov 1 | commit `ccf5326f0adffd65edf930cbbac052454cdd82be`, tree `dccddbbd164db4e516ec655597cebd5e9b6d482e` | `sim-model/urdf/asimov_1.urdf` | `ac681a57789c12ba6fafe5101ea6790b7e8c97da8523b83373defea42633bb6f` |
| OrcaHand description | commit `b9b349a21ee0238c62b6cf92ae7597027867adf8`, tree `694a70bec9d960eecb05ae5b6746259b2e43d6f6` | `v1/models/urdf/orcahand_right_extended.urdf` | `4ca4f81ba73038abcf1b989f164077d0a0b88bf446e19386d888028eee78e2e2` |

`upstream_manifest.json` additionally records commit timestamps and subjects,
the source URDF Git blob IDs, all 93 copied mesh hashes, and every copied notice
hash. The upstream repositories were cloned directly and treated as read-only
inputs; the derivation script wrote only inside this spike directory.

## Combination details

The Asimov model remains the world-rooted robot with `pelvis_link` as its only
root. From OrcaHand, only the source-world anchoring pair was bypassed:

- Removed links: `world`, `world2right_tower_fixed_jointbody`
- Removed joints: `world2right_tower_fixed`,
  `world2right_tower_fixed_offset`
- Retained Orca subtree root: `right_tower`
- Retained Asimov attachment link: `right_wrist_yaw_link`

The derived fixed joint is:

| Field | Value |
|---|---|
| Joint | `asimov_to_orcahand_mount` |
| Parent | `right_wrist_yaw_link` |
| Child | `right_tower` |
| xyz (m) | `0.0459626666 0 -0.0385672566` |
| rpy (rad) | `0 2.2689280276 0` |

The 130-degree +Y rotation aligns Orca's local +Z forearm/tower direction with
the Asimov wrist axis `(0.7660444431, 0, -0.6427876097)`. The 60 mm axial offset
clears the approximately 40 mm Asimov wrist body. A derived 22 mm radius,
70 mm long cylindrical collar is attached to `right_tower` and bridges back
toward the Asimov wrist. It has both visual and collision geometry.

No upstream source checkout was edited. Mesh URIs were rewritten to the copied
relative paths under `assets/`; there are no remaining `package://` or absolute
mesh URIs. Existing right-prefixed Orca names do not collide with Asimov names.

## Evidence

### Static model and collision import

`validate_combined_model.py` passed and wrote
`evidence/cpu_model_validation.json`:

- one root, 66 links and 65 joints;
- 40 revolute and 25 fixed joints;
- 94 mesh references resolving to 93 unique files;
- all 34 Orca collision STL files load through `trimesh`, have finite geometry,
  and remain in meter-scale bounds;
- a 0.25 rad Asimov wrist-yaw command changes the retained Orca tower rotation;
- a 0.30 rad Orca wrist command changes the palm rotation;
- flexing the hand displaces every fingertip by 53.6 to 71.6 mm.

The 102 reported nonadjacent AABB candidates are deliberately conservative.
They include upstream primitive boxes and nearby multipart hand links. They are
not exact mesh collision results and are retained as tuning work rather than
reported as a pass. The close render has a projected hand/thigh overlap, and
the broad scan includes `right_hip_pitch_link` versus `right_tower` candidates.
No narrow-phase contact simulation was run, so this spike cannot distinguish a
camera projection from real mesh penetration at that pose or claim that the
pose is self-collision-free. Isaac self-collision was explicitly disabled.

### Actual Isaac import and articulation

`run_isaac_spike.py` passed in Isaac Sim
`6.1.0-rc.26+release.49347.2d230af4.gl` and wrote
`evidence/isaac_final/isaac_import_articulation.json`:

- the derived URDF imported to the checked-in nine-file USD package;
- Isaac exposed one articulation root at
  `/asimov_1_orcahand_v1_right_spike/Geometry/pelvis_link`;
- 40 DOFs, 61 rigid-body prims, and 48 collision prims were present;
- 24 frames commanded Asimov right wrist yaw, Orca wrist motion, thumb motion,
  and flexion across all four fingers;
- the fixed wrist-to-tower transform changed by at most
  `3.0228017466882307e-07` across the first and last commanded poses;
- the right index fingertip moved from
  `(0.293364022, -0.378485778, -0.089670258)` m to
  `(0.262837031, -0.393171493, -0.077328887)` m in the actual Isaac stage.

Actual render evidence:

- `evidence/isaac_final/renders/full_body_3q.png`
- `evidence/isaac_final/renders/wrist_connection.png`
- `evidence/isaac_final/renders/orcahand_close.png`
- `evidence/isaac_final/motion_frames/rgb_0000.png` through
  `rgb_0023.png` (12 fps nominal)

Independent visual inspection passed these artifacts for spike evidence: the
tower, wrist, palm, thumb, and four fingers read as one contiguous assembly and
the motion progresses clearly from open to curled. The scale is visibly bulky,
the mating face is partly occluded by the tower housing, and the hand overlaps
the thigh in projection. Pixels alone do not establish collision clearance.

`ffmpeg` was unavailable on this machine, so the short motion is retained as
the exact 24-frame sequence instead of a re-encoded clip.

## Material diagnosis

The Orca V1 URDF uses `material name="white"` references but does not define
the material's color or alpha. Isaac's importer rendered the source material
black, glassy, or transparent, making close views look fragmented. The mesh
geometry itself was intact.

For diagnostic renders only, the runtime script binds an opaque neutral blue
USD Preview Surface to 34 `right_visual_*` prims after import. This makes the
tower, palm, and digits legible and confirms continuity. The combined URDF,
copied meshes, and checked-in imported USD package preserve the source visual
content; the neutral binding exists only in the render session.

## Key hashes

Except for the attached request, these are SHA-256 digests of the exact Git
blob bytes committed in the reviewed candidate, read with `git cat-file blob
HEAD:<path>`. This avoids checkout line-ending conversion. The attached request
digest is over the exact bytes of the supplied `Pasted text.txt` attachment.

| Artifact | SHA-256 |
|---|---|
| Attached request | `5cd97a8330f536afb56e0ea8ae2d5372b620eb532d96f381e481ef766de0c476` |
| Upstream manifest | `6ddcd328db7c61de83e6ccea5bbf5a496b714bc9370acb937831f3a21f9a6b80` |
| Combined URDF | `25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea` |
| CPU evidence JSON | `f562ace378f95d2a6e5e37173fd64bcaac4f25298ad5c362e656e87703cb41e3` |
| Isaac evidence JSON | `ea22b6db4167895d99f26c2df598231d9febc16d8d7b315b6676882822425c17` |
| Full-body render | `70882c1c779a616dc8068fbbd6263ca62308413ceef670f87a73d3829f3e9fc6` |
| Wrist render | `2480406dfcb098049482263298d3533c8cd0ba28a664360739b87757f393b575` |
| Hand render | `fd6fc8b198962b0972e0f7a60d285f56defbe28b6843e7f7e6ca5a2d8407529b` |
| Imported USD root | `ddc4ccac1f4026c348f49cfdef6ccf728d7e6ae9532b34b79fa52b70dcec563d` |

## Licensing and redistribution

The Asimov repository includes distinct `SOFTWARE-LICENSE.txt` (GPL-2.0) and
`HARDWARE-LICENSE.txt` (CERN-OHL-S-2.0) notices. Its README labels repository
software and hardware accordingly, while the selected `sim-model` paths do not
carry path-level SPDX markers. Both notices and the upstream README are copied.

The OrcaHand description repository states MIT and its `LICENSE` and README are
copied. This report does not determine compatibility between those notices or
license the derived combination for redistribution. The spike remains local
and unpushed; redistribution requires separate review.

## Recommendation

Use this result to continue only if the long tower-plus-Asimov-forearm form is
acceptable for the intended embodiment. The next engineering pass should
replace the visual collar with a measured bracket, establish collision groups
and simplified proxies, tune inertials and drives under gravity/contact, and
define a downstream material intentionally. A separate licensing review is
required before distributing combined assets.
