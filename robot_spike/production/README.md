# Restocking production robot seam

This directory contains the isolated R1 derivative of the approved Asimov 1 +
OrcaHand V1 spike. The upstream spike URDF, assets, evidence, and manifest are
inputs and are never rewritten.

`build_production_model.py` makes two intentional URDF edits: it gives the
derived robot a production-specific name and changes each `assets/...` mesh URI
to `../assets/...` so the isolated file resolves the exact same approved mesh.
It does not change links, joints, inertials, collision geometry, or mesh scale.

`model.py` validates source/output hashes, the single rooted joint tree, finite
joint limits, mesh paths, mesh scales, required Asimov/OrcaHand links, and every
documented inertia/collision assumption. `runtime.py` binds imported DOFs by
name. Normal motion calls `set_dof_position_targets`; direct root/joint state
setters are confined to the explicit deterministic reset method.

The importer configuration uses `fix_base=false` for the supplied biped. The
neutral reset places the pelvis at 0.635 m, clears all root and DOF velocities,
and restores all 40 joints to zero radians. This is a deterministic starting
state, not evidence of passive balance or gait. Self-collision stays disabled
until the known broad collision candidates are filtered and tested in Isaac.

CPU rebuild and checks:

```powershell
C:\isaacsim\python.bat robot_spike\production\build_production_model.py
C:\isaacsim\python.bat -m unittest tests.test_robot_production_model
```

Isaac import, free-base settling, joint actuation, contact behavior, and
repeatable reset remain runtime checks. Do not run them without the allocated
GPU slot.
